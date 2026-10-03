"""Lockstep bridge between the Isaac runner and slam_toolbox (online SLAM).

Plan: docs/plans/2026-10-04-online-slam-closed-loop.md (v3). The runner sends
one scan at a time over a Unix socket and waits; this node publishes /clock,
/tf (odom->base_link), /odom and then /scan at that scan's stamp, waits for
slam_toolbox's /pose with exactly that stamp, and answers with the map<-odom it
implies. base_link->laser is published once on /tf_static before the socket
opens, and the socket only opens once slam_toolbox subscribes to /scan.

Which scans slam_toolbox sees (plan v3.4): it runs with its processing
thresholds at 0, so it answers every scan it receives; ``slam_link.
KeyframeGate`` decides which scans to send -- the first five at start-up (the
2nd-4th are dropped by slam_toolbox itself; waited for ``warmup_wait_s`` each),
then keyframes only (0.5 s and 0.447 m or 0.5 rad since the last answered
one). A sent scan's /pose is waited for (``reply_timeout_s``, liveness only:
the simulation is frozen), so no scan is ever in flight unanswered. Other scans
get /clock and the odom transform but no /scan, and are answered ``skipped``
with the last correction. A /pose for a stamp not waited for fails the run. Every /map is kept with the scan id it arrived after, so a video can
show only maps the run had actually received by then.

    ros2 run forklift_ros isaac_slam_bridge --ros-args \\
        -p socket_path:=/run/slam.sock -p output_dir:=/out \\
        -p laser_xyz_yaw:="[-0.12, 0.0, 1.05, 0.0]"
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from forklift_core.localization import slam_link
from forklift_core.localization.slam_pose import compose, invert


def ros_time(stamp_s: float) -> Time:
    ns = int(round(stamp_s * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def stamp_ns(stamp: Time) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quaternion_yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class IsaacSlamBridge(Node):
    def __init__(self):
        super().__init__("isaac_slam_bridge")
        self.declare_parameter("socket_path", "/run/slam/slam.sock")
        self.declare_parameter("output_dir", "/tmp/slam_bridge")
        self.declare_parameter("laser_xyz_yaw", [-0.12, 0.0, 1.05, 0.0])
        self.declare_parameter("reply_timeout_s", 60.0)
        self.declare_parameter("warmup_wait_s", 1.0)
        self.socket_path = self.get_parameter("socket_path").value
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.laser = [float(v) for v in self.get_parameter("laser_xyz_yaw").value]
        self.reply_timeout_s = float(self.get_parameter("reply_timeout_s").value)
        self.warmup_wait_s = float(self.get_parameter("warmup_wait_s").value)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 100)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 100)
        self.poses: dict[int, tuple] = {}
        # Stamps a /pose may legitimately carry: the scan being waited for.
        # Anything else is kept here and fails the run, even after the last
        # reply (Codex v3.3 P2).
        self.awaited: int | None = None
        self.stray: list[int] = []
        self.cond = threading.Condition()
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_pose, 100)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.maps: list[dict] = []
        self.map_lock = threading.Lock()  # metadata and grids change together
        self.map_arrays: dict[str, np.ndarray] = {}
        self.last_scan_id = -1
        self.records: list[dict] = []

    # --- ROS side ---------------------------------------------------------
    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        with self.cond:
            key = stamp_ns(msg.header.stamp)
            if key != self.awaited:
                self.stray.append(key)
                return
            self.poses[key] = (
                p.position.x,
                p.position.y,
                quaternion_yaw(p.orientation),
            )
            self.cond.notify_all()

    def on_map(self, msg: OccupancyGrid) -> None:
        with self.map_lock:
            self._add_map(msg)

    def _add_map(self, msg: OccupancyGrid) -> None:
        index = len(self.maps)
        info = msg.info
        self.maps.append(
            {
                "index": index,
                "after_scan_id": self.last_scan_id,
                "stamp_ns": stamp_ns(msg.header.stamp),
                "received_wall_s": time.monotonic(),
                "resolution_m": info.resolution,
                "width": info.width,
                "height": info.height,
                "origin": [
                    info.origin.position.x,
                    info.origin.position.y,
                    quaternion_yaw(info.origin.orientation),
                ],
            }
        )
        self.map_arrays[f"map_{index:04d}"] = np.asarray(msg.data, dtype=np.int8).reshape(
            info.height, info.width
        )

    def publish_static(self) -> None:
        x, y, z, yaw = self.laser
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "base_link", "laser"
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.translation.z = z
        qx, qy, qz, qw = yaw_quaternion(yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_static_pub.publish(TFMessage(transforms=[t]))

    def publish_scan(self, scan: slam_link.Scan, *, send_scan: bool = True) -> None:
        stamp = ros_time(scan.stamp_s)
        self.clock_pub.publish(Clock(clock=stamp))
        x, y, yaw = scan.odom_from_base
        qx, qy, qz, qw = yaw_quaternion(yaw)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_pub.publish(TFMessage(transforms=[t]))
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "odom", "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        self.odom_pub.publish(odom)
        if not send_scan:
            return
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp, "laser"
        msg.angle_min = float(scan.angle_min_rad)
        msg.angle_increment = float(scan.angle_increment_rad)
        msg.angle_max = float(scan.angle_min_rad + scan.angle_increment_rad * (len(scan.ranges_m) - 1))
        msg.range_min, msg.range_max = float(scan.range_min_m), float(scan.range_max_m)
        msg.ranges = scan.ranges_m.astype(float).tolist()
        self.scan_pub.publish(msg)

    # --- socket side ------------------------------------------------------
    def wait_ready(self) -> None:
        """/tf_static out, and slam_toolbox listening on /scan."""
        self.publish_static()
        deadline = time.monotonic() + 120.0
        while self.count_subscribers("/scan") == 0:
            if time.monotonic() > deadline:
                raise RuntimeError("slam_toolbox never subscribed to /scan")
            time.sleep(0.1)
        time.sleep(1.0)  # let slam_toolbox's tf buffer take the static transform

    def serve(self) -> None:
        Path(self.socket_path).unlink(missing_ok=True)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(self.socket_path)
        server.listen(1)
        (self.output_dir / "bridge_ready").write_text("ready\n")
        conn, _ = server.accept()
        gate = slam_link.KeyframeGate()
        map_from_odom = (0.0, 0.0, 0.0)
        version = 0
        with conn:
            while True:
                try:
                    scan = slam_link.recv_scan(conn)
                except ConnectionError:
                    break
                self.last_scan_id = scan.scan_id
                key = int(round(scan.stamp_s * 1e9))
                start = time.monotonic()
                kind = gate.classify(scan.stamp_s, scan.odom_from_base)
                if kind != "local":
                    with self.cond:
                        self.awaited = key
                self.publish_scan(scan, send_scan=kind != "local")
                expected = kind == "expect_pose"
                pose, stray = None, []
                if kind != "local":
                    wait = self.reply_timeout_s if expected else self.warmup_wait_s
                    with self.cond:
                        got = self.cond.wait_for(lambda: key in self.poses, timeout=wait)
                        pose = self.poses.pop(key, None) if got else None
                        self.awaited = None
                with self.cond:
                    stray = sorted(self.stray)
                    self.poses.clear()
                # A skipped reply vouches for the bridge; check slam_toolbox is
                # still there too (Codex v3.3 P2: skipped is not SLAM liveness).
                slam_alive = (
                    self.count_publishers("/pose") > 0 and self.count_subscribers("/scan") > 0
                )
                elapsed = time.monotonic() - start
                if stray or not slam_alive:
                    status = "failed"
                elif pose is not None:
                    map_from_odom = compose(pose, invert(scan.odom_from_base))
                    version += 1
                    status = "processed"
                    gate.processed(scan.stamp_s, scan.odom_from_base)
                elif expected:
                    status = "failed"
                else:
                    status = "skipped"
                reply = slam_link.Reply(
                    scan.scan_id, scan.stamp_s, status, map_from_odom, version, elapsed
                )
                self.records.append(
                    {
                        "scan_id": scan.scan_id,
                        "stamp_s": scan.stamp_s,
                        "status": status,
                        "sent": kind,
                        "stray_pose_stamps_ns": stray,
                        "slam_alive": slam_alive,
                        "map_from_odom": list(map_from_odom),
                        "slam_pose": list(pose) if pose else None,
                        "wall_s": elapsed,
                    }
                )
                conn.sendall(slam_link.encode_reply(reply))
                if status == "failed":
                    break
        server.close()

    def save(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        with self.cond:
            if self.stray and not any(r.get("stray_pose_stamps_ns") for r in self.records):
                # Arrived after the last reply: keep it in the record.
                self.records.append({"status": "stray_after_last_reply", "stray": self.stray})
        (self.output_dir / "bridge_records.json").write_text(json.dumps(self.records))
        with self.map_lock:
            maps, arrays = list(self.maps), dict(self.map_arrays)
        (self.output_dir / "maps.json").write_text(json.dumps(maps))
        if arrays:
            np.savez_compressed(self.output_dir / "maps.npz", **arrays)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = IsaacSlamBridge()
    node.output_dir.mkdir(parents=True, exist_ok=True)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        node.wait_ready()
        node.serve()
    finally:
        # Stop the callbacks first, then save one consistent snapshot.
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.save()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
