"""The real-mode SLAM bridge (docs/design/2026-11-25-real-sources-pr7.md 6.1, 2-3; PR 7c-1a).

A thin ROS shell around ``forklift_core.localization.real_bridge_core.BridgeCore``, which makes every decision:

- one connection on ``socket_path`` (one session); the bridge ends when it closes or the core faults;
- slam_toolbox instances are this process's direct children (``slam_instance.SlamInstances``: the node executable
  itself, its own process group), instance ``k`` on its own topics ``/slam_k/{scan,pose,map}``; ``configure`` ->
  ``activate``, its scan subscription and the static TF are checked before the core hears ``on_instance_ready``;
- ``/clock`` is the current host ``CLOCK_MONOTONIC``, from a wall timer (100 Hz) that does not read ROS time; scans and
  ``odom -> base_link`` are stamped with the frame's ``stamp_ns``; slam_toolbox runs on ``use_sim_time``;
- the container runs in its own ``ROS_DOMAIN_ID`` with ``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST``.

Parameters: ``socket_path``, ``localization_bundle``, ``laser_xyz_yaw`` (base_link <- laser, from the rear-axle mount),
``mapping_params`` (the mapping YAML the localization keys go on), ``output_dir``.
"""

from __future__ import annotations

import json
import math
import os
import select
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from forklift_core.localization import slam_map
from forklift_core.localization import slam_real_link as rl
from forklift_core.localization.real_bridge_core import BridgeCore
from forklift_core.localization.slam_instance import SlamInstances

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
NODE_NAME = "slam_toolbox"
LIFECYCLE_S = 20.0
SUBSCRIBE_S = 15.0


def stamp_of(ns: int) -> Time:
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def ns_of(t: Time) -> int:
    return t.sec * 1_000_000_000 + t.nanosec


def quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def instance_params(mapping: dict, prefix: str, start_pose, k: int) -> dict:
    """The localization params of instance ``k``: the survey bundle's graph and start pose on the mapping params
    (``tools/slam_localization_params.localization_params``'s keys), its own topics, no map TF (the bridge reads only
    ``/pose``), simulated time."""
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools"))
    from slam_localization_params import localization_params

    p = dict(localization_params(mapping, prefix, [float(v) for v in start_pose])["slam_toolbox"]["ros__parameters"])
    p.update(scan_topic=f"/slam_{k}/scan", map_name=f"/slam_{k}/map", transform_publish_period=0.0, use_sim_time=True)
    return {"/**": {"ros__parameters": p}}


class RealSlamBridge(Node):
    def __init__(self) -> None:
        super().__init__("real_slam_bridge")
        self.declare_parameter("socket_path", "/run/slam/slam.sock")
        self.declare_parameter("localization_bundle", "")
        self.declare_parameter("laser_xyz_yaw", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("mapping_params", "")
        self.declare_parameter("output_dir", "/out")
        self.out = Path(self.get_parameter("output_dir").value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.laser = tuple(float(v) for v in self.get_parameter("laser_xyz_yaw").value)
        self.bundle = slam_map.load_bundle(Path(self.get_parameter("localization_bundle").value))
        self.mapping = yaml.safe_load(Path(self.get_parameter("mapping_params").value).read_text())
        self.lock = threading.RLock()
        self.conn: socket.socket | None = None
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", LATCHED)
        self.scan_pubs: dict = {}
        self.subs: dict = {}
        exe = Path(subprocess.run(["ros2", "pkg", "prefix", "slam_toolbox"], capture_output=True, text=True)
                   .stdout.strip()) / "lib/slam_toolbox/localization_slam_toolbox_node"
        self.instances = SlamInstances(
            lambda k, pose: [str(exe), "--ros-args", "--params-file", str(self._params(k, pose)),
                             "-r", f"__ns:=/slam_{k}"],
            log_dir=self.out, graph_has=self._in_graph, drain=self._drain)
        self.core = BridgeCore(self, self.bundle, self.laser)
        self.events: list = []
        self.bring_ups: dict = {}  # k -> (thread, cancel event, {"cli": Popen | None})
        self.map_drops: set = set()
        self.unreaped: list = []  # lifecycle CLIs not reaped at their instance's stop: tried again at the end
        self.stopping = threading.Event()
        # /clock from its own thread, not a ROS timer: a ROS timer runs on the node's (possibly simulated) clock and on
        # the executor that the core's lock can hold up (Codex 7c-1a round 1 P2-3)
        threading.Thread(target=self._clock_loop, daemon=True).start()

    # --- the core's io -------------------------------------------------------------------------------------------
    def now_ns(self) -> int:
        return time.monotonic_ns()

    def send(self, frame: bytes) -> None:
        if self.conn is not None:
            try:
                self.conn.sendall(frame)
            except OSError:
                self.core.closed = True

    def start_instance(self, k: int, start_pose) -> None:
        self.scan_pubs[k] = self.create_publisher(LaserScan, f"/slam_{k}/scan", 10)
        self.subs[k] = (
            self.create_subscription(PoseWithCovarianceStamped, f"/slam_{k}/pose",
                                     lambda m, k=k: self._on_pose(k, m), 10),
            self.create_subscription(OccupancyGrid, f"/slam_{k}/map", lambda m, k=k: self._on_map(k, m), LATCHED))
        self.instances.start(k, start_pose)
        cancel, box = threading.Event(), {"cli": None, "lock": threading.Lock()}
        thread = threading.Thread(target=self._bring_up, args=(k, cancel, box), daemon=True)
        self.bring_ups[k] = (thread, cancel, box)
        thread.start()

    def stop_instance(self, k: int) -> bool:
        # the instance's bring-up first: no lifecycle call may follow its stop (Codex 7c-1a round 1 P2-5)
        up = self.bring_ups.pop(k, None)
        if up is not None:
            thread, cancel, box = up
            with box["lock"]:  # with _cli's registration under the same lock, no CLI starts after this
                cancel.set()
                cli = box.get("cli")
            if cli is not None and cli.poll() is None:
                cli.kill()
                try:
                    cli.wait(timeout=1.0)
                except subprocess.TimeoutExpired:  # recorded and kept; the instance's own stop goes on (round 3 P2-2)
                    self.events.append({"event": "cli_not_reaped", "k": k, "pid": cli.pid})
                    self.unreaped.append(cli)
            thread.join(timeout=1.0)
            if thread.is_alive():
                self.events.append({"event": "bring_up_still_running", "k": k})  # it can start nothing more
        for sub in self.subs.pop(k, ()):
            if sub is not None:
                self.destroy_subscription(sub)
        pub = self.scan_pubs.pop(k, None)
        if pub is not None:
            self.destroy_publisher(pub)
        ok = self.instances.stop(k)
        self.events.append({"event": "stop", **self.instances.records[-1]})
        return ok

    def publish(self, k: int, scan, stamp_ns: int) -> None:
        stamp = stamp_of(int(stamp_ns))
        x, y, yaw = scan.odom_from_base
        qx, qy, qz, qw = quat(yaw)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf_pub.publish(TFMessage(transforms=[t]))
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp, "laser"
        msg.angle_min = float(scan.angle_min_rad)
        msg.angle_increment = float(scan.angle_increment_rad)
        msg.angle_max = float(scan.angle_min_rad + scan.angle_increment_rad * (len(scan.ranges_m) - 1))
        msg.range_min, msg.range_max = float(scan.range_min_m), float(scan.range_max_m)
        msg.ranges = np.asarray(scan.ranges_m, dtype=float).tolist()  # NaN stays NaN: invalid (design 6.1, 1)
        pub = self.scan_pubs.get(k)
        if pub is not None:
            pub.publish(msg)

    # --- ROS side ----------------------------------------------------------------------------------------------------
    def _clock_loop(self) -> None:  # not _clock: rclpy.Node keeps its ROSClock there
        while not self.stopping.is_set():
            self.clock_pub.publish(Clock(clock=stamp_of(time.monotonic_ns())))
            time.sleep(0.01)

    def _static_tf(self) -> None:
        x, y, z, yaw = self.laser
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "base_link", "laser"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        qx, qy, qz, qw = quat(yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf_static_pub.publish(TFMessage(transforms=[t]))

    def _params(self, k: int, pose) -> Path:
        path = self.out / f"params_{k}.yaml"
        path.write_text(yaml.safe_dump(instance_params(self.mapping, str(self.bundle.graph_prefix), pose, k)))
        return path

    def _in_graph(self, k: int) -> bool:
        return any(n == NODE_NAME and ns == f"/slam_{k}" for n, ns in self.get_node_names_and_namespaces())

    def _drain(self, k: int) -> None:
        pass  # the instance's subscriptions were destroyed before the stop; callbacks already taken are ignored by k

    def _cli(self, args: list, cancel: threading.Event, box: dict, timeout: float) -> tuple[int | None, str]:
        """A ros2 CLI call the instance's stop can kill (None: cancelled or timed out)."""
        with box["lock"]:  # checked and registered together (Codex 7c-1a round 2 P2-1)
            if cancel.is_set():
                return None, "cancelled"
            proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            box["cli"] = proc
        try:
            out, _ = proc.communicate(timeout=timeout)
            return proc.returncode, (out or "").strip()[-200:]
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            return None, "timeout"
        finally:
            box["cli"] = None

    def _fail(self, k: int, cancel: threading.Event, why: str) -> None:
        with self.lock:
            if not cancel.is_set():
                self.core.on_instance_failed(k, why)

    def _bring_up(self, k: int, cancel: threading.Event, box: dict) -> None:
        fq = f"/slam_{k}/{NODE_NAME}"
        deadline = time.monotonic() + LIFECYCLE_S
        while True:
            code, text = self._cli(["ros2", "lifecycle", "get", fq], cancel, box, LIFECYCLE_S)
            if cancel.is_set():
                return
            if code == 0:
                break
            if time.monotonic() > deadline:
                return self._fail(k, cancel, f"lifecycle node never appeared: {text}")
            time.sleep(0.3)
        for transition in ("configure", "activate"):
            code, text = self._cli(["ros2", "lifecycle", "set", fq, transition], cancel, box, LIFECYCLE_S)
            if cancel.is_set():
                return
            ok = code == 0 and "successful" in text
            self.events.append({"event": "lifecycle", "k": k, "transition": transition, "ok": ok, "text": text})
            if not ok:
                return self._fail(k, cancel, f"lifecycle {transition} failed: {text}")
        t0 = time.monotonic()
        while self.count_subscribers(f"/slam_{k}/scan") == 0:
            if cancel.is_set():
                return
            if time.monotonic() - t0 > SUBSCRIBE_S:
                return self._fail(k, cancel, "never subscribed to its scan topic")
            time.sleep(0.05)
        self._static_tf()
        time.sleep(0.5)  # the static transform into slam_toolbox's tf buffer
        proc = self.instances.procs.get(k)
        if proc is None or proc.poll() is not None:
            return self._fail(k, cancel, "the instance exited during its bring-up")
        with self.lock:
            if not cancel.is_set():
                self.core.on_instance_ready(k)

    def _on_pose(self, k: int, m) -> None:
        p = m.pose.pose
        with self.lock:
            self.core.on_pose(k, ns_of(m.header.stamp), (p.position.x, p.position.y, yaw_of(p.orientation)))

    def _on_map(self, k: int, m) -> None:
        with self.lock:
            if not self.core.wants_map(k):
                return
        grid = slam_map.Grid(np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width),
                             (m.info.origin.position.x, m.info.origin.position.y, yaw_of(m.info.origin.orientation)),
                             float(m.info.resolution))
        with self.lock:
            self.core.on_map(k, grid)
            adopted = not self.core.wants_map(k)
        if adopted:
            # no later map is read: its subscription goes (round 1 P3-2), from the socket loop -- not from inside its
            # own callback
            self.map_drops.add(k)

    # --- the socket ----------------------------------------------------------------------------------------------------
    def serve(self) -> int:
        path = Path(self.get_parameter("socket_path").value)
        if path.exists():
            path.unlink()
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(path))
        server.listen(1)
        (self.out / "bridge_ready").write_text("ready\n")
        conn, _ = server.accept()
        server.close()  # one session: any later connection is refused
        os.unlink(path)
        self.conn = conn
        buf = b""
        try:
            while not self.core.closed:
                readable, _, _ = select.select([conn], [], [], 0.02)
                if readable:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while len(buf) >= 4:
                        (length,) = struct.unpack(">I", buf[:4])
                        if len(buf) < 4 + length:
                            break
                        body, buf = buf[4:4 + length], buf[4 + length:]
                        with self.lock:
                            try:
                                obj = rl.decode(body)
                            except Exception as exc:  # noqa: BLE001 -- any malformed frame ends with a fault state
                                self.core.fault(f"protocol: {type(exc).__name__}: {exc}")
                                break
                            self.core.on_frame(obj)
                with self.lock:
                    self.core.tick()
                    for k in list(self.map_drops):
                        self.map_drops.discard(k)
                        subs = self.subs.get(k)
                        if subs is not None and subs[1] is not None:
                            self.destroy_subscription(subs[1])
                            self.subs[k] = (subs[0], None)
        finally:
            with self.lock:
                for k in list(self.bring_ups):
                    self.stop_instance(k)
                self.instances.stop_all()
                for cli in self.unreaped:  # round 4 P3
                    try:
                        cli.kill()
                        cli.wait(timeout=2.0)
                    except (OSError, subprocess.TimeoutExpired):
                        self.events.append({"event": "cli_left", "pid": cli.pid})
            self.stopping.set()
            conn.close()
            (self.out / "events.json").write_text(json.dumps({"events": self.events, "counts": self.core.counts,
                                                              "failures": self.core.failures,
                                                              "instances": self.instances.records}, indent=1))
        return 0


def main() -> int:
    rclpy.init()
    node = RealSlamBridge()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    code = node.serve()
    os._exit(code)  # rclpy's teardown with the spinning thread aborts the process (premise test, job 21220)


if __name__ == "__main__":
    main()
