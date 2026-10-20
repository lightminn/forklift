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
import os
import math
import socket
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from forklift_core.localization import slam_link
from forklift_core.localization import slam_map
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


class MapStore:
    """The maps slam_toolbox publishes. ``latest_map.npz`` is rewritten at every map either way (the runner's planner
    reads it while the run goes on). A batch bridge keeps every map for ``maps.json``/``maps.npz`` at the end. A
    persistent one (a live session left up for hours, persistent console plan D8b) keeps none in memory: a map the
    same as the last (cells, size, resolution, origin) only bumps a count, a changed one goes to
    ``maps/map_NNNNNN.npz`` with its metadata a line of ``maps.jsonl``; a run of repeats is closed by a
    ``{"repeat_of": i, "repeats": n, ...}`` line. Every map keeps its own index, as before."""

    def __init__(self, output_dir: Path, persistent: bool) -> None:
        self.output_dir, self.persistent = Path(output_dir), bool(persistent)
        self.maps: list[dict] = []
        self.map_arrays: dict[str, np.ndarray] = {}
        self.received = 0
        self._last = None  # persistent: (meta, data) of the last map written
        self._repeats = None  # persistent: the open run of repeats of the last map written

    def add(self, *, stamp_ns: int, resolution_m: float, width: int, height: int, origin: list, data: np.ndarray,
            after_scan_id: int, received_wall_s: float, frame_id: str = "map") -> None:
        index = self.received
        self.received += 1
        meta = {"index": index, "after_scan_id": after_scan_id, "stamp_ns": stamp_ns,
                "received_wall_s": received_wall_s, "resolution_m": resolution_m, "width": width, "height": height,
                "origin": origin}
        if self.persistent:
            meta["frame_id"] = frame_id
        if not self.persistent:
            self.maps.append(meta)
            self.map_arrays[f"map_{index:04d}"] = data
        else:
            last = self._last
            if (last is not None and last[0]["resolution_m"] == resolution_m and last[0]["origin"] == origin
                    and last[0]["frame_id"] == frame_id and last[1].shape == data.shape
                    and np.array_equal(last[1], data)):
                run = self._repeats or {"repeat_of": last[0]["index"], "repeats": 0}
                run.update(repeats=run["repeats"] + 1, last_index=index, last_stamp_ns=stamp_ns,
                           last_after_scan_id=after_scan_id, last_received_wall_s=received_wall_s)
                self._repeats = run
            else:
                self._close_repeats()
                folder = self.output_dir / "maps"
                folder.mkdir(parents=True, exist_ok=True)
                tmp = folder / f"map_{index:06d}.tmp.npz"
                np.savez_compressed(tmp, data=data)
                os.replace(tmp, folder / f"map_{index:06d}.npz")
                self._append_line(meta)
                self._last = (meta, data)
        tmp = self.output_dir / "latest_map.tmp.npz"
        np.savez(tmp, data=data, origin=np.asarray(origin, dtype=float), resolution_m=float(resolution_m),
                 index=int(index), after_scan_id=int(after_scan_id), stamp_ns=int(stamp_ns))
        os.replace(tmp, self.output_dir / "latest_map.npz")

    def _append_line(self, entry: dict) -> None:
        with open(self.output_dir / "maps.jsonl", "a") as handle:
            handle.write(json.dumps(entry) + "\n")

    def _close_repeats(self) -> None:
        if self._repeats is not None:
            self._append_line(self._repeats)
            self._repeats = None

    def save(self) -> None:
        if self.persistent:
            self._close_repeats()
            return
        (self.output_dir / "maps.json").write_text(json.dumps(self.maps))
        if self.map_arrays:
            np.savez_compressed(self.output_dir / "maps.npz", **self.map_arrays)


class ReplyRecords:
    """The bridge's reply records. A batch bridge keeps them for ``bridge_records.json``; a persistent one hands
    each, as a line of ``records.jsonl``, to a writer thread of its own and keeps only what the verdict needs (plan
    D8b). The serve thread never waits on the disk -- it reads the next scan right after the append, and the
    runner's reply limit is 1 s (Codex D8b round 2 P2). More than ``queue_records`` lines waiting drops the line,
    counted in ``dropped`` (a write error too); lines still waiting or being written when ``save`` gives up are
    ``unsettled``, and what ``save`` reported stays (Codex D8b round 3 P2-1)."""

    def __init__(self, output_dir: Path, persistent: bool, queue_records: int = 10_000) -> None:
        self.output_dir, self.persistent = Path(output_dir), bool(persistent)
        self.kept: list[dict] = []
        self.failed = 0
        self.any_stray = False
        self.dropped = 0
        self.unsettled = 0
        self.queue_records = int(queue_records)
        self._inflight = 0
        self._closed = False
        self._lines: deque = deque()
        self._cond = threading.Condition()
        self._closing = False
        self._file = None
        self._writer = None

    def append(self, record: dict) -> None:
        if record["status"] not in ("processed", "skipped"):
            self.failed += 1
        self.any_stray = self.any_stray or bool(record.get("stray_pose_stamps_ns"))
        if not self.persistent:
            self.kept.append(record)
            return
        line = json.dumps(record) + "\n"
        with self._cond:
            if self._writer is None:
                self._writer = threading.Thread(target=self._write_loop, name="bridge-records", daemon=True)
                self._writer.start()
            if len(self._lines) >= self.queue_records:
                self.dropped += 1
                return
            self._lines.append(line)
            self._cond.notify()

    def _write_line(self, line: str) -> None:
        if self._file is None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self._file = open(self.output_dir / "records.jsonl", "a")  # noqa: SIM115 -- closed by the writer
        self._file.write(line)
        self._file.flush()

    def _write_loop(self) -> None:
        while True:
            with self._cond:
                while not self._lines and not self._closing:
                    self._cond.wait(0.5)
                if not self._lines:
                    break
                line = self._lines.popleft()
                self._inflight = 1
            try:
                self._write_line(line)
                ok = True
            except OSError:
                ok = False
            with self._cond:
                self._inflight = 0
                if not ok and not self._closed:
                    self.dropped += 1
        if self._file is not None:
            self._file.close()

    def save(self, timeout_s: float = 30.0) -> None:
        if self.persistent:
            with self._cond:
                self._closing = True
                self._cond.notify()
                writer = self._writer
            if writer is not None:
                writer.join(timeout_s)
            with self._cond:
                self._closed = True
                self.unsettled = len(self._lines) + self._inflight  # waiting or being written at the deadline
                self._lines.clear()
            return
        (self.output_dir / "bridge_records.json").write_text(json.dumps(self.kept))


class Localization:
    """A localization bridge's bundle and its start checks (SLAM localization plan L4). A bundle that cannot be used
    is kept as ``error`` (``slam_map_mismatch: ...``): the bridge still opens its socket and answers the first scan
    ``failed`` with it, so the reason reaches the runner and its supervisor (Codex S3 design P2)."""

    def __init__(self, bundle, error, start, laser):
        self.bundle, self.error, self.start, self.laser = bundle, error, tuple(float(v) for v in start), laser
        self.first_processed = False
        self.map: slam_map.Grid | None = None
        self.fractions: list[float] = []
        self.poses: list[tuple] = []
        self.coverage: float | None = None
        self.passed = False
        self.failure: str | None = None

    @classmethod
    def from_parameters(cls, folder: str, expected: str, start, laser) -> "Localization | None":
        if not folder:
            return None
        try:
            bundle, error = slam_map.load_bundle(Path(folder), expected or None), None
        except slam_map.MapBundleError as exc:
            bundle, error = None, str(exc)
        if bundle is not None and not expected:
            bundle, error = None, f"{slam_map.MISMATCH}: no expected map id given"
        return cls(bundle, error, start, laser)

    def add_bootstrap(self, scan: slam_link.Scan, base_pose) -> None:
        x, y, _, yaw = self.laser
        laser_pose = compose(tuple(base_pose), (x, y, yaw))
        self.fractions.append(slam_map.endpoint_fraction(
            self.bundle.grid, laser_pose, scan.ranges_m, scan.angle_min_rad, scan.angle_increment_rad,
            scan.range_min_m, scan.range_max_m))
        self.poses.append(tuple(float(v) for v in base_pose))

    def check(self) -> str | None:
        """None when both start checks pass, else the reason."""
        if self.map is None:
            self.failure = f"{slam_map.NOT_LOADED}: no /map after the first processed scan"
            return self.failure
        self.coverage = slam_map.map_coverage(self.bundle.grid, self.map)
        if not self.coverage >= slam_map.MAP_LOADED_MIN:
            self.failure = f"{slam_map.NOT_LOADED}: the /map shows {self.coverage:.3f} of the survey's occupied cells"
            return self.failure
        if not slam_map.localized(self.fractions, self.poses, self.start):
            self.failure = (f"{slam_map.NOT_LOCALIZED}: endpoint fractions {[round(f, 3) for f in self.fractions]}, "
                            f"poses {[[round(v, 3) for v in p] for p in self.poses]}")
            return self.failure
        self.passed = True
        return None

    def record(self) -> dict:
        return {"map_id": None if self.bundle is None else self.bundle.map_id, "error": self.error,
                "map_loaded": self.coverage is not None and self.coverage >= slam_map.MAP_LOADED_MIN,
                "coverage": self.coverage, "endpoint_fractions": self.fractions, "bootstrap_poses": self.poses,
                "start": list(self.start), "passed": self.passed, "failure": self.failure}


class IsaacSlamBridge(Node):
    def __init__(self):
        super().__init__("isaac_slam_bridge")
        self.declare_parameter("socket_path", "/run/slam/slam.sock")
        self.declare_parameter("output_dir", "/tmp/slam_bridge")
        self.declare_parameter("laser_xyz_yaw", [-0.12, 0.0, 1.05, 0.0])
        self.declare_parameter("reply_timeout_s", 60.0)
        self.declare_parameter("warmup_wait_s", 1.0)
        self.declare_parameter("persistent", False)  # a live session left up for hours (plan D8b)
        # SLAM localization plan L4: a survey bundle folder ('' = mapping, today's path), the session's map id and
        # the start pose slam_toolbox is told (map <- base_link)
        self.declare_parameter("localization_bundle", "")
        self.declare_parameter("expected_map_id", "")
        self.declare_parameter("map_start_pose", [0.0, 0.0, 0.0])
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
        # Its own group (plan D8b, Codex D8b round 1 P2): the default group is mutually exclusive, so a map being
        # stored (compressed to disk in a persistent bridge) would hold up the /pose callback a reply waits on.
        self.map_group = MutuallyExclusiveCallbackGroup()
        self._map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos,
                                                 callback_group=self.map_group)
        persistent = bool(self.get_parameter("persistent").value)
        self.map_lock = threading.Lock()  # metadata and grids change together
        self.map_store = MapStore(self.output_dir, persistent)
        self.last_scan_id = -1
        self.replies = ReplyRecords(self.output_dir, persistent)
        self.localization = Localization.from_parameters(
            self.get_parameter("localization_bundle").value, self.get_parameter("expected_map_id").value,
            self.get_parameter("map_start_pose").value, self.laser)

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
        loc = self.localization
        with self.cond:
            if loc is not None and loc.map is not None:
                return  # localization keeps the first /map after the first processed scan only (plan L3)
            # decided at the entry (Codex S3 review P2): a map that came before the first processed reply is not
            # adopted because the reply landed while it was being stored
            eligible = loc is not None and loc.first_processed
        with self.map_lock:
            self._add_map(msg)
        if eligible:
            info = msg.info
            grid = slam_map.Grid(np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width),
                                 (info.origin.position.x, info.origin.position.y, quaternion_yaw(info.origin.orientation)),
                                 float(info.resolution))
            with self.cond:
                if loc.map is None:
                    loc.map = grid
                self.cond.notify_all()

    def _add_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self.map_store.add(
            stamp_ns=stamp_ns(msg.header.stamp),
            resolution_m=info.resolution,
            width=info.width,
            height=info.height,
            origin=[info.origin.position.x, info.origin.position.y, quaternion_yaw(info.origin.orientation)],
            frame_id=msg.header.frame_id,
            # The runner's planner reads the newest map while the run goes on (P5 D3 delta, 2026-10-06): written
            # whole, then renamed, so a reader never sees half a file.
            data=np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width),
            after_scan_id=self.last_scan_id,
            received_wall_s=time.monotonic(),
        )

    def _drop_map_subscription(self) -> None:
        """No /map subscriber: slam_toolbox's updateMap then skips the grid's rebuild (plan L3)."""
        sub, self._map_sub = getattr(self, "_map_sub", None), None
        if sub is not None:
            self.destroy_subscription(sub)

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
        if self.localization is not None and self.localization.error is not None:
            return  # no slam_toolbox was started on an unusable bundle: the first scan is answered with the reason
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
        loc = self.localization
        gate = (slam_link.KeyframeGate(startup_scans=7, startup_expect=(1, 5, 6, 7)) if loc is not None
                else slam_link.KeyframeGate())
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
                startup_index = gate.sent if kind != "local" and gate.sent <= gate.startup_scans else None
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
                reason, map_id = None, None
                if loc is not None and loc.error is not None:
                    status, reason = "failed", loc.error  # the bundle could not be used (slam_map_mismatch: ...)
                elif stray or not slam_alive:
                    status = "failed"
                elif pose is not None:
                    map_from_odom = compose(pose, invert(scan.odom_from_base))
                    version += 1
                    status = "processed"
                    gate.processed(scan.stamp_s, scan.odom_from_base)
                    if loc is not None:
                        with self.cond:
                            loc.first_processed = True
                        if startup_index in (5, 6, 7):
                            loc.add_bootstrap(scan, pose)
                        if startup_index == 7:
                            # the checks before the 7th reply, within this scan's one deadline (pose + /map)
                            remaining = max(0.0, self.reply_timeout_s - (time.monotonic() - start))
                            with self.cond:
                                self.cond.wait_for(lambda: loc.map is not None, timeout=remaining)
                            reason = loc.check()
                            if reason is not None:
                                status = "failed"
                            else:
                                map_id = loc.bundle.map_id
                            self._drop_map_subscription()
                        elif loc.passed:
                            map_id = loc.bundle.map_id
                elif expected:
                    status = "failed"
                else:
                    status = "skipped"
                elapsed = time.monotonic() - start
                reply = slam_link.Reply(
                    scan.scan_id, scan.stamp_s, status, map_from_odom, version, elapsed, reason=reason, map_id=map_id
                )
                record_ = {
                    "scan_id": scan.scan_id,
                    "stamp_s": scan.stamp_s,
                    "status": status,
                    "sent": kind,
                    "stray_pose_stamps_ns": stray,
                    "slam_alive": slam_alive,
                    "map_from_odom": list(map_from_odom),
                    "slam_pose": list(pose) if pose else None,
                    "wall_s": elapsed,
                    **({"reason": reason} if reason is not None else {}),
                }
                try:
                    conn.sendall(slam_link.encode_reply(reply))
                finally:
                    # After the reply (plan D8b, Codex D8b round 1 P2): a persistent bridge writes the record to
                    # disk, and the runner waits on the reply with a 1 s limit.
                    self.replies.append(record_)
                if status == "failed":
                    break
        server.close()

    def _write_status(self, status: dict) -> None:
        tmp = self.output_dir / "bridge_status.tmp.json"
        tmp.write_text(json.dumps(status))
        os.replace(tmp, self.output_dir / "bridge_status.json")

    def save(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        # Pending until the records are settled (Codex D8b round 3 P2-2): a container removed while they drain
        # leaves a status that does not read as final and clean.
        self._write_status({"pending": True, "ok": False})
        with self.cond:
            stray = list(self.stray)
        if stray and not self.replies.any_stray:
            # Arrived after the last reply: keep it in the record.
            self.replies.append({"status": "stray_after_last_reply", "stray": stray})
        self.replies.save()
        with self.map_lock:
            self.map_store.save()
        failed = self.replies.failed
        # The verdict a run summary must read: a stray after the last reply
        # never reached the runner (Codex v3.4 P2). A persistent bridge's record lines it could not keep, or had
        # not written by the deadline, are counted beside it.
        self.ok = not stray and not failed
        status = {"ok": self.ok, "stray_pose_stamps_ns": stray, "failed": failed}
        loc = getattr(self, "localization", None)  # a bridge built without __init__ (the rule tests) has none
        if loc is not None:
            status["localization"] = loc.record()
        if self.replies.persistent:
            status.update(records_dropped=self.replies.dropped, records_unsettled=self.replies.unsettled)
        self._write_status(status)


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
        status = 0 if node.ok else 3
        node.destroy_node()
        rclpy.shutdown()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
