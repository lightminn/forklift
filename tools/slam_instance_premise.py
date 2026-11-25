#!/usr/bin/env python3
"""PR 7c-1a's first test (docs/design/2026-11-25-real-sources-pr7.md 6.1, 2 and 9): does the bridge-owned slam_toolbox
instance replacement hold on the real stack?

Inside the ROS container (forklift/ros2-dev:jazzy), on a survey bundle, with scans ray-cast from the bundle's own grid
(no Isaac): instance 1 starts at pose A, instance 2 at pose B after instance 1 is stopped. For each instance it records

- whether the node started directly (``ros2 run``, its own process group) needs and accepts the lifecycle
  ``configure`` -> ``activate`` transitions, and when it subscribes to its own scan topic;
- which of the seven start scans get a ``/pose`` (the expected 1, 5, 6, 7) and how far each is from the start pose;
- the ``/map`` coverage of the survey's occupied cells (L4's load check);
- after the stop: the process group reaped, the node gone from the ROS graph, and any message from the stopped
  instance's topics received after the reap (expected 0).

    python3 tools/slam_instance_premise.py BUNDLE_DIR MAP_ID OUT_DIR

Writes OUT_DIR/premise.json and prints a one-line summary; exit 0 when every check holds, 1 otherwise.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))
from forklift_core.localization import slam_map  # noqa: E402
from slam_localization_params import localization_params  # noqa: E402

MAPPING_PARAMS = ROOT / "ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml"
BEAMS, RANGE_MAX = 360, 12.0
EXPECT = (1, 5, 6, 7)


def stamp_of(ns: int) -> Time:
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def ns_of(t: Time) -> int:
    return t.sec * 1_000_000_000 + t.nanosec


def quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def raycast(grid: slam_map.Grid, pose, beams: int = BEAMS, range_max: float = RANGE_MAX) -> np.ndarray:
    """Ranges from ``pose`` (laser = base) to the grid's first occupied cell (inf past ``range_max``)."""
    x, y, yaw = pose
    angles = yaw - math.pi + np.arange(beams) * (2 * math.pi / beams)
    steps = np.arange(0.05, range_max, grid.resolution_m / 2)
    px = x + np.cos(angles)[:, None] * steps[None, :]
    py = y + np.sin(angles)[:, None] * steps[None, :]
    rows, cols, inside = grid.cells_of(np.column_stack((px.ravel(), py.ravel())))
    occ = np.zeros(rows.shape, bool)
    occ[inside] = grid.occupied()[rows[inside], cols[inside]]
    occ = occ.reshape(beams, len(steps))
    hit = occ.any(axis=1)
    first = occ.argmax(axis=1)
    return np.where(hit, steps[first], np.inf)


def free_at(grid: slam_map.Grid, pose) -> bool:
    rows, cols, inside = grid.cells_of(np.array([[pose[0], pose[1]]]))
    return bool(inside[0]) and not bool(grid.occupied()[rows[0], cols[0]])


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("premise_driver")
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", latched)
        self.lock = threading.Lock()
        self.poses: dict = {}  # k -> {stamp_ns: (x, y, yaw, recv_mono)}
        self.maps: dict = {}  # k -> [OccupancyGrid]
        self.after: dict = {}  # k -> messages received after that instance was reaped
        self.reaped: dict = {}  # k -> monotonic ns of the reap
        self.scan_pubs: dict = {}
        self.subs: dict = {}
        self.create_timer(0.01, self._tick)

    def _tick(self) -> None:
        self.clock_pub.publish(Clock(clock=stamp_of(time.monotonic_ns())))

    def static_tf(self) -> None:
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "base_link", "laser"
        t.transform.rotation.w = 1.0
        self.tf_static_pub.publish(TFMessage(transforms=[t]))

    def watch(self, k: int) -> None:
        self.poses[k], self.maps[k], self.after[k] = {}, [], 0
        self.scan_pubs[k] = self.create_publisher(LaserScan, f"/slam_{k}/scan", 10)
        self.subs[k] = (
            self.create_subscription(PoseWithCovarianceStamped, f"/slam_{k}/pose", lambda m, k=k: self._pose(k, m), 10),
            self.create_subscription(OccupancyGrid, f"/slam_{k}/map", lambda m, k=k: self._map(k, m),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE)))

    def _late(self, k: int) -> bool:
        if k in self.reaped:
            self.after[k] += 1
            return True
        return False

    def _pose(self, k: int, m) -> None:
        with self.lock:
            if self._late(k):
                return
            p = m.pose.pose
            self.poses[k][ns_of(m.header.stamp)] = (p.position.x, p.position.y, yaw_of(p.orientation),
                                                    time.monotonic_ns())

    def _map(self, k: int, m) -> None:
        with self.lock:
            if not self._late(k):
                self.maps[k].append(m)

    def send(self, k: int, ranges: np.ndarray) -> int:
        ns = time.monotonic_ns()
        stamp = stamp_of(ns)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_link"
        t.transform.rotation.w = 1.0
        self.tf_pub.publish(TFMessage(transforms=[t]))
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp, "laser"
        msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / BEAMS
        msg.angle_max = msg.angle_min + msg.angle_increment * (BEAMS - 1)
        msg.range_min, msg.range_max = 0.1, RANGE_MAX
        msg.ranges = [float(r) for r in ranges]
        self.scan_pubs[k].publish(msg)
        return ns


def run_cli(args, timeout=20.0) -> tuple[int, str]:
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()[-400:]
    except subprocess.TimeoutExpired:
        return -1, "timeout"


def start_instance(k: int, bundle, start, out: Path):
    mapping = yaml.safe_load(MAPPING_PARAMS.read_text())
    params = localization_params(mapping, str(bundle.graph_prefix), list(start))
    p = dict(params["slam_toolbox"]["ros__parameters"])
    p.update(scan_topic=f"/slam_{k}/scan", map_name=f"/slam_{k}/map", transform_publish_period=0.0, use_sim_time=True)
    path = out / f"params_{k}.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": p}}))
    log = open(out / f"slam_{k}.log", "w")  # noqa: SIM115 -- kept for the process's life
    proc = subprocess.Popen(["ros2", "run", "slam_toolbox", "localization_slam_toolbox_node", "--ros-args",
                             "--params-file", str(path), "-r", f"__ns:=/slam_{k}"],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc


def stop_instance(proc, node: Driver, k: int, name: str) -> dict:
    t0 = time.monotonic()
    out = {"sigkill": False}
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        out["sigkill"] = True
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)
    # the whole group gone (ros2 run's leader exits before the node it started): wait for it, then SIGKILL the rest
    def members():
        return subprocess.run(["pgrep", "-g", str(proc.pid)], capture_output=True, text=True).stdout.split()
    out["group_after_leader"] = members()
    deadline = time.monotonic() + 5
    while members() and time.monotonic() < deadline:
        time.sleep(0.1)
    if members():
        out["group_sigkill"] = True
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 5
        while members() and time.monotonic() < deadline:
            time.sleep(0.1)
    left = members()
    out["group_left"] = left
    gone = False
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        names = node.get_node_names_and_namespaces()
        if not any(n == name and ns == f"/slam_{k}" for n, ns in names):
            gone = True
            break
        time.sleep(0.2)
    out["graph_gone"] = gone
    with node.lock:
        node.reaped[k] = time.monotonic_ns()
    out["stop_s"] = round(time.monotonic() - t0, 3)
    return out


def bring_up(proc, node: Driver, k: int) -> dict:
    out = {}
    fq = f"/slam_{k}/slam_toolbox"
    deadline = time.monotonic() + 30
    state = None
    while time.monotonic() < deadline:
        code, text = run_cli(["ros2", "lifecycle", "get", fq], timeout=10)
        if code == 0:
            state = text
            break
        time.sleep(0.5)
    out["lifecycle_get"] = state
    out["lifecycle_node"] = state is not None
    if state is not None:
        for tr in ("configure", "activate"):
            code, text = run_cli(["ros2", "lifecycle", "set", fq, tr], timeout=20)
            out[f"lifecycle_{tr}"] = {"code": code, "text": text}
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30 and node.count_subscribers(f"/slam_{k}/scan") == 0:
        time.sleep(0.1)
    out["subscribed"] = node.count_subscribers(f"/slam_{k}/scan") > 0
    out["subscribe_wait_s"] = round(time.monotonic() - t0, 3)
    node.static_tf()
    time.sleep(1.0)
    return out


def start_check(node: Driver, k: int, grid, start) -> dict:
    ranges = raycast(grid, start)
    rows = []
    for i in range(1, 8):
        ns = node.send(k, ranges)
        deadline = time.monotonic() + (3.0 if i in EXPECT else 1.0)
        got = None
        while time.monotonic() < deadline:
            with node.lock:
                got = node.poses[k].get(ns)
            if got is not None:
                break
            time.sleep(0.02)
        row = {"i": i, "pose": got is not None}
        if got is not None:
            pos, yaw = slam_map.start_pose_error(got[:3], start)
            row.update(position_m=round(pos, 4), yaw_rad=round(yaw, 4), latency_s=round((got[3] - ns) / 1e9, 3))
        rows.append(row)
        time.sleep(0.2)
    with node.lock:
        maps = list(node.maps[k])
    cov = None
    if maps:
        m = maps[-1]
        live = slam_map.Grid(np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width),
                             (m.info.origin.position.x, m.info.origin.position.y, yaw_of(m.info.origin.orientation)),
                             float(m.info.resolution))
        cov = round(float(slam_map.map_coverage(grid, live)), 4)
    answered = tuple(r["i"] for r in rows if r["pose"])
    near = all(r.get("position_m", 9) <= slam_map.START_POSITION_M and r.get("yaw_rad", 9) <= slam_map.START_YAW_RAD
               for r in rows if r["i"] in EXPECT and r["pose"])
    return {"scans": rows, "answered": answered, "expected_answered": answered == EXPECT, "near_start": near,
            "map_coverage": cov, "map_loaded": cov is not None and cov >= slam_map.MAP_LOADED_MIN}


def main(argv) -> int:
    bundle_dir, map_id, out = Path(argv[1]), argv[2], Path(argv[3])
    out.mkdir(parents=True, exist_ok=True)
    bundle = slam_map.load_bundle(bundle_dir, expected_map_id=map_id)
    grid = bundle.grid
    a = (-1.7529200315475464, 0.0, 0.0)  # random_m registry start (map <- base_link)
    b = None
    for dx, dy, dyaw in ((0.3, 0.0, 0.1), (0.4, 0.2, 0.15), (0.3, -0.2, -0.1), (-0.3, 0.0, 0.1)):
        cand = (a[0] + dx, a[1] + dy, a[2] + dyaw)
        if free_at(grid, cand):
            b = cand
            break
    result_extra = {"finite_beams_a": float(np.isfinite(raycast(grid, a)).mean())}
    result = {"bundle": str(bundle_dir), "map_id": map_id, "a": a, "b": b, **result_extra,
              "slam_toolbox": run_cli(["ros2", "pkg", "xml", "slam_toolbox", "--tag", "version"])[1]}
    rclpy.init()
    node = Driver()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    ok = b is not None
    try:
        for k, pose in ((1, a), (2, b)):
            if pose is None:
                break
            node.watch(k)
            proc = start_instance(k, bundle, pose, out)
            r = {"bring_up": bring_up(proc, node, k)}
            r["start_check"] = start_check(node, k, grid, pose)
            if k == 1:
                r["stop"] = stop_instance(proc, node, k, "slam_toolbox")
                time.sleep(3.0)  # anything the stopped instance still had in flight
                r["after_reap_messages"] = node.after[k]
            else:
                r["stop"] = stop_instance(proc, node, k, "slam_toolbox")
            result[f"instance_{k}"] = r
            sc, st = r["start_check"], r["stop"]
            ok = ok and r["bring_up"]["subscribed"] and sc["expected_answered"] and sc["near_start"] \
                and sc["map_loaded"] and st["graph_gone"] and not st["group_left"]
        ok = ok and result.get("instance_1", {}).get("after_reap_messages", 1) == 0
    finally:
        result["ok"] = bool(ok)
        (out / "premise.json").write_text(json.dumps(result, indent=1, default=str))
        print(json.dumps({"ok": result["ok"], **{k: {"answered": result[k]["start_check"]["answered"],
                                                      "map_coverage": result[k]["start_check"]["map_coverage"],
                                                      "lifecycle": result[k]["bring_up"].get("lifecycle_node")}
                                                  for k in ("instance_1", "instance_2") if k in result}}))
        sys.stdout.flush()
    # rclpy's teardown with the executor thread still spinning aborts the process (exit 139 seen in job 21220): the
    # record is written, so leave without it
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    main(sys.argv)
