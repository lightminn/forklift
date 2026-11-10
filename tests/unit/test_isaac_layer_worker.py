"""Real-time plan 5.28 G3-1e: the obstacle layer's scan refresh split into prepare / compute / install, and the layer
worker (delay, thread) that installs it at the scan tick + k or at an earlier barrier. No Isaac."""

import copy
import importlib.util
import json
import math
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.perception.obstacle_grid import AgeErrorTable
from forklift_core.perf import TIMERS
from forklift_core.planning.geometry import Bounds, Footprint

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
SPEC = importlib.util.spec_from_file_location("obstacle_layer", ROOT / "sim/isaac/obstacle_layer.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["obstacle_layer"] = MODULE
SPEC.loader.exec_module(MODULE)
import layer_worker as lw  # noqa: E402

CONFIG = MODULE.load_layer_config(ROOT / "config/obstacle_layer_single_team_spec.yaml")
AGE = json.loads((ROOT / CONFIG["odometry_age"]).read_text())
TABLE = AgeErrorTable(tuple(AGE["ages_s"]), tuple(AGE["cumulative_position_m"]), tuple(AGE["cumulative_yaw_rad"]))
PATH = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))


def layer():
    return MODULE.ObstacleLayer(
        CONFIG, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
        unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
        body_front_m=0.884, rear_axle_x_in_base_m=-0.34, noise_seed=1,
    )


def raw_scan(lay, pose, wall_x=2.4, box=None):
    """One scan of a wall ahead (and a box) as the high LiDAR sees it from ``pose`` (rear axle, map frame)."""
    out = {}
    for s in lay.sensors:
        yaw = pose[2] + s.yaw_rad
        ox = pose[0] + (s.xyz_m[0] + 0.34) * math.cos(pose[2])
        oy = pose[1] + (s.xyz_m[0] + 0.34) * math.sin(pose[2])
        ang = lay.beam_angles + yaw
        c, sn = np.cos(ang), np.sin(ang)
        d = np.where(c > 0.05, (wall_x - ox) / np.where(c > 0.05, c, 1.0), np.nan)
        if box is not None:  # a box face at x = box[0] for |y - box[1]| < box[2]
            db = np.where(c > 0.05, (box[0] - ox) / np.where(c > 0.05, c, 1.0), np.nan)
            yb = oy + db * sn
            take = np.isfinite(db) & (np.abs(yb - box[1]) < box[2]) & (db < np.nan_to_num(d, nan=np.inf))
            d = np.where(take, db, d)
        ok = np.isfinite(d) & (d > 0) & (np.abs(oy + d * sn) < 3)
        out[s.name] = (np.where(ok, d, np.nan), ok, np.zeros(len(ang), bool))
    return out


def poses():
    """A short drive: forward with a turn, a correction that changes half way."""
    for k in range(8):
        pose = (0.05 * k, 0.01 * k, 0.02 * k)
        corr = (0.0, 0.0, 0.0) if k < 4 else (0.01, -0.005, 0.002)
        yield k, 0.1 * k, pose, corr


def same_snapshot(a, b):
    assert np.array_equal(a.state, b.state)
    assert np.array_equal(a.free_stamp, b.free_stamp, equal_nan=True)
    assert (a.origin_x_m, a.origin_y_m, a.stamp_s, a.correction_version) == \
        (b.origin_x_m, b.origin_y_m, b.stamp_s, b.correction_version)


def same_check(a, b):
    assert (a.verified_m, a.blocked, a.oldest_free_s, a.reached_end) == \
        (b.verified_m, b.blocked, b.oldest_free_s, b.reached_end)


def test_a_detached_job_gives_what_the_synchronous_refresh_gives():
    sync, split = layer(), layer()
    for k, t, pose, corr in poses():
        for lay in (sync, split):
            lay.add_scans(t, raw_scan(lay, pose, box=(1.6, 0.4, 0.2)), odom_rear=pose, loaded=False)
        c1 = sync.refresh(t, corr, k // 4, current_pose=pose, path_ahead=PATH, loaded=False)
        job = split.refresh_prepare(t, corr, k // 4, current_pose=pose, path_ahead=PATH, loaded=False, detach=True)
        c2 = split.refresh_install(job, split.refresh_compute(job))
        same_snapshot(sync.snapshot, split.snapshot)
        same_check(c1, c2)
        assert list(sync.shadow.memory.items()) == list(split.shadow.memory.items())
        assert sync.checked_scan_s == split.checked_scan_s == sync.last_scan_s
    assert sync.memory.last_scan_stamp == split.memory.last_scan_stamp


def run_worker(mode, k=3):
    """Scans every 12 ticks; the worker's installs as (scan tick, install tick, reason) and the installed snapshots."""
    lay = layer()
    w = lw.LayerWorker(lay, mode, k)
    lay.worker = w
    out = []
    w.on_install = lambda job, check, info: out.append(
        (info["scan_tick"], info["install_tick"], info["reason"], copy.deepcopy(lay.snapshot), check))
    for k_, t, pose, corr in poses():
        scan_tick = 12 * k_
        for tick in range(scan_tick - 11 if k_ else 0, scan_tick + 1):
            w.tick_start(tick)
        lay.add_scans(t, raw_scan(lay, pose, box=(1.6, 0.4, 0.2)), odom_rear=pose, loaded=False)
        job = lay.refresh_prepare(t, corr, k_ // 4, current_pose=pose, path_ahead=PATH, loaded=False, detach=True)
        w.submit(job, scan_tick)
        assert lay.checked_scan_s != lay.last_scan_s  # received, not yet checked
    for tick in range(12 * 7 + 1, 12 * 7 + 6):
        w.tick_start(tick)
    return lay, w, out


@pytest.mark.parametrize("k", [2, 3, 4])
def test_delay_and_thread_install_the_same_results_at_the_scheduled_tick(k):
    _, _, delay = run_worker("delay", k)
    _, wt, thread = run_worker("thread", k)
    assert [(a, b, c) for a, b, c, _, _ in delay] == [(a, b, c) for a, b, c, _, _ in thread]
    assert all(b - a == k and c == "scheduled" for a, b, c, _, _ in thread)  # the recorded ticks, not the formula
    for d, th in zip(delay, thread):
        same_snapshot(d[3], th[3])
        same_check(d[4], th[4])
    # and each installed snapshot is the synchronous refresh's of the same scan
    sync = layer()
    for (k_, t, pose, corr), inst in zip(poses(), thread):
        sync.add_scans(t, raw_scan(sync, pose, box=(1.6, 0.4, 0.2)), odom_rear=pose, loaded=False)
        sync.refresh(t, corr, k_ // 4, current_pose=pose, path_ahead=PATH, loaded=False)
        same_snapshot(sync.snapshot, inst[3])
    assert all(r["tick_t"].keys() >= {1, 2} for r in wt.records[:-1])  # the paired slack for 1e-0


@pytest.mark.parametrize("touch", ["refresh", "planner", "add_scans", "set_exempt", "set_known", "deepcopy",
                                   "exit_exempt_clear"])
def test_a_barrier_installs_the_pending_refresh_first(touch):
    lay = layer()
    w = lw.LayerWorker(lay, "thread", 3)
    lay.worker = w
    pose = (0.0, 0.0, 0.0)
    lay.add_scans(0.0, raw_scan(lay, pose), odom_rear=pose, loaded=False)
    w.submit(lay.refresh_prepare(0.0, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=PATH, loaded=False,
                                 detach=True), 0)
    assert w.pending
    if touch == "refresh":
        lay.refresh(0.05, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=PATH, loaded=False)
    elif touch == "planner":
        lay.planner_grid(0.05, (0.0, 0.0, 0.0), 0)
    elif touch == "add_scans":
        lay.add_scans(0.1, raw_scan(lay, pose), odom_rear=pose, loaded=False)
    elif touch == "set_exempt":
        lay.exempt = None
    elif touch == "set_known":
        lay.known = []
    elif touch == "deepcopy":
        dup = copy.deepcopy(lay)
        assert dup.worker is None and dup.snapshot is not None
    else:
        lay.exit_exempt_clear("test")
    assert not w.pending and w.records[0]["reason"] == {"set_exempt": "set_exempt", "set_known": "set_known"}.get(
        touch, touch)
    assert w.records[0]["install_tick"] is None


def test_worker_timers_are_merged_on_the_main_thread_at_install(monkeypatch):
    lay = layer()
    real = lay.refresh_compute
    gate = threading.Event()

    def compute(job):
        TIMERS.add("worker.test", 0.25, 0.25)  # captured: the shared dicts are not touched off the main thread
        gate.wait(5)
        return real(job)

    monkeypatch.setattr(lay, "refresh_compute", compute)
    w = lw.LayerWorker(lay, "thread", 3)
    object.__setattr__(lay, "worker", w)
    pose = (0.0, 0.0, 0.0)
    lay.add_scans(0.0, raw_scan(lay, pose), odom_rear=pose, loaded=False)
    TIMERS.take()
    w.submit(lay.refresh_prepare(0.0, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=PATH, loaded=False,
                                 detach=True), 0)
    assert "worker.test" not in TIMERS.take()  # a take while the worker runs
    gate.set()
    w.install_now("test")
    assert TIMERS.take()["worker.test"][1] == 1


def test_a_worker_failure_or_timeout_raises_and_the_worker_stays_failed(monkeypatch):
    lay = layer()
    pose = (0.0, 0.0, 0.0)
    lay.add_scans(0.0, raw_scan(lay, pose), odom_rear=pose, loaded=False)
    job = lay.refresh_prepare(0.0, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=PATH, loaded=False, detach=True)

    def boom(job):
        raise ValueError("bad scan")

    monkeypatch.setattr(lay, "refresh_compute", boom)
    w = lw.LayerWorker(lay, "thread", 3)
    w.submit(job, 0)
    with pytest.raises(lw.LayerWorkerFailure, match="layer_worker_failed: ValueError: bad scan"):
        w.install_now("test")
    with pytest.raises(lw.LayerWorkerFailure):
        w.submit(job, 12)
    stop = threading.Event()
    monkeypatch.setattr(lay, "refresh_compute", lambda job: stop.wait(10))
    w2 = lw.LayerWorker(lay, "thread", 3, timeout_s=0.05)
    w2.submit(job, 0)
    with pytest.raises(lw.LayerWorkerFailure, match="timeout"):
        w2.install_now("test")
    assert w2.failed and w2.pending
    stop.set()


def test_the_depth_support_reads_only_the_window_geometry():
    lay = layer()
    seen = []

    def support(snapshot, now_s):
        seen.append((snapshot.origin_x_m, snapshot.origin_y_m, snapshot.resolution_m, snapshot.state.shape))
        return {}

    lay.depth_support = support
    pose = (0.3, -0.2, 0.0)
    lay.add_scans(0.0, raw_scan(lay, pose), odom_rear=pose, loaded=False)
    lay.refresh(0.0, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=PATH, loaded=False)
    lay.add_scans(0.1, raw_scan(lay, pose), odom_rear=pose, loaded=False)
    job = lay.refresh_prepare(0.1, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=PATH, loaded=False, detach=True)
    assert job["depth_support"] is None and job["support_value"] == {}
    lay.refresh_install(job, lay.refresh_compute(job))
    assert seen[0] == seen[1]  # the real snapshot's geometry and the window's are the same
    with pytest.raises(AttributeError):
        MODULE.WindowGeometry(0.0, 0.0, 0.05, (10, 10)).free_stamp


def test_the_core_reads_the_checked_scan_when_a_worker_is_on():
    from forklift_core.mission.core import MissionCore

    sensors = [SimpleNamespace(name="high", may_clear=True)]
    lay = SimpleNamespace(sensors=sensors, last_scan_s={"high": 0.3}, checked_scan_s={"high": 0.2})
    core = SimpleNamespace(obstacle={"layer": lay, "last_stamp": 0.3})
    assert MissionCore._clearing_scans(core) == {"high": 0.3}
    assert MissionCore.cusp_scan_id(core, 0.31) == 0.3
    core.obstacle.update(layer_worker=True, checked_stamp=0.2)
    assert MissionCore._clearing_scans(core) == {"high": 0.2}  # the scan not yet checked does not count
    assert MissionCore.cusp_scan_id(core, 0.31) == 0.2


def test_the_near_contract_knows_the_layer_worker_arguments():
    text = (ROOT / "sim/isaac/near_tracking.py").read_text()
    assert '"layer_worker": "off", "layer_worker_k": 3, "layer_worker_timeout_s": 0.5' in text
