"""7b: the teleop bridge's scan permit (docs/design/2026-11-16-bridge-scan-permit.md) -- real scans (scan/1) and MCU
odometry into the existing obstacle layer and BundlePermit, pinned without the A2M12 by synthetic scans of boxes."""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.mcu_link.messages import (
    FaultFlag,
    McuState,
    Mode,
    RejectReason,
    StateFlag,
)
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED
from forklift_core.plant.bringup import Sample
from forklift_core.plant.real_vehicle import load_real_vehicle
from forklift_core.sensors import scan_feed as sf

REPO = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("bridge_scan", REPO / "tools" / "bridge_scan.py")
bs = importlib.util.module_from_spec(spec)
sys.modules["bridge_scan"] = bs
spec.loader.exec_module(bs)

VEH = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
CONFIG = REPO / "config" / "real" / "obstacle_layer_real.yaml"
MS = 1_000_000
T0 = 50_000 * MS  # host monotonic start
MOUNT = (0.9, 0.0, 1.05, 0.0)  # rear-axle frame x, y, z, yaw (rad) -- a stand-in until S7 measures it
WALLS = [(-6.0, 7.0, 5.0, 5.4), (-6.0, 7.0, -5.4, -5.0), (6.6, 7.0, -5.0, 5.0), (-6.4, -6.0, -5.0, 5.0)]
BOX_AHEAD = (1.55, 1.85, -0.2, 0.2)  # 0.26 m ahead of the 1.29 m front face (rear-axle frame at the origin)
BOX_BEHIND = (-0.7, -0.4, -0.2, 0.2)
N_BEAMS = 720
CURVS = [-0.05, 0.0, 0.05]


def cast(pose, mount, boxes, n=N_BEAMS, rmax=12.0):
    """Ranges of a planar scan from ``mount`` on a truck at rear-axle ``pose`` against axis-aligned boxes."""
    x, y, th = pose
    mx, my, _mz, myaw = mount
    ox, oy = x + mx * math.cos(th) - my * math.sin(th), y + mx * math.sin(th) + my * math.cos(th)
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    world = th + myaw + angles
    dx, dy = np.cos(world), np.sin(world)
    best = np.full(n, np.inf)
    for x0, x1, y0, y1 in boxes:
        with np.errstate(divide="ignore", invalid="ignore"):
            tx0, tx1 = (x0 - ox) / dx, (x1 - ox) / dx
            ty0, ty1 = (y0 - oy) / dy, (y1 - oy) / dy
        tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
        tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
        best = np.where(hit & (tmin < best), tmin, best)
    ranges = np.where(best <= rmax, best, np.nan)
    return angles, ranges


def scan(k, host_ns, ranges, angles=None, *, lag_ms=20, n=None):
    n = len(ranges) if n is None else n
    angles = -math.pi + np.arange(n) * (2 * math.pi / n) if angles is None else angles
    stamp = host_ns - lag_ms * MS
    return sf.RealScan(k, stamp, host_ns, host_ns - stamp, np.asarray(angles, float), np.asarray(ranges, float), None,
                       {"frame_id": "laser", "scan_time": 0.1, "time_increment": 0.1 / n, "range_min": 0.2,
                        "range_max": 12.0, "sys_minus_mono_ns": 0})


class Feed:
    def __init__(self):
        self.queue = []

    def poll(self):
        out, self.queue = self.queue, []
        return out


def state(t_us, counts=(0, 0), *, boot=1, steer=0.0, faults=None):
    faults = FaultFlag(0) if faults is None else faults
    return McuState(t_us, boot, 1, 0, Mode.ARMED, RejectReason.NONE, faults, int(counts[0]), int(counts[1]), 0.0, 0.0,
                    steer, 0.0, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag.DRIVERS_READY)


class World:
    """The permit, a synthetic feed and a synthetic MCU: STATE every 20 ms, a scan every 100 ms."""

    def __init__(self, boxes=(), *, mount=MOUNT, mask=None, config=CONFIG):
        self.feed = Feed()
        self.permit = bs.ScanPermit(self.feed, mount=mount, config_path=config, vehicle=VEH, v_max_mps=0.2,
                                    mask_deg=mask)
        self.boxes, self.mount = list(boxes), mount
        self.host, self.t_us, self.k, self.counts, self.boot, self.pose_x = T0, 1_000_000, 0, [0.0, 0.0], 1, 0.0
        self.override = None  # ranges to send instead of the cast

    def step(self, n=1, *, scans=True, speed=0.0):
        for _ in range(n):
            self.host += 20 * MS
            self.t_us += 20_000
            if speed:
                self.pose_x += speed * 0.02
                for i in (0, 1):
                    self.counts[i] += speed * 0.02 / VEH.wheel_radius_m[i] * VEH.counts_per_wheel_rev[i] / (2 * math.pi)
            self.permit.on_state(Sample(self.host, state(self.t_us, [round(c) for c in self.counts], boot=self.boot)),
                                 delay_s=0.03)
            if scans and (self.host - T0) % (100 * MS) == 0:
                angles, ranges = cast((self.pose_x, 0.0, 0.0), self.mount, WALLS + self.boxes)
                if self.override is not None:
                    ranges = self.override(angles, ranges)
                self.feed.queue.append(scan(self.k, self.host, ranges, angles))
                self.k += 1
            self.permit.poll(self.host)

    def ask(self, direction=1, check=0.13):
        return self.permit(CURVS, direction, check)


def test_open_floor_gives_a_nonzero_permit_and_takes_each_scans_own_angles():
    w = World()
    assert w.ask()[0] == 0.0  # nothing seen yet
    w.step(25)
    lim, why = w.ask()
    assert lim > 0.0, why
    assert w.permit.status()["scans_used"] >= 4
    # a scan with another beam count goes in with its own angles
    angles, ranges = cast((0.0, 0.0, 0.0), MOUNT, WALLS, n=500)
    w.feed.queue.append(scan(99, w.host + 20 * MS, ranges, angles))
    w.step(1, scans=False)
    assert w.ask()[0] > 0.0


def test_a_box_ahead_stops_forward_not_reverse_and_clearing_it_reopens():
    w = World([BOX_AHEAD])
    w.step(25)
    assert w.ask(1)[0] == 0.0
    assert w.ask(-1)[0] > 0.0
    w.boxes = []
    w.step(25)
    assert w.ask(1)[0] > 0.0


def test_a_box_behind_stops_reverse_only():
    w = World([BOX_BEHIND])
    w.step(25)
    assert w.ask(1)[0] > 0.0 and w.ask(-1)[0] == 0.0


def test_a_backward_mount_keeps_the_world_where_it_is():
    back = (0.9, 0.0, 1.05, math.pi)
    w = World([BOX_AHEAD], mount=back)
    w.step(25)
    assert w.ask(1)[0] == 0.0 and w.ask(-1)[0] > 0.0


def cell_state(snap, x, y):
    i = int(math.floor((x - snap.origin_x_m) / snap.resolution_m))
    j = int(math.floor((y - snap.origin_y_m) / snap.resolution_m))
    return int(snap.state[i, j])


def test_masked_beams_are_not_used_and_unknown_ahead_blocks():
    # the mount's own body hides the forward sector: the box there is not marked and what it hides stays unknown --
    # forward is closed, and so is reversing, since the hull's fork part (ahead of the body) is unseen
    w = World([BOX_AHEAD], mask=[(-30.0, 30.0)])
    w.step(25)
    snap = w.permit.layer.snapshot
    assert cell_state(snap, 1.6, 0.0) != OCCUPIED and cell_state(snap, 1.6, 0.0) != FREE
    assert w.ask(1)[0] == 0.0 and w.ask(-1)[0] == 0.0
    # a narrow mask straight behind leaves forward open and closes reversing (what it hides is in the reverse path);
    # a mask over the band beside the hull closes both -- the permission needs the truck's surroundings seen
    w2 = World(mask=[(170.0, 180.0)])
    w2.step(25)
    assert w2.ask(1)[0] > 0.0 and w2.ask(-1)[0] == 0.0
    w3 = World(mask=[(130.0, 150.0)])
    w3.step(25)
    assert w3.ask(1)[0] == 0.0


def test_unknown_ranges_ahead_block():
    w = World()
    w.override = lambda a, r: np.where(np.abs(a) < math.radians(40), np.nan, r)
    w.step(25)
    # unknown ahead closes forward; reversing too, since the hull's fork part ahead of the body is not seen free
    assert w.ask(1)[0] == 0.0 and w.ask(-1)[0] == 0.0
    w.override = None
    w.step(25)
    assert w.ask(1)[0] > 0.0


def test_a_recorded_scan_frame_replays_to_the_same_permit():
    w = World([BOX_AHEAD])
    feed = sf.ScanFeed()
    off = 1_700_000_000_000 * MS
    frames = []
    for k in range(sf.ESTABLISH_COUNT + 20):
        host = T0 + (k + 1) * 100 * MS
        angles, ranges = cast((0.0, 0.0, 0.0), MOUNT, WALLS + [BOX_AHEAD])
        head = {"scan_id": k, "ros_stamp_ns": host - 20 * MS + off, "recv_mono_ns": host, "sys_minus_mono_ns": off,
                "angle_min": float(angles[0]), "angle_increment": float(angles[1] - angles[0]),
                "time_increment": 0.1 / len(angles), "scan_time": 0.1, "range_min": 0.2, "range_max": 12.0,
                "frame_id": "laser", "n": len(angles)}
        frames.append((host, sf.encode_frame(head, np.nan_to_num(ranges, nan=0.0), None)))
    judged = {host: feed.judge(data, host)[0] for host, data in frames}
    for _ in range(5 * (sf.ESTABLISH_COUNT + 20)):
        w.host += 20 * MS
        w.t_us += 20_000
        w.permit.on_state(Sample(w.host, state(w.t_us)), delay_s=0.03)
        got = judged.get(w.host)
        if got is not None:
            w.feed.queue.append(got)
        w.permit.poll(w.host)
    assert w.permit.status()["scans_used"] >= 15
    assert w.ask(1)[0] == 0.0 and w.ask(-1)[0] > 0.0


def test_silence_closes_after_the_timeout_and_not_before():
    w = World()
    w.step(25)
    last = w.permit.status()["last_scan_s"]
    timeout = w.permit.layer.permission.config.sensor_timeout_s
    while w.host / 1e9 < last + timeout - 0.03:
        w.step(1, scans=False)
    assert w.ask()[0] > 0.0
    while w.host / 1e9 <= last + timeout + 0.02:
        w.step(1, scans=False)
    assert w.ask()[0] == 0.0


def test_late_and_early_scans_are_dropped():
    w = World()
    w.step(25)
    used = w.permit.status()["scans_used"]
    angles, ranges = cast((0.0, 0.0, 0.0), MOUNT, WALLS)
    w.feed.queue.append(scan(500, w.host + 20 * MS, ranges, angles, lag_ms=160))  # stamp -> receipt over 0.15 s
    w.step(1, scans=False)
    st = w.permit.status()
    assert st["scans_used"] == used and st["dropped"].get("late", 0) == 1
    w2 = World()
    w2.feed.queue.append(scan(0, T0 + 10 * MS, ranges, angles))  # before any STATE
    w2.permit.poll(T0 + 10 * MS)
    assert w2.permit.status()["dropped"].get("before_states", 0) == 1


def test_the_error_table_carries_the_timing_uncertainty_on_both_sides():
    w = World()
    p = w.permit
    dt = p.timing["dt_s"]
    kappa = max(abs(k) for k in VEH.steering.curvatures_inv_m)
    assert p.timing["d_e_m"] == pytest.approx(0.2 * dt) and p.timing["d_psi_rad"] == pytest.approx(0.2 * kappa * dt)
    base = p.base_table
    grid = p.layer.grid
    for age, rho in ((0.1, 1.0), (0.3, 4.0)):
        e0, psi0 = base.at(age)
        want = e0 + p.timing["d_e_m"] + 2 * rho * math.sin((psi0 + p.timing["d_psi_rad"]) / 2)
        assert grid.radius_m(age, rho, sensor=False) == pytest.approx(want)


def test_a_table_shorter_than_the_use_age_is_refused(tmp_path):
    import json

    import yaml

    cfg = yaml.safe_load(CONFIG.read_text())
    short = tmp_path / "age.json"
    short.write_text(json.dumps({"ages_s": [0.1, 0.2], "cumulative_position_m": [0.01, 0.02],
                                 "cumulative_yaw_rad": [0.001, 0.002]}))
    cfg["odometry_age"] = str(short)
    path = tmp_path / "layer.yaml"
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="age table"):
        bs.ScanPermit(Feed(), mount=MOUNT, config_path=path, vehicle=VEH, v_max_mps=0.2)


def test_an_odometry_break_closes_at_once_and_reopens_on_new_scans():
    w = World()
    w.step(25)
    assert w.ask()[0] > 0.0
    w.boot = 2  # the MCU rebooted
    w.step(1, scans=False)
    lim, why = w.ask()
    assert lim == 0.0 and w.permit.status()["breaks"] == 1
    w.step(25)
    assert w.ask()[0] > 0.0


def test_driving_advances_the_bundle_cache():
    w = World()
    w.step(25)
    w.ask()
    w.step(5, speed=0.1)
    w.ask()
    since = [e["since_m"] for e in w.permit.bundle._cache.values()]
    assert since and max(since) > 0.0 or w.permit.status()["driven_m"] > 0.0
    assert w.permit.status()["driven_m"] == pytest.approx(0.1 * 0.02 * 5, rel=0.05)


def test_bad_config_or_mount_is_refused():
    with pytest.raises(ValueError):
        bs.ScanPermit(Feed(), mount=(0.9, 0.0, float("nan"), 0.0), config_path=CONFIG, vehicle=VEH, v_max_mps=0.2)
    with pytest.raises(ValueError):
        bs.parse_mount("0.9,0.0,1.05")
    assert bs.parse_mount("0.9,0,1.05,180") == pytest.approx((0.9, 0.0, 1.05, math.pi))


def test_a_state_delay_over_the_budget_or_unknown_closes():
    w = World()
    w.step(25)
    assert w.ask()[0] > 0.0
    w.host += 20 * MS
    w.t_us += 20_000
    w.permit.on_state(Sample(w.host, state(w.t_us)), delay_s=0.05)  # over the 0.03 s the timing budget assumed
    assert w.ask() == (0.0, "state_delay_over_budget")
    w.host += 20 * MS
    w.t_us += 20_000
    w.permit.on_state(Sample(w.host, state(w.t_us)), delay_s=None)
    assert w.ask() == (0.0, "state_delay_over_budget")
    w.step(1)  # the next STATE back inside the budget
    assert w.ask()[0] > 0.0


def test_synthetic_scan_frames_never_open_a_real_bridge():
    w = World()
    w.step(25)
    w.feed.synthetic_frames = 1
    assert w.ask() == (0.0, "synthetic_scan")
    w.permit.allow_synthetic = True
    assert w.ask()[0] > 0.0


def test_parse_mask():
    assert bs.parse_mask("170:180,-180:-170") == [(170.0, 180.0), (-180.0, -170.0)]
    assert bs.parse_mask(None) == [] and bs.parse_mask("") == []
    with pytest.raises(ValueError):
        bs.parse_mask("10:5")


def test_a_scan_newer_than_the_newest_state_is_dropped_not_pinned():
    w = World()
    w.step(25)
    angles, ranges = cast((0.0, 0.0, 0.0), MOUNT, WALLS)
    future = w.host + 400 * MS
    w.feed.queue.append(scan(700, future, ranges, angles))  # its time is past every STATE so far
    w.permit.poll(w.host)
    assert w.permit.status()["dropped"].get("after_states", 0) == 1


def test_a_slow_scan_period_closes_and_a_normal_one_reopens():
    w = World()
    w.step(25)
    assert w.ask()[0] > 0.0
    angles, ranges = cast((0.0, 0.0, 0.0), MOUNT, WALLS)
    for _ in range(12):  # scans every 200 ms: past 1.5 x the nominal 100 ms
        w.step(10, scans=False)
        w.feed.queue.append(scan(w.k, w.host, ranges, angles))
        w.k += 1
        w.permit.poll(w.host)
    assert w.ask() == (0.0, "scan_period")
    w.step(60)
    assert w.ask()[0] > 0.0
