"""7b: the teleop bridge's floor permission from the real scan (docs/design/2026-11-16-bridge-scan-permit.md).

Real scans (``ScanFeed``, scan/1) and the MCU's wheel odometry go into the existing obstacle layer
(``sim/isaac/obstacle_layer.py``, no Isaac import) exactly as the Isaac live console's teleop does: each scan into the
grid, ``refresh()`` with the truck's pose as the path ahead, and ``BundlePermit.limit()`` on the layer's snapshot.
The timing uncertainty of a scan (its revolution, its start before the driver's stamp, the STATE's measurement time)
is added to the age error table's position and yaw columns, so the grid grows both OCCUPIED and the FREE band by it.
Valid only under the memo's operating conditions; every value in ``config/real/obstacle_layer_real.yaml`` is
provisional until the A2M12, S5 and S7 measure them.
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.perception.obstacle_grid import AgeErrorTable, ObstacleScan
from forklift_core.planning.geometry import Footprint
from forklift_core.plant.real_vehicle import RealOdometry
from forklift_core.safety.teleop_drive import BundlePermit
from forklift_core.sensors.lidar_cfg import load_lidar_config, self_hit_mask

REPO = Path(__file__).resolve().parents[1]
SENSOR = "lidar"
ENCODER_CUT = FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R
HISTORY_S = 2.0
MAX_GAP_S = 0.1  # an MCU-clock gap past this breaks the odometry (the bridge's STATEs come every 20 ms)


def _layer_module():
    mod = sys.modules.get("obstacle_layer")
    if mod is None:
        spec = importlib.util.spec_from_file_location("obstacle_layer", REPO / "sim" / "isaac" / "obstacle_layer.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules["obstacle_layer"] = mod
        spec.loader.exec_module(mod)
    return mod


def parse_mount(text: str) -> tuple[float, float, float, float]:
    """``x,y,z,yaw_deg`` of the LiDAR in the rear-axle frame (x forward, y left, z up from the floor) -> radians."""
    parts = [float(v) for v in str(text).split(",")]
    if len(parts) != 4 or not all(math.isfinite(v) for v in parts):
        raise ValueError("--lidar-mount takes four finite numbers: x,y,z,yaw_deg (rear-axle frame)")
    return parts[0], parts[1], parts[2], math.radians(parts[3])


def _interp_pose(hist, t: float):
    """The pose at ``t`` between the two history entries around it (yaw unwrapped); None outside the history (a
    scan newer than the newest STATE is never pinned to it)."""
    if t > hist[-1][0]:
        return None
    if t == hist[-1][0]:
        return hist[-1][1]
    items = list(hist)
    for (ta, pa), (tb, pb) in zip(items, items[1:], strict=False):
        if ta <= t <= tb:
            f = 0.0 if tb == ta else (t - ta) / (tb - ta)
            dyaw = math.atan2(math.sin(pb[2] - pa[2]), math.cos(pb[2] - pa[2]))
            return (pa[0] + f * (pb[0] - pa[0]), pa[1] + f * (pb[1] - pa[1]), pa[2] + f * dyaw)
    return None


class ScanPermit:
    """The bridge's ``floor_permit(curvatures, direction, check_speed) -> (limit, reason)`` on the real scan.

    ``on_state(sample, delay_s)`` with each STATE, ``poll(now_ns)`` once per cycle, then the call. ``mount``:
    (x, y, z, yaw_rad) in the rear-axle frame. ``v_max_mps``: the bridge's overspeed limit (the fastest the truck can
    go before the bridge stops it). ``mask_deg``: sensor-frame angle intervals (degrees) the truck's own body hides;
    their beams are self hits -- never marked or cleared, so what they cover stays unknown."""

    def __init__(self, feed, *, mount, config_path, vehicle, v_max_mps: float, mask_deg=None,
                 allow_synthetic: bool = False) -> None:
        import yaml

        self.mount = tuple(float(v) for v in mount)
        if len(self.mount) != 4 or not all(math.isfinite(v) for v in self.mount):
            raise ValueError("mount must be four finite numbers (x, y, z, yaw_rad)")
        if not (math.isfinite(v_max_mps) and v_max_mps > 0):
            raise ValueError("v_max_mps must be positive")
        self.feed, self.vehicle, self.v_max = feed, vehicle, float(v_max_mps)
        self.allow_synthetic = bool(allow_synthetic)
        self.delay_over = False  # the STATE delay bound passed the budget dt was computed with
        self.mask = [(math.radians(a), math.radians(b)) for a, b in (mask_deg or [])]
        lm = _layer_module()
        raw = yaml.safe_load(Path(config_path).read_text())
        b = raw["bridge"]
        self.bcfg = b
        cfg = lm.load_layer_config(config_path)
        table_path = Path(cfg["odometry_age"])
        table_path = table_path if table_path.is_absolute() else REPO / table_path
        t = json.loads(table_path.read_text())
        g = cfg["grid"]
        need = max(float(g.get("free_use_age_s") or g["free_max_age_s"]), float(g["occupied_max_age_s"]))
        if float(t["ages_s"][-1]) < need:
            raise ValueError(f"the odometry age table ends at {t['ages_s'][-1]} s, before the {need} s the grid uses")
        self.base_table = AgeErrorTable(tuple(t["ages_s"]), tuple(t["cumulative_position_m"]),
                                        tuple(t["cumulative_yaw_rad"]))
        # the timing uncertainty (memo): a revolution, the start before the stamp, the STATE measurement time
        period = float(b["scan_period_s"])
        dt = 2 * period + float(b["state_delay_s"])
        kappa = max(abs(k) for k in vehicle.steering.curvatures_inv_m)
        self.timing = {"dt_s": dt, "d_e_m": self.v_max * dt, "d_psi_rad": self.v_max * kappa * dt,
                       "v_max_mps": self.v_max, "kappa_max_inv_m": kappa}
        self.table = AgeErrorTable(self.base_table.ages_s,
                                   tuple(p + self.timing["d_e_m"] for p in self.base_table.position_m),
                                   tuple(y + self.timing["d_psi_rad"] for y in self.base_table.yaw_rad))
        fp = b["footprint"]
        self.footprint = Footprint(float(fp["front_m"]), float(fp["rear_m"]), float(fp["half_width_m"]))
        self.body_front_m = float(b["body_front_m"])
        h = float(b["hall_half_m"])
        self.hall = SimpleNamespace(x_min_m=-h, x_max_m=h, y_min_m=-h, y_max_m=h)
        self.cfg = cfg
        self.period_nominal = period
        self.max_lag_ns = int(float(b["max_stamp_to_recv_s"]) * 1e9)
        # the height band's one source is config/real/lidar.yaml, read as the observe run's conversion reads it
        # (PR 7 design 4, Codex 7a-1 P3)
        self.limit_m = load_lidar_config().limit_m
        self.odo = RealOdometry(vehicle, max_gap_s=MAX_GAP_S)
        self.pose = (0.0, 0.0, 0.0)
        self.hist: deque = deque()
        self.stamps: deque = deque(maxlen=11)
        self.now_ns = 0
        self.scans_used = 0
        self.dropped: dict = {}
        self.breaks = 0
        self.driven_m = 0.0
        self.generation = 0
        self._new_layer()

    # ------------------------------------------------------------------ parts
    def _new_layer(self) -> None:
        lm = _layer_module()
        self.layer = lm.ObstacleLayer(self.cfg, hall=self.hall, error_table=self.table, unloaded=self.footprint,
                                      loaded=self.footprint, body_front_m=self.body_front_m,
                                      rear_axle_x_in_base_m=0.0, noise_seed=0, blades_rear_m=())
        self.bundle = BundlePermit(self.layer.permission.config)
        self.generation += 1

    def _drop(self, why: str) -> None:
        self.dropped[why] = self.dropped.get(why, 0) + 1

    def period_s(self) -> float | None:
        if len(self.stamps) < 3:
            return None
        return float(np.median(np.diff(np.asarray(self.stamps, dtype=np.int64)))) / 1e9

    # ------------------------------------------------------------------ inputs
    def on_state(self, sample, delay_s: float) -> None:
        """One STATE: the odometry, its pose at the STATE's measurement time (receipt minus the delay bound), the
        distance driven for the bundle; a continuity break starts a new layer and permission (memo 3')."""
        s = sample.state
        step = self.odo.update(s.mcu_time_us, s.boot_id, s.wheel_count_left, s.wheel_count_right, s.steering_rad,
                               encoder_cut=bool(int(s.fault_flags) & int(ENCODER_CUT)))
        if step.reason == "time_not_increasing":
            return
        # the STATE's measurement time is known within the delay bound only while it stays inside the budget the
        # timing uncertainty dt assumed (or there is no bound at all): otherwise the permit closes until it is back
        self.delay_over = delay_s is None or float(delay_s) > float(self.bcfg["state_delay_s"])
        if delay_s is None:
            delay_s = float(self.bcfg["state_delay_s"])
        if step.continuity_broken:
            self.breaks += 1
            self.hist.clear()
            self._new_layer()
        t = (int(sample.host_ns) - int(float(delay_s) * 1e9)) / 1e9
        new = tuple(float(v) for v in step.pose)
        if self.hist and not step.continuity_broken:
            d = math.hypot(new[0] - self.pose[0], new[1] - self.pose[1])
            if d > 0.0:
                self.bundle.advance(d)
                self.driven_m += d
        self.pose = new
        self.hist.append((t, new))
        while self.hist and self.hist[0][0] < t - HISTORY_S:
            self.hist.popleft()

    def poll(self, now_ns: int) -> None:
        """The scans received since the last cycle, each into the grid at its pose, then a new snapshot."""
        self.now_ns = int(now_ns)
        added = False
        for scan in self.feed.poll():
            if scan.meta.get("stamp_kind") not in (None, "call_start"):
                # first, before anything reads it into the stamp history: the bridge places a scan at (stamp -
                # period), a call-start stamp only -- a first-sample stamp would put it a rotation early and shift
                # the period the permit checks (Codex 7a-2 rounds 1-2); run the adapter with --source raw
                self._drop("stamp_kind")
                continue
            if scan.stamp_to_recv_ns > self.max_lag_ns:
                self._drop("late")
                continue
            self.stamps.append(int(scan.stamp_mono_ns))
            period = self.period_s() or self.period_nominal
            period = min(max(period, 0.05), 0.2)
            t_scan = (int(scan.stamp_mono_ns) - int(period * 1e9)) / 1e9
            if not self.hist or t_scan < self.hist[0][0]:
                self._drop("before_states")
                continue
            pose = _interp_pose(self.hist, t_scan)
            if pose is None:
                self._drop("after_states")
                continue
            angles = np.asarray(scan.angles, dtype=float)
            ranges = np.asarray(scan.ranges, dtype=float)
            if angles.shape != ranges.shape or angles.ndim != 1 or not np.isfinite(angles).all():
                self._drop("bad_scan")
                continue
            self_hit = self_hit_mask(angles, self.mask)
            own = self.layer.footprints(False)[1]
            mx, my, _mz, myaw = self.mount
            self.layer.grid.add_scan(ObstacleScan(
                t_scan, SENSOR, pose, (mx, my, myaw), angles, ranges, self_hit, True,
                (own.front_m, own.rear_m, own.half_width_m), np.full(len(angles), self.limit_m)))
            self.layer.last_scan_s[SENSOR] = t_scan
            self.scans_used += 1
            added = True
        if added:
            now_s = self.now_ns / 1e9
            self.layer.refresh(now_s, (0.0, 0.0, 0.0), 0, current_pose=self.pose,
                               path_ahead=np.array([self.pose]), loaded=False, direction=0)

    # ------------------------------------------------------------------ the permit
    def __call__(self, curvatures, direction, check_speed):
        if getattr(self.feed, "synthetic_frames", 0) and not self.allow_synthetic:
            return 0.0, "synthetic_scan"  # a rehearsal sender's frames never open a real bridge (arrival-day D6)
        if self.delay_over:
            return 0.0, "state_delay_over_budget"
        snap = self.layer.snapshot
        if snap is None:
            return 0.0, "no_snapshot"
        period = self.period_s()
        if period is not None and period > 1.5 * self.period_nominal:
            return 0.0, "scan_period"
        key = (id(snap), float(snap.stamp_s), self.generation)
        fp, own = self.layer.footprints(False, direction)
        return self.bundle.limit(snap, key, self.now_ns / 1e9, self.layer.last_scan_s, pose=self.pose,
                                 curvatures=list(curvatures), direction=direction, check_speed_mps=check_speed,
                                 footprint=fp, own_footprint=own)

    def status(self) -> dict:
        return {"scans_used": self.scans_used, "dropped": dict(self.dropped), "breaks": self.breaks,
                "last_scan_s": self.layer.last_scan_s.get(SENSOR), "driven_m": self.driven_m,
                "period_s": self.period_s(), "generation": self.generation, "delay_over": self.delay_over}

    def record_view(self) -> dict:
        """What each cycle record carries (small): scans used, the newest scan time and the drops so far."""
        return {"used": self.scans_used, "last_s": self.layer.last_scan_s.get(SENSOR), "breaks": self.breaks,
                "dropped": sum(self.dropped.values())}


def parse_mask(text: str | None) -> list:
    """``a:b,c:d`` (degrees, sensor frame) -> [(a, b), (c, d)]; empty or None -> []."""
    if not text:
        return []
    out = []
    for part in text.split(","):
        a, b = (float(v) for v in part.split(":"))
        if not (math.isfinite(a) and math.isfinite(b) and -180.0 <= a <= b <= 180.0):
            raise ValueError("--lidar-mask takes a:b,c:d with -180 <= a <= b <= 180 (degrees)")
        out.append((a, b))
    return out
