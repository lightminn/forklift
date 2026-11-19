"""The real truck's obstacle layer (docs/design/2026-11-19-real-safety-wiring.md 1-3, stage 5a).

The Isaac layer (``sim/isaac/obstacle_layer.py``) carries ray casting, synthetic noise and the sim clock; the real
truck does not import it. This layer puts the S10-A part of it on the shared ``ObstacleGrid`` and ``DrivePermission``:

- **scans** (``add_scan``) are placed on the raw odometry by the loop's ``placer`` (the teleop scan bridge's way: a
  continuous history over the whole scan, a motion bound checked, the timing error in the age table) or dropped;
- **refresh** is a request only: a worker computes the snapshot, the exemption and the path check on a copy; the
  control thread installs a result only when its request key -- correction generation, exemption serial, mission
  generation, path id (tracker serial and gear leg), loaded -- equals the current one;
- **limit** answers on the installed result: another key is ``snapshot_installing``, the newest scan too old is
  ``scan_stale``, the watchdog's stop comes next, else ``DrivePermission.allowed_speed``;
- **watchdog**: a received scan's deadline (last beam + B) passed without a result holding it or a newer one, the
  expected next scan (last beam + period + B) not arrived, or a result first used later than B after its newest
  scan -- ``layer_overdue``; held over ``overdue_end_s`` -- ``layer_latency_exceeded``; a dead worker at once;
- **planning grid**: the worker also builds the planning snapshot with the frozen pallet's returns suppressed
  (``ObstacleGrid.snapshot(suppress=)``); the driving snapshot keeps them (the D5 exemption covers the pocket).

Times are core seconds (the loop converts host ns before handing them in).
"""

from __future__ import annotations

import copy
import math
import threading
from dataclasses import dataclass, field, replace

import numpy as np

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.perception.obstacle_grid import (
    FREE,
    AgeErrorTable,
    GridConfig,
    ObstacleGrid,
    ObstacleScan,
    snapshot_to_occupancy,
)
from forklift_core.planning.geometry import Footprint

__all__ = ["LayerKey", "ManualWorker", "RealLayer", "RealScan", "ThreadWorker", "real_layer_configs",
           "exempt_region"]


@dataclass(frozen=True)
class LayerKey:
    correction_gen: int
    exempt_serial: int
    mission_gen: int | None
    path_id: tuple
    loaded: bool


@dataclass
class RealScan:
    """One LiDAR revolution: core-second times of its first and last beam, REP-117 ranges, a per-beam validity mask
    (a zero, NaN or bad-quality reading neither marks nor clears) and the truck's own returns."""

    sensor: str
    first_s: float
    last_s: float
    angles_rad: np.ndarray
    ranges_m: np.ndarray
    valid: np.ndarray
    self_hit: np.ndarray
    limit_m: np.ndarray | None = None


def exempt_region(snapshot, region):
    """Cells wholly inside the rectangle ``(cx, cy, length, width, yaw)`` become FREE at the snapshot time: the depth
    pocket check, not the grid, answers for them. The Isaac layer's rule (``sim/isaac/obstacle_layer.py``), copied
    expression for expression: the real truck does not import the simulation."""
    if region is None:
        return snapshot
    cx, cy, length, width, yaw = region
    res = snapshot.resolution_m
    nx, ny = snapshot.state.shape
    c, s = math.cos(yaw), math.sin(yaw)
    xs = snapshot.origin_x_m + np.arange(nx + 1) * res - cx
    ys = snapshot.origin_y_m + np.arange(ny + 1) * res - cy
    u = xs[:, None] * c + ys[None, :] * s
    v = -xs[:, None] * s + ys[None, :] * c
    corner_in = (np.abs(u) <= length / 2 + 1e-9) & (np.abs(v) <= width / 2 + 1e-9)
    inside = corner_in[:-1, :-1] & corner_in[1:, :-1] & corner_in[:-1, 1:] & corner_in[1:, 1:]
    if not inside.any():
        return snapshot
    state = snapshot.state.copy()
    stamp = snapshot.free_stamp.copy()
    state[inside] = FREE
    stamp[inside] = snapshot.stamp_s
    return replace(snapshot, state=state, free_stamp=stamp)


def real_layer_configs(layer: dict, *, error_table: AgeErrorTable, budget_s: float, scan_period_s: float,
                       state_delay_s: float, v_scan_mps: float, kappa_max_inv_m: float, hall) -> tuple:
    """(GridConfig, PermissionConfig) for the real layer (design 1.1-1.2): the age table takes the scan timing
    error (``v_scan * dt``, ``v_scan * kappa_max * dt``, ``dt = 2 period + state_delay``, the teleop scan bridge's
    formula), the grid's FREE use age is ``free_max_age_s + B`` and the permission takes that same value as its
    evidence age, and the stopping latency takes a revolution plus B (the first beam's obstacle waits one revolution
    and then the processing)."""
    g, p = layer["grid"], layer["permission"]
    dt = 2.0 * float(scan_period_s) + float(state_delay_s)
    d_e, d_psi = float(v_scan_mps) * dt, float(v_scan_mps) * float(kappa_max_inv_m) * dt
    table = AgeErrorTable(error_table.ages_s, tuple(v + d_e for v in error_table.position_m),
                          tuple(v + d_psi for v in error_table.yaw_rad))
    use_age = round(float(g["free_max_age_s"]) + float(budget_s), 9)
    grid = GridConfig(
        hall.x_min_m, hall.x_max_m, hall.y_min_m, hall.y_max_m, table,
        resolution_m=float(g["resolution_m"]), max_mark_m=float(g["max_mark_m"]),
        max_clear_m=float(g["max_clear_m"]), range_min_m=float(layer["range_min_m"]),
        free_max_age_s=float(g["free_max_age_s"]), occupied_max_age_s=float(g["occupied_max_age_s"]),
        free_use_age_s=use_age, sensor_bound_m=float(layer["noise_cut_m"]), free_r_cap_m=float(g["free_r_cap_m"]),
        free_rho_m=float(g["free_rho_m"]), close_gap_m=float(g.get("close_gap_m", 0.0)),
        free_min_width_m=float(g.get("free_min_width_m", 0.0)), taper_m=float(g.get("taper_m", 0.0)),
    )
    permission = PermissionConfig(
        StoppingModel(round(float(p["latency_s"]) + float(scan_period_s) + float(budget_s), 9),
                      float(p["decel_mps2"]), float(p["margin_m"])),
        envelope_offset_m=float(p["envelope_m"]), evidence_max_age_s=grid.use_age_s,
        envelope_ramp_m=float(p["envelope_ramp_m"]), sensor_timeout_s=float(p["sensor_timeout_s"]),
        lookahead_m=float(p["lookahead_m"]),
    )
    return grid, permission


class ManualWorker:
    """Tests: the computation runs when ``step()`` is called (asynchrony made explicit)."""

    def __init__(self) -> None:
        self.pending = None
        self.done: list = []
        self.alive = True

    def submit(self, job, fn) -> None:
        self.pending = (job, fn)  # the newest request only

    def step(self) -> None:
        if self.pending is not None:
            job, fn = self.pending
            self.pending = None
            self.done.append((job, fn(job)))

    def take(self) -> list:
        out, self.done = self.done, []
        return out


class ThreadWorker:
    """One daemon thread computing the newest request; results are taken by the control thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self.pending = None
        self.done: list = []
        self._thread = threading.Thread(target=self._run, name="real-layer", daemon=True)
        self._stop = False
        self._thread.start()

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def submit(self, job, fn) -> None:
        with self._wake:
            self.pending = (job, fn)
            self._wake.notify()

    def _run(self) -> None:
        while True:
            with self._wake:
                while self.pending is None and not self._stop:
                    self._wake.wait()
                if self._stop:
                    return
                job, fn = self.pending
                self.pending = None
            result = fn(job)  # an exception ends the thread: alive goes False, the watchdog stops the truck
            with self._lock:
                self.done.append((job, result))

    def take(self) -> list:
        with self._lock:
            out, self.done = self.done, []
        return out

    def close(self) -> None:
        with self._wake:
            self._stop = True
            self._wake.notify()


class RealLayer:
    known_enabled = False  # the known-pallet layer stays off on the real truck (4a refuses it on)
    memory = None

    def __init__(self, grid_config: GridConfig, permission_config: PermissionConfig, *, sensors: dict,
                 unloaded: Footprint, loaded: Footprint, body: Footprint, placer, budget_s: float,
                 overdue_end_s: float, scan_period_s: float, scan_max_age_s: float, worker=None,
                 valid_fraction_min: float = 0.5, odom_now=None, place_wait_s: float = 0.10) -> None:
        self.grid_config = grid_config
        self.grid = ObstacleGrid(grid_config)
        self.permission = DrivePermission(permission_config)
        self.sensors = sensors  # name -> (laser_in_rear (x, y, yaw), may_clear)
        self.unloaded, self.loaded, self.body = unloaded, loaded, body
        self.placer = placer  # (first_s, last_s) -> odometry rear pose (x, y, yaw) at the first beam, or None
        self.odom_now = odom_now  # () -> the raw odometry pose now (the install's distance driven since the request)
        self.budget_s, self.overdue_end_s = float(budget_s), float(overdue_end_s)
        self.scan_period_s, self.scan_max_age_s = float(scan_period_s), float(scan_max_age_s)
        self.valid_fraction_min = float(valid_fraction_min)
        self.worker = worker if worker is not None else ThreadWorker()
        self.snapshot = None
        self.depth_support = None  # stored only (no shadow memory on the real truck: the strict rule)
        self._exempt = None
        self.exempt_serial = 0
        self.context = (None, ())  # (mission generation, path id) -- the loop sets it each tick
        self.planning_pallet = None  # (pallet rectangle, key) the planning grid suppresses
        self.last_scan_s: dict = {}
        self.counts = {"scans": 0, "scans_dropped": 0, "stale_results": 0, "installs": 0}
        self._request = None  # the newest refresh request (dict)
        self._installed = None  # {"key", "newest_scan_s", "scan_id", "first_used_s", "planning"}
        self._scan_id = 0
        self._unplaced: list = []  # (scan id, scan, received at): waiting for a STATE measured after the last beam
        self.place_wait_s = float(place_wait_s)  # how long a scan may wait for that (then dropped)
        self._initial = ((0.0, 0.0, 0.0), 0)
        self._deadlines: dict = {}  # scan id -> (deadline_s, dropped)
        self._expected_s = None  # the next scan's expected deadline
        self._missed_expected = None  # a passed expected deadline: held until a result past it is installed
        self._first_use_late = False
        self._overdue_since = None
        self._watch_cache = None

    # --- the exemption -----------------------------------------------------------------------------------------
    @property
    def exempt(self):
        return self._exempt

    @exempt.setter
    def exempt(self, value) -> None:
        if value != self._exempt:
            self.exempt_serial += 1
            self._exempt = value  # the key changes: an installed result of the old exemption no longer answers

    # --- keys and requests -------------------------------------------------------------------------------------
    def _key(self, correction_gen: int, loaded: bool) -> LayerKey:
        mission_gen, path_id = self.context
        return LayerKey(int(correction_gen), self.exempt_serial, mission_gen, tuple(path_id), bool(loaded))

    def current_key(self) -> LayerKey | None:
        r = self._request
        return None if r is None else self._key(r["version"], r["loaded"])

    def refresh(self, now_s: float, correction, version: int, *, current_pose, path_ahead, loaded: bool,
                direction: int = 0):
        """A request only (design 1.2): recorded and handed to the worker; returns None (nothing to read back)."""
        self._request = {"now_s": float(now_s), "correction": tuple(float(v) for v in correction),
                         "version": int(version), "current_pose": tuple(float(v) for v in current_pose),
                         "path_ahead": None if path_ahead is None else np.asarray(path_ahead, dtype=float).copy(),
                         "loaded": bool(loaded), "direction": int(direction), "odom_pose": None}
        self._submit(now_s)
        return None

    def track(self, current_pose, path_ahead) -> None:
        """Each control tick (the loop): the pose and the path still ahead the next scan-driven computation checks
        from -- the Isaac runner refreshes with them at every scan; the key does not change."""
        if self._request is not None:
            self._request["current_pose"] = tuple(float(v) for v in current_pose)
            self._request["path_ahead"] = None if path_ahead is None else np.asarray(path_ahead, dtype=float).copy()

    def _submit(self, now_s: float) -> None:
        r = self._request
        if r is None:
            return
        job = dict(r)
        job["now_s"] = max(float(now_s), r["now_s"])
        job["key"] = self._key(r["version"], r["loaded"])
        job["exempt"] = copy.deepcopy(self._exempt)
        job["scan_id"] = getattr(self, "_placed_id", 0)  # the newest scan the copy holds
        job["newest_scan_s"] = max(self.last_scan_s.values(), default=None)
        job["grid"] = copy.deepcopy(self.grid)  # the worker owns its copy: the control thread never shares state
        job["planning_pallet"] = self.planning_pallet
        footprint, own = self.footprints(r["loaded"], r["direction"])
        job["footprint"], job["own"] = footprint, own
        job["permission_config"] = self.permission.config
        job["odom_pose"] = None if self.odom_now is None else tuple(float(v) for v in self.odom_now())
        self.worker.submit(job, _compute)

    def footprints(self, loaded: bool, direction: int = 0) -> tuple:
        return (self.loaded, self.loaded) if loaded else (self.unloaded, self.body)

    # --- scans -------------------------------------------------------------------------------------------------
    def add_scan(self, scan: RealScan, now_s: float, *, initial_correction=(0.0, 0.0, 0.0), initial_version: int = 0):
        """One scan received: checked, its deadlines registered (from its receipt), and queued for placement -- the
        raw odometry must reach past its last beam, so it is placed at a later tick head (``place_pending``) or
        dropped after ``place_wait_s``. Returns why it was dropped at once, or None."""
        self.counts["scans"] += 1
        self._scan_id += 1
        sid = self._scan_id
        self._deadlines[sid] = (float(scan.last_s) + self.budget_s, False)
        self._expected_s = float(scan.last_s) + self.scan_period_s + self.budget_s
        self._initial = (tuple(float(v) for v in initial_correction), int(initial_version))
        valid = np.asarray(scan.valid, dtype=bool)
        why = None
        if scan.sensor not in self.sensors:
            why = "unknown_sensor"
        elif valid.size == 0 or float(np.mean(valid)) < self.valid_fraction_min:
            why = "too_few_valid_beams"
        if why is not None:
            self._drop(sid)
            return why
        self._unplaced.append((sid, scan, float(now_s)))
        self.place_pending(now_s)
        return None

    def _drop(self, sid: int) -> None:
        self.counts["scans_dropped"] += 1
        self._deadlines[sid] = (self._deadlines[sid][0], True)

    def place_pending(self, now_s: float) -> None:
        """Place every waiting scan the odometry now reaches past (in order); drop one waiting too long."""
        keep = []
        placed = False
        for sid, scan, received in self._unplaced:
            pose = self.placer(float(scan.first_s), float(scan.last_s))
            if pose is None:
                if now_s - received > self.place_wait_s:
                    self._drop(sid)
                else:
                    keep.append((sid, scan, received))
                continue
            self._place(sid, scan, pose)
            placed = True
        self._unplaced = keep
        if placed:
            if self._request is None:
                correction, version = self._initial
                self._request = {"now_s": float(now_s), "correction": correction, "version": version,
                                 "current_pose": (0.0, 0.0, 0.0), "path_ahead": None, "loaded": False,
                                 "direction": 0, "odom_pose": None}
            self._submit(now_s)

    def _place(self, sid: int, scan: RealScan, pose) -> None:
        valid = np.asarray(scan.valid, dtype=bool)
        laser_in_rear, may_clear = self.sensors[scan.sensor]
        ranges = np.where(valid, np.asarray(scan.ranges_m, dtype=float), np.nan)
        self.grid.add_scan(ObstacleScan(
            float(scan.first_s), scan.sensor, tuple(float(v) for v in pose), tuple(float(v) for v in laser_in_rear),
            np.asarray(scan.angles_rad, dtype=float), ranges,
            np.asarray(scan.self_hit, dtype=bool) | ~valid, bool(may_clear),
            (self.body.front_m, self.body.rear_m, self.body.half_width_m),
            None if scan.limit_m is None else np.asarray(scan.limit_m, dtype=float)))
        self.last_scan_s[scan.sensor] = float(scan.first_s)  # FREE age from the first beam (conservative)
        self._placed_id = sid

    # --- install (control thread, tick head) ---------------------------------------------------------------------
    def install_ready(self, now_s: float, odom_pose=None) -> int:
        """Place the waiting scans, then install the finished results whose key is the current one; others are
        counted and dropped."""
        self.place_pending(now_s)
        installed = 0
        for job, result in self.worker.take():
            if job["key"] != self.current_key():
                self.counts["stale_results"] += 1
                continue
            self.snapshot = result["snapshot"]
            self.permission.update(self.snapshot, result["path_ahead"], job["footprint"], job["own"],
                                   current_pose=job["current_pose"], direction=job["direction"])
            start = job.get("odom_pose")
            if start is not None and odom_pose is not None:
                self.permission.advance(math.hypot(odom_pose[0] - start[0], odom_pose[1] - start[1]))
            # first used at this tick (installed at the tick head, read by this tick's control)
            self._installed = {"key": job["key"], "newest_scan_s": job["newest_scan_s"], "scan_id": job["scan_id"],
                               "first_used_s": float(now_s), "planning": result.get("planning")}
            self._first_use_late = job["newest_scan_s"] is not None and \
                now_s - job["newest_scan_s"] > self.scan_period_s + self.budget_s + 1e-6
            for sid in [s for s in self._deadlines if s <= job["scan_id"]]:
                del self._deadlines[sid]  # held by this result (a dropped one: by this newer result)
            if self._missed_expected is not None and job["scan_id"] > self._missed_expected[2]:
                self._missed_expected = None  # a result holding a scan received after the missed expectation
            self.counts["installs"] += 1
            installed += 1
        return installed

    def advance(self, distance_m: float) -> None:
        self.permission.advance(distance_m)

    # --- answers -----------------------------------------------------------------------------------------------
    def installed_key(self):
        return None if self._installed is None else self._installed["key"]

    def installing(self) -> bool:
        return self._installed is None or self._installed["key"] != self.current_key()

    def limit(self, now_s: float, *, current_pose, curvature_inv_m: float, direction: int, loaded: bool,
              cap_mps: float):
        if self.installing():
            return 0.0, "snapshot_installing"
        newest = self._installed["newest_scan_s"]
        if newest is None or now_s - newest > self.scan_max_age_s:
            return 0.0, "scan_stale"
        watch_ = self.watchdog(now_s)
        if watch_ is not None:
            return 0.0, watch_
        footprint, own = self.footprints(loaded, direction)
        return self.permission.allowed_speed(now_s, self.last_scan_s, current_pose=current_pose,
                                             curvature_inv_m=curvature_inv_m, direction=direction,
                                             footprint=footprint, own_footprint=own, speed_cap_mps=cap_mps)

    def watchdog(self, now_s: float) -> str | None:
        if self._watch_cache is not None and self._watch_cache[0] == now_s:
            return self._watch_cache[1]
        if not self.worker.alive:
            self._watch_cache = (now_s, "layer_worker_dead")
            return "layer_worker_dead"
        passed = any(d < now_s for d, _ in self._deadlines.values())
        if self._expected_s is not None and now_s > self._expected_s and self._missed_expected is None:
            # held until a result holding a later-received scan is installed (a late receipt alone does not release)
            self._missed_expected = (now_s, self._expected_s, self._scan_id)
        overdue = passed or self._missed_expected is not None or self._first_use_late
        reason = None
        if overdue:
            if self._overdue_since is None:
                self._overdue_since = float(now_s)
            reason = "layer_latency_exceeded" if now_s - self._overdue_since > self.overdue_end_s else "layer_overdue"
        else:
            self._overdue_since = None
        self._watch_cache = (now_s, reason)
        return reason

    def ok(self, now_s: float) -> bool:
        """Alive, scans arriving, at least one result installed (an invalidated one is a wait, not a fault)."""
        newest = max(self.last_scan_s.values(), default=None)
        return (self.worker.alive and self._installed is not None and newest is not None
                and now_s - newest <= self.scan_max_age_s + self.scan_period_s)

    def planning_grid(self, pallet_key):
        """The installed planning occupancy for ``pallet_key`` (the frozen pallet and the correction generation), or
        None when not ready (design 1.2: the hook refuses ``planning_grid_not_ready``)."""
        inst = self._installed
        if inst is None or inst["planning"] is None or inst["planning"][0] != pallet_key:
            return None
        return inst["planning"][1]


def _compute(job: dict) -> dict:
    """The worker's part: the driving snapshot (with the exemption) and the planning snapshot (the frozen pallet's
    returns suppressed) on the job's own grid copy."""
    grid, now_s = job["grid"], job["now_s"]
    raw = grid.snapshot(now_s, job["correction"], job["version"])
    snapshot = exempt_region(raw, job["exempt"]) if job["exempt"] is not None else raw
    planning = None
    pallet = job.get("planning_pallet")
    if pallet is not None:
        rect, pallet_key = pallet
        planning = (pallet_key, snapshot_to_occupancy(grid.snapshot(now_s, job["correction"], job["version"],
                                                                    suppress=[rect])))
    path = job["path_ahead"]
    if path is None:
        path = np.asarray([job["current_pose"]], dtype=float)
    return {"snapshot": snapshot, "path_ahead": path, "planning": planning}
