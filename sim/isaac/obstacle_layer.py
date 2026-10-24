"""The priority-5 obstacle layer between the Isaac runner and forklift_core (no Isaac imports).

docs/plans/2026-10-04-lidar-obstacle-map.md. The runner casts the obstacle
LiDARs at every scan tick and hands the raw rays here; this module turns them
into REP-117 scans with the synthetic noise contract (sigma 0.02 m cut at
+-0.06 m, P0a), keeps the rolling grid, refreshes the drive permission with
a local snapshot around the truck, answers the per-tick speed limit, and
builds the hall-wide occupancy grid the planner replans with. Everything is
in the frame control uses: the applied SLAM correction o wheel odometry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.control.shadow_memory import ShadowMemory
from forklift_core.perception.obstacle_grid import (
    A2_SPEC_MEASURED_FACTOR,
    FREE,
    AgeErrorTable,
    GridConfig,
    ObstacleGrid,
    ObstacleScan,
    a2_spec_cut_m,
    a2_spec_sigma_m,
    snapshot_to_occupancy,
)
from forklift_core.planning.geometry import Footprint


def _arc_slice(poses, start_m: float, end_m: float):
    """The stretch of a pose array between two arc lengths, its end points interpolated (leg-failures plan A1)."""
    poses = np.asarray(poses, dtype=float)
    seg = np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))

    def at(s_):
        i = int(np.clip(np.searchsorted(cum, s_, side="right") - 1, 0, len(poses) - 2))
        f = 0.0 if seg[i] <= 0 else (s_ - cum[i]) / seg[i]
        a_, b_ = poses[i], poses[i + 1]
        dyaw = float(np.arctan2(np.sin(b_[2] - a_[2]), np.cos(b_[2] - a_[2])))
        return np.array([a_[0] + f * (b_[0] - a_[0]), a_[1] + f * (b_[1] - a_[1]), a_[2] + f * dyaw])

    inner = poses[(cum > start_m) & (cum < end_m)]
    return np.vstack([at(start_m)] + ([inner] if len(inner) else []) + [at(end_m)])


@dataclass(frozen=True)
class ObstacleSensor:
    name: str
    xyz_m: tuple[float, float, float]  # base_link
    yaw_rad: float
    may_clear: bool


def load_layer_config(path: Path) -> dict:
    data = yaml.safe_load(Path(path).read_text())
    clear_max = float(data["clear_max_height_m"])
    sensors = [
        ObstacleSensor(
            s["name"], tuple(float(v) for v in s["xyz"]), math.radians(float(s["yaw_deg"])),
            float(s["xyz"][2]) <= clear_max,
        )
        for s in data["sensors"]
    ]
    return {**data, "sensors": sensors}


def beam_limits(origin, directions, *, band_top_m: float, floor_margin_m: float = 0.05,
                band_bottom_m: float | None = None) -> np.ndarray:
    """Per beam, the range up to which a tilted planar beam stays between the
    floor and band_top_m (h_det): rising beams leave the band, sinking ones hit
    the floor (Codex L0b P1). With band_bottom_m (h_lo, plan v10 D0) a sinking
    beam stops speaking where it drops below h_lo instead of at the floor: the
    single-plane operating assumption covers only [h_lo, h_det], and below it a
    beam could pass under a cover between supports (Codex v10 2nd P1-3)."""
    z0 = float(origin[2])
    dz = np.asarray(directions, dtype=float)[:, 2]
    out = np.full(len(dz), np.inf)
    up = dz > 1e-9
    down = dz < -1e-9
    out[up] = (band_top_m - z0) / dz[up]
    floor = floor_margin_m if band_bottom_m is None else max(float(band_bottom_m), floor_margin_m)
    out[down] = np.maximum(z0 - floor, 0.0) / -dz[down]
    return np.maximum(out, 0.0)


def exempt_region(snapshot, region):
    """Cells wholly inside the rectangle become FREE at the snapshot time: the
    depth pocket check, not the grid, answers for them (it must gate every
    control tick while this is set)."""
    from forklift_core.perception.obstacle_grid import FREE

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


class ObstacleLayer:
    def __init__(self, config: dict, *, hall, error_table: AgeErrorTable, unloaded: Footprint,
                 loaded: Footprint, body_front_m: float, rear_axle_x_in_base_m: float, noise_seed: int,
                 blades_rear_m: tuple = ()):
        self.config = config
        self.sensors: list[ObstacleSensor] = config["sensors"]
        self.beam_angles = np.linspace(-math.pi, math.pi, int(config["beams"]), endpoint=False)
        self.range_min_m = float(config["range_min_m"])
        self.range_max_m = float(config["range_max_m"])
        self.noise_std_m = float(config["noise_std_m"])
        self.noise_cut_m = float(config["noise_cut_m"])
        # P3a (docs/plans/2026-10-23-planner-permission-clearance.md): range_noise.model "a2_spec" draws each beam's
        # noise with sigma = acc / 2 cut at max(0.02, 1.5 acc) of its true range (Slamtec's A2 spec rows, read as
        # +-2 sigma -- an assumption) and gives the grid the same model; absent: the constant sigma / cut above.
        self.range_noise_model = str((config.get("range_noise") or {}).get("model", "const"))
        if self.range_noise_model not in ("const", "a2_spec"):
            raise ValueError(f"unknown range_noise model {self.range_noise_model!r}")
        if self.range_noise_model == "a2_spec":
            # the widest bound the grid can use (a hit at max_mark_m): what a caller that needs one bound takes
            self.noise_cut_m = float(a2_spec_cut_m(np.array(A2_SPEC_MEASURED_FACTOR * float(config["grid"]["max_mark_m"]))))
        self.band_top_m = float(config["band_top_m"])  # h_det: a beam above it no longer clears
        # Plan v10: the brief's one LiDAR feeds SLAM and this layer from one raw scan.
        self.shared_with_slam = bool(config.get("shared_with_slam", False))
        # h_lo (plan v10 D0): a sinking beam below it no longer clears; absent = the floor margin.
        self.band_bottom_m = float(config["band_bottom_m"]) if config.get("band_bottom_m") is not None else None
        g = config["grid"]
        self.grid_config = GridConfig(
            hall.x_min_m, hall.x_max_m, hall.y_min_m, hall.y_max_m, error_table,
            resolution_m=float(g["resolution_m"]), max_mark_m=float(g["max_mark_m"]),
            max_clear_m=float(g["max_clear_m"]), range_min_m=self.range_min_m,
            free_max_age_s=float(g["free_max_age_s"]), occupied_max_age_s=float(g["occupied_max_age_s"]),
            sensor_bound_m=self.noise_cut_m, free_r_cap_m=float(g["free_r_cap_m"]),
            free_rho_m=float(g["free_rho_m"]), close_gap_m=float(g.get("close_gap_m", 0.0)),
            free_min_width_m=float(g.get("free_min_width_m", 0.0)),
            taper_m=float(g.get("taper_m", 0.0)),
            range_bound_model=self.range_noise_model,
        )
        self.grid = ObstacleGrid(self.grid_config)
        self.error_table = error_table
        # Known pallets (plan v10 D5): the 1.05 m plane never sees an EPAL 6, so the
        # pickup zone, the recognised pallet and the delivered pallet reach the
        # permission as rectangles (forklift_core.perception.known_obstacles).
        kp = config.get("known_pallets") or {}
        self.known_enabled = bool(kp.get("enabled", False))
        self.known_config = kp
        self.known: list = []
        self.corridor = None  # (path, shape, until_m) while a withdrawal is certified
        p = config["permission"]
        self.permission = DrivePermission(
            PermissionConfig(
                StoppingModel(float(p["latency_s"]), float(p["decel_mps2"]), float(p["margin_m"])),
                envelope_offset_m=float(p["envelope_m"]),
                evidence_max_age_s=float(g["free_max_age_s"]),
                envelope_ramp_m=float(p["envelope_ramp_m"]),
                sensor_timeout_s=float(p["sensor_timeout_s"]),
                lookahead_m=float(p["lookahead_m"]),
                shadow_band_m=float(p.get("shadow_band_m", 0.0)),
            )
        )
        self.window_m = float(g["window_m"])
        # Shadow-band memory (D4 delta approved 2026-10-05); absent = strict rule.
        band = p.get("shadow_band_m")
        self.shadow = None if band is None else ShadowMemory(float(band), error_table, evidence_max_age_s=float(g["free_max_age_s"]))
        self.unloaded, self.loaded = unloaded, loaded
        self.body = Footprint(body_front_m, unloaded.rear_m, unloaded.half_width_m)
        # Unloaded, the permission checks the body and the fork blades (rear-axle
        # frame (x0, x1, y0, y1)), not their hull: an object between the blades
        # is met only by the body's front face, which the body part checks (D4).
        # The planner checks the hull, and driving forward the permission checks
        # it too, so the truck never enters a pose the planner cannot start
        # from (L3b v23: an object beside a blade, inside the hull, made every
        # replan invalid_start). Reversing, the forks leave the hull's front:
        # the body and blades are checked instead (L3c v19: after the withdraw
        # the hull's fork end lay in the delivered pallet's grid swelling and
        # no reverse could start). permission.shape: hull_forward (default),
        # hull or parts.
        mode = str(p.get("shape", "hull_forward"))
        self.parts_shape = None if not blades_rear_m else (
            [(self.body, 0.0, 0.0)] + [
                (Footprint((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2), (y0 + y1) / 2, (x0 + x1) / 2)
                for x0, x1, y0, y1 in blades_rear_m
            ]
        )
        self.shape_mode = mode if self.parts_shape is not None else "hull"
        self.unloaded_shape = self.parts_shape if self.shape_mode == "parts" else unloaded
        self.rear_x = rear_axle_x_in_base_m
        self.rng = np.random.default_rng([noise_seed, 7])
        self.last_scan_s: dict = {}
        self.snapshot = None
        # Docking exemption (plan D5): (x, y, length, width, yaw) in the control
        # frame; set only while the depth pocket check gates every tick.
        self.exempt = None
        self.depth_support = None  # callable(snapshot, now) -> {cell: certification time}, set with exempt
        # Planning-only memory and SLAM static layer (D2/D3 delta 2026-10-06):
        # never seen by the permission.
        self.memory = None
        if g.get("planning_memory", False):
            from forklift_core.perception.planning_memory import PlanningMemory

            hit_radius_ = float(g.get("memory_hit_radius_m", 0.1))
            if self.range_noise_model == "a2_spec":
                # an endpoint marks a fixed disk of cells: at least the widest range bound it can need (Codex P3a
                # review P2 -- 0.1 m left a hit read 0.135 m short at 4.5 m outside the memory once its scan expired),
                # plus the quantization (round 2 P2): the endpoint's cell and the memory cell under the planning grid
                # cell holding the truth sit < bound/res + 1.5 cells apart on each axis, so 1.5 sqrt(2) res more
                hit_radius_ = max(hit_radius_, self.noise_cut_m + 1.5 * math.sqrt(2.0) * float(g["resolution_m"]))
            self.memory = PlanningMemory(hall.x_min_m, hall.y_min_m, hall.x_max_m, hall.y_max_m,
                                         float(g["resolution_m"]), hit_radius_m=hit_radius_)

    def _memory_hits(self, now_s: float, correction) -> None:
        """Every raw scan not yet taken, once, with its own stamp (never a
        snapshot's re-raised mark -- Codex design review P1)."""
        mem = self.memory
        cfg = self.grid.config
        for scan in self.grid.scans:
            if scan.stamp_s <= mem.last_scan_stamp or scan.stamp_s > now_s + 1e-9:
                continue
            _, laser, ranges, cos_a, sin_a, usable, limit = self.grid._geometry(scan, correction)
            hit = (usable & np.isfinite(ranges) & (ranges >= cfg.range_min_m) & (ranges <= cfg.max_mark_m)
                   & (ranges <= limit))
            mem.add_hits(laser[0] + ranges[hit] * cos_a[hit], laser[1] + ranges[hit] * sin_a[hit], scan.stamp_s)
        newest = max((sc.stamp_s for sc in self.grid.scans if sc.stamp_s <= now_s + 1e-9), default=None)
        if newest is not None:
            mem.last_scan_stamp = max(mem.last_scan_stamp, newest)

    def _memory_clears(self, snapshot, now_s: float) -> None:
        """Fresh FREE cells of the raw snapshot (before the shadow memory and the
        exemption), with their own observation times."""
        fresh = (snapshot.state == FREE) & (now_s - snapshot.free_stamp <= self.grid_config.free_max_age_s + 1e-9)
        ii, jj = np.nonzero(fresh)
        if len(ii):
            res = snapshot.resolution_m
            self.memory.add_clears(snapshot.origin_x_m + (ii + 0.5) * res, snapshot.origin_y_m + (jj + 0.5) * res,
                                   snapshot.free_stamp[ii, jj])

    def footprints(self, loaded: bool, direction: int = 0) -> tuple[Footprint, Footprint]:
        """(footprint checked, own outline excluded): a carried pallet is part of the truck."""
        if loaded:
            return self.loaded, self.loaded
        if self.shape_mode == "hull_forward" and direction < 0:
            # Reversing, the present body and blades are the truck: a cell the
            # blade tips cover now is entered only where the reverse sweep
            # reaches its part outside them (L3c v21: the delivered pallet's
            # swelling at the tips held every return start).
            return self.parts_shape, self.parts_shape
        return self.unloaded_shape, self.body

    def _sigma_cut(self, true_m: np.ndarray):
        """Per beam (sigma, cut) at its true range: constant, or P3a's A2 spec model."""
        if self.range_noise_model == "const":
            return self.noise_std_m, self.noise_cut_m
        d = np.where(np.isfinite(true_m), true_m, 0.0)
        return a2_spec_sigma_m(d), a2_spec_cut_m(d)

    def _draw(self, true_m: np.ndarray) -> np.ndarray:
        # one normal draw per beam either way, so a run's random stream stays the same length
        sigma, cut = self._sigma_cut(true_m)
        return np.clip(self.rng.normal(0, sigma, len(true_m)), -cut, cut)

    def ranges(self, distances, hits) -> np.ndarray:
        """REP-117 ranges with the truncated noise contract."""
        out = np.where(hits, np.asarray(distances, dtype=float), np.inf)
        finite = np.isfinite(out)
        noise = self._draw(out[finite])
        out[finite] = out[finite] + noise
        out[finite & (out > self.range_max_m)] = np.inf
        out[finite & (out < self.range_min_m)] = -np.inf
        return out

    def beam_noise(self, count: int, distances=None) -> np.ndarray:
        """One truncated noise draw per beam, for a scan shared with SLAM (plan v10, one raw scan). ``distances``: the
        beams' true ranges (inf for no return), which the A2 spec model needs; the constant model ignores them."""
        if self.range_noise_model == "const":
            return np.clip(self.rng.normal(0, self.noise_std_m, int(count)), -self.noise_cut_m, self.noise_cut_m)
        if distances is None or len(distances) != count:
            raise ValueError("the A2 spec range noise needs every beam's true range")
        return self._draw(np.asarray(distances, dtype=float))

    def ranges_from(self, distances, hits, noise) -> np.ndarray:
        """REP-117 ranges from a given per-beam noise draw (the shared scan's)."""
        out = np.where(hits, np.asarray(distances, dtype=float), np.inf)
        finite = np.isfinite(out)
        out[finite] = out[finite] + np.asarray(noise, dtype=float)[finite]
        out[finite & (out > self.range_max_m)] = np.inf
        out[finite & (out < self.range_min_m)] = -np.inf
        return out

    def add_scans(self, stamp_s: float, raw: dict, *, odom_rear, loaded: bool) -> None:
        """raw: sensor name -> (distances, hits, own) from planar_lidar.cast_scan_flags."""
        _, own_outline = self.footprints(loaded)
        for sensor in self.sensors:
            if sensor.name not in raw:
                continue  # a silent sensor (L4 N9/N13): its last stamp ages
            distances, hits, own, *rest = raw[sensor.name]
            limit = rest[0] if rest else None
            given = rest[1] if len(rest) > 1 else None  # ranges already drawn for the shared scan
            self.grid.add_scan(
                ObstacleScan(
                    float(stamp_s), sensor.name, tuple(float(v) for v in odom_rear),
                    (sensor.xyz_m[0] - self.rear_x, sensor.xyz_m[1], sensor.yaw_rad),
                    self.beam_angles, self.ranges(distances, hits) if given is None else np.asarray(given, dtype=float),
                    np.asarray(own, dtype=bool),
                    sensor.may_clear, (own_outline.front_m, own_outline.rear_m, own_outline.half_width_m),
                    None if limit is None else np.asarray(limit, dtype=float),
                )
            )
            self.last_scan_s[sensor.name] = float(stamp_s)

    def refresh(self, now_s: float, correction, version: int, *, current_pose, path_ahead, loaded: bool,
                direction: int = 0):
        """New local snapshot and path check around the truck."""
        x, y = current_pose[0], current_pose[1]
        w, res = self.window_m, self.grid_config.resolution_m
        x0 = math.floor((x - w) / res) * res
        y0 = math.floor((y - w) / res) * res
        if self.memory is not None:
            self._memory_hits(now_s, correction)
        self.grid.config = replace(self.grid_config, x_min_m=x0, x_max_m=x0 + 2 * w, y_min_m=y0, y_max_m=y0 + 2 * w)
        try:
            self.snapshot = self.grid.snapshot(now_s, correction, version)
        finally:
            self.grid.config = self.grid_config
        if self.memory is not None:
            self._memory_clears(self.snapshot, now_s)
        footprint, own = self.footprints(loaded, direction)
        # The shadow-band memory may lean on cells the depth pocket check saw
        # free (observation, with its time), never on the exemption itself
        # (Codex checkpoint P1: exempt cells as fresh FREE spread RETAINED past
        # the region). The face cells straddling the region's edge need that
        # evidence (L3c v8, insertion start).
        if self.shadow is not None:
            support = self.depth_support(self.snapshot, now_s) if self.depth_support is not None else None
            self.snapshot = self.shadow.apply(self.snapshot, current_pose, own if isinstance(own, Footprint) else self.body,
                                              extra_support=support)
        if self.known:
            from forklift_core.perception.known_obstacles import apply_known, corridor_mask

            mask = None
            if self.corridor is not None:
                c_path, c_shape, c_until = self.corridor
                mask = corridor_mask(self.snapshot, c_path, c_shape, self.permission.config, direction=-1,
                                     until_m=c_until, tracking_m=float(self.known_config.get("tracking_m", 0.006)))
            self.snapshot = apply_known(
                self.snapshot, self.known, now_s, exclude=mask, table=self.error_table, current_pose=current_pose,
                cap_m=float(self.grid_config.free_r_cap_m),
                horizon_s=float(self.permission.config.evidence_max_age_s),
                empirical_m=self.known_config.get("empirical_m"),
                empirical_max_age_s=self.known_config.get("empirical_max_age_s"),
            )
        if self.exempt is not None:
            self.snapshot = exempt_region(self.snapshot, self.exempt)
        return self.permission.update(self.snapshot, path_ahead, footprint, own, current_pose=current_pose,
                                      direction=direction)

    def check_candidate(self, path_ahead, *, current_pose, direction: int, loaded: bool, extra_margin_m: float = 0.0,
                        extra_ramp_m: float | None = None, unknown_passes: bool = False,
                        lookahead_m: float | None = None):
        """Obstacle recovery v4 D1: a candidate path checked on the snapshot the permission uses
        (shadow memory, known pallets and exemptions applied), leaving the permission as it is.
        lookahead_m: DrivePermission.check_path's walk length for this check (None: the config's)."""
        footprint, own = self.footprints(loaded, direction)
        return self.permission.check_path(self.snapshot, path_ahead, footprint, own, current_pose=current_pose,
                                          direction=direction, extra_margin_m=extra_margin_m,
                                          extra_ramp_m=extra_ramp_m, unknown_passes=unknown_passes,
                                          lookahead_m=lookahead_m)

    def check_turnaround_arc(self, poses, *, current_pose) -> str | None:
        """Return turnaround (docs/plans/2026-10-12-return-forward-turnaround.md): None when the unloaded
        reverse arc ``poses`` (rear-axle samples from current_pose) meets the spatial conditions for a
        turnaround candidate to be compared, else why not. It does not decide the start itself: evidence
        age, a silent sensor and the stop arc at the present steering stay allowed_speed's.
        What a start needs -- the permission's own walk ahead -- must be verified as for any path, and no
        cell known occupied may lie anywhere on the arc (a walk past unknown cells). The rest is verified
        by the permission on the way, as for every path: the grid clears cells only within max_clear_m
        of the sensor and its last rho band (4 m from the rear axle by default), so an arc whose body
        sweeps farther cannot be verified whole from its start (the
        cusp position-stall prefix check of plan design v7 has the same two parts)."""
        from forklift_core.control.turnaround_check import turnaround_arc_reason

        footprint, own = self.footprints(False, -1)
        return turnaround_arc_reason(self.permission, self.snapshot, footprint, own, poses,
                                     current_pose=current_pose)  # one function with the frozen checker (plan D11)

    def turnaround_arc_check(self, callback_spec: dict | None = None):
        """Persistent console plan D11: the arc check frozen now (permission and snapshot copied) for the planning
        worker -- the same verdicts the layer gives at this instant."""
        from forklift_core.control.turnaround_check import TurnaroundArcCheck

        footprint, own = self.footprints(False, -1)
        return TurnaroundArcCheck(self.permission, self.snapshot, footprint, own,
                                  callback_spec or {"kind": "turnaround_arc"})

    def candidate_permit(self, now_s: float, path_ahead, *, current_pose, direction: int, loaded: bool,
                         curvature_inv_m: float, measured_curvature_inv_m: float, cap_mps: float,
                         strict: bool = True, strict_from_m: float = 0.0) -> dict:
        """The scene-mode replan's check before a candidate is installed (docs/plans/2026-10-23-scene-replan.md
        2.1 step 6, 7판): ``path_ahead`` -- the candidate tracker's first gear leg -- on the processed snapshot, in a
        fresh DrivePermission so the layer's own (its path check, the distance driven since, its last stop arc) is
        neither read nor changed. Always: no occupied cell within the lookahead, walked past unknown cells (an
        occupied one behind an unknown is still found), and the start allowed above zero both at the first command's
        curvature and at the wheels' measured one -- the driving permission's own start rule. strict (the leg reaches
        where the permission does not act, Codex Isaac round 2 P2): also verified min(lookahead, the leg's length)
        with nothing blocked; otherwise an unknown beyond the verified distance is the driving permission's, as on
        any path. ``strict_from_m`` (leg-failures plan A1): where along the leg the zone the permission does not act on
        begins -- strict then verifies [strict_from_m, min(lookahead, the leg's length)] only (empty when reversed),
        the unknown before it being the driving permission's; 0 is the whole stretch from the truck, as before."""
        poses = np.asarray(path_ahead, dtype=float)
        out = {"ok": False, "verified_m": 0.0, "blocked": None, "allowed_mps": 0.0, "measured_allowed_mps": 0.0,
               "why": None}
        if self.snapshot is None:
            out["why"] = "no_snapshot"
            return out
        footprint, own = self.footprints(loaded, direction)
        candidate = DrivePermission(self.permission.config)
        if self.permission.release_anchor is not None:
            # the start-band release's anchor (docs/plans/2026-10-24-start-band-release.md): the same truck standing at
            # the same place; a copy, so the candidate's checks never move the driving permission's best distances
            import copy as _copy

            candidate.release_anchor = _copy.deepcopy(self.permission.release_anchor)
        check = candidate.update(self.snapshot, poses, footprint, own, current_pose=current_pose, direction=direction)
        length = float(np.sum(np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1])))) if len(poses) > 1 else 0.0
        need = min(float(self.permission.config.lookahead_m), length)
        zone_from = max(0.0, float(strict_from_m))
        zone_len = max(0.0, need - zone_from) if strict else 0.0
        allowed = [candidate.allowed_speed(now_s, self.last_scan_s, current_pose=current_pose, curvature_inv_m=k,
                                           direction=direction, footprint=footprint, own_footprint=own,
                                           speed_cap_mps=cap_mps)
                   for k in (curvature_inv_m, measured_curvature_inv_m)]
        ahead = candidate.check_path(self.snapshot, poses, footprint, own, current_pose=current_pose,
                                     direction=direction, unknown_passes=True)
        if zone_len > 0.0 and zone_from > 0.0:
            # the zone's part of the leg alone, its own walk over just that stretch (no lookahead re-applied past it)
            zone_check = candidate.check_path(self.snapshot, _arc_slice(poses, zone_from, need), footprint, own,
                                              current_pose=current_pose, direction=direction, lookahead_m=zone_len)
            zone_verified = float(zone_check.verified_m)
        else:
            zone_check, zone_verified = check, float(check.verified_m)  # from the truck: the walk above, as before
        out.update(verified_m=float(check.verified_m), blocked=check.blocked, allowed_mps=float(allowed[0][0]),
                   measured_allowed_mps=float(allowed[1][0]), allowed_why=allowed[0][1],
                   measured_allowed_why=allowed[1][1], need_m=need, strict=bool(strict),
                   strict_from_m=zone_from, strict_checked_m=zone_len, occupied_ahead=ahead.blocked == "occupied")
        need_zone = zone_len if zone_from > 0.0 else need
        if check.blocked == "occupied" or ahead.blocked == "occupied":
            out["why"] = "blocked:occupied"
        elif zone_len > 0.0 and zone_check.blocked is not None:
            out["why"] = f"blocked:{zone_check.blocked}"
        elif zone_len > 0.0 and zone_verified + 1e-6 < need_zone:
            out["why"] = "unverified"
        elif allowed[0][0] <= 0.0:
            out["why"] = f"start:{allowed[0][1]}"
        elif allowed[1][0] <= 0.0:
            out["why"] = f"measured_start:{allowed[1][1]}"
        else:
            out["ok"], out["why"] = True, "ok"
        return out

    def check_arc_candidate(self, *, current_pose, curvature_inv_m: float, direction: int, loaded: bool,
                            cap_mps: float):
        """The emergency-stop arc the permission would walk at this curvature and direction."""
        footprint, own = self.footprints(loaded, direction)
        return self.permission.check_arc(self.snapshot, current_pose, curvature_inv_m, direction, footprint, own,
                                         cap_mps)

    def certify_withdrawal(self, path, shape, until_m: float, gaps_m) -> bool:
        """Plan v10 D5 withdrawal corridor: certified only when the smallest of the
        four blade-to-pocket gaps covers the checker's side width plus b_w and e_w
        (both measured at the gate; unmeasured -> refused). While certified the
        permission samples every withdraw_step_m and the delivered pallet is not
        applied inside the checker's own corridor."""
        from forklift_core.perception.known_obstacles import withdraw_certified

        kp = self.known_config
        step = float(kp.get("withdraw_step_m", 0.005))
        side = float(self.permission.config.envelope_offset_m) + step / 2 + float(kp.get("tracking_m", 0.006))
        ok = withdraw_certified(gaps_m, b_w=kp.get("b_w_m"), e_w=kp.get("e_w_m"), side_m=side)
        if ok:
            self._normal_config = self.permission.config
            self.permission.config = replace(self.permission.config, step_m=step)
            self.corridor = (np.asarray(path, dtype=float), shape, float(until_m))
        return ok

    def end_withdrawal(self, now_s: float | None = None) -> None:
        """End the corridor. A certified withdrawal re-fixes the delivered pallet: at the
        stop its relative pose is known to b_w + e_w (release error plus the measured
        drift to the stop), so its age restarts there instead of growing from the
        release -- otherwise it turns unknown around the truck that just left it
        (Codex stage-1 P1-6, plan v10 D5)."""
        if self.corridor is not None:
            self.permission.config = self._normal_config
            self.corridor = None
            kp = self.known_config
            # e_w is the lateral drift the certificate needs; the re-fix needs the full
            # relative error at the stop (all four corners, along the pallet too), measured
            # separately -- without it the pallet keeps ageing from the release (Codex
            # stage-1 2nd P1-2: a 0.163 m longitudinal drift left a pallet cell FREE).
            if now_s is not None and self.known and kp.get("b_w_m") is not None and kp.get("e_w_full_m") is not None:
                self.known = [replace(k, fix_stamp_s=float(now_s), r_fix_m=float(kp["b_w_m"]) + float(kp["e_w_full_m"]))
                              if k.kind == "pallet" else k for k in self.known]

    def limit(self, now_s: float, *, current_pose, curvature_inv_m: float, direction: int, loaded: bool, cap_mps: float):
        footprint, own = self.footprints(loaded, direction)
        return self.permission.allowed_speed(
            now_s, self.last_scan_s, current_pose=current_pose, curvature_inv_m=curvature_inv_m,
            direction=direction, footprint=footprint, own_footprint=own, speed_cap_mps=cap_mps,
        )

    def planner_grid(self, now_s: float, correction, version: int, *, use_slam: bool = True,
                     own_pose=None, loaded: bool = False):
        """Hall-wide OccupancyGrid of the occupied cells for a (re)plan: the live
        grid, and with the planning memory its remembered and SLAM cells."""
        if self.memory is not None:
            self._memory_hits(now_s, correction)
        snap = self.grid.snapshot(now_s, correction, version)
        if self.memory is None:
            return snapshot_to_occupancy(snap)
        self._memory_clears(snap, now_s)
        occupied = snap.occupied.copy()
        nx, ny = occupied.shape
        res = snap.resolution_m
        ci, cj = self.memory.cell(snap.origin_x_m + (np.arange(nx) + 0.5) * res,
                                  snap.origin_y_m + (np.arange(ny) + 0.5) * res)
        mem = self.memory.occupied(use_slam=use_slam)
        oki = (ci >= 0) & (ci < mem.shape[0])
        okj = (cj >= 0) & (cj < mem.shape[1])
        sub = np.zeros_like(occupied)
        sub[np.ix_(oki, okj)] = mem[np.ix_(ci[oki], cj[okj])]
        if own_pose is not None:
            # Where the truck stands nothing else can: the remembered cells wholly
            # inside its parts (blades and body apart, or the loaded outline) are
            # left out of this plan's copy only -- the memory keeps them, and a
            # cell straddling the outline stays (Codex design review P1).
            from forklift_core.control.drive_permission import parts_of
            from forklift_core.control.shadow_memory import whole_cells

            # Several poses (obstacle recovery v4: an escape's end region): only the cells wholly
            # inside the parts at every one of them, so the copy holds each pose's own cells.
            shape = self.loaded if loaded else (self.parts_shape or self.unloaded)
            poses = [own_pose] if np.ndim(own_pose) == 1 else list(own_pose)
            out = None
            for x, y, yaw in poses:
                c, s = math.cos(yaw), math.sin(yaw)
                cells = set()
                for fp, lat, lon in parts_of(shape):
                    part = (x + lon * c - lat * s, y + lon * s + lat * c, yaw)
                    cells.update((a, b) for a, b in whole_cells(snap, part, fp) if 0 <= a < nx and 0 <= b < ny)
                out = cells if out is None else out & cells
            for a, b in out:
                sub[a, b] = False
        from forklift_core.planning.grid_collision import OccupancyGrid

        return OccupancyGrid(snap.origin_x_m, snap.origin_y_m, res, occupied | sub, version=snap.correction_version)


__all__ = ["ObstacleLayer", "ObstacleSensor", "load_layer_config"]
