"""D5 v11: depth pocket check referenced to the tracked pallet (no Isaac imports).

docs/plans/2026-10-13-d5-v11-pallet-referenced-evidence.md. The v10 check
(pocket_check.PocketCheck) places its evidence in the control frame fixed at the
near capture and lets it expire after 20 s. Here the evidence lives in the
**pallet frame** of the frame that produced it: origin at the midpoint of the two
observed pocket front centres, x along the insertion axis (away from the truck),
z from the floor. A depth frame certifies only when the near-field tracker
accepted an observation in it (V1), and its rays go camera -> base_link -> pallet
through that observation, so the base pose cancels and only the observation error
(the measured bound, per axis) and the residual alignment motion m_k remain (V2).
Certification is eroded per axis by the frame's own error before it is merged
(V4); there is no lifetime, the store is dropped on invalidation events (V6). A
truck box asked at a tick is placed in the pallet frame through the tracker's
carried estimate and inflated per axis by e_d (V3). The body is not asked within
one voxel of the face (V8). V is the two pocket interiors plus the body band in
front of the face (V7).

The v10 path is untouched: this module adds the v11 check beside it.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

from forklift_core.control.drive_permission import StoppingModel, arc_poses
from forklift_core.perception.insertion_clearance import (
    Box,
    InsertionVolume,
    overlapping_voxels,
)
from forklift_core.perception.pallet_geometry import pallet_boxes
from forklift_core.perception.pocket_clearance import DepthCamera

RULES = "d5_v11"
H_DET_M = 0.17  # body band top (D0 detection band), as v10
SCOPE_FLOOR_M = 0.03  # D0 scope floor (user decision 2026-10-05), as v10
FORK_Z_M = (SCOPE_FLOOR_M, 0.072)  # lowered blades 0.028-0.052 +0.01, from the scope floor, as v10
PALLET_TERM_M = 0.00005  # the pallet-stillness term the S4 tracker carries (near_tracking.PALLET_TERM_M)


@dataclass
class V11Config:
    voxel_m: float = 0.01
    band_ahead_m: float = 0.10  # pocket columns start this far in front of the face (as v10)
    band_along_m: float = 0.60  # body band length in front of the face (as v10)
    band_across_m: float = 0.20  # the region's width beyond the pallet (as v10)
    body_lateral_m: float = 0.026  # v10's column/band allowance: estimate 0.01 + tracking 0.006 + envelope 0.01
    body_slack_m: float = 0.05  # as v10
    real_stop_extra_m: float = 0.02  # pocket columns reach the insertion depth + the real stop
    demand_extra_m: float = 0.03  # ... and this e_d room (V7)
    envelope_m: float = 0.01
    sample_m: float = 0.01
    surface_estimate_m: float = 0.02  # surface tolerance's estimate share (D5 delta, 2026-10-05)
    halo_m: float = 0.06  # V is evaluated this far around itself (x, y); wider erosion is refused
    face_cut_m: float = 0.01  # V8: the body is not asked within this of the estimated face
    # The truck's front below h_det is the carriage alone: the chassis box ends carriage_depth_m
    # behind the carriage face and the carriage is carriage_half_width_m wide (the dls08_measured
    # URDF: chassis front 0.099 m behind the carriage face, carriage |y| 0.235 m -- model values,
    # not measurements: the carriage width is estimated and the body front computed, README). None
    # keeps one body box to the carriage face (the plan's v4 model).
    carriage_depth_m: float | None = None
    carriage_half_width_m: float = 0.235
    frame_max_age_s: float = 0.2  # newest new frame's pixels at most this old (with L_hi)


def pallet_from_base(observation) -> tuple[float, float, float]:
    """(ox, oy, yaw): the pallet frame in base_link -- origin at the midpoint of the two
    pocket front centres, x along the observation's insertion yaw."""
    left, right = observation.left.center_m, observation.right.center_m
    return (left[0] + right[0]) / 2, (left[1] + right[1]) / 2, float(observation.insertion_yaw_rad)


def _to_frame(frame, x, y):
    """Points (x, y) expressed in a planar frame (ox, oy, yaw) given in their own frame."""
    ox, oy, yaw = frame
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = np.asarray(x, dtype=float) - ox, np.asarray(y, dtype=float) - oy
    return dx * c + dy * s, -dx * s + dy * c


def pose_in_frame(frame, pose) -> tuple[float, float, float]:
    """A planar pose (x, y, yaw) expressed in frame (ox, oy, yaw), both in one parent."""
    x, y = _to_frame(frame, pose[0], pose[1])
    return float(x), float(y), math.atan2(math.sin(pose[2] - frame[2]), math.cos(pose[2] - frame[2]))


def box_filter_xy(mask: np.ndarray, kx: int, ky: int) -> np.ndarray:
    """Erosion of a boolean grid over a (2kx+1) x (2ky+1) rectangle in x, y (V4: per axis);
    outside the grid counts as not free."""
    out = mask.copy()
    for axis, k in ((0, kx), (1, ky)):
        if k <= 0:
            continue
        n = out.shape[axis]
        pad = [(0, 0)] * out.ndim
        pad[axis] = (k, k)
        padded = np.pad(out, pad, constant_values=False)
        out = np.logical_and.reduce([np.take(padded, range(i, i + n), axis=axis) for i in range(2 * k + 1)])
    return out


def bound_ok(bound) -> bool:
    """A usable observation bound: lateral, along and yaw present, finite and not negative."""
    try:
        vals = [float(bound[k]) for k in ("lateral_m", "along_m", "yaw_rad")]
    except (KeyError, TypeError, ValueError):
        return False
    return all(math.isfinite(v) and v >= 0.0 for v in vals)


def all_finite(*values) -> bool:
    return bool(np.all(np.isfinite(np.asarray(values, dtype=float))))


def axis_errors(bound: dict, x_extent: float, y_extent: float, extra_m: float) -> tuple[float, float]:
    """(e_x, e_y): a per-axis error at the farthest |x|, |y| of a region -- V2/V3's
    e_.,x = along + |y|·yaw + term, e_.,y = lateral + |x|·yaw + term."""
    yaw = float(bound["yaw_rad"])
    return (float(bound["along_m"]) + y_extent * yaw + extra_m,
            float(bound["lateral_m"]) + x_extent * yaw + extra_m)


@dataclass
class PalletPocketCheck:
    pallet_geometry: object
    blades_rear: tuple  # rear-axle frame (x0, x1, y0, y1) per blade
    body_front_m: float
    body_rear_m: float
    body_half_width_m: float
    insertion_depth_m: float
    base_z_m: float
    base_from_optical: tuple  # (R, t) optical -> base_link at lift 0
    rear_axle_offset_m: float  # base_link x of the rear axle is -offset
    camera: DepthCamera
    stopping: StoppingModel
    bounds: object  # forklift_core.perception.near_field_bounds.NearFieldBounds
    config: V11Config = field(default_factory=V11Config)
    latency_band_s: tuple | None = None  # (L_lo, L_hi) of the bounds file, for Δ_L (V2)
    noise_seed: int = 0
    obstacle: bool = False
    obstacle_points: list = field(default_factory=list)
    records: list = field(default_factory=list)
    events: list = field(default_factory=list)
    last_hash: str | None = None
    last_seen_s: float = -math.inf  # the latest new render stamp (newness)
    last_new_s: float = -math.inf  # the latest frame placed and checked (freshness: placed frames only)

    def __post_init__(self) -> None:
        g, cfg = self.pallet_geometry, self.config
        half_depth = g.overall_depth_m / 2
        # Pallet solids in the pallet frame (face at x = 0).
        self.solids = [Box((b.centre_m[0] + half_depth, b.centre_m[1], b.centre_m[2]),
                           tuple(float(v) / 2 for v in b.size_m)) for b in pallet_boxes(g)]
        self.floor = Box((0.0, 0.0, -0.5), (50.0, 50.0, 0.5))
        parts = self._pocket_parts() + [self._body_band()]
        self.volume = InsertionVolume(parts, voxel_m=cfg.voxel_m, halo_m=cfg.halo_m)
        self._reset_store()
        self.region = (-cfg.band_along_m, g.overall_depth_m + cfg.band_along_m,
                       -(g.overall_width_m / 2 + cfg.band_across_m), g.overall_width_m / 2 + cfg.band_across_m)
        self.rng = np.random.default_rng([self.noise_seed, 13])
        self.real_stop = StoppingModel(self.stopping.latency_s, self.stopping.decel_mps2, cfg.envelope_m)
        self.l_hi = float(self.bounds.max_latency_s)
        self.align_s = float(self.bounds.align_latency_s)
        low = float(self.latency_band_s[0]) if self.latency_band_s else self.align_s
        self.delta_l = max(self.l_hi - self.align_s, self.align_s - low)
        v = cfg.voxel_m
        idx, centres = self.volume.centres(evaluated=True)
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        self._x_extent = float(np.abs(centres[in_v, 0]).max() + v / 2)  # V's farthest |x|, |y| (V2, V4)
        self._y_extent = float(np.abs(centres[in_v, 1]).max() + v / 2)

    # V ----------------------------------------------------------------------
    def _pocket_parts(self) -> list:
        """V7: each pocket between its walls (only voxels wholly inside), from band_ahead in
        front of the face to the insertion depth + the real stop + the e_d room."""
        g, cfg, v = self.pallet_geometry, self.config, self.config.voxel_m
        ys = g.block_centres_y_m()
        widths = g.block_widths_m
        x0 = -cfg.band_ahead_m
        x1 = self.insertion_depth_m + cfg.real_stop_extra_m + cfg.demand_extra_m
        parts = []
        for k in range(len(ys) - 1):
            lo = ys[k] + widths[k] / 2
            hi = ys[k + 1] - widths[k + 1] / 2
            lo_in = math.ceil(lo / v - 1e-9) * v  # whole voxels inside the walls
            hi_in = math.floor(hi / v + 1e-9) * v
            if hi_in - lo_in < v - 1e-12:
                continue
            parts.append(Box(((x0 + x1) / 2, (lo_in + hi_in) / 2, sum(FORK_Z_M) / 2),
                             ((x1 - x0) / 2, (hi_in - lo_in) / 2, (FORK_Z_M[1] - FORK_Z_M[0]) / 2)))
        return parts

    def _body_band(self) -> Box:
        cfg = self.config
        x0 = -cfg.band_along_m
        half_w = self.body_half_width_m + cfg.body_lateral_m + cfg.body_slack_m
        return Box((x0 / 2, 0.0, (SCOPE_FLOOR_M + H_DET_M) / 2),
                   (-x0 / 2, half_w, (H_DET_M - SCOPE_FLOOR_M) / 2))

    def _reset_store(self) -> None:
        shape = self.volume.shape
        self.certified = np.zeros(shape, dtype=bool)
        self.best_k = np.full(shape + (2,), 10**6, dtype=np.int64)  # (kx, ky) of the smallest certification
        self.cert_frame = np.full(shape, -1, dtype=np.int64)
        self.last_cert_s = np.full(shape, -np.inf)
        self.last_code = np.zeros(shape, dtype=np.int8)

    # Frames -------------------------------------------------------------------
    def optical_from_pallet(self, observation, lift_m: float):
        """(R, t) pallet frame -> optical through the frame's own observation (V1: the base
        pose cancels)."""
        ox, oy, yaw = pallet_from_base(observation)
        c, s = math.cos(yaw), math.sin(yaw)
        r_bp = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # pallet -> base (rotation)
        t_bp = np.array([ox, oy, -self.base_z_m])  # pallet z from the floor, base z from base_link
        r_bo, t_bo = (np.asarray(a, dtype=float) for a in self.base_from_optical)
        t_bo = t_bo + np.array([0.0, 0.0, lift_m])
        r_ob, t_ob = r_bo.T, -r_bo.T @ t_bo
        return r_ob @ r_bp, r_ob @ t_bp + t_ob

    def frame_errors(self, bound: dict, motion: tuple) -> tuple[float, float]:
        """(e_c,x, e_c,y) at V's farthest point: the observation bound per axis, the pallet
        term and m_k = Δ_L (v̄ + ω̄ ρ) (V2), ρ the rear axle to V's farthest point."""
        vbar, wbar, rho = (float(v) for v in motion)
        m_k = self.delta_l * (abs(vbar) + abs(wbar) * rho)
        return axis_errors(bound, self._x_extent, self._y_extent, PALLET_TERM_M + m_k)

    def add_frame(self, stamp_s: float, depth_m: np.ndarray, observation, bound: dict | None, lift_m: float,
                  motion: tuple = (0.0, 0.0, 0.0), truck_pallet=None, carried=None) -> dict:
        """One depth frame. observation/bound: the tracker's accepted observation in this frame
        (None: the frame is only an obstacle check, V5 -- then truck_pallet, the base pose in the
        pallet frame by the carried estimate, places it and carried = (the latest accepted
        observation's bound, its stamp) gives that estimate's e_d). motion: (v̄, ω̄, ρ) over the
        frame's pixel-time window. A non-finite observation, bound or motion certifies nothing."""
        # A frame is new by its render stamp: a re-read of the same render carries the same stamp,
        # and a standing camera may render identical pixels at a new time.
        if not math.isfinite(float(stamp_s)) or float(stamp_s) <= self.last_seen_s + 1e-9:
            rec = {"time_s": float(stamp_s), "new": False}
            self.records.append(rec)
            return rec
        raw = np.ascontiguousarray(np.asarray(depth_m, dtype=np.float32))
        self.last_hash = hashlib.sha256(raw.tobytes()).hexdigest()
        self.last_seen_s = float(stamp_s)
        depth = np.asarray(depth_m, dtype=float).copy()
        finite = np.isfinite(depth) & (depth > 0)
        k_sigma = self.camera.sigma_k
        depth[finite] += np.clip(self.rng.normal(0.0, 1.0, int(finite.sum())), -k_sigma, k_sigma) * (
            self.camera.sigma_a * depth[finite] ** 2)
        accepted = (observation is not None and bound_ok(bound) and all_finite(*motion, lift_m)
                    and all_finite(*pallet_from_base(observation)))
        errors_at = None
        if accepted:
            rot, trans = self.optical_from_pallet(observation, lift_m)
            e_x, e_y = self.frame_errors(bound, motion)
            errors_at = self._frame_errors_at(bound, motion)
        elif truck_pallet is not None and all_finite(*truck_pallet, lift_m) and carried is not None:
            errors_at = self._carried_errors_at(truck_pallet, carried, stamp_s)
        if errors_at is None:  # not placed: no obstacle check, so not fresh either
            rec = {"time_s": float(stamp_s), "new": True, "accepted": False, "placed": False}
            self.records.append(rec)
            return rec
        self.last_new_s = float(stamp_s)
        if not accepted:
            rot, trans = self._optical_from_truck(truck_pallet, lift_m)
        idx, centres = self.volume.centres(evaluated=True)
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        pts = centres @ rot.T + trans
        z = pts[:, 2]
        cam = self.camera
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.round(cam.fx * pts[:, 0] / z + cam.cx)
            w = np.round(cam.fy * pts[:, 1] / z + cam.cy)
        inside = (z > 0) & (u >= 0) & (u < cam.width) & (w >= 0) & (w < cam.height)
        ui = np.where(inside, u, 0).astype(np.int64)
        wi = np.where(inside, w, 0).astype(np.int64)
        measured = np.where(inside, depth[wi, ui], np.nan)
        valid = inside & np.isfinite(measured) & (measured > 0)
        margin = cam.margin_m(z)
        free = valid & (measured > z + margin)
        front = valid & (measured < z - margin) & in_v
        latched = self.obstacle
        obstacle_count = self._obstacle_check(stamp_s, rot, trans, ui, wi, measured, front, errors_at,
                                              observation if accepted else None, truck_pallet, lift_m)
        rec = {"time_s": float(stamp_s), "new": True, "accepted": accepted, "obstacle": obstacle_count}
        if self.obstacle and not latched:
            self.invalidate("obstacle", stamp_s)  # V6: the latch drops the store; nothing certifies after it
        if accepted and not self.obstacle:
            v = self.config.voxel_m
            kx = max(1, math.ceil(e_x / v - 1e-9))
            ky = max(1, math.ceil(e_y / v - 1e-9))
            rec.update(e_c=[e_x, e_y], k=[kx, ky])
            if max(kx, ky) * v > self.config.halo_m + 1e-12:
                rec["refused"] = "erosion_beyond_halo"
            else:
                free_grid = np.zeros(self.volume.shape, dtype=bool)
                free_grid[idx[free, 0], idx[free, 1], idx[free, 2]] = True
                cert = box_filter_xy(free_grid, kx, ky) & self.volume.in_v
                frame_no = sum(1 for r in self.records if r.get("accepted"))
                better = cert & ((kx + ky) < self.best_k.sum(axis=-1))
                self.best_k[better] = (kx, ky)
                self.cert_frame[better] = frame_no
                self.certified |= cert
                self.last_cert_s[cert] = float(stamp_s)
                rec["certified"] = int(cert.sum())
            seen = inside & in_v
            code = np.full(len(idx), 5, dtype=np.int8)
            code[free] = 2
            code[front] = 3
            si = idx[seen]
            self.last_code[si[:, 0], si[:, 1], si[:, 2]] = code[seen]
            if "certified" in rec:
                self.last_code[cert] = 1
        self.records.append(rec)
        return rec

    def _optical_from_truck(self, truck_pallet, lift_m: float):
        """(R, t) pallet -> optical with the truck's base pose in the pallet frame (V5's frames
        without an accepted observation)."""
        x, y, yaw = truck_pallet
        c, s = math.cos(yaw), math.sin(yaw)
        r_pb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # base -> pallet
        r_bp = r_pb.T
        t_bp = -r_bp @ np.array([x, y, self.base_z_m])
        r_bo, t_bo = (np.asarray(a, dtype=float) for a in self.base_from_optical)
        t_bo = t_bo + np.array([0.0, 0.0, lift_m])
        r_ob, t_ob = r_bo.T, -r_bo.T @ t_bo
        return r_ob @ r_bp, r_ob @ t_bp + t_ob

    def _frame_errors_at(self, bound: dict, motion: tuple):
        """Per-point e_c (V2) of an accepted frame: along + |y|·yaw, lateral + |x|·yaw, + term + m_k."""
        vbar, wbar, rho = (float(v) for v in motion)
        extra = PALLET_TERM_M + self.delta_l * (abs(vbar) + abs(wbar) * rho)
        yaw = float(bound["yaw_rad"])

        def at(pts):
            return (float(bound["along_m"]) + np.abs(pts[:, 1]) * yaw + extra,
                    float(bound["lateral_m"]) + np.abs(pts[:, 0]) * yaw + extra)
        return at

    def _carried_errors_at(self, truck_pallet, carried, stamp_s: float):
        """Per-point e_d (V3) of the carried estimate that places a frame without an accepted
        observation: the latest bound + e(a) + ρ_p ψ(a) + term, ρ_p from the rear axle; None when
        the bound is unusable or the age has no odometry row (the frame cannot be placed)."""
        bound, latest_stamp_s = carried
        if not bound_ok(bound) or not all_finite(latest_stamp_s):
            return None
        odo = self.demand_terms(float(stamp_s) - (float(latest_stamp_s) - self.l_hi))
        if odo is None:
            return None
        e, psi = odo
        x, y, yaw_t = truck_pallet
        rear = (x - self.rear_axle_offset_m * math.cos(yaw_t), y - self.rear_axle_offset_m * math.sin(yaw_t))
        yaw = float(bound["yaw_rad"])

        def at(pts):
            extra = e + np.hypot(pts[:, 0] - rear[0], pts[:, 1] - rear[1]) * psi + PALLET_TERM_M
            return (float(bound["along_m"]) + np.abs(pts[:, 1]) * yaw + extra,
                    float(bound["lateral_m"]) + np.abs(pts[:, 0]) * yaw + extra)
        return at

    def _obstacle_check(self, stamp_s, rot, trans, ui, wi, measured, front, errors_at, observation, truck_pallet,
                        lift_m) -> int:
        """V5: a return in front of a V voxel that lies off the pallet surface (placed by this
        frame's observation with e_c, or by the carried estimate with e_d, both at the return's
        own position) and off the floor and the truck's own parts is an obstacle (latched).
        Tolerance kσ + the estimate share + the error along the surface normal (front face e_x,
        side faces e_y, none for horizontal)."""
        if not front.any():
            return 0
        cam = self.camera
        ret_opt = np.column_stack(((ui[front] - cam.cx) / cam.fx, (wi[front] - cam.cy) / cam.fy,
                                   np.ones(int(front.sum())))) * measured[front][:, None]
        ret = (ret_opt - trans) @ rot  # pallet frame
        base_tol = cam.margin_m(measured[front]) + self.config.surface_estimate_m
        e_x, e_y = errors_at(ret)
        on = np.zeros(len(ret), dtype=bool)
        for b in self.solids:
            tx, ty, tz = base_tol + e_x, base_tol + e_y, base_tol
            on |= ((np.abs(ret[:, 0] - b.center[0]) <= b.half[0] + tx)
                   & (np.abs(ret[:, 1] - b.center[1]) <= b.half[1] + ty)
                   & (np.abs(ret[:, 2] - b.center[2]) <= b.half[2] + tz))
        on |= ret[:, 2] <= base_tol  # the floor
        truck = truck_pallet if truck_pallet is not None else self._truck_from_observation(observation)
        for b in self.own_boxes(truck, lift_m):
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            dx, dy = ret[:, 0] - b.center[0], ret[:, 1] - b.center[1]
            on |= ((np.abs(dx * c + dy * s) <= b.half[0] + base_tol) & (np.abs(-dx * s + dy * c) <= b.half[1] + base_tol)
                   & (np.abs(ret[:, 2] - b.center[2]) <= b.half[2] + base_tol))
        count = int((~on).sum())
        if count:
            self.obstacle = True
            self.obstacle_points.append({"time_s": float(stamp_s), "points": ret[~on][:20].tolist()})
        return count

    def _truck_from_observation(self, observation):
        """The truck's base pose in the pallet frame of an observation (base_link origin)."""
        return pose_in_frame(pallet_from_base(observation), (0.0, 0.0, 0.0))

    def invalidate(self, reason: str, now_s: float) -> None:
        """V6: drop the whole store (mismatch, unbounded, loss end, a new episode, an obstacle)."""
        self._reset_store()
        self.events.append({"time_s": float(now_s), "event": "invalidated", "reason": reason})

    # Per tick ----------------------------------------------------------------
    def own_boxes(self, pose_p, lift_m: float) -> list:
        """The blades and the carriage face (0.02 m deep) at a base pose in the pallet frame."""
        x, y, yaw = pose_p
        c, s = math.cos(yaw), math.sin(yaw)
        off = self.rear_axle_offset_m
        out = []
        for x0, x1, y0, y1 in self.blades_rear:
            u, v = (x0 + x1) / 2 - off, (y0 + y1) / 2
            out.append(Box((x + u * c - v * s, y + u * s + v * c, 0.04 + lift_m),
                           ((x1 - x0) / 2, (y1 - y0) / 2, 0.012), yaw))
        u = self.body_front_m - 0.01 - off
        out.append(Box((x + u * c, y + u * s, 0.25 + lift_m), (0.01, 0.235, 0.21), yaw))
        return out

    def demand_terms(self, age_s: float):
        """(e(a), ψ(a)) of the odometry table for the carried estimate's age, or None when the
        table has no row for it (V3: pocket_unbounded)."""
        return self.bounds.odometry(max(0.0, age_s))

    def demand_errors(self, bound: dict, odo, rear_p, rect) -> tuple[float, float]:
        """(e_d,x, e_d,y) of one truck box (V3): at each corner (x_p, y_p) of the rear-axle frame
        rectangle rect = (x0, x1, y0, y1) placed at rear_p in the pallet frame, e_d,x = along +
        |y_p|·yaw + e(a) + ρ_p ψ(a) + term and e_d,y = lateral + |x_p|·yaw + the same, ρ_p the
        corner's distance from the rear axle; the largest over the corners per axis."""
        e, psi = odo
        x, y, yaw_t = rear_p
        c, s = math.cos(yaw_t), math.sin(yaw_t)
        yaw = float(bound["yaw_rad"])
        x0, x1, y0, y1 = rect
        e_x = e_y = -math.inf
        for u in (x0, x1):
            for w in (y0, y1):
                xp, yp = x + u * c - w * s, y + u * s + w * c
                extra = e + math.hypot(u, w) * psi + PALLET_TERM_M
                e_x = max(e_x, float(bound["along_m"]) + abs(yp) * yaw + extra)
                e_y = max(e_y, float(bound["lateral_m"]) + abs(xp) * yaw + extra)
        return e_x, e_y

    def truck_rects(self) -> list:
        """(name, rear-axle frame rectangle (x0, x1, y0, y1), z range, is body) of the truck's boxes
        below h_det: the body (chassis to the carriage when carriage_depth_m is set, else to
        the carriage face), the carriage and the blades (z at lift 0)."""
        depth = self.config.carriage_depth_m
        chassis_front = self.body_front_m - (depth or 0.0)
        hw = self.body_half_width_m
        out = [("body", (-self.body_rear_m, chassis_front, -hw, hw), True)]
        if depth:
            cw = self.config.carriage_half_width_m
            out.append(("carriage", (self.body_front_m - depth, self.body_front_m, -cw, cw), True))
        for x0, x1, y0, y1 in self.blades_rear:
            out.append(("blade_left" if (y0 + y1) > 0 else "blade_right", (x0, x1, y0, y1), False))
        return out

    def truck_boxes(self, rear_p, lift_m: float, bound: dict, odo) -> tuple[list, list]:
        """Body (scope floor .. h_det), carriage and blades at the lift around a rear-axle pose in
        the pallet frame, each inflated per axis by its own e_d (V3, at the box's corners) and
        sideways by the envelope (along the axis the envelope is in the stop length). Returns
        (boxes, their e_d)."""
        x, y, yaw = rear_p
        c, s = math.cos(yaw), math.sin(yaw)
        sin_t = abs(math.sin(yaw))
        env = self.config.envelope_m
        boxes, errors = [], []
        z_blade = (max(0.028 + lift_m - 0.01, SCOPE_FLOOR_M), 0.052 + lift_m + 0.01)
        for _, rect, body in self.truck_rects():
            e_d = self.demand_errors(bound, odo, rear_p, rect)
            e_x, e_y = e_d
            along, across = e_x + e_y * sin_t, e_y + e_x * sin_t  # V3 as written (>= the exact projection)
            x0, x1, y0, y1 = rect
            u, w = (x0 + x1) / 2, (y0 + y1) / 2
            z0, z1 = (SCOPE_FLOOR_M, H_DET_M) if body else z_blade
            boxes.append(Box((x + u * c - w * s, y + u * s + w * c, (z0 + z1) / 2),
                             ((x1 - x0) / 2 + along, (y1 - y0) / 2 + across + env, (z1 - z0) / 2), yaw))
            errors.append(e_d)
        return boxes, errors

    def contained(self, box: Box, body: bool) -> bool:
        """Every voxel the box overlaps inside the region is in V and certified; the body is
        not asked within face_cut_m of the face, inside the face width (V8)."""
        if self.obstacle:
            return False
        if not all_finite(*box.center, *box.half, box.yaw):
            return False
        if not self._meets_region(box):
            return True  # wholly outside the region: the grid's business (as v10)
        idx, leaves = overlapping_voxels(self.volume, box, self.region)
        v = self.volume.voxel_m
        lo = self.volume.lo
        vx0 = lo[0] + idx[:, 0] * v
        vy0 = lo[1] + idx[:, 1] * v
        x0, x1, y0, y1 = self.region
        keep = (vx0 + v > x0) & (vx0 < x1) & (vy0 + v > y0) & (vy0 < y1)
        if body:
            half_face = self.pallet_geometry.overall_width_m / 2
            cut = (vx0 + v > -self.config.face_cut_m) & (np.abs(vy0 + v / 2) <= half_face)
            keep &= ~cut
        idx = idx[keep]
        if leaves and self._box_leaves_grid(box):
            return False
        if not len(idx):
            return True
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        cert = self.certified[idx[:, 0], idx[:, 1], idx[:, 2]]
        return bool((in_v & cert).all())

    def _meets_region(self, box: Box) -> bool:
        """Whether the box's xy footprint overlaps the region (2D SAT, as v10)."""
        x0, x1, y0, y1 = self.region
        c, s = math.cos(box.yaw), math.sin(box.yaw)
        hx, hy, _ = box.half
        ex, ey = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        if box.center[0] + ex < x0 or box.center[0] - ex > x1 or box.center[1] + ey < y0 or box.center[1] - ey > y1:
            return False
        rx, ry = (x1 - x0) / 2, (y1 - y0) / 2
        dx, dy = box.center[0] - (x0 + x1) / 2, box.center[1] - (y0 + y1) / 2
        return (abs(dx * c + dy * s) <= hx + rx * abs(c) + ry * abs(s)
                and abs(-dx * s + dy * c) <= hy + rx * abs(s) + ry * abs(c))

    def _explain(self, box: Box, body: bool) -> dict:
        """The asked voxels a box fails on: outside V or never certified, with examples."""
        idx, leaves = overlapping_voxels(self.volume, box, self.region)
        v, lo = self.volume.voxel_m, self.volume.lo
        vx0 = lo[0] + idx[:, 0] * v
        vy0 = lo[1] + idx[:, 1] * v
        x0, x1, y0, y1 = self.region
        keep = (vx0 + v > x0) & (vx0 < x1) & (vy0 + v > y0) & (vy0 < y1)
        if body:
            keep &= ~((vx0 + v > -self.config.face_cut_m)
                      & (np.abs(vy0 + v / 2) <= self.pallet_geometry.overall_width_m / 2))
        idx = idx[keep]
        in_v = self.volume.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        cert = self.certified[idx[:, 0], idx[:, 1], idx[:, 2]]
        bad = idx[~(in_v & cert)]
        centres = lo + (bad + 0.5) * v
        return {"leaves": bool(leaves), "asked": int(len(idx)), "outside_v": int((~in_v).sum()),
                "uncertified": int((in_v & ~cert).sum()), "examples": np.round(centres[:12], 3).tolist(),
                "examples_code": [int(self.last_code[tuple(b)]) for b in bad[:12]]}

    def _box_leaves_grid(self, box: Box) -> bool:
        """Whether the part of the box inside the region leaves the voxel grid in x, y (1e-9
        tolerance). Height is not asked: below the scope floor and above h_det nothing is in
        scope (D0), and the grid spans exactly that band."""
        c, s = math.cos(box.yaw), math.sin(box.yaw)
        hx, hy, _ = box.half
        corners = [(box.center[0] + a * hx * c - b * hy * s, box.center[1] + a * hx * s + b * hy * c)
                   for a in (-1, 1) for b in (-1, 1)]
        x0, x1, y0, y1 = self.region
        xs = np.clip([p[0] for p in corners], x0, x1)
        ys = np.clip([p[1] for p in corners], y0, y1)
        vol = self.volume
        top = vol.lo + np.array(vol.shape) * vol.voxel_m
        tol = 1e-9
        return not (xs.min() >= vol.lo[0] - tol and xs.max() <= top[0] + tol
                    and ys.min() >= vol.lo[1] - tol and ys.max() <= top[1] + tol)

    def limit(self, now_s: float, rear_p, latest_bound: dict, latest_stamp_s: float, curvature_inv_m: float,
              direction: int, speed_mps: float, lift_m: float) -> tuple[float, str]:
        """The speed the stop arc allows: rear_p is the rear axle in the pallet frame by the
        tracker's carried estimate, latest_bound/latest_stamp_s its latest accepted observation.
        A non-finite pose, bound or motion input is unbounded: nothing can be placed."""
        if self.obstacle:
            return 0.0, "pocket_obstacle"
        if not all_finite(now_s, *rear_p, latest_stamp_s, curvature_inv_m, speed_mps, lift_m) or not bound_ok(latest_bound):
            return 0.0, "pocket_unbounded"
        if now_s - (self.last_new_s - self.l_hi) > self.config.frame_max_age_s:
            return 0.0, "depth_stale"
        age = now_s - (latest_stamp_s - self.l_hi)
        odo = self.demand_terms(age)
        if odo is None:
            return 0.0, "pocket_unbounded"
        length = self.real_stop.distance_m(abs(speed_mps)) + self.config.sample_m
        samples, arc = arc_poses(tuple(rear_p), curvature_inv_m, 1 if direction >= 0 else -1, length,
                                 self.config.sample_m)
        verified = 0.0
        rects = self.truck_rects()
        for pose, s in zip(samples, arc, strict=True):
            boxes, errors = self.truck_boxes(pose, lift_m, latest_bound, odo)
            failing = [k for k, b in enumerate(boxes) if not self.contained(b, body=rects[k][2])]
            if failing:
                k = failing[0]
                self.last_fail = {"box": rects[k][0], "arc_s": float(s),
                                  "pose_pallet": [round(float(c), 4) for c in pose],
                                  "e_d": list(errors[k]), "age_s": float(age),
                                  **self._explain(boxes[k], rects[k][2])}
                break
            verified = float(s)
        else:
            return math.inf, "ok"
        allowed = self.real_stop.speed_for(verified)
        return allowed, "ok" if allowed > 0 else "pocket_unverified"

    def summary(self) -> dict:
        accepted = [r for r in self.records if r.get("accepted")]
        best = self.best_k[self.certified]
        return {
            "rules": RULES,
            "obstacle": self.obstacle,
            "obstacle_points": self.obstacle_points[:5],
            "last_fail": getattr(self, "last_fail", None),
            "frames": len(self.records),
            "new_frames": sum(1 for r in self.records if r.get("new")),
            "accepted_frames": len(accepted),
            "refused_frames": sum(1 for r in accepted if r.get("refused")),
            "certified_voxels": int(self.certified.sum()),
            "voxels": int(self.volume.in_v.sum()),
            "best_erosion": {f"{kx},{ky}": int(n) for (kx, ky), n in
                             zip(*np.unique(best, axis=0, return_counts=True), strict=True)} if len(best) else {},
            "events": self.events,
            "config": self.config.__dict__,
        }


__all__ = ["PalletPocketCheck", "RULES", "V11Config", "box_filter_xy", "pallet_from_base", "pose_in_frame"]
