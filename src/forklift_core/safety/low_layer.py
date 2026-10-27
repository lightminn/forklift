"""The near-field low layer (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md section S).

The 2D LiDAR plane (1.05 m) does not see an empty pallet (0.144 m) or anything else below it. This layer keeps, in the
odometry frame, what the forward depth camera established about the floor near the truck, and where the truck itself
has been:

- camera cells (a grid): ``FREE`` when a valid depth ray reached the verified floor inside the cell, the rays of the
  same frame crossed the cell's column at every height from ``band_low_m`` to ``verify_high_m`` (a floor hit under a
  floating box is no proof), and no point of the collision band (``band_low_m``..``band_high_m`` above the floor) fell
  in it; ``OCCUPIED`` when a band point fell in it; ``UNOBSERVED`` otherwise (dropouts, occlusions, the self mask, a
  frame whose floor plane failed). The newest evidence wins.
- the swept history (geometric, not cells): the truck's footprint at each step, with its time; a strip declared clear
  at the start (the explicit start condition) is kept the same way. A cell the body only clips cannot be proven clear
  by cells -- the camera never sees beside the truck -- so the body's own area is kept as rectangles.

Evidence is good for ``max_age_s`` and is shrunk by the odometry error since its time (the team-spec age table: the
position error plus the heading error times the distance from the rear axle, the form the obstacle layer's known-pallet
growth uses; the first tabulated age at or above it, zero only for the current pose); camera cells also by half a cell
and the depth error. A path is admissible when every point of the footprint along it, up to the stopping distance and
outside the footprint the truck stands in now, lies in a shrunk past footprint (whose area no band point has claimed
since) or in a usable camera cell -- camera cells only going forward (the camera does not look back).

Pure numpy; no ROS and no Isaac. Points come in already transformed into the odometry frame.
"""

from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from forklift_core.planning.geometry import Footprint

UNOBSERVED, FREE, OCCUPIED = 0, 1, 2
_AGE_TABLE = Path(__file__).resolve().parents[3] / "config" / "obstacle_odometry_age_team_spec.json"
REACTION_S, DECEL_MPS2 = 0.15, 1.5  # control.drive_permission's stopping model


def _age_table(path: Path = _AGE_TABLE) -> tuple:
    data = json.loads(path.read_text())
    return tuple(data["ages_s"]), tuple(data["position_max_m"]), tuple(data["yaw_max_rad"])


@dataclass(frozen=True)
class LowLayerConfig:
    resolution_m: float = 0.05
    band_low_m: float = 0.03  # the collision band above the floor
    band_high_m: float = 1.0
    verify_high_m: float = 0.15  # the column rays must have crossed for a free cell (plan S H_VERIFY)
    verify_bin_m: float = 0.03
    floor_tol_m: float = 0.02  # a point this close to the floor plane is a floor hit
    max_age_s: float = 3.0  # evidence older than this is not used (plan S; the table goes to 10 s)
    depth_error_m: float = 0.02
    fine_m: float = 0.005  # the envelope's raster: a pixel must lie wholly inside known-clear ground
    # the floor plane check: the plane must be where the floor is expected under the mount
    plane_height_tol_m: float = 0.05
    plane_normal_tol_rad: float = math.radians(5.0)
    plane_min_points: int = 400
    plane_max_rms_m: float = 0.01
    plane_iterations: int = 64
    plane_seed: int = 0
    age_table: tuple = field(default_factory=_age_table)


def odometry_error_m(cfg: LowLayerConfig, age_s: float, rho_m=0.0):
    """The odometry error after ``age_s`` at ``rho_m`` from the rear axle: the first tabulated age at or above it
    (position + 2 rho sin(heading / 2)); zero for age 0 (the current pose)."""
    if age_s <= 0.0:
        return 0.0 * np.asarray(rho_m, float)
    ages, pos, yaw = cfg.age_table
    i = min(bisect.bisect_left(ages, age_s - 1e-9), len(ages) - 1)
    return pos[i] + 2.0 * np.asarray(rho_m, float) * math.sin(yaw[i] / 2.0)


def erosion_m(cfg: LowLayerConfig, age_s: float, rho_m=0.0):
    """How far in from the edge of camera evidence of ``age_s`` must a point be: the odometry error + half a cell +
    the depth error."""
    return 0.5 * cfg.resolution_m + cfg.depth_error_m + odometry_error_m(cfg, age_s, rho_m)


def braking_end_s(speed_mps: float) -> float:
    """How long after now the truck stands still if it brakes now (reaction, then the deceleration)."""
    return REACTION_S + abs(speed_mps) / DECEL_MPS2


@dataclass(frozen=True)
class Plane:
    ok: bool
    normal: np.ndarray
    offset_m: float  # plane is normal . p + offset = 0
    rms_m: float
    inliers: int
    reason: str | None = None


def fit_floor(points: np.ndarray, cfg: LowLayerConfig, expected_z: float = 0.0) -> Plane:
    """The floor plane by RANSAC over points within ``plane_height_tol_m`` of the expected floor height, refused unless
    it is near that height, near horizontal, supported by ``plane_min_points`` and flat to ``plane_max_rms_m``."""
    none = Plane(False, np.array([0.0, 0.0, 1.0]), -expected_z, math.inf, 0)
    pts = np.asarray(points, float).reshape(-1, 3)
    near = pts[np.abs(pts[:, 2] - expected_z) <= cfg.plane_height_tol_m]
    if len(near) < cfg.plane_min_points:
        return Plane(**{**none.__dict__, "reason": "too_few_points"})
    rng = np.random.default_rng(cfg.plane_seed)
    best = None
    for _ in range(cfg.plane_iterations):
        a, b, c = near[rng.choice(len(near), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if n[2] < 0:
            n = -n
        d = -float(n @ a)
        count = int((np.abs(near @ n + d) <= cfg.floor_tol_m).sum())
        if best is None or count > best[2]:
            best = (n, d, count)
    if best is None:
        return Plane(**{**none.__dict__, "reason": "degenerate"})
    n, d, _ = best
    inl = near[np.abs(near @ n + d) <= cfg.floor_tol_m]
    A = np.c_[inl[:, 0], inl[:, 1], np.ones(len(inl))]  # least squares on the inliers (z = a x + b y + c)
    coef, *_ = np.linalg.lstsq(A, inl[:, 2], rcond=None)
    n = np.array([-coef[0], -coef[1], 1.0])
    n /= np.linalg.norm(n)
    d = -coef[2] * n[2]
    rms = float(np.sqrt(np.mean((inl @ n + d) ** 2)))
    tilt = math.acos(min(1.0, float(n[2])))
    height = -d / n[2]
    reason = None
    if len(inl) < cfg.plane_min_points:
        reason = "too_few_inliers"
    elif tilt > cfg.plane_normal_tol_rad:
        reason = "not_horizontal"
    elif abs(height - expected_z) > cfg.plane_height_tol_m:
        reason = "not_at_floor_height"
    elif rms > cfg.plane_max_rms_m:
        reason = "not_flat"
    return Plane(reason is None, n, float(d), rms, len(inl), reason)


@dataclass(frozen=True)
class Integration:
    ok: bool
    plane: Plane
    free_cells: int
    occupied_cells: int


def _local(points: np.ndarray, pose) -> tuple:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = points[..., 0] - x, points[..., 1] - y
    return c * dx + s * dy, -s * dx + c * dy


class LowLayer:
    """Camera cells on a fixed odometry-frame grid of ``size_cells`` (nx, ny) from ``origin_xy`` (the corner of cell
    (0, 0)), and the swept history."""

    def __init__(self, cfg: LowLayerConfig, origin_xy: tuple, size_cells: tuple):
        self.cfg = cfg
        self.origin = np.asarray(origin_xy, float)
        self.shape = tuple(int(v) for v in size_cells)
        self.free_t = np.full(self.shape, -np.inf)
        self.occupied_t = np.full(self.shape, -np.inf)
        self.swept: list = []  # (t, rear pose, x0, x1, half width): rectangles in the truck frame of that pose
        self.frames: list = []  # (t, cells freed by that frame): each frame's own support

    # --- cells -------------------------------------------------------------------------------------------------
    def cell(self, x: float, y: float) -> tuple:
        i = int(math.floor((x - self.origin[0]) / self.cfg.resolution_m))
        j = int(math.floor((y - self.origin[1]) / self.cfg.resolution_m))
        return i, j

    def _cells(self, xy: np.ndarray) -> tuple:
        ij = np.floor((np.asarray(xy, float) - self.origin) / self.cfg.resolution_m).astype(int)
        inside = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        return ij[inside, 0], ij[inside, 1], inside

    def centres(self) -> tuple:
        res = self.cfg.resolution_m
        xs = self.origin[0] + (np.arange(self.shape[0]) + 0.5) * res
        ys = self.origin[1] + (np.arange(self.shape[1]) + 0.5) * res
        return np.meshgrid(xs, ys, indexing="ij")

    def state_at(self, x: float, y: float) -> int:
        i, j = self.cell(x, y)
        if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1]):
            return UNOBSERVED
        f, o = self.free_t[i, j], self.occupied_t[i, j]
        if o > -np.inf and o >= f:
            return OCCUPIED
        return FREE if f > -np.inf else UNOBSERVED

    # --- evidence ----------------------------------------------------------------------------------------------
    def integrate(self, points: np.ndarray, t: float, *, origin, expected_floor_z: float = 0.0,
                  self_mask=None) -> Integration:
        """A frame's valid depth points (odometry frame) captured at ``t`` from the camera at ``origin``: points inside
        ``self_mask`` = (footprint, rear pose at capture) are the truck itself (its forks) and are dropped; the floor
        plane next (a refused plane marks nothing); then band points occupy their cells and floor hits free theirs
        where the frame's rays crossed the whole verified column."""
        pts = np.asarray(points, float).reshape(-1, 3)
        if self_mask is not None and len(pts):
            foot, pose = self_mask
            lx, ly = _local(pts, pose)
            pts = pts[~((lx >= -foot.rear_m) & (lx <= foot.front_m) & (np.abs(ly) <= foot.half_width_m))]
        plane = fit_floor(pts, self.cfg, expected_floor_z)
        if not plane.ok:
            return Integration(False, plane, 0, 0)
        h = pts @ plane.normal + plane.offset_m  # height above the floor plane
        band = (h >= self.cfg.band_low_m) & (h <= self.cfg.band_high_m)
        floor = np.abs(h) <= self.cfg.floor_tol_m
        oi, oj, _ = self._cells(pts[band, :2])
        fi, fj, _ = self._cells(pts[floor, :2])
        hit = np.zeros(self.shape, bool)
        hit[oi, oj] = True
        floor_hit = np.zeros(self.shape, bool)
        floor_hit[fi, fj] = True
        verified = self._crossed_columns(pts, np.asarray(origin, float), plane)
        self.occupied_t[hit] = np.maximum(self.occupied_t[hit], t)
        free_now = floor_hit & verified & ~hit  # a band point in the cell in this frame overrides its floor hits
        self.free_t[free_now] = np.maximum(self.free_t[free_now], t)
        self.frames.append((t, free_now))
        return Integration(True, plane, int(free_now.sum()), int(hit.sum()))

    def _crossed_columns(self, pts: np.ndarray, origin: np.ndarray, plane: Plane) -> np.ndarray:
        """Cells whose column from ``band_low_m`` to ``verify_high_m`` the frame's rays crossed in every height bin
        (samples every half cell along each ray, stopping a cell short of its return)."""
        cfg = self.cfg
        edges = np.arange(cfg.band_low_m, cfg.verify_high_m + 1e-9, cfg.verify_bin_m)
        nbins = max(1, len(edges) - 1)
        seen = np.zeros(self.shape + (nbins,), bool)
        vec = pts - origin
        length = np.hypot(vec[:, 0], vec[:, 1])
        keep = length > cfg.resolution_m
        vec, length = vec[keep], length[keep]
        step = 0.5 * cfg.resolution_m
        n = int(np.ceil(length.max() / step)) if len(length) else 0
        for k in range(1, n):
            frac = k * step / length
            live = frac < (length - cfg.resolution_m) / length
            if not live.any():
                break
            s = origin + vec[live] * frac[live, None]
            hz = s @ plane.normal + plane.offset_m
            b = np.floor((hz - cfg.band_low_m) / cfg.verify_bin_m).astype(int)
            inb = (b >= 0) & (b < nbins)
            ci, cj, inside = self._cells(s[inb, :2])
            seen[ci, cj, b[inb][inside]] = True
        return seen.all(axis=2)

    def mark_swept(self, footprint: Footprint, rear_pose: tuple, t: float) -> None:
        """The truck's footprint at ``rear_pose`` and time ``t`` (its own body stands there)."""
        self.swept.append((t, tuple(rear_pose), -footprint.rear_m, footprint.front_m, footprint.half_width_m))

    def mark_swept_box(self, rear_pose: tuple, x0: float, x1: float, half_width: float, t: float) -> None:
        """A rectangle in the truck's frame (x0..x1 along it, +-half_width across) declared clear at ``t`` -- the
        explicit start condition (plan S alternative 3), kept like the swept footprints."""
        self.swept.append((t, tuple(rear_pose), x0, x1, half_width))

    # --- use -----------------------------------------------------------------------------------------------------
    def usable(self, t: float, rear_pose=None) -> np.ndarray:
        """Camera cells usable at ``t``: each frame within the age limit keeps its own support (the cells it freed,
        less those a band point claimed after it), eroded by the error since that frame (at the cell's distance from
        the rear axle); the union over frames. A newer frame never cuts an older one's support short, and evidence of
        different frames is never merged before eroding (an older patch may have drifted away from a newer one)."""
        from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

        out = np.zeros(self.shape, bool)
        if rear_pose is None:
            rho = np.zeros(self.shape)
        else:
            X, Y = self.centres()
            rho = np.hypot(X - rear_pose[0], Y - rear_pose[1])
        for t_f, mask in self.frames:
            age = t - t_f
            if age > self.cfg.max_age_s:
                continue
            support = mask & ~(self.occupied_t > t_f)
            if not support.any():
                continue
            # outside the grid is not support (and the transform is undefined on an image with no background)
            dist = distance_transform_edt(np.pad(support, 1))[1:-1, 1:-1] * self.cfg.resolution_m
            out |= support & (dist >= erosion_m(self.cfg, age, rho))
        return out

    def _occupied_after(self, xy: np.ndarray, t0: float) -> np.ndarray:
        """Whether a band point claimed the cell of each point after ``t0`` (a swept area is no proof after that)."""
        ij = np.floor((xy - self.origin) / self.cfg.resolution_m).astype(int)
        inside = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        out = np.zeros(len(xy), bool)
        out[inside] = self.occupied_t[ij[inside, 0], ij[inside, 1]] > t0
        return out

    def squares_in_swept(self, corners: np.ndarray, t: float, rear_pose) -> np.ndarray:
        """Squares (N, 4 corners, xy) lying wholly inside one swept rectangle within the age limit, shrunk by the
        odometry error since its time (at each corner's distance from the current rear axle), none of whose cells -- those
        under its bounding box -- a band point claimed since."""
        n = len(corners)
        flat = corners.reshape(-1, 2)
        box = _bbox_corners(corners).reshape(-1, 2)
        rho = np.hypot(flat[:, 0] - rear_pose[0], flat[:, 1] - rear_pose[1])
        ok = np.zeros(n, bool)
        for t0, pose, x0, x1, hw in self.swept:
            age = t - t0
            if age > self.cfg.max_age_s:
                continue
            r = odometry_error_m(self.cfg, age, rho) - 1e-9  # rounding of the raster's edges
            lx, ly = _local(flat, pose)
            inside = (lx >= x0 + r) & (lx <= x1 - r) & (np.abs(ly) <= hw - r)
            ok |= inside.reshape(n, 4).all(axis=1) & ~self._occupied_after(box, t0).reshape(n, 4).any(axis=1)
        return ok

    def squares_in_cells(self, corners: np.ndarray, usable: np.ndarray) -> np.ndarray:
        """Squares (smaller than a cell, any rotation) whose every overlapped cell -- the cells under the corners of
        their bounding box -- is usable; a corner outside the grid has none."""
        flat = _bbox_corners(corners).reshape(-1, 2)
        ij = np.floor((flat - self.origin) / self.cfg.resolution_m).astype(int)
        inside = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        good = np.zeros(len(flat), bool)
        good[inside] = usable[ij[inside, 0], ij[inside, 1]]
        return good.reshape(-1, 4).all(axis=1)

    def prune(self, t: float) -> None:
        self.swept = [s for s in self.swept if t - s[0] <= self.cfg.max_age_s]
        self.frames = [f for f in self.frames if t - f[0] <= self.cfg.max_age_s]


def _bbox_corners(corners: np.ndarray) -> np.ndarray:
    """The four corners of each polygon's axis-aligned bounding box (N, 4, 2)."""
    lo, hi = corners.min(axis=1), corners.max(axis=1)
    return np.stack([lo, np.c_[hi[:, 0], lo[:, 1]], hi, np.c_[lo[:, 0], hi[:, 1]]], 1)


def _rect_corners(pose, x0: float, x1: float, hw: float) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + c * a - s * b, y + s * a + c * b) for a, b in ((x0, -hw), (x0, hw), (x1, hw), (x1, -hw))])


def path_poses(start_poses, segments, step_m: float = 0.05) -> list:
    """Rear-axle poses from ``start_poses[-1]`` along ``segments`` of (signed length m, curvature 1/m), every
    ``step_m``: a negative length is reverse along the same arc."""
    x, y, yaw = start_poses[-1]
    out = [(x, y, yaw)]
    for length, k in segments:
        n = max(1, int(math.ceil(abs(length) / step_m)))
        ds = length / n
        for _ in range(n):
            if abs(k) < 1e-12:
                x, y = x + ds * math.cos(yaw), y + ds * math.sin(yaw)
            else:
                dyaw = ds * k
                x += (math.sin(yaw + dyaw) - math.sin(yaw)) / k
                y += (math.cos(yaw) - math.cos(yaw + dyaw)) / k
                yaw += dyaw
            out.append((x, y, yaw))
    return out


def _hull(points: np.ndarray) -> np.ndarray:
    """The convex hull of a few points, counter-clockwise (monotone chain)."""
    pts = sorted(map(tuple, np.round(points, 12)))
    if len(pts) <= 2:
        return np.asarray(pts)

    def half(seq):
        out: list = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0.0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(reversed(pts))
    return np.asarray(lower[:-1] + upper[:-1])


def _sweep_pieces(poses, x0: float, x1: float, hw: float) -> list:
    """Convex pieces covering the footprint's continuous sweep between neighbouring poses: the hull of the two
    rectangles, grown by the largest sagitta of a body point's arc between them (zero on a straight segment, where the
    hull of two rectangles moved along their own axis is their union). Each piece is (hull vertices, growth)."""
    rmax = max(math.hypot(a, b) for a in (x0, x1) for b in (-hw, hw))
    out = []
    for p, q in zip(poses, poses[1:], strict=False):
        dth = abs(math.atan2(math.sin(q[2] - p[2]), math.cos(q[2] - p[2])))
        chord = math.hypot(q[0] - p[0], q[1] - p[1])
        grow = 0.0
        if dth > 1e-12:
            radius = chord / (2.0 * math.sin(dth / 2.0))  # of the rear axle's arc; zero for a turn on the spot
            grow = (radius + rmax) * (1.0 - math.cos(dth / 2.0))
        out.append((_hull(np.vstack([_rect_corners(p, x0, x1, hw), _rect_corners(q, x0, x1, hw)])), grow))
    return out


def _squares_touch(lo: np.ndarray, hi: np.ndarray, hull: np.ndarray, grow: float) -> np.ndarray:
    """Whether axis-aligned boxes (lo, hi corners) come within ``grow`` of a convex polygon -- strictly: a box that only
    touches the polygon's edge does not (separating-axis test over the box axes and the polygon's edge normals; with
    ``grow`` > 0 it may say yes near a corner where the rounded growth would not -- the safe side)."""
    eps = 1e-9
    ok = (lo[:, 0] < hull[:, 0].max() + grow - eps) & (hi[:, 0] > hull[:, 0].min() - grow + eps)
    ok &= (lo[:, 1] < hull[:, 1].max() + grow - eps) & (hi[:, 1] > hull[:, 1].min() - grow + eps)
    corners = np.stack([lo, np.c_[hi[:, 0], lo[:, 1]], hi, np.c_[lo[:, 0], hi[:, 1]]], 1)
    for a, b in zip(hull, np.roll(hull, -1, axis=0), strict=True):
        n = np.array([b[1] - a[1], a[0] - b[0]])  # outward for a counter-clockwise hull
        length = float(np.hypot(*n))
        if length < 1e-15:
            continue
        n /= length
        ok &= (corners @ n).min(axis=1) < float(a @ n) + grow - eps
    return ok


def envelope_admissible(layer: LowLayer, t: float, poses, footprint: Footprint, *, reverse: bool,
                        t_check: float | None = None) -> tuple:
    """Whether the footprint along ``poses`` (the path from the current pose to the stopping distance, rear axle)
    stays within what is known clear.

    The check runs on a raster of about ``fine_m`` laid in the current footprint's frame, its lines on the footprint's
    edges, so the footprint the truck stands in is exactly a block of pixels and is exempt. Every other pixel that
    reaches into the sweep between neighbouring poses (``_sweep_pieces``: a pixel only partly in it counts) must lie
    wholly inside one shrunk swept rectangle or (forward only) on usable camera cells -- the cells under its bounding
    box. Returns (True, []) or (False, a few refused pixels' centres in the odometry frame). ``t_check`` (default
    ``t``): the time the evidence must still hold -- the end of braking (plan S), no new observation assumed."""
    tc = t if t_check is None else t_check
    f = layer.cfg.fine_m
    x0, x1, hw = -footprint.rear_m, footprint.front_m, footprint.half_width_m
    path = list(poses)
    if len(path) < 2:
        return True, []
    p0 = path[0]
    c0, s0 = math.cos(p0[2]), math.sin(p0[2])

    def to_body(xy):
        d = xy - np.array(p0[:2])
        return np.stack([c0 * d[..., 0] + s0 * d[..., 1], -s0 * d[..., 0] + c0 * d[..., 1]], -1)

    def to_world(ab):
        return np.stack([p0[0] + c0 * ab[..., 0] - s0 * ab[..., 1], p0[1] + s0 * ab[..., 0] + c0 * ab[..., 1]], -1)

    pieces = [(to_body(h), g) for h, g in _sweep_pieces(path, x0, x1, hw)]
    allv = np.vstack([h for h, _ in pieces])
    gmax = max(g for _, g in pieces)
    nx, ny = max(1, round((x1 - x0) / f)), max(1, round(2 * hw / f))
    fx, fy = (x1 - x0) / nx, 2 * hw / ny  # pixel edges fall on the footprint's edges
    i_lo = int(math.floor((allv[:, 0].min() - gmax - x0) / fx)) - 1
    i_hi = int(math.ceil((allv[:, 0].max() + gmax - x0) / fx)) + 1
    j_lo = int(math.floor((allv[:, 1].min() - gmax + hw) / fy)) - 1
    j_hi = int(math.ceil((allv[:, 1].max() + gmax + hw) / fy)) + 1
    ii, jj = np.meshgrid(np.arange(i_lo, i_hi), np.arange(j_lo, j_hi), indexing="ij")
    ii, jj = ii.ravel(), jj.ravel()
    outside = (ii < 0) | (ii >= nx) | (jj < 0) | (jj >= ny)  # not a pixel of the footprint the truck stands in
    ii, jj = ii[outside], jj[outside]
    lo = np.stack([x0 + ii * fx, -hw + jj * fy], 1)
    hi = np.stack([x0 + (ii + 1) * fx, -hw + (jj + 1) * fy], 1)
    need = np.zeros(len(lo), bool)
    for hull, grow in pieces:
        need |= _squares_touch(lo, hi, hull, grow)
    if not need.any():
        return True, []
    lo, hi = lo[need], hi[need]
    corners = to_world(np.stack([lo, np.c_[hi[:, 0], lo[:, 1]], hi, np.c_[lo[:, 0], hi[:, 1]]], 1))
    ok = layer.squares_in_swept(corners, tc, p0)
    if not reverse:
        ok |= layer.squares_in_cells(corners, layer.usable(tc, p0))
    if ok.all():
        return True, []
    bad = to_world((lo[~ok] + hi[~ok]) / 2)
    return False, [tuple(p) for p in np.round(bad[:: max(1, len(bad) // 12)], 3).tolist()]
