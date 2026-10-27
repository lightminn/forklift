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
    sample_m: float = 0.01  # the envelope is checked at points this far apart
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
        """Camera cells usable at ``t``: free evidence of time tau within the age limit, inside the region of free
        evidence of that same time (one frame: one odometry error) eroded by the error after ``t - tau`` (at the cell's
        distance from the rear axle). Evidence of different times is never merged before eroding: an older patch may
        have drifted away from a newer neighbour and left a gap at the seam (S-P1 implementation review P1)."""
        from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

        e = np.where(self.free_t > self.occupied_t, self.free_t, -np.inf)
        good = (t - e) <= self.cfg.max_age_s
        out = np.zeros(self.shape, bool)
        if not good.any():
            return out
        if rear_pose is None:
            rho = np.zeros(self.shape)
        else:
            X, Y = self.centres()
            rho = np.hypot(X - rear_pose[0], Y - rear_pose[1])
        for tau in np.unique(e[good]):
            support = good & (e == tau)
            dist = distance_transform_edt(support) * self.cfg.resolution_m
            out |= good & (e == tau) & (dist >= erosion_m(self.cfg, t - tau, rho))
        return out

    def _occupied_after(self, xy: np.ndarray, t0: float) -> np.ndarray:
        """Whether a band point claimed the cell of each point after ``t0`` (a swept area is no proof after that)."""
        ij = np.floor((xy - self.origin) / self.cfg.resolution_m).astype(int)
        inside = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        out = np.zeros(len(xy), bool)
        out[inside] = self.occupied_t[ij[inside, 0], ij[inside, 1]] > t0
        return out

    def in_swept(self, xy: np.ndarray, t: float, rear_pose) -> np.ndarray:
        """Points inside some swept rectangle within the age limit, shrunk by the odometry error since its time (at the
        point's distance from the current rear axle), whose cell no band point claimed since."""
        ok = np.zeros(len(xy), bool)
        rho = np.hypot(xy[:, 0] - rear_pose[0], xy[:, 1] - rear_pose[1])
        for t0, pose, x0, x1, hw in self.swept:
            age = t - t0
            if age > self.cfg.max_age_s:
                continue
            r = odometry_error_m(self.cfg, age, rho)
            lx, ly = _local(xy, pose)
            inside = (lx >= x0 + r) & (lx <= x1 - r) & (np.abs(ly) <= hw - r)
            ok |= inside & ~self._occupied_after(xy, t0)
        return ok

    def prune(self, t: float) -> None:
        self.swept = [s for s in self.swept if t - s[0] <= self.cfg.max_age_s]


def footprint_points(footprint: Footprint, poses, step_m: float) -> np.ndarray:
    """Points every ``step_m`` covering the footprint rectangle at each pose (its boundary included)."""
    xs = np.linspace(-footprint.rear_m, footprint.front_m,
                     max(2, int(math.ceil((footprint.front_m + footprint.rear_m) / step_m)) + 1))
    ys = np.linspace(-footprint.half_width_m, footprint.half_width_m,
                     max(2, int(math.ceil(2 * footprint.half_width_m / step_m)) + 1))
    LX, LY = np.meshgrid(xs, ys)
    local = np.stack([LX.ravel(), LY.ravel()], 1)
    out = []
    for x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        out.append(np.stack([x + c * local[:, 0] - s * local[:, 1], y + s * local[:, 0] + c * local[:, 1]], 1))
    return np.vstack(out) if out else np.zeros((0, 2))


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


def envelope_admissible(layer: LowLayer, t: float, poses, footprint: Footprint, *, reverse: bool,
                        t_check: float | None = None) -> tuple:
    """Whether every point of the footprint along ``poses`` (the path from the current pose to the stopping distance,
    rear axle) outside the footprint at the current pose lies in a shrunk swept rectangle or (forward only) a usable
    camera cell; the refused points (x, y) otherwise, a few of them. A point outside the grid has no camera evidence.
    ``t_check`` (default ``t``): the time the evidence must still hold -- the end of braking (plan S), with no new
    observation assumed in between."""
    tc = t if t_check is None else t_check
    pts = footprint_points(footprint, list(poses[1:]) or list(poses), layer.cfg.sample_m)
    lx, ly = _local(pts, poses[0])
    own = (lx >= -footprint.rear_m) & (lx <= footprint.front_m) & (np.abs(ly) <= footprint.half_width_m)
    pts = pts[~own]
    if not len(pts):
        return True, []
    ok = layer.in_swept(pts, tc, poses[0])
    if not reverse:
        cells = layer.usable(tc, poses[0])
        i, j, inside = layer._cells(pts)
        cam = np.zeros(len(pts), bool)
        cam[np.flatnonzero(inside)] = cells[i, j]
        ok |= cam
    if ok.all():
        return True, []
    bad = pts[~ok]
    return False, [tuple(p) for p in np.round(bad[:: max(1, len(bad) // 12)], 3).tolist()]
