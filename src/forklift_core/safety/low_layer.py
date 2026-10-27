"""The near-field low layer (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md section S).

The 2D LiDAR plane (1.05 m) does not see an empty pallet (0.144 m) or anything else below it. This layer keeps, in the
odometry frame, what the forward depth camera established about the floor near the truck:

- ``FREE``: a valid depth ray reached the verified floor inside the cell, the rays of the same frame crossed the cell's
  column at every height from ``band_low_m`` to ``verify_high_m`` (the camera cannot vouch for heights no ray passed
  through: a floor hit under a floating box is no proof), and no point of the collision band
  (``band_low_m``..``band_high_m`` above the floor) fell in it;
- ``OCCUPIED``: a band point fell in the cell;
- ``UNOBSERVED``: everything else -- dropouts, occlusions, the self mask, a frame whose floor plane failed.

The newest evidence wins. Evidence is only good for ``max_age_s`` and is eroded by the odometry error since it was seen
(the team-spec age table: position error plus heading error times the distance from the rear axle, the form the
obstacle layer's known-pallet growth uses), half a cell and the depth error. The truck's own swept footprint is kept the
same way. A path is admissible only when the footprint along it, up to the stopping distance, lies in usable cells:
free or swept going forward, swept only in reverse (the camera does not look back).

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
    max_age_s: float = 3.0  # evidence older than this is not used (plan S)
    depth_error_m: float = 0.02
    # the floor plane check: the plane must be where the floor is expected under the mount
    plane_height_tol_m: float = 0.05
    plane_normal_tol_rad: float = math.radians(5.0)
    plane_min_points: int = 400
    plane_max_rms_m: float = 0.01
    plane_iterations: int = 64
    plane_seed: int = 0
    age_table: tuple = field(default_factory=_age_table)


def erosion_m(cfg: LowLayerConfig, age_s: float, rho_m: float = 0.0) -> float:
    """How far in from the edge of usable evidence of ``age_s`` must a cell be: the age table's position and heading
    error (the first tabulated age >= ``age_s``; at ``rho_m`` from the rear axle) + half a cell + the depth error."""
    ages, pos, yaw = cfg.age_table
    i = min(bisect.bisect_left(ages, max(age_s, 0.0) - 1e-9), len(ages) - 1)
    return 0.5 * cfg.resolution_m + cfg.depth_error_m + pos[i] + 2.0 * rho_m * math.sin(yaw[i] / 2.0)


@dataclass(frozen=True)
class Plane:
    ok: bool
    normal: np.ndarray
    offset_m: float  # signed distance of the odometry origin's floor point: plane is normal . p + offset = 0
    rms_m: float
    inliers: int
    reason: str | None = None


def fit_floor(points: np.ndarray, cfg: LowLayerConfig, expected_z: float = 0.0) -> Plane:
    """The floor plane by RANSAC over points within ``plane_height_tol_m`` of the expected floor height, refused unless
    it is near that height, near horizontal, supported by ``plane_min_points`` and flat to ``plane_max_rms_m``."""
    none = Plane(False, np.array([0.0, 0.0, 1.0]), -expected_z, math.inf, 0)
    pts = np.asarray(points, float)
    near = pts[np.abs(pts[:, 2] - expected_z) <= cfg.plane_height_tol_m] if len(pts) else pts
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
    # refine by least squares on the inliers (z = a x + b y + c)
    A = np.c_[inl[:, 0], inl[:, 1], np.ones(len(inl))]
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


class LowLayer:
    """A fixed odometry-frame grid of ``size_cells`` (nx, ny) from ``origin_xy`` (the corner of cell (0, 0))."""

    def __init__(self, cfg: LowLayerConfig, origin_xy: tuple, size_cells: tuple):
        self.cfg = cfg
        self.origin = np.asarray(origin_xy, float)
        self.shape = tuple(int(v) for v in size_cells)
        self.free_t = np.full(self.shape, -np.inf)
        self.occupied_t = np.full(self.shape, -np.inf)
        self.swept_t = np.full(self.shape, -np.inf)

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
    def integrate(self, points: np.ndarray, t: float, *, origin, expected_floor_z: float = 0.0) -> Integration:
        """A frame's valid depth points (odometry frame) seen from the camera at ``origin``: the floor plane first (a
        refused plane marks nothing), then band points occupy their cells and floor hits free theirs where the frame's
        rays crossed the whole verified column."""
        plane = fit_floor(points, self.cfg, expected_floor_z)
        if not plane.ok:
            return Integration(False, plane, 0, 0)
        pts = np.asarray(points, float)
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
        self.occupied_t[hit] = t
        free_now = floor_hit & verified & ~hit  # a band point in the cell in this frame overrides its floor hits
        self.free_t[free_now] = t
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
        """The cells the truck's footprint covers at ``rear_pose`` (its own body stands there)."""
        mask = self._footprint_mask(footprint, [rear_pose])
        self.swept_t[mask] = t

    # --- use -----------------------------------------------------------------------------------------------------
    def _rho(self, rear_pose) -> np.ndarray:
        if rear_pose is None:
            return np.zeros(self.shape)
        X, Y = self.centres()
        return np.hypot(X - rear_pose[0], Y - rear_pose[1])

    def _usable(self, ok_t: np.ndarray, t: float, rear_pose) -> np.ndarray:
        """Cells whose evidence time ``ok_t`` is within the age limit and whose distance to the nearest cell without
        such evidence is at least the erosion for that age (and distance from the rear axle)."""
        from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

        age = t - ok_t
        good = age <= self.cfg.max_age_s
        if not good.any():
            return good
        dist = distance_transform_edt(good) * self.cfg.resolution_m  # to the nearest cell without good evidence
        rho = self._rho(rear_pose)
        ages, pos, yaw = self.cfg.age_table
        idx = np.clip(np.searchsorted(np.asarray(ages), np.clip(age, 0.0, None) - 1e-9), 0, len(ages) - 1)
        r = (0.5 * self.cfg.resolution_m + self.cfg.depth_error_m + np.asarray(pos)[idx]
             + 2.0 * rho * np.sin(np.asarray(yaw)[idx] / 2.0))
        return good & (dist >= r)

    def usable(self, t: float, rear_pose=None, *, reverse: bool = False) -> np.ndarray:
        """The cells a footprint may enter at ``t``: free or swept evidence (swept only in reverse), eroded."""
        free_now = (self.free_t > self.occupied_t)
        free_t = np.where(free_now, self.free_t, -np.inf)
        if reverse:
            evidence_t = self.swept_t
        else:
            # the union of the two kinds, each cell with its newest good evidence, never over an occupied cell
            evidence_t = np.where(self.occupied_t >= np.maximum(free_t, self.swept_t),
                                  np.where(self.swept_t > self.occupied_t, self.swept_t, -np.inf),
                                  np.maximum(free_t, self.swept_t))
        return self._usable(evidence_t, t, rear_pose)

    def _footprint_mask(self, footprint: Footprint, rear_poses) -> np.ndarray:
        X, Y = self.centres()
        half = 0.5 * self.cfg.resolution_m  # a cell counts when any part of it is under the footprint
        mask = np.zeros(self.shape, bool)
        for x, y, yaw in rear_poses:
            c, s = math.cos(yaw), math.sin(yaw)
            lx = c * (X - x) + s * (Y - y)
            ly = -s * (X - x) + c * (Y - y)
            mask |= ((lx >= -footprint.rear_m - half) & (lx <= footprint.front_m + half)
                     & (np.abs(ly) <= footprint.half_width_m + half))
        return mask


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


REACTION_S, DECEL_MPS2 = 0.15, 1.5  # control.drive_permission's stopping model


def braking_end_s(speed_mps: float) -> float:
    """How long after now the truck stands still if it brakes now (reaction, then the deceleration)."""
    return REACTION_S + abs(speed_mps) / DECEL_MPS2


def envelope_admissible(layer: LowLayer, t: float, poses, footprint: Footprint, *, reverse: bool,
                        t_check: float | None = None) -> tuple:
    """Whether the footprint along ``poses`` (the path from the current pose to the stopping distance, rear axle)
    lies in usable cells; the refused cells' centres (x, y) otherwise. The cells under the first pose are the truck's
    own and are allowed. ``t_check`` (default ``t``): the time the evidence must still hold -- the end of braking
    (plan S), with no new observation assumed in between."""
    need = layer._footprint_mask(footprint, poses)
    own = layer._footprint_mask(footprint, poses[:1])
    ok = layer.usable(t if t_check is None else t_check, poses[0], reverse=reverse)
    bad = need & ~ok & ~own
    if not bad.any():
        return True, []
    X, Y = layer.centres()
    return False, list(zip(X[bad].round(3).tolist(), Y[bad].round(3).tolist()))
