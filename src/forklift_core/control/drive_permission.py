"""Drive permission from the obstacle grid (priority-5 plan D4).

The global plan may cross cells nobody has seen; the truck may not. Whenever
a grid snapshot arrives, ``evaluate`` walks the active path from the truck's
position and finds how far along it every cell the footprint (inflated by
the measured stopping-envelope offset) would newly cover is observed FREE in
that snapshot -- cells the truck already stands on are excluded, since its
own body hides them. That length is the verified distance.

Every control tick, independently of scan arrival, ``allowed_speed`` turns
the verified distance left (minus what the truck has driven since) into the
fastest speed that can still stop inside it:
    v * latency + v^2 / (2 decel) + margin <= distance left,
and returns 0 when the snapshot's evidence has aged past its limit or any
obstacle sensor has been silent too long. The stopping-distance model is the
P0a braking measurement; latency covers scan period, processing and the
command-to-deceleration delay.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, floor, sin, sqrt

import numpy as np

from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.planning.geometry import Footprint


@dataclass(frozen=True)
class StoppingModel:
    latency_s: float
    decel_mps2: float
    margin_m: float

    def distance_m(self, speed_mps: float) -> float:
        v = abs(speed_mps)
        return v * self.latency_s + v * v / (2 * self.decel_mps2) + self.margin_m

    def speed_for(self, distance_m: float) -> float:
        """Largest speed whose stopping distance fits in distance_m (0 if none)."""
        d = distance_m - self.margin_m
        if d <= 0:
            return 0.0
        a, t = self.decel_mps2, self.latency_s
        # v^2/(2a) + v t - d = 0
        return -a * t + sqrt((a * t) ** 2 + 2 * a * d)


@dataclass(frozen=True)
class PermissionConfig:
    stopping: StoppingModel
    envelope_offset_m: float
    evidence_max_age_s: float
    sensor_timeout_s: float = 0.25
    step_m: float = 0.05
    lookahead_m: float = 3.0
    creep_mps: float = 0.03


@dataclass
class Evaluation:
    stamp_s: float
    verified_m: float
    blocked: str | None  # "occupied" / "unknown" / None (verified to the lookahead or path end)
    path_end: bool
    newest_scan_s: float


def _footprint_cells(snapshot: GridSnapshot, pose, footprint: Footprint, margin_m: float) -> np.ndarray:
    """Indices (k, 2) of cells whose square the inflated footprint at pose overlaps."""
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    offset = (footprint.front_m - footprint.rear_m) / 2
    cx, cy = x + offset * c, y + offset * s
    hl = (footprint.front_m + footprint.rear_m) / 2 + margin_m
    hw = footprint.half_width_m + margin_m
    ex = abs(c) * hl + abs(s) * hw
    ey = abs(s) * hl + abs(c) * hw
    res = snapshot.resolution_m
    nx, ny = snapshot.state.shape
    i0 = int(floor((cx - ex - snapshot.origin_x_m) / res)) - 1
    i1 = int(ceil((cx + ex - snapshot.origin_x_m) / res)) + 1
    j0 = int(floor((cy - ey - snapshot.origin_y_m) / res)) - 1
    j1 = int(ceil((cy + ey - snapshot.origin_y_m) / res)) + 1
    ii, jj = np.meshgrid(np.arange(i0, i1), np.arange(j0, j1), indexing="ij")
    ii, jj = ii.ravel(), jj.ravel()
    sx = snapshot.origin_x_m + (ii + 0.5) * res
    sy = snapshot.origin_y_m + (jj + 0.5) * res
    half = res / 2
    dx, dy = sx - cx, sy - cy
    proj = half * (abs(c) + abs(s))
    hit = (
        (np.abs(dx) <= ex + half)
        & (np.abs(dy) <= ey + half)
        & (np.abs(dx * c + dy * s) <= hl + proj)
        & (np.abs(-dx * s + dy * c) <= hw + proj)
    )
    out = np.column_stack((ii[hit], jj[hit]))
    outside = (out[:, 0] < 0) | (out[:, 0] >= nx) | (out[:, 1] < 0) | (out[:, 1] >= ny)
    return out, bool(outside.any())


def _resample(poses: np.ndarray, step_m: float, limit_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Poses every step_m along a polyline up to limit_m, with their arc length."""
    xy = poses[:, :2]
    seg = np.hypot(*np.diff(xy, axis=0).T) if len(poses) > 1 else np.zeros(0)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = min(float(s[-1]), limit_m)
    targets = np.arange(0.0, total + 1e-9, step_m)
    if not len(targets) or targets[-1] < total - 1e-9:
        targets = np.append(targets, total)
    x = np.interp(targets, s, xy[:, 0])
    y = np.interp(targets, s, xy[:, 1])
    yaw = np.interp(targets, s, np.unwrap(poses[:, 2]))
    return np.column_stack((x, y, yaw)), targets


class DrivePermission:
    def __init__(self, config: PermissionConfig):
        self.config = config
        self.evaluation: Evaluation | None = None
        self._driven_since_m = 0.0

    def evaluate(
        self,
        snapshot: GridSnapshot,
        path_ahead: np.ndarray,
        footprint: Footprint,
        own_footprint: Footprint,
        *,
        current_pose,
    ) -> Evaluation:
        """Verified distance along path_ahead (rear-axle poses from the truck onwards).

        own_footprint is the part of the truck whose current cells are
        excluded (the body that hides them); the fork gap is not part of it.
        """
        cfg = self.config
        poses = np.asarray(path_ahead, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or not len(poses):
            raise ValueError("path_ahead must be a non-empty (N, 3) array")
        samples, arc = _resample(poses, cfg.step_m, cfg.lookahead_m)
        path_total = float(np.hypot(*np.diff(poses[:, :2], axis=0).T).sum()) if len(poses) > 1 else 0.0
        # The cells the truck already covers, with the same envelope the samples use.
        own, _ = _footprint_cells(snapshot, current_pose, own_footprint, cfg.envelope_offset_m)
        own_set = set(map(tuple, own))
        verified = 0.0
        blocked = None
        for pose, s in zip(samples, arc):
            cells, outside = _footprint_cells(snapshot, pose, footprint, cfg.envelope_offset_m)
            if outside:
                blocked = "unknown"
                break
            keep = np.array([tuple(c) not in own_set for c in cells], dtype=bool) if own_set else np.ones(len(cells), bool)
            states = snapshot.state[cells[keep, 0], cells[keep, 1]]
            if (states == OCCUPIED).any():
                blocked = "occupied"
                break
            if (states != FREE).any():
                blocked = "unknown"
                break
            verified = float(s)
        reached_end = blocked is None and arc[-1] >= path_total - 1e-9
        newest = max(snapshot.newest_scan_s.values()) if snapshot.newest_scan_s else -np.inf
        self.evaluation = Evaluation(snapshot.stamp_s, verified, blocked, reached_end, newest)
        self._driven_since_m = 0.0
        return self.evaluation

    def advance(self, distance_m: float) -> None:
        """Path length driven since the last evaluation (both directions count)."""
        self._driven_since_m += abs(distance_m)

    def allowed_speed(self, now_s: float, sensors_last_s: dict) -> tuple[float, str]:
        """(speed limit, reason) for this control tick."""
        cfg = self.config
        e = self.evaluation
        if e is None:
            return 0.0, "no_evaluation"
        for name, last in sensors_last_s.items():
            if now_s - last > cfg.sensor_timeout_s:
                return 0.0, f"sensor_silent:{name}"
        if now_s - e.newest_scan_s > cfg.evidence_max_age_s:
            return 0.0, "evidence_stale"
        left = e.verified_m - self._driven_since_m
        if e.path_end and e.blocked is None:
            # Verified to the end of the path: the tracker stops there itself.
            return float("inf"), "path_end"
        speed = cfg.stopping.speed_for(left)
        if speed < cfg.creep_mps:
            return 0.0, e.blocked or "limit"
        return speed, e.blocked or "lookahead"


__all__ = ["DrivePermission", "Evaluation", "PermissionConfig", "StoppingModel"]
