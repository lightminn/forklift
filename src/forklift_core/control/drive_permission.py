"""Drive permission from the obstacle grid (priority-5 plan D4).

The global plan may cross cells nobody has seen; the truck may not. Two
checks bound the speed, and the smaller wins:

* the emergency-stop arc (every control tick): an emergency stop zeroes the
  wheels and holds the steering, so the truck stops along the arc of its
  current curvature, not along the plan (Codex L0 P1). Every cell the
  footprint newly covers along that arc -- inflated by the measured stopping
  envelope, which grows from zero at the present pose -- must be FREE, and
  the speed must stop inside the verified part of the arc;
* the path ahead (each grid snapshot): how far along the active path the
  same holds, so the truck slows before a blocked stretch instead of only at
  the last moment.

Cells the truck's own body covers now are excluded (its body hides them);
nothing around the body is (Codex L0 P1: an envelope-grown exclusion let an
obstacle cell in). FREE evidence expires per cell: the oldest FREE cell a
check relied on must be no older than evidence_max_age_s at the tick, which
the grid sized its error radius for. No snapshot, an expired one, or any
obstacle sensor silent for sensor_timeout_s gives 0.
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
        return -a * t + sqrt((a * t) ** 2 + 2 * a * d)


@dataclass(frozen=True)
class PermissionConfig:
    stopping: StoppingModel
    envelope_offset_m: float
    evidence_max_age_s: float
    envelope_ramp_m: float = 0.2  # the envelope reaches its full width this far along the stop
    sensor_timeout_s: float = 0.25
    step_m: float = 0.05
    lookahead_m: float = 3.0
    end_creep_mps: float = 0.02


@dataclass
class Check:
    verified_m: float
    blocked: str | None  # "occupied" / "unknown" / "edge" / None
    oldest_free_s: float  # oldest FREE evidence the verified part relied on (inf: none needed)
    reached_end: bool


def footprint_cells(snapshot: GridSnapshot, pose, footprint: Footprint, margin_m: float):
    """(cells (k, 2), any outside the grid) whose square the inflated footprint at pose overlaps."""
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


def resample_path(poses: np.ndarray, step_m: float, limit_m: float):
    """Poses every step_m along a polyline up to limit_m, their arc length, and the full length."""
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
    return np.column_stack((x, y, yaw)), targets, float(s[-1])


def arc_poses(pose, curvature_inv_m: float, direction: int, length_m: float, step_m: float):
    """Rear-axle poses along the arc of fixed curvature (forward or reverse), with arc length."""
    x0, y0, yaw0 = pose
    s = np.arange(0.0, length_m + 1e-9, step_m)
    if not len(s) or s[-1] < length_m - 1e-9:
        s = np.append(s, length_m)
    d = direction * s
    k = curvature_inv_m
    if abs(k) < 1e-9:
        xs = x0 + d * cos(yaw0)
        ys = y0 + d * sin(yaw0)
        yaws = np.full(len(s), float(yaw0))
    else:
        yaws = yaw0 + d * k
        xs = x0 + (np.sin(yaws) - sin(yaw0)) / k
        ys = y0 + (cos(yaw0) - np.cos(yaws)) / k
    return np.column_stack((xs, ys, yaws)), s


class DrivePermission:
    def __init__(self, config: PermissionConfig):
        self.config = config
        self.snapshot: GridSnapshot | None = None
        self.path_check: Check | None = None
        self._driven_since_m = 0.0

    def _walk(self, snapshot, samples, arc, footprint, own_cells, *, full_path_m=None) -> Check:
        """Walk the samples; between two samples the midpoint is checked with half
        the interval's motion added (translation + farthest corner x rotation), so
        the continuous sweep is covered, not just the samples (Codex L0b P1)."""
        cfg = self.config
        verified, blocked, oldest = 0.0, None, np.inf
        radius = float(np.hypot(max(footprint.front_m, footprint.rear_m), footprint.half_width_m))
        checks = [(samples[0], float(arc[0]), 0.0)]
        for i in range(1, len(samples)):
            a, b = samples[i - 1], samples[i]
            dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
            motion = float(np.hypot(b[0] - a[0], b[1] - a[1])) + radius * abs(dyaw)
            mid = np.array([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2] + dyaw / 2])
            checks.append((mid, float(arc[i]), motion / 2))
        for pose, s, pad in checks:
            ramp = min(1.0, s / cfg.envelope_ramp_m) if cfg.envelope_ramp_m > 0 else 1.0
            cells, outside = footprint_cells(snapshot, pose, footprint, cfg.envelope_offset_m * ramp + pad)
            if outside:
                blocked = "edge"
                break
            if own_cells:
                keep = np.fromiter(((int(a), int(b)) not in own_cells for a, b in cells), bool, len(cells))
                cells = cells[keep]
            states = snapshot.state[cells[:, 0], cells[:, 1]]
            if (states == OCCUPIED).any():
                blocked = "occupied"
                break
            if (states != FREE).any():
                blocked = "unknown"
                break
            if len(cells):
                oldest = min(oldest, float(np.nanmin(snapshot.free_stamp[cells[:, 0], cells[:, 1]])))
            verified = float(s)
        reached_end = blocked is None and full_path_m is not None and arc[-1] >= full_path_m - 1e-9
        return Check(verified, blocked, oldest, reached_end)

    @staticmethod
    def _own_cells(snapshot, pose, own_footprint) -> set:
        """Cells whose whole square lies inside the truck's own outline (Codex L0b
        P1): a cell the body only partly covers can still hold an obstacle in its
        other part, so it is checked like any other."""
        cells, _ = footprint_cells(snapshot, pose, own_footprint, 0.0)
        if not len(cells):
            return set()
        x, y, yaw = pose
        c, s = cos(yaw), sin(yaw)
        res = snapshot.resolution_m
        out = set()
        for a, b in cells:
            x0 = snapshot.origin_x_m + a * res
            y0 = snapshot.origin_y_m + b * res
            inside = True
            for dx, dy in ((0, 0), (res, 0), (0, res), (res, res)):
                px, py = x0 + dx - x, y0 + dy - y
                u = px * c + py * s
                w = -px * s + py * c
                if not (-own_footprint.rear_m <= u <= own_footprint.front_m and abs(w) <= own_footprint.half_width_m):
                    inside = False
                    break
            if inside:
                out.add((int(a), int(b)))
        return out

    def update(self, snapshot: GridSnapshot, path_ahead, footprint: Footprint, own_footprint: Footprint, *, current_pose) -> Check:
        """New snapshot: the verified distance along path_ahead (rear-axle poses from the truck on)."""
        poses = np.asarray(path_ahead, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or not len(poses):
            raise ValueError("path_ahead must be a non-empty (N, 3) array")
        samples, arc, total = resample_path(poses, self.config.step_m, self.config.lookahead_m)
        own = self._own_cells(snapshot, current_pose, own_footprint)
        self.snapshot = snapshot
        self.path_check = self._walk(snapshot, samples, arc, footprint, own, full_path_m=total)
        self._driven_since_m = 0.0
        return self.path_check

    def advance(self, distance_m: float) -> None:
        """Path length driven since the last snapshot (both directions count)."""
        self._driven_since_m += abs(distance_m)

    def allowed_speed(
        self,
        now_s: float,
        sensors_last_s: dict,
        *,
        current_pose,
        curvature_inv_m: float,
        direction: int,
        footprint: Footprint,
        own_footprint: Footprint,
        speed_cap_mps: float,
    ) -> tuple[float, str]:
        """(speed limit, reason) for this control tick."""
        cfg = self.config
        snap, path = self.snapshot, self.path_check
        if snap is None or path is None:
            return 0.0, "no_snapshot"
        for name, last in sensors_last_s.items():
            if now_s - last > cfg.sensor_timeout_s:
                return 0.0, f"sensor_silent:{name}"
        # Emergency-stop arc at the present curvature and direction.
        length = cfg.stopping.distance_m(speed_cap_mps) + cfg.step_m
        samples, arc = arc_poses(current_pose, curvature_inv_m, 1 if direction >= 0 else -1, length, cfg.step_m)
        own = self._own_cells(snap, current_pose, own_footprint)
        estop = self._walk(snap, samples, arc, footprint, own)
        path_left = path.verified_m - self._driven_since_m
        if now_s - estop.oldest_free_s > cfg.evidence_max_age_s or now_s - path.oldest_free_s > cfg.evidence_max_age_s:
            return 0.0, "evidence_stale"
        arc_limit = cfg.stopping.speed_for(estop.verified_m)
        if path.reached_end:
            # Stopping at the goal needs no margin past it; the arc check
            # still covers the actual stopping volume beyond the end.
            path_limit = max(
                cfg.stopping.speed_for(path_left + cfg.stopping.margin_m),
                cfg.end_creep_mps if path_left > 0 else 0.0,
            )
        else:
            path_limit = cfg.stopping.speed_for(path_left)
        if arc_limit <= path_limit:
            return arc_limit, estop.blocked or ("ok" if arc_limit > 0 else "limit")
        return path_limit, path.blocked or ("ok" if path_limit > 0 else "limit")


__all__ = ["Check", "DrivePermission", "PermissionConfig", "StoppingModel", "arc_poses", "footprint_cells"]
