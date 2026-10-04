"""Destination scan docking (online SLAM plan v3.8).

Online SLAM + pre-scanned destination docking: at the start of the delivery
straight the truck matches its live (noisy) scan to a reference scan taken at
the delivery pose and re-places the delivery goal in its own estimate frame.
The reference is an idealised taught-station prior; this does not make the
SLAM map more accurate (Codex v3.8 P3).

Frames (Codex v3.8): Q = reference laser <- live laser (the ICP result),
X = rear <- laser (the mount seen from the rear axle), A = estimate <- live
rear at the scan instant, D = the planned delivery rear pose. Then
R = X o Q o X^-1 is reference rear <- live rear, and the delivery pose in the
estimate frame is goal = A o R^-1 -- no ground truth is read. The initial
guess is the planned relative pose D^-1 o A (about -0.7 m along), several
perturbed starts must agree (ICP can settle in a wrong minimum with good
residuals -- Codex v3.8 P2), and the correction relative to the plan is
bounded.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, sin

import numpy as np

from forklift_core.localization.scan_match import match_scans
from forklift_core.localization.slam_pose import compose, invert


@dataclass(frozen=True)
class DockingResult:
    accepted: bool
    reason: str
    goal_estimate: tuple | None  # delivery rear pose in the estimate frame
    relative_rear: tuple | None  # R: reference rear <- live rear
    correction: tuple | None  # planned -> matched relative pose
    starts: tuple  # per start: (accepted, reason, Q)


def laser_points(ranges, angles) -> np.ndarray:
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    ok = np.isfinite(ranges)
    return np.column_stack((ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])))


def ray_scan(segments, laser_pose, angles, range_max_m: float) -> np.ndarray:
    """Ranges from a laser at laser_pose to 2D line segments (tests, CPU checks)."""
    x, y, yaw = laser_pose
    out = np.full(len(angles), np.inf)
    a = np.asarray([s[0] for s in segments], dtype=float)
    b = np.asarray([s[1] for s in segments], dtype=float)
    for k, angle in enumerate(angles):
        d = np.array([cos(yaw + angle), sin(yaw + angle)])
        e = b - a
        denom = d[0] * e[:, 1] - d[1] * e[:, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            w = a - [x, y]
            t = (w[:, 0] * e[:, 1] - w[:, 1] * e[:, 0]) / denom
            u = (w[:, 0] * d[1] - w[:, 1] * d[0]) / denom
        hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1) & (t <= range_max_m)
        if hit.any():
            out[k] = t[hit].min()
    return out


def _close(p, q, metres: float, radians: float) -> bool:
    turn = atan2(sin(p[2] - q[2]), cos(p[2] - q[2]))
    return hypot(p[0] - q[0], p[1] - q[1]) <= metres and abs(turn) <= radians


def dock(
    reference_points,
    live_points,
    *,
    rear_from_laser,
    estimate_rear,
    delivery_rear,
    max_correction_m: float = 0.3,
    max_correction_rad: float = 0.15,
    agree_m: float = 0.01,
    agree_rad: float = 0.005,
    perturbations=((0.0, 0.0, 0.0), (0.15, 0.0, 0.0), (-0.15, 0.0, 0.0), (0.0, 0.15, 0.0),
                   (0.0, -0.15, 0.0), (0.0, 0.0, 0.05), (0.0, 0.0, -0.05)),
) -> DockingResult:
    x = tuple(float(v) for v in rear_from_laser)
    planned = compose(invert(delivery_rear), estimate_rear)  # R0
    q0 = compose(compose(invert(x), planned), x)
    starts, poses = [], []
    for delta in perturbations:
        result = match_scans(reference_points, live_points, initial=compose(q0, delta))
        starts.append((result.accepted, result.reason, result.pose))
        if result.accepted:
            poses.append(result.pose)
    if len(poses) < len(perturbations):
        bad = [s[1] for s in starts if not s[0]]
        return DockingResult(False, f"start_rejected:{bad[0]}", None, None, None, tuple(starts))
    if not all(_close(p, poses[0], agree_m, agree_rad) for p in poses[1:]):
        return DockingResult(False, "ambiguous", None, None, None, tuple(starts))
    q = poses[0]
    relative = compose(compose(x, q), invert(x))
    correction = compose(invert(planned), relative)
    if hypot(correction[0], correction[1]) > max_correction_m or abs(
        atan2(sin(correction[2]), cos(correction[2]))
    ) > max_correction_rad:
        return DockingResult(False, "correction_out_of_bounds", None, relative, correction, tuple(starts))
    goal = compose(estimate_rear, invert(relative))
    return DockingResult(True, "ok", tuple(goal), tuple(relative), tuple(correction), tuple(starts))


__all__ = ["DockingResult", "dock", "laser_points", "ray_scan"]
