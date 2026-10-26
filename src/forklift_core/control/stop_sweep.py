"""The poses a stop sweeps from a pose: straight on and along the held steering's arc, at the walk's dense spacing.

docs/plans/2026-10-26-observe-start-backoff.md B4: a straight driven by its own tracker is checked against
rectangles the drive permission does not see (the props below the LiDAR plane, a pickup prior) at every tick -- the
stop from where the truck is now, with the steering it holds, must stay clear. ``dense_error_m`` is how much nearer
the continuous sweep can come between two of these samples; a check takes it as its margin."""

from __future__ import annotations

import math

from forklift_core.control.start_release import DENSE_STEP_M, DENSE_YAW_RAD, dense_error_m, shape_radius


def stop_poses(rear, direction: int, length_m: float, kappa: float) -> list:
    """The pose and the poses ahead of it for ``length_m`` in ``direction`` (+1 forward, -1 back), on the straight
    and, when ``kappa`` is not zero, on the arc of that curvature; spaced at most DENSE_STEP_M and DENSE_YAW_RAD."""
    x0, y0, yaw0 = (float(v) for v in rear)
    d = 1.0 if direction >= 0 else -1.0
    poses = [(x0, y0, yaw0)]
    for k in sorted({0.0, float(kappa)}):
        n = max(1, int(math.ceil(max(length_m / DENSE_STEP_M, abs(k) * length_m / DENSE_YAW_RAD))))
        for i in range(1, n + 1):
            s = d * length_m * i / n
            if abs(k) < 1e-9:
                poses.append((x0 + s * math.cos(yaw0), y0 + s * math.sin(yaw0), yaw0))
            else:
                yaw = yaw0 + k * s
                poses.append((x0 + (math.sin(yaw) - math.sin(yaw0)) / k, y0 - (math.cos(yaw) - math.cos(yaw0)) / k,
                              yaw))
    return poses


def sweep_margin_m(footprint) -> float:
    """The margin a check of ``stop_poses`` takes: the walk's bound on the continuous sweep between its samples."""
    return dense_error_m(shape_radius(footprint))


__all__ = ["stop_poses", "sweep_margin_m"]
