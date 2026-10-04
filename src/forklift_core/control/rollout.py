"""Kinematic dry run of a path tracker (online SLAM plan v3.7).

Before docking from the near capture, the runner asks whether the approach
tracker, started from the truck's pose, can reach the re-drawn final straight's
goal inside its tolerances -- with margin -- on an ideal kinematic bicycle
(speed follows the command, curvature applied exactly). A truck that only just
converges in the ideal model will not in the simulator; such a start is
refused rather than driven (Codex v3.6 re-review P2: 3-5 cm and 0.04 rad left
too little straight to converge at 0.8 m).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin

import numpy as np

from .path_tracking import RearAxlePathTracker, TrackerConfig


@dataclass(frozen=True)
class RolloutResult:
    status: str  # "arrived", "failed" or "timeout"
    position_error_m: float
    yaw_error_rad: float
    time_s: float


def bicycle_rollout(
    poses,
    directions,
    curvatures,
    config: TrackerConfig,
    start_pose,
    *,
    dt_s: float = 1.0 / 120.0,
    max_time_s: float = 60.0,
) -> RolloutResult:
    tracker = RearAxlePathTracker(poses, directions, curvatures, config)
    pose = np.asarray(start_pose, dtype=float).copy()
    speed = 0.0
    steps = int(max_time_s / dt_s)
    for k in range(steps):
        command = tracker.update(pose, speed, dt_s)
        if command.status in ("arrived", "failed"):
            return RolloutResult(
                command.status, command.position_error_m, command.yaw_error_rad, k * dt_s
            )
        speed = command.speed_mps
        change = speed * command.curvature_inv_m * dt_s
        heading = pose[2] + change / 2
        pose[:2] += speed * dt_s * np.array([cos(heading), sin(heading)])
        pose[2] += change
    return RolloutResult("timeout", float("nan"), float("nan"), max_time_s)


__all__ = ["RolloutResult", "bicycle_rollout"]
