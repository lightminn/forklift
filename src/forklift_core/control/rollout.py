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

from dataclasses import dataclass, replace
from math import cos, sin

import numpy as np

from .path_tracking import RearAxlePathTracker, TrackerConfig


@dataclass(frozen=True)
class RolloutResult:
    status: str  # "arrived", "failed" or "timeout"
    position_error_m: float
    yaw_error_rad: float
    time_s: float
    # Rear-axle poses visited at every step, for a swept-footprint check (an
    # arrival can still cut a corner -- Codex v3.7 P1; sampling every 10th
    # step missed a 0.1 mm overlap three steps long -- Codex 5de6c3a P2).
    trajectory: tuple = ()


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
    # A rollout judges with the tracker's own cusp rule, without the caller's hold (fix A).
    tracker = RearAxlePathTracker(poses, directions, curvatures, replace(config, cusp_heading_pending_rad=None))
    pose = np.asarray(start_pose, dtype=float).copy()
    speed = 0.0
    steps = int(max_time_s / dt_s)
    visited = []
    for k in range(steps):
        visited.append(tuple(float(v) for v in pose))
        command = tracker.update(pose, speed, dt_s)
        if command.status in ("arrived", "failed"):
            visited.append(tuple(float(v) for v in pose))
            return RolloutResult(
                command.status,
                command.position_error_m,
                command.yaw_error_rad,
                k * dt_s,
                tuple(visited),
            )
        speed = command.speed_mps
        change = speed * command.curvature_inv_m * dt_s
        heading = pose[2] + change / 2
        pose[:2] += speed * dt_s * np.array([cos(heading), sin(heading)])
        pose[2] += change
    return RolloutResult("timeout", float("nan"), float("nan"), max_time_s, tuple(visited))


@dataclass(frozen=True)
class PrefixResult:
    """A dry run of the next leg's first part (cusp position stall, plan design v7)."""

    # "prefix" (went prefix_m), "leg_end" (the tracker accepted the leg end), "arrived", "failed",
    # "goal_stall" (standing at the final goal outside its tolerance: the runner's stall rule's state), "timeout"
    status: str
    progress_m: float  # tracker progress along the leg from its start
    lateral_m: float  # signed offset from the path at that progress (left positive)
    heading_rad: float  # yaw minus the path's interpolated yaw there
    failure: str | None
    leg_end_is_goal: bool
    trajectory: tuple = ()


GOAL_STALL_REMAINING_M = 1e-4  # a goal stall is judged only with this little of the leg left


def leg_prefix_rollout(
    poses,
    directions,
    curvatures,
    config: TrackerConfig,
    start_pose,
    *,
    start_leg: int,
    prefix_m: float,
    dt_s: float = 1.0 / 120.0,
) -> PrefixResult:
    """Drive leg ``start_leg`` of the whole path from ``start_pose`` on an ideal bicycle, as
    bicycle_rollout does, until it has gone ``prefix_m`` along that leg or the leg ends.

    The tracker is built on the whole path and started at that leg by its own leg switch, so a
    leg end is judged with its own cusp or goal tolerances and brake window (a slice would turn
    a cusp into a goal). The time budget is 3 x prefix / cruise + 10 s. The same call from the
    previous leg end's path pose gives the reference run to compare with at equal progress.
    """
    # A rollout judges with the tracker's own cusp rule, without the caller's hold (fix A).
    tracker = RearAxlePathTracker(poses, directions, curvatures, replace(config, cusp_heading_pending_rad=None))
    tracker.skip_legs(start_leg)
    path = np.asarray(poses, dtype=float)
    start = float(tracker._progress)
    leg_end = tracker._leg_ends[start_leg]
    goal = leg_end == len(path) - 1

    def offset_at(progress, at):
        # The signed lateral and heading offset of pose `at` from the path at progress.
        i = int(np.clip(np.searchsorted(tracker._distance, progress, side="right") - 1, 0, len(path) - 2))
        fraction = float(np.clip((progress - tracker._distance[i]) / tracker._lengths[i], 0.0, 1.0))
        point = path[i, :2] + fraction * (path[i + 1, :2] - path[i, :2])
        yaw0, yaw1 = path[i, 2], path[i + 1, 2]
        path_yaw = yaw0 + fraction * float(np.arctan2(np.sin(yaw1 - yaw0), np.cos(yaw1 - yaw0)))
        tangent = np.array([cos(path_yaw), sin(path_yaw)])
        d = at[:2] - point
        return (float(tangent[0] * d[1] - tangent[1] * d[0]),
                float(np.arctan2(np.sin(at[2] - path_yaw), np.cos(at[2] - path_yaw))))

    pose = np.asarray(start_pose, dtype=float).copy()
    speed = 0.0
    visited = [tuple(float(v) for v in pose)]
    status, failure = "timeout", None
    previous = None  # (progress, pose) at the last check
    # The budget at the leg's own speed cap: a reverse leg under max_reverse_speed_mps runs slower than the
    # cruise (Codex review of the cusp amendment).
    speed_cap = config.cruise_speed_mps
    if config.max_reverse_speed_mps is not None and int(np.asarray(directions)[min(leg_end, len(path) - 1)]) < 0:
        speed_cap = min(speed_cap, config.max_reverse_speed_mps)
    for _ in range(int((3.0 * prefix_m / speed_cap + 10.0) / dt_s)):
        command = tracker.update(pose, speed, dt_s)
        if command.status in ("arrived", "failed"):
            status, failure = command.status, command.failure
            break
        if tracker.leg_index != start_leg:
            status = "leg_end"
            break
        progress = float(tracker._progress)
        if progress - start >= prefix_m:
            # Both runs are compared at exactly prefix_m (Codex cusp implementation review P2): the
            # offsets of the two checked poses either side, interpolated in progress.
            target = start + prefix_m
            lateral, heading = offset_at(progress, pose)
            if previous is not None and progress > previous[0]:
                lateral0, heading0 = offset_at(previous[0], previous[1])
                w = (target - previous[0]) / (progress - previous[0])
                lateral = lateral0 + w * (lateral - lateral0)
                heading = heading0 + w * float(np.arctan2(np.sin(heading - heading0), np.cos(heading - heading0)))
            return PrefixResult("prefix", prefix_m, lateral, heading, None, goal, tuple(visited))
        previous = (progress, pose.copy())
        speed = command.speed_mps
        change = speed * command.curvature_inv_m * dt_s
        heading = pose[2] + change / 2
        pose[:2] += speed * dt_s * np.array([cos(heading), sin(heading)])
        pose[2] += change
        visited.append(tuple(float(v) for v in pose))
    if status == "timeout" and goal and command.status == "tracking":
        # The final goal missed in position: the whole budget spent inside the goal's brake window, never
        # arriving, farther across the goal than the position tolerance (it may still shrink while the
        # remaining length decays, so this is judged only at the end -- Codex review of the amendment). In
        # the runner the control pose's noise or a braking overshoot locks the remaining length at zero,
        # which its final-goal stall rule takes (cusp design amendment).
        # Only once the remaining length has run out (it bounds how far the offset across can still move),
        # never while the truck is still approaching (Codex review of the cusp amendment).
        goal_pose = path[leg_end]
        across = abs(-sin(goal_pose[2]) * (pose[0] - goal_pose[0]) + cos(goal_pose[2]) * (pose[1] - goal_pose[1]))
        remaining = tracker.remaining_to_goal_m()
        if remaining <= GOAL_STALL_REMAINING_M and across > config.position_tolerance_m + remaining:
            status = "goal_stall"
    progress = float(tracker._progress)
    lateral, heading_error = offset_at(progress, pose)
    return PrefixResult(status, progress - start, lateral, heading_error, failure, goal, tuple(visited))


__all__ = ["GOAL_STALL_REMAINING_M", "PrefixResult", "RolloutResult", "bicycle_rollout", "leg_prefix_rollout"]
