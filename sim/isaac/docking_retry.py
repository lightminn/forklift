"""Plan D7c delivery docking retry: the frame bookkeeping, pure and testable without Isaac.

A docking match runs with the SLAM correction held, so the goal it commits is in that held
frame ("held"). When the hold is released -- a retry starting, or a path being replaced
during a re-approach -- the truck's estimate jumps by T = after o before^-1, and everything
expressed in the held frame (the goal, the withdraw path, the corrected destination) moves
by T once, after which the goal is a map goal ("map"). A map goal never moves again with a
release or a later correction change: those correct odometry drift, and following them would
cancel SLAM and stop the truck where odometry says the line is (Codex D7c reviews).
"""

from __future__ import annotations

from dataclasses import asdict, replace

import numpy as np
from forklift_core.control.rollout import bicycle_rollout
from forklift_core.localization.slam_pose import compose, invert
from forklift_core.planning.pallet_mission import final_straight_prefix


def line_start(goal, keep_m: float) -> tuple[float, float, float]:
    """The docking line's start keep_m behind the goal, on its axis."""
    return compose(tuple(goal), (-float(keep_m), 0.0, 0.0))


def carry_on_release(docking: dict, before, after, *, withdraw_poses=None, destination=None) -> dict:
    """Apply a release's frame change to a held goal and everything held with it.

    ``docking`` holds ``previous_goal``, ``goal_frame`` and optionally ``to_world`` (W:
    estimate -> world, which becomes W o T^-1). Returns the frame change and the moved
    withdraw poses / destination pose (unchanged when the goal was already a map goal).
    """
    frame_change = compose(tuple(after), invert(tuple(before)))
    out = {"frame_change": list(frame_change), "moved": False, "withdraw_poses": withdraw_poses,
           "destination": destination}
    if docking.get("goal_frame") != "held":
        return out
    docking["previous_goal"] = list(compose(frame_change, tuple(docking["previous_goal"])))
    if docking.get("to_world") is not None:
        docking["to_world"] = list(compose(tuple(docking["to_world"]), invert(frame_change)))
    if withdraw_poses is not None:
        out["withdraw_poses"] = np.array([compose(frame_change, tuple(pose)) for pose in withdraw_poses])
    if destination is not None:
        out["destination"] = compose(frame_change, tuple(destination))
    docking["goal_frame"] = "map"
    out["moved"] = True
    return out


def destination_with_prior_error(site, error_m: float):
    """The site moved error_m along its own lateral axis (plan D7c δ runs: the planning prior
    is wrong by δ while the reference scan and the evaluation keep the true site)."""
    x, y, yaw = compose((site.x_m, site.y_m, site.yaw_rad), (0.0, float(error_m), 0.0))
    return replace(site, x_m=x, y_m=y, yaw_rad=yaw)


def round_two_stop_check(straight, start, stop_config, keep_m: float, *, clear=None) -> tuple[bool, dict | None]:
    """The straight as it is driven after an accepted first docking round: cut keep_m before
    the goal (the round-2 stop) and judged at the stop's tolerance. Dry-run that path and
    sweep it with ``clear(pose)`` (Codex D7 Isaac review P1 and its re-review P1: the whole
    straight can pass both while the stop path fails either). (True, None) when the straight
    has no such stop (no round 2 is armed then)."""
    prefix = final_straight_prefix(straight, keep_m)
    if prefix is None:
        return True, None
    roll = bicycle_rollout(prefix.poses, prefix.directions, prefix.curvatures_inv_m, stop_config, start)
    swept = True if clear is None else all(clear(tuple(float(v) for v in pose)) for pose in roll.trajectory)
    record = {**{k: v for k, v in asdict(roll).items() if k != "trajectory"},
              "trajectory_samples": len(roll.trajectory), "swept_clear": swept, "length_m": float(prefix.length_m)}
    ok = (roll.status == "arrived" and roll.position_error_m <= stop_config.position_tolerance_m
          and abs(roll.yaw_error_rad) <= stop_config.yaw_tolerance_rad and swept)
    return ok, record
