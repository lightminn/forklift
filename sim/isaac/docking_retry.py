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

from dataclasses import replace

import numpy as np
from forklift_core.localization.slam_pose import compose, invert


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
