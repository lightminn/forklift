"""C3: a second near observation on the approach straight (docs/plans/2026-10-27-insertion-lateral-drift.md, T2).

After the first near capture's straight is adopted the approach follower stops where the estimated fork tip is the
given gap before the entry face; there the pallet is captured again (the hold kept) and the straight re-drawn with the
first near capture's acceptance. Adoption is atomic: the candidate approach, the insertion and extraction moved with
the pallet, and the pickup replace the old ones together, or nothing changes and the truck resumes the first line.
"""

from __future__ import annotations

from dataclasses import replace
from math import cos, sin

import numpy as np

from forklift_core.localization.slam_pose import compose, invert

STATUSES = ("pending", "skipped_short", "adopted", "box_refused", "dry_run_refused", "no_detection")
BOX_LATERAL_M = 0.05  # the first near capture's projection box (core.near_leg_or_recapture)
BOX_MIN_LENGTH_M = 0.3
DRY_POSITION_M, DRY_YAW_RAD = 0.006, 0.015  # the first near capture's dry-run acceptance


def keep_m(gap_m: float, approach_gap_m: float) -> float:
    """How much of the final straight is left when the fork tip is ``gap_m`` before the face: the approach ends with
    the tip ``approach_gap_m`` before it."""
    return float(gap_m) - float(approach_gap_m)


def line_of(path, alignment_straight_m: float):
    """The final straight's line, (start, end) as (x, y, yaw) tuples: the path's last pose and the point
    ``alignment_straight_m`` behind it along its heading."""
    x, y, yaw = (float(v) for v in np.asarray(path.poses, dtype=float)[-1])
    return (x - alignment_straight_m * cos(yaw), y - alignment_straight_m * sin(yaw), yaw), (x, y, yaw)


def site_tuple(site) -> tuple[float, float, float]:
    return float(site.x_m), float(site.y_m), float(site.yaw_rad)


def moved_path(path, old_site, new_site):
    """``path`` carried rigidly from the old pallet site to the new one (insertion and extraction are straights
    defined relative to the site, so this is the path the new site gives)."""
    delta = compose(site_tuple(new_site), invert(site_tuple(old_site)))
    poses = np.array([compose(delta, p) for p in np.asarray(path.poses, dtype=float)], dtype=float)
    return replace(path, poses=poses)


def accepted(dry) -> bool:
    """The first near capture's dry-run acceptance."""
    return dry.status == "arrived" and dry.position_error_m <= DRY_POSITION_M and abs(dry.yaw_error_rad) <= DRY_YAW_RAD
