# src/forklift_core/perception/target_selection.py
"""Layers and target selection over PocketCandidates (design §7.3, §8, §10).

Only a fitted and confirmed candidate is selected automatically. With an
instruction (a map position of the pallet centre, a radius and optionally an
absolute layer), a unique counted candidate is not enough: any other candidate
in the radius sends the robot to look again, and two counted ones go to the
operator. Without one, any pallet will do, so ties are broken deterministically.
Selecting a candidate is not permission to insert: pickable None stays None
until the near-range re-observation (§9.1).

Stage 2A has no path planner, so the approach cost without an instruction is the
camera range of the candidate and reachability defaults to "all reachable"; the
planner's cost and reachability replace both when the executor calls this (§8-2).
"""

import math
from dataclasses import dataclass

import numpy as np

SELECTION_STATUSES = ("selected", "ask_operator", "reobserve", "none", "not_pickable")
ABSOLUTE_SUPPORT = ("visible", "coplanar")  # an upper layer's support edge must be seen, not guessed
# Any visible point of a pallet lies within its half-diagonal of the centre; the largest pallet
# the scenes allow is 1.2 x 1.2 m (S7, design §5.1), so 0.85 m covers every shape.
MAX_HALF_DIAGONAL_M = 0.85


@dataclass(frozen=True)
class Selection:
    status: str
    candidate_id: str | None
    reasons: tuple

    def __post_init__(self) -> None:
        if self.status not in SELECTION_STATUSES:
            raise ValueError(f"status must be one of {SELECTION_STATUSES}")
        object.__setattr__(self, "reasons", tuple(self.reasons))


def _height(c) -> float:
    spec = c.shape.spec
    return spec.overall_height_m if spec is not None else math.nan


def assign_layers(candidates, *, stack_xy_m=0.15, stack_yaw_rad=math.radians(10), height_tol_m=0.02) -> dict:
    """Candidate id -> (relative layer, absolute layer or None).

    Candidates whose pallet centres are within stack_xy_m and whose yaws agree form a
    stack, numbered by support height. A stack is absolute only if its bottom stands on a
    seen floor and every layer above rests on the one below (support height = lower support
    + lower pallet height) with its lower edge seen (visible or coplanar, never uncertain).
    """
    stacks = []
    for c in sorted(candidates, key=lambda c: c.support_z_m):
        for stack in stacks:
            ref = stack[0]
            dyaw = abs((c.insertion_yaw_rad - ref.insertion_yaw_rad + math.pi) % (2 * math.pi) - math.pi)
            if math.dist(c.pallet_centre_m[:2], ref.pallet_centre_m[:2]) <= stack_xy_m and dyaw <= stack_yaw_rad:
                stack.append(c)
                break
        else:
            stacks.append([c])
    out = {}
    for stack in stacks:
        absolute = stack[0].on_floor and all(
            upper.support_state in ABSOLUTE_SUPPORT
            and abs(upper.support_z_m - (lower.support_z_m + _height(lower))) <= height_tol_m
            for lower, upper in zip(stack, stack[1:]))
        for k, c in enumerate(stack):
            out[c.candidate_id] = (k, k if absolute else None)
    return out


def _decide(c) -> Selection:
    if c.pickable is False:
        return Selection("not_pickable", c.candidate_id, c.fit_reasons)
    return Selection("selected", c.candidate_id, ())


def _nearest(cands):
    return min(cands, key=lambda c: (round(c.range_m, 3), c.pallet_centre_m[1]))


def _could_be_inside(c, target, radius) -> bool:
    """A fitted, confirmed candidate is where its fit says. Any other candidate is uncertain: it is
    in the radius if its centre, a competing reading or -- conservatively, like an unfitted object --
    its visible points within the largest half-diagonal could be (Codex review A round 4)."""
    centres = [c.pallet_centre_m] + ([] if c.counted else list(c.alternative_centres_m))
    if any(np.linalg.norm(np.asarray(p[:2]) - target) <= radius for p in centres):
        return True
    if c.counted:
        return False
    if c.observed_centre_xy is None:
        return True
    return np.linalg.norm(np.asarray(c.observed_centre_xy) - target) - c.observed_extent_m <= radius + MAX_HALF_DIAGONAL_M


def select_target(candidates, instruction, *, reachable=None, unresolved=()) -> Selection:
    """unresolved: objects seen but not fitted (instance_fit.FitFailure). If such an object's
    centre could lie inside the instruction's radius -- its nearest visible point within radius +
    the largest half-diagonal, or its position unknown -- the instructed target may not be unique,
    so the robot looks again (Codex 2026-10-16). Uncertain candidates are treated the same way and
    the layer filter applies to confirmed fits only: an uncertain candidate can sit on any layer."""
    reachable = (lambda c: True) if reachable is None else reachable
    if instruction.get("present"):
        target = np.array([instruction["x_m"], instruction["y_m"]])
        radius = instruction["radius_m"]
        reach = radius + MAX_HALF_DIAGONAL_M
        if any(f.centre_xy is None or np.linalg.norm(np.asarray(f.centre_xy) - target) - f.extent_m <= reach
               for f in unresolved):
            return Selection("reobserve", None, ("unresolved_in_radius",))
        near = [c for c in candidates if _could_be_inside(c, target, radius)]
        uncertain = [c for c in near if not c.counted]
        counted = [c for c in near if c.counted]
        layer = instruction.get("layer")
        if layer is not None and counted:
            layers = assign_layers(candidates)
            if any(layers[c.candidate_id][1] is None for c in counted):
                return Selection("reobserve", None, ("layer_relative",))
            counted = [c for c in counted if layers[c.candidate_id][1] == layer]
        if len(counted) >= 2:
            return Selection("ask_operator", None, ("multiple_in_radius",))
        if uncertain:
            return Selection("reobserve", None, ("uncertain_in_radius",))
        if len(counted) == 1:
            return _decide(counted[0])
        return Selection("none", None, ("no_candidate_in_radius",))
    usable = [c for c in candidates if c.counted and reachable(c)]
    options = [c for c in usable if c.pickable is not False]
    if options:
        return _decide(_nearest(options))
    if usable:
        # Every identified pallet in reach fails the fork fit: report it, not "nothing there" (§10).
        return _decide(_nearest(usable))
    return Selection("none", None, ("no_candidate",))
