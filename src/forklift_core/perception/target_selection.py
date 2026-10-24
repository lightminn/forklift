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
# The other bound of the same allowance: the shortest side of any pallet the scenes allow is the S7 minimum depth
# 0.50 m (widths 0.60-1.20, depths 0.50-1.20; the known EPAL 6 and T11 x 0.6 are larger), so 0.25 m is the smallest
# inscribed half-width. Used by the floor contract's occupancy exclusion (plan 2026-10-24-far-target-occupancy O1).
MIN_HALF_SHORT_SIDE_M = 0.25


@dataclass(frozen=True)
class Selection:
    status: str
    candidate_id: str | None
    reasons: tuple
    detail: dict | None = None  # the occupancy exclusion's inequality and what it excluded

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


def _segment_distance(p, a, b) -> float:
    """Horizontal distance from p to the segment a-b."""
    a, b = np.asarray(a[:2], float), np.asarray(b[:2], float)
    d = b - a
    t = 0.0 if not d.any() else float(np.clip((p - a) @ d / (d @ d), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * d)))


def _could_be_inside(c, target, radius) -> bool:
    """A fitted, confirmed candidate is where its fit says. Any other candidate is uncertain: it is
    in the radius if its centre, a competing reading or -- conservatively, like an unfitted object --
    its visible points within the largest half-diagonal could be (Codex review A round 4)."""
    # A counted candidate is judged on its own reading: its centre and, when the depth is unseen,
    # the depth segment. Competing readings (a rejected known shape among them) only make an
    # uncertain candidate possibly inside (Codex review 2026-10-16).
    centres = [c.pallet_centre_m] + ([] if c.counted else list(c.alternative_centres_m))
    if any(np.linalg.norm(np.asarray(p[:2]) - target) <= radius for p in centres):
        return True
    if c.centre_segment_m and _segment_distance(target, *c.centre_segment_m) <= radius:
        return True
    if c.counted:
        # A fit -- known or non-standard -- is where its centres say; the visible-points fallback
        # below is for uncertain candidates only (independent review 2026-10-16).
        return False
    if c.observed_centre_xy is None:
        return True
    return np.linalg.norm(np.asarray(c.observed_centre_xy) - target) - c.observed_extent_m <= radius + MAX_HALF_DIAGONAL_M


def check_occupancy_margin(margin_m) -> float:
    """A margin is finite and not negative: a negative one loosens the inequality and NaN passes its comparison
    (Codex implementation review 2026-10-24 P2)."""
    m = float(margin_m)
    if not math.isfinite(m) or m < 0:
        raise ValueError(f"occupancy margin must be finite and not negative, got {margin_m!r}")
    return m


def _unresolved_inside(f, target, radius) -> bool:
    """An unfitted object's centre could lie inside: its nearest visible point within radius + the largest
    half-diagonal, or its position unknown."""
    return f.centre_xy is None or np.linalg.norm(np.asarray(f.centre_xy) - target) - f.extent_m <= radius + MAX_HALF_DIAGONAL_M


def _occupied(candidates, instruction, unresolved, margin_m):
    """The floor contract's occupancy exclusion (plan 2026-10-24-far-target-occupancy O1), or None.

    Under the contract the run's pallets all stand on the floor without overlapping, and two rectangles overlap
    whenever their centres are nearer than the sum of their inscribed half-widths. So when exactly one confirmed fit
    H could be inside the radius, stands on the floor, is a known shape read without a competing centre or depth
    segment, and |c_H - t| + r + margin < h_H + MIN_HALF_SHORT_SIDE_M, no pallet can be centred in the radius beside
    it: whatever else could be inside (an uncertain candidate, an unfitted object) is not the instructed pallet.
    margin_m bounds H's centre error -- an empirical Isaac value, not a measured bound (the plan's residual risk).
    An instruction naming a layer keeps the layer path (Codex review 2026-10-24, 3rd edition)."""
    if instruction.get("layer") is not None:
        return None
    target = np.array([instruction["x_m"], instruction["y_m"]])
    radius = instruction["radius_m"]
    near = [c for c in candidates if _could_be_inside(c, target, radius)]
    counted = [c for c in near if c.counted]
    if len(counted) != 1:
        return None
    host = counted[0]
    spec = host.shape.spec
    if not host.on_floor or spec is None or host.alternative_centres_m or host.centre_segment_m:
        return None
    offset = float(np.linalg.norm(np.asarray(host.pallet_centre_m[:2]) - target))
    half = min(spec.overall_width_m, spec.overall_depth_m) / 2
    slack = half + MIN_HALF_SHORT_SIDE_M - (offset + radius + margin_m)
    if slack <= 0:
        return None
    decided = _decide(host)
    excluded = [c.candidate_id for c in near if c is not host] + [f.proposal_id for f in unresolved
                                                                    if _unresolved_inside(f, target, radius)]
    return Selection(decided.status, decided.candidate_id, decided.reasons + ("occupancy_excluded",),
                     {"excluded": excluded, "offset_m": offset, "half_short_side_m": half, "margin_m": margin_m,
                      "slack_m": slack})


def select_target(candidates, instruction, *, reachable=None, unresolved=(), occupancy_margin_m=None) -> Selection:
    """unresolved: objects seen but not fitted (instance_fit.FitFailure). If such an object's
    centre could lie inside the instruction's radius -- its nearest visible point within radius +
    the largest half-diagonal, or its position unknown -- the instructed target may not be unique,
    so the robot looks again (Codex 2026-10-16). Uncertain candidates are treated the same way and
    the layer filter applies to confirmed fits only: an uncertain candidate can sit on any layer.
    occupancy_margin_m: the floor contract's occupancy exclusion (_occupied) with this centre margin; None is off."""
    if occupancy_margin_m is not None:
        check_occupancy_margin(occupancy_margin_m)
    if instruction.get("present") and occupancy_margin_m is not None:
        first = select_target(candidates, instruction, reachable=reachable, unresolved=unresolved)
        if first.status != "reobserve":
            return first
        return _occupied(candidates, instruction, unresolved, float(occupancy_margin_m)) or first
    reachable = (lambda c: True) if reachable is None else reachable
    if instruction.get("present"):
        target = np.array([instruction["x_m"], instruction["y_m"]])
        radius = instruction["radius_m"]
        if any(_unresolved_inside(f, target, radius) for f in unresolved):
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
