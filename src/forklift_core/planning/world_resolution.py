"""Carry out fleet_traffic's TA/RA advice for one truck on a world job.

Plan: docs/plans/2026-10-10-fleet-ta-ra.md. ``resolve`` re-plans only the
stage the truck is on (the stages after it start from fixed poses and keep
their plans):

- TA: a travel stage again from where the truck stands, round the standing
  truck's body (``TrafficManager.body_keepout``).
- RA: a travel stage again round the higher-ranked truck's nearby remaining
  route (``TrafficManager.wall``); failing that, if the truck stands on that
  route, back along the part of this stage it has driven to the nearest place
  it may stop (``TrafficManager.back_to``) and plan the stage again from there,
  as one path with a gear change.
- Making room (plan 2026-10-11, (h)): a truck that its protective field has
  held for a while -- a person who cannot get out of its way -- backs straight
  away from it a little, comes back and drives its stage on (``make_room``;
  no planner call).

Every new path keeps clear of where the lower-ranked trucks may stop
(``TrafficManager.stop_keepout``), and a TA or RA detour that would touch the
other truck's claim -- so wait for it again -- is turned down
(``TrafficManager.path_conflicts``). The caller swaps the stage's path and
gives traffic control the new route (rank kept). Nothing here moves a truck.
Callers put the job's own slots that hold a pallet again in ``extra``
(``own_pallets_keepout``).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .fleet_traffic import (
    Advisory,
    TrafficManager,
    arc_lengths,
    oriented_rects,
    overlap_matrix,
    pose_at,
)
from .geometry import Pose2D, Rectangle
from .hybrid_astar import PlanResult
from .pallet_mission import _append_straight
from .world_gen import PALLET_KEEP_M, pallet_box
from .world_mission import STAGES, TRAVEL_STAGES, plan_stage

MIN_BACK_M = 0.5  # a back-off shorter than this does not clear anything


@dataclass(frozen=True)
class Resolution:
    how: str  # "detour", "retreat", "failed" or "skipped" (callers add "stale")
    path: PlanResult | None  # the stage's new path when how is detour or retreat
    plans: tuple = ()  # (what, status, wall_s) per planner call
    left_m: float | None = None  # what was left of the stage's old path
    new_m: float | None = None  # the new path's length
    note: str | None = None  # why a back-off was turned down, if it was

    def record(self) -> dict:
        """Event fields: how, the lengths when there is a new path, the note."""
        out = {"how": self.how}
        if self.path is not None:
            out.update(left_m=round(self.left_m, 1), new_m=round(self.new_m, 1))
        if self.note:
            out["note"] = self.note
        return out


def rectangles(rects: np.ndarray) -> list[Rectangle]:
    """(N, 6) centre/half-size/cos/sin rects as planner obstacles."""
    return [
        Rectangle(float(x), float(y), 2 * float(hl), 2 * float(hw), float(np.arctan2(s, c)))
        for x, y, hl, hw, c, s in np.asarray(rects, dtype=float).reshape(-1, 6)
    ]


def own_pallets_keepout(
    manager: TrafficManager, name: str, job, stage: str, occupied, geometry, keep_m: float
) -> list[Rectangle]:
    """Keep-out of name's own slots that hold a pallet again -- the source once
    lifted from (restocked), the sink once lowered on (its drop) -- in pieces
    clear of name's spot. A job's stage plans leave its own slots out of the
    keep-outs, so a route planned on the way would run over these. Small
    pieces: only a sliver by the truck goes, not the pallet's near side."""
    at = STAGES.index(stage) if stage in STAGES else -1
    boxes = [
        pallet_box(slot.site, geometry).grow(PALLET_KEEP_M)
        for slot, after in ((job.source, "to_drop"), (job.sink, "to_holding"))
        if at >= STAGES.index(after) and occupied[slot.name]
    ]
    rects = [[(b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, (b.x1 - b.x0) / 2, (b.y1 - b.y0) / 2, 1.0, 0.0] for b in boxes]
    return rectangles(manager.keepout(name, np.array(rects).reshape(-1, 6), keep_m, piece_m=0.15)) if rects else []


def reverse_path(result: PlanResult, from_m: float, to_m: float) -> PlanResult:
    """Drive result's path backwards from arc length from_m to to_m < from_m.

    The planner's curvature is the steering curvature (yaw change per signed
    distance), so the same curve driven the other way keeps it and flips the
    gear.
    """
    poses = np.asarray(result.poses, dtype=float)
    s = arc_lengths(poses)
    from_m = min(from_m, float(s[-1]))
    if not 0.0 <= to_m < from_m:
        raise ValueError("need 0 <= to_m < from_m <= path length")
    directions = np.asarray(result.directions)
    curvatures = np.asarray(result.curvatures_inv_m, dtype=float)
    inner = [i for i in range(len(s) - 1, -1, -1) if to_m < s[i] < from_m]
    out = [pose_at(poses, s, from_m)]
    gear, steer = [], []
    for i in inner:
        out.append(poses[i])
        # The segment arriving here runs back along the old one arriving at i + 1.
        gear.append(-int(directions[i + 1]))
        steer.append(float(curvatures[i + 1]))
    last = int(np.searchsorted(s, to_m, side="right"))  # old segment holding to_m arrives at last
    out.append(pose_at(poses, s, to_m))
    gear.append(-int(directions[min(last, len(s) - 1)]))
    steer.append(float(curvatures[min(last, len(s) - 1)]))
    gear.insert(0, gear[0])
    steer.insert(0, steer[0])
    return PlanResult(
        True,
        "success",
        np.asarray(out),
        np.asarray(gear, dtype=np.int8),
        np.asarray(steer),
        float(from_m - to_m),
        0,
    )


def resolve(
    advice: Advisory,
    manager: TrafficManager,
    *,
    stage: str,
    path: PlanResult,
    driven_m: float,
    stage_start_m: float,
    pose,
    job,
    world,
    config,
    travel_config,
    geometry,
    extra: list[Rectangle],
    wall_m: float,
    retreat_max_m: float,
    clock=None,
) -> Resolution:
    """Plan advice.truck's way out; see the module docstring.

    path is the stage's current path, driven_m how far along it the truck is,
    stage_start_m where the stage starts on the truck's traffic route, pose its
    rear-axle pose now. clock() (optional) times each planner call.
    """
    keep_m = config.clearance_m + 0.05
    start = Pose2D(*map(float, pose))
    plans = []
    left = float(arc_lengths(np.asarray(path.poses))[-1]) - driven_m
    # Every new path keeps clear of where the lower-ranked trucks may stop.
    stops = rectangles(manager.stop_keepout(advice.truck, keep_m))

    def found(how, result):
        length = float(arc_lengths(np.asarray(result.poses))[-1])
        return Resolution(how, result, tuple(plans), left, length)

    def attempt(what, start_pose, avoid, other=None):
        """The stage planned from start_pose round avoid, or None if it fails or
        breaks a traffic rule (path_conflicts, against other if given)."""
        t0 = clock() if clock else 0.0
        result = plan_stage(
            stage, start_pose, job, world, config, travel_config, geometry, extra + stops + avoid
        )
        status = result.status
        if result.success:
            why = manager.path_conflicts(advice.truck, result.poses, other)
            status = "rejected:" + "+".join(why) if why else status
        plans.append((what, status, round((clock() - t0) if clock else 0.0, 2)))
        return result if status == "success" else None

    if advice.kind == "ta":
        if stage not in TRAVEL_STAGES:
            return Resolution("skipped", None)
        avoid = rectangles(manager.body_keepout(advice.truck, advice.other, keep_m))
        result = attempt("ta", start, avoid, advice.other)
        return found("detour", result) if result else Resolution("failed", None, tuple(plans))
    if stage in TRAVEL_STAGES:
        avoid = rectangles(manager.wall(advice.truck, advice.other, wall_m, keep_m))
        result = attempt("ra_wall", start, avoid, advice.other)
        if result:
            return found("detour", result)
    if not manager.on_route(advice.truck, advice.other):
        return Resolution("failed", None, tuple(plans))
    here_m = manager.trucks[advice.truck].progress_m
    lowest = max(stage_start_m, here_m - min(driven_m, retreat_max_m))
    back = manager.back_to(advice.truck, lowest, here_m - MIN_BACK_M, other=advice.other)
    if back is None:
        return Resolution("failed", None, tuple(plans))
    retreat = reverse_path(path, driven_m, max(driven_m - (here_m - back), 0.0))
    if manager.path_conflicts(advice.truck, retreat.poses):
        return Resolution("failed", None, tuple(plans), note="back:covers_lower")
    end = retreat.poses[-1]
    # Not checked against other: from the back-off place the truck stands clear
    # of other's route, so if it waits for other there, other goes first.
    onward = attempt("ra_back", Pose2D(*map(float, end)), [])
    if onward is None:
        return Resolution("failed", None, tuple(plans))
    return found("retreat", _append_straight(retreat, onward))


def rest_of_path(result: PlanResult, from_m: float) -> PlanResult:
    """result's path from arc length from_m on (0 <= from_m < its length)."""
    poses = np.asarray(result.poses, dtype=float)
    s = arc_lengths(poses)
    if not 0.0 <= from_m < float(s[-1]):
        raise ValueError("need 0 <= from_m < path length")
    keep = [i for i in range(len(s)) if s[i] > from_m + 1e-9]
    directions = np.asarray(result.directions)
    curvatures = np.asarray(result.curvatures_inv_m, dtype=float)
    # The piece up to keep[0] lies on the old segment arriving at keep[0].
    return PlanResult(
        True,
        "success",
        np.vstack([pose_at(poses, s, from_m)] + [poses[i] for i in keep]),
        np.asarray([directions[keep[0]]] + [directions[i] for i in keep], dtype=np.int8),
        np.asarray([curvatures[keep[0]]] + [curvatures[i] for i in keep]),
        float(s[-1] - from_m),
        0,
    )


def make_room(
    manager: TrafficManager,
    name: str,
    path: PlanResult,
    driven_m: float,
    gear: int,
    back_m: float,
    static: np.ndarray,
    step_m: float = 0.5,
    clear_m: float = 0.05,
) -> Resolution:
    """A truck that its protective field holds (a person, say) backs straight
    away from it -- in the other gear, heading kept -- and comes the same way
    back to drive on along its stage: the stage's new path, or "failed" if no
    back-off of at least MIN_BACK_M keeps clear of every other truck's claim
    (traffic margin) and of ``static`` ((N, 6) rects: floor, pallets, spills,
    broken trucks; grown by clear_m). The longest back-off up to back_m in
    step_m steps is taken. gear is the stage's gear where the truck is."""
    poses = np.asarray(path.poses, dtype=float)
    here = pose_at(poses, arc_lengths(poses), driven_m)
    truck = manager.trucks[name]
    claims = np.vstack([np.zeros((0, 6)), *(t.claim for n, t in manager.trucks.items() if n != name)])
    static = np.asarray(static, dtype=float).reshape(-1, 6)
    away = -int(gear) * np.array([np.cos(here[2]), np.sin(here[2]), 0.0])
    back = back_m
    while back >= MIN_BACK_M - 1e-9:
        d = np.append(np.arange(0.0, back, 0.1), back)
        out = here + d[:, None] * away
        hit = overlap_matrix(manager._path_rects(name, out), claims).any() or overlap_matrix(
            oriented_rects(out, truck.footprint, clear_m), static
        ).any()
        if not hit:
            n = len(out)
            retreat = PlanResult(True, "success", out, np.full(n, -int(gear), dtype=np.int8), np.zeros(n), float(back), 0)
            again = PlanResult(True, "success", out[::-1].copy(), np.full(n, int(gear), dtype=np.int8), np.zeros(n), float(back), 0)
            new = _append_straight(_append_straight(retreat, again), rest_of_path(path, driven_m))
            left = float(arc_lengths(poses)[-1]) - driven_m
            return Resolution("back", new, (), left, float(new.length_m))
        back -= step_m
    return Resolution("failed", None, note="no_room")


def wait_stats(blocked: np.ndarray) -> dict:
    """Waits and waiting cycles (any length) in a (seconds, trucks) record of
    whom each truck waited for (-1: nobody) at 1 Hz, in seconds: one count
    for every version and simulator."""
    waits = []
    for k in range(blocked.shape[1]):
        start = None
        for i, b in enumerate(list(blocked[:, k]) + [-1]):
            if b >= 0 and start is None:
                start = i
            elif b < 0 and start is not None:
                waits.append(i - start)
                start = None
    cycles, alive = [], {}
    for i, row in enumerate(list(blocked) + [np.full(blocked.shape[1], -1)]):
        now = set()
        for k in range(len(row)):
            seen, j = [k], row[k]
            while j >= 0 and j not in seen:
                seen.append(j)
                j = row[j]
            if j >= 0:
                now.add(frozenset(seen[seen.index(j) :]))
        for c in [c for c in alive if c not in now]:
            cycles.append(i - alive.pop(c))
        for c in now:
            alive.setdefault(c, i)
    return {
        "over_60": sum(w >= 60 for w in waits),
        "over_300": sum(w >= 300 for w in waits),
        "max_s": max(waits, default=0),
        "cycles": len(cycles),
        "cycle_max_s": max(cycles, default=0),
    }


def resolution_counts(events) -> dict:
    """TA/RA outcomes (detour, retreat, failed, skipped) and CC count in events."""
    out = {"cc": sum(e["kind"] == "cc" for e in events)}
    for kind in ("ta", "ra"):
        hows = [e["how"] for e in events if e["kind"] == kind]
        out[kind] = {h: hows.count(h) for h in ("detour", "retreat", "failed", "skipped", "stale") if h in hows}
    return out
