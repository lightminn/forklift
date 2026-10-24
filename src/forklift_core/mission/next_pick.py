"""The next pallet and destination for "move all remaining pallets" (CT-2, docs/plans/2026-10-24-ct2-next-pick.md D2).

Pure, given a ``PickSnapshot`` (picklable: the runner builds it at the request's tick and hands it to the planning
worker). The rule has no geometry of its own: a neighbour is the outline the mission's planner holds it as
(``session_pallet_props``, its stage tags kept), and a pair is open only when the planner's own five fixed straights
pass with the same poses, footprints, bounds and clearances (``_straight_plan``) -- the alignment straight, the
insertion and the extraction at the pallet, the delivery straight and the withdrawal at the slot. Collisions are judged
per obstacle (a clearance to each), so a straight blocked by a set of neighbours is blocked by one of them: the
relations are counted once, pair by pair, and the search only reads booleans.

The search tries every order of open pairs under ideal deliveries (a pair moves its pallet to the slot as a delivered
outline grown by ``RECOGNISED_MARGIN_M``) and keeps the one with the most missions, then the shortest summed estimate
(integer millimetres), then the lexicographically first sequence; the pick is its first pair. The estimate of a mission
is straight-line: start -> the pallet's alignment start, its extraction -> the slot's pre-delivery pose, the slot's
withdrawal -> home, where the first mission starts from the truck and every later one from home (the runner returns
home after each mission). It ranks; the mission's own preparation still plans every leg.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from forklift_core.mission.targets import Pallet, Pose, TargetsBook

REASONS = ("ok", "none_remaining", "no_free_destination", "all_blocked")  # and "error:<type>" from the runner


@dataclass(frozen=True)
class PickSnapshot:
    targets: TargetsBook
    truck: tuple | None  # rear axle (x, y, yaw) now
    home: tuple | None  # where a mission returns to; None: the truck's pose stands in (home_assumed)
    scenario: object  # TransportScenario: the static props and the bounds (pickup / destination unused)
    geometry: object  # SyntheticMissionGeometry
    clearance_m: float  # the planner config's clearance
    nominal: dict  # pallet id -> (x, y, yaw), a prior pallet's drawn slot
    shape: tuple  # (depth, width, height)
    prior_grow_m: float
    # destination id -> {pallet id: (x, y, yaw)}: the delivered pallets anchored to their slots, in that destination's
    # frame (what session_pallet_props gets as slot_anchors when the mission goes there)
    slot_anchors: dict = field(default_factory=dict)
    # pallet id -> Rectangle the alignment straight holds the mission's own pallet as (live: its zone while prior)
    pickup_obstacles: dict = field(default_factory=dict)


def _pose_of(snap: PickSnapshot, pallet: Pallet) -> tuple:
    if pallet.basis == "prior" and pallet.pallet_id in snap.nominal:
        return tuple(float(v) for v in snap.nominal[pallet.pallet_id])
    return (float(pallet.pose.x_m), float(pallet.pose.y_m), float(pallet.pose.yaw_rad))


def _props_by_pallet(props) -> dict:
    out: dict = {}
    for prop in props:
        name = prop.asset.uri.split("session:pallet:", 1)[1]
        out.setdefault(name.split("@", 1)[0], []).append(prop)
    return out


def recommend(snap: PickSnapshot, *, open_pairs: bool = False) -> dict:
    """The pick (module docstring). ``open_pairs``: also every pair open now, under ``"open"`` (CT-2 V2's
    criterion 4 compares them with the planner); not part of the pick message."""
    from forklift_core.planning.geometry import Rectangle
    from forklift_core.planning.pallet_mission import (
        AssetSpec, PalletSite, PlacedProp, _straight_plan, scenario_props, site_poses)
    from forklift_core.safety.pallet_permit import RECOGNISED_MARGIN_M, obb_overlap, preparation_obstacle, \
        session_pallet_props

    book, g = snap.targets, snap.geometry
    depth0, width0, height0 = (float(v) for v in snap.shape)
    pallets = book.pallets
    available = sorted(pid for pid, p in pallets.items() if p.state == "available")
    floor = sorted(pid for pid, p in pallets.items() if p.state != "carried" and p.pose is not None)
    candidates = [pid for pid in available if pallets[pid].pose is not None]
    blocked: dict = {pid: ["no_pose"] for pid in available if pallets[pid].pose is None}
    destinations = sorted(book.destinations)
    home_assumed = snap.home is None
    truck = snap.truck if snap.truck is not None else snap.home
    home = snap.home if snap.home is not None else truck

    def result(reason, plan=(), missions=0, total_mm=0):
        return {"pick": None if not plan else {"pallet_id": plan[0][0], "destination_id": plan[0][1]},
                "reason": reason, "plan": [list(x) for x in plan], "missions": missions,
                "estimate_total_m": total_mm / 1000.0, "home_assumed": home_assumed,
                "blocked": {k: v for k, v in sorted(blocked.items()) if v}}

    if not available:
        return result("none_remaining")
    if not destinations:
        return result("no_free_destination")

    base = snap.scenario
    bounds = base.bounds
    static = replace(base, props=tuple(base.props))
    unloaded, loaded = g.unloaded_footprint, g.loaded_footprint
    approach_clearance = min(float(snap.clearance_m), float(g.approach_gap_m) / 2)
    clearance = float(snap.clearance_m)
    grow = RECOGNISED_MARGIN_M

    def tagged(props, stage):
        return scenario_props(replace(static, props=tuple(props)), stage)

    def free(start, goal, footprint, rects, margin) -> bool:
        return bool(_straight_plan(start, goal, 1, rects, footprint, bounds, margin).success)

    # the neighbour props as the planner holds them: search-stage (estimates) once, slot-stage per destination
    search_props = _props_by_pallet(session_pallet_props(
        book, exclude_id=None, nominal=snap.nominal, shape=snap.shape, prior_grow_m=snap.prior_grow_m))
    slot_props = {d: _props_by_pallet(session_pallet_props(
        book, exclude_id=None, nominal=snap.nominal, shape=snap.shape, prior_grow_m=snap.prior_grow_m,
        slot_anchors=snap.slot_anchors.get(d))) for d in destinations}

    def virtual(e):  # a pallet put down at e, untagged (its estimate and its slot are the same place)
        dp = book.destinations[e]
        size = (depth0 + 2 * grow, width0 + 2 * grow)
        return [PlacedProp(AssetSpec(f"session:pallet:virtual:{e}", *size, height0),
                           Rectangle(float(dp.x_m), float(dp.y_m), *size, float(dp.yaw_rad)))]

    def pickup_straights(pid):
        x, y, yaw = _pose_of(snap, pallets[pid])
        s = site_poses(PalletSite(x, y, yaw), g)
        own = snap.pickup_obstacles.get(pid) or Rectangle(x, y, g.pallet_depth_m, g.pallet_width_m, yaw)
        return s, [(s["prealign"], s["approach"], unloaded, approach_clearance, [own]),
                   (s["approach"], s["inserted"], unloaded, clearance, []),
                   (s["inserted"], s["extracted"], loaded, clearance, [])]

    def delivery_straights(d):
        dp = book.destinations[d]
        s = site_poses(PalletSite(float(dp.x_m), float(dp.y_m), float(dp.yaw_rad)), g)
        return s, [(s["predelivery"], s["delivery"], loaded, clearance), (s["delivery"], s["withdrawn"], unloaded, clearance)]

    def blocks_pickup(pid, props) -> bool:
        _, straights = pickup_straights(pid)
        rects = tagged(props, "search")
        return any(not free(a, b, fp, rects, m) for a, b, fp, m, _ in straights)

    def blocks_delivery(d, props) -> bool:
        _, straights = delivery_straights(d)
        rects = tagged(props, "slot")
        return any(not free(a, b, fp, rects, m) for a, b, fp, m in straights)

    sites = {pid: pickup_straights(pid)[0] for pid in candidates}
    dsites = {d: delivery_straights(d)[0] for d in destinations}
    FREE = {}
    for pid in candidates:
        rects = scenario_props(static, "search")
        FREE[pid] = all(free(a, b, fp, rects + own, m) for a, b, fp, m, own in pickup_straights(pid)[1])
        if not FREE[pid]:
            blocked.setdefault(pid, []).append("props")
    FREED = {}
    for d in destinations:
        rects = scenario_props(static, "slot")
        FREED[d] = all(free(a, b, fp, rects, m) for a, b, fp, m in delivery_straights(d)[1])
        if not FREED[d]:
            blocked.setdefault(d, []).append("props")
    PB = {(p, q): blocks_pickup(p, search_props.get(q, [])) for p in candidates for q in floor if q != p}
    PBD = {(p, e): blocks_pickup(p, virtual(e)) for p in candidates for e in destinations}
    DB = {(d, q): blocks_delivery(d, slot_props[d].get(q, [])) for d in destinations for q in floor}
    DBD = {(d, e): blocks_delivery(d, virtual(e)) for d in destinations for e in destinations if e != d}

    # occupancy as the submit check judges it (TargetsBook._occupied), one other pallet at a time
    def size_of(pallet):
        return book._size(pallet)

    def overlaps(pid, d, other) -> bool | None:
        size, other_size = size_of(pallets[pid]), size_of(other)
        if size is None or other_size is None:
            return None
        rect = preparation_obstacle(other, None, size=other_size)
        dp = book.destinations[d]
        return bool(obb_overlap([[rect.x_m, rect.y_m]], (rect.length_m / 2, rect.width_m / 2), [rect.yaw_rad],
                                (dp.x_m, dp.y_m), (size[0] / 2, size[1] / 2), dp.yaw_rad)[0])

    def ghost(e):
        dp = book.destinations[e]
        return Pallet(f"virtual:{e}", pallets[candidates[0]].kind if candidates else "", Pose(dp.x_m, dp.y_m, dp.yaw_rad),
                      state="delivered", depth_m=depth0, width_m=width0, basis="delivered", uncertainty_m=grow, at=e)

    OCC = {(p, d, q): overlaps(p, d, pallets[q]) for p in candidates for d in destinations for q in floor if q != p}
    OCCD = {(p, d, e): overlaps(p, d, ghost(e)) for p in candidates for d in destinations for e in destinations if e != d}

    def dist(a, b) -> float:
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))

    def xy(pose) -> tuple:
        return (float(pose.x_m), float(pose.y_m))

    def estimate_mm(p, d, first: bool) -> int:
        start = truck if first else home
        return int(round(1000.0 * (dist(start, xy(sites[p]["prealign"])) + dist(xy(sites[p]["extracted"]),
                   xy(dsites[d]["predelivery"])) + dist(xy(dsites[d]["withdrawn"]), home))))

    def is_open(p, d, moved, filled) -> bool:
        if p in moved or d in filled or not FREE[p] or not FREED[d]:
            return False
        rest = [q for q in floor if q not in moved and q != p]
        if any(pallets[q].at == d for q in floor if q not in moved):
            return False
        for q in rest:
            if PB[(p, q)] or DB[(d, q)] or OCC[(p, d, q)] is not False:
                return False
        for e in filled:
            if PBD[(p, e)] or DBD[(d, e)] or OCCD[(p, d, e)] is not False:
                return False
        return True

    memo: dict = {}

    def best(moved: frozenset, filled: frozenset):
        key = (moved, filled)
        if key in memo:
            return memo[key]
        top = (0, 0, ())  # missions, total mm, sequence
        for p in candidates:
            for d in destinations:
                if not is_open(p, d, moved, filled):
                    continue
                n, total, seq = best(moved | {p}, filled | {d})
                cand = (n + 1, total + estimate_mm(p, d, not moved), ((p, d),) + seq)
                if (-cand[0], cand[1], cand[2]) < (-top[0], top[1], top[2]) or not top[2]:
                    top = cand
        memo[key] = top
        return top

    missions, total, plan = best(frozenset(), frozenset())
    # what blocks what at the start, for the screen
    for p in candidates:
        blocked.setdefault(p, []).extend(f"pickup:{q}" for q in floor if q != p and PB[(p, q)])
        if any(v is None for (p_, _, _), v in OCC.items() if p_ == p):
            blocked[p].append("no_size")
    for d in destinations:
        holders = [q for q in floor if pallets[q].at == d]
        blocked.setdefault(d, []).extend(f"occupied:{q}" for q in holders)
        blocked[d].extend(f"delivery:{q}" for q in floor if DB[(d, q)])
    if plan:
        out = result("ok", plan, missions, total)
    elif not any(not any(pallets[q].at == d for q in floor) for d in destinations):
        out = result("no_free_destination")
    else:
        out = result("all_blocked")
    if open_pairs:
        out["open"] = [[p, d] for p in candidates for d in destinations if is_open(p, d, frozenset(), frozenset())]
    return out
