"""The live-default plan's one-hour order (docs/plans/2026-10-26-slam-localization-default.md step 3): one pallet carried
round the three destinations of the CT-2 scenes, every submit accepted by the targets list and every straight of every
leg (pickup alignment, insertion, extraction; delivery, withdrawal) clear of the other pallets and the scene's props --
the two-pallet order it replaced blocks line_m's re-pickup straight on its third leg (Codex round 2)."""

import pytest

from forklift_core.mission.targets import Pose, TargetsBook
from forklift_core.planning import Bounds, Pose2D
from forklift_core.planning import pallet_mission as pm
from forklift_core.planning.geometry import Rectangle
from forklift_core.planning.pallet_mission import AssetSpec, PalletSite, PlacedProp, site_poses
from forklift_core.safety.pallet_permit import session_pallet_props
from tools import live_rotation_order as order
from tools import riibotics_isaac_scenes as tool

SHAPE = (0.6, 0.8, 0.144)


def _scene(kind):
    rec = tool.session_record(kind, markers=True)
    pallet = tool.load_pallet_geometry(tool.PALLETS[rec["pallet"]])
    geometry = tool.mission_geometry(tool.URDF, pallet, tool.RESERVE_M, **tool.STRAIGHTS)
    static = [PlacedProp(AssetSpec(**q["asset"]), Rectangle(**q["rectangle"])) for q in rec["static_props"]]
    return rec, geometry, static


def _straights(pickup: PalletSite, destination: PalletSite, geometry):
    """(start, goal, direction, footprint) of a leg's straights, as the mission plans them."""
    p, d = site_poses(pickup, geometry), site_poses(destination, geometry)
    unloaded, loaded = geometry.unloaded_footprint, geometry.loaded_footprint
    return [(p["prealign"], p["approach"], 1, unloaded), (p["approach"], p["inserted"], 1, unloaded),
            (p["inserted"], p["extracted"], -1, loaded), (d["predelivery"], d["delivery"], 1, loaded),
            (d["delivery"], d["withdrawn"], -1, unloaded)]


def _first_blocked(kind, legs):
    """None when every submit is accepted and every straight is clear, else (leg, pallet, destination, why)."""
    rec, geometry, static = _scene(kind)
    slots = {d["destination_id"]: d for d in rec["destinations"]}
    book = TargetsBook()
    for p in rec["pallets"]:
        n = p["nominal"]
        book.add_pallet(p["pallet_id"], "epal6", Pose(n["x_m"], n["y_m"], n["yaw_rad"]), depth_m=0.6, width_m=0.8)
    for did, s in slots.items():
        book.add_destination(did, Pose(s["x_m"], s["y_m"], s["yaw_rad"]))
    nominal = {p["pallet_id"]: tuple(p["nominal"][k] for k in ("x_m", "y_m", "yaw_rad")) for p in rec["pallets"]}
    bounds = Bounds(**rec["bounds"])
    where = {p["pallet_id"]: PalletSite(**p["nominal"]) for p in rec["pallets"]}
    for i, (pid, did) in enumerate(legs):
        refusal = book.check_submit(pid, did, book.revision)
        if refusal is not None:
            return i, pid, did, refusal
        others = session_pallet_props(book, exclude_id=pid, nominal=nominal, shape=SHAPE, prior_grow_m=0.25)
        obstacles = [r.rectangle for r in static] + [r.rectangle for r in others]
        goal = PalletSite(slots[did]["x_m"], slots[did]["y_m"], slots[did]["yaw_rad"])
        for start, end, direction, footprint in _straights(where[pid], goal, geometry):
            plan = pm._straight_plan(start, end, direction, obstacles, footprint, bounds, tool.CLEARANCE_M)
            if not plan.success:
                return i, pid, did, plan.status
        book.put_down(pid, Pose(goal.x_m, goal.y_m, goal.yaw_rad), uncertainty_m=0.05)
        where[pid] = goal
    return None


@pytest.mark.parametrize("kind, pallet_id", [("line", "P-1"), ("random", "P-4")])
def test_the_rotation_keeps_every_submit_and_every_straight_clear(kind, pallet_id):
    assert _first_blocked(kind, order.rotation(pallet_id, ["D-1", "D-2", "D-3"], 24)) is None


def test_the_two_pallet_order_it_replaced_is_blocked_on_line_m():
    # control: the check sees the collision Codex found (P-2 at D-2 across the straight to P-1 at D-1)
    two = [("P-1", "D-1"), ("P-2", "D-2"), ("P-1", "D-3"), ("P-2", "D-1")]
    blocked = _first_blocked("line", two)
    assert blocked is not None and blocked[:3] == (2, "P-1", "D-3") and "collision" in blocked[3]


def test_the_order_never_asks_for_the_place_the_pallet_is_in():
    legs = order.rotation("P-1", ["D-1", "D-2", "D-3"], 7)
    assert legs[:4] == [("P-1", "D-1"), ("P-1", "D-2"), ("P-1", "D-3"), ("P-1", "D-1")] and len(legs) == 7
    assert all(a[1] != b[1] for a, b in zip(legs, legs[1:]))
    with pytest.raises(ValueError):
        order.rotation("P-1", ["D-1"], 3)
