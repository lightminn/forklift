"""Stage-tagged props (docs/plans/2026-10-22-slot-anchored-neighbours.md 2.4): a session's delivered neighbour can be
two props -- ``@search`` at its estimate for every search and straight but the two inside the slot, ``@slot`` at its
slot (in the goal's frame) for the delivery straight and the withdraw only. Untagged props are in both."""

import pickle

import pytest

from forklift_core.planning import Bounds, Pose2D
from forklift_core.planning import pallet_mission as pm
from forklift_core.planning.geometry import Rectangle
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    make_scenario,
    plan_loaded_delivery,
    plan_transport,
    plan_transport_leg,
    scenario_props,
    site_poses,
)

ASSETS = (
    AssetSpec("test://barrel.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),
)


def prop(uri, x, y, length=0.3, width=0.3, yaw=0.0):
    return PlacedProp(AssetSpec(uri, length, width, 0.5), Rectangle(x, y, length, width, yaw))


def open_scene(*props):
    """Pickup ahead of the start, the destination far to the right, nothing else."""
    return TransportScenario(0, Pose2D(-3.0, 0.0, 0.0), PalletSite(1.0, 0.0, 0.0), PalletSite(8.0, 0.0, 0.0),
                             tuple(props), Bounds(-6.0, 11.0, -5.0, 5.0))


def test_untagged_props_are_the_same_list_for_both_stages():
    scenario = make_scenario(0, ASSETS)
    rects = [p.rectangle for p in scenario.props]
    assert scenario_props(scenario) == rects == scenario_props(scenario, "slot") == scenario_props(scenario, "search")
    with pytest.raises(ValueError):
        scenario_props(scenario, "approach")


def test_a_tagged_prop_goes_to_its_stage_only():
    plain, search, slot = prop("p", 0, 0), prop("session:pallet:P-5@search", 1, 0), prop("session:pallet:P-5@slot", 2, 0)
    scenario = open_scene(plain, search, slot)
    assert scenario_props(scenario, "search") == [plain.rectangle, search.rectangle]
    assert scenario_props(scenario, "slot") == [plain.rectangle, slot.rectangle]


def _beside_delivery(scenario, geometry, tag):
    """A box just beside the delivery straight's loaded outline near the slot (inside the clearance), far from where
    the transport search runs."""
    site = scenario.destination
    return prop(f"session:pallet:P-9{tag}", site.x_m + 0.2, site.y_m + geometry.loaded_footprint.half_width_m + 0.12)


@pytest.mark.parametrize("tag, ok", [("@slot", False), ("@search", True)])
def test_plan_transport_checks_the_delivery_straight_against_the_slot_stage(tag, ok):
    geometry = SyntheticMissionGeometry()
    scenario = open_scene()
    scenario = open_scene(_beside_delivery(scenario, geometry, tag))
    plan = plan_transport(scenario, geometry=geometry)
    assert plan.success is ok
    if not ok:
        assert plan.status == "transport:straight_collision"


def _spy(monkeypatch):
    calls = []
    real = pm._straight_plan

    def spy(start, goal, direction, obstacles, *a, **k):
        calls.append((start, goal, direction, list(obstacles)))
        return real(start, goal, direction, obstacles, *a, **k)

    monkeypatch.setattr(pm, "_straight_plan", spy)
    return calls


def _stage_of(call, scenario, geometry):
    start, goal, direction, _ = call
    d = site_poses(scenario.destination, geometry)
    if (start, goal) == (d["predelivery"], d["delivery"]):
        return "delivery"
    if (start, goal) == (d["delivery"], d["withdrawn"]):
        return "withdraw"
    return "other"


def _check_stages(calls, scenario, geometry, search, slot):
    seen = set()
    for call in calls:
        stage = _stage_of(call, scenario, geometry)
        seen.add(stage)
        obstacles = call[3]
        if stage in ("delivery", "withdraw"):
            assert slot.rectangle in obstacles and search.rectangle not in obstacles, stage
        else:
            assert slot.rectangle not in obstacles, stage
    return seen


def _far_pair():
    """A neighbour's two props far from every path: each stage's list shows which one it got."""
    return prop("session:pallet:P-8@search", -5.0, 4.0), prop("session:pallet:P-8@slot", 10.0, -4.0)


def test_plan_transport_gives_the_slot_stage_to_the_delivery_straight_and_the_withdraw_only(monkeypatch):
    geometry = SyntheticMissionGeometry()
    search, slot = _far_pair()
    scenario = open_scene(search, slot)
    calls = _spy(monkeypatch)
    plan = plan_transport(scenario, geometry=geometry, return_to=scenario.start_rear)
    assert plan.success
    assert {"delivery", "withdraw", "other"} <= _check_stages(calls, scenario, geometry, search, slot)


def test_plan_transport_leg_gives_the_slot_stage_to_its_tail(monkeypatch):
    geometry = SyntheticMissionGeometry()
    search, slot = _far_pair()
    scenario = open_scene(search, slot)
    calls = _spy(monkeypatch)
    start = site_poses(scenario.pickup, geometry)["extracted"]
    leg = plan_transport_leg(scenario, start, geometry=geometry)
    assert leg.success
    assert "delivery" in _check_stages(calls, scenario, geometry, search, slot)


def test_plan_loaded_delivery_gives_the_slot_stage_to_its_two_straights(monkeypatch):
    geometry = SyntheticMissionGeometry()
    search, slot = _far_pair()
    scenario = open_scene(search, slot)
    calls = _spy(monkeypatch)
    start = site_poses(scenario.pickup, geometry)["extracted"]
    plan = plan_loaded_delivery(scenario, start, geometry=geometry, return_to=scenario.start_rear)
    assert plan.success
    assert {"delivery", "withdraw"} <= _check_stages(calls, scenario, geometry, search, slot)


def test_the_searches_see_the_search_stage(monkeypatch):
    """A search-stage box on the transport search's way is avoided or refused; a slot-stage one there is not seen."""
    geometry = SyntheticMissionGeometry()
    seen = []
    real = pm._transport_search

    def spy(start, goal, obstacles, *a, **k):
        seen.append(list(obstacles))
        return real(start, goal, obstacles, *a, **k)

    monkeypatch.setattr(pm, "_transport_search", spy)
    search, slot = _far_pair()
    scenario = open_scene(search, slot)
    assert plan_transport(scenario, geometry=geometry).success
    assert seen and all(search.rectangle in o and slot.rectangle not in o for o in seen)
    kind_props, _, _ = pm.leg_obstacles("transport", scenario, geometry)
    assert search.rectangle in kind_props and slot.rectangle not in kind_props


def test_the_tags_survive_the_worker_pickle():
    geometry = SyntheticMissionGeometry()
    scenario = open_scene()
    scenario = open_scene(_beside_delivery(scenario, geometry, "@slot"))
    again = pickle.loads(pickle.dumps(scenario))
    assert scenario_props(again, "slot") == scenario_props(scenario, "slot")
    assert plan_transport(again, geometry=geometry).status == "transport:straight_collision"


def _random_m_third_mission(anchor):
    """random_m's third mission (P-3 -> D-3) with the delivered P-4 / P-5 as the failing run registered them
    (a770031 seed 0, rebuilt from its records: frame change o docking goal + 1.26 m -- the PR 3 validation and plan
    2026-10-22-slot-anchored-neighbours 1.2), P-5 anchored at D-2 or not."""
    from tools import riibotics_isaac_scenes as tool

    from forklift_core.mission.targets import Pose, TargetsBook
    from forklift_core.safety.pallet_permit import session_pallet_props

    rec = tool.session_record("random", markers=True)
    pallet = tool.load_pallet_geometry(tool.PALLETS[rec["pallet"]])
    geometry = tool.mission_geometry(tool.URDF, pallet, tool.RESERVE_M, **tool.STRAIGHTS)
    slots = {d["destination_id"]: d for d in rec["destinations"]}
    tb = TargetsBook()
    for p in rec["pallets"]:
        n = p["nominal"]
        tb.add_pallet(p["pallet_id"], "epal6", Pose(n["x_m"], n["y_m"], n["yaw_rad"]), depth_m=0.6, width_m=0.8)
    tb.put_down("P-4", Pose(3.3660, -7.2519, 0.0028), uncertainty_m=0.21)
    tb.put_down("P-5", Pose(3.2917, -8.6319, -0.0146), uncertainty_m=0.21)
    nominal = {p["pallet_id"]: tuple(p["nominal"][k] for k in ("x_m", "y_m", "yaw_rad")) for p in rec["pallets"]}
    anchors = {"P-5": tuple(slots["D-2"][k] for k in ("x_m", "y_m", "yaw_rad"))} if anchor else None
    others = session_pallet_props(tb, exclude_id="P-3", nominal=nominal, shape=(0.6, 0.8, 0.144), prior_grow_m=0.25,
                                  slot_anchors=anchors)
    static = tuple(PlacedProp(AssetSpec(**q["asset"]), Rectangle(**q["rectangle"])) for q in rec["static_props"])
    d3 = slots["D-3"]
    scenario = TransportScenario(0, Pose2D(**rec["start_rear"]), PalletSite(**rec["pallets"][2]["nominal"]),
                                 PalletSite(d3["x_m"], d3["y_m"], d3["yaw_rad"]), static + tuple(others),
                                 Bounds(**rec["bounds"]))
    return scenario, geometry, tool.CLEARANCE_M


@pytest.mark.parametrize("anchor, status", [(False, "straight_collision"), (True, "success")])
def test_random_m_third_missions_delivery_straight_with_and_without_the_anchor(anchor, status):
    scenario, geometry, clearance = _random_m_third_mission(anchor)
    d = site_poses(scenario.destination, geometry)
    straight = pm._straight_plan(d["predelivery"], d["delivery"], 1, scenario_props(scenario, "slot"),
                                 geometry.loaded_footprint, scenario.bounds, clearance)
    assert straight.status == status
    if anchor:  # the searches still see P-5 where the targets list has it
        assert any(r.y_m == pytest.approx(-8.6319) for r in scenario_props(scenario, "search"))
        assert not any(r.y_m == pytest.approx(-8.6319) for r in scenario_props(scenario, "slot"))


@pytest.mark.parametrize("tag, ok", [("@slot", False), ("@search", True)])
def test_the_leg_and_the_loaded_delivery_check_their_delivery_straight_against_the_slot_stage(tag, ok):
    """Codex plan round 3 P3: each planner, the other way too -- a slot-stage box beside the delivery straight refuses
    it, the same box at the search stage does not."""
    geometry = SyntheticMissionGeometry()
    scenario = open_scene(_beside_delivery(open_scene(), geometry, tag))
    start = site_poses(scenario.pickup, geometry)["extracted"]
    leg = plan_transport_leg(scenario, start, geometry=geometry)
    loaded = plan_loaded_delivery(scenario, start, geometry=geometry)
    assert leg.success is ok and loaded.success is ok
    if not ok:
        assert leg.status == "straight_collision" and loaded.status == "transport:straight_collision"


@pytest.mark.parametrize("tag, ok", [("@slot", False), ("@search", True)])
def test_the_workers_branches_after_the_payload_pickle(tag, ok):
    """The plan worker's three planning jobs on a pickled payload, as the runner submits them (the leg with its tight
    retry asked on the refusal it would retry)."""
    from forklift_core.planning import plan_worker as pw

    geometry = SyntheticMissionGeometry()
    scenario = open_scene(_beside_delivery(open_scene(), geometry, tag))
    start = site_poses(scenario.pickup, geometry)["extracted"]
    jobs = {
        "plan_transport": {"args": (scenario,), "kwargs": {"geometry": geometry, "trace": []}},
        "leg": {"kind": "transport", "args": (scenario, start), "kwargs": {"geometry": geometry},
                "retry_on": ("straight_collision",), "tight_args": (scenario, start), "tight_kwargs": {"geometry": geometry}},
        "loaded_delivery": {"args": (scenario, start), "kwargs": {"geometry": geometry},
                            "keep_m": geometry.delivery_straight_m},
    }
    for name, payload in jobs.items():
        out = pw._run(name, pickle.loads(pickle.dumps(payload)))
        assert out["result"].success is ok, name
        if name == "leg":
            assert out["tight"] is (not ok)  # asked again, refused again: the tight retry keeps the slot stage
