"""The return-start backoff (docs/plans/2026-10-27-return-start-withdraw-extension.md R3/R4): when the return search's
start -- the withdrawal end at the delivered pallet -- is inside the travel clearance, a straight reverse of length L is
put before the return. L comes from the first distance whose end the return's own check accepts while the straight to it
keeps the planner clearance from the slot-stage props; the recorded case is rh_5d280e80_off_random_m_s8 m3, whose
neighbour estimate (P-5@search) stood 0.24 m toward the slot."""

import dataclasses
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.planning import Footprint, PlannerConfig, Pose2D, Rectangle
from forklift_core.planning import pallet_mission as pm
from forklift_core.planning.hybrid_astar import PlanResult
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    Bounds,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    plan_loaded_delivery,
    plan_return_leg,
    return_start_backoff,
    site_poses,
)

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures/return_start_backoff_rh_s8_m3.json"
GEOMETRY = SyntheticMissionGeometry(unloaded_footprint=Footprint(1.29, 0.17, 0.36), alignment_straight_m=2.1,
                                    delivery_straight_m=1.5, withdrawal_m=0.55, carriage_limit_m=0.346,
                                    insertion_reserve_m=0.016)
CONFIG = PlannerConfig(curvature_limit_inv_m=0.29, xy_resolution_m=0.2, yaw_resolution_rad=math.radians(10),
                       primitive_length_m=0.25, collision_step_m=0.06, max_expansions=30000,
                       analytic_expansion_interval=8, heuristic_weight=1.8, reverse_penalty=1.3,
                       gear_change_penalty_m=1.0, steering_penalty=0.15, steering_change_penalty_m=0.15, clearance_m=0.1,
                       goal_connection="reeds_shepp", rs_min_segment_m=0.1, shot_cap_m=4.0, shot_cap_patience=2000)
CAP = pm.RETURN_START_BACKOFF_CAP_M


def recorded() -> TransportScenario:
    s = json.loads(FIXTURE.read_text())["mission_scenario"]
    return TransportScenario(
        seed=int(s["seed"]), start_rear=Pose2D(**s["start_rear"]), pickup=PalletSite(**s["pickup"]),
        destination=PalletSite(**s["destination"]),
        props=tuple(PlacedProp(AssetSpec(**p["asset"]), Rectangle(**p["rectangle"])) for p in s["props"]),
        bounds=Bounds(**s["bounds"]))


def moved(scenario, uri, **rect):
    """``scenario`` with the prop ``uri``'s rectangle changed."""
    props = tuple(PlacedProp(p.asset, replace(p.rectangle, **rect)) if p.asset.uri == uri else p for p in scenario.props)
    return replace(scenario, props=props)


def with_prop(scenario, uri, rect):
    return replace(scenario, props=scenario.props + (PlacedProp(AssetSpec(uri, rect.length_m, rect.width_m, 0.144), rect),))


def withdrawn(scenario):
    return site_poses(scenario.destination, GEOMETRY)["withdrawn"]


def back(pose, d):
    return Pose2D(pose.x_m - d * math.cos(pose.yaw_rad), pose.y_m - d * math.sin(pose.yaw_rad), pose.yaw_rad)


def travelled(poses):
    return np.r_[0.0, np.cumsum(np.hypot(*np.diff(np.asarray(poses)[:, :2], axis=0).T))]


def test_the_recorded_return_start_is_invalid_without_the_backoff():
    sc = recorded()
    plan = plan_return_leg(sc, withdrawn(sc), sc.start_rear, CONFIG, geometry=GEOMETRY)
    assert plan.status == "invalid_start"
    assert plan.start_backoff is None


def test_the_backoff_reverses_straight_then_returns_from_its_end():
    sc = recorded()
    start = withdrawn(sc)
    plan = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP)
    assert plan.success
    rec = plan.start_backoff
    assert rec["status"] == "prefix"
    assert rec["advice_m"] == pytest.approx(0.17) and rec["length_m"] == pytest.approx(0.20)
    s = travelled(plan.poses)
    inside = s <= rec["length_m"] + 1e-9
    assert np.all(plan.directions[inside] == -1) and np.all(plan.curvatures_inv_m[inside] == 0.0)
    np.testing.assert_allclose(plan.poses[0], [start.x_m, start.y_m, start.yaw_rad])
    end = back(start, rec["length_m"])
    rest = plan_return_leg(sc, end, sc.start_rear, CONFIG, geometry=GEOMETRY)
    assert rest.success
    assert plan.length_m == pytest.approx(rec["length_m"] + rest.length_m)
    np.testing.assert_allclose(plan.poses[-len(rest.poses) + 1:], rest.poses[1:])


def test_a_valid_start_plans_exactly_as_without_the_backoff():
    sc = moved(recorded(), "session:pallet:P-5@search", x_m=3.4, y_m=-8.469, yaw_rad=0.0)  # the estimate at its slot
    start = withdrawn(sc)
    off = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY)
    on = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP)
    assert off.success and on.success and on.start_backoff is None
    for name in ("poses", "directions", "curvatures_inv_m"):
        np.testing.assert_array_equal(getattr(on, name), getattr(off, name))


def test_no_clear_end_within_the_cap_keeps_the_invalid_start():
    sc = recorded()
    plan = plan_return_leg(sc, withdrawn(sc), sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=0.10)
    assert plan.status == "invalid_start"
    assert plan.start_backoff["status"] == "unmet:no_advice" and plan.start_backoff["length_m"] == 0.0


def test_a_backoff_cut_short_by_a_prop_behind_is_not_taken():
    sc = recorded()
    start = withdrawn(sc)
    # a low prop behind the truck: the straight keeps 0.10 from it up to 0.17 m back (the advice) and no further
    rear_edge = start.x_m - GEOMETRY.unloaded_footprint.rear_m - 0.175
    sc = with_prop(sc, "test:behind", Rectangle(rear_edge - 0.10 - 0.05, start.y_m, 0.10, 3.0, 0.0))
    plan = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP)
    assert plan.status == "invalid_start"
    assert plan.start_backoff["status"] == "unmet:clipped"
    assert plan.start_backoff["advice_m"] == pytest.approx(0.17)


def test_the_straight_is_checked_against_the_slot_stage_props():
    sc = recorded()
    start = withdrawn(sc)
    # a slot-stage prop right behind the start: the search never sees it, the straight does
    rear_edge = start.x_m - GEOMETRY.unloaded_footprint.rear_m
    sc = with_prop(sc, "session:pallet:P-9@slot", Rectangle(rear_edge - 0.05 - 0.05, start.y_m, 0.10, 1.0, 0.0))
    plan = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP)
    assert plan.status == "invalid_start" and plan.start_backoff["status"] == "unmet:no_advice"


def test_a_start_inside_the_grown_estimate_still_backs_off_on_the_slot_props():
    # the estimate 0.31 m toward the slot (agy review 5): the start overlaps it even at clearance 0
    sc = moved(recorded(), "session:pallet:P-5@search", x_m=3.3, y_m=-8.469 - 0.31, yaw_rad=0.015)
    start = withdrawn(sc)
    checker = pm.make_checker(pm.leg_obstacles("return", sc, GEOMETRY)[0], GEOMETRY.unloaded_footprint, sc.bounds, None)
    assert not checker.free((start.x_m, start.y_m, start.yaw_rad), 0.0)
    plan = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP)
    assert plan.success and plan.start_backoff["status"] == "prefix"


def test_a_failed_return_from_the_backoff_end_reports_its_status():
    sc = recorded()
    home = Pose2D(3.4, -8.469, 0.0)  # inside the P-5 estimate: no return can end there
    plan = plan_return_leg(sc, withdrawn(sc), home, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP)
    assert plan.status == "invalid_goal"
    assert plan.start_backoff["status"] == "unmet:search_invalid_goal"


def test_the_explicit_slot_props_replace_the_scenarios():
    sc = recorded()
    start = withdrawn(sc)
    blocking = [Rectangle(start.x_m - GEOMETRY.unloaded_footprint.rear_m - 0.10, start.y_m, 0.10, 1.0, 0.0)]
    plan = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP,
                           backoff_slot_props=blocking)
    assert plan.status == "invalid_start" and plan.start_backoff["status"] == "unmet:no_advice"


def test_the_backoff_goes_before_the_forward_turnaround():
    sc = recorded()
    start = withdrawn(sc)
    plan = plan_return_leg(sc, start, sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP,
                           forward_turnaround=True)
    assert plan.success and plan.start_backoff["status"] == "prefix"
    assert plan.turnaround is not None
    inside = travelled(plan.poses) <= plan.start_backoff["length_m"] + 1e-9
    assert np.all(plan.directions[inside] == -1)


def test_the_loaded_delivery_plan_takes_the_backoff_and_traces_it():
    sc = recorded()
    predelivery = site_poses(sc.destination, GEOMETRY)["predelivery"]
    start = Pose2D(predelivery.x_m - 1.0, predelivery.y_m, 0.0)
    trace_off, trace_on = [], []
    off = plan_loaded_delivery(sc, start, CONFIG, geometry=GEOMETRY, return_to=sc.start_rear, trace=trace_off)
    on = plan_loaded_delivery(sc, start, CONFIG, geometry=GEOMETRY, return_to=sc.start_rear, trace=trace_on,
                              return_start_backoff_m=CAP)
    assert off.status == "return_home:invalid_start"
    assert trace_off[-1]["stage"] == "return_home" and "start_backoff" not in trace_off[-1]
    assert on.success and on.return_home.start_backoff["status"] == "prefix"
    assert trace_on[-1]["stage"] == "return_home" and trace_on[-1]["start_backoff"]["status"] == "prefix"
    assert on.withdraw.length_m == pytest.approx(GEOMETRY.withdrawal_m)  # the withdrawal itself is left as it is
    np.testing.assert_allclose(on.withdraw.poses[-1], on.return_home.poses[0])


def test_a_failed_attempt_is_traced_too():
    sc = recorded()
    predelivery = site_poses(sc.destination, GEOMETRY)["predelivery"]
    start = Pose2D(predelivery.x_m - 1.0, predelivery.y_m, 0.0)
    trace = []
    plan = plan_loaded_delivery(sc, start, CONFIG, geometry=GEOMETRY, return_to=sc.start_rear, trace=trace,
                                return_start_backoff_m=0.10)
    assert plan.status == "return_home:invalid_start"
    assert trace[-1]["start_backoff"]["status"] == "unmet:no_advice"


def test_the_advice_helper_reports_the_first_and_last_clear_distance():
    sc = recorded()
    start = withdrawn(sc)
    obstacles, footprint, bounds = pm.leg_obstacles("return", sc, GEOMETRY)
    rec = return_start_backoff(start, obstacles, footprint, bounds, 0.10, pm.scenario_props(sc, "slot"), 0.10, CAP)
    assert rec["status"] == "prefix" and rec["advice_m"] == pytest.approx(0.17)
    assert rec["last_m"] == pytest.approx(CAP) and rec["length_m"] == pytest.approx(0.20)


def test_an_unused_backoff_field_is_left_out_of_records():
    field = {f.name: f for f in dataclasses.fields(PlanResult)}["start_backoff"]
    assert field.default is None and field.metadata.get("record_omit_default") is True
    from forklift_core.mission.replay_codec import recorded_fields

    plain = PlanResult(True, "success", np.zeros((1, 3)), np.zeros(1, dtype=np.int8), np.zeros(1), 0.0, 0)
    assert "start_backoff" not in [f.name for f in recorded_fields(plain)]
    assert "start_backoff" in [f.name for f in recorded_fields(replace(plain, start_backoff={"status": "prefix"}))]


# The follower over a backed-off return (plan C1 execution): the ideal kinematic bicycle the runner's dry runs use, the
# tracker at the runner's return lookahead.
def _follow(plan):
    from forklift_core.control.path_tracking import TrackerConfig
    from forklift_core.control.rollout import bicycle_rollout

    cfg = TrackerConfig(cruise_speed_mps=0.25, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
                        position_tolerance_m=0.03, yaw_tolerance_rad=0.03)
    return bicycle_rollout(plan.poses, plan.directions, plan.curvatures_inv_m, cfg, plan.poses[0], max_time_s=200.0)


def test_the_recorded_backed_off_return_is_followed_home_clear_of_the_slot_props():
    # the return goes on reversing past the backoff (same gear): the follower steers before L (lookahead), so the swept
    # outline is measured on the followed trajectory, not the straight
    from forklift_core.control.low_clearance import shape_clearance

    sc = recorded()
    plan = plan_return_leg(sc, withdrawn(sc), sc.start_rear, CONFIG, geometry=GEOMETRY, return_start_backoff_m=CAP)
    length = plan.start_backoff["length_m"]
    assert plan.directions[np.searchsorted(travelled(plan.poses), length + 1e-6)] == -1  # no cusp at the backoff's end
    run = _follow(plan)
    assert run.status == "arrived"
    trajectory = np.array(run.trajectory)
    near = trajectory[travelled(trajectory) <= length + 0.30]
    slot = pm.scenario_props(sc, "slot")
    assert min(shape_clearance(p, GEOMETRY.unloaded_footprint, slot) for p in near[::3]) >= CONFIG.clearance_m


def test_a_cusp_right_after_the_backoff_is_followed():
    # a synthetic join (the recorded row leaves no forward way out at L): 0.20 m straight back, then 1 m forward
    start = Pose2D(0.0, 0.0, 0.0)
    back_ = pm._straight_plan(start, back(start, 0.20), -1, [], GEOMETRY.unloaded_footprint,
                              Bounds(-5, 5, -5, 5), 0.10)
    ahead = pm._straight_plan(back(start, 0.20), Pose2D(0.8, 0.0, 0.0), 1, [], GEOMETRY.unloaded_footprint,
                              Bounds(-5, 5, -5, 5), 0.10)
    joined = pm._join_prefix(back_, ahead, {"status": "prefix", "length_m": 0.20}, "start_backoff")
    assert list(np.unique(joined.directions)) == [-1, 1] and joined.start_backoff["length_m"] == 0.20
    run = _follow(joined)
    assert run.status == "arrived" and run.position_error_m <= 0.03
    reached = min(p[0] for p in run.trajectory)
    assert -0.20 - 0.03 <= reached <= -0.20 + 0.03  # it stopped at the cusp before going forward
