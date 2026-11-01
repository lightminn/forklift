"""The return-start backoff widened to boxed starts, F2'' (docs/plans/2026-11-02-return-start-backoff-boxed.md): a start
every search attempt leaves after its root backs straight off too, and either trigger's distances must let the search
take a first step -- the search's own primitives, sweep margin and boxed-start config, not copies of them."""

import math
from dataclasses import replace
from math import ceil

import numpy as np
import pytest

from forklift_core.planning import Footprint, PlannerConfig, Pose2D, Rectangle
from forklift_core.planning import hybrid_astar as ha
from forklift_core.planning import pallet_mission as pm
from forklift_core.planning.grid_collision import make_checker
from forklift_core.planning.hybrid_astar import PlanResult, first_step_free, plan_hybrid_astar, sweep_margin_m
from forklift_core.planning.pallet_mission import Bounds, boxed_fallback_config, boxed_start, return_start_backoff

FP = Footprint(1.29, 0.17, 0.36)
CONFIG = PlannerConfig(curvature_limit_inv_m=0.29, xy_resolution_m=0.2, yaw_resolution_rad=math.radians(10),
                       primitive_length_m=0.25, collision_step_m=0.06, max_expansions=30000,
                       analytic_expansion_interval=8, heuristic_weight=1.8, reverse_penalty=1.3,
                       gear_change_penalty_m=1.0, steering_penalty=0.15, steering_change_penalty_m=0.15, clearance_m=0.1,
                       goal_connection="reeds_shepp", rs_min_segment_m=0.1, shot_cap_m=4.0, shot_cap_patience=2000)
BOUNDS = Bounds(-6, 6, -6, 6)
START = Pose2D(0.0, 0.0, 0.0)
GOAL = Pose2D(4.0, 5.0, math.pi)  # behind a turn: no root shot, the primitives decide
CAP = pm.RETURN_START_BACKOFF_CAP_M


def leg(gap, x0, x1):
    """A rack leg alongside the truck's right side, its edge ``gap`` past the footprint, from x0 to x1 (rear-axle frame):
    the release starts of rc_95e61a74 stood 0.093-0.116 m from such a leg."""
    return Rectangle((x0 + x1) / 2, -(FP.half_width_m + gap + 0.15), x1 - x0, 0.3, 0.0)


SIDE = [leg(0.1025, -0.4, 1.6)]  # alongside the whole truck: backing off does not open a first step
CORNER = [leg(0.1025, 1.35, 1.70)]  # by the front corner: 0.05 m back opens one (as rc_95e61a74 G s1 / s8)


def search(start, obstacles, config=CONFIG):
    return pm._search(start, GOAL, obstacles, FP, BOUNDS, config)


def backed_off(obstacles, *, boxed, slot_props=(), plan_from=None, direct=None):
    plan_from = plan_from or (lambda s: search(s, obstacles))
    direct = direct if direct is not None else plan_from(START)
    return pm._backed_off(START, direct, plan_from, obstacles, FP, BOUNDS, CONFIG.clearance_m, list(slot_props),
                          CONFIG.clearance_m, CAP, None, boxed_config=CONFIG if boxed else None)


class Recording:
    """A checker that records the margins it is asked at."""

    def __init__(self, inner):
        self.inner, self.radius_m, self.margins = inner, inner.radius_m, []

    def free(self, pose, margin_m=0.0):
        self.margins.append(margin_m)
        return True


def failed(status, attempts):
    return PlanResult(False, status, np.zeros((0, 3)), np.zeros(0, dtype=np.int8), np.zeros(0), 0.0,
                      attempts[-1][2] if attempts else 0, search_attempts=tuple(attempts))


def attempt(status, nodes):
    return (8, status, nodes, 0.2, math.radians(10), 0.06, 0.25, 30000, 0, 0)


# -- the search's own pieces, shared, not copied (D3)

def test_the_expansion_tries_the_six_primitives_in_their_old_order():
    assert ha.PRIMITIVES == ((1, 0), (1, -1), (1, 1), (-1, 0), (-1, -1), (-1, 1))


@pytest.mark.parametrize("step", [0.005, 0.01, 0.06])
@pytest.mark.parametrize("curvature", [0.0, 0.29])
def test_the_sweep_margin_is_the_one_the_motion_check_uses(step, curvature):
    config = replace(CONFIG, collision_step_m=step)
    checker = Recording(make_checker([], FP, BOUNDS, None))
    ha._sample_arc((0.0, 0.0, 0.0), config.primitive_length_m, 1, curvature, checker, config)
    n = ceil(config.primitive_length_m / step)
    ds = config.primitive_length_m / n
    assert checker.margins == [sweep_margin_m(config.clearance_m, ds, checker.radius_m, curvature)] * n + [
        config.clearance_m]
    assert sweep_margin_m(0.10, 0.01, 1.33929, 0.29) == pytest.approx(0.106942, abs=1e-6)  # plan §5


@pytest.mark.parametrize("step, expected", [(0.005, 0.005), (0.01, 0.01), (0.06, 0.01)])
def test_the_boxed_start_config_is_the_ladders_and_never_coarser(step, expected):
    config = replace(CONFIG, collision_step_m=step)
    assert boxed_fallback_config(config).collision_step_m == expected
    steps = [a[5] for a in search(START, SIDE, config).search_attempts]
    assert steps[0] == step and set(steps[1:]) <= {expected}  # the ladder retried at the same step


# -- first_step_free against a single attempt (D3: its preconditions)

def test_a_first_step_is_a_single_attempt_that_expands():
    for obstacles, start, want in ((SIDE, START, False), (CORNER, START, False),
                                   (CORNER, pm._reversed_by(START, 0.07), True)):
        checker = make_checker(obstacles, FP, BOUNDS, None)
        config = boxed_fallback_config(CONFIG)
        single = plan_hybrid_astar(start, GOAL, obstacles, FP, BOUNDS, config)
        assert single.root_shot != "taken"
        assert first_step_free((start.x_m, start.y_m, start.yaw_rad), checker, config) is want
        assert (single.expanded_nodes > 1) is want


def test_an_invalid_start_has_no_first_step():
    obstacles = [leg(0.09, -0.4, 1.6)]
    checker = make_checker(obstacles, FP, BOUNDS, None)
    assert plan_hybrid_astar(START, GOAL, obstacles, FP, BOUNDS, CONFIG).status == "invalid_start"
    assert not first_step_free((0.0, 0.0, 0.0), checker, CONFIG)


def test_a_root_shot_is_not_a_first_step_count():
    # the equivalence's exception: open ground and a near goal, the root's shot plans it at one node
    near = Pose2D(-1.5, 0.0, 0.0)
    single = plan_hybrid_astar(START, near, [], FP, BOUNDS, CONFIG)
    assert single.success and single.expanded_nodes == 1 and single.root_shot == "taken"
    assert first_step_free((0.0, 0.0, 0.0), make_checker([], FP, BOUNDS, None), CONFIG)


def test_the_heading_is_wrapped_as_the_search_wraps_it():
    checker = make_checker(CORNER, FP, BOUNDS, None)
    end = pm._reversed_by(START, 0.07)
    for yaw in (end.yaw_rad, end.yaw_rad + 2 * math.pi, end.yaw_rad - 4 * math.pi):
        assert first_step_free((end.x_m, end.y_m, yaw), checker, CONFIG)


# -- the trigger (D2)

def test_a_boxed_start_is_every_attempt_no_path_at_one_node():
    assert boxed_start(failed("no_path", [attempt("no_path", 1)] * 8))
    assert boxed_start(search(START, SIDE)) and boxed_start(search(START, CORNER))


@pytest.mark.parametrize("result", [
    failed("no_path", [attempt("no_path", 1), attempt("no_path", 17)]),  # a 0.10 m primitive attempt got out
    failed("no_path", [attempt("no_path", 1), attempt("no_path", 1), attempt("timeout", 0)]),  # a failed tail
    failed("no_path", [attempt("no_path", 1), attempt("no_path", 1), attempt("no_path:late", 1)]),  # a late tail
    failed("timeout", [attempt("no_path", 1), attempt("timeout", 0)]),
    failed("invalid_goal", [attempt("invalid_goal", 0)]),
    failed("invalid_start", [attempt("invalid_start", 0)]),
    failed("no_path", []),
])
def test_anything_else_is_not_a_boxed_start(result):
    assert not boxed_start(result)


def test_on_leaves_a_boxed_start_as_it_was():
    direct = search(START, CORNER)
    assert backed_off(CORNER, boxed=False, direct=direct) is direct


def test_boxed_backs_a_boxed_start_off_to_where_the_search_can_step():
    plan = backed_off(CORNER, boxed=True)
    assert plan.success
    rec = plan.start_backoff
    assert (rec["status"], rec["trigger"], rec["check"]) == ("prefix", "boxed", "first_step")
    assert (rec["advice_m"], rec["length_m"]) == (0.05, 0.08)
    # on's own check would have taken 0.04 m back (first clear 0.01 + 0.03), still boxed there
    on = return_start_backoff(START, CORNER, FP, BOUNDS, 0.1, [], 0.1, CAP)
    assert (on["status"], on["advice_m"], on["length_m"]) == ("prefix", 0.01, 0.04)
    assert boxed_start(search(pm._reversed_by(START, 0.04), CORNER))


# -- the truly boxed stay refused (D4)

def test_no_first_step_within_the_cap_is_no_advice():
    direct = search(START, SIDE)
    plan = backed_off(SIDE, boxed=True, direct=direct)
    assert plan.start_backoff["status"] == "unmet:no_advice" and plan.start_backoff["trigger"] == "boxed"
    assert (plan.status, plan.search_attempts) == (direct.status, direct.search_attempts)


def test_a_blocked_rear_is_no_advice():
    behind = Rectangle(-FP.rear_m - 0.1 - 0.05 - 0.15, 0.0, 0.3, 1.2, 0.0)  # 0.05 m past the clearance, behind
    direct = search(START, CORNER + [behind])
    assert boxed_start(direct)
    plan = backed_off(CORNER + [behind], boxed=True, slot_props=[behind], direct=direct)
    assert plan.start_backoff["status"] == "unmet:no_advice"
    assert (plan.status, plan.search_attempts) == (direct.status, direct.search_attempts)


def test_a_first_step_run_cut_at_its_first_distance_is_clipped():
    # a slot prop behind, its face at x -0.326: the straight keeps the clearance to 0.05 m back (the advice) only
    behind = Rectangle(-0.326 - 0.15, 0.0, 0.3, 1.2, 0.0)
    direct = search(START, CORNER)
    plan = backed_off(CORNER, boxed=True, slot_props=[behind], direct=direct)
    rec = plan.start_backoff
    assert (rec["status"], rec["advice_m"], rec["last_m"]) == ("unmet:clipped", 0.05, 0.05)
    assert (plan.status, plan.search_attempts) == (direct.status, direct.search_attempts)


# -- a failed return from the end keeps direct (D6)

@pytest.mark.parametrize("end_status", ["no_path", "timeout"])
def test_boxed_keeps_the_invalid_start_when_the_return_from_the_end_fails(end_status):
    obstacles = [leg(0.09, 1.35, 1.70)]
    direct = search(START, obstacles)
    assert direct.status == "invalid_start"
    end_failed = failed(end_status, [attempt(end_status, 3)])
    plan_from = lambda s: direct if s == START else end_failed
    boxed = backed_off(obstacles, boxed=True, plan_from=plan_from, direct=direct)
    assert boxed.status == "invalid_start" and boxed.start_backoff["status"] == f"unmet:search_{end_status}"
    assert boxed.start_backoff["trigger"] == "invalid_start"
    assert boxed.start_backoff["end_search_attempts"] == [list(a) for a in end_failed.search_attempts]
    on = backed_off(obstacles, boxed=False, plan_from=plan_from, direct=direct)  # F2' as it was: the end's failure
    assert on.status == end_status and "end_search_attempts" not in on.start_backoff and "trigger" not in on.start_backoff


def test_on_records_no_new_fields():
    obstacles = [leg(0.09, 1.35, 1.70)]
    plan = backed_off(obstacles, boxed=False)
    assert plan.success and not {"trigger", "check"} & set(plan.start_backoff)


# -- the recorded release replans (plan §4 recall; rc_95e61a74 G s1 boxed, GN s10 invalid start)

import dataclasses
import json
from pathlib import Path

from forklift_core.planning.pallet_mission import (
    AssetSpec,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    plan_return_leg,
)

RECORDED = Path(__file__).resolve().parents[2] / "fixtures/return_start_backoff_boxed_rc.json"


def _init(cls, d):
    return cls(**{f.name: d[f.name] for f in dataclasses.fields(cls) if f.init and f.name in d})


def case(name):
    c = json.loads(RECORDED.read_text())["cases"][name]
    s = c["scenario"]
    scenario = TransportScenario(
        seed=int(s["seed"]), start_rear=Pose2D(**s["start_rear"]), pickup=PalletSite(**s["pickup"]),
        destination=PalletSite(**s["destination"]),
        props=tuple(PlacedProp(AssetSpec(**p["asset"]), Rectangle(**p["rectangle"])) for p in s["props"]),
        bounds=Bounds(**s["bounds"]))
    g = dict(c["geometry"])
    g["unloaded_footprint"] = Footprint(**g["unloaded_footprint"])
    if isinstance(g.get("loaded_footprint"), dict):
        g["loaded_footprint"] = Footprint(**g["loaded_footprint"])
    return (c["recorded"], scenario, Pose2D(**c["start"]), Pose2D(**c["return_to"]), _init(PlannerConfig, c["config"]),
            _init(SyntheticMissionGeometry, g))


@pytest.mark.parametrize("name, recorded_status, on_ok", [("rc_G_aisle_rack_s1", "no_path", False),
                                                         ("rc_GN_aisle_rack_s10", "invalid_start", True)])
def test_the_recorded_release_replans_return_with_boxed(name, recorded_status, on_ok):
    status, scenario, start, home, config, geometry = case(name)
    assert status == recorded_status
    off = plan_return_leg(scenario, start, home, config, geometry=geometry)
    assert off.status == recorded_status
    on = plan_return_leg(scenario, start, home, config, geometry=geometry, return_start_backoff_m=CAP)
    assert on.success is on_ok
    boxed = plan_return_leg(scenario, start, home, config, geometry=geometry, return_start_backoff_m=CAP,
                            return_start_backoff_boxed=True)
    assert boxed.success
    rec = boxed.start_backoff
    assert (rec["status"], rec["advice_m"], rec["length_m"], rec["check"]) == ("prefix", 0.05, 0.08, "first_step")
    assert rec["trigger"] == ("boxed" if recorded_status == "no_path" else "invalid_start")


def test_boxed_with_the_forward_turnaround_backs_off_first():
    _, scenario, start, home, config, geometry = case("rc_G_aisle_rack_s1")
    plan = plan_return_leg(scenario, start, home, config, geometry=geometry, return_start_backoff_m=CAP,
                           return_start_backoff_boxed=True, forward_turnaround=True)
    assert plan.success and plan.start_backoff["status"] == "prefix" and plan.start_backoff["trigger"] == "boxed"
    assert plan.turnaround is not None  # the turnaround was weighed from the backoff's end


@pytest.mark.parametrize("name", ["rc_G_aisle_rack_s1", "rc_GN_aisle_rack_s10"])
def test_the_recorded_backed_off_return_is_followed_no_closer_to_the_leg(name):
    # D7: the backoff and the return go on in the same gear; the ideal follower (the runner's return lookahead)
    # steers before L, and still comes no closer to the props than the start was
    from forklift_core.control.low_clearance import shape_clearance
    from forklift_core.control.path_tracking import TrackerConfig
    from forklift_core.control.rollout import bicycle_rollout

    _, scenario, start, home, config, geometry = case(name)
    plan = plan_return_leg(scenario, start, home, config, geometry=geometry, return_start_backoff_m=CAP,
                           return_start_backoff_boxed=True)
    length = plan.start_backoff["length_m"]
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(plan.poses[:, :2], axis=0).T))]
    assert plan.directions[np.searchsorted(s, length + 1e-6)] == -1  # no cusp at the backoff's end
    cfg = TrackerConfig(cruise_speed_mps=0.25, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
                        position_tolerance_m=0.03, yaw_tolerance_rad=0.03)
    run = bicycle_rollout(plan.poses, plan.directions, plan.curvatures_inv_m, cfg, plan.poses[0], max_time_s=300.0)
    assert run.status == "arrived"
    trajectory = np.array(run.trajectory)
    near = trajectory[np.r_[0.0, np.cumsum(np.hypot(*np.diff(trajectory[:, :2], axis=0).T))] <= length + 0.30]
    obstacles, footprint, _ = pm.leg_obstacles("return", scenario, geometry)
    at_start = min(shape_clearance((start.x_m, start.y_m, start.yaw_rad), footprint, [r]) for r in obstacles)
    assert min(shape_clearance(p, footprint, obstacles) for p in near) >= at_start - 1e-6
    slot = pm.scenario_props(scenario, "slot")
    assert min(shape_clearance(p, footprint, slot) for p in near) >= config.clearance_m
