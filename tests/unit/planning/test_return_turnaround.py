"""Return-leg forward preference: plan_return_leg(forward_turnaround=True)
(docs/plans/2026-10-12-return-forward-turnaround.md)."""

import math
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.planning import Bounds, Pose2D, collision_free_path, pallet_mission
from forklift_core.planning.pallet_mission import (
    D7_PLANNER_OPTIONS,
    PalletSite,
    SyntheticMissionGeometry,
    TransportScenario,
    leg_obstacles,
    make_transport_planner_config,
    path_terms,
    plan_return_leg,
)

GEOMETRY = SyntheticMissionGeometry()
# The measured chassis' planning curvature with the D7 options, as the recorded runs plan travel.
CONFIG = replace(
    make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10, max_expansions=30000),
    **D7_PLANNER_OPTIONS,
)
TRAVEL = replace(CONFIG, obstacle_heuristic_resolution_m=0.25)
# Withdrawn from a pallet delivered southwards, home 15 m north (the recorded seed 5's shape).
WITHDRAWN = Pose2D(0.0, 0.0, -math.pi / 2)
HOME = Pose2D(2.0, 15.0, 0.0)


def scenario(destination=PalletSite(0.0, -2.0, -math.pi / 2)):
    return TransportScenario(
        seed=0,
        start_rear=HOME,
        pickup=PalletSite(15.0, 15.0, 0.0),
        destination=destination,
        props=(),
        bounds=Bounds(-12.0, 14.0, -8.0, 20.0),
    )


def plan(start=WITHDRAWN, **kwargs):
    return plan_return_leg(scenario(kwargs.pop("destination", PalletSite(0.0, -2.0, -math.pi / 2))),
                           start, HOME, CONFIG, geometry=GEOMETRY, travel_config=TRAVEL, **kwargs)


def same_path(a, b):
    np.testing.assert_array_equal(a.poses, b.poses)
    np.testing.assert_array_equal(a.directions, b.directions)
    np.testing.assert_array_equal(a.curvatures_inv_m, b.curvatures_inv_m)


def test_home_behind_the_truck_is_reached_driving_forward_after_turning_round():
    direct = plan()
    assert direct.success and direct.turnaround is None
    assert path_terms(direct, TRAVEL, 1.0)["reverse_m"] > 10.0  # the direct plan reverses home
    turned = plan(forward_turnaround=True)
    assert turned.success
    record = turned.turnaround
    assert record["evaluated"] and record["chosen"].startswith("reverse_arc:")
    terms = path_terms(turned, TRAVEL, 1.0)
    assert terms["forward_m"] > 2 * terms["reverse_m"]
    assert path_terms(turned, TRAVEL, pallet_mission.RETURN_TURNAROUND_REVERSE_WEIGHT)["cost"] < record["direct"]["cost"]
    # One path in the array contract: it starts at the start, opens with the reverse arc and
    # stays collision-free against the leg's rectangles at the search clearance.
    np.testing.assert_allclose(turned.poses[0], [WITHDRAWN.x_m, WITHDRAWN.y_m, WITHDRAWN.yaw_rad])
    assert turned.directions[0] == turned.directions[1] == -1
    assert np.max(np.hypot(*np.diff(turned.poses[:, :2], axis=0).T)) < 0.3
    obstacles, footprint, bounds = leg_obstacles("return", scenario(), GEOMETRY)
    assert collision_free_path(turned.poses, obstacles, footprint, bounds)
    np.testing.assert_allclose(turned.poses[-1][:2], [HOME.x_m, HOME.y_m], atol=1e-6)


def test_a_return_that_already_faces_home_is_left_as_planned():
    start = Pose2D(0.0, 0.0, math.pi / 2)
    away = PalletSite(-8.0, -5.0, 0.0)
    direct = plan(start, destination=away)
    turned = plan(start, destination=away, forward_turnaround=True)
    same_path(turned, direct)
    assert not turned.turnaround["evaluated"] and turned.turnaround["chosen"] == "direct"


def test_arcs_the_permission_refuses_leave_the_direct_plan():
    seen = []

    def refuse(poses):
        seen.append(poses)
        return "unknown"

    direct = plan()
    turned = plan(forward_turnaround=True, turnaround_check=refuse)
    same_path(turned, direct)
    assert turned.turnaround["chosen"] == "direct"
    statuses = {c["status"] for c in turned.turnaround["candidates"]}
    assert statuses <= {"arc_not_permitted:unknown", "arc_blocked"} and "arc_not_permitted:unknown" in statuses
    # The check sees the arc from the start on.
    np.testing.assert_allclose(seen[0][0], [WITHDRAWN.x_m, WITHDRAWN.y_m, WITHDRAWN.yaw_rad])
    assert len(seen[0]) > 2


def test_the_searches_run_cheapest_bound_first_and_hopeless_ones_are_pruned():
    record = plan(forward_turnaround=True).turnaround
    bounded = [c for c in record["candidates"] if "lower_bound" in c]
    assert bounded
    searched = [c for c in bounded if "expansions" in c]
    best = min([record["direct"]["cost"]] + [c["cost"] for c in searched if "cost" in c])
    for c in bounded:
        if c["status"] == "pruned":
            assert c["lower_bound"] >= best - 1e-9
        if "cost" in c:  # the bound is a bound on the joined path's own cost
            assert c["lower_bound"] <= c["cost"] + 1e-9
    assert record["chosen"] == "direct" or record["chosen"] in {c["name"] for c in searched}


def test_equal_inputs_give_equal_turnarounds():
    a, b = plan(forward_turnaround=True), plan(forward_turnaround=True)
    same_path(a, b)
    assert a.turnaround == b.turnaround


def test_the_time_safety_skips_candidates_and_keeps_the_direct_plan(monkeypatch):
    monkeypatch.setattr(pallet_mission, "RETURN_TURNAROUND_MAX_S", -1.0)
    direct = plan()
    turned = plan(forward_turnaround=True)
    same_path(turned, direct)
    assert turned.turnaround["timed_out"]
    statuses = {c["status"] for c in turned.turnaround["candidates"]}
    assert "skipped_time" in statuses and statuses <= {"skipped_time", "pruned", "arc_blocked"}
    assert not any("expansions" in c for c in turned.turnaround["candidates"])  # no search ran
    assert len(turned.turnaround["candidates"]) == 2 * len(pallet_mission.RETURN_TURNAROUND_ANGLES_DEG)


@pytest.mark.parametrize("forward_turnaround", [False, True])
def test_a_failed_direct_plan_is_returned_as_it_is(forward_turnaround):
    blocked_start = Pose2D(0.0, -2.0, -math.pi / 2)  # inside the delivered pallet
    result = plan(blocked_start, forward_turnaround=forward_turnaround)
    assert not result.success and result.status == "invalid_start"


class _Clock:
    """A clock that only moves when a search returns (each takes ``step`` seconds)."""

    def __init__(self, step):
        self.now, self.step = 0.0, step

    def monotonic(self):
        return self.now


def test_a_search_budget_is_shared_with_the_direct_plan_and_late_candidates_are_dropped(monkeypatch):
    from forklift_core.planning import hybrid_astar

    clock = _Clock(3.0)
    real = pallet_mission.plan_hybrid_astar

    def slow(*args, **kwargs):
        result = real(*args, **kwargs)
        clock.now += clock.step
        return result

    monkeypatch.setattr(pallet_mission, "time", clock)
    monkeypatch.setattr(hybrid_astar, "time", clock)
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", slow)
    # The direct plan takes one search (3 s); the budget leaves 4 s of the call's 7 s for the
    # candidates, not 10 s after the direct plan.
    turned = plan(forward_turnaround=True, deadline=pallet_mission.SearchBudget(7.0))
    record = turned.turnaround
    assert record["evaluated"] and record["timed_out"]
    # Searches run cheapest lower bound first: the first is compared, the second ends after the limit.
    searched = sorted((c for c in record["candidates"] if "expansions" in c), key=lambda c: c["lower_bound"])
    assert len(searched) == 2 and searched[1]["status"] == "late" and "cost" not in searched[1]
    assert searched[0]["status"] == "success" and "cost" in searched[0]
    rest = [c for c in record["candidates"] if "expansions" not in c]
    assert rest and all(c["status"] in ("skipped_time", "pruned", "arc_blocked") for c in rest)
    assert record["chosen"] != searched[1]["name"]


@pytest.mark.parametrize("travel_curvature, expected", [(0.29, 0.25), (0.25, 0.25), (0.20, 0.20)])
def test_only_the_candidates_plan_at_the_gentler_curvature(travel_curvature, expected):
    travel = replace(TRAVEL, curvature_limit_inv_m=travel_curvature)
    config = replace(CONFIG, curvature_limit_inv_m=travel_curvature)
    args = (scenario(), WITHDRAWN, HOME, config)
    direct = plan_return_leg(*args, geometry=GEOMETRY, travel_config=travel)
    turned = plan_return_leg(*args, geometry=GEOMETRY, travel_config=travel, forward_turnaround=True)
    record = turned.turnaround
    assert record["candidate_curvature_inv_m"] == pytest.approx(expected)
    assert record["cost_curvature_inv_m"] == pytest.approx(travel_curvature)
    assert record["direct"]["cost"] == pytest.approx(path_terms(direct, travel, pallet_mission.RETURN_TURNAROUND_REVERSE_WEIGHT)["cost"])
    if record["chosen"] == "direct":
        same_path(turned, direct)
    else:  # the arc's curvature is the candidates' (sample zero carries the arc's controls)
        assert abs(turned.curvatures_inv_m[0]) == pytest.approx(expected)


def test_the_arc_checks_stop_at_the_limit_too(monkeypatch):
    """Codex 2026-10-12 6th P2: with the direct plan taking almost all of a SearchBudget, the arcs'
    checks must not run on past it unrecorded."""
    from forklift_core.planning import hybrid_astar

    clock = _Clock(0.0)
    real = pallet_mission.plan_hybrid_astar

    def slow_direct(*args, **kwargs):
        result = real(*args, **kwargs)
        clock.now += 19.0  # only the direct plan's searches run before the arcs
        return result

    def slow_check(poses):
        clock.now += 0.4
        return None

    monkeypatch.setattr(pallet_mission, "time", clock)
    monkeypatch.setattr(hybrid_astar, "time", clock)
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", slow_direct)
    turned = plan(forward_turnaround=True, deadline=pallet_mission.SearchBudget(20.0), turnaround_check=slow_check)
    record = turned.turnaround
    assert record["timed_out"] and clock.now <= 19.0 + 0.4 * 4 + 1e-9  # a few checks, not all ten
    assert any(c["status"] == "skipped_time" for c in record["candidates"])


@pytest.mark.parametrize("collision_step_m", [0.06, 1.0])
def test_the_lower_bound_never_exceeds_the_candidates_own_cost(collision_step_m):
    """Codex 2026-10-12 6th P2: with home at the end of a candidate's own reverse arc that candidate is
    the arc alone, so its true cost is path_terms over the arc's samples; a bound on the analytic arc
    length (longer than those chords, most at a coarse step) exceeded it and could prune a cheaper
    candidate. Ties still go to the direct plan."""
    from math import ceil

    from forklift_core.planning.hybrid_astar import PlanResult, _advance

    k = pallet_mission.RETURN_TURNAROUND_CURVATURE_INV_M
    travel = replace(TRAVEL, curvature_limit_inv_m=k, collision_step_m=collision_step_m)
    config = replace(CONFIG, curvature_limit_inv_m=k, collision_step_m=collision_step_m)
    start = (WITHDRAWN.x_m, WITHDRAWN.y_m, WITHDRAWN.yaw_rad)
    arc_m = math.radians(60) / k
    home = Pose2D(*_advance(start, -arc_m, -k))  # reverse_arc:-1:60's end
    turned = plan_return_leg(replace(scenario(), start_rear=home), WITHDRAWN, home, config,
                             geometry=GEOMETRY, travel_config=travel, forward_turnaround=True)
    record = turned.turnaround
    assert record["evaluated"]
    own = next(c for c in record["candidates"] if c["name"] == "reverse_arc:-1:60")
    # The arc as _sample_arc samples it: ceil(length / collision step) equal steps.
    n = max(1, ceil(arc_m / collision_step_m))
    poses = np.asarray([_advance(start, -arc_m * i / n, -k) for i in range(n + 1)])
    alone = PlanResult(True, "success", poses, np.full(n + 1, -1, dtype=np.int8), np.full(n + 1, -k), arc_m, 0)
    true_cost = path_terms(alone, travel, pallet_mission.RETURN_TURNAROUND_REVERSE_WEIGHT)["cost"]
    assert own["lower_bound"] <= true_cost + 1e-9
    if own["status"] == "pruned":  # only on a tie or worse with the best so far
        assert own["lower_bound"] >= record["direct"]["cost"] - 1e-9
