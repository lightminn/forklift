"""Persistent console plan D13 item 6: how far straight back the truck must go before the next preparation's start
check passes -- the planner's own check (hybrid_astar's invalid_start), on the preparation's own inputs."""

import math

import pytest

from forklift_core.planning import Bounds, Pose2D, Rectangle
from forklift_core.planning.pallet_mission import (
    PalletSite,
    TransportScenario,
    backoff_advice,
    make_transport_planner_config,
    observation_start_checker,
    plan_observation_leg,
)

SCENARIO = TransportScenario(0, Pose2D(-3, 0, 0), PalletSite(4, 3, 0), PalletSite(4, -3, 0), (), Bounds(-5, 6, -4, 4))
# the truck's front is at x = -1.71 (front_m 1.29): this box starts 1 cm ahead -- inside the 0.10 m clearance
AHEAD = Rectangle(-1.5, 0.0, 0.4, 1.0, 0.0)


def back(pose, d):
    return Pose2D(pose.x_m - d * math.cos(pose.yaw_rad), pose.y_m - d * math.sin(pose.yaw_rad), pose.yaw_rad)


def test_the_checker_judges_the_start_as_the_observation_leg_does():
    config = make_transport_planner_config()
    free, clearance = observation_start_checker(SCENARIO, config, pickup_obstacle=AHEAD)
    for d in (0.0, 0.05, 0.10, 0.20):
        start = back(SCENARIO.start_rear, d)
        status = plan_observation_leg(SCENARIO, Pose2D(2, 0, 0), config, start_rear=start,
                                      pickup_obstacle=AHEAD).status
        assert (status == "invalid_start") == (not free((start.x_m, start.y_m, start.yaw_rad), clearance)), d


def test_the_advice_is_the_first_distance_the_planner_takes():
    config = make_transport_planner_config()
    free, clearance = observation_start_checker(SCENARIO, config, pickup_obstacle=AHEAD)
    advice = backoff_advice(free, clearance, SCENARIO.start_rear)
    d = advice["backoff_m"]
    assert d == pytest.approx(0.10) and advice["backoff_max_m"] == pytest.approx(1.0)
    assert advice["from_rear"] == [-3.0, 0.0, 0.0]
    assert plan_observation_leg(SCENARIO, Pose2D(2, 0, 0), config, start_rear=back(SCENARIO.start_rear, d),
                                pickup_obstacle=AHEAD).status != "invalid_start"
    assert plan_observation_leg(SCENARIO, Pose2D(2, 0, 0), config, start_rear=back(SCENARIO.start_rear, d - 0.05),
                                pickup_obstacle=AHEAD).status == "invalid_start"


def fake(passing, blocked=()):
    """A checker over the distance backed off (x = -d): passes the start check (the clearance) at ``passing`` and the
    swept check (clearance 0) everywhere but ``blocked``."""
    def free(pose, clearance):
        d = round(-pose[0], 3)
        if clearance > 0:
            return d in passing
        return d not in blocked
    return free


START = Pose2D(0.0, 0.0, 0.0)


def test_a_single_passing_distance_is_the_whole_interval():
    """Codex D13 round 1 P2-2: 0.15 fails, 0.20 passes, 0.25 fails -- the interval is 0.20 alone."""
    advice = backoff_advice(fake({0.2}), 0.1, START)
    assert advice["backoff_m"] == pytest.approx(0.20) and advice["backoff_max_m"] == pytest.approx(0.20)


def test_a_wall_behind_on_the_way_is_reported_where_it_blocks():
    """agy: the swept path behind is checked too -- the advice never leads into a wall."""
    advice = backoff_advice(fake({0.3}, blocked={0.15}), 0.1, START)
    assert advice["backoff_m"] is None and advice["blocked_at_m"] == pytest.approx(0.15)


def test_nothing_within_the_limit_says_so():
    advice = backoff_advice(fake(set()), 0.1, START)
    assert advice["backoff_m"] is None and advice["within_m"] == pytest.approx(1.0)


def test_a_spent_deadline_skips_the_advice_rather_than_reporting_a_partial_search():
    """Codex D13 round 2 P3: an exhausted budget is no "not within 1.0 m"."""
    ticks = iter([0.0, 0.0, 5.0])
    advice = backoff_advice(fake({0.9}), 0.1, START, deadline=1.0, clock=lambda: next(ticks, 5.0))
    assert advice == {"skipped": "deadline"}


def ladder(waypoints, **grid):
    return {"world": SCENARIO, "waypoints": waypoints, "config": make_transport_planner_config(), "geometry": None,
            "pickup_bounds": None, "grid_kwargs": grid, "deadline": None}


def test_the_worker_adds_the_advice_when_every_candidate_meets_a_blocked_start():
    from forklift_core.planning import plan_worker as pw  # noqa: PLC0415

    out = pw._run("observe_ladder", ladder([(2, 0, 0), (2, 1, 0)], pickup_obstacle=AHEAD))
    assert out["result"] is None and {c["status"] for c in out["candidates"]} == {"invalid_start"}
    assert out["advice"]["backoff_m"] == pytest.approx(0.10)


def test_the_worker_gives_no_advice_when_the_start_was_not_the_reason():
    from forklift_core.planning import plan_worker as pw  # noqa: PLC0415

    out = pw._run("observe_ladder", ladder([(2, 0, 0)]))
    assert out["result"] is not None and "advice" not in out
