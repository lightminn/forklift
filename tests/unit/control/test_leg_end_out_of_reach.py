"""The leg end filter (docs/plans/2026-10-24-goal-creep-stall.md G1): a read-only view, true at a final goal in the
last LEG_END_SLACK_M where the floor is off and the end, extrapolated on the present heading, stays across the
tolerance by more than the curvature cap can bend it back over the travel left (to the end normal plus the overshoot
allowance). A filter, not a proof of no arrival: the core acts on it only with its own observation (G2)."""

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, TrackerConfig, path_tracking

from test_leg_end_min_speed import BASE, GOAL, along, straight


def _at(path, config, setup, pose):
    tracker = RearAxlePathTracker(*path, config)
    for p in setup:
        tracker.update(p, 0.1, 0.02)
    tracker.update(np.asarray(pose, float), 0.0, 1 / 120)
    return tracker


def test_nothing_before_the_first_update():
    tracker = RearAxlePathTracker(*straight(), GOAL)
    assert not tracker.leg_end_out_of_reach()


@pytest.mark.parametrize("direction", [1, -1])
def test_an_end_well_across_is_out_of_reach(direction):
    # 8.4 mm across, heading on the path, 0.04 mm left: 8.4 - 0.1635 = 8.237 mm > 8.01 mm.
    tracker = _at(straight(direction), GOAL, along(0.99996, 0.0084, 0.0, direction), [0.99996 * direction, 0.0084, 0.0])
    assert tracker.leg_end_out_of_reach()


@pytest.mark.parametrize("across", [0.0079, 0.0081, 0.00815])
def test_an_end_within_the_curvature_allowance_is_not(across):
    # The allowance over the 30.14 mm of travel is 0.36 * 0.03014^2 / 2 = 0.1635 mm: up to 8.17 mm a bent run may still
    # arrive (Codex rounds 1 and 3), so the filter stays false.
    tracker = _at(straight(), GOAL, along(0.99996, across, 0.0), [0.99996, across, 0.0])
    assert not tracker.leg_end_out_of_reach()


def test_without_an_overshoot_allowance_the_bound_is_the_normal_distance():
    config = TrackerConfig(**{**BASE, "overshoot_tolerance_m": None})
    tracker = _at(straight(), config, along(0.99996, 0.0081, 0.0), [0.99996, 0.0081, 0.0])
    assert tracker.leg_end_out_of_reach()


def test_a_floor_owned_end_is_not():
    # 44 mm across: the floor drives it (#105), not this filter.
    tracker = _at(straight(), GOAL, along(0.99996, 0.044, 0.0), [0.99996, 0.044, 0.0])
    assert not tracker.leg_end_out_of_reach()


def test_more_than_the_slack_left_is_not():
    tracker = _at(straight(), GOAL, along(0.99, 0.0084, 0.0), [0.99, 0.0084, 0.0])
    assert tracker.remaining_to_goal_m() > path_tracking.LEG_END_SLACK_M
    assert not tracker.leg_end_out_of_reach()


def test_a_cusp_end_is_not():
    poses = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.0, 0.0]])
    directions = np.array([1, 1, 1, -1], np.int8)
    tracker = RearAxlePathTracker(poses, directions, np.zeros(4), GOAL)
    for x in np.linspace(0.0, 0.99996, 40):
        tracker.update([x, 0.0084, 0.0], 0.1, 0.02)
    assert tracker.leg_end_index == 2
    assert not tracker.leg_end_out_of_reach()


def test_the_distance_to_the_end_normal_reads_the_pose_not_the_progress():
    tracker = _at(straight(), GOAL, along(0.9995, 0.0084, 0.0), [0.9986, 0.0084, 0.0])
    assert tracker.distance_to_end_normal_m() == pytest.approx(0.0014, abs=1e-9)
