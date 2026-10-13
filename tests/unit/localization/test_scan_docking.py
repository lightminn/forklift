"""Destination scan docking (plan v3.8): frames, initial guess, ambiguity."""

import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.localization.scan_docking import (
    dock,
    laser_points,
    ray_scan,
)
from forklift_core.localization.slam_pose import compose, invert

LASER = (-0.12 + 0.34, 0.0, 0.0)  # rear <- laser: base is 0.34 m ahead of the rear axle
ANGLES = -math.pi + np.arange(1600) * (2 * math.pi / 1600)


def _segments():
    # A corner and two boxes around the delivery site; walls at known lines.
    return [
        ((-6.0, 3.0), (6.0, 3.0)),
        ((5.0, -4.0), (5.0, 3.0)),
        ((-2.0, -1.5), (-1.2, -1.5)), ((-1.2, -1.5), (-1.2, -0.7)),
        ((1.5, 1.0), (2.3, 1.0)), ((2.3, 1.0), (2.3, 1.6)),
    ]


def _scan(rear_world, noise=0.0, seed=0):
    laser_world = compose(rear_world, LASER)
    ranges = ray_scan(_segments(), laser_world, ANGLES, 12.0)
    rng = np.random.default_rng(seed)
    noisy = ranges + rng.normal(0, noise, ranges.shape)
    return np.where(np.isfinite(ranges), noisy, ranges)


@pytest.mark.parametrize("error", [(0.0, 0.0, 0.0), (0.09, -0.08, 0.02), (-0.12, 0.05, -0.03)])
def test_the_delivery_goal_is_recovered_in_the_estimate_frame(error):
    delivery = (0.3, 0.2, 0.1)
    reference = laser_points(_scan(delivery), ANGLES)
    truth_now = compose(delivery, (-0.7, 0.02, -0.03))  # stopped at the straight start, a bit off
    estimate_now = compose(truth_now, error)  # what control believes
    live = laser_points(_scan(truth_now, noise=0.02, seed=1), ANGLES)
    result = dock(reference, live, rear_from_laser=LASER, estimate_rear=estimate_now, delivery_rear=delivery)
    assert result.accepted, result.reason
    # Where the delivery pose is in the estimate frame, from the truth:
    expected = compose(estimate_now, compose(invert(truth_now), delivery))
    np.testing.assert_allclose(result.goal_estimate[:2], expected[:2], atol=0.01)
    assert result.goal_estimate[2] == pytest.approx(expected[2], abs=0.004)


def test_a_correction_beyond_the_bound_is_refused():
    delivery = (0.3, 0.2, 0.1)
    reference = laser_points(_scan(delivery), ANGLES)
    truth_now = compose(delivery, (-0.7, 0.0, 0.0))
    estimate_now = compose(truth_now, (0.6, 0.0, 0.0))  # 0.6 m off: outside 0.3 m
    live = laser_points(_scan(truth_now, noise=0.02, seed=2), ANGLES)
    result = dock(reference, live, rear_from_laser=LASER, estimate_rear=estimate_now, delivery_rear=delivery)
    assert not result.accepted


def test_a_corridor_is_ambiguous_along_its_length():
    walls = [((-20.0, 1.0), (20.0, 1.0)), ((-20.0, -1.0), (20.0, -1.0))]
    delivery = (0.0, 0.0, 0.0)
    ref = laser_points(ray_scan(walls, compose(delivery, LASER), ANGLES, 12.0), ANGLES)
    truth_now = compose(delivery, (-0.7, 0.0, 0.0))
    live = laser_points(ray_scan(walls, compose(truth_now, LASER), ANGLES, 12.0), ANGLES)
    result = dock(ref, live, rear_from_laser=LASER, estimate_rear=compose(truth_now, (0.1, 0, 0)), delivery_rear=delivery)
    assert not result.accepted


def test_the_recorded_cycle_is_docked_and_every_start_reports_its_details():
    """2026-10-13: the recorded seed 6 delivery match (tests/fixtures/delivery_match) was
    refused not_converged on a period-two cycle; settled, it docks within a millimetre."""
    z = np.load(Path(__file__).resolve().parents[2] / "fixtures/delivery_match/seed6_cycle.npz")
    estimate = tuple(z["estimate"])
    result = dock(z["reference"], z["live"], rear_from_laser=tuple(z["rear_from_laser"]), estimate_rear=estimate,
                  delivery_rear=tuple(z["seed"]))
    assert result.accepted, result.reason
    assert len(result.starts) == 7 and all(len(start) == 3 for start in result.starts)
    assert len(result.start_details) == 7
    assert all(d["cycle"] is not None and d["iterations"] == d["cycle"][2] for d in result.start_details)
    expected = compose(estimate, compose(invert(tuple(z["truth_rear"])), tuple(z["reference_rear"])))
    assert math.hypot(result.goal_estimate[0] - expected[0], result.goal_estimate[1] - expected[1]) < 1e-3
    assert abs(result.goal_estimate[2] - expected[2]) < 1e-4


def test_a_refused_dock_still_reports_every_start():
    delivery = (0.3, 0.2, 0.1)
    reference = laser_points(_scan(delivery), ANGLES)
    truth_now = compose(delivery, (-0.7, 0.0, 0.0))
    estimate_now = compose(truth_now, (0.6, 0.0, 0.0))
    live = laser_points(_scan(truth_now, noise=0.02, seed=2), ANGLES)
    result = dock(reference, live, rear_from_laser=LASER, estimate_rear=estimate_now, delivery_rear=delivery)
    assert not result.accepted and len(result.start_details) == len(result.starts) == 7
    assert set(result.start_details[0]) == {"iterations", "cycle", "inlier_fraction", "median_residual_m",
                                            "min_information"}
