"""Destination scan docking (plan v3.8): frames, initial guess, ambiguity."""

import math

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
