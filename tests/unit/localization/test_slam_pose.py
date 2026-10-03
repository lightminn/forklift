"""Online pose estimation pieces (docs/plans/2026-10-04-online-slam-closed-loop.md)."""

import math

import numpy as np
import pytest

from forklift_core.localization.slam_pose import (
    IncrementalWheelOdometry,
    LocalizationStale,
    SlamPoseEstimator,
    compose,
    invert,
)
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
)

GEOMETRY = AckermannOdometryGeometry(0.64, 0.51, 0.135)


def _drive(n=600, seed=0):
    rng = np.random.default_rng(seed)
    stamps = np.arange(n) / 120.0
    rates = np.column_stack([np.full(n, 3.0), np.full(n, 3.1)]) + rng.normal(0, 0.1, (n, 2))
    steer = np.column_stack([np.linspace(0, 0.3, n), np.linspace(0, 0.25, n)])
    return stamps, rates, steer


def test_incremental_odometry_matches_the_batch_integrator_exactly():
    stamps, rates, steer = _drive()
    batch = integrate_wheel_odometry(stamps, rates, steer, GEOMETRY, initial_pose=(0.34, 0.0, 0.1))
    odometry = IncrementalWheelOdometry(GEOMETRY, initial_pose=(0.34, 0.0, 0.1))
    poses = [odometry.update(t, r, s) for t, r, s in zip(stamps, rates, steer)]
    np.testing.assert_array_equal(np.array(poses), batch)


def test_incremental_odometry_rejects_time_going_backwards():
    odometry = IncrementalWheelOdometry(GEOMETRY)
    odometry.update(1.0, (1, 1), (0, 0))
    with pytest.raises(ValueError):
        odometry.update(1.0, (1, 1), (0, 0))


def test_compose_and_invert_are_consistent():
    a, b = (1.0, 2.0, 0.3), (-0.5, 0.25, -1.1)
    ab = compose(a, b)
    np.testing.assert_allclose(compose(invert(a), ab), b, atol=1e-12)
    np.testing.assert_allclose(compose(a, invert(a)), (0, 0, 0), atol=1e-12)


def test_the_estimate_is_map_from_odom_times_odom_from_base():
    estimator = SlamPoseEstimator(max_age_s=0.3)
    estimator.set_map_from_odom(10.0, (0.5, -0.2, 0.05))
    pose = estimator.map_from_base(10.05, (1.0, 0.0, 0.0))
    expected = (0.5 + math.cos(0.05), -0.2 + math.sin(0.05), 0.05)
    np.testing.assert_allclose(pose, expected, atol=1e-12)


def test_a_stale_correction_stops_the_estimate():
    estimator = SlamPoseEstimator(max_age_s=0.3)
    with pytest.raises(LocalizationStale):
        estimator.map_from_base(0.0, (0, 0, 0))  # nothing received yet
    estimator.set_map_from_odom(10.0, (0, 0, 0))
    estimator.map_from_base(10.3, (0, 0, 0))  # exactly at the limit is fine
    with pytest.raises(LocalizationStale):
        estimator.map_from_base(10.31, (0, 0, 0))


def test_corrections_must_not_go_back_in_time():
    estimator = SlamPoseEstimator(max_age_s=0.3)
    estimator.set_map_from_odom(10.0, (0, 0, 0))
    with pytest.raises(ValueError):
        estimator.set_map_from_odom(9.9, (0, 0, 0))
