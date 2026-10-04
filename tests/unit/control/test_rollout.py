"""Kinematic dry run used to accept a near-capture start (plan v3.7)."""

import numpy as np

from forklift_core.control import TrackerConfig
from forklift_core.control.rollout import bicycle_rollout


def _straight(length=0.8, step=0.04):
    n = int(round(length / step)) + 1
    xs = np.linspace(0.0, length, n)
    return np.column_stack((xs, np.zeros(n), np.zeros(n))), np.ones(n, np.int8), np.zeros(n)


CONFIG = TrackerConfig(
    cruise_speed_mps=0.18,
    max_curvature_inv_m=0.9,
    max_acceleration_mps2=0.3,
    lookahead_m=0.28,
    position_tolerance_m=0.008,
    yaw_tolerance_rad=0.02,
    stop_speed_mps=0.012,
)


def test_an_aligned_start_arrives_with_no_error():
    result = bicycle_rollout(*_straight(), CONFIG, (0.0, 0.0, 0.0))
    assert result.status == "arrived"
    assert result.position_error_m < 0.008 and abs(result.yaw_error_rad) < 0.005


def test_a_large_offset_does_not_arrive_cleanly():
    result = bicycle_rollout(*_straight(), CONFIG, (0.0, 0.08, 0.06))
    assert result.status != "arrived" or abs(result.yaw_error_rad) > 0.015


def test_the_rollout_reports_the_poses_it_drove_through():
    result = bicycle_rollout(*_straight(), CONFIG, (0.0, 0.03, 0.0))
    poses = np.array(result.trajectory)
    assert len(poses) > 10
    np.testing.assert_allclose(poses[0], (0.0, 0.03, 0.0))
    assert poses[-1][0] > 0.7
