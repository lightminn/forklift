"""Ackermann dead reckoning from rear wheel rates and front steering angles."""

import math

import numpy as np
import pytest

from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
)

GEOMETRY = AckermannOdometryGeometry(
    wheelbase_m=0.64, track_m=0.51, wheel_radius_m=0.135
)


def ackermann_angles(curvature):
    # Independent of the implementation: each front wheel points at the turn
    # centre, which lies 1/curvature to the left of the rear axle centre.
    radius = 1 / curvature
    left = math.atan(GEOMETRY.wheelbase_m / (radius - GEOMETRY.track_m / 2))
    right = math.atan(GEOMETRY.wheelbase_m / (radius + GEOMETRY.track_m / 2))
    return left, right


def test_straight_driving_integrates_speed_times_time():
    stamps = np.linspace(0, 10, 1201)
    rate = 0.5 / GEOMETRY.wheel_radius_m
    poses = integrate_wheel_odometry(
        stamps, np.full((1201, 2), rate), np.zeros((1201, 2)), GEOMETRY
    )
    np.testing.assert_allclose(poses[-1], [5.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(poses[:, 0], 0.5 * stamps, atol=1e-12)


def test_constant_steering_closes_a_circle_of_the_commanded_radius():
    curvature = 0.5
    speed = 0.4
    period = 2 * math.pi / (speed * curvature)
    stamps = np.linspace(0, period, 2001)
    steering = np.tile(ackermann_angles(curvature), (2001, 1))
    rates = np.full((2001, 2), speed / GEOMETRY.wheel_radius_m)
    poses = integrate_wheel_odometry(stamps, rates, steering, GEOMETRY)
    np.testing.assert_allclose(poses[-1, :2], [0.0, 0.0], atol=1e-9)
    quarter = poses[500]
    np.testing.assert_allclose(quarter[:2], [2.0, 2.0], atol=1e-9)
    assert math.cos(quarter[2] - math.pi / 2) == pytest.approx(1.0)


def test_reversing_moves_backwards_and_start_pose_is_respected():
    stamps = np.array([0.0, 1.0, 2.0])
    rates = np.full((3, 2), -0.2 / GEOMETRY.wheel_radius_m)
    poses = integrate_wheel_odometry(
        stamps, rates, np.zeros((3, 2)), GEOMETRY, initial_pose=(1.0, 2.0, math.pi / 2)
    )
    np.testing.assert_allclose(poses[-1], [1.0, 1.6, math.pi / 2], atol=1e-12)


def test_unequal_rear_wheels_move_the_axle_centre_at_their_mean():
    stamps = np.array([0.0, 1.0])
    rates = np.array([[1.0, 3.0], [1.0, 3.0]]) / GEOMETRY.wheel_radius_m
    poses = integrate_wheel_odometry(stamps, rates, np.zeros((2, 2)), GEOMETRY)
    np.testing.assert_allclose(poses[-1], [2.0, 0.0, 0.0], atol=1e-12)


@pytest.mark.parametrize(
    "stamps, rates, steering",
    [
        (np.array([0.0, 0.0]), np.zeros((2, 2)), np.zeros((2, 2))),
        (np.array([0.0, 1.0]), np.zeros((3, 2)), np.zeros((2, 2))),
        (np.array([0.0, 1.0]), np.zeros((2, 2)), np.zeros((2, 3))),
        (np.array([0.0, np.nan]), np.zeros((2, 2)), np.zeros((2, 2))),
        (np.array([0.0, 1.0]), np.array([[0, 0], [np.inf, 0]]), np.zeros((2, 2))),
        (np.array([0.0]), np.zeros((1, 2)), np.zeros((1, 2))),
    ],
)
def test_malformed_encoder_series_raise(stamps, rates, steering):
    with pytest.raises(ValueError):
        integrate_wheel_odometry(stamps, rates, steering, GEOMETRY)


def test_nonpositive_geometry_raises():
    with pytest.raises(ValueError):
        AckermannOdometryGeometry(wheelbase_m=0.0, track_m=0.51, wheel_radius_m=0.135)
