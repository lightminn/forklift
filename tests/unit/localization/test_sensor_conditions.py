"""Replay/send-time sensor degradations (plan 2026-10-07 D5)."""

import math

import numpy as np
import pytest

from forklift_core.localization.sensor_conditions import (
    CONDITIONS,
    SensorCondition,
    condition,
)


def test_nominal_changes_nothing():
    nominal = condition("nominal")
    ranges = np.array([0.5, 11.0, math.inf, -math.inf])
    np.testing.assert_array_equal(nominal.ranges(ranges), ranges)
    assert nominal.lidar_available(70.0) and nominal.cameras_available(70.0)
    np.testing.assert_array_equal(nominal.wheel_rates([1.0, 2.0]), [1.0, 2.0])


def test_blackout_windows_are_half_open():
    lidar = condition("lidar_blackout")
    assert lidar.lidar_available(59.999) and not lidar.lidar_available(60.0)
    assert not lidar.lidar_available(79.999) and lidar.lidar_available(80.0)
    assert lidar.cameras_available(70.0)
    cameras = condition("camera_blackout")
    assert not cameras.cameras_available(140.0) and cameras.lidar_available(140.0)


def test_short_lidar_reads_far_beams_as_nothing_in_range():
    short = condition("lidar_short")
    out = short.ranges(np.array([3.9, 4.0, 4.1, 11.0, -math.inf]))
    np.testing.assert_array_equal(out, [3.9, 4.0, math.inf, math.inf, -math.inf])


def test_wheel_slip_scales_reported_travel():
    np.testing.assert_allclose(
        condition("wheel_slip").wheel_rates([2.0, -1.0]), [2.1, -1.05]
    )


def test_unknown_condition_and_bad_windows_are_refused():
    with pytest.raises(ValueError, match="unknown sensor condition"):
        condition("fog")
    with pytest.raises(ValueError):
        SensorCondition("bad", lidar_blackout_s=((5.0, 5.0),))
    assert set(CONDITIONS) == {
        "nominal",
        "lidar_blackout",
        "lidar_short",
        "camera_blackout",
        "wheel_slip",
    }
