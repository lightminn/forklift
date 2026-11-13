"""G3 (docs/design/2026-11-14-imu-record-and-mount.md 5): the body-up axis in the IMU frame from a standstill's
gravity and from a constant-curvature arc with the standstill bias removed, the two directions compared, the sign
checked against gravity, and the rotation off the axis as a planar-violation diagnostic."""

from __future__ import annotations

import math

import numpy as np
import pytest

from forklift_core.sensors import imu_mount as im

G = 9.80665
# the SDK's depth frame: x right, y down, z forward -- a level camera has body-up = -y
BODY_UP_LEVEL = np.array([0.0, -1.0, 0.0])
BIAS = np.array([0.004, -0.0025, 0.003])  # rad/s, desk-capture size


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def still(up, *, n=2000, bias=BIAS, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    acc = np.tile(up * G, (n, 1)) + rng.normal(0, noise, (n, 3))
    gyr = np.tile(bias, (n, 1)) + rng.normal(0, noise * 0.01, (n, 3))
    return acc, gyr


def arc(up, rate, *, n=4000, bias=BIAS, extra=None, noise=0.0, seed=1):
    extra = np.zeros(3) if extra is None else extra
    rng = np.random.default_rng(seed)
    return np.tile(up * rate + bias + extra, (n, 1)) + rng.normal(0, noise * 0.01, (n, 3))


@pytest.mark.parametrize("pitch_deg", [0.0, 5.7, -3.0])
def test_gravity_and_the_bias_free_arc_axis_agree_for_a_tilted_mount(pitch_deg):
    up = rot_x(math.radians(pitch_deg)) @ BODY_UP_LEVEL  # the camera pitched by the mount
    acc, g_still = still(up, noise=0.02)
    bias = im.bias_from_still(g_still)
    ug = im.up_from_gravity(acc)
    uw = im.up_from_spin(arc(up, 0.17, noise=0.02), bias=bias, operator_ccw=True)
    assert np.allclose(ug, up, atol=1e-3) and np.allclose(uw, up, atol=1e-3)
    assert im.angle_deg(ug, uw) < 0.2


def test_leaving_the_bias_in_tilts_the_axis():
    up = BODY_UP_LEVEL
    big = np.array([0.02, 0.0, 0.0])
    tilted = im.up_from_spin(arc(up, 0.17, bias=big), bias=np.zeros(3), operator_ccw=True)
    assert im.angle_deg(tilted, up) > 5.0
    fixed = im.up_from_spin(arc(up, 0.17, bias=big), bias=big, operator_ccw=True)
    assert im.angle_deg(fixed, up) < 1e-6


def test_the_bias_drift_between_standstills_is_reported_and_their_mean_used():
    before, after = BIAS, BIAS + np.array([0.001, 0.0, 0.0])
    b = im.bias_between(im.bias_from_still(still(BODY_UP_LEVEL, bias=before)[1]),
                        im.bias_from_still(still(BODY_UP_LEVEL, bias=after)[1]))
    assert np.allclose(b["bias"], (before + after) / 2)
    assert b["change_rad_s"] == pytest.approx(0.001)


def test_both_directions_point_up_and_their_sign_is_checked_against_gravity():
    up = rot_x(math.radians(5.7)) @ BODY_UP_LEVEL
    left, right = arc(up, 0.17), arc(up, -0.17)
    u_l = im.up_from_spin(left, bias=BIAS, operator_ccw=True)
    u_r = im.up_from_spin(right, bias=BIAS, operator_ccw=False)
    assert im.angle_deg(u_l, u_r) < 1e-6  # both signed upward
    assert im.spin_check(left, bias=BIAS, up_gravity=up, operator_ccw=True)["sign_matches_operator"] is True
    # an operator who called the left arc clockwise is caught by gravity, not by the arc's own axis
    assert im.spin_check(left, bias=BIAS, up_gravity=up, operator_ccw=False)["sign_matches_operator"] is False
    assert im.spin_check(right, bias=BIAS, up_gravity=up, operator_ccw=False)["mean_yaw_rad_s"] < 0


def test_yaw_rate_is_the_projection_and_the_off_axis_part_is_measured():
    up = rot_x(math.radians(5.7)) @ BODY_UP_LEVEL
    extra = np.array([0.03, 0.0, 0.0])  # a roll-pitch rate leaking in (accelerating, bumps)
    gyr = arc(up, 0.3, extra=extra)
    yaw = im.yaw_rate(gyr, up, bias=BIAS)
    assert np.allclose(yaw, 0.3 + float(extra @ up), atol=1e-9)
    off = im.off_axis_rate(gyr, up, bias=BIAS)
    perp = np.linalg.norm(extra - (extra @ up) * up)
    assert off["mean_vector"] == pytest.approx(perp, rel=1e-6) and off["rms"] == pytest.approx(perp, rel=1e-6)


def test_an_oscillation_off_the_axis_does_not_cancel():
    up = BODY_UP_LEVEL
    gyr = arc(up, 0.2, n=1000)
    gyr[0::2] += [0.1, 0.0, 0.0]
    gyr[1::2] -= [0.1, 0.0, 0.0]  # a mast swaying back and forth
    off = im.off_axis_rate(gyr, up, bias=BIAS)
    assert off["mean_vector"] == pytest.approx(0.0, abs=1e-9)
    assert off["rms"] == pytest.approx(0.1) and off["p95"] == pytest.approx(0.1)


def test_non_finite_short_or_misshapen_input_is_refused():
    up = BODY_UP_LEVEL
    gyr = arc(up, 0.2, n=100)
    gyr[7, 1] = np.nan
    for f in (lambda: im.up_from_spin(gyr, bias=BIAS, operator_ccw=True),
              lambda: im.spin_check(gyr, bias=BIAS, up_gravity=up, operator_ccw=False),
              lambda: im.yaw_rate(gyr, up, bias=BIAS)):
        with pytest.raises(ValueError, match="non-finite"):
            f()
    with pytest.raises(ValueError, match="non-finite"):
        im.up_from_spin(arc(up, 0.2), bias=[np.nan, 0, 0], operator_ccw=True)
    with pytest.raises(ValueError, match="samples"):
        im.up_from_gravity(np.tile(up * G, (3, 1)))
    with pytest.raises(ValueError, match="N x 3"):
        im.bias_from_still(np.zeros((100, 2)))


def test_no_rotation_is_refused_by_the_sign_check_not_read_as_clockwise():
    up = BODY_UP_LEVEL
    with pytest.raises(ValueError, match="spin"):
        im.spin_check(arc(up, 0.0), bias=BIAS, up_gravity=up, operator_ccw=False)


def test_too_little_rotation_or_a_moving_standstill_is_refused():
    up = BODY_UP_LEVEL
    with pytest.raises(ValueError, match="spin"):
        im.up_from_spin(arc(up, 0.01, noise=0.01), bias=BIAS, operator_ccw=True)
    acc = np.tile(up * G, (100, 1))
    acc[50:] += np.array([3.0, 0.0, 0.0])  # moving: the mean is not gravity alone
    with pytest.raises(ValueError, match="still"):
        im.up_from_gravity(acc)
