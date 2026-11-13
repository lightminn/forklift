"""The body-up axis of an IMU and the body yaw rate under the planar assumption (docs/design/2026-11-14-imu-record-and-
mount.md 5, G3).

On a level floor with the truck moving only on it, the body yaw rate is ``(omega - bias) . u``, ``u`` the body-up unit
vector seen in the IMU frame; a full mount rotation is not needed. ``u`` is measured two ways and compared: the mean
acceleration of a standstill (an accelerometer reads the upward reaction +g) and the mean bias-free rotation of a
constant-curvature arc (the truck cannot turn on the spot; on a plane an arc's rotation is about the up axis alone), its
sign set by the turn the operator saw (counter-clockwise seen from above = +yaw). The bias is the mean of the
standstills before and after the arc. The sign is checked against gravity's up, not against the arc's own axis.
``off_axis_rate`` is a planar-violation diagnostic, not a bound on the yaw error.
"""

from __future__ import annotations

import math

import numpy as np

G = 9.80665
STILL_SPREAD_MPS2 = 0.5  # a standstill's acceleration samples stay this close to their mean (else: moving)
STILL_NORM_TOL = 0.1  # and the mean's length is g within this fraction
MIN_SPIN_RAD_S = 0.05  # an arc turns at least this fast on average once the bias is removed


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    if n == 0.0:
        raise ValueError("zero vector")
    return v / n


def _vec3(v) -> np.ndarray:
    v = np.asarray(v, float).reshape(3)
    return v


def up_from_gravity(accel) -> np.ndarray:
    """The body-up axis from a standstill's accelerometer samples (N x 3, m/s^2). Never from a moving segment: an arc
    adds centripetal acceleration."""
    a = np.asarray(accel, float)
    mean = a.mean(axis=0)
    spread = float(np.sqrt(((a - mean) ** 2).sum(axis=1)).max())
    if spread > STILL_SPREAD_MPS2 or abs(np.linalg.norm(mean) / G - 1.0) > STILL_NORM_TOL:
        raise ValueError(f"not still: spread {spread:.3f} m/s^2, |mean| {np.linalg.norm(mean):.3f} m/s^2")
    return _unit(mean)


def bias_from_still(gyro) -> np.ndarray:
    """The gyro bias (rad/s): a standstill's mean rate."""
    return np.asarray(gyro, float).mean(axis=0)


def bias_between(before, after) -> dict:
    """The bias to remove from the arc between two standstills (their mean) and how far it moved (warm-up)."""
    b0, b1 = _vec3(before), _vec3(after)
    return {"bias": (b0 + b1) / 2, "change_rad_s": float(np.linalg.norm(b1 - b0))}


def up_from_spin(gyro, *, bias, operator_ccw: bool) -> np.ndarray:
    """The body-up axis from an arc (N x 3, rad/s): the bias-free mean rotation's direction, signed so that the turn
    the operator saw (``operator_ccw``) is positive yaw -- both directions then point up."""
    mean = np.asarray(gyro, float).mean(axis=0) - _vec3(bias)
    if float(np.linalg.norm(mean)) < MIN_SPIN_RAD_S:
        raise ValueError(f"too little spin: bias-free mean rate {np.linalg.norm(mean):.4f} rad/s < {MIN_SPIN_RAD_S}")
    u = _unit(mean)
    return u if operator_ccw else -u


def angle_deg(u, v) -> float:
    c = float(np.clip(_unit(u) @ _unit(v), -1.0, 1.0))
    return math.degrees(math.acos(c))


def yaw_rate(gyro, up, *, bias) -> np.ndarray:
    """Body yaw rate per sample (rad/s) under the planar assumption."""
    return (np.asarray(gyro, float) - _vec3(bias)) @ _unit(up)


def off_axis_rate(gyro, up, *, bias) -> float:
    """The mean bias-free rotation off the up axis (rad/s): a planar-violation diagnostic (roll and pitch rates while
    driving). It is not the yaw error: a tilted body turning about its own z has none and still differs from the Euler
    yaw rate."""
    g = np.asarray(gyro, float) - _vec3(bias)
    u = _unit(up)
    perp = g - np.outer(g @ u, u)
    return float(np.linalg.norm(perp.mean(axis=0)))


def spin_check(gyro, *, bias, up_gravity, operator_ccw: bool) -> dict:
    """Does the arc turn the way the operator saw, judged against gravity's up (independent of the arc's axis)?"""
    mean_yaw = float(yaw_rate(gyro, up_gravity, bias=bias).mean())
    return {"mean_yaw_rad_s": mean_yaw, "sign_matches_operator": (mean_yaw > 0) == operator_ccw}
