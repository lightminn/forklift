"""Online pose estimate from wheel odometry and a SLAM map<-odom correction.

Plan: docs/plans/2026-10-04-online-slam-closed-loop.md. The estimate is
map<-base(t) = map<-odom(last SLAM reply) * odom<-base(t, odometry): odometry
carries the pose between scans, SLAM corrects its drift. A correction older
than ``max_age_s`` stops the estimate (``LocalizationStale``) instead of letting
the robot drive on dead reckoning (roadmap: never drive on stale information).
Poses are planar (x, y metres, yaw radians).
"""

from __future__ import annotations

from math import atan2, cos, sin

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    _curvature_inv_m,
)


class LocalizationStale(RuntimeError):
    """No SLAM correction recent enough to drive on."""


def compose(a, b) -> tuple[float, float, float]:
    """a * b for planar poses (b expressed in a's frame)."""
    ax, ay, ayaw = (float(v) for v in a)
    bx, by, byaw = (float(v) for v in b)
    c, s = cos(ayaw), sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw


def invert(a) -> tuple[float, float, float]:
    x, y, yaw = (float(v) for v in a)
    c, s = cos(yaw), sin(yaw)
    return -(c * x + s * y), s * x - c * y, -yaw


def wrap(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


class IncrementalWheelOdometry:
    """``integrate_wheel_odometry`` one sample at a time, bit for bit.

    Each interval uses the mean speed and curvature of its two end samples and
    advances along that exact arc; yaw is left unwrapped like the batch form.
    """

    def __init__(self, geometry: AckermannOdometryGeometry, *, initial_pose=(0.0, 0.0, 0.0)):
        self.geometry = geometry
        self.pose = [_finite_scalar(v, "initial pose") for v in initial_pose]
        self._last = None  # (stamp, speed, curvature)

    def update(self, stamp_s, rear_wheel_rates_rad_s, steering_rad) -> tuple[float, float, float]:
        stamp = _finite_scalar(stamp_s, "stamp_s")
        rates = np.asarray(rear_wheel_rates_rad_s, dtype=float)
        steering = np.asarray(steering_rad, dtype=float)
        if rates.shape != (2,) or steering.shape != (2,):
            raise ValueError("wheel rates and steering must each hold two values")
        if not (np.isfinite(rates).all() and np.isfinite(steering).all()):
            raise ValueError("wheel rates and steering must be finite")
        if np.any(np.abs(steering) >= np.pi / 2):
            raise ValueError("steering angles must be below pi/2")
        speed = rates.mean() * self.geometry.wheel_radius_m
        curvature = float(_curvature_inv_m(steering[None, :], self.geometry)[0])
        if self._last is not None:
            last_stamp, last_speed, last_curvature = self._last
            dt = stamp - last_stamp
            if dt <= 0:
                raise ValueError("stamps must be strictly increasing")
            x, y, yaw = self.pose
            distance = (last_speed + speed) / 2 * dt
            kappa = (last_curvature + curvature) / 2
            turn = distance * kappa
            if abs(turn) < 1e-12:
                x += distance * cos(yaw + turn / 2)
                y += distance * sin(yaw + turn / 2)
            else:
                x += (sin(yaw + turn) - sin(yaw)) / kappa
                y += (cos(yaw) - cos(yaw + turn)) / kappa
            yaw += turn
            self.pose = [x, y, yaw]
        self._last = (stamp, speed, curvature)
        return tuple(self.pose)


class SlamPoseEstimator:
    """Holds the latest map<-odom and composes it with the odometry pose."""

    def __init__(self, *, max_age_s: float):
        self.max_age_s = _finite_scalar(max_age_s, "max_age_s")
        if self.max_age_s <= 0:
            raise ValueError("max_age_s must be positive")
        self.map_from_odom = None
        self.correction_stamp_s = None

    def set_map_from_odom(self, stamp_s, map_from_odom) -> None:
        stamp = _finite_scalar(stamp_s, "stamp_s")
        if self.correction_stamp_s is not None and stamp < self.correction_stamp_s:
            raise ValueError("SLAM corrections must not go back in time")
        pose = tuple(_finite_scalar(v, "map_from_odom") for v in map_from_odom)
        self.map_from_odom, self.correction_stamp_s = pose, stamp

    def age_s(self, now_s) -> float | None:
        if self.correction_stamp_s is None:
            return None
        return _finite_scalar(now_s, "now_s") - self.correction_stamp_s

    def map_from_base(self, now_s, odom_from_base) -> tuple[float, float, float]:
        age = self.age_s(now_s)
        if age is None or age > self.max_age_s + 1e-12:
            raise LocalizationStale(
                "no SLAM correction yet" if age is None else f"correction {age:.3f} s old"
            )
        return compose(self.map_from_odom, odom_from_base)


__all__ = [
    "IncrementalWheelOdometry",
    "LocalizationStale",
    "SlamPoseEstimator",
    "compose",
    "invert",
    "wrap",
]
