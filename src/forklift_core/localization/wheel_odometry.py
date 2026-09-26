"""Ackermann dead reckoning from rear wheel rates and front steering angles.

The rear wheels do not steer, so their mean rolling speed is the rear-axle
speed. Each front steering angle implies a path curvature about the rear axle;
the two estimates are averaged. Neither slip nor encoder quantisation is
modelled: a simulated or measured series already contains whatever slip the
vehicle had, which is what makes odometry drift.
"""

from dataclasses import dataclass
from math import cos, sin

import numpy as np
from numpy.typing import ArrayLike, NDArray

from forklift_core._validation import _finite_scalar, _real_array


@dataclass(frozen=True)
class AckermannOdometryGeometry:
    """Wheelbase (rear to front axle), front track and rear wheel radius."""

    wheelbase_m: float
    track_m: float
    wheel_radius_m: float

    def __post_init__(self) -> None:
        for name in ("wheelbase_m", "track_m", "wheel_radius_m"):
            if _finite_scalar(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")


def _curvature_inv_m(steering_rad: NDArray, geometry: AckermannOdometryGeometry):
    half_track = geometry.track_m / 2
    left, right = np.tan(steering_rad[:, 0]), np.tan(steering_rad[:, 1])
    from_left = left / (geometry.wheelbase_m + left * half_track)
    from_right = right / (geometry.wheelbase_m - right * half_track)
    return (from_left + from_right) / 2


def integrate_wheel_odometry(
    stamps_s: ArrayLike,
    rear_wheel_rates_rad_s: ArrayLike,
    steering_rad: ArrayLike,
    geometry: AckermannOdometryGeometry,
    *,
    initial_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> NDArray[np.float64]:
    """Integrate rear-axle poses (N, 3) as x, y metres and unwrapped yaw.

    stamps_s: (N,) strictly increasing sample times in one clock, seconds.
    rear_wheel_rates_rad_s: (N, 2) rear left, rear right, positive rolling
    forward. steering_rad: (N, 2) front left, front right, positive left.
    Each interval uses the mean of its two end samples and advances along the
    exact arc of that speed and curvature. The first pose is initial_pose.
    """
    stamps = _real_array(stamps_s, "stamps_s")
    rates = _real_array(rear_wheel_rates_rad_s, "rear_wheel_rates_rad_s")
    steering = _real_array(steering_rad, "steering_rad")
    if stamps.ndim != 1 or len(stamps) < 2:
        raise ValueError("stamps_s must hold at least two samples")
    if rates.shape != (len(stamps), 2) or steering.shape != (len(stamps), 2):
        raise ValueError("wheel rates and steering must be (N, 2) like the stamps")
    if not (np.isfinite(stamps).all() and np.isfinite(rates).all()):
        raise ValueError("stamps and wheel rates must be finite")
    if not np.isfinite(steering).all() or np.any(np.abs(steering) >= np.pi / 2):
        raise ValueError("steering angles must be finite and below pi/2")
    if np.any(np.diff(stamps) <= 0):
        raise ValueError("stamps_s must be strictly increasing")
    x, y, yaw = (_finite_scalar(v, "initial pose") for v in initial_pose)
    speed = rates.mean(axis=1) * geometry.wheel_radius_m
    curvature = _curvature_inv_m(steering, geometry)
    poses = np.empty((len(stamps), 3))
    poses[0] = x, y, yaw
    for k, dt in enumerate(np.diff(stamps)):
        distance = (speed[k] + speed[k + 1]) / 2 * dt
        kappa = (curvature[k] + curvature[k + 1]) / 2
        turn = distance * kappa
        if abs(turn) < 1e-12:
            x += distance * cos(yaw + turn / 2)
            y += distance * sin(yaw + turn / 2)
        else:
            x += (sin(yaw + turn) - sin(yaw)) / kappa
            y += (cos(yaw) - cos(yaw + turn)) / kappa
        yaw += turn
        poses[k + 1] = x, y, yaw
    return poses


__all__ = ["AckermannOdometryGeometry", "integrate_wheel_odometry"]
