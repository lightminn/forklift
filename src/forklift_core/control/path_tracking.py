"""Rear-axle geometric tracking and generic Ackermann conversion.

These calculations are hardware independent. Poses and curvature are synthetic
or adapter-supplied values; this module provides no obstacle or contact sensing.
"""

from dataclasses import dataclass
from math import atan, atan2, cos, hypot, isfinite, pi, sin, sqrt, tan
from typing import Literal

import numpy as np
from numpy.typing import ArrayLike

from forklift_core._validation import _finite_scalar


def _positive(name: str, value: float) -> None:
    if _finite_scalar(value, name) <= 0:
        raise ValueError(f"{name} must be finite and positive")


def _angle(value: float) -> float:
    return atan2(sin(value), cos(value))


@dataclass(frozen=True)
class TrackerConfig:
    """Geometric tracking limits in SI units; defaults are synthetic tuning."""

    cruise_speed_mps: float = 0.18
    max_curvature_inv_m: float = 0.50
    max_acceleration_mps2: float = 0.30
    lookahead_m: float = 0.35
    position_tolerance_m: float = 0.035
    yaw_tolerance_rad: float = 0.07
    stop_speed_mps: float = 0.015
    max_cross_track_error_m: float = 0.65
    # Tolerances at intermediate gear-change cusps. None applies the goal
    # tolerances there too. A cusp is not a goal: the next leg starts from the
    # measured pose, while a car-like vehicle stopped just off the cusp cannot
    # close a sideways offset and would otherwise wait at zero speed forever.
    cusp_position_tolerance_m: float | None = None
    cusp_yaw_tolerance_rad: float | None = None
    # How far past an endpoint, along the direction of travel, a stop still
    # counts as arrived. The sideways and heading limits stay as they are.
    # Braking lag carries a truck a few millimetres beyond the goal; with no
    # allowance it neither arrives nor backs up. None keeps the round tolerance.
    overshoot_tolerance_m: float | None = None
    # Optional speed caps along the path; None leaves speed to cruise and
    # braking alone. The lateral limit caps speed at sqrt(a / |curvature|) and
    # brakes ahead of a curve at max_acceleration_mps2, so a fast cruise does
    # not carry into a turn the steering cannot follow.
    max_lateral_acceleration_mps2: float | None = None
    max_reverse_speed_mps: float | None = None

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            optional = (
                "cusp_",
                "overshoot_",
                "max_lateral_",
                "max_reverse_",
            )
            if name.startswith(optional) and value is None:
                continue
            _positive(name, value)
        if self.yaw_tolerance_rad >= pi:
            raise ValueError("yaw_tolerance_rad must be less than pi")


@dataclass(frozen=True)
class TrackingCommand:
    """One calculation result, without a transport timestamp or clock domain.

    ``segment_index`` is the departure sample of the current path segment.
    ``progress_m`` never decreases. Errors refer to the current gear-leg endpoint.
    A failed result requires the caller to stop/replan; it is never arrival.
    """

    speed_mps: float
    curvature_inv_m: float
    status: Literal["tracking", "braking", "arrived", "failed"]
    progress_m: float
    segment_index: int
    position_error_m: float
    yaw_error_rad: float


class RearAxlePathTracker:
    """Track ordered rear-axle poses while stopping at every gear cusp.

    ``poses`` is finite float-like (N, 3), x/y metres and body yaw radians in
    one fixed frame, N >= 2. ``directions`` and ``curvatures_inv_m`` have shape
    (N,). Entry i describes the segment arriving at sample i; entry 0 repeats
    entry 1. Direction is +1/-1 and curvature is yaw change per signed distance.
    Adjacent samples must have different positions. Inputs are copied.

    The caller supplies measured signed rear-axle speed on every update. The
    return value is a numerical setpoint, so an adapter must attach its own
    timestamp, clock domain and watchdog before using it as a device command.
    """

    def __init__(
        self,
        poses: ArrayLike,
        directions: ArrayLike,
        curvatures_inv_m: ArrayLike,
        config: TrackerConfig | None = None,
    ) -> None:
        config = TrackerConfig() if config is None else config
        self.config = config
        self._poses = np.array(poses, dtype=float, copy=True)
        self._directions = np.array(directions, dtype=float, copy=True)
        self._curvatures = np.array(curvatures_inv_m, dtype=float, copy=True)
        if (
            self._poses.ndim != 2
            or self._poses.shape[1:] != (3,)
            or len(self._poses) < 2
        ):
            raise ValueError("poses must have shape (N, 3), N >= 2")
        size = len(self._poses)
        if self._directions.shape != (size,) or self._curvatures.shape != (size,):
            raise ValueError("directions and curvatures must have shape (N,)")
        if not all(
            np.isfinite(a).all()
            for a in (self._poses, self._directions, self._curvatures)
        ):
            raise ValueError("path values must be finite")
        if not np.isin(self._directions, [-1, 1]).all():
            raise ValueError("directions must be +1 or -1")
        if (
            self._directions[0] != self._directions[1]
            or self._curvatures[0] != self._curvatures[1]
        ):
            raise ValueError("sample 0 must repeat sample 1 segment metadata")
        if np.max(np.abs(self._curvatures)) > config.max_curvature_inv_m + 1e-9:
            raise ValueError("path curvature exceeds tracker curvature limit")
        self._vectors = np.diff(self._poses[:, :2], axis=0)
        self._lengths = np.linalg.norm(self._vectors, axis=1)
        if np.any(self._lengths <= 1e-9):
            raise ValueError("adjacent path positions must differ")
        self._distance = np.r_[0.0, np.cumsum(self._lengths)]
        self._speed_cap = self._speed_profile()
        self._leg_ends = (
            np.flatnonzero(self._directions[1:-1] != self._directions[2:]) + 1
        ).tolist() + [size - 1]
        self._leg = 0
        self._segment = 0
        self._progress = 0.0
        self._failed = False
        self._arrived = False
        self._command_speed: float | None = None

    def _speed_profile(self) -> np.ndarray:
        """Largest speed on the segment arriving at each sample, braked ahead."""
        cfg = self.config
        cap = np.full(len(self._poses), cfg.cruise_speed_mps)
        if cfg.max_reverse_speed_mps is not None:
            cap[self._directions < 0] = np.minimum(
                cap[self._directions < 0], cfg.max_reverse_speed_mps
            )
        if cfg.max_lateral_acceleration_mps2 is not None:
            curved = np.abs(self._curvatures) > 1e-9
            cap[curved] = np.minimum(
                cap[curved],
                np.sqrt(
                    cfg.max_lateral_acceleration_mps2 / np.abs(self._curvatures[curved])
                ),
            )
        # Reach each segment's cap by braking over the segments before it.
        for i in range(len(cap) - 2, -1, -1):
            cap[i] = min(
                cap[i],
                sqrt(
                    cap[i + 1] ** 2 + 2 * cfg.max_acceleration_mps2 * self._lengths[i]
                ),
            )
        return cap

    def nominal_duration_s(self) -> float:
        """Path time at each segment's speed cap, ignoring acceleration.

        Without speed caps this is length / cruise speed; a timeout scaled from
        it then allows for the slow curves the caps impose.
        """
        return float(np.sum(self._lengths / self._speed_cap[1:]))

    def _overshoot_accepted(
        self, pose: np.ndarray, goal: np.ndarray, endpoint: int, tolerance: float
    ) -> bool:
        limit = self.config.overshoot_tolerance_m
        if limit is None:
            return False
        offset = pose[:2] - goal[:2]
        heading = np.array([cos(goal[2]), sin(goal[2])])
        along = float(self._directions[endpoint] * np.dot(offset, heading))
        across = abs(float(np.dot(offset, [-heading[1], heading[0]])))
        return across <= tolerance and 0.0 <= along <= limit

    def update(
        self, pose_rear_axle: ArrayLike, signed_speed_mps: float, dt_s: float
    ) -> TrackingCommand:
        """Compute a bounded setpoint from finite (3,) x/y/yaw and measured speed.

        Local segment projection is ordered, never a global nearest-point search.
        A cusp is released only after a zero command and measured stopping speed.
        Failure latches on excessive deviation or endpoint heading mismatch; no
        elapsed-time heuristic converts stalled motion to success or failure.
        """
        pose = np.asarray(pose_rear_axle, dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError("pose_rear_axle must be a finite (3,) array")
        if not isfinite(signed_speed_mps):
            raise ValueError("signed_speed_mps must be finite")
        _positive("dt_s", dt_s)
        cfg = self.config
        endpoint = self._leg_ends[self._leg]
        goal = self._poses[endpoint]
        position_error = float(np.linalg.norm(goal[:2] - pose[:2]))
        yaw_error = _angle(float(goal[2] - pose[2]))

        # Only walk adjacent segments; repeated spatial locations cannot select
        # a later leg. Bound projection advance to a local motion-sized window.
        advance_limit = (
            self._progress + abs(signed_speed_mps) * dt_s + cfg.lookahead_m / 2
        )
        while True:
            i = self._segment
            fraction = float(
                np.dot(pose[:2] - self._poses[i, :2], self._vectors[i])
                / self._lengths[i] ** 2
            )
            if (
                fraction < 1
                or i + 1 >= endpoint
                or self._distance[i + 1] > advance_limit
            ):
                break
            self._segment += 1
        projected = (
            self._distance[i] + float(np.clip(fraction, 0, 1)) * self._lengths[i]
        )
        self._progress = max(self._progress, min(projected, advance_limit))
        nearest = self._poses[i, :2] + np.clip(fraction, 0, 1) * self._vectors[i]
        if np.linalg.norm(pose[:2] - nearest) > cfg.max_cross_track_error_m:
            self._failed = True
        remaining = max(0.0, float(self._distance[endpoint] - self._progress))
        position_tolerance, yaw_tolerance = (
            cfg.position_tolerance_m,
            cfg.yaw_tolerance_rad,
        )
        if endpoint != len(self._poses) - 1:
            if cfg.cusp_position_tolerance_m is not None:
                position_tolerance = cfg.cusp_position_tolerance_m
            if cfg.cusp_yaw_tolerance_rad is not None:
                yaw_tolerance = cfg.cusp_yaw_tolerance_rad
        at_endpoint = remaining <= position_tolerance and (
            position_error <= position_tolerance
            or self._overshoot_accepted(pose, goal, endpoint, position_tolerance)
        )
        if at_endpoint and abs(yaw_error) > yaw_tolerance:
            self._failed = True
        direction = self._directions[self._segment + 1]
        curvature = 0.0
        status = "tracking"
        if self._failed or self._arrived or at_endpoint:
            target_speed = 0.0
            status = "failed" if self._failed else "braking"
        else:
            target_distance = self._progress + cfg.lookahead_m
            if target_distance >= self._distance[endpoint]:
                extra = target_distance - self._distance[endpoint]
                signed_extra = direction * extra
                endpoint_curvature = self._curvatures[endpoint]
                if abs(endpoint_curvature) < 1e-9:
                    target = goal[:2] + signed_extra * np.array(
                        [cos(goal[2]), sin(goal[2])]
                    )
                else:
                    extended_yaw = goal[2] + endpoint_curvature * signed_extra
                    target = (
                        goal[:2]
                        + np.array(
                            [
                                sin(extended_yaw) - sin(goal[2]),
                                cos(goal[2]) - cos(extended_yaw),
                            ]
                        )
                        / endpoint_curvature
                    )
            else:
                j = min(
                    endpoint - 1,
                    int(
                        np.searchsorted(self._distance, target_distance, side="right")
                        - 1
                    ),
                )
                ratio = (target_distance - self._distance[j]) / self._lengths[j]
                target = self._poses[j, :2] + ratio * self._vectors[j]
            delta = target - pose[:2]
            lateral = -sin(pose[2]) * delta[0] + cos(pose[2]) * delta[1]
            curvature = float(
                np.clip(
                    2 * lateral / max(float(np.dot(delta, delta)), 1e-9),
                    -cfg.max_curvature_inv_m,
                    cfg.max_curvature_inv_m,
                )
            )
            speed_limit = min(
                cfg.cruise_speed_mps,
                sqrt(2 * cfg.max_acceleration_mps2 * remaining),
                1.5 * remaining,
                float(self._speed_cap[self._segment + 1]),
            )
            target_speed = float(direction * speed_limit)
            if (
                target_speed * signed_speed_mps < 0
                and abs(signed_speed_mps) > cfg.stop_speed_mps
            ):
                target_speed = 0.0
                status = "braking"
        delta_speed = cfg.max_acceleration_mps2 * dt_s
        # Slew the requested speed, not the measured actuator response. Rebasing
        # every tick on measured speed weakens braking by dt / actuator_tau.
        if self._command_speed is None:
            self._command_speed = signed_speed_mps
        speed = float(
            np.clip(
                target_speed,
                self._command_speed - delta_speed,
                self._command_speed + delta_speed,
            )
        )
        self._command_speed = speed
        if (
            at_endpoint
            and not self._failed
            and abs(signed_speed_mps) <= cfg.stop_speed_mps
            and abs(speed) < 1e-12
        ):
            if endpoint == len(self._poses) - 1:
                self._arrived = True
                status = "arrived"
            else:
                self._leg += 1
                self._segment = endpoint
                self._progress = float(self._distance[endpoint])
        if self._arrived:
            status = "arrived"
        return TrackingCommand(
            speed,
            curvature,
            status,
            self._progress,
            self._segment,
            position_error,
            yaw_error,
        )


@dataclass(frozen=True)
class AckermannGeometry:
    """Explicit generic geometry and actuator limits; no hardware defaults."""

    wheelbase_m: float
    track_m: float
    wheel_radius_m: float
    max_steering_rad: float
    max_wheel_rate_rad_s: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            _positive(name, getattr(self, name))
        if self.max_steering_rad >= pi / 2:
            raise ValueError("max_steering_rad must be below pi/2")


@dataclass(frozen=True)
class AckermannCommand:
    """Rear-axle speed/curvature, steer (FL, FR), wheel rates (FL, FR, RL, RR).

    Wheel rates are radians/s, positive for vehicle-forward rolling. Adapters
    must account for their wheel joint axes and attach command timing metadata.
    """

    speed_mps: float
    curvature_inv_m: float
    steering_rad: tuple[float, float]
    wheel_rates_rad_s: tuple[float, float, float, float]


def ackermann_command(
    speed_mps: float, curvature_inv_m: float, geometry: AckermannGeometry
) -> AckermannCommand:
    """Clamp curvature to both steering limits, then scale speed for wheel limits.

    Scaling all wheel rates together preserves the achievable curvature. Positive
    curvature turns the vehicle left when moving forward; steering does not flip
    on reversing, while each wheel rate does.
    """
    if not isfinite(speed_mps) or not isfinite(curvature_inv_m):
        raise ValueError("speed and curvature must be finite")
    half_track = geometry.track_m / 2
    tangent = tan(geometry.max_steering_rad)
    limit = tangent / (geometry.wheelbase_m + tangent * half_track)
    curvature = float(np.clip(curvature_inv_m, -limit, limit))
    left = 1 - curvature * half_track
    right = 1 + curvature * half_track
    yaw_factor = geometry.wheelbase_m * curvature
    steering = (atan(yaw_factor / left), atan(yaw_factor / right))
    factors = (hypot(left, yaw_factor), hypot(right, yaw_factor), left, right)
    allowed_speed = (
        geometry.max_wheel_rate_rad_s * geometry.wheel_radius_m / max(factors)
    )
    speed = float(np.clip(speed_mps, -allowed_speed, allowed_speed))
    wheels = tuple(speed * factor / geometry.wheel_radius_m for factor in factors)
    return AckermannCommand(speed, curvature, steering, wheels)
