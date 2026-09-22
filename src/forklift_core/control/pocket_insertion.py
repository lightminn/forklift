"""Observation-gated low-speed insertion for an already aligned synthetic rig.

This controller neither localizes the vehicle nor certifies collision clearance.
It requires a new base-frame pocket observation on every update. An adapter must
execute the zero setpoint and supply its own watchdog between observations.
"""

import math
from dataclasses import dataclass, fields
from numbers import Integral
from typing import Literal

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pocket_observation import (
    CLOCK_DOMAINS,
    PocketObservation,
)


def _integer(value: int, name: str, minimum: int) -> int:
    if (
        isinstance(value, (bool, np.bool_))
        or not isinstance(value, Integral)
        or value < minimum
    ):
        raise ValueError(f"{name} must be an integer >= {minimum}, excluding bool")
    return int(value)


@dataclass(frozen=True)
class PocketInsertionConfig:
    """Explicitly synthetic limits, with all fork dimensions in base_link.

    Fork z bounds are relative to the base origin, not the ground: adapters must
    account for e.g. a base located 0.002 m above ground. Opening containment uses
    a configurable geometric margin; it is not a contact or uncertainty proof.
    Cruise applies only above slowdown_front_x_m; at or below that observed
    front distance the insertion speed caps the request. This setpoint schedule
    does not model actuator braking distance or enforce physical deceleration.
    """

    target_front_x_m: float = 0.59
    max_start_front_x_m: float = 1.1
    fork_center_y_m: float = 0.145
    fork_width_m: float = 0.055
    fork_bottom_z_m: float = 0.028
    fork_top_z_m: float = 0.052
    clearance_margin_m: float = 0.002
    max_observation_age_ns: int = 150_000_000
    max_frame_interval_ns: int = 100_000_000
    max_geometry_jump_m: float = 0.03
    confirmation_frames: int = 3
    cruise_speed_mps: float = 0.04
    insertion_speed_mps: float = 0.04
    slowdown_front_x_m: float = 1.0
    max_speed_mps: float = 0.055
    stopped_speed_mps: float = 0.012
    front_tolerance_m: float = 0.005
    max_yaw_error_rad: float = 0.025
    max_lateral_error_m: float = 0.012
    max_curvature_inv_m: float = 0.3
    lateral_gain_inv_m2: float = 8.0
    yaw_gain_inv_m: float = 2.0
    approach_gain_inv_s: float = 1.0

    def __post_init__(self) -> None:
        integers = {
            "max_observation_age_ns",
            "max_frame_interval_ns",
            "confirmation_frames",
        }
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in integers:
                value = _integer(value, field.name, 1)
            else:
                value = _finite_scalar(value, field.name)
                if field.name not in {"fork_bottom_z_m", "fork_top_z_m"} and value <= 0:
                    raise ValueError(f"{field.name} must be positive")
            object.__setattr__(self, field.name, value)
        if self.fork_bottom_z_m >= self.fork_top_z_m:
            raise ValueError("fork_bottom_z_m must be below fork_top_z_m")
        if self.fork_width_m >= 2 * self.fork_center_y_m:
            raise ValueError("forks must not overlap")
        if (
            not self.front_tolerance_m
            < self.target_front_x_m
            < self.max_start_front_x_m
        ):
            raise ValueError("target must lie beyond tolerance and below start limit")
        if not (
            self.stopped_speed_mps
            < self.insertion_speed_mps
            <= self.cruise_speed_mps
            <= self.max_speed_mps
        ):
            raise ValueError(
                "require stopped speed < insertion speed <= cruise speed <= maximum speed"
            )
        if not (
            self.target_front_x_m < self.slowdown_front_x_m <= self.max_start_front_x_m
        ):
            raise ValueError("require target front < slowdown front <= start limit")
        if self.max_yaw_error_rad >= math.pi / 2:
            raise ValueError("max_yaw_error_rad must be below pi/2")


@dataclass(frozen=True)
class PocketInsertionCommand:
    """Setpoint at controller time; status never turns loss into completion."""

    stamp_ns: int
    clock_domain: str
    status: Literal["acquiring", "tracking", "lost", "complete"]
    speed_mps: float
    curvature_inv_m: float
    reason: str
    confirmed_frames: int


class PocketInsertionController:
    """Confirm fresh geometry while stopped, then servo within a small envelope.

    Config is required to make the synthetic geometry choice explicit. No map,
    world pose, pallet ground truth or cached-target drive input is accepted.
    Completion latches zero motion; instantiate a new controller for a new task.
    """

    def __init__(self, config: PocketInsertionConfig) -> None:
        if not isinstance(config, PocketInsertionConfig):
            raise ValueError("config must be PocketInsertionConfig")
        self.config = config
        self._clock_domain: str | None = None
        self._last_now_ns: int | None = None
        self._last_stamp_ns: int | None = None
        self._previous: PocketObservation | None = None
        self._confirmed = 0
        self._tracking = False
        self._completed = False

    def update(
        self,
        observation: PocketObservation | None,
        *,
        now_ns: int,
        clock_domain: str,
        measured_speed_mps: float,
    ) -> PocketInsertionCommand:
        """Return zero on observation loss and require stopped fresh reacquisition.

        Acquisition stamps strictly increase; controller time cannot decrease or
        change domain. Invalid time/domain argument types raise ValueError. Sensor
        loss, clock discontinuities and invalid measured speed yield lost/zero.
        """
        now_ns = _integer(now_ns, "now_ns", 0)
        if not isinstance(clock_domain, str) or clock_domain not in CLOCK_DOMAINS:
            raise ValueError("Unsupported controller clock_domain")

        def command(status, reason, speed=0.0, curvature=0.0):
            return PocketInsertionCommand(
                now_ns, clock_domain, status, speed, curvature, reason, self._confirmed
            )

        def lost(reason):
            self._tracking = False
            self._confirmed = 0
            self._previous = None
            return command("lost", reason)

        if self._clock_domain is None:
            self._clock_domain = clock_domain
        if self._clock_domain != clock_domain:
            return lost("controller_clock_domain_changed")
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            return lost("controller_clock_backwards")
        self._last_now_ns = now_ns
        try:
            speed = _finite_scalar(measured_speed_mps, "measured_speed_mps")
        except ValueError:
            return lost("invalid_measured_speed")
        cfg = self.config
        if abs(speed) > cfg.max_speed_mps:
            return lost("measured_speed_exceeds_limit")
        if observation is None:
            return lost("missing_observation")
        if not isinstance(observation, PocketObservation):
            return lost("invalid_observation_type")
        if observation.clock_domain != clock_domain:
            return lost("observation_clock_domain_mismatch")
        if observation.stamp_ns > now_ns:
            return lost("future_observation")
        if (
            self._last_stamp_ns is not None
            and observation.stamp_ns <= self._last_stamp_ns
        ):
            return lost("observation_stamp_not_increasing")
        self._last_stamp_ns = observation.stamp_ns
        if now_ns - observation.stamp_ns > cfg.max_observation_age_ns:
            return lost("stale_observation")
        if observation.status != "valid":
            return lost("invalid_observation")
        if observation.source_provenance == "synthetic_ground_truth":
            return lost("ground_truth_observation")
        geometry_reason = self._geometry_failure(observation)
        if geometry_reason is not None:
            return lost(geometry_reason)
        if self._previous is not None:
            if (
                observation.stamp_ns - self._previous.stamp_ns
                > cfg.max_frame_interval_ns
            ):
                return lost("observation_interval_exceeded")
            if (
                self._geometry_jump(observation, self._previous)
                > cfg.max_geometry_jump_m
            ):
                return lost("geometry_jump")
        self._previous = observation
        if not self._tracking:
            if abs(speed) >= cfg.stopped_speed_mps:
                self._confirmed = 0
                return command("acquiring", "waiting_for_measured_stop")
            self._confirmed += 1
            if self._confirmed < cfg.confirmation_frames:
                return command("acquiring", "confirming_fresh_observations")
            self._tracking = True
        front_x = (observation.left.center_m[0] + observation.right.center_m[0]) / 2
        distance = front_x - cfg.target_front_x_m
        if (
            cfg.target_front_x_m - cfg.front_tolerance_m
            <= front_x
            <= cfg.target_front_x_m + cfg.front_tolerance_m
        ):
            if abs(speed) < cfg.stopped_speed_mps:
                self._completed = True
                return command("complete", "observed_goal_and_measured_stop")
            return command("tracking", "stopping_at_observed_goal")
        if self._completed:
            return lost("completed_goal_moved")
        lateral = (observation.left.center_m[1] + observation.right.center_m[1]) / 2
        curvature = (
            cfg.lateral_gain_inv_m2 * lateral
            + cfg.yaw_gain_inv_m * observation.insertion_yaw_rad
        )
        curvature = max(
            -cfg.max_curvature_inv_m, min(curvature, cfg.max_curvature_inv_m)
        )
        speed_limit = (
            cfg.cruise_speed_mps
            if front_x > cfg.slowdown_front_x_m
            else cfg.insertion_speed_mps
        )
        target_speed = min(speed_limit, cfg.approach_gain_inv_s * distance)
        return command("tracking", "fresh_observation", target_speed, curvature)

    def _geometry_failure(self, observation: PocketObservation) -> str | None:
        cfg = self.config
        left, right = observation.left, observation.right
        yaw = observation.insertion_yaw_rad
        front_x = (left.center_m[0] + right.center_m[0]) / 2
        lateral = (left.center_m[1] + right.center_m[1]) / 2
        if (
            not cfg.target_front_x_m - cfg.front_tolerance_m
            <= front_x
            <= cfg.max_start_front_x_m
        ):
            return "front_distance_outside_envelope"
        if abs(yaw) > cfg.max_yaw_error_rad:
            return "yaw_outside_envelope"
        if abs(lateral) > cfg.max_lateral_error_m:
            return "lateral_error_outside_envelope"
        for pocket, fork_y in (
            (left, cfg.fork_center_y_m),
            (right, -cfg.fork_center_y_m),
        ):
            # A horizontal fork strip intersects a yawed front plane with width
            # enlarged by 1/cos(yaw) in the opening's lateral coordinates.
            required_half_width = (
                abs(fork_y - pocket.center_m[1]) + cfg.fork_width_m / 2
            ) / math.cos(yaw)
            if required_half_width + cfg.clearance_margin_m >= pocket.width_m / 2:
                return "insufficient_lateral_clearance"
            bottom = pocket.center_m[2] - pocket.height_m / 2
            top = pocket.center_m[2] + pocket.height_m / 2
            if (
                bottom + cfg.clearance_margin_m >= cfg.fork_bottom_z_m
                or top - cfg.clearance_margin_m <= cfg.fork_top_z_m
            ):
                return "insufficient_vertical_clearance"
        return None

    @staticmethod
    def _geometry_jump(
        current: PocketObservation, previous: PocketObservation
    ) -> float:
        return max(
            max(
                math.dist(new.center_m, old.center_m),
                abs(new.width_m - old.width_m),
                abs(new.height_m - old.height_m),
            )
            for new, old in (
                (current.left, previous.left),
                (current.right, previous.right),
            )
        )
