"""Vehicle kinematics injected into the mission core (sim-to-real P0-4 4a',
docs/design/2026-11-11-p04a-kinematics-and-command.md D1).

``AckermannKinematics`` is the simulation's: every operation is the arithmetic the core and runner use today,
in the same order (the replay gate holds it to bit equality), with no refusal. ``RealKinematics`` is the real
truck's: two driven rear wheels and one steering mechanism angle q (K6, ``config/real/vehicle.yaml``), a
refusal for a curvature its steering map cannot reach, and one steering state slewed at its own rate.

Steering vectors have ``steering_dim`` entries (2: the front wheels' angles; 1: q). A curvature the
kinematics cannot tell (an invalid or out-of-map q) is ``None``, never 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.control.path_tracking import AckermannGeometry, ackermann_command
from forklift_core.control.speed_chain import CREEP_STEERING_ERROR_RAD
from forklift_core.plant.real_vehicle import (
    DriveRefusal,
    RealVehicleConfig,
    RearDrive,
    rear_drive_command,
)

# The scene replan's presteer tolerance (mission/scene_replan.py SceneReplanConfig default), restated here
# without importing the mission package into the plant package.
SIM_PRESTEER_TOLERANCE_RAD = 0.02


def _vec(steer, dim: int, name: str = "steering") -> np.ndarray:
    v = np.asarray(steer, dtype=float)
    if v.shape != (dim,):
        raise ValueError(f"{name} must have shape ({dim},), has {v.shape}")
    return v


class AckermannKinematics:
    """The simulation's front-steered Ackermann truck: today's arithmetic, no refusal."""

    steering_dim = 2
    creep_threshold_rad = CREEP_STEERING_ERROR_RAD
    presteer_threshold_rad = SIM_PRESTEER_TOLERANCE_RAD

    def __init__(self, geometry: AckermannGeometry, steering_rate_rad_s) -> None:
        """``steering_rate_rad_s``: a number, or a callable read at each slew (the core passes its settings'
        ``steering_command_rate_rad_s``, read when it slews, as before)."""
        self.geometry = geometry
        self._rate = steering_rate_rad_s

    @property
    def steering_rate_rad_s(self) -> float:
        return self._rate() if callable(self._rate) else self._rate

    def command(self, speed_mps: float, curvature_inv_m: float):
        return ackermann_command(speed_mps, curvature_inv_m, self.geometry)

    def steering_targets(self, curvature_inv_m: float):
        return ackermann_command(0.0, float(curvature_inv_m), self.geometry).steering_rad

    def slew_steering(self, steering_command: np.ndarray, target, dt: float) -> None:
        rate = self.steering_rate_rad_s * dt
        steering_command += np.clip(target - steering_command, -rate, rate)

    def steering_error(self, target, actual) -> float:
        return float(np.max(np.abs(target - actual)))

    def _per_wheel(self, steer) -> tuple[float, float]:
        s = _vec(steer, 2)
        h, wb = self.geometry.track_m / 2, self.geometry.wheelbase_m
        return (math.tan(s[0]) / (wb + math.tan(s[0]) * h), math.tan(s[1]) / (wb - math.tan(s[1]) * h))

    def held_curvature(self, steer) -> float:
        """The mean of the two wheels' curvatures (the runner's held_kappa)."""
        return float(np.mean(list(self._per_wheel(steer))))

    def curvature_bound(self, steer) -> float:
        """The larger of the two wheels' |curvature| (the near protection's)."""
        a, b = self._per_wheel(steer)
        return max(abs(a), abs(b))

    def finalize(self, drive, steering_command):
        return drive


@dataclass(frozen=True)
class RealDrive:
    """The real truck's command in the core's shape: one steering entry, the two rear wheels."""

    speed_mps: float
    curvature_inv_m: float
    steering_rad: tuple[float]
    wheel_rates_rad_s: tuple[float, float]
    speed_limited: bool

    @classmethod
    def of(cls, d: RearDrive) -> RealDrive:
        return cls(d.speed_mps, d.curvature_inv_m, (d.steering_rad,), (d.left_wheel_rad_s, d.right_wheel_rad_s),
                   d.speed_limited)

    def rear(self) -> RearDrive:
        return RearDrive(self.speed_mps, self.curvature_inv_m, self.wheel_rates_rad_s[0], self.wheel_rates_rad_s[1],
                         self.steering_rad[0], self.speed_limited)


class RealKinematics:
    """The real truck: rear-wheel electronic differential and one mechanism angle (K6)."""

    steering_dim = 1

    def __init__(self, cfg: RealVehicleConfig) -> None:
        self.cfg = cfg
        sc = cfg.steering_control
        self.steering_rate_rad_s = sc["steering_rate_rad_s"]
        self.creep_threshold_rad = sc["creep_threshold_rad"]
        self.presteer_threshold_rad = sc["presteer_threshold_rad"]
        self.straight_threshold_rad = sc["straight_threshold_rad"]

    def command(self, speed_mps: float, curvature_inv_m: float) -> RealDrive | DriveRefusal:
        out = rear_drive_command(speed_mps, curvature_inv_m, self.cfg)
        return out if isinstance(out, DriveRefusal) else RealDrive.of(out)

    def steering_targets(self, curvature_inv_m: float):
        q = self.cfg.steering.angle(float(curvature_inv_m))
        return None if q is None else (q,)

    def slew_steering(self, steering_command: np.ndarray, target, dt: float) -> None:
        _vec(steering_command, 1, "steering_command")
        tgt = _vec(target, 1, "target")
        rate = self.steering_rate_rad_s * dt
        steering_command += np.clip(tgt - steering_command, -rate, rate)

    def steering_error(self, target, actual) -> float:
        return float(abs(_vec(target, 1, "target")[0] - _vec(actual, 1, "actual")[0]))

    def held_curvature(self, steer) -> float | None:
        return self.cfg.steering.curvature(float(_vec(steer, 1)[0]))

    def curvature_bound(self, steer) -> float | None:
        k = self.held_curvature(steer)
        return None if k is None else abs(k)

    def stopped(self, steering_command) -> RealDrive:
        """Both wheels at zero with the steering held, needing no curvature (an unknown q included)."""
        q = float(_vec(steering_command, 1, "steering_command")[0])
        return RealDrive(0.0, 0.0, (q,), (0.0, 0.0), False)

    def finalize(self, drive: RealDrive, steering_command) -> RealDrive:
        """The wheels on the curvature the slewed q holds (speed kept, inside the wheel limit). An unknown q
        brakes both wheels to zero without a curvature (design D1)."""
        q = float(_vec(steering_command, 1, "steering_command")[0])
        k = self.cfg.steering.curvature(q)
        if k is None:
            return RealDrive(0.0, drive.curvature_inv_m, (q,), (0.0, 0.0), drive.speed_limited)
        out = rear_drive_command(drive.speed_mps, k, self.cfg)
        assert not isinstance(out, DriveRefusal)  # k is the map's own value
        return RealDrive(out.speed_mps, out.curvature_inv_m, (q,), (out.left_wheel_rad_s, out.right_wheel_rad_s),
                         out.speed_limited)


__all__ = ["AckermannKinematics", "RealDrive", "RealKinematics", "SIM_PRESTEER_TOLERANCE_RAD"]
