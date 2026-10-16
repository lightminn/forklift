"""The wheel-speed chain the mission core and the arbiter share (M-b plan B8; design 4.4 order).

From the core's ``assemble_command``, unchanged in arithmetic: the steering creep (a quarter of the target while the
steering catches up), the slew (braking at the stopping model's deceleration, pulling away at the command
acceleration, a sign change goes through zero first), the final permission cap cut as a step, and the stop to zero.
Kept in one place so a teleoperated command brakes exactly as an autonomous one does.
"""

from __future__ import annotations

import math

import numpy as np

CREEP_STEERING_ERROR_RAD = 0.05
CREEP_FACTOR = 0.25


def creep(target_mps: float, steering_error_rad: float) -> float:
    """A quarter of the target while the steering is more than 0.05 rad from where it is going."""
    return target_mps * CREEP_FACTOR if steering_error_rad > CREEP_STEERING_ERROR_RAD else target_mps


def slew_cap_stop(previous_mps: float, target_mps: float, *, decel_mps2: float, accel_mps2: float, dt: float,
                  cap_mps: float | None = None, stop: bool = False) -> float:
    """One tick of the slew from ``previous_mps`` toward ``target_mps``, then the cap (a step), then the stop."""
    target = target_mps
    if previous_mps * target < 0:
        target = 0.0
    rate = (decel_mps2 if abs(target) < abs(previous_mps) else accel_mps2) * dt
    speed = previous_mps + float(np.clip(target - previous_mps, -rate, rate))
    if cap_mps is not None and abs(speed) > cap_mps:
        speed = math.copysign(cap_mps, speed)
    if stop:
        speed = 0.0
    return speed
