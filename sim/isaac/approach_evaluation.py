"""SDK-free, downstream truth evaluation of a completed synthetic approach.

Call only after execution. Ground-truth pallet pose belongs to this evaluation
boundary and must never supply perception goals or path-following commands.
Reaching an observed target is distinct from passing these alignment checks.
"""

import math
from numbers import Real

import numpy as np
from numpy.typing import ArrayLike, NDArray

from forklift_core.planning.pallet_mission import (
    PalletSite,
    SyntheticMissionGeometry,
    site_poses,
)


def _finite_scalar(value: float, name: str) -> float:
    if (
        isinstance(value, (bool, np.bool_))
        or not isinstance(value, Real)
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be a finite real number")
    return float(value)


def _finite_array(value: ArrayLike, name: str) -> NDArray[np.float64]:
    array = np.asarray(value)
    if array.dtype.kind not in "fiu" or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain finite real numbers")
    return array.astype(np.float64, copy=False)


def _yaw_error(first: float, second: float) -> float:
    # Reduce separately before subtraction to avoid overflowing finite angles.
    return abs((first % math.tau - second % math.tau + math.pi) % math.tau - math.pi)


def evaluate_alignment(
    *,
    final_rear_pose: ArrayLike,
    observed_site: PalletSite,
    pallet_position_m: ArrayLike,
    pallet_yaw_rad: float,
    initial_pallet_xy_m: ArrayLike,
    geometry: SyntheticMissionGeometry | None = None,
    max_rear_position_error_m: float = 0.04,
    max_rear_yaw_error_rad: float = 0.03,
    max_detected_center_error_m: float = 0.03,
    max_detected_yaw_error_rad: float = 0.02,
    max_pallet_displacement_m: float = 0.018,
) -> dict:
    """Evaluate actual alignment, detection error, and planar pallet motion.

    All positions use metres in one common world frame; yaw uses radians.
    final_rear_pose is [x, y, yaw] with shape (3,), or a nonempty (N, 3)
    trajectory whose final row is evaluated. Every supplied row must be finite.
    pallet_position_m is the actual final (3,) XYZ vector; initial_pallet_xy_m
    is the actual initial (2,) XY vector. Displacement is planar, not vertical.

    The five maximum errors are explicit synthetic evaluation defaults, not
    measured robot safety tolerances. All must pass; the boundary is inclusive
    within 1e-12 numerical roundoff. Return JSON-compatible success, absolute
    errors, and applied thresholds. Invalid shapes, numbers, or negative
    thresholds raise ValueError. This gate does not certify pocket clearance,
    contact forces, or tracking safety during the preceding trajectory.
    """
    rear = _finite_array(final_rear_pose, "final_rear_pose")
    if rear.ndim == 2 and rear.shape[1:] == (3,) and len(rear):
        rear = rear[-1]
    if rear.shape != (3,):
        raise ValueError("final_rear_pose must have shape (3,) or nonempty (N, 3)")
    position = _finite_array(pallet_position_m, "pallet_position_m")
    initial = _finite_array(initial_pallet_xy_m, "initial_pallet_xy_m")
    if position.shape != (3,) or initial.shape != (2,):
        raise ValueError(
            "pallet_position_m requires (3,) and initial_pallet_xy_m requires (2,)"
        )
    yaw = _finite_scalar(pallet_yaw_rad, "pallet_yaw_rad")
    if not isinstance(observed_site, PalletSite):
        raise ValueError("observed_site must be a PalletSite")
    thresholds = {
        "rear_position_m": max_rear_position_error_m,
        "rear_yaw_rad": max_rear_yaw_error_rad,
        "detected_center_m": max_detected_center_error_m,
        "detected_yaw_rad": max_detected_yaw_error_rad,
        "pallet_displacement_m": max_pallet_displacement_m,
    }
    thresholds = {
        name: _finite_scalar(value, name) for name, value in thresholds.items()
    }
    if any(value < 0 for value in thresholds.values()):
        raise ValueError("maximum evaluation errors must be nonnegative")
    true_site = PalletSite(float(position[0]), float(position[1]), yaw)
    true_goal = site_poses(true_site, geometry)["approach"]
    errors = {
        "rear_position_m": math.hypot(
            float(rear[0]) - true_goal.x_m, float(rear[1]) - true_goal.y_m
        ),
        "rear_yaw_rad": _yaw_error(float(rear[2]), yaw),
        "detected_center_m": math.hypot(
            observed_site.x_m - float(position[0]),
            observed_site.y_m - float(position[1]),
        ),
        "detected_yaw_rad": _yaw_error(observed_site.yaw_rad, yaw),
        "pallet_displacement_m": math.hypot(
            float(position[0]) - float(initial[0]),
            float(position[1]) - float(initial[1]),
        ),
    }
    if not all(math.isfinite(value) for value in errors.values()):
        raise ValueError("evaluation errors exceed finite numerical range")
    success = all(
        error <= thresholds[name]
        or math.isclose(error, thresholds[name], rel_tol=0, abs_tol=1e-12)
        for name, error in errors.items()
    )
    return {"success": success, "errors": errors, "thresholds": thresholds}
