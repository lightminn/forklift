"""Pose-by-pose error of an estimated planar trajectory against ground truth.

Two alignments are reported because they answer different questions. The
least-squares rigid fit (no scale) is the usual absolute trajectory error: it
removes the arbitrary map origin and measures shape. Aligning the first poses
only is what a robot that knows its start pose would see: it keeps the heading
error at the start, which least squares would partly hide.
"""

from dataclasses import dataclass
from math import atan2, cos, sin

import numpy as np
from numpy.typing import ArrayLike, NDArray

from forklift_core._validation import _real_array


@dataclass(frozen=True)
class TrajectoryError:
    """Metres and radians. truth_from_estimate is (dx, dy, dtheta)."""

    samples: int
    path_length_m: float
    ate_rmse_m: float
    ate_max_m: float
    first_pose_ate_rmse_m: float
    yaw_rmse_rad: float
    final_error_m: float
    final_drift_ratio: float
    truth_from_estimate: tuple[float, float, float]


def _apply(transform, poses):
    dx, dy, dtheta = transform
    c, s = cos(dtheta), sin(dtheta)
    return np.column_stack(
        (
            c * poses[:, 0] - s * poses[:, 1] + dx,
            s * poses[:, 0] + c * poses[:, 1] + dy,
            poses[:, 2] + dtheta,
        )
    )


def _least_squares(estimate: NDArray, truth: NDArray) -> tuple[float, float, float]:
    mean_e, mean_t = estimate[:, :2].mean(axis=0), truth[:, :2].mean(axis=0)
    e, t = estimate[:, :2] - mean_e, truth[:, :2] - mean_t
    theta = atan2(
        float(np.sum(e[:, 0] * t[:, 1] - e[:, 1] * t[:, 0])),
        float(np.sum(e[:, 0] * t[:, 0] + e[:, 1] * t[:, 1])),
    )
    c, s = cos(theta), sin(theta)
    return (
        float(mean_t[0] - (c * mean_e[0] - s * mean_e[1])),
        float(mean_t[1] - (s * mean_e[0] + c * mean_e[1])),
        theta,
    )


def _first_pose(estimate: NDArray, truth: NDArray) -> tuple[float, float, float]:
    theta = float(truth[0, 2] - estimate[0, 2])
    c, s = cos(theta), sin(theta)
    return (
        float(truth[0, 0] - (c * estimate[0, 0] - s * estimate[0, 1])),
        float(truth[0, 1] - (s * estimate[0, 0] + c * estimate[0, 1])),
        theta,
    )


def trajectory_error(estimate: ArrayLike, truth: ArrayLike) -> TrajectoryError:
    """Compare (N, 3) x, y, yaw poses sampled at the same N instants.

    The caller pairs the samples in time; this function never interpolates.
    Final error and drift use the least-squares alignment.
    """
    estimate = _real_array(estimate, "estimate")
    truth = _real_array(truth, "truth")
    if estimate.ndim != 2 or estimate.shape[1] != 3 or estimate.shape != truth.shape:
        raise ValueError("estimate and truth must both be (N, 3)")
    if len(truth) < 2:
        raise ValueError("at least two paired poses are required")
    if not (np.isfinite(estimate).all() and np.isfinite(truth).all()):
        raise ValueError("trajectories must be finite")
    fit = _least_squares(estimate, truth)
    aligned = _apply(fit, estimate)
    residual = np.hypot(*(aligned[:, :2] - truth[:, :2]).T)
    first = _apply(_first_pose(estimate, truth), estimate)
    first_residual = np.hypot(*(first[:, :2] - truth[:, :2]).T)
    yaw_error = np.angle(np.exp(1j * (aligned[:, 2] - truth[:, 2])))
    length = float(np.sum(np.hypot(*np.diff(truth[:, :2], axis=0).T)))
    return TrajectoryError(
        samples=len(truth),
        path_length_m=length,
        ate_rmse_m=float(np.sqrt(np.mean(residual**2))),
        ate_max_m=float(residual.max()),
        first_pose_ate_rmse_m=float(np.sqrt(np.mean(first_residual**2))),
        yaw_rmse_rad=float(np.sqrt(np.mean(yaw_error**2))),
        final_error_m=float(residual[-1]),
        final_drift_ratio=float(residual[-1] / length) if length > 0 else 0.0,
        truth_from_estimate=fit,
    )
