"""Trajectory error against ground truth; the fitted alignment is reported."""

import math

import numpy as np
import pytest

from forklift_core.localization.trajectory_error import trajectory_error


def loop(n=400):
    t = np.linspace(0, 2 * math.pi, n)
    return np.column_stack(
        (
            6 * np.cos(t),
            3 * np.sin(2 * t),
            np.unwrap(np.arctan2(6 * np.cos(2 * t), -6 * np.sin(t))),
        )
    )


def transformed(poses, dx, dy, dtheta):
    c, s = math.cos(dtheta), math.sin(dtheta)
    x = c * poses[:, 0] - s * poses[:, 1] + dx
    y = s * poses[:, 0] + c * poses[:, 1] + dy
    return np.column_stack((x, y, poses[:, 2] + dtheta))


def test_identical_trajectories_have_zero_error():
    truth = loop()
    result = trajectory_error(truth, truth)
    assert result.ate_rmse_m == pytest.approx(0.0, abs=1e-12)
    assert result.first_pose_ate_rmse_m == pytest.approx(0.0, abs=1e-12)
    assert result.path_length_m == pytest.approx(
        np.sum(np.hypot(*np.diff(truth[:, :2], axis=0).T))
    )


def test_rigid_offset_is_removed_by_least_squares_and_first_pose_alignment():
    truth = loop()
    estimate = transformed(truth, 3.0, -1.0, 0.7)
    result = trajectory_error(estimate, truth)
    assert result.ate_rmse_m == pytest.approx(0.0, abs=1e-9)
    assert result.first_pose_ate_rmse_m == pytest.approx(0.0, abs=1e-9)
    assert result.yaw_rmse_rad == pytest.approx(0.0, abs=1e-9)
    # truth = inverse of the applied transform: rotate by -0.7, then shift by
    # -R(-0.7) (3, -1).
    c, s = math.cos(-0.7), math.sin(-0.7)
    np.testing.assert_allclose(
        result.truth_from_estimate,
        [-(c * 3.0 - s * -1.0), -(s * 3.0 + c * -1.0), -0.7],
        atol=1e-9,
    )


def test_heading_error_after_the_start_shows_only_in_first_pose_alignment():
    truth = loop()
    # The whole path is rotated about its first point, but the first heading
    # is reported correctly: shape is right, orientation drifted at once.
    shifted = truth.copy()
    shifted[:, :2] -= truth[0, :2]
    estimate = transformed(shifted, *truth[0, :2], 0.05)
    estimate[0, 2] = truth[0, 2]
    result = trajectory_error(estimate, truth)
    assert result.ate_rmse_m == pytest.approx(0.0, abs=1e-9)
    assert result.first_pose_ate_rmse_m > 0.1


def test_final_error_and_drift_ratio_follow_the_last_pose():
    truth = loop()
    estimate = truth.copy()
    estimate[-1, 0] += 0.5
    result = trajectory_error(estimate, truth)
    # First-pose alignment is the identity here, so only the last sample errs.
    assert result.first_pose_ate_rmse_m == pytest.approx(0.5 / math.sqrt(len(truth)))
    # Least squares spreads that one outlier a little over the whole path.
    assert result.final_error_m == pytest.approx(0.5, abs=0.01)
    assert result.final_drift_ratio == pytest.approx(
        result.final_error_m / result.path_length_m
    )


@pytest.mark.parametrize(
    "estimate, truth",
    [
        (np.zeros((3, 3)), np.zeros((4, 3))),
        (np.zeros((3, 2)), np.zeros((3, 2))),
        (np.full((3, 3), np.nan), np.zeros((3, 3))),
        (np.zeros((1, 3)), np.zeros((1, 3))),
    ],
)
def test_malformed_trajectories_raise(estimate, truth):
    with pytest.raises(ValueError):
        trajectory_error(estimate, truth)
