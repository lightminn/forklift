"""Candidate evidence must survive point removal by earlier RANSAC planes."""

import numpy as np
import pytest

from forklift_core.perception.pocket_detector import (
    DetectorParams,
    _vertical_plane_candidates,
)


def test_later_plane_keeps_shared_inliers_and_their_residuals():
    """Intersecting synthetic walls reproduce point stealing without a dataset.

    The larger x=2 wall is found first. It consumes the x=2.01 column (20 points) of
    the y=0 wall, plus its own 120 points within 10 mm of y=0. The later
    plane must retain all 1120 inliers, including those 140 shared points.
    """
    y, z = np.meshgrid(
        np.r_[
            np.linspace(-1.0, -0.05, 40),
            [-0.01, 0.0, 0.01],
            np.linspace(0.05, 1.0, 40),
        ],
        np.linspace(0.05, 0.25, 40),
    )
    first_wall = np.column_stack((np.full(y.size, 2.0), y.ravel(), z.ravel()))
    x, z = np.meshgrid(np.linspace(2.01, 2.99, 50), np.linspace(0.05, 0.25, 20))
    later_wall = np.column_stack((x.ravel(), np.zeros(x.size), z.ravel()))
    points = np.vstack((first_wall, later_wall))

    planes = _vertical_plane_candidates(
        points, np.array([0.0, -2.0, 0.2]), DetectorParams()
    )

    assert len(planes) == 2
    np.testing.assert_allclose(planes[0].normal, [-1.0, 0.0, 0.0], atol=1e-12)
    assert len(planes[0].points) == 3340
    np.testing.assert_allclose(planes[1].normal, [0.0, -1.0, 0.0], atol=1e-12)
    assert len(planes[1].points) == 1120
    assert planes[1].residual_p95_m == pytest.approx(0.01)
