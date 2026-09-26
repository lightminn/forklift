"""A minority of nearby surfaces must not move the dominant front plane."""

import numpy as np
import pytest

from forklift_core.perception import pocket_detector as detector


@pytest.mark.parametrize("yaw_rad", [0.0, 0.3])
def test_nearby_returns_do_not_push_the_front_plane_behind_its_support(yaw_rad):
    # Ten returns from a surface 6 mm behind a 101-point face remain inside
    # the EPAL 6 inlier band. Their mean offset would exclude coplanar lower
    # evidence even though the face itself is exact.
    front = np.column_stack(
        (np.full(101, 2.0), np.linspace(-0.4, 0.4, 101), np.full(101, 0.06))
    )
    behind = np.column_stack((np.full(10, 2.006), np.zeros(10), np.full(10, 0.12)))
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    plane = detector._refit_vertical(
        np.vstack((front, behind)) @ rotation.T, np.zeros(3), median_offset=True
    )

    np.testing.assert_allclose(plane.normal, [-c, -s, 0.0], atol=1e-12)
    assert np.dot(plane.point, [c, s, 0.0]) == pytest.approx(2.0, abs=1e-12)
    # The residual must describe the relocated plane, including the minority.
    assert plane.residual_p95_m == pytest.approx(0.006, abs=1e-12)

    # The acquisition default preserves the original mean-anchor behavior.
    original = detector._refit_vertical(
        np.vstack((front, behind)) @ rotation.T, np.zeros(3)
    )
    assert np.dot(original.point, [c, s, 0.0]) == pytest.approx(
        2.0 + 0.006 * 10 / 111, abs=1e-12
    )


@pytest.mark.parametrize("invalid", [0, 1, 0.0, 1.0, "true", None, np.bool_(True)])
def test_median_plane_offset_requires_an_explicit_boolean(invalid):
    with pytest.raises(ValueError, match="median_plane_offset"):
        detector.DetectorParams(median_plane_offset=invalid)
