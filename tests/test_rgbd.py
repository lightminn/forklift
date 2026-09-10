from importlib import import_module

import numpy as np
import pytest


def setup_camera(**overrides):
    rgbd = import_module("forklift_core.rgbd")
    values = dict(width=5, height=3, fx=2, fy=4, cx=2, cy=1, frame_id="depth_optical")
    values.update(overrides)
    return rgbd, rgbd.PinholeIntrinsics(**values)


def sample(rgbd, intrinsics, depth, pixels, **overrides):
    options = dict(meters_per_unit=0.001, pixel_frame="depth_optical", rectified=True)
    options.update(overrides)
    return rgbd.deproject_depth_pixels(depth, pixels, intrinsics, **options)


def test_depth_scale_and_optical_pixel_axes_have_literal_metric_coordinates():
    rgbd, intrinsics = setup_camera()
    result = sample(rgbd, intrinsics, np.full((3, 5), 2000), [[2, 1], [3, 1], [2, 2]])
    assert result.frame_id == "depth_optical"
    np.testing.assert_allclose(result.xyz_m, [[0, 0, 2], [1, 0, 2], [0, 0.5, 2]])


def test_float_depth_already_in_metres_is_not_rescaled_as_millimetres():
    rgbd, intrinsics = setup_camera()
    result = sample(rgbd, intrinsics, np.full((3, 5), 1.5), [[2, 1]], meters_per_unit=1)
    np.testing.assert_allclose(result.xyz_m, [[0, 0, 1.5]])


def test_invalid_depths_remain_missing_at_their_original_indices():
    rgbd, intrinsics = setup_camera()
    depth = np.full((3, 5), 1000.0)
    depth[0] = [0, -1, np.nan, np.inf, 1000]
    result = sample(rgbd, intrinsics, depth, [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]])
    np.testing.assert_array_equal(result.valid, [False, False, False, False, True])
    assert np.isnan(result.xyz_m[:4]).all()
    np.testing.assert_allclose(result.xyz_m[4], [1, -0.25, 1])


@pytest.mark.parametrize(
    "pixels",
    [[[-1, 1]], [[5, 1]], [[2, 3]], [[2, -1]], [[2.5, 1]], [[np.nan, 1]], [1, 2]],
)
def test_bad_pixel_indices_are_rejected_instead_of_wrapped_clamped_or_rounded(pixels):
    rgbd, intrinsics = setup_camera()
    with pytest.raises(ValueError):
        sample(rgbd, intrinsics, np.full((3, 5), 1000), pixels)


@pytest.mark.parametrize("scale", [0, -0.001, np.nan, np.inf])
def test_invalid_depth_scale_is_rejected(scale):
    rgbd, intrinsics = setup_camera()
    with pytest.raises(ValueError):
        sample(rgbd, intrinsics, np.ones((3, 5)), [[2, 1]], meters_per_unit=scale)


@pytest.mark.parametrize(
    "override",
    [
        dict(fx=0),
        dict(fy=-1),
        dict(cx=np.nan),
        dict(cy=np.inf),
        dict(width=0),
        dict(height=2.5),
        dict(frame_id=""),
    ],
)
def test_bad_calibration_is_rejected(override):
    with pytest.raises(ValueError):
        setup_camera(**override)


def test_color_pixels_cannot_silently_index_the_depth_grid():
    rgbd, intrinsics = setup_camera()
    with pytest.raises(ValueError):
        sample(rgbd, intrinsics, np.ones((3, 5)), [[2, 1]], pixel_frame="color_optical")


def test_unrectified_pixels_require_a_distortion_aware_adapter():
    rgbd, intrinsics = setup_camera()
    with pytest.raises(ValueError):
        sample(rgbd, intrinsics, np.ones((3, 5)), [[2, 1]], rectified=False)


@pytest.mark.parametrize(
    "depth", [np.ones((5, 3)), np.ones((3, 5, 3)), np.ones((3, 5), dtype=complex)]
)
def test_depth_grid_must_match_calibration_and_be_a_real_scalar_image(depth):
    rgbd, intrinsics = setup_camera()
    with pytest.raises(ValueError):
        sample(rgbd, intrinsics, depth, [[2, 1]])


def test_empty_pixel_selection_returns_empty_points():
    rgbd, intrinsics = setup_camera()
    result = sample(rgbd, intrinsics, np.ones((3, 5)), np.empty((0, 2), dtype=int))
    assert result.xyz_m.shape == (0, 3)
