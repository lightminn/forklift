"""RGB-D rig geometry and depth noise (docs/plans/2026-10-07-visual-slam-and-fusion.md)."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw
from forklift_core.sensors.camera_rig import (
    DepthNoise,
    base_from_optical_rotation,
    pixel_grid,
    rig_from_config,
)

ROOT = Path(__file__).resolve().parents[3]


def _config():
    return yaml.safe_load((ROOT / "config/isaac_depth_rig.yaml").read_text())


def test_forward_camera_matches_the_perception_rig_optical_axes():
    # tools/scene_rig.py OPTICAL_QUATERNION_XYZW: the canonical forward camera.
    expected = rotation_matrix_from_quaternion_xyzw((-0.5, 0.5, -0.5, 0.5))
    np.testing.assert_allclose(
        base_from_optical_rotation(0.0, 0.0), expected, atol=1e-12
    )


def test_optical_z_points_where_each_camera_looks():
    rig = rig_from_config(_config())
    looks = {"front": (1, 0), "rear": (-1, 0), "left": (0, 1), "right": (0, -1)}
    for camera in rig.cameras:
        z_axis = camera.rotation_base_from_optical[:, 2]
        horizontal = np.hypot(z_axis[0], z_axis[1])
        np.testing.assert_allclose(
            z_axis[:2] / horizontal, looks[camera.name], atol=1e-12
        )
        # 15 deg below the horizon; image "down" (optical +y) has a -z component.
        assert math.degrees(math.atan2(-z_axis[2], horizontal)) == pytest.approx(15.0)
        assert camera.rotation_base_from_optical[2, 1] < 0


def test_principal_point_is_the_integer_index_image_centre():
    camera = rig_from_config(_config()).camera("front")
    k = camera.intrinsics
    assert (k.width, k.height, k.cx, k.cy) == (640, 480, 319.5, 239.5)
    assert k.fx == k.fy == pytest.approx(465.741156)
    assert math.degrees(2 * math.atan(320 / k.fx)) == pytest.approx(69.0, abs=0.05)


def test_centre_pixel_ray_is_the_optical_axis():
    camera = rig_from_config(_config()).camera("left")
    ray = camera.ray_directions_base(np.array([[319.5, 239.5]]))[0]
    np.testing.assert_allclose(ray, camera.rotation_base_from_optical[:, 2], atol=1e-12)


def test_unknown_and_missing_keys_are_refused():
    config = _config()
    config["exposure"] = 1
    with pytest.raises(ValueError, match="keys"):
        rig_from_config(config)
    config = _config()
    del config["cameras"]["front"]["pitch_down_deg"]
    with pytest.raises(ValueError, match="camera front"):
        rig_from_config(config)


def test_pixel_grid_stays_inside_the_image():
    k = rig_from_config(_config()).camera("front").intrinsics
    uv = pixel_grid(k, 8, 6)
    assert uv.shape == (48, 2)
    assert uv[:, 0].min() > 0 and uv[:, 0].max() < k.width - 1
    assert uv[:, 1].min() > 0 and uv[:, 1].max() < k.height - 1


def test_depth_noise_is_keyed_by_seed_camera_and_frame():
    depth = np.full((48, 64), 3.0)
    noise = DepthNoise(coefficient_per_m=0.0036, range_m=(0.28, 8.0), seed=4)
    a = noise.depth_mm(depth, 1, 10)
    np.testing.assert_array_equal(a, noise.depth_mm(depth, 1, 10))
    assert not np.array_equal(a, noise.depth_mm(depth, 2, 10))
    assert not np.array_equal(a, noise.depth_mm(depth, 1, 11))
    # sigma at 3 m = 0.0036 * 9 = 32.4 mm.
    assert np.std(a.astype(float)) == pytest.approx(32.4, rel=0.1)


def test_depth_outside_range_or_nonfinite_reads_zero():
    depth = np.array([[0.1, 0.5, 9.0, np.inf, np.nan, 2.0004]])
    out = DepthNoise(
        coefficient_per_m=0.0036, range_m=(0.28, 8.0), seed=0, enabled=False
    ).depth_mm(depth, 0, 0)
    assert out.dtype == np.uint16
    np.testing.assert_array_equal(out, [[0, 500, 0, 0, 0, 2000]])


def test_clipping_must_bracket_the_depth_range():
    config = _config()
    config["clipping_range_m"] = [1.0, 100.0]  # the USD default near plane
    with pytest.raises(ValueError, match="clipping_range_m"):
        rig_from_config(config)
    assert rig_from_config(_config()).clipping_range_m == (0.05, 100.0)
