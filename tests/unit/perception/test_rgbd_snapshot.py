"""SDK-buffer ownership and explicit metric/grid contracts for RGB-D capture."""

from dataclasses import replace

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception.rgbd_snapshot import scene_input_from_rgbd_snapshot
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import PinholeIntrinsics, deproject_depth_pixels


def snapshot_arguments():
    return {
        "rgb": np.full((2, 3, 3), 73, dtype=np.uint8),
        "depth_m": np.full((2, 3), 1.2345678912345, dtype=np.float64),
        "intrinsics": PinholeIntrinsics(3, 2, 2.0, 2.0, 0.0, 0.0, "camera_optical"),
        "base_from_optical": RigidTransform(
            "camera_optical",
            "base_link",
            [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
            [0.2, 0, 0.5],
        ),
        "pixel_frame": "camera_optical",
        "stamp_ns": np.int64(123_456_789),
        "clock_domain": "synthetic",
        "source_provenance": "synthetic",
        "rectified": True,
        "rgb_registered_to_depth_grid": True,
        "depth_kind": "optical_axis_z",
        "depth_unit": "m",
    }


@pytest.mark.parametrize("dtype", [np.float32, np.float64])
def test_snapshot_owns_reused_sdk_buffers_without_depth_quantization(dtype):
    kwargs = snapshot_arguments()
    kwargs["depth_m"] = kwargs["depth_m"].astype(dtype)
    original_value = float(kwargs["depth_m"][0, 0])
    given = scene_input_from_rgbd_snapshot(**kwargs)
    assert isinstance(given, SceneInput)
    assert given.depth_m.dtype == np.float64
    assert given.depth_m[0, 0] == original_value
    assert given.depth_m[0, 0] != round(original_value, 3)
    kwargs["rgb"].fill(0)
    kwargs["depth_m"].fill(9)
    np.testing.assert_array_equal(given.rgb, np.full((2, 3, 3), 73, np.uint8))
    np.testing.assert_array_equal(given.depth_m, np.full((2, 3), original_value))
    assert given.pixel_frame == "camera_optical"
    assert given.stamp_ns == 123_456_789 and type(given.stamp_ns) is int
    assert given.clock_domain == "synthetic"
    assert given.source_provenance == "synthetic"


def test_invalid_depth_samples_become_unknown_without_mutating_source():
    kwargs = snapshot_arguments()
    depth = np.array([[0, -0.1, np.inf], [-np.inf, np.nan, 1.23456789]])
    kwargs["depth_m"] = depth
    given = scene_input_from_rgbd_snapshot(**kwargs)
    assert np.isnan(given.depth_m.ravel()[:5]).all()
    assert given.depth_m[1, 2] == 1.23456789
    assert depth[0, 0] == 0 and depth[0, 1] == -0.1 and np.isinf(depth[0, 2])


def test_metric_axial_depth_deprojects_and_transforms_without_rescaling():
    given = scene_input_from_rgbd_snapshot(**snapshot_arguments())
    optical = deproject_depth_pixels(
        given.depth_m,
        np.array([[2, 1]]),
        given.intrinsics,
        meters_per_unit=1.0,
        pixel_frame=given.pixel_frame,
        rectified=given.rectified,
    )
    base = given.base_from_optical.apply(optical)
    np.testing.assert_allclose(
        base.xyz_m[0],
        [1.4345678912345, -1.2345678912345, -0.11728394561725],
        rtol=0,
        atol=1e-14,
    )


@pytest.mark.parametrize(
    "key,value",
    [
        ("rectified", False),
        ("rectified", 1),
        ("rectified", np.bool_(True)),
        ("rgb_registered_to_depth_grid", False),
        ("rgb_registered_to_depth_grid", "yes"),
        ("depth_kind", "range"),
        ("depth_kind", "distance_to_camera"),
        ("depth_unit", "mm"),
        ("depth_unit", None),
        ("pixel_frame", "different_camera"),
        ("stamp_ns", True),
        ("stamp_ns", -1),
        ("stamp_ns", 1.5),
        ("clock_domain", "unknown_clock"),
        ("clock_domain", None),
        ("source_provenance", "live"),
        ("source_provenance", "synthetic_ground_truth"),
        ("intrinsics", None),
        ("base_from_optical", None),
    ],
)
def test_snapshot_rejects_unsupported_or_implicit_contract(key, value):
    kwargs = snapshot_arguments()
    kwargs[key] = value
    with pytest.raises(ValueError):
        scene_input_from_rgbd_snapshot(**kwargs)


@pytest.mark.parametrize(
    "key,value",
    [
        ("rgb", np.zeros((2, 3, 4), np.uint8)),
        ("rgb", np.zeros((3, 2, 3), np.uint8)),
        ("rgb", np.zeros((2, 3, 3), np.float32)),
        ("depth_m", np.zeros((3, 2), np.float32)),
        ("depth_m", np.zeros((2, 3, 1), np.float32)),
        ("depth_m", np.zeros((2, 3), np.uint16)),
        ("depth_m", np.zeros((2, 3), np.complex128)),
    ],
)
def test_snapshot_rejects_grid_or_buffer_type_mismatch(key, value):
    kwargs = snapshot_arguments()
    kwargs[key] = value
    with pytest.raises(ValueError):
        scene_input_from_rgbd_snapshot(**kwargs)


@pytest.mark.parametrize(
    "source,target",
    [
        ("other_optical", "base_link"),
        ("camera_optical", "map"),
    ],
)
def test_snapshot_rejects_transform_frame_mismatch(source, target):
    kwargs = snapshot_arguments()
    kwargs["base_from_optical"] = replace(
        kwargs["base_from_optical"], source_frame=source, target_frame=target
    )
    with pytest.raises(ValueError):
        scene_input_from_rgbd_snapshot(**kwargs)


def test_equal_rgb_depth_shapes_do_not_override_calibration_dimensions():
    kwargs = snapshot_arguments()
    kwargs["intrinsics"] = replace(kwargs["intrinsics"], width=4)
    with pytest.raises(ValueError):
        scene_input_from_rgbd_snapshot(**kwargs)


@pytest.mark.parametrize(
    "key", ["rectified", "rgb_registered_to_depth_grid", "depth_kind", "depth_unit"]
)
def test_grid_and_metric_declarations_cannot_be_omitted(key):
    kwargs = snapshot_arguments()
    del kwargs[key]
    with pytest.raises(TypeError):
        scene_input_from_rgbd_snapshot(**kwargs)
