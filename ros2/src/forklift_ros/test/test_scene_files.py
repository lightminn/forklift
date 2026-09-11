"""Independent producer fixtures; no ROS installation is needed."""

import hashlib
import importlib.util
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from PIL import Image


def load_module(name):
    path = Path(__file__).parents[1] / "forklift_ros" / f"{name}.py"
    assert path.exists(), f"{name} implementation is missing"
    spec = importlib.util.spec_from_file_location(f"tested_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def api():
    class PendingModule:
        def __getattr__(self, name):
            return getattr(load_module("scene_files"), name)

    return PendingModule()


def header(sec=2, nanosec=400_000_000, frame="camera_optical_frame"):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec))


def info():
    return NS(
        header=header(),
        width=3,
        height=2,
        distortion_model="plumb_bob",
        d=[0.0] * 5,
        k=[4.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[4.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x=0,
        binning_y=1,
        roi=NS(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )


def transform():
    return NS(
        header=header(sec=0, nanosec=0, frame="base_link"),
        child_frame_id="camera_optical_frame",
        transform=NS(
            translation=NS(x=0.75, y=0.0, z=0.5),
            rotation=NS(x=-0.5, y=0.5, z=-0.5, w=0.5),
        ),
    )


def message(encoding="rgb8"):
    data = (
        np.full((2, 3, 3), [20, 30, 40], dtype=np.uint8)
        if encoding == "rgb8"
        else np.full((2, 3), 2.25, dtype="<f4")
    )
    return NS(
        header=header(),
        width=3,
        height=2,
        encoding=encoding,
        is_bigendian=0,
        step=9 if encoding == "rgb8" else 12,
        data=data.tobytes(),
    )


def entry():
    return {
        "scene_id": "s001",
        "catalogue_version": "v1",
        "category": "positive",
        "split": "dev",
        "camera": {"width": 3, "height": 2},
        "visibility": {"left_in_view": True, "occluded_side": None},
        "ground_truth": {
            "stamp_ns": 0,
            "clock_domain": "synthetic",
            "source_provenance": "synthetic_ground_truth",
            "status": "valid",
            "frame_id": "base_link",
            "left": {"center_m": [2.0, 0.17, 0.15], "width_m": 0.24, "height_m": 0.2},
            "right": {"center_m": [2.0, -0.17, 0.15], "width_m": 0.24, "height_m": 0.2},
            "insertion_yaw_rad": 0.0,
            "position_sigma_m": 0.0,
            "yaw_sigma_rad": 0.0,
            "reason": None,
        },
    }


def payload(api):
    given = entry()
    return dict(
        rgb=np.full((2, 3, 3), [20, 30, 40], dtype=np.uint8),
        depth_m=np.array([[0.0, 1.25, np.nan], [2.5, 5.0, 6.0]]),
        camera_info=api.camera_info_to_json(info()),
        tf=api.tf_static_to_json(transform()),
        ground_truth=api.ground_truth_for_capture(given, 2_400_000_000),
        scene=api.build_scene_json(
            given, 2_400_000_000, "sha256:image", "a" * 64, "run-1", {"captured": 4.0}
        ),
    )


def test_stamp_uses_integer_header_nanoseconds(api):
    assert api.stamp_to_ns(header().stamp) == 2_400_000_000
    assert (
        api.stamp_to_ns(NS(sec=2_000_000_000, nanosec=7)) == 2_000_000_000_000_000_007
    )


@pytest.mark.parametrize(
    "stamp",
    [
        NS(sec=True, nanosec=0),
        NS(sec=-1, nanosec=0),
        NS(sec=1.0, nanosec=0),
        NS(sec=1, nanosec=10**9),
    ],
)
def test_invalid_header_stamp_is_rejected(api, stamp):
    with pytest.raises(ValueError):
        api.stamp_to_ns(stamp)


def test_depth_rounds_to_nearest_mm_and_preserves_input(api):
    depth = np.array([[1.2344, 1.2346, 0.0006, 65.535]])
    before = depth.copy()
    result = api.depth_to_millimetre_png_array(depth)
    assert result.dtype == np.uint16
    np.testing.assert_array_equal(result, [[1234, 1235, 1, 65535]])
    np.testing.assert_array_equal(depth, before)


def test_nonfinite_and_nonpositive_depth_is_unknown(api):
    depth = np.array([[np.nan, np.inf, -np.inf, 0.0, -1.0]])
    np.testing.assert_array_equal(api.depth_to_millimetre_png_array(depth), [[0] * 5])


def test_finite_depth_over_limit_fails(api):
    with pytest.raises(ValueError, match="65.535"):
        api.depth_to_millimetre_png_array(np.array([[65.536]]))


def test_float32_boundary_compares_actual_value_in_float64(api):
    # float32(65.535) is 65.53500366210938, strictly over the v1 limit.
    with pytest.raises(ValueError, match="65.535"):
        api.depth_to_millimetre_png_array(np.array([[65.535]], dtype=np.float32))
    below = np.nextafter(np.float32(65.535), np.float32(0))
    np.testing.assert_array_equal(
        api.depth_to_millimetre_png_array(np.array([[below]])), [[65535]]
    )


@pytest.mark.parametrize(
    "depth",
    [
        np.ones((2, 3), dtype=np.uint16),
        np.ones((2, 3), dtype=np.float16),
        np.ones((2, 3), dtype=complex),
        np.ones(3),
    ],
)
def test_depth_rejects_wrong_dtype_or_shape(api, depth):
    with pytest.raises(ValueError):
        api.depth_to_millimetre_png_array(depth)


@pytest.mark.parametrize("encoding", ["rgb8", "32FC1"])
def test_image_decodes_packed_little_endian_data(api, encoding):
    result = api.image_to_array(message(encoding), encoding)
    if encoding == "rgb8":
        assert result.dtype == np.uint8
        np.testing.assert_array_equal(result, np.full((2, 3, 3), [20, 30, 40]))
    else:
        assert result.dtype == np.float32
        np.testing.assert_array_equal(result, np.full((2, 3), 2.25))


@pytest.mark.parametrize("encoding", ["rgb8", "32FC1"])
@pytest.mark.parametrize(
    "field,value",
    [("encoding", "bgr8"), ("step", 99), ("data", b"short"), ("is_bigendian", 1)],
)
def test_image_rejects_invalid_wire_format(api, encoding, field, value):
    msg = message(encoding)
    setattr(msg, field, value)
    with pytest.raises(ValueError):
        api.image_to_array(msg, encoding)


def test_image_rejects_wrong_frame(api):
    msg = message()
    msg.header.frame_id = "camera_link"
    with pytest.raises(ValueError, match="frame"):
        api.image_to_array(msg, "rgb8")


def test_camera_info_preserves_all_twelve_fields_and_five_roi_fields(api):
    result = api.camera_info_to_json(info())
    assert set(result) == {
        "frame_id",
        "stamp_ns",
        "width",
        "height",
        "distortion_model",
        "d",
        "k",
        "r",
        "p",
        "binning_x",
        "binning_y",
        "roi",
    }
    assert result["roi"] == dict(
        x_offset=0, y_offset=0, height=0, width=0, do_rectify=False
    )
    assert result["stamp_ns"] == 2_400_000_000
    for name in ("d", "k", "r", "p", "binning_x", "binning_y"):
        assert result[name] == getattr(info(), name)
    json.dumps(result, allow_nan=False)


def test_camera_info_rejects_wrong_frame(api):
    msg = info()
    msg.header.frame_id = "base_link"
    with pytest.raises(ValueError, match="frame"):
        api.camera_info_to_json(msg)


def test_camera_info_rejects_nonfinite_values(api):
    msg = info()
    msg.k[0] = np.nan
    with pytest.raises(ValueError):
        api.camera_info_to_json(msg)


def test_received_tf_preserves_values_without_requiring_image_stamp(api):
    result = api.tf_static_to_json(transform())
    assert result == dict(
        target_frame="base_link",
        source_frame="camera_optical_frame",
        translation_m=[0.75, 0.0, 0.5],
        quaternion_xyzw=[-0.5, 0.5, -0.5, 0.5],
        origin="received_tf_static",
    )


@pytest.mark.parametrize("field", ["parent", "child"])
def test_tf_rejects_a_different_frame_pair(api, field):
    msg = transform()
    if field == "parent":
        msg.header.frame_id = "map"
    else:
        msg.child_frame_id = "lidar_link"
    with pytest.raises(ValueError, match="frame"):
        api.tf_static_to_json(msg)


def test_selection_skips_warmup_black_and_incomplete_sets(api):
    rgb = {n: np.ones((2, 3, 3), dtype=np.uint8) for n in [1, 2, 3, 4, 5]}
    rgb[2][:] = 0
    buffers = {
        "rgb": rgb,
        "depth": {n: np.ones((2, 3)) for n in [1, 2, 3, 4, 5]},
        "info": {n: {} for n in [1, 2, 4, 5]},
    }
    assert api.select_synchronized_set(buffers, 2) == 4


def test_no_synchronized_set_returns_none(api):
    assert api.select_synchronized_set({"rgb": {}, "depth": {}, "info": {}}, 2) is None


def test_scene_metadata_uses_synthetic_provenance_without_entry_top_level_provenance(
    api,
):
    given = entry()
    before = deepcopy(given)
    result = api.build_scene_json(
        given, 2_400_000_000, "sha256:image", "a" * 64, "run-1", {"captured": 4.0}
    )
    assert result == {
        **{k: v for k, v in given.items() if k != "ground_truth"},
        "stamp_ns": 2_400_000_000,
        "clock_domain": "ros_sim",
        "source_provenance": "synthetic",
        "image_id": "sha256:image",
        "source_snapshot_sha256": "a" * 64,
        "run_id": "run-1",
        "wall_times_s": {"captured": 4.0},
    }
    result["camera"]["width"] = 999
    assert given == before


def test_ground_truth_only_replaces_stamp_and_clock_without_mutation(api):
    given = entry()
    before = deepcopy(given)
    result = api.ground_truth_for_capture(given, 2_400_000_000)
    assert result == {
        **before["ground_truth"],
        "stamp_ns": 2_400_000_000,
        "clock_domain": "ros_sim",
    }
    result["left"]["center_m"][0] = 100
    assert given == before


def test_write_produces_eight_hashed_files_and_16bit_mm_png(api, tmp_path):
    hashes = api.write_scene_files(tmp_path, **payload(api))
    assert set(hashes) == {
        "rgb.png",
        "depth_mm.png",
        "depth_meta.json",
        "camera_info.json",
        "tf.json",
        "ground_truth.json",
        "scene.json",
        "depth_preview.png",
    }
    for name, digest in hashes.items():
        assert hashlib.sha256((tmp_path / name).read_bytes()).hexdigest() == digest
    assert (tmp_path / "depth_mm.png").read_bytes()[24:26] == bytes([16, 0])
    with Image.open(tmp_path / "depth_mm.png") as image:
        np.testing.assert_array_equal(
            np.array(image), [[0, 1250, 0], [2500, 5000, 6000]]
        )
    with Image.open(tmp_path / "depth_preview.png") as image:
        np.testing.assert_array_equal(np.array(image), [[0, 64, 0], [128, 255, 255]])
    assert json.loads((tmp_path / "depth_meta.json").read_text()) == {
        "unit": "mm",
        "meters_per_unit": 0.001,
        "unknown_value": 0,
        "kind": "optical_axis_z",
    }
    assert not list(tmp_path.glob("*.tmp"))


def test_all_unknown_depth_saves_black_preview(api, tmp_path):
    given = payload(api)
    given["depth_m"][:] = np.nan
    api.write_scene_files(tmp_path, **given)
    for name in ("depth_mm.png", "depth_preview.png"):
        with Image.open(tmp_path / name) as image:
            assert not np.array(image).any()


def test_save_failure_propagates(api, tmp_path):
    path = tmp_path / "not_a_directory"
    path.write_text("occupied")
    with pytest.raises(OSError):
        api.write_scene_files(path, **payload(api))


def test_save_rejects_mismatched_image_dimensions(api, tmp_path):
    given = payload(api)
    given["rgb"] = np.zeros((1, 3, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="shape"):
        api.write_scene_files(tmp_path, **given)


def test_capture_node_imports_on_host_without_ros():
    module = load_module("scene_capture")
    assert callable(module.main)


def feed_images(state):
    for kind, msg in [
        ("rgb", message()),
        ("depth", message("32FC1")),
        ("info", info()),
    ]:
        state.accept(kind, msg)


def test_capture_waits_for_actual_clock_and_late_received_tf(api):
    state = api.CaptureState(warmup_ns=2_000_000_000)
    feed_images(state)
    assert state.selected_stamp() is None
    state.accept("clock", NS(clock=header().stamp))
    assert state.selected_stamp() is None
    unrelated = transform()
    unrelated.child_frame_id = "lidar_link"
    state.accept("tf_static", NS(transforms=[unrelated]))
    assert state.selected_stamp() is None
    state.accept("tf_static", NS(transforms=[unrelated, transform()]))
    assert state.selected_stamp() == 2_400_000_000
    assert state.tf["translation_m"] == [0.75, 0.0, 0.5]
    assert state.counts == {"rgb": 1, "depth": 1, "info": 1, "clock": 1, "tf_static": 2}


def test_capture_waits_for_actual_clock_even_if_tf_arrives_first(api):
    state = api.CaptureState(warmup_ns=2_000_000_000)
    feed_images(state)
    state.accept("tf_static", NS(transforms=[transform()]))
    assert state.selected_stamp() is None
    state.accept("clock", NS(clock=header().stamp))
    assert state.selected_stamp() == 2_400_000_000


def test_capture_limits_each_stream_to_latest_fifty_stamps(api):
    state = api.CaptureState(warmup_ns=2_000_000_000)
    for sec in [*range(70), 0]:
        for kind, msg in [
            ("rgb", message()),
            ("depth", message("32FC1")),
            ("info", info()),
        ]:
            msg.header = header(sec=sec, nanosec=0)
            state.accept(kind, msg)
    for buffer in state.buffers.values():
        assert len(buffer) == 50
        assert min(buffer) == 20_000_000_000
        assert max(buffer) == 69_000_000_000


def test_capture_does_not_match_nearby_image_stamps(api):
    state = api.CaptureState(warmup_ns=2_000_000_000)
    state.accept("rgb", message())
    state.accept("info", info())
    depth = message("32FC1")
    depth.header.stamp.nanosec += 1
    state.accept("depth", depth)
    state.accept("clock", NS(clock=header().stamp))
    state.accept("tf_static", NS(transforms=[transform()]))
    assert state.selected_stamp() is None


def test_capture_rejects_encoding_violation_immediately(api):
    state = api.CaptureState(warmup_ns=2_000_000_000)
    bad = message("32FC1")
    bad.step += 1
    with pytest.raises(ValueError, match="step"):
        state.accept("depth", bad)
    assert state.buffers["depth"] == {}


def test_capture_skips_black_rgb_but_all_unknown_depth_is_capturable(api):
    state = api.CaptureState(warmup_ns=2_000_000_000)
    black = message()
    black.data = bytes(18)
    state.accept("rgb", black)
    depth = message("32FC1")
    depth.data = np.full((2, 3), np.nan, dtype="<f4").tobytes()
    state.accept("depth", depth)
    state.accept("info", info())
    state.accept("clock", NS(clock=header().stamp))
    state.accept("tf_static", NS(transforms=[transform()]))
    assert state.selected_stamp() is None
    state.accept("rgb", message())
    assert state.selected_stamp() == 2_400_000_000
