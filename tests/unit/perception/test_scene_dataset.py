import builtins
import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from forklift_core.perception import scene_dataset as sd
from forklift_core.sensors.rgbd import deproject_depth_pixels

WIDTH, HEIGHT = 8, 6
K = [4.0, 0.0, 4.0, 0.0, 4.0, 3.0, 0.0, 0.0, 1.0]  # fx=fy=4, cx=4, cy=3


def write_scene(root: Path, **overrides) -> Path:
    scene = root / "s001"
    scene.mkdir(parents=True, exist_ok=False)  # distinct root per call
    rgb = np.full((HEIGHT, WIDTH, 3), 90, dtype=np.uint8)
    Image.fromarray(rgb).save(scene / "rgb.png")
    depth = np.full((HEIGHT, WIDTH), 2000, dtype=np.uint16)
    depth[1, 6] = 2000
    depth[2, 2] = 1500
    depth[4, 0] = 0  # unknown
    Image.fromarray(depth).save(scene / "depth_mm.png")
    files = {
        "depth_meta.json": {
            "unit": "mm",
            "meters_per_unit": 0.001,
            "unknown_value": 0,
            "kind": "optical_axis_z",
        },
        "camera_info.json": {
            "frame_id": "camera_optical_frame",
            "stamp_ns": 5_000_000_000,
            "width": WIDTH,
            "height": HEIGHT,
            "distortion_model": "plumb_bob",
            "d": [0.0] * 5,
            "k": K,
            "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            "p": [4.0, 0.0, 4.0, 0.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            "binning_x": 0,
            "binning_y": 0,
            "roi": {
                "x_offset": 0,
                "y_offset": 0,
                "height": 0,
                "width": 0,
                "do_rectify": False,
            },
        },
        "tf.json": {
            "target_frame": "base_link",
            "source_frame": "camera_optical_frame",
            "translation_m": [0.2, 0.0, 0.5],
            "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
            "origin": "received_tf_static",
        },
        "ground_truth.json": {
            "stamp_ns": 5_000_000_000,
            "clock_domain": "ros_sim",
            "frame_id": "base_link",
            "source_provenance": "synthetic_ground_truth",
            "status": "valid",
            "left": {"center_m": [1.7, 0.175, 0.15], "width_m": 0.25, "height_m": 0.2},
            "right": {
                "center_m": [1.7, -0.175, 0.15],
                "width_m": 0.25,
                "height_m": 0.2,
            },
            "insertion_yaw_rad": 0.0,
            "position_sigma_m": 0.0,
            "yaw_sigma_rad": 0.0,
            "reason": None,
        },
        "scene.json": {
            "scene_id": "s001",
            "catalogue_version": "test",
            "category": "positive",
            "split": "dev",
            "stamp_ns": 5_000_000_000,
            "clock_domain": "ros_sim",
            "source_provenance": "synthetic",
        },
    }
    for name, payload in overrides.items():
        files[name] = payload
    for name, payload in files.items():
        (scene / name).write_text(json.dumps(payload))
    return scene


def test_loader_yields_metric_depth_and_core_types(tmp_path):
    sample = sd.load_scene_sample(write_scene(tmp_path))
    given = sample.input
    assert given.rgb.shape == (HEIGHT, WIDTH, 3) and given.rgb.dtype == np.uint8
    assert given.depth_m.shape == (HEIGHT, WIDTH)
    assert given.depth_m[1, 6] == pytest.approx(2.0) and math.isnan(given.depth_m[4, 0])
    assert (
        given.intrinsics.fx == 4
        and given.intrinsics.cx == 4
        and given.intrinsics.cy == 3
    )
    assert given.pixel_frame == "camera_optical_frame" and given.rectified is True
    assert given.base_from_optical.source_frame == "camera_optical_frame"
    assert sample.ground_truth.left.center_m == (1.7, 0.175, 0.15)
    assert sample.scene["scene_id"] == "s001"


def test_depth_is_already_metric_so_deprojection_uses_unit_scale(tmp_path):
    given = sd.load_scene_input(write_scene(tmp_path))
    optical = deproject_depth_pixels(
        given.depth_m,
        np.array([[6, 1], [0, 4]]),
        given.intrinsics,
        meters_per_unit=1.0,
        pixel_frame=given.pixel_frame,
        rectified=True,
    )
    base = given.base_from_optical.apply(optical)
    # pixel (6,1) at 2.0 m: optical (1.0, -1.0, 2.0) -> base (2.2, -1.0, 1.5)
    np.testing.assert_allclose(base.xyz_m[0], [2.2, -1.0, 1.5], atol=1e-12)
    assert not base.valid[1]  # unknown pixel stays unknown


def test_scene_input_does_not_carry_ground_truth(tmp_path):
    given = sd.load_scene_input(write_scene(tmp_path))
    assert not hasattr(given, "ground_truth") and not hasattr(given, "scene")


@pytest.mark.parametrize(
    "patch",
    [
        {"d": [0.1, 0, 0, 0, 0]},
        {"r": [1, 0, 0, 0, 0.99, 0, 0, 0, 1]},
        {
            "p": [4.5, 0.0, 4.0, 0.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        },  # P[:, :3] != K
        {
            "p": [4.0, 0.0, 4.0, 1.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        },  # P[:, 3] != 0
        {"k": [4.0, 0.5, 4.0, 0.0, 4.0, 3.0, 0.0, 0.0, 1.0]},
        {"binning_x": 2},
        {
            "roi": {
                "x_offset": 1,
                "y_offset": 0,
                "height": 0,
                "width": 0,
                "do_rectify": False,
            }
        },
        {"width": WIDTH + 1},
    ],
)
def test_camera_info_outside_the_narrow_contract_is_rejected(tmp_path, patch):
    base = json.loads(
        json.dumps(
            {
                "frame_id": "camera_optical_frame",
                "stamp_ns": 5_000_000_000,
                "width": WIDTH,
                "height": HEIGHT,
                "distortion_model": "plumb_bob",
                "d": [0.0] * 5,
                "k": K,
                "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                "p": [4.0, 0.0, 4.0, 0.0, 0.0, 4.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                "binning_x": 0,
                "binning_y": 0,
                "roi": {
                    "x_offset": 0,
                    "y_offset": 0,
                    "height": 0,
                    "width": 0,
                    "do_rectify": False,
                },
            }
        )
    )
    base.update(patch)
    with pytest.raises(ValueError):
        sd.load_scene_input(write_scene(tmp_path, **{"camera_info.json": base}))


def test_principal_point_is_preserved_without_half_pixel_adjustment():
    info = {
        "frame_id": "f",
        "stamp_ns": 0,
        "width": 320,
        "height": 240,
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [160.0, 0, 160.0, 0, 160.0, 120.0, 0, 0, 1],
        "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "p": [160.0, 0, 160.0, 0, 0, 160.0, 120.0, 0, 0, 0, 1, 0],
        "binning_x": 0,
        "binning_y": 0,
        "roi": {
            "x_offset": 0,
            "y_offset": 0,
            "height": 0,
            "width": 0,
            "do_rectify": False,
        },
    }
    intrinsics = sd.intrinsics_from_camera_info(info)
    assert (intrinsics.cx, intrinsics.cy) == (160.0, 120.0)


def test_depth_meta_kind_unit_and_unknown_value_are_validated():
    raw = np.array([[0, 1500]], dtype=np.uint16)
    good = {
        "unit": "mm",
        "meters_per_unit": 0.001,
        "unknown_value": 0,
        "kind": "optical_axis_z",
    }
    decoded = sd.decode_depth_mm(raw, good)
    assert math.isnan(decoded[0, 0]) and decoded[0, 1] == pytest.approx(1.5)
    for bad in (
        {**good, "kind": "ray_length"},
        {**good, "unit": "m"},
        {**good, "meters_per_unit": 0},
        {**good, "unknown_value": 0.5},
    ):
        with pytest.raises(ValueError):
            sd.decode_depth_mm(raw, bad)


def test_tf_json_frames_and_origin_are_checked(tmp_path):
    for patch in (
        {"target_frame": "odom"},
        {"source_frame": "camera_link"},
        {"origin": "config_copy"},
    ):
        tf = {
            "target_frame": "base_link",
            "source_frame": "camera_optical_frame",
            "translation_m": [0.2, 0.0, 0.5],
            "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
            "origin": "received_tf_static",
            **patch,
        }
        with pytest.raises(ValueError):
            sd.load_scene_input(
                write_scene(tmp_path / patch[list(patch)[0]], **{"tf.json": tf})
            )


def test_ground_truth_stamp_must_match_scene_stamp(tmp_path):
    gt = json.loads((write_scene(tmp_path / "a") / "ground_truth.json").read_text())
    gt["stamp_ns"] = 1
    with pytest.raises(ValueError):
        sd.load_scene_sample(write_scene(tmp_path / "b", **{"ground_truth.json": gt}))


def test_eight_bit_depth_png_is_rejected(tmp_path):
    scene = write_scene(tmp_path)
    Image.fromarray(np.zeros((HEIGHT, WIDTH), dtype=np.uint8)).save(
        scene / "depth_mm.png"
    )
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


def patch_json(scene, filename, **patch):
    path = scene / filename
    obj = json.loads(path.read_text())
    obj.update(patch)
    path.write_text(json.dumps(obj))


@pytest.mark.parametrize(
    "field,value", [("stamp_ns", 1), ("frame_id", "another_optical_frame")]
)
def test_camera_stamp_and_frame_must_match_scene_and_tf(tmp_path, field, value):
    scene = write_scene(tmp_path)
    patch_json(scene, "camera_info.json", **{field: value})
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


def test_ground_truth_clock_must_match_scene_clock(tmp_path):
    scene = write_scene(tmp_path)
    patch_json(scene, "ground_truth.json", clock_domain="synthetic")
    with pytest.raises(ValueError):
        sd.load_scene_sample(scene)


def test_non_valid_ground_truth_loads_as_an_evaluation_sample(tmp_path):
    scene = write_scene(tmp_path)
    patch_json(
        scene,
        "ground_truth.json",
        status="no_pallet",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="no target pallet in scene",
    )
    assert sd.load_scene_sample(scene).ground_truth.status == "no_pallet"


@pytest.mark.parametrize("filename", ["rgb.png", "depth_mm.png"])
def test_image_shape_must_match_camera_info(tmp_path, filename):
    scene = write_scene(tmp_path)
    raw = (
        np.zeros((HEIGHT - 1, WIDTH, 3), dtype=np.uint8)
        if filename == "rgb.png"
        else np.zeros((HEIGHT - 1, WIDTH), dtype=np.uint16)
    )
    Image.fromarray(raw).save(scene / filename)
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


@pytest.mark.parametrize("filename", ["rgb.png", "depth_mm.png"])
def test_png_extension_does_not_substitute_for_png_format(tmp_path, filename):
    scene = write_scene(tmp_path)
    raw = (
        np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        if filename == "rgb.png"
        else np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
    )
    Image.fromarray(raw).save(scene / filename, format="TIFF")
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


@pytest.mark.parametrize("mode", ["L", "RGBA", "P"])
def test_rgb_png_must_be_eight_bit_rgb_without_conversion(tmp_path, mode):
    scene = write_scene(tmp_path)
    Image.new(mode, (WIDTH, HEIGHT)).save(scene / "rgb.png")
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


def test_rgb_depth_png_is_rejected(tmp_path):
    scene = write_scene(tmp_path)
    Image.fromarray(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)).save(
        scene / "depth_mm.png"
    )
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


@pytest.mark.parametrize("value", [1500, -1, 65536])
def test_legacy_pillow_i_mode_requires_uint16_range(tmp_path, monkeypatch, value):
    scene = write_scene(tmp_path)
    original_open = Image.open

    def legacy_open(path):
        if Path(path).name == "depth_mm.png":
            # Simulate old Pillow decoding a 16-bit PNG into an actual I image.
            image = Image.fromarray(np.full((HEIGHT, WIDTH), value, dtype=np.int32))
            image.format = "PNG"
            return image
        return original_open(path)

    monkeypatch.setattr(Image, "open", legacy_open)
    if value == 1500:
        given = sd.load_scene_input(scene)
        assert given.depth_m.dtype == np.float64
        np.testing.assert_allclose(given.depth_m, 1.5)
    else:
        with pytest.raises(ValueError):
            sd.load_scene_input(scene)


def depth_meta():
    return {
        "unit": "mm",
        "meters_per_unit": 0.001,
        "unknown_value": 0,
        "kind": "optical_axis_z",
    }


@pytest.mark.parametrize(
    "raw",
    [
        np.array([[90]], dtype=np.uint8),
        np.array([[1500]], dtype=np.int32),
        np.array([[1500.0]]),
        np.array([1500], dtype=np.uint16),
        np.array([[[1500]]], dtype=np.uint16),
    ],
)
def test_decode_depth_requires_two_dimensional_uint16(raw):
    with pytest.raises(ValueError):
        sd.decode_depth_mm(raw, depth_meta())


@pytest.mark.parametrize(
    "field,value",
    [
        ("meters_per_unit", math.nan),
        ("meters_per_unit", math.inf),
        ("meters_per_unit", 0.0010000001),
        ("meters_per_unit", 1.0),
        ("meters_per_unit", "0.001"),
        ("meters_per_unit", True),
        ("unknown_value", 1),
        ("unknown_value", False),
        ("unknown_value", 0.0),
    ],
)
def test_depth_v1_metadata_is_exact_not_approximately_matching(field, value):
    with pytest.raises(ValueError):
        sd.decode_depth_mm(
            np.array([[0, 1500]], dtype=np.uint16), {**depth_meta(), field: value}
        )


def test_depth_decode_preserves_unknown_and_uint16_endpoints_without_mutating_raw():
    raw = np.array([[0, 1, 65535]], dtype=np.uint16)
    decoded = sd.decode_depth_mm(raw, depth_meta())
    assert decoded.dtype == np.float64
    np.testing.assert_allclose(decoded, [[math.nan, 0.001, 65.535]], equal_nan=True)
    np.testing.assert_array_equal(raw, [[0, 1, 65535]])


@pytest.mark.parametrize(
    "patch",
    [
        {"binning_y": 2},
        {"binning_x": -1},
        {"binning_y": 0.5},
        {"d": [math.nan]},
        {"d": [[0.0]]},
        {"r": [1] * 8},
        {"p": [1] * 11},
        {"width": True},
        {"height": 0},
        {"k": [4, 0, 4, 0.5, 4, 3, 0, 0, 1]},
        {"k": [4, 0, 4, 0, 4, 3, 0.1, 0, 1]},
        {"k": [4, 0, 4, 0, 4, 3, 0, 0.1, 1]},
        {"k": [4, 0, 4, 0, 4, 3, 0, 0, 2]},
        {"k": [4, 0, math.inf, 0, 4, 3, 0, 0, 1]},
    ],
)
def test_camera_info_other_malformed_values_are_rejected(tmp_path, patch):
    scene = write_scene(tmp_path)
    patch_json(scene, "camera_info.json", **patch)
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


@pytest.mark.parametrize(
    "index,value", [(1, 0.1), (3, 0.1), (6, 0.1), (7, 0.1), (8, 2)]
)
def test_noncanonical_k_is_rejected_even_when_p_matches_it(tmp_path, index, value):
    scene = write_scene(tmp_path)
    info = json.loads((scene / "camera_info.json").read_text())
    info["k"][index] = value
    info["p"] = (
        np.column_stack([np.array(info["k"]).reshape(3, 3), np.zeros(3)])
        .ravel()
        .tolist()
    )
    with pytest.raises(ValueError):
        sd.intrinsics_from_camera_info(info)


@pytest.mark.parametrize(
    "field,value",
    [
        ("y_offset", 1),
        ("width", 1),
        ("height", 1),
        ("do_rectify", True),
    ],
)
def test_each_nonempty_roi_component_is_rejected(tmp_path, field, value):
    scene = write_scene(tmp_path)
    info = json.loads((scene / "camera_info.json").read_text())
    info["roi"][field] = value
    with pytest.raises(ValueError):
        sd.intrinsics_from_camera_info(info)


def test_unbinned_value_one_and_empty_distortion_preserve_intrinsics(tmp_path):
    scene = write_scene(tmp_path)
    patch_json(scene, "camera_info.json", binning_x=1, binning_y=1, d=[])
    given = sd.load_scene_input(scene)
    assert (
        given.intrinsics.fx,
        given.intrinsics.fy,
        given.intrinsics.cx,
        given.intrinsics.cy,
    ) == (4, 4, 4, 3)


@pytest.mark.parametrize(
    "field",
    [
        "scene_id",
        "catalogue_version",
        "category",
        "split",
        "stamp_ns",
        "clock_domain",
        "source_provenance",
    ],
)
def test_scene_metadata_requires_each_contract_key(tmp_path, field):
    scene = write_scene(tmp_path)
    obj = json.loads((scene / "scene.json").read_text())
    del obj[field]
    (scene / "scene.json").write_text(json.dumps(obj))
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


@pytest.mark.parametrize(
    "field,value",
    [
        ("clock_domain", "wall"),
        ("source_provenance", "guess"),
        ("stamp_ns", -1),
        ("stamp_ns", True),
        ("stamp_ns", 5_000_000_000.0),
    ],
)
def test_scene_clock_provenance_and_integer_stamp_are_validated(tmp_path, field, value):
    scene = write_scene(tmp_path)
    patch_json(scene, "scene.json", **{field: value})
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


def test_camera_stamp_requires_integer_even_when_numerically_equal(tmp_path):
    scene = write_scene(tmp_path)
    patch_json(scene, "camera_info.json", stamp_ns=5_000_000_000.0)
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


def test_scene_metadata_is_preserved_only_on_sample_and_input_carries_acquisition_metadata(
    tmp_path,
):
    scene = write_scene(tmp_path)
    patch_json(
        scene, "scene.json", extra={"note": "preserve"}, source_provenance="replay"
    )
    sample = sd.load_scene_sample(scene)
    assert sample.scene["extra"] == {"note": "preserve"}
    assert sample.input.stamp_ns == 5_000_000_000
    assert sample.input.clock_domain == "ros_sim"
    assert sample.input.source_provenance == "replay"
    assert sample.input.rgb_registered_to_depth_grid is True


@pytest.mark.parametrize("contents", [None, "invalid json"])
def test_input_loader_never_requires_or_parses_ground_truth(tmp_path, contents):
    scene = write_scene(tmp_path)
    path = scene / "ground_truth.json"
    if contents is None:
        path.unlink()
    else:
        path.write_text(contents)
    assert sd.load_scene_input(scene).depth_m[2, 2] == pytest.approx(1.5)
    with pytest.raises((ValueError, FileNotFoundError)):
        sd.load_scene_sample(scene)


@pytest.mark.parametrize(
    "filename",
    [
        "rgb.png",
        "depth_mm.png",
        "depth_meta.json",
        "camera_info.json",
        "tf.json",
        "scene.json",
    ],
)
def test_missing_input_file_raises(tmp_path, filename):
    scene = write_scene(tmp_path)
    (scene / filename).unlink()
    with pytest.raises((ValueError, FileNotFoundError)):
        sd.load_scene_input(scene)


@pytest.mark.parametrize(
    "filename", ["depth_meta.json", "camera_info.json", "tf.json", "scene.json"]
)
def test_json_files_require_objects(tmp_path, filename):
    scene = write_scene(tmp_path)
    (scene / filename).write_text("[]")
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


@pytest.mark.parametrize(
    "filename,key",
    [
        ("depth_meta.json", "unit"),
        ("camera_info.json", "r"),
        ("camera_info.json", "distortion_model"),
        ("tf.json", "quaternion_xyzw"),
    ],
)
def test_missing_metadata_keys_raise_value_error(tmp_path, filename, key):
    scene = write_scene(tmp_path)
    obj = json.loads((scene / filename).read_text())
    del obj[key]
    (scene / filename).write_text(json.dumps(obj))
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


@pytest.mark.parametrize(
    "patch",
    [
        {"translation_m": [0, 0]},
        {"translation_m": [0, math.inf, 0]},
        {"quaternion_xyzw": [0, 0, 0, 2]},
    ],
)
def test_tf_invalid_geometry_is_rejected(tmp_path, patch):
    scene = write_scene(tmp_path)
    patch_json(scene, "tf.json", **patch)
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


def test_missing_pillow_explains_dataset_extra_at_load_time(tmp_path, monkeypatch):
    scene = write_scene(tmp_path)
    original_import = builtins.__import__

    def without_pillow(name, *args, **kwargs):
        if name == "PIL" or name.startswith("PIL."):
            raise ImportError("Pillow unavailable")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", without_pillow)
    with pytest.raises(ImportError, match=r"pip install 'forklift-core\[dataset\]'"):
        sd.load_scene_input(scene)


@pytest.mark.parametrize(
    "filename", ["depth_meta.json", "camera_info.json", "tf.json", "roi"]
)
def test_fixed_metadata_rejects_unknown_keys(tmp_path, filename):
    scene = write_scene(tmp_path)
    if filename == "roi":
        info = json.loads((scene / "camera_info.json").read_text())
        info["roi"]["extra"] = 1
        patch_json(scene, "camera_info.json", roi=info["roi"])
    else:
        patch_json(scene, filename, extra=1)
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)


def test_sixteen_bit_rgb_png_is_rejected_before_pillow_reduces_it(tmp_path):
    scene = write_scene(tmp_path)

    def chunk(kind, payload):
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload))
        )

    # Generate real 16-bit RGB PNG bytes: Pillow decodes these into RGB uint8.
    header = struct.pack(">IIBBBBB", WIDTH, HEIGHT, 16, 2, 0, 0, 0)
    row = b"\x00" + struct.pack(">HHH", 0x1234, 0x5678, 0x9ABC) * WIDTH
    encoded = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(row * HEIGHT))
        + chunk(b"IEND", b"")
    )
    (scene / "rgb.png").write_bytes(encoded)
    with pytest.raises(ValueError):
        sd.load_scene_input(scene)
