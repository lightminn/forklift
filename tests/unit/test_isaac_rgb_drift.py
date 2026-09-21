"""CPU-only contracts for the separate RGB drift observation tool."""

import ast
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim/isaac/diagnose_rgb_drift.py"


@pytest.fixture
def drift():
    assert SCRIPT.is_file(), "RGB drift diagnostic has not been implemented"
    spec = importlib.util.spec_from_file_location("rgb_drift_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "condition,subframes,aa",
    [
        ("A", 4, None),
        ("B", 16, None),
        ("C", 4, 0),
        ("D", 4, None),
    ],
)
def test_condition_changes_only_the_requested_variable(drift, condition, subframes, aa):
    config = drift.condition_config(condition)
    expected_app = {"headless": True, "renderer": "RaytracedLighting"}
    if aa is not None:
        expected_app["anti_aliasing"] = aa
    assert config["simulation_app"] == expected_app
    assert config["step"] == {
        "rt_subframes": subframes,
        "delta_time": 0.0,
        "pause_timeline": True,
    }
    assert config["distance_m"] == 3.0
    assert config["captures_per_board"] == 12
    expected_indices = list(range(8, -1, -1)) if condition == "D" else list(range(9))
    assert [p["position_index"] for p in config["positions"]] == expected_indices
    assert sorted(tuple(p["centre_uv"]) for p in config["positions"]) == [
        (100, 80),
        (100, 240),
        (100, 400),
        (320, 80),
        (320, 240),
        (320, 400),
        (540, 80),
        (540, 240),
        (540, 400),
    ]
    config["simulation_app"]["anti_aliasing"] = 99
    assert drift.condition_config("A")["simulation_app"] == {
        "headless": True,
        "renderer": "RaytracedLighting",
    }


def test_unknown_condition_is_rejected(drift):
    with pytest.raises(ValueError, match="condition"):
        drift.condition_config("all")


def test_the_anchor_distance_is_read_from_the_protocol_not_restated(drift):
    """One source of truth for r.

    run() needs Isaac, so its two distance call sites are pinned in source: the
    recorded protocol value must be what the board is laid out at and what the
    board record reports, or the record can disagree with the actual geometry.
    """
    run = next(
        node
        for node in ast.walk(ast.parse(SCRIPT.read_text(encoding="utf-8")))
        if isinstance(node, ast.FunctionDef) and node.name == "run"
    )

    def reads_protocol_distance(node):
        return (
            isinstance(node, ast.Subscript)
            and isinstance(node.slice, ast.Constant)
            and node.slice.value == "distance_m"
        )

    layouts = [
        node
        for node in ast.walk(run)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "checkerboard_layout"
    ]
    assert len(layouts) == 1
    assert reads_protocol_distance(layouts[0].args[0]), ast.dump(layouts[0].args[0])
    anchors = [
        keyword
        for node in ast.walk(run)
        if isinstance(node, ast.Call)
        for keyword in node.keywords
        if keyword.arg == "anchor_horizontal_m"
    ]
    assert len(anchors) == 1
    assert reads_protocol_distance(anchors[0].value), ast.dump(anchors[0].value)
    assert 3.0 not in {
        node.value
        for node in ast.walk(run)
        if isinstance(node, ast.Constant) and isinstance(node.value, float)
    }
    assert drift.condition_config("A")["distance_m"] == 3.0


def test_translation_is_removed_before_residual_p95_and_regression(drift):
    uv = np.array([[10, 20], [12, 20], [10, 24], [12, 24]], dtype=float)
    stats = drift.displacement_statistics(uv + [0.25, -0.5], uv)
    assert stats["mean_delta_uv_px"] == pytest.approx([0.25, -0.5])
    assert stats["centered_euclidean_p95_px"] == pytest.approx(0)
    assert stats["centered_abs_p95_uv_px"] == pytest.approx([0, 0])
    np.testing.assert_allclose(stats["residual_affine_coefficients"], 0, atol=1e-14)


def test_regression_retains_cross_axis_rotation_and_shear(drift):
    uv = np.array([[99, 198], [101, 198], [99, 202], [101, 202]], dtype=float)
    # du = 2 + .1*(u-100) + .2*(v-200); dv = -3 - .3*(u-100) + .4*(v-200)
    delta = np.array([[1.5, -3.5], [1.7, -4.1], [2.3, -1.9], [2.5, -2.5]])
    stats = drift.displacement_statistics(uv + delta, uv)
    assert stats["mean_delta_uv_px"] == pytest.approx([2, -3])
    assert stats["regression_origin_uv_px"] == pytest.approx([100, 200])
    np.testing.assert_allclose(
        stats["residual_affine_coefficients"],
        [[0, 0.1, 0.2], [0, -0.3, 0.4]],
        atol=1e-13,
    )
    assert stats["regression_rank"] == 3
    # Norms are sqrt(.50), sqrt(1.30), sqrt(1.30), sqrt(.50).
    assert stats["centered_euclidean_p95_px"] == pytest.approx(np.sqrt(1.3))
    assert stats["centered_abs_p95_uv_px"] == pytest.approx([0.5, 1.1])


def test_absent_corners_are_not_zero_drift_or_a_minimum_norm_fit(drift):
    uv = np.array([[1, 2], [3, 2], [np.nan, np.nan]])
    stats = drift.displacement_statistics(uv, uv)
    assert stats["count"] == 2
    assert stats["unavailable_count"] == 1
    assert stats["regression_rank"] == 2
    assert stats["residual_affine_coefficients"] is None
    empty = drift.displacement_statistics(uv * np.nan, uv)
    assert empty["count"] == 0
    assert empty["mean_delta_uv_px"] is None
    assert empty["centered_euclidean_p95_px"] is None
    json.dumps(empty, allow_nan=False)


def test_crop_origin_matches_actual_checkerboard_input(drift, monkeypatch):
    import cv2

    rgb = np.zeros((40, 50, 3), dtype=np.uint8)
    mask = np.zeros((40, 50), dtype=bool)
    mask[10:30, 2:22] = True
    roi = drift.mask_geometry(mask)
    assert roi["bbox_xyxy_exclusive"] == [2, 10, 22, 30]
    assert roi["crop_xyxy_exclusive"] == [0, 6, 26, 34]
    assert roi["crop_origin_uv"] == [0, 6]

    def detect(gray, pattern, flags):
        assert gray.shape == (28, 26)
        assert gray[0, 0] == 255
        assert gray[4, 2] == 0
        return True, np.array(
            [[[j, i]] for i in range(7) for j in range(9)], np.float32
        )

    monkeypatch.setattr(cv2, "findChessboardCornersSB", detect)
    corners = drift.MEASURE.checkerboard_corners(rgb, mask)
    np.testing.assert_array_equal(corners[0], roi["crop_origin_uv"])
    empty = drift.mask_geometry(np.zeros((40, 50), dtype=bool))
    assert empty["crop_origin_uv"] is None


def test_depth_compares_values_at_fixed_pixels_not_missing_count(drift):
    first = np.array([[1.0, 2.0, np.nan], [0.0, np.inf, 6.0]])
    later = np.array([[1.25, 1.5, 4.0], [0.0, np.inf, np.nan]])
    stats = drift.depth_difference(later, first)
    assert stats["common_valid_count"] == 2
    assert stats["changed_valid_count"] == 2
    assert stats["validity_changed_count"] == 2
    assert stats["mean_delta_m"] == pytest.approx(-0.125)
    assert stats["max_abs_delta_m"] == pytest.approx(0.5)
    assert stats["p95_abs_delta_m"] == pytest.approx(0.4875)
    selected = drift.depth_difference(
        later, first, np.array([[True, False, False], [False, False, False]])
    )
    assert selected["common_valid_count"] == 1
    assert selected["mean_delta_m"] == pytest.approx(0.25)
    empty = drift.depth_difference(np.zeros((2, 3)), first)
    assert empty["max_abs_delta_m"] is None


def test_render_settings_are_read_back_not_inferred_from_condition(drift):
    values = {
        "/rtx/post/aa/op": 1,
        "/rtx/rendermode": "RaytracedLighting",
        "/rtx/post/dlss/execMode": 2,
        "/rtx/post/dlss/rr/enabled": True,
        "/rtx-transient/dlssg/enabled": False,
    }
    actual = drift.read_render_settings(SimpleNamespace(get=values.get))
    # The whole DLSS evidence set, not a subset: the run has to record what was
    # actually applied, and an unread key is indistinguishable from an unset one.
    assert actual["values"] == {
        "/rtx/post/aa/op": 1,
        "/rtx/rendermode": "RaytracedLighting",
        "/rtx/post/dlss/execMode": 2,
        "/rtx/post/dlss/enabled": None,
        "/rtx/post/dlss/rr/enabled": True,
        "/rtx-transient/dlssg/enabled": False,
    }
    assert actual["readback_errors"] == {}

    def broken(key):
        if key == "/rtx/post/dlss/rr/enabled":
            raise RuntimeError("carb refused")
        return values.get(key)

    guarded = drift.read_render_settings(SimpleNamespace(get=broken))
    assert guarded["values"]["/rtx/post/dlss/rr/enabled"] is None
    assert guarded["readback_errors"] == {
        "/rtx/post/dlss/rr/enabled": "RuntimeError: carb refused"
    }
    assert guarded["values"]["/rtx/post/aa/op"] == 1


def test_an_unserializable_setting_value_is_recorded_not_fatal(drift, tmp_path):
    """One odd carb value must cost that value's type, never the whole condition."""

    class Odd:
        def __repr__(self):
            return "<carb.Odd>"

    values = {
        "/rtx/post/aa/op": Odd(),
        "/rtx/rendermode": "RaytracedLighting",
        "/rtx/post/dlss/enabled": float("nan"),
        "/rtx/post/dlss/rr/enabled": [0, 1],
    }
    record = drift.read_render_settings(SimpleNamespace(get=values.get))
    drift.write_record(tmp_path / "settings.json", record)
    saved = json.loads((tmp_path / "settings.json").read_text(encoding="utf-8"))
    assert saved["values"]["/rtx/post/aa/op"] == {
        "unserializable_type": "Odd",
        "value_repr": "<carb.Odd>",
    }
    # allow_nan=False rejects NaN too, so it is text here rather than a lost key.
    assert saved["values"]["/rtx/post/dlss/enabled"]["unserializable_type"] == "float"
    assert saved["values"]["/rtx/rendermode"] == "RaytracedLighting"
    assert saved["values"]["/rtx/post/dlss/rr/enabled"] == [0, 1]
    assert saved["values"]["/rtx-transient/dlssg/enabled"] is None
    # The coercion is confined to those values; nothing else is loosened.
    with pytest.raises(ValueError):
        drift.write_record(tmp_path / "nan.json", {"other": float("nan")})
    with pytest.raises(TypeError, match="Unsupported record type"):
        drift.write_record(tmp_path / "odd.json", {"other": Odd()})


def test_one_capture_is_one_step_and_owns_original_buffers(drift):
    shared = np.zeros((2, 3), np.float32)
    calls = []
    rep = SimpleNamespace(
        orchestrator=SimpleNamespace(step=lambda **kw: calls.append(kw))
    )
    annotators = {"z": SimpleNamespace(get_data=lambda: shared)}
    frame = drift.capture_once(rep, annotators, drift.condition_config("B")["step"])
    shared[:] = 17
    assert calls == [{"rt_subframes": 16, "delta_time": 0.0, "pause_timeline": True}]
    np.testing.assert_array_equal(frame["z"], np.zeros((2, 3)))


def test_capture_measures_the_transforms_after_the_step_not_only_before(
    drift, tmp_path
):
    """The premise "the scene is untouched between captures" has to be measured.

    Recording the matrices only before the step leaves a change caused *within*
    the step invisible, which is exactly the change that would explain the RGB.
    """
    stepped = []

    class Scene:
        def world_matrix(self, path):
            matrix = np.eye(4)
            if path == "/World/board" and stepped:
                matrix[0, 3] = 0.002  # The step itself moved the board.
            return matrix

    capture = drift.make_capture(
        Scene(),
        {
            "world_from_base": "/World/base",
            "world_from_camera_usd": "/World/camera",
            "world_from_board": "/World/board",
        },
        SimpleNamespace(get_current_time=lambda: 1.5, is_playing=lambda: False),
        SimpleNamespace(
            orchestrator=SimpleNamespace(step=lambda **kw: stepped.append(kw))
        ),
        {"rgb": SimpleNamespace(get_data=lambda: np.zeros((2, 2, 3), np.uint8))},
        {"rt_subframes": 4, "delta_time": 0.0, "pause_timeline": True},
        SimpleNamespace(get={"/rtx/post/aa/op": 3}.get),
        SimpleNamespace(
            get_current_frame=lambda: {"rendering_frame": 9, "rendering_time": 1.5}
        ),
    )
    frame, metadata = capture()
    assert stepped == [{"rt_subframes": 4, "delta_time": 0.0, "pause_timeline": True}]
    np.testing.assert_array_equal(frame["rgb"], np.zeros((2, 2, 3), np.uint8))
    names = {"world_from_base", "world_from_camera_usd", "world_from_board"}
    assert set(metadata["after"]) == names
    assert names <= set(metadata["before"])
    assert metadata["before"]["world_from_board"][0][3] == 0.0
    assert metadata["after"]["world_from_board"][0][3] == pytest.approx(0.002)
    assert metadata["transforms_changed"] == {
        "world_from_base": False,
        "world_from_camera_usd": False,
        "world_from_board": True,
    }
    assert metadata["before"]["timeline_time_s"] == 1.5
    assert metadata["time_after_s"] == 1.5
    assert metadata["timeline_playing_after"] is False
    assert metadata["render_settings"]["values"]["/rtx/post/aa/op"] == 3
    assert metadata["camera_frame_readback"] == {
        "rendering_frame": 9,
        "rendering_time": 1.5,
    }
    drift.write_record(tmp_path / "metadata.json", metadata)


def test_board_keeps_all_twelve_raw_frames_and_unobservable_slots(
    drift, monkeypatch, tmp_path
):
    corners = np.array([[10 + j, 10 + i] for i in range(7) for j in range(9)], float)
    monkeypatch.setattr(
        drift.MEASURE, "checkerboard_corners", lambda rgb, mask: corners + rgb[0, 0, 0]
    )
    calls = []

    def capture():
        i = len(calls)
        calls.append(i)
        ids = np.ones((30, 30), np.uint32)
        if i == 1:
            ids[0, 0] = 0  # Mask changes are visible even if corners still exist.
        labels = {} if i == 2 else {"1": {"class": "calibration_board"}}
        return {
            "rgb": np.full((30, 30, 3), i, np.uint8),
            "z": np.full((30, 30), 3 + 0.01 * i, np.float32),
            "seg": {"data": ids, "info": {"idToLabels": labels}},
        }, {"actual_frame": i + 80}

    board = drift.collect_board(tmp_path, 4, capture, corners)
    assert calls == list(range(12))
    assert len(board["captures"]) == 12
    assert board["captures"][1]["mask_vs_first"]["changed_pixel_count"] == 1
    assert board["captures"][1]["depth_vs_first"]["changed_valid_count"] == 900
    assert board["captures"][2]["measurement_error"]
    assert board["captures"][2]["vs_first"]["mean_delta_uv_px"] is None
    assert board["captures"][3]["vs_previous"]["count"] == 0
    assert board["captures"][3]["vs_first"]["mean_delta_uv_px"] == [3.0, 3.0]
    with np.load(tmp_path / "board_p4_corners.npz", allow_pickle=False) as saved:
        assert saved["corners_uv_px"].shape == (12, 63, 2)
        assert np.isnan(saved["corners_uv_px"][2]).all()
    with np.load(tmp_path / "board_p4_capture11.npz", allow_pickle=False) as saved:
        np.testing.assert_array_equal(saved["rgb"], np.full((30, 30, 3), 11, np.uint8))
        assert saved["segmentation"].dtype == np.uint32
        assert saved["mask"].dtype == bool
        assert saved["depth_m"][5, 5] == pytest.approx(3.11)
        assert saved["crop_origin_uv"].tolist() == [0, 0]
    encoded = json.dumps(board, allow_nan=False)
    assert "PASS" not in encoded and "FAIL" not in encoded


def test_curves_and_table_preserve_signed_means_and_missing_points(drift, tmp_path):
    stats = {
        "mean_delta_uv_px": [-0.25, 0.5],
        "centered_euclidean_p95_px": 0.1,
        "residual_affine_coefficients": [[0.0, 0.01, 0.02], [0.0, 0.03, 0.04]],
    }
    missing = {
        "mean_delta_uv_px": None,
        "centered_euclidean_p95_px": None,
        "residual_affine_coefficients": None,
    }
    board = {
        "position_index": 4,
        "captures": [
            {
                "capture_index": 0,
                "vs_nominal": stats,
                "vs_first": stats,
                "vs_previous": missing,
            },
            {
                "capture_index": 1,
                "vs_nominal": missing,
                "vs_first": missing,
                "vs_previous": missing,
            },
        ],
    }
    drift.write_curves(tmp_path, [board])
    import csv
    import xml.etree.ElementTree as ET

    with (tmp_path / "curves.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["nominal_mean_du_px"] == "-0.25"
    assert rows[1]["nominal_mean_du_px"] == ""
    assert rows[0]["nominal_du_dv"] == "0.02"
    for name in ("nominal", "first", "previous", "nominal_affine"):
        root = ET.parse(tmp_path / f"curves_{name}.svg").getroot()
        assert root.tag.endswith("svg")


def test_rgb_change_is_recorded_beside_identical_depth_and_segmentation(
    drift, monkeypatch, tmp_path
):
    """Section 9.5b's claim must be readable from the record, not only the NPZs."""
    uv = np.array([[j, i] for i in range(7) for j in range(9)], float)
    monkeypatch.setattr(drift.MEASURE, "checkerboard_corners", lambda rgb, mask: uv)
    calls = []

    def capture():
        index = len(calls)
        calls.append(index)
        rgb = np.full((12, 12, 4), 40, np.uint8)
        rgb[0, :index, 0] = 250  # Only the colour buffer moves.
        return {
            "rgb": rgb,
            "z": np.full((12, 12), 2.5, np.float32),
            "seg": {
                "data": np.ones((12, 12), np.uint32),
                "info": {"idToLabels": {"1": {"class": "calibration_board"}}},
            },
        }, {}

    board = drift.collect_board(tmp_path, 7, capture, uv)
    first = board["captures"][0]
    assert first["rgb_vs_first"]["bitwise_identical"] is True
    assert first["rgb_vs_previous"] is None
    third = board["captures"][3]
    assert third["rgb_vs_first"]["bitwise_identical"] is False
    assert third["rgb_vs_first"]["changed_pixel_count"] == 3
    assert third["rgb_vs_previous"]["changed_pixel_count"] == 1
    # The two buffers that section 9.5b found unchanged must read as unchanged.
    assert third["segmentation_vs_first"]["bitwise_identical"] is True
    assert third["depth_vs_first"]["changed_valid_count"] == 0
    drift.write_record(tmp_path / "board.json", board)


def test_empty_render_frame_is_named_not_left_as_a_corner_error(
    drift, monkeypatch, tmp_path
):
    uv = np.array([[j, i] for i in range(7) for j in range(9)], float)
    monkeypatch.setattr(drift.MEASURE, "checkerboard_corners", lambda rgb, mask: uv)
    labels = {"1": {"class": "calibration_board"}}
    calls = []

    def capture():
        index = len(calls)
        calls.append(index)
        if index == 0:  # The renderer returned nothing at all.
            return {"rgb": None, "z": None, "seg": {"data": None}}, {}
        if index == 1:  # G1's observed all-black frame with no finite depth.
            return {
                "rgb": np.zeros((12, 12, 4), np.uint8),
                "z": np.full((12, 12), np.nan, np.float32),
                "seg": {
                    "data": np.zeros((12, 12), np.uint32),
                    "info": {"idToLabels": labels},
                },
            }, {}
        return {
            "rgb": np.full((12, 12, 4), 60, np.uint8),
            "z": np.full((12, 12), 2.5, np.float32),
            "seg": {
                "data": np.ones((12, 12), np.uint32),
                "info": {"idToLabels": labels},
            },
        }, {}

    board = drift.collect_board(tmp_path, 2, capture, uv)
    assert len(calls) == 12, "an empty frame must not trigger a retry"
    assert board["captures"][0]["render"]["status"] == "invalid_render_buffers"
    assert board["captures"][1]["render"]["status"] == "empty_render_frame"
    assert board["captures"][1]["render"]["rgb"]["nonzero_count"] == 0
    assert board["captures"][2]["render"]["status"] == "nonempty"
    # The black frame itself is kept, so the cause stays reviewable offline.
    with np.load(tmp_path / "board_p2_capture01.npz", allow_pickle=False) as saved:
        assert saved["rgb"].shape == (12, 12, 4)
        assert not saved["rgb"].any()
    drift.write_record(tmp_path / "board.json", board)


@pytest.mark.parametrize("problem", ["missing_buffers", "detector_error"])
def test_unready_first_capture_is_retained_without_rebasing_or_retry(
    drift, monkeypatch, tmp_path, problem
):
    import cv2

    uv = np.array([[j, i] for i in range(7) for j in range(9)], float)
    calls = []

    def detect(rgb, mask):
        if rgb[0, 0, 0] == 0:
            raise cv2.error("SB cannot process this frame")
        return uv + rgb[0, 0, 0]

    monkeypatch.setattr(drift.MEASURE, "checkerboard_corners", detect)

    def capture():
        index = len(calls)
        calls.append(index)
        if index == 0 and problem == "missing_buffers":
            return {"rgb": None, "z": None, "seg": {"data": None}}, {}
        return {
            "rgb": np.full((10, 10, 3), index, np.uint8),
            "z": np.full((10, 10), 3.0, np.float32),
            "seg": {
                "data": np.ones((10, 10, 1), np.uint32),
                "info": {"idToLabels": {"1": {"class": "calibration_board"}}},
            },
        }, {}

    board = drift.collect_board(tmp_path, 0, capture, uv)
    assert len(calls) == 12
    assert board["captures"][0]["measurement_error"]
    assert board["captures"][1]["vs_first"]["count"] == 0
    assert board["captures"][1]["vs_previous"]["count"] == 0
    assert board["captures"][2]["vs_previous"]["mean_delta_uv_px"] == [1.0, 1.0]
    with np.load(tmp_path / "board_p0_capture00.npz", allow_pickle=False) as raw:
        for name in raw.files:
            assert raw[name].dtype != object
    with np.load(tmp_path / "board_p0_capture01.npz", allow_pickle=False) as raw:
        assert raw["segmentation"].shape == (10, 10, 1)
