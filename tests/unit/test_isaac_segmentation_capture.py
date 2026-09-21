"""G1 semantic readiness and failure evidence without an Isaac runtime."""

import importlib.util
import json
import runpy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]
K = PinholeIntrinsics(8, 6, 10, 10, 4, 3, "optical")


@pytest.fixture
def measure():
    spec = importlib.util.spec_from_file_location(
        "g1_segmentation_test", ROOT / "sim/isaac/camera_calibration.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def payload(data=None, *, labels=None):
    return {
        "data": np.zeros((6, 8), dtype=np.uint32) if data is None else data,
        "info": {
            "idToLabels": labels
            if labels is not None
            else {
                "0": {"class": "BACKGROUND"},
                "1": {"class": "UNLABELLED"},
                "7": {"class": "axis_marker"},
                "91": {"class": ["axis_marker", "other"]},
            }
        },
    }


def marker_pixels():
    data = np.zeros((6, 8), dtype=np.uint32)
    data[2:4, 3:5] = 7
    data[2, 3] = 91
    return data


class FakeCapture:
    """External annotators publish different channels on each paused render."""

    def __init__(self, frames, *, bright=True, shape=(6, 8)):
        self.frames, self.steps = frames, []
        self.rgb = np.zeros((*shape, 3), dtype=np.uint8)
        if bright:
            self.rgb[2:4, 3:5] = 255
        self.rep = SimpleNamespace(orchestrator=SimpleNamespace(step=self.step))
        self.annotators = {
            "rgb": SimpleNamespace(get_data=lambda: self.rgb),
            "seg": SimpleNamespace(get_data=self.seg),
            "z": SimpleNamespace(
                get_data=lambda: np.full(shape, len(self.steps), dtype=float)
            ),
        }

    def step(self, **kwargs):
        self.steps.append(kwargs)

    def seg(self):
        return self.frames[min(len(self.steps), len(self.frames)) - 1]


def test_label_metadata_alone_is_not_ready_then_all_channels_are_recaptured(measure):
    fake = FakeCapture([payload(), payload(marker_pixels())])
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker", max_steps=4
    )
    report = frame["capture_diagnostics"]
    assert report["ready"] is True
    assert report["first_ready_step"] == 2
    assert report["attempts"][0]["segmentation"]["status"] == "all_background"
    assert report["attempts"][0]["segmentation"]["label_pixel_counts"] == {
        "BACKGROUND": 48,
        "UNLABELLED": 0,
        "axis_marker": 0,
        "other": 0,
    }
    final = report["attempts"][1]["segmentation"]
    assert final["id_pixel_counts"] == {"0": 44, "7": 3, "91": 1}
    assert final["label_pixel_counts"]["axis_marker"] == 4
    assert final["target_bbox_xyxy"] == [3, 2, 4, 3]
    assert report["attempts"][1]["bright_inside_target_count"] == 4
    assert np.all(frame["z"] == 2)  # Never combine old depth with new semantics.
    assert (
        fake.steps
        == [{"rt_subframes": 4, "delta_time": 0.0, "pause_timeline": True}] * 2
    )
    # Annotator-owned arrays/maps must not change saved data after return.
    fake.frames[-1]["data"][:] = 0
    fake.frames[-1]["info"]["idToLabels"].clear()
    assert frame["seg"]["data"][2, 3] == 91
    assert report["attempts"][1]["segmentation"]["id_to_labels"]["7"] == {
        "class": "axis_marker"
    }


def test_ready_first_step_does_not_add_unnecessary_renders(measure):
    fake = FakeCapture([payload(marker_pixels())])
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker"
    )
    measure.require_capture_semantics(frame)
    assert frame["capture_diagnostics"]["first_ready_step"] == 1
    assert len(fake.steps) == 1


@pytest.mark.parametrize("bright", [True, False])
def test_background_exhaustion_is_bounded_and_failure_reason_keeps_both_signals(
    measure, bright
):
    fake = FakeCapture([payload()], bright=bright)
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker", max_steps=3
    )
    report = frame["capture_diagnostics"]
    assert report["ready"] is False
    assert report["first_ready_step"] is None
    assert len(report["attempts"]) == len(fake.steps) == 3
    with pytest.raises(ValueError, match="segmentation_all_background") as exc:
        measure.require_capture_semantics(frame)
    assert ("no_bright_pixels" in str(exc.value)) is (not bright)
    assert np.count_nonzero(frame["seg"]["data"]) == 0  # No RGB fallback mask.


@pytest.mark.parametrize(
    ("seg", "status"),
    [
        (None, "missing_buffer"),
        ({}, "missing_buffer"),
        (payload(np.empty((0, 0), dtype=np.uint32)), "empty_buffer"),
        (payload(np.zeros((6, 8, 4), dtype=np.uint8)), "invalid_format"),
        (payload(np.zeros((6, 8), dtype=float)), "invalid_format"),
        (payload(np.zeros((8, 6), dtype=np.uint32)), "shape_mismatch"),
        (
            payload(marker_pixels(), labels={"0": {"class": "BACKGROUND"}}),
            "target_label_missing",
        ),
        (payload(np.ones((6, 8), dtype=np.uint32)), "target_mask_empty"),
        (payload(np.full((6, 8), 876, dtype=np.uint32)), "target_mask_empty"),
    ],
)
def test_bad_semantic_buffers_are_diagnosed_without_reinterpretation(
    measure, seg, status
):
    fake = FakeCapture([seg])
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker", max_steps=2
    )
    report = frame["capture_diagnostics"]
    assert report["ready"] is False
    assert report["attempts"][-1]["segmentation"]["status"] == status
    with pytest.raises(ValueError, match="segmentation_" + status):
        measure.require_capture_semantics(frame)
    json.dumps(report, allow_nan=False)


def test_singleton_channel_and_arbitrary_ids_are_valid(measure):
    fake = FakeCapture([payload(marker_pixels()[..., None])])
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker"
    )
    assert frame["capture_diagnostics"]["ready"] is True
    assert frame["capture_diagnostics"]["attempts"][0]["segmentation"][
        "data_shape"
    ] == [6, 8, 1]


def test_semantic_readiness_does_not_waive_missing_bright_pixels(measure):
    fake = FakeCapture([payload(marker_pixels())], bright=False)
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker"
    )
    assert frame["capture_diagnostics"]["ready"] is True
    with pytest.raises(ValueError, match="no_bright_pixels") as exc:
        measure.require_capture_semantics(frame)
    assert "segmentation_" not in str(exc.value)
    assert len(fake.steps) == 1  # Do not hunt for a brighter frame.


@pytest.mark.parametrize("budget", [0, -1, True, 1.5])
def test_invalid_step_budget_is_rejected_before_rendering(measure, budget):
    fake = FakeCapture([payload()])
    with pytest.raises(ValueError, match="max_steps"):
        measure.capture_semantic_static(
            fake.rep, fake.annotators, target_label="axis_marker", max_steps=budget
        )
    assert fake.steps == []


@pytest.mark.parametrize(
    ("has_mask", "bright", "missing"),
    [
        (False, True, ["segmentation_target_mask_empty"]),
        (True, False, ["no_bright_pixels"]),
        (False, False, ["segmentation_target_mask_empty", "no_bright_pixels"]),
    ],
)
def test_marker_centroid_failure_names_each_missing_signal(
    measure, has_mask, bright, missing
):
    fake = FakeCapture([], bright=bright)
    mask = marker_pixels() > 0 if has_mask else np.zeros((6, 8), dtype=bool)
    with pytest.raises(ValueError) as exc:
        measure.marker_centroids(fake.rgb, mask, np.array([4, 3]), 1.25, K)
    for reason in missing:
        assert reason in str(exc.value)
    if has_mask:
        assert "segmentation_target_mask_empty" not in str(exc.value)
    if bright:
        assert "no_bright_pixels" not in str(exc.value)


def test_saved_failed_capture_retains_step_evidence_and_raw_segmentation(
    measure, tmp_path
):
    save = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))[
        "save_capture"
    ]
    fake = FakeCapture(
        [payload(np.zeros((480, 640), dtype=np.uint32))], shape=(480, 640)
    )
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker", max_steps=2
    )
    save(tmp_path, "marker_0", frame, {"target_prim": {"path": "/World/Marker"}})
    record = json.loads((tmp_path / "marker_0_capture.json").read_text())
    assert record["capture_diagnostics"]["ready"] is False
    assert len(record["capture_diagnostics"]["attempts"]) == 2
    assert record["orchestrator_step"]["count"] == 2
    assert record["target_prim"]["path"] == "/World/Marker"
    with np.load(tmp_path / "marker_0_arrays.npz") as arrays:
        assert arrays["segmentation"].dtype == np.uint32
        assert not arrays["segmentation"].any()


def test_missing_semantic_payload_does_not_erase_failure_evidence(measure, tmp_path):
    save = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))[
        "save_capture"
    ]
    fake = FakeCapture([None], bright=False, shape=(480, 640))
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker", max_steps=2
    )
    save(tmp_path, "marker_0", frame, {})
    record = json.loads((tmp_path / "marker_0_capture.json").read_text())
    last = record["capture_diagnostics"]["attempts"][-1]
    assert last["segmentation"]["status"] == "missing_buffer"
    assert last["bright_count"] == 0
    assert (tmp_path / "marker_0_rgb.png").exists()
    with np.load(tmp_path / "marker_0_arrays.npz") as arrays:
        assert "segmentation" not in arrays  # Never fabricate missing data.
        assert arrays["depth_m"].shape == (480, 640)
    with pytest.raises(
        ValueError, match="segmentation_missing_buffer; no_bright_pixels"
    ):
        measure.require_capture_semantics(frame)


def test_saved_target_mask_uses_semantics_not_bright_pixels(measure, tmp_path):
    from PIL import Image

    save = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))[
        "save_capture"
    ]
    data = np.zeros((480, 640), dtype=np.uint32)
    data[100:110, 200:210] = 7  # Deliberately disjoint from FakeCapture's highlights.
    fake = FakeCapture([payload(data)], shape=(480, 640))
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker"
    )
    save(tmp_path, "marker_0", frame, {})
    with Image.open(tmp_path / "marker_0_target_mask.png") as image:
        mask = np.asarray(image)
        assert np.count_nonzero(mask) == 100
        assert mask[100, 200] == 255
        assert mask[2, 3] == 0


@pytest.mark.parametrize("depth_value", [np.inf, -np.inf, np.nan])
def test_black_rgb_and_nonfinite_depth_report_empty_render_not_missing_label(
    measure, depth_value
):
    fake = FakeCapture([payload()], bright=False)
    fake.annotators["z"].get_data = lambda: np.full((6, 8), depth_value)
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="calibration_board", max_steps=8
    )
    with pytest.raises(ValueError, match="empty_render_frame") as exc:
        measure.require_capture_semantics(frame)
    assert "segmentation_all_background" not in str(exc.value)
    report = frame["capture_diagnostics"]
    assert len(report["attempts"]) == 8
    assert report["empty_render_steps"] == list(range(1, 9))
    for attempt in report["attempts"]:
        health = attempt["render"]
        assert health["rgb"]["mean"] == health["rgb"]["std"] == 0
        assert health["depth"]["finite_fraction"] == 0
        assert health["depth"]["min_m"] is None
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize("bright,depth_value", [(True, np.inf), (False, 1.25)])
def test_one_missing_render_signal_is_not_a_wholly_empty_frame(
    measure, bright, depth_value
):
    fake = FakeCapture([payload()], bright=bright)
    fake.annotators["z"].get_data = lambda: np.full((6, 8), depth_value)
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="calibration_board", max_steps=2
    )
    assert frame["capture_diagnostics"]["empty_render_steps"] == []
    with pytest.raises(ValueError, match="segmentation_all_background"):
        measure.require_capture_semantics(frame)


def test_alpha_is_not_render_evidence_and_stale_semantics_cannot_waive_empty_rgbd(
    measure,
):
    fake = FakeCapture([payload(marker_pixels())], bright=False)
    fake.rgb = np.zeros((6, 8, 4), dtype=np.uint8)
    fake.rgb[..., 3] = 255
    fake.annotators["z"].get_data = lambda: np.full((6, 8, 1), np.inf)
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker"
    )
    # The existing semantic readiness criterion and step budget stay unchanged.
    assert frame["capture_diagnostics"]["first_ready_step"] == 1
    with pytest.raises(ValueError, match="empty_render_frame"):
        measure.require_capture_semantics(frame)


def test_each_step_retains_render_health_and_pipeline_state_before_and_after(measure):
    fake = FakeCapture([payload(), payload(marker_pixels())])
    fake.annotators["z"].get_data = lambda: np.full(
        (6, 8), np.inf if len(fake.steps) == 1 else 1.25
    )
    original_rgb = fake.rgb.copy()
    fake.annotators["rgb"].get_data = lambda: (
        np.zeros_like(original_rgb) if len(fake.steps) == 1 else original_rgb
    )
    state = {"steps": 0}

    def snapshot():
        state["steps"] = len(fake.steps)
        return state  # A changing external object must not overwrite history.

    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="axis_marker", pipeline_state=snapshot
    )
    report = frame["capture_diagnostics"]
    assert report["first_ready_step"] == 2
    assert report["empty_render_steps"] == [1]
    assert report["attempts"][0]["pipeline_before"] == {"steps": 0}
    assert report["attempts"][0]["pipeline_after"] == {"steps": 1}
    assert report["attempts"][1]["pipeline_after"] == {"steps": 2}
    assert report["attempts"][1]["render"]["depth"]["finite_fraction"] == 1
    assert report["attempts"][1]["target_depth_finite_fraction"] == 1


def test_saved_empty_capture_keeps_rgb_depth_and_new_failure_reason(measure, tmp_path):
    save = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))[
        "save_capture"
    ]
    fake = FakeCapture([None], bright=False, shape=(480, 640))
    fake.annotators["z"].get_data = lambda: np.full((480, 640), np.inf)
    frame = measure.capture_semantic_static(
        fake.rep, fake.annotators, target_label="calibration_board", max_steps=2
    )
    save(tmp_path, "board", frame, {})
    record = json.loads((tmp_path / "board_capture.json").read_text())
    assert record["capture_diagnostics"]["failure_reason"] == "empty_render_frame"
    assert (tmp_path / "board_rgb.png").exists()
    with np.load(tmp_path / "board_arrays.npz") as arrays:
        assert np.isinf(arrays["depth_m"]).all()
