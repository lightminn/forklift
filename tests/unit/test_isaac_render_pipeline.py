"""CPU evidence for clipping and capture-pipeline diagnostics; no Isaac startup."""

import json
import runpy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
VERIFY = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))
MEASURE = VERIFY["MEASURE"]


class ClipCamera:
    def __init__(self, near=1.0, far=1e6, *, ignore_set=False):
        self.clip = (near, far)
        self.ignore_set = ignore_set

    def get_clipping_range(self):
        return self.clip

    def set_clipping_range(self, *, near_distance):
        if not self.ignore_set:
            self.clip = (near_distance, self.clip[1])


@pytest.fixture
def board():
    from forklift_core.sensors.rgbd import PinholeIntrinsics

    k = PinholeIntrinsics(640, 480, 465.741156, 465.741156, 320, 240, "optical")
    optical_from_base = np.array(
        [[0, -1, 0, 0], [0, 0, -1, 0.5], [1, 0, 0, -0.75], [0, 0, 0, 1]]
    )
    base_from_optical = np.linalg.inv(optical_from_base)
    layout = MEASURE.checkerboard_layout(0.8, (100, 80), k, base_from_optical)
    return layout["vertices_base_m"], base_from_optical @ np.diag([1, -1, -1, 1])


def test_first_board_is_completely_before_usd_default_near_clip(board):
    vertices, camera_world = board
    report = MEASURE.target_clipping_diagnostics(vertices, camera_world, (1.0, 1e6))
    assert report["optical_z_min_m"] == pytest.approx(0.72335893, abs=1e-8)
    assert report["optical_z_max_m"] == pytest.approx(0.72335893, abs=1e-8)
    assert report["all_before_near"] is True
    assert report["vertices_in_depth_range"] == 0
    json.dumps(report, allow_nan=False)


def test_clip_fix_changes_only_near_and_requires_readback(board):
    vertices, camera_world = board
    camera = ClipCamera()
    report = MEASURE.target_clipping_diagnostics(
        vertices, camera_world, camera.get_clipping_range()
    )
    adjustment = MEASURE.lower_near_clip_for_target(camera, report)
    assert adjustment["applied"] is True
    assert camera.clip[0] == pytest.approx(0.361679465, abs=1e-8)
    assert camera.clip[1] == 1e6
    assert adjustment["before_m"] == [1.0, 1e6]
    assert adjustment["after_m"] == list(camera.clip)
    assert MEASURE.target_clipping_diagnostics(vertices, camera_world, camera.clip)[
        "vertices_in_depth_range"
    ] == len(vertices)


@pytest.mark.parametrize("clip", [(0.05, 200), (1.0, 1.1)])
def test_in_range_or_behind_camera_target_does_not_trigger_speculative_clip_fix(
    board, clip
):
    vertices, camera_world = board
    if clip == (1.0, 1.1):
        vertices = vertices - [3, 0, 0]
    camera = ClipCamera(*clip)
    report = MEASURE.target_clipping_diagnostics(vertices, camera_world, clip)
    assert MEASURE.lower_near_clip_for_target(camera, report)["applied"] is False
    assert camera.clip == clip


def test_ignored_clip_setter_is_not_reported_as_a_fix(board):
    vertices, camera_world = board
    camera = ClipCamera(ignore_set=True)
    report = MEASURE.target_clipping_diagnostics(vertices, camera_world, camera.clip)
    with pytest.raises(ValueError, match="clipping readback"):
        MEASURE.lower_near_clip_for_target(camera, report)


def test_same_step_fresh_annotator_probe_distinguishes_old_empty_stream():
    steps = []
    rep = SimpleNamespace(
        orchestrator=SimpleNamespace(step=lambda **kw: steps.append(kw))
    )

    def stream(fresh):
        return {
            "rgb": SimpleNamespace(
                get_data=lambda: np.full((6, 8, 3), 100 if fresh else 0, np.uint8)
            ),
            "z": SimpleNamespace(
                get_data=lambda: np.full((6, 8), len(steps) if fresh else np.inf)
            ),
            "seg": SimpleNamespace(get_data=lambda: None),
        }

    report, frames = MEASURE.compare_annotator_streams(
        rep, stream(False), stream(True), max_steps=3, pipeline_state=lambda: {}
    )
    assert len(report["attempts"]) == len(steps) == 3
    assert report["observation"] == "existing_empty_fresh_nonempty"
    assert report["attempts"][-1]["fresh"]["depth"]["min_m"] == 3
    assert np.all(frames["fresh"]["z"] == 3)
    assert np.isinf(frames["existing"]["z"]).all()
    assert all(
        s == dict(rt_subframes=4, delta_time=0.0, pause_timeline=True) for s in steps
    )


@pytest.mark.parametrize("valid", [True, False])
def test_pipeline_snapshot_distinguishes_stopped_timeline_and_invalid_product(valid):
    class Prim:
        def IsValid(self):
            return valid

        def IsActive(self):
            return True

        def GetTypeName(self):
            return "RenderProduct"

        def GetAttribute(self, name):
            return SimpleNamespace(Get=lambda: (640, 480))

        def GetRelationship(self, name):
            return SimpleNamespace(GetTargets=lambda: ["/Camera"])

    camera = ClipCamera()
    camera.get_render_product_path = lambda: "/Render/Test"
    stage = SimpleNamespace(GetPrimAtPath=lambda _: Prim())
    timeline = SimpleNamespace(
        is_playing=lambda: False, is_stopped=lambda: True, get_current_time=lambda: 0.5
    )
    rep = SimpleNamespace(orchestrator=SimpleNamespace(get_status=lambda: "PAUSED"))
    snapshot = VERIFY["render_pipeline_state"](
        stage, camera, timeline, rep, attached_product="/Render/Original"
    )
    assert snapshot["timeline"]["is_stopped"] is True
    assert snapshot["timeline"]["is_playing"] is False
    assert snapshot["render_product"]["valid"] is valid
    assert snapshot["render_product"]["matches_attached_product"] is False
    assert snapshot["camera_clipping_range_m"] == [1.0, 1e6]
    json.dumps(snapshot, allow_nan=False)


@pytest.mark.parametrize("recovers", [True, False])
def test_board_clip_experiment_preserves_before_capture_and_reports_actual_recovery(
    board, recovers
):
    vertices, camera_world = board
    camera = ClipCamera()
    captures, result = [], {}

    def capture(name, path, label, *, validate=True):
        captures.append((name, camera.clip, validate))
        if validate and not recovers:
            raise ValueError("empty_render_frame")
        status = "nonempty" if validate else "empty_render_frame"
        return (
            {
                "capture_diagnostics": {
                    "attempts": [
                        {
                            "render": {
                                "status": status,
                                "rgb": {"max": 255 if validate else 0},
                            },
                            "target_depth_finite_fraction": 1.0 if validate else 0.0,
                        }
                    ]
                }
            },
            1,
            2,
        )

    run = VERIFY["capture_board_with_clip_check"]
    if recovers:
        assert run(camera, vertices, camera_world, capture, "board", "/Board", result)[
            1:
        ] == (1, 2)
    else:
        with pytest.raises(ValueError, match="empty_render_frame"):
            run(camera, vertices, camera_world, capture, "board", "/Board", result)
    assert captures[0] == ("board_before_near_clip", (1.0, 1e6), False)
    assert captures[1][0] == "board"
    assert captures[1][1][0] == pytest.approx(0.361679465, abs=1e-8)
    experiment = result["near_clip_experiments"][0]
    assert experiment["before_capture"] == "board_before_near_clip"
    assert experiment["observation"] == (
        "target_recovered_after_near_only_change"
        if recovers
        else "recovery_not_confirmed"
    )


def test_board_with_sufficient_clip_range_does_not_add_steps_or_change_camera(board):
    vertices, camera_world = board
    camera = ClipCamera(0.05, 200)
    captures, result = [], {}

    def capture(*args):
        captures.append(args)
        return "original capture result"

    assert (
        VERIFY["capture_board_with_clip_check"](
            camera, vertices, camera_world, capture, "board", "/Board", result
        )
        == "original capture result"
    )
    assert captures == [("board", "/Board", "calibration_board")]
    assert camera.clip == (0.05, 200)
    assert result == {}


def test_clip_experiment_does_not_claim_rgbd_recovery_from_semantics_alone(board):
    vertices, camera_world = board
    camera, result = ClipCamera(), {}

    def capture(*args, **kwargs):
        return (
            {
                "capture_diagnostics": {
                    "attempts": [
                        {
                            "render": {"status": "nonempty", "rgb": {"max": 255}},
                            "target_depth_finite_fraction": 0.0,
                        }
                    ]
                }
            },
            1,
            2,
        )

    VERIFY["capture_board_with_clip_check"](
        camera, vertices, camera_world, capture, "board", "/Board", result
    )
    assert result["near_clip_experiments"][0]["observation"] == "recovery_not_confirmed"
