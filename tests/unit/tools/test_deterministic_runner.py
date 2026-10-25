"""The runner's side of the deterministic-run switch (docs/plans/2026-10-25-deterministic-sim-mode.md D0, D1, D4):
the argument and its near-field contract entry, the far driving frames' pixel lag set to zero (synced frames do not
trail), every camera read synced first, the SLAM static layer kept out of deterministic planning, and the run's
record."""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

SOURCE = (ROOT / "sim/isaac/run_transport.py").read_text()


def test_off_by_default_and_on_with_the_switch():
    assert _runner_args([]).deterministic is False
    assert _runner_args(["--deterministic"]).deterministic is True


def test_the_near_field_contract_compares_it_and_reads_its_absence_as_off():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["deterministic"] is False
    assert "deterministic" not in near_tracking.RUN_SPECIFIC_ARGUMENTS


def test_the_far_pixel_lag_is_zero_in_a_deterministic_run_and_the_override_kept():
    far = ["--recognition", "far", "--recognition-pixel-lag-steps", "12", "--slam-feedback", "slam.sock"]
    assert _runner_args(far).recognition_pixel_lag_steps == 12
    args = _runner_args(far + ["--deterministic"])
    assert args.recognition_pixel_lag_steps == 0
    assert args.deterministic_overrides == {"recognition_pixel_lag_steps": 12}
    assert _runner_args(["--deterministic"]).deterministic_overrides == {}


def test_a_deterministic_run_takes_the_oracle_masks_instead_of_the_learned_proposer(tmp_path):
    # plan D1b: RTX RGB differs between processes even on the same timeline (probes v7-v9), the learned proposer reads
    # RGB; the oracle instance masks come from the segmentation, identical once the frame is synced
    onnx = tmp_path / "model.onnx"
    onnx.write_bytes(b"")
    far = ["--recognition", "far", "--slam-feedback", "slam.sock", "--recognition-proposals", "learned",
           "--recognition-onnx", str(onnx), "--recognition-tau", "0.3",
           "--recognition-learned-python", "/usr/bin/python3"]
    assert _runner_args(far).recognition_proposals == "learned"
    args = _runner_args(far + ["--deterministic"])
    assert args.recognition_proposals == "oracle"
    assert args.deterministic_overrides == {"recognition_proposals": "learned"}
    oracle = _runner_args(["--recognition", "far", "--slam-feedback", "slam.sock", "--deterministic"])
    assert oracle.recognition_proposals == "oracle" and oracle.deterministic_overrides == {}
    # the learned proposer's own inputs are not asked for when the deterministic run replaces it (Codex P3)
    bare = _runner_args(["--recognition", "far", "--slam-feedback", "slam.sock", "--recognition-proposals", "learned",
                         "--deterministic"])
    assert bare.recognition_proposals == "oracle" and bare.deterministic_overrides == {"recognition_proposals": "learned"}


def test_a_deterministic_run_refuses_a_boundary_record():
    # boundary records keep RGB by content hash (arrays_*.npz members, image_save pixel entries), which the strict
    # comparison cannot tell from the rest -- the two are not combined (plan D1b, Codex)
    import pytest

    with pytest.raises(SystemExit):
        _runner_args(["--deterministic", "--record-boundary"])


def _body(name: str) -> str:
    """The nested function's source, up to the next def at its own indentation or less."""
    m = re.search(rf"\n( *)def {name}\(", SOURCE)
    end = re.compile(rf"\n {{0,{len(m.group(1))}}}def ").search(SOURCE, m.end())
    return SOURCE[m.start():end.start()]


def test_every_camera_read_is_synced_before_its_pixels():
    far = _body("far_drive_frame")
    assert far.index("frame_sync.sync(perception_camera") < far.index("perception_camera.get_current_frame()")
    pocket = _body("read_pocket_frame")
    assert pocket.index("frame_sync.sync(pocket_camera") < pocket.index("pocket_camera.get_depth()")
    video = _body("write_video_frame")
    assert video.index("frame_sync.sync(camera") < video.index("camera.get_current_frame()")
    capture = SOURCE[SOURCE.index("perception_capture = adapter.SensorCapture("):][:900]
    assert "frame_sync.sync(perception_camera" in capture


def test_rgb_is_read_through_rgba_of_where_it_is_recorded():
    """The direct getter's RGB differs between processes even after the sync (2026-10-25 probe); a deterministic run
    reads the current frame's own rgba (frame_sync.rgba_of)."""
    assert "frame_sync_rgba(perception_camera)" in _body("far_drive_frame")
    assert "frame_sync_rgba(camera)" in _body("write_video_frame")
    adapter = (ROOT / "sim/isaac/perception_adapter.py").read_text()
    assert "np.array(_rgba_of(camera), copy=True)" in adapter


def test_deterministic_planning_does_not_read_the_slam_map():
    block = SOURCE[SOURCE.index('latest = args.slam_map_dir / "latest_map.npz"') - 400:]
    assert re.search(r"elif layer_\.memory is not None and args\.slam_map_dir is not None and "
                     r"not getattr\(args, \"deterministic\", False\):", block)
    assert '"slam_static_layer": "off_deterministic"' in SOURCE


def test_the_run_records_the_mode_its_broken_reasons_and_diagnostics():
    assert "determinism.configure(getattr(args, \"deterministic\", False))" in SOURCE
    assert 'state["deterministic"]' in SOURCE and "determinism.broken()" in SOURCE
    assert "frame_sync.summary()" in SOURCE
