"""The videos' x264 preset (lead decision 2026-10-27): crf 20 kept for the deck's Isaac videos, the preset an option
(default veryfast; runs before it encoded with x264's own medium), recorded with the camera; the encoders shared the
job's cores with Isaac -- three per riib session."""
import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import run_transport  # noqa: E402
from near_tracking import RUN_SPECIFIC_ARGUMENTS  # noqa: E402

SOURCE = (ROOT / "sim/isaac/run_transport.py").read_text()


def _ffmpeg_lists():
    """Every list literal in the runner that starts an ffmpeg command."""
    out = []
    for node in ast.walk(ast.parse(SOURCE)):
        if isinstance(node, ast.List) and node.elts and isinstance(node.elts[0], ast.Constant) \
                and node.elts[0].value == "ffmpeg":
            out.append(node)
    return out


def test_every_x264_encoder_takes_the_preset_and_keeps_crf_20():
    lists = _ffmpeg_lists()
    assert len(lists) == 3  # the overview, the extra views, open_encoder (the robot camera)
    for node in lists:
        words = [ast.unparse(e) for e in node.elts]
        i = words.index("'libx264'")
        assert words[i + 1] == "'-preset'" and words[i + 2] in ("preset", "video_preset_of(args)")
        assert words[words.index("'-crf'") + 1] == "'20'"


def test_open_encoder_passes_its_preset(monkeypatch, tmp_path):
    seen = []
    monkeypatch.setattr(run_transport.subprocess, "Popen", lambda argv, **k: seen.append(argv))
    run_transport.open_encoder(tmp_path / "a.mp4", 640, 480, 30, "ultrafast")
    run_transport.open_encoder(tmp_path / "b.mp4", 640, 480, 30)
    assert seen[0][seen[0].index("-preset") + 1] == "ultrafast"
    assert seen[1][seen[1].index("-preset") + 1] == run_transport.VIDEO_PRESET_DEFAULT == "veryfast"


def test_the_option_defaults_to_veryfast_is_recorded_and_is_run_specific(monkeypatch):
    base = ["run_transport.py", "--base-scene", "s.usda", "--output", "/nonexistent/o", "--seed", "0",
            "--pallet-urdf", str(ROOT / "sim/models/epal6_pallet/pallet.urdf"),
            "--pallet-geometry", str(ROOT / "config/pallet_geometry_epal6.yaml"),
            "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf"),
            "--settings", str(ROOT / "config/isaac_transport_measured.yaml")]
    monkeypatch.setattr(sys, "argv", [*base, "--video-preset", "ultrafast"])
    assert run_transport.arguments().video_preset == "ultrafast"
    monkeypatch.setattr(sys, "argv", base)
    args = run_transport.arguments()
    assert args.video_preset == "veryfast"
    assert "video_preset" in RUN_SPECIFIC_ARGUMENTS  # the near-field contract compares runs, not encoders
    assert '"encoder": {"codec": "libx264", "crf": 20, "preset": video_preset_of(args)}' in SOURCE
    # the frame log: off by default and then not recorded, recorded when asked
    assert args.frame_log is False and "frame_log" not in run_transport.recorded_arguments(args)
    args.frame_log = True
    assert run_transport.recorded_arguments(args)["frame_log"] is True


def test_a_namespace_from_before_the_option_encodes_with_medium():
    # Codex: a boundary replay restores the recorded Namespace, which has no video_preset
    import argparse

    assert run_transport.video_preset_of(argparse.Namespace()) == "medium"
    assert run_transport.video_preset_of(argparse.Namespace(video_preset="ultrafast")) == "ultrafast"


def test_the_comparers_treat_the_encoder_and_the_frame_log_as_records():
    # Codex: the D8b reference comparison and the A/B comparison have lists of their own
    from tools import mission_ab_compare, s4_drive_terms

    assert {"video_preset", "frame_log"} <= set(s4_drive_terms.ARGUMENTS_BY_DESIGN)
    assert {"video_preset", "frame_log"} <= mission_ab_compare.RECORDING_ONLY_ARGUMENTS
    assert "frame_log" in RUN_SPECIFIC_ARGUMENTS


def test_the_frame_log_has_its_own_guard():
    # Codex: without the extra views the riib bundle's progress watch (run/frames.jsonl) would see nothing
    assert 'if args.extra_views or getattr(args, "frame_log", False):' in SOURCE
    block = SOURCE[SOURCE.index('            if frame_log is not None:\n                write_frame_log(state["frames"] - 1)'):]
    assert block.index("write_frame_log(") < block.index("frame_audit.append(")
    assert "frame_log.write(" in SOURCE[SOURCE.index("def write_frame_log("):]


def test_without_video_the_frame_log_still_gets_a_line_per_frame_tick():
    # riib VIDEO=off (lead 2026-10-28): nothing renders for video, the bundle still watches run/frames.jsonl
    assert "pose_due = frame_log is not None and not args.video and tick % fps_divisor == 0" in SOURCE
    step = SOURCE[SOURCE.index("        def step_world(render: bool) -> None:"):]
    step = step[: step.index("\n        def ", 10)]
    assert step.count("elif pose_due:\n") == 2 and step.count("write_frame_log(tick // fps_divisor)") == 2
    # the video path is unchanged: with --video every frame tick writes the video frame (and its line) as before
    assert step.count("if frame_due:\n") == 2
