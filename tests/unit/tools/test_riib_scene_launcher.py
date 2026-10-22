"""The Riibotics scene launcher's defaults (deploy/riibotics/riib_scene.sbatch, user decision 2026-10-22): PR #88's near
options on and the slot-marker copies for line / random, with the older combinations chosen explicitly; the arguments it
builds are the runner's own."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = ROOT / "deploy/riibotics/riib_scene.sbatch"
NEAR = ["--near-capture-expected", "on", "--near-capture-lower", "waive_far_confirmed", "--near-capture-redraw", "always",
        "--near-capture-redraw-lateral-m", "0.10"]


def dry_run(tmp_path, scene="line_2", **env):
    scenes = tmp_path / "scenes"
    scenes.mkdir(exist_ok=True)
    for name in ("line_2", "line_2_m", "aisle"):
        (scenes / f"{name}.json").write_text(json.dumps({"views": {"quarter_eye": [1, 2, 3], "quarter_target": [4, 5, 6]}}))
    base = {"SNAP": str(ROOT), "SCENE_DIR": str(scenes), "SCENE": scene, "BASE_SCENE": "scene.usda",
            "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": sys.executable,
            "ONNX": str(tmp_path / "weights.onnx"), "LEARNED_PY": sys.executable, "DRY_RUN": "1",
            "PATH": os.environ["PATH"]}
    (tmp_path / "weights.onnx").write_bytes(b"")
    base.update(env)
    out = subprocess.run(["bash", str(LAUNCHER), str(tmp_path / "out")], env=base, capture_output=True, text=True)
    return out


def args_of(out):
    lines = out.stdout.splitlines()
    return lines[lines.index("RUNNER_ARGS") + 1:]


def contains(args, seq):
    return any(args[i:i + len(seq)] == seq for i in range(len(args)))


def test_the_defaults_turn_the_near_options_and_the_marker_copy_on(tmp_path):
    out = dry_run(tmp_path)
    assert out.returncode == 0, out.stderr
    args = args_of(out)
    assert contains(args, NEAR)
    assert args[args.index("--scene-file") + 1].endswith("line_2_m.json")
    assert contains(args, ["--recognition", "far"]) and contains(args, ["--d7-docking"])
    assert (tmp_path / "out/riib_defaults.txt").read_text().startswith("MODE=slam NEAR=on MARKERS=on")


def test_the_older_combinations_are_chosen_explicitly(tmp_path):
    args = args_of(dry_run(tmp_path, NEAR="off", MARKERS="off"))
    assert not any(a.startswith("--near-capture") for a in args)
    assert args[args.index("--scene-file") + 1].endswith("line_2.json")
    truth = dry_run(tmp_path, MODE="truth")
    targs = args_of(truth)
    assert "--use-perception" not in targs and not any(a.startswith("--near-capture") for a in targs)
    assert "no effect in MODE=truth" in truth.stderr


def test_scenes_without_slot_markers_run_as_they_are(tmp_path):
    args = args_of(dry_run(tmp_path, scene="aisle"))
    assert args[args.index("--scene-file") + 1].endswith("aisle.json")


@pytest.mark.parametrize("env", [{"MODE": "fast"}, {"NEAR": "maybe"}, {"SCENE": "line_2_m", "MARKERS": "off"},
                                 {"SCENE": "line_9"}])
def test_bad_choices_are_refused(tmp_path, env):
    out = dry_run(tmp_path, **env)
    assert out.returncode != 0


@pytest.mark.parametrize("env", [{}, {"MODE": "truth"}, {"NEAR": "off", "MARKERS": "off"}])
def test_the_runner_parser_takes_the_built_arguments(tmp_path, env):
    args = args_of(dry_run(tmp_path, **env))
    sys.path.insert(0, str(ROOT / "sim/isaac"))
    import run_transport

    scene_file = Path(args[args.index("--scene-file") + 1])
    from tools import riibotics_isaac_scenes as tool  # a real scene file for the parser's scene checks

    name = scene_file.stem
    built = tool.scenes()["line_2"]
    record = tool.scene_record(name, tool.with_slot_markers("line", built) if name.endswith("_m") else built)
    scene_file.write_text(json.dumps(record))
    argv = sys.argv
    sys.argv = ["run_transport.py", *args, "--output", str(tmp_path / "run")]
    cwd = os.getcwd()
    os.chdir(ROOT)
    try:
        parsed = run_transport.arguments()
    finally:
        sys.argv = argv
        os.chdir(cwd)
    assert parsed.layout == "scene"
    if not env:
        assert parsed.near_capture_expected == "on" and parsed.near_capture_lower == "waive_far_confirmed"
        assert parsed.near_capture_redraw == "always" and parsed.near_capture_redraw_lateral_m == pytest.approx(0.10)
