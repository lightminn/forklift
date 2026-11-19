"""The Riibotics launcher's side-room copies and bottom-deck switch (docs/plans/2026-11-18-bottom-deck-structure-default.md
section 3-2): SIDE maps line / random to their `_side` copies, BOTTOM_DECK=auto turns `--bottom-deck-check structure`
on only where it is defined (a side copy, the slam mode, no second near capture, no near tracking / pocket check /
repeated captures / live), anything else off with the reason recorded; an explicit structure that cannot run is
refused before Isaac. BOTTOM_DECK stays off by default until the Isaac check (the plan's merge order), so SIDE=auto keeps
the originals."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = ROOT / "deploy/riibotics/riib_scene.sbatch"
SCENES = ("line_1_m", "line_2", "line_2_m", "line_2_side", "line_2_m_side", "random_1_m_side", "aisle", "aisle_rack",
          "trailer")


def dry_run(tmp_path, scene="line_2", **env):
    scenes = tmp_path / "scenes"
    scenes.mkdir(exist_ok=True)
    for name in SCENES:
        (scenes / f"{name}.json").write_text(json.dumps({"views": {"quarter_eye": [1, 2, 3], "quarter_target": [4, 5, 6]}}))
    (tmp_path / "weights.onnx").write_bytes(b"")
    base = {"SNAP": str(ROOT), "SCENE_DIR": str(scenes), "SCENE": scene, "BASE_SCENE": "scene.usda",
            "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": sys.executable,
            "ONNX": str(tmp_path / "weights.onnx"), "LEARNED_PY": sys.executable, "DRY_RUN": "1", "DRY_RUN_CHECK": "0",
            "PATH": os.environ["PATH"]}
    base.update(env)
    return subprocess.run(["bash", str(LAUNCHER), str(tmp_path / "out")], env=base, capture_output=True, text=True)


def args_of(out):
    lines = out.stdout.splitlines()
    return lines[lines.index("RUNNER_ARGS") + 1:]


def scene_of(args):
    return Path(args[args.index("--scene-file") + 1]).name


def check_of(args):
    return args[args.index("--bottom-deck-check") + 1] if "--bottom-deck-check" in args else "off"


def recorded(tmp_path):
    return (tmp_path / "out/riib_defaults.txt").read_text()


def test_by_default_nothing_changes_no_side_copy_and_no_check(tmp_path):
    out = dry_run(tmp_path)
    assert out.returncode == 0, out.stderr
    args = args_of(out)
    assert scene_of(args) == "line_2_m.json" and check_of(args) == "off"
    assert "BOTTOM_DECK=off BOTTOM_DECK_FROM=launcher BOTTOM_DECK_REASON=default_off" in recorded(tmp_path)
    assert "SIDE=off" in recorded(tmp_path)


@pytest.mark.parametrize("scene, markers, expected", [("line_2", "on", "line_2_m_side.json"),
                                                      ("line_2", "off", "line_2_side.json"),
                                                      ("line_2_m_side", "on", "line_2_m_side.json"),
                                                      ("aisle", "on", "aisle_rack.json"),
                                                      ("trailer", "on", "trailer.json")])
def test_side_on_takes_the_side_copy_of_line_and_random_only(tmp_path, scene, markers, expected):
    out = dry_run(tmp_path, scene=scene, SIDE="on", MARKERS=markers)
    assert out.returncode == 0, out.stderr
    assert scene_of(args_of(out)) == expected


def test_a_side_copy_named_with_side_off_is_refused(tmp_path):
    out = dry_run(tmp_path, scene="line_2_m_side", SIDE="off")
    assert out.returncode == 2 and "SIDE=off" in out.stdout


def test_the_per_scene_defaults_follow_the_scene_without_its_side_suffix(tmp_path):
    # line_2_m's degenerate-continue allow list holds for its side copy as well
    args = args_of(dry_run(tmp_path, SIDE="on"))
    assert args[args.index("--docking-degenerate-continue") + 1] == "on"


def test_auto_turns_structure_on_for_a_side_copy_in_the_slam_mode(tmp_path):
    out = dry_run(tmp_path, SIDE="on", BOTTOM_DECK="auto")
    assert out.returncode == 0, out.stderr
    assert check_of(args_of(out)) == "structure"
    assert "BOTTOM_DECK=structure BOTTOM_DECK_FROM=launcher BOTTOM_DECK_REASON=auto" in recorded(tmp_path)


@pytest.mark.parametrize("env, reason, scene_file", [
    ({"SIDE": "off"}, "auto_off:side_off", "line_2_m.json"),
    ({"SCENE": "aisle"}, "auto_off:no_side_copy_for_scene", "aisle_rack.json"),
    ({"MODE": "truth"}, "auto_off:mode_truth", "line_2_m.json"),
    ({"NEAR_RECAPTURE_GAP": "1.0"}, "auto_off:near_recapture_gap", "line_2_m.json"),
    ({"EXTRA_ARGS": "--near-recapture-gap-m=1.0"}, "auto_off:near_recapture_gap", "line_2_m.json"),
    ({"EXTRA_ARGS": "--near-recapt 1.0"}, "auto_off:near_recapture_gap", "line_2_m.json"),
    ({"EXTRA_ARGS": "--near-r 1.0"}, "auto_off:near_recapture_gap", "line_2_m.json"),  # the runner's shortest prefix
    ({"EXTRA_ARGS": "--pocket-check"}, "auto_off:extra_args:--pocket-check", "line_2_m.json"),
    ({"EXTRA_ARGS": "--repeat-captures 1"}, "auto_off:extra_args:--repeat-captures", "line_2_m.json"),
    ({"EXTRA_ARGS": "--repeat-capt 1 --repeat-at-attempt 1"}, "auto_off:extra_args:--repeat-captures", "line_2_m.json"),
    ({"EXTRA_ARGS": "--live"}, "auto_off:extra_args:--live", "line_2_m.json"),
    # an older scene folder without the side copy keeps the original
    ({"SCENE": "line_1"}, "auto_off:no_side_copy_file", "line_1_m.json"),
    # SIDE=on keeps the copy for a paired arm even where the check stays off
    ({"SIDE": "on", "MODE": "truth"}, "auto_off:mode_truth", "line_2_m_side.json"),
])
def test_auto_stays_off_on_the_original_scene_where_the_check_is_not_defined_and_says_why(tmp_path, env, reason,
                                                                                          scene_file):
    env = dict(env)
    scene = env.pop("SCENE", "line_2")
    out = dry_run(tmp_path, scene=scene, BOTTOM_DECK="auto", **env)
    assert out.returncode == 0, out.stdout + out.stderr
    args = args_of(out)
    assert check_of(args) == "off" and scene_of(args) == scene_file
    assert f"BOTTOM_DECK=off BOTTOM_DECK_FROM=launcher BOTTOM_DECK_REASON={reason}" in recorded(tmp_path)


@pytest.mark.parametrize("zero", ["--repeat-captures=0", "--repeat-captures 00", "--repeat-capt=+0"])
def test_auto_takes_the_side_copy_itself_and_a_zero_repeat_count_does_not_block_it(tmp_path, zero):
    out = dry_run(tmp_path, BOTTOM_DECK="auto", EXTRA_ARGS=zero)
    assert out.returncode == 0, out.stderr
    args = args_of(out)
    assert check_of(args) == "structure" and scene_of(args) == "line_2_m_side.json"
    assert "SIDE=on" in recorded(tmp_path)


@pytest.mark.parametrize("env", [{"MODE": "truth"}, {"NEAR_RECAPTURE_GAP": "1.0"}, {"EXTRA_ARGS": "--pocket-check"},
                                 {"EXTRA_ARGS": "--live"}, {"EXTRA_ARGS": "--near-recapture-gap-m 1.0"}])
@pytest.mark.parametrize("mode", ["structure", "on"])
def test_an_explicit_check_that_cannot_run_is_refused_before_isaac(tmp_path, env, mode):
    out = dry_run(tmp_path, SIDE="on", BOTTOM_DECK=mode, **env)
    assert out.returncode == 2 and "BOTTOM_DECK" in out.stdout


def test_an_explicit_check_runs_on_a_scene_auto_would_leave_off(tmp_path):
    # a deliberate run (e.g. a plain copy whose side stands do not fit): the runner's own checks decide
    out = dry_run(tmp_path, BOTTOM_DECK="structure")
    assert out.returncode == 0, out.stderr
    assert check_of(args_of(out)) == "structure"
    assert "BOTTOM_DECK=structure BOTTOM_DECK_FROM=launcher BOTTOM_DECK_REASON=explicit" in recorded(tmp_path)


@pytest.mark.parametrize("extra", ["--bottom-deck-check structure", "--bottom-deck-check=off", "--bottom-deck structure",
                                   "--bo=off", "--bo structure"])
def test_the_check_has_one_switch(tmp_path, extra):
    out = dry_run(tmp_path, EXTRA_ARGS=extra)
    assert out.returncode == 2 and "BOTTOM_DECK" in out.stdout


@pytest.mark.parametrize("value", ["maybe", "true"])
def test_bad_switch_values_are_refused(tmp_path, value):
    """SIDE takes auto / on / off, BOTTOM_DECK auto / structure / on / off."""
    assert dry_run(tmp_path, BOTTOM_DECK=value).returncode == 2
    assert dry_run(tmp_path, SIDE=value).returncode == 2


def test_the_runner_parser_takes_structure_on_a_real_side_copy(tmp_path):
    out = dry_run(tmp_path, SIDE="on", BOTTOM_DECK="auto")
    args = args_of(out)
    sys.path.insert(0, str(ROOT / "sim/isaac"))
    import run_transport

    from tools import riibotics_isaac_scenes as tool

    scene_file = Path(args[args.index("--scene-file") + 1])
    record = tool.scene_record("line_2_m", tool.with_slot_markers("line", tool.scenes()["line_2"]))
    scene_file.write_text(json.dumps(tool.side_room_record(record)))
    argv, cwd = sys.argv, os.getcwd()
    sys.argv = ["run_transport.py", *args, "--output", str(tmp_path / "run")]
    os.chdir(ROOT)
    try:
        parsed = run_transport.arguments()
    finally:
        sys.argv = argv
        os.chdir(cwd)
    assert parsed.bottom_deck_check == "structure" and parsed.kit_arguments == []
    assert parsed.recognition == "far" and parsed.near_capture_lower == "waive"


def test_a_side_name_of_a_scene_without_a_copy_is_refused_and_side_on_records_the_file_run(tmp_path):
    out = dry_run(tmp_path, scene="aisle_side", BOTTOM_DECK="auto")
    assert out.returncode == 2 and "no side-room copy" in out.stdout
    out = dry_run(tmp_path, scene="aisle", SIDE="on")
    assert out.returncode == 0, out.stderr
    assert scene_of(args_of(out)) == "aisle_rack.json" and "SIDE=off" in recorded(tmp_path)


def test_extra_args_reach_the_runner_as_the_scan_read_them_unglobbed(tmp_path):
    # a glob that matches a file under SNAP (the launcher cds there) stays the literal word
    word = "config/pallet_prior_epal[6].yaml"
    out = dry_run(tmp_path, EXTRA_ARGS=f"--pallet-prior {word}")
    assert out.returncode == 0, out.stderr
    assert word in args_of(out)
