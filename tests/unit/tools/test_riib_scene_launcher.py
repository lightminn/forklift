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
    for name in ("line_2", "line_2_m", "aisle", "aisle_rack", "trailer"):
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
    assert (tmp_path / "out/riib_defaults.txt").read_text().startswith("MODE=slam NEAR=on MARKERS=on RACK=on KEEPOUT=0 ")
    assert contains(args, ["--lidar-keepout-m", "0"])  # P1c off outside the rack aisle


def test_the_older_combinations_are_chosen_explicitly(tmp_path):
    args = args_of(dry_run(tmp_path, NEAR="off", MARKERS="off"))
    assert not any(a.startswith("--near-capture") for a in args)
    assert args[args.index("--scene-file") + 1].endswith("line_2.json")
    truth = dry_run(tmp_path, MODE="truth")
    targs = args_of(truth)
    assert "--use-perception" not in targs and not any(a.startswith("--near-capture") for a in targs)
    assert "no effect in MODE=truth" in truth.stderr


def test_scenes_without_copies_run_as_they_are(tmp_path):
    args = args_of(dry_run(tmp_path, scene="trailer"))
    assert args[args.index("--scene-file") + 1].endswith("trailer.json")


def test_the_aisle_runs_its_rack_copy_by_default_and_the_plain_scene_with_rack_off(tmp_path):
    args = args_of(dry_run(tmp_path, scene="aisle"))
    assert args[args.index("--scene-file") + 1].endswith("aisle_rack.json")
    plain = args_of(dry_run(tmp_path, scene="aisle", RACK="off"))
    assert plain[plain.index("--scene-file") + 1].endswith("aisle.json")


def test_the_start_band_release_is_off_unless_asked(tmp_path):
    assert contains(args_of(dry_run(tmp_path)), ["--start-band-release", "off"])
    assert " START_BAND=off " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, START_BAND="on")), ["--start-band-release", "on"])


def test_the_low_clearance_guard_is_on_unless_turned_off(tmp_path):
    # docs/validation/2026-10-25-low-clearance-guard.md
    assert contains(args_of(dry_run(tmp_path)), ["--low-clearance-guard", "on"])
    assert " LOW_CLEARANCE=on " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, LOW_CLEARANCE="off")), ["--low-clearance-guard", "off"])


def test_the_restart_confirmation_is_off_unless_asked(tmp_path):
    # docs/plans/2026-10-25-restart-confirmation.md: 0 until the Isaac comparison
    assert contains(args_of(dry_run(tmp_path)), ["--restart-confirm-scans", "0"])
    assert " RESTART_CONFIRM=0 " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, RESTART_CONFIRM="3")), ["--restart-confirm-scans", "3"])


def test_the_keepout_can_be_turned_on(tmp_path):
    assert contains(args_of(dry_run(tmp_path, KEEPOUT="0.15")), ["--lidar-keepout-m", "0.15"])
    assert contains(args_of(dry_run(tmp_path, KEEPOUT="0.10")), ["--lidar-keepout-m", "0.10"])


def test_the_rack_aisle_keeps_its_uprights_wider_by_default_and_the_other_scenes_do_not(tmp_path):
    # docs/validation/2026-10-24-start-band-release.md: 0.15 for aisle_rack only, any KEEPOUT overrides it
    assert contains(args_of(dry_run(tmp_path, scene="aisle")), ["--lidar-keepout-m", "0.15"])
    assert " KEEPOUT=0.15 " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, scene="aisle", KEEPOUT="0")), ["--lidar-keepout-m", "0"])
    assert contains(args_of(dry_run(tmp_path, scene="aisle", RACK="off")), ["--lidar-keepout-m", "0"])
    assert contains(args_of(dry_run(tmp_path, scene="trailer")), ["--lidar-keepout-m", "0"])


def test_the_obstacle_layer_keeps_the_constant_noise_and_draws_the_a2_spec_on_request(tmp_path):
    # P3a: const by default (user decision 2026-10-24 -- the A2 scan also feeds SLAM and the scan docking), a2 on request
    out = dry_run(tmp_path)
    assert contains(args_of(out), ["--obstacle-layer", "config/obstacle_layer_single.yaml"])
    assert " NOISE=const " in (tmp_path / "out/riib_defaults.txt").read_text()
    a2 = args_of(dry_run(tmp_path, NOISE="a2"))
    assert contains(a2, ["--obstacle-layer", "config/obstacle_layer_single_a2.yaml"])


@pytest.mark.parametrize("env", [{"RESTART_CONFIRM": "-1"}, {"RESTART_CONFIRM": "x"}, {"LOW_CLEARANCE": "maybe"}, {"MODE": "fast"}, {"NEAR": "maybe"}, {"KEEPOUT": "-0.1"}, {"KEEPOUT": "abc"}, {"SCENE": "line_2_m", "MARKERS": "off"},
                                 {"SCENE": "line_9"}, {"RACK": "maybe"}, {"SCENE": "aisle_rack", "RACK": "off"},
                                 {"SCENE": "aisle_out"}, {"NOISE": "loud"}, {"START_BAND": "maybe"}])
def test_bad_choices_are_refused(tmp_path, env):
    out = dry_run(tmp_path, **env)
    assert out.returncode != 0


@pytest.mark.parametrize("env", [{}, {"MODE": "truth"}, {"NEAR": "off", "MARKERS": "off"}, {"NOISE": "a2"}])
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
    assert parsed.layout == "scene" and parsed.kit_arguments == []  # every argument is the runner's own
    # the settings every Riibotics run had before the defaults (the earlier riib.sbatch / riib_truth.sbatch)
    assert (parsed.fps, parsed.max_sim_seconds, parsed.delivery_straight_m, parsed.alignment_straight_m,
            parsed.withdrawal_m, parsed.insertion_reserve_m) == (60, 1500.0, 1.5, 2.1, 0.55, 0.016)
    assert parsed.record_slam and parsed.return_home and parsed.obstacle_act and parsed.d7_planner
    layer = "config/obstacle_layer_single_a2.yaml" if env.get("NOISE") == "a2" else "config/obstacle_layer_single.yaml"
    assert str(parsed.obstacle_layer).endswith(layer)
    if env.get("MODE") == "truth":
        assert not parsed.use_perception and not parsed.d7_docking and parsed.recognition == "off"
    else:
        assert parsed.use_perception and parsed.d7_docking and parsed.planning_target == "perception"
        assert (parsed.recognition, parsed.recognition_proposals) == ("far", "learned")
        assert parsed.perception_mount == "carriage_low_measured"
    if env.get("MODE") != "truth" and env.get("NEAR", "on") == "on":
        assert parsed.near_capture_expected == "on" and parsed.near_capture_lower == "waive_far_confirmed"
        assert parsed.near_capture_redraw == "always" and parsed.near_capture_redraw_lateral_m == pytest.approx(0.10)
    else:
        assert parsed.near_capture_expected == "off" and parsed.near_capture_lower == "gate"


def test_paths_are_made_absolute_before_the_cd(tmp_path):
    out = dry_run(tmp_path, SCENE_DIR=os.path.relpath(tmp_path / "scenes"))
    assert out.returncode == 0, out.stderr
    args = args_of(out)
    assert Path(args[args.index("--scene-file") + 1]).is_absolute()


def test_a_used_run_directory_is_refused(tmp_path):
    (tmp_path / "out/run").mkdir(parents=True)
    env = {"DRY_RUN": "0", "MODE": "truth"}
    out = dry_run(tmp_path, **env)
    assert out.returncode == 2 and "earlier run" in out.stdout


def test_the_runner_exit_code_is_the_jobs(tmp_path):
    # a fake runner that fails: the launcher must not report success
    fake = tmp_path / "fakepy"
    fake.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\nexit 7\n")
    fake.chmod(0o755)
    out = dry_run(tmp_path, DRY_RUN="0", MODE="truth", VENV_PY=str(fake))
    assert out.returncode == 7, (out.returncode, out.stdout, out.stderr)


@pytest.mark.parametrize("recorded, expected", [(0, 0), (3, 3)])
def test_the_runners_recorded_code_counts_when_the_process_ended_with_zero(tmp_path, recorded, expected):
    # Codex re-review P2: the runner writes run/exit_code as JSON {"code": .., "reason": ..}
    fake = tmp_path / "fakepy"
    fake.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\n"
                    "for a in \"$@\"; do if [ \"$prev\" = --output ]; then mkdir -p \"$a\"; "
                    f"echo '{{\"code\": {recorded}, \"reason\": null}}' > \"$a/exit_code\"; fi; prev=$a; done\nexit 0\n")
    fake.chmod(0o755)
    out = dry_run(tmp_path, DRY_RUN="0", MODE="truth", VENV_PY=str(fake))
    assert out.returncode == expected, (out.returncode, out.stdout, out.stderr)


def test_a_bundle_names_and_pins_each_run_s_bridge():
    # several runs share one Slurm job in a bundle (tools/isaac_bundle.py): the bridge container takes its name from
    # SESSION_ID and its cpuset from BRIDGE_CPU, the job ID and the job's first CPU otherwise
    text = LAUNCHER.read_text()
    assert "NAME_B=slam_bridge_${SESSION_ID:-${SLURM_JOB_ID:-$$}}_riib" in text
    assert "CPU0=${BRIDGE_CPU:-$(taskset -cp $$" in text
    assert 'trap \'kill -TERM "$RUNNER" 2>/dev/null\' TERM INT' in text
