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
NEAR = ["--near-capture-expected", "on", "--near-capture-lower", "waive", "--near-capture-redraw", "always",
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


def test_the_escape_start_release_is_off_unless_asked(tmp_path):
    # docs/plans/2026-10-25-escape-start-release.md: the escape's sweep alone, apart from START_BAND
    assert contains(args_of(dry_run(tmp_path)), ["--escape-start-release", "off"])
    assert " ESCAPE_START=off " in (tmp_path / "out/riib_defaults.txt").read_text()
    on = args_of(dry_run(tmp_path, ESCAPE_START="on"))
    assert contains(on, ["--escape-start-release", "on"]) and contains(on, ["--start-band-release", "off"])


def test_the_low_clearance_guard_is_on_unless_turned_off(tmp_path):
    # docs/validation/2026-10-25-low-clearance-guard.md
    assert contains(args_of(dry_run(tmp_path)), ["--low-clearance-guard", "on"])
    assert " LOW_CLEARANCE=on " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, LOW_CLEARANCE="off")), ["--low-clearance-guard", "off"])


def test_the_restart_confirmation_takes_three_scans_unless_turned_off(tmp_path):
    # docs/validation/2026-10-26-restart-confirmation.md
    assert contains(args_of(dry_run(tmp_path)), ["--restart-confirm-scans", "3"])
    assert " RESTART_CONFIRM=3 " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, RESTART_CONFIRM="0")), ["--restart-confirm-scans", "0"])


def test_the_d7_reapproach_hold_is_on_unless_turned_off(tmp_path):
    # docs/validation/2026-10-25-reapproach-slam-hold.md V5 (registered in plan 2a: scene launchers on, runner off)
    assert contains(args_of(dry_run(tmp_path)), ["--d7-reapproach-hold", "on"])
    assert " D7_HOLD=on " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, D7_HOLD="off")), ["--d7-reapproach-hold", "off"])
    assert contains(args_of(dry_run(tmp_path, MODE="truth")), ["--d7-reapproach-hold", "on"])


def test_the_slam_mode_is_said_and_recorded(tmp_path):
    # live-default plan 5d: the live console localizes by default; this launcher maps online and says so, never silently
    out = dry_run(tmp_path)
    assert "slam: mapping (riib_scene.sbatch maps online" in out.stdout
    assert "\nSLAM=mapping (" in (tmp_path / "out/riib_defaults.txt").read_text()
    assert not any(a.startswith("--slam-map-bundle") for a in args_of(out))
    dry_run(tmp_path, MODE="truth")
    assert "\nSLAM=none (MODE=truth)" in (tmp_path / "out/riib_defaults.txt").read_text()


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
    out = dry_run(tmp_path, ENCODER_MODEL="reported")
    assert contains(args_of(out), ["--obstacle-layer", "config/obstacle_layer_single.yaml"])
    assert " NOISE=const " in (tmp_path / "out/riib_defaults.txt").read_text()
    a2 = args_of(dry_run(tmp_path, NOISE="a2", ENCODER_MODEL="reported"))
    assert contains(a2, ["--obstacle-layer", "config/obstacle_layer_single_a2.yaml"])


@pytest.mark.parametrize("noise, encoder, layer", [
    ("const", None, "config/obstacle_layer_single_team_spec.yaml"),
    ("a2", None, "config/obstacle_layer_single_a2_team_spec.yaml"),
    ("const", "reported", "config/obstacle_layer_single.yaml"),
    ("a2", "reported", "config/obstacle_layer_single_a2.yaml"),
    ("const", "team_spec_quantized", "config/obstacle_layer_single_team_spec.yaml")])
def test_the_odometry_is_the_joint_angle_model_by_default_with_its_age_table(tmp_path, noise, encoder, layer):
    """Plan 2026-10-25-team-spec-default (lead decision): the joint-angle encoders by default, reported on request;
    the layer is the NOISE row with the model's own odometry age table (the runner refuses a mismatched table)."""
    env = {"NOISE": noise, **({} if encoder is None else {"ENCODER_MODEL": encoder})}
    args = args_of(dry_run(tmp_path, **env))
    assert contains(args, ["--encoder-model", encoder or "team_spec_quantized"])
    assert contains(args, ["--obstacle-layer", layer])
    assert f" ENCODER_MODEL={encoder or 'team_spec_quantized'}" in (tmp_path / "out/riib_defaults.txt").read_text()
    truth = args_of(dry_run(tmp_path, MODE="truth", **env))
    assert contains(truth, ["--encoder-model", encoder or "team_spec_quantized"]) and contains(truth, ["--obstacle-layer", layer])


GYRO_MODEL = "config/gyro_model_d435i_static_20261023.yaml"
GYRO_LAYER = "config/obstacle_layer_single_team_spec_gyro_d435i_static.yaml"


def test_the_gyro_heading_is_off_unless_asked_and_measured_turns_it_on_with_its_age_table(tmp_path):
    """T2's C1 arm (docs/plans/2026-10-27-insertion-lateral-drift.md): GYRO=measured runs the gyro heading on the model
    fitted to the 2026-10-23 static IMU record, with the age table made for that model; off by default."""
    plain = args_of(dry_run(tmp_path))
    assert "--odometry-heading" not in plain and "--gyro-model" not in plain
    assert " GYRO=off" in (tmp_path / "out/riib_defaults.txt").read_text()
    args = args_of(dry_run(tmp_path, GYRO="measured"))
    assert contains(args, ["--odometry-heading", "gyro"]) and contains(args, ["--gyro-model", GYRO_MODEL])
    assert contains(args, ["--obstacle-layer", GYRO_LAYER])
    assert " GYRO=measured" in (tmp_path / "out/riib_defaults.txt").read_text()
    assert (ROOT / GYRO_MODEL).is_file() and (ROOT / GYRO_LAYER).is_file()


@pytest.mark.parametrize("env", [{"GYRO": "maybe"}, {"GYRO": "measured", "MODE": "truth"},
                                 {"GYRO": "measured", "ENCODER_MODEL": "reported"}, {"GYRO": "measured", "NOISE": "a2"}])
def test_the_gyro_heading_is_refused_where_it_is_not_defined(tmp_path, env):
    # The runner's gyro heading needs the perception run and the team-spec encoders; the a2 noise has no gyro table.
    assert dry_run(tmp_path, **env).returncode != 0


@pytest.mark.parametrize("env", [{"RESTART_CONFIRM": "-1"}, {"RESTART_CONFIRM": "x"}, {"LOW_CLEARANCE": "maybe"}, {"MODE": "fast"}, {"NEAR": "maybe"}, {"KEEPOUT": "-0.1"}, {"KEEPOUT": "abc"}, {"SCENE": "line_2_m", "MARKERS": "off"},
                                 {"SCENE": "line_9"}, {"RACK": "maybe"}, {"SCENE": "aisle_rack", "RACK": "off"},
                                 {"SCENE": "aisle_out"}, {"NOISE": "loud"}, {"START_BAND": "maybe"},
                                 {"ESCAPE_START": "maybe"}, {"D7_HOLD": "maybe"},
                                 {"ENCODER_MODEL": "team_spec_typical_inl"}, {"ENCODER_MODEL": "x"},
                                 {"NEAR_RECAPTURE_GAP": "-1"}, {"NEAR_RECAPTURE_GAP": "x"},
                                 {"NEAR_RECAPTURE_GAP": "1.0", "NEAR": "off"}, {"NEAR_RECAPTURE_GAP": "1.0", "MODE": "truth"}])
def test_bad_choices_are_refused(tmp_path, env):
    out = dry_run(tmp_path, **env)
    assert out.returncode != 0


@pytest.mark.parametrize("env", [{}, {"MODE": "truth"}, {"NEAR": "off", "MARKERS": "off"}, {"NOISE": "a2"},
                                 {"ENCODER_MODEL": "reported"}, {"NEAR_RECAPTURE_GAP": "1.0"}, {"GYRO": "measured"}])
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
    encoder = env.get("ENCODER_MODEL", "team_spec_quantized")
    age = "" if encoder == "reported" else "_team_spec"
    layer = f"config/obstacle_layer_single{'_a2' if env.get('NOISE') == 'a2' else ''}{age}.yaml"
    if env.get("GYRO") == "measured":
        layer = GYRO_LAYER
        assert parsed.odometry_heading == "gyro" and str(parsed.gyro_model).endswith(GYRO_MODEL)
    else:
        assert parsed.odometry_heading == "steering" and parsed.gyro_model is None
    assert str(parsed.obstacle_layer).endswith(layer) and parsed.encoder_model == encoder
    if env.get("MODE") == "truth":
        assert not parsed.use_perception and not parsed.d7_docking and parsed.recognition == "off"
    else:
        assert parsed.use_perception and parsed.d7_docking and parsed.planning_target == "perception"
        assert (parsed.recognition, parsed.recognition_proposals) == ("far", "learned")
        assert parsed.perception_mount == "carriage_low_measured"
    if env.get("MODE") != "truth" and env.get("NEAR", "on") == "on":
        assert parsed.near_capture_expected == "on" and parsed.near_capture_lower == "waive"
        assert parsed.near_capture_redraw == "always" and parsed.near_capture_redraw_lateral_m == pytest.approx(0.10)
    else:
        assert parsed.near_capture_expected == "off" and parsed.near_capture_lower == "gate"
    # C3: the second near capture only when asked
    assert parsed.near_recapture_gap_m == (float(env["NEAR_RECAPTURE_GAP"]) if "NEAR_RECAPTURE_GAP" in env else None)


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


def test_the_second_near_capture_is_off_unless_a_gap_is_given(tmp_path):
    assert "--near-recapture-gap-m" not in args_of(dry_run(tmp_path))
    args = args_of(dry_run(tmp_path, NEAR_RECAPTURE_GAP="1.0"))
    assert contains(args, ["--near-recapture-gap-m", "1.0"])
