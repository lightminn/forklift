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


def dry_run(tmp_path, scene="line_2", out=None, **env):
    scenes = tmp_path / "scenes"
    scenes.mkdir(exist_ok=True)
    for name in ("line_2", "line_2_m", "aisle", "aisle_rack", "trailer"):
        (scenes / f"{name}.json").write_text(json.dumps({"views": {"quarter_eye": [1, 2, 3], "quarter_target": [4, 5, 6]}}))
    base = {"SNAP": str(ROOT), "SCENE_DIR": str(scenes), "SCENE": scene, "BASE_SCENE": "scene.usda",
            "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": sys.executable,
            "ONNX": str(tmp_path / "weights.onnx"), "LEARNED_PY": sys.executable, "DRY_RUN": "1", "DRY_RUN_CHECK": "0",
            "PATH": os.environ["PATH"]}
    (tmp_path / "weights.onnx").write_bytes(b"")
    base.update(env)
    return subprocess.run(["bash", str(LAUNCHER), str(out or tmp_path / "out")], env=base, capture_output=True, text=True)


# a fake runner's answer to the launcher's preflight (--check-args): its checks pass
FAKE_CHECK = 'for a in "$@"; do [ "$a" = --check-args ] && { echo ARGS_OK; exit 0; }; done\n'


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


def test_the_start_band_release_and_the_escape_presteer_are_off_unless_asked(tmp_path):
    # docs/validation/2026-10-28-escape-presteer.md: off after the C4 comparison; on gives the width (0.03 by default)
    off = args_of(dry_run(tmp_path))
    assert contains(off, ["--start-band-release", "off"]) and "--start-band-m" not in off
    assert contains(off, ["--escape-presteer", "off"])
    defaults = (tmp_path / "out/riib_defaults.txt").read_text()
    assert " START_BAND=off " in defaults and " START_BAND_M=0.03 " in defaults and " ESCAPE_PRESTEER=off " in defaults
    on = args_of(dry_run(tmp_path, START_BAND="on", ESCAPE_PRESTEER="on", out=tmp_path / "on"))
    assert contains(on, ["--start-band-release", "on"]) and contains(on, ["--start-band-m", "0.03"])
    assert contains(on, ["--escape-presteer", "on"])
    width = args_of(dry_run(tmp_path, START_BAND="on", START_BAND_M="0.02", out=tmp_path / "w"))
    assert contains(width, ["--start-band-m", "0.02"])


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


def test_the_exit_prefix_exemption_is_on_in_the_slam_mode_unless_turned_off(tmp_path):
    # docs/validation/2026-10-31-riib-default-batch2.md: rule ① passed for MODE=slam (the batch's mode); MODE=truth
    # was not in it, so it keeps the exemption off unless asked
    assert contains(args_of(dry_run(tmp_path)), ["--exit-prefix-exempt", "on"])
    assert " EXIT_EXEMPT=on " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, EXIT_EXEMPT="off")), ["--exit-prefix-exempt", "off"])
    assert contains(args_of(dry_run(tmp_path, MODE="truth")), ["--exit-prefix-exempt", "off"])
    assert contains(args_of(dry_run(tmp_path, MODE="truth", EXIT_EXEMPT="on")), ["--exit-prefix-exempt", "on"])


def test_the_return_start_backoff_is_boxed_in_the_slam_mode_unless_turned_off(tmp_path):
    # docs/validation/2026-11-02-return-start-backoff-boxed-batch.md: the F2'' batch passed in MODE=slam (the batch's
    # mode); MODE=truth was not in it, so it keeps the backoff off unless asked
    assert contains(args_of(dry_run(tmp_path)), ["--return-start-backoff", "boxed"])
    assert " RETURN_START=boxed " in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, RETURN_START="off")), ["--return-start-backoff", "off"])
    assert contains(args_of(dry_run(tmp_path, RETURN_START="on")), ["--return-start-backoff", "on"])
    assert contains(args_of(dry_run(tmp_path, MODE="truth")), ["--return-start-backoff", "off"])
    assert contains(args_of(dry_run(tmp_path, MODE="truth", RETURN_START="boxed")), ["--return-start-backoff", "boxed"])


@pytest.mark.parametrize("env", [{"RETURN_START": "maybe"}, {"EXTRA_ARGS": "--return-start-backoff off"},
                                 {"EXTRA_ARGS": "--return-start-backoff=boxed"}, {"EXTRA_ARGS": "--return-s off"}])
def test_the_return_start_backoff_has_one_switch(tmp_path, env):
    assert dry_run(tmp_path, **env).returncode != 0


@pytest.mark.parametrize("env", [{"EXIT_EXEMPT": "maybe"}, {"EXTRA_ARGS": "--exit-prefix-exempt off"},
                                 {"EXTRA_ARGS": "--exit-prefix-exempt=on"}, {"EXTRA_ARGS": "--exit-prefix-exe off"},
                                 {"EXTRA_ARGS": "--exit-p=off"}, {"EXTRA_ARGS": "--near-tracking"},
                                 {"EXTRA_ARGS": "--near-tracking", "EXIT_EXEMPT": "on"}])
def test_the_exemption_has_one_switch_and_refuses_near_tracking(tmp_path, env):
    # EXIT_EXEMPT is the one switch (riib_defaults.txt records it), abbreviations included -- the runner's parser takes
    # a unique prefix (Codex results review P3); the runner refuses the exemption with
    # --near-tracking (exit-prefix-extract-rules D1), so the launcher says so before Isaac starts
    assert dry_run(tmp_path, **env).returncode != 0


def test_near_tracking_goes_with_the_exemption_turned_off(tmp_path):
    # and with the return cusp replan budget at the 0 its contract was measured at (the slam default is 2)
    # and, on line_2_m, with the degenerate-continue switch off as its contract was measured
    out = dry_run(tmp_path, EXIT_EXEMPT="off", EXTRA_ARGS="--near-tracking", RETURN_CUSP_REPLANS="0",
                  DOCKING_DEGENERATE_CONTINUE="off")
    assert out.returncode == 0, out.stderr
    assert contains(args_of(out), ["--exit-prefix-exempt", "off"])


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
                                 {"START_BAND_M": "0"}, {"START_BAND_M": "x"}, {"ESCAPE_PRESTEER": "maybe"},
                                 {"ESCAPE_START": "maybe"}, {"D7_HOLD": "maybe"}, {"EXIT_EXEMPT": "maybe"},
                                 {"ENCODER_MODEL": "team_spec_typical_inl"}, {"ENCODER_MODEL": "x"},
                                 {"NEAR_RECAPTURE_GAP": "-1"}, {"NEAR_RECAPTURE_GAP": "x"},
                                 {"NEAR_RECAPTURE_GAP": "1.0", "NEAR": "off"}, {"NEAR_RECAPTURE_GAP": "1.0", "MODE": "truth"},
                                 {"NEAR_REDRAW_LATERAL": "-0.05"}, {"NEAR_REDRAW_LATERAL": "x"},
                                 {"NEAR_REDRAW_LATERAL": "0.05", "NEAR": "off"}, {"NEAR_REDRAW_LATERAL": "0.05", "MODE": "truth"}])
def test_bad_choices_are_refused(tmp_path, env):
    out = dry_run(tmp_path, **env)
    assert out.returncode != 0


@pytest.mark.parametrize("env", [{}, {"MODE": "truth"}, {"NEAR": "off", "MARKERS": "off"}, {"NOISE": "a2"},
                                 {"ENCODER_MODEL": "reported"}, {"NEAR_RECAPTURE_GAP": "1.0"}, {"GYRO": "measured"},
                                 {"NEAR_REDRAW_LATERAL": "0.05"}, {"VIDEO": "off"}, {"EXIT_EXEMPT": "off"},
                                 {"MODE": "truth", "EXIT_EXEMPT": "on"}, {"RETURN_START": "on"},
                                 {"MODE": "truth", "RETURN_START": "boxed"}])
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
    video_on = env.get("VIDEO", "on") == "on"
    assert parsed.video is video_on and parsed.frame_log is not video_on and bool(parsed.extra_views) is video_on
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
        lateral = float(env.get("NEAR_REDRAW_LATERAL", "0.10"))
        assert parsed.near_capture_redraw == "always" and parsed.near_capture_redraw_lateral_m == pytest.approx(lateral)
    else:
        assert parsed.near_capture_expected == "off" and parsed.near_capture_lower == "gate"
    # the degenerate-continue allow list: line_2_m in the slam mode only (the parametrised MARKERS=off is plain line_2)
    ddc = "on" if env.get("MODE") != "truth" and env.get("MARKERS", "on") == "on" else "off"
    assert parsed.docking_degenerate_continue == ddc
    # C3: the second near capture only when asked
    assert parsed.near_recapture_gap_m == (float(env["NEAR_RECAPTURE_GAP"]) if "NEAR_RECAPTURE_GAP" in env else None)


def test_paths_are_made_absolute_before_the_cd(tmp_path):
    out = dry_run(tmp_path, SCENE_DIR=os.path.relpath(tmp_path / "scenes"))
    assert out.returncode == 0, out.stderr
    args = args_of(out)
    assert Path(args[args.index("--scene-file") + 1]).is_absolute()


def test_the_dry_run_ends_with_the_runner_s_own_check(tmp_path):
    # preflight plan: DRY_RUN prints the arguments, then the runner checks them (ARGS_OK, or its refusal and exit 2)
    fake = tmp_path / "fakepy"
    fake.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\n"
                    'for a in "$@"; do [ "$a" = --check-args ] && { echo "$@" > "$0.argv"; echo ARGS_OK; exit 0; }; done\n'
                    "exit 9\n")
    fake.chmod(0o755)
    out = dry_run(tmp_path, MODE="truth", VENV_PY=str(fake), DRY_RUN_CHECK="1")
    assert out.returncode == 0 and out.stdout.strip().splitlines()[-1] == "ARGS_OK", out.stdout + out.stderr
    assert f"--output {tmp_path / 'out' / 'run'}" in (tmp_path / "fakepy.argv").read_text()
    refuse = tmp_path / "refuse"
    refuse.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\necho refused >&2; exit 2\n")
    refuse.chmod(0o755)
    assert dry_run(tmp_path, MODE="truth", VENV_PY=str(refuse), DRY_RUN_CHECK="1").returncode == 2


def test_a_run_whose_arguments_the_runner_refuses_ends_before_it_starts(tmp_path):
    refuse = tmp_path / "refuse"
    refuse.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\n"
                      'for a in "$@"; do [ "$a" = --check-args ] && { echo "check-args: nope" >&2; exit 2; }; done\n'
                      "touch \"$0.started\"; exit 0\n")
    refuse.chmod(0o755)
    out = dry_run(tmp_path, DRY_RUN="0", MODE="truth", VENV_PY=str(refuse))
    assert out.returncode == 2 and "nope" in out.stdout and not (tmp_path / "refuse.started").exists()


def test_a_used_run_directory_is_refused(tmp_path):
    (tmp_path / "out/run").mkdir(parents=True)
    env = {"DRY_RUN": "0", "MODE": "truth"}
    out = dry_run(tmp_path, **env)
    assert out.returncode == 2 and "earlier run" in out.stdout


def test_the_runner_exit_code_is_the_jobs(tmp_path):
    # a fake runner that fails: the launcher must not report success
    fake = tmp_path / "fakepy"
    fake.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\n" + FAKE_CHECK + "exit 7\n")
    fake.chmod(0o755)
    out = dry_run(tmp_path, DRY_RUN="0", MODE="truth", VENV_PY=str(fake))
    assert out.returncode == 7, (out.returncode, out.stdout, out.stderr)


@pytest.mark.parametrize("recorded, expected", [(0, 0), (3, 3)])
def test_the_runners_recorded_code_counts_when_the_process_ended_with_zero(tmp_path, recorded, expected):
    # Codex re-review P2: the runner writes run/exit_code as JSON {"code": .., "reason": ..}
    fake = tmp_path / "fakepy"
    fake.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\n" + FAKE_CHECK +
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
    # the runner's TERM: armed before its start and waited for to its end (deploy/lib/term_relay.sh)
    assert text.index("term_relay_arm") < text.index('timeout "${RUN_TIMEOUT:-6600}"') < text.index('term_relay_add "$RUNNER"')


def test_a_snapshot_whose_runner_has_no_check_is_refused(tmp_path):
    # Codex: an older runner takes --check-args for a Kit argument and would start Isaac on the login node
    snap = tmp_path / "snap"
    for rel in ("config", "sim/models", "src", "deploy/lib"):
        (snap / rel).parent.mkdir(parents=True, exist_ok=True)
        (snap / rel).symlink_to(ROOT / rel)
    (snap / "sim/isaac").mkdir(parents=True)
    (snap / "sim/isaac/run_transport.py").write_text("print('an older runner')\n")
    out = dry_run(tmp_path, MODE="truth", SNAP=str(snap), DRY_RUN_CHECK="1")
    assert out.returncode == 2 and "no --check-args" in out.stderr, out.stdout + out.stderr


def test_a_slam_run_s_arguments_are_checked_before_its_bridge_starts(tmp_path):
    # Codex: the check came after the container and its 120 s wait; refused arguments now start no container
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    (bin_ / "docker").write_text(f"#!/bin/bash\necho \"$*\" >> {tmp_path / 'docker_calls'}\n")
    (bin_ / "docker").chmod(0o755)
    refuse = tmp_path / "refuse"
    refuse.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\n"
                      'for a in "$@"; do [ "$a" = --check-args ] && { echo "check-args: nope" >&2; exit 2; }; done\n'
                      "exit 0\n")
    refuse.chmod(0o755)
    out = dry_run(tmp_path, DRY_RUN="0", MODE="slam", VENV_PY=str(refuse), PATH=f"{bin_}:{os.environ['PATH']}")
    assert out.returncode == 2 and "nope" in out.stdout and not (tmp_path / "docker_calls").exists()


def test_the_second_near_capture_is_off_unless_a_gap_is_given(tmp_path):
    assert "--near-recapture-gap-m" not in args_of(dry_run(tmp_path))
    args = args_of(dry_run(tmp_path, NEAR_RECAPTURE_GAP="1.0"))
    assert contains(args, ["--near-recapture-gap-m", "1.0"])


def test_the_near_redraw_lateral_box_stays_0_10_unless_given(tmp_path):
    # near tracking re-measure (plan 2026-10-27-insertion-lateral-drift 5.3): 0.05 is the value --near-tracking takes
    assert contains(args_of(dry_run(tmp_path)), ["--near-capture-redraw-lateral-m", "0.10"])
    assert "NEAR_REDRAW_LATERAL" not in (tmp_path / "out/riib_defaults.txt").read_text()  # earlier records unchanged
    args = args_of(dry_run(tmp_path, NEAR_REDRAW_LATERAL="0.05"))
    assert contains(args, ["--near-capture-redraw-lateral-m", "0.05"])
    assert "--near-capture-redraw-lateral-m" in args and args.count("--near-capture-redraw-lateral-m") == 1
    assert "\nNEAR_REDRAW_LATERAL=0.05\n" in (tmp_path / "out/riib_defaults.txt").read_text()



def test_an_out_too_long_for_the_slam_socket_is_refused(tmp_path):
    out = tmp_path / ("x" * max(1, 100 - len(str(tmp_path))))
    p = dry_run(tmp_path, out=out)
    assert p.returncode == 2 and "AF_UNIX" in p.stderr, p.stdout + p.stderr
    assert dry_run(tmp_path, out=out, MODE="truth").returncode == 0  # a truth run opens no socket


def test_video_stays_on_unless_video_off_drops_every_render_but_the_progress_log(tmp_path):
    # lead 2026-10-28 (near tracking re-measure): VIDEO=off renders no video, so only the near camera renders
    args = args_of(dry_run(tmp_path))
    assert "--video" in args and "--frame-log" not in args
    assert "VIDEO=" not in (tmp_path / "out" / "riib_defaults.txt").read_text()  # the default reads as before
    args = args_of(dry_run(tmp_path, VIDEO="off"))
    assert "--video" not in args and "--extra-views" not in args and not any(a.startswith("--quarter-") for a in args)
    assert "--frame-log" in args  # the bundle's progress watch (run/frames.jsonl) keeps its lines
    record = (tmp_path / "out" / "riib_defaults.txt").read_text()
    assert "\nVIEWS=none\n" in record and "\nVIDEO=off\n" in record
    assert dry_run(tmp_path, VIDEO="off", VIEWS="none").returncode == 0
    for env in ({"VIDEO": "off", "VIEWS": "chase"}, {"VIDEO": "maybe"}):
        assert dry_run(tmp_path, **env).returncode == 2


def test_views_default_to_both_extra_videos_and_none_drops_them(tmp_path):
    # lead 2026-10-27: a comparison batch that needs no videos says VIEWS=none in its list, for both arms
    args = args_of(dry_run(tmp_path))
    assert contains(args, ["--extra-views", "quarter,chase"]) and any(a.startswith("--quarter-eye=") for a in args)
    assert "VIEWS=" not in (tmp_path / "out" / "riib_defaults.txt").read_text()  # the default reads as before
    args = args_of(dry_run(tmp_path, VIEWS="none"))
    assert "--extra-views" not in args and not any(a.startswith("--quarter-") for a in args) and "--video" in args
    assert "--frame-log" in args  # the bundle watches run/frames.jsonl for progress (Codex)
    assert "VIEWS=none" in (tmp_path / "out" / "riib_defaults.txt").read_text()
    args = args_of(dry_run(tmp_path, VIEWS="chase"))
    assert contains(args, ["--extra-views", "chase"]) and not any(a.startswith("--quarter-") for a in args)
    assert dry_run(tmp_path, VIEWS="perception").returncode == 2


@pytest.mark.parametrize("keepout", ["0", "0.0", "00"])
def test_trailer_gets_its_exit_prefix_by_default(tmp_path, keepout):
    # scene requirements plan: the batch that left it out went 0/10 (82ffffa8); 0, 0.0 and 00 are all "no keepout"
    args = args_of(dry_run(tmp_path, scene="trailer", KEEPOUT=keepout))
    assert contains(args, ["--exit-clearance-m", "0.30"])


@pytest.mark.parametrize("env", [{"KEEPOUT": "0.15"}, {"EXTRA_ARGS": "--lidar-keepout-m 0.15"},
                                 {"EXTRA_ARGS": "--exit-clearance-m=0.4"},
                                 # Codex: tabs and newlines split the runner's words too
                                 {"EXTRA_ARGS": "--seed 1\t--lidar-keepout-m 0.15"},
                                 {"EXTRA_ARGS": "--seed 1\n--exit-clearance-m 0.4"}])
def test_trailer_s_default_prefix_gives_way_to_the_keepout_s_or_an_explicit_one(tmp_path, env):
    args = args_of(dry_run(tmp_path, scene="trailer", **env))
    assert not contains(args, ["--exit-clearance-m", "0.30"])  # the default stays out; EXTRA_ARGS's own may be there


def test_an_exit_clearance_a_profile_and_an_override_pass_through(tmp_path):
    args = args_of(dry_run(tmp_path, scene="trailer", EXIT_CLEARANCE="0.40", REQUIREMENTS_PROFILE="ct_baseline",
                           SCENE_REQUIREMENTS_OVERRIDE="a comparison"))
    assert contains(args, ["--exit-clearance-m", "0.40"]) and contains(args, ["--requirements-profile", "ct_baseline"])
    assert contains(args, ["--scene-requirements-override", "a comparison"])
    assert "--exit-clearance-m" not in args_of(dry_run(tmp_path, scene="line_2"))  # other scenes plan as before
    assert dry_run(tmp_path, scene="trailer", EXIT_CLEARANCE="x").returncode == 2


def test_the_real_trailer_and_rack_scenes_pass_the_runner_s_check_with_the_defaults(tmp_path):
    # the runner's own check (DRY_RUN_CHECK=1) on generated scene files, not the minimal ones above (Codex P3)
    from tools import riibotics_isaac_scenes as tool

    scenes = tmp_path / "real"
    scenes.mkdir()
    for name, built in (("trailer", tool.trailer_scene()), ("aisle_rack", tool.rack_scene())):
        (scenes / f"{name}.json").write_text(json.dumps(tool.scene_record(name, built)))
    for scene in ("trailer", "aisle"):
        out = dry_run(tmp_path, scene=scene, out=tmp_path / f"o_{scene}", SCENE_DIR=str(scenes), MODE="truth",
                      DRY_RUN_CHECK="1", PYTHONPATH=str(ROOT / "src"))
        assert out.returncode == 0 and out.stdout.splitlines()[-1] == "ARGS_OK", out.stdout[-800:] + out.stderr
    out = dry_run(tmp_path, scene="trailer", out=tmp_path / "o_none", SCENE_DIR=str(scenes), MODE="truth",
                  DRY_RUN_CHECK="1", PYTHONPATH=str(ROOT / "src"),
                  EXTRA_ARGS="--exit-clearance-m=0.30 --lidar-keepout-m=0.15")
    assert out.returncode == 2 and "scene trailer needs one of" in out.stderr


def test_c2_is_on_in_the_slam_mode_and_off_in_truth_unless_switched(tmp_path):
    # lead decision 2026-11-03: the riib default turns C2 on (docs/validation/2026-11-03-near-end-stand-c2-injection-results.md)
    args = args_of(dry_run(tmp_path))
    assert contains(args, ["--near-end-stand", "on"]) and args.count("--near-end-stand") == 1
    assert "NEAR_END_STAND=on NEAR_END_STAND_FROM=launcher" in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, NEAR_END_STAND="off")), ["--near-end-stand", "off"])
    assert contains(args_of(dry_run(tmp_path, NEAR_END_STAND="shadow")), ["--near-end-stand", "shadow"])
    assert contains(args_of(dry_run(tmp_path, MODE="truth")), ["--near-end-stand", "off"])


@pytest.mark.parametrize("extra, value", [("--near-end-stand shadow", "shadow"), ("--near-end-stand=off", "off"),
                                          ("--near-end-s on --record-boundary", "on")])
def test_c2_named_in_extra_args_wins_and_is_recorded(tmp_path, extra, value):
    out = dry_run(tmp_path, EXTRA_ARGS=extra, NEAR_END_STAND="on")
    assert out.returncode == 0, out.stderr
    args = args_of(out)
    named = [a for a in args if a.startswith("--near-end")]
    assert len(named) == 1  # the launcher adds none of its own: the runner reads one
    assert f"NEAR_END_STAND={value} NEAR_END_STAND_FROM=EXTRA_ARGS" in (tmp_path / "out/riib_defaults.txt").read_text()


@pytest.mark.parametrize("env", [{"NEAR_END_STAND": "maybe"}, {"EXTRA_ARGS": "--near-end-stand maybe"}])
def test_c2_takes_only_its_three_values(tmp_path, env):
    out = dry_run(tmp_path, **env)
    assert out.returncode == 2 and "must be on, shadow or off" in out.stdout


def test_c2_named_twice_in_extra_args_records_the_last_one(tmp_path):
    # Codex promotion review P3: the runner's parser takes the last occurrence; the record says the same
    out = dry_run(tmp_path, EXTRA_ARGS="--near-end-stand shadow --near-end-stand=off")
    assert out.returncode == 0, out.stderr
    assert "NEAR_END_STAND=off NEAR_END_STAND_FROM=EXTRA_ARGS" in (tmp_path / "out/riib_defaults.txt").read_text()
    assert not contains(args_of(out), ["--near-end-stand", "on"])


def test_return_cusp_replans_is_2_in_the_slam_mode_and_0_in_truth_unless_switched(tmp_path):
    # lead decision 2026-11-04: the re-registered batch passed (docs/validation/2026-11-04-return-cusp-replan-reregistration-results.md)
    args = args_of(dry_run(tmp_path))
    assert contains(args, ["--return-cusp-replans", "2"]) and args.count("--return-cusp-replans") == 1
    assert "RETURN_CUSP_REPLANS=2 RETURN_CUSP_REPLANS_FROM=launcher" in (tmp_path / "out/riib_defaults.txt").read_text()
    assert contains(args_of(dry_run(tmp_path, RETURN_CUSP_REPLANS="0")), ["--return-cusp-replans", "0"])
    assert contains(args_of(dry_run(tmp_path, MODE="truth")), ["--return-cusp-replans", "0"])


@pytest.mark.parametrize("extra, value", [("--return-cusp-replans 0", "0"), ("--return-cusp-replans=1", "1"),
                                          ("--return-c 3 --record-boundary", "3")])
def test_return_cusp_replans_named_in_extra_args_wins_and_is_recorded(tmp_path, extra, value):
    out = dry_run(tmp_path, EXTRA_ARGS=extra, RETURN_CUSP_REPLANS="2")
    assert out.returncode == 0, out.stderr
    named = [a for a in args_of(out) if a.startswith("--return-c")]
    assert len(named) == 1  # the launcher adds none of its own: the runner reads one
    text = (tmp_path / "out/riib_defaults.txt").read_text()
    assert f"RETURN_CUSP_REPLANS={value} RETURN_CUSP_REPLANS_FROM=EXTRA_ARGS" in text


@pytest.mark.parametrize("env", [{"RETURN_CUSP_REPLANS": "two"}, {"RETURN_CUSP_REPLANS": "-1"},
                                 {"EXTRA_ARGS": "--return-cusp-replans x"}])
def test_return_cusp_replans_takes_only_a_count(tmp_path, env):
    out = dry_run(tmp_path, **env)
    assert out.returncode == 2 and "must be a count" in out.stdout


def test_return_cusp_replans_named_twice_records_the_last_one(tmp_path):
    out = dry_run(tmp_path, EXTRA_ARGS="--return-cusp-replans 1 --return-cusp-replans=0")
    assert out.returncode == 0, out.stderr
    assert "RETURN_CUSP_REPLANS=0 RETURN_CUSP_REPLANS_FROM=EXTRA_ARGS" in (tmp_path / "out/riib_defaults.txt").read_text()
    assert not contains(args_of(out), ["--return-cusp-replans", "2"])


def test_near_tracking_needs_return_cusp_replans_0(tmp_path):
    # Codex promotion review P3: the near-tracking contract was measured at 0; the launcher says so before the runner does
    out = dry_run(tmp_path, EXTRA_ARGS="--near-tracking", EXIT_EXEMPT="off")
    assert out.returncode == 2 and "--near-tracking needs RETURN_CUSP_REPLANS=0" in out.stdout
    out = dry_run(tmp_path, EXTRA_ARGS="--near-tracking", EXIT_EXEMPT="off", RETURN_CUSP_REPLANS="0")
    assert "needs RETURN_CUSP_REPLANS" not in out.stdout


def test_the_runner_s_check_takes_the_default_budget_and_refuses_a_missing_value(tmp_path):
    # Codex promotion review P3: the real parser (DRY_RUN_CHECK=1), not the launcher's own argument list
    from tools import riibotics_isaac_scenes as tool

    scenes = tmp_path / "real"
    scenes.mkdir()
    (scenes / "aisle_rack.json").write_text(json.dumps(tool.scene_record("aisle_rack", tool.rack_scene())))
    out = dry_run(tmp_path, scene="aisle", out=tmp_path / "o_ok", SCENE_DIR=str(scenes), DRY_RUN_CHECK="1",
                  PYTHONPATH=str(ROOT / "src"))
    assert out.returncode == 0 and out.stdout.splitlines()[-1] == "ARGS_OK", out.stdout[-800:] + out.stderr
    assert contains(args_of(out), ["--return-cusp-replans", "2"])
    out = dry_run(tmp_path, scene="aisle", out=tmp_path / "o_bad", SCENE_DIR=str(scenes), DRY_RUN_CHECK="1",
                  PYTHONPATH=str(ROOT / "src"), EXTRA_ARGS="--return-cusp-replans")
    assert out.returncode == 2 and "--return-cusp-replans: expected one argument" in out.stderr


def ddc_record(tmp_path, out="out"):
    return [l for l in (tmp_path / out / "riib_defaults.txt").read_text().splitlines()
            if l.startswith("DOCKING_DEGENERATE_CONTINUE=")]


@pytest.mark.parametrize("scene, env, value", [
    ("line_2", {}, "on"), ("line_2_m", {}, "on"),
    ("line_2", {"MARKERS": "off"}, "off"), ("line_2", {"MODE": "truth"}, "off"),
    ("aisle", {}, "off"), ("aisle", {"RACK": "off"}, "off"), ("trailer", {}, "off")])
def test_degenerate_continue_is_on_only_for_line_2_m_in_the_slam_mode(tmp_path, scene, env, value):
    # lead decision 2026-11-05 (docs/validation/2026-11-05-line2-robustness-v3.md): an allow list on the final NAME
    out = dry_run(tmp_path, scene=scene, **env)
    assert out.returncode == 0, out.stdout + out.stderr
    args = args_of(out)
    assert contains(args, ["--docking-degenerate-continue", value]) and args.count("--docking-degenerate-continue") == 1
    assert ddc_record(tmp_path) == [f"DOCKING_DEGENERATE_CONTINUE={value} DOCKING_DEGENERATE_CONTINUE_FROM=launcher"]


def test_degenerate_continue_can_be_switched_either_way_where_it_is_defined(tmp_path):
    out = dry_run(tmp_path, out=tmp_path / "a", DOCKING_DEGENERATE_CONTINUE="off")
    assert contains(args_of(out), ["--docking-degenerate-continue", "off"])
    out = dry_run(tmp_path, scene="aisle", out=tmp_path / "b", DOCKING_DEGENERATE_CONTINUE="on")
    assert out.returncode == 0, out.stdout
    assert contains(args_of(out), ["--docking-degenerate-continue", "on"])
    assert ddc_record(tmp_path, "b") == ["DOCKING_DEGENERATE_CONTINUE=on DOCKING_DEGENERATE_CONTINUE_FROM=launcher"]


@pytest.mark.parametrize("extra, value", [("--docking-degenerate-continue off", "off"),
                                          ("--docking-degenerate-continue=off", "off"),
                                          ("--docking-d off --record-boundary", "off"),
                                          ("--docking-degenerate-continue on --docking-degenerate-continue=off", "off")])
def test_degenerate_continue_named_in_extra_args_wins_and_is_recorded(tmp_path, extra, value):
    out = dry_run(tmp_path, EXTRA_ARGS=extra)
    assert out.returncode == 0, out.stdout + out.stderr
    args = args_of(out)
    assert sum(a.startswith("--docking-d") for a in args) == extra.count("--docking-d")  # none of the launcher's own
    assert ddc_record(tmp_path) == [f"DOCKING_DEGENERATE_CONTINUE={value} DOCKING_DEGENERATE_CONTINUE_FROM=EXTRA_ARGS"]


@pytest.mark.parametrize("env, said", [
    ({"DOCKING_DEGENERATE_CONTINUE": "yes"}, "must be on or off"),
    ({"EXTRA_ARGS": "--docking-degenerate-continue maybe"}, "must be on or off"),
    ({"MODE": "truth", "DOCKING_DEGENERATE_CONTINUE": "on"}, "needs MODE=slam"),
    ({"MODE": "truth", "EXTRA_ARGS": "--docking-degenerate-continue on"}, "needs MODE=slam"),
    ({"EXTRA_ARGS": "--near-tracking", "EXIT_EXEMPT": "off", "RETURN_CUSP_REPLANS": "0"},
     "--near-tracking needs DOCKING_DEGENERATE_CONTINUE=off"),
    # Codex promotion review P3: the words as the runner gets them, a tab or a newline around the flag included
    ({"EXTRA_ARGS": "--record-boundary\t--near-tracking\t--seed=0", "EXIT_EXEMPT": "off", "RETURN_CUSP_REPLANS": "0"},
     "--near-tracking needs DOCKING_DEGENERATE_CONTINUE=off"),
    ({"EXTRA_ARGS": "--near-tracking\n", "EXIT_EXEMPT": "off", "RETURN_CUSP_REPLANS": "0"},
     "--near-tracking needs DOCKING_DEGENERATE_CONTINUE=off")])
def test_degenerate_continue_is_refused_where_it_is_not_defined(tmp_path, env, said):
    out = dry_run(tmp_path, **env)
    assert out.returncode == 2 and said in out.stdout, out.stdout


def test_the_runner_s_check_takes_the_line_2_m_default(tmp_path):
    # the real parser (DRY_RUN_CHECK=1) with the real line_2_m scene: on passes the runner's --d7-docking check
    from tools import riibotics_isaac_scenes as tool

    scenes = tmp_path / "real"
    scenes.mkdir()
    (scenes / "line_2_m.json").write_text(
        json.dumps(tool.scene_record("line_2_m", tool.with_slot_markers("line", tool.scenes()["line_2"]))))
    out = dry_run(tmp_path, out=tmp_path / "o", SCENE_DIR=str(scenes), DRY_RUN_CHECK="1", PYTHONPATH=str(ROOT / "src"))
    assert out.returncode == 0 and out.stdout.splitlines()[-1] == "ARGS_OK", out.stdout[-800:] + out.stderr
    assert contains(args_of(out), ["--docking-degenerate-continue", "on"])


@pytest.mark.parametrize("extra", ["--scene-file /x/random_1_m.json", "--scene-file=/x/aisle_rack.json",
                                   "--scene-f=/x/random_1_m.json"])
def test_a_scene_file_past_scene_is_refused(tmp_path, extra):
    # Codex promotion review P2: SCENE=line_2 with another scene file in EXTRA_ARGS ran that scene with line_2_m's
    # degenerate-continue default; SCENE is the scene the defaults and the record follow
    out = dry_run(tmp_path, EXTRA_ARGS=extra)
    assert out.returncode == 2 and "SCENE= sets the scene" in out.stdout, out.stdout
    out = dry_run(tmp_path, out=tmp_path / "o2", EXTRA_ARGS="--scene-requirements-override {}")
    assert "SCENE= sets the scene" not in out.stdout  # the other --scene-... option is not a scene file


# ---- K1 (the lead's promotion decision 2026-11-14, docs/validation/2026-11-14-k1-comparison.md) ------------------


def test_k1_follows_the_near_end_stand_applied(tmp_path):
    args = args_of(dry_run(tmp_path))
    assert contains(args, ["--near-observe-arrival", "0.05,0.05"]) and args.count("--near-observe-arrival") == 1
    assert "NEAR_OBSERVE_ARRIVAL=0.05,0.05 NEAR_OBSERVE_ARRIVAL_FROM=launcher" in (
        tmp_path / "out/riib_defaults.txt").read_text()
    for env in ({"NEAR_END_STAND": "off"}, {"NEAR_END_STAND": "shadow"}, {"MODE": "truth"},
                {"EXTRA_ARGS": "--near-end-stand off"}, {"NEAR_OBSERVE_ARRIVAL": "off"}):
        out = dry_run(tmp_path, **env)
        assert out.returncode == 0, (env, out.stdout + out.stderr)
        assert "--near-observe-arrival" not in args_of(out), env
        assert "NEAR_OBSERVE_ARRIVAL=off NEAR_OBSERVE_ARRIVAL_FROM=launcher" in (
            tmp_path / "out/riib_defaults.txt").read_text(), env


@pytest.mark.parametrize("extra", ["--near-observe-arrival 0.04,0.06", "--near-observe-arrival=0.04,0.06",
                                   "--near-obs 0.04,0.06"])
def test_k1_named_in_extra_args_wins_and_is_recorded(tmp_path, extra):
    out = dry_run(tmp_path, EXTRA_ARGS=extra)
    assert out.returncode == 0, out.stdout + out.stderr
    assert len([a for a in args_of(out) if a.startswith("--near-o")]) == 1  # the launcher adds none of its own
    assert "NEAR_OBSERVE_ARRIVAL=0.04,0.06 NEAR_OBSERVE_ARRIVAL_FROM=EXTRA_ARGS" in (
        tmp_path / "out/riib_defaults.txt").read_text()


@pytest.mark.parametrize("env", [{"NEAR_OBSERVE_ARRIVAL": "0.05,0.05", "NEAR_END_STAND": "off"},
                                 {"EXTRA_ARGS": "--near-end-stand shadow --near-observe-arrival 0.05,0.05"},
                                 {"NEAR_OBSERVE_ARRIVAL": "narrow"}])
def test_k1_without_the_near_end_stand_on_or_malformed_is_refused(tmp_path, env):
    out = dry_run(tmp_path, **env)
    assert out.returncode == 2 and "NEAR_OBSERVE_ARRIVAL" in out.stdout, out.stdout + out.stderr
