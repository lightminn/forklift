"""The optional simulator launcher must expose usage without starting Isaac Sim."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim/isaac/run_transport.py"
CORE = ROOT / "src/forklift_core/mission/core.py"  # the mission logic moved out of the runner (M-a1)
PROVISIONAL_URDF = ROOT / "sim/models/dls08_provisional/forklift.urdf"


def runner_and_core_functions(tree) -> dict:
    """The runner's functions by name (unparsed), with the mission core's methods that moved out of
    run() (M-a1 slices 6c, 6d) in their place."""
    import ast

    functions = {n.name: ast.unparse(n) for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    core = ast.parse(CORE.read_text())
    cls = next(n for n in core.body if isinstance(n, ast.ClassDef) and n.name == "MissionCore")
    functions.update({n.name: ast.unparse(n) for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name not in functions})
    return functions


def test_isaac_transport_help_does_not_require_simulator_sdk() -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "--seed" in result.stdout
    assert "--base-scene" in result.stdout
    assert "--video" in result.stdout


def test_return_home_is_opt_in_and_documented_in_usage() -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    usage = " ".join(result.stdout.split())
    # A bare flag in usage: opting in takes no value, and omitting it keeps the
    # five-stage mission every recorded run was measured with.
    assert "[--return-home]" in usage
    assert "drive back to the rear-axle pose the mission started from" in usage


def test_isaac_transport_rejects_unsupported_camera_rate_before_startup() -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--fps", "59"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert "divide 120" in result.stderr
    assert "ModuleNotFoundError" not in result.stderr


def test_extra_views_require_video_and_perception_before_startup() -> None:
    for flags, expected in (
        (["--extra-views", "chase"], "--video"),
        (["--extra-views", "perception", "--video"], "--use-perception"),
        (["--extra-views", "unknown", "--video"], "Unknown"),
    ):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--base-scene",
                "unused.usda",
                "--pallet-urdf",
                "unused.urdf",
                "--forklift-urdf",
                str(PROVISIONAL_URDF),
                "--pallet-geometry",
                "unused.yaml",
                "--settings",
                "unused.yaml",
                "--output",
                "unused",
                "--seed",
                "2",
                *flags,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 2
        assert expected in result.stderr


def test_g2_rerun_options_validate_before_startup() -> None:
    prior = str(ROOT / "config/pallet_prior_epal6.yaml")
    for flags, expected in (
        (["--planning-target", "oracle_nominal"], "requires --use-perception"),
        (["--planning-target", "oracle_actual"], "invalid choice"),
        (
            ["--use-perception", "--pallet-prior", prior, "--repeat-captures", "10"],
            "--repeat-at-attempt",
        ),
        (
            [
                "--use-perception",
                "--pallet-prior",
                prior,
                "--repeat-captures",
                "10",
                "--repeat-at-attempt",
                "0",
            ],
            "counts from 1",
        ),
        (
            [
                "--use-perception",
                "--pallet-prior",
                prior,
                "--repeat-captures",
                "10",
                "--repeat-at-attempt",
                "1",
                "--planning-target",
                "oracle_nominal",
            ],
            "stops before planning",
        ),
        (["--repeat-at-attempt", "1"], "requires --repeat-captures"),
        (["--repeat-at-near-capture"], "requires --repeat-captures"),
        (
            [
                "--use-perception",
                "--pallet-prior",
                prior,
                "--repeat-captures",
                "10",
                "--repeat-at-attempt",
                "1",
                "--repeat-at-near-capture",
            ],
            "one of --repeat-at-attempt or --repeat-at-near-capture",
        ),
    ):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--base-scene",
                "unused.usda",
                "--pallet-urdf",
                "unused.urdf",
                "--forklift-urdf",
                str(PROVISIONAL_URDF),
                "--pallet-geometry",
                "unused.yaml",
                "--settings",
                "unused.yaml",
                "--output",
                "unused",
                "--seed",
                "2",
                *flags,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 2, (flags, result.stderr)
        assert expected in result.stderr, (flags, result.stderr)


def test_quarter_options_validate_before_simulator_startup(tmp_path) -> None:
    base = [
        sys.executable,
        str(SCRIPT),
        "--base-scene",
        "unused.usda",
        "--pallet-urdf",
        "unused.urdf",
        "--forklift-urdf",
        str(PROVISIONAL_URDF),
        "--pallet-geometry",
        "unused.yaml",
        "--settings",
        "unused.yaml",
        "--output",
        str(tmp_path / "run"),
        "--seed",
        "2",
    ]
    for flags in (
        ["--quarter-eye", "1,2"],
        ["--quarter-eye", "1,nan,3"],
        ["--quarter-target", "inf,2,3"],
        ["--quarter-focal", "0"],
        ["--quarter-focal", "nan"],
        ["--quarter-eye", "1,2,3", "--quarter-target", "1,2,3"],
    ):
        result = subprocess.run(
            base + flags, capture_output=True, text=True, check=False
        )
        assert result.returncode == 2, (flags, result.stderr)
        assert "quarter" in result.stderr.lower()
        assert "ModuleNotFoundError" not in result.stderr


def test_tracking_sample_remains_json_serializable_after_a_gear_change() -> None:
    import json
    from dataclasses import asdict

    from forklift_core.control import RearAxlePathTracker

    tracker = RearAxlePathTracker(
        [[0, 0, 0], [1, 0, 0], [0.5, 0, 0]], [1, 1, -1], [0, 0, 0]
    )
    for _ in range(20):
        command = tracker.update([1, 0, 0], 0, 1 / 120)
    assert command.segment_index == 1
    record = json.loads(json.dumps({"tracking": asdict(command)}))
    assert record["tracking"]["segment_index"] == 1


def test_isaac_record_encoder_preserves_numpy_numbers_and_arrays() -> None:
    import json
    import runpy

    import numpy as np

    module = runpy.run_path(str(SCRIPT))
    encode = module["record_json"]
    record = {
        "index": np.int64(60),
        "pose": np.array([1.0, 2.0]),
        "success": np.bool_(False),
        "time": np.float32(1.5),
    }
    assert json.loads(encode(record)) == {
        "index": 60,
        "pose": [1.0, 2.0],
        "success": False,
        "time": 1.5,
    }


def run_geometry_cli(tmp_path, pallet_urdf, geometry="t11_06", *extra):
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--base-scene",
            "unused.usda",
            "--pallet-urdf",
            str(pallet_urdf),
            "--forklift-urdf",
            str(PROVISIONAL_URDF),
            "--pallet-geometry",
            str(ROOT / f"config/pallet_geometry_{geometry}.yaml"),
            "--settings",
            str(ROOT / "config/isaac_transport.yaml"),
            "--output",
            str(tmp_path / "run"),
            "--seed",
            "0",
            *extra,
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def test_camera_inset_without_perception_is_rejected_before_startup(
    tmp_path, full_t11_pallet_urdf
):
    result = run_geometry_cli(
        tmp_path, full_t11_pallet_urdf(), "t11_06", "--video", "--camera-inset"
    )
    assert result.returncode == 2
    assert "--camera-inset requires --video and --use-perception" in result.stderr
    assert not (tmp_path / "run").exists()


def test_camera_inset_keeps_the_perception_prior_loaded(tmp_path, full_t11_pallet_urdf):
    # Regression: the inset check once split the perception block and left the
    # prior unloaded. Reaching SDK startup with the prior recorded proves it.
    import json
    import os
    from unittest.mock import patch

    (tmp_path / "isaacsim.py").write_text('raise RuntimeError("SDK_STARTUP_REACHED")')
    old = os.environ.get("PYTHONPATH", "")
    with patch.dict(os.environ, {"PYTHONPATH": str(tmp_path) + os.pathsep + old}):
        result = run_geometry_cli(
            tmp_path,
            full_t11_pallet_urdf(),
            "t11_06",
            "--video",
            "--use-perception",
            "--pallet-prior",
            str(ROOT / "config/pallet_prior_epal6.yaml"),
            "--camera-inset",
        )
    assert result.returncode == 1, result.stderr
    assert "SDK_STARTUP_REACHED" in result.stderr
    record = json.loads((tmp_path / "run/result.json").read_text())
    assert record["arguments"]["camera_inset"] is True
    assert record["arguments"]["pallet_prior_loaded"] is not None


def test_t11_configuration_reaches_sdk_startup(tmp_path, full_t11_pallet_urdf):
    # A sentinel SDK stops startup even on hosts with Isaac installed.
    import json
    import os

    (tmp_path / "isaacsim.py").write_text('raise RuntimeError("SDK_STARTUP_REACHED")')
    old = os.environ.get("PYTHONPATH", "")
    from unittest.mock import patch

    with patch.dict(os.environ, {"PYTHONPATH": str(tmp_path) + os.pathsep + old}):
        result = run_geometry_cli(tmp_path, full_t11_pallet_urdf())
    assert result.returncode == 1, result.stderr
    assert "SDK_STARTUP_REACHED" in result.stderr
    record = json.loads((tmp_path / "run/result.json").read_text())
    assert record["arguments"]["axle_to_fork_tip_m"] == 1.29
    assert record["arguments"]["rear_axle_offset_m"] == -0.34
    assert record["arguments"]["pallet_geometry_loaded"]["overall_depth_m"] == 0.66
    assert "quarter_eye" not in record["arguments"]
    assert "quarter_target" not in record["arguments"]
    assert "quarter_focal" not in record["arguments"]


def test_epal_yaml_rejects_t11_envelope_before_startup(tmp_path, synthetic_pallet_urdf):
    result = run_geometry_cli(tmp_path, synthetic_pallet_urdf(), "epal6")
    assert result.returncode == 2
    assert "envelope" in result.stderr
    assert "ModuleNotFoundError" not in result.stderr
    assert not (tmp_path / "run").exists()


def test_noncentred_pallet_is_rejected_before_startup(tmp_path, synthetic_pallet_urdf):
    result = run_geometry_cli(tmp_path, synthetic_pallet_urdf(x_m=0.03))
    assert result.returncode == 2
    assert "envelope" in result.stderr
    assert "ModuleNotFoundError" not in result.stderr
    assert not (tmp_path / "run").exists()


def test_t11_named_boxes_reject_90_degree_swap_before_startup(
    tmp_path, full_t11_pallet_urdf
):
    result = run_geometry_cli(tmp_path, full_t11_pallet_urdf(swap_all=True))
    assert result.returncode == 2, result.stderr
    assert "bottom_board_0" in result.stderr
    assert "ModuleNotFoundError" not in result.stderr
    assert not (tmp_path / "run").exists()


def test_slam_feedback_options_validate_before_startup(tmp_path) -> None:
    socket_path = str(tmp_path / "slam.sock")
    for flags, expected in (
        (["--slam-feedback", socket_path], "--record-slam and --use-perception"),
        (["--slam-noise-seed", "3"], "--slam-noise-seed requires --slam-feedback"),
    ):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--base-scene",
                "unused.usda",
                "--pallet-urdf",
                "unused.urdf",
                "--forklift-urdf",
                str(PROVISIONAL_URDF),
                "--pallet-geometry",
                "unused.yaml",
                "--settings",
                "unused.yaml",
                "--output",
                "unused",
                "--seed",
                "2",
                *flags,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 2, flags
        assert expected in result.stderr
        assert "ModuleNotFoundError" not in result.stderr


def test_every_physics_step_after_start_goes_through_step_world() -> None:
    """SLAM plan v3.1: odometry, scans and the lockstep see capture steps too."""
    import ast

    tree = ast.parse(SCRIPT.read_text())
    step_world = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "step_world"
    )
    body = ast.unparse(step_world)
    for needle in ("world.step(", "slam['odometry'].update(", "slam['link'].exchange(",
                   "slam['tracker'].receive(", "slam['stop'].update(", "tick % scan_every"):
        assert needle in body, needle
    run = next(
        node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "run"
    )
    loop = next(
        node
        for node in ast.walk(run)
        if isinstance(node, ast.For) and "max_sim_seconds" in ast.unparse(node.iter)
    )
    loop_source = ast.unparse(loop)
    assert "world.step(" not in loop_source
    assert "stepper['fn'](" in loop_source
    assert "stepper['fn'](True)" in ast.unparse(run)  # the capture path


def test_slam_release_replans_both_travel_legs_and_gates_on_a_stop() -> None:
    import ast

    tree = ast.parse(CORE.read_text())  # the SLAM release moved to the mission core (M-a1 slice 6c)
    release = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "slam_release"
    )
    body = ast.unparse(release)
    assert "jump_m <= 0.02 and abs(jump_rad) <= 0.02" in body
    assert "plan_transport_leg(" in body and "plan_return_leg(" in body
    source = SCRIPT.read_text()
    gate = CORE.read_text()  # the release gate moved to the mission core (M-a1 slice 5)
    assert "if slam[\"stop_now\"]:\n                slam_release(t)" in gate
    assert "rear = slam_rear(t)  # the released estimate" in gate
    assert "if ms.phase in trackers and not releasing and not warming:" in source


def test_slam_runs_see_the_pallet_again_before_the_final_straight() -> None:
    """Plan v3.6: near capture from final_straight_prefix, no silent fallback."""
    core = CORE.read_text()  # the observation's handling moved to the mission core (M-a1 slices 4b-iii/iv)
    assert "final_straight_prefix(\n                paths[\"approach\"], geometry.alignment_straight_m" in core
    assert 'f"near_capture_failed:{attempt[\'retry_reason\']}"' in core
    assert 'if slam is not None and "near_capture" not in state:' in core


def test_the_delivery_match_inputs_are_recorded_without_stopping_the_match() -> None:
    """2026-10-13: every delivery match round's inputs go to delivery_match_N.npz for
    offline replay; a failed write is noted and the round is still recorded (Codex P2)."""
    source = CORE.read_text()  # the delivery docking is a MissionCore method since M-a1 slice 6b
    start = source.index("    def dock_at_delivery_straight(self, t: float) -> None:")
    end = source.find("\n    def ", start + 10)
    body = source[start : end if end >= 0 else len(source)]
    save = body.index("f\"delivery_match_{len(docking.get('rounds', [])) + 1}.npz\"")
    guard = body.rindex("try:", 0, save)
    handler = body.index("except OSError as exc:", save)
    assert body.index("result = dock(") < guard
    assert body.index('docking.setdefault("match_input_errors", []).append(', handler) < body.index(
        'docking.setdefault("rounds", []).append('
    )
    # Every round keeps its own starts and their details (iterations, cycle, gates).
    assert "start_details=[dict(d) for d in result.start_details]," in body
    rounds = body[body.index('docking.setdefault("rounds", []).append('):]
    assert '"starts": docking["starts"],' in rounds and '"start_details": docking["start_details"],' in rounds


def test_slam_runs_dock_on_a_scan_before_the_delivery_straight() -> None:
    """Plan v3.8: stop at the straight start, match, move the whole drop."""
    source = SCRIPT.read_text()
    # The transport arrival (M-a1 slice 2) and the delivery docking (slice 6b) are the mission core's.
    core = CORE.read_text()
    assert 'self.dock_at_delivery_straight(t)' in core
    assert 'ignore_self=True' in source  # the runner's scan for the match (a port)
    # The withdraw follows every goal change (plan D7c factored it into move_withdraw).
    assert "set_withdraw(np.array([compose(tuple(frame_change), tuple(pose)) for pose in paths[\"withdraw\"].poses]))" in core
    assert core.count("move_withdraw(step_shift)") == 2
    # Three SLAM recoveries, plus the priority-5 obstacle replan of the
    # transport leg and the return leg's fallback to it.
    # (and the zero-clearance retry of that obstacle replan), plus the live
    # replans of the transport and return legs when they start.
    # Plan D7c: the two obstacle replans of the transport now share transport_replan
    # (one lookup), and the docking retry's release and re-approach plan add three.
    # Obstacle recovery v4: the escape's leg (transport and return) looks them up as the replans do.
    # (the obstacle replan and the live leg moved to the mission core, M-a1 slices 3a/3c)
    assert source.count('slam.get("transport_scenario", scenario)') + CORE.read_text().count(
        'slam.get("transport_scenario", scenario)') == 12


def test_d8b_recording_options_validate_before_startup(tmp_path) -> None:
    socket_path = str(tmp_path / "slam.sock")
    for flags, expected in (
        (["--record-pocket-frames"], "--record-pocket-frames needs --slam-feedback"),
        (["--record-dwell-gaps", "1.0,0.5"], "--record-dwell-gaps needs --record-pocket-frames"),
        (["--record-dwell-gaps", "0.5,1.0"], "--record-dwell-gaps"),
        (["--approach-straight-speed-mps", "0"], "(0, 1] m/s"),
        (["--approach-straight-speed-mps", "1.5"], "(0, 1] m/s"),
        (["--record-pocket-frames", "--slam-feedback", socket_path], "--record-slam and --use-perception"),
        (["--measure-safety-stops-m", "1.0,0.5"], "--measure-safety-stops-m needs --record-pocket-frames"),
        (["--measure-safety-stops-m", "0.5,1.0"], "--measure-safety-stops-m"),
    ):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--base-scene", "unused.usda", "--pallet-urdf", "unused.urdf",
             "--forklift-urdf", str(PROVISIONAL_URDF), "--pallet-geometry", "unused.yaml", "--settings",
             "unused.yaml", "--output", "unused", "--seed", "2", *flags],
            capture_output=True, text=True, check=False,
        )
        assert result.returncode == 2, flags
        assert expected in result.stderr, (flags, result.stderr)
        assert "ModuleNotFoundError" not in result.stderr


def test_d8b_every_read_is_recorded_before_the_d5_check_can_drop_it() -> None:
    """Plan D8b: standing, timeless and out-of-order frames are kept and labelled."""
    import ast

    tree = ast.parse(SCRIPT.read_text())
    functions = runner_and_core_functions(tree)
    read = functions["read_pocket_frame"]
    assert read.index("record_pocket_read(raw)") < read.index("if raw is None:") < read.index("frames_without_time")
    assert "if check is None:" in read
    # Without the recorder the D5 path keeps its order: time checks before the reshape and
    # the joint read (Codex D8b impl P2).
    assert read.index("frames_bad_time") < read.index("normalize_depth(") < read.index("get_joint_positions")
    assert "recorder.record_read(" in functions["record_pocket_read"]
    # The truth of every tick, capture steps included, comes from step_world, after its
    # encoders and odometry.
    step = functions["step_world"]
    assert step.index("slam['stop_now'] = ") < step.index("record_truth_tick(") < step.index("tick % scan_every")
    assert "recorder.tick_error = " in functions["record_truth_tick"]
    source = SCRIPT.read_text()
    # The dwells restart the tracker's slew and hold the switch to the insertion.
    command = CORE.read_text()  # the dwells moved to the mission core (M-a1 slice 4a-iv)
    assert "trackers[\"approach\"].restart_speed_slew(0.0)" in command
    assert "tracking = replace(tracking, status=\"tracking\", speed_mps=0.0)" in command
    # The recorder is written first in the finally block, guarded.
    finally_at = source.index("    finally:\n        if recorder is not None:")
    assert finally_at < source.index("state[\"pocket_recording\"] = recorder.close(") < source.index(
        "state[\"pocket_recording_close_error\"] = repr(exc)") < source.index("write_slam_record(args, state, replace(scenario")
    # A failed close fails the run without replacing an earlier failure reason.
    assert "state.setdefault(\"failure_reason\", f\"pocket_recording_close_failed: {exc!r}\")" in source
    # The control pose read with a frame carries the time it was taken at (a capture in the
    # same loop iteration moves it past the loop's t).
    assert "rear = slam_rear(now_s)\n            rear_stamp = now_s" in CORE.read_text()  # (M-a1 slice 4b-i)
    assert "    rear_stamp = captured_.rear_stamp" in source  # handed back to the runner's control stamp
    assert "pocket[\"control_stamped\"] = (rear_stamp," in source


def test_d8b_custom_cruise_drives_the_straight_but_not_the_acceptance_dry_run() -> None:
    """The near capture's go/no-go stays the baseline's (smoke run: a 0.30 m/s dry run
    arrives 7.76 mm short, past the 6 mm acceptance); the custom-cruise one is recorded."""
    source = CORE.read_text()  # the near capture's straight moved to the mission core (M-a1 slice 4b-v)
    acceptance = source.index("acceptance_config = replace(acceptance_config,\n")
    assert "cruise_speed_mps=speeds[\"approach\"])" in source[acceptance:acceptance + 200]
    dry = source.index("acceptance_config,\n                        rear,\n")
    driven = source.index("state[\"near_capture\"][\"dry_run_driven\"] = asdict(bicycle_rollout(")
    tracker = source.index("approach_config,\n                    )\n")
    assert acceptance < dry < driven < tracker


def test_d8b_dwells_hold_the_steering_as_well_as_the_wheels() -> None:
    """Matrix 1883: steering turned 0.18-0.25 rad in every start dwell and the body crept."""
    source = SCRIPT.read_text()
    command = CORE.read_text()  # the dwells moved to the mission core (M-a1 slice 4a-iv)
    block = command[command.index("hold_, released_ = dwells.update("):command.index("elif released_:")]
    assert "dwell_hold = True" in block
    # (A cusp position stall's hold, plan cusp stall v7, joins the same path.)
    # (the command assembly moved to the mission core, M-a1 slice 5)
    assert "if estop_holding or obstacle_hold or dwell_hold or cusp_hold:" in command
    assert "steering_command.copy() if (estop_holding or obstacle_hold or dwell_hold or cusp_hold)" in command
    # The wheels brake on the held steering's curvature, in the command actually applied.
    assert "wheel_curvature = kappa if (dwell_hold or cusp_hold) else curvature" in command
    assert "drive = ackermann_command(wheel_speed, wheel_curvature, drive_geometry)" in command
    assert "ackermann_command(wheel_speed, curvature," not in command and "ackermann_command(" not in source
    assert source.index("dwell_hold = dwell_standing = False") < source.index("command_ = core.command(")


def test_d8b_dwells_lock_the_wheels_and_give_their_gains_back() -> None:
    """Confirmation 1968: the velocity drives let a standing truck roll 0.1-0.15 mm/s."""
    source = SCRIPT.read_text()
    lock = source[source.index('if dwell_standing and wheel_lock["q"] is None:'):source.index("joint_positions=steering_command, joint_indices=steers")]
    assert "locked_kps[wheels] = DWELL_WHEEL_KP" in lock and "controller_.set_gains(kps=locked_kps, kds=kds_)" in lock
    assert 'elif not dwell_standing and wheel_lock["q"] is not None:' in lock
    # Locked once standing, never while braking (diagnostic 1984).
    assert 'dwell_standing = dwells.active is not None and dwells.active["still_since_s"] is not None' in CORE.read_text()
    assert 'set_gains(kps=wheel_lock["gains"][0], kds=wheel_lock["gains"][1])' in lock
    assert 'joint_positions=wheel_lock["q"],' in lock


def test_s4_safety_stops_take_the_dwell_path_without_the_wheel_lock_and_record_the_command() -> None:
    """Plan D8 S4a-1 ⓪: a scheduled safety stop zeroes the command and holds the steering
    (dwell_hold) but never sets dwell_standing (the wheel lock); the final wheel command
    steps to zero after the obstacle slew, as the e-stop's does (Codex S4a-1 ⓪ review P1);
    the applied command of every tick -- the Ackermann-limited speed (P3) -- reaches the
    truth record."""
    import ast

    source = SCRIPT.read_text()
    command = CORE.read_text()  # the safety stops moved to the mission core (M-a1 slice 4a-iv)
    block = command[command.index("safety_stops.update("):command.index("phase == \"observe\" and tracking.status")]
    assert "dwell_hold = True" in block and "dwell_standing" not in block
    assert "safety_hold = True" in block
    assert "restart_speed_slew(0.0)" in block
    # Reset every loop tick, before the stop block can set it.
    assert source.index("safety_hold = False") < source.index("command_ = core.command(")
    # The zero comes after the slew and the permission cap, last before the drive command.
    assembly = CORE.read_text()  # the command assembly moved to the mission core (M-a1 slice 5)
    slew = assembly.index("wheel_speed = previous + float(np.clip(target_speed - previous, -rate, rate))")
    zero = assembly.index("if estop_holding or safety_hold:\n                wheel_speed = 0.0")
    assert slew < assembly.index("cap = obstacle.get(\"permission_cap\")") < zero
    assert zero < assembly.index("drive = ackermann_command(wheel_speed, wheel_curvature, drive_geometry)")
    tree = ast.parse(source)
    functions = runner_and_core_functions(tree)
    assert "applied_speed_mps=state.get('applied_command'" in functions["record_truth_tick"]
    recorded = assembly.index("state[\"applied_command\"] = (float(drive.speed_mps)")
    assert assembly.index("drive = ackermann_command(wheel_speed, wheel_curvature, drive_geometry)") < recorded
    # The runner applies the assembled command afterwards (M-a1 slice 5).
    assert source.index("command_ = core.assemble_command(") < source.index(
        "joint_velocities=np.asarray(drive.wheel_rates_rad_s)")


def test_s4_near_tracking_options_validate_before_startup() -> None:
    for flags, expected in (
        (["--near-tracking"], "--near-tracking needs --slam-feedback"),
        (["--near-tracking-stop-at", "insert"], "--near-tracking-stop-at needs --near-tracking"),
        # Plan D8 S4c-1: the injections need the tracker and one size.
        (["--near-inject", "correction", "--near-inject-lateral-m", "0.02"], "--near-inject needs --near-tracking"),
        (["--near-inject-yaw-rad", "0.015"], "need --near-inject"),
        (["--near-inject-lateral-m", "nan"], "must be finite"),
        (["--near-drop-frames-from-m", "0.3"], "--near-drop-frames-from-m needs --near-tracking"),
        # Plan D8 S4c-3: the test-only near-capture heading box, narrower only.
        (["--near-capture-box-yaw-rad", "0.01"], "--near-capture-box-yaw-rad needs --near-tracking"),
    ):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--base-scene", "unused.usda", "--pallet-urdf", "unused.urdf",
             "--forklift-urdf", str(PROVISIONAL_URDF), "--pallet-geometry", "unused.yaml", "--settings",
             "unused.yaml", "--output", "unused", "--seed", "2", *flags],
            capture_output=True, text=True, check=False,
        )
        assert result.returncode == 2, flags
        assert expected in result.stderr, (flags, result.stderr)


def _core_hook(call, parents) -> bool:
    """The runner's ``near_tick=lambda *a: near_tick(*a)`` handed to MissionCore -- the only call of a guarded
    function allowed outside a ``near is not None`` test, since the core guards it (Codex 4a-iii/iv P3)."""
    import ast

    lam = parents.get(call)
    kw = parents.get(lam)
    owner = parents.get(parents.get(parents.get(kw)))  # keyword -> SimpleNamespace(...) -> hooks= -> MissionCore(...)
    return (isinstance(lam, ast.Lambda) and lam.body is call and isinstance(kw, ast.keyword)
            and kw.arg == call.func.id and isinstance(owner, ast.Call) and ast.unparse(owner.func) == "MissionCore")


def test_s4_near_tracking_runs_only_behind_the_flag_and_before_the_arrival() -> None:
    """Plan D8 S4a: with --near-tracking off the run is unchanged -- every call into the
    near-field path sits under a `near is not None` test -- and the stop is decided before
    the tick's arrival transition, takes the safety-stop path and defers the arrival."""
    import ast

    source = SCRIPT.read_text()
    tree = ast.parse(source)
    parents = {child: node for node in ast.walk(tree) for child in ast.iter_child_nodes(node)}
    guarded_calls = ("build_near_tracker", "feed_near_frame", "near_tick")
    core_tree = ast.parse(CORE.read_text())  # the near tick moved to the mission core (M-a1 slice 4a-iv)
    parents.update({child: node for node in ast.walk(core_tree) for child in ast.iter_child_nodes(node)})
    calls = [n for t_ in (tree, core_tree) for n in ast.walk(t_) if isinstance(n, ast.Call)
             and getattr(n.func, "id", None) in guarded_calls
             and not _core_hook(n, parents)]
    assert {c.func.id for c in calls} == set(guarded_calls)
    for call in calls:
        node, guarded = call, False
        while node in parents:
            node = parents[node]
            if isinstance(node, ast.If) and "near is not None" in ast.unparse(node.test):
                guarded = True
                break
        assert guarded, ast.unparse(call)
    assert source.count("near = None\n") == 1
    # The contract is checked before the first tick; S4a-2 opens the insertion.
    check_ = source.index("            check_near_contract()\n")
    loop_ = source.index("        for step in range(int(120 * args.max_sim_seconds)):")
    assert check_ < loop_
    # Only the M-a0 boundary-record wrapping (recording only) sits between the check and the loop.
    between_ = source[check_ + len("            check_near_contract()\n"): loop_]
    assert between_.startswith("        if boundary is not None:\n") and "require(" not in between_
    assert "until S4a-2" not in source
    # Decided after the scheduled safety stops and before the arrival transitions.
    moved = CORE.read_text()
    tick = moved.index("hold_, released_, arrive_ = near_tick(t, rear_stamp, rear, requested_speed, ms.phase, curvature,")
    assert moved.index("safety_stops.update(") < tick
    assert source.index("command_ = core.command(") < source.index("arrival_ = core.arrive(t, tracking)")
    block = moved[tick:moved.index("phase == \"observe\" and tracking.status", tick)]
    for line in ("requested_speed = 0.0", "dwell_hold = True", "safety_hold = True",
                 "tracking = replace(tracking, status=\"tracking\", speed_mps=0.0)",
                 "trackers[ms.phase].restart_speed_slew(0.0)"):
        assert line in block, line
    # S4a-2: the carriage-corner insertion end becomes the insertion's arrival, standing.
    arrive = block.index("if arrive_:")
    assert arrive < block.index("tracking = replace(tracking, status=\"arrived\", speed_mps=0.0)") < block.index("elif hold_:")
    core = CORE.read_text()  # the arrival transitions moved to the mission core (M-a1 slice 2)
    assert "state[\"near_tracking\"][\"insert_arrival\"]" in core
    # Built in the hold the approach transition begins; fed the raw frame before D5 reads it.
    build = moved.index("build_near_tracker(t)  # in the hold the transition began")  # (M-a1 slice 4b-v)
    assert moved.rindex("transition(\"approach\", t)", 0, build) > build - 300
    feed = source.index("if feed_near_frame(raw) == \"bad_depth\":\n                return")
    assert source.index("record_pocket_read(raw)\n        if raw is None:") < feed < source.index(
        "        if check is None:\n            pocket[\"frames_read\"] += 1")
    # S4a-1 ends at the approach's arrival, and the loop end lets that phase return.
    assert "transition(\"near_tracking_stopped\", t)\n                return SimpleNamespace(stop=True, rear=rear)" in core
    assert "if arrival_.stop:\n                            break" in source
    assert "\"repeat_target_not_reached\", \"near_tracking_stopped\"):\n            return" in source
    # Terminal stops end the run only once the stop detector sees the truck stand.
    functions = runner_and_core_functions(tree)
    feed_fn = functions["feed_near_frame"]
    assert feed_fn.index("skipped='bad_depth'") < feed_fn.index("get_current_frame()") < feed_fn.index(
        "row['skipped'] = 'uncovered'")
    tick_fn = functions["near_tick"]
    assert "near_tracking.decide_stop(" in tick_fn and "if action['end']:" in tick_fn
    assert "standing = bool(slam['stop_now'])" in tick_fn
    # S4c-1: a terminal end waits TERMINAL_HOLD_S standing before ending the run.
    hold = tick_fn.index("near.setdefault('terminal_standing_since', now_s)")
    assert hold < tick_fn.index("near_tracking.TERMINAL_HOLD_S") < tick_fn.index("'event': 'terminal_standing'")
    assert "near.pop('terminal_standing_since', None)" in tick_fn
    latched = tick_fn.index("if near['stop_state'].terminal:")
    assert latched < tick_fn.index("near_tracking.correction_update(") < tick_fn.index("near_redraw(latest_, rear_now)")
    # A gap shortfall ends the run only on a fresh observation; otherwise it waits.
    core_src = CORE.read_text()  # near_tick moved to the mission core (M-a1 slice 6d)
    tick_src = core_src[core_src.index("    def near_tick("):core_src.index("\n    def ", core_src.index("    def near_tick(") + 10)]
    fresh_branch = tick_src[tick_src.index("if margins_[worst_] < 0:"):tick_src.index("ghat = ")]
    assert fresh_branch.index("if fresh:") < fresh_branch.index('terminal = terminal or "stuck"') < fresh_branch.index(
        'stale_now.add("gap_wait")')
    assert "stale_now.add('gap_wait')" in tick_fn and "stale_now.add('carriage_uncertain')" in tick_fn
    assert "if phase_ == 'insert' and min(" in tick_fn  # the insertion end only in the insertion
    # Codex S4a-2 code review: latched only on a fresh, certain observation; old-evidence
    # waits hold until a new accepted observation; the held steering command bounds kappa.
    assert "if fresh and certain:" in tick_fn and "stale_now.add('insert_end_unconfirmed')" in tick_fn
    assert "st_ >= latest_.stamp_s" in tick_fn
    assert "steering_command" in tick_fn
    unbounded = tick_src.index('terminal = terminal or "unbounded"  # no bound: not a measured shortfall')
    assert unbounded < tick_src.index("elif face_distance - along_e <= k_tip * sweep:")


def test_s4c1_injections_bias_only_the_captures_before_the_tracker_and_drop_before_it() -> None:
    """Plan D8 S4c-1: a capture is biased once and only until the tracker is built (no retry
    capture); correction biases the target after it is made, mismatch the observation before;
    the loss latches on a near-field tick and drops a frame before anything reads it, leaving
    tau's reference alone; the injection arguments are run-specific, the known pallet refused."""
    import ast

    source = SCRIPT.read_text()
    tree = ast.parse(source)
    functions = runner_and_core_functions(tree)
    assert "near['injection_closed'] = True" in functions["build_near_tracker"]
    core = CORE.read_text()  # the injection moved to the mission core with the observation (M-a1 slice 4b-iv)
    inject = core.index("inject_ = (near is not None and options.near_inject is not None")
    window = core[inject : inject + 4000]
    assert 'not near.get("injection_closed"))' in window
    assert window.index("near_tracking.biased_observation(") < window.index("base_xy = adapter.estimate_pallet_center_m(")
    assert window.index("target_pickup = adapter.estimate_world_pallet_site(") < window.index("near_tracking.biased_site(")
    feed_fn = functions["feed_near_frame"]
    assert feed_fn.index("row.update(skipped='dropped'") < feed_fn.index("adapter.normalize_depth(")
    assert feed_fn.index("return 'dropped'") < feed_fn.index("near['last_stamp'] = stamp")
    tick_fn = functions["near_tick"]
    assert "face_distance <= options.near_drop_frames_from_m" in tick_fn and "'face_distance_m': face_distance" in tick_fn
    assert "known pallet would carry the injected bias" in functions["check_near_contract"]
    near_tracking = (ROOT / "sim/isaac/near_tracking.py").read_text()
    for name in ("near_inject", "near_inject_lateral_m", "near_inject_yaw_rad", "near_drop_frames_from_m"):
        assert f'    "{name}",\n' in near_tracking[near_tracking.index("RUN_SPECIFIC_ARGUMENTS = ("):], name


def test_s4c2_a_terminal_mismatch_or_refusal_retries_once_from_a_fresh_far_capture() -> None:
    """Plan D8 S4c-2: after the terminal hold a mismatch or refused correction before the
    protection becomes one retry -- execution latches reset and records kept, the SLAM hold
    released and the grid re-projected, the old estimate carried, the near capture forgotten,
    and a stand in observe whose follower, hold and recoveries are bypassed until the
    re-observation, which must be valid."""
    import ast

    source = SCRIPT.read_text()
    tree = ast.parse(source)
    functions = runner_and_core_functions(tree)
    tick_fn = functions["near_tick"]
    retry = tick_fn.index("near_tracking.retry_allowed(")
    assert tick_fn.index("near.setdefault('terminal_standing_since', now_s)") < retry
    assert retry < tick_fn.index("start_pickup_retry(now_s, current.reason, face_distance, episode)")
    assert retry < tick_fn.index("'event': 'terminal_standing'")
    retry_fn = functions["start_pickup_retry"]
    order = ["near['retries'] = near.get('retries', 0) + 1", "near['tracker'] = None",
             "state.pop('near_capture')", "release_and_stand(now_s, 'release_pickup_retry')",
             "transition('observe', now_s)"]
    positions = [retry_fn.index(o) for o in order]
    assert positions == sorted(positions), order
    # The release and the stand, shared with the S4c-3 realignment.
    shared = functions["release_and_stand"]
    order = ["slam['tracker'].release(", "obstacle['layer'].refresh(", "carried(obstacle['pickup_estimate'])",
             "near['retry_stand'] = True"]
    positions = [shared.index(o) for o in order]
    assert positions == sorted(positions), order
    assert "near['mismatch'] = near['unbounded'] = False" in retry_fn
    build_fn = functions["build_near_tracker"]
    assert "mismatch=False, unbounded=False, path_revision=0" in build_fn
    assert "near.setdefault('stop_events', [])" in build_fn and "stop_events=[]" not in build_fn
    core = CORE.read_text()  # the follower's tick moved to the mission core (M-a1 slice 4a-i)
    stand = core.index("retry_stand_ = near is not None and near.get('retry_stand') and ms.phase == 'observe'".replace("'", '"'))
    window = core[stand : stand + 2700]  # obstacle recovery v4's escape end and the cusp stall sit between
    assert 'tracking = replace(tracking, status="arrived", speed_mps=0.0)' in window
    assert "and not retry_stand_" in window
    moved = CORE.read_text()  # the capture and detection moved to the mission core (M-a1 slice 4b-i)
    seen = moved.index("Plan D8 S4c-2 / S4c-3: the re-observation must see the pallet")
    assert '"near_field_retry_no_detection" if stand_key_ == "retry"' in moved[seen : seen + 900]
    assert "require(False, reason_)" in moved[seen : seen + 900]
    assert moved.index("detection = detect_pockets(detector_input, prior, params)") < seen
    # What (x) judges by: each frame's hold, the re-projection, and the re-observation as
    # placed with the target the planner used.
    assert "'episode': near['episode']" in functions["feed_near_frame"]
    assert "'reprojected': reprojected_" in shared and "reprojected_ = {" in shared
    assert shared.index("applied_before_ = [") < shared.index("slam['tracker'].release(")
    moved = CORE.read_text()  # the observation's handling moved to the mission core (M-a1 slice 4b-iv)
    placed = moved.index("Plan D8 S4c-2 / S4c-3: the re-observation as placed and its target.")
    assert "capture_base=" in moved[placed : placed + 500] and "target=[target_pickup.x_m" in moved[placed : placed + 500]
    assert moved.index('state["near_tracking"][stand_key_]["planned_target"] = [') > moved.index(
        "planning_pickup, state[\"planning_target_source\"] = hooks.planning_target(target_pickup)"
    )


def test_s4b_correction_points_are_decided_before_the_stop_and_install_at_once() -> None:
    """Plan D8 S4b: the correction points join the stop decision as a waiting reason, are
    decided on the tracker's latest observation standing, refuse D5 alongside, check the
    docking straights on the grid with only B0 and the new band cleared, and install the
    paths, followers, pickup target, phase start and permission path together."""
    import ast

    source = SCRIPT.read_text()
    tree = ast.parse(source)
    functions = runner_and_core_functions(tree)
    tick_fn = functions["near_tick"]
    assert tick_fn.index("near_tracking.correction_update(") < tick_fn.index("near_tracking.decide_stop(")
    assert tick_fn.index("near_redraw(latest_, rear_now)") < tick_fn.index("near_install(")
    assert "terminal = terminal or 'correction_missed'" in tick_fn and "terminal = 'replan_rejected'" in tick_fn
    assert "revision_used = near['path_revision']" in tick_fn and "'path_revision': revision_used" in tick_fn
    moved = CORE.read_text()  # the near tick moved to the mission core (M-a1 slice 4a-iv)
    assert 'tracking.status == "arrived")' in moved[moved.index("arrive_ = near_tick("):][:200]
    grid_fn = functions["correction_grid"]
    assert "docking_band(occupancy_, first_target) & docking_band(occupancy_, target)" in grid_fn
    redraw_fn = functions["near_redraw"]
    assert "near['first_target']" in redraw_fn and "dry(acceptance_), dry(driven_)" in redraw_fn
    assert "max_yaw_rad=near_tracking.CORRECTION_MAX_YAW_RAD" in redraw_fn  # S4b revision: 0.12 rad
    install_fn = functions["near_install"]
    for line in ("ms.phase_started = now_s",  # M-a1 step 0: the phase clock lives on the MissionState "pocket['planning_pickup'] = target_",
                 "obstacle['pickup_estimate'] = target_", "obstacle['layer'].refresh(", "trackers[name_] = RearAxlePathTracker(",
                 "trackers[name_].path_arrays()", "near['path_revision'] = revision_"):
        assert line in install_fn, line
    contract_fn = functions["check_near_contract"]
    assert "if args.pocket_check:" in contract_fn and "waits for D5 v11" in contract_fn
    assert "first_target=pocket['planning_pickup']" in functions["build_near_tracker"]
    # The mission's own docking band is the same computation as before (extracted only).
    assert "band = docking_band(occupancy, target)" in functions["grid_kwargs"]


def test_s4c3_stage_one_redraws_a_refused_near_capture_before_ending() -> None:
    """Plan D8 S4c-3 stage 1: a near capture the box or its dry run refuses is tried with the
    S4b re-draw (heading box 0.12 rad, both dry runs, the grid with this target's band, the
    correction points) before the run ends, near-tracking runs only; an acceptance installs
    the approach, insertion and extraction and records them by content and by the
    followers' copies, and the tracker's start records the paths it starts on."""
    import ast

    source = SCRIPT.read_text()
    tree = ast.parse(source)
    functions = runner_and_core_functions(tree)
    redraw_fn = functions["near_capture_redraw"]
    for needed in (
        "redraw_docking_traced(",  # the dry runs' trajectories come back in the Planner response (M-a1 6d)
        "dry_configs=(acceptance_, driven_)",
        "max_yaw_rad=near_tracking.CORRECTION_MAX_YAW_RAD",
        "correction_grid(target_, target_, rear_now)",
        "record_['refused'] = 'correction_point'",
        "'raw_target':",
        "'line':",
        "held_pockets(observation_,",
        "near_tracking.rollout_face_distance(held_, widths_, tip_x_, options.rear_axle_offset_m)",
        "near_tracking.correction_points_check(list(trajectories_),",
        "'fork_tip_x_m': tip_x_",
    ):
        assert needed in redraw_fn, needed
    # (x) v3: the capture's own control sample, taken with its SLAM error at the same instant.
    moved = CORE.read_text()  # the capture moved to the mission core (M-a1 slice 4b-i)
    sample = moved.index('attempt["capture_control"] = {')
    assert moved.index('attempt["slam_error"] = slam_error()') < sample
    assert moved.index("truth_rear_capture = [float(v) for v in rear]") < moved.index(
        "rear = slam_rear(now_s)", moved.index("truth_rear_capture")
    )
    # The correction trigger and the check measure the same distance.
    assert (
        "face_distance = near_tracking.face_distance_m(face, tip_x)"
        in functions["near_tick"]
    )
    moved = CORE.read_text()  # the near capture's straight moved to the mission core (M-a1 slice 4b-v)
    branch = moved.index(
        "Plan D8 S4c-3 stage 1: the S4b re-draw acceptance at the near"
    )
    window = moved[branch - 2500 : branch + 3500]
    assert window.index(
        'refusal = f"near_capture_misaligned:{offsets}"'
    ) < window.index('if straight is None and near is not None and options.near_capture_redraw == "on":')
    assert 'refusal = f"near_capture_misaligned:dry_run:{asdict(dry)}"' in window
    assert window.index("near_capture_redraw(") < window.index(
        "require(straight is not None, refusal)"
    )
    assert (
        'state["near_capture"].setdefault("recoveries", []).append(redraw_entry)'
        in window
    )
    installed = moved.index('redraw_entry["installed"] = {')
    assert (
        moved.index('trackers["approach"] = RearAxlePathTracker(', branch) < installed
    )
    assert (  # the tracker's build moved to the mission core (M-a1 slice 6e)
        '"paths0": {name_: near_tracking.path_digest(*trackers[name_].path_arrays())'
        in CORE.read_text()
    )


def test_s4c3_stage_two_realigns_once_on_the_shared_release_and_records_every_install() -> None:
    """Plan D8 S4c-3 stage 2: a near capture refused by its acceptance and stage 1 realigns once
    per pickup (no retry event or count, the refused capture kept apart), on the release-and-
    stand shared with the S4c-2 retry; every observe installation in the realignment is
    recorded and capped, a held recovery ends the run, and the re-capture keeps 1.5 m."""
    import ast

    source = SCRIPT.read_text()
    tree = ast.parse(source)
    functions = runner_and_core_functions(tree)
    shared = functions["release_and_stand"]
    for needed in ("slam['tracker'].release(", "obstacle['layer'].refresh(", "carried(pocket['planning_pickup'])",
                   "near['retry_stand'] = True", "'applied_before': applied_before_"):
        assert needed in shared, needed
    assert "release_and_stand(now_s, 'release_pickup_retry')" in functions["start_pickup_retry"]
    realign = functions["start_near_realign"]
    assert "release_and_stand(now_s, 'release_near_realign')" in realign
    for absent in ("retry_start", "near['retries']", "near['tracker'] = None"):
        assert absent not in realign, absent
    assert realign.index("state['near_capture']['refused_attempt'] = attempt_") < realign.index(
        "state.setdefault('near_capture_refused', [])")
    assert "realign_install('stand')" in realign
    install = functions["realign_install"]
    assert "near_tracking.REALIGN_MAX_LENGTH_M" in install and "'slam_mode': slam['tracker'].mode" in install
    core = CORE.read_text()  # the obstacle replan's installs moved to the mission core (M-a1 slice 3c)
    for site in ("backoff", "obstacle_replan", "observe_replan", "observe_candidate", "near_leg"):
        assert f'realign_install("{site}")' in core, site
    assert source.count('"near_capture_realign_held_recovery"') + core.count('"near_capture_realign_held_recovery"') == 2
    branch = core.index("Plan D8 S4c-3 stage 2: once per pickup, instead of ending.")  # (M-a1 slice 4b-v)
    window = core[branch - 1500 : branch + 600]
    assert 'state["near_tracking"].get("realigns", 0) == 0' in window
    assert window.index("near_tracking.REALIGN_MIN_LEFT_M") < window.index("start_near_realign(t, attempt_number)")
    assert window.index("start_near_realign(t, attempt_number)") < window.index("require(straight is not None, refusal)")
    assert 'if straight is None and near is not None and options.near_capture_redraw == "on":' in core
