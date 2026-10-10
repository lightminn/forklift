"""The optional simulator launcher must expose usage without starting Isaac Sim."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim/isaac/run_transport.py"
PROVISIONAL_URDF = ROOT / "sim/models/dls08_provisional/forklift.urdf"


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

    tree = ast.parse(SCRIPT.read_text())
    release = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "slam_release"
    )
    body = ast.unparse(release)
    assert "jump_m <= 0.02 and abs(jump_rad) <= 0.02" in body
    assert "plan_transport_leg(" in body and "plan_return_leg(" in body
    source = SCRIPT.read_text()
    assert "if slam[\"stop_now\"]:\n                    slam_release(t)" in source
    assert "rear = slam_rear(t)  # the released estimate" in source
    assert "if phase in trackers and not releasing and not warming:" in source


def test_slam_runs_see_the_pallet_again_before_the_final_straight() -> None:
    """Plan v3.6: near capture from final_straight_prefix, no silent fallback."""
    source = SCRIPT.read_text()
    assert "final_straight_prefix(\n                                    paths[\"approach\"], geometry.alignment_straight_m" in source
    assert 'f"near_capture_failed:{attempt[\'retry_reason\']}"' in source
    assert 'if slam is not None and "near_capture" not in state:' in source


def test_slam_runs_dock_on_a_scan_before_the_delivery_straight() -> None:
    """Plan v3.8: stop at the straight start, match, move the whole drop."""
    source = SCRIPT.read_text()
    assert 'dock_at_delivery_straight(t)' in source
    assert 'ignore_self=True' in source
    # The withdraw follows every goal change (plan D7c factored it into move_withdraw).
    assert "set_withdraw(np.array([compose(tuple(frame_change), tuple(pose)) for pose in paths[\"withdraw\"].poses]))" in source
    assert source.count("move_withdraw(step_shift)") == 2
    # Three SLAM recoveries, plus the priority-5 obstacle replan of the
    # transport leg and the return leg's fallback to it.
    # (and the zero-clearance retry of that obstacle replan), plus the live
    # replans of the transport and return legs when they start.
    # Plan D7c: the two obstacle replans of the transport now share transport_replan
    # (one lookup), and the docking retry's release and re-approach plan add three.
    assert source.count('slam.get("transport_scenario", scenario)') == 10


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
    functions = {node.name: ast.unparse(node) for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
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
    assert "trackers[\"approach\"].restart_speed_slew(0.0)" in source
    assert "tracking = replace(tracking, status=\"tracking\", speed_mps=0.0)" in source
    # The recorder is written first in the finally block, guarded.
    finally_at = source.index("    finally:\n        if recorder is not None:")
    assert finally_at < source.index("state[\"pocket_recording\"] = recorder.close(") < source.index(
        "state[\"pocket_recording_close_error\"] = repr(exc)") < source.index("write_slam_record(args, state, replace(scenario")
    # A failed close fails the run without replacing an earlier failure reason.
    assert "state.setdefault(\"failure_reason\", f\"pocket_recording_close_failed: {exc!r}\")" in source
    # The control pose read with a frame carries the time it was taken at (a capture in the
    # same loop iteration moves it past the loop's t).
    assert "rear = slam_rear(now_s)\n                            rear_stamp = now_s" in source
    assert "pocket[\"control_stamped\"] = (rear_stamp," in source


def test_d8b_custom_cruise_drives_the_straight_but_not_the_acceptance_dry_run() -> None:
    """The near capture's go/no-go stays the baseline's (smoke run: a 0.30 m/s dry run
    arrives 7.76 mm short, past the 6 mm acceptance); the custom-cruise one is recorded."""
    source = SCRIPT.read_text()
    acceptance = source.index("acceptance_config = replace(acceptance_config,\n")
    assert "cruise_speed_mps=speeds[\"approach\"])" in source[acceptance:acceptance + 200]
    dry = source.index("acceptance_config,\n                                        rear,\n")
    driven = source.index("state[\"near_capture\"][\"dry_run_driven\"] = asdict(bicycle_rollout(")
    tracker = source.index("approach_config,\n                                    )\n")
    assert acceptance < dry < driven < tracker


def test_d8b_dwells_hold_the_steering_as_well_as_the_wheels() -> None:
    """Matrix 1883: steering turned 0.18-0.25 rad in every start dwell and the body crept."""
    source = SCRIPT.read_text()
    block = source[source.index("hold_, released_ = dwells.update("):source.index("elif released_:")]
    assert "dwell_hold = True" in block
    assert "if estop_holding or obstacle_hold or dwell_hold:" in source
    assert "steering_command.copy() if (estop_holding or obstacle_hold or dwell_hold)" in source
    # The wheels brake on the held steering's curvature, in the command actually applied.
    assert "wheel_curvature = kappa if dwell_hold else curvature" in source
    assert "drive = ackermann_command(wheel_speed, wheel_curvature, drive_geometry)" in source
    assert "ackermann_command(wheel_speed, curvature," not in source
    assert source.index("dwell_hold = dwell_standing = False") < source.index("hold_, released_ = dwells.update(")


def test_d8b_dwells_lock_the_wheels_and_give_their_gains_back() -> None:
    """Confirmation 1968: the velocity drives let a standing truck roll 0.1-0.15 mm/s."""
    source = SCRIPT.read_text()
    lock = source[source.index('if dwell_standing and wheel_lock["q"] is None:'):source.index("joint_positions=steering_command, joint_indices=steers")]
    assert "locked_kps[wheels] = DWELL_WHEEL_KP" in lock and "controller_.set_gains(kps=locked_kps, kds=kds_)" in lock
    assert 'elif not dwell_standing and wheel_lock["q"] is not None:' in lock
    # Locked once standing, never while braking (diagnostic 1984).
    assert 'dwell_standing = dwells.active is not None and dwells.active["still_since_s"] is not None' in source
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
    block = source[source.index("safety_stops.update("):source.index("phase == \"observe\" and tracking.status")]
    assert "dwell_hold = True" in block and "dwell_standing" not in block
    assert "safety_hold = True" in block
    assert "restart_speed_slew(0.0)" in block
    # Reset every loop tick, before the stop block can set it.
    assert source.index("safety_hold = False") < source.index("safety_stops.update(")
    # The zero comes after the slew and the permission cap, last before the drive command.
    slew = source.index("wheel_speed = previous + float(np.clip(target_speed - previous, -rate, rate))")
    zero = source.index("if estop_holding or safety_hold:\n                    wheel_speed = 0.0")
    assert slew < source.index("cap = obstacle.get(\"permission_cap\")") < zero
    assert zero < source.index("drive = ackermann_command(wheel_speed, wheel_curvature, drive_geometry)")
    tree = ast.parse(source)
    functions = {node.name: ast.unparse(node) for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    assert "applied_speed_mps=state.get('applied_command'" in functions["record_truth_tick"]
    recorded = source.index("state[\"applied_command\"] = (float(drive.speed_mps)")
    assert source.index("drive = ackermann_command(wheel_speed, wheel_curvature, drive_geometry)") < recorded
    assert recorded < source.index("joint_velocities=np.asarray(drive.wheel_rates_rad_s)")


def test_s4_near_tracking_options_validate_before_startup() -> None:
    for flags, expected in (
        (["--near-tracking"], "--near-tracking needs --slam-feedback"),
        (["--near-tracking-stop-at", "insert"], "--near-tracking-stop-at needs --near-tracking"),
    ):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--base-scene", "unused.usda", "--pallet-urdf", "unused.urdf",
             "--forklift-urdf", str(PROVISIONAL_URDF), "--pallet-geometry", "unused.yaml", "--settings",
             "unused.yaml", "--output", "unused", "--seed", "2", *flags],
            capture_output=True, text=True, check=False,
        )
        assert result.returncode == 2, flags
        assert expected in result.stderr, (flags, result.stderr)


def test_s4_near_tracking_runs_only_behind_the_flag_and_before_the_arrival() -> None:
    """Plan D8 S4a: with --near-tracking off the run is unchanged -- every call into the
    near-field path sits under a `near is not None` test -- and the stop is decided before
    the tick's arrival transition, takes the safety-stop path and defers the arrival."""
    import ast

    source = SCRIPT.read_text()
    tree = ast.parse(source)
    parents = {child: node for node in ast.walk(tree) for child in ast.iter_child_nodes(node)}
    guarded_calls = ("build_near_tracker", "feed_near_frame", "near_tick")
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", None) in guarded_calls]
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
    assert source.index("            check_near_contract()\n        for step in range(") > 0
    assert "until S4a-2" not in source
    # Decided after the scheduled safety stops and before the arrival transitions.
    tick = source.index("hold_, released_, arrive_ = near_tick(t, rear_stamp, rear, requested_speed, phase, curvature)")
    assert source.index("safety_stops.update(") < tick < source.index("elif phase == \"approach\":\n")
    block = source[tick:source.index("phase == \"observe\" and tracking.status", tick)]
    for line in ("requested_speed = 0.0", "dwell_hold = True", "safety_hold = True",
                 "tracking = replace(tracking, status=\"tracking\", speed_mps=0.0)",
                 "trackers[phase].restart_speed_slew(0.0)"):
        assert line in block, line
    # S4a-2: the carriage-corner insertion end becomes the insertion's arrival, standing.
    arrive = block.index("if arrive_:")
    assert arrive < block.index("tracking = replace(tracking, status=\"arrived\", speed_mps=0.0)") < block.index("elif hold_:")
    assert "state[\"near_tracking\"][\"insert_arrival\"]" in source
    # Built in the hold the approach transition begins; fed the raw frame before D5 reads it.
    build = source.index("build_near_tracker(t)  # in the hold the transition began")
    assert source.rindex("transition(\"approach\", t)", 0, build) > build - 300
    feed = source.index("if feed_near_frame(raw) == \"bad_depth\":\n                return")
    assert source.index("record_pocket_read(raw)\n        if raw is None:") < feed < source.index(
        "        if check is None:\n            pocket[\"frames_read\"] += 1")
    # S4a-1 ends at the approach's arrival, and the loop end lets that phase return.
    assert "transition(\"near_tracking_stopped\", t)\n                            break" in source
    assert "\"repeat_target_not_reached\", \"near_tracking_stopped\"):\n            return" in source
    # Terminal stops end the run only once the stop detector sees the truck stand.
    functions = {n.name: ast.unparse(n) for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    feed_fn = functions["feed_near_frame"]
    assert feed_fn.index("skipped='bad_depth'") < feed_fn.index("get_current_frame()") < feed_fn.index(
        "row['skipped'] = 'uncovered'")
    tick_fn = functions["near_tick"]
    assert "near_tracking.decide_stop(" in tick_fn and "if action['end']:" in tick_fn
    assert "standing = bool(slam['stop_now'])" in tick_fn
    # A gap shortfall ends the run only on a fresh observation; otherwise it waits.
    tick_src = source[source.index("    def near_tick("):source.index("    def read_pocket_frame(")]
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
