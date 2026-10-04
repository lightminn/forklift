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
    assert "moved = np.array([compose(step_shift, tuple(pose)) for pose in paths[\"withdraw\"].poses])" in source
    assert source.count('slam.get("transport_scenario", scenario)') == 2
