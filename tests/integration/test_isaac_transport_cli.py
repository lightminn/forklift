"""The optional simulator launcher must expose usage without starting Isaac Sim."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim/isaac/run_transport.py"


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


def run_geometry_cli(tmp_path, pallet_urdf, geometry="t11_06"):
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--base-scene",
            "unused.usda",
            "--pallet-urdf",
            str(pallet_urdf),
            "--pallet-geometry",
            str(ROOT / f"config/pallet_geometry_{geometry}.yaml"),
            "--settings",
            str(ROOT / "config/isaac_transport.yaml"),
            "--output",
            str(tmp_path / "run"),
            "--seed",
            "0",
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def test_t11_configuration_reaches_sdk_startup(tmp_path, synthetic_pallet_urdf):
    # A sentinel SDK stops startup even on hosts with Isaac installed.
    import json
    import os

    (tmp_path / "isaacsim.py").write_text('raise RuntimeError("SDK_STARTUP_REACHED")')
    old = os.environ.get("PYTHONPATH", "")
    from unittest.mock import patch

    with patch.dict(os.environ, {"PYTHONPATH": str(tmp_path) + os.pathsep + old}):
        result = run_geometry_cli(tmp_path, synthetic_pallet_urdf())
    assert result.returncode == 1, result.stderr
    assert "SDK_STARTUP_REACHED" in result.stderr
    record = json.loads((tmp_path / "run/result.json").read_text())
    assert record["arguments"]["axle_to_fork_tip_m"] == 1.29
    assert record["arguments"]["rear_axle_offset_m"] == -0.34
    assert record["arguments"]["pallet_geometry_loaded"]["overall_depth_m"] == 0.66


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
