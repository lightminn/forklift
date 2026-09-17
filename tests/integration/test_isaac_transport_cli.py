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
