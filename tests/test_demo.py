import json
import subprocess
import sys
from pathlib import Path


def test_documented_demo_executes_real_converters_and_preserves_unknown_samples():
    result = subprocess.run(
        [sys.executable, "-m", "forklift_core.demo"],
        cwd=Path(__file__).resolve().parents[1],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr
    output = json.loads(
        result.stdout,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)),
    )
    assert output["input_source"] == "synthetic"
    assert output["depth_in_base"]["xyz_m"][0] == [2.2, 0.0, 0.5]
    assert output["depth_in_base"]["valid"] == [True, True, False]
    assert output["depth_in_base"]["xyz_m"][2] == [None, None, None]
    assert output["lidar_in_sensor"]["valid"] == [True, True, False]
