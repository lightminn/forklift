import json
import subprocess
import sys
from pathlib import Path

EXAMPLE = Path(__file__).resolve().parents[2] / "examples" / "sensor_geometry.py"


def test_documented_example_executes_real_converters_and_preserves_unknown_samples(
    tmp_path: Path,
) -> None:
    result = subprocess.run(
        [sys.executable, str(EXAMPLE)],
        cwd=tmp_path,  # the example must not depend on the checkout as cwd
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
