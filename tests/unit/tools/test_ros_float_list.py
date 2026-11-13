"""A list ROS parameter built from hand-given numbers (S2 a_1, 2026-11-13: '-2.612,0,0' made rclpy refuse
'Sequence should be of same type'): every value written with a decimal point and no exponent."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from tools import ros_float_list as rfl

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("given, expected", [
    ("-2.612,0,0", "-2.612,0.0,0.0"),  # the S2 a_1 case: integers become floats
    ("2,-3,1", "2.0,-3.0,1.0"),
    ("-2.61191987991333,3.476402099522602e-08,-1.2130309475948675e-07",  # the registry's line_m row
     "-2.61191987991333,0.000000034764021,-0.000000121303095"),
    ("1e-05,1E3,-0.0", "0.00001,1000.0,-0.0"),  # exponents written out: no 'e' reaches ROS
    (" 1.5 , 2 ,3 ", "1.5,2.0,3.0"),
])
def test_values_are_written_with_a_decimal_point_and_no_exponent(given, expected):
    out = rfl.float_list(given, count=3)
    assert out == expected
    for v in out.split(","):
        assert "." in v and "e" not in v.lower()


@pytest.mark.parametrize("bad", ["nan,0,0", "inf,0,0", "x,0,0", "1,2", "1,2,3,4", "", "1,,2"])
def test_non_numbers_non_finite_and_wrong_counts_are_refused(bad):
    with pytest.raises(ValueError):
        rfl.float_list(bad, count=3)


def test_the_cli_prints_the_list_or_exits_2():
    tool = ROOT / "tools" / "ros_float_list.py"
    ok = subprocess.run([sys.executable, str(tool), "3", "-2.612,0,0"], capture_output=True, text=True)
    assert ok.returncode == 0 and ok.stdout.strip() == "-2.612,0.0,0.0"
    bad = subprocess.run([sys.executable, str(tool), "3", "x,0,0"], capture_output=True, text=True)
    assert bad.returncode == 2 and "x" in bad.stderr


def test_every_container_builds_map_start_pose_from_the_normalized_list():
    for name in ("slam_bridge_container.sh", "slam_cost_container.sh", "slam_loc_replay_container.sh"):
        script = (ROOT / "tools" / name).read_text()
        assert 'map_start_pose:=[$SLAM_MAP_START_POSE]' not in script, name
        assert "/workspace/tools/ros_float_list.py 3" in script, name
