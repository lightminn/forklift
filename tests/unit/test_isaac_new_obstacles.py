"""Priority-5 L4 new-obstacle rules (no Isaac): placement along the path, firing once."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("new_obstacles", ROOT / "sim/isaac/new_obstacles.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["new_obstacles"] = MODULE
SPEC.loader.exec_module(MODULE)

PATH = np.column_stack((np.linspace(0, 4, 41), np.zeros(41), np.zeros(41)))


def test_a_box_lands_ahead_along_the_path_and_to_its_left():
    x, y, yaw = MODULE.pose_along(PATH, 2.5, 0.3)
    assert math.isclose(x, 2.5) and math.isclose(y, 0.3) and yaw == 0.0
    assert MODULE.pose_along(PATH, 9.0, 0.0)[0] == 4.0  # clamped to the end


def test_events_fire_once_in_their_phase_after_their_distance():
    events = [
        MODULE.Event("n1", "spawn", "transport", 2.0, ahead_m=1.5),
        MODULE.Event("n10", "remove", "transport", 5.0, target="n1"),
        MODULE.Event("n9", "silence", "return_home", 0.0, target="low_fl"),
    ]
    sched = MODULE.Schedule(events)
    assert sched.update(0.0, "transport", 1.0, PATH) == []
    assert sched.update(0.1, "approach", 3.0, PATH) == []
    spawn = sched.update(0.2, "transport", 2.0, PATH)
    assert spawn[0][:2] == ("spawn", "n1") and math.isclose(spawn[0][2], 1.5)
    assert sched.update(0.3, "transport", 3.0, PATH) == []
    assert sched.update(0.4, "transport", 5.0, PATH) == [("remove", "n1")]
    assert sched.update(0.5, "return_home", 0.0, None) == [("silence", "low_fl")]
    assert sched.silenced == {"low_fl"} and not sched.spawned and len(sched.log) == 3
