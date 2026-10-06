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


def test_a_removal_can_follow_a_spawn_by_time_and_a_box_can_sit_past_the_end():
    events = [
        MODULE.Event("n10", "spawn", "transport", 1.0, ahead_m=2.0),
        MODULE.Event("n10_gone", "remove", "transport", 0.0, target="n10", after_event="n10", delay_s=5.0),
        MODULE.Event("n8", "spawn", "transport", 0.5, beyond_end_m=1.23),
    ]
    sched = MODULE.Schedule(events)
    first = sched.update(0.0, "transport", 1.0, PATH)
    assert {a[1] for a in first} == {"n10", "n8"}
    n8 = next(a for a in first if a[1] == "n8")
    assert math.isclose(n8[2], 4.0 + 1.23)
    assert sched.update(4.0, "transport", 1.0, PATH) == []  # still standing in front of it
    assert sched.update(5.5, "transport", 1.0, PATH) == [("remove", "n10")]


def test_a_bar_can_be_placed_in_the_pallet_frame():
    # N11/N12/N14: along the insertion axis from the approach face (inward
    # positive), lateral to the left of the insertion direction.
    events = [MODULE.Event("n11", "spawn", "approach", 0.5, frame="pallet", along_m=0.05, lateral_m=0.145,
                           size_m=(0.03, 0.03, 0.09))]
    sched = MODULE.Schedule(events)
    face = (2.0, 1.0, math.pi / 2)  # face centre, insertion heading +y
    assert sched.update(0.0, "approach", 0.4, PATH, pallet_face=face) == []
    out = sched.update(0.1, "approach", 0.6, PATH, pallet_face=face)
    _, _, x, y, yaw, size = out[0]
    assert math.isclose(x, 2.0 - 0.145) and math.isclose(y, 1.05) and math.isclose(yaw, math.pi / 2)
    assert size == (0.03, 0.03, 0.09)


def test_a_pallet_frame_spawn_waits_for_the_pallet_face():
    sched = MODULE.Schedule([MODULE.Event("n12", "spawn", "approach", 0.0, frame="pallet", along_m=-0.1)])
    assert sched.update(0.0, "approach", 1.0, PATH, pallet_face=None) == []
    assert sched.update(0.1, "approach", 1.0, PATH, pallet_face=(0.0, 0.0, 0.0))[0][2] == -0.1


def test_a_reverse_only_event_fires_on_a_reverse_leg():
    # N4: a box on the reverse leg after a cusp; it waits for one.
    sched = MODULE.Schedule([MODULE.Event("n4", "spawn", "transport", 0.0, ahead_m=1.0, on_reverse=True)])
    assert sched.update(0.0, "transport", 2.0, PATH, leg_direction=1) == []
    reverse_leg = PATH[::-1]
    out = sched.update(0.1, "transport", 2.5, reverse_leg, leg_direction=-1)
    assert out[0][:2] == ("spawn", "n4") and math.isclose(out[0][2], 3.0)


def test_events_in_one_group_fire_only_once_between_them():
    events = [
        MODULE.Event("n4_obs", "spawn", "observe", 0.0, ahead_m=1.0, on_reverse=True, group="n4"),
        MODULE.Event("n4_tr", "spawn", "transport", 0.0, ahead_m=1.0, on_reverse=True, group="n4"),
    ]
    sched = MODULE.Schedule(events)
    assert len(sched.update(0.0, "observe", 1.0, PATH, leg_direction=-1)) == 1
    assert sched.update(1.0, "transport", 1.0, PATH, leg_direction=-1) == []


def test_the_rules_load_the_new_fields():
    import tempfile

    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write("events:\n  - {id: a, action: spawn, phase: approach, after_m: 0.5, frame: pallet, along_m: 0.3,"
                " lateral_m: -0.145, size_m: [0.03, 0.03, 0.09]}\n"
                "  - {id: b, action: spawn, phase: transport, after_m: 0, ahead_m: 1, on_reverse: true, group: g}\n"
                "  - {id: c, action: silence, phase: insert, after_m: 0.05, target: pocket_camera}\n")
    a, b, c = MODULE.load_events(Path(f.name))
    assert a.frame == "pallet" and a.along_m == 0.3 and a.lateral_m == -0.145
    assert b.on_reverse and b.group == "g" and c.target == "pocket_camera"
