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


def test_a_leg_conditioned_spawn_waits_for_a_leg_long_enough():
    # Codex checkpoint 9 P1: a short reverse leg (a 0.30 m backoff) clamped the
    # box to its end, inside the truck. A leg-conditioned spawn needs the leg to
    # reach ahead_m + 0.5 m; a shorter leg leaves the event (and its group) unfired.
    sched = MODULE.Schedule([MODULE.Event("n4", "spawn", "transport", 0.0, ahead_m=1.5, leg="reverse", group="g")])
    short = np.column_stack((np.linspace(0, -0.3, 4), np.zeros(4), np.zeros(4)))
    assert sched.update(0.0, "transport", 1.0, short, leg_direction=-1) == []
    long_ = np.column_stack((np.linspace(0, -3.0, 31), np.zeros(31), np.zeros(31)))
    out = sched.update(0.1, "transport", 1.0, long_, leg_direction=-1)
    assert out and math.isclose(out[0][2], -1.5)


def test_a_forward_only_spawn_and_no_leg_during_a_backoff():
    sched = MODULE.Schedule([MODULE.Event("n6", "spawn", "return_home", 0.0, ahead_m=1.59, leg="forward")])
    assert sched.update(0.0, "return_home", 1.0, PATH, leg_direction=-1) == []
    assert sched.update(0.0, "return_home", 1.0, PATH, leg_direction=0) == []  # backoff: no leg
    assert sched.update(0.1, "return_home", 1.0, PATH, leg_direction=1)[0][1] == "n6"


def test_a_spawn_logs_the_path_curvature_there():
    # N3 is judged afterwards: was the box inside a curve?
    theta = np.linspace(0, 1.5, 61)
    arc = np.column_stack((2 * np.sin(theta), 2 * (1 - np.cos(theta)), theta))  # radius 2, left turn
    sched = MODULE.Schedule([MODULE.Event("n3", "spawn", "transport", 0.0, ahead_m=1.0, lateral_m=0.55)])
    sched.update(0.0, "transport", 1.0, arc)
    assert math.isclose(sched.log[-1]["curvature_inv_m"], 0.5, rel_tol=0.05)


def test_a_curve_conditioned_spawn_waits_for_a_curve_and_takes_its_inside():
    # N3 (Codex checkpoint 9 P2): only inside a curve, on the inner side.
    theta = np.linspace(0, -1.5, 61)
    right = np.column_stack((2 * np.sin(-theta), -2 * (1 - np.cos(theta)), theta))  # radius 2, right turn
    sched = MODULE.Schedule([MODULE.Event("n3", "spawn", "transport", 0.0, ahead_m=1.0, lateral_m=0.7,
                                          min_curvature_inv_m=0.3, inside=True)])
    assert sched.update(0.0, "transport", 1.0, PATH) == []  # a straight: wait
    out = sched.update(0.1, "transport", 1.0, right)
    x, y, yaw = MODULE.pose_along(right, 1.0, -0.7)  # the inside of a right turn is to the right
    assert math.isclose(out[0][2], x) and math.isclose(out[0][3], y)


def test_an_outside_spawn_takes_the_outer_side_of_the_curve():
    theta = np.linspace(0, 1.5, 61)
    left = np.column_stack((2 * np.sin(theta), 2 * (1 - np.cos(theta)), theta))  # left turn
    sched = MODULE.Schedule([MODULE.Event("n3", "spawn", "transport", 0.0, ahead_m=1.0, lateral_m=0.75,
                                          min_curvature_inv_m=0.3, outside=True)])
    out = sched.update(0.0, "transport", 1.0, left)
    x, y, _ = MODULE.pose_along(left, 1.0, -0.75)  # the outside of a left turn is to the right
    assert math.isclose(out[0][2], x) and math.isclose(out[0][3], y)


def test_a_sweep_checked_spawn_takes_the_widest_side_offset_the_turn_meets_and_a_straight_clears():
    # Codex checkpoint 14: the curvature rule alone placed a box no sweep met.
    calls = []

    def sweep(box, path):
        calls.append(box[3] if False else None)
        lateral = abs(box[1])  # the test path runs along x: |y| is the side offset
        return lateral <= 0.65, lateral <= 0.55  # the turn reaches 0.65, a straight 0.55

    theta = np.linspace(0, 1.5, 61)
    left = np.column_stack((2 * np.sin(theta), 2 * (1 - np.cos(theta)), theta))
    path = np.column_stack((np.linspace(0, 4, 41), np.zeros(41), np.zeros(41)))
    sched = MODULE.Schedule([MODULE.Event("n3", "spawn", "transport", 0.0, ahead_m=1.0, lateral_m=0.75,
                                          outside=True, sweep=True)])
    out = sched.update(0.0, "transport", 1.0, path, sweep_test=lambda b, p: sweep((b[0], b[1]), p))
    assert out and math.isclose(abs(out[0][3]), 0.65)
    never = MODULE.Schedule([MODULE.Event("n3", "spawn", "transport", 0.0, ahead_m=1.0, lateral_m=0.75, sweep=True)])
    assert never.update(0.0, "transport", 1.0, path, sweep_test=lambda b, p: (False, False)) == []
    assert not never.events[0].fired  # waits for a later tick


def test_a_cusp_is_not_read_as_a_sharp_curve():
    # Plan v10 D6 (independent review P1-6): the heading came from point differences, so a
    # gear change flipped it by pi and read |kappa| ~ 5 1/m; D7 adds cusps, so N3 could
    # spawn at a cusp. Near a cusp the curvature is 0 (the event waits).
    forward = np.column_stack((np.linspace(0, 1, 11), np.zeros(11), np.zeros(11)))
    back = np.column_stack((np.linspace(0.9, 0.0, 10), np.zeros(10), np.zeros(10)))
    path = np.vstack((forward, back))
    assert MODULE._curvature_at(path, 1.0) == 0.0
    arc = np.array([(2 * math.sin(t), 2 * (1 - math.cos(t)), t) for t in np.linspace(0, 1, 41)])
    assert math.isclose(MODULE._curvature_at(arc, 1.0), 0.5, rel_tol=0.02)


def test_the_tall_and_single_lidar_scenario_files_keep_their_shapes():
    # Plan v10 D6: same footprint, place and timing as the v9 files, 1.3 m tall; N3 fires at
    # 0.25 1/m (the measured chassis plans at 0.29); N9 silences the one LiDAR.
    base = ROOT / "config/p5_scenarios"
    pairs = {"n03_side_box": 0.25, "n04_reverse_box": None, "n06_close_box": None, "n07_wall": None,
             "n08_destination_box": None, "n10_removed_box": None}
    for name, curvature in pairs.items():
        old = MODULE.load_events(base / f"{name}.yaml")
        new = MODULE.load_events(base / f"{name}_tall.yaml")
        assert len(old) == len(new)
        for a, b in zip(old, new):
            if a.action == "spawn":
                assert tuple(b.size_m[:2]) == tuple(a.size_m[:2]) and b.size_m[2] == 1.3
                assert (b.phase, b.after_m, b.ahead_m, b.lateral_m) == (a.phase, a.after_m, a.ahead_m, a.lateral_m)
                if curvature is not None:
                    assert b.min_curvature_inv_m == curvature
    silence = MODULE.load_events(base / "n09_single_silence.yaml")
    assert [(e.action, e.target) for e in silence] == [("silence", "high")]
