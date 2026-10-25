"""The P6 escape's own start-pad release (docs/plans/2026-10-25-escape-start-release.md E1-E3): the scene replan's
escape sweep passes the trigger's rectangles the stop already stands within the pad of while it moves away from them,
with the permission left as it is -- no start band, no anchor; the escape's other checks and its final permit stay."""

import numpy as np
import pytest
from test_core_scene_replan import DT, make  # the device's core fixtures
from test_start_band_escape import _escape_through_the_device, _near_rect, _standing

from forklift_core.planning import Rectangle


def _plan(core, extra):
    pose = (2.0, 0.3, 0.05)
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    return ports.escape_plan(tuple(extra), None)


def test_on_the_escape_backs_away_from_a_rectangle_within_the_pad_and_names_it():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    chosen = _plan(core, [rect])
    rec = state["scene_escape_candidates"][-1]
    assert chosen is not None and rec["permit"]["ok"]
    assert rec["released"] == [{"x_m": rect.x_m, "y_m": rect.y_m, "length_m": 0.05, "width_m": 0.05, "yaw_rad": 0.0}]
    assert core.obstacle["escape_start_release"] == {"escapes_released": 1}


def test_off_the_same_rectangle_refuses_every_straight_at_its_start():
    core, ms, state, calls, rear, held = make()
    pose = _standing(core)
    rect = _near_rect(core, pose)
    assert _plan(core, [rect]) is None
    rec = state["scene_escape_candidates"][-1]
    assert not any(c["sweep_free"] for c in rec["candidates"])
    assert "released" not in rec and "escape_start_release" not in core.obstacle


def test_on_the_permission_keeps_its_own_start_rule():
    # the release is the sweep's alone: no start band, no anchor latched (unlike --start-band-release on)
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    _plan(core, [rect])
    perm = core.obstacle["layer"].permission
    assert perm.config.start_band_m == 0.0 and perm.release_anchor is None


def test_on_a_rectangle_the_start_overlaps_is_not_released():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    over = Rectangle(pose[0] + 0.6, pose[1], 0.05, 0.05, 0.0)  # under the truck's middle
    assert _plan(core, [over]) is None
    rec = state["scene_escape_candidates"][-1]
    assert "released" not in rec and not any(c["sweep_free"] for c in rec["candidates"])
    assert "escape_start_release" not in core.obstacle


def test_on_a_released_rectangle_still_counts_for_the_end_region():
    # a second rectangle within the pad that the straight backs along stays within the end threshold of its end
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    long_ = Rectangle(rect.x_m - 1.5 * np.cos(pose[2]), rect.y_m - 1.5 * np.sin(pose[2]), 3.0, 0.05, pose[2])
    _plan(core, [long_])
    rec = state["scene_escape_candidates"][-1]
    assert rec.get("released"), "the fixture's rectangle must be released"
    backs = [c for c in rec["candidates"] if c["direction"] == -1]
    assert all(c["sweep_free"] for c in backs)  # it runs alongside: never nearer
    assert all(not c["qualifies"] and c["end_clearance_m"] < c["threshold_m"] for c in backs)


def test_on_a_refused_final_permit_installs_no_escape():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    layer = core.obstacle["layer"]
    layer.candidate_permit = lambda *a, **k: {"ok": False, "why": "occupied"}
    assert _plan(core, [rect]) is None
    assert state["scene_escape_candidates"][-1]["chosen"] is not None


def test_on_through_the_device_the_permission_still_holds_a_start_within_its_own_band():
    # the start-band test's stop beside a wall of cells 8-19 mm off: the sweep part alone does not override the
    # permission's own rule, which holds the start (only --start-band-release on releases the walk)
    import test_start_band_escape as sbe

    real_make = sbe.make

    def make_escape_only():
        core, *rest = real_make()
        core.options.escape_start_release = "on"
        return (core, *rest)

    sbe.make = make_escape_only
    try:
        state = _escape_through_the_device(False)
    finally:
        sbe.make = real_make
    rec = state["scene_escape_candidates"][-1]
    assert rec["chosen"] is None


def test_on_a_start_nearer_than_the_dense_sweep_error_is_not_passed():
    # agy: 0 < d0 < min_d0_m -- named as within the pad, but the moving-away rule refuses it (the continuous sweep
    # could touch between the checked poses), so no straight passes the sweep
    import math

    from forklift_core.control import escape as esc
    from forklift_core.control.start_release import footprint_rect_distance, min_d0_m

    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    footprint = core.escape_leg("transport")[3][1]
    rect = _near_rect(core, pose)
    gap = footprint_rect_distance(pose, footprint, rect)
    shift = gap - min_d0_m(footprint) / 2.0  # move it in to half the dense error
    c, s = math.cos(pose[2]), math.sin(pose[2])
    close = Rectangle(rect.x_m + shift * s, rect.y_m - shift * c, 0.05, 0.05, 0.0)
    assert 0.0 < footprint_rect_distance(pose, footprint, close) < min_d0_m(footprint)
    assert esc.start_band_rectangles([close], footprint, pose)[0] == [close]
    assert _plan(core, [close]) is None
    rec = state["scene_escape_candidates"][-1]
    assert rec["released"] and not any(c_["sweep_free"] for c_ in rec["candidates"])


def test_both_switches_on_release_the_sweep_once_and_keep_both_records_apart():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    core.options.start_band_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    assert _plan(core, [rect]) is not None
    assert len(state["scene_escape_candidates"][-1]["released"]) == 1
    assert core.obstacle["escape_start_release"] == {"escapes_released": 1}


def test_the_start_band_release_alone_keeps_no_escape_start_record():
    core, ms, state, calls, rear, held = make()
    core.options.start_band_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    _plan(core, [rect])
    assert state["scene_escape_candidates"][-1]["released"]
    assert "escape_start_release" not in core.obstacle


def _installed(core, state):
    """A released escape planned and installed; the device's tick drives it (stub device: always the escape)."""
    from types import SimpleNamespace

    core.options.escape_start_release = "on"
    pose = _standing(core)
    rect = _near_rect(core, pose)
    ports = core._scene_ports(0.2, np.array(pose), 0.0, DT)
    chosen = ports.escape_plan((rect,), None)
    assert chosen is not None and chosen[3] is not None
    ports.escape_install(chosen)
    core.hooks.truth_velocity = lambda: np.zeros(3)
    core.scene_replan = SimpleNamespace(active=lambda: True,
                                        step=lambda t, p: SimpleNamespace(owned=True, output="escape", steer=None,
                                                                          end=None))
    return pose, rect, chosen


def test_a_released_escape_stops_once_the_outline_comes_nearer_than_half_its_start_gap():
    # Codex plan review P2-1: the driving permission checks other shapes than the planner's outline
    import math

    core, ms, state, calls, rear, held = make()
    pose, rect, chosen = _installed(core, state)
    d0 = chosen[3]["d0"][0]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    near = np.array([pose[0] - 0.75 * d0 * s, pose[1] + 0.75 * d0 * c, pose[2]])  # toward the left side's rectangle
    core.scene_replan_tick(0.3, near, 0.0, DT)
    assert core.scene_escape["status"] == "stopped"
    assert core.obstacle["escape_start_release"]["stopped_approach"] == 1


def test_a_released_escape_ends_done_only_where_its_end_criterion_holds():
    # Codex plan review P2-2: the escape counts as done on a failed or used-up tracker, off the checked end region
    core, ms, state, calls, rear, held = make()
    pose, rect, chosen = _installed(core, state)
    end = np.array(chosen[0][-1])
    core.scene_replan_tick(0.3, end, 0.0, DT)
    assert core.scene_escape["status"] == "done"
    assert core.obstacle["escape_start_release"]["done"] == 1

    core2, ms2, state2, *_ = make()
    pose2, rect2, chosen2 = _installed(core2, state2)
    core2.scene_escape["guard"]["threshold_m"] = 10.0  # an end no pose can keep
    core2.scene_replan_tick(0.3, np.array(chosen2[0][-1]), 0.0, DT)
    assert core2.scene_escape["status"] == "stopped"
    assert core2.obstacle["escape_start_release"]["stopped_end"] == 1


def test_an_escape_without_a_released_rectangle_carries_no_guard():
    core, ms, state, calls, rear, held = make()
    core.options.escape_start_release = "on"
    pose = _standing(core)
    chosen = _plan(core, [])
    assert chosen is not None and chosen[3] is None


def test_the_guard_s_stop_holds_the_wheels_at_zero_on_the_same_tick_with_the_steering_held():
    # Codex re-review P2-1: the slew left -0.0875 m/s and a new steering on the tick the guard stopped
    import math

    core, ms, state, calls, rear, held = make()
    pose, rect, chosen = _installed(core, state)
    d0 = chosen[3]["d0"][0]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    near = np.array([pose[0] - 0.75 * d0 * s, pose[1] + 0.75 * d0 * c, pose[2]])
    core.scene_replan_tick(0.3, near, -0.1, DT)
    assert core.scene_escape["status"] == "stopped"
    core.options.obstacle_act = True
    core.options.command_accel_mps2 = 0.5
    core.settings = {"steering_command_rate_rad_s": 1.0}
    core.hooks.steering_position = lambda: np.zeros(2)
    state["command_speed"] = -0.1
    steering = np.array([0.02, 0.02])
    out = core.assemble_command(0.3, DT, -0.1, 0.3, 0.0, steering, False, False, False, False, False)
    assert out.drive.speed_mps == 0.0 and out.requested_speed == 0.0
    assert np.allclose(out.steering_command, [0.02, 0.02])  # held, not turned toward 0.3 m^-1
    other = core.assemble_command(0.3 + DT, DT, -0.1, 0.0, 0.0, steering, False, False, False, False, False)
    assert other.drive.speed_mps != 0.0  # only the tick it stopped on


def test_the_guard_looks_a_stop_length_ahead_of_the_pose():
    from forklift_core.mission.core import MissionCore
    from forklift_core.planning import Footprint

    foot = Footprint(1.29, 0.17, 0.36)
    # a rectangle behind the rear face, 0.05 m off; backing at it: the pose is clear, the stop ahead is not
    behind = Rectangle(-0.17 - 0.05 - 0.025, 0.0, 0.05, 0.05, 0.0)
    guard = {"rects": [behind], "d0": [0.05], "footprint": foot, "direction": -1, "stop_m": 0.04}
    assert MissionCore._escape_guard_breached(guard, (0.0, 0.0, 0.0))
    assert not MissionCore._escape_guard_breached(dict(guard, stop_m=0.01), (0.0, 0.0, 0.0))
    assert not MissionCore._escape_guard_breached(dict(guard, direction=1), (0.0, 0.0, 0.0))


def test_a_correction_change_moves_the_guard_s_rectangles_with_the_grid():
    # Codex re-review P2-2: +50 mm of correction alone stopped the escape against the rectangles of the old
    # projection; -50 mm with a 7 mm drift toward the rectangle went unseen
    import math

    from forklift_core.localization.slam_pose import compose

    core, ms, state, calls, rear, held = make()
    pose, rect, chosen = _installed(core, state)
    guard = core.scene_escape["guard"]
    delta = (0.0, 0.05, 0.0)
    core._escape_guard_reexpress(delta)
    guard["applied"] = delta
    moved = compose(delta, pose)
    assert not core._escape_guard_breached(guard, moved)  # the same place on the grid: no false stop
    d0 = guard["d0"][0]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    toward = (moved[0] - 0.75 * d0 * s, moved[1] + 0.75 * d0 * c, moved[2])
    assert core._escape_guard_breached(guard, toward)
    assert guard["checker"].free(moved, 0.0)
    assert core.obstacle["escape_start_release"]["reexpressed"] == 1


def test_the_guard_follows_the_correction_applied_by_any_path_before_it_judges():
    # Codex re-review 3 P2-3: the watch and the runner's scan apply a correction without _reproject
    import math

    from forklift_core.localization.slam_pose import compose

    core, ms, state, calls, rear, held = make()
    pose, rect, chosen = _installed(core, state)
    core.obstacle["applied"] = (0.0, 0.05, 0.0)  # applied elsewhere; the guard was planned under (0, 0, 0)
    moved = np.array(compose((0.0, 0.05, 0.0), pose))
    core.scene_replan_tick(0.3, moved, 0.0, DT)
    assert core.scene_escape["status"] == "running"  # the same place on the grid: no false stop
    assert core.obstacle["escape_start_release"]["reexpressed"] == 1
    core.scene_replan_tick(0.3 + DT, moved, 0.0, DT)
    assert core.obstacle["escape_start_release"]["reexpressed"] == 1  # once per change
    d0 = chosen[3]["d0"][0]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    core.scene_replan_tick(0.3 + 2 * DT, moved + np.array([-0.75 * d0 * s, 0.75 * d0 * c, 0.0]), 0.0, DT)
    assert core.scene_escape["status"] == "stopped"


def test_an_end_criterion_stop_is_a_hold_on_its_own_tick_too():
    # Codex re-review 3 P2-1: stopped_end set no stop time and left -0.0875 m/s
    core, ms, state, calls, rear, held = make()
    pose, rect, chosen = _installed(core, state)
    core.scene_escape["guard"]["threshold_m"] = 10.0
    core.scene_replan_tick(0.3, np.array(chosen[0][-1]), 0.0, DT)  # arrived, standing
    assert core.obstacle["escape_start_release"]["stopped_end"] == 1
    core.options.obstacle_act = True
    core.options.command_accel_mps2 = 0.5
    core.settings = {"steering_command_rate_rad_s": 1.0}
    core.hooks.steering_position = lambda: np.zeros(2)
    state["command_speed"] = -0.1
    out = core.assemble_command(0.3, DT, -0.1, 0.0, 0.0, np.array([0.02, 0.02]), False, False, False, False, False)
    assert out.drive.speed_mps == 0.0


def test_the_guard_looks_along_the_held_steering_s_stop_arc():
    # Codex re-review 3 P2-2: backing at -0.29 m^-1 swung the outline onto the rectangle 11 mm on while the
    # straight stayed clear
    from forklift_core.mission.core import MissionCore
    from forklift_core.planning import Footprint

    foot = Footprint(1.29, 0.17, 0.36)
    # beside the left side near the front, 8 mm off: a straight back keeps the gap
    side = Rectangle(1.0, 0.36 + 0.008 + 0.025, 0.05, 0.05, 0.0)
    guard = {"rects": [side], "d0": [0.008], "footprint": foot, "direction": -1, "stop_m": 0.1}
    assert not MissionCore._escape_guard_breached(guard, (0.0, 0.0, 0.0), 0.0)
    swung = [k for k in (-0.29, 0.29) if MissionCore._escape_guard_breached(guard, (0.0, 0.0, 0.0), k)]
    assert len(swung) == 1  # the arc that swings the front toward it, not the other
