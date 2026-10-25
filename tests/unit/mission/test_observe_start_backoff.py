"""The first far observation's start backoff (docs/plans/2026-10-26-observe-start-backoff.md): an observe miss whose
replan comes back invalid_start -- the stop within the planner clearance of the pickup prior -- backs off straight by
the start check's own advice and plans the same target once; every other case ends the mission as before."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from test_core_scene_replan import DT, make  # the device's core fixtures (a scene, a real planner and layer)
from test_start_band_escape import _standing

from forklift_core.control import RearAxlePathTracker
from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.baseline import CORE_FIELDS, MISSION_KEYS
from forklift_core.mission.failures import MISSION, classify
from forklift_core.planning import PlanResult, Rectangle
from forklift_core.planning import Pose2D as PlanningPose
from forklift_core.planning import pallet_mission as pm

GAP_M = 0.06  # the stop's gap to the prior, as in the recorded stops (0.060-0.098 m by the checker)


def _observe_core(*, gap_m=GAP_M):
    """A scene core in the observe phase standing at (2.0, 0.3, 0.05) with the pickup prior ``gap_m`` ahead of its
    front: free at zero clearance, not at the planner's 0.10; the observe target 1.2 m straight back."""
    core, ms, state, calls, rear, held = make()
    pose = _standing(core)
    rear["pose"] = np.array(pose)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    fp = core.geometry.unloaded_footprint
    ahead = fp.front_m + gap_m + 0.1
    zone = Rectangle(pose[0] + ahead * c, pose[1] + ahead * s, 0.2, 1.0, pose[2])
    target = PlanningPose(pose[0] - 1.2 * c, pose[1] - 1.2 * s, pose[2])
    ms.phase = "observe"
    poses = np.array([[pose[0] - 1.2 * c * f, pose[1] - 1.2 * s * f, pose[2]] for f in np.linspace(0, 1, 13)])
    core.paths["observe"] = PlanResult(True, "success", poses, np.full(13, -1, np.int8), np.zeros(13), 1.2, 0)
    core.trackers["observe"] = RearAxlePathTracker(poses, np.full(13, -1, np.int8), np.zeros(13),
                                                   TrackerConfig(position_tolerance_m=0.03))
    state["observe_replan_leg"] = {"start": 0, "serial": 0}
    state["observe_replans"] = []
    core.options.observe_start_backoff = "on"
    core.options.pickup_bounds = None
    core.hooks.grid_kwargs = lambda kind=None, **k: {"pickup_obstacle": zone}
    core.hooks.plan_observation_leg = pm.plan_observation_leg
    core.hooks.show_path = lambda *a: None
    core.hooks.truth_velocity = lambda: np.zeros(3)
    core.realign_install = lambda reason: None
    core.release_after_backoff = lambda t, event: None
    # the fixture's layer has seen only ahead of the truck: its checks of the way back pass here (the tests of a
    # refusal set them again), the real ones read unknown as blocked
    core.obstacle["layer"].check_candidate = lambda *a, **k: SimpleNamespace(blocked=None, verified_m=1.0)
    core.obstacle["layer"].candidate_permit = lambda *a, **k: {"ok": True}
    return core, state, pose, target, zone


def test_the_fixture_stop_is_free_at_zero_but_not_at_the_planner_clearance():
    core, state, pose, target, zone = _observe_core()
    checker, footprint, clearance = core._observe_start_check()
    assert clearance == pytest.approx(0.10)
    assert checker.free(pose, 0.0) and not checker.free(pose, clearance)


def test_an_invalid_start_backs_off_straight_by_the_advice_once_per_leg():
    core, state, pose, target, zone = _observe_core()
    leg_config = core.trackers["observe"].config
    assert core._observe_backoff_start(10.0, np.array(pose), target)
    rec = state["observe_start_backoffs"][-1]
    assert rec["installed"] and rec["advice"]["backoff_m"] is not None
    assert rec["length_m"] == pytest.approx(min(rec["advice"]["backoff_m"] + 0.03, rec["advice"]["backoff_max_m"], 0.30))
    tracker = core.trackers["observe"]
    assert tracker is core.observe_backoff["tracker"] and core.observe_backoff["config"] is leg_config
    assert tracker.config.position_tolerance_m == pytest.approx(0.01)  # its own end, not the leg's 0.03
    assert core.observe_backoff_used == [0] and not core.scene_far_observe()
    # once per observe leg: a second miss on the same leg ends as before
    core.observe_backoff = None
    assert not core._observe_backoff_start(11.0, np.array(pose), target)
    assert state["observe_start_backoffs"][-1]["outcome"] == "not_eligible"


def test_a_stop_overlapping_the_prior_does_not_back_off():
    core, state, pose, target, zone = _observe_core(gap_m=-0.02)
    assert not core._observe_backoff_start(10.0, np.array(pose), target)
    assert state["observe_start_backoffs"][-1]["outcome"] == "overlap" and core.observe_backoff is None


@pytest.mark.parametrize("advice, outcome", [({"backoff_m": None, "within_m": 0.30}, "no_room"),
                                             ({"backoff_m": None, "blocked_at_m": 0.05}, "blocked")])
def test_no_valid_start_within_the_cap_or_a_blocked_way_back_ends_as_before(monkeypatch, advice, outcome):
    core, state, pose, target, zone = _observe_core()
    seen = {}
    monkeypatch.setattr(pm, "backoff_advice", lambda free, c, start, **k: seen.update(k) or dict(advice))
    assert not core._observe_backoff_start(10.0, np.array(pose), target)
    assert state["observe_start_backoffs"][-1]["outcome"] == outcome
    assert seen == {"step_m": 0.01, "limit_m": 0.30}


def test_the_length_is_clipped_by_the_advice_s_free_run_and_the_cap(monkeypatch):
    core, state, pose, target, zone = _observe_core()
    monkeypatch.setattr(pm, "backoff_advice", lambda *a, **k: {"backoff_m": 0.05, "backoff_max_m": 0.06})
    assert core._observe_backoff_start(10.0, np.array(pose), target)
    assert state["observe_start_backoffs"][-1]["length_m"] == pytest.approx(0.06)
    core2, state2, pose2, target2, _ = _observe_core()
    monkeypatch.setattr(pm, "backoff_advice", lambda *a, **k: {"backoff_m": 0.29, "backoff_max_m": 1.0})
    assert core2._observe_backoff_start(10.0, np.array(pose2), target2)
    assert state2["observe_start_backoffs"][-1]["length_m"] == pytest.approx(0.30)


def test_a_refused_sweep_path_or_permit_installs_nothing(monkeypatch):
    from forklift_core.control import escape as esc

    core, state, pose, target, zone = _observe_core()
    monkeypatch.setattr(esc, "rectangles_sweep_free", lambda checker, poses, *a: False)
    assert not core._observe_backoff_start(10.0, np.array(pose), target)
    assert state["observe_start_backoffs"][-1]["outcome"] == "precheck_sweep"
    monkeypatch.undo()
    core, state, pose, target, zone = _observe_core()
    core.obstacle["layer"].check_candidate = lambda *a, **k: SimpleNamespace(blocked="occupied")
    assert not core._observe_backoff_start(10.0, np.array(pose), target)
    assert state["observe_start_backoffs"][-1]["outcome"] == "precheck_path:occupied"
    core, state, pose, target, zone = _observe_core()
    core.obstacle["layer"].candidate_permit = lambda *a, **k: {"ok": False}
    assert not core._observe_backoff_start(10.0, np.array(pose), target)
    assert state["observe_start_backoffs"][-1]["outcome"] == "precheck_permit" and core.observe_backoff is None


def _tracking(core, pose, **fields):
    """A tracking result of the installed tracker with ``fields`` replaced."""
    from dataclasses import replace

    return replace(core.trackers["observe"].update(np.array(pose), 0.0, DT), **fields)


def _installed():
    core, state, pose, target, zone = _observe_core()
    assert core._observe_backoff_start(10.0, np.array(pose), target)
    return core, state, pose, target


def test_the_end_stands_checks_the_start_again_plans_once_and_installs_with_the_leg_s_settings():
    core, state, pose, target = _installed()
    leg_config = core.observe_backoff["config"]
    end = np.array(core.trackers["observe"]._poses[-1])
    arrived = core.trackers["observe"].update(end, 0.0, DT)
    assert arrived.status == "arrived"
    out = core.observe_backoff_tick(10.5, end, arrived, 0.0, DT)
    rec = state["observe_start_backoffs"][-1]
    assert rec["outcome"] == "replanned" and rec["end_free"] and rec["replan_status"] == "success"
    assert core.observe_backoff is None and core.trackers["observe"].config is leg_config
    assert out.tracking.status != "arrived"  # never the observe arrival (the capture)
    assert state["observe_replans"][-1]["after_backoff"] is True
    assert core.paths["observe"].success


def test_an_end_short_of_a_valid_start_ends_the_mission_by_name():
    core, state, pose, target = _installed()
    arrived = _tracking(core, pose, status="arrived", speed_mps=0.0)
    with pytest.raises(RuntimeError, match="observe_start_backoff_short"):
        core.observe_backoff_tick(10.5, np.array(pose), arrived, 0.0, DT)
    assert classify("observe_start_backoff_short") == MISSION


def test_a_failed_replan_after_the_backoff_ends_the_mission():
    core, state, pose, target = _installed()
    core.hooks.plan_observation_leg = lambda *a, **k: PlanResult(False, "no_path", np.empty((0, 3)),
                                                                 np.empty(0, np.int8), np.empty(0), 0.0, 0)
    end = np.array(core.trackers["observe"]._poses[-1])
    arrived = core.trackers["observe"].update(end, 0.0, DT)
    with pytest.raises(RuntimeError, match="observe_replan_failed:no_path"):
        core.observe_backoff_tick(10.5, end, arrived, 0.0, DT)


def test_a_failure_while_moving_brakes_first_and_never_ends_the_mission():
    core, state, pose, target = _installed()
    core.hooks.truth_velocity = lambda: np.array([0.1, 0.0, 0.0])  # moving
    failed = _tracking(core, pose, status="failed", speed_mps=-0.1)
    out = core.observe_backoff_tick(10.2, np.array(pose), failed, -0.1, DT)
    assert out.tracking.status == "braking" and out.tracking.speed_mps == 0.0 and core.observe_backoff is not None


def test_the_guard_holds_the_tick_its_stop_reaches_a_rectangle_then_ends_the_mission(monkeypatch):
    import forklift_core.control.stop_sweep as ss

    core, state, pose, target = _installed()
    c, s = math.cos(pose[2]), math.sin(pose[2])
    into = (pose[0] + 0.2 * c, pose[1] + 0.2 * s, pose[2])  # a pose with its front in the prior
    monkeypatch.setattr(ss, "stop_poses", lambda rear, d, length, kappa: [tuple(rear), into])
    moving = _tracking(core, pose, status="tracking", speed_mps=-0.1)
    out = core.observe_backoff_tick(10.2, np.array(pose), moving, -0.1, DT)
    assert out.tracking.status == "braking" and core._observe_backoff_stop_t == 10.2
    core.options.obstacle_act = True
    core.options.command_accel_mps2 = 0.5
    core.settings = {"steering_command_rate_rad_s": 1.0}
    core.hooks.steering_position = lambda: np.zeros(2)
    state["command_speed"] = -0.1
    steering = np.array([0.02, 0.02])
    cmd = core.assemble_command(10.2, DT, -0.1, 0.3, 0.0, steering, False, False, False, False, False)
    assert cmd.drive.speed_mps == 0.0 and np.allclose(cmd.steering_command, [0.02, 0.02])
    with pytest.raises(RuntimeError, match="observe_start_backoff_stopped"):
        core.observe_backoff_tick(10.2 + DT, np.array(pose), moving, -0.1, DT)
    assert classify("observe_start_backoff_stopped") == MISSION


def test_the_guard_does_not_stop_a_plain_backoff_even_with_the_steering_at_the_planner_limit():
    for kappa in (0.0, 0.29, -0.29):
        core, state, pose, target = _installed()
        core.hooks.held_kappa = lambda: kappa
        core.hooks.truth_velocity = lambda: np.array([0.1, 0.0, 0.0])
        moving = _tracking(core, pose, status="tracking", speed_mps=-0.1)
        out = core.observe_backoff_tick(10.2, np.array(pose), moving, -0.1, DT)
        assert out.tracking.status == "tracking" and not core.observe_backoff["stopping"], kappa


def test_the_records_and_markers_are_a_mission_s_own():
    assert "observe_start_backoffs" in MISSION_KEYS["state"]
    assert {"observe_backoff", "observe_backoff_used"} <= set(CORE_FIELDS)


def test_the_baseline_restores_a_clean_backoff_state_for_the_next_mission():
    from forklift_core.mission.baseline import MissionBaseline

    core, state, pose, target, zone = _observe_core()
    base = MissionBaseline.capture(core)
    assert core._observe_backoff_start(10.0, np.array(pose), target)
    records = base.restore(core)
    assert core.observe_backoff is None and core.observe_backoff_used == []
    assert records["state"]["observe_start_backoffs"][-1]["installed"] is True
    assert "observe_start_backoffs" not in core.state or core.state["observe_start_backoffs"] == []


# --- the observe miss's replan (the runner's flow, with test_core_cusp_observe's fakes) ---

def _miss_world(start_ok, *, on=True):
    from test_core_cusp_observe import OBSERVE_MISS, World

    invalid = PlanResult(False, "invalid_start", np.empty((0, 3)), np.empty(0, np.int8), np.empty(0), 0.0, 0)
    w = World("observe", stop_now=True, plans=[invalid])
    w.ms.last_tracking = OBSERVE_MISS
    if on:
        w.core.options.observe_start_backoff = "on"
    w.core._observe_backoff_start = lambda t, rear, target: w.calls.append(("backoff_start", (target,), {})) or start_ok
    return w, OBSERVE_MISS


def test_off_an_invalid_start_ends_the_mission_as_before():
    w, miss = _miss_world(True, on=False)
    with pytest.raises(RuntimeError, match="observe_replan_failed:invalid_start"):
        w.tick(miss)
    assert "backoff_start" not in w.names()


def test_on_an_installed_backoff_keeps_the_mission_and_installs_no_plan():
    w, miss = _miss_world(True)
    before = w.core.paths["observe"]
    out = w.tick(miss)
    assert "backoff_start" in w.names() and "write_paths" not in w.names() and w.core.paths["observe"] is before
    assert out.tracking.status == "tracking"
    (call,) = [c for c in w.calls if c[0] == "backoff_start"]
    assert (call[1][0].x_m, call[1][0].y_m) == (1.0, 0.0)  # the leg's own target


def test_on_a_refused_backoff_ends_the_mission_as_before():
    w, miss = _miss_world(False)
    with pytest.raises(RuntimeError, match="observe_replan_failed:invalid_start"):
        w.tick(miss)


def test_a_resume_during_the_backoff_never_runs_the_common_replan():
    # Codex plan re-review P2: installed, paused before the first follow, resumed with the way occupied
    from test_core_pause import Layer, make as make_paused, pause, resume

    core, _, _ = make_paused("observe", layer=Layer(blocked="occupied"))
    core.observe_backoff = {"serial": 0}
    core.blocked_replan = lambda *a: pytest.fail("the common recovery ran during a start backoff")
    pause(core, 10.0)
    resume(core, 11.0)
    core.paused_tick(11.0, np.array([0.0, 0.0, 0.0]), 0.0, True)
    assert core.commands.status == "paused" and core.commands.resume_check["reason"] == "no_resume:path_blocked:occupied"
    core2, _, _ = make_paused("observe")
    core2.observe_backoff = {"serial": 0}
    pause(core2, 10.0)
    resume(core2, 11.0)
    core2.paused_tick(11.0, np.array([0.0, 0.0, 0.0]), 0.0, True)
    assert core2.commands.status == "running" and core2.observe_backoff == {"serial": 0}  # the same backoff goes on


def test_the_runner_runs_the_backoff_tick_instead_of_the_stall_and_observe_recoveries():
    from pathlib import Path

    src = Path(__file__).resolve().parents[3].joinpath("sim/isaac/run_transport.py").read_text()
    i = src.index("core.observe_backoff_tick(t, rear, tracking, signed_speed, dt)")
    j = src.index("stall_ = core.docking_and_stall(", i)
    k = src.index("recover_ = core.cusp_and_observe_recovery(", j)
    assert "elif not scene_raised_:" in src[i:j] and i < j < k
    assert src.index("command_ = core.command(", k) > k
