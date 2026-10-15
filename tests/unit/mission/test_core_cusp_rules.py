"""M-a1 slice 6a: the cusp position stall's rules, moved from run() closures into MissionCore methods
(behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import RearAxlePathTracker, TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState


class Check:
    def __init__(self, blocked=None, verified_m=5.0, oldest_free_s=9.5, first_unknown_m=None):
        self.blocked, self.verified_m, self.oldest_free_s, self.first_unknown_m = (
            blocked, verified_m, oldest_free_s, first_unknown_m)


def make(phase="transport", *, slam=None, obstacle=None, profile="default", state=None, standing=True, calls=None):
    calls = [] if calls is None else calls

    def require(ok, reason):
        calls.append(("require", bool(ok), reason))
        if not ok:
            raise RuntimeError(reason)

    tracker = RearAxlePathTracker(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), np.ones(3),
                                  np.zeros(3), TrackerConfig(cruise_speed_mps=0.4))
    hooks = SimpleNamespace(
        require=require, realign_active=lambda: False, cusp_standing=lambda: standing, loaded=lambda: True,
        signed_speed=lambda: -0.05, held_kappa=lambda: 0.2, escape_leg=lambda phase_: (None, None, "RECTS"),
        grid_world=lambda sc: sc, dump_tracking=lambda reason, tr: calls.append(("dump", reason)),
        cusp_monitor=lambda: None,
    )
    core = MissionCore(MissionState(phase=phase, phase_started=0.0),
                       state={"transitions": [], "cusp_replans": [], "stall_replans": [], **(state or {})},
                       obstacle=obstacle, slam=slam, pocket={"check": None}, paths={}, trackers={phase: tracker},
                       geometry=None, settings={}, pallet_geometry=None, hooks=hooks,
                       options=SimpleNamespace(tracker_profile=profile, max_cusp_replans=2, dt=1 / 120,
                                               grid_planning=True, pickup_zone="ZONE", pickup_bounds="B",
                                               scenario="SC"))
    # Helpers moved into MissionCore (slices 6b/6c): this test's stand-ins go on the instance.
    for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry', 'release_and_stand', 'release_after_backoff', 'slam_release', 'escape_leg', 'find_escape', 'hold_count', 'near_tick', 'near_redraw', 'near_capture_redraw', 'near_protection_prediction', 'near_install', 'start_pickup_retry']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    return core, tracker, calls


def test_other_rules_own_a_cusp_stop_in_the_legacy_profile_the_d7_transport_and_a_held_realignment():
    assert make(profile="20260921")[0].cusp_excluded("observe") == "tracker_profile"
    slam = {"docking": {"accepted_any": True}, "tracker": SimpleNamespace(mode="tracking")}
    core, _, _ = make(slam=slam)
    assert core.cusp_excluded("transport") == "d7_transport" and core.cusp_excluded("return_home") is None
    slam = {"docking": {}, "tracker": SimpleNamespace(mode="holding")}
    core, _, _ = make("observe", slam=slam)
    core.hooks.realign_active = lambda: True
    assert core.cusp_excluded("observe") == "realign_held"
    assert make()[0].cusp_excluded("transport") is None


def test_stopped_is_standing_with_a_zero_command_applied():
    core, _, _ = make(state={"applied_command": (0.0, [0, 0])})
    assert core.cusp_stopped() is True
    core, _, _ = make(state={"applied_command": (0.1, [0, 0])})
    assert core.cusp_stopped() is False
    core, _, _ = make(standing=False, state={"applied_command": (0.0, [0, 0])})
    assert core.cusp_stopped() is False


def test_the_heading_records_and_the_offset_from_the_path():
    core, tracker, _ = make()
    tr = SimpleNamespace(status="tracking", failure=None, position_error_m=0.01, yaw_error_rad=0.02)
    assert core.cusp_heading_end(tr) == {"status": "tracking", "failure": None, "position_error_m": 0.01,
                                         "yaw_error_rad": 0.02}
    lateral, heading = core.cusp_heading_offset(tracker, 0.5, (0.5, 0.03, 0.1))
    assert lateral == pytest.approx(0.03) and heading == pytest.approx(0.1)


def layer(**kw):
    return SimpleNamespace(
        permission=SimpleNamespace(config=SimpleNamespace(sensor_timeout_s=0.5, step_m=0.05, evidence_max_age_s=2.0,
                                                          stopping=SimpleNamespace(distance_m=lambda v: 0.5))),
        last_scan_s=kw.pop("scans", {"front": 9.8}), snapshot=kw.pop("snapshot", "SNAP"), **kw)


def test_sensors_alive_and_the_scan_a_decision_belongs_to():
    core, _, _ = make()
    assert core.cusp_alive(10.0) is True and core.cusp_scan_id(1.234) == 12
    core, _, _ = make(obstacle={"layer": layer(scans={"front": 9.8, "rear": 9.0}), "last_stamp": 9.8})
    assert core.cusp_alive(10.0) is False and core.cusp_scan_id(10.0) == 9.8


def test_the_obstacle_side_leaves_a_stop_alone_only_with_its_path_and_arc_free_and_sensors_alive():
    asked = []
    lay = layer(check_candidate=lambda leg, **k: asked.append(("path", k)) or Check(),
                check_arc_candidate=lambda **k: asked.append(("arc", k)) or Check())
    core, tracker, _ = make(obstacle={"layer": lay})
    ok, record = core.cusp_ownership(10.0, (0.5, 0.0, 0.0), 0.3)
    assert ok is True and record["kappa_inv_m"] == 0.2 and record["silent"] == []
    assert asked[0][1] == {"current_pose": (0.5, 0.0, 0.0), "direction": 1, "loaded": True}
    assert asked[1][1] == {"current_pose": (0.5, 0.0, 0.0), "curvature_inv_m": 0.2, "direction": 1, "loaded": True,
                           "cap_mps": 0.3}
    lay.check_arc_candidate = lambda **k: Check(blocked="occupied")
    assert core.cusp_ownership(10.0, (0.5, 0.0, 0.0), 0.0)[0] is False
    lay.snapshot = None
    assert core.cusp_ownership(10.0, (0.5, 0.0, 0.0), 0.0) == (False, {"time_s": 10.0, "why": "no_snapshot"})
    assert make()[0].cusp_ownership(10.0, (0, 0, 0), 0.0) == (True, None)


def test_the_next_legs_sensor_check():
    answers = {False: Check(blocked="unknown", verified_m=0.3, oldest_free_s=9.0), True: Check(first_unknown_m=0.3)}
    lay = layer(check_candidate=lambda traj, unknown_passes=False, **k: answers[unknown_passes])
    core, _, _ = make(obstacle={"layer": lay})
    ok, occupied, record = core.cusp_sensors(10.0, (0, 0, 0), "TRAJ", 1)
    assert (ok, occupied) == (False, False) and record["stop_need_m"] == pytest.approx(0.55)  # unknown within it
    answers[False] = Check(blocked="unknown", verified_m=0.6, oldest_free_s=9.0)
    assert core.cusp_sensors(10.0, (0, 0, 0), "TRAJ", 1)[0] is True
    answers[True] = Check(blocked="occupied")
    assert core.cusp_sensors(10.0, (0, 0, 0), "TRAJ", 1)[:2] == (False, True)


def test_which_short_leg_ends_an_existing_rule_recovers():
    core, _, _ = make("transport", slam={"docking": {}})
    assert core.cusp_end_recovered(False) is True and core.cusp_end_recovered(True) is True
    core.state["cusp_replans"] = [{}, {}]
    core.state["stall_replans"] = [{}, {}]
    assert core.cusp_end_recovered(False) is False and core.cusp_end_recovered(True) is False
    core, _, _ = make("return_home")
    assert core.cusp_end_recovered(True) is False  # without SLAM no stall replan
    core, _, _ = make("observe", slam={"docking": {}})
    assert core.cusp_end_recovered(True) is False


def test_the_leg_checker_takes_the_travel_legs_rectangles():
    core, _, _ = make("return_home")
    assert core.cusp_leg_checker() == "RECTS"


def test_the_tick_forces_the_obstacle_replan_on_a_hand_off_and_ends_the_run_on_an_unrecoverable_stall():
    seen = {}

    def tick(now_s, phase, tracker, raw, tracking, **k):
        seen.update(k, now_s=now_s, phase=phase)
        return SimpleNamespace(force_replan=True, end=None, tracking="T", hold=True)

    monitor = SimpleNamespace(tick=tick, latch=None)
    obstacle = {"layer": layer(check_candidate=lambda *a, **k: Check()), "progress": {"since_s": 3.0}}
    core, tracker, calls = make(obstacle=obstacle)
    core.hooks.cusp_monitor = lambda: monitor
    assert core.cusp_tick(10.0, (0.5, 0, 0), "RAW", "TR", False) == ("T", True)
    assert obstacle["force_replan"] == "transport" and seen["watch_since_s"] == 3.0
    assert seen["excluded"] is None and seen["retry_stand"] is False and seen["scan_id"]() is None
    monitor.tick = lambda *a, **k: SimpleNamespace(force_replan=False, end="cusp_stall_unrecoverable", tracking="T",
                                                   hold=False)
    with pytest.raises(RuntimeError, match="cusp_stall_unrecoverable"):
        core.cusp_tick(10.0, (0.5, 0, 0), "RAW", "TR", False)
    assert ("dump", "cusp_stall_unrecoverable") in calls


def test_a_released_leg_is_updated_from_the_tick_pose_at_the_control_speed():
    updates = []
    monitor = SimpleNamespace(latch=None, tick=lambda *a, **k: SimpleNamespace(
        force_replan=False, end=None, tracking=k["release_update"](), hold=False))
    core, _, _ = make()
    core.trackers["transport"] = SimpleNamespace(update=lambda rear, v, dt: updates.append((rear, v, dt)) or "UPD")
    core.hooks.cusp_monitor = lambda: monitor
    assert core.cusp_tick(10.0, (0.5, 0.1, 0.2), "RAW", "TR", False) == ("UPD", False)
    assert updates == [((0.5, 0.1, 0.2), -0.05, 1 / 120)]  # signed_speed and dt from the runner
