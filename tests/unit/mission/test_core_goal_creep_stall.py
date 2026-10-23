"""The final goal's creep stall handed to the existing receivers (docs/plans/2026-10-24-goal-creep-stall.md G2).

R1: 0 < remaining <= 1 um, the leg end filter true, the pose at the progress point, the tracker's output and the last
applied wheel speed at the creep's level -- the command goes to zero (status kept), so the stop detector can see the
stop and the existing "remaining 0" rules take it. R2: 1 um < remaining <= 4 mm, the filter true and a nonzero applied
wheel speed: over a window of the last 0.8 mm of commanded travel and at least 120 ticks, odometry travel under a
quarter of it latches; the command goes to zero (braking) and the stall rules take it as they take a stop at a path
end -- the D7c docking stop when the transport's docking is armed and holding, else the docking retry or the stall
replan. No applied speed recorded (an older record) or a tracker without the views: nothing changes."""

from types import SimpleNamespace

import numpy as np

from test_core_docking_stall import Tracking, World, plan

DT = 1 / 120


def creep_world(phase="return_home", *, remaining=2.4e-5, out=True, normal=4e-4, applied=0.01, odom=0.0,
                mode="tracking", **kw):
    w = World(phase, mode=mode, remaining=remaining, **kw)
    view = SimpleNamespace(out=out, normal=normal)
    w.view = view
    w.tracker.leg_end_out_of_reach = lambda: view.out
    w.tracker.distance_to_end_normal_m = lambda: view.normal
    w.tracker.leg_index = 0
    w.slam["applied_speed"] = applied
    w.slam["odom_speed"] = odom
    return w


def run_ticks(w, n, tracking, t0=10.0):
    out = None
    for k in range(n):
        out = w.tick(tracking, t=t0 + k * DT)
    return out


CREEP = Tracking(status="tracking", speed_mps=-3.6e-5)


def test_r2_latches_once_the_window_holds_120_ticks_and_0_8_mm_of_command_without_odometry():
    w = creep_world(stop_now=False)
    out = run_ticks(w, 119, CREEP)
    assert out.tracking.speed_mps == CREEP.speed_mps  # 119 ticks: not yet
    out = w.tick(CREEP, t=10.0 + 119 * DT)
    assert out.tracking.status == "braking" and out.tracking.speed_mps == 0.0
    assert w.core.goal_creep_watch["latched"]


def test_r2_does_not_latch_while_the_odometry_follows_the_command():
    w = creep_world(odom=0.01, stop_now=False)
    out = run_ticks(w, 400, CREEP)
    assert out.tracking.speed_mps == CREEP.speed_mps and not w.core.goal_creep_watch["latched"]


def test_r2_needs_0_8_mm_of_command_in_the_window():
    w = creep_world(applied=3.6e-5, stop_now=False)  # on_s7's creep: 0.8 mm takes 22.2 s
    run_ticks(w, int(22.0 / DT), CREEP)
    assert not w.core.goal_creep_watch["latched"]
    run_ticks(w, int(0.5 / DT), CREEP, t0=10.0 + 22.0)
    assert w.core.goal_creep_watch["latched"]


def test_a_zero_applied_speed_or_a_skipped_tick_empties_the_window():
    w = creep_world(stop_now=False)
    run_ticks(w, 100, CREEP)
    w.slam["applied_speed"] = 0.0  # the permission held the wheels: not the creep's time
    w.tick(CREEP, t=10.0 + 100 * DT)
    w.slam["applied_speed"] = 0.01
    run_ticks(w, 119, CREEP, t0=10.0 + 101 * DT)
    assert not w.core.goal_creep_watch["latched"]
    run_ticks(w, 50, CREEP, t0=20.0)  # a pause: the ticks resume 8 s later
    assert not w.core.goal_creep_watch["latched"]


def test_after_the_latch_the_stall_replan_takes_it_standing():
    w = creep_world(plans=[plan()], stop_now=True)
    run_ticks(w, 120, CREEP)
    assert w.core.goal_creep_watch["latched"]
    w.slam["applied_speed"] = 0.0  # its own zero: the latch holds
    run_ticks(w, 119, CREEP, t0=12.0)
    assert "plan_return_leg" in w.names()
    assert w.state["stall_replans"][-1]["phase"] == "return_home"


def test_the_latch_ends_when_the_filter_turns_false_or_the_tracker_changes():
    w = creep_world(stop_now=False)
    run_ticks(w, 120, CREEP)
    assert w.core.goal_creep_watch["latched"]
    w.view.out = False
    out = w.tick(CREEP, t=12.0)
    assert not w.core.goal_creep_watch["latched"] and out.tracking.speed_mps == CREEP.speed_mps


def test_an_armed_docking_stop_takes_it_as_the_d7c_stop():
    w = creep_world("transport", docking={"status": "armed", "round": 1}, mode="holding", stop_now=True)
    run_ticks(w, 120, CREEP)
    run_ticks(w, 120, CREEP, t0=12.0)
    assert "dock_at_delivery_straight" in w.names() and "plan_transport_leg" not in w.names()


def test_an_exhausted_stall_budget_leaves_r2_as_it_was():
    w = creep_world(stall_replans=2, stop_now=True)
    out = run_ticks(w, 300, CREEP)
    assert out.tracking.speed_mps == CREEP.speed_mps and w.names() == []


def test_r1_zeroes_a_creep_at_the_progress_point_and_the_remaining_zero_rule_takes_it():
    rem = 5e-7
    creep = Tracking(status="tracking", speed_mps=1.5 * rem)
    w = creep_world(remaining=rem, normal=0.0, applied=1.5 * rem, plans=[plan()], stop_now=True)
    out = w.tick(creep)
    assert out.tracking.status == "tracking" and out.tracking.speed_mps == 0.0
    run_ticks(w, 120, creep, t0=11.0)
    assert "plan_return_leg" in w.names()


def test_r1_leaves_an_approach_behind_the_progress_or_still_braking():
    rem = 5e-7
    creep = Tracking(status="tracking", speed_mps=1.5 * rem)
    for kw, tracking in (({"normal": 1e-3}, creep),  # the pose behind the progress (Codex round 5)
                         ({"normal": 0.0}, Tracking(status="tracking", speed_mps=0.05)),  # still braking (round 6)
                         ({"normal": 0.0, "applied": 0.05}, creep)):  # the wheels still driven
        w = creep_world(remaining=rem, **{"applied": 1.5 * rem, **kw}, stop_now=False)
        out = w.tick(tracking)
        assert out.tracking.speed_mps == tracking.speed_mps


def test_nothing_without_an_applied_speed_or_the_tracker_views():
    w = creep_world(stop_now=False)
    del w.slam["applied_speed"]  # an older record
    out = run_ticks(w, 300, CREEP)
    assert out.tracking.speed_mps == CREEP.speed_mps
    w = World("return_home", mode="tracking", remaining=2.4e-5, stop_now=False)  # the existing tests' tracker
    w.slam["applied_speed"], w.slam["odom_speed"] = 0.01, 0.0
    out = run_ticks(w, 300, CREEP)
    assert out.tracking.speed_mps == CREEP.speed_mps


def test_a_mission_restore_resets_the_watch_and_hands_its_latches_over():
    # Codex implementation round 1 P2: the watch is the core's (CORE_FIELDS), its latches the mission's state
    # (MISSION_KEYS) -- a live session's next mission starts with neither.
    from forklift_core.mission.baseline import MissionBaseline

    w = creep_world(stop_now=False)
    base = MissionBaseline.capture(w.core)
    run_ticks(w, 120, CREEP)
    assert w.core.goal_creep_watch["latched"] and len(w.state["goal_creep_stalls"]) == 1
    records = base.restore(w.core)
    assert records["state"]["goal_creep_stalls"][0]["phase"] == "return_home"
    assert "goal_creep_stalls" not in w.state
    assert w.core.goal_creep_watch == {"key": None, "last_t": None, "window": [], "latched": False}
