"""C2: the near re-observation leg's end stand (docs/plans/2026-11-02-near-leg-end-stand-c2.md).

O1's rule on the near leg: a final-goal stand (remaining 0, tracking, not at a cusp, a zero command, standing) outside
the loose arrival bounds for 3 s is planned again to the same target under the final-goal SLAM hold, once per near leg
serial and within its replan budget; an installed replan restarts the no-progress watch; a failed attempt or a spent
budget waits. ``shadow`` records the judgement only. Off is every earlier run."""

from dataclasses import replace

import pytest
import test_core_cusp_observe as co

DT = 1 / 120
# S1 st_on_random_1_s5's stand (control frame): 52.8 mm, 0.112 rad from the near waypoint
STAND = co.Tracking(status="tracking", speed_mps=0.0, position_error_m=0.0528, yaw_error_rad=-0.112)
WAYPOINT = [-3.05, 0.56, 0.18]


def n_world(mode="on", *, plans=(), observe_replans=0, remaining=0.0, holds_observe=False, stop_now=True,
            near_capture=None, far=False, realign=False, bottom_deck=False, observe_backoff=None, obstacle_backoff=None):
    w = co.World("observe", plans=plans, observe_replans=observe_replans, holds_observe=holds_observe,
                 stop_now=stop_now, slam_mode="holding", realign=realign)
    w.tracker.remaining_to_goal_m = lambda: remaining
    w.core.options.observe_arrival_m, w.core.options.observe_arrival_yaw_rad = 0.10, 0.10
    w.core.options.near_end_stand = mode
    w.core.scene_far_observe = lambda: far
    w.core.bottom_deck_leg_active = lambda: bottom_deck
    w.core.observe_backoff = observe_backoff
    if obstacle_backoff is not None:
        w.obstacle["backoff"] = {"phase": obstacle_backoff}
    w.state["near_capture"] = ({"status": "pending", "waypoint": WAYPOINT} if near_capture is None else near_capture)
    return w


def stand_for(w, seconds, tracking=STAND, t0=40.0):
    out = None
    for k in range(int(round(seconds / DT))):
        out = w.tick(tracking, t=t0 + k * DT)
    return out


def near_records(w):
    return [r for r in w.state.get("observe_end_stands", ()) if r.get("kind") == "near"]


def test_a_near_leg_stand_outside_the_loose_bounds_for_3_s_is_planned_again_under_the_hold():
    w = n_world(plans=[co.plan()])
    stand_for(w, 2.99)
    assert "plan_observation_leg" not in w.names()
    w.tick(STAND, t=40.0 + 3.0)
    assert w.names()[:4] == ["plan_observation_leg", "write_paths", "show_path", "realign_install"]
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_observation_leg"][0]
    assert (args[1].x_m, args[1].y_m) == (1.0, 0.0) and kwargs["extended"] is False  # the same target
    (rec,) = near_records(w)
    assert rec["outcome"] == "installed" and rec["leg_serial"] == 7 and rec["slam_mode"] == "holding"
    assert w.state["observe_replans"][-1]["trigger"] == "near_end_stand"
    assert w.core.trackers["observe"] is not w.tracker and w.ms.phase_started == 43.0
    assert "progress" not in w.obstacle  # the no-progress watch starts over on the new leg
    assert w.slam["tracker"].mode == "holding"  # the hold is kept
    assert "near_end_stand" not in w.state


def test_off_is_every_earlier_run():
    w = n_world("off")
    stand_for(w, 6.0)
    assert w.names() == [] and "observe_end_stands" not in w.state and "near_end_stand" not in w.state


def test_shadow_records_the_judgement_and_changes_nothing():
    w = n_world("shadow")
    stand_for(w, 6.0)
    (rec,) = near_records(w)
    assert rec["outcome"] == "shadow" and rec["zero_command_s"] >= 3.0
    assert w.names() == [] and w.state["observe_replans"] == [] and w.core.trackers["observe"] is w.tracker
    assert w.obstacle["progress"] == {"x": 1}


def test_shadow_with_the_budget_spent_is_told_apart():
    w = n_world("shadow", observe_replans=2)
    stand_for(w, 4.0)
    assert [r["outcome"] for r in near_records(w)] == ["shadow_no_budget"]


def test_a_spent_budget_waits():
    w = n_world(observe_replans=2)
    out = stand_for(w, 4.0)
    assert [r["outcome"] for r in near_records(w)] == ["no_budget"] and "plan_observation_leg" not in w.names()
    assert out.tracking == STAND and w.obstacle["progress"] == {"x": 1}


def test_a_failed_replan_waits_spends_nothing_and_is_tried_once_per_leg():
    w = n_world(plans=[co.plan(False)])
    out = stand_for(w, 3.0 + DT)
    assert [r["outcome"] for r in near_records(w)] == ["failed:no_path"]
    assert w.state["observe_replans"] == [] and w.core.trackers["observe"] is w.tracker and out.tracking == STAND
    assert w.ms.phase_started == 0.0 and w.obstacle["progress"] == {"x": 1}
    stand_for(w, 5.0, t0=45.0)
    assert [c[0] for c in w.calls].count("plan_observation_leg") == 1


def test_a_new_near_leg_serial_may_try_again():
    w = n_world(plans=[co.plan(False), co.plan()])
    stand_for(w, 3.0 + DT)
    w.state["observe_replan_leg"] = {"serial": 8, "start": 0}
    stand_for(w, 3.0 + DT, t0=50.0)
    assert [r["outcome"] for r in near_records(w)] == ["failed:no_path", "installed"]


def test_inside_the_loose_bounds_is_left_to_the_loose_arrival():
    w = n_world()
    stand_for(w, 6.0, replace(STAND, position_error_m=0.09, yaw_error_rad=0.09))
    assert w.names() == [] and near_records(w) == []


@pytest.mark.parametrize("change", [
    {"far": True}, {"realign": True}, {"bottom_deck": True}, {"observe_backoff": {"serial": 7}},
    {"obstacle_backoff": "observe"}, {"near_capture": {"status": "done", "waypoint": WAYPOINT}},
    {"near_capture": {"status": "failed", "waypoint": WAYPOINT}}, {"near_capture": {"status": "pending"}},
    {"near_capture": {}}, {"remaining": 0.01}, {"holds_observe": True},
])
def test_outside_the_near_leg_or_the_stand_nothing_is_judged(change):
    far = change.get("far", False)
    # with far set, O1 owns the stand; its spent budget keeps it from planning (an O1 record, never a near one)
    w = n_world(observe_replans=2 if far else 0, **change)
    stand_for(w, 6.0)
    assert near_records(w) == [] and "plan_observation_leg" not in w.names()


@pytest.mark.parametrize("tracking", [replace(STAND, at_cusp=True), replace(STAND, speed_mps=0.01)])
def test_a_cusp_or_a_commanded_tick_is_not_a_stand(tracking):
    w = n_world(plans=[co.plan()] * 4)  # a cusp stand is the observe miss's own (it may plan); never C2's
    stand_for(w, 6.0, tracking)
    assert near_records(w) == []


def test_o1_clearing_its_own_clock_does_not_restart_c2s():
    w = n_world(plans=[co.plan()])
    stand_for(w, 3.0 + DT)  # O1 is not the leg: it pops observe_end_stand on every tick; C2 still judges at 3 s
    assert [r["outcome"] for r in near_records(w)] == ["installed"]


def test_the_stop_detectors_flicker_does_not_restart_the_count():
    w = n_world(plans=[co.plan()])
    t, k = 40.0, 0
    while t < 43.6 and not near_records(w):
        w.slam["stop_now"] = (k // 48) % 2 == 0
        w.tick(STAND, t=t)
        t, k = t + DT, k + 1
    (rec,) = near_records(w)
    assert rec["outcome"] == "installed" and w.slam["stop_now"]


def test_the_k3_release_judgement_is_untouched():
    # R1: C2 changes neither cusp_end_recovered nor the K3 convergence it feeds
    import inspect

    from forklift_core.control import cusp_stall
    from forklift_core.mission.core import MissionCore

    assert "near_end_stand" not in inspect.getsource(MissionCore.cusp_end_recovered)
    assert "near_end_stand" not in inspect.getsource(cusp_stall)


def test_the_pending_stand_is_a_missions_own_state():
    from forklift_core.mission import baseline

    assert "near_end_stand" in baseline.MISSION_KEYS["state"]
    assert "observe_end_stands" in baseline.MISSION_KEYS["state"]  # C2's records join O1's


# --- C2 5.1: the shadow episode, tied to its very stand ------------------------------------------------------------


def shadowed():
    w = n_world("shadow")
    stand_for(w, 3.0 + DT)
    (rec,) = near_records(w)
    return w, rec


def test_a_shadow_record_carries_what_ties_it_to_its_stand():
    w, rec = shadowed()
    assert rec["outcome"] == "shadow" and rec["phase_started_s"] == 0.0 and rec["stand_since_s"] == 40.0
    assert rec["goal"] == [1.0, 0.0, 0.0] and len(rec["path_sha256"]) == 64 and rec["loose_arrivals_n"] == 0
    assert w.state["near_end_shadow_open"] == {"index": 0, "leg_serial": 7, "phase_started_s": 0.0,
                                               "observe_replans_n": 0}
    assert "end" not in rec


def test_a_loose_arrival_of_the_same_stand_ends_the_episode_as_such():
    w, rec = shadowed()
    stand_for(w, 1.0, t0=43.1)  # still the same stand: open
    assert "end" not in rec
    w.core.near_end_shadow_arrival(44.2)
    assert rec["end"] == {"kind": "loose_arrival", "time_s": 44.2} and "near_end_shadow_open" not in w.state


def test_a_resumed_command_ends_the_episode_before_any_arrival():
    w, rec = shadowed()
    w.tick(replace(STAND, speed_mps=0.01), t=43.05)  # one tick commanded: no longer the judged stand
    assert rec["end"] == {"kind": "command_resumed", "time_s": 43.05}
    w.core.near_end_shadow_arrival(44.2)  # a later arrival is not this stand's
    assert rec["end"]["kind"] == "command_resumed"


def test_another_tracker_ends_the_episode():
    w, rec = shadowed()
    w.ms.phase_started = 43.5  # a replan installed another tracker
    w.tick(STAND, t=43.6)
    assert rec["end"] == {"kind": "tracker_changed", "time_s": 43.5}


def test_the_loose_arrival_rule_reports_to_the_shadow_episode():
    import inspect

    from forklift_core.mission.core import MissionCore

    src = inspect.getsource(MissionCore.command)
    at = src.index('state.setdefault("observe_loose_arrivals", []).append(')
    assert "self.near_end_shadow_arrival(t)" in src[at:at + 400]


# --- the installed replan's consumers ---------------------------------------------------------------------------------


def test_the_installed_replan_restarts_the_no_progress_watch_in_the_permission():
    import test_core_permission as cp

    core, obstacle, _, _ = cp.make("observe", remaining=1.0, act=True)
    obstacle["progress"] = {"phase": "observe", "best_m": 0.0, "since_s": 0.0}  # the stand's watch: best 0
    obstacle.pop("progress")  # what C2's install does
    core.permission(53.0, cp.REAR, 0.2, 0.4, True, None)
    assert obstacle["progress"] == {"phase": "observe", "best_m": 1.0, "since_s": 53.0}
    core.permission(60.1, cp.REAR, 0.2, 0.4, True, None)  # 7.1 s on the new leg: no obstacle_no_progress
    core2, obstacle2, _, _ = cp.make("observe", remaining=1.0, act=True)
    obstacle2["progress"] = {"phase": "observe", "best_m": 0.0, "since_s": 0.0}  # kept (a failed attempt)
    with pytest.raises(RuntimeError, match="obstacle_no_progress in observe"):
        core2.permission(60.1, cp.REAR, 0.2, 0.4, True, None)


def test_the_replan_starts_from_the_held_control_pose_and_the_grid_of_that_tick():
    w = n_world(plans=[co.plan()])
    w.runner_rear = co.REAR  # the runner's control pose under the final-goal hold
    stand_for(w, 3.0 + DT)
    (_, args, kwargs), = [c for c in w.calls if c[0] == "plan_observation_leg"]
    start = kwargs["start_rear"]
    assert (start.x_m, start.y_m, start.yaw_rad) == tuple(float(v) for v in co.REAR)
    (grid,) = [c for c in w.calls if c[0] == "grid_kwargs"]
    assert grid[1] == ("observe",) and grid[2]["own"] == tuple(co.REAR)  # the same pose's frame, the same tick
    assert w.slam["tracker"].mode == "holding"


@pytest.mark.parametrize("accepted", [True, False])
def test_after_an_install_the_near_capture_flow_accepts_or_refuses_as_before(accepted):
    import test_core_near_capture as nc

    # refused: without near tracking a refused straight ends the run (with it, the S4c-3 realignment takes it)
    w = nc.World(near={"tracker": None}) if accepted else nc.World(dry=nc.Dry(status="failed"))
    # the state C2 leaves: the near leg's waypoint, an installed replan in the leg's budget, another observe path
    w.state["near_capture"]["waypoint"] = [2.5, 2.0, 0.0]
    w.state["observe_replans"].append({"trigger": "near_end_stand", "leg_serial": 2})
    w.core.paths["observe"] = nc.leg(1.2, 1.3)
    if accepted:
        w.run()
        assert w.state["near_capture"]["status"] == "done" and w.ms.phase == "approach"
        assert w.calls[-1][0] == "build_near_tracker"
    else:
        with pytest.raises(RuntimeError, match="near_capture_misaligned:dry_run:"):
            w.run()
        assert w.state["near_capture"]["status"] == "pending" and w.ms.phase == "observe"
        assert "build_near_tracker" not in w.names()


def test_two_missions_each_judge_their_own_near_leg():
    from forklift_core.mission.baseline import MissionBaseline

    w = n_world("shadow")
    w.ms.last_tracking = None  # as at a mission's start (the baseline takes plain data only)
    base = MissionBaseline.capture(w.core)
    stand_for(w, 3.0 + DT)
    assert len(near_records(w)) == 1 and "near_end_shadow_open" in w.state
    records = base.restore(w.core)
    assert records["state"]["observe_end_stands"][0]["kind"] == "near"
    assert "near_end_shadow_open" not in w.state and "observe_end_stands" not in w.state
    w.state["near_capture"] = {"status": "pending", "waypoint": WAYPOINT}  # the next mission's near leg
    w.state["observe_replan_leg"] = {"serial": 7, "start": 0}  # even the same serial: a new mission's records
    stand_for(w, 3.0 + DT, t0=100.0)
    assert [r["outcome"] for r in near_records(w)] == ["shadow"]
