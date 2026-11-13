"""K1: the near re-observation leg's loose arrival narrowed together with C2's outside judgement
(docs/plans/2026-11-13-k1-near-loose-arrival.md 2.1-2.3).

The narrow bounds apply only on the near leg with ``--near-observe-arrival``; they give way to the common 0.10 m /
0.10 rad once C2 was tried on the leg serial, its replan budget is spent or less than 5 s of its time limit remain. An
intervention (a blocked arrival or an extra C2) is recorded once per stand, before C2 records or installs, and closed
at the arrival (with the bounds source and whether the error was inside the narrow bounds) or when the stand ends.
Without the option nothing changes and nothing is written."""

import copy
import json
from dataclasses import replace

import test_core_near_end_stand as ne
from test_core_cusp_observe import Tracking, plan

from forklift_core.mission.baseline import MISSION_KEYS
from forklift_core.mission.core import NEAR_LOOSE_DEADLINE_MARGIN_S

DT = ne.DT
# inside the common bounds, outside the narrow ones: a stand B0 accepts as a loose arrival
MID = Tracking(status="tracking", speed_mps=0.0, position_error_m=0.06, yaw_error_rad=0.07)
NARROW = (0.05, 0.05)


def k1_world(narrow=NARROW, *, limit=60.0, **kw):
    """A C2 world on the near leg; the follower's time limit is ``limit`` s (tracking_limit)."""
    w = ne.n_world(**kw)
    if narrow is not None:
        w.core.options.near_observe_arrival = narrow
    w.core.tracking_limit = lambda t: limit
    w.core.commands.mission_id = "m1"
    return w


def run_c2(w, seconds, tracking=MID, t0=40.0):
    return ne.stand_for(w, seconds, tracking, t0)


def snapshot(w):
    return json.dumps(w.state, sort_keys=True, default=str)


# ---- bounds ------------------------------------------------------------------------------------------------------


def test_without_the_option_the_bounds_are_the_common_ones_and_nothing_is_written():
    w = k1_world(None)
    before = snapshot(w)
    assert w.core.near_loose_bounds(40.0) == (0.10, 0.10, "common")
    assert snapshot(w) == before


def test_on_the_near_leg_the_narrow_bounds_apply():
    w = k1_world()
    assert w.core.near_loose_bounds(40.0) == (0.05, 0.05, "narrow")


def test_other_legs_keep_the_common_bounds():
    for change in ({"far": True}, {"realign": True}, {"bottom_deck": True}, {"observe_backoff": {"serial": 7}},
                   {"obstacle_backoff": "observe"}, {"near_capture": {"status": "done", "waypoint": ne.WAYPOINT}}):
        w = k1_world(**change)
        assert w.core.near_loose_bounds(40.0)[2] == "common", change
    w = k1_world()
    w.ms.phase = "approach"
    assert w.core.near_loose_bounds(40.0)[2] == "common"


def test_the_bounds_fall_back_after_c2_was_tried_whatever_its_outcome():
    for plans in ([plan()], [plan(False)]):
        w = k1_world(plans=plans)
        run_c2(w, 3.0 + DT)
        assert ne.near_records(w), plans
        assert w.core.near_loose_bounds(45.0)[2] == "fallback"
        assert w.state["near_loose_fallbacks"] == [{"time_s": 45.0, "leg_serial": 7, "reason": "c2_tried"}]
        w.core.near_loose_bounds(46.0)  # recorded once per leg and reason
        assert len(w.state["near_loose_fallbacks"]) == 1
    w = k1_world(observe_replans=2)  # a spent budget's no_budget record is a try too
    run_c2(w, 4.0)
    assert w.core.near_loose_bounds(45.0)[2] == "fallback"


def test_the_bounds_fall_back_with_the_budget_spent():
    w = k1_world(observe_replans=2)
    assert w.core.near_loose_bounds(40.0)[2] == "fallback"
    assert w.state["near_loose_fallbacks"][0]["reason"] == "budget_spent"


def test_the_bounds_fall_back_near_the_legs_time_limit():
    w = k1_world(limit=30.0)
    w.ms.phase_started = 0.0
    w.core.commands.work_elapsed = lambda since, t: t - since
    assert w.core.near_loose_bounds(30.0 - NEAR_LOOSE_DEADLINE_MARGIN_S - 0.01)[2] == "narrow"
    assert w.core.near_loose_bounds(30.0 - NEAR_LOOSE_DEADLINE_MARGIN_S + 0.01)[2] == "fallback"
    assert w.state["near_loose_fallbacks"][0]["reason"] == "deadline"
    assert NEAR_LOOSE_DEADLINE_MARGIN_S == 5.0


# ---- C2 with the narrow bounds -----------------------------------------------------------------------------------


def test_c2_takes_a_stand_inside_the_common_bounds_and_records_the_extra_c2_first():
    w = k1_world(plans=[plan()])
    run_c2(w, 2.99)
    assert "plan_observation_leg" not in w.names() and "near_loose_refusals" not in w.state
    w.tick(MID, t=43.0)
    (rec,) = ne.near_records(w)
    assert rec["outcome"] == "installed"
    (ev,) = w.state["near_loose_refusals"]
    assert ev["kinds"] == ["extra_c2"] and ev["leg_serial"] == 7 and ev["mission_id"] == "m1"
    assert ev["c2_record_index"] == w.state["observe_end_stands"].index(rec)  # opened before C2 recorded
    assert (ev["position_error_m"], ev["yaw_error_rad"]) == (0.06, 0.07) and ev["along_m"] is not None


def test_b0_leaves_the_same_stand_to_the_loose_arrival():
    w = k1_world(None)
    run_c2(w, 6.0)
    assert ne.near_records(w) == [] and "near_loose_refusals" not in w.state


def test_a_stand_outside_the_common_bounds_is_c2s_already_and_no_intervention():
    w = k1_world(plans=[plan()])
    run_c2(w, 3.0 + DT, ne.STAND)  # 52.8 mm, 0.112 rad: outside 0.10 rad
    assert ne.near_records(w)[0]["outcome"] == "installed" and "near_loose_refusals" not in w.state


def test_inside_the_narrow_bounds_nothing_is_judged():
    w = k1_world()
    run_c2(w, 6.0, replace(MID, position_error_m=0.04, yaw_error_rad=0.04))
    assert ne.near_records(w) == [] and "near_loose_refusals" not in w.state


def test_shadow_is_never_combined_in_the_core_rule_either():
    """The runner refuses --near-observe-arrival without --near-end-stand on; the core rule itself only narrows."""
    w = k1_world(mode="shadow")
    assert w.core.near_loose_bounds(40.0)[2] == "narrow"


# ---- the mission record ------------------------------------------------------------------------------------------


def test_the_new_state_keys_are_mission_keys():
    for key in ("near_loose_refusals", "near_loose_open", "near_loose_pending", "near_loose_fallbacks",
                "near_loose_watch"):
        assert key in MISSION_KEYS["state"]


def test_the_safety_state_carries_no_new_key():
    """The vehicle's and the console's view (safety_state) is the same with the option and an intervention open."""
    w_off, w_on = k1_world(None), k1_world(plans=[plan()])
    run_c2(w_on, 3.0 + DT)
    assert "near_loose_refusals" in w_on.state
    off, on = w_off.core.safety_state(43.0), w_on.core.safety_state(43.0)

    def keys(d, prefix=""):
        out = set()
        for k, v in d.items():
            out.add(prefix + k)
            if isinstance(v, dict):
                out |= keys(v, prefix + k + ".")
        return out
    assert keys(off) == keys(on)
    assert "near_loose" not in json.dumps(on, default=str)


def test_a_copy_of_the_world_without_the_option_runs_the_same():
    """The option absent and the option None give the same state, tick for tick (no read beyond the common values)."""
    a = k1_world(None, plans=[plan()])
    b = k1_world(None, plans=[plan()])
    b.core.options = copy.copy(b.core.options)
    b.core.options.near_observe_arrival = None
    for w in (a, b):
        run_c2(w, 6.0, ne.STAND)
    assert snapshot(a) == snapshot(b)
    assert [r["outcome"] for r in ne.near_records(a)] == [r["outcome"] for r in ne.near_records(b)]


# ---- command(): the blocked arrival and the stand's end ----------------------------------------------------------

import test_core_command as cc  # noqa: E402

CMID = replace(cc.Tracking(), status="tracking", speed_mps=0.0, position_error_m=0.06, yaw_error_rad=0.07)


def cmd_world(source="narrow", narrow=NARROW, **kw):
    w = cc.World("observe", remaining=0.0, near_capture="pending", **kw)
    w.state.update({"observe_replan_leg": {"start": 0, "serial": 7}, "observe_replans": []})
    w.state["near_capture"]["waypoint"] = [0.0, 0.0, 0.0]
    w.core.observe_backoff = None
    w.core.options.near_observe_arrival = narrow
    w.core.commands.mission_id = "m1"
    w.core.near_loose_bounds = lambda t: ((narrow[0], narrow[1], "narrow") if source == "narrow"
                                          else (0.10, 0.10, source))
    return w


def test_a_stand_inside_the_common_bounds_is_refused_once_and_recorded_as_a_blocked_arrival():
    w = cmd_world()
    for k in range(3):
        out = w.tick(CMID, t=10.0 + k)
        assert out.tracking.status == "tracking"  # B0 would have arrived at 11 s
    (ev,) = w.state["near_loose_refusals"]
    assert ev["kinds"] == ["blocked_arrival"] and ev["time_s"] == 10.0 and "observe_loose_arrivals" not in w.state
    assert w.state["near_loose_open"] == 0


def test_b0_arrives_at_the_same_stand_after_its_second():
    w = cc.World("observe", remaining=0.0)
    w.tick(CMID, t=10.0)
    assert w.tick(CMID, t=11.0).tracking.status == "arrived"


def test_the_stand_closes_at_the_arrival_with_its_source_and_whether_it_was_inside_the_narrow_bounds():
    w = cmd_world()
    w.tick(CMID, t=10.0)
    w.core.near_loose_bounds = lambda t: (0.10, 0.10, "fallback")  # C2 tried: the common bounds came back
    w.tick(CMID, t=11.0)
    out = w.tick(CMID, t=12.0)
    assert out.tracking.status == "arrived"
    (ev,) = w.state["near_loose_refusals"]
    assert ev["arrival"]["bounds_source"] == "fallback" and ev["arrival"]["inside_narrow"] is False
    assert "near_loose_open" not in w.state


def test_a_narrow_arrival_after_a_replan_is_inside_the_narrow_bounds():
    w = cmd_world()
    w.tick(CMID, t=10.0)
    close = replace(CMID, position_error_m=0.03, yaw_error_rad=0.02)
    w.tick(close, t=20.0)
    out = w.tick(close, t=21.0)
    assert out.tracking.status == "arrived"
    arrival = w.state["near_loose_refusals"][0]["arrival"]
    assert arrival["bounds_source"] == "narrow" and arrival["inside_narrow"] is True


def test_the_stand_closes_when_the_truck_moves_again():
    w = cmd_world()
    w.tick(CMID, t=10.0)
    w.tick(replace(CMID, speed_mps=0.1), t=10.5)
    (ev,) = w.state["near_loose_refusals"]
    assert ev["ends"][0]["how"] == "moved" and "near_loose_open" not in w.state
    assert w.state["near_loose_pending"] == 0  # its outcome still to come
    w.tick(CMID, t=11.0)  # a new stand: a new record, which supersedes the waiting one
    assert len(w.state["near_loose_refusals"]) == 2
    assert w.state["near_loose_refusals"][0]["superseded_by"] == 1 and "arrival" not in w.state["near_loose_refusals"][0]
    assert w.state["near_loose_open"] == 1 and "near_loose_pending" not in w.state


def test_the_stop_detection_does_not_end_the_stand():
    w = cmd_world()
    w.tick(CMID, t=10.0)
    w.slam["stop_now"] = False  # standing still, the detector not yet sure (C2's wait spans this)
    w.tick(CMID, t=10.5)
    assert w.state["near_loose_open"] == 0


def test_without_the_option_command_writes_nothing_new():
    w = cc.World("observe", remaining=0.0, near_capture="pending")
    before = set(w.state)
    w.tick(CMID, t=10.0)
    w.tick(CMID, t=11.0)
    assert set(w.state) - before == {"observe_loose_arrivals"}


def test_both_kinds_on_one_stand_make_one_record():
    w = k1_world()
    w.core.near_loose_event(40.0, "blocked_arrival", MID, [0.1, 0.0, 0.0])
    w.core.near_loose_event(43.0, "extra_c2", MID, [0.1, 0.0, 0.0])
    (ev,) = w.state["near_loose_refusals"]
    assert ev["kinds"] == ["blocked_arrival", "extra_c2"] and ev["time_s"] == 40.0


# ---- the outcome after the stand (Codex review: C2 -> move -> arrival -> capture) --------------------------------


def test_c2_then_a_move_then_an_arrival_then_the_capture_are_all_on_the_one_record():
    w = cmd_world()
    w.tick(CMID, t=10.0)  # the stand: blocked arrival
    w.core.near_loose_event(13.0, "extra_c2", CMID, [0.0, 0.06, 0.0])  # C2 judged it and installed a replan
    w.tick(replace(CMID, speed_mps=0.1), t=13.1)  # the replan drives
    assert w.state["near_loose_pending"] == 0 and "near_loose_open" not in w.state
    close = replace(CMID, position_error_m=0.03, yaw_error_rad=0.02)
    w.tick(close, t=20.0)
    assert w.tick(close, t=21.0).tracking.status == "arrived"
    (ev,) = w.state["near_loose_refusals"]
    assert ev["kinds"] == ["blocked_arrival", "extra_c2"] and ev["ends"][0]["how"] == "moved"
    assert ev["arrival"]["bounds_source"] == "narrow" and ev["arrival"]["inside_narrow"] is True
    assert ev["arrival"]["leg_serial"] == 7 and "capture" not in ev
    w.tick(close, t=21.1)  # the capture has not run yet
    assert "capture" not in ev and w.state["near_loose_pending"] == 0
    w.state["near_capture"]["status"] = "done"  # the runner's capture_and_detect between two ticks
    w.tick(close, t=21.5)
    assert ev["capture"] == {"time_s": 21.5, "status": "done"} and "near_loose_pending" not in w.state
    assert ev["arrival"]["time_s"] == 21.0  # the first arrival is kept


def test_an_arrival_on_the_stand_waits_for_the_capture_too():
    w = cmd_world()
    w.tick(CMID, t=10.0)
    w.core.near_loose_bounds = lambda t: (0.10, 0.10, "fallback")
    w.tick(CMID, t=11.0)
    assert w.tick(CMID, t=12.0).tracking.status == "arrived"
    assert w.state["near_loose_pending"] == 0
    w.state["near_capture"]["status"] = "failed"
    w.tick(CMID, t=12.5)
    assert w.state["near_loose_refusals"][0]["capture"]["status"] == "failed"


def test_two_missions_in_a_row_keep_their_records_apart():
    """The baseline restore at the second mission's start removes every K1 key, so nothing waits across missions."""
    from forklift_core.mission.baseline import MissionBaseline
    w = cmd_world()
    baseline = MissionBaseline.capture(w.core)
    w.tick(CMID, t=10.0)
    w.tick(replace(CMID, speed_mps=0.1), t=10.5)
    assert w.state["near_loose_pending"] == 0
    first = w.state["near_loose_refusals"]
    baseline.restore(w.core)
    w.core.commands.mission_id = "m2"
    for key in ("near_loose_refusals", "near_loose_open", "near_loose_pending", "near_loose_watch"):
        assert key not in w.state, key
    w.state.update({"observe_replan_leg": {"start": 0, "serial": 7}, "observe_replans": []})
    w.state["near_capture"] = {"status": "pending", "waypoint": [0.0, 0.0, 0.0]}
    w.tick(CMID, t=30.0)
    (ev,) = w.state["near_loose_refusals"]
    assert ev["mission_id"] == "m2" and w.state["near_loose_refusals"] is not first and len(first) == 1


def test_without_the_option_command_never_reads_the_pending_key():
    """Off: the outcome check is a dict lookup that finds nothing, and command()'s output is the same."""
    w = cc.World("observe", remaining=0.0, near_capture="pending")
    called = []
    w.core.near_loose_outcome = lambda t: called.append(t)
    w.core.near_loose_close = lambda *a: called.append(a)
    for k in range(3):
        w.tick(CMID, t=10.0 + k)
    assert called == []
