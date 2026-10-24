"""Leg-end stands handed to a receiver (docs/plans/2026-10-24-leg-end-budget-stands.md).

O1: on the first far observation, a final-goal stand (remaining 0, tracking, not at a cusp, a zero command, standing)
outside the loose arrival bounds for 3 s is planned again to the same target, once per observe leg serial; a failed
attempt keeps the truck standing and spends none of the leg's budget; with the budget spent there is no attempt. R1:
a return_home final-goal stand at remaining 0 after the stall budget is spent is an arrival within 0.10 m / 0.10 rad
only when the next preparation's start check passes for every pallet a submit could name next."""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np

import test_core_cusp_observe as co
import test_core_docking_stall as ds

DT = 1 / 120


# --- O1 -------------------------------------------------------------------------------------------------------------

STAND = co.Tracking(status="tracking", speed_mps=0.0, position_error_m=0.092, yaw_error_rad=-0.154)


def o_world(*, plans=(), observe_replans=0, far=True, remaining=0.0, holds_observe=False, stop_now=True):
    w = co.World("observe", plans=plans, observe_replans=observe_replans, holds_observe=holds_observe,
                 stop_now=stop_now)
    w.tracker.remaining_to_goal_m = lambda: remaining
    w.core.options.observe_arrival_m, w.core.options.observe_arrival_yaw_rad = 0.10, 0.10
    w.core.scene_far_observe = lambda: far
    return w


def stand_for(w, seconds, tracking=STAND, t0=10.0):
    out = None
    for k in range(int(round(seconds / DT))):
        out = w.tick(tracking, t=t0 + k * DT)
    return out


def test_o1_a_stand_outside_the_loose_bounds_for_3_s_is_planned_again_to_the_same_target():
    w = o_world(plans=[co.plan()])
    stand_for(w, 2.99)
    assert "plan_observation_leg" not in w.names()
    w.tick(STAND, t=10.0 + 3.0)
    assert w.names()[:4] == ["plan_observation_leg", "write_paths", "show_path", "realign_install"]
    _, args, kwargs = [c for c in w.calls if c[0] == "plan_observation_leg"][0]
    assert (args[1].x_m, args[1].y_m) == (1.0, 0.0) and kwargs["extended"] is False
    assert len(w.state["observe_replans"]) == 1 and w.core.trackers["observe"] is not w.tracker
    (rec,) = w.state["observe_end_stands"]
    assert rec["outcome"] == "installed" and rec["leg_serial"] == 7 and rec["position_error_m"] == 0.092
    assert w.ms.phase_started == 13.0


def test_o1_inside_the_loose_bounds_is_left_to_the_loose_arrival():
    w = o_world()
    stand_for(w, 5.0, replace(STAND, position_error_m=0.09, yaw_error_rad=0.09))
    assert w.names() == [] and "observe_end_stands" not in w.state


def test_o1_a_stand_that_settles_inside_the_bounds_is_left_to_the_loose_arrival():
    w = o_world()
    stand_for(w, 2.5)
    stand_for(w, 3.0, replace(STAND, yaw_error_rad=0.099), t0=12.5)  # inside on every tick from 2.5 s: never judged
    assert w.names() == []


def test_o1_a_stand_flickering_across_the_bound_is_timed_through_and_judged_on_an_outside_tick():
    # a synthetic heading on the bound, in and out (not the Isaac case: that one stayed outside, see the next test)
    w = o_world(plans=[co.plan()])
    t, k = 10.0, 0
    while t < 13.5:
        w.tick(replace(STAND, position_error_m=0.055, yaw_error_rad=(-0.101 if (k // 30) % 2 else -0.099)), t=t)
        t, k = t + DT, k + 1
    (rec,) = w.state["observe_end_stands"]
    assert rec["outcome"] == "installed" and rec["zero_command_s"] >= 3.0 and abs(rec["yaw_error_rad"]) > 0.10


def test_o1_the_stop_detectors_flicker_does_not_restart_the_count_and_the_judged_tick_must_stand():
    # Isaac ler_1451714_random_m_s1 m2: stop_lost / stop_confirmed every 0.1-1.3 s at a stand with a zero command
    w = o_world(plans=[co.plan()])
    t, k = 10.0, 0
    while t < 13.0 + 0.6 and "observe_end_stands" not in w.state:
        w.slam["stop_now"] = (k // 48) % 2 == 0  # 0.4 s standing, 0.4 s lost, ...
        w.tick(STAND, t=t)
        t, k = t + DT, k + 1
    (rec,) = w.state["observe_end_stands"]
    assert rec["outcome"] == "installed" and rec["zero_command_s"] >= 3.0 and w.slam["stop_now"]


def test_o1_leaving_the_final_goal_stand_restarts_the_count():
    w = o_world()
    stand_for(w, 2.9)
    w.tick(replace(STAND, speed_mps=0.01), t=12.95)  # commanded again: not a stand
    stand_for(w, 2.9, t0=13.0)
    assert w.names() == []


def test_o1_a_failed_replan_keeps_standing_spends_no_budget_and_leaves_the_tracking_to_the_loose_arrival():
    # the loose arrival itself is command()'s (unchanged); what this rule owes it is the tracking and state untouched
    w = o_world(plans=[co.plan(False)])
    out = stand_for(w, 3.0 + DT)
    assert w.state["observe_replans"] == [] and w.core.trackers["observe"] is w.tracker
    assert w.state["observe_end_stands"][0]["outcome"] == "failed:no_path"
    assert w.ms.phase_started == 0.0 and out.tracking == STAND and w.ms.observe_stall_ticks == 0
    stand_for(w, 5.0, t0=14.0)  # tried once for this leg serial: no second planning call
    assert [c[0] for c in w.calls].count("plan_observation_leg") == 1
    inside = replace(STAND, yaw_error_rad=0.099)  # the yaw settles inside the bounds afterwards
    out = w.tick(inside, t=20.0)
    assert out.tracking == inside and "observe_end_stand" not in w.state


def test_o1_does_not_try_with_the_legs_budget_spent():
    w = o_world(observe_replans=2)
    stand_for(w, 4.0)
    assert "plan_observation_leg" not in w.names()
    assert w.state["observe_end_stands"][0]["outcome"] == "no_budget"


def test_o1_does_not_re_arm_on_the_leg_it_installed():
    w = o_world(plans=[co.plan()])
    stand_for(w, 3.0 + DT)
    w.core.trackers["observe"].remaining_to_goal_m = lambda: 0.0
    stand_for(w, 6.0, t0=20.0)
    assert [c[0] for c in w.calls].count("plan_observation_leg") == 1


def test_o1_a_new_leg_serial_arms_again():
    w = o_world(plans=[co.plan(), co.plan()])
    stand_for(w, 3.0 + DT)
    w.state["observe_replan_leg"] = {"start": 1, "serial": 8}  # a B candidate's leg
    w.core.trackers["observe"].remaining_to_goal_m = lambda: 0.0
    stand_for(w, 3.0 + DT, t0=20.0)
    assert [c[0] for c in w.calls].count("plan_observation_leg") == 2


def test_o1_only_on_the_first_far_observation_and_only_standing_at_the_final_goal():
    for kw, tracking in (({"far": False}, STAND), ({"remaining": 0.02}, STAND), ({"holds_observe": True}, STAND),
                         ({"stop_now": False}, STAND), ({}, replace(STAND, speed_mps=0.01))):  # never standing: no judgement
        w = o_world(**kw)
        stand_for(w, 4.0, tracking)
        assert "plan_observation_leg" not in w.names() and "observe_end_stands" not in w.state, kw
    w = o_world(plans=[co.plan()])  # at a cusp: the existing observe stall's (one replan at 1 s), never O1's
    stand_for(w, 1.5, replace(STAND, at_cusp=True))
    assert "observe_end_stands" not in w.state and len(w.state["observe_replans"]) == 1


def test_o1_after_an_install_the_existing_cusp_recovery_takes_the_new_leg():
    # budget use 1 after O1: a cusp stall on the new leg is the existing observe_miss's (a further replan)
    w = o_world(plans=[co.plan(), co.plan()])
    stand_for(w, 3.0 + DT)
    w.ms.observe_stall_ticks = 119
    w.tick(co.Tracking(status="tracking", at_cusp=True, speed_mps=0.0), t=30.0)
    assert len(w.state["observe_replans"]) == 2
    # budget use 2: the same stall goes on to the existing B path (the scene device's candidates)
    w2 = o_world(plans=[co.plan()], observe_replans=1)
    stand_for(w2, 3.0 + DT)
    taken = []
    w2.core.scene_observe_candidate = lambda t, rear: taken.append(t) or SimpleNamespace(
        rear=rear, installed=True, tried=True)
    w2.ms.observe_stall_ticks = 119
    w2.tick(co.Tracking(status="tracking", at_cusp=True, speed_mps=0.0), t=30.0)
    assert taken == [30.0]


# --- R1 -------------------------------------------------------------------------------------------------------------

RSTAND = ds.Tracking(status="tracking", speed_mps=0.0, position_error_m=0.023, yaw_error_rad=0.053)


def r_world(*, stall_replans=2, remaining=0.0, gate=True, gates=None, **kw):
    w = ds.World("return_home", mode="tracking", stall_replans=stall_replans, remaining=remaining, **kw)
    w.core.options.observe_arrival_m, w.core.options.observe_arrival_yaw_rad = 0.10, 0.10
    seen = []

    def next_start_ok(t, rear):
        seen.append(t)
        return SimpleNamespace(ok=gate, results=gates or {"P-1": gate})
    w.core.next_start_ok = next_start_ok
    w.gate_calls = seen
    return w


def r_ticks(w, n, tracking=RSTAND, t0=50.0):
    out = None
    for k in range(n):
        out = w.tick(tracking, t=t0 + k * DT)
    return out


def test_r1_after_the_spent_budget_a_stand_within_the_bounds_that_every_next_start_accepts_arrives():
    w = r_world()
    out = r_ticks(w, 119)
    assert out.tracking.status == "tracking"
    out = w.tick(RSTAND, t=50.0 + 119 * DT)
    assert out.tracking.status == "arrived" and out.tracking.speed_mps == 0.0
    (rec,) = w.state["return_loose_arrivals"]
    assert rec["position_error_m"] == 0.023 and rec["start_checks"] == {"P-1": True}
    assert w.names() == []  # no planning call


def test_r1_a_refused_next_start_keeps_standing_and_is_asked_again_each_second():
    w = r_world(gate=False, gates={"P-1": True, "P-2": False})
    out = r_ticks(w, 120 + 240)
    assert out.tracking.status == "tracking" and "return_loose_arrivals" not in w.state
    assert len(w.gate_calls) == 3  # at 120, 240 and 360 ticks
    assert w.state["return_loose_checks"][0]["start_checks"] == {"P-1": True, "P-2": False}


def test_r1_with_budget_left_the_stall_replan_runs_as_before():
    w = r_world(stall_replans=1, plans=[ds.plan()])
    r_ticks(w, 120)
    assert "plan_return_leg" in w.names() and "return_loose_arrivals" not in w.state


def test_r1_outside_the_bounds_or_short_of_the_goal_or_not_returning_does_nothing():
    for kw, tracking in (({}, replace(RSTAND, position_error_m=0.11)), ({}, replace(RSTAND, yaw_error_rad=-0.11)),
                         ({"remaining": 2.4e-5}, RSTAND)):
        w = r_world(**kw)
        out = r_ticks(w, 240, tracking)
        assert out.tracking.status == "tracking" and w.gate_calls == [] and "return_loose_arrivals" not in w.state
    w = ds.World("transport", mode="tracking", stall_replans=2)
    w.core.next_start_ok = lambda t, rear: SimpleNamespace(ok=True, results={})
    out = r_ticks(w, 240)
    assert out.tracking.status != "arrived"


def test_r1_the_diagnostic_return_budget_zero_skips_the_replan():
    w = r_world(stall_replans=0)
    w.core.options.return_stall_budget = 0
    out = r_ticks(w, 120)
    assert out.tracking.status == "arrived" and "plan_return_leg" not in w.names()


# --- the next-start check over the candidates ----------------------------------------------------------------------

def test_next_start_ok_asks_every_available_or_delivered_pallet_and_needs_all():
    # every available or delivered one, whatever check_submit says now: the end release may still open a submit
    w = r_world()
    del w.core.next_start_ok
    asked = []

    def prepare_start_ok(rear, *, extra_m=0.0, preview=True, t=None, target_id=None):
        asked.append(target_id)
        return target_id != "P-3"
    w.core.hooks.prepare_start_ok = prepare_start_ok
    w.core.baseline = object()
    names = {"P-1": "delivered", "P-2": "available", "P-3": "available", "P-4": "carried", "P-5": "delivered"}
    w.core.targets = SimpleNamespace(
        revision=3, destinations={"D-1": None},
        pallets={k: SimpleNamespace(pallet_id=k, state=v) for k, v in names.items()},
        check_submit=lambda *a, **k: "destination_occupied")  # nothing submittable before the release
    out = w.core.next_start_ok(50.0, np.array([1.0, 2.0, 0.0]))
    assert sorted(asked) == ["P-1", "P-2", "P-3", "P-5"] and out.ok is False
    assert out.results == {"P-1": True, "P-2": True, "P-3": False, "P-5": True}


def test_r1_is_not_a_backoffs_end():
    w = r_world(obstacle={"backoff": {"phase": "return_home"}})
    out = r_ticks(w, 240)
    assert out.tracking.status == "tracking" and w.gate_calls == [] and "return_loose_arrivals" not in w.state


def test_next_start_ok_without_a_check_or_candidates_falls_back_to_the_end_gate():
    w = r_world()
    del w.core.next_start_ok
    out = w.core.next_start_ok(50.0, np.array([1.0, 2.0, 0.0]))  # a batch run: no check, no targets
    assert out.ok is True and out.results == {}
