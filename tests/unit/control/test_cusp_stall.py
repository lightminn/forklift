"""Cusp position stall (docs/plans/2026-10-04-lidar-obstacle-map.md, "cusp 위치 정체 복구 설계 7판")."""

from dataclasses import replace

import numpy as np

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.control.cusp_stall import (
    CuspStallMonitor,
    StallConfig,
    judge,
    pose_bounds,
    prefix_converged,
    thin,
)
from forklift_core.control.rollout import PrefixResult

DT = 1.0 / 120.0
CONFIG = TrackerConfig(
    cruise_speed_mps=0.18,
    max_curvature_inv_m=0.9,
    max_acceleration_mps2=0.3,
    lookahead_m=0.28,
    position_tolerance_m=0.008,
    yaw_tolerance_rad=0.02,
    stop_speed_mps=0.012,
    cusp_position_tolerance_m=0.03,
    cusp_yaw_tolerance_rad=0.05,
    cusp_brake_window_m=0.008,
)


def forward_then_reverse(reverse_m=4.0):
    xs = np.linspace(0.0, 1.0, 26)
    back = 1.0 - np.linspace(0.04, reverse_m, int(round(reverse_m / 0.04)))
    x = np.r_[xs, back]
    poses = np.column_stack((x, np.zeros(len(x)), np.zeros(len(x))))
    return (
        poses,
        np.r_[np.ones(len(xs)), -np.ones(len(back))].astype(np.int8),
        np.zeros(len(x)),
    )


def stalled_tracker(lateral, yaw, reverse_m=4.0):
    """A tracker that reached the cusp at x = 1 lateral metres to the side and stands there."""
    tracker = RearAxlePathTracker(*forward_then_reverse(reverse_m), CONFIG)
    for x in np.linspace(0.0, 1.0, 51):
        tracker.update([x, lateral, yaw], 0.1, 0.02)
    return tracker


def run(
    monitor,
    tracker,
    pose,
    ticks,
    *,
    phase="transport",
    t0=0.0,
    sweep=True,
    sensors=None,
    **callbacks,
):
    """Stand at pose for ticks, calling the monitor as the runner does; stops at an end."""
    out = []
    for i in range(ticks):
        t = t0 + i * DT
        raw = tracker.update(pose, 0.0, DT)

        def judge_now(latch, raw=raw):
            return judge(
                tracker,
                pose,
                raw,
                latch.cache,
                end_recovered=lambda goal: False,
                sweep_free=lambda trajectory: sweep,
                sensors=sensors,
                stall=monitor.stall,
            )

        given = dict(
            standing=lambda: True,
            stopped=lambda: True,
            alive=lambda: True,
            ownership=lambda command: (True, {"owned": True}),
            judge_now=judge_now,
            scan_id=lambda t=t: int(t * 10 + 1e-9),
            replan_pending=lambda: False,
            prefix_occupied=lambda latch: False,
            release_update=lambda: tracker.update(pose, 0.0, DT),
        )
        given.update(callbacks)
        result = monitor.tick(
            t,
            phase,
            tracker,
            raw,
            raw,
            rear=pose,
            excluded=None,
            retry_stand=False,
            **given,
        )
        out.append(result)
        if (
            result.end is not None
            or monitor.latch is None
            and result.tracking.status != "tracking"
        ):
            break
    return out


def test_a_4_cm_cusp_stall_is_held_then_started_from_the_measured_pose():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    results = run(monitor, tracker, pose, 200)
    # 120 ticks of progress under 1 mm first: nothing latched, the tracker's own command.
    assert all(monitor_free.hold is False for monitor_free in results[:120])
    assert (
        len(records) == 1
        and records[0]["phase"] == "transport"
        and records[0]["leg_index"] == 0
    )
    assert (
        records[0]["position_error_m"] > 0.03 and records[0]["leg_remaining_m"] <= 0.008
    )
    # Standing and owned: the stop is confirmed and the first scan releases into the next leg.
    assert records[0]["disposition"] == "released"
    assert [e["event"] for e in records[0]["events"]] == ["stop_confirmed", "released"]
    assert tracker.leg_index == 1 and monitor.latch is None
    scan = records[0]["scans"][0]
    assert (
        scan["converged"] and scan["sweep_ok"] and scan["prefix"]["status"] == "prefix"
    )
    assert (
        abs(scan["prefix"]["d_lateral_m"]) <= 0.01
        and abs(scan["prefix"]["d_heading_rad"]) <= 0.01
    )
    released = next(
        r for r in results if r.tracking.speed_mps < 0 or not r.tracking.at_cusp
    )
    assert not released.hold and released.tracking.status == "tracking"


def test_a_braking_stall_is_held_at_zero_with_the_steering_until_it_stands():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    results = run(monitor, tracker, pose, 121 + 40, stopped=lambda: False)
    held = results[121:]
    assert all(
        r.hold and r.tracking.speed_mps == 0.0 and r.tracking.status == "tracking"
        for r in held
    )
    assert monitor.latch.mode == "brake"
    # Unconfirmed for 5 s: the run ends, in every phase.
    ended = run(monitor, tracker, pose, 700, t0=2.0, stopped=lambda: False)
    assert (
        ended[-1].end == "cusp_position_stall_unconfirmed"
        and records[0]["disposition"] == "ended"
    )


def test_a_heading_past_the_cusp_tolerance_is_not_released_and_ends_after_the_wait():
    pose = (1.0, 0.04, 0.06)  # 0.06 rad: past 0.05, not far (0.08)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    results = run(monitor, tracker, pose, 121 + 120 * 6)
    assert tracker.leg_index == 0
    assert results[-1].end == "cusp_position_stall"
    stop_s = next(
        e["time_s"] for e in records[0]["events"] if e["event"] == "stop_confirmed"
    )
    assert results[-1].hold and len(results) * DT - stop_s >= 5.0 - DT
    assert all(not s["bounds_ok"] and not s["far"] for s in records[0]["scans"])
    # Observe hands the same stall to its existing stop rule instead, still at zero.
    observe = stalled_tracker(*pose[1:])
    monitor_o = CuspStallMonitor([])
    held = run(monitor_o, observe, pose, 121 + 120 * 6, phase="observe")
    assert (
        held[-1].end is None
        and held[-1].hold
        and monitor_o.latch.mode == "observe_rule"
    )
    assert not monitor_o.holds_observe()  # the observe rule runs now


def test_far_past_the_bounds_is_decided_at_the_first_scan():
    pose = (1.0, 0.04, 0.09)
    tracker = stalled_tracker(*pose[1:])
    records = []
    results = run(CuspStallMonitor(records), tracker, pose, 400)
    assert results[-1].end == "cusp_position_stall" and len(records[0]["scans"]) == 1
    assert records[0]["scans"][0]["far"]


def test_a_blocked_sweep_or_a_non_converging_prefix_is_not_released():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    results = run(CuspStallMonitor(records), tracker, pose, 121 + 120 * 6, sweep=False)
    assert tracker.leg_index == 0 and results[-1].end == "cusp_position_stall"
    assert all(s["converged"] and not s["sweep_ok"] for s in records[0]["scans"])
    # A stricter match than the prefix reaches: not converged, not released.
    strict = stalled_tracker(*pose[1:])
    tight = CuspStallMonitor([], StallConfig(match_m=1e-6, match_rad=1e-6))
    ended = run(tight, strict, pose, 121 + 120 * 6)
    assert strict.leg_index == 0 and ended[-1].end == "cusp_position_stall"


def test_a_failing_sensor_check_waits_then_hands_to_the_obstacle_side_once_per_key():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    blind = lambda trajectory, direction: (False, False, {"plain": "unknown"})  # noqa: E731
    # The forced replan pending: handed it stays.
    results = run(
        monitor,
        tracker,
        pose,
        121 + 120 * 6,
        sensors=blind,
        replan_pending=lambda: True,
    )
    handed = [r for r in results if r.force_replan]
    assert len(handed) == 1 and not handed[0].hold
    assert (
        records[0]["disposition"] == "handed_to_obstacle"
        and monitor.handed("transport") is monitor.latch
    )
    assert (
        monitor.latch.hand_reason == "sensor" and monitor.latch.hand_direction == 1
    )  # the stalled leg's
    after = results[results.index(handed[0]) + 1 :]
    assert after and all(not r.hold for r in after)  # the zero override stops
    # The replan done and the checks passing again: ownership returns, the stall is judged anew from
    # the stop, and a second hand-off forces no second replan.
    again = run(monitor, tracker, pose, 120 * 7, t0=10.0, sensors=blind)
    events = records[0]["events"]
    assert "ownership_returned" in [e["event"] for e in events]
    assert [e["forced"] for e in events if e["event"] == "handed_to_obstacle"][:2] == [
        True,
        False,
    ]
    assert not any(r.force_replan for r in again) and len(records) == 1


def test_an_occupied_prefix_for_three_scans_hands_over_in_the_next_legs_direction():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    occupied = lambda trajectory, direction: (False, True, {"passes": "occupied"})  # noqa: E731
    results = run(
        monitor,
        tracker,
        pose,
        121 + 60,
        sensors=occupied,
        prefix_occupied=lambda latch: True,
    )
    assert any(r.force_replan for r in results)
    latch = monitor.latch
    assert (
        latch.hand_reason == "prefix_occupied"
        and latch.hand_direction == -1
        and latch.prefix_occupied_now
    )
    assert len(records[0]["scans"]) == 3 and latch.prefix_trajectory is not None
    # Kept while it stays occupied, even with no replan pending.
    run(monitor, tracker, pose, 40, t0=3.0, prefix_occupied=lambda latch: True)
    assert monitor.latch.mode == "handed"
    run(monitor, tracker, pose, 40, t0=4.0, prefix_occupied=lambda latch: False)
    assert "ownership_returned" in [e["event"] for e in records[0]["events"]]
    assert (
        records[0]["disposition"] == "released"
    )  # judged anew from the stop, the prefix clear now


def test_the_obstacle_side_keeps_a_stall_it_owns():
    pose = (1.0, 0.04, 0.0)
    blocked = lambda command: (False, {"path": "occupied"})  # noqa: E731
    # Still creeping on a command: the obstacle block runs and owns it -- nothing is latched.
    creeping = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    orig_update = creeping.update

    def update_creeping(p, s, dt):
        return replace(orig_update(p, s, dt), speed_mps=0.0016)

    creeping.update = update_creeping
    run(monitor, creeping, pose, 200, ownership=blocked)
    assert records == [] and monitor.latch is None
    # Standing on an exact zero the block does not run: handed over at once, with the forced replan.
    standing = stalled_tracker(*pose[1:])
    monitor_s = CuspStallMonitor([])
    results = run(monitor_s, standing, pose, 125, ownership=blocked)
    assert results[120].force_replan and monitor_s.latch.mode == "handed"
    assert monitor_s.latch.hand_reason == "ownership"
    # Ownership lost while judging: handed too, not ended.
    judged = stalled_tracker(*pose[1:])
    monitor_j = CuspStallMonitor([])
    calls = {"n": 0}

    def flips(command):
        calls["n"] += 1
        return (calls["n"] <= 1, {"n": calls["n"]})

    blind = lambda trajectory, direction: (False, False, {})  # noqa: E731
    out = run(monitor_j, judged, pose, 121 + 30, ownership=flips, sensors=blind)
    assert monitor_j.latch.mode == "handed" and not any(r.end for r in out)


def test_the_latch_ends_with_its_key_and_never_covers_a_failure():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    run(monitor, tracker, pose, 122, stopped=lambda: False)
    assert monitor.latch is not None
    # A new tracker for the phase (a replan): closed.
    other = stalled_tracker(*pose[1:])
    raw = other.update(pose, 0.0, DT)
    monitor.tick(
        2.0,
        "transport",
        other,
        raw,
        raw,
        rear=pose,
        excluded=None,
        retry_stand=False,
        standing=lambda: True,
        stopped=lambda: False,
        alive=lambda: True,
        ownership=lambda c: (True, None),
        judge_now=None,
        scan_id=lambda: 0,
        replan_pending=lambda: False,
        prefix_occupied=lambda latch: False,
        release_update=None,
    )
    assert monitor.latch is None and records[0]["closed"] == "key_changed"
    # A failed tracker passes through untouched.
    run(monitor, other, pose, 122, stopped=lambda: False)
    raw = replace(
        other.update(pose, 0.0, DT), status="failed", failure="endpoint_heading"
    )
    result = monitor.tick(
        3.0,
        "transport",
        other,
        raw,
        raw,
        rear=pose,
        excluded=None,
        retry_stand=False,
        standing=lambda: True,
        stopped=lambda: False,
        alive=lambda: True,
        ownership=lambda c: (True, None),
        judge_now=None,
        scan_id=lambda: 0,
        replan_pending=lambda: False,
        prefix_occupied=lambda latch: False,
        release_update=None,
    )
    assert result.tracking is raw and not result.hold and monitor.latch is None
    assert records[-1]["closed"] == "status:failed"


def test_exclusions_and_other_phases_never_latch():
    pose = (1.0, 0.04, 0.0)
    for phase, excluded in (
        ("transport", "d7_transport"),
        ("observe", "realign_held"),
        ("approach", None),
    ):
        tracker = stalled_tracker(*pose[1:])
        records = []
        monitor = CuspStallMonitor(records)
        for i in range(200):
            raw = tracker.update(pose, 0.0, DT)
            result = monitor.tick(
                i * DT,
                phase,
                tracker,
                raw,
                raw,
                rear=pose,
                excluded=excluded,
                retry_stand=False,
                standing=lambda: True,
                stopped=lambda: True,
                alive=lambda: True,
                ownership=lambda c: (True, None),
                judge_now=None,
                scan_id=lambda: 0,
                replan_pending=lambda: False,
                prefix_occupied=lambda latch: False,
                release_update=None,
            )
            assert result.tracking is raw and not result.hold
        assert records == []


def test_the_bounds_and_the_prefix_rule():
    assert pose_bounds(CONFIG, 0.06, 0.05) == (True, False)
    assert pose_bounds(CONFIG, 0.0601, 0.0) == (False, False)
    assert pose_bounds(CONFIG, 0.0901, 0.0) == (False, True)
    assert pose_bounds(CONFIG, 0.0, -0.0801) == (False, True)
    ref = PrefixResult("prefix", 3.0, 0.002, -0.004, None, False)
    assert prefix_converged(
        PrefixResult("prefix", 3.0, 0.011, -0.004, None, False), ref, lambda g: False
    )
    assert not prefix_converged(
        PrefixResult("prefix", 3.0, 0.0121, -0.004, None, False), ref, lambda g: False
    )
    assert not prefix_converged(
        PrefixResult("prefix", 3.0, 0.002, 0.0061, None, False), ref, lambda g: False
    )
    assert prefix_converged(
        PrefixResult("leg_end", 1.2, 0.02, 0.03, None, False), ref, lambda g: False
    )
    heading_miss = PrefixResult("failed", 0.6, 0.0, 0.04, "endpoint_heading", True)
    assert prefix_converged(
        heading_miss, ref, lambda goal: goal
    )  # within an existing rule's budget
    assert not prefix_converged(heading_miss, ref, lambda goal: False)
    assert not prefix_converged(
        PrefixResult("failed", 0.6, 0.0, 0.0, "cross_track", True), ref, lambda g: True
    )
    assert not prefix_converged(
        PrefixResult("timeout", 0.6, 0.0, 0.0, None, True), ref, lambda g: True
    )
    poses = np.column_stack((np.linspace(0, 1, 1001), np.zeros(1001), np.zeros(1001)))
    thinned = thin(poses, 0.02)
    assert thinned[0][0] == 0.0 and thinned[-1][0] == 1.0 and 45 <= len(thinned) <= 51
    assert (np.diff(thinned[:-1, 0]) >= 0.02 - 1e-9).all()


def test_a_short_next_leg_ending_at_the_goal_in_a_heading_miss_follows_the_budget():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:], reverse_m=0.6)
    raw = tracker.update(pose, 0.0, DT)
    seen = []
    judged = judge(
        tracker,
        pose,
        raw,
        {},
        end_recovered=lambda goal: seen.append(goal) or True,
        sweep_free=lambda trajectory: True,
        sensors=None,
    )
    assert (
        seen == [True]
        and judged.pose_ok
        and judged.record["prefix"]["status"] == "failed"
    )
    refused = judge(
        tracker,
        pose,
        raw,
        {},
        end_recovered=lambda goal: False,
        sweep_free=lambda trajectory: True,
        sensors=None,
    )
    assert not refused.pose_ok


# Codex implementation review counterexamples (P1/P2).
def blind(trajectory, direction):
    return False, False, {"plain": "unknown"}


def test_a_frozen_scan_still_ends_the_wait_on_the_last_judgement():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    results = run(
        monitor,
        tracker,
        pose,
        121 + 120 * 6,
        sensors=blind,
        scan_id=lambda: 0,
        replan_pending=lambda: True,
    )
    assert len(records[0]["scans"]) == 1  # one scan only
    assert (
        sum(r.force_replan for r in results) == 1
        and monitor.latch.hand_reason == "sensor"
    )
    # The pose-bound outcome too: a 0.06 rad heading ends on time without another scan.
    turned = stalled_tracker(0.04, 0.06)
    ended = run(
        CuspStallMonitor([]),
        turned,
        (1.0, 0.04, 0.06),
        121 + 120 * 6,
        scan_id=lambda: 0,
    )
    assert ended[-1].end == "cusp_position_stall"


def test_a_silent_sensor_is_handed_over_at_once_while_standing():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    results = run(
        monitor, tracker, pose, 130, alive=lambda: False, replan_pending=lambda: True
    )
    assert results[120].force_replan and monitor.latch.hand_reason == "silent"
    assert records[0]["scans"] == []  # no decision on a silent sensor's stale snapshot


def test_one_forced_replan_per_key_across_a_new_latch():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    occupied = lambda trajectory, direction: (False, True, {})  # noqa: E731
    first = run(
        monitor,
        tracker,
        pose,
        121 + 40,
        sensors=occupied,
        prefix_occupied=lambda latch: True,
    )
    assert sum(r.force_replan for r in first) == 1
    # A status other than tracking closes the latch; the same key latched again is handed again
    # without a second forced replan.
    raw = replace(tracker.update(pose, 0.0, DT), status="braking")
    monitor.tick(
        2.0, "transport", tracker, raw, raw, rear=pose, excluded=None, retry_stand=False,
        standing=lambda: True, stopped=lambda: True, alive=lambda: True, ownership=lambda c: (True, None),
        judge_now=None, scan_id=lambda: 0, replan_pending=lambda: False,
        prefix_occupied=lambda latch: True, release_update=None,
    )  # fmt: skip
    assert monitor.latch is None and records[0]["closed"] == "status:braking"
    again = run(
        monitor,
        tracker,
        pose,
        60,
        t0=3.0,
        sensors=occupied,
        prefix_occupied=lambda latch: True,
    )
    assert (
        len(records) == 2
        and records[1]["tracker_serial"] == records[0]["tracker_serial"]
    )
    assert monitor.latch.mode == "handed" and not any(r.force_replan for r in again)
    assert [
        e["forced"] for e in records[1]["events"] if e["event"] == "handed_to_obstacle"
    ] == [False]


def test_a_lost_stop_goes_back_to_braking_and_is_never_handed_over_moving():
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    calls = {"n": 0}

    def stopped_once():
        calls["n"] += 1
        return calls["n"] == 1

    results = run(
        monitor, tracker, pose, 121 + 120 * 7, sensors=blind, stopped=stopped_once
    )
    events = [e["event"] for e in records[0]["events"]]
    assert events[:2] == ["stop_confirmed", "stop_lost"]
    assert not any(r.force_replan for r in results) and all(
        r.hold for r in results[120:]
    )
    assert results[-1].end == "cusp_position_stall_unconfirmed"


def test_each_tracker_object_gets_its_own_serial():
    monitor = CuspStallMonitor([])
    a, b = stalled_tracker(0.04, 0.0), stalled_tracker(0.04, 0.0)
    assert monitor.serial(a) == 1 and monitor.serial(b) == 2 and monitor.serial(a) == 1


def test_a_stop_confirmed_again_does_not_count_the_same_scan_twice():
    # Codex implementation review 2 P1: standing lost and confirmed again on one scan re-judges it, but
    # one occupied scan stays one.
    pose = (1.0, 0.04, 0.0)
    tracker = stalled_tracker(*pose[1:])
    records = []
    monitor = CuspStallMonitor(records)
    occupied = lambda trajectory, direction: (False, True, {})  # noqa: E731
    flips = {"n": 0}

    def flapping():
        flips["n"] += 1
        return flips["n"] % 2 == 1  # confirmed, lost, confirmed, ...

    results = run(
        monitor,
        tracker,
        pose,
        121 + 60,
        sensors=occupied,
        scan_id=lambda: 99,
        stopped=flapping,
    )
    assert len(records[0]["scans"]) > 3  # re-judged at every confirmation
    assert monitor.latch.occupied_scans == 1 and not any(
        r.force_replan for r in results
    )
    assert monitor.latch.mode in ("brake", "judge")


def test_a_goal_stall_counts_only_within_an_existing_rules_budget():
    # Cusp design amendment: a short next leg that stalls at the final goal is the stall replan's.
    ref = PrefixResult("arrived", 1.6, 0.0, 0.0, None, True)
    stall = PrefixResult("goal_stall", 1.6, -0.068, 0.032, None, True)
    assert prefix_converged(stall, ref, lambda goal: goal)
    assert not prefix_converged(stall, ref, lambda goal: False)
    assert not prefix_converged(
        PrefixResult("timeout", 1.6, -0.068, 0.032, None, True), ref, lambda goal: True
    )
