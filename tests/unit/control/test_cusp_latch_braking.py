"""The cusp stall latch and the tracker's against-motion brake (docs/plans/2026-10-22-cusp-latch-braking.md).

A standing truck's odometry mean crossed the stop speed against the leg for a few ticks at a time (Isaac
n2_random_1_s0); each crossing made the tracker brake against that motion and report ``braking``, and the latch,
closed by any status other than ``tracking``, never reached its 5 s judgement. The latch now keeps through that
brake -- and only that one."""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.control.cusp_stall import CuspStallMonitor, judge
from forklift_core.localization.slam_pose import StopDetector

# The path and tracker of tests/unit/control/test_cusp_stall.py (no package to import them from).
DT = 1.0 / 120.0
CONFIG = TrackerConfig(
    cruise_speed_mps=0.18, max_curvature_inv_m=0.9, max_acceleration_mps2=0.3, lookahead_m=0.28,
    position_tolerance_m=0.008, yaw_tolerance_rad=0.02, stop_speed_mps=0.012, cusp_position_tolerance_m=0.03,
    cusp_yaw_tolerance_rad=0.05, cusp_brake_window_m=0.008,
)


def forward_then_reverse(reverse_m=4.0):
    xs = np.linspace(0.0, 1.0, 26)
    back = 1.0 - np.linspace(0.04, reverse_m, int(round(reverse_m / 0.04)))
    x = np.r_[xs, back]
    poses = np.column_stack((x, np.zeros(len(x)), np.zeros(len(x))))
    return poses, np.r_[np.ones(len(xs)), -np.ones(len(back))].astype(np.int8), np.zeros(len(x))


def stalled_tracker(lateral, yaw, reverse_m=4.0, end_x=1.0):
    """A tracker that reached x = end_x (the cusp is at 1) lateral metres to the side and stands there."""
    tracker = RearAxlePathTracker(*forward_then_reverse(reverse_m), CONFIG)
    for x in np.linspace(0.0, end_x, 51):
        tracker.update([x, lateral, yaw], 0.1, 0.02)
    return tracker

# Half a millimetre short of the cusp (n2_random_1_s0 stood 0.5 mm short: the tracker's own target +0.0008 m/s, so a
# crossing against the leg brakes against it), past the 0.05 rad cusp heading but not far: judged out after the wait.
END_X = 0.9995
POSE = (END_X, 0.04, 0.06)


def stalled(pose=POSE):
    return stalled_tracker(pose[1], pose[2], end_x=pose[0])


def burst_speeds(ticks, *, every=240, length=6, value=-0.03, first=150):
    """The standing truck's odometry: zero, with a short burst against the leg every `every` ticks (n2_random_1_s0
    crossed for 1-8 ticks every 0.3-6 s; here every 2 s from 1.25 s, after the first latch)."""
    speeds = np.zeros(ticks)
    for start in range(first, ticks, every):
        speeds[start:start + length] = value
    return speeds


def stand(monitor, tracker, speeds, *, phase="observe", pose=POSE, inject=None, ownership=None, replan_pending=None):
    """The runner's chain on a standing truck: stop detector -> tracker_speed -> tracker -> monitor. ``inject(raw,
    tracker, monitor)`` may return (raw, monitor keywords) for this tick. Returns one record per tick: raw, the
    monitor's result, the against-motion flag of this raw, the latch's mode and observe hold after the tick, the stop
    detector's verdict."""
    stop = StopDetector(tick_s=DT)
    out = []
    for i, speed in enumerate(speeds):
        t = i * DT
        stop.update(commanded_speed=0.0, speed=float(speed), yaw_rate=0.0)
        raw = tracker.update(pose, stop.tracker_speed(0.012), DT)
        extra = {}
        if inject is not None:
            injected = inject(raw, tracker, monitor)
            if injected is not None:
                raw, extra = injected

        def judge_now(latch, raw=raw):
            return judge(tracker, pose, raw, latch.cache, end_recovered=lambda goal: False,
                         sweep_free=lambda trajectory: True, sensors=None, stall=monitor.stall)

        kw = dict(excluded=None, retry_stand=False)
        kw.update(extra)
        flag = tracker.braking_against_motion
        result = monitor.tick(
            t, phase, tracker, raw, raw, rear=pose,
            standing=lambda: stop.stopped, stopped=lambda: stop.stopped, alive=lambda: True,
            ownership=ownership or (lambda command: (True, {"owned": True})), judge_now=judge_now,
            scan_id=lambda t=t: int(t * 10 + 1e-9), replan_pending=replan_pending or (lambda: False),
            prefix_occupied=lambda latch: False, release_update=lambda: tracker.update(pose, 0.0, DT), **kw,
        )  # fmt: skip
        latch = monitor.latch
        out.append(SimpleNamespace(raw=raw, result=result, flag=flag, mode=latch.mode if latch else None,
                                   held=monitor.holds_observe(), stopped=stop.stopped))
        if result.end is not None or (inject is not None and extra and monitor.latch is None):
            break
    return out


def braked_in(out, modes):
    """Ticks that braked against motion while the latch was in one of these modes (the mode before the tick)."""
    before = [None] + [o.mode for o in out[:-1]]
    return [o for o, mode in zip(out, before, strict=True) if o.raw.status == "braking" and o.flag and mode in modes]


def at_first_latched_brake(change):
    """An injection at the first tick that brakes against motion while a stall is latched."""
    done = {"v": False}

    def inject(raw, tracker, monitor):
        if done["v"] or monitor.latch is None or raw.status != "braking" or not tracker.braking_against_motion:
            return None
        done["v"] = True
        return change(raw)

    return inject, done


# --- the tracker's flag -------------------------------------------------------------------------------------

def test_the_tracker_names_its_against_motion_brake_and_nothing_else():
    tracker = RearAxlePathTracker(*forward_then_reverse(), CONFIG)
    assert tracker.braking_against_motion is False
    rolling = tracker.update([0.3, 0.0, 0.0], 0.1, DT)
    assert rolling.status == "tracking" and tracker.braking_against_motion is False
    against = tracker.update([0.3, 0.0, 0.0], -0.02, DT)  # moving back on a forward leg
    assert against.status == "braking" and tracker.braking_against_motion is True
    again = tracker.update([0.3, 0.0, 0.0], 0.1, DT)
    assert again.status == "tracking" and tracker.braking_against_motion is False
    # Within the stop speed: no brake.
    tracker.update([0.3, 0.0, 0.0], -0.011, DT)
    assert tracker.braking_against_motion is False


def test_an_endpoint_brake_is_not_an_against_motion_brake():
    tracker = stalled_tracker(0.0, 0.0)  # on the cusp itself: the endpoint brake
    raw = tracker.update([1.0, 0.0, 0.0], -0.02, DT)
    assert raw.status in ("braking", "arrived") and tracker.braking_against_motion is False


# --- the latch ------------------------------------------------------------------------------------------------

def test_the_noise_bursts_closed_the_latch_before_its_judgement_and_now_do_not():
    records = []
    monitor = CuspStallMonitor(records)
    out = stand(monitor, stalled(), burst_speeds(121 + 120 * 9))
    assert braked_in(out, ("brake", "judge"))  # the bursts braked against motion while it was being judged
    assert len(records) == 1 and "closed" not in records[0]
    events = [e["event"] for e in records[0]["events"]]
    assert "stop_confirmed" in events and records[0]["disposition"] == "observe_rule"
    assert monitor.latch is not None and monitor.latch.mode == "observe_rule"
    assert braked_in(out, ("observe_rule",))  # and after the hand-over to the observe rule
    # Held at zero throughout, the bursts included.
    assert all(o.result.tracking.speed_mps == 0.0 for o in out[121:])


def test_the_latch_keeps_through_a_brake_while_braking_and_judging():
    records = []
    monitor = CuspStallMonitor(records)
    # Bursts every 0.33 s from the latch on: through the brake (stop not yet confirmed) and the judgement.
    out = stand(monitor, stalled(), burst_speeds(121 + 120 * 3, every=40, length=7, first=122), phase="transport")
    assert braked_in(out, ("brake",)) and braked_in(out, ("judge",))
    assert len(records) == 1 and "closed" not in records[0]


def test_the_latch_keeps_through_a_brake_while_handed_to_the_obstacle_side():
    records = []
    monitor = CuspStallMonitor(records)
    owned = {"calls": 0}

    def ownership(command):
        # Owned when the stall latches, then the obstacle side takes it at the first judged scan.
        owned["calls"] += 1
        return (owned["calls"] == 1, {"owned": owned["calls"] == 1})

    out = stand(monitor, stalled(), burst_speeds(121 + 120 * 3, every=60, length=6, first=200), phase="transport",
                ownership=ownership, replan_pending=lambda: True)
    assert monitor.latch is not None and monitor.latch.mode == "handed"
    assert braked_in(out, ("handed",))
    assert len(records) == 1 and "closed" not in records[0]
    assert sum(o.result.force_replan for o in out) == 1  # once per key, the brakes notwithstanding


@pytest.mark.parametrize("why, change", [
    ("status:failed", lambda raw: (replace(raw, status="failed", failure="endpoint_heading"), {})),
    ("status:arrived", lambda raw: (replace(raw, status="arrived"), {})),
    ("status:braking", lambda raw: (replace(raw, cusp_pending=True), {})),  # a held cusp's brake
    ("excluded:d7_transport", lambda raw: (raw, {"excluded": "d7_transport"})),
    ("excluded:retry_stand", lambda raw: (raw, {"retry_stand": True})),
])
def test_on_a_brake_against_motion_the_other_closes_still_close_the_latch(why, change):
    records = []
    monitor = CuspStallMonitor(records)
    inject, done = at_first_latched_brake(change)
    stand(monitor, stalled(), burst_speeds(121 + 120 * 4), phase="transport", inject=inject)
    assert done["v"] and records[0]["closed"] == why


def test_a_brake_the_tracker_did_not_mark_still_closes_the_latch():
    # Standing still (no crossing, so the flag stays False): a braking status from anywhere else closes it.
    records = []
    monitor = CuspStallMonitor(records)
    out = stand(monitor, stalled(), np.zeros(400), inject=lambda raw, tr, mon: (
        None if mon.latch is None else (replace(raw, status="braking"), {})))
    assert records[0]["closed"] == "status:braking" and not out[-1].flag


def test_a_truck_that_keeps_moving_against_the_leg_still_ends_unconfirmed():
    records = []
    monitor = CuspStallMonitor(records)
    speeds = np.r_[np.zeros(121), np.full(120 * 7, -0.02)]  # never standing again after the latch
    out = stand(monitor, stalled(), speeds, phase="transport")
    assert out[-1].result.end == "cusp_position_stall_unconfirmed" and "closed" not in records[0]
    assert braked_in(out, ("brake",))


# --- the observe rule after the hand-over (core.py's observe stall counter, emulated per tick) -----------------

def observe_rule_counts(out):
    """core.py ~2087: status tracking, at the cusp, at zero, standing and the stall not held by the monitor -- else
    reset; from each tick's own returned command, stop verdict and holds_observe()."""
    counter, counts = 0, []
    for o in out:
        cmd = o.result.tracking
        ok = cmd.status == "tracking" and cmd.at_cusp and cmd.speed_mps == 0.0 and o.stopped and not o.held
        counter = counter + 1 if ok else 0
        counts.append(counter)
    return counts


def test_after_the_hand_over_the_observe_rule_reaches_its_120_ticks_once_the_bursts_stop():
    speeds = burst_speeds(121 + 120 * 12, every=240, length=6, first=150)
    speeds[121 + 120 * 8:] = 0.0  # 4 s standing still after the last burst
    records = []
    monitor = CuspStallMonitor(records)
    out = stand(monitor, stalled(), speeds)
    assert records[0]["disposition"] == "observe_rule" and "closed" not in records[0]
    handed_at = next(i for i, o in enumerate(out) if o.mode == "observe_rule")
    assert all(o.held for o in out[121:handed_at]) and not any(o.held for o in out[handed_at:])
    counts = observe_rule_counts(out)
    assert max(counts[:handed_at]) == 0 and max(counts[handed_at:]) >= 120


def test_a_stop_broken_again_and_again_resets_the_observe_rules_count():
    speeds = burst_speeds(121 + 120 * 12, every=90, length=7, first=150)  # a burst every 0.75 s, never ending
    records = []
    monitor = CuspStallMonitor(records)
    out = stand(monitor, stalled(), speeds)
    assert records[0]["disposition"] == "observe_rule" and "closed" not in records[0]
    handed_at = next(i for i, o in enumerate(out) if o.mode == "observe_rule")
    counts = observe_rule_counts(out)[handed_at:]
    # It counts between the bursts and starts again after each: this change does not promise the observe replan
    # under every noise sequence.
    assert 0 < max(counts) < 120 and counts.count(0) > 1


# --- the joint-angle odometry's discrete speeds ---------------------------------------------------------------

def test_single_count_ticks_of_the_joint_angle_odometry_never_brake():
    speeds = np.zeros(121 + 120 * 9)
    speeds[150::27] = -0.0115  # one count of one wheel now and then: the 12-tick mean stays far under 0.012
    records = []
    monitor = CuspStallMonitor(records)
    out = stand(monitor, stalled(), speeds)
    assert not any(o.raw.status == "braking" for o in out)
    assert len(records) == 1 and records[0]["disposition"] == "observe_rule"


def test_seven_two_count_ticks_running_cross_the_stop_speed_and_the_latch_keeps():
    speeds = np.zeros(121 + 120 * 9)
    speeds[300:307] = -0.023  # mean of 12 ticks -0.0134
    records = []
    monitor = CuspStallMonitor(records)
    out = stand(monitor, stalled(), speeds)
    assert braked_in(out, ("brake", "judge"))
    assert len(records) == 1 and "closed" not in records[0] and records[0]["disposition"] == "observe_rule"
