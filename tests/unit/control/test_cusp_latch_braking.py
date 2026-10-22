"""The cusp stall latch and the tracker's against-motion brake (docs/plans/2026-10-22-cusp-latch-braking.md).

A standing truck's odometry mean crossed the stop speed against the leg for a few ticks at a time (Isaac
n2_random_1_s0); each crossing made the tracker brake against that motion and report ``braking``, and the latch,
closed by any status other than ``tracking``, never reached its 5 s judgement. The latch now keeps through that
brake -- and only that one."""

from dataclasses import replace

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


def stand(monitor, tracker, speeds, *, phase="observe", pose=POSE, stop=None, close_at=None):
    """The runner's chain on a standing truck: stop detector -> tracker_speed -> tracker -> monitor."""
    stop = stop or StopDetector(tick_s=DT)
    out = []
    for i, speed in enumerate(speeds):
        t = i * DT
        stop.update(commanded_speed=0.0, speed=float(speed), yaw_rate=0.0)
        raw = tracker.update(pose, stop.tracker_speed(0.012), DT)
        if close_at is not None and i == close_at[0]:
            raw = close_at[1](raw)

        def judge_now(latch, raw=raw):
            return judge(tracker, pose, raw, latch.cache, end_recovered=lambda goal: False,
                         sweep_free=lambda trajectory: True, sensors=None, stall=monitor.stall)

        result = monitor.tick(
            t, phase, tracker, raw, raw, rear=pose, excluded=None, retry_stand=False,
            standing=lambda: stop.stopped, stopped=lambda: stop.stopped, alive=lambda: True,
            ownership=lambda command: (True, {"owned": True}), judge_now=judge_now,
            scan_id=lambda t=t: int(t * 10 + 1e-9), replan_pending=lambda: False,
            prefix_occupied=lambda latch: False, release_update=lambda: tracker.update(pose, 0.0, DT),
        )  # fmt: skip
        out.append((raw, result))
        if result.end is not None:
            break
    return out


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
    tracker = stalled()
    out = stand(monitor, tracker, burst_speeds(121 + 120 * 9))
    assert any(raw.status == "braking" for raw, _ in out)  # the bursts did brake against motion
    assert len(records) == 1 and "closed" not in records[0]
    events = [e["event"] for e in records[0]["events"]]
    assert "stop_confirmed" in events and records[0]["disposition"] == "observe_rule"
    assert monitor.latch is not None and monitor.latch.mode == "observe_rule"
    # Held at zero throughout, the bursts included.
    assert all(r.tracking.speed_mps == 0.0 for _, r in out[121:])


def test_the_latch_keeps_through_a_brake_in_every_mode():
    # observe_rule above; brake / judge: bursts right after the latch; handed: below.
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    stand(monitor, tracker, burst_speeds(121 + 120 * 2, every=40, length=3, first=122), phase="transport")
    assert len(records) == 1 and "closed" not in records[0]
    handed_records = []
    handed = CuspStallMonitor(handed_records)
    tracker_h = stalled()
    out = []
    stop = StopDetector(tick_s=DT)
    speeds = burst_speeds(121 + 120 * 3, every=60, length=6, first=200)
    owned = {"calls": 0}

    def ownership(command):
        # Owned when the stall latches, then the obstacle side takes it at the first judged scan.
        owned["calls"] += 1
        return (owned["calls"] == 1, {"owned": owned["calls"] == 1})

    for i, speed in enumerate(speeds):
        t = i * DT
        stop.update(commanded_speed=0.0, speed=float(speed), yaw_rate=0.0)
        raw = tracker_h.update(POSE, stop.tracker_speed(0.012), DT)
        result = handed.tick(
            t, "transport", tracker_h, raw, raw, rear=POSE, excluded=None, retry_stand=False,
            standing=lambda: stop.stopped, stopped=lambda: stop.stopped, alive=lambda: True,
            ownership=ownership, judge_now=None, scan_id=lambda t=t: int(t * 10),
            replan_pending=lambda: True, prefix_occupied=lambda latch: False, release_update=None,
        )  # fmt: skip
        out.append((raw, result))
    assert handed.latch is not None and handed.latch.mode == "handed"
    assert any(raw.status == "braking" for raw, _ in out[200:])  # braked against motion while handed
    assert len(handed_records) == 1 and "closed" not in handed_records[0]
    assert sum(r.force_replan for _, r in out) == 1  # once per key, the brakes notwithstanding


def observe_rule_ticks(out, stop_flags):
    """core.py's observe stall counter over the monitor's output: status tracking, at the cusp, at zero, standing,
    and the stall not held by the monitor -- reset otherwise. Returns the counter after each tick."""
    counter, counts = 0, []
    for (raw, result), (standing, held) in zip(out, stop_flags, strict=False):
        ok = (result.tracking.status == "tracking" and result.tracking.at_cusp and result.tracking.speed_mps == 0.0
              and standing and not held)
        counter = counter + 1 if ok else 0
        counts.append(counter)
    return counts


def stand_recording(monitor, tracker, speeds):
    """stand(), also recording the stop detector and the monitor's observe hold every tick."""
    stop = StopDetector(tick_s=DT)
    flags = []
    real = stop.update

    def update(**kw):
        value = real(**kw)
        flags.append([value, None])
        return value

    stop.update = update
    out = stand(monitor, tracker, speeds, stop=stop)
    for i in range(len(out)):
        flags[i][1] = False  # filled below from the latch's mode after the run
    return out, flags


def test_after_the_hand_over_the_observe_rule_reaches_its_120_ticks_once_the_bursts_stop():
    speeds = burst_speeds(121 + 120 * 12, every=240, length=6, first=150)
    speeds[121 + 120 * 8:] = 0.0  # 4 s standing still after the last burst
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    out, flags = stand_recording(monitor, tracker, speeds)
    observe_from = next(i for i, (_, r) in enumerate(out) if monitor.latch is not None) if out else 0
    rule_start = next(e["time_s"] for e in records[0]["events"] if e["event"] == "observe_rule")
    held = [(i * DT) < rule_start for i in range(len(out))]
    counts = observe_rule_ticks(out, [(f[0], h) for f, h in zip(flags, held, strict=False)])
    assert observe_from >= 0 and max(counts) >= 120


def test_a_stop_broken_again_and_again_resets_the_observe_rules_count():
    speeds = burst_speeds(121 + 120 * 12, every=90, length=6, first=150)  # a burst every 0.75 s, never ending
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    out, flags = stand_recording(monitor, tracker, speeds)
    rule = [e["time_s"] for e in records[0]["events"] if e["event"] == "observe_rule"]
    held = [(i * DT) < (rule[0] if rule else 1e9) for i in range(len(out))]
    counts = observe_rule_ticks(out, [(f[0], h) for f, h in zip(flags, held, strict=False)])
    assert max(counts) < 120  # this change does not promise the observe replan under every noise sequence


@pytest.mark.parametrize("why, change", [
    ("status:failed", lambda raw: replace(raw, status="failed", failure="endpoint_heading")),
    ("status:arrived", lambda raw: replace(raw, status="arrived")),
])
def test_a_failure_or_an_arrival_still_closes_the_latch_on_a_braking_tick(why, change):
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    stand(monitor, tracker, burst_speeds(400), close_at=(300, change))
    assert records[0]["closed"] == why


def test_a_pending_or_endpoint_brake_still_closes_the_latch():
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    # A braking result the tracker did not mark as against motion (the flag is False on a zero-speed update).
    stand(monitor, tracker, np.zeros(400), close_at=(300, lambda raw: replace(raw, status="braking")))
    assert records[0]["closed"] == "status:braking"


def test_an_exclusion_or_a_retry_stand_still_closes_on_a_braking_tick():
    for kw, why in ((dict(excluded="d7_transport"), "excluded:d7_transport"),
                    (dict(retry_stand=True), "excluded:retry_stand")):
        records = []
        monitor = CuspStallMonitor(records)
        tracker = stalled()
        stop = StopDetector(tick_s=DT)
        for i, speed in enumerate(burst_speeds(400, first=300)):
            stop.update(commanded_speed=0.0, speed=float(speed), yaw_rate=0.0)
            raw = tracker.update(POSE, stop.tracker_speed(0.012), DT)
            args = dict(excluded=None, retry_stand=False)
            if i >= 300:
                args.update(kw)
            monitor.tick(
                i * DT, "transport", tracker, raw, raw, rear=POSE, standing=lambda: stop.stopped,
                stopped=lambda: stop.stopped, alive=lambda: True, ownership=lambda command: (True, None),
                judge_now=lambda latch, raw=raw: judge(tracker, POSE, raw, latch.cache, end_recovered=lambda g: False,
                                                       sweep_free=lambda tr: True, sensors=None, stall=monitor.stall),
                scan_id=lambda i=i: i // 12, replan_pending=lambda: False, prefix_occupied=lambda latch: False,
                release_update=lambda: tracker.update(POSE, 0.0, DT), **args,
            )  # fmt: skip
            if i >= 300 and monitor.latch is None:
                break
        assert records[0]["closed"] == why


def test_a_truck_that_keeps_moving_against_the_leg_still_ends_unconfirmed():
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    speeds = np.r_[np.zeros(121), np.full(120 * 7, -0.02)]  # never standing again after the latch
    out = stand(monitor, tracker, speeds, phase="transport")
    assert out[-1][1].end == "cusp_position_stall_unconfirmed" and "closed" not in records[0]


# --- the joint-angle odometry's discrete speeds ---------------------------------------------------------------

def test_single_count_ticks_of_the_joint_angle_odometry_never_brake():
    speeds = np.zeros(121 + 120 * 9)
    speeds[150::27] = -0.0115  # one count of one wheel now and then: the 12-tick mean stays far under 0.012
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    out = stand(monitor, tracker, speeds)
    assert not any(raw.status == "braking" for raw, _ in out)
    assert len(records) == 1 and records[0]["disposition"] == "observe_rule"


def test_seven_two_count_ticks_running_cross_the_stop_speed_and_the_latch_keeps():
    speeds = np.zeros(121 + 120 * 9)
    speeds[300:307] = -0.023  # mean of 12 ticks -0.0134
    records = []
    monitor = CuspStallMonitor(records)
    tracker = stalled()
    out = stand(monitor, tracker, speeds)
    assert any(raw.status == "braking" for raw, _ in out)
    assert len(records) == 1 and "closed" not in records[0] and records[0]["disposition"] == "observe_rule"
