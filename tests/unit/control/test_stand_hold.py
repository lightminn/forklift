"""docs/plans/2026-11-01-standstill-wheel-hold.md S1: one wheel lock state (none / stand / dwell) for the four wheel
drives (front left, front right, rear left, rear right), the original gains kept once, the handovers fixed."""

import math

import numpy as np
import pytest

from forklift_core.control.stand_hold import (STAND_LOCK_AFTER_S, STAND_LOCK_WHEEL_RAD_S, STAND_PHASES, WHEEL_KP,
                                              WheelHold)

KPS = np.array([0.0, 0.0, 0.0, 0.0, 5.0, 5.0, 7.0])  # four wheels, two steers, the lift
KDS = np.array([20.0, 20.0, 20.0, 20.0, 1.0, 1.0, 2.0])
WHEELS = np.array([0, 1, 2, 3])
ZERO = np.zeros(4)


class Robot:
    def __init__(self):
        self.kps, self.kds = KPS.copy(), KDS.copy()
        self.q = np.array([1.0, 2.0, 3.0, 4.0])
        self.sets = []

    def gains(self):
        return self.kps.copy(), self.kds.copy()

    def set_gains(self, kps, kds):
        self.sets.append((kps.copy(), kds.copy()))
        self.kps, self.kds = kps.copy(), kds.copy()


def tick(hold, robot, t, *, cmd=ZERO, phase="transport", rates=ZERO, dwell=False):
    out = hold.step(t, dwell_standing=dwell, wheel_cmd=cmd, phase=phase, wheel_rates=rates, q_now=robot.q.copy(),
                    read_gains=robot.gains)
    if out.gains is not None:
        robot.set_gains(*out.gains)
    return out


def stand_until_locked(hold, robot, t0=0.0, **kw):
    t = t0
    while t <= t0 + STAND_LOCK_AFTER_S + 0.05:
        out = tick(hold, robot, t, **kw)
        t += 1 / 120
    return out, t


def test_constants_are_the_plans():
    assert (STAND_LOCK_AFTER_S, STAND_LOCK_WHEEL_RAD_S, WHEEL_KP) == (0.5, 0.05, 2000.0)
    assert STAND_PHASES == {"observe", "approach", "insert", "extract", "transport", "withdraw", "return_home"}


def test_a_zero_command_stand_locks_the_rear_wheels_after_half_a_second():
    hold, robot = WheelHold("rear"), Robot()
    out = tick(hold, robot, 0.0)
    assert hold.mode == "none" and out.q is None and robot.sets == []
    out = tick(hold, robot, STAND_LOCK_AFTER_S - 1e-6)
    assert hold.mode == "none"
    out = tick(hold, robot, STAND_LOCK_AFTER_S)
    assert hold.mode == "stand"
    kps, kds = robot.sets[-1]
    assert list(kps[WHEELS]) == [0.0, 0.0, WHEEL_KP, WHEEL_KP] and list(kps[4:]) == [5.0, 5.0, 7.0]
    assert np.array_equal(kds, KDS) and list(out.q) == [1.0, 2.0, 3.0, 4.0]
    assert hold.events[-1]["to"] == "stand" and hold.events[-1]["from"] == "none"


def test_the_all_variant_locks_four_wheels_for_the_probe():
    hold, robot = WheelHold("all"), Robot()
    stand_until_locked(hold, robot)
    assert list(robot.kps[WHEELS]) == [WHEEL_KP] * 4


@pytest.mark.parametrize("rates", [
    np.array([0.0, 0.0, 0.06, 0.0]), np.array([0.0, 0.0, -0.06, 0.0]), np.array([0.0, 0.0, 0.0, -0.08]),
    np.array([0.0, 0.0, math.nan, 0.0]), np.array([0.0, 0.0, 0.0, math.inf]),
    np.array([0.0, 0.0, STAND_LOCK_WHEEL_RAD_S, 0.0]),
])
def test_a_rear_wheel_still_turning_either_way_or_unread_keeps_it_unlocked(rates):
    hold, robot = WheelHold("rear"), Robot()
    stand_until_locked(hold, robot, rates=rates)
    assert hold.mode == "none" and robot.sets == []


def test_the_front_wheels_rates_do_not_matter():
    hold, robot = WheelHold("rear"), Robot()
    stand_until_locked(hold, robot, rates=np.array([0.3, -0.3, 0.0, 0.01]))
    assert hold.mode == "stand"


def test_a_command_releases_before_it_is_applied_and_the_count_starts_again():
    hold, robot = WheelHold("rear"), Robot()
    _, t = stand_until_locked(hold, robot)
    out = tick(hold, robot, t, cmd=np.array([0.0, 0.0, 0.1, 0.1]))
    assert hold.mode == "none" and out.q is None
    assert np.array_equal(robot.kps, KPS) and np.array_equal(robot.kds, KDS)  # the original gains back
    assert hold.events[-1]["to"] == "none"
    tick(hold, robot, t + 0.01)
    assert hold.mode == "none"  # a new half second from here


@pytest.mark.parametrize("phase", ["lift", "lower", "settle", "home_settle", "complete"])
def test_outside_the_driving_phases_nothing_locks_and_a_lock_is_released(phase):
    hold, robot = WheelHold("rear"), Robot()
    stand_until_locked(hold, robot, phase=phase)
    assert hold.mode == "none"
    hold, robot = WheelHold("rear"), Robot()
    _, t = stand_until_locked(hold, robot)
    tick(hold, robot, t, phase=phase)
    assert hold.mode == "none" and np.array_equal(robot.kps, KPS)


def test_handover_none_stand_dwell_stand_none_keeps_the_rear_target_and_the_original_gains():
    hold, robot = WheelHold("rear"), Robot()
    _, t = stand_until_locked(hold, robot)
    robot.q = np.array([1.5, 2.5, 3.0001, 4.0001])  # the front turned, the rear held
    out = tick(hold, robot, t, dwell=True)
    assert hold.mode == "dwell" and list(robot.kps[WHEELS]) == [WHEEL_KP] * 4
    assert list(out.q) == [1.5, 2.5, 3.0, 4.0]  # rear target kept, front from now
    out = tick(hold, robot, t + 0.01, dwell=False)  # the dwell ends, still a zero-command stand
    assert hold.mode == "stand" and list(robot.kps[WHEELS]) == [0.0, 0.0, WHEEL_KP, WHEEL_KP]
    assert list(out.q[2:]) == [3.0, 4.0]
    tick(hold, robot, t + 0.02, cmd=np.array([0.2, 0.2, 0.2, 0.2]))
    assert hold.mode == "none" and np.array_equal(robot.kps, KPS) and np.array_equal(robot.kds, KDS)
    assert [e["to"] for e in hold.events] == ["stand", "dwell", "stand", "none"]


def test_handover_none_dwell_none_is_the_dwell_lock_as_before():
    hold, robot = WheelHold("rear"), Robot()
    out = tick(hold, robot, 0.0, dwell=True)
    assert hold.mode == "dwell" and list(robot.kps[WHEELS]) == [WHEEL_KP] * 4 and list(out.q) == [1.0, 2.0, 3.0, 4.0]
    out = tick(hold, robot, 0.01, dwell=False, cmd=np.array([0.1] * 4))
    assert hold.mode == "none" and np.array_equal(robot.kps, KPS)


def test_gains_are_read_once_when_a_lock_starts():
    hold, robot = WheelHold("rear"), Robot()
    reads = []

    def read():
        reads.append(1)
        return robot.gains()
    t = 0.0
    while t < 1.0:
        hold.step(t, dwell_standing=False, wheel_cmd=ZERO, phase="transport", wheel_rates=ZERO, q_now=robot.q,
                  read_gains=read)
        t += 1 / 120
    assert len(reads) == 1 and hold.mode == "stand"


def test_off_is_rejected_here():
    with pytest.raises(ValueError):
        WheelHold("off")  # off keeps the runner's own dwell lock path untouched


def test_the_runner_option_is_off_by_default_batch_only_and_in_the_near_field_contract():
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(root / "sim/isaac"))
    sys.path.insert(0, str(root / "tests/unit/tools"))
    import near_tracking
    from test_riibotics_isaac_scenes import _runner_args

    args = _runner_args([])
    assert args.stand_wheel_hold == "off" and args.stand_probe_log is None
    assert _runner_args(["--stand-wheel-hold", "rear"]).stand_wheel_hold == "rear"
    with pytest.raises(SystemExit):
        _runner_args(["--stand-wheel-hold", "maybe"])
    with pytest.raises(SystemExit):
        _runner_args(["--stand-wheel-hold", "rear", "--live"])
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["stand_wheel_hold"] == "off"
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["stand_probe_log"] is None
    source = (root / "sim/isaac/run_transport.py").read_text()
    # off keeps the dwell lock's own path: the new branch is taken only with the stand lock on
    assert "if stand_hold is not None:" in source and "elif dwell_standing and wheel_lock[\"q\"] is None:" in source
    assert 'if getattr(args, "stand_wheel_hold", "off") != "off":' in source
