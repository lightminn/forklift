"""Persistent console plan D14 (5th ed.): the teleoperated lift (one target, the autonomous ramp's form: a target
position moved at a capped rate and clipped to the model's travel) and the recovery's straight reverse."""

import math

import pytest

from forklift_core.safety.teleop_drive import TeleopDrive, TeleopLift, TeleopLimits

DT = 1 / 120


def lift(**kw):
    return TeleopLift(TeleopLimits(**kw), limit_m=0.28, dt=DT)


def cmd(dir_, deadman=True):
    return {"deadman": deadman, "lift_dir": dir_}


def test_it_starts_from_the_target_it_takes_over_and_moves_at_the_rate_within_the_travel():
    tl = lift()
    assert tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=0.05, start_from=0.10) == pytest.approx(
        0.10 + 0.02 * DT)  # from the handed-over target, not the (sagged) measurement
    for _ in range(int(20 / DT)):
        out = tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=0.27, start_from=None)
    assert out == pytest.approx(0.28)  # clipped to the model's travel


@pytest.mark.parametrize("why", ["deadman", "dir0", "policy", "disarm", "leave"])
def test_an_ended_active_stretch_latches_the_measured_lift_once(why):
    tl = lift()
    for _ in range(10):
        tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=0.08, start_from=0.10)
    kw = dict(armed=True, dirs={-1, 1}, measured=0.08, start_from=None)
    if why == "deadman":
        out = tl.step(True, cmd(1, deadman=False), **kw)
    elif why == "dir0":
        out = tl.step(True, cmd(0), **kw)
    elif why == "policy":
        out = tl.step(True, cmd(1), **{**kw, "dirs": set()})
    elif why == "disarm":
        out = tl.step(True, cmd(1), **{**kw, "armed": False})
    else:
        out = tl.step(False, None, **kw)  # TELEOP left: the latched value is handed over once
    assert out == pytest.approx(0.08)
    # held from then on: a sagging measurement is not followed (Codex D14 design round 1 P1-1)
    nxt = tl.step(why != "leave", cmd(0), armed=True, dirs={-1, 1}, measured=0.07, start_from=None)
    assert nxt == (None if why == "leave" else pytest.approx(0.08))


def test_a_clipped_stretch_still_latches_when_it_ends():
    """Codex D14 design round 3 P2-2: the target at the limit does not change, but the stretch is active."""
    tl = lift()
    for _ in range(int(20 / DT)):
        tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=0.26, start_from=0.0)
    assert tl.step(True, cmd(1, deadman=False), armed=True, dirs={-1, 1}, measured=0.26, start_from=None) == \
        pytest.approx(0.26)


def test_a_lift_never_moved_by_hand_keeps_its_target():
    tl = lift()
    out = tl.step(True, cmd(0), armed=True, dirs=set(), measured=0.17, start_from=0.20)  # carrying, sagged 30 mm
    assert out == pytest.approx(0.20)
    assert tl.step(False, None, armed=True, dirs=set(), measured=0.17, start_from=None) == pytest.approx(0.20)


def test_without_a_measurement_it_does_not_move():
    tl = lift()
    assert tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=None, start_from=0.10) == pytest.approx(0.10)


@pytest.mark.parametrize("field, value", [("lift_rate_mps", 0.0), ("lift_rate_mps", math.nan),
                                          ("recovery_reverse_mps", -0.1), ("recovery_steer_tol_rad", math.inf)])
def test_the_new_limits_are_positive_and_finite(field, value):
    with pytest.raises(ValueError):
        TeleopLimits(**{field: value})


def drive_step(drv, teleop, *, last, steering=0.0, measured_mps=0.0):
    return drv.step(teleop, armed=True, drive_allowed=True, loaded=False, preempt=False, last_applied_mps=last,
                    measured_mps=measured_mps, measured_kappa=0.0, steering_error_rad=0.0,
                    permit=lambda arcs, d, s: (math.inf, "ok"), mode="reverse_straight", steering_rad=steering)


def test_the_recovery_reverse_is_straight_slow_and_backward_only():
    drv = TeleopDrive(TeleopLimits(), 1.5, DT)
    back = {"deadman": True, "mode": "direct", "speed_ratio": -1.0, "curvature_ratio": 1.0}
    speed = 0.0
    for _ in range(240):
        speed, kappa, _ = drive_step(drv, back, last=speed)
    assert speed == pytest.approx(-0.10) and kappa == 0.0
    fwd = {**back, "speed_ratio": 1.0}
    s2 = 0.0
    for _ in range(60):
        s2, _, _ = drive_step(drv, fwd, last=s2)
    assert s2 == 0.0


def test_a_steering_off_zero_zeroes_the_output_in_that_tick():
    """Codex D14 design round 1 P1-2: a zero target alone left -0.0875 m/s on the slew."""
    drv = TeleopDrive(TeleopLimits(), 1.5, DT)
    back = {"deadman": True, "mode": "direct", "speed_ratio": -1.0, "curvature_ratio": 0.0}
    speed, _, why = drive_step(drv, back, last=-0.10, steering=0.02, measured_mps=-0.10)
    assert speed == 0.0 and why[1] == "recovery:steering"


# --- in the arbiter's closed loop (the M-b5 harness, a lift that follows its target with a lag) -------------------
from test_arbiter_loop import DT as LOOP_DT, Loop, SAFE  # noqa: E402

from forklift_core.safety.arbiter import Feedback  # noqa: E402


class LiftLoop(Loop):
    def __init__(self, lift=0.0):
        super().__init__()
        self.lift, self.cmd = lift, lift

    def cycle(self, *msgs, teleop=None, pacing=True):
        self.a.pacing_ok = pacing
        a = self.a
        original_observe = a.observe
        a.observe = lambda t, fb: original_observe(t, Feedback(t, self.v, lift_m=self.lift, steering_rad=0.0))
        original_step = a.step
        a.step = lambda t, safety, cand, drive=None: original_step(t, safety, cand,
                                                                   {**(drive or {}), "lift_command_m": self.cmd})
        try:
            out, replies = super().cycle(*msgs, teleop=teleop)
        finally:
            a.observe, a.step = original_observe, original_step
        if out.lift_target_m is not None:
            self.cmd = out.lift_target_m
        self.lift += (self.cmd - self.lift) * LOOP_DT / 0.2  # the actuator follows with a 0.2 s lag
        return out, replies


def test_teleop_lifts_at_its_rate_and_hands_the_target_over_when_left():
    loop = LiftLoop()
    loop.to_teleop_armed()
    for _ in range(int(2 / LOOP_DT)):
        out, _ = loop.cycle(teleop={"deadman": True, "speed_ratio": 0.0, "curvature_ratio": 0.0, "lift_dir": 1})
    assert out.lift_target_m == pytest.approx(0.04, abs=0.002)  # 2 s at 0.02 m/s
    out, _ = loop.cycle(loop.msg("mode_request", to="PAUSED"))
    assert out.mode == "PAUSED" and out.lift_target_m == pytest.approx(loop.lift, abs=0.002)  # latched measured
    out, _ = loop.cycle()
    assert out.lift_target_m is None  # the arbiter no longer owns it; the handed-over target stays


def test_losing_the_client_mid_lift_latches_the_measured_lift():
    loop = LiftLoop()
    loop.to_teleop_armed()
    for _ in range(int(2 / LOOP_DT)):
        loop.cycle(teleop={"deadman": True, "speed_ratio": 0.0, "curvature_ratio": 0.0, "lift_dir": 1})
    loop.client_alive = False  # no echo, no commands: the deadline and the lease expire
    seen = []
    for _ in range(int(2 / LOOP_DT)):
        out, _ = loop.cycle()
        seen.append(loop.cmd)
    # the last command holds until its deadline (command_max_age_s 0.3 s, as for the drive), then the target is
    # latched to the measured lift once and held
    assert max(seen) <= seen[0] + 0.02 * 0.30 + 1e-6
    later = seen[int(0.5 / LOOP_DT):]
    assert max(later) - min(later) < 1e-9


def test_no_pacing_no_lift():
    loop = LiftLoop()
    loop.to_teleop_armed()
    for _ in range(60):
        out, _ = loop.cycle(teleop={"deadman": True, "speed_ratio": 0.0, "curvature_ratio": 0.0, "lift_dir": 1},
                            pacing=False)
    assert out.policy.lift_dirs == frozenset() and loop.cmd == pytest.approx(0.0)
