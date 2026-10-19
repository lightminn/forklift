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
    # the target ran ahead (0.10 + 10 ticks); it stops within a = 0.02 m of the measured lift (main, 2026-10-19)
    assert out == pytest.approx(0.08 + 0.02)
    # held from then on: a sagging measurement is not followed (Codex D14 design round 1 P1-1)
    nxt = tl.step(why != "leave", cmd(0), armed=True, dirs={-1, 1}, measured=0.07, start_from=None)
    assert nxt == (None if why == "leave" else pytest.approx(0.10))


def test_a_clipped_stretch_still_latches_when_it_ends():
    """Codex D14 design round 3 P2-2: the target at the limit does not change, but the stretch is active."""
    tl = lift()
    for _ in range(int(20 / DT)):
        tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=0.26, start_from=0.0)
    assert tl.step(True, cmd(1, deadman=False), armed=True, dirs={-1, 1}, measured=0.26, start_from=None) == \
        pytest.approx(0.28)  # 0.02 m above the measurement: within a, the target is kept


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
    # held within the hold tolerance of the measured lift (main 2026-10-19: the ramp's target, clipped to it)
    assert out.mode == "PAUSED" and abs(out.lift_target_m - loop.lift) <= 0.02 + 1e-9
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


def test_a_gate_latch_is_the_manual_lifts_target_from_then_on():
    """Codex D14 impl round 1 P1-1: the gate held the lift at 0.17 m; the old 0.20 m target must not come back."""
    tl = lift()
    tl.step(True, cmd(0), armed=True, dirs={-1, 1}, measured=0.17, start_from=0.20)
    tl.sync(0.17)
    assert tl.step(True, cmd(0), armed=True, dirs={-1, 1}, measured=0.17, start_from=None) == pytest.approx(0.17)
    idle = lift()
    idle.sync(0.17)  # not owned: nothing to sync
    assert idle.target is None


@pytest.mark.parametrize("direction", [1, -1])
def test_an_event_stops_the_target_within_the_hold_tolerance_of_the_measurement(direction):
    """Main's condition 3 as amended (2026-10-19): held = clip(target, measured - a, measured + a)."""
    tl = lift()
    start = 0.10
    for _ in range(int(3 / DT)):  # 3 s: the target runs 0.06 m away while the lift lags at the start
        tl.step(True, cmd(direction), armed=True, dirs={-1, 1}, measured=start, start_from=start)
    held = tl.step(True, cmd(0), armed=True, dirs={-1, 1}, measured=start, start_from=None)
    assert held == pytest.approx(start + direction * 0.02)


def test_repeated_events_do_not_ratchet_a_standing_lift_down():
    """Main (2026-10-19): with the position drive's gravity sag s < a, three release events in a row leave the lift
    no lower than one sag below where it was put (the old rule lowered the target by s each time)."""
    sag = 0.0087 + 0.003  # carriage 1.6 kg x 9.81 / 1800 N/m, plus the blades
    tl = lift()
    measured = 0.0
    for _ in range(int(6 / DT)):
        target = tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=measured, start_from=0.0)
        measured = max(0.0, target - sag)
    first = None
    for _ in range(3):
        target = tl.step(True, cmd(0), armed=True, dirs={-1, 1}, measured=measured, start_from=None)
        measured = max(0.0, target - sag)  # the plant settles a sag below its target
        first = measured if first is None else first
        for _ in range(5):  # a short press that the policy refuses (moving: no lift) is another stretch's end
            tl.step(True, cmd(1), armed=True, dirs=set(), measured=measured, start_from=None)
    assert measured >= first - 1e-9


@pytest.mark.parametrize("value", [0.0, -0.01, math.nan])
def test_the_hold_tolerance_is_positive_and_finite(value):
    with pytest.raises(ValueError):
        TeleopLimits(lift_hold_tolerance_m=value)


def test_a_stretch_ending_with_the_target_standing_keeps_it_whatever_the_sag():
    """Main 2026-10-19 (option 1): only a ramp target moving in that tick is clipped to the measurement; at the
    travel's end the target stands, so a loaded sag of 36 mm (above a) does not lower it."""
    tl = lift()
    for _ in range(int(20 / DT)):
        tl.step(True, cmd(1), armed=True, dirs={-1, 1}, measured=0.244, start_from=0.0)  # at 0.28, a 36 mm sag
    assert tl.step(True, cmd(0), armed=True, dirs={-1, 1}, measured=0.244, start_from=None) == pytest.approx(0.28)


def test_a_loaded_manual_lift_holds_within_the_loaded_tolerance():
    """Main 2026-10-19: lowering a pallet by hand (recovery: lower only) with a 63 mm loaded sag -- the stretch's end
    keeps the target within the loaded tolerance instead of dropping the load by the sag beyond a."""
    tl = lift()
    tl.loaded_tolerance_m = 0.073
    for _ in range(30):
        tl.step(True, cmd(-1), armed=True, dirs={-1}, measured=0.037, start_from=0.10, loaded=True)
    out = tl.step(True, cmd(0), armed=True, dirs={-1}, measured=0.037, start_from=None, loaded=True)
    assert out == pytest.approx(0.10 - 30 * 0.02 * DT)
