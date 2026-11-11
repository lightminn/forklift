"""The teleop bridge's pure parts (docs/design/2026-11-12-teleop-bridge.md 2, 3, 7, 8): the host-differenced encoder
speed, the command cap, the direction rule (the core's real-truck branch), the real-truck safety overrides."""

import math
from types import SimpleNamespace

import pytest

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.live import real_teleop as rt
from forklift_core.safety.teleop_drive import TeleopLimits
from forklift_core.safety.teleop_policy import teleop_policy
from forklift_core.safety.truth_access import SafetySpeed

R, CPR, TRACK = 0.125, 327680.0, 0.53
P = rt.EncoderSpeedParams(radius_m=(R, R), counts_per_rev=(CPR, CPR), track_m=TRACK, kappa_max=0.36,
                          axle_accel_mps2=0.8, max_age_s=0.05, dt_min_s=0.01, dt_max_s=0.025)
NOW = 10_000_000_000


def st(t_ms, cl, cr, *, boot=7, faults=FaultFlag(0), rx_ms=None, steer=0.0):
    rx = NOW + int(((t_ms if rx_ms is None else rx_ms) - 100) * 1e6)
    return Sample(rx, McuState(int(t_ms * 1000), boot, 1, 0, Mode.ARMED, RejectReason.NONE, faults, cl, cr, 0.0,
                               0.0, steer, 0.0, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag(0)))


def counts(v, dt_s):
    return int(round(v * dt_s * CPR / (2 * math.pi * R)))


def ok(q):
    return True


def run(*samples, now=NOW):
    return rt.encoder_speed(list(samples), now, P, ok)


def test_speed_is_the_host_difference_of_two_states():
    c = counts(0.10, 0.02)
    r = run(st(80, 0, 0), st(100, c, c))
    assert r.speed.valid and r.speed.signed_mps == pytest.approx(0.10, abs=1e-4)
    assert r.yaw_rate == pytest.approx(0.0, abs=1e-6) and r.dt_s == pytest.approx(0.02)
    # error width: wheel accel x (dt/2 + age) + one count over dt (age: receipt age + the delay bound, here 0)
    a_w = 0.8 * (1 + 0.36 * TRACK / 2)
    e = a_w * (0.01 + 0.0) + 2 * math.pi * R / (CPR * 0.02)
    assert r.speed.bound_mps == pytest.approx(abs(r.speed.signed_mps) + e, rel=1e-6)
    assert r.speed.directions == (1,)


def test_the_bound_is_the_outer_wheel_and_yaw_the_wheel_difference():
    cl, cr = counts(0.05, 0.02), counts(0.15, 0.02)
    r = run(st(80, 0, 0), st(100, cl, cr))
    assert r.speed.signed_mps == pytest.approx(0.10, abs=1e-4)
    assert r.yaw_rate == pytest.approx((0.15 - 0.05) / TRACK, abs=1e-3)
    assert r.speed.bound_mps > 0.15  # the outer wheel's, not the mean's


def test_opposite_wheels_are_not_a_stand():
    c = counts(0.08, 0.02)
    r = run(st(80, 0, 0), st(100, -c, c))
    assert abs(r.speed.signed_mps) < 1e-6 and r.speed.bound_mps > 0.08
    assert r.speed.directions == (1, -1)


def test_a_slow_reading_spans_both_directions():
    r = run(st(80, 0, 0), st(100, 0, 0))
    assert r.speed.valid and r.speed.signed_mps == 0.0 and r.speed.directions == (1, -1)


def test_the_interval_takes_the_latest_state_far_enough_back():
    c = counts(0.1, 0.005)
    r = run(st(85, 0, 0), st(95, 2 * c, 2 * c), st(100, 3 * c, 3 * c))
    assert r.dt_s == pytest.approx(0.015)  # 95 is under dt_min from 100; 85 is the latest at or past it


@pytest.mark.parametrize("samples,why", [
    ((st(100, 0, 0),), "no_interval"),
    ((st(50, 0, 0), st(100, 10, 10)), "no_interval"),  # 50 ms apart: past dt_max
    ((st(80, 0, 0, boot=6), st(100, 10, 10)), "no_interval"),  # a new boot has no earlier sample
    ((st(80, 0, 0), st(100, 10, 10, faults=FaultFlag.ENCODER_GAP)), "encoder_fault"),
    ((st(80, 0, 0, faults=FaultFlag.ENCODER_STUCK_L), st(100, 10, 10)), "encoder_fault"),
    ((st(80, 0, 0), st(100, 10, 10, faults=FaultFlag.ENCODER_POLARITY_R)), "encoder_fault"),
    ((st(80, 0, 0), st(100, 10, 10, rx_ms=40)), "stale"),  # received 60 ms before now
])
def test_invalid_readings(samples, why):
    r = run(*samples)
    assert not r.speed.valid and r.reason == why


def test_a_steering_off_the_map_invalidates():
    r = rt.encoder_speed([st(80, 0, 0), st(100, 0, 0, steer=9.9)], NOW, P, lambda q: abs(q) < 1)
    assert not r.speed.valid and r.reason == "steering_invalid"


def test_the_command_cap_keeps_a_turning_outer_wheel_under_the_bound():
    cap = rt.command_cap_mps(0.2, P)
    worst = P.worst_error()
    assert cap == pytest.approx((0.2 - worst) / (1 + 0.36 * TRACK / 2))
    # the outer wheel at the cap and the largest curvature, read at the worst age and interval
    outer = cap * (1 + 0.36 * TRACK / 2)
    assert outer + worst <= 0.2 + 1e-12
    with pytest.raises(ValueError):
        rt.command_cap_mps(worst, P)


def test_the_teleop_limits_are_cut_to_the_cap():
    lim = rt.real_teleop_limits(TeleopLimits(), 0.13)
    assert lim.direct_forward_mps == lim.video_forward_mps == 0.13
    assert lim.direct_reverse_mps == lim.video_reverse_mps == 0.13 and lim.loaded_mps == 0.13
    assert lim.recovery_reverse_mps <= 0.13
    small = rt.real_teleop_limits(TeleopLimits(direct_reverse_mps=0.05, video_reverse_mps=0.05), 0.13)
    assert small.direct_reverse_mps == 0.05  # never raised


# --- the direction rule: the core's real-truck branch (mission/core.py _direction_limits) --------------------------

def core_rule(limit_fn, speed, direction, cap, commanded):
    from forklift_core.mission.core import MissionCore

    stub = SimpleNamespace(_real_vehicle=lambda: True, _commanded_now=lambda: commanded)
    return MissionCore._direction_limits(stub, limit_fn, speed, direction, cap)


def limits(fwd, back):
    return lambda d, v: ((fwd if d > 0 else back), "ok" if (fwd if d > 0 else back) >= v else f"blocked_{d}")


CASES = [
    (SafetySpeed(-0.005, 0.012, (1, -1)), 1, 0.016, 0.0, limits(1.0, 0.0)),  # Codex P1-1's counterexample
    (SafetySpeed(-0.05, 0.05, (-1,)), 1, 0.15, -0.15, limits(1.0, 0.08)),  # Codex re-review P1-2's
    (SafetySpeed(0.10, 0.13, (1,)), 1, 0.13, 0.10, limits(1.0, 0.0)),
    (SafetySpeed(0.0, 0.004, (1, -1)), None, 0.0, 0.0, limits(0.5, 0.5)),
    (SafetySpeed(0.08, 0.11, (1,)), -1, 0.05, 0.08, limits(0.5, 0.2)),
]


@pytest.mark.parametrize("speed,direction,cap,commanded,fn", CASES)
def test_the_direction_rule_is_the_cores(speed, direction, cap, commanded, fn):
    assert rt.direction_limits(fn, speed, direction, cap, commanded) == core_rule(fn, speed, direction, cap, commanded)


def test_the_two_counterexamples_hold():
    a, _w, hold, *_ = rt.direction_limits(CASES[0][4], *CASES[0][:4])
    assert a == 0.0 and hold
    a, _w, hold, *_ = rt.direction_limits(CASES[1][4], *CASES[1][:4])
    assert a == 0.0 and hold


# --- safety state (design 7) ---------------------------------------------------------------------------------------

ALL = frozenset(rt.CONFIRMATIONS)


def test_the_bench_safety_is_unknown_unless_all_four_are_confirmed():
    for missing in rt.CONFIRMATIONS:
        s = rt.bench_safety(ALL - {missing})
        assert teleop_policy(s, feedback_ok=True, standing=True).reason == "state_unknown"
    s = rt.bench_safety(ALL)
    p = teleop_policy(s, feedback_ok=True, standing=True)
    assert p.drive_allowed and s["frame"]["slam"] is False


@pytest.mark.parametrize("change,reason", [
    ({"load": {"loaded": True}}, "real_loaded"),
    ({"recovery_required": True}, "real_recovery_required"),
    ({"fork_in_pocket": {"value": True, "relation": "carrying"}}, "real_in_pocket"),
])
def test_the_real_truck_overrides_come_before_the_policy(change, reason):
    s = rt.enforce_real({**rt.bench_safety(ALL), **change})
    assert s["teleop_refused"] == reason
    assert not teleop_policy(s, feedback_ok=True, standing=True).drive_allowed


def test_a_recovery_context_cannot_open_the_straight_reverse():
    """Codex re-review P1-3: the policy alone opens reverse_straight for this picture."""
    s = {**rt.bench_safety(ALL), "recovery_required": True,
         "fork_in_pocket": {"value": True, "relation": "constrained"},
         "recovery": {"stage": "in_pocket", "grounded": True, "evidence": True, "aligned": True}}
    plain = teleop_policy(s, feedback_ok=True, standing=True, lift_m=0.0, steering_rad=0.0)
    assert plain.drive_allowed and plain.drive_mode == "reverse_straight"  # the premise
    real = teleop_policy(rt.enforce_real(s), feedback_ok=True, standing=True, lift_m=0.0, steering_rad=0.0)
    assert not real.drive_allowed and not real.lift_allowed


def test_unknown_stays_unknown():
    assert rt.enforce_real(None) is None


def test_the_repository_config_loads_and_gives_a_command_cap():
    from pathlib import Path

    from forklift_core.plant.real_vehicle import load_real_vehicle

    repo = Path(__file__).resolve().parents[3]
    veh = load_real_vehicle(repo / "config" / "real" / "vehicle.yaml")
    p = rt.load_bridge_params(repo / "config" / "real" / "teleop_bridge.yaml", veh)
    assert p.axle_accel_mps2 == 0.8 and p.max_age_s == 0.05 and p.radius_m == veh.wheel_radius_m
    cap = rt.command_cap_mps(veh.teleop_speed_limit_mps, p)
    assert 0.10 < cap < 0.15  # the provisional cap under the 0.2 bound (plan's table: about 0.14 or less)


def test_the_config_is_strict(tmp_path):
    from forklift_core.plant.real_vehicle import load_real_vehicle
    from pathlib import Path

    repo = Path(__file__).resolve().parents[3]
    veh = load_real_vehicle(repo / "config" / "real" / "vehicle.yaml")
    bad = tmp_path / "b.yaml"
    bad.write_text("schema: teleop_bridge/1\nencoder_speed: {axle_accel_mps2: 0.8}\n")
    with pytest.raises(ValueError):
        rt.load_bridge_params(bad, veh)


def test_the_output_check_takes_the_operators_direction_not_the_slews():
    """Codex round 3 P1: old command -0.15, new request +0.15, the slew's output still -0.08, bound 0.05, the rear
    allows 0.08 -- the output direction would pass it; the request direction holds (the core's result)."""
    speed = SafetySpeed(-0.05, 0.05, (-1,))
    fn = limits(1.0, 0.08)
    by_output = rt.direction_limits(fn, speed, -1, 0.08, -0.15)
    assert by_output[0] == 0.08 and not by_output[2]  # the wrong reading the check must not use
    out, why = rt.output_direction_check(fn, speed, requested_mps=0.15, output_mps=-0.08, commanded_mps=-0.15)
    assert out == 0.0 and why == "blocked_-1"
    assert core_rule(fn, speed, 1, 0.08, -0.15)[2] is True  # the core holds too


def test_the_output_check_passes_a_clear_drive():
    out, why = rt.output_direction_check(limits(1.0, 1.0), SafetySpeed(0.1, 0.13, (1,)), 0.13, 0.12, 0.1)
    assert out == 0.12 and why is None




def _with(sample, **kw):
    return Sample(kw.pop("host_ns", sample.host_ns), McuState(**{**sample.state.__dict__, **kw}))


def test_the_state_after_an_acknowledgement_is_bounded_by_the_round_trip():
    """Frame 7 written; the STATE whose last_rx_seq moves to 7 may have been measured before 7 was handled -- the
    next STATE was not: its delay is at most its receipt minus the write."""
    db = rt.DelayBound()
    db.written(7, NOW - 30_000_000)
    db.add(_with(st(80, 0, 0), host_ns=NOW - 20_000_000, last_rx_seq=6))
    ack = _with(st(90, 0, 0), host_ns=NOW - 10_000_000, last_rx_seq=7)
    db.add(ack)
    assert db.delay_s(ack) is None  # the acknowledging STATE itself is no anchor
    nxt = _with(st(100, 0, 0), host_ns=NOW, last_rx_seq=7)
    db.add(nxt)
    assert db.delay_s(nxt) == pytest.approx(0.030)
    later = _with(nxt, host_ns=NOW + 15_000_000, mcu_time_us=nxt.state.mcu_time_us + 10_000)
    assert db.delay_s(later) == pytest.approx(0.030 + 0.005 + 100e-6 / (1 - 100e-6) * 0.010, abs=1e-9)


def test_the_initial_seq_is_no_acknowledgement():
    """Codex re-review 4 P1: a late STATE still naming the MCU's initial seq gives no bound."""
    db = rt.DelayBound()
    db.written(0, NOW - 10_000_000)
    db.add(_with(st(100, 0, 0), host_ns=NOW, last_rx_seq=0))
    late = _with(st(110, 0, 0), host_ns=NOW + 2_000_000, last_rx_seq=0)
    db.add(late)
    assert db.delay_s(late) is None


def test_an_earlier_sessions_late_states_are_not_this_sessions_acknowledgements():
    """Codex re-review 5 P1: a restarted host on the same MCU boot; the old session's late STATEs move 0 -> 1 -> 2.
    This session starts at a random seq, so those moves name no frame it wrote."""
    import secrets as _secrets

    start = 40_000
    db = rt.DelayBound()
    for k in range(3):
        db.written(start + k, NOW - 5_000_000 + k)
    for k, seq in enumerate((0, 1, 2, 2)):  # 130 ms late STATEs of the old session
        db.add(_with(st(100 + 10 * k, 0, 0), host_ns=NOW + k * 10_000_000, last_rx_seq=seq))
    assert db.delay_s(_with(st(130, 0, 0), host_ns=NOW + 30_000_000, last_rx_seq=2)) is None
    assert 0 <= _secrets.randbelow(0x10000) < 0x10000  # the CLI's start (tools/teleop_bridge.py)


def test_a_cycle_that_ran_long_after_its_measurement_cannot_shrink_the_bound():
    """Codex re-review 5 P2: the acknowledging cycle measured 4.1 ms before its STATE; the bound comes from the next
    STATE, measured after the frame was handled, whatever the cycle took."""
    db = rt.DelayBound()
    w_ = NOW - 2_000_000
    db.written(9, w_)
    db.add(_with(st(80, 0, 0), host_ns=NOW - 12_000_000, last_rx_seq=8))
    ack = _with(st(90, 0, 0), host_ns=NOW + 2_100_000, last_rx_seq=9)  # measured at about w_ - 2 ms
    db.add(ack)
    nxt = _with(st(100, 0, 0), host_ns=NOW + 10_500_000, last_rx_seq=9)
    db.add(nxt)
    measured_ack = w_ - 2_000_000
    assert db.delay_s(ack) is None or db.delay_s(ack) * 1e9 >= ack.host_ns - measured_ack
    assert db.delay_s(nxt) == pytest.approx((nxt.host_ns - w_) / 1e9)


def test_the_drift_allowance_runs_on_the_measurement_interval():
    """Codex re-review 4 P2 (re-review 5 P3: with a real, positive delay). The anchor measured at host 0 and received
    1.000 s later; j measured 1.910 s later on the host (the MCU clock 100 ppm fast), received 1 ms after -- the delay
    fell from 1 s to 1 ms. The receipt interval (0.911 s) would undercount the drift; the MCU interval does not."""
    db = rt.DelayBound()
    db.written(5, 0)
    db.add(_with(st(0, 0, 0), host_ns=1, mcu_time_us=0, last_rx_seq=4))
    db.add(_with(st(0, 0, 0), host_ns=2, mcu_time_us=1, last_rx_seq=5))  # acknowledges 5 (written at 0)
    a = _with(st(0, 0, 0), host_ns=1_000_000_000, mcu_time_us=2, last_rx_seq=5)  # measured ~0, received 1 s later
    db.add(a)
    t_meas_j = 1_910_000_000
    rx_j = t_meas_j + 1_000_000
    j = _with(a, host_ns=rx_j, mcu_time_us=int(t_meas_j * (1 + 100e-6) / 1000))
    true_delay = (rx_j - t_meas_j) / 1e9
    assert true_delay == pytest.approx(0.001) and db.delay_s(j) >= true_delay


def test_no_anchor_no_bound_no_speed():
    db = rt.DelayBound()
    s1 = st(100, 0, 0)
    db.add(s1)  # names seq 0, never written
    assert db.delay_s(s1) is None
    assert rt.encoder_speed([st(80, 0, 0), s1], s1.host_ns, P, ok, None).reason == "no_delay_bound"


def _true_world(drift_ppm, delay_fn, seconds=1200.0):
    """Host time T. The MCU clock runs (1 + drift) as fast. STATEs every 10 ms of MCU time, each measured at T_m
    and received at T_m + delay_fn(T_m). A frame is written every 20 ms and taken by the MCU 1 ms later; each STATE
    names the newest frame taken by its measurement. Returns (bound, true delay) pairs along the way."""
    db = rt.DelayBound()
    drift = drift_ppm * 1e-6
    frames, seq, next_write, t_m, out = [], 0, 0, 0, []
    k = 0
    while t_m < seconds * 1e9:
        while next_write <= t_m:
            frames.append((seq, next_write, next_write + 1_000_000))
            db.written(seq, next_write)
            seq = (seq + 1) & 0xFFFF
            next_write += 20_000_000
        taken = [f for f in frames if f[2] <= t_m]
        named = taken[-1][0] if taken else 0xFFFF
        d = int(delay_fn(t_m))
        mcu_us = int(t_m * (1 + drift) / 1000) + 1_000_000
        smp = Sample(t_m + d, McuState(mcu_us, 7, 1, named, Mode.ARMED, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0,
                                       0.0, 0.0, 0.0, (0,) * 4, (0,) * 4, (0xC0,) * 4, 0, 0, StateFlag(0)))
        db.add(smp)
        if k % 997 == 0:
            out.append((db.delay_s(smp), d / 1e9))
        frames = frames[-8:]
        t_m += int(10_000_000 / (1 + drift))
        k += 1
    return out


@pytest.mark.parametrize("drift_ppm", [50.0, -50.0, 0.0])
def test_the_bound_never_falls_below_the_true_delay(drift_ppm):
    """Codex re-review 3 P1: a fast (or slow) MCU clock and a delay growing from 1 to 61 ms over 1200 s."""
    pairs = _true_world(drift_ppm, lambda t: 1_000_000 + 60_000_000 * t / 1.2e12)
    assert pairs[0][0] is None  # before the first frame is acknowledged: no bound (no safety speed), the safe side
    rest = pairs[1:]
    assert len(rest) > 100 and all(b is not None and b >= true - 1e-9 for b, true in rest), rest[-3:]
    assert pairs[-1][0] < pairs[-1][1] + 0.025  # and not uselessly loose: about a STATE period plus the link
