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
                          axle_accel_mps2=0.8, state_latency_s=0.02, max_age_s=0.03, dt_min_s=0.01, dt_max_s=0.025)
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
    # error width: wheel accel x (dt/2 + age + latency) + one count over dt
    a_w = 0.8 * (1 + 0.36 * TRACK / 2)
    e = a_w * (0.01 + 0.0 + 0.02) + 2 * math.pi * R / (CPR * 0.02)
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
    ((st(80, 0, 0), st(100, 10, 10, rx_ms=60)), "stale"),  # received 40 ms before now
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
    assert p.axle_accel_mps2 == 0.8 and p.state_latency_s == 0.02 and p.radius_m == veh.wheel_radius_m
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


def test_a_late_state_ages_by_its_delay_over_the_best_case():
    tr = rt.DelayTracker()
    early = st(80, 0, 0, rx_ms=81)  # 1 ms after its MCU time: the best case
    late = st(100, 0, 0, rx_ms=125)  # 25 ms after its MCU time: 24 ms late
    tr.add(early)
    tr.add(late)
    assert tr.excess_s(late) == pytest.approx(0.024, abs=1e-5)  # less the clock-drift allowance (4 us)
    now = late.host_ns + 1_000_000
    fresh = rt.encoder_speed([early, late], now, P, ok)
    aged = rt.encoder_speed([early, late], now, P, ok, tr.excess_s(late))
    assert aged.age_s == pytest.approx(fresh.age_s + 0.024, abs=1e-5) and aged.speed.bound_mps > fresh.speed.bound_mps
    # later than the age limit allows: no safety speed at all
    assert not rt.encoder_speed([early, late], now, P, ok, 0.05).speed.valid


def test_a_sustained_delay_never_becomes_the_new_best_case():
    """Codex impl P1-1 and re-review P1: 1 ms once, then 80 ms -- still 79 ms late after 6 s and after 600 s."""
    tr = rt.DelayTracker()
    tr.add(st(0, 0, 0, rx_ms=1))
    for k in range(1, 60_001):
        t = k * 10
        last = st(t, 0, 0, rx_ms=t + 80)
        tr.add(last)
        if k in (600, 60_000):
            assert tr.excess_s(last) == pytest.approx(0.079, abs=1e-6), k


def test_a_slow_mcu_clock_ages_the_states_the_safe_way():
    tr = rt.DelayTracker()
    last = None
    for k in range(0, 601):  # the offset grows 50 ppm of the elapsed time (an MCU clock 50 ppm slow)
        t = k * 10
        last = st(t, 0, 0, rx_ms=t + 1 + t * 50e-6)
        tr.add(last)
    assert tr.excess_s(last) == pytest.approx(0.0003, abs=1e-5)  # 6 s x 50 ppm, counted as delay
