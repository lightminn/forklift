"""The real truck's safety watch (docs/design/2026-11-16-real-safety-watch.md): motor current, per-wheel target
against measurement, tilt -- judged while armed on fresh new STATEs, a reason past each hold."""

import math
from pathlib import Path

import pytest

from forklift_core.safety import real_watch as rw

REPO = Path(__file__).resolve().parents[3]
CFG = rw.load_watch_config(REPO / "config" / "real" / "safety_watch.yaml")
R = (0.05, 0.05)  # wheel radius (m) for the tests
W = rw.SafetyWatch(CFG, wheel_radius_m=R)


def mps(v, r=0.05):
    return v / r


def run(watch, seconds, *, dt=0.01, t0=0.0, **kw):
    """Steps of ``dt`` from ``t0`` for ``seconds``; the first reason seen and the time of the last step."""
    t, first = t0, None
    n = int(round(seconds / dt))
    for _ in range(n):
        t = round(t + dt, 9)
        res = watch.step(t, **kw)
        if res.trip and first is None:
            first = (res.trip, t)
    return first, t


def base(**over):
    kw = dict(armed=True, fresh=True, new_state=True, currents_a=(0.5, 0.5, 0.2, 0.0),
              wheel_targets_rad_s=(mps(0.1), mps(0.1)), wheel_rates_rad_s=(mps(0.1), mps(0.1)))
    kw.update(over)
    return kw


def test_config_values_carry_provenance_and_are_provisional():
    import yaml
    raw = yaml.safe_load((REPO / "config" / "real" / "safety_watch.yaml").read_text())
    for part in ("current", "wheel", "tilt"):
        assert raw[part]["provenance"]["measured"] is False and raw[part]["provenance"]["until"]
    assert CFG.current_limit_a == (3.0, 3.0, 3.0, 3.0) and CFG.current_hold_s == 0.5


def test_normal_driving_never_trips():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 3.0, **base())
    assert first is None


def test_a_current_held_over_the_limit_trips_after_the_hold_only():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 0.45, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert first is None
    first, _ = run(w, 0.1, t0=0.45, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert first is not None and first[0] == "watch_current"


def test_a_short_start_current_does_not_trip():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, t = run(w, 0.3, **base(currents_a=(4.5, 4.5, 0.2, 0.0)))  # start-up spike shorter than the hold
    first2, _ = run(w, 2.0, t0=t, **base())
    assert first is None and first2 is None


def test_any_channel_counts_including_the_lift_and_the_sign():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 0.6, **base(currents_a=(0.5, 0.5, 0.2, -3.4)))
    assert first[0] == "watch_current"


def test_one_wheel_stopped_while_steering_trips_the_wheel_watch():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    # turning: outer 0.18, inner 0.08 m/s asked; the inner wheel does not turn
    first, _ = run(w, 0.6, **base(wheel_targets_rad_s=(mps(0.08), mps(0.18)), wheel_rates_rad_s=(0.0, mps(0.18))))
    assert first[0] == "watch_wheel"


def test_turning_with_different_wheel_speeds_followed_is_fine():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 2.0, **base(wheel_targets_rad_s=(mps(0.08), mps(0.18)),
                                  wheel_rates_rad_s=(mps(0.07), mps(0.19))))
    assert first is None


def test_rolling_while_the_target_is_zero_trips():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 0.6, **base(wheel_targets_rad_s=(0.0, 0.0), wheel_rates_rad_s=(mps(0.08), mps(0.08))))
    assert first[0] == "watch_wheel"


def test_both_under_the_floor_are_not_judged():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 2.0, **base(wheel_targets_rad_s=(mps(0.02), 0.0), wheel_rates_rad_s=(0.0, mps(0.025))))
    assert first is None


def test_the_accel_lag_of_a_ramp_does_not_trip():
    """0 -> 0.2 m/s at 0.8 m/s^2 with the measurement one 20 ms cycle behind, then the reverse ramp."""
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    t, first = 0.0, None
    for k in range(200):
        t = round(t + 0.01, 9)
        tgt = min(0.2, 0.8 * t) if t < 1.0 else max(-0.2, 0.2 - 0.8 * (t - 1.0))
        meas = min(0.2, 0.8 * max(0.0, t - 0.02)) if t < 1.02 else max(-0.2, 0.2 - 0.8 * (t - 1.02))
        res = w.step(t, **base(wheel_targets_rad_s=(mps(tgt), mps(tgt)), wheel_rates_rad_s=(mps(meas), mps(meas))))
        first = first or res.trip
    assert first is None


def test_disarm_clears_the_holds():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    run(w, 0.4, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    w.step(0.41, **base(armed=False, currents_a=(3.5, 0.5, 0.2, 0.0)))
    first, _ = run(w, 0.4, t0=0.41, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert first is None  # the hold starts again after the arm


def test_not_armed_never_trips():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 2.0, **base(armed=False, currents_a=(9.0,) * 4))
    assert first is None


def test_a_stale_link_clears_and_a_repeated_state_does_not_accumulate():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    run(w, 0.4, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    w.step(0.41, **base(fresh=False, currents_a=(3.5, 0.5, 0.2, 0.0)))  # the link went stale: cleared
    first, _ = run(w, 0.4, t0=0.41, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert first is None
    w2 = rw.SafetyWatch(CFG, wheel_radius_m=R)
    w2.step(0.01, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    first, _ = run(w2, 1.0, t0=0.01, **base(new_state=False, currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert first is None  # no new STATE: nothing judged, no hold reached


def test_a_trip_is_counted_and_the_hold_restarts():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, t = run(w, 0.6, **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert first[0] == "watch_current" and w.counts == {"watch_current": 1}
    res = w.step(round(t + 0.01, 9), **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert res.trip is None  # not every cycle after the trip
    run(w, 0.6, t0=round(t + 0.01, 9), **base(currents_a=(3.5, 0.5, 0.2, 0.0)))
    assert w.counts["watch_current"] == 2


def test_the_record_values_name_the_worst_channel_and_wheel():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    res = w.step(0.01, **base(currents_a=(1.0, 2.0, 0.2, 0.0), wheel_targets_rad_s=(mps(0.1), mps(0.1)),
                              wheel_rates_rad_s=(mps(0.1), mps(0.07))))
    assert res.values["current_max_a"] == pytest.approx(2.0) and res.values["current_channel"] == 1
    assert res.values["wheel_error_mps"] == pytest.approx(0.03) and res.values["wheel"] == 1


def test_missing_inputs_are_not_judged():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 1.0, **base(currents_a=None, wheel_rates_rad_s=None))
    assert first is None


# --- tilt (an IMU owner hands samples; the teleop bridge does not) -----------------------------------------------

G = 9.81


def acc(deg):
    a = math.radians(deg)
    return (0.0, G * math.cos(a), G * math.sin(a))  # SDK axes: y down; a pitch shows on z


def test_without_samples_tilt_is_not_judged():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, _ = run(w, 2.0, **base())
    assert first is None


def test_a_tilt_over_the_limit_from_the_arm_baseline_trips():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    first, t = run(w, 0.2, **base(accel=acc(2.0), accel_t_s=None, imu_expected=True), )
    assert first is None  # baseline at the arm: a 2 deg mount tilt is zero
    t0 = t
    first = None
    for k in range(40):
        tt = round(t0 + 0.01 * (k + 1), 9)
        res = w.step(tt, **base(accel=acc(11.0), accel_t_s=tt, imu_expected=True))
        first = first or res.trip
    assert first == "watch_tilt"


def test_a_short_tilt_does_not_trip_and_a_stale_imu_does():
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    for k in range(10):
        t = round(0.01 * (k + 1), 9)
        assert w.step(t, **base(accel=acc(0.0), accel_t_s=t, imu_expected=True)).trip is None
    for k in range(20):  # 0.2 s over the limit, shorter than the 0.3 s hold
        t = round(0.1 + 0.01 * (k + 1), 9)
        assert w.step(t, **base(accel=acc(12.0), accel_t_s=t, imu_expected=True)).trip is None
    last = 0.3
    first = None
    for k in range(70):  # samples stop arriving
        t = round(last + 0.01 * (k + 1), 9)
        res = w.step(t, **base(accel=None, accel_t_s=last, imu_expected=True))
        first = first or res.trip
    assert first == "watch_imu_stale"


def test_a_gap_between_states_restarts_the_hold():
    """High current, then no STATE for 0.6 s, then one more: the gap is not counted as held (Codex P2)."""
    w = rw.SafetyWatch(CFG, wheel_radius_m=R)
    hot = base(currents_a=(3.5, 0.5, 0.2, 0.0))
    first, t = run(w, 0.1, **hot)
    first2, t = run(w, 0.6, t0=t, **{**hot, "new_state": False})
    res = w.step(round(t + 0.01, 9), **hot)
    assert first is None and first2 is None and res.trip is None
    first3, _ = run(w, 0.6, t0=round(t + 0.01, 9), **hot)
    assert first3[0] == "watch_current"  # held again on consecutive STATEs


def test_a_skipped_check_is_not_judged_and_does_not_hide_the_other():
    """Real safety wiring design 5 (5c-1 r1 P2-6): with watch_current skipped (bypassed), a wheel fault on the same
    STATEs still trips -- the current check does not take the trip or clear the wheel's hold."""
    from forklift_core.safety.real_watch import SafetyWatch

    cfg = CFG
    w = SafetyWatch(cfg, wheel_radius_m=(0.05, 0.05))
    trips = []
    for k in range(40):
        t = k * 0.02
        res = w.step(t, armed=True, fresh=True, new_state=True, currents_a=(9.0, 9.0, 0.0, 0.0),
                     wheel_targets_rad_s=(10.0, 10.0), wheel_rates_rad_s=(0.0, 0.0), skip={"watch_current"})
        trips.append(res.trip)
    assert "watch_wheel" in trips and "watch_current" not in trips
    w2 = SafetyWatch(cfg, wheel_radius_m=(0.05, 0.05))
    for k in range(40):
        res = w2.step(k * 0.02, armed=True, fresh=True, new_state=True, currents_a=(9.0, 9.0, 0.0, 0.0),
                      wheel_targets_rad_s=(10.0, 10.0), wheel_rates_rad_s=(0.0, 0.0),
                      skip={"watch_current", "watch_wheel"})
        assert res.trip is None
