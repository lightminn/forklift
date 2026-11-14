"""G8 edge cases (docs/design/2026-11-14-profile-drive-and-m-analysis.md 3) for every public function of
``forklift_core.plant.m_analysis``, on synthetic records from ``m_synth`` (same folder). Each expectation follows from
the function's docstring or the design; where those and the module part, the test is a strict xfail naming the
suspected defect."""

from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant import m_analysis as ma
from forklift_core.sensors import imu_mount

_HERE = str(Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from m_synth import (  # noqa: E402
    NS,
    drive_states,
    imu_series,
    ns,
    odometry_cfg,
    odometry_distance_factor,
    profile_row,
    profile_rows,
    unit,
    wrap_i32,
)

UP = unit([0.08, -0.99, 0.11])  # a slightly tilted mount
BIAS = np.array([0.003, -0.002, 0.001])
CFG = odometry_cfg()


# --- trials --------------------------------------------------------------------------------------------------------

def test_a_record_without_profile_rows_has_no_trials():
    assert ma.trials([]) == []
    rows = [{"t": "state", "host_ns": 1}, {"t": "marker", "label": "profile:1:arc:start", "host_ns": 2},
            {"t": "profile_start", "host_ns": 3}, {"t": "profile_end", "trial": 1, "result": "done"}]
    assert ma.trials(rows) == [] and ma.trials(rows, valid_only=True) == []


def test_a_trial_cut_off_before_its_result_is_kept_but_not_valid():
    rows = profile_rows(4, [("presteer", 0.0, 1.0), ("still_pre", 1.02, 3.0), ("arc", 3.02, 5.0)])
    (tr,) = ma.trials(rows)
    assert tr.trial == 4 and tr.result is None and not tr.valid
    assert tr.spans == {"presteer": (0, ns(1.0)), "still_pre": (ns(1.02), ns(3.0)), "arc": (ns(3.02), ns(5.0))}
    assert ma.trials(rows, valid_only=True) == []


def test_a_trial_void_at_its_first_cycle_has_no_spans():
    rows = [profile_row(5, "presteer", ns(2.0), state="zeroing", result="void:disarmed"),
            profile_row(5, None, ns(2.02), state="zeroing", result="void:disarmed")]
    (tr,) = ma.trials(rows)
    assert tr.result == "void:disarmed" and not tr.valid and tr.spans == {}


def test_interleaved_trials_are_told_apart_and_the_ending_cycle_is_in_no_span():
    t1 = profile_rows(1, [("presteer", 0.0, 1.0), ("still_pre", 1.02, 4.0), ("arc", 4.02, 9.0),
                          ("still_post", 9.02, 12.0)], result="done")
    t2 = profile_rows(2, [("presteer", 20.0, 21.0), ("still_pre", 21.02, 24.0), ("arc", 24.02, 25.0)],
                      result="void:limited:no_scan")
    t3 = profile_rows(3, [("presteer", 30.0, 31.0), ("arc", 31.02, 33.0)], result="done")
    other = [{"t": "state", "host_ns": ns(x)} for x in np.arange(0.0, 35.0, 0.5)]
    other.append({"t": "profile_end", "trial": 2, "result": "void:limited:no_scan"})
    rows = [r for group in itertools.zip_longest(t3, t1, other, t2) for r in group if r is not None]
    found = ma.trials(rows)
    assert [t.trial for t in found] == [1, 2, 3]
    a, b, c = found
    assert (a.result, b.result, c.result) == ("done", "void:limited:no_scan", "done")
    assert [t.trial for t in ma.trials(rows, valid_only=True)] == [1, 3]
    assert a.spans["still_post"] == (ns(9.02), ns(12.0))  # the zeroing row at 12.02 that ended it is not in it
    assert b.spans["arc"] == (ns(24.02), ns(25.0))  # nor the row at 25.02 that voided it
    assert c.spans == {"presteer": (ns(30.0), ns(31.0)), "arc": (ns(31.02), ns(33.0))}


# --- quiet_window --------------------------------------------------------------------------------------------------

def _standing(t1_s=25.0, seed=1):
    return imu_series(0.0, t1_s, 200, lambda x: 0.0, up=UP, bias=BIAS, noise=0.001, accel_noise=0.005, seed=seed)


@pytest.mark.parametrize("boundary_s, side", [(-1.0, "before"), (100.0, "before"), (100.0, "after"),
                                              (-20.0, "after")])
def test_no_samples_in_range_is_no_window(boundary_s, side):
    t, w, a = _standing()
    assert ma.quiet_window(t, w, a, boundary_ns=ns(boundary_s), side=side, max_s=15.0, min_s=0.0,
                           standing=None) is None


def test_an_unknown_side_is_refused():
    t, w, a = _standing()
    with pytest.raises(ValueError):
        ma.quiet_window(t, w, a, boundary_ns=ns(10.0), side="around", max_s=5.0, min_s=1.0, standing=None)


def test_a_window_that_keeps_moving_has_no_quiet_run():
    t, w, a = imu_series(0.0, 25.0, 200, lambda x: 0.3 * math.sin(math.pi * x), up=UP, bias=BIAS, noise=0.001,
                         accel_fn=lambda x: math.sin(math.pi * x + 0.3), seed=3)
    assert ma.quiet_window(t, w, a, boundary_ns=ns(20.0), side="before", max_s=15.0, min_s=2.0,
                           standing=None) is None
    assert ma.quiet_window(t, w, a, boundary_ns=ns(20.0), side="after", max_s=5.0, min_s=2.0,
                           standing=None) is None


def test_a_steady_turn_is_not_taken_for_a_standstill():
    t, w, a = imu_series(0.0, 25.0, 200, lambda x: 0.2, up=UP, bias=BIAS, noise=0.001, accel_fn=lambda x: 0.02,
                         seed=4)
    kw = {"boundary_ns": ns(20.0), "side": "before", "max_s": 15.0, "min_s": 5.0}
    # the encoder standstill intervals all lie outside the window: nothing counts, however steady the IMU is
    assert ma.quiet_window(t, w, a, standing=[(0, ns(3.0))], **kw) is None
    assert ma.quiet_window(t, w, a, standing=[], **kw) is None


def test_standing_intervals_cut_the_window_to_where_the_encoders_stand():
    t, w, a = _standing()
    s, e = ma.quiet_window(t, w, a, boundary_ns=ns(20.0), side="before", max_s=15.0, min_s=2.0,
                           standing=[(ns(8.0), ns(12.0))])
    assert t[s] >= ns(8.0) and t[e - 1] <= ns(12.0)


def test_quiet_window_requires_the_standing_keyword():
    t, w, a = _standing()
    with pytest.raises(TypeError):
        ma.quiet_window(t, w, a, boundary_ns=ns(10.0), side="before", max_s=5.0, min_s=1.0)


def test_a_boundary_on_the_first_sample():
    t, w, a = _standing()
    b = int(t[0])
    # a single sample cannot give a reference (the stillest chunk needs 5 samples): refused, not a one-sample window
    assert ma.quiet_window(t, w, a, boundary_ns=b, side="before", max_s=15.0, min_s=0.0, standing=None) is None
    assert ma.quiet_window(t, w, a, boundary_ns=b, side="before", max_s=15.0, min_s=0.5, standing=None) is None
    s, e = ma.quiet_window(t, w, a, boundary_ns=b, side="after", max_s=5.0, min_s=2.0, standing=None)
    assert s == 0 and t[e - 1] == b + 5 * NS


def test_a_boundary_on_the_last_sample():
    t, w, a = _standing()
    n, b = len(t), int(t[-1])
    s, e = ma.quiet_window(t, w, a, boundary_ns=b, side="before", max_s=15.0, min_s=5.0, standing=None)
    assert e == n and t[s] == b - 15 * NS
    assert ma.quiet_window(t, w, a, boundary_ns=b, side="after", max_s=15.0, min_s=0.0, standing=None) is None
    assert ma.quiet_window(t, w, a, boundary_ns=b, side="after", max_s=15.0, min_s=0.5, standing=None) is None


def test_before_the_boundary_the_closest_run_long_enough_is_taken():
    def bump(x):
        return 0.15 if 17.0 <= x < 17.3 else 0.0

    t, w, a = imu_series(0.0, 25.0, 200, bump, up=UP, bias=BIAS, noise=0.001, accel_noise=0.005,
                         accel_fn=lambda x: 0.4 if 17.0 <= x < 17.3 else 0.0, creep=(20.0, 0.5, 0.1, 0.3), seed=6)
    s, e = ma.quiet_window(t, w, a, boundary_ns=ns(20.0), side="before", max_s=15.0, min_s=5.0, standing=None)
    assert (t[s], t[e - 1]) == (ns(5.0), ns(16.995))  # the 2.2 s run after the bump is too short
    s, e = ma.quiet_window(t, w, a, boundary_ns=ns(20.0), side="before", max_s=15.0, min_s=2.0, standing=None)
    assert (t[s], t[e - 1]) == (ns(17.3), ns(19.495))  # the creep before the boundary is cut off


def test_after_the_boundary_the_end_of_the_drive_is_cut_off():
    t, w, a = imu_series(0.0, 40.0, 200, lambda x: 0.2 if 20.0 <= x < 20.5 else 0.0, up=UP, bias=BIAS,
                         noise=0.001, accel_noise=0.005, accel_fn=lambda x: -0.5 if 20.0 <= x < 20.5 else 0.0,
                         seed=7)
    s, e = ma.quiet_window(t, w, a, boundary_ns=ns(20.0), side="after", max_s=15.0, min_s=5.0, standing=None)
    assert (t[s], t[e - 1]) == (ns(20.5), ns(35.0))
    assert np.allclose(w[s:e].mean(axis=0), BIAS, atol=2e-4)


# --- imu_heading, heading_sigma_lower, gyro_scale ------------------------------------------------------------------

def test_imu_heading_over_two_samples():
    t = np.array([0, 10_000_000], dtype=np.int64)
    w = np.outer([0.1, 0.3], UP) + BIAS
    assert ma.imu_heading(t, w, UP, BIAS) == pytest.approx([0.0, 0.2 * 0.01])


def test_imu_heading_is_a_trapezoid_over_uneven_samples():
    dt_ms = np.array([5, 5, 7, 3, 10, 5, 5, 2, 8])
    t = (NS + np.concatenate([[0], np.cumsum(dt_ms)]) * 1_000_000).astype(np.int64)
    rates = np.array([0.0, 0.1, 0.2, -0.1, 0.4, 0.4, 0.0, -0.3, 0.2, 0.1])
    h = ma.imu_heading(t, np.outer(rates, UP) + BIAS, UP, BIAS)
    expected = np.concatenate([[0.0], np.cumsum((rates[1:] + rates[:-1]) / 2 * np.diff(t) / NS)])
    assert h == pytest.approx(expected, abs=1e-12)


def test_imu_heading_of_a_clockwise_turn_falls():
    t, w, _ = imu_series(0.0, 10.0, 400, lambda x: -0.15, up=UP, bias=BIAS)
    h = ma.imu_heading(t, w, UP, BIAS)
    assert h[0] == 0.0 and np.all(np.diff(h) < 0)
    assert h[-1] == pytest.approx(-1.5, rel=1e-9)


def test_a_linear_bias_drift_cancels_with_the_mean_of_symmetric_windows():
    drift = np.array([2e-5, -1e-5, 3e-5])
    t, w, _ = imu_series(0.0, 40.0, 200, lambda x: 0.05 if 20.0 <= x <= 30.0 else 0.0, up=UP, bias=BIAS,
                         bias_drift=drift)
    ts = t / NS
    before, after = (ts >= 10.0) & (ts < 20.0), (ts > 30.0) & (ts <= 40.0)
    drive = (ts >= 20.0) & (ts <= 30.0)
    b_before = imu_mount.bias_from_still(w[before])
    b = imu_mount.bias_between(b_before, imu_mount.bias_from_still(w[after]))
    assert ma.imu_heading(t[drive], w[drive], UP, b["bias"])[-1] == pytest.approx(0.5, abs=1e-9)
    # the window before alone leaves the drift: (drift . up) x (drive midpoint - window midpoint) x drive time
    h_pre = ma.imu_heading(t[drive], w[drive], UP, b_before)
    assert h_pre[-1] - 0.5 == pytest.approx((drift @ UP) * (25.0 - 14.9975) * 10.0, rel=1e-6)


def test_zero_bias_change_leaves_white_noise_and_the_bias_mean():
    u = ma.heading_sigma_lower(arw_rad_per_sqrt_s=2e-4, drive_s=20.0, bias_window_s=40.0, bias_change_rad_s=0.0)
    assert u["bias_change_range"] == (0.0, 0.0)
    assert u["lower_bound"] == pytest.approx(math.hypot(2e-4 * math.sqrt(20.0), 2e-4 / math.sqrt(80.0) * 20.0))
    assert u["is_lower_bound"] is True
    assert set(u["not_included"]) == {"bias_change", "scale", "mount_axis", "roll_pitch"}


def test_a_falling_bias_counts_as_much_as_a_rising_one_and_nothing_is_zero():
    kw = {"arw_rad_per_sqrt_s": 1e-4, "drive_s": 30.0, "bias_window_s": 30.0}
    assert ma.heading_sigma_lower(**kw, bias_change_rad_s=-3e-5) == ma.heading_sigma_lower(**kw,
                                                                                           bias_change_rad_s=3e-5)
    zero = ma.heading_sigma_lower(arw_rad_per_sqrt_s=0.0, drive_s=30.0, bias_window_s=30.0, bias_change_rad_s=0.0)
    assert zero["lower_bound"] == 0.0 and zero["is_lower_bound"] is True


def test_the_gyro_scale_of_a_clockwise_double_turn():
    rate = -4 * math.pi / 40.0
    t, w, _ = imu_series(0.0, 40.0, 200, lambda x: rate, up=UP, bias=BIAS, scale=1.004)
    h = ma.imu_heading(t, w, UP, BIAS)
    assert h[-1] == pytest.approx(-4 * math.pi * 1.004, rel=1e-9)
    g = ma.gyro_scale(imu_dyaw=h[-1], turns=2)
    assert g["scale"] == pytest.approx(1.004, rel=1e-9) and g["sigma"] == 0.0 and g["turns"] == 2
    assert ma.gyro_scale(imu_dyaw=0.0, turns=1)["scale"] == 0.0
    with pytest.raises(ValueError):
        ma.gyro_scale(imu_dyaw=-2 * math.pi, turns=-1)


def test_the_gyro_scale_refuses_part_turns():
    with pytest.raises(ValueError):
        ma.gyro_scale(imu_dyaw=3 * math.pi, turns=1.5)


# --- odometry_heading, encoder_yaw_rate ----------------------------------------------------------------------------

def _odo(states, max_gap_s=0.1):
    return ma.odometry_heading(states, CFG, max_gap_s=max_gap_s)


@pytest.mark.parametrize("speed, kappa, err", [(0.1, 0.2, (0.02, 0.02)), (0.1, -0.2, (0.02, 0.02)),
                                               (-0.1, 0.2, (0.02, 0.02)), (-0.1, -0.2, (0.02, 0.02)),
                                               (0.1, 0.2, (0.0, 0.03))])
def test_odometry_heading_of_turns_both_ways_and_a_radius_error(speed, kappa, err):
    st = drive_states(1001, speed_mps=speed, kappa=kappa, cfg=CFG, radius_error=err)
    out = _odo(st)
    assert out["broken"] == []
    assert np.array_equal(out["mcu_time_us"], np.arange(1001) * 10_000)
    assert out["yaw"][-1] == pytest.approx(st[-1].true_yaw * odometry_distance_factor(kappa, err), abs=1e-4)
    assert np.sign(out["yaw"][-1]) == np.sign(speed * kappa)


def test_a_boot_change_breaks_continuity_and_keeps_the_yaw():
    a = drive_states(501, speed_mps=0.1, kappa=0.2, cfg=CFG, t0_us=5_000_000, boot_id=7, count0=(123456, -98765))
    b = drive_states(501, speed_mps=0.1, kappa=0.2, cfg=CFG, t0_us=0, boot_id=8, s0_m=a[-1].true_s_m,
                     yaw0=a[-1].true_yaw)
    out = _odo(a + b)
    assert out["broken"] == [(0, "boot_changed")]
    assert out["yaw"][501] == out["yaw"][500]
    assert out["yaw"][500] == pytest.approx(0.2 * 0.5, abs=1e-4)
    assert out["yaw"][-1] == pytest.approx(0.2 * 1.0, abs=1e-4)
    assert out["mcu_time_us"][501] < out["mcu_time_us"][500]


def test_an_encoder_jump_is_cut_out():
    st = drive_states(1001, speed_mps=0.1, kappa=0.2, cfg=CFG)
    k = 400
    st[k].fault_flags = int(FaultFlag.ENCODER_JUMP_L)
    for s in st[k:]:
        s.wheel_count_left = wrap_i32(s.wheel_count_left + 100_000)  # about 1.9 m of left wheel if integrated
    out = _odo(st)
    assert out["broken"] == [(st[k].mcu_time_us, "encoder_continuity")]
    assert out["yaw"][k] == out["yaw"][k - 1] and out["yaw"][k + 1] == out["yaw"][k - 1]  # cut, then re-reference
    assert out["yaw"][k + 2] > out["yaw"][k + 1]
    assert out["yaw"][-1] == pytest.approx(0.2 * 0.998, abs=1.5e-4)  # two 1 mm intervals lost, the jump not taken


def test_a_fault_that_is_not_an_encoder_cut_keeps_integrating():
    st = drive_states(1001, speed_mps=0.1, kappa=0.2, cfg=CFG)
    base = _odo(st)["yaw"]
    st[400].fault_flags = int(FaultFlag.COMMAND_TIMEOUT)
    out = _odo(st)
    assert out["broken"] == [] and np.array_equal(out["yaw"], base)


def test_a_time_gap_over_max_gap_is_not_integrated():
    st = drive_states(1001, speed_mps=0.1, kappa=0.2, cfg=CFG)
    kept = st[:301] + st[800:]  # 5 s without STATEs while the truck moved 0.5 m
    out = _odo(kept, max_gap_s=0.1)
    assert out["broken"] == [(st[800].mcu_time_us, "gap")]
    assert out["yaw"][301] == out["yaw"][300]
    assert out["yaw"][-1] == pytest.approx(0.2 * 0.5, abs=1e-4)
    wide = _odo(kept, max_gap_s=6.0)
    assert wide["broken"] == [] and wide["yaw"][-1] == pytest.approx(0.2, abs=1e-4)


@pytest.mark.parametrize("speed, count0", [(0.1, (2**31 - 3000, 2**31 - 2000)),
                                           (-0.1, (-(2**31) + 3000, -(2**31) + 2000))])
def test_wheel_counts_that_wrap_read_as_without_the_wrap(speed, count0):
    wrapped = drive_states(1001, speed_mps=speed, kappa=0.2, cfg=CFG, count0=count0)
    plain = drive_states(1001, speed_mps=speed, kappa=0.2, cfg=CFG)
    for side in ("wheel_count_left", "wheel_count_right"):
        assert np.sign(getattr(wrapped[0], side)) != np.sign(getattr(wrapped[-1], side))  # the wrap happened
    a, b = _odo(wrapped), _odo(plain)
    assert a["broken"] == [] and np.allclose(a["yaw"], b["yaw"], rtol=0.0, atol=1e-12)


def test_a_repeated_state_changes_nothing():
    st = drive_states(201, speed_mps=0.1, kappa=0.2, cfg=CFG)
    out = _odo(st[:100] + [st[99]] + st[100:])
    assert out["broken"] == [] and out["yaw"][100] == out["yaw"][99]
    assert out["yaw"][-1] == _odo(st)["yaw"][-1]


@pytest.mark.parametrize("speed, kappa", [(0.1, 0.2), (0.1, -0.2), (-0.1, 0.2), (-0.1, -0.2)])
def test_the_encoder_yaw_rate_sign(speed, kappa):
    st = drive_states(3, speed_mps=speed, kappa=kappa, cfg=CFG, radius_error=(0.02, 0.02))
    rl = np.array([s.wheel_rate_left_rad_s for s in st])
    rr = np.array([s.wheel_rate_right_rad_s for s in st])
    true_r = tuple(r * (1 + 0.02) for r in CFG.wheel_radius_m)
    yr = ma.encoder_yaw_rate(rl, rr, radius_m=true_r, track_m=CFG.track_m)
    assert yr == pytest.approx(np.full(3, speed * kappa), rel=1e-9)
    nominal = ma.encoder_yaw_rate(rl, rr, radius_m=CFG.wheel_radius_m, track_m=CFG.track_m)
    assert nominal == pytest.approx(np.full(3, speed * kappa / 1.02), rel=1e-9)
    if speed > 0 and kappa > 0:
        assert np.all(yr > 0)  # a forward left turn: the right wheel is faster, counter-clockwise is +


# --- M1 / M2 -------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("speed, kappa", [(0.1, -0.2), (-0.1, 0.2), (-0.1, -0.2)])
def test_m1_drift_from_records_of_right_turns_and_reversing(speed, kappa):
    st = drive_states(1001, speed_mps=speed, kappa=kappa, cfg=CFG, radius_error=(0.02, 0.02))
    odo = ma.odometry_heading(st, CFG, max_gap_s=0.1)
    t, w, _ = imu_series(0.0, 10.0, 200, lambda x: speed * kappa, up=UP, bias=BIAS)
    imu = ma.imu_heading(t, w, UP, BIAS)
    expected = speed * kappa * 10.0 * (1 / 1.02 - 1)  # the odometry under-reads by the radius error
    m = ma.m1_drift(odo_dyaw=odo["yaw"][-1] - odo["yaw"][0], imu_dyaw=imu[-1], distance_m=abs(st[-1].true_s_m),
                    distance_source="truth")
    assert m["odo_minus_imu_rad"] == pytest.approx(expected, abs=1e-4)
    assert m["rad_per_m"] == pytest.approx(expected / 1.0, abs=1e-4)
    assert np.sign(m["odo_minus_imu_rad"]) == -np.sign(speed * kappa)
    enc = ma.m1_drift(odo_dyaw=odo["yaw"][-1], imu_dyaw=imu[-1], distance_m=1.0 / 1.02, distance_source="encoder")
    assert enc["distance_source"] == "encoder"
    assert enc["rad_per_m"] == pytest.approx(enc["odo_minus_imu_rad"] * 1.02, rel=1e-12)


def test_m1_with_encoder_distance_says_the_radius_enters():
    m = ma.m1_drift(odo_dyaw=0.19, imu_dyaw=0.2, distance_m=0.98, distance_source="encoder")
    assert "radius" in m.get("note", "")


def test_m2_gains_of_a_right_turn():
    g = ma.m2_gains(imu_dyaw=-0.45, distance_m=1.5, direction=1, kappa_measured_q=-0.33, kappa_commanded_q=-0.36,
                    distance_source="truth")
    assert g["kappa_true"] == pytest.approx(-0.3)
    assert g["gain_measured_q"] == pytest.approx(0.3 / 0.33) and g["gain_commanded_q"] == pytest.approx(0.3 / 0.36)
    assert "radius" not in g["note"]


def test_m2_reverse_with_direction_minus_one():
    # reversing 1.5 m (the path length is positive) with the steering for +0.3 /m turns the body clockwise
    t, w, _ = imu_series(0.0, 15.0, 200, lambda x: -0.1 * 0.3, up=UP, bias=BIAS)
    dyaw = ma.imu_heading(t, w, UP, BIAS)[-1]
    g = ma.m2_gains(imu_dyaw=dyaw, distance_m=1.5, direction=-1, kappa_measured_q=0.3, kappa_commanded_q=0.3,
                    distance_source="truth")
    assert g["kappa_true"] == pytest.approx(0.3, rel=1e-9) and g["gain_measured_q"] == pytest.approx(1.0, rel=1e-9)
    assert g["direction"] == -1
    enc = ma.m2_gains(imu_dyaw=dyaw, distance_m=1.5, direction=-1, kappa_measured_q=0.3, kappa_commanded_q=0.3,
                      distance_source="encoder")
    assert enc["gain_measured_q"] == pytest.approx(1.0, rel=1e-6)


def test_m2_refuses_a_bad_direction_and_a_non_positive_distance():
    kw = {"imu_dyaw": 0.1, "kappa_measured_q": 0.3, "kappa_commanded_q": 0.3, "distance_source": "truth"}
    for bad in (0, 2):
        with pytest.raises(ValueError):
            ma.m2_gains(distance_m=1.0, direction=bad, **kw)
    with pytest.raises(ValueError):
        ma.m2_gains(distance_m=-1.0, direction=-1, **kw)
    with pytest.raises(ValueError):
        ma.m2_gains(distance_m=0.0, direction=1, **kw)


# --- M3 ------------------------------------------------------------------------------------------------------------

DT_NS = 2_500_000
POST = (2_500_000_000, 3_500_000_000)  # the steady stretch after the step at 1.0 s


def _grid(t1_s):
    t_ns = np.arange(ns(t1_s) // DT_NS, dtype=np.int64) * DT_NS  # 0 .. t1 exclusive
    return t_ns, t_ns / NS


def _first_order(t, start, tau, amp):
    return amp * (1 - np.exp(-np.maximum(t - start, 0.0) / tau))


def _second_order(t, start, zeta, wn, amp):
    tp = np.maximum(t - start, 0.0)
    wd, k = wn * math.sqrt(1 - zeta**2), zeta / math.sqrt(1 - zeta**2)
    return amp * (1 - np.exp(-zeta * wn * tp) * (np.cos(wd * tp) + k * np.sin(wd * tp)))


def _second_order_crossing(zeta, wn, frac):
    """First time the unit second-order step response reaches ``frac`` (monotonic up to its first peak)."""
    wd, k = wn * math.sqrt(1 - zeta**2), zeta / math.sqrt(1 - zeta**2)
    lo, hi = 0.0, math.pi / wd
    for _ in range(80):
        mid = (lo + hi) / 2
        if 1 - math.exp(-zeta * wn * mid) * (math.cos(wd * mid) + k * math.sin(wd * mid)) < frac:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def test_a_negative_step_from_a_nonzero_level():
    t_ns, t = _grid(4.0)
    d, tau = 0.1, 0.05
    q = 0.10 - np.minimum(np.maximum(t - 1.0, 0.0) / 0.2, 1.0) * 0.13
    y = 0.01 - _first_order(t, 1.0 + d, tau, 0.02)
    r = ma.step_response_delay(t_ns, q, t_ns, y, q_threshold=0.005, post_window_ns=POST)
    onset = 1.0 + 0.005 / 0.13 * 0.2
    assert r["delay_s"] == pytest.approx(1.0 + d + tau * math.log(1 / (1 - 0.63)) - onset, abs=2 * DT_NS / NS)
    assert r["y_pre"] == pytest.approx(0.01, abs=1e-12) and r["y_post"] == pytest.approx(-0.01, rel=1e-6)


def test_a_response_that_never_settles_in_the_named_window_is_refused():
    t_ns, t = _grid(3.0)
    q = np.where(t >= 1.0, 0.13, 0.0)
    y = _first_order(t, 1.1, 2.0, 0.02)
    r = ma.step_response_delay(t_ns, q, t_ns, y, q_threshold=0.005, post_window_ns=(ns(1.5), ns(2.5)))
    assert r["delay_s"] is None and r["reason"] in ("not_steady", "not_reached")


def test_a_post_window_before_the_step_is_refused():
    t_ns, t = _grid(4.0)
    q = np.where(t >= 1.0, 0.13, 0.0)
    y = _first_order(t, 1.1, 0.05, 0.02)
    r = ma.step_response_delay(t_ns, q, t_ns, y, q_threshold=0.005, post_window_ns=(ns(0.5), ns(0.9)))
    assert r["delay_s"] is None and r["reason"] == "bad_post_window"


def test_one_label_naming_two_segments_is_refused():
    rows = profile_rows(1, [("arc", 0.0, 1.0)]) + [profile_row(1, "arc", ns(2.0), seg=3)]
    with pytest.raises(ValueError, match="two segments"):
        ma.trials(rows)


def test_a_noisy_response():
    t_ns, t = _grid(4.0)
    rng = np.random.default_rng(11)
    d, tau = 0.12, 0.05
    q = np.minimum(np.maximum(t - 1.0, 0.0) / 0.2, 1.0) * 0.13 + rng.normal(0.0, 0.0005, len(t))
    y = _first_order(t, 1.0 + d, tau, 0.02) + rng.normal(0.0, 0.0005, len(t))
    r = ma.step_response_delay(t_ns, q, t_ns, y, q_threshold=0.005, post_window_ns=POST)
    onset = 1.0 + 0.005 / 0.13 * 0.2
    assert r["delay_s"] == pytest.approx(1.0 + d + tau * math.log(1 / (1 - 0.63)) - onset, abs=0.02)
    assert r["y_post"] == pytest.approx(0.02, abs=2e-4)


def test_a_response_that_overshoots_is_timed_at_its_first_crossing():
    t_ns, t = _grid(4.0)
    d, zeta, wn = 0.08, 0.3, 40.0
    q = np.where(t >= 1.0, 0.13, 0.0)
    y = _second_order(t, 1.0 + d, zeta, wn, 0.02)
    assert y.max() > 0.02 * 1.3
    r = ma.step_response_delay(t_ns, q, t_ns, y, q_threshold=0.005, post_window_ns=POST)
    assert r["onset_ns"] == NS
    assert r["delay_s"] == pytest.approx(d + _second_order_crossing(zeta, wn, 0.63), abs=1e-3)
    assert r["y_post"] == pytest.approx(0.02, rel=1e-6)


def test_q_that_stays_within_the_threshold_is_no_step():
    t_ns, t = _grid(4.0)
    rng = np.random.default_rng(12)
    r = ma.step_response_delay(t_ns, 0.05 + rng.normal(0.0, 0.0005, len(t)), t_ns,
                               rng.normal(0.0, 0.001, len(t)), q_threshold=0.005, post_window_ns=POST)
    assert r["delay_s"] is None and r["reason"] == "no_step"


def test_a_step_inside_the_first_pre_window_gives_no_delay():
    t_ns, t = _grid(4.0)
    q = np.where(t >= 0.2, 0.13, 0.0)
    y = _first_order(t, 0.3, 0.05, 0.02)
    assert ma.step_response_delay(t_ns, q, t_ns, y, q_threshold=0.005, post_window_ns=POST)["delay_s"] is None


def test_a_constant_clock_offset_of_y_adds_to_the_delay():
    t_ns, t = _grid(4.0)
    q = np.where(t >= 1.0, 0.13, 0.0)
    y = _first_order(t, 1.1, 0.05, 0.02)
    r0 = ma.step_response_delay(t_ns, q, t_ns, y, q_threshold=0.005, post_window_ns=POST)
    r1 = ma.step_response_delay(t_ns, q, t_ns + 10_000_000, y, q_threshold=0.005, post_window_ns=POST)
    assert r1["delay_s"] - r0["delay_s"] == pytest.approx(0.01, abs=1e-9)
