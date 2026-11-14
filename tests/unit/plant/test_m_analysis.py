"""G8 (docs/design/2026-11-14-profile-drive-and-m-analysis.md 3): the M1-M3 values from synthetic records with known
gains, biases, delays and radius errors -- trials from the profile rows, bias windows from the record, IMU heading,
its lower-bound uncertainty, M1 drift, M2 gains, the encoder-differential and IMU step responses."""

from __future__ import annotations

import math

import numpy as np
import pytest

from forklift_core.plant import m_analysis as ma

NS = 1_000_000_000
UP = np.array([0.0, -1.0, 0.0])  # body-up in the IMU frame (a level D435i)


def prow(trial, label, t_s, result=None, state="running"):
    return {"t": "profile", "trial": trial, "label": label, "host_ns": int(t_s * NS), "state": state,
            "result": result}


def test_trials_come_from_the_profile_rows_and_void_ones_are_marked():
    rows = [prow(1, "presteer", 0.0), prow(1, "still_pre", 1.0), prow(1, "arc", 2.0), prow(1, "arc", 5.0),
            prow(1, "still_post", 5.02), prow(1, None, 6.0, "done", "zeroing"),
            prow(2, "presteer", 10.0), prow(2, "arc", 12.0), prow(2, None, 12.5, "void:deadman", "zeroing"),
            {"t": "state", "host_ns": 1}]
    trials = ma.trials(rows)
    assert [t.trial for t in trials] == [1, 2]
    t1, t2 = trials
    assert t1.result == "done" and t1.valid and not t2.valid
    assert t1.spans["arc"] == (2 * NS, 5 * NS) and t1.spans["still_post"][0] == int(5.02 * NS)
    assert [t.trial for t in ma.trials(rows, valid_only=True)] == [1]


def imu_series(t0, t1, rate_hz, yaw_rate_fn, bias, noise=0.0, seed=0, accel_motion=None):
    rng = np.random.default_rng(seed)
    t = np.arange(t0, t1, 1.0 / rate_hz)
    w = np.outer([yaw_rate_fn(x) for x in t], UP) + bias + rng.normal(0, noise, (len(t), 3))
    a = np.tile(UP * 9.80665, (len(t), 1))
    if accel_motion is not None:
        a = a + np.array([[accel_motion(x), 0.0, 0.0] for x in t])
    return (t * NS).astype(np.int64), w, a


def test_a_quiet_window_is_found_next_to_the_trial_and_motion_is_cut_off():
    bias = np.array([0.003, -0.002, 0.001])
    # standing 0-20 s, the drive starts at 20 s; the last 0.5 s before it the truck already creeps
    t, w, a = imu_series(0, 25, 200, lambda x: 0.2 if x > 19.5 else 0.0, bias, noise=0.002,
                         accel_motion=lambda x: 0.5 if x > 19.5 else 0.0)
    win = ma.quiet_window(t, w, a, boundary_ns=20 * NS, side="before", max_s=15.0, min_s=5.0)
    assert win is not None
    s, e = win
    assert t[e - 1] <= int(19.5 * NS) and (t[e - 1] - t[s]) / NS > 10.0
    assert np.allclose(w[s:e].mean(axis=0), bias, atol=5e-4)
    assert ma.quiet_window(t, w, a, boundary_ns=20 * NS, side="before", max_s=15.0, min_s=30.0) is None


def test_imu_heading_integrates_the_bias_free_yaw_rate():
    bias = np.array([0.004, 0.0, 0.0])
    t, w, _ = imu_series(0, 10, 400, lambda x: 0.1, bias)
    h = ma.imu_heading(t, w, UP, bias)
    assert h[0] == 0.0 and h[-1] == pytest.approx(0.1 * (t[-1] - t[0]) / NS, rel=1e-6)


def test_the_heading_uncertainty_is_a_lower_bound_with_its_parts():
    u = ma.heading_sigma_lower(arw_rad_per_sqrt_s=1e-4, drive_s=30.0, bias_window_s=30.0, bias_change_rad_s=2e-5)
    assert u["white"] == pytest.approx(1e-4 * math.sqrt(30))
    assert u["bias_mean"] == pytest.approx(1e-4 / math.sqrt(60) * 30)
    assert u["bias_change"] == pytest.approx(2e-5 * 30 / 2)
    assert u["lower_bound"] == pytest.approx(math.sqrt(u["white"] ** 2 + u["bias_mean"] ** 2 + u["bias_change"] ** 2))
    assert u["is_lower_bound"] is True and "scale" in u["not_included"]


def test_m1_drift_names_its_distance_source():
    m = ma.m1_drift(odo_dyaw=0.012, imu_dyaw=0.003, distance_m=3.0, distance_source="truth")
    assert m == {"rad_per_m": pytest.approx(0.003), "odo_minus_imu_rad": pytest.approx(0.009),
                 "distance_m": 3.0, "distance_source": "truth"}
    with pytest.raises(ValueError, match="source"):
        ma.m1_drift(odo_dyaw=0.0, imu_dyaw=0.0, distance_m=3.0, distance_source="guess")


def test_m2_gains_measured_and_commanded_q_are_told_apart():
    # true curvature 0.3 over 1.5 m; the measured q says 0.33, the command asked 0.36
    g = ma.m2_gains(imu_dyaw=0.45, distance_m=1.5, kappa_measured_q=0.33, kappa_commanded_q=0.36,
                    distance_source="encoder")
    assert g["kappa_true"] == pytest.approx(0.3)
    assert g["gain_measured_q"] == pytest.approx(0.3 / 0.33) and g["gain_commanded_q"] == pytest.approx(0.3 / 0.36)
    assert g["distance_source"] == "encoder" and "radius" in g["note"]


def test_the_encoder_differential_yaw_rate_is_wheel_speed_difference_over_track():
    yr = ma.encoder_yaw_rate(np.array([1.0, 1.2]), np.array([1.4, 1.2]), radius_m=(0.125, 0.125), track_m=0.53)
    assert yr == pytest.approx([(1.4 - 1.0) * 0.125 / 0.53, 0.0])


@pytest.mark.parametrize("delay_s", [0.08, 0.15])
def test_a_step_response_delay_is_from_the_q_onset_to_63_percent(delay_s):
    dt = 0.0025
    t = np.arange(0, 4, dt)
    q = np.where(t >= 1.0, np.minimum((t - 1.0) / 0.2, 1.0) * 0.13, 0.0)  # a slewed step from 1.0 s
    tau = 0.05  # first-order lag after a pure delay
    y = np.where(t >= 1.0 + delay_s, 0.02 * (1 - np.exp(-(t - 1.0 - delay_s) / tau)), 0.0)
    r = ma.step_response_delay((t * NS).astype(np.int64), q, (t * NS).astype(np.int64), y, q_threshold=0.005)
    onset = 1.0 + 0.005 / 0.13 * 0.2
    expected = 1.0 + delay_s + tau * math.log(1 / (1 - 0.63)) - onset
    assert r["delay_s"] == pytest.approx(expected, abs=2 * dt)
    assert r["y_pre"] == pytest.approx(0.0, abs=1e-9) and r["y_post"] == pytest.approx(0.02, rel=1e-3)


def test_a_step_with_no_response_is_none():
    t = (np.arange(0, 4, 0.01) * NS).astype(np.int64)
    q = np.where(t >= NS, 0.13, 0.0)
    r = ma.step_response_delay(t, q, t, np.zeros(len(t)), q_threshold=0.005)
    assert r["delay_s"] is None and r["reason"] == "no_response"


def test_odometry_heading_reruns_the_real_odometry_on_recorded_states():
    from types import SimpleNamespace as NS_

    class Map:
        def curvature(self, q):
            return q / 0.65

    cfg = NS_(steering=Map(), counts_per_wheel_rev=(4096, 4096), wheel_radius_m=(0.125, 0.125))
    # a constant q over 1 m: heading change = kappa * distance
    states = []
    counts = 0
    per_m = 4096 / (2 * math.pi * 0.125)
    for i in range(101):
        states.append(NS_(mcu_time_us=i * 10_000, boot_id=1, wheel_count_left=int(counts), wheel_count_right=int(counts),
                          steering_rad=0.13, fault_flags=0))
        counts += per_m * 0.01
    out = ma.odometry_heading(states, cfg, max_gap_s=0.1)
    assert out["yaw"][-1] == pytest.approx(0.2 * 1.0, rel=0.01) and out["broken"] == []


def test_the_gyro_scale_comes_from_whole_turns_back_to_the_same_heading():
    g = ma.gyro_scale(imu_dyaw=-2 * 2 * math.pi * 1.004, turns=2, heading_error_rad=0.002)
    assert g["scale"] == pytest.approx(1.004) and g["sigma"] == pytest.approx(math.sqrt(2) * 0.002 / (4 * math.pi))
    with pytest.raises(ValueError):
        ma.gyro_scale(imu_dyaw=1.0, turns=0)

