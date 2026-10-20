"""Gyro-heading plan G2 (A) (docs/plans/2026-10-21-gyro-g2-runtime-safety.md): the simulated gyro, the standstill bias,
the faults and the heading odometry (V-G1 items 1-3, 6)."""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.localization import gyro as G
from forklift_core.localization.slam_pose import IncrementalWheelOdometry
from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry

ROOT = Path(__file__).resolve().parents[3]
MODEL, _ = G.GyroModel.load(ROOT / "config" / "gyro_model_spec_typical.yaml")
QUIET = replace(MODEL, scale=0.0, initial_bias_dps=0.0, temperature_dps=0.0, random_walk_dps_rts=0.0, noise_dps_rthz=0.0,
                lsb_dps=1e-9, mount_error_deg=0.0)
# the carriage camera (perception mount carriage_low_measured): base_link <- optical, tilted 0.10 rad down
OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
TILT = np.array([[math.cos(0.1), 0.0, math.sin(0.1)], [0.0, 1.0, 0.0], [-math.sin(0.1), 0.0, math.cos(0.1)]])
MOUNT = TILT @ OPTICAL


def test_the_model_file_loads_and_a_bad_one_is_refused(tmp_path):
    assert MODEL.noise_dps_rthz == 0.014 and abs(MODEL.lsb_dps - 1 / 32.8) < 1e-15 and MODEL.gap_s == 0.05
    (tmp_path / "bad.yaml").write_text("format: other/1\n")
    with pytest.raises(ValueError, match="not a gyro_model/1"):
        G.GyroModel.load(tmp_path / "bad.yaml")


def test_a_yaw_rate_reads_on_the_imu_axes_and_comes_back_vertical_through_the_known_mount():
    sampler = G.GyroSampler(QUIET, MOUNT, seed=0)
    stamp, w_imu, seq = sampler.sample(1 / 120, [0.0, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    assert seq == 1 and stamp == 1 / 120
    assert abs(G.vertical_rate(MOUNT, w_imu) - 0.2) < 1e-9
    assert abs(w_imu[1] + 0.2 * math.cos(0.1)) < 1e-9  # mostly on the sensor's -y (the camera looks ahead, tilted)


def test_the_samples_carry_the_model_s_noise_scale_and_quantisation():
    model = replace(MODEL, initial_bias_dps=0.0, temperature_dps=0.0, random_walk_dps_rts=0.0, mount_error_deg=0.0)
    sampler = G.GyroSampler(model, MOUNT, seed=1)
    rates = []
    for i in range(1, 12_001):
        _, w, _ = sampler.sample(i / 120, [0.0, 0.0, 0.3], [1.0, 0.0, 0.0, 0.0])
        rates.append(G.vertical_rate(MOUNT, w))
    rates = np.asarray(rates)
    assert abs(rates.mean() / 0.3 - 1.01) < 2e-3
    assert abs(rates.std() * math.sqrt(1 / 120) * 180 / math.pi / 0.014 - 1) < 0.05


def test_a_fault_injection_turns_samples_nan_from_its_time_on():
    sampler = G.GyroSampler(MODEL, MOUNT, seed=0, fault_at_s=0.5)
    assert all(math.isfinite(v) for v in sampler.sample(0.4, [0, 0, 0], [1, 0, 0, 0])[1])
    assert all(math.isnan(v) for v in sampler.sample(0.5, [0, 0, 0], [1, 0, 0, 0])[1])


def _step(state, *, stamp, seq, rate, still=True, zero=True, model=MODEL):
    return G.heading_step(state, model, stamp_s=stamp, seq=seq, raw_rate=rate, wheels_still=still, command_zero=zero,
                          dt=1 / 120)


def test_a_standstill_block_sets_the_bias_and_motion_or_a_command_or_a_varying_rate_does_not():
    noise = np.random.default_rng(2).normal(0, MODEL.noise_dps_rthz * math.pi / 180 * math.sqrt(120), 240)
    bias = math.radians(0.8)
    state = G.new_heading_state()
    out = [_step(state, stamp=(i + 1) / 120, seq=i + 1, rate=bias + noise[i]) for i in range(240)]
    assert out[58] is None and out[59] is not None  # the first block ends at the 60th tick
    assert state["updates"] == 4 and abs(state["bias"] - bias) < math.radians(0.05)
    for still, zero in ((False, True), (True, False)):
        state = G.new_heading_state()
        for i in range(240):
            _step(state, stamp=(i + 1) / 120, seq=i + 1, rate=bias, still=still, zero=zero)
        assert state["updates"] == 0 and state["bias"] is None
    state = G.new_heading_state()
    for i in range(240):  # a turn starting inside the block: not white noise
        _step(state, stamp=(i + 1) / 120, seq=i + 1, rate=bias + 0.002 * i)
    assert state["updates"] == 0


def test_the_heading_increment_is_the_rate_less_its_bias():
    state = G.new_heading_state()
    for i in range(60):
        _step(state, stamp=(i + 1) / 120, seq=i + 1, rate=0.002, model=QUIET)
    assert _step(state, stamp=61 / 120, seq=61, rate=0.102, still=False, zero=False, model=QUIET) == pytest.approx(0.1 / 120)


@pytest.mark.parametrize("stamps, seqs, rate, match", [
    ((0.1, 0.2), (1, 2), 0.0, "without a sample"),  # 100 ms gap
    ((0.1, 0.1), (1, 2), 0.0, "does not advance"),
    ((0.1, 0.11), (1, 1), 0.0, "does not advance"),  # the same sample twice
    ((0.1,), (1,), float("nan"), "rate nan"),
    ((0.1,), (1,), math.radians(1001), "rate"),
])
def test_a_gap_a_repeat_a_nan_or_an_out_of_range_rate_is_a_gyro_fault(stamps, seqs, rate, match):
    state = G.new_heading_state()
    with pytest.raises(G.GyroFault, match=f"slam_gyro_fault: .*{match}"):
        for stamp, seq in zip(stamps, seqs):
            _step(state, stamp=stamp, seq=seq, rate=rate if seq == seqs[-1] and stamp == stamps[-1] else 0.0)


def test_the_heading_odometry_moves_by_the_wheels_along_the_gyro_heading():
    odo = IncrementalWheelOdometry(AckermannOdometryGeometry(0.66, 0.53, 0.125), initial_pose=(0.0, 0.0, 0.0))
    odo.update_increments(0.0, (0.0, 0.0), (0.0, 0.0))  # standing before the bias (the steering path)
    pose = odo.update_increments_heading(1 / 120, (0.08, 0.08), 0.01)
    assert pose[2] == pytest.approx(0.01)
    assert pose[0] == pytest.approx(0.01 * math.cos(0.005)) and pose[1] == pytest.approx(0.01 * math.sin(0.005))


def test_an_age_table_from_another_heading_is_refused():
    from forklift_core.localization.encoders import odometry_table_problem

    steering = {"noise": {"encoder_model": "team_spec_quantized"}, "odometry_wheel_radius_m": 0.125}
    gyro = {"noise": {"encoder_model": "team_spec_quantized", "odometry_heading": "gyro", "gyro_model_sha256": "a",
                      "gyro_mount": "carriage_low_measured"}, "odometry_wheel_radius_m": 0.125}
    kw = dict(encoder_model="team_spec_quantized", wheel_radius_m=0.125, geometric_radius=True)
    assert odometry_table_problem(steering, **kw) is None
    assert "steering heading" in odometry_table_problem(steering, **kw, odometry_heading="gyro")
    assert odometry_table_problem(gyro, **kw, odometry_heading="gyro", gyro_model_sha256="a",
                                  gyro_mount="carriage_low_measured") is None
    assert "gyro model" in odometry_table_problem(gyro, **kw, odometry_heading="gyro", gyro_model_sha256="b",
                                                  gyro_mount="carriage_low_measured")


def test_no_drive_before_the_first_bias():
    from types import SimpleNamespace

    from forklift_core.mission.core import slam_may_drive

    tracker = SimpleNamespace(may_drive=lambda: True)
    assert slam_may_drive({"tracker": tracker})  # steering: as before
    assert not slam_may_drive({"tracker": tracker, "gyro": G.new_heading_state()})
    assert slam_may_drive({"tracker": tracker, "gyro": {**G.new_heading_state(), "bias": 0.001}})
