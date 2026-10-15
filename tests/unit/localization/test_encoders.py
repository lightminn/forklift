"""The team's encoder models (MT6701 x4 on the rear wheels, KTH7801-C 12-bit on the steering) and the
increment path of the wheel odometry (EPAL 6 fork spacing plan, encoder model)."""

import math

import numpy as np
import pytest

from forklift_core.localization.encoders import (
    KTH7801_C_ANGLE_BITS,
    MT6701_COUNTS_PER_REV,
    AbsoluteAngleEncoder,
    AngleUnwrapper,
    QuadratureWheelEncoder,
)
from forklift_core.localization.slam_pose import IncrementalWheelOdometry
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
    integrate_wheel_odometry_increments,
)

GEOMETRY = AckermannOdometryGeometry(0.66, 0.53, 0.125)


def test_the_team_specs():
    assert MT6701_COUNTS_PER_REV == 4096  # 1,024 pulses per revolution, counted x4
    assert KTH7801_C_ANGLE_BITS == 12


def test_the_unwrapper_follows_a_joint_that_wraps_at_two_pi():
    unwrap = AngleUnwrapper()
    truth = np.arange(0, 40, 0.05)  # about six turns, 0.05 rad per reading
    raw = np.mod(truth + 2 * np.pi, 4 * np.pi) - 2 * np.pi  # wraps within +-2 pi like the Isaac joints
    out = [unwrap.update(v) for v in raw]
    assert np.allclose(np.asarray(out) - out[0], truth - truth[0], atol=1e-9)


def test_the_wheel_encoder_counts_whole_steps_and_returns_their_rotation():
    enc = QuadratureWheelEncoder()
    step = 2 * math.pi / 4096
    assert enc.update(0.3 * step) == 0.0  # the first reading sets the reference
    assert enc.update(1.2 * step) == pytest.approx(step)
    assert enc.update(1.9 * step) == 0.0
    assert enc.update(-0.1 * step) == pytest.approx(-2 * step)
    # Over many readings the summed increments track the shaft to one count.
    enc = QuadratureWheelEncoder()
    angles = np.cumsum(np.full(5000, 0.0123)) % (2 * np.pi)
    total = sum(enc.update(a) for a in angles)
    assert abs(total - (np.unwrap(angles)[-1] - angles[0])) <= step


def test_the_wheel_encoder_inl_is_bounded_and_periodic():
    enc = QuadratureWheelEncoder(inl_amplitude_rad=math.radians(1.0), inl_phase_rad=0.7)
    step = enc.step_rad
    angles = np.linspace(0, 4 * np.pi, 20001)
    measured = np.array([enc.counts_at(a) for a in angles]) * step
    error = measured - angles
    assert np.max(np.abs(error)) <= math.radians(1.0) + step
    # A whole revolution's count is unaffected by a periodic error.
    assert enc.counts_at(2 * np.pi + 0.1) - enc.counts_at(0.1) == 4096


def test_the_absolute_encoder_quantises_to_twelve_bits_with_its_zero_calibrated():
    enc = AbsoluteAngleEncoder(inl_amplitude_rad=math.radians(0.35), inl_phase_rad=1.1)
    step = 2 * math.pi / 4096
    assert enc.read(0.0) == 0.0
    for angle in np.linspace(-0.27, 0.27, 541):
        value = enc.read(angle)
        assert abs(value / step - round(value / step)) < 1e-9
        assert abs(value - angle) <= 2 * math.radians(0.35) + step / 2
    ideal = AbsoluteAngleEncoder()
    for angle in np.linspace(-0.27, 0.27, 541):
        assert abs(ideal.read(angle) - angle) <= step / 2 + 1e-12


def test_the_increment_path_does_not_lag_where_the_rate_path_does():
    """Codex (E results round 1): Delta-theta / dt is the interval's mean rate; fed to the rate
    path's trapezoid it lags. A smooth 0.8 m/s^2 ramp to 0.6 m/s at 120 Hz: the increment path
    lands on the true distance, the rate path fed with increment / dt does not."""
    dt = 1 / 120
    t = np.arange(0, 1.5 + dt / 2, dt)
    s = np.where(t < 0.75, 0.4 * t ** 2, 0.4 * 0.75 ** 2 + 0.6 * (t - 0.75))  # true rear-axle distance
    theta = s / GEOMETRY.wheel_radius_m
    inc = np.r_[0.0, np.diff(theta)]
    increments = np.c_[inc, inc]
    steering = np.zeros((len(t), 2))
    poses = integrate_wheel_odometry_increments(t, increments, steering, GEOMETRY)
    assert poses[-1, 0] == pytest.approx(s[-1], abs=1e-12)
    lagging = integrate_wheel_odometry(t, increments / dt, steering, GEOMETRY)
    assert s[-1] - lagging[-1, 0] > 0.002


def test_the_incremental_increment_path_equals_the_batch_bit_for_bit_and_refuses_mixing():
    rng = np.random.default_rng(3)
    t = np.cumsum(np.full(400, 1 / 120))
    increments = rng.normal(0.02, 0.005, (400, 2))
    steering = np.clip(np.cumsum(rng.normal(0, 0.003, (400, 2)), axis=0), -0.26, 0.26)
    batch = integrate_wheel_odometry_increments(t, increments, steering, GEOMETRY, initial_pose=(1.0, -2.0, 0.3))
    odo = IncrementalWheelOdometry(GEOMETRY, initial_pose=(1.0, -2.0, 0.3))
    online = np.array([odo.update_increments(a, b, c) for a, b, c in zip(t, increments, steering, strict=True)])
    assert np.array_equal(batch, online)
    assert odo.interval_speed == pytest.approx(increments[-1].mean() * 0.125 * 120)
    with pytest.raises(ValueError, match="integrates increments"):
        odo.update(t[-1] + 0.01, (1.0, 1.0), (0.0, 0.0))
    fresh = IncrementalWheelOdometry(GEOMETRY)
    fresh.update(0.0, (1.0, 1.0), (0.0, 0.0))
    with pytest.raises(ValueError, match="integrates rates"):
        fresh.update_increments(0.01, (0.01, 0.01), (0.0, 0.0))


def test_the_hysteresis_holds_the_count_through_a_reversal_inside_its_band():
    from forklift_core.localization.encoders import MT6701_ABZ_HYSTERESIS_RAD

    step = 2 * math.pi / 4096
    enc = QuadratureWheelEncoder(hysteresis_rad=MT6701_ABZ_HYSTERESIS_RAD)
    assert enc.update_continuous(0.2 * step) == 0.0 and enc.count == 0
    # Up: the next transition moves out by half the band.
    assert enc.update_continuous(1.3 * step) == 0.0
    assert enc.update_continuous(1.6 * step) == pytest.approx(step)
    # A small reversal inside the band leaves the count; a full one steps down.
    assert enc.update_continuous(0.8 * step) == 0.0
    assert enc.update_continuous(0.4 * step) == pytest.approx(-step)
    # Steady forward motion still counts every step, one per step on average.
    enc = QuadratureWheelEncoder(hysteresis_rad=MT6701_ABZ_HYSTERESIS_RAD)
    angles = np.arange(0, 2 * np.pi, 0.37 * step)
    total = sum(enc.update_continuous(a) for a in angles)
    assert abs(total - (angles[-1] - angles[0])) <= 2 * step


def test_the_first_reading_sets_a_nonzero_reference_without_a_spurious_increment():
    """Codex (encoder design P2): with an INL phase the count at theta = 0 is not 0 (A = 1 deg,
    phase pi/2 -> 11 counts); differencing it against 0 would invent 2.11 mm of travel."""
    enc = QuadratureWheelEncoder(inl_amplitude_rad=math.radians(1.0), inl_phase_rad=math.pi / 2)
    assert enc.update_continuous(0.0) == 0.0
    assert enc.count == 11
    assert enc.update_continuous(0.0) == 0.0


def _team_log(name, seed, n=600):
    from forklift_core.localization.encoders import TeamEncoderModel, encoder_model_spec, sensor_phases

    rng = np.random.default_rng(5)
    t = np.arange(n) / 120
    rear = np.cumsum(rng.normal(0.02, 0.01, (n, 2)), axis=0) + 3.0
    angles = np.c_[rear + 0.1, rear]
    steering = np.clip(np.cumsum(rng.normal(0, 0.004, (n, 2)), axis=0), -0.26, 0.26)
    spec = encoder_model_spec(name)
    model = TeamEncoderModel(spec, sensor_phases(seed))
    rows = [model.read(angles[k, 2:4], steering[k]) for k in range(n)]
    log = {"joint_stamps_s": t, "steering_rad": steering, "wheel_angles_rad": angles,
           "wheel_rates_rad_s": np.zeros((n, 4)),
           "wheel_increments_rad": np.array([r[0] for r in rows]),
           "steering_measured_rad": np.array([r[1] for r in rows]),
           "wheel_counts": np.array([r[2] for r in rows], dtype=np.int64)}
    return log, {"encoder_model": model.record()}


def test_a_team_spec_log_replays_to_its_recorded_counts_and_damage_is_refused():
    from forklift_core.localization.encoders import encoder_odometry_inputs

    log, meta = _team_log("team_spec_typical_inl", seed=4)
    out = encoder_odometry_inputs(log, meta)
    assert out["kind"] == "increments" and np.array_equal(out["counts"], log["wheel_counts"])
    other = encoder_odometry_inputs(log, meta, phase_seed=9)  # another sensor unit: a different draw
    assert not np.array_equal(other["counts"], log["wheel_counts"])
    damaged = dict(log, wheel_counts=log["wheel_counts"] + np.r_[0, np.ones(len(log["wheel_counts"]) - 1, int)][:, None])
    with pytest.raises(ValueError, match="wheel_counts"):
        encoder_odometry_inputs(damaged, meta)
    with pytest.raises(ValueError, match="unknown encoder model"):
        encoder_odometry_inputs(log, {"encoder_model": {**meta["encoder_model"], "name": "team_spec_v2"}})
    with pytest.raises(ValueError, match="differs"):
        encoder_odometry_inputs(log, {"encoder_model": {**meta["encoder_model"], "wheel_counts_per_rev": 2048}})
    with pytest.raises(ValueError, match="wheel_angles_rad"):
        encoder_odometry_inputs(dict(log, wheel_angles_rad=log["wheel_angles_rad"][:, :2]), meta)
    # A run without the record (or "reported") replays the reported rates as before.
    assert encoder_odometry_inputs(log, {})["kind"] == "rates"
    quantised, qmeta = _team_log("team_spec_quantized", seed=None)
    assert encoder_odometry_inputs(quantised, qmeta)["kind"] == "increments"


def test_a_mid_span_replay_takes_no_increment_from_before_its_first_row():
    from forklift_core.localization.encoders import encoder_odometry_inputs

    log, meta = _team_log("team_spec_quantized", seed=None)
    out = encoder_odometry_inputs(log, meta)
    i0, i1 = 200, 400
    t = log["joint_stamps_s"][i0:i1]
    poses = integrate_wheel_odometry_increments(t, out["increments"][i0:i1], out["steering"][i0:i1], GEOMETRY)
    expected = (np.diff(out["counts"][i0:i1], axis=0).sum(axis=0) * 2 * math.pi / 4096).mean() * 0.125
    travelled = np.hypot(*np.diff(poses[:, :2], axis=0).T).sum()
    assert travelled == pytest.approx(abs(expected), rel=1e-6)
