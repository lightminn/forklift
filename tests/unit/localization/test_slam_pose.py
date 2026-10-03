"""Online pose estimation pieces (docs/plans/2026-10-04-online-slam-closed-loop.md)."""

import math

import numpy as np
import pytest

from forklift_core.localization.slam_pose import (
    IncrementalWheelOdometry,
    LocalizationStale,
    SlamPoseEstimator,
    compose,
    invert,
)
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
)

GEOMETRY = AckermannOdometryGeometry(0.64, 0.51, 0.135)


def _drive(n=600, seed=0):
    rng = np.random.default_rng(seed)
    stamps = np.arange(n) / 120.0
    rates = np.column_stack([np.full(n, 3.0), np.full(n, 3.1)]) + rng.normal(0, 0.1, (n, 2))
    steer = np.column_stack([np.linspace(0, 0.3, n), np.linspace(0, 0.25, n)])
    return stamps, rates, steer


def test_incremental_odometry_matches_the_batch_integrator_exactly():
    stamps, rates, steer = _drive()
    batch = integrate_wheel_odometry(stamps, rates, steer, GEOMETRY, initial_pose=(0.34, 0.0, 0.1))
    odometry = IncrementalWheelOdometry(GEOMETRY, initial_pose=(0.34, 0.0, 0.1))
    poses = [odometry.update(t, r, s) for t, r, s in zip(stamps, rates, steer)]
    np.testing.assert_array_equal(np.array(poses), batch)


def test_incremental_odometry_rejects_time_going_backwards():
    odometry = IncrementalWheelOdometry(GEOMETRY)
    odometry.update(1.0, (1, 1), (0, 0))
    with pytest.raises(ValueError):
        odometry.update(1.0, (1, 1), (0, 0))


def test_compose_and_invert_are_consistent():
    a, b = (1.0, 2.0, 0.3), (-0.5, 0.25, -1.1)
    ab = compose(a, b)
    np.testing.assert_allclose(compose(invert(a), ab), b, atol=1e-12)
    np.testing.assert_allclose(compose(a, invert(a)), (0, 0, 0), atol=1e-12)


def test_the_estimate_is_map_from_odom_times_odom_from_base():
    estimator = SlamPoseEstimator(max_age_s=0.3)
    estimator.set_map_from_odom(10.0, (0.5, -0.2, 0.05))
    pose = estimator.map_from_base(10.05, (1.0, 0.0, 0.0))
    expected = (0.5 + math.cos(0.05), -0.2 + math.sin(0.05), 0.05)
    np.testing.assert_allclose(pose, expected, atol=1e-12)


def test_a_stale_correction_stops_the_estimate():
    estimator = SlamPoseEstimator(max_age_s=0.3)
    with pytest.raises(LocalizationStale):
        estimator.map_from_base(0.0, (0, 0, 0))  # nothing received yet
    estimator.set_map_from_odom(10.0, (0, 0, 0))
    estimator.map_from_base(10.3, (0, 0, 0))  # exactly at the limit is fine
    with pytest.raises(LocalizationStale):
        estimator.map_from_base(10.31, (0, 0, 0))


def test_corrections_must_not_go_back_in_time():
    estimator = SlamPoseEstimator(max_age_s=0.3)
    estimator.set_map_from_odom(10.0, (0, 0, 0))
    with pytest.raises(ValueError):
        estimator.set_map_from_odom(9.9, (0, 0, 0))


# --- v3 state machine, stop detector, noise ---------------------------------
from forklift_core.localization.slam_pose import (  # noqa: E402
    OdometryNoise,
    SlamPoseTracker,
    StopDetector,
)


def test_warming_up_holds_the_drive_until_the_first_processed_reply():
    tracker = SlamPoseTracker(max_age_s=0.25, hold_limit_m=1.5)
    assert tracker.mode == "warming_up" and not tracker.may_drive()
    with pytest.raises(ValueError):
        tracker.receive(0, 0.0, "skipped", (0, 0, 0))  # nothing to confirm yet
    tracker.receive(0, 0.0, "processed", (0, 0, 0))
    assert tracker.mode == "tracking" and tracker.may_drive()


def test_skipped_replies_keep_the_correction_and_prove_slam_is_alive():
    tracker = SlamPoseTracker(max_age_s=0.25, hold_limit_m=1.5)
    tracker.receive(0, 0.0, "processed", (0.1, 0, 0))
    for k in range(1, 20):
        # The bridge repeats the last correction; the tracker keeps its own.
        tracker.receive(k, 0.1 * k, "skipped", (9.9, 9.9, 9.9))
        np.testing.assert_allclose(tracker.map_from_base(0.1 * k + 0.05, (1, 0, 0)), (1.1, 0, 0))
    assert tracker.version == 1 and tracker.skipped_replies == 19
    with pytest.raises(LocalizationStale):
        tracker.map_from_base(1.9 + 0.26, (1, 0, 0))  # no reply at all for 0.26 s


def _tracking():
    tracker = SlamPoseTracker(max_age_s=0.25, hold_limit_m=1.5)
    tracker.receive(0, 0.0, "processed", (0, 0, 0))
    tracker.receive(4, 0.4, "processed", (0, 0, 0))
    return tracker


def test_tracking_applies_every_correction_and_goes_stale_after_the_limit():
    tracker = _tracking()
    tracker.receive(5, 0.5, "processed", (0.2, 0.0, 0.0))
    np.testing.assert_allclose(tracker.map_from_base(0.55, (1, 0, 0)), (1.2, 0, 0))
    tracker.map_from_base(0.75, (1, 0, 0))
    with pytest.raises(LocalizationStale):
        tracker.map_from_base(0.76, (1, 0, 0))


def test_holding_keeps_the_applied_correction_and_checks_freshness_on_the_received():
    tracker = _tracking()
    tracker.hold(odom_from_base=(0, 0, 0))
    tracker.receive(5, 0.5, "processed", (0.3, 0.0, 0.0))
    np.testing.assert_allclose(tracker.map_from_base(0.6, (0.1, 0, 0)), (0.1, 0, 0))  # still the old one
    with pytest.raises(LocalizationStale):
        tracker.map_from_base(0.76, (0.2, 0, 0))  # nothing received since 0.5
    tracker.receive(6, 0.7, "processed", (0.3, 0.0, 0.0))
    tracker.map_from_base(0.76, (0.2, 0, 0))


def test_holding_fails_after_too_much_dead_reckoning():
    tracker = _tracking()
    tracker.hold(odom_from_base=(0, 0, 0))
    tracker.receive(5, 0.5, "processed", (0, 0, 0))
    tracker.map_from_base(0.5, (1.4, 0, 0))
    with pytest.raises(LocalizationStale):
        tracker.map_from_base(0.5, (1.6, 0, 0))


def test_release_applies_the_received_correction_and_reports_the_jump():
    tracker = _tracking()
    tracker.hold(odom_from_base=(0, 0, 0))
    tracker.receive(5, 0.5, "processed", (0.03, 0.0, 0.01))
    jump_m, jump_rad = tracker.release(odom_from_base=(1.0, 0, 0))
    # The odometry position (1, 0) rotates with the 0.01 rad correction too.
    expected = math.hypot(0.03 + math.cos(0.01) - 1.0, math.sin(0.01))
    assert jump_m == pytest.approx(expected, abs=1e-12)
    assert jump_rad == pytest.approx(0.01)
    assert tracker.mode == "tracking"


def test_stop_detector_passes_true_rest_quickly_despite_wheel_noise():
    noise = OdometryNoise(seed=0)
    detector = StopDetector(tick_s=1 / 120)
    passes = []
    for k in range(240):
        rates = noise.wheel_rates((0.0, 0.0))
        speed = float(np.mean(rates)) * 0.135
        passes.append(detector.update(commanded_speed=0.0, speed=speed, yaw_rate=0.0))
    first = passes.index(True)
    assert first / 120 < 0.6  # 0.2 s command hold + 0.1 s window + 0.1 s run, with margin


def test_stop_detector_rejects_a_slow_creep():
    detector = StopDetector(tick_s=1 / 120)
    assert not any(
        detector.update(commanded_speed=0.0, speed=0.02, yaw_rate=0.0) for _ in range(240)
    )


def test_stop_detector_requires_a_zero_command():
    detector = StopDetector(tick_s=1 / 120)
    assert not any(
        detector.update(commanded_speed=0.05, speed=0.0, yaw_rate=0.0) for _ in range(240)
    )


def test_noise_streams_are_independent_and_reproducible():
    a, b = OdometryNoise(seed=3), OdometryNoise(seed=3)
    np.testing.assert_array_equal(a.wheel_rates((1.0, 1.0)), b.wheel_rates((1.0, 1.0)))
    np.testing.assert_array_equal(a.steering((0.1, 0.1)), b.steering((0.1, 0.1)))
    ranges = np.array([1.0, 5.0, np.inf, 11.99])
    noisy = a.ranges(ranges, range_min_m=0.2, range_max_m=12.0)
    assert np.isinf(noisy[2])
    assert np.all(noisy[np.isfinite(noisy)] <= 12.0)
    assert OdometryNoise(seed=3, enabled=False).wheel_rates((1.0, 1.0)) == pytest.approx((1.0, 1.0))


@pytest.mark.parametrize("speed, yaw_rate", [(-0.02, 0.0), (0.02, 0.0), (0.0, 0.03), (0.0, -0.03)])
def test_stop_detector_rejects_reversing_and_turning_in_either_sign(speed, yaw_rate):
    detector = StopDetector(tick_s=1 / 120)
    assert not any(
        detector.update(commanded_speed=0.0, speed=speed, yaw_rate=yaw_rate) for _ in range(240)
    )


def test_holding_counts_distance_travelled_not_displacement():
    # Insert 0.8 m forward, then back out 0.8 m: displacement 0, travel 1.6 m.
    tracker = _tracking()
    tracker.hold(odom_from_base=(0, 0, 0))
    tracker.receive(5, 0.5, "processed", (0, 0, 0))
    tracker.map_from_base(0.5, (0.8, 0, 0))
    with pytest.raises(LocalizationStale):
        tracker.map_from_base(0.5, (0.0, 0, 0))
    assert tracker.hold_travel_m == pytest.approx(1.6)


def test_tracker_speed_waits_for_the_stop_detector():
    detector = StopDetector(tick_s=1 / 120)
    detector.update(commanded_speed=0.0, speed=0.004, yaw_rate=0.0)
    # A lucky slow sample is not a stop: the tracker sees just over its threshold.
    assert abs(detector.tracker_speed(0.012)) > 0.012
    for _ in range(240):
        detector.update(commanded_speed=0.0, speed=0.0, yaw_rate=0.0)
    assert detector.stopped and detector.tracker_speed(0.012) == 0.0
    moving = StopDetector(tick_s=1 / 120)
    for _ in range(12):
        moving.update(commanded_speed=-0.2, speed=-0.2, yaw_rate=0.0)
    assert moving.tracker_speed(0.012) == pytest.approx(-0.2)


def test_tracker_speed_from_rest_follows_the_command_sign_not_the_noise():
    detector = StopDetector(tick_s=1 / 120)
    for _ in range(240):
        detector.update(commanded_speed=0.0, speed=0.0, yaw_rate=0.0)
    # Start a reverse leg; the noisy mean is still slightly positive.
    detector.update(commanded_speed=-0.003, speed=0.002, yaw_rate=0.0)
    assert not detector.stopped
    assert detector.tracker_speed(0.012) < 0
