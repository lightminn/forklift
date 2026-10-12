"""Kinematic dry run used to accept a near-capture start (plan v3.7)."""

import numpy as np

from forklift_core.control import TrackerConfig
from forklift_core.control.rollout import bicycle_rollout


def _straight(length=0.8, step=0.04):
    n = int(round(length / step)) + 1
    xs = np.linspace(0.0, length, n)
    return np.column_stack((xs, np.zeros(n), np.zeros(n))), np.ones(n, np.int8), np.zeros(n)


CONFIG = TrackerConfig(
    cruise_speed_mps=0.18,
    max_curvature_inv_m=0.9,
    max_acceleration_mps2=0.3,
    lookahead_m=0.28,
    position_tolerance_m=0.008,
    yaw_tolerance_rad=0.02,
    stop_speed_mps=0.012,
)


def test_an_aligned_start_arrives_with_no_error():
    result = bicycle_rollout(*_straight(), CONFIG, (0.0, 0.0, 0.0))
    assert result.status == "arrived"
    assert result.position_error_m < 0.008 and abs(result.yaw_error_rad) < 0.005


def test_a_large_offset_does_not_arrive_cleanly():
    result = bicycle_rollout(*_straight(), CONFIG, (0.0, 0.08, 0.06))
    assert result.status != "arrived" or abs(result.yaw_error_rad) > 0.015


def test_the_rollout_reports_the_poses_it_drove_through():
    result = bicycle_rollout(*_straight(), CONFIG, (0.0, 0.03, 0.0))
    poses = np.array(result.trajectory)
    assert len(poses) > 10
    np.testing.assert_allclose(poses[0], (0.0, 0.03, 0.0))
    assert poses[-1][0] > 0.7


# Cusp position stall (docs/plans/2026-10-04-lidar-obstacle-map.md, "cusp 위치 정체 복구 설계 7판").
from forklift_core.control.rollout import leg_prefix_rollout  # noqa: E402

CUSP_CONFIG = TrackerConfig(
    cruise_speed_mps=0.18,
    max_curvature_inv_m=0.9,
    max_acceleration_mps2=0.3,
    lookahead_m=0.28,
    position_tolerance_m=0.008,
    yaw_tolerance_rad=0.02,
    stop_speed_mps=0.012,
    cusp_position_tolerance_m=0.03,
    cusp_yaw_tolerance_rad=0.05,
    cusp_brake_window_m=0.008,
)


def _forward_then_reverse(reverse_m):
    xs = np.linspace(0.0, 1.0, 26)
    back = 1.0 - np.linspace(0.04, reverse_m, int(round(reverse_m / 0.04)))
    x = np.r_[xs, back]
    poses = np.column_stack((x, np.zeros(len(x)), np.zeros(len(x))))
    return poses, np.r_[np.ones(len(xs)), -np.ones(len(back))].astype(np.int8), np.zeros(len(x))


def test_the_next_leg_prefix_converges_to_the_reference_from_an_offset_stop():
    path = _forward_then_reverse(4.0)
    reference = leg_prefix_rollout(*path, CUSP_CONFIG, path[0][25], start_leg=1, prefix_m=3.0)
    assert reference.status == "prefix" and abs(reference.lateral_m) < 1e-9 and abs(reference.heading_rad) < 1e-9
    offset = leg_prefix_rollout(*path, CUSP_CONFIG, (1.0, 0.04, 0.0), start_leg=1, prefix_m=3.0)
    # Compared at exactly the same progress (Codex cusp implementation review P2).
    assert offset.status == "prefix" and offset.progress_m == reference.progress_m == 3.0
    assert abs(offset.lateral_m - reference.lateral_m) <= 0.01 and abs(offset.heading_rad - reference.heading_rad) <= 0.01
    early = leg_prefix_rollout(*path, CUSP_CONFIG, (1.0, 0.04, 0.0), start_leg=1, prefix_m=0.2)
    assert abs(early.lateral_m) > 0.01  # not converged yet that early
    assert len(offset.trajectory) > 10


def test_a_short_next_leg_is_judged_at_its_own_end_with_its_own_tolerances():
    # The next leg ends at the goal 0.6 m back: the goal's 8 mm / 20 mrad, not the cusp's.
    path = _forward_then_reverse(0.6)
    clean = leg_prefix_rollout(*path, CUSP_CONFIG, path[0][25], start_leg=1, prefix_m=3.0)
    assert clean.status == "arrived" and clean.leg_end_is_goal
    rough = leg_prefix_rollout(*path, CUSP_CONFIG, (1.0, 0.04, 0.0), start_leg=1, prefix_m=3.0)
    assert rough.status == "failed" and rough.failure == "endpoint_heading" and rough.leg_end_is_goal
    # A next leg ending at a cusp is accepted at that cusp with the cusp tolerances.
    xs = np.linspace(0.0, 1.0, 26)
    back = 1.0 - np.linspace(0.04, 0.6, 15)
    ahead = back[-1] + np.linspace(0.04, 1.0, 25)
    x = np.r_[xs, back, ahead]
    three = (np.column_stack((x, np.zeros(len(x)), np.zeros(len(x)))),
             np.r_[np.ones(26), -np.ones(15), np.ones(25)].astype(np.int8), np.zeros(len(x)))
    at_cusp = leg_prefix_rollout(*three, CUSP_CONFIG, (1.0, 0.02, 0.0), start_leg=1, prefix_m=3.0)
    assert at_cusp.status == "leg_end" and not at_cusp.leg_end_is_goal


def test_the_prefix_offsets_are_interpolated_to_the_prefix_on_a_curve():
    # A 0.29 1/m reverse arc after the cusp: the two runs stop at different ticks past 3 m, and
    # their offsets are read at 3 m exactly, between the two checked poses either side.
    xs = np.linspace(0.0, 1.0, 26)
    k = 0.29
    s_ = np.linspace(0.0, 4.0, 101)[1:]
    arc = np.column_stack((1.0 - np.sin(k * s_) / k, -(1.0 - np.cos(k * s_)) / k, -k * s_))
    poses = np.vstack((np.column_stack((xs, np.zeros(26), np.zeros(26))), arc))
    directions = np.r_[np.ones(26), -np.ones(100)].astype(np.int8)
    curvatures = np.r_[np.zeros(26), np.full(100, k)]
    path = (poses, directions, curvatures)
    reference = leg_prefix_rollout(*path, CUSP_CONFIG, poses[25], start_leg=1, prefix_m=3.0)
    fine = leg_prefix_rollout(*path, CUSP_CONFIG, poses[25], start_leg=1, prefix_m=3.0, dt_s=1.0 / 1200.0)
    assert reference.status == fine.status == "prefix" and reference.progress_m == 3.0
    # Interpolated, the reading does not depend on where the tick fell (to the model's own dt error).
    assert abs(reference.lateral_m - fine.lateral_m) < 1e-3 and abs(reference.heading_rad - fine.heading_rad) < 2e-3
