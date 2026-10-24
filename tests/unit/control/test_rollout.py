"""Kinematic dry run used to accept a near-capture start (plan v3.7)."""

import numpy as np
import pytest

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


def test_a_short_last_leg_missed_at_the_goal_ends_in_a_goal_stall():
    # Cusp design amendment, on the recorded case (S4 dev rerun, seed 4 return): from the stall at the
    # end of leg 1 the last leg (1.6 m) ends standing at the goal 68 mm off -- inside the goal's brake
    # window it neither arrives nor fails; at the end of the budget the dry run reports the stall the
    # runner's final-goal stall rule takes (the ideal rollout's remaining length only decays).
    import json
    from pathlib import Path

    case = json.loads((Path(__file__).resolve().parents[2] / "fixtures/cusp_goal_stall/seed4_return.json").read_text())
    path = tuple(np.asarray(case["path"][k], dtype=float) for k in ("poses", "directions", "curvatures_inv_m"))
    stall = leg_prefix_rollout(*path, TrackerConfig(**case["config"]), case["rear"], start_leg=2, prefix_m=3.0)
    assert stall.status == "goal_stall" and stall.leg_end_is_goal
    assert stall.lateral_m == pytest.approx(-0.068, abs=0.001)


def test_a_late_arrival_just_inside_the_tolerance_is_not_a_goal_stall():
    # Codex review of the amendment: the offset across the goal can still shrink while the remaining
    # length decays -- 8.04 mm early on, then an arrival at 7.9999 mm; judged only at the budget's end
    # it stays an arrival.
    import json
    from pathlib import Path

    case = json.loads((Path(__file__).resolve().parents[2] / "fixtures/cusp_goal_stall/seed4_return.json").read_text())
    config = TrackerConfig(**case["config"])
    xs = np.linspace(0.0, 1.0, 26)
    back = np.array([1.0, 0.99, 0.98])[1:]
    x = np.r_[xs, back]
    path = (np.column_stack((x, np.zeros(len(x)), np.zeros(len(x)))),
            np.r_[np.ones(len(xs)), -np.ones(len(back))].astype(np.int8), np.zeros(len(x)))
    late = leg_prefix_rollout(*path, config, (1.04, 0.0093, 0.02), start_leg=1, prefix_m=3.0)
    assert late.status == "arrived"


def test_a_slow_reverse_leg_still_approaching_at_the_budgets_end_is_not_a_goal_stall():
    # Codex review of the amendment: under max_reverse_speed_mps 0.07 the cruise-based budget ended with
    # 2 mm still to go and the truck arriving 1.8 s later; the budget now follows the leg's own speed cap
    # and a stall needs the remaining length all but gone.
    import json
    from dataclasses import replace
    from pathlib import Path

    from forklift_core.control.rollout import GOAL_STALL_REMAINING_M

    case = json.loads((Path(__file__).resolve().parents[2] / "fixtures/cusp_goal_stall/seed4_return.json").read_text())
    path = tuple(np.asarray(case["path"][k], dtype=float) for k in ("poses", "directions", "curvatures_inv_m"))
    slow = replace(TrackerConfig(**case["config"]), max_reverse_speed_mps=0.07)
    start = (-0.7719976411860825, 0.4086998744060603, 0.44737152281226444)
    out = leg_prefix_rollout(*path, slow, start, start_leg=2, prefix_m=3.0)
    assert out.status == "arrived"
    assert GOAL_STALL_REMAINING_M == 1e-4


# --- the curvature response (docs/plans/2026-10-24-curvature-response.md arm A) ----------------------------------

def _arc(radius=3.0, angle=0.6, step=0.02, direction=1):
    n = int(round(radius * angle / step)) + 1
    th = np.linspace(0.0, angle, n)
    poses = np.column_stack((radius * np.sin(th), radius * (1 - np.cos(th)), th))
    return poses, np.full(n, direction, np.int8), np.full(n, 1.0 / radius)


ARC_CONFIG = TrackerConfig(cruise_speed_mps=0.3, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3,
                           lookahead_m=0.28, position_tolerance_m=0.008, yaw_tolerance_rad=0.02, stop_speed_mps=0.012)


def test_without_a_response_the_rollout_is_the_ideal_one_bit_for_bit():
    from forklift_core.control.rollout import CurvatureResponse

    ideal = bicycle_rollout(*_arc(), ARC_CONFIG, (0.0, 0.0, 0.0))
    for unit in (None, CurvatureResponse(1.0, 1.0)):
        again = bicycle_rollout(*_arc(), ARC_CONFIG, (0.0, 0.0, 0.0), curvature_response=unit)
        assert again == ideal


def test_a_forward_shortfall_realises_less_turn_and_the_tracker_compensates_up_to_its_cap():
    from forklift_core.control.rollout import CurvatureResponse

    ideal = bicycle_rollout(*_arc(), ARC_CONFIG, (0.0, 0.0, 0.0))
    short = bicycle_rollout(*_arc(), ARC_CONFIG, (0.0, 0.0, 0.0), curvature_response=CurvatureResponse(0.88, 1.0))
    # the arc's 1/3 needs 0.379 commanded at 0.88: above the 0.36 cap, so it cannot be held
    assert ideal.status == "arrived"
    assert short.status != "arrived" or abs(short.yaw_error_rad) > abs(ideal.yaw_error_rad)


def test_the_response_is_chosen_by_the_gear_of_each_step():
    from forklift_core.control.rollout import CurvatureResponse

    reverse = _arc(direction=-1)
    poses = reverse[0].copy()
    # a reverse arc driven backwards from its end pose: the forward factor must not touch it
    start = tuple(poses[0])
    a = bicycle_rollout(*reverse, ARC_CONFIG, start, curvature_response=CurvatureResponse(0.5, 1.0))
    b = bicycle_rollout(*reverse, ARC_CONFIG, start)
    assert a == b


def test_the_prefix_rollout_takes_the_same_response():
    from forklift_core.control.rollout import CurvatureResponse, leg_prefix_rollout

    path = _arc()
    ideal = leg_prefix_rollout(*path, ARC_CONFIG, (0.0, 0.0, 0.0), start_leg=0, prefix_m=0.8)
    same = leg_prefix_rollout(*path, ARC_CONFIG, (0.0, 0.0, 0.0), start_leg=0, prefix_m=0.8,
                              curvature_response=CurvatureResponse(1.0, 1.0))
    short = leg_prefix_rollout(*path, ARC_CONFIG, (0.0, 0.0, 0.0), start_leg=0, prefix_m=0.8,
                               curvature_response=CurvatureResponse(0.88, 1.0))
    assert same == ideal and abs(short.lateral_m) > abs(ideal.lateral_m)
