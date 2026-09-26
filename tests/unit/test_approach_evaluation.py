"""Downstream truth evaluation is distinct from reaching the detected target."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.planning.pallet_mission import PalletSite, SyntheticMissionGeometry

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "approach_evaluation", ROOT / "sim/isaac/approach_evaluation.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def evaluate(**changes):
    values = dict(
        final_rear_pose=[0.81, 0.1, 0],
        observed_site=PalletSite(2.5, 0.1, 0),
        pallet_position_m=[2.5, 0.1, 0.015],
        pallet_yaw_rad=0,
        initial_pallet_xy_m=[2.5, 0.1],
    )
    return MODULE.evaluate_alignment(**(values | changes))


def test_correct_alignment_passes_all_reported_thresholds():
    result = evaluate()
    assert result["success"]
    assert set(result["errors"]) == set(result["thresholds"])
    assert all(error == pytest.approx(0) for error in result["errors"].values())
    assert result["thresholds"] == {
        "rear_position_m": 0.04,
        "rear_yaw_rad": 0.03,
        "detected_center_m": 0.03,
        "detected_yaw_rad": 0.02,
        "pallet_displacement_m": 0.018,
    }
    json.dumps(result, allow_nan=False)


def test_arriving_at_wrong_observed_target_does_not_pass_truth_evaluation():
    result = evaluate(
        final_rear_pose=[0.91, 0.1, 0], observed_site=PalletSite(2.6, 0.1, 0)
    )
    assert not result["success"]
    assert result["errors"]["rear_position_m"] == pytest.approx(0.1)
    assert result["errors"]["detected_center_m"] == pytest.approx(0.1)


def test_pallet_motion_fails_even_if_detection_and_final_alignment_agree():
    result = evaluate(
        final_rear_pose=[0.83, 0.1, 0],
        observed_site=PalletSite(2.52, 0.1, 0),
        pallet_position_m=[2.52, 0.1, 0.015],
    )
    assert not result["success"]
    assert result["errors"]["rear_position_m"] == pytest.approx(0)
    assert result["errors"]["detected_center_m"] == pytest.approx(0)
    assert result["errors"]["pallet_displacement_m"] == pytest.approx(0.02)


def test_yaw_errors_wrap_at_pi_for_both_detection_and_vehicle():
    yaw = math.pi - 0.005
    result = evaluate(
        final_rear_pose=[
            2.5 - 1.69 * math.cos(yaw),
            0.1 - 1.69 * math.sin(yaw),
            -math.pi + 0.005,
        ],
        observed_site=PalletSite(2.5, 0.1, -math.pi + 0.005),
        pallet_yaw_rad=yaw,
    )
    assert result["success"]
    assert result["errors"]["rear_yaw_rad"] == pytest.approx(0.01)
    assert result["errors"]["detected_yaw_rad"] == pytest.approx(0.01)


def test_true_pallet_heading_rotates_rear_axle_offset():
    result = evaluate(
        final_rear_pose=[2, 1.31, math.pi / 2],
        observed_site=PalletSite(2, 3, math.pi / 2),
        pallet_position_m=[2, 3, 0.02],
        initial_pallet_xy_m=[2, 3],
        pallet_yaw_rad=math.pi / 2,
    )
    assert result["success"]
    assert result["errors"]["rear_position_m"] == pytest.approx(0)


@pytest.mark.parametrize(
    "changes,error",
    [
        ({"final_rear_pose": [0.81, 0.1, 0.031]}, "rear_yaw_rad"),
        ({"observed_site": PalletSite(2.5, 0.1, 0.021)}, "detected_yaw_rad"),
        ({"final_rear_pose": [0.81, 0.141, 0]}, "rear_position_m"),
        ({"observed_site": PalletSite(2.5, 0.131, 0)}, "detected_center_m"),
    ],
)
def test_each_error_gate_can_fail_independently(changes, error):
    result = evaluate(**changes)
    assert not result["success"]
    assert result["errors"][error] > result["thresholds"][error]
    assert all(
        value <= result["thresholds"][key]
        for key, value in result["errors"].items()
        if key != error
    )


def test_thresholds_can_be_set_explicitly_and_boundary_is_inclusive():
    result = evaluate(
        final_rear_pose=[0.85, 0.1, 0.03], observed_site=PalletSite(2.53, 0.1, 0.02)
    )
    assert result["success"]
    relaxed = evaluate(
        final_rear_pose=[0.91, 0.1, 0],
        observed_site=PalletSite(2.6, 0.1, 0),
        max_rear_position_error_m=0.15,
        max_detected_center_error_m=0.15,
    )
    assert relaxed["success"]


def test_explicit_robot_geometry_and_last_pose_of_trajectory_are_used():
    result = evaluate(
        final_rear_pose=[[0, 0, 0], [0.7, 0.1, 0]],
        geometry=SyntheticMissionGeometry(axle_to_fork_tip_m=1.40),
    )
    assert result["success"]


@pytest.mark.parametrize(
    "changes",
    [
        {"final_rear_pose": []},
        {"final_rear_pose": [0, 0]},
        {"final_rear_pose": [[0, 0], [1, 1]]},
        {"final_rear_pose": [[float("nan"), 0, 0], [0.81, 0.1, 0]]},
        {"final_rear_pose": [0.81, 0.1, float("inf")]},
        {"final_rear_pose": [True, False, True]},
        {"pallet_position_m": [2.5, 0.1]},
        {"pallet_position_m": [2.5, 0.1, float("nan")]},
        {"initial_pallet_xy_m": [2.5, 0.1, 0]},
        {"initial_pallet_xy_m": [float("inf"), 0.1]},
        {"pallet_yaw_rad": float("nan")},
        {"pallet_yaw_rad": True},
        {"max_rear_position_error_m": -0.01},
        {"max_rear_yaw_error_rad": float("inf")},
        {"max_detected_center_error_m": True},
        {"max_detected_yaw_error_rad": "0.02"},
        {"max_pallet_displacement_m": np.nan},
        {"observed_site": None},
    ],
)
def test_malformed_inputs_are_rejected(changes):
    with pytest.raises(ValueError):
        evaluate(**changes)
