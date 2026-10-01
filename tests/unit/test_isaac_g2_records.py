"""SDK-free rules of the G2 rerun (plan docs/plans/2026-10-01-g2-rerun.md)."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "g2_records", ROOT / "sim/isaac/g2_records.py"
)
G = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(G)


def test_oracle_substitutes_only_the_target():
    assert G.planning_target("perception", "est", "nom") == (
        "est",
        "camera_depth_detection",
    )
    assert G.planning_target("oracle_nominal", "est", "nom") == (
        "nom",
        "scenario_nominal_pickup",
    )
    with pytest.raises(ValueError):
        G.planning_target("oracle_actual", "est", "nom")


def test_site_error_wraps_yaw():
    error = G.site_error((1.0, 0.0, math.pi - 0.01), (1.003, 0.004, -math.pi + 0.01))
    assert error["position_m"] == pytest.approx(0.005)
    assert error["yaw_rad"] == pytest.approx(-0.02)


def test_depth_hash_sees_nan_pattern_and_dtype():
    depth = np.ones((4, 4), dtype=np.float32)
    other = depth.copy()
    other[0, 0] = np.nan
    assert G.depth_sha256(depth) == G.depth_sha256(depth.copy())
    assert G.depth_sha256(depth) != G.depth_sha256(other)
    assert G.depth_sha256(depth) != G.depth_sha256(depth.astype(np.float64))


def handoff(**changes):
    record = {
        "attempt_number": 2,
        "candidate_index": 1,
        "depth_sha256": "abc",
        "loop_step": 1400,
        "accepted_pose": [[1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        "linear_velocity_mps": [0.001, 0.0, 0.0],
        "angular_velocity_radps": [0.0, 0.0, 0.0001],
        "steering_rad": [0.01, 0.01],
        "pallet_position_m": [2.86, 0.31, 0.0],
        "pallet_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "simulation_time_s": 11.9,
        "t_before_capture_s": 11.8,
    }
    record.update(changes)
    return record


def test_handoff_corresponds_only_when_every_item_matches():
    assert G.handoff_correspondence(handoff(), handoff())["status"] == "corresponds"
    for change in (
        {"depth_sha256": "abd"},
        {"loop_step": 1401},
        {"candidate_index": 2},
        {"accepted_pose": [[1.0, 2.0 + 2e-6, 0.0], [1.0, 0.0, 0.0, 0.0]]},
        {"steering_rad": [0.01, 0.0101]},
        {"t_before_capture_s": 11.8 + 1e-6},
        {"pallet_orientation_wxyz": [0.99, 0.0, 0.0, 0.14]},
    ):
        result = G.handoff_correspondence(handoff(), handoff(**change))
        assert result["status"] == "does_not_correspond"
        assert result["mismatched"] == list(change)
    assert G.handoff_correspondence(None, None)["status"] == "no_handoff"
    assert G.handoff_correspondence(handoff(), None)["status"] == "one_sided"


def test_undetected_seeds_correspond_by_every_attempt():
    attempts = [
        {"depth_sha256": "a", "candidate_index": 0},
        {"depth_sha256": "b", "candidate_index": 1},
    ]
    assert G.attempts_correspond(attempts, [dict(a) for a in attempts])
    assert not G.attempts_correspond(attempts, attempts[:1])
    assert not G.attempts_correspond(
        attempts, [attempts[0], {"depth_sha256": "c", "candidate_index": 1}]
    )
    assert not G.attempts_correspond([{"depth_sha256": None}], [{"depth_sha256": None}])


def test_repeat_batch_corresponds_by_candidate_depth_and_pose():
    a = {
        "candidate_index": 1,
        "depth_sha256": "x",
        "accepted_pose": [[0, 0, 0], [1, 0, 0, 0]],
    }
    assert G.repeat_correspondence(a, dict(a))["status"] == "corresponds"
    b = dict(a, accepted_pose=[[0, 1e-5, 0], [1, 0, 0, 0]])
    assert G.repeat_correspondence(a, b)["mismatched"] == ["accepted_pose"]


def test_depth_change_separates_values_from_validity_flips():
    first = np.array([[1.0, 2.0, np.nan], [3.0, 0.0, 4.0]])
    other = np.array([[1.0, 2.5, 1.0], [np.nan, 0.0, 4.0]])
    change = G.depth_change(first, other)
    assert change == {
        "common_valid_pixels": 3,
        "changed_pixels": 1,
        "max_abs_change_m": 0.5,
        "mean_abs_change_m": pytest.approx(0.5 / 3),
        "valid_to_invalid": 1,
        "invalid_to_valid": 1,
    }


def quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def test_drift_is_cumulative_from_the_first_pose():
    first = ([0.0, 0.0, 0.0], quat(0.0))
    assert not G.drift(first, ([0.0009, 0.0, 0.0], quat(0.0009)))["moved"]
    assert G.drift(first, ([0.0011, 0.0, 0.0], quat(0.0)))["moved"]
    assert G.drift(first, ([0.0, 0.0, 0.0], quat(0.0011)))["moved"]
    wrapped = G.drift(
        ([0, 0, 0], quat(math.pi - 0.0002)), ([0, 0, 0], quat(-math.pi + 0.0002))
    )
    assert wrapped["yaw_rad"] == pytest.approx(0.0004)


def cap(
    index,
    status="valid",
    left=(3.0, 0.2, 0.06),
    right=(3.0, -0.2, 0.06),
    yaw=0.1,
    moved=False,
):
    record = {"index": index, "status": status, "drift": {"moved": moved}}
    if status == "valid":
        record.update(
            left_center_m=list(left), right_center_m=list(right), insertion_yaw_rad=yaw
        )
    return record


def test_identical_repeats_give_zero_spread():
    summary = G.repeat_summary([cap(i) for i in range(11)])
    assert summary["valid"] == 11 and summary["status_changes"] == 0
    assert summary["left_std_m"] == 0 and summary["yaw_max_dev_rad"] == 0
    assert summary["original_valid"] and summary["reference_index"] == 0


def test_spread_is_measured_from_the_first_valid_detection():
    captures = [
        cap(0, status="invalid"),
        cap(1, yaw=0.10),
        cap(2, left=(3.002, 0.2, 0.06), yaw=math.pi + 0.2),
    ]
    summary = G.repeat_summary(captures)
    assert summary["reference_index"] == 1 and not summary["original_valid"]
    assert summary["left_max_dev_m"] == pytest.approx(0.002)
    assert summary["yaw_max_dev_rad"] == pytest.approx(abs(G.wrap_angle(math.pi + 0.1)))
    assert summary["status_changes"] == 1


def test_moved_captures_are_left_out_and_one_valid_is_null():
    captures = [
        cap(0),
        cap(1, left=(3.5, 0.2, 0.06), moved=True),
        cap(2, status="invalid"),
    ]
    summary = G.repeat_summary(captures)
    assert summary["moved"] == 1 and summary["kept"] == 2 and summary["valid"] == 1
    assert summary["left_std_m"] is None and summary["reference_index"] == 0


def test_repeat_correspondence_reads_the_runner_record_shape():
    """Codex: the runner keeps the pose under capture_diagnostics."""
    pose = [[1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0]]
    nested = {
        "candidate_index": 1,
        "depth_sha256": "x",
        "capture_diagnostics": {"accepted_pose": pose},
    }
    top = {"candidate_index": 1, "depth_sha256": "x", "accepted_pose": pose}
    assert G.repeat_correspondence(nested, top)["status"] == "corresponds"
    bare = {"candidate_index": 1, "depth_sha256": "x"}
    assert G.repeat_correspondence(nested, bare)["mismatched"] == ["accepted_pose"]
