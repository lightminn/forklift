import dataclasses
import math

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception.evaluation import Outcome, evaluate_scene, summarize
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.perception.scene_dataset import SceneInput, SceneSample
from forklift_core.sensors.rgbd import PinholeIntrinsics


def test_an_accurate_estimate_on_a_positive_scene_is_a_true_positive():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.01), elapsed_s=0.2)
    assert result.outcome is Outcome.TRUE_POSITIVE
    assert result.position_error_m == pytest.approx(0.01, abs=1e-9)
    assert result.yaw_error_rad == pytest.approx(0.0, abs=1e-9)
    assert result.elapsed_s == 0.2


def test_the_scene_position_error_is_the_worse_of_the_two_pockets():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.01, right_extra=0.03))
    assert result.left_error_m == pytest.approx(0.01)
    assert result.right_error_m == pytest.approx(0.04)
    assert result.position_error_m == pytest.approx(0.04)


def test_a_valid_but_far_estimate_is_wrong_pose_not_a_true_positive():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.5))
    assert result.outcome is Outcome.WRONG_POSE and result.position_error_m > 0.2


def test_an_error_exactly_at_the_threshold_still_counts_as_a_true_positive():
    result = evaluate_scene(sample("positive"), shifted(OBS, 0.20))
    assert result.outcome is Outcome.TRUE_POSITIVE


@pytest.mark.parametrize("yaw", [-1.0, 1.0])
def test_yaw_is_compared_by_absolute_wrapped_difference(yaw):
    result = evaluate_scene(sample("positive"), rotated(OBS, yaw))
    assert result.outcome is Outcome.WRONG_POSE
    assert result.yaw_error_rad == pytest.approx(1.0, abs=1e-9)


def test_yaw_differences_across_pi_wrap_to_the_short_way_round():
    result = evaluate_scene(
        sample("positive", truth_yaw=math.pi - 0.05), rotated(OBS, -math.pi + 0.05)
    )
    assert result.yaw_error_rad == pytest.approx(0.10, abs=1e-9)
    assert result.outcome is Outcome.TRUE_POSITIVE


@pytest.mark.parametrize(
    "category,estimate,expected",
    [
        ("positive", "valid_close", Outcome.TRUE_POSITIVE),
        ("positive", "valid_far", Outcome.WRONG_POSE),
        ("positive", "no_pallet", Outcome.FALSE_NEGATIVE),
        ("positive", "invalid", Outcome.INVALID),
        ("occluded", "valid_close", Outcome.TRUE_POSITIVE),
        ("occluded", "valid_far", Outcome.WRONG_POSE),
        ("occluded", "no_pallet", Outcome.FALSE_NEGATIVE),
        ("occluded", "invalid", Outcome.INVALID),
        ("negative_no_pallet", "valid_close", Outcome.FALSE_POSITIVE),
        ("negative_no_pallet", "no_pallet", Outcome.TRUE_NEGATIVE),
        ("negative_no_pallet", "invalid", Outcome.INVALID),
        ("negative_lookalike", "valid_close", Outcome.FALSE_POSITIVE),
        ("negative_lookalike", "no_pallet", Outcome.TRUE_NEGATIVE),
        ("negative_lookalike", "invalid", Outcome.INVALID),
    ],
)
def test_the_full_judgment_table(category, estimate, expected):
    assert evaluate_scene(sample(category), estimate_for(estimate)).outcome is expected


def test_a_false_positive_on_a_negative_scene_has_no_pose_error():
    result = evaluate_scene(sample("negative_lookalike"), OBS)
    assert result.outcome is Outcome.FALSE_POSITIVE
    assert result.position_error_m is None and result.yaw_error_rad is None


def test_detection_rate_uses_the_whole_category_as_denominator():
    results = [
        evaluate_scene(sample("positive", scene_id=f"s{i:03d}"), shifted(OBS, 0.01))
        for i in range(17)
    ]
    results.append(
        evaluate_scene(sample("positive", scene_id="s018"), estimate_for("invalid"))
    )
    summary = summarize(results)
    assert summary["detection_rate"]["positive"] == pytest.approx(17 / 18)
    assert summary["targets"]["met"]["detection_rate_positive"] is False
    assert summary["counts"]["positive"]["invalid"] == 1
    assert summary["counts"]["positive"]["false_negative"] == 0


def test_false_positive_rate_counts_invalid_negatives_in_the_denominator():
    results = [
        evaluate_scene(sample("negative_no_pallet", scene_id="s001"), OBS),
        evaluate_scene(
            sample("negative_no_pallet", scene_id="s002"), estimate_for("invalid")
        ),
        evaluate_scene(
            sample("negative_no_pallet", scene_id="s003"), estimate_for("no_pallet")
        ),
    ]
    assert summarize(results)["false_positive_rate"][
        "negative_no_pallet"
    ] == pytest.approx(1 / 3)


def test_error_distribution_covers_every_valid_output_not_only_successes():
    close = evaluate_scene(sample("positive", scene_id="s001"), shifted(OBS, 0.01))
    far = evaluate_scene(sample("positive", scene_id="s002"), shifted(OBS, 0.5))
    summary = summarize([close, far])
    assert summary["position_error_m"]["all_valid"]["n"] == 2
    assert summary["position_error_m"]["all_valid"]["max"] == pytest.approx(0.5)
    assert summary["position_error_m"]["true_positive"]["n"] == 1
    assert summary["position_error_m"]["true_positive"]["max"] == pytest.approx(0.01)
    assert summary["position_error_m"]["positive"]["n"] == 2


def test_empty_samples_give_null_quantiles_and_never_meet_a_target():
    summary = summarize([evaluate_scene(sample("positive"), estimate_for("no_pallet"))])
    stats = summary["position_error_m"]["all_valid"]
    assert stats["n"] == 0 and stats["p50"] is None and stats["p95"] is None
    assert summary["targets"]["met"]["position_p95_positive"] is False
    assert summary["targets"]["met"]["yaw_p95_positive"] is False


def test_summarize_of_no_results_is_defined():
    summary = summarize([])
    assert summary["scene_count"] == 0 and summary["counts"] == {}
    assert summary["detection_rate"]["positive"] is None
    assert all(value is False for value in summary["targets"]["met"].values())


def test_elapsed_times_without_a_measurement_are_excluded():
    a = evaluate_scene(
        sample("positive", scene_id="s001"), shifted(OBS, 0.01), elapsed_s=0.4
    )
    b = evaluate_scene(sample("positive", scene_id="s002"), shifted(OBS, 0.01))
    assert summarize([a, b])["elapsed_s"]["n"] == 1


OBS = PocketObservation(
    stamp_ns=2_000_000_000,
    clock_domain="synthetic",
    frame_id="base_link",
    source_provenance="synthetic",
    status="valid",
    left=Pocket((0.0, 0.17, 0.15), 0.24, 0.20),
    right=Pocket((0.0, -0.17, 0.15), 0.24, 0.20),
    insertion_yaw_rad=0.0,
    position_sigma_m=None,
    yaw_sigma_rad=None,
    reason=None,
)


def shifted(obs, dx, *, right_extra=0.0):
    def move(pocket, amount):
        x, y, z = pocket.center_m
        return dataclasses.replace(pocket, center_m=(x + amount, y, z))

    return dataclasses.replace(
        obs, left=move(obs.left, dx), right=move(obs.right, dx + right_extra)
    )


def rotated(obs, yaw):
    def turn(pocket):
        x, y, z = pocket.center_m
        return dataclasses.replace(
            pocket,
            center_m=(
                math.cos(yaw) * x - math.sin(yaw) * y,
                math.sin(yaw) * x + math.cos(yaw) * y,
                z,
            ),
        )

    return dataclasses.replace(
        obs, left=turn(obs.left), right=turn(obs.right), insertion_yaw_rad=yaw
    )


def estimate_for(kind):
    if kind == "valid_close":
        return shifted(OBS, 0.01)
    if kind == "valid_far":
        return shifted(OBS, 0.5)
    return dataclasses.replace(
        OBS,
        status=kind,
        left=None,
        right=None,
        insertion_yaw_rad=None,
        reason="test_observation_loss",
    )


def sample(category, *, truth_status=None, truth_yaw=0.0, split="dev", scene_id="s001"):
    if truth_status is None:
        truth_status = "valid" if category in ("positive", "occluded") else "no_pallet"
    truth = (
        rotated(OBS, truth_yaw)
        if truth_status == "valid"
        else estimate_for(truth_status)
    )
    truth = dataclasses.replace(truth, source_provenance="synthetic_ground_truth")
    dummy = SceneInput(
        rgb=np.zeros((1, 1, 3), dtype=np.uint8),
        depth_m=np.ones((1, 1), dtype=np.float64),
        intrinsics=PinholeIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0, "camera_optical_frame"),
        base_from_optical=RigidTransform(
            "camera_optical_frame", "base_link", np.eye(3), (0, 0, 0)
        ),
        stamp_ns=OBS.stamp_ns,
        clock_domain=OBS.clock_domain,
        source_provenance="synthetic",
    )
    return SceneSample(
        dummy, truth, {"scene_id": scene_id, "category": category, "split": split}
    )


@pytest.mark.parametrize("yaw", [-0.35, 0.35])
def test_yaw_threshold_is_inclusive_on_both_sides(yaw):
    result = evaluate_scene(sample("positive"), rotated(OBS, yaw))
    assert result.outcome is Outcome.TRUE_POSITIVE


def test_position_threshold_is_inclusive_away_from_the_coordinate_origin():
    scene = sample("positive")
    scene = dataclasses.replace(scene, ground_truth=shifted(scene.ground_truth, 2.2))
    estimate = shifted(shifted(OBS, 2.2), 0.20)
    assert evaluate_scene(scene, estimate).outcome is Outcome.TRUE_POSITIVE


@pytest.mark.parametrize(
    "estimate", [shifted(OBS, 0.20 + 1e-8), rotated(OBS, 0.35 + 1e-8)]
)
def test_roundoff_handling_does_not_accept_errors_beyond_the_threshold(estimate):
    assert evaluate_scene(sample("positive"), estimate).outcome is Outcome.WRONG_POSE


def test_bare_blocks_score_as_a_negative_not_as_an_unbudgeted_invalid():
    """negative_block_row is registered everywhere a category must be.

    The plan worried this negative would land in `invalid`, which has no
    budget, and quietly cost a true negative. With `lower` still in the gate it
    returns no_pallet instead, so it scores as the negative it is -- but only
    because the category is registered; an unregistered one raises instead.
    """
    from forklift_core.perception.evaluation import (
        NEGATIVE_CATEGORIES,
        POSITIVE_CATEGORIES,
    )

    assert "negative_block_row" in NEGATIVE_CATEGORIES
    assert "negative_block_row" not in POSITIVE_CATEGORIES


def test_every_category_the_world_generator_emits_is_known_to_the_evaluator():
    """Four places register a category and they drift apart silently.

    A scene the generator can build but the evaluator refuses fails only once
    the dataset exists, which is after the capture that produced it.
    """
    import importlib.util
    import sys
    from pathlib import Path

    from forklift_core.perception.evaluation import (
        NEGATIVE_CATEGORIES,
        POSITIVE_CATEGORIES,
    )

    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "merge_for_categories", root / "tools/merge_scene_batches.py"
    )
    merge = importlib.util.module_from_spec(spec)
    sys.modules["merge_for_categories"] = merge
    spec.loader.exec_module(merge)

    known = set(POSITIVE_CATEGORIES) | set(NEGATIVE_CATEGORIES)
    assert set(merge.CATEGORY_STATUS) == known
    source = (root / "sim/gazebo/build_scene_world.py").read_text()
    for category in known:
        assert f'"{category}"' in source, category
