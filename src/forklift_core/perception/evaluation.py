"""Pure scene judgment and aggregation, with explicit denominators and nulls."""

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pocket_observation import (
    PocketObservation,
    yaw_difference_rad,
)
from forklift_core.perception.scene_dataset import SceneSample

POSITIVE_CATEGORIES = ("positive", "occluded")
# negative_block_row is nine bare blocks: two openings, no deck over either.
# It is the negative that tests upper-deck evidence rather than the absence of
# a pallet-shaped object, so it belongs with the negatives and not with them.
NEGATIVE_CATEGORIES = (
    "negative_no_pallet",
    "negative_lookalike",
    "negative_block_row",
)
POSITION_TOLERANCE_M = 0.20
YAW_TOLERANCE_RAD = 0.35
TARGET_POSITION_P95_M = 0.020
TARGET_YAW_P95_RAD = 0.0349
TARGET_DETECTION_RATE = 0.95


class Outcome(str, Enum):
    TRUE_POSITIVE = "true_positive"
    WRONG_POSE = "wrong_pose"
    FALSE_NEGATIVE = "false_negative"
    FALSE_POSITIVE = "false_positive"
    TRUE_NEGATIVE = "true_negative"
    INVALID = "invalid"


@dataclass(frozen=True)
class SceneResult:
    """Euclidean pocket errors and absolute wrapped yaw, absent without geometry."""

    scene_id: str
    category: str
    split: str
    truth_status: str
    estimate_status: str
    outcome: Outcome
    left_error_m: float | None
    right_error_m: float | None
    position_error_m: float | None
    yaw_error_rad: float | None
    reason: str | None
    elapsed_s: float | None


def evaluate_scene(
    sample: SceneSample,
    observation: PocketObservation,
    *,
    elapsed_s: float | None = None,
    position_tolerance_m: float = POSITION_TOLERANCE_M,
    yaw_tolerance_rad: float = YAW_TOLERANCE_RAD,
) -> SceneResult:
    """Apply category/status judgment; tolerances are correspondence thresholds.

    Position error is max(left, right), each measured as Euclidean distance.
    Negative-scene false positives have no invented ground-truth pose error.
    Nonfinite or missing elapsed times are excluded later; negatives are errors.
    """
    for name, value in (
        ("position_tolerance_m", position_tolerance_m),
        ("yaw_tolerance_rad", yaw_tolerance_rad),
    ):
        if _finite_scalar(value, name) < 0:
            raise ValueError(f"{name} must be nonnegative")
    if elapsed_s is not None and elapsed_s < 0:
        raise ValueError("elapsed_s must not be negative")
    category = sample.scene["category"]
    if category not in (*POSITIVE_CATEGORIES, *NEGATIVE_CATEGORIES):
        raise ValueError(f"Unsupported scene category: {category!r}")
    truth = sample.ground_truth
    left = right = position = yaw = None
    position_roundoff = 0.0
    if (
        category in POSITIVE_CATEGORIES
        and truth.status == observation.status == "valid"
    ):
        left = math.dist(observation.left.center_m, truth.left.center_m)
        right = math.dist(observation.right.center_m, truth.right.center_m)
        position = max(left, right)
        yaw = abs(
            yaw_difference_rad(observation.insertion_yaw_rad, truth.insertion_yaw_rad)
        )
        # Subtraction/norm and modulo wrapping can move an exact boundary a
        # few representable floats above the threshold. Account only for
        # arithmetic roundoff at the input scale, without changing thresholds
        # or the reported errors (e.g. x=2.2 plus 0.20, yaw=+0.35).
        coordinate_scale = max(
            abs(value)
            for pocket in (truth.left, truth.right, observation.left, observation.right)
            for value in pocket.center_m
        )
        position_roundoff = 4 * math.ulp(coordinate_scale)
    if observation.status == "invalid":
        outcome = Outcome.INVALID
    elif category in NEGATIVE_CATEGORIES:
        outcome = (
            Outcome.FALSE_POSITIVE
            if observation.status == "valid"
            else Outcome.TRUE_NEGATIVE
        )
    elif observation.status == "no_pallet":
        outcome = Outcome.FALSE_NEGATIVE
    elif position is None or yaw is None:
        outcome = Outcome.INVALID
    elif (
        position <= position_tolerance_m + position_roundoff
        and yaw <= yaw_tolerance_rad + 4 * math.ulp(math.tau)
    ):
        outcome = Outcome.TRUE_POSITIVE
    else:
        outcome = Outcome.WRONG_POSE
    return SceneResult(
        sample.scene["scene_id"],
        category,
        sample.scene["split"],
        truth.status,
        observation.status,
        outcome,
        left,
        right,
        position,
        yaw,
        observation.reason,
        elapsed_s,
    )


def _stats(values):
    finite = [
        float(value) for value in values if value is not None and math.isfinite(value)
    ]
    if not finite:
        return {"p50": None, "p95": None, "max": None, "n": 0}
    p50, p95 = np.percentile(finite, (50, 95), method="linear")
    return {"p50": float(p50), "p95": float(p95), "max": max(finite), "n": len(finite)}


def summarize(results: Sequence[SceneResult]) -> dict:
    """Aggregate all observed splits and categories without excluding failures.

    Accuracy distributions include TP and wrong_pose; TP-only is secondary.
    Targets concern positive scenes only and missing samples never meet them.
    """
    if any(result.elapsed_s is not None and result.elapsed_s < 0 for result in results):
        raise ValueError("elapsed_s must not be negative")
    categories = sorted({result.category for result in results})
    counts = {
        category: {outcome.value: 0 for outcome in Outcome} for category in categories
    }
    for result in results:
        counts[result.category][result.outcome.value] += 1

    def rate(category, outcome):
        observed = counts.get(category, {})
        denominator = sum(observed.values())
        return observed[outcome.value] / denominator if denominator else None

    detection = {
        category: rate(category, Outcome.TRUE_POSITIVE)
        for category in POSITIVE_CATEGORIES
    }
    false_positive = {
        category: rate(category, Outcome.FALSE_POSITIVE)
        for category in NEGATIVE_CATEGORIES
    }
    # An invalid observation on a negative scene is not a false positive -- it
    # carries no geometry, so nothing downstream can act on it -- but it is not
    # a true negative either, and it is not in any budget. Reporting the rate
    # beside the false-positive one keeps it from disappearing into a bucket
    # label: the detector refusing a negative for the wrong reason still looks
    # like success on every number above.
    negative_invalid = {
        category: rate(category, Outcome.INVALID) for category in NEGATIVE_CATEGORIES
    }
    # One number over every negative, which is what a budget is actually set
    # against. Per-category rates cannot be compared with one: the single-sided
    # 95 % bound on 0 of 6 is 39 %, and on 0 of 18 it is 15.3 %.
    negative_results = [
        result for result in results if result.category in NEGATIVE_CATEGORIES
    ]
    combined_negative = {
        "scene_count": len(negative_results),
        "false_positive": sum(
            result.outcome is Outcome.FALSE_POSITIVE for result in negative_results
        ),
        "invalid": sum(
            result.outcome is Outcome.INVALID for result in negative_results
        ),
        "true_negative": sum(
            result.outcome is Outcome.TRUE_NEGATIVE for result in negative_results
        ),
    }
    combined_negative["false_positive_rate"] = (
        combined_negative["false_positive"] / len(negative_results)
        if negative_results
        else None
    )
    valid = [
        result
        for result in results
        if result.category in POSITIVE_CATEGORIES
        and result.estimate_status == "valid"
        and result.truth_status == "valid"
    ]
    groups = {
        "all_valid": valid,
        "true_positive": [
            result for result in valid if result.outcome is Outcome.TRUE_POSITIVE
        ],
        **{
            category: [result for result in valid if result.category == category]
            for category in POSITIVE_CATEGORIES
        },
    }
    errors = {
        name: {
            group: _stats(getattr(result, name) for result in members)
            for group, members in groups.items()
        }
        for name in ("position_error_m", "yaw_error_rad")
    }

    def at_most(value, limit):
        return value is not None and value <= limit

    return {
        "scene_count": len(results),
        "splits": sorted({result.split for result in results}),
        "counts": counts,
        "detection_rate": detection,
        "false_positive_rate": false_positive,
        "negative_invalid_rate": negative_invalid,
        "negatives_combined": combined_negative,
        **errors,
        "elapsed_s": _stats(result.elapsed_s for result in results),
        "targets": {
            "position_p95_m": TARGET_POSITION_P95_M,
            "yaw_p95_rad": TARGET_YAW_P95_RAD,
            "detection_rate": TARGET_DETECTION_RATE,
            "met": {
                "position_p95_positive": at_most(
                    errors["position_error_m"]["positive"]["p95"], TARGET_POSITION_P95_M
                ),
                "yaw_p95_positive": at_most(
                    errors["yaw_error_rad"]["positive"]["p95"], TARGET_YAW_P95_RAD
                ),
                "detection_rate_positive": detection["positive"] is not None
                and detection["positive"] >= TARGET_DETECTION_RATE,
            },
        },
    }
