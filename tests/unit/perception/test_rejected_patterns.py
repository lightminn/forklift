"""Rejected evidence stays measurable without becoming a selected pattern."""

import json
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception import pocket_detector as detector
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from tools import scene_rig
from tools.measure_pocket_evidence import structure

ROOT = Path(__file__).resolve().parents[3]
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")


def test_lower_only_rejection_is_serializable_and_not_selected():
    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    scene = scene_rig.render(scene_rig.place(structure(geometry, "deckless"), x_m=3.0))
    params = detector.DetectorParams.derived_for(PRIOR, seed=0, min_band_points=100)
    result = detector.detect_pockets(scene, PRIOR, params)
    assert result.observation.reason == "no_opening_pattern"
    diag = result.diagnostics
    assert diag.selected_lower is None
    assert diag.selected_support_min is None
    assert diag.selected_upper_left is None
    assert diag.selected_upper_right is None
    records = json.loads(json.dumps(asdict(diag)))["rejected_patterns"]
    lower_only = [r for r in records if r["failed_items"] == ["lower"]]
    assert lower_only
    record = lower_only[0]
    assert record["stage"] == "insufficient_evidence"
    assert record["lower"] < 100
    assert min(record["supports"]) >= 100
    assert record["upper"] >= 100
    assert record["upper_left"] is not None
    assert record["upper_right"] is not None
    assert dict(record["thresholds"])["lower"] == 100
    assert len(record["gaps"]) == 2


def scaffold():
    # Three columns with distinct counts, ordered right, centre, left.
    points = [
        (2.0, y, 0.06) for y, n in [(-0.30, 3), (0.0, 4), (0.30, 5)] for _ in range(n)
    ]
    points += [(2.0, y, PRIOR.height_m) for y in (-0.15, 0.15)]
    points = np.array(points)
    plane = detector._Plane(
        np.array((2.0, 0.0, 0.0)), np.array((-1.0, 0.0, 0.0)), points, 0.0
    )
    prior = replace(PRIOR, centre_spacer_min_m=0.0, centre_spacer_max_m=0.1)
    return plane, prior


@pytest.mark.parametrize(
    "stage",
    [
        "no_gap_pair",
        "spacer_out_of_range",
        "insufficient_evidence",
        "insufficient_upper",
    ],
)
def test_rejection_stages_preserve_counts_and_candidate_eligibility(stage):
    plane, prior = scaffold()
    params = detector.DetectorParams.derived_for(prior, min_band_points=10)
    workspace = plane.points
    if stage == "no_gap_pair":
        plane = replace(plane, points=plane.points[:3])
    elif stage == "spacer_out_of_range":
        prior = replace(prior, centre_spacer_min_m=0.2, centre_spacer_max_m=0.3)
    elif stage == "insufficient_upper":
        params = replace(params, min_band_points=1, upper_band_points=3)
        workspace = np.vstack((workspace, (2.1, 0.0, prior.deck_bottom_m)))
    patterns, rejected = detector._opening_candidates(
        plane, prior, params, workspace, plane_index=7
    )
    assert len(rejected) == 1
    record = rejected[0]
    assert record.plane_index == 7
    assert record.stage == stage
    if stage in {"no_gap_pair", "spacer_out_of_range"}:
        assert record.lower is None
        assert record.upper_left is None
        assert record.supports is None
    else:
        assert record.supports == (5, 4, 3)
        assert record.upper == 2
        assert record.upper_left == record.upper_right == 1
        if stage == "insufficient_evidence":
            assert set(record.failed_items) == {
                "support_left",
                "support_middle",
                "support_right",
                "lower",
                "upper",
                "upper_left",
                "upper_right",
            }
            assert all(value == 10 for _, value in record.thresholds)
        else:
            assert record.failed_items == ("upper_left", "upper_right")
            assert dict(record.thresholds)["upper_left"] == 3
    # Failed per-opening verification still participates in the legacy ranking.
    assert len(patterns) == (1 if stage == "insufficient_upper" else 0)
    if patterns:
        assert not patterns[0].upper_ok
