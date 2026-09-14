"""The derivation that unties the detector from one pallet size.

Five parameters were absolute lengths and absolute point counts tuned on the
catalogue v1 pallet, which is why the detector only worked on that shape.
`DetectorParams.derived_for` scales them from the prior. These tests pin the
formula, the anchor, and what the anchor buys -- and pin just as firmly the
four parameters that are deliberately left alone, each for a measured reason.

The anchor is the point. s(v1) = 1, so the scaled terms return the frozen
values by identity: the v1 regression holds by definition rather than by
measurement. Anchoring on any other shape does not, and anchoring on EPAL 6
was measured to take v1 from 65/100 detections to 0/100.
"""

import dataclasses
from pathlib import Path

import pytest
import yaml

from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.scene_dataset import load_scene_sample

ROOT = Path(__file__).resolve().parents[3]
FROZEN = DetectorParams(
    **yaml.safe_load((ROOT / "config/detector_params_v1.yaml").read_text())
)
SCALED = ("plane_inlier_m", "band_margin_m", "min_band_points", "min_plane_points")
# Left alone on purpose. Each entry names why, because a later reader will
# otherwise assume the list is an oversight.
NOT_DERIVED = {
    # The width gate's lower bound is opening_width_min_m - 2 * cell_m, so
    # shrinking cells shrinks the acceptance window with them.
    "cell_m": "shrinking cells shrinks the width acceptance window",
    # Tying this to plane_inlier_m reopens the detection comb.
    "max_plane_residual_m": "tying it to plane_inlier_m reopens the comb",
    # Never measured against shape.
    "deck_evidence_tol_m": "unmeasured against shape",
    "front_margin_m": "unmeasured against shape",
}


def prior(name):
    return load_pallet_prior(ROOT / f"config/pallet_prior_{name}.yaml")


@pytest.mark.parametrize("name", ["v1", "epal6", "t11_06"])
def test_the_scale_factor_is_the_opening_height_against_v1(name):
    p = prior(name)
    derived = DetectorParams.derived_for(p)
    scale = p.opening_height_m / 0.200
    assert derived.plane_inlier_m == pytest.approx(FROZEN.plane_inlier_m * scale)
    assert derived.band_margin_m == pytest.approx(FROZEN.band_margin_m * scale)
    # Counts are a projected area, so they go as the square.
    assert derived.min_band_points == max(1, round(FROZEN.min_band_points * scale**2))
    assert derived.min_plane_points == max(3, round(FROZEN.min_plane_points * scale**2))


def test_v1_recovers_the_frozen_values_by_identity():
    """This is what choosing v1 as the anchor buys, and the whole reason for it."""
    derived = DetectorParams.derived_for(prior("v1"))
    for field in SCALED:
        assert getattr(derived, field) == getattr(FROZEN, field), field


def test_floor_z_is_not_one_of_the_scaled_terms():
    """It comes from the bottom deck, so it is not identity at v1 either.

    Recorded rather than hidden: the four scaled terms are identity at the
    anchor and this fifth one is not.
    """
    derived = DetectorParams.derived_for(prior("v1"))
    assert derived.floor_z_m != FROZEN.floor_z_m


@pytest.mark.parametrize("name", ["v1", "epal6", "t11_06"])
def test_floor_z_stays_under_the_bound_that_would_clip_the_lower_band(name):
    """Above deck_bottom_m - deck_evidence_tol_m the evidence band is cut off.

    Past that bound a pallet, a bottom-deckless structure and a floor-standing
    rack supply identical lower evidence -- the detector stops being able to
    tell them apart at all. The frozen 20 mm violates it on both real shapes.
    """
    p = prior(name)
    derived = DetectorParams.derived_for(p)
    bound = p.deck_bottom_m - derived.deck_evidence_tol_m
    assert derived.floor_z_m < bound, f"{name}: {derived.floor_z_m} >= {bound}"


@pytest.mark.parametrize("name", ["epal6", "t11_06"])
def test_the_frozen_floor_z_violates_that_bound_on_the_real_shapes(name):
    """The limitation the derivation exists to remove, pinned as a fact."""
    p = prior(name)
    bound = p.deck_bottom_m - FROZEN.deck_evidence_tol_m
    assert FROZEN.floor_z_m >= bound


@pytest.mark.parametrize("name,expected", [("epal6", 0.39), ("t11_06", 0.225)])
def test_the_real_shapes_scale_as_measured(name, expected):
    assert prior(name).opening_height_m / 0.200 == pytest.approx(expected, abs=1e-6)


@pytest.mark.parametrize("field,why", sorted(NOT_DERIVED.items()))
def test_the_underived_parameters_stay_at_their_frozen_values(field, why):
    """Leaving these alone is a decision, not an oversight. `why` records it."""
    for name in ("v1", "epal6", "t11_06"):
        derived = DetectorParams.derived_for(prior(name))
        assert getattr(derived, field) == getattr(DetectorParams(), field), why


def test_overrides_win_over_the_derivation():
    derived = DetectorParams.derived_for(prior("t11_06"), seed=7, cell_m=0.02)
    assert derived.seed == 7
    assert derived.cell_m == 0.02
    assert derived.min_band_points == 5


def test_a_nonpositive_opening_height_is_refused():
    with pytest.raises((ValueError, TypeError)):
        DetectorParams.derived_for(
            dataclasses.replace(prior("v1"), opening_height_m=0.0)
        )


# --------------------------------------------------------------------------
# The regression guard the frozen replay cannot provide
# --------------------------------------------------------------------------


CATALOGUE = ROOT / "data/synthetic_scenes/catalogue_v1/scenes"


@pytest.mark.skipif(not CATALOGUE.is_dir(), reason="catalogue v1 is untracked")
def test_deriving_parameters_for_v1_preserves_its_detections():
    """The replay test replays the frozen YAML and cannot see this change.

    It asserts the config file still matches the historical run and then uses
    that file, so deriving parameters at runtime never touches the derivation
    path at all. This is the guard that does, per C-17 (8) of the restructure
    plan: same scenes, derived parameters, compared against the frozen run.
    """
    p = prior("v1")
    derived = dataclasses.replace(
        DetectorParams.derived_for(p), seed=FROZEN.seed
    )
    frozen_valid = derived_valid = 0
    changes = []
    for scene_dir in sorted(CATALOGUE.iterdir()):
        sample = load_scene_sample(scene_dir)
        before = detect_pockets(sample.input, p, FROZEN).observation
        after = detect_pockets(sample.input, p, derived).observation
        frozen_valid += before.status == "valid"
        derived_valid += after.status == "valid"
        if (before.status, before.reason) != (after.status, after.reason):
            changes.append((scene_dir.name, before.status, before.reason, after.status, after.reason))
    assert frozen_valid == 65, "the frozen baseline moved; re-establish it first"
    assert derived_valid == frozen_valid, f"derivation changed the detection count: {changes}"
    # One scene swaps which of two invalid reasons fires; both refuse the same
    # pocket on the same side, so no caller is told anything different.
    assert len(changes) <= 1, changes
    for _, before_status, _, after_status, _ in changes:
        assert before_status == after_status == "invalid"
