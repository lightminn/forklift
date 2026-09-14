"""What the opening-evidence rules accept and refuse, on named structures.

Each case names a shape and pins a verdict. Several pin a *limitation*: the
detector refuses something it should accept, or cannot tell two shapes apart.
Those are labelled, because a test that quietly encodes a defect as expected
behaviour is how the defect stops being visible.

The structures come from tools.measure_pocket_evidence rather than being
defined here. Three review rounds reported different false-positive rates
because each invented its own lookalikes.

Read with the gate in mind: the pattern gate is
``min(supports, lower, upper) >= min_band_points``. Real EPAL and T11 pallets
are open to the floor, so their ``lower`` is structurally zero and no pattern
survives -- which is why the shapes below that carry a continuous bottom slab
are the ones that exercise the upper-deck rules at all.
"""

import dataclasses
from pathlib import Path

import numpy as np
import pytest
import yaml

from forklift_core.perception import pocket_detector as detector
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from tools import scene_rig
from tools.measure_pocket_evidence import structure

ROOT = Path(__file__).resolve().parents[3]
EPAL6 = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
EPAL6_PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
T11 = load_pallet_geometry(ROOT / "config/pallet_geometry_t11_06.yaml")
T11_PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_t11_06.yaml")
FROZEN = DetectorParams(
    **yaml.safe_load((ROOT / "config/detector_params_v1.yaml").read_text())
)
FLOOR_THROUGH = ("pallet", "deckless", "grounded", "shelf", "blocks")


def observe(boxes, prior, *, seed=0, **overrides):
    scene = scene_rig.render(boxes)
    return detect_pockets(
        scene, prior, dataclasses.replace(FROZEN, seed=seed, **overrides)
    )


def at(geometry, kind, x, *, y=0.0, yaw=0.0):
    return scene_rig.place(structure(geometry, kind), x_m=x, y_m=y, yaw_rad=yaw)


def _is_upper_deck(box, geometry):
    """Stringers and top boards: everything above the fork opening."""
    return box.centre_m[2] > geometry.deck_bottom_m + geometry.block_height_m


def without_upper_deck(boxes, geometry):
    return [b for b in boxes if not _is_upper_deck(b, geometry)]


def upper_deck_lifted_clear(boxes, geometry, *, rise_m):
    """Raise the whole upper deck above the band the per-opening count reads.

    This is the overhead case: there is deck-like material above the openings,
    high enough that it cannot be this pallet's deck. The combined count still
    sees it, because that count has no upper bound; the per-opening count does
    not, because its band is no thicker than the deck itself.
    """
    lifted = []
    for box in boxes:
        if _is_upper_deck(box, geometry):
            x, y, z = box.centre_m
            lifted.append(scene_rig.Box((x, y, z + rise_m), box.size_m, box.yaw_rad))
        else:
            lifted.append(box)
    return lifted


def upper_deck_over_one_opening(boxes, geometry, *, keep_sign):
    """Clip the upper deck to one side, so one opening has deck and one has sky.

    Removing whole boxes does not work: the stringers span the full width, so
    dropping them takes the deck off both openings at once and the pattern
    stops existing rather than becoming one-sided. Each upper-deck box is
    clipped in y instead, keeping the half on ``keep_sign``.
    """
    kept = []
    for box in boxes:
        if not _is_upper_deck(box, geometry):
            kept.append(box)
            continue
        y, half = box.centre_m[1], box.size_m[1] / 2
        low, high = y - half, y + half
        if keep_sign > 0:
            low = max(low, 0.0)
        else:
            high = min(high, 0.0)
        if high - low <= 1e-9:
            continue
        kept.append(
            scene_rig.Box(
                (box.centre_m[0], (low + high) / 2, box.centre_m[2]),
                (box.size_m[0], high - low, box.size_m[2]),
                box.yaw_rad,
            )
        )
    return kept


# --------------------------------------------------------------------------
# A shape with a closed underside is accepted, and reported where it is
# --------------------------------------------------------------------------


@pytest.mark.parametrize("x", [2.0, 3.0, 4.0])
def test_a_closed_underside_pallet_is_valid_with_pockets_on_the_approach_face(x):
    result = observe(at(EPAL6, "slab", x), EPAL6_PRIOR)
    assert result.observation.status == "valid", result.observation.reason
    pockets = (result.observation.left.center_m, result.observation.right.center_m)
    truth = scene_rig.true_pockets(EPAL6, x_m=x)
    assert scene_rig.position_error_m(pockets, truth) < 0.02
    lateral = sorted(p[1] for p in pockets)
    assert lateral[0] == pytest.approx(-0.186, abs=0.01)
    assert lateral[1] == pytest.approx(0.186, abs=0.01)


@pytest.mark.parametrize("x", [2.0, 3.0, 4.0])
def test_both_openings_supply_upper_deck_evidence_and_the_counts_match(x):
    """A symmetric pallet seen head-on must give the two openings equal deck."""
    diagnostics = observe(at(EPAL6, "slab", x), EPAL6_PRIOR).diagnostics
    assert diagnostics.selected_upper_left >= FROZEN.min_band_points
    assert diagnostics.selected_upper_right >= FROZEN.min_band_points
    # Symmetric, but not to the point: which plane RANSAC settles on shifts the
    # inlier set slightly, so pin the symmetry rather than an equality.
    left, right = diagnostics.selected_upper_left, diagnostics.selected_upper_right
    assert abs(left - right) / max(left, right) < 0.1


# --------------------------------------------------------------------------
# Missing upper deck: absence, and obstruction on one side
# --------------------------------------------------------------------------


def test_deck_material_too_high_to_be_this_pallet_is_invalid_not_absent():
    """Something is there; it is the wrong shape.

    Reporting no_pallet would tell a caller the space is clear, so the contract
    reserves that for nothing being there. Here the overhead material is what
    the combined upper count sees, and the per-opening count correctly refuses
    to accept it as this pallet's deck.
    """
    boxes = upper_deck_lifted_clear(at(EPAL6, "slab", 3.0), EPAL6, rise_m=0.10)
    result = observe(boxes, EPAL6_PRIOR)
    assert result.observation.status == "invalid"
    assert result.observation.reason == "no_upper_deck"
    assert result.diagnostics.selected_upper_left == 0
    assert result.diagnostics.selected_upper_right == 0


def test_removing_the_upper_deck_entirely_never_reaches_the_per_opening_rule():
    """With `lower` still in the gate, the combined count refuses this first.

    Worth pinning: it means no_upper_deck cannot be read as "the deck was
    absent". The per-opening rule only ever speaks about shapes that already
    cleared the combined count.
    """
    boxes = without_upper_deck(at(EPAL6, "slab", 3.0), EPAL6)
    observation = observe(boxes, EPAL6_PRIOR).observation
    assert observation.status == "no_pallet"
    assert observation.reason == "no_opening_pattern"


@pytest.mark.parametrize("keep,missing", [(1.0, "right"), (-1.0, "left")])
def test_upper_deck_over_one_opening_names_the_side_that_is_missing(keep, missing):
    """Deck over one gap and not the other is this pallet with something in the way.

    The side matters: it is the pocket the caller cannot trust.
    """
    boxes = upper_deck_over_one_opening(at(EPAL6, "slab", 3.0), EPAL6, keep_sign=keep)
    # Clipping the deck exposes its cut face, which the plane fit sees and which
    # pushes the residual past the frozen 15 mm. That gate is not what this case
    # is about, so give it room; every other parameter stays frozen.
    result = observe(boxes, EPAL6_PRIOR, max_plane_residual_m=0.030)
    assert result.observation.status == "invalid"
    assert result.observation.reason == f"upper_deck_occluded:{missing}"
    counts = {
        "left": result.diagnostics.selected_upper_left,
        "right": result.diagnostics.selected_upper_right,
    }
    assert counts[missing] < FROZEN.min_band_points
    assert counts["left" if missing == "right" else "right"] >= FROZEN.min_band_points


# --------------------------------------------------------------------------
# Floor-through shapes: refused, and the term that refuses them
# --------------------------------------------------------------------------


@pytest.mark.parametrize("kind", FLOOR_THROUGH)
@pytest.mark.parametrize(
    "geometry,prior", [(EPAL6, EPAL6_PRIOR), (T11, T11_PRIOR)], ids=["epal6", "t11"]
)
def test_every_floor_through_shape_is_refused_for_want_of_lower_evidence(
    kind, geometry, prior
):
    """Including the real pallets. This is the limitation, stated as a test.

    Both adopted articles have openings open to the floor, so nothing spans
    beneath them and `lower` cannot be anything but zero. No pattern survives
    the gate, so there is no selected evidence to read.
    """
    result = observe(at(geometry, kind, 3.0), prior)
    assert result.observation.status == "no_pallet"
    assert result.observation.reason == "no_opening_pattern"
    assert result.diagnostics.selected_lower is None


@pytest.mark.parametrize(
    "geometry,prior", [(EPAL6, EPAL6_PRIOR), (T11, T11_PRIOR)], ids=["epal6", "t11"]
)
def test_a_real_pallet_clears_every_gate_term_except_lower(geometry, prior):
    """Decompose the gate by hand: supports and upper pass, lower is what fails.

    This is the measurement the decision to keep or drop `lower` rests on, so
    it is pinned rather than described in prose.
    """
    scene = scene_rig.render(at(geometry, "pallet", 3.0))
    points, _ = detector._base_points(scene)
    camera = scene.base_from_optical.translation_m
    workspace = detector._filter_workspace(points, camera, prior, FROZEN)
    planes = detector._vertical_plane_candidates(workspace, camera, FROZEN)
    assert planes, "the front face must be found"
    plane = planes[0]
    lateral = plane.points @ plane.left_axis
    local = np.column_stack((np.zeros(len(lateral)), lateral, plane.points[:, 2]))
    counts, origin = detector._column_grid(local, prior, FROZEN)
    gaps = detector._gap_runs(counts > 0)
    assert len(gaps) == 2, "two openings must be visible"
    first, second = gaps
    supports = (
        counts[: first[0]].sum(),
        counts[first[1] : second[0]].sum(),
        counts[second[1] :].sum(),
    )
    assert min(supports) >= FROZEN.min_band_points
    left_edge = (origin + first[0]) * FROZEN.cell_m
    right_edge = (origin + second[1]) * FROZEN.cell_m
    over = (lateral >= left_edge) & (lateral <= right_edge)
    upper = np.count_nonzero(over & (local[:, 2] >= prior.height_m - prior.deck_top_m))
    assert upper >= FROZEN.min_band_points
    depth = -(workspace - plane.point) @ plane.normal
    workspace_lateral = workspace @ plane.left_axis
    lower = np.count_nonzero(
        (np.abs(workspace[:, 2] - prior.deck_bottom_m) <= FROZEN.deck_evidence_tol_m)
        & (depth >= 0)
        & (depth <= prior.overall_depth_m + FROZEN.plane_inlier_m)
        & (workspace_lateral >= left_edge)
        & (workspace_lateral <= right_edge)
    )
    assert lower < FROZEN.min_band_points, "lower is the term that refuses the pallet"


# --------------------------------------------------------------------------
# The lookalike the detector provably cannot separate
# --------------------------------------------------------------------------


def test_known_limitation_floor_standing_columns_match_a_pallet_term_by_term():
    """Columns that reach the floor look exactly like the article to this gate.

    They differ only in material deep behind the front plane, and no lower-band
    point is ever observed that deep from this viewpoint. Pinned as an
    equality, because the equality is the finding: see C-18 and C-19 of
    docs/plans/2026-09-13-pocket-evidence-restructure.md.
    """
    scenes = {}
    for kind in ("pallet", "grounded"):
        scene = scene_rig.render(at(T11, kind, 3.0))
        points, _ = detector._base_points(scene)
        camera = scene.base_from_optical.translation_m
        workspace = detector._filter_workspace(points, camera, T11_PRIOR, FROZEN)
        plane = detector._vertical_plane_candidates(workspace, camera, FROZEN)[0]
        lateral = plane.points @ plane.left_axis
        local = np.column_stack((np.zeros(len(lateral)), lateral, plane.points[:, 2]))
        counts, _ = detector._column_grid(local, T11_PRIOR, FROZEN)
        scenes[kind] = (
            int((counts > 0).sum()),
            int(counts.sum()),
            len(detector._gap_runs(counts > 0)),
        )
    assert scenes["pallet"] == scenes["grounded"]
