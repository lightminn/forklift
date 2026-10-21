"""Plan 2026-10-21 task C: the near capture's lower-deck band can be waived by the caller (the bottom-deck check
proved the deck from the side faces). The waiver removes that one item from the evidence gate and nothing else."""

from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception import pocket_detector as P
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from tools import scene_rig
from tools.measure_pocket_evidence import structure

ROOT = Path(__file__).resolve().parents[3]
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
PARAMS = P.DetectorParams.derived_for(PRIOR, range_min_m=0.03, range_max_m=4.0, max_plane_candidates=12)


def scene(kind, x_m=2.0):
    return scene_rig.render(scene_rig.place(structure(GEOMETRY, kind), x_m=x_m),
                            camera=scene_rig.Camera((0.619, 0.0, 0.27)), quantize=True)


@pytest.mark.parametrize("invalid", [0, 1, "false", None, np.bool_(False)])
def test_the_waiver_takes_an_explicit_boolean_only(invalid):
    with pytest.raises(ValueError, match="lower_evidence"):
        P.detect_pockets(scene("pallet"), PRIOR, PARAMS, lower_evidence=invalid)


def test_a_real_pallet_short_of_lower_evidence_is_found_with_the_waiver():
    # The 2026-10-21 spike: x 2.0 with millimetre depth leaves the lower band at 2 points (threshold 15).
    sc = scene("pallet")
    default = P.detect_pockets(sc, PRIOR, PARAMS)
    assert default.observation.status == "no_pallet"
    assert any("lower" in p.failed_items for p in default.diagnostics.rejected_patterns)
    waived = P.detect_pockets(sc, PRIOR, PARAMS, lower_evidence=False)
    assert waived.observation.status == "valid"
    # The count is still recorded for the selected pattern.
    assert waived.diagnostics.selected_lower is not None and waived.diagnostics.selected_lower < PARAMS.min_band_points


def test_the_waiver_keeps_every_other_evidence_item():
    # Free-standing blocks: no decks, so the upper evidence still fails and nothing is found.
    for x in (2.0, 2.5, 2.9):
        assert P.detect_pockets(scene("blocks", x), PRIOR, PARAMS, lower_evidence=False).observation.status != "valid"


def test_the_candidate_builder_takes_the_same_waiver():
    sc = scene("pallet")
    points, _ = P._base_points(sc)
    camera = sc.base_from_optical.translation_m
    workspace = P._filter_workspace(points, camera, PRIOR, PARAMS)
    planes = P._vertical_plane_candidates(workspace, camera, PARAMS)
    found = [len(P._opening_candidates(pl, PRIOR, PARAMS, workspace, plane_index=i, lower_evidence=False)[0])
             for i, pl in enumerate(planes)]
    default = [len(P._opening_candidates(pl, PRIOR, PARAMS, workspace, plane_index=i)[0]) for i, pl in enumerate(planes)]
    assert sum(found) > sum(default) == 0
