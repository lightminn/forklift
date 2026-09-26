"""Optional near-range fitting does not certify a physical bottom deck."""

from pathlib import Path

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from tools import scene_rig
from tools.measure_pocket_evidence import structure

ROOT = Path(__file__).resolve().parents[3]


def test_opt_in_median_can_accept_a_shelf_without_bottom_boards():
    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    scene = scene_rig.render(
        scene_rig.place(structure(geometry, "shelf"), x_m=2.0),
        camera=scene_rig.Camera((0.75, 0.0, 0.27)),
        quantize=False,
    )
    settings = dict(range_min_m=0.03, range_max_m=2.0, max_plane_candidates=12)
    original = detect_pockets(
        scene, prior, DetectorParams.derived_for(prior, **settings)
    )
    assert original.observation.status == "no_pallet"

    opted_in = detect_pockets(
        scene,
        prior,
        DetectorParams.derived_for(prior, median_plane_offset=True, **settings),
    )
    # Known false positive: front columns supply lower-band points. Enabling
    # robust fitting alone neither identifies a pallet nor proves its bottom.
    assert opted_in.observation.status == "valid"
