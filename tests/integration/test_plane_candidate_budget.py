"""Large foreground boxes must not exhaust the pallet plane search budget."""

from dataclasses import replace
from pathlib import Path

import numpy as np

from forklift_core.perception import pocket_detector as detector
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[2]


def test_default_budget_reaches_pallet_behind_larger_planes():
    """CPU mechanism regression, not a reproduction of the Isaac seed 4 image."""
    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = detector.DetectorParams.derived_for(prior)
    pallet_x_m = 3.3
    scene = scene_rig.render(
        scene_rig.place(scene_rig.pallet(geometry), x_m=pallet_x_m)
        + [
            # Front faces at x=2.3 and 2.6 m, both ahead of the pallet.
            # Their inner sides leave a 1 m corridor around the 0.8 m pallet.
            # Two box fronts and the left inner side outrank its front plane.
            scene_rig.Box((2.9, -1.0, 0.5), (1.2, 1.0, 1.0)),
            scene_rig.Box((3.2, 1.0, 0.5), (1.2, 1.0, 1.0)),
        ]
    )
    truth = scene_rig.true_pockets(geometry, x_m=pallet_x_m)
    points, _ = detector._base_points(scene)
    camera = scene.base_from_optical.translation_m
    workspace = detector._filter_workspace(points, camera, prior, params)

    for budget in (3, 5):
        explicit = replace(params, max_plane_candidates=budget)
        planes = detector._vertical_plane_candidates(workspace, camera, explicit)
        # Check the actual geometric cause, not only the final verdict.
        front_indices = [
            i
            for i, plane in enumerate(planes)
            if np.all(np.abs((truth - plane.point) @ plane.normal) < 0.002)
        ]
        assert front_indices == ([] if budget == 3 else [3])
        observation = detector.detect_pockets(scene, prior, explicit).observation
        assert observation.status == ("no_pallet" if budget == 3 else "valid")

    # No budget override: lowering the shipped default to 3 breaks this result.
    observation = detector.detect_pockets(scene, prior, params).observation
    assert observation.status == "valid"
    reported = np.array([observation.left.center_m, observation.right.center_m])
    # The lateral occupancy grid resolves centres to half a 10 mm cell.
    np.testing.assert_allclose(reported, truth, atol=0.005)
    assert abs(observation.insertion_yaw_rad) < 0.001
