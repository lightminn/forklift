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


def test_default_budget_reaches_a_pallet_behind_five_larger_planes():
    """Budget 6 (G4, 2026-10-02): the pallet front is the sixth plane here.

    CPU mechanism regression for Isaac seed 1 of the G2 rerun, whose front
    (941-984 px) lost all five candidates to background planes
    (docs/validation/2026-10-02-g3-detection-diagnosis.md); not a
    reproduction of that image.
    """
    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = detector.DetectorParams.derived_for(prior)
    pallet_x_m = 3.3
    scene = scene_rig.render(
        scene_rig.place(scene_rig.pallet(geometry), x_m=pallet_x_m)
        + [
            scene_rig.Box((2.9, -1.0, 0.5), (1.2, 1.0, 1.0)),
            scene_rig.Box((3.2, 1.0, 0.5), (1.2, 1.0, 1.0)),
            # A wall behind the pallet and a low box in front of it.
            scene_rig.Box((4.3, 0.0, 0.5), (0.3, 1.6, 1.0)),
            scene_rig.Box((2.3, 0.0, 0.05), (0.2, 0.3, 0.1)),
        ]
    )
    truth = scene_rig.true_pockets(geometry, x_m=pallet_x_m)
    points, _ = detector._base_points(scene)
    camera = scene.base_from_optical.translation_m
    workspace = detector._filter_workspace(points, camera, prior, params)
    planes = detector._vertical_plane_candidates(
        workspace, camera, replace(params, max_plane_candidates=12)
    )
    front = [
        i
        for i, plane in enumerate(planes)
        if np.all(np.abs((truth - plane.point) @ plane.normal) < 0.002)
    ]
    assert front == [5]
    five = detector.detect_pockets(
        scene, prior, replace(params, max_plane_candidates=5)
    )
    assert five.observation.status == "no_pallet"
    # The shipped default reaches it.
    observation = detector.detect_pockets(scene, prior, params).observation
    assert observation.status == "valid"
    reported = np.array([observation.left.center_m, observation.right.center_m])
    np.testing.assert_allclose(reported, truth, atol=0.005)


def test_known_limit_budget_six_also_reaches_a_grounded_lookalike():
    """Known limit, kept on purpose (Codex counterexample, 2026-10-02).

    The same clutter with the pallet replaced by the `grounded` negative (no
    bottom boards, columns to the floor): budget 5 never examines it, budget 6
    does, and the detector's existing weakness on `grounded` (29/36 valid in
    the CPU pose sweep at either budget) turns it into a false positive. The
    budget makes that weakness reachable in clutter; it does not create it.
    Render-population impact is unmeasured.
    """
    from tools.measure_pocket_evidence import structure

    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = detector.DetectorParams.derived_for(prior)
    scene = scene_rig.render(
        scene_rig.place(structure(geometry, "grounded"), x_m=3.3)
        + [
            scene_rig.Box((2.9, -1.0, 0.5), (1.2, 1.0, 1.0)),
            scene_rig.Box((3.2, 1.0, 0.5), (1.2, 1.0, 1.0)),
            scene_rig.Box((4.3, 0.0, 0.5), (0.3, 1.6, 1.0)),
            scene_rig.Box((2.3, 0.0, 0.05), (0.2, 0.3, 0.1)),
        ]
    )
    five = detector.detect_pockets(
        scene, prior, replace(params, max_plane_candidates=5)
    )
    assert five.observation.status == "no_pallet"
    assert detector.detect_pockets(scene, prior, params).observation.status == "valid"


SEVEN_PLANE_CLUTTER = (
    scene_rig.Box((2.9, -1.0, 0.5), (1.2, 1.0, 1.0)),
    scene_rig.Box((3.2, 1.0, 0.5), (1.2, 1.0, 1.0)),
    scene_rig.Box((4.3, 0.0, 0.5), (0.3, 1.6, 1.0)),
    scene_rig.Box((2.3, 0.0, 0.05), (0.2, 0.3, 0.1)),
    # One more post ahead of the pallet (Codex counterexample, 2026-10-02).
    scene_rig.Box((2.0, -0.75, 0.5), (0.2, 0.4, 1.0)),
)


def _clutter_scene(boxes):
    return scene_rig.render(scene_rig.place(boxes, x_m=3.3) + list(SEVEN_PLANE_CLUTTER))


def test_default_budget_reaches_a_pallet_behind_six_larger_planes():
    """Budget 8 (2026-10-02, docs/plans/2026-10-02-detection-and-handoff-fixes.md, D1).

    CPU mechanism regression for second-evaluation seed 2023 (candidate 4),
    whose front lost all six candidates to background planes (G3 B); not a
    reproduction of that image.
    """
    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = detector.DetectorParams.derived_for(prior)
    assert params.max_plane_candidates == 8
    scene = _clutter_scene(scene_rig.pallet(geometry))
    six = detector.detect_pockets(scene, prior, replace(params, max_plane_candidates=6))
    assert six.observation.status == "no_pallet"
    observation = detector.detect_pockets(scene, prior, params).observation
    assert observation.status == "valid"
    truth = scene_rig.true_pockets(geometry, x_m=3.3)
    reported = np.array([observation.left.center_m, observation.right.center_m])
    np.testing.assert_allclose(reported, truth, atol=0.005)


def test_known_limit_budget_eight_also_reaches_a_grounded_lookalike():
    """Known limit (Codex counterexample, 2026-10-02): the same clutter with the
    `grounded` negative is no_pallet at 6 and a false positive at 8, as it was
    for 5 -> 6. The budget makes the detector's `grounded` weakness reachable
    in more clutter; render-population impact is unmeasured."""
    from tools.measure_pocket_evidence import structure

    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = detector.DetectorParams.derived_for(prior)
    scene = _clutter_scene(structure(geometry, "grounded"))
    six = detector.detect_pockets(scene, prior, replace(params, max_plane_candidates=6))
    assert six.observation.status == "no_pallet"
    assert detector.detect_pockets(scene, prior, params).observation.status == "valid"
