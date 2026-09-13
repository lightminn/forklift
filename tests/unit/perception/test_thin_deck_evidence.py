"""v4 evidence-location contract, using independent first-hit box rays."""

import dataclasses
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.perception import pocket_detector as detector
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior

ROOT = Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "thin_deck_scene", ROOT / "tests/fixtures/synthetic_scene.py"
)
_SCENE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_SCENE)


def box_scene(boxes, *, quantize=True):
    scene, _ = _SCENE.make_pallet_scene(pallet=False)
    camera = scene.base_from_optical.translation_m
    rays = _SCENE._rays(scene.intrinsics, scene.base_from_optical.rotation)
    depth = scene.depth_m.copy()
    for centre, size in boxes:
        hit = _SCENE._box_depth(camera, rays, np.asarray(centre), size, np.eye(3))
        depth = np.fmin(depth, hit)
    depth[~np.isfinite(depth)] = np.nan
    if quantize:
        depth = np.round(depth / 0.001) * 0.001
    return dataclasses.replace(scene, depth_m=depth)


def epal6_boxes(x):
    geometry = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
    boxes = [
        (
            (x, 0.0, geometry.deck_bottom_m / 2),
            (
                geometry.overall_depth_m,
                geometry.overall_width_m,
                geometry.deck_bottom_m,
            ),
        ),
        (
            (x, 0.0, geometry.overall_height_m - geometry.deck_top_m / 2),
            (geometry.overall_depth_m, geometry.overall_width_m, geometry.deck_top_m),
        ),
    ]
    for dx in geometry.block_centres_x_m():
        for y in geometry.block_centres_y_m():
            boxes.append(
                (
                    (x + dx, y, geometry.opening_centre_height_m),
                    (
                        geometry.block_depth_m,
                        geometry.block_width_m,
                        geometry.block_height_m,
                    ),
                )
            )
    return boxes


def columns_and_top():
    # The two 280 mm channels have the EPAL 6 approach-face dimensions.
    return [((3.0, 0.0, 0.122), (0.6, 0.8, 0.044))] + [
        ((3.0, y, 0.050), (0.6, 0.08, 0.100)) for y in (-0.36, 0.0, 0.36)
    ]


def evidence_counts(scene, front_x, *, params=None, extra_points=()):
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = params or detector.DetectorParams()
    points, _ = detector._base_points(scene)
    workspace = detector._filter_workspace(
        points, scene.base_from_optical.translation_m, prior, params
    )
    # Use the actual fitted front, as in the review's counterexamples. An exact
    # plane would also admit front-column points into the lower evidence band.
    plane = detector._vertical_plane_candidates(
        workspace, scene.base_from_optical.translation_m, params
    )[0]
    assert abs(plane.point[0] - front_x) < params.plane_inlier_m
    assert plane.normal[0] < -0.99
    if len(extra_points):
        workspace = np.vstack(
            (
                workspace,
                detector._filter_workspace(
                    extra_points, scene.base_from_optical.translation_m, prior, params
                ),
            )
        )
    return opening_evidence_counts(plane, workspace, prior, params)


def opening_evidence_counts(plane, workspace, prior, params):
    # Disable only the private candidate's count gate to observe a zero lower
    # count. Public DetectorParams still rejects min_band_points=0.
    measuring = SimpleNamespace(**(dataclasses.asdict(params) | {"min_band_points": 0}))
    patterns = detector._opening_candidates(plane, prior, measuring, workspace)
    assert len(patterns) == 1, "fixture must expose exactly two supported openings"
    pattern = patterns[0]
    inliers = plane.points
    lateral = inliers @ plane.left_axis
    across = (lateral >= pattern.gaps[0][0]) & (lateral <= pattern.gaps[1][1])
    upper = np.count_nonzero(
        across & (inliers[:, 2] >= prior.height_m - prior.deck_top_m)
    )
    legacy_lower = np.count_nonzero(across & (inliers[:, 2] <= prior.deck_bottom_m))
    assert upper >= params.min_band_points
    # deck_count is the production aggregate; subtract the unchanged upper count.
    return int(legacy_lower), pattern.deck_count - int(upper)


@pytest.mark.parametrize("x", [2.0, 3.7, 4.0])
def test_thin_deck_evidence_exceeds_the_gate_at_each_distance(x):
    before, after = evidence_counts(box_scene(epal6_boxes(x)), x - 0.3)
    print(f"x={x:.1f} m: lower before={before}, after={after}")
    threshold = detector.DetectorParams().min_band_points
    if x == 2.0:
        assert before >= threshold
    else:
        assert before < threshold
    assert after > threshold


def test_a_quantized_epal6_at_three_metres_reaches_the_final_observation():
    result = detector.detect_pockets(
        box_scene(epal6_boxes(3.0)),
        load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml"),
    )
    assert result.observation.status == "valid", result.observation.reason


def test_a_24mm_strip_ahead_of_the_front_supplies_zero_lower_evidence():
    # Review 2 section 4: x=2.6805..2.6995, i.e. 0.5..19.5 mm ahead.
    strips = [((2.690, y, 0.012), (0.019, 0.260, 0.024)) for y in (-0.18, 0.18)]
    # Keep exact depths so the strip's physical x bounds identify its returns.
    scene = box_scene(columns_and_top() + strips, quantize=False)
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = detector.DetectorParams()
    camera = scene.base_from_optical.translation_m
    points, _ = detector._base_points(scene)
    workspace = detector._filter_workspace(points, camera, prior, params)
    plane = detector._vertical_plane_candidates(workspace, camera, params)[0]
    assert abs(plane.point[0] - 2.7) < params.plane_inlier_m
    strip_points = workspace[
        (workspace[:, 0] >= 2.6805 - 1e-9) & (workspace[:, 0] <= 2.6995 + 1e-9)
    ]
    assert len(strip_points) > params.min_band_points
    # Count the strip's contribution with the SAME fitted plane and lateral
    # aggregate. The central column also lies inside that aggregate, so total
    # scene evidence need not be zero; excluding it in production is out of scope.
    _, lower = opening_evidence_counts(plane, strip_points, prior, params)
    assert lower == 0
    # Positive control: the observed strip supplies evidence if moved behind
    # the face. This prevents an empty or otherwise ineligible strip from passing.
    behind = strip_points.copy()
    behind[:, 0] = plane.point[0] + 0.01
    _, behind_lower = opening_evidence_counts(plane, behind, prior, params)
    assert behind_lower > params.min_band_points


def test_a_box_800mm_beyond_the_rear_supplies_zero_lower_evidence():
    # Rear face x=3.3; the box's front face is x=4.1 (800 mm beyond it).
    # Supply sampled box-top points explicitly: first-hit rays can hide the box
    # behind the upper deck, which would make a depth-bound test vacuous.
    x, y = np.meshgrid(np.linspace(4.1, 4.3, 20), np.linspace(-0.3, 0.3, 40))
    box_top = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.024)))
    _, lower = evidence_counts(box_scene(columns_and_top()), 2.7, extra_points=box_top)
    assert lower == 0


@pytest.mark.parametrize(
    "depth_m,z_m,expected",
    [
        (0.0, 0.024, 200),
        (-0.0195, 0.024, 0),
        (-0.0100, 0.024, 0),
        (-0.0005, 0.024, 0),
        (0.619, 0.024, 200),
        (0.621, 0.024, 0),
        (0.2, 0.0279, 200),
        (0.2, 0.0281, 0),
        (0.2, 0.020, 0),
    ],
)
def test_evidence_volume_boundaries(depth_m, z_m, expected):
    # An exact planar scaffold with all supports above the lower band isolates
    # search-volume boundaries from fitting and unrelated front-column returns.
    prior = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    params = detector.DetectorParams()
    scaffold = np.array(
        [
            (2.7, y, 0.07)
            for centre in (-0.36, 0.0, 0.36)
            for y in np.linspace(centre - 0.04, centre + 0.04, 200)
        ]
        + [(2.7, y, 0.12) for y in np.linspace(-0.4, 0.4, 400)]
    )
    plane = detector._Plane(
        np.array((2.7, 0.0, 0.0)), np.array((-1.0, 0.0, 0.0)), scaffold, 0.0
    )
    points = np.column_stack(
        (np.full(200, 2.7 + depth_m), np.linspace(-0.3, 0.3, 200), np.full(200, z_m))
    )
    measuring = SimpleNamespace(**(dataclasses.asdict(params) | {"min_band_points": 0}))
    base = detector._opening_candidates(plane, prior, measuring, scaffold)[0]
    workspace = detector._filter_workspace(
        np.vstack((scaffold, points)), np.array((0.75, 0.0, 0.5)), prior, params
    )
    actual = detector._opening_candidates(plane, prior, measuring, workspace)[0]
    assert actual.deck_count - base.deck_count == expected


def test_the_floor_still_supplies_zero_lower_evidence():
    _, lower = evidence_counts(box_scene(columns_and_top()), 2.7)
    assert lower == 0


def test_disconnected_pads_are_a_known_valid_false_positive():
    pads = [((2.89, y, 0.012), (0.160, 0.200, 0.024)) for y in (-0.18, 0.18)]
    # Pad y edges +/-0.08 and +/-0.28 leave 40 mm to the adjacent columns.
    # This pins a known limitation, not desired connectivity: once connectivity
    # checking is implemented, this test should flip and its expectation change.
    scene = box_scene(columns_and_top() + pads)
    result = detector.detect_pockets(
        scene, load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
    )
    assert result.observation.status == "valid", result.observation.reason


def test_deck_evidence_tolerance_has_the_v4_dev_starting_value():
    assert detector.DetectorParams().deck_evidence_tol_m == 0.006


@pytest.mark.parametrize("value", [0.0, -0.001, float("nan"), float("inf"), True])
def test_invalid_deck_evidence_tolerances_are_rejected(value):
    with pytest.raises(ValueError):
        detector.DetectorParams(deck_evidence_tol_m=value)
