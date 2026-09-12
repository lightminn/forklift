import dataclasses
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import (
    _DEFAULT_PARAMS,
    count_interior_gaps,
    detect_pockets,
    occupied_columns,
)
from forklift_core.perception.pocket_observation import yaw_difference_rad

REPO_PRIOR = Path(__file__).resolve().parents[3] / "config" / "pallet_prior_v1.yaml"


def test_the_scene_helper_puts_the_openings_and_the_front_face_where_it_says(
    pallet_scene,
):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.2), yaw_rad=0.4, opening_width_m=0.24
    )
    assert scene.depth_m.shape == (480, 640) and scene.rgb.shape == (480, 640, 3)
    assert truth["left_centre_m"][2] == pytest.approx(0.15)
    spacing = math.dist(truth["left_centre_m"], truth["right_centre_m"])
    assert spacing == pytest.approx(0.10 + 0.24)  # 2d = 0.10 + w
    # a ray through an opening centre must return something well BEHIND the front plane;
    # which surface it is (deck top or back wall) depends on the geometry, so check the depth
    front_depth = depth_at(scene, truth["front_plane_point_m"])
    opening_depth = depth_at(scene, truth["left_centre_m"])
    assert opening_depth > front_depth + 0.2
    # a ray through the centre spacer must stop ON the front plane
    spacer = midpoint(truth["left_centre_m"], truth["right_centre_m"])
    assert depth_at(scene, spacer) == pytest.approx(front_depth, abs=0.02)


def test_the_scene_helper_reports_ray_fractions_that_separate_open_from_occluded(
    pallet_scene,
):
    clear, truth = pallet_scene()
    assert opening_ray_fractions(clear, truth, "left")["behind"] > 0.5
    assert opening_ray_fractions(clear, truth, "left")["front"] == 0.0
    blocked, truth_b = pallet_scene(
        occluder={
            "side": "left",
            "gap_m": 0.4,
            "width_frac": 0.4,
            "depth_m": 0.10,
            "height_m": 0.80,
        }
    )
    left = opening_ray_fractions(blocked, truth_b, "left")
    right = opening_ray_fractions(blocked, truth_b, "right")
    assert left["front"] > 0.5  # the detector threshold occluded_front_frac
    assert right["behind"] > 0.5  # the other pocket stays open


def depth_at(scene, point_base):
    transform = scene.base_from_optical
    optical = (np.asarray(point_base) - transform.translation_m) @ transform.rotation
    x, y, z = optical
    intrinsics = scene.intrinsics
    u = round(x * intrinsics.fx / z + intrinsics.cx)
    v = round(y * intrinsics.fy / z + intrinsics.cy)
    return scene.depth_m[v, u]


def midpoint(left, right):
    return (np.asarray(left) + np.asarray(right)) / 2


# The plan calls this independent checker as a function, without a fixture
# parameter. Load the same helper by path while preserving those test bodies.
_SPEC = importlib.util.spec_from_file_location(
    "synthetic_scene_check",
    Path(__file__).resolve().parents[2] / "fixtures" / "synthetic_scene.py",
)
_SCENE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_SCENE)
opening_ray_fractions = _SCENE.opening_ray_fractions


def detect(scene, **overrides):
    params = (
        dataclasses.replace(_DEFAULT_PARAMS, **overrides)
        if overrides
        else _DEFAULT_PARAMS
    )
    return detect_pockets(scene, load_pallet_prior(REPO_PRIOR), params)


def assert_pockets_match(obs, truth, tol_m=0.02):
    assert math.dist(obs.left.center_m, truth["left_centre_m"]) <= tol_m
    assert math.dist(obs.right.center_m, truth["right_centre_m"]) <= tol_m


def test_a_straight_pallet_is_detected_within_two_centimetres(pallet_scene):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.0), yaw_rad=0.0, opening_width_m=0.24
    )
    obs = detect(scene).observation
    assert obs.status == "valid" and obs.frame_id == "base_link"
    assert_pockets_match(obs, truth)
    assert abs(yaw_difference_rad(obs.insertion_yaw_rad, 0.0)) < math.radians(2)
    assert obs.position_sigma_m is None and obs.yaw_sigma_rad is None
    assert obs.stamp_ns == scene.stamp_ns and obs.clock_domain == scene.clock_domain
    assert obs.source_provenance == scene.source_provenance


@pytest.mark.parametrize("yaw", [-0.4, -0.2, 0.2, 0.4])
def test_rotated_pallets_keep_the_insertion_axis_sign(pallet_scene, yaw):
    scene, truth = pallet_scene(centre_xy_m=(2.6, 0.1), yaw_rad=yaw)
    obs = detect(scene).observation
    assert obs.status == "valid"
    # a double sign flip would land on yaw + pi; the wrapped difference catches it
    assert abs(yaw_difference_rad(obs.insertion_yaw_rad, yaw)) < math.radians(2)
    left_axis = (-math.sin(yaw), math.cos(yaw), 0.0)
    delta = np.array(obs.left.center_m) - np.array(obs.right.center_m)
    assert float(np.dot(delta, left_axis)) > 0


@pytest.mark.parametrize(
    "centre_x,width", [(2.2, 0.20), (2.5, 0.24), (3.0, 0.28), (2.37, 0.22)]
)
def test_accuracy_holds_across_distance_and_grid_phase(pallet_scene, centre_x, width):
    scene, truth = pallet_scene(centre_xy_m=(centre_x, 0.0), opening_width_m=width)
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth)
    spacing = math.dist(obs.left.center_m, obs.right.center_m)
    assert spacing == pytest.approx(0.10 + width, abs=0.03)


def test_the_z_band_margin_keeps_deck_edge_points_out_of_the_opening_columns():
    # Measured on catalogue v1: a naive band [deck, height-deck] fails all 60 positive
    # scenes because deck-top returns sit at z ~ 0.0501-0.0514. Reproducing that through
    # ray tracing depends on a single pixel row landing in a ~1 px window, so this is a
    # direct unit test of the column builder with an explicit point cloud instead.
    prior = load_pallet_prior(REPO_PRIOR)
    deck_edge = np.array([[0.0, y, 0.0505] for y in np.linspace(-0.35, 0.35, 71)])
    spacer = np.array(
        [[0.0, y, z] for y in (-0.0, 0.30, -0.30) for z in np.linspace(0.07, 0.23, 17)]
    )
    points = np.vstack([deck_edge, spacer])
    naive = occupied_columns(
        points, prior, dataclasses.replace(_DEFAULT_PARAMS, band_margin_m=0.0)
    )
    guarded = occupied_columns(points, prior, _DEFAULT_PARAMS)
    assert count_interior_gaps(naive) == 0  # deck edge fills every column
    assert count_interior_gaps(guarded) == 2  # the two openings reappear


def test_space_outside_the_pallet_is_not_mistaken_for_openings(pallet_scene):
    scene, truth = pallet_scene(centre_xy_m=(2.2, 0.0), opening_width_m=0.20)
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth)
    # fake outer gaps would put the centres about 0.6 m apart instead of 0.30 m
    assert math.dist(obs.left.center_m, obs.right.center_m) == pytest.approx(
        0.30, abs=0.03
    )


def test_an_occluded_pocket_is_invalid_not_a_silent_guess(pallet_scene):
    scene, truth = pallet_scene(
        occluder={
            "side": "left",
            "gap_m": 0.4,
            "width_frac": 0.4,
            "depth_m": 0.10,
            "height_m": 0.80,
        }
    )
    assert (
        opening_ray_fractions(scene, truth, "left")["front"] > 0.5
    )  # fixture precondition
    obs = detect(scene).observation
    assert obs.status == "invalid" and obs.reason.startswith("pocket_occluded:left")
    assert obs.left is None and obs.right is None and obs.insertion_yaw_rad is None
    assert obs.position_sigma_m is None


def test_unknown_returns_in_the_openings_are_not_treated_as_open(pallet_scene):
    scene, _ = pallet_scene(openings_unknown=True)
    obs = detect(scene).observation
    assert obs.status == "invalid" and "ambiguous" in obs.reason


def test_a_partial_unknown_patch_does_not_invent_an_opening(pallet_scene):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.0), unknown_patch=(280, 200, 320, 260)
    )
    obs = detect(scene).observation
    assert obs.status in ("valid", "invalid")
    if obs.status == "valid":
        assert_pockets_match(obs, truth, tol_m=0.03)
    else:
        assert "ambiguous" in obs.reason or "occluded" in obs.reason


def test_a_few_stray_points_inside_an_opening_do_not_destroy_the_detection(
    pallet_scene,
):
    scene, truth = pallet_scene(
        extra_box={"centre_xy_m": (2.19, 0.17), "size_m": (0.02, 0.02, 0.02)}
    )
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth, tol_m=0.03)


def test_scenes_without_a_target_pallet_report_no_pallet(pallet_scene):
    for kwargs in ({"pallet": False}, {"pallet": False, "lookalike": True}):
        scene, _ = pallet_scene(**kwargs)
        obs = detect(scene).observation
        assert obs.status == "no_pallet" and obs.reason


def test_all_unknown_depth_is_invalid_not_no_pallet(pallet_scene):
    scene, _ = pallet_scene()
    blank = dataclasses.replace(scene, depth_m=np.full_like(scene.depth_m, np.nan))
    obs = detect(blank).observation
    assert obs.status == "invalid" and obs.reason == "insufficient_points"


def test_a_larger_competing_box_face_does_not_win_over_the_pallet(pallet_scene):
    scene, truth = pallet_scene(
        centre_xy_m=(3.0, 0.0),
        extra_box={"centre_xy_m": (2.0, 1.1), "size_m": (0.5, 1.4, 1.2)},
    )
    obs = detect(scene).observation
    assert obs.status == "valid"
    assert_pockets_match(obs, truth, tol_m=0.03)


def test_openings_outside_the_prior_width_range_are_reported_as_a_mismatch(
    pallet_scene,
):
    scene, _ = pallet_scene(opening_width_m=0.34)  # prior allows 0.18-0.30
    obs = detect(scene).observation
    assert obs.status == "invalid" and obs.reason == "opening_width_mismatch"


def test_a_pallet_closer_than_the_search_range_gives_a_defined_result(pallet_scene):
    scene, _ = pallet_scene(
        centre_xy_m=(1.4, 0.0)
    )  # front face ~0.35 m from the camera
    obs = detect(scene).observation
    assert obs.status in ("no_pallet", "invalid") and obs.reason


def test_diagnostics_report_plane_quality_ray_counts_and_the_prior_flag(pallet_scene):
    scene, _ = pallet_scene()
    diag = detect(scene).diagnostics
    assert diag.plane_inlier_count >= 300
    assert diag.plane_residual_p95_m is not None and diag.plane_residual_p95_m < 0.01
    left = diag.opening_rays["left"]
    assert left.total == left.front + left.behind + left.near_plane + left.unknown
    assert left.behind_fraction > 0.3
    assert diag.centre_height_is_prior is True
    assert diag.boundary_resolution_m == pytest.approx(0.01)
    assert diag.seed == 20260913 and diag.elapsed_s > 0


def test_detection_is_deterministic_for_a_fixed_seed(pallet_scene):
    scene, _ = pallet_scene(centre_xy_m=(2.7, -0.3), yaw_rad=0.25)
    assert detect(scene).observation.to_json() == detect(scene).observation.to_json()


@pytest.mark.parametrize(
    "overrides",
    [
        {"cell_m": 0.0},
        {"range_min_m": 5.0, "range_max_m": 1.0},
        {"occluded_front_frac": 1.5},
        {"ransac_iterations": 0},
        {"plane_inlier_m": float("nan")},
    ],
)
def test_invalid_detector_params_are_rejected(overrides):
    with pytest.raises(ValueError):
        dataclasses.replace(_DEFAULT_PARAMS, **overrides)


@pytest.mark.parametrize(
    "width,expected_status", [(0.16, "invalid"), (0.29, "valid"), (0.30, "valid")]
)
def test_width_validation_precedes_the_reporting_correction(
    pallet_scene, width, expected_status
):
    scene, _ = pallet_scene(opening_width_m=width)
    obs = detect(scene).observation
    assert obs.status == expected_status
    if expected_status == "invalid":
        assert obs.reason == "opening_width_mismatch"
