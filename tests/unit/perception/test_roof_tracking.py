"""Known empty-pallet continuation requires fresh rear and groove depth evidence."""

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.roof_tracking import RoofTrackingParams, track_roof
from tools import scene_rig as rig

ROOT = Path(__file__).resolve().parents[3]
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
CAMERA = rig.Camera((0.75, 0, 0.27))


@pytest.fixture(scope="module")
def inserted_scene():
    return rig.render(rig.place(rig.pallet(GEOMETRY), x_m=0.89), camera=CAMERA)


@pytest.mark.parametrize(
    "front,yaw,lateral", [(1.3, -0.02, 0.015), (0.85, 0.02, -0.015), (0.59, 0.0, 0.0)]
)
def test_measured_pose_recovers_from_biased_association(front, yaw, lateral):
    scene = rig.render(
        rig.place(rig.pallet(GEOMETRY), x_m=front + 0.3, y_m=lateral, yaw_rad=yaw),
        camera=CAMERA,
    )
    # The association is deliberately wrong by 10/9 mm and 7 mrad.
    result = track_roof(scene, GEOMETRY, (front + 0.01, lateral + 0.009), yaw + 0.007)
    obs = result.observation
    assert obs.status == "valid", result.diagnostics
    true_front = np.array(
        [front + 0.3 - 0.3 * math.cos(yaw), lateral - 0.3 * math.sin(yaw)]
    )
    expected_left = np.r_[
        true_front + 0.18625 * np.array([-math.sin(yaw), math.cos(yaw)]), 0.061
    ]
    assert np.linalg.norm(np.array(obs.left.center_m) - expected_left) < 0.015
    assert abs(obs.insertion_yaw_rad - yaw) < 0.006
    if front == 0.59:
        assert np.linalg.norm(np.array(obs.left.center_m) - expected_left) < 0.004
    assert obs.stamp_ns == scene.stamp_ns
    assert obs.source_provenance == "synthetic"
    assert obs.position_sigma_m >= 0.001
    assert result.diagnostics.distinct_groove_edges >= 4
    assert result.diagnostics.interior_groove_edges >= 2


def test_no_cached_pose_after_total_depth_loss(inserted_scene):
    assert (
        track_roof(inserted_scene, GEOMETRY, (0.60, 0.009), 0.007).observation.status
        == "valid"
    )
    lost = dataclasses.replace(
        inserted_scene,
        depth_m=np.full_like(inserted_scene.depth_m, np.nan),
        stamp_ns=1234,
    )
    obs = track_roof(lost, GEOMETRY, (0.60, 0.009), 0.007).observation
    assert obs.status == "invalid"
    assert obs.left is None and obs.right is None
    assert obs.stamp_ns == 1234


@pytest.mark.parametrize("missing", ["rear", "grooves"])
def test_unknown_depth_is_not_edge_evidence(inserted_scene, missing):
    xyz = inserted_scene.base_from_optical.translation_m + inserted_scene.depth_m[
        ..., None
    ] * rig._rays(inserted_scene.intrinsics, inserted_scene.base_from_optical.rotation)
    mask = xyz[..., 0] > 1.16 if missing == "rear" else xyz[..., 2] < 0.142
    depth = inserted_scene.depth_m.copy()
    depth[mask] = np.nan
    result = track_roof(
        dataclasses.replace(inserted_scene, depth_m=depth),
        GEOMETRY,
        (0.60, 0.009),
        0.007,
    )
    assert result.observation.status == "invalid", result.diagnostics


@pytest.mark.parametrize(
    "boxes", [[], [rig.Box((0.89, 0.0, 0.072), (0.6, 0.8, 0.144))]]
)
def test_floor_or_solid_slab_does_not_supply_groove_pattern(boxes):
    scene = rig.render(boxes, camera=CAMERA)
    assert (
        track_roof(scene, GEOMETRY, (0.60, 0.009), 0.007).observation.status
        == "invalid"
    )


def test_reference_cannot_recover_wrong_pallet_location(inserted_scene):
    assert (
        track_roof(inserted_scene, GEOMETRY, (0.85, 0.009), 0.007).observation.status
        == "invalid"
    )


@pytest.mark.parametrize(
    "change",
    [
        {"source_provenance": "synthetic_ground_truth"},
        {"rectified": False},
        {"rgb_registered_to_depth_grid": False},
        {"stamp_ns": True},
        {"clock_domain": "wall"},
        {"depth_m": np.zeros((10, 10))},
    ],
)
def test_malformed_or_ground_truth_input_is_rejected(inserted_scene, change):
    with pytest.raises(ValueError):
        track_roof(
            dataclasses.replace(inserted_scene, **change), GEOMETRY, (0.60, 0), 0
        )


@pytest.mark.parametrize(
    "reference,yaw", [((float("nan"), 0), 0), ((0.6,), 0), ((0.6, 0), True)]
)
def test_malformed_association_is_rejected(inserted_scene, reference, yaw):
    with pytest.raises(ValueError):
        track_roof(inserted_scene, GEOMETRY, reference, yaw)


@pytest.mark.parametrize(
    "parameters",
    [
        {"min_roof_points": True},
        {"min_distinct_edges": 3},
        {"roof_height_tolerance_m": float("nan")},
        {"min_pattern_inlier_fraction": 1.1},
    ],
)
def test_parameters_reject_invalid_or_underconstrained_values(parameters):
    with pytest.raises(ValueError):
        RoofTrackingParams(**parameters)


def test_output_height_comes_from_depth_when_reference_height_is_biased(inserted_scene):
    result = track_roof(inserted_scene, GEOMETRY, (0.60, 0.009, 0.058), 0.007)
    assert result.observation.status == "valid", result.diagnostics
    assert abs(result.observation.left.center_m[2] - 0.061) < 0.001


def test_geometry_dimensions_determine_inferred_hidden_pockets():
    larger = dataclasses.replace(
        GEOMETRY, overall_depth_m=0.64, overall_height_m=0.154, block_height_m=0.088
    )
    scene = rig.render(rig.place(rig.pallet(larger), x_m=0.91), camera=CAMERA)
    result = track_roof(scene, larger, (0.6, 0.009), 0.007)
    assert result.observation.status == "valid", result.diagnostics
    assert (
        np.linalg.norm(
            np.array(result.observation.left.center_m) - [0.59, 0.18625, 0.066]
        )
        < 0.005
    )
    assert result.observation.left.height_m == pytest.approx(0.088)


def test_far_rear_staircase_is_compared_to_projected_pixel_interval():
    # At this range/yaw a straight rear edge straddles two pixel rows 22 mm apart.
    scene = rig.render(
        rig.place(rig.pallet(GEOMETRY), x_m=1.6, yaw_rad=0.005), camera=CAMERA
    )
    result = track_roof(scene, GEOMETRY, (1.31, 0.009), 0.012)
    assert result.observation.status == "valid", result.diagnostics
    assert result.diagnostics.rear_rms_m > 0.008
    assert abs(result.diagnostics.front_center_m[0] - 1.3) < 0.015
    assert abs(result.observation.insertion_yaw_rad - 0.005) < 0.006


def test_wrong_board_pattern_and_foreground_occlusion_are_rejected():
    wrong = dataclasses.replace(GEOMETRY, top_board_count=5)
    wrong_scene = rig.render(rig.place(rig.pallet(wrong), x_m=0.89), camera=CAMERA)
    assert (
        track_roof(wrong_scene, GEOMETRY, (0.6, 0.009), 0.007).observation.status
        == "invalid"
    )
    # A foreground box replaces the measured rear edge while leaving some roof.
    boxes = rig.place(rig.pallet(GEOMETRY), x_m=0.89)
    boxes.append(rig.Box((1.11, 0, 0.21), (0.08, 0.9, 0.18)))
    blocked = rig.render(boxes, camera=CAMERA)
    assert (
        track_roof(blocked, GEOMETRY, (0.6, 0.009), 0.007).observation.status
        == "invalid"
    )
