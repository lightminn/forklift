"""Replay actual Isaac-rendered depth that exhausted three candidate planes.

This is a deterministic SDK-free regression of a tuning capture, not a held-out
detector benchmark. RGB is deliberately zero because detect_pockets is depth-only.
"""

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from forklift_core.geometry import RigidTransform
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.rgbd_snapshot import scene_input_from_rgbd_snapshot
from forklift_core.planning.observed_approach import observed_approach_goal
from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "tests/fixtures/isaac_epal6_clutter"
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")


@pytest.fixture
def captured_scene():
    metadata = json.loads((FIXTURE / "metadata.json").read_text())
    provenance = json.loads((FIXTURE / "provenance.json").read_text())
    with np.load(FIXTURE / "isaac_seed0_depth_m.npz", allow_pickle=False) as archive:
        depth = archive["depth_m"].astype(np.float64)
    assert (
        hashlib.sha256(depth.tobytes(order="C")).hexdigest()
        == provenance["original_float64_depth_sha256"]
    )
    scene = scene_input_from_rgbd_snapshot(
        rgb=np.zeros((*depth.shape, 3), dtype=np.uint8),
        depth_m=depth,
        intrinsics=PinholeIntrinsics(**metadata["intrinsics"]),
        base_from_optical=RigidTransform(**metadata["base_from_optical"]),
        pixel_frame=metadata["intrinsics"]["frame_id"],
        **{
            key: metadata[key]
            for key in (
                "stamp_ns",
                "clock_domain",
                "source_provenance",
                "rectified",
                "rgb_registered_to_depth_grid",
                "depth_kind",
                "depth_unit",
            )
        },
    )
    return scene, RigidTransform(**metadata["world_from_base"])


def target_from_capture(detection, scene, transform):
    return observed_approach_goal(
        detection.observation,
        PRIOR,
        transform,
        transform_stamp_ns=scene.stamp_ns,
        transform_clock_domain=scene.clock_domain,
        now_ns=scene.stamp_ns,
        now_clock_domain=scene.clock_domain,
    )


def test_actual_isaac_depth_exhausts_default_candidate_budget_without_a_goal(
    captured_scene,
):
    scene, transform = captured_scene
    params = DetectorParams.derived_for(PRIOR)
    assert params.max_plane_candidates == 3
    detection = detect_pockets(scene, PRIOR, params)
    assert detection.observation.status == "no_pallet"
    assert detection.observation.reason == "no_opening_pattern"
    assert detection.diagnostics.candidate_plane_count == 3
    target = target_from_capture(detection, scene, transform)
    assert not target.success
    assert target.approach_rear is None


def test_actual_isaac_depth_detects_pallet_with_runtime_candidate_budget(
    captured_scene,
):
    scene, transform = captured_scene
    settings = yaml.safe_load(
        (ROOT / "config/isaac_perception_detector.yaml").read_text()
    )
    params = DetectorParams.derived_for(PRIOR, **settings)
    detection = detect_pockets(scene, PRIOR, params)
    assert detection.observation.status == "valid", detection.observation.reason
    assert detection.diagnostics.candidate_plane_count > 3
    assert detection.diagnostics.selected_lower > 0
    assert detection.diagnostics.selected_support_min > 0
    target = target_from_capture(detection, scene, transform)
    assert target.success
    # Original seed-0 scene placement is an evaluation label only, supplied
    # after detection and target generation. No truth is inside the fixture.
    expected_world_center = np.array([3.2369616873214544, 0.2586374133985795])
    estimated_world_center = np.array([target.pallet_site.x_m, target.pallet_site.y_m])
    assert np.linalg.norm(estimated_world_center - expected_world_center) < 0.01
    yaw_error = target.pallet_site.yaw_rad - (-0.19227656066736823)
    assert abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error))) < 0.005
