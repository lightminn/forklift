"""Ray-rendered EPAL 6 depth to an observed approach plan, without simulator SDKs.

The rig receives placement truth to render depth. Only pixels, calibration and
acquisition metadata cross into perception; truth is used solely for assertions.
This is synthetic optical-z geometry, not an Isaac or physical-camera test.
"""

import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.rgbd_snapshot import scene_input_from_rgbd_snapshot
from forklift_core.planning import (
    Bounds,
    Footprint,
    PlannerConfig,
    Pose2D,
    Rectangle,
    collision_free_path,
    plan_hybrid_astar,
)
from forklift_core.planning.observed_approach import observed_approach_goal
from tools.scene_rig import Camera, pallet, place, render

ROOT = Path(__file__).resolve().parents[2]
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
STAMP_NS = 2_000_000_000


def snapshot_from_rig(scene, *, depth_m=None):
    """Pass the acquisition boundary explicitly, without scene labels or truth."""
    return scene_input_from_rgbd_snapshot(
        rgb=scene.rgb,
        depth_m=scene.depth_m if depth_m is None else depth_m,
        intrinsics=scene.intrinsics,
        base_from_optical=scene.base_from_optical,
        pixel_frame=scene.intrinsics.frame_id,
        stamp_ns=STAMP_NS,
        clock_domain="ros_sim",
        source_provenance="synthetic",
        rectified=True,
        rgb_registered_to_depth_grid=True,
        depth_kind="optical_axis_z",
        depth_unit="m",
    )


def world_from_base():
    yaw = 0.3
    c, s = math.cos(yaw), math.sin(yaw)
    return RigidTransform(
        "base_link", "world", [[c, -s, 0], [s, c, 0], [0, 0, 1]], [1, -0.5, 0]
    )


def observed_target(observation):
    return observed_approach_goal(
        observation,
        PRIOR,
        world_from_base(),
        transform_stamp_ns=STAMP_NS,
        transform_clock_domain="ros_sim",
        now_ns=STAMP_NS,
        now_clock_domain="ros_sim",
    )


@pytest.mark.parametrize("pallet_yaw_rad", [0.1, 0.2])
def test_epal6_depth_produces_observed_world_goal_and_feasible_plan(pallet_yaw_rad):
    scene = render(
        place(pallet(GEOMETRY), x_m=2.5, y_m=0.1, yaw_rad=pallet_yaw_rad),
        camera=Camera(xyz_m=(0.75, 0, 0.5)),
        quantize=False,
    )
    acquired = snapshot_from_rig(scene)
    np.testing.assert_array_equal(acquired.depth_m, scene.depth_m)
    detection = detect_pockets(acquired, PRIOR, DetectorParams.derived_for(PRIOR))
    assert detection.observation.status == "valid", detection.observation.reason
    target = observed_target(detection.observation)
    assert target.success, target.status

    # Truth is first consulted after the observed target has been computed.
    transform = world_from_base()
    expected_center = transform.rotation @ [2.5, 0.1, 0] + transform.translation_m
    site = target.pallet_site
    assert (
        np.linalg.norm([site.x_m - expected_center[0], site.y_m - expected_center[1]])
        < 0.025
    )
    expected_yaw = pallet_yaw_rad + 0.3
    assert (
        abs(
            math.atan2(
                math.sin(site.yaw_rad - expected_yaw),
                math.cos(site.yaw_rad - expected_yaw),
            )
        )
        < 0.01
    )
    expected_approach = expected_center[:2] - 1.69 * np.array(
        [math.cos(expected_yaw), math.sin(expected_yaw)]
    )
    goal = target.approach_rear
    assert (
        np.linalg.norm(
            [goal.x_m - expected_approach[0], goal.y_m - expected_approach[1]]
        )
        < 0.025
    )

    # Planning receives the inferred pallet envelope, never its rendered pose.
    start_world = transform.rotation @ [-0.34, 0, 0] + transform.translation_m
    start = Pose2D(start_world[0], start_world[1], 0.3)
    obstacles = [
        Rectangle(
            site.x_m,
            site.y_m,
            PRIOR.overall_depth_m,
            PRIOR.overall_width_m,
            site.yaw_rad,
        )
    ]
    footprint = Footprint(1.29, 0.17, 0.36)
    bounds = Bounds(-6, 6, -6, 6)
    config = PlannerConfig(curvature_limit_inv_m=0.5, clearance_m=0.03)
    plan = plan_hybrid_astar(start, goal, obstacles, footprint, bounds, config)
    assert plan.success, plan.status
    np.testing.assert_allclose(
        plan.poses[-1], [goal.x_m, goal.y_m, goal.yaw_rad], atol=1e-8
    )
    assert collision_free_path(plan.poses, obstacles, footprint, bounds)
    delta = np.diff(plan.poses[:, :2], axis=0)
    yaw_delta = np.arctan2(
        np.sin(np.diff(plan.poses[:, 2])), np.cos(np.diff(plan.poses[:, 2]))
    )
    midpoint_yaw = plan.poses[:-1, 2] + yaw_delta / 2
    lateral = -delta[:, 0] * np.sin(midpoint_yaw) + delta[:, 1] * np.cos(midpoint_yaw)
    np.testing.assert_allclose(lateral, 0, atol=1e-9)
    assert np.max(np.abs(plan.curvatures_inv_m)) <= 0.5


@pytest.mark.parametrize("missing_depth", [False, True])
def test_absent_or_invalid_depth_exposes_no_drive_goal(missing_depth):
    scene = render([], camera=Camera(xyz_m=(0.75, 0, 0.5)), quantize=False)
    depth = np.full_like(scene.depth_m, np.nan) if missing_depth else scene.depth_m
    acquired = snapshot_from_rig(scene, depth_m=depth)
    detection = detect_pockets(acquired, PRIOR, DetectorParams.derived_for(PRIOR))
    assert detection.observation.status == ("invalid" if missing_depth else "no_pallet")
    target = observed_target(detection.observation)
    assert not target.success
    assert target.pallet_site is None
    assert target.approach_rear is None
    assert target.prealign_rear is None
