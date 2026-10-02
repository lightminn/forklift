"""Run-time observation viewpoints (docs/plans/2026-10-03-runtime-observation-viewpoints.md)."""

import math
from dataclasses import replace

from forklift_core.planning import Bounds, Footprint, Pose2D, Rectangle, collision_free_pose
from forklift_core.planning.observation_viewpoints import (
    BAY_PICKUP_ZONE,
    ViewpointConfig,
    runtime_viewpoints,
    view_score,
)

BOUNDS = Bounds(-3.0, 4.7, -1.75, 3.05)
FOOTPRINT = Footprint(1.25, 0.25, 0.32)
HALF_FOV = math.atan(320 / 465.74)
COMMON = dict(
    margin_m=0.1,
    pallet_depth_m=0.6,
    pallet_width_m=0.8,
    rear_to_camera_m=1.09,
    half_fov_rad=HALF_FOV,
)


def _score(pose, occluders):
    return view_score(
        pose, occluders, BAY_PICKUP_ZONE, 0.6, 0.8, 1.09, HALF_FOV, ViewpointConfig()
    )


def test_a_box_in_the_line_of_sight_lowers_the_score():
    pose = Pose2D(-0.5, 0.65, 0.0)
    clear = _score(pose, [])
    blocked = _score(pose, [Rectangle(1.2, 0.65, 0.4, 1.2)])
    assert clear > 0.3
    assert blocked < clear


def test_viewpoints_keep_the_margin_over_the_whole_arrival_box():
    occupied = [Rectangle(0.0, 0.3, 0.5, 0.5), Rectangle(-1.0, 1.4, 0.5, 0.5)]
    views = runtime_viewpoints(occupied, FOOTPRINT, BOUNDS, **COMMON)
    assert views
    for view in views:
        for dx in (-0.03, 0.0, 0.03):
            for dy in (-0.03, 0.0, 0.03):
                for dyaw in (-0.05, 0.0, 0.05):
                    pose = Pose2D(view.pose.x_m + dx, view.pose.y_m + dy, view.pose.yaw_rad + dyaw)
                    assert collision_free_pose(pose, occupied, FOOTPRINT, BOUNDS, margin_m=0.1)


def test_viewpoints_are_ranked_spread_and_bounded():
    views = runtime_viewpoints([], FOOTPRINT, BOUNDS, **COMMON)
    assert len(views) == ViewpointConfig().limit
    scores = [v.score for v in views]
    assert scores == sorted(scores, reverse=True)
    assert min(scores) >= ViewpointConfig().min_score
    for i, a in enumerate(views):
        for b in views[i + 1 :]:
            assert math.hypot(a.pose.x_m - b.pose.x_m, a.pose.y_m - b.pose.y_m) >= 0.5


def test_generation_is_deterministic():
    occupied = [Rectangle(0.5, -0.4, 0.6, 0.4, 0.3)]
    assert runtime_viewpoints(occupied, FOOTPRINT, BOUNDS, **COMMON) == runtime_viewpoints(
        occupied, FOOTPRINT, BOUNDS, **COMMON
    )


def test_a_rectangle_inside_the_pickup_zone_never_occludes():
    """It may be the pallet; nothing tells the generator which rectangle is."""
    pose = Pose2D(-0.5, 0.65, 0.0)
    inside = Rectangle(2.7, 0.65, 0.6, 0.8)
    views_without = runtime_viewpoints([], FOOTPRINT, BOUNDS, **COMMON)
    views_with = runtime_viewpoints([inside], FOOTPRINT, BOUNDS, **COMMON)
    # Far from every candidate footprint, so it only could have occluded.
    assert views_with == views_without
    assert _score(pose, []) > 0


def test_where_the_pallet_rectangle_sits_in_the_zone_does_not_move_the_viewpoints():
    """No ground-truth aiming: moving the pallet blob inside the zone changes nothing."""
    props = [Rectangle(0.2, 1.6, 0.5, 0.5)]
    a = runtime_viewpoints(props + [Rectangle(2.7, -0.1, 0.6, 0.8, 0.2)], FOOTPRINT, BOUNDS, **COMMON)
    b = runtime_viewpoints(props + [Rectangle(3.5, 1.4, 0.6, 0.8, -0.2)], FOOTPRINT, BOUNDS, **COMMON)
    assert a == b


def test_a_narrow_oblique_limit_removes_side_views():
    wide = runtime_viewpoints([], FOOTPRINT, BOUNDS, **COMMON)
    narrow = runtime_viewpoints(
        [], FOOTPRINT, BOUNDS, config=replace(ViewpointConfig(), oblique_max_rad=math.radians(5)), **COMMON
    )
    assert sum(v.score for v in narrow) < sum(v.score for v in wide)
