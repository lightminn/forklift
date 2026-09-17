"""Synthetic geometry/kinematics checks; no physical driving claims."""

import math

import numpy as np
import pytest

from forklift_core.planning import (
    Bounds,
    Footprint,
    FootprintCollisionChecker,
    PlannerConfig,
    Pose2D,
    Rectangle,
    collision_free_path,
    collision_free_pose,
    plan_hybrid_astar,
)

BOUNDS = Bounds(-6, 6, -6, 6)
FOOTPRINT = Footprint(0.55, 0.2, 0.25)


def assert_feasible(result, goal, config):
    assert result.success, result.status
    np.testing.assert_allclose(result.poses[-1, :2], [goal.x_m, goal.y_m], atol=1e-8)
    yaw_error = (result.poses[-1, 2] - goal.yaw_rad + math.pi) % (2 * math.pi) - math.pi
    assert abs(yaw_error) < 1e-8
    assert (
        np.max(np.abs(result.curvatures_inv_m)) <= config.curvature_limit_inv_m + 1e-9
    )
    assert set(result.directions).issubset({-1, 1})
    # Independent nonholonomic check: displacement follows the average heading,
    # with sign given by the arriving segment, including at gear-change cusps.
    delta = np.diff(result.poses[:, :2], axis=0)
    yaw_delta = (np.diff(result.poses[:, 2]) + math.pi) % (2 * math.pi) - math.pi
    mid_yaw = result.poses[:-1, 2] + yaw_delta / 2
    lateral = -delta[:, 0] * np.sin(mid_yaw) + delta[:, 1] * np.cos(mid_yaw)
    np.testing.assert_allclose(lateral, 0, atol=1e-9)
    longitudinal = delta[:, 0] * np.cos(mid_yaw) + delta[:, 1] * np.sin(mid_yaw)
    assert np.all(longitudinal * result.directions[1:] >= -1e-10)
    chord = np.linalg.norm(delta, axis=1)
    assert np.all(chord > 1e-10), "cusp poses must not be duplicated"
    np.testing.assert_allclose(
        2 * np.sin(yaw_delta / 2),
        chord * result.directions[1:] * result.curvatures_inv_m[1:],
        atol=1e-9,
    )


def test_straight_reaches_exact_goal():
    goal = Pose2D(3.137, 0, 0)
    config = PlannerConfig()
    result = plan_hybrid_astar(Pose2D(0, 0, 0), goal, [], FOOTPRINT, BOUNDS, config)
    assert_feasible(result, goal, config)
    assert result.length_m == pytest.approx(3.137)
    assert np.all(result.directions == 1)


@pytest.mark.parametrize("goal_yaw", [1.2, 1.2 + 2 * math.pi])
def test_already_at_goal_with_wrapped_heading_returns_stationary_success(goal_yaw):
    result = plan_hybrid_astar(
        Pose2D(0, 0, 1.2),
        Pose2D(0, 0, goal_yaw),
        [],
        FOOTPRINT,
        BOUNDS,
    )
    assert result.success
    assert result.length_m == 0
    assert result.poses.shape == (1, 3)


@pytest.mark.parametrize("goal", [Pose2D(3, 1, 0), Pose2D(3, 2, math.pi / 2)])
def test_lateral_and_heading_changes_are_car_feasible(goal):
    config = PlannerConfig(curvature_limit_inv_m=0.8)
    result = plan_hybrid_astar(Pose2D(0, 0, 0), goal, [], FOOTPRINT, BOUNDS, config)
    assert_feasible(result, goal, config)


def test_reverse_path_in_narrow_corridor():
    bounds = Bounds(-4, 2, -0.4, 0.4)
    goal = Pose2D(-2, 0, 0)
    config = PlannerConfig()
    result = plan_hybrid_astar(Pose2D(0, 0, 0), goal, [], FOOTPRINT, bounds, config)
    assert_feasible(result, goal, config)
    assert np.all(result.directions == -1)


def test_gear_change_preserves_feasible_arriving_segment_controls():
    goal = Pose2D(0.5, 0.3, 0)
    config = PlannerConfig(curvature_limit_inv_m=1.0, primitive_length_m=0.3)
    result = plan_hybrid_astar(
        Pose2D(0, 0, 0),
        goal,
        [],
        FOOTPRINT,
        Bounds(-1.5, 1.5, -1.5, 1.5),
        config,
    )
    assert_feasible(result, goal, config)
    assert np.any(np.diff(result.directions) != 0)


def test_public_reusable_checker_rejects_nonfinite_or_negative_margin():
    checker = FootprintCollisionChecker([], FOOTPRINT, BOUNDS)
    with pytest.raises(ValueError):
        checker.free((float("nan"), 0, 0))
    with pytest.raises(ValueError):
        checker.free((0, 0, 0), margin_m=-0.1)


def test_detours_obstacle_with_full_footprint():
    obstacle = Rectangle(0, 0, 1.0, 1.8)
    goal = Pose2D(3, 0, 0)
    config = PlannerConfig(curvature_limit_inv_m=0.8)
    result = plan_hybrid_astar(
        Pose2D(-3, 0, 0), goal, [obstacle], FOOTPRINT, BOUNDS, config
    )
    assert_feasible(result, goal, config)
    assert np.max(np.abs(result.poses[:, 1])) > 1.15
    assert collision_free_path(result.poses, [obstacle], FOOTPRINT, BOUNDS)


def test_blocked_wall_returns_failure_with_bounded_work():
    config = PlannerConfig(max_expansions=120)
    result = plan_hybrid_astar(
        Pose2D(-2, 0, 0),
        Pose2D(2, 0, 0),
        [Rectangle(0, 0, 0.1, 12)],
        FOOTPRINT,
        BOUNDS,
        config,
    )
    assert not result.success
    assert result.status in {"no_path", "expansion_limit"}
    assert result.expanded_nodes <= 120
    assert result.poses.shape == (0, 3)


def test_footprint_corner_collision_and_rotated_obstacle():
    assert not collision_free_pose(
        Pose2D(0, 0, math.pi / 4),
        [Rectangle(0.33, 0.4, 0.05, 0.05, 0.3)],
        FOOTPRINT,
        BOUNDS,
    )
    assert collision_free_pose(Pose2D(0, 0, 0), [], FOOTPRINT, BOUNDS)
    assert not collision_free_pose(Pose2D(5.6, 0, 0), [], FOOTPRINT, BOUNDS)


def test_loaded_footprint_can_invalidate_empty_route():
    obstacle = Rectangle(0.9, 0, 0.2, 0.2)
    pose = Pose2D(0, 0, 0)
    assert collision_free_pose(pose, [obstacle], FOOTPRINT, BOUNDS)
    assert not collision_free_pose(pose, [obstacle], Footprint(1.0, 0.2, 0.4), BOUNDS)


def test_sparse_path_cannot_tunnel_through_thin_obstacle():
    poses = np.array([[0.0, 0, 0], [2.0, 0, 0]])
    assert not collision_free_path(
        poses,
        [Rectangle(0.537, 0, 0.001, 0.02)],
        Footprint(0.001, 0.001, 0.001),
        BOUNDS,
        max_step_m=0.2,
    )


@pytest.mark.parametrize(
    "build",
    [
        lambda: Pose2D(float("nan"), 0, 0),
        lambda: Rectangle(0, 0, -1, 1),
        lambda: Footprint(1, -1, 1),
        lambda: Bounds(1, -1, 0, 1),
        lambda: PlannerConfig(curvature_limit_inv_m=0),
        lambda: PlannerConfig(xy_resolution_m=float("inf")),
        lambda: PlannerConfig(max_expansions=1.5),
    ],
)
def test_invalid_geometry_or_settings_raise(build):
    with pytest.raises(ValueError):
        build()


def test_colliding_goal_returns_invalid_goal():
    result = plan_hybrid_astar(
        Pose2D(-2, 0, 0),
        Pose2D(0, 0, 0),
        [Rectangle(0, 0, 1, 1)],
        FOOTPRINT,
        BOUNDS,
    )
    assert not result.success
    assert result.status == "invalid_goal"


def test_fixed_seed_scene_is_deterministic():
    rng = np.random.default_rng(1729)
    obstacles = [
        Rectangle(x, y, 0.3, 0.3, yaw) for x, y, yaw in rng.uniform(1, 3, (5, 3))
    ]
    args = (Pose2D(-3, -2, 0), Pose2D(3, -1, 0), obstacles, FOOTPRINT, BOUNDS)
    first, second = plan_hybrid_astar(*args), plan_hybrid_astar(*args)
    assert first.success and second.success
    np.testing.assert_array_equal(first.poses, second.poses)
    assert first.expanded_nodes == second.expanded_nodes
