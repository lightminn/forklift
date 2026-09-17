"""A–D synthetic approach geometry only: no perception, insertion, or load test."""

import numpy as np

from forklift_core.planning import (
    Bounds,
    Footprint,
    PlannerConfig,
    Pose2D,
    Rectangle,
    collision_free_path,
    plan_hybrid_astar,
)

FOOTPRINT = Footprint(0.55, 0.2, 0.25)
CONFIG = PlannerConfig(curvature_limit_inv_m=1.0, primitive_length_m=0.3)


def test_case_a_straight():
    result = plan_hybrid_astar(
        Pose2D(-2, 0, 0),
        Pose2D(2, 0, 0),
        [],
        FOOTPRINT,
        Bounds(-4, 4, -3, 3),
        CONFIG,
    )
    assert result.success
    np.testing.assert_allclose(result.poses[:, 1:], 0, atol=1e-10)
    assert np.all(result.directions == 1)


def test_case_b_curved():
    result = plan_hybrid_astar(
        Pose2D(-2, 0, 0),
        Pose2D(2, 1, 0),
        [],
        FOOTPRINT,
        Bounds(-4, 4, -3, 3),
        CONFIG,
    )
    assert result.success
    assert np.all(result.directions == 1)
    assert np.max(np.abs(result.poses[:, 2])) > 0.1
    np.testing.assert_allclose(result.poses[-1], [2, 1, 0], atol=1e-8)


def test_case_c_reverse():
    # Nearby lateral pocket approach in a confined synthetic aisle requires
    # room-making before the final forward approach.
    bounds = Bounds(-1.5, 1.5, -1.5, 1.5)
    result = plan_hybrid_astar(
        Pose2D(0, 0, 0),
        Pose2D(0.5, 0.3, 0),
        [],
        FOOTPRINT,
        bounds,
        CONFIG,
    )
    assert result.success
    assert result.directions[0] == -1
    assert result.directions[-1] == 1
    assert result.poses[:, 0].min() < -0.5
    assert collision_free_path(result.poses, [], FOOTPRINT, bounds)


def test_case_d_rear_obstacle():
    bounds = Bounds(-1.5, 1.5, -1.5, 1.5)
    obstacles = [Rectangle(-0.75, 0, 0.15, 0.8)]
    result = plan_hybrid_astar(
        Pose2D(0, 0, 0),
        Pose2D(0.5, 0.3, 0),
        obstacles,
        FOOTPRINT,
        bounds,
        CONFIG,
    )
    assert result.success
    # The rear obstacle prevents the unrestricted case-C retreat. Relocate
    # forward, change gear in the available space, then approach forward.
    assert result.directions[0] == 1
    assert -1 in result.directions
    assert result.directions[-1] == 1
    assert result.poses[:, 0].min() > -0.4
    assert collision_free_path(result.poses, obstacles, FOOTPRINT, bounds)
    np.testing.assert_allclose(result.poses[-1], [0.5, 0.3, 0], atol=1e-8)
