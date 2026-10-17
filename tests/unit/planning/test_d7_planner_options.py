"""Priority-5 plan D7a/D7b planner options: Reeds-Shepp goal connection, shot cap, turn cycles."""

import math

import numpy as np
import pytest

from forklift_core.planning import Bounds, Footprint, Pose2D, Rectangle
from forklift_core.planning.hybrid_astar import PlannerConfig, _advance, plan_hybrid_astar
from forklift_core.planning.reeds_shepp import reeds_shepp_words

OPEN = Bounds(-30.0, 10.0, -30.0, 10.0)
LOADED = Footprint(1.56, 0.17, 0.40)
MEASURED = dict(curvature_limit_inv_m=0.29, primitive_length_m=0.25, max_expansions=30000, clearance_m=0.10)
# l8_measured seed 1 + N1 (plan D7d): the stall replan's start and its goal 1 cm ahead, 0.023 rad off.
STALL = Pose2D(-7.848, -18.669, -1.422)
STALL_GOAL = Pose2D(-7.851, -18.681, -1.399)


def gear_changes(result):
    d = result.directions
    return int((d[1:] != d[:-1]).sum()) if len(d) > 1 else 0


def test_every_reeds_shepp_word_ends_at_the_goal():
    rng = np.random.default_rng(3)
    for _ in range(300):
        goal = (rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-math.pi, math.pi))
        words = reeds_shepp_words((0.0, 0.0, 0.0), goal, 0.29)
        assert words
        for word in words:
            pose = (0.0, 0.0, 0.0)
            for turn, gear, length in word:
                pose = _advance(pose, gear * length, turn * 0.29)
            assert math.hypot(pose[0] - goal[0], pose[1] - goal[1]) < 1e-6
            assert abs(math.atan2(math.sin(pose[2] - goal[2]), math.cos(pose[2] - goal[2]))) < 1e-6


def test_the_defaults_keep_the_dubins_loop_and_reeds_shepp_shortens_a_lateral_offset():
    start = STALL
    offset = Pose2D(start.x_m - 0.0375 * math.sin(start.yaw_rad), start.y_m + 0.0375 * math.cos(start.yaw_rad), start.yaw_rad)
    dubins = plan_hybrid_astar(start, offset, [], LOADED, OPEN, PlannerConfig(**MEASURED))
    rs = plan_hybrid_astar(start, offset, [], LOADED, OPEN, PlannerConfig(**MEASURED, goal_connection="reeds_shepp"))
    assert dubins.success and dubins.length_m > 20.0 and gear_changes(dubins) == 0
    assert rs.success and rs.length_m < 1.5 and gear_changes(rs) >= 1
    # Every Reeds-Shepp segment it drives is at least the minimum (no centimetre cusps).
    edges = np.flatnonzero(rs.directions[1:] != rs.directions[:-1]) + 1
    bounds = [0, *edges, len(rs.poses) - 1]
    for a, b in zip(bounds, bounds[1:]):
        seg = float(np.sum(np.hypot(*np.diff(rs.poses[a : b + 1, :2], axis=0).T)))
        assert seg >= 0.10 - 1e-6


def test_the_shot_cap_replaces_the_loop_and_falls_back_when_nothing_else_plans():
    loop = plan_hybrid_astar(STALL, STALL_GOAL, [], LOADED, OPEN, PlannerConfig(**MEASURED))
    capped = plan_hybrid_astar(STALL, STALL_GOAL, [], LOADED, OPEN, PlannerConfig(**MEASURED, shot_cap_m=4.0))
    assert loop.length_m > 20.0 and loop.shots_capped == 0
    assert capped.success and capped.length_m < 1.5 and capped.shots_capped >= 1 and not capped.shot_fallback
    assert loop.root_shot == "taken" and capped.root_shot == "capped"
    # A search that expands nothing: only the capped-out connection exists, and it is returned.
    stuck = plan_hybrid_astar(STALL, STALL_GOAL, [], LOADED, OPEN,
                              PlannerConfig(**{**MEASURED, "max_expansions": 1}, shot_cap_m=4.0))
    assert stuck.success and stuck.shot_fallback and stuck.length_m == pytest.approx(loop.length_m)


def test_turn_cycles_are_queued_and_counted_on_the_path():
    # A narrow corridor whose end needs the truck turned round; cycles are offered.
    walls = [Rectangle(0.0, 1.6, 12.0, 0.4, 0.0), Rectangle(0.0, -1.6, 12.0, 0.4, 0.0)]
    bounds = Bounds(-6.0, 6.0, -2.0, 2.0)
    footprint = Footprint(0.6, 0.2, 0.3)
    config = PlannerConfig(curvature_limit_inv_m=0.5, primitive_length_m=0.25, max_expansions=20000,
                           turn_cycles=True, turn_cycle_m=0.5)
    result = plan_hybrid_astar(Pose2D(-1.0, 0.0, 0.0), Pose2D(-1.0, 0.0, math.pi / 2), walls, footprint, bounds, config)
    assert result.cycles_queued > 0
    if result.success:
        # A cycle on the path shows as a gear change between two 0.5 m full-steering halves.
        assert result.cycles_in_path <= gear_changes(result)
    off = plan_hybrid_astar(Pose2D(-1.0, 0.0, 0.0), Pose2D(-1.0, 0.0, math.pi / 2), walls, footprint, bounds,
                            PlannerConfig(curvature_limit_inv_m=0.5, primitive_length_m=0.25, max_expansions=20000))
    assert off.cycles_queued == 0 and off.cycles_in_path == 0


def test_a_cycle_path_length_counts_both_halves():
    # The goal is exactly one cycle away (forward left 0.5 m, reverse right 0.5 m); the root
    # shot is capped out and every analytic try is taken, so the cycle child connects with
    # a zero-length shot (Codex D7 5th P3: the old test never had a cycle on the path).
    k, half = 0.5, 0.5
    goal = _advance(_advance((0.0, 0.0, 0.0), half, k), -half, -k)
    config = PlannerConfig(curvature_limit_inv_m=k, primitive_length_m=0.25, max_expansions=2000,
                           analytic_expansion_interval=1, turn_cycles=True, turn_cycle_m=half, shot_cap_m=0.0)
    result = plan_hybrid_astar(Pose2D(0.0, 0.0, 0.0), Pose2D(*goal), [], Footprint(0.6, 0.2, 0.3),
                               Bounds(-5, 5, -5, 5), config)
    assert result.success and result.cycles_in_path == 1 and result.root_shot == "capped"
    assert result.cycles_generated >= result.cycles_queued >= 1
    assert result.length_m == pytest.approx(2 * half)
    driven = float(np.sum(np.hypot(*np.diff(result.poses[:, :2], axis=0).T)))
    assert driven == pytest.approx(2 * half, rel=0.02)
    assert gear_changes(result) == 1


def test_a_shot_within_the_cap_is_not_hidden_by_a_cheaper_one_over_it():
    # Codex D7 5th P2: the first collision-free shot (10.266 m) is over the cap
    # (d + 4 = 10.116 m) while a 9.850 m Reeds-Shepp shot is under it.
    goal = Pose2D(-3.259706, -5.174994, -0.125052)
    config = PlannerConfig(curvature_limit_inv_m=0.29, primitive_length_m=0.25, max_expansions=1,
                           goal_connection="reeds_shepp", shot_cap_m=4.0)
    result = plan_hybrid_astar(Pose2D(0.0, 0.0, 0.0), goal, [], Footprint(0.6, 0.2, 0.3), OPEN, config)
    cap = math.hypot(goal.x_m, goal.y_m) + 4.0
    assert result.success and not result.shot_fallback and result.root_shot == "taken"
    assert result.length_m <= cap


def test_bad_option_values_are_refused():
    with pytest.raises(ValueError):
        PlannerConfig(goal_connection="dubbins")
    with pytest.raises(ValueError):
        PlannerConfig(shot_cap_m=-1.0)
    with pytest.raises(ValueError):
        PlannerConfig(turn_cycle_m=0.0)
    with pytest.raises(ValueError):
        PlannerConfig(shot_cap_patience=0)
