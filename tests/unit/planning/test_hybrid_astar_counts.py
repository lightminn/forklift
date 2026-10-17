"""Hybrid A* search diagnostics (priority-5 plan D7d): counted, never acted on."""

import numpy as np

from forklift_core.planning import Bounds, Footprint, Pose2D, Rectangle
from forklift_core.planning import pallet_mission
from forklift_core.planning.hybrid_astar import PlannerConfig, plan_hybrid_astar

# A wall across the line in a hall too small for a Dubins loop: only the search gets round.
BOUNDS = Bounds(-3.5, 3.5, -2.5, 2.5)
FOOTPRINT = Footprint(0.6, 0.2, 0.3)
WALL = [Rectangle(0.0, -0.85, 0.4, 3.3, 0.0)]
CONFIG = PlannerConfig(primitive_length_m=0.25, max_expansions=20000)


def test_pruned_and_superseded_nodes_are_counted():
    result = plan_hybrid_astar(Pose2D(-2.5, 0.0, 0.0), Pose2D(2.5, 0.0, 0.0), WALL, FOOTPRINT, BOUNDS, CONFIG)
    assert result.success and result.expanded_nodes > 1
    # This input drops children on keys already reached cheaper and pops entries that a
    # cheaper node replaced (Codex D7 2nd P3: ">= 0" let an always-zero counter pass).
    assert result.pruned_children > 0 and result.superseded_pops > 0


def test_a_failure_keeps_its_counts():
    tight = PlannerConfig(primitive_length_m=0.25, max_expansions=50)
    result = plan_hybrid_astar(Pose2D(-2.5, 0.0, 0.0), Pose2D(2.5, 0.0, 0.0), WALL, FOOTPRINT, BOUNDS, tight)
    assert result.status == "expansion_limit"
    assert result.pruned_children > 0


def test_the_counts_survive_an_appended_straight_and_the_ladder_records_them():
    first = plan_hybrid_astar(Pose2D(-2.5, 0.0, 0.0), Pose2D(2.5, 0.0, 0.0), WALL, FOOTPRINT, BOUNDS, CONFIG)
    joined = pallet_mission._append_straight(first, first)
    # Codex D7 2nd P2: _append_straight built a fresh result without them.
    assert joined.pruned_children == 2 * first.pruned_children
    assert joined.superseded_pops == 2 * first.superseded_pops
    searched = pallet_mission._search(Pose2D(-2.5, 0.0, 0.0), Pose2D(2.5, 0.0, 0.0), WALL, FOOTPRINT, BOUNDS, CONFIG)
    entry = searched.search_attempts[-1]
    assert len(entry) == 10 and entry[8] == searched.pruned_children and entry[9] == searched.superseded_pops
    assert np.array_equal(searched.poses, first.poses)  # counting changes nothing in the plan
