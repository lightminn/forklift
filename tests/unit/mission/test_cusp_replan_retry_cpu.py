"""Plan 2026-11-17-goal-band-and-cusp-retry B on CPU with synthetic occupancy: a refused candidate's occupied cells as
rectangles (unknown cells never), the planner detouring around them, and no detour when they close the way."""

import numpy as np

from forklift_core.mission.permit_stop import candidate_blocked_rectangles
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, GridSnapshot
from forklift_core.planning.geometry import Bounds, Footprint, FootprintCollisionChecker, Pose2D
from forklift_core.planning.hybrid_astar import PlannerConfig, plan_hybrid_astar

RES = 0.05


def snapshot(occupied, unknown=(), shape=(120, 60), origin=(0.0, 0.0)):
    state = np.full(shape, FREE, dtype=np.uint8)
    for i, j in unknown:
        state[i, j] = UNKNOWN
    for i, j in occupied:
        state[i, j] = OCCUPIED
    return GridSnapshot(state, np.zeros(shape), 0.0, (0.0, 0.0, 0.0), 0, 1, {}, origin[0], origin[1], RES)


def blob(i0, i1, j0, j1):
    return [(i, j) for i in range(i0, i1) for j in range(j0, j1)]


def covers(rects, x, y):
    return any(abs(x - r.x_m) <= r.length_m / 2 and abs(y - r.y_m) <= r.width_m / 2 for r in rects)


def test_only_the_occupied_cells_of_the_candidate_check_become_rectangles():
    occ = blob(60, 64, 28, 32)  # x 3.00-3.20, y 1.40-1.60
    unk = blob(40, 44, 28, 32)  # x 2.00-2.20
    snap = snapshot(occ, unk)
    cells = {"walk": [[61, 29], [41, 29]], "ahead": [[62, 30]], "zone": [[42, 30]]}
    rects = candidate_blocked_rectangles(snap, cells)
    assert rects and covers(rects, 3.1, 1.5)
    assert not covers(rects, 2.1, 1.5)  # the unknown cells are never obstacles
    assert candidate_blocked_rectangles(snap, {}) == [] and candidate_blocked_rectangles(None, cells) == []
    assert candidate_blocked_rectangles(snap, {"walk": [[41, 29]]}) == []  # only unknown seeds: nothing


FOOT = Footprint(0.4, 0.1, 0.15)
CFG = PlannerConfig(curvature_limit_inv_m=0.5, primitive_length_m=0.25, max_expansions=20000)
BOUNDS = Bounds(0.0, 6.0, 0.0, 3.0)


def plan(rects):
    return plan_hybrid_astar(Pose2D(0.5, 1.5, 0.0), Pose2D(5.5, 1.5, 0.0), list(rects), FOOT, BOUNDS, CFG)


def test_the_blocked_cells_make_the_planner_go_round_them():
    snap = snapshot(blob(58, 66, 26, 34))  # an occupied box on the straight line, x 2.9-3.3, y 1.3-1.7
    straight = plan([])
    assert straight.success
    rects = candidate_blocked_rectangles(snap, {"walk": [[60, 30]]})
    checker = FootprintCollisionChecker(rects, FOOT, BOUNDS)
    assert not all(checker.free(tuple(p), 0.0) for p in straight.poses)  # the scene-only plan crosses them
    detour = plan(rects)
    assert detour.success and all(checker.free(tuple(p), 0.0) for p in detour.poses)


def test_no_detour_when_the_blocked_cells_close_the_way():
    snap = snapshot(blob(58, 62, 0, 60))  # occupied across the whole width at x 2.9-3.1
    rects = candidate_blocked_rectangles(snap, {"walk": [[60, 30]]}, grow_m=3.0)
    assert not plan(rects).success  # the device then refuses as before (retry_<status>)
