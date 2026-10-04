"""Occupancy-grid footprint checks (priority-5 plan D3); synthetic geometry only."""

import math

import numpy as np

from forklift_core.planning import Bounds, Footprint, FootprintCollisionChecker, Rectangle
from forklift_core.planning.grid_collision import (
    GridFootprintChecker,
    OccupancyGrid,
    rasterize,
)

UNLOADED = Footprint(1.29, 0.17, 0.36)
BOUNDS = Bounds(-4, 4, -4, 4)


def single_cell_grid(cx, cy, r=0.05, origin=-1.0, n=60):
    occupied = np.zeros((n, n), dtype=bool)
    occupied[int(math.floor((cx - origin) / r)), int(math.floor((cy - origin) / r))] = True
    return OccupancyGrid(origin, origin, r, occupied)


def test_the_codex_counterexample_cell_is_a_collision():
    # SAT says the footprint cuts the cell; a 5 cm point lattice put 0 of 496 points in it.
    grid = single_cell_grid(-0.225, 0.125)
    i, j = grid.cell_of(-0.225, 0.125)
    assert math.isclose(grid.origin_x_m + (i + 0.5) * 0.05, -0.225)
    assert math.isclose(grid.origin_y_m + (j + 0.5) * 0.05, 0.125)
    for use_dt in (True, False):
        checker = GridFootprintChecker(grid, UNLOADED, BOUNDS, use_distance_transform=use_dt)
        assert not checker.free((0.013, 0.017, math.pi / 4))


def test_margin_and_contact_follow_the_rectangle_checker_contract():
    # The cell [0.45, 0.50] in y: its near face is 0.09 m beside the footprint's left side.
    grid = single_cell_grid(0.525, 0.475)
    checker = GridFootprintChecker(grid, UNLOADED, BOUNDS)
    assert checker.free((0.0, 0.0, 0.0), 0.089)
    assert not checker.free((0.0, 0.0, 0.0), 0.091)
    assert not GridFootprintChecker(grid, UNLOADED, Bounds(-0.5, 2, -0.3, 0.3)).free((0, 0, 0))


def test_a_rasterised_map_never_lets_through_what_the_rectangles_block():
    rng = np.random.default_rng(5)
    for _ in range(20):
        obstacles = [
            Rectangle(*rng.uniform(-3, 3, 2), *rng.uniform(0.1, 0.9, 2), rng.uniform(-math.pi, math.pi))
            for _ in range(6)
        ]
        grid = rasterize(obstacles, BOUNDS, 0.05)
        exact = FootprintCollisionChecker(obstacles, UNLOADED, BOUNDS)
        on = GridFootprintChecker(grid, UNLOADED, BOUNDS)
        off = GridFootprintChecker(grid, UNLOADED, BOUNDS, use_distance_transform=False)
        for _ in range(150):
            pose = (*rng.uniform(-2.5, 2.5, 2), rng.uniform(-math.pi, math.pi))
            margin = float(rng.choice([0.0, 0.05, 0.1, 0.3, 0.6]))
            a = on.free(pose, margin)
            assert a == off.free(pose, margin)  # the distance transform only skips work
            if not exact.free(pose, margin):
                assert not a


def test_the_distance_transform_skips_far_poses_and_answers_the_same_at_its_edge():
    grid = single_cell_grid(0.0, 0.0, n=200, origin=-5.0)
    checker = GridFootprintChecker(grid, UNLOADED, Bounds(-6, 6, -6, 6))
    reference = GridFootprintChecker(grid, UNLOADED, Bounds(-6, 6, -6, 6), use_distance_transform=False)
    assert checker.free((3.0, 3.0, 0.3))
    assert checker.skipped_checks == 1
    reach = checker.reach_m(0.2)
    for d in np.linspace(reach - 0.2, reach + 0.2, 41):
        for yaw in np.linspace(-math.pi, math.pi, 13):
            for heading in np.linspace(0, 2 * math.pi, 8, endpoint=False):
                pose = (d * math.cos(heading), d * math.sin(heading), yaw)
                assert checker.free(pose, 0.2) == reference.free(pose, 0.2)
