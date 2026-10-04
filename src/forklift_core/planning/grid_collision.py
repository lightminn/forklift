"""Footprint collision against an occupancy grid (priority-5 plan D3).

docs/plans/2026-10-04-lidar-obstacle-map.md: the planner checks the rotated
footprint rectangle against every occupied cell it overlaps, each cell taken
as the closed axis-aligned square it covers. No point sampling: a 5 cm point
lattice misses a cell the rectangle cuts (Codex P1 counterexample, kept as a
test). A lower bound on the distance to the nearest occupied cell centre
first passes poses that are certainly clear; it never changes an answer, only
skips work. The bound comes from square windows counted on a summed-area
table: no occupied cell within k cells (Chebyshev) means the nearest
occupied centre is at least (k + 1) cells away. numpy only.

Unknown cells are not obstacles here -- the global plan is optimistic about
what nobody has seen; whether the truck may actually enter is the drive
permission's job (plan D4), not the planner's.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from math import ceil, cos, floor, hypot, isfinite, sin, sqrt

import numpy as np
from forklift_core.planning.geometry import Bounds, Footprint, FootprintCollisionChecker, Rectangle


@dataclass(frozen=True)
class OccupancyGrid:
    """Occupied cells of an axis-aligned grid; cell (i, j) covers
    [x0 + i r, x0 + (i + 1) r] x [y0 + j r, y0 + (j + 1) r], closed."""

    origin_x_m: float
    origin_y_m: float
    resolution_m: float
    occupied: np.ndarray  # bool (nx, ny)
    version: int = 0
    _clear_cells: np.ndarray = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        # An independent, read-only copy: the distance cache is computed once,
        # so a caller editing its array afterwards must not change answers
        # (Codex L0 P2).
        occupied = np.array(self.occupied, dtype=bool, copy=True)
        occupied.setflags(write=False)
        if occupied.ndim != 2 or not occupied.size:
            raise ValueError("occupied must be a non-empty 2D array")
        if not (isfinite(self.resolution_m) and self.resolution_m > 0):
            raise ValueError("resolution_m must be positive")
        object.__setattr__(self, "occupied", occupied)
        object.__setattr__(self, "_clear_cells", _clear_radius_cells(occupied))

    @property
    def half_diagonal_m(self) -> float:
        return self.resolution_m * sqrt(2) / 2

    def cell_of(self, x_m: float, y_m: float) -> tuple[int, int]:
        return (
            int(floor((x_m - self.origin_x_m) / self.resolution_m)),
            int(floor((y_m - self.origin_y_m) / self.resolution_m)),
        )

    def centre_distance_lower_m(self, x_m: float, y_m: float) -> float:
        """A lower bound on the distance from the centre of the cell holding
        (x, y) to the nearest occupied cell centre."""
        if not self.occupied.any():
            return float("inf")
        i, j = self.cell_of(x_m, y_m)
        if 0 <= i < self.occupied.shape[0] and 0 <= j < self.occupied.shape[1]:
            return float(self._clear_cells[i, j] + 1) * self.resolution_m
        return self._outside_distance(x_m, y_m)

    def _outside_distance(self, x_m: float, y_m: float) -> float:
        # Distance from a point outside the grid to the grid's box is a lower
        # bound on the distance to any occupied centre inside it.
        nx, ny = self.occupied.shape
        x1 = self.origin_x_m + nx * self.resolution_m
        y1 = self.origin_y_m + ny * self.resolution_m
        dx = max(self.origin_x_m - x_m, 0.0, x_m - x1)
        dy = max(self.origin_y_m - y_m, 0.0, y_m - y1)
        return hypot(dx, dy)


CLEAR_LEVELS_CELLS = (0, 2, 4, 8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64, 80, 96, 128)


def _clear_radius_cells(occupied: np.ndarray) -> np.ndarray:
    """Per cell, the largest listed k with no occupied cell within k cells (Chebyshev); -1 if itself occupied."""
    nx, ny = occupied.shape
    table = np.zeros((nx + 1, ny + 1), dtype=np.int64)
    table[1:, 1:] = np.cumsum(np.cumsum(occupied, axis=0), axis=1)
    i = np.arange(nx)[:, None]
    j = np.arange(ny)[None, :]
    clear = np.full(occupied.shape, -1, dtype=np.int64)
    for k in CLEAR_LEVELS_CELLS:
        i0, i1 = np.clip(i - k, 0, nx), np.clip(i + k + 1, 0, nx)
        j0, j1 = np.clip(j - k, 0, ny), np.clip(j + k + 1, 0, ny)
        count = table[i1, j1] - table[i0, j1] - table[i1, j0] + table[i0, j0]
        empty = count == 0
        if not empty.any():
            break
        clear[empty] = k
    return clear


def rectangle_hits_squares(cx, cy, c, s, half_length, half_width, sq_x, sq_y, half) -> bool:
    """Separating-axis test of one oriented rectangle against many axis-aligned squares."""
    if not len(sq_x):
        return False
    dx = sq_x - cx
    dy = sq_y - cy
    ex = abs(c) * half_length + abs(s) * half_width
    ey = abs(s) * half_length + abs(c) * half_width
    proj = half * (abs(c) + abs(s))
    overlap = (
        (np.abs(dx) <= ex + half)
        & (np.abs(dy) <= ey + half)
        & (np.abs(dx * c + dy * s) <= half_length + proj)
        & (np.abs(-dx * s + dy * c) <= half_width + proj)
    )
    return bool(overlap.any())


class GridFootprintChecker:
    """free(pose, margin) of the footprint against the occupied cells and the bounds.

    The same contract as FootprintCollisionChecker: pose is the rear axle,
    margin inflates every side, contact counts as collision, bounds must
    contain the whole inflated footprint, radius_m is the footprint's
    farthest corner from the axle (the planner's swept-arc bound).
    """

    def __init__(self, grid: OccupancyGrid, footprint: Footprint, bounds: Bounds, *, use_distance_transform=True):
        self.grid = grid
        self.footprint = footprint
        self.bounds = bounds
        self.radius_m = hypot(max(footprint.front_m, footprint.rear_m), footprint.half_width_m)
        self.use_distance_transform = use_distance_transform
        self.exact_checks = 0
        self.skipped_checks = 0

    def reach_m(self, margin_m: float) -> float:
        """R(m): the farthest the inflated footprint reaches from the rear axle."""
        fp = self.footprint
        return hypot(max(fp.front_m, fp.rear_m) + margin_m, fp.half_width_m + margin_m)

    def free(self, pose: Sequence[float], margin_m: float = 0.0) -> bool:
        x, y, yaw = pose
        if not all(isfinite(v) for v in (x, y, yaw, margin_m)) or margin_m < 0:
            raise ValueError("pose and margin must be finite; margin nonnegative")
        c, s = cos(yaw), sin(yaw)
        fp = self.footprint
        offset = (fp.front_m - fp.rear_m) / 2
        cx, cy = x + offset * c, y + offset * s
        half_length = (fp.front_m + fp.rear_m) / 2 + margin_m
        half_width = fp.half_width_m + margin_m
        ex = abs(c) * half_length + abs(s) * half_width
        ey = abs(s) * half_length + abs(c) * half_width
        b = self.bounds
        if cx - ex < b.x_min_m or cx + ex > b.x_max_m or cy - ey < b.y_min_m or cy + ey > b.y_max_m:
            return False
        g = self.grid
        if self.use_distance_transform:
            # The axle may sit anywhere in its cell (half diagonal) and each
            # occupied cell extends a half diagonal past its centre.
            if g.centre_distance_lower_m(x, y) > self.reach_m(margin_m) + 2 * g.half_diagonal_m:
                self.skipped_checks += 1
                return True
        self.exact_checks += 1
        r = g.resolution_m
        nx, ny = g.occupied.shape
        i0 = max(int(floor((cx - ex - g.origin_x_m) / r)) - 1, 0)
        i1 = min(int(ceil((cx + ex - g.origin_x_m) / r)) + 1, nx)
        j0 = max(int(floor((cy - ey - g.origin_y_m) / r)) - 1, 0)
        j1 = min(int(ceil((cy + ey - g.origin_y_m) / r)) + 1, ny)
        if i0 >= i1 or j0 >= j1:
            return True
        ii, jj = np.nonzero(g.occupied[i0:i1, j0:j1])
        if not len(ii):
            return True
        sq_x = g.origin_x_m + (ii + i0 + 0.5) * r
        sq_y = g.origin_y_m + (jj + j0 + 0.5) * r
        return not rectangle_hits_squares(cx, cy, c, s, half_length, half_width, sq_x, sq_y, r / 2)


class CompositeChecker:
    """Rectangles (perception estimates, priors) and the grid, both must be free."""

    def __init__(self, rectangles: Sequence[Rectangle], grid: OccupancyGrid, footprint: Footprint, bounds: Bounds):
        self.rectangles = FootprintCollisionChecker(rectangles, footprint, bounds)
        self.grid = GridFootprintChecker(grid, footprint, bounds)
        self.footprint = footprint
        self.bounds = bounds
        self.radius_m = self.rectangles.radius_m

    def free(self, pose: Sequence[float], margin_m: float = 0.0) -> bool:
        return self.rectangles.free(pose, margin_m) and self.grid.free(pose, margin_m)


def make_checker(obstacles: Sequence[Rectangle], footprint: Footprint, bounds: Bounds, occupancy: OccupancyGrid | None):
    if occupancy is None:
        return FootprintCollisionChecker(obstacles, footprint, bounds)
    return CompositeChecker(obstacles, occupancy, footprint, bounds)


def rasterize(rectangles: Sequence[Rectangle], bounds: Bounds, resolution_m: float) -> OccupancyGrid:
    """Grid of every cell whose square touches a rectangle (tests, truth comparisons).

    Rasterising can only grow an obstacle, never shrink it.
    """
    nx = max(1, ceil((bounds.x_max_m - bounds.x_min_m) / resolution_m))
    ny = max(1, ceil((bounds.y_max_m - bounds.y_min_m) / resolution_m))
    occupied = np.zeros((nx, ny), dtype=bool)
    xs = bounds.x_min_m + (np.arange(nx) + 0.5) * resolution_m
    ys = bounds.y_min_m + (np.arange(ny) + 0.5) * resolution_m
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    half = resolution_m / 2
    for o in rectangles:
        c, s = cos(o.yaw_rad), sin(o.yaw_rad)
        dx, dy = gx - o.x_m, gy - o.y_m
        hl, hw = o.length_m / 2, o.width_m / 2
        ex = abs(c) * hl + abs(s) * hw
        ey = abs(s) * hl + abs(c) * hw
        proj = half * (abs(c) + abs(s))
        occupied |= (
            (np.abs(dx) <= ex + half)
            & (np.abs(dy) <= ey + half)
            & (np.abs(dx * c + dy * s) <= hl + proj)
            & (np.abs(-dx * s + dy * c) <= hw + proj)
        )
    return OccupancyGrid(bounds.x_min_m, bounds.y_min_m, resolution_m, occupied)


__all__ = [
    "CompositeChecker",
    "GridFootprintChecker",
    "OccupancyGrid",
    "make_checker",
    "rasterize",
    "rectangle_hits_squares",
]
