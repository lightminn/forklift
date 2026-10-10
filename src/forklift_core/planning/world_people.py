"""Walkers the traffic manager does not know about, and the trucks' protective field.

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W4). People walk at
PERSON_SPEED between random free points of a WorldMap on a coarse grid that
keeps them off obstacles and pallet places; one stands still while a truck
is within a metre ahead of them, and after GIVE_WAY_S walks off to a new
point on a route that keeps clear of the trucks (otherwise a person waiting
for a truck that waits for the person never moves). A truck stops for anything in its
protective field: ahead in its direction of travel, its stopping distance
plus FIELD_EXTRA_M, the body's width plus FIELD_SIDE_M a side. These are
simple scenario rules (Claude's choices), not a model of people or a
certified safety function.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .fleet_traffic import overlap_matrix
from .world_gen import WorldMap, pallet_box

PERSON_SPEED = 1.2
PERSON_RADIUS = 0.3
FIELD_EXTRA_M = 1.0
FIELD_SIDE_M = 0.3
STOP_BEFORE_M = 0.5
GIVE_WAY_S = 2.0
GIVE_WAY_CLEAR_M = PERSON_RADIUS + 0.2  # grown truck boxes a give-way route avoids
LOOK_AHEAD_M = 1.0  # a truck this close ahead of a person's body stops them


class People:
    """Walkers between random free points; they wait while a truck is close ahead."""

    def __init__(self, world: WorldMap, geometry, count: int, rng, grid_m: float = 1.0):
        self.rng, self.grid = rng, grid_m
        b = world.bounds
        self.origin = np.array([b.x_min_m, b.y_min_m])
        nx = int((b.x_max_m - b.x_min_m) // grid_m)
        ny = int((b.y_max_m - b.y_min_m) // grid_m)
        self.free = np.ones((nx, ny), dtype=bool)
        boxes = [x for _, x in world.obstacles]
        boxes += [pallet_box(s.site, geometry).grow(0.2) for s in world.slots]
        for x in boxes:
            g = x.grow(PERSON_RADIUS + 0.2)
            i0 = max(int((g.x0 - b.x_min_m) // grid_m), 0)
            i1 = min(int(math.ceil((g.x1 - b.x_min_m) / grid_m)), nx)
            j0 = max(int((g.y0 - b.y_min_m) // grid_m), 0)
            j1 = min(int(math.ceil((g.y1 - b.y_min_m) / grid_m)), ny)
            if i0 < i1 and j0 < j1:
                self.free[i0:i1, j0:j1] = False
        self.cells = np.argwhere(self.free)
        self.pos = (
            np.array([self._random_point() for _ in range(count)])
            if count
            else np.zeros((0, 2))
        )
        self.paths: list[list] = [[] for _ in range(count)]
        self.waited_s = np.zeros(count)
        self.walked_m = 0.0

    def _random_point(self):
        c = self.cells[int(self.rng.integers(len(self.cells)))]
        return self.origin + (c + 0.5) * self.grid

    def _without(self, rects: np.ndarray) -> np.ndarray:
        """The free grid less the cells near these oriented rects' bounding boxes."""
        free = self.free.copy()
        nx, ny = free.shape
        for cx, cy, hl, hw, c, s in rects:
            ex = abs(c) * hl + abs(s) * hw + GIVE_WAY_CLEAR_M
            ey = abs(s) * hl + abs(c) * hw + GIVE_WAY_CLEAR_M
            i0 = max(int((cx - ex - self.origin[0]) // self.grid), 0)
            i1 = min(int(math.ceil((cx + ex - self.origin[0]) / self.grid)), nx)
            j0 = max(int((cy - ey - self.origin[1]) // self.grid), 0)
            j1 = min(int(math.ceil((cy + ey - self.origin[1]) / self.grid)), ny)
            if i0 < i1 and j0 < j1:
                free[i0:i1, j0:j1] = False
        return free

    def _route(self, a, b, avoid=None):
        """4-connected breadth-first path on the free grid (less ``avoid``'s cells)."""
        free = self.free if avoid is None or not len(avoid) else self._without(avoid)
        nx, ny = free.shape
        start = tuple(
            np.clip(((a - self.origin) // self.grid).astype(int), 0, [nx - 1, ny - 1])
        )
        goal = tuple(
            np.clip(((b - self.origin) // self.grid).astype(int), 0, [nx - 1, ny - 1])
        )
        prev = {start: None}
        queue = deque([start])
        while queue:
            cur = queue.popleft()
            if cur == goal:
                break
            i, j = cur
            for nb in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if (
                    0 <= nb[0] < nx
                    and 0 <= nb[1] < ny
                    and free[nb]
                    and nb not in prev
                ):
                    prev[nb] = cur
                    queue.append(nb)
        if goal not in prev:
            return []
        cells, cur = [], goal
        while cur is not None:
            cells.append(cur)
            cur = prev[cur]
        return [self.origin + (np.array(c) + 0.5) * self.grid for c in reversed(cells)][
            1:
        ]

    def step(self, dt_s: float, truck_rects: np.ndarray) -> None:
        """Walk dt_s; truck_rects are the trucks' true bodies (oriented rects)."""
        for k in range(len(self.pos)):
            if not self.paths[k]:
                self.paths[k] = self._route(self.pos[k], self._random_point())
                if not self.paths[k]:
                    continue
            d = self.paths[k][0] - self.pos[k]
            dist = float(np.hypot(*d))
            heading = d / dist if dist > 1e-9 else np.array([1.0, 0.0])
            reach = (PERSON_RADIUS + LOOK_AHEAD_M) / 2
            ahead = self.pos[k] + heading * reach
            probe = np.array([[ahead[0], ahead[1], reach, PERSON_RADIUS, heading[0], heading[1]]])
            if len(truck_rects) and overlap_matrix(probe, truck_rects).any():
                self.waited_s[k] += dt_s
                if self.waited_s[k] > GIVE_WAY_S:
                    self.waited_s[k] = 0.0
                    self.paths[k] = self._route(self.pos[k], self._random_point(), truck_rects)
                continue
            self.waited_s[k] = 0.0
            move = min(PERSON_SPEED * dt_s, dist)
            self.pos[k] = self.pos[k] + heading * move
            self.walked_m += move
            if move >= dist - 1e-9:
                self.paths[k].pop(0)


def protective_block_m(
    pose, footprint, direction: int, speed_mps: float, accel_mps2: float, points
) -> float | None:
    """Distance the truck may still drive before something in its field, or None.

    ``points`` are (N, 2) world positions of things the traffic manager does
    not know: people (CPU), or LiDAR returns off the known map (Isaac). The
    field reaches the stopping distance plus FIELD_EXTRA_M past the body's
    end in the direction of travel, FIELD_SIDE_M wider than the body a side.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(points):
        return None
    x, y, yaw = (float(v) for v in pose)
    reach = speed_mps**2 / (2 * accel_mps2) + FIELD_EXTRA_M
    c, s = math.cos(yaw), math.sin(yaw)
    nose = footprint.front_m if direction > 0 else footprint.rear_m
    rel = points - np.array([x, y])
    along = (rel[:, 0] * c + rel[:, 1] * s) * direction
    side = np.abs(-rel[:, 0] * s + rel[:, 1] * c)
    inside = (
        (along > 0)
        & (along < nose + reach + PERSON_RADIUS)
        & (side < footprint.half_width_m + FIELD_SIDE_M + PERSON_RADIUS)
    )
    if not inside.any():
        return None
    return float(along[inside].min() - nose - PERSON_RADIUS - STOP_BEFORE_M)
