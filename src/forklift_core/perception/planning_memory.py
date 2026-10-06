"""Planning-only obstacle memory and SLAM static layer (priority-5 plan D2/D3
delta, user decision 2026-10-06).

The LiDAR obstacle grid keeps a hit for occupied_max_age_s (0.3 s) and the
planner sees nothing else, so an obstacle seen on the way out is forgotten on
the way back and planned through (L3c N2: a return plan within 0.4 m of six
stacks the grid no longer held). This module gives the PLANNER -- never the
drive permission, which stays on live evidence -- two longer sources:

* observation memory: per cell of a hall-wide map-frame grid, the time of the
  last hit and of the last fresh FREE observation. A cell is occupied while its
  last hit is newer than its last clear. Hits come only from the raw scans,
  each processed once with its own stamp, so a stale hit a snapshot re-raises
  after a clear is never registered again (Codex design review P1). Positions
  are fixed in the map frame by the estimate of their observation time; a
  later correction does not move them (an approximation the planning clearance
  absorbs; the permission guards every tick).
* SLAM static layer: the latest slam_toolbox /map (1.05 m plane, values 100
  occupied / 0 free / -1 unknown), each occupied source cell marking every
  planning cell it overlaps, grown by one cell. A SLAM cell is refuted where
  the live evidence's latest word is a clear (a removed tall object), so an
  old map republished never blocks a cell seen free (Codex design review P1).

numpy only; no ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

SLAM_OCCUPIED = 100  # nav_msgs/OccupancyGrid as slam_toolbox publishes it


@dataclass
class PlanningMemory:
    x_min_m: float
    y_min_m: float
    x_max_m: float
    y_max_m: float
    resolution_m: float
    hit_radius_m: float = 0.1  # a hit marks the cells within the sensor bound around it
    last_hit: np.ndarray = field(init=False)
    last_clear: np.ndarray = field(init=False)
    slam: np.ndarray | None = field(init=False, default=None)
    slam_since: np.ndarray = field(init=False)
    slam_info: dict | None = field(init=False, default=None)
    revision: int = field(init=False, default=0)
    last_scan_stamp: float = field(init=False, default=-math.inf)

    def __post_init__(self) -> None:
        self.ox = math.floor(self.x_min_m / self.resolution_m) * self.resolution_m
        self.oy = math.floor(self.y_min_m / self.resolution_m) * self.resolution_m
        nx = int(math.ceil((self.x_max_m - self.ox) / self.resolution_m))
        ny = int(math.ceil((self.y_max_m - self.oy) / self.resolution_m))
        self.last_hit = np.full((nx, ny), -np.inf)
        self.last_clear = np.full((nx, ny), -np.inf)
        self.slam_since = np.full((nx, ny), np.inf)
        r = self.hit_radius_m / self.resolution_m
        k = int(math.ceil(r))
        self._disk = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1) if math.hypot(i, j) <= r + 1e-9]

    @property
    def shape(self) -> tuple[int, int]:
        return self.last_hit.shape

    def cell(self, x, y):
        return (np.floor((np.asarray(x) - self.ox) / self.resolution_m).astype(int),
                np.floor((np.asarray(y) - self.oy) / self.resolution_m).astype(int))

    def add_hits(self, xs, ys, stamp_s: float) -> None:
        """Beam endpoints (map frame) of one scan measured at stamp_s."""
        if len(xs) == 0:
            return
        ci, cj = self.cell(xs, ys)
        nx, ny = self.shape
        for di, dj in self._disk:
            a, b = ci + di, cj + dj
            ok = (a >= 0) & (a < nx) & (b >= 0) & (b < ny)
            np.maximum.at(self.last_hit, (a[ok], b[ok]), stamp_s)
        self.revision += 1

    def add_clears(self, xs, ys, stamps) -> None:
        """Cells observed FREE (centres in the map frame) with their observation times."""
        if len(xs) == 0:
            return
        ci, cj = self.cell(xs, ys)
        nx, ny = self.shape
        ok = (ci >= 0) & (ci < nx) & (cj >= 0) & (cj < ny)
        np.maximum.at(self.last_clear, (ci[ok], cj[ok]), np.asarray(stamps, dtype=float)[ok])
        self.revision += 1

    def set_slam(self, data: np.ndarray, origin_xy, resolution_m: float, yaw_rad: float = 0.0, *,
                 received_s: float = 0.0, **info) -> None:
        """slam_toolbox map data (height, width) -> occupied planning cells: every
        planning cell an occupied source square overlaps, grown by one cell. A
        cell that turns occupied in this map dates from received_s; one already
        occupied keeps its date, so a republished old map is no new evidence."""
        mask = np.zeros(self.shape, dtype=bool)
        rows, cols = np.nonzero(np.asarray(data) == SLAM_OCCUPIED)
        if len(rows):
            c, s = math.cos(yaw_rad), math.sin(yaw_rad)
            corners_x, corners_y = [], []
            for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
                u = (cols + du) * resolution_m
                v = (rows + dv) * resolution_m
                corners_x.append(origin_xy[0] + u * c - v * s)
                corners_y.append(origin_xy[1] + u * s + v * c)
            cx, cy = np.array(corners_x), np.array(corners_y)
            i0, j0 = self.cell(cx.min(axis=0) + 1e-9, cy.min(axis=0) + 1e-9)
            i1, j1 = self.cell(cx.max(axis=0) - 1e-9, cy.max(axis=0) - 1e-9)
            nx, ny = self.shape
            for a0, a1, b0, b1 in zip(i0 - 1, i1 + 1, j0 - 1, j1 + 1):  # grown by one cell
                a0, a1, b0, b1 = max(int(a0), 0), min(int(a1) + 1, nx), max(int(b0), 0), min(int(b1) + 1, ny)
                if a0 < a1 and b0 < b1:  # a square wholly outside the hall marks nothing (Codex review P1)
                    mask[a0:a1, b0:b1] = True
        previous = self.slam if self.slam is not None else np.zeros(self.shape, dtype=bool)
        self.slam_since[mask & ~previous] = float(received_s)
        self.slam_since[~mask] = np.inf
        self.slam = mask
        self.slam_info = {**info, "received_s": float(received_s)}
        self.revision += 1

    def retract_rect(self, x: float, y: float, length: float, width: float, yaw: float, grow_m: float) -> int:
        """Forget the hits inside a rectangle grown by grow_m (the pallet just
        lifted: it is the truck's now, by the recognised estimate -- Codex review
        P1: its old hits around the loaded outline kept the start blocked)."""
        nx, ny = self.shape
        cx = self.ox + (np.arange(nx) + 0.5) * self.resolution_m
        cy = self.oy + (np.arange(ny) + 0.5) * self.resolution_m
        gx, gy = np.meshgrid(cx, cy, indexing="ij")
        c, s = math.cos(yaw), math.sin(yaw)
        u = (gx - x) * c + (gy - y) * s
        v = -(gx - x) * s + (gy - y) * c
        inside = (np.abs(u) <= length / 2 + grow_m) & (np.abs(v) <= width / 2 + grow_m) & np.isfinite(self.last_hit)
        self.last_hit[inside] = -np.inf
        self.revision += 1
        return int(inside.sum())

    def occupied(self, *, use_slam: bool = True) -> np.ndarray:
        """Observation memory (last hit newer than last clear), plus the SLAM layer
        where the live evidence's latest word is not a clear."""
        out = self.last_hit > self.last_clear
        if use_slam and self.slam is not None:
            # Refuted only by a clear seen after the cell turned occupied in the
            # map: an old clear never hides a new tall obstacle (Codex review P1).
            out |= self.slam & ~(self.last_clear > self.slam_since)
        return out


__all__ = ["PlanningMemory", "SLAM_OCCUPIED"]
