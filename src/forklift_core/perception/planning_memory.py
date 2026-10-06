"""Planning-only obstacle memory and SLAM static layer (priority-5 plan D2/D3
delta, user decision 2026-10-06).

The LiDAR obstacle grid keeps a hit for occupied_max_age_s (0.3 s) and the
planner sees nothing else, so an obstacle seen on the way out is forgotten on
the way back and planned through (L3c N2: a return plan within 0.4 m of six
stacks the grid no longer held). This module gives the PLANNER -- never the
drive permission, which stays on live evidence -- two longer sources:

* observation memory: per cell of a hall-wide map-frame grid, the time of the
  last beam endpoint that fell in it and of the last fresh FREE observation. A
  cell is occupied while the newest endpoint within hit_radius_m of it is newer
  than its last clear. Endpoints come only from the raw scans, each processed
  once with its own stamp, so a stale hit a snapshot re-raises after a clear is
  never registered again (Codex design review P1). Keeping the endpoints apart
  from their spread lets a lifted pallet's own endpoints be retracted without
  forgetting an obstacle beside it (Codex review P1). Positions are fixed in the
  map frame by the estimate of their observation time; a later correction does
  not move them (see the plan for the limit of that approximation).
* SLAM static layer: the latest slam_toolbox /map (1.05 m plane, values 100
  occupied / 0 free / -1 unknown). Each occupied source square marks every
  planning cell it overlaps (raw cells), dated by the map's own stamp when it
  turns occupied; the layer is the raw cells grown by one, each grown cell
  dated by its newest raw neighbour. A cell is refuted by a live clear newer
  than that date (a removed tall object), so a republished old map, or one read
  late, never overturns a newer clear, and a new occupancy is never hidden by an
  older one (Codex review P1).

numpy only; no ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

SLAM_OCCUPIED = 100  # nav_msgs/OccupancyGrid as slam_toolbox publishes it


def _shift(a: np.ndarray, di: int, dj: int, fill) -> np.ndarray:
    """a moved by (di, dj) cells, the vacated edge filled: out[i, j] = a[i - di, j - dj]."""
    out = np.full_like(a, fill)
    nx, ny = a.shape
    out[max(di, 0):nx + min(di, 0), max(dj, 0):ny + min(dj, 0)] = \
        a[max(-di, 0):nx + min(-di, 0), max(-dj, 0):ny + min(-dj, 0)]
    return out


@dataclass
class PlanningMemory:
    x_min_m: float
    y_min_m: float
    x_max_m: float
    y_max_m: float
    resolution_m: float
    hit_radius_m: float = 0.1  # an endpoint marks the cells within the sensor bound around it
    endpoint: np.ndarray = field(init=False)
    last_clear: np.ndarray = field(init=False)
    slam: np.ndarray | None = field(init=False, default=None)  # raw occupied cells of the latest map
    slam_since: np.ndarray = field(init=False)  # per raw cell: the map stamp it turned occupied
    slam_info: dict | None = field(init=False, default=None)
    revision: int = field(init=False, default=0)
    last_scan_stamp: float = field(init=False, default=-math.inf)

    def __post_init__(self) -> None:
        self.ox = math.floor(self.x_min_m / self.resolution_m) * self.resolution_m
        self.oy = math.floor(self.y_min_m / self.resolution_m) * self.resolution_m
        nx = int(math.ceil((self.x_max_m - self.ox) / self.resolution_m))
        ny = int(math.ceil((self.y_max_m - self.oy) / self.resolution_m))
        self.endpoint = np.full((nx, ny), -np.inf)
        self.last_clear = np.full((nx, ny), -np.inf)
        self.slam_since = np.full((nx, ny), np.inf)
        r = self.hit_radius_m / self.resolution_m
        k = int(math.ceil(r))
        self._disk = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1) if math.hypot(i, j) <= r + 1e-9]

    @property
    def shape(self) -> tuple[int, int]:
        return self.endpoint.shape

    @property
    def last_hit(self) -> np.ndarray:
        """Per cell, the newest endpoint within hit_radius_m."""
        out = np.full(self.shape, -np.inf)
        for di, dj in self._disk:
            np.maximum(out, _shift(self.endpoint, di, dj, -np.inf), out=out)
        return out

    def cell(self, x, y):
        return (np.floor((np.asarray(x) - self.ox) / self.resolution_m).astype(int),
                np.floor((np.asarray(y) - self.oy) / self.resolution_m).astype(int))

    def _centres(self):
        nx, ny = self.shape
        cx = self.ox + (np.arange(nx) + 0.5) * self.resolution_m
        cy = self.oy + (np.arange(ny) + 0.5) * self.resolution_m
        return np.meshgrid(cx, cy, indexing="ij")

    def add_hits(self, xs, ys, stamp_s: float) -> None:
        """Beam endpoints (map frame) of one scan measured at stamp_s."""
        if len(xs) == 0:
            return
        ci, cj = self.cell(xs, ys)
        nx, ny = self.shape
        ok = (ci >= 0) & (ci < nx) & (cj >= 0) & (cj < ny)
        np.maximum.at(self.endpoint, (ci[ok], cj[ok]), stamp_s)
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
                 stamp_s: float, **info) -> None:
        """slam_toolbox map data (height, width) stamped stamp_s -> raw occupied
        planning cells (every cell an occupied source square overlaps); a cell
        that turns occupied in this map is dated stamp_s, one already occupied
        keeps its date."""
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
            for a0, a1, b0, b1 in zip(i0, i1, j0, j1):
                a0, a1, b0, b1 = max(int(a0), 0), min(int(a1) + 1, nx), max(int(b0), 0), min(int(b1) + 1, ny)
                if a0 < a1 and b0 < b1:  # a square wholly outside the hall marks nothing (Codex review P1)
                    mask[a0:a1, b0:b1] = True
        previous = self.slam if self.slam is not None else np.zeros(self.shape, dtype=bool)
        self.slam_since[mask & ~previous] = float(stamp_s)
        self.slam_since[~mask] = np.inf
        self.slam = mask
        self.slam_info = {**info, "stamp_s": float(stamp_s)}
        self.revision += 1

    def retract_endpoints(self, x: float, y: float, length: float, width: float, yaw: float, grow_m: float) -> int:
        """Forget the endpoints inside a rectangle grown by grow_m (the pallet
        just lifted: it is the truck's now, by the recognised estimate). Only
        endpoints go; an obstacle beside the pallet keeps its own (Codex review P1)."""
        gx, gy = self._centres()
        c, s = math.cos(yaw), math.sin(yaw)
        u = (gx - x) * c + (gy - y) * s
        v = -(gx - x) * s + (gy - y) * c
        inside = (np.abs(u) <= length / 2 + grow_m) & (np.abs(v) <= width / 2 + grow_m) & np.isfinite(self.endpoint)
        self.endpoint[inside] = -np.inf
        self.revision += 1
        return int(inside.sum())

    def occupied(self, *, use_slam: bool = True) -> np.ndarray:
        """Observation memory (newest endpoint in reach newer than the last clear),
        plus the SLAM layer where no clear is newer than the occupancy it carries."""
        out = self.last_hit > self.last_clear
        if use_slam and self.slam is not None:
            dated = np.where(self.slam, self.slam_since, -np.inf)
            grown_since = dated.copy()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    np.maximum(grown_since, _shift(dated, di, dj, -np.inf), out=grown_since)
            grown = np.isfinite(grown_since)
            out |= grown & ~(self.last_clear > grown_since)
        return out


__all__ = ["PlanningMemory", "SLAM_OCCUPIED"]
