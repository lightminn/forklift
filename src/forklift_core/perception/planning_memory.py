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
  forgetting an obstacle beside it (Codex review P1); the retraction is judged on
  a 1 cm endpoint record, never on the 5 cm cell centres, so an endpoint outside
  the rectangle is never retracted (Codex review P1). Positions are fixed in the
  map frame by the estimate of their observation time; a later correction does
  not move them (see the plan for the limit of that approximation).
* SLAM static layer: the latest slam_toolbox /map (1.05 m plane, values 100
  occupied / 0 free / -1 unknown). Each occupied source square marks every
  planning cell it overlaps (raw cells). Each source square is dated by the
  map's own stamp when it turns occupied -- tracked per source square by its
  map-frame centre, since two source squares can project onto the same planning
  cells (Codex review P1) -- and a raw cell carries its newest square's date; the layer is the raw cells grown by one, each grown cell
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
FINE_M = 0.01  # the endpoint record a retraction is judged on
_KEY_OFF = 1 << 30


def _key(i, j) -> np.ndarray:
    return (np.asarray(i, dtype=np.int64) + _KEY_OFF) * (1 << 31) + (np.asarray(j, dtype=np.int64) + _KEY_OFF)


def _unkey(k: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return k // (1 << 31) - _KEY_OFF, k % (1 << 31) - _KEY_OFF


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
    fine_key: np.ndarray = field(init=False)  # sorted 1 cm endpoint cells (x, y keyed)
    fine_stamp: np.ndarray = field(init=False)  # their newest endpoint time
    slam_src_key: np.ndarray = field(init=False)  # sorted occupied source squares of the latest map
    slam_src_since: np.ndarray = field(init=False)  # their dates
    last_scan_stamp: float = field(init=False, default=-math.inf)

    def __post_init__(self) -> None:
        self.ox = math.floor(self.x_min_m / self.resolution_m) * self.resolution_m
        self.oy = math.floor(self.y_min_m / self.resolution_m) * self.resolution_m
        nx = int(math.ceil((self.x_max_m - self.ox) / self.resolution_m))
        ny = int(math.ceil((self.y_max_m - self.oy) / self.resolution_m))
        self.endpoint = np.full((nx, ny), -np.inf)
        self.last_clear = np.full((nx, ny), -np.inf)
        self.slam_since = np.full((nx, ny), np.inf)
        self.fine_key = np.zeros(0, dtype=np.int64)
        self.fine_stamp = np.zeros(0)
        self.slam_src_key = np.zeros(0, dtype=np.int64)
        self.slam_src_since = np.zeros(0)
        self._pending, self._pending_n = [], 0
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
        xs, ys = np.asarray(xs, dtype=float)[ok], np.asarray(ys, dtype=float)[ok]
        self._pending.append((_key(np.floor(xs / FINE_M), np.floor(ys / FINE_M)), np.full(len(xs), float(stamp_s))))
        self._pending_n += len(xs)
        if self._pending_n > 200_000:
            self._compact()
        self.revision += 1

    def _compact(self) -> None:
        """Merge the scans taken since into the sorted 1 cm record."""
        if not self._pending:
            return
        keys = np.concatenate([self.fine_key] + [k for k, _ in self._pending])
        stamps = np.concatenate([self.fine_stamp] + [t for _, t in self._pending])
        self._pending, self._pending_n = [], 0
        self.fine_key, inverse = np.unique(keys, return_inverse=True)
        self.fine_stamp = np.full(len(self.fine_key), -np.inf)
        np.maximum.at(self.fine_stamp, inverse.ravel(), stamps)

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
        raw_since = np.full(self.shape, -np.inf)
        rows, cols = np.nonzero(np.asarray(data) == SLAM_OCCUPIED)
        src_key = np.zeros(0, dtype=np.int64)
        src_since = np.zeros(0)
        if len(rows):
            c, s = math.cos(yaw_rad), math.sin(yaw_rad)
            # A source square is the same square in the next map by its
            # map-frame centre (the map grows, its origin moves).
            u, v = (cols + 0.5) * resolution_m, (rows + 0.5) * resolution_m
            half = resolution_m / 2
            src_key = _key(np.round((origin_xy[0] + u * c - v * s) / half),
                           np.round((origin_xy[1] + u * s + v * c) / half))
            src_since = np.full(len(src_key), float(stamp_s))
            if len(self.slam_src_key):
                pos = np.minimum(np.searchsorted(self.slam_src_key, src_key), len(self.slam_src_key) - 1)
                known = self.slam_src_key[pos] == src_key
                src_since[known] = self.slam_src_since[pos[known]]
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
            for a0, a1, b0, b1, since in zip(i0, i1, j0, j1, src_since):
                a0, a1, b0, b1 = max(int(a0), 0), min(int(a1) + 1, nx), max(int(b0), 0), min(int(b1) + 1, ny)
                if a0 < a1 and b0 < b1:  # a square wholly outside the hall marks nothing (Codex review P1)
                    np.maximum(raw_since[a0:a1, b0:b1], since, out=raw_since[a0:a1, b0:b1])
        order = np.argsort(src_key, kind="stable")
        self.slam_src_key, self.slam_src_since = src_key[order], src_since[order]
        mask = np.isfinite(raw_since)
        self.slam_since = np.where(mask, raw_since, np.inf)
        self.slam = mask
        self.slam_info = {**info, "stamp_s": float(stamp_s)}
        self.revision += 1

    def retract_endpoints(self, x: float, y: float, length: float, width: float, yaw: float, grow_m: float) -> int:
        """Forget the endpoints inside a rectangle grown by grow_m (the pallet
        just lifted: it is the truck's now, by the recognised estimate). Only
        endpoints go, and only those whose whole 1 cm record cell lies inside:
        an endpoint outside the rectangle is never retracted, so an obstacle
        beside the pallet keeps its own (Codex review P1). Returns the number of
        1 cm endpoint cells retracted."""
        self._compact()
        if len(self.fine_key) == 0:
            return 0
        fi, fj = _unkey(self.fine_key)
        c, s = math.cos(yaw), math.sin(yaw)
        inside = np.ones(len(fi), dtype=bool)
        for di in (0, 1):
            for dj in (0, 1):
                dx, dy = (fi + di) * FINE_M - x, (fj + dj) * FINE_M - y
                inside &= (np.abs(dx * c + dy * s) <= length / 2 + grow_m) & \
                    (np.abs(-dx * s + dy * c) <= width / 2 + grow_m)
        keep = ~inside
        self.fine_key, self.fine_stamp = self.fine_key[keep], self.fine_stamp[keep]
        # The planning cells re-derived from what stays.
        self.endpoint = np.full(self.shape, -np.inf)
        if keep.any():
            ci, cj = self.cell((fi[keep] + 0.5) * FINE_M, (fj[keep] + 0.5) * FINE_M)
            nx, ny = self.shape
            ok = (ci >= 0) & (ci < nx) & (cj >= 0) & (cj < ny)
            np.maximum.at(self.endpoint, (ci[ok], cj[ok]), self.fine_stamp[ok])
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
