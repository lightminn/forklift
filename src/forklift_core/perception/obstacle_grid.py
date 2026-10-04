"""Rolling LiDAR obstacle grid (priority-5 plan D2, docs/plans/2026-10-04-lidar-obstacle-map.md).

Every obstacle scan is kept raw with the wheel-odometry rear-axle pose of its
measurement instant. A snapshot places all of them with ONE map <- odom
correction -- the one control uses now -- so scans placed under different
SLAM corrections never mix; what is left between an old scan and the present
is the odometry drift over its age, which P0a measured (e(age), psi(age)).

Cell states, newest observation wins (scans applied oldest first, within a
scan clearing before marking):
  OCCUPIED  a hit, inflated by r = e(age) + 2 rho sin(psi(age)/2) + sensor
            bound (rho: rear axle at the scan -> point), and then by the cell
            quantisation (every cell whose square can come within r of a point
            anywhere in the hit's cell); kept for occupied_max_age_s.
  FREE      a cell a beam crossed, only from scans no older than
            free_max_age_s whose r stays within free_r_cap_m, shrunk by that
            r at every edge to non-free cells of the same scan. An older scan
            that saw through a cell still clears older marks there (to
            UNKNOWN): a removed box does not come back when the scan that saw
            it gone ages past free_max_age_s.
  UNKNOWN   everything else, including cells only an older scan saw free.
The planner reads OCCUPIED (unknown is free to it); the drive permission
needs FREE (plan D4).

Beam rules (REP-117 ranges): +inf clears to max_clear_m; a finite hit beyond
max_mark_m clears only to max_mark_m and marks nothing; beams flagged as
self hits (own body, forks, carried pallet at measurement) neither mark nor
clear; -inf and hits inside range_min_m that the self mask does not explain
mark the whole near sector [0, range_min_m] occupied (plan D2, Codex v2 P1).
numpy only.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import ceil, cos, floor, hypot, sin, sqrt

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


@dataclass(frozen=True)
class AgeErrorTable:
    """Odometry error bound by observation age (P0a): cumulative maxima, step-up between points."""

    ages_s: tuple[float, ...]
    position_m: tuple[float, ...]
    yaw_rad: tuple[float, ...]

    def __post_init__(self) -> None:
        if not (len(self.ages_s) == len(self.position_m) == len(self.yaw_rad)) or not self.ages_s:
            raise ValueError("ages, position and yaw must be equally long and non-empty")
        if any(b <= a for a, b in zip(self.ages_s, self.ages_s[1:])):
            raise ValueError("ages must increase")
        for series in (self.position_m, self.yaw_rad):
            if any(b < a for a, b in zip(series, series[1:])) or min(series) < 0:
                raise ValueError("error bounds must be nonnegative and non-decreasing")

    def at(self, age_s: float) -> tuple[float, float] | None:
        """(position, yaw) bound for an age: the next listed age's values; None past the table."""
        if age_s < 0:
            age_s = 0.0
        for a, p, y in zip(self.ages_s, self.position_m, self.yaw_rad):
            if age_s <= a:
                return p, y
        return None


@dataclass(frozen=True)
class GridConfig:
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    error: AgeErrorTable
    resolution_m: float = 0.05
    max_mark_m: float = 5.0
    max_clear_m: float = 5.0
    range_min_m: float = 0.15
    free_max_age_s: float = 0.2
    occupied_max_age_s: float = 3.0
    sensor_bound_m: float = 0.06
    free_r_cap_m: float = 0.20
    free_rho_m: float = 4.0  # free evidence only within this distance of the rear axle
    rho_bands_m: tuple[float, ...] = (1.5, 2.5, 3.25, 4.0)  # free shrink computed per band edge

    @property
    def shape(self) -> tuple[int, int]:
        return (
            max(1, ceil((self.x_max_m - self.x_min_m) / self.resolution_m)),
            max(1, ceil((self.y_max_m - self.y_min_m) / self.resolution_m)),
        )

    @property
    def half_diagonal_m(self) -> float:
        return self.resolution_m * sqrt(2) / 2


@dataclass(frozen=True)
class ObstacleScan:
    """One planar scan in the frame of its own sensor.

    odom_rear: wheel-odometry rear-axle pose (x, y, yaw) at the measurement
    instant. laser_in_rear: the sensor pose in the rear-axle frame at that
    instant. self_hit: per beam, True when the return is explained by the
    truck itself (body, forks, carried pallet) as posed at measurement.
    may_clear: whether this sensor's beams may clear cells. A planar beam
    only says its own plane is empty; only a plane low enough for the plan's
    floor-standing assumption (D0, below h_det) may clear. A higher plane
    (the SLAM LiDAR) passes over low obstacles and only marks.
    own_footprint: the truck's own outline at measurement; its cells are not
    obstacles, so the free shrink does not eat a ring around the truck.
    """

    stamp_s: float
    sensor: str
    odom_rear: tuple[float, float, float]
    laser_in_rear: tuple[float, float, float]
    angles_rad: np.ndarray
    ranges_m: np.ndarray
    self_hit: np.ndarray
    may_clear: bool = True
    own_footprint: tuple[float, float, float] | None = None  # front, rear, half width from the rear axle


def compose(a, b):
    x, y, t = a
    c, s = cos(t), sin(t)
    return (x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], t + b[2])


@dataclass
class GridSnapshot:
    state: np.ndarray  # uint8 UNKNOWN / FREE / OCCUPIED
    stamp_s: float
    correction: tuple[float, float, float]
    correction_version: int
    scans_used: int
    newest_scan_s: dict = field(default_factory=dict)  # sensor -> newest stamp
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    resolution_m: float = 0.05

    @property
    def occupied(self) -> np.ndarray:
        return self.state == OCCUPIED

    @property
    def free(self) -> np.ndarray:
        return self.state == FREE


def _disk_offsets(radius_cells: float) -> np.ndarray:
    span = int(ceil(radius_cells))
    di, dj = np.meshgrid(np.arange(-span, span + 1), np.arange(-span, span + 1), indexing="ij")
    keep = np.hypot(di, dj) <= radius_cells
    return np.column_stack((di[keep], dj[keep]))


def _erode(mask: np.ndarray, radius_cells: float) -> np.ndarray:
    """Cells of mask whose whole disk of radius_cells lies inside mask."""
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    nx, ny = mask.shape
    padded = np.zeros((nx + 2, ny + 2), dtype=bool)
    padded[1:-1, 1:-1] = mask
    span = int(ceil(radius_cells))
    big = np.zeros((nx + 2 * span, ny + 2 * span), dtype=bool)
    big[span : span + nx, span : span + ny] = mask
    for di, dj in _disk_offsets(radius_cells):
        out &= big[span + di : span + di + nx, span + dj : span + dj + ny]
    return out


class ObstacleGrid:
    def __init__(self, config: GridConfig):
        self.config = config
        self.scans: deque[ObstacleScan] = deque()
        self.version = 0

    def add_scan(self, scan: ObstacleScan) -> None:
        if self.scans and scan.stamp_s < self.scans[-1].stamp_s:
            raise ValueError("scans must arrive in measurement order")
        n = len(scan.angles_rad)
        if len(scan.ranges_m) != n or len(scan.self_hit) != n:
            raise ValueError("angles, ranges and self_hit must be equally long")
        self.scans.append(scan)

    def prune(self, now_s: float) -> None:
        keep = self.config.occupied_max_age_s
        while self.scans and now_s - self.scans[0].stamp_s > keep:
            self.scans.popleft()

    def radius_m(self, age_s: float, rho_m: float) -> float | None:
        bound = self.config.error.at(age_s)
        if bound is None:
            return None
        e, psi = bound
        return e + 2 * rho_m * sin(psi / 2) + self.config.sensor_bound_m

    def snapshot(self, now_s: float, map_from_odom, correction_version: int = 0) -> GridSnapshot:
        cfg = self.config
        self.prune(now_s)
        nx, ny = cfg.shape
        state = np.zeros((nx, ny), dtype=np.uint8)
        res = cfg.resolution_m
        newest: dict = {}
        used = 0
        for scan in self.scans:
            age = now_s - scan.stamp_s
            if age < -1e-9:
                continue  # measured after the snapshot instant
            used += 1
            newest[scan.sensor] = max(newest.get(scan.sensor, -np.inf), scan.stamp_s)
            rear_map = compose(map_from_odom, scan.odom_rear)
            laser = compose(rear_map, scan.laser_in_rear)
            ranges = np.asarray(scan.ranges_m, dtype=float)
            angles = np.asarray(scan.angles_rad, dtype=float) + laser[2]
            usable = ~np.asarray(scan.self_hit, dtype=bool)
            cos_a, sin_a = np.cos(angles), np.sin(angles)
            # -- clearing: a newer scan that saw through a cell removes older
            # marks there; only a fresh one also makes it FREE evidence.
            r_free = self.radius_m(age, cfg.free_rho_m)
            if r_free is not None and scan.may_clear:
                finite = np.isfinite(ranges)
                clear_to = np.where(
                    ranges == np.inf,
                    cfg.max_clear_m,
                    np.where(finite, np.minimum(ranges, cfg.max_mark_m), 0.0),
                )
                clear_to = np.where(usable, np.minimum(clear_to, cfg.max_clear_m), 0.0)
                # Hit beams: stop short of the hit by a cell so its own cell is not cleared.
                clear_to = np.where(finite & (ranges <= cfg.max_mark_m), clear_to - res, clear_to)
                raw = self._ray_mask(laser, cos_a, sin_a, np.maximum(clear_to, 0.0))
                own = self._rect_mask(rear_map, scan.own_footprint) if scan.own_footprint else None
                support = raw | own if own is not None else raw
                # Distance of each cell centre from the rear axle at the scan.
                gx = cfg.x_min_m + (np.arange(nx) + 0.5) * res
                gy = cfg.y_min_m + (np.arange(ny) + 0.5) * res
                rho = np.hypot(gx[:, None] - rear_map[0], gy[None, :] - rear_map[1])
                lower = 0.0
                for edge in cfg.rho_bands_m:
                    band = (rho >= lower) & (rho < edge)
                    lower = edge
                    if not (band & raw).any():
                        continue
                    r_band = self.radius_m(age, edge)
                    fresh = age <= cfg.free_max_age_s and r_band <= cfg.free_r_cap_m
                    eroded = _erode(support, (r_band + 2 * cfg.half_diagonal_m) / res) & raw & band
                    if own is not None:
                        eroded &= ~own
                    if fresh:
                        state[eroded] = FREE
                    else:
                        state[eroded & (state == OCCUPIED)] = UNKNOWN
                # Beyond the last band: clearing of older marks only.
                beyond = raw & (rho >= lower)
                state[beyond & (state == OCCUPIED)] = UNKNOWN
            # -- marking
            r_occ = self.radius_m(age, 0.0)
            if r_occ is None:
                continue
            hit = usable & np.isfinite(ranges) & (ranges >= cfg.range_min_m) & (ranges <= cfg.max_mark_m)
            if hit.any():
                px = laser[0] + ranges[hit] * cos_a[hit]
                py = laser[1] + ranges[hit] * sin_a[hit]
                rho = np.hypot(px - rear_map[0], py - rear_map[1])
                _, psi = cfg.error.at(age)
                radii = r_occ + 2 * rho * np.sin(psi / 2)
                self._mark_disks(state, px, py, radii)
            near = usable & ((ranges == -np.inf) | (np.isfinite(ranges) & (ranges < cfg.range_min_m)))
            if near.any():
                steps = np.linspace(0.0, cfg.range_min_m, max(2, int(ceil(cfg.range_min_m / (res / 2))) + 1))
                px = (laser[0] + np.outer(cos_a[near], steps)).ravel()
                py = (laser[1] + np.outer(sin_a[near], steps)).ravel()
                self._mark_disks(state, px, py, np.full(px.shape, r_occ))
        return GridSnapshot(
            state, now_s, tuple(float(v) for v in map_from_odom), correction_version, used, newest,
            cfg.x_min_m, cfg.y_min_m, res,
        )

    def _rect_mask(self, rear, footprint) -> np.ndarray:
        """Cells whose square touches the footprint rectangle at the rear-axle pose
        (centre within the rectangle grown by a half diagonal)."""
        cfg = self.config
        nx, ny = cfg.shape
        grow = cfg.half_diagonal_m
        front, back, half = footprint[0] + grow, footprint[1] + grow, footprint[2] + grow
        gx = cfg.x_min_m + (np.arange(nx) + 0.5) * cfg.resolution_m - rear[0]
        gy = cfg.y_min_m + (np.arange(ny) + 0.5) * cfg.resolution_m - rear[1]
        c, s = cos(rear[2]), sin(rear[2])
        u = gx[:, None] * c + gy[None, :] * s
        v = -gx[:, None] * s + gy[None, :] * c
        return (u <= front) & (u >= -back) & (np.abs(v) <= half)

    def _ray_mask(self, laser, cos_a, sin_a, lengths) -> np.ndarray:
        """Cells crossed by each beam from the sensor to its length (half-cell samples, conservative)."""
        cfg = self.config
        nx, ny = cfg.shape
        mask = np.zeros((nx, ny), dtype=bool)
        step = cfg.resolution_m / 2
        longest = float(lengths.max()) if len(lengths) else 0.0
        if longest <= 0:
            return mask
        s = np.arange(0.0, longest + 1e-9, step)
        keep = s[None, :] <= lengths[:, None]
        x = laser[0] + cos_a[:, None] * s[None, :]
        y = laser[1] + sin_a[:, None] * s[None, :]
        i = np.floor((x[keep] - cfg.x_min_m) / cfg.resolution_m).astype(int)
        j = np.floor((y[keep] - cfg.y_min_m) / cfg.resolution_m).astype(int)
        inside = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        mask[i[inside], j[inside]] = True
        return mask

    def _mark_disks(self, state, px, py, radii) -> None:
        """OCCUPIED in every cell whose square comes within each radius of its point."""
        cfg = self.config
        res = cfg.resolution_m
        nx, ny = cfg.shape
        ci = np.floor((px - cfg.x_min_m) / res).astype(int)
        cj = np.floor((py - cfg.y_min_m) / res).astype(int)
        # The point sits anywhere in its cell and each target cell extends a
        # half diagonal: centre-to-centre within r + 2 half diagonals.
        levels = np.ceil((radii + 2 * cfg.half_diagonal_m) / (res / 2)) * (res / 2)
        for level in np.unique(levels):
            sel = levels == level
            offsets = _disk_offsets(level / res)
            ti = (ci[sel][:, None] + offsets[None, :, 0]).ravel()
            tj = (cj[sel][:, None] + offsets[None, :, 1]).ravel()
            inside = (ti >= 0) & (ti < nx) & (tj >= 0) & (tj < ny)
            state[ti[inside], tj[inside]] = OCCUPIED


def snapshot_to_occupancy(snapshot: GridSnapshot):
    """The planner's view: OccupancyGrid of the OCCUPIED cells (unknown is free to it)."""
    from forklift_core.planning.grid_collision import OccupancyGrid

    return OccupancyGrid(
        snapshot.origin_x_m, snapshot.origin_y_m, snapshot.resolution_m, snapshot.occupied,
        version=snapshot.correction_version,
    )


__all__ = [
    "FREE",
    "OCCUPIED",
    "UNKNOWN",
    "AgeErrorTable",
    "GridConfig",
    "GridSnapshot",
    "ObstacleGrid",
    "ObstacleScan",
    "snapshot_to_occupancy",
]
