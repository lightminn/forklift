"""Start poses for the global matcher's calibration (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md B4a).

Base poses on a survey map where the empty truck can stand: its footprint (rear-axle frame, ``FOOTPRINT``) overlaps
only free cells of the survey grid (no occupied, no unknown cell, judged per cell with a strict rectangle test), and
none of the scene's own obstacles the grid does not show (low props, pallets: ``keep_out`` rectangles). Stratum A
draws a cell uniformly from those where the base may stand; stratum B from the ridge of the free space's distance field
(a cell no lower than both neighbours along x or along y and higher than one of them: the middle of an aisle, where the scan is most symmetric).
stratum C from cells ``WALL_BAND_M`` from the nearest non-free cell (beside a wall or rack, where trucks are parked).
Headings are uniform. A pose is taken only if its laser stands at least ``sep_m`` from every laser position already
taken on that map (the earlier split's included): the 360 deg scan from one spot is the same observation at any
heading, so a turn on the spot is not an independent start (Codex B-P1 pre-registration review P2). Pure numpy/scipy; the sampler reads the grid and
rectangles, never a matcher output.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.localization.slam_map import Grid
from forklift_core.planning.geometry import Footprint, Rectangle

FOOTPRINT = Footprint(1.29, 0.17, 0.36)  # dls08_measured, empty, rear axle
REAR_AXLE_OFFSET_M = 0.34  # base_link ahead of the rear axle
SEP_M = 3.0  # plan B4a: laser positions this far apart
LASER_X_M = -0.12  # the LiDAR ahead of base_link (config/obstacle_layer_single_team_spec.yaml)
WALL_BAND_M = (0.4, 0.9)  # stratum C: the base cell's distance to the nearest non-free cell


@dataclass(frozen=True)
class StartPose:
    index: int
    stratum: str  # "A" uniform | "B" ridge | "C" beside a wall
    base: tuple  # (x, y, yaw) of base_link in the map


def rect_corners(x: float, y: float, yaw: float, x0: float, x1: float, hw: float) -> np.ndarray:
    """Counter-clockwise corners of the box [x0, x1] x [-hw, hw] in a frame at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    local = ((x0, -hw), (x1, -hw), (x1, hw), (x0, hw))
    return np.array([(x + c * a - s * b, y + s * a + c * b) for a, b in local])


def rectangle_corners(r: Rectangle) -> np.ndarray:
    return rect_corners(r.x_m, r.y_m, r.yaw_rad, -r.length_m / 2, r.length_m / 2, r.width_m / 2)


def _overlap(a: np.ndarray, b: np.ndarray) -> bool:
    """Whether two convex polygons overlap with positive area (separating axes, touching is not overlap)."""
    for poly in (a, b):
        for p, q in zip(poly, np.roll(poly, -1, axis=0), strict=True):
            n = np.array([q[1] - p[1], p[0] - q[0]])
            pa, pb = a @ n, b @ n
            if pa.max() <= pb.min() + 1e-12 or pb.max() <= pa.min() + 1e-12:
                return False
    return True


def footprint_corners(base, footprint: Footprint = FOOTPRINT) -> np.ndarray:
    x, y, yaw = base
    rx, ry = x - REAR_AXLE_OFFSET_M * math.cos(yaw), y - REAR_AXLE_OFFSET_M * math.sin(yaw)
    return rect_corners(rx, ry, yaw, -footprint.rear_m, footprint.front_m, footprint.half_width_m)


def footprint_free(grid: Grid, base, keep_out=(), footprint: Footprint = FOOTPRINT) -> bool:
    """Whether the footprint at ``base`` overlaps only free grid cells (an overlapped cell outside the grid counts as
    unknown) and no ``keep_out`` rectangle."""
    if abs(grid.origin[2]) > 1e-12:
        raise ValueError("an unrotated grid origin is required")
    fp = footprint_corners(base, footprint)
    res, (ox, oy, _) = grid.resolution_m, grid.origin
    c0, c1 = int(math.floor((fp[:, 0].min() - ox) / res)), int(math.floor((fp[:, 0].max() - ox) / res))
    r0, r1 = int(math.floor((fp[:, 1].min() - oy) / res)), int(math.floor((fp[:, 1].max() - oy) / res))
    h, w = grid.data.shape
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            inside = 0 <= r < h and 0 <= c < w
            if inside and grid.data[r, c] == 0:
                continue
            cell = rect_corners(ox + (c + 0.5) * res, oy + (r + 0.5) * res, 0.0, -res / 2, res / 2, res / 2)
            if _overlap(fp, cell):
                return False
    return not any(_overlap(fp, k) for k in keep_out)


def ridge_cells(grid: Grid) -> np.ndarray:
    """Free cells no lower than both neighbours along x or along y in the distance to the nearest non-free cell."""
    from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

    free = grid.data == 0
    d = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
    p = np.pad(d, 1)
    left, right, down, up = p[1:-1, :-2], p[1:-1, 2:], p[:-2, 1:-1], p[2:, 1:-1]
    # no lower than both neighbours and higher than one: a crest, not a flat run along it
    x_ridge = (d >= left) & (d >= right) & ((d > left) | (d > right))
    y_ridge = (d >= down) & (d >= up) & ((d > down) | (d > up))
    return free & (x_ridge | y_ridge)


def laser_xy(base) -> tuple:
    return (base[0] + LASER_X_M * math.cos(base[2]), base[1] + LASER_X_M * math.sin(base[2]))


def independent(base, taken, sep_m: float = SEP_M) -> bool:
    """Whether the laser at ``base`` stands at least ``sep_m`` from the laser of every pose in ``taken``."""
    lx, ly = laser_xy(base)
    return all(math.hypot(lx - t[0], ly - t[1]) >= sep_m for t in map(laser_xy, taken))


def wall_band_cells(grid: Grid, band=WALL_BAND_M) -> np.ndarray:
    from scipy.ndimage import distance_transform_edt  # noqa: PLC0415

    free = grid.data == 0
    d = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * grid.resolution_m
    return free & (d >= band[0]) & (d <= band[1])


def sample_starts(grid: Grid, *, count: int, seed: int, keep_out=(), taken=(), sep_m: float = SEP_M,
                  pattern=("A", "A", "B", "C"), max_attempts: int = 200_000, fill: bool = False,
                  exhaust_after: int = 20_000, report: dict | None = None) -> list:
    """``count`` independent start poses, the strata in ``pattern`` order (A A B C A A B C ...), independent of
    ``taken`` too. When the attempts run out: raises. With ``fill`` (a validation map drawn until full, plan B4a) a
    stratum that fails ``exhaust_after`` draws in a row is exhausted and leaves the pattern, the others go on, and the
    map is full when every stratum is exhausted (Codex B4a re-review P3: a blocked stratum must not end the draw while
    another still has room); ``max_attempts`` still bounds the whole draw. ``report`` (a dict) receives why the draw
    stopped (``count`` | ``exhausted`` | ``cap``), the attempts and the exhausted strata."""
    rng = np.random.default_rng(seed)
    free_rc = np.argwhere(grid.data == 0)
    ridge_rc = np.argwhere(ridge_cells(grid))
    pool = {"A": free_rc, "B": ridge_rc, "C": np.argwhere(wall_band_cells(grid))}
    res, (ox, oy, _) = grid.resolution_m, grid.origin
    out: list = []
    seen = list(taken)
    attempts = 0
    live = list(pattern)
    turn = 0  # position in the live pattern
    misses = {k: 0 for k in pattern}

    def done(reason: str) -> list:
        if report is not None:
            report.update(stop_reason=reason, attempts=attempts,
                          exhausted=sorted(set(pattern) - set(live)), strata={k: sum(p.stratum == k for p in out)
                                                                             for k in sorted(set(pattern))})
        return out

    while len(out) < count:
        if not live:
            return done("exhausted")  # fill: every stratum exhausted
        stratum = live[turn % len(live)]
        cells = pool[stratum]
        if not len(cells):
            if not fill:
                raise ValueError(f"stratum {stratum} is empty")
            live = [k for k in live if k != stratum]
            continue
        if attempts >= max_attempts:
            if fill:
                return done("cap")
            raise ValueError(f"only {len(out)} of {count} independent starts after {max_attempts} attempts")
        attempts += 1
        r, c = cells[rng.integers(len(cells))]
        base = (ox + (c + rng.random()) * res, oy + (r + rng.random()) * res, float(rng.uniform(0.0, 2 * math.pi)))
        if not independent(base, seen, sep_m) or not footprint_free(grid, base, keep_out):
            misses[stratum] += 1
            if fill and misses[stratum] >= exhaust_after:
                live = [k for k in live if k != stratum]
            continue
        misses[stratum] = 0
        out.append(StartPose(len(out), stratum, base))
        seen.append(base)
        turn += 1
    return done("count")


__all__ = ["FOOTPRINT", "LASER_X_M", "REAR_AXLE_OFFSET_M", "SEP_M", "StartPose", "WALL_BAND_M", "footprint_corners",
           "footprint_free", "independent", "laser_xy", "rect_corners", "rectangle_corners", "ridge_cells",
           "sample_starts", "wall_band_cells"]
