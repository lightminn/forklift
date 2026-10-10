"""Where is the truck on a saved occupancy map? Scan agreement and global relocalization.

Plan: docs/plans/2026-10-08-slam-operating-conditions.md, O5. A saved
slam_toolbox map (the nav2 map_server PGM + YAML that forklift_ros
slam_recorder writes) is turned into a truncated distance field: for every
cell, the distance to the nearest occupied cell, capped. Two questions are
then asked of a 2D scan:

- **Does this pose agree with the map?** ``hit_fraction``: the share of the
  scan's endpoints that land within a small distance of an occupied cell.
  A localizer that has lost track keeps publishing a pose; this is how the
  truck notices. ``LossDetector`` turns it into "lost" after a few
  consecutive low scans.
- **Where on the map could this scan have been taken?** ``relocalize``:
  likelihood-field matching over every free cell and heading, coarse to fine.
  A warehouse repeats itself, so the answer is refused when the best pose has
  a rival elsewhere (more than ``distinct_m`` or ``distinct_rad`` away) that
  scores almost as well -- a refused relocalization is recoverable, a wrong
  accepted one drives the truck from the wrong place.

Planar x, y metres and yaw radians, in the map frame. NumPy only; nothing here
touches ROS, a simulator or a LiDAR driver.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray

from forklift_core._validation import _finite_scalar, _real_array


@dataclass(frozen=True)
class OccupancyMap:
    """Trinary grid; row 0 is the map origin's y, column 0 its x (nav_msgs)."""

    occupied: NDArray[np.bool_]
    free: NDArray[np.bool_]
    resolution_m: float
    origin_xy_m: tuple[float, float]

    def __post_init__(self) -> None:
        if self.occupied.shape != self.free.shape or self.occupied.ndim != 2:
            raise ValueError("occupied and free must be the same 2D grid")
        if np.any(self.occupied & self.free):
            raise ValueError("a cell cannot be both occupied and free")
        _finite_scalar(self.resolution_m, "resolution_m")
        if self.resolution_m <= 0:
            raise ValueError("resolution_m must be positive")

    def cells(self, xy: NDArray[np.float64]) -> tuple[NDArray, NDArray, NDArray]:
        """(row, column, inside) of map-frame points."""
        col = np.floor((xy[..., 0] - self.origin_xy_m[0]) / self.resolution_m)
        row = np.floor((xy[..., 1] - self.origin_xy_m[1]) / self.resolution_m)
        rows, cols = self.occupied.shape
        inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
        return (
            np.where(inside, row, 0).astype(np.intp),
            np.where(inside, col, 0).astype(np.intp),
            inside,
        )

    def centres(self, rows: NDArray, cols: NDArray) -> NDArray[np.float64]:
        return np.column_stack(
            (
                self.origin_xy_m[0] + (cols + 0.5) * self.resolution_m,
                self.origin_xy_m[1] + (rows + 0.5) * self.resolution_m,
            )
        )


UNKNOWN_PIXEL = 205


def read_map(yaml_path: str | Path) -> OccupancyMap:
    """A nav2 map_server map (binary P5 PGM, negate 0, origin yaw 0).

    Pixel 205 is read as unknown whatever the thresholds: it is the unknown
    value slam_recorder (like nav2 map_saver) writes, but the recorder's YAML
    says free_thresh 0.25, under which nav2's rule (occupancy 0.196 below
    free_thresh) would read it as free; nav2's own default is 0.196.
    """
    yaml_path = Path(yaml_path)
    fields = {}
    for line in yaml_path.read_text().splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key.strip()] = value.strip()
    if fields.get("negate", "0") != "0":
        raise ValueError("negate: 1 maps are not supported")
    origin = [float(v) for v in fields["origin"].strip("[]").split(",")]
    if abs(origin[2]) > 1e-9:
        raise ValueError("a rotated map origin is not supported")
    image = _read_pgm(yaml_path.parent / fields["image"])
    image = np.flipud(image)
    occupancy = (255.0 - image) / 255.0
    return OccupancyMap(
        occupied=occupancy > float(fields.get("occupied_thresh", 0.65)),
        free=(occupancy < float(fields.get("free_thresh", 0.25)))
        & (image != UNKNOWN_PIXEL),
        resolution_m=float(fields["resolution"]),
        origin_xy_m=(origin[0], origin[1]),
    )


def _read_pgm(path: Path) -> NDArray[np.uint8]:
    data = path.read_bytes()
    tokens, position = [], 0
    while len(tokens) < 4:
        while data[position : position + 1].isspace():
            position += 1
        if data[position : position + 1] == b"#":
            position = data.index(b"\n", position) + 1
            continue
        end = position
        while not data[end : end + 1].isspace():
            end += 1
        tokens.append(data[position:end])
        position = end
    if tokens[0] != b"P5" or int(tokens[3]) != 255:
        raise ValueError(f"{path} is not an 8-bit binary PGM")
    width, height = int(tokens[1]), int(tokens[2])
    pixels = np.frombuffer(data, np.uint8, count=width * height, offset=position + 1)
    return pixels.reshape(height, width)


def distance_field(occupied: ArrayLike, resolution_m: float, max_m: float) -> NDArray:
    """Exact Euclidean distance (m) to the nearest occupied cell, capped at max_m.

    Two passes: the distance along each column, then for every cell the
    minimum over column offsets k of k^2 + column_distance^2 -- only offsets up
    to the cap matter, so it is a few dozen whole-array operations.
    """
    occupied = np.asarray(occupied, dtype=bool)
    _finite_scalar(resolution_m, "resolution_m")
    _finite_scalar(max_m, "max_m")
    reach = int(math.ceil(max_m / resolution_m))
    big = reach + 1
    rows, cols = occupied.shape
    down = np.full(occupied.shape, big, dtype=np.int64)
    run = np.full(cols, big, dtype=np.int64)
    for r in range(rows):
        run = np.where(occupied[r], 0, np.minimum(run + 1, big))
        down[r] = run
    column = down.copy()
    run = np.full(cols, big, dtype=np.int64)
    for r in range(rows - 1, -1, -1):
        run = np.where(occupied[r], 0, np.minimum(run + 1, big))
        column[r] = np.minimum(column[r], run)
    squared = column.astype(np.float64) ** 2
    padded = np.pad(squared, ((0, 0), (reach, reach)), constant_values=float(big) ** 2)
    best = squared.copy()
    for k in range(-reach, reach + 1):
        if k:
            best = np.minimum(best, k * k + padded[:, reach + k : reach + k + cols])
    return np.minimum(np.sqrt(best) * resolution_m, max_m)


def scan_points(
    ranges_m: ArrayLike,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    mount_xy_yaw: tuple[float, float, float] = (0.0, 0.0, 0.0),
    stride: int = 1,
) -> NDArray[np.float64]:
    """Endpoints of the finite, in-range beams in base_link (x forward, y left)."""
    ranges = _real_array(ranges_m, "ranges_m")
    angles = angle_min_rad + angle_increment_rad * np.arange(ranges.size)
    keep = np.isfinite(ranges) & (ranges >= range_min_m) & (ranges <= range_max_m)
    keep[np.arange(ranges.size) % max(int(stride), 1) != 0] = False
    x, y, yaw = mount_xy_yaw
    beam = angles[keep] + yaw
    return np.column_stack(
        (x + ranges[keep] * np.cos(beam), y + ranges[keep] * np.sin(beam))
    )


def stack_scans(points: list[ArrayLike], poses: ArrayLike) -> NDArray[np.float64]:
    """Several scans' base-frame points, each taken at poses[i] (odometry),
    expressed in the base frame of the last pose -- a short local submap."""
    poses = _real_array(poses, "poses").reshape(-1, 3)
    if len(points) != len(poses):
        raise ValueError("one pose per scan is required")
    last = poses[-1]
    out = []
    for p, pose in zip(points, poses, strict=True):
        world = _transform(np.asarray(p, dtype=float), pose)
        c, s = math.cos(last[2]), math.sin(last[2])
        d = world - last[:2]
        out.append(
            np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))
        )
    return np.vstack(out)


def _transform(points: NDArray, pose: ArrayLike) -> NDArray:
    x, y, yaw = (float(v) for v in pose)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack(
        (
            x + c * points[:, 0] - s * points[:, 1],
            y + s * points[:, 0] + c * points[:, 1],
        )
    )


def _distances(grid: OccupancyMap, field: NDArray, xy: NDArray, cap: float) -> NDArray:
    row, col, inside = grid.cells(xy)
    return np.where(inside, field[row, col], cap)


def hit_fraction(
    grid: OccupancyMap,
    field: NDArray,
    points_base: ArrayLike,
    pose: ArrayLike,
    hit_m: float,
) -> float:
    """Share of scan endpoints within hit_m of an occupied cell at this pose."""
    points = _real_array(points_base, "points_base").reshape(-1, 2)
    if not len(points):
        return 0.0
    d = _distances(grid, field, _transform(points, pose), float(field.max()) + hit_m)
    return float(np.mean(d <= hit_m))


@dataclass
class LossDetector:
    """Lost after ``consecutive`` scans in a row below ``threshold``."""

    threshold: float
    consecutive: int
    _low: int = 0

    def update(self, fraction: float) -> bool:
        self._low = self._low + 1 if fraction < self.threshold else 0
        return self._low >= self.consecutive


@dataclass(frozen=True)
class Relocalization:
    """The best pose, its rival elsewhere and whether the answer is accepted."""

    pose: tuple[float, float, float] | None
    score: float
    rival_pose: tuple[float, float, float] | None
    rival_score: float
    accepted: bool
    reason: str
    candidates_evaluated: int


def _scores(grid, field, sigma_m, points, xy, yaws, cap, chunk=4_000_000):
    """Mean likelihood exp(-d^2 / 2 sigma^2) for every (position, yaw)."""
    out = np.empty((len(xy), len(yaws)))
    step = max(1, chunk // max(len(points), 1))
    for j, yaw in enumerate(yaws):
        c, s = math.cos(yaw), math.sin(yaw)
        rotated = np.column_stack(
            (c * points[:, 0] - s * points[:, 1], s * points[:, 0] + c * points[:, 1])
        )
        for start in range(0, len(xy), step):
            block = xy[start : start + step, None, :] + rotated[None, :, :]
            d = _distances(grid, field, block, cap)
            out[start : start + step, j] = np.exp(-0.5 * (d / sigma_m) ** 2).mean(
                axis=1
            )
    return out


def _score_poses(grid, field, sigma_m, points, poses, cap):
    out = np.empty(len(poses))
    for i, (x, y, yaw) in enumerate(poses):
        d = _distances(grid, field, _transform(points, (x, y, yaw)), cap)
        out[i] = np.exp(-0.5 * (d / sigma_m) ** 2).mean()
    return out


def _refine(grid, field, sigma_m, points, pose, step_m, step_rad, cap):
    pose = np.array(pose, dtype=float)
    score = _score_poses(grid, field, sigma_m, points, pose[None], cap)[0]
    offsets = np.array(
        [(dx, dy, dt) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dt in (-1, 0, 1)],
        dtype=float,
    )
    while step_m >= grid.resolution_m / 2:
        trial = pose + offsets * (step_m, step_m, step_rad)
        scores = _score_poses(grid, field, sigma_m, points, trial, cap)
        best = int(np.argmax(scores))
        if scores[best] > score + 1e-12:
            pose, score = trial[best], scores[best]
        else:
            step_m, step_rad = step_m / 2, step_rad / 2
    pose[2] = (pose[2] + math.pi) % (2 * math.pi) - math.pi
    return pose, score


def relocalize(
    grid: OccupancyMap,
    points_base: ArrayLike,
    *,
    clearance_m: float = 0.3,
    coarse_step_m: float = 0.25,
    coarse_yaw_step_rad: float = math.radians(3.0),
    coarse_sigma_m: float = 0.25,
    fine_sigma_m: float = 0.05,
    field_max_m: float = 1.0,
    candidates: int = 40,
    distinct_m: float = 1.0,
    distinct_rad: float = 0.5,
    min_score: float = 0.5,
    max_rival_ratio: float = 0.9,
    field: NDArray | None = None,
) -> Relocalization:
    """Search the whole map for the pose that best explains the scan.

    Positions: free cells at least clearance_m from an occupied cell, on a
    coarse_step_m lattice; headings every coarse_yaw_step_rad. The best
    ``candidates`` are refined by a shrinking local search with the fine
    likelihood. Accepted when the best fine score reaches ``min_score`` and the
    best distinct rival scores at most ``max_rival_ratio`` of it. Seeds are
    kept half the distinct distance apart so that rival modes get refined too.
    ``field`` = ``distance_field(grid.occupied, grid.resolution_m,
    field_max_m)``, to reuse it across calls.
    """
    points = _real_array(points_base, "points_base").reshape(-1, 2)
    if not len(points):
        return Relocalization(None, 0.0, None, 0.0, False, "no scan points", 0)
    if field is None:
        field = distance_field(grid.occupied, grid.resolution_m, field_max_m)
    cap = field_max_m
    stride = max(1, int(round(coarse_step_m / grid.resolution_m)))
    rows, cols = np.nonzero(grid.free & (field >= clearance_m))
    keep = (rows % stride == 0) & (cols % stride == 0)
    xy = grid.centres(rows[keep], cols[keep])
    if not len(xy):
        return Relocalization(None, 0.0, None, 0.0, False, "no free position", 0)
    yaws = np.arange(-math.pi, math.pi, coarse_yaw_step_rad)
    coarse = _scores(grid, field, coarse_sigma_m, points, xy, yaws, cap)
    flat = np.argsort(coarse, axis=None)[::-1]
    apart_m = max(2 * coarse_step_m, distinct_m / 2)
    apart_rad = max(2 * coarse_yaw_step_rad, distinct_rad / 2)
    seeds = []
    for index in flat:
        p, j = np.unravel_index(index, coarse.shape)
        pose = np.array([xy[p, 0], xy[p, 1], yaws[j]])
        if all(not _same(pose, s, apart_m, apart_rad) for s in seeds):
            seeds.append(pose)
        if len(seeds) == candidates:
            break
    refined = [
        _refine(
            grid,
            field,
            fine_sigma_m,
            points,
            s,
            coarse_step_m / 2,
            coarse_yaw_step_rad / 2,
            cap,
        )
        for s in seeds
    ]
    refined.sort(key=lambda r: -r[1])
    best_pose, best_score = refined[0]
    rival = next(
        (
            r
            for r in refined[1:]
            if not _same(r[0], best_pose, distinct_m, distinct_rad)
        ),
        None,
    )
    rival_pose = None if rival is None else tuple(float(v) for v in rival[0])
    rival_score = 0.0 if rival is None else float(rival[1])
    if best_score < min_score:
        accepted, reason = False, "best score below min_score"
    elif rival is not None and rival_score > max_rival_ratio * best_score:
        accepted, reason = False, "ambiguous: a distinct pose scores almost as well"
    else:
        accepted, reason = True, "accepted"
    return Relocalization(
        pose=tuple(float(v) for v in best_pose),
        score=float(best_score),
        rival_pose=rival_pose,
        rival_score=rival_score,
        accepted=accepted,
        reason=reason,
        candidates_evaluated=int(coarse.size),
    )


def _same(a, b, distance_m, angle_rad) -> bool:
    turn = abs((a[2] - b[2] + math.pi) % (2 * math.pi) - math.pi)
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= distance_m and turn <= angle_rad
