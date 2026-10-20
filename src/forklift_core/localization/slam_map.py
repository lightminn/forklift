"""A survey map bundle and the localization start checks (SLAM localization plan docs/plans/2026-10-20-slam-localization-
session.md L2, L4).

The bundle (tools/slam_survey_bundle.py): ``map.posegraph`` and ``map.data`` (slam_toolbox's serialized graph),
``grid.npz`` (that graph's occupancy grid: int8 ``data`` row-major from ``origin`` (x, y, yaw), ``resolution_m``) and
``manifest.json`` with ``map_id`` -- SHA-256 over ``name sha256`` lines of the three files in name order.

``load_bundle`` re-hashes the files and refuses (``slam_map_mismatch``) a bundle whose files, manifest or expected id
disagree; ``run_mismatches`` lists what of this run differs from the survey's (seed and the input files it ran on).
The start checks are pure: ``map_coverage`` (how much of the survey's occupied cells a /map from slam_toolbox shows
occupied -- a graph not loaded shows next to nothing) and ``endpoint_fraction`` (how much of a scan's finite endpoints
fall on the survey's occupied cells, one cell dilated). NaN never passes: an empty set gives NaN and fails.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FORMAT = "slam_map_bundle/1"
BUNDLE_FILES = ("map.posegraph", "map.data", "grid.npz")
OCCUPIED = 65  # /map values: 100 occupied, 0 free, -1 unknown (the survey's own caster reads >= 65)
# the input files a survey and a localization run must share (manifest "inputs", by content)
SHARED_INPUTS = ("base_scene", "forklift_urdf", "factory_layout", "lidar")
MAP_LOADED_MIN = 0.95  # plan L4: the /map after the first processed scan covers >= 95 % of the survey's occupied cells
READY_MIN = 0.80  # plan L4: bootstrap scans 5-7 each put >= 80 % of their endpoints on the survey's walls
START_POSITION_M, START_YAW_RAD = 0.10, 0.05

MISMATCH, NOT_LOADED, NOT_LOCALIZED = "slam_map_mismatch", "slam_map_not_loaded", "slam_not_localized"
REASONS = (MISMATCH, NOT_LOADED, NOT_LOCALIZED)


class MapBundleError(ValueError):
    """A bundle that may not be used; ``str`` starts with ``slam_map_mismatch``."""


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def map_id_of(hashes: dict) -> str:
    text = "".join(f"{name} {hashes[name]}\n" for name in sorted(hashes))
    return hashlib.sha256(text.encode()).hexdigest()


@dataclass(frozen=True)
class Grid:
    data: np.ndarray  # int8 (rows = y, columns = x)
    origin: tuple  # x, y, yaw of cell (0, 0)'s corner
    resolution_m: float

    @classmethod
    def from_npz(cls, path: Path) -> "Grid":
        with np.load(path) as z:
            return cls(np.asarray(z["data"], np.int8), tuple(float(v) for v in z["origin"]), float(z["resolution_m"]))

    def occupied(self) -> np.ndarray:
        return self.data >= OCCUPIED

    def cell_centres(self, mask: np.ndarray) -> np.ndarray:
        """(N, 2) world positions of the cells ``mask`` selects."""
        rows, cols = np.nonzero(mask)
        local = np.column_stack(((cols + 0.5) * self.resolution_m, (rows + 0.5) * self.resolution_m))
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return np.column_stack((self.origin[0] + c * local[:, 0] - s * local[:, 1],
                                self.origin[1] + s * local[:, 0] + c * local[:, 1]))

    def cells_of(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(rows, cols, inside) of world ``points``."""
        points = np.asarray(points, float).reshape(-1, 2)
        dx, dy = points[:, 0] - self.origin[0], points[:, 1] - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        with np.errstate(invalid="ignore"):
            cols = np.floor(lx / self.resolution_m)
            rows = np.floor(ly / self.resolution_m)
        finite = np.isfinite(cols) & np.isfinite(rows)
        h, w = self.data.shape
        inside = finite & (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        return np.where(inside, rows, 0).astype(int), np.where(inside, cols, 0).astype(int), inside


@dataclass(frozen=True)
class MapBundle:
    folder: Path
    map_id: str
    manifest: dict
    grid: Grid

    @property
    def graph_prefix(self) -> Path:
        """What slam_toolbox's ``map_file_name`` names (``<prefix>.posegraph`` / ``.data``)."""
        return self.folder / "map"


def load_bundle(folder: Path, expected_map_id: str | None = None) -> MapBundle:
    folder = Path(folder)
    try:
        manifest = json.loads((folder / "manifest.json").read_text())
    except (OSError, ValueError) as exc:
        raise MapBundleError(f"{MISMATCH}: manifest unreadable ({exc})") from exc
    if manifest.get("format") != FORMAT:
        raise MapBundleError(f"{MISMATCH}: format {manifest.get('format')!r}, not {FORMAT}")
    hashes = {}
    for name in BUNDLE_FILES:
        if not (folder / name).is_file():
            raise MapBundleError(f"{MISMATCH}: {name} missing")
        hashes[name] = file_sha256(folder / name)
    if hashes != manifest.get("files"):
        raise MapBundleError(f"{MISMATCH}: the files differ from the manifest's hashes")
    actual = map_id_of(hashes)
    if actual != manifest.get("map_id"):
        raise MapBundleError(f"{MISMATCH}: the manifest's map_id is not its files'")
    if expected_map_id is not None and actual != expected_map_id:
        raise MapBundleError(f"{MISMATCH}: map_id {actual[:12]} is not the session's {str(expected_map_id)[:12]}")
    try:
        grid = Grid.from_npz(folder / "grid.npz")
    except (OSError, ValueError, KeyError) as exc:
        raise MapBundleError(f"{MISMATCH}: grid unreadable ({exc})") from exc
    return MapBundle(folder, actual, manifest, grid)


def run_mismatches(manifest: dict, *, seed, input_sha256: dict) -> list[str]:
    """What of this run differs from the survey's: its seed and the shared input files (``input_sha256``: argument
    name -> this run's file hash, None when the run names none)."""
    out = []
    if manifest.get("seed") != seed:
        out.append(f"seed {seed} is not the survey's {manifest.get('seed')}")
    inputs = manifest.get("inputs") or {}
    for name in SHARED_INPUTS:
        want = (inputs.get(name) or {}).get("sha256")
        if want is None or input_sha256.get(name) != want:
            out.append(f"{name} differs from the survey's")
    return out


def map_coverage(survey: Grid, live: Grid) -> float:
    """The fraction of the survey's occupied cells whose centres fall on occupied cells of ``live`` (NaN: none)."""
    centres = survey.cell_centres(survey.occupied())
    if not len(centres):
        return math.nan
    rows, cols, inside = live.cells_of(centres)
    hit = inside & live.occupied()[rows, cols]
    return float(hit.sum()) / len(centres)


def dilated(mask: np.ndarray, cells: int = 1) -> np.ndarray:
    out = mask.copy()
    for _ in range(cells):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        grown[1:, 1:] |= out[:-1, :-1]
        grown[:-1, :-1] |= out[1:, 1:]
        grown[1:, :-1] |= out[:-1, 1:]
        grown[:-1, 1:] |= out[1:, :-1]
        out = grown
    return out


def endpoint_fraction(survey: Grid, laser_pose, ranges, angle_min_rad: float, angle_increment_rad: float,
                      range_min_m: float, range_max_m: float, dilate_cells: int = 1) -> float:
    """The fraction of the scan's measured endpoints (finite, within the range limits) on the survey's occupied cells
    dilated by ``dilate_cells`` (NaN: no such endpoint). ``laser_pose``: the laser in the map, (x, y, yaw)."""
    r = np.asarray(ranges, float)
    angles = laser_pose[2] + angle_min_rad + angle_increment_rad * np.arange(len(r))
    measured = np.isfinite(r) & (r >= range_min_m) & (r <= range_max_m)
    if not measured.any():
        return math.nan
    points = np.column_stack((laser_pose[0] + r[measured] * np.cos(angles[measured]),
                              laser_pose[1] + r[measured] * np.sin(angles[measured])))
    rows, cols, inside = survey.cells_of(points)
    walls = dilated(survey.occupied(), dilate_cells)
    return float((inside & walls[rows, cols]).sum()) / len(points)


def start_pose_error(pose, start) -> tuple[float, float]:
    dyaw = math.atan2(math.sin(pose[2] - start[2]), math.cos(pose[2] - start[2]))
    return math.hypot(pose[0] - start[0], pose[1] - start[1]), abs(dyaw)


def localized(fractions: list, poses: list, start) -> bool:
    """Plan L4's readiness: every bootstrap scan's endpoint fraction >= READY_MIN and pose near the start."""
    if not fractions or len(fractions) != len(poses):
        return False
    for fraction, pose in zip(fractions, poses):
        position, yaw = start_pose_error(pose, start)
        if not (fraction >= READY_MIN and position <= START_POSITION_M and yaw <= START_YAW_RAD):
            return False  # NaN compares False
    return True


__all__ = ["BUNDLE_FILES", "FORMAT", "Grid", "MAP_LOADED_MIN", "MISMATCH", "MapBundle", "MapBundleError", "NOT_LOADED",
           "NOT_LOCALIZED", "READY_MIN", "REASONS", "SHARED_INPUTS", "dilated", "endpoint_fraction", "file_sha256",
           "load_bundle", "localized", "map_coverage", "map_id_of", "run_mismatches", "start_pose_error"]
