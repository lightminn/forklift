#!/usr/bin/env python3
"""Map quality for the slam-bounded-graph plan (criterion 4): the planner's grid against a truth map.

The truth map is the recorded clean scans (spool ``slam_log.scan``: ranges and the Isaac truth laser pose) placed at
their truth poses: every beam end short of the range limit hits a cell (0.05 m), and a cell hit by at least
``min_hits`` beams is a truth obstacle. It holds what the LiDAR plane sees -- walls, racks, markers -- with no scene
geometry needed (the run's scenario lists only some props). A grid is scored in world cells:

- preservation: truth obstacle cells the grid holds as occupied within one cell (an obstacle turned free or unknown
  loses it);
- registration: the grid's occupied cells within 0.10 m of a truth obstacle (a shifted or warped map loses it).

Grids: a mapping run's ``bridge/maps`` (the map written last before each unit's first scan, from ``maps.jsonl``),
or a bundle's ``grid.npz`` (candidate A: the planner's static layer).

    python tools/slam_map_quality.py --truth-runs RUN ... (--drive DRIVE_JSONL --maps BRIDGE_DIR | --grid GRID.npz)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

OCCUPIED = 50  # int8 grids: -1 unknown, 0 free, 100 occupied (bundle and /map conventions)
TOLERANCE_M = 0.10


@dataclass(frozen=True)
class Grid:
    data: np.ndarray
    origin: tuple  # x, y, yaw of cell (0, 0)'s corner
    resolution_m: float


def truth_cells(poses, ranges, angles, *, resolution_m: float, min_hits: int, range_max_m: float) -> set:
    """World cells (floor(x / res), floor(y / res)) hit by at least ``min_hits`` beam ends."""
    hits = {}
    for (x, y, yaw), r in zip(poses, ranges):
        r = np.asarray(r, float)
        a = np.asarray(angles, float) + float(yaw)
        ok = np.isfinite(r) & (r < range_max_m - 1e-3)
        ex = float(x) + r[ok] * np.cos(a[ok])
        ey = float(y) + r[ok] * np.sin(a[ok])
        for cell in zip(np.floor(ex / resolution_m).astype(int).tolist(), np.floor(ey / resolution_m).astype(int).tolist()):
            hits[cell] = hits.get(cell, 0) + 1
    return {c for c, n in hits.items() if n >= min_hits}


def grid_cells(grid: Grid) -> set:
    """The grid's occupied cells as world cells (the cell centre's floor index at the grid's resolution)."""
    rows, cols = np.nonzero(np.asarray(grid.data) >= OCCUPIED)
    ox, oy = float(grid.origin[0]), float(grid.origin[1])
    res = float(grid.resolution_m)
    wx = ox + (cols + 0.5) * res
    wy = oy + (rows + 0.5) * res
    return set(zip(np.floor(wx / res).astype(int).tolist(), np.floor(wy / res).astype(int).tolist()))


def _dilate(cells: set, k: int) -> set:
    out = set()
    for ix, iy in cells:
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                out.add((ix + dx, iy + dy))
    return out


def quality(grid: set, truth: set, *, resolution_m: float) -> dict:
    near_grid = _dilate(grid, 1)
    near_truth = _dilate(truth, int(math.ceil(TOLERANCE_M / resolution_m)))
    return {"truth_cells": len(truth), "grid_cells": len(grid),
            "preservation": (sum(1 for c in truth if c in near_grid) / len(truth)) if truth else None,
            "registration": (sum(1 for c in grid if c in near_truth) / len(grid)) if grid else None}


def map_index_before(rows: list, scan_id: int):
    """The index of the map file written last before ``scan_id`` (a run of repeats points at its first map)."""
    best, current = None, None
    for row in rows:
        if "index" in row:
            current = int(row["index"])
            after = int(row["after_scan_id"])
        else:
            after = int(row["last_after_scan_id"])  # repeats of ``current``: same content, its file
        if after < scan_id:
            best = current
    return best


def _truth_from_runs(runs: list, resolution_m: float, min_hits: int) -> set:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from forklift_core.live.spool import load_slam_log  # noqa: PLC0415

    from slam_remap_drive import ANG0, BEAMS, DANG, RMAX  # noqa: PLC0415

    angles = ANG0 + DANG * np.arange(BEAMS)
    cells = set()
    for run in runs:
        log = load_slam_log(Path(run))
        cells |= truth_cells(log["laser_pose_world"], log["scan_ranges_m"], angles, resolution_m=resolution_m,
                             min_hits=min_hits, range_max_m=RMAX)
    return cells


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--truth-runs", type=Path, nargs="+", required=True)
    ap.add_argument("--drive", type=Path, default=None)
    ap.add_argument("--maps", type=Path, default=None, help="a mapping replay's bridge folder (maps.jsonl, maps/)")
    ap.add_argument("--grid", type=Path, default=None, help="a bundle's grid.npz")
    ap.add_argument("--resolution-m", type=float, default=0.05)
    ap.add_argument("--min-hits", type=int, default=3)
    a = ap.parse_args(argv)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    truth = _truth_from_runs(a.truth_runs, a.resolution_m, a.min_hits)
    out = {"truth_cells": len(truth)}
    if a.grid is not None:
        with np.load(a.grid) as z:
            g = Grid(np.asarray(z["data"]), tuple(float(v) for v in z["origin"]), float(z["resolution_m"]))
        out["grid"] = quality(grid_cells(g), truth, resolution_m=a.resolution_m)
    if a.maps is not None and a.drive is not None:
        rows = [json.loads(x) for x in (a.maps / "maps.jsonl").read_text().splitlines() if x.strip()]
        drive = [json.loads(x) for x in a.drive.read_text().splitlines() if x.strip()]
        firsts = {}
        for r in drive:
            firsts.setdefault(int(r.get("unit", 0)), int(r["scan_id"]))
        per_unit = []
        for unit in sorted(firsts):
            idx = map_index_before(rows, firsts[unit])
            if idx is None:
                continue
            meta = next(r for r in rows if r.get("index") == idx)
            with np.load(a.maps / "maps" / f"map_{idx:06d}.npz") as z:
                g = Grid(np.asarray(z["data"]), tuple(meta["origin"]), float(meta["resolution_m"]))
            per_unit.append({"unit": unit, "map_index": idx, **quality(grid_cells(g), truth, resolution_m=a.resolution_m)})
        out["units"] = per_unit
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
