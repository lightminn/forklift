#!/usr/bin/env python3
"""Does the top of the column band alone decide the 1.0-1.5 m frames? The levelled frames with only the column band's top
moved: inside this process ``_column_grid`` is replaced by a copy with that one limit changed, so the upper-deck evidence,
the opening rays, the planes and the prior stay those of the measured heights (the detector's file is not touched). Tops
65.6 mm (the measured prior's own), 62, 55, 50, 45, 40; tape and opened widths; background removed by range and by the ROI.
With --lateral, instead: the top at 62 mm and range only, the two lateral gates opened one at a time (none, opening width,
centre spacer, both), so "opened widths" can be split.

  PYTHONPATH=<repo>/src python e4_smear.py DATA_DIR --json e4_smear.json [--jobs N] [--lateral]
"""

import argparse
import collections
import json
import multiprocessing
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import e4_diagnose as E  # noqa: E402

N, det = E.N, E.det
TOPS_M = (None, 0.062, 0.055, 0.050, 0.045, 0.040)
CAPTURES = ("toy_1p00m_640", "toy_1p00m_848", "toy_1p50m_640", "toy_1p50m_848", "toy30_1p00m_640", "toy30_1p00m_848")
REAL_GRID = det._column_grid


def column_grid_with_top(top):
    """det._column_grid with the band's upper limit ``top`` (None: the prior's own)."""

    def grid(points, prior, params):
        upper = prior.height_m - prior.deck_top_m - params.band_margin_m if top is None else top
        mask = (points[:, 2] >= prior.deck_bottom_m + params.band_margin_m) & (points[:, 2] <= upper)
        lateral = points[mask, 1]
        if not len(lateral):
            return np.zeros(0, dtype=int), 0
        indices = np.floor(lateral / params.cell_m + 1e-9).astype(int)
        origin = int(indices.min())
        return np.bincount(indices - origin), origin

    return grid


def run_capture(task) -> dict:
    root, name, lateral = task
    capture = Path(root) / name
    meta = json.loads((capture / "capture.json").read_text())
    kind, roi = name.split("_")[0], meta["setup"]["roi"]
    distance = float(meta["setup"]["distance_m"])
    frames = [np.asarray(Image.open(q)) for q in sorted(capture.glob("z16_*.png"))]
    scenes = [N.toy_input(meta, f)[0] for f in frames]
    fits = []
    for scene in scenes[::5]:
        grid = E.base_grid(scene).copy()
        grid[roi[1]:roi[3], roi[0]:roi[2]] = np.nan
        q = grid.reshape(-1, 3)
        fits.append(E.floor_fit(q[np.isfinite(q).all(axis=1)]))
    fit = tuple(float(np.mean([f[i] for f in fits])) for i in range(3))
    stack = np.stack(frames).astype(float)
    stack[stack == 0] = np.nan
    with E.warnings.catch_warnings():
        E.warnings.simplefilter("ignore", RuntimeWarning)
        median = np.nan_to_num(np.nanmedian(stack, axis=0)).astype(np.uint16)
    front = E.front_of(E.levelled(N.toy_input(meta, median)[0], fit), roi, distance)
    level = [E.levelled(s, fit) for s in scenes]
    rows = []
    if lateral:
        det._column_grid = column_grid_with_top(0.062)
        for gates in (False, "opening", "spacer", True):
            prior = E.measured_prior(kind, gates)
            params = E.params_for(prior, "derived", None, front[0] + 0.8)
            steps, reasons = collections.Counter(), collections.Counter()
            for s in level:
                rec = E.classify(s, prior, params, front, E.LANES_M[kind])
                steps[rec["step"]] += 1
                reasons[str(rec["reason"])] += 1
            rows.append({"crop": "range", "top_m": 0.062, "gates": gates, "steps": dict(steps), "reasons": dict(reasons)})
        det._column_grid = REAL_GRID
        return {"capture": name, "cells": rows}
    for crop in ("range", "roi"):
        group = [E.cropped(s, roi) for s in level] if crop == "roi" else level
        for top in TOPS_M:
            for open_widths in (False, True):
                prior = E.measured_prior(kind, open_widths)
                params = E.params_for(prior, "derived", None, front[0] + 0.8 if crop == "range" else None)
                det._column_grid = column_grid_with_top(top)
                steps, reasons = collections.Counter(), collections.Counter()
                for s in group:
                    rec = E.classify(s, prior, params, front, E.LANES_M[kind])
                    steps[rec["step"]] += 1
                    reasons[str(rec["reason"])] += 1
                det._column_grid = REAL_GRID
                rows.append({"crop": crop, "top_m": top, "open_widths": open_widths, "steps": dict(steps),
                             "reasons": dict(reasons)})
    return {"capture": name, "cells": rows}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("data", type=Path)
    p.add_argument("--json", type=Path, required=True)
    p.add_argument("--jobs", type=int, default=1)
    p.add_argument("--lateral", action="store_true")
    args = p.parse_args()
    with multiprocessing.Pool(args.jobs) as pool:
        out = pool.map(run_capture, [(str(args.data), n, args.lateral) for n in CAPTURES], chunksize=1)
    args.json.write_text(json.dumps({"captures": out}, indent=1) + "\n")


if __name__ == "__main__":
    main()
