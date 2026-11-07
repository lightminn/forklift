#!/usr/bin/env python3
"""The E4 diagnosis's tables from e4_diagnose.json, and how much of the workspace the pallet's front is in a frame.

  PYTHONPATH=<repo>/src python e4_report.py e4_diagnose.json --data DATA_DIR --json e4_report.json

The front's share is measured per capture on 5 frames: the workspace of E4's call, the points within the derived inlier
of a plane fitted to the ROI's front layer. share^3 is the chance one RANSAC draw lands three points on it (the normal
check ignored), so 1 - (1 - share^3)^200 estimates one round's chance of a hypothesis there. An estimate, reported as one.
"""

import argparse
import collections
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import e4_diagnose as E  # noqa: E402

N, det = E.N, E.det
LADDER = [("nominal", "full", "provisional", "derived"), ("level", "full", "provisional", "derived"),
          ("nominal", "range", "provisional", "derived"), ("level", "range", "provisional", "derived"),
          ("nominal", "roi", "provisional", "derived"), ("level", "roi", "provisional", "derived"),
          ("level", "roi", "heights", "derived"), ("level", "roi", "open_widths", "derived"),
          ("level", "roi", "provisional", "wide"), ("level", "roi", "open_widths", "wide"),
          ("level", "range", "open_widths", "wide")]
# frame by frame, what one change does from the first cell to the second (the same frames, in order)
PAIRS = [(("nominal", "full", "provisional", "derived"), ("nominal", "range", "provisional", "derived")),
         (("nominal", "full", "provisional", "derived"), ("nominal", "roi", "provisional", "derived")),
         (("nominal", "roi", "provisional", "derived"), ("level", "roi", "provisional", "derived")),
         (("nominal", "range", "provisional", "derived"), ("level", "range", "provisional", "derived")),
         (("level", "range", "provisional", "derived"), ("level", "roi", "provisional", "derived")),
         (("level", "roi", "provisional", "derived"), ("level", "roi", "heights", "derived")),
         (("level", "roi", "provisional", "derived"), ("level", "roi", "open_widths", "derived")),
         (("level", "roi", "open_widths", "derived"), ("level", "roi", "open_widths", "wide"))]


def ladder(captures: list) -> dict:
    """Per set, (kind, distance) and ladder cell: the funnel steps, summed over both resolutions."""
    out = collections.defaultdict(dict)
    for c in captures:
        if c["kind"] == "wall":
            continue
        for r in c["rows"]:
            cell = (r["mount"], r["crop"], r["prior"], r["inlier"])
            if cell not in LADDER:
                continue
            key = f"{r['set']}/{c['kind']}/{c['distance_m']}"
            row = out[key].setdefault("/".join(cell), collections.Counter())
            row.update(r["steps"])
            row["frames"] += r["frames"]
            for reason, n in r["reasons"].items():
                row[f"reason:{reason}"] += n
    return {k: {cell: dict(v) for cell, v in cells.items()} for k, cells in out.items()}


def face_candidates(captures: list) -> dict:
    """Per ladder cell over the E4 frames: frames with a plane candidate at the front, whatever the outcome."""
    out = collections.Counter()
    for c in captures:
        for r in c["rows"]:
            if r["set"] == "e4":
                out["/".join((r["mount"], r["crop"], r["prior"], r["inlier"]))] += r.get("face_plane_frames", 0)
    return dict(out)


def transitions(captures: list) -> list:
    """Over the E4 frames, per pair: valid before and after, gained, lost, and the same by capture kind and distance."""
    rows = []
    for first, second in PAIRS:
        total = collections.Counter()
        per = collections.defaultdict(collections.Counter)
        for c in captures:
            cells = {(r["mount"], r["crop"], r["prior"], r["inlier"]): r["frame_steps"] for r in c["rows"] if r["set"] == "e4"}
            if first not in cells:
                continue
            for a, b in zip(cells[first], cells[second], strict=True):
                key = "both" if a == b == "V" else "gained" if b == "V" else "lost" if a == "V" else "neither"
                total[key] += 1
                per[f"{c['kind']}/{c['distance_m']}"][key] += 1
        rows.append({"from": "/".join(first), "to": "/".join(second), "frames": dict(total),
                     "by_bucket": {k: dict(v) for k, v in per.items()}})
    return rows


def walls(captures: list) -> dict:
    out = collections.Counter()
    for c in captures:
        if c["kind"] == "wall":
            for r in c["rows"]:
                n = r["steps"].get("valid_wrong", 0)
                out["frames"] += r["frames"]
                out["valid"] += n
                if n:
                    out[f"{r['mount']}/{r['crop']}/{r['prior']}/{r['inlier']}/{r['prior_kind']}/{c['capture']}"] += n
    return dict(out)


def front_share(data: Path, captures: list) -> list:
    rows = []
    for c in captures:
        if c["kind"] == "wall":
            continue
        name, capture = c["capture"], data / c["capture"]
        meta = json.loads((capture / "capture.json").read_text())
        prior = N.TOY_PRIORS[c["kind"]]
        range_min = 0.4 if c["distance_m"] < E.DetectorParams().range_min_m else None
        params = E.params_for(prior, "derived", range_min)
        x0, y0, x1, y1 = meta["setup"]["roi"]
        shares, sizes = [], []
        for path in sorted(capture.glob("z16_*.png"))[:5]:
            scene = N.toy_input(meta, np.asarray(Image.open(path)))[0]
            camera = scene.base_from_optical.translation_m
            grid = E.base_grid(scene)
            workspace = det._filter_workspace(grid.reshape(-1, 3), camera, prior, params)
            roi = grid[y0:y1, x0:x1].reshape(-1, 3)
            roi = roi[np.isfinite(roi).all(axis=1) & (roi[:, 2] > 0.012) & (roi[:, 0] < c["sensor"]["front_m"] + 0.04)]
            normal = np.array((1.0, 0.0, 0.0))
            offset = float(np.median(roi[:, 0]))
            inside = np.abs(workspace @ normal - offset) <= params.plane_inlier_m
            near = np.abs(workspace[:, 1]) < 0.35
            shares.append(float((inside & near).mean()))
            sizes.append(len(workspace))
        share = float(np.median(shares))
        rows.append({"capture": name, "workspace": int(np.median(sizes)), "front_share": share,
                     "one_round_hypothesis_estimate": 1 - (1 - share**3) ** params.ransac_iterations})
    return rows


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("diagnosis", type=Path)
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    captures = json.loads(args.diagnosis.read_text())["captures"]
    sensor = {c["capture"]: {**c["sensor"], "floor": c["floor"]} for c in captures if c["kind"] != "wall"}
    wall_floor = {c["capture"]: c["floor"] for c in captures if c["kind"] == "wall"}
    out = {"ladder_cells": ["/".join(c) for c in LADDER], "ladder": ladder(captures), "walls": walls(captures),
           "face_candidates": face_candidates(captures), "transitions": transitions(captures),
           "sensor": sensor, "wall_floor": wall_floor, "front_share": front_share(args.data, captures)}
    args.json.write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()
