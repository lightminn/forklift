#!/usr/bin/env python3
"""Single-frame bound table and fusion window for the far-range tracker (stage 2B-3 plan sections 4-5).

Reads calibration-band straight sequences recorded with --clean
(tools/recognition_sequences.py). Every counted candidate matched to a truth face
contributes its errors to its range bin and family: the per-pocket and pair-midpoint
lateral / along p95, the pair-midpoint height, yaw and support p95, the layout p95
(each pocket's coordinate along the face against the truth, and its spread inside
one sequence) and sigma, the spread of (noisy - clean) midpoint lateral offsets of
the same instance in the same frame. Candidates matched to no face (beyond the
evaluator's 0.2 m / 0.35 rad) are counted, not used. A bin with fewer than
MIN_SAMPLES samples is left out, so its observations go untracked. The window W is
then the one of WINDOWS whose V1 output yaw p95 beyond 2.0 m is smallest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml

from forklift_core.perception.candidate_tracking import TrackBounds
from tools.evaluate_track_sequences import (
    FAR_BINS,
    SPEED_SIGMA_MPS,
    STEER_SIGMA_RAD,
    SUBSTEPS,
    WHEELBASE_M,
    _bin_row,
    load_records,
    run,
)

ROOT = Path(__file__).resolve().parents[1]
RANGE_BINS = ((1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 3.5), (3.5, 4.0))
MIN_SAMPLES = 20
WINDOWS = (0.0, 0.6, 1.2, 2.0)
LAYOUT_FLOOR_M = 0.001
# The fusion weights' odometry spread: the OdometryNoise assumption on a straight 0.6 m/s approach as
# random walks (tools/evaluate_track_sequences.py V1). Assumed, not measured; the budget keeps the bound.
WEIGHT_MODEL = {"position_m_per_sqrt_s": SPEED_SIGMA_MPS * math.sqrt(0.2 / SUBSTEPS),
                "yaw_rad_per_sqrt_s": 0.6 * STEER_SIGMA_RAD / WHEELBASE_M * math.sqrt(0.2 / SUBSTEPS)}
REAR_AXLE_XY_M = (-0.34, 0.0)  # sim/models/dls08_measured/parameters.yaml rear_axle_x_m


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _family(c) -> str:
    return "parametric" if c.hypothesis.startswith("parametric") else "known"


def sample_errors(cand, face, support_z) -> dict:
    yaw = face["insertion_yaw_rad"]
    a = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-a[1], a[0]])
    left, right = np.asarray(cand.left.centre_m), np.asarray(cand.right.centre_m)
    tl, tr = np.asarray(face["left_m"]), np.asarray(face["right_m"])
    mid, tmid = (left + right) / 2, (tl + tr) / 2
    d_l, d_r, d_m = left - tl, right - tr, mid - tmid
    # each pocket's coordinate along the face, measured from the pair midpoint (layout error)
    layout = max(abs(float((left - mid)[:2] @ n - (tl - tmid)[:2] @ n)),
                 abs(float((right - mid)[:2] @ n - (tr - tmid)[:2] @ n)))
    return {
        "lateral_m": max(abs(float(d_l[:2] @ n)), abs(float(d_r[:2] @ n))),
        "along_m": max(abs(float(d_l[:2] @ a)), abs(float(d_r[:2] @ a))),
        "centre_lateral_m": abs(float(d_m[:2] @ n)),
        "centre_along_m": abs(float(d_m[:2] @ a)),
        "centre_height_m": abs(float(d_m[2])),
        "yaw_rad": abs(_wrap(cand.insertion_yaw_rad - yaw)),
        "support_m": abs(float(cand.support_z_m - support_z)),
        "layout_m": layout,
        "mid_lateral_signed": float(d_m[:2] @ n),
        "offsets": tuple(float((p - np.asarray(cand.face_centre_m))[:2] @ n) for p in (left, right)),
    }


def collect(records):
    """Per (family, bin): error samples, sigma samples and face-offset spreads."""
    samples, sigma, offsets, unmatched = defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(int)
    for rec in records:
        if rec["skipped"] or rec["kind"] != "straight":
            continue
        per_face = defaultdict(list)
        for f in rec["frames"]:
            clean_by_label = {}
            for c in f.get("clean", ()):
                lab = tuple(f["clean_labels"][c.candidate_id])
                if c.counted and lab[1] is not None:
                    clean_by_label[lab] = c
            for c in f["candidates"]:
                if not c.counted:
                    continue
                b = next((r for r in RANGE_BINS if r[0] <= c.range_m < r[1]), None)
                if b is None:
                    continue
                key = (_family(c), b)
                k, i = f["labels"][c.candidate_id]
                if i is None:
                    unmatched[key] += 1
                    continue
                face = f["faces"][k][i]
                e = sample_errors(c, face, f["support_z_m"][k])
                samples[key].append(e)
                per_face[(k, i, key)].append(e["offsets"])
                ref = clean_by_label.get((k, i))
                if ref is not None:
                    sigma[key].append(e["mid_lateral_signed"] - sample_errors(ref, face, f["support_z_m"][k])["mid_lateral_signed"])
        for (k, i, key), offs in per_face.items():
            arr = np.asarray(offs)
            offsets[key] += list(np.abs(arr - np.median(arr, axis=0)).max(axis=1))
    return samples, sigma, offsets, unmatched


def table(records) -> tuple[dict, dict]:
    samples, sigma, offsets, unmatched = collect(records)
    rows = {"known": [], "parametric": []}
    report = {}
    for family in rows:
        for b in RANGE_BINS:
            key = (family, b)
            s = samples.get(key, [])
            report[f"{family} {b[0]}-{b[1]}"] = {"samples": len(s), "sigma_samples": len(sigma.get(key, [])),
                                                 "unmatched": unmatched.get(key, 0)}
            if len(s) < MIN_SAMPLES or len(sigma.get(key, [])) < MIN_SAMPLES:
                continue

            def p95(name):
                return float(np.percentile([e[name] for e in s], 95))
            # LAYOUT_FLOOR_M is a development allowance so a constant known-shape layout cannot fail
            # the gate on rounding; larger empirical values are kept
            layout = max(p95("layout_m"), float(np.percentile(offsets.get(key, [0.0]), 95)), LAYOUT_FLOOR_M)
            rows[family].append({
                "lo_m": b[0], "hi_m": b[1], "lateral_m": p95("lateral_m"), "along_m": p95("along_m"),
                "centre_lateral_m": p95("centre_lateral_m"), "centre_along_m": p95("centre_along_m"),
                "centre_height_m": p95("centre_height_m"), "yaw_rad": p95("yaw_rad"), "support_m": p95("support_m"),
                "layout_m": layout, "sigma_m": max(float(np.std(sigma[key], ddof=1)), 1e-4),
            })
    return rows, report


def config(rows, odometry: dict, window_s: float) -> dict:
    return {
        "role": "provisional",
        "window_s": window_s, "expiry_s": 0.5, "max_observations": 10,
        "rear_axle_xy_m": list(REAR_AXLE_XY_M),
        "weight_model": dict(WEIGHT_MODEL),
        "odometry": {"source": "config/obstacle_odometry_age.json", "ages_s": odometry["ages_s"],
                     "e_m": odometry["cumulative_position_m"], "psi_rad": odometry["cumulative_yaw_rad"]},
        "rows": rows,
    }


def choose_window(records, rows, odometry) -> tuple[float, dict]:
    scores = {}
    for w in WINDOWS:
        bounds = TrackBounds.from_dict(config(rows, odometry, w))
        yaws = []
        for rec in records:
            if rec["skipped"] or rec["kind"] != "straight":
                continue
            res = run(rec, bounds, "V1", odometry, seed=rec["seed"] * 7 + 1)
            for lo, hi in FAR_BINS:
                p = _bin_row(res["rows"], lo, hi)
                if p is not None and "output" in p:
                    yaws.append(p["output"]["yaw_rad"])
        scores[w] = float(np.percentile(yaws, 95)) if yaws else math.inf
    best = min(WINDOWS, key=lambda w: (scores[w], w))
    return best, scores


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("records", type=Path)
    ap.add_argument("--odometry", type=Path, default=ROOT / "config/obstacle_odometry_age.json")
    ap.add_argument("--out", type=Path, default=ROOT / "config/recognition_track_bounds.yaml")
    args = ap.parse_args(argv)
    records = load_records(args.records)
    odometry = json.loads(args.odometry.read_text())
    rows, report = table(records)
    window, scores = choose_window(records, rows, odometry)
    digest = hashlib.sha256()
    for p in sorted(args.records.glob("*.pkl")):
        digest.update(p.name.encode())
        digest.update(p.read_bytes())
    commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    cfg = config(rows, odometry, window)
    cfg["provenance"] = {
        "records": len(records), "records_sha256": digest.hexdigest(), "commit": commit,
        "seeds": sorted({r["seed"] for r in records}), "level": sorted({r["level"] for r in records}),
        "bins": report, "window_p95_yaw_rad_beyond_2m": {str(k): v for k, v in scores.items()},
        "note": ("single-frame p95 rows from the CPU rig with team noise; empirical budgets, not probability "
                 "bounds. The odometry table is the worst-case age table of reported odometry"),
    }
    args.out.write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(json.dumps({"window_s": window, "scores": scores, "bins": report}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
