#!/usr/bin/env python3
"""Run the far-range tracker on recorded approach sequences and score it (stage 2B-3 plan section 7).

Records come from tools/recognition_sequences.py. Each variant builds the odometry
the tracker reads (V0 perfect, V1 integrated OdometryNoise-like noise, V4 read 30 ms
late, V5 the worst constant drift inside every window of the odometry table), V3
drops the target's candidates in three gaps, then the tracker runs frame by frame.
Scoring uses the records' oracle labels, which the tracker never sees.
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
from collections import defaultdict
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from forklift_core.geometry import RigidTransform
from forklift_core.perception.candidate_tracking import CandidateTracker, Frame, TrackBounds

ROOT = Path(__file__).resolve().parents[1]
SUBSTEPS = 24  # 120 Hz odometry samples per 0.2 s frame
WHEEL_RADIUS_M, WHEELBASE_M = 0.125, 0.66
# OdometryNoise: wheel rate sigma 0.2 rad/s and steering sigma 0.005 rad per sample on two wheels each;
# the forward speed and the steering angle each average two independent draws.
SPEED_SIGMA_MPS = 0.2 * WHEEL_RADIUS_M / math.sqrt(2)
STEER_SIGMA_RAD = 0.005 / math.sqrt(2)
READ_DELAY_S = 0.03
DROP_GAPS = ((3.0, 1), (2.4, 2), (1.8, 3))  # (camera-face distance, frames dropped) for V3
BINS = ((3.0, 3.5), (2.5, 3.0), (2.0, 2.5), (1.5, 2.0), (1.3, 1.5))
FAR_BINS = ((3.0, 3.5), (2.5, 3.0), (2.0, 2.5))
VARIANTS = {"V0": ("straight", "perfect"), "V1": ("straight", "noise"), "V2": ("curve", "noise"),
            "V3": ("straight", "noise"), "V4": ("pitch", "late"), "V5": ("straight", "drift")}
JUDGED = ("V0", "V1", "V2")
FRONT_BACK_SIDE = ("S2", "S3", "S6")
AXES = ("lateral_m", "along_m", "yaw_rad", "height_m")
TOLERANCE = {"lateral_m": 0.002, "along_m": 0.002, "yaw_rad": math.radians(0.2), "height_m": 0.002}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _transform(x, y, yaw) -> RigidTransform:
    c, s = math.cos(yaw), math.sin(yaw)
    return RigidTransform("base_link", "odom", ((c, -s, 0), (s, c, 0), (0, 0, 1)), (x, y, 0.0))


def _substeps(poses):
    """True poses at 120 Hz, one extra frame extrapolated past the end (for the late read)."""
    p = [np.array(q, float) for q in poses]
    p.append(p[-1] + (p[-1] - p[-2]))
    out = []
    for a, b in zip(p[:-1], p[1:]):
        d = b - a
        d[2] = _wrap(d[2])
        out += [a + d * (k / SUBSTEPS) for k in range(SUBSTEPS)]
    out.append(p[-1])
    return out


def odometry(poses, mode: str, seed: int, table: dict | None = None):
    """Reported odom poses (x, y, yaw) at each frame time; odom starts on the true first pose."""
    if mode == "perfect":
        return [tuple(q) for q in poses]
    if mode == "drift":
        rate_p = min(e / a for a, e in zip(table["ages_s"], table["cumulative_position_m"]))
        rate_y = min(e / a for a, e in zip(table["ages_s"], table["cumulative_yaw_rad"]))
        y0 = poses[0][2]
        n0 = (-math.sin(y0), math.cos(y0))
        out = []
        for k, (x, y, yaw) in enumerate(poses):
            t = 0.2 * k
            out.append((x + rate_p * t * n0[0], y + rate_p * t * n0[1], _wrap(yaw + rate_y * t)))
        return out
    rng = np.random.default_rng(seed)
    true = _substeps(poses)
    dt = 0.2 / SUBSTEPS
    est = [np.array(true[0], float)]
    for a, b in zip(true[:-1], true[1:]):
        c, s = math.cos(a[2]), math.sin(a[2])
        dx, dy = b[0] - a[0], b[1] - a[1]
        fwd, side = c * dx + s * dy, -s * dx + c * dy
        dyaw = _wrap(b[2] - a[2])
        v = fwd / dt
        fwd += rng.normal(0, SPEED_SIGMA_MPS) * dt
        dyaw += abs(v) * rng.normal(0, STEER_SIGMA_RAD) / WHEELBASE_M * dt
        e = est[-1]
        ce, se = math.cos(e[2]), math.sin(e[2])
        est.append(np.array([e[0] + ce * fwd - se * side, e[1] + se * fwd + ce * side, _wrap(e[2] + dyaw)]))
    if mode == "noise":
        return [tuple(est[k * SUBSTEPS]) for k in range(len(poses))]
    if mode == "late":
        shift = READ_DELAY_S / dt
        out = []
        for k in range(len(poses)):
            i = k * SUBSTEPS + shift
            lo = int(math.floor(i))
            f = i - lo
            a, b = est[lo], est[min(lo + 1, len(est) - 1)]
            d = b - a
            d[2] = _wrap(d[2])
            out.append(tuple(a + f * d))
        return out
    raise ValueError(mode)


def drift_problem(poses, table) -> str | None:
    """Why the V5 drift leaves the table in some window, or None (checked at every frame pair)."""
    reported = odometry(poses, "drift", 0, table)
    ages, pos, yaw = table["ages_s"], table["cumulative_position_m"], table["cumulative_yaw_rad"]
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            age = 0.2 * (j - i)
            k = next((m for m, a in enumerate(ages) if a >= age - 1e-9), None)
            if k is None:
                continue
            ep = math.hypot(reported[j][0] - poses[j][0] - (reported[i][0] - poses[i][0]),
                            reported[j][1] - poses[j][1] - (reported[i][1] - poses[i][1]))
            ey = abs(_wrap(reported[j][2] - poses[j][2] - (reported[i][2] - poses[i][2])))
            if ep > pos[k] + 1e-12 or ey > yaw[k] + 1e-12:
                return f"frames {i}-{j}: {ep:.4f} m / {ey:.4f} rad over {age:.1f} s"
    return None


def _distance(face, camera_x=0.75) -> float:
    yaw = face["insertion_yaw_rad"]
    mid = (np.asarray(face["left_m"][:2]) + np.asarray(face["right_m"][:2])) / 2
    return float((mid - np.array([camera_x, 0.0])) @ np.array([math.cos(yaw), math.sin(yaw)]))


def errors(cand, face) -> dict:
    """Worst-pocket lateral / along, yaw and height of a candidate against a truth face."""
    yaw = face["insertion_yaw_rad"]
    a = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-a[1], a[0]])
    out = {"lateral_m": 0.0, "along_m": 0.0, "height_m": 0.0}
    for side, key in (("left", "left_m"), ("right", "right_m")):
        d = np.subtract(getattr(cand, side).centre_m, face[key])
        out["lateral_m"] = max(out["lateral_m"], abs(float(d[:2] @ n)))
        out["along_m"] = max(out["along_m"], abs(float(d[:2] @ a)))
        out["height_m"] = max(out["height_m"], abs(float(d[2])))
    out["yaw_rad"] = abs(_wrap(cand.insertion_yaw_rad - yaw))
    return out


def identity_errors(labels_of: dict) -> int:
    """Tracks that accepted another instance (any accepted reading) or another face of one instance
    (readings matched to a face); an unmatched reading of the same instance is not a switch."""
    return sum(1 for labels in labels_of.values()
               if len({lab[0] for lab in labels}) > 1 or len({lab for lab in labels if lab[1] is not None}) > 1)


def _drop_frames(seq) -> set:
    k, i = seq["target"]
    dist = [_distance(f["faces"][k][i]) for f in seq["frames"]]
    out = set()
    for d0, n in DROP_GAPS:
        start = min(range(len(dist)), key=lambda j: abs(dist[j] - d0))
        out.update(range(start, min(start + n, len(dist))))
    return out


def run(seq, bounds: TrackBounds, variant: str, table: dict, seed: int) -> dict:
    """One variant over one sequence: per-frame states and the tracker's counters."""
    _, mode = VARIANTS[variant]
    target = tuple(seq["target"])
    poses = [f["pose"] for f in seq["frames"]]
    reported = odometry(poses, mode, seed, table)
    drops = _drop_frames(seq) if variant == "V3" else set()
    tr = CandidateTracker(bounds)
    labels_of: dict = defaultdict(list)
    selected, rows = None, []
    for j, f in enumerate(seq["frames"]):
        stamp = int(round(f["t_s"] * 1e9))
        cands = [replace(c, stamp_ns=stamp) for c in f["candidates"]
                 if not (j in drops and tuple(f["labels"][c.candidate_id]) == target)]
        odom = _transform(*reported[j])
        report = tr.update(Frame(stamp, "sim", 0, odom, tuple(cands)))
        for cid, tid in report.accepted.items():
            labels_of[tid].append(tuple(f["labels"][cid]))
        if selected is None:
            selected = next((tid for cid, tid in sorted(report.accepted.items())
                             if tuple(f["labels"][cid]) == target), None)
        live = {t.track_id: t for t in tr.tracks(stamp, odom)}
        face = f["faces"][target[0]][target[1]]
        on_target = [tid for tid in live if labels_of[tid] and labels_of[tid][-1] == target]
        retained = selected in live and labels_of[selected][-1] == target
        single = next((c for c in cands if c.counted and tuple(f["labels"][c.candidate_id]) == target), None)
        row = {"frame": j, "distance_m": _distance(face), "retained": retained,
               "reacquired": (not retained) and bool(on_target), "single": single is not None,
               "duplicates": max(0, len(on_target) - 1), "held": report.held, "initialised": selected is not None}
        if retained:
            out = live[selected]
            row["output"] = errors(out.candidate, face)
            row["budget"] = dict(out.budget)
            row["observations"] = out.observations
        if single is not None:
            row["latest"] = errors(single, face)
        rows.append(row)
    identity = identity_errors(labels_of)
    unmatched = sum(1 for labels in labels_of.values() for lab in labels if lab[1] is None)
    resets = sum(t.resets for t in tr.tracks(int(round(seq["frames"][-1]["t_s"] * 1e9)),
                                             _transform(*reported[-1])))
    return {"scene_id": seq["scene_id"], "case": seq["case"], "shape_key": seq["shape_key"], "variant": variant,
            "rows": rows, "identity_errors": identity, "unmatched_accepts": unmatched, "resets_live": resets}


def _bin_row(rows, lo, hi):
    centre = (lo + hi) / 2
    inside = [r for r in rows if lo <= r["distance_m"] < hi]
    return min(inside, key=lambda r: abs(r["distance_m"] - centre)) if inside else None


def _p95(values):
    return float(np.percentile(values, 95)) if values else None


def summarise(results: list, n_skipped: int) -> dict:
    """Per-variant tables and the fixed development criteria (plan section 7)."""
    out = {}
    for variant in VARIANTS:
        res = [r for r in results if r["variant"] == variant]
        if not res:
            continue
        n = len(res)
        table = []
        for lo, hi in BINS:
            picked = [(_bin_row(r["rows"], lo, hi), r) for r in res]
            retained = [p for p, _ in picked if p is not None and p["retained"]]
            both = [p for p in retained if "latest" in p]
            cell = {"bin": [lo, hi], "sequences": n,
                    "retained": len(retained), "reacquired": sum(1 for p, _ in picked if p and p["reacquired"]),
                    "single_available": sum(1 for p, _ in picked if p and p["single"]),
                    "both": len(both)}
            for axis in AXES:
                cell[f"output_{axis}"] = _p95([p["output"][axis] for p in both])
                cell[f"latest_{axis}"] = _p95([p["latest"][axis] for p in both])
            cell["cases"] = {}
            for case in FRONT_BACK_SIDE:  # judged one case at a time (plan section 7)
                sub = [(p, r) for p, r in picked if r["case"] == case]
                cell["cases"][case] = {"sequences": len(sub),
                                       "retained": sum(1 for p, _ in sub if p and p["retained"]),
                                       "single_available": sum(1 for p, _ in sub if p and p["single"])}
            table.append(cell)
        after = [p for r in res for lo, hi in BINS for p in [_bin_row(r["rows"], lo, hi)] if p and p["initialised"]]
        coverage = {axis: None for axis in AXES}
        frames = [row for r in res for row in r["rows"] if "output" in row]
        if frames:
            coverage = {axis: float(np.mean([row["output"][axis] <= row["budget"][axis] + 1e-12 for row in frames]))
                        for axis in AXES}
        summary = {
            "sequences": n, "skipped": n_skipped, "bins": table,
            "identity_errors": sum(r["identity_errors"] for r in res),
            "duplicate_frames": sum(row["duplicates"] > 0 for r in res for row in r["rows"]),
            "held_frames": sum(row["held"] > 0 for r in res for row in r["rows"]),
            "retention_after_init": (sum(p["retained"] for p in after) / len(after)) if after else None,
            "coverage": coverage,
            "initialised_m": [next((row["distance_m"] for row in r["rows"] if row["initialised"]), None) for r in res],
        }
        if variant in JUDGED:
            summary["verdict"] = verdict(summary)
        out[variant] = summary
    return out


def verdict(s: dict) -> dict:
    fails = []
    if s["identity_errors"]:
        fails.append(f"identity errors {s['identity_errors']}")
    if s["duplicate_frames"]:
        fails.append(f"duplicate target tracks in {s['duplicate_frames']} frames")
    for cell in s["bins"]:
        b = cell["bin"]
        if cell["retained"] < cell["single_available"] - 0.05 * cell["sequences"]:
            fails.append(f"bin {b}: retained {cell['retained']} < single {cell['single_available']} - 5 %p")
        for case, c in cell["cases"].items():
            if c["sequences"] and c["retained"] < c["single_available"] - 0.05 * c["sequences"]:
                fails.append(f"bin {b}: {case} retained {c['retained']} < single {c['single_available']} - 5 %p")
        if cell["both"] < 10:
            fails.append(f"bin {b}: {cell['both']} sequences with both outputs (< 10)")
            continue
        for axis in AXES:
            if cell[f"output_{axis}"] > cell[f"latest_{axis}"] + TOLERANCE[axis]:
                fails.append(f"bin {b}: {axis} output p95 above latest + tolerance")
        if tuple(b) in FAR_BINS:
            for axis in ("yaw_rad", "along_m"):
                if cell[f"output_{axis}"] > 0.8 * cell[f"latest_{axis}"]:
                    fails.append(f"bin {b}: {axis} output p95 above 0.8 x latest")
    if s["retention_after_init"] is None or s["retention_after_init"] < 0.95:
        fails.append(f"retention after initialisation {s['retention_after_init']}")
    for axis, value in s["coverage"].items():
        if value is None or value < 0.90:
            fails.append(f"budget coverage {axis} {value}")
    return {"pass": not fails, "failures": fails}


def load_records(directory: Path, kinds=None):
    recs = []
    for p in sorted(directory.glob("*.pkl")):
        with open(p, "rb") as fh:
            rec = pickle.load(fh)
        if kinds is None or rec["kind"] in kinds:
            recs.append(rec)
    return recs


def evaluate(records, bounds: TrackBounds, table: dict, variants=tuple(VARIANTS)) -> dict:
    results, skipped = [], defaultdict(int)
    for rec in records:
        for variant in variants:
            kind, _ = VARIANTS[variant]
            if rec["kind"] != kind:
                continue
            if rec["skipped"]:
                skipped[variant] += 1
                continue
            results.append(run(rec, bounds, variant, table, seed=rec["seed"] * 7 + list(VARIANTS).index(variant)))
    return {"summary": {v: s for v, s in summarise(results, 0).items()},
            "skipped": dict(skipped), "results": results}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("records", type=Path)
    ap.add_argument("--bounds", type=Path, default=ROOT / "config/recognition_track_bounds.yaml")
    ap.add_argument("--odometry", type=Path, default=ROOT / "config/obstacle_odometry_age.json")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    bounds = TrackBounds.load(args.bounds)
    table = json.loads(args.odometry.read_text())
    report = evaluate(load_records(args.records), bounds, table, tuple(args.variants.split(",")))
    for variant, s in report["summary"].items():
        s["skipped"] = report["skipped"].get(variant, 0)
    args.out.write_text(json.dumps({"summary": report["summary"], "results": report["results"]}, indent=1))
    for variant, s in report["summary"].items():
        print(variant, json.dumps(s.get("verdict")), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
