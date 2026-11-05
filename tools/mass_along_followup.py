"""Records-only follow-up to the pallet mass mechanism diagnosis
(docs/plans/2026-11-06-mass-along-followup-intervals-map.md): S2 compares the loaded-transport over-rotation of the
two arms inside like-for-like windows (direction, turn, curvature, speed, steadiness); S1 aligns the SLAM's published
map to truth scan endpoints around the delivery reference. An exploration after the results were read: it describes,
without thresholds or causes.

    python tools/mass_along_followup.py OUT VERDICT_JSON --json FOLLOWUP --precision PRECISION [--csv-windows WINDOWS]
    python tools/mass_along_followup.py OUT VERDICT_JSON --json FOLLOWUP --no-map   # S2 only

OUT holds the run folders ``mae_{A|B}_{scene}_s{seed}`` (``run/`` and ``bridge/``); VERDICT_JSON is the registered
tool's verdict (tools/mass_along_judge.py, #319) for the pair list and the split times."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mass_along_judge as J  # noqa: E402
import mass_along_mechanism as M  # noqa: E402

# ---------------------------------------------------------------- S2 constants (plan section 2)
WINDOW_S = 0.25
MIN_ABS_TRAVEL_M = 0.02
ONE_WAY_SHARE = 0.95
MIN_MEAN_CURVATURE = 0.05  # 1/m
MIN_PREDICTED_ROTATION = 0.002  # rad
CURVATURE_BINS = (0.05, 0.15, 0.30, 0.60, 1.0)  # lower edges; the last bin is open
SPEED_BINS = (0.08, 0.15, 0.30)  # m/s, lower edges; the last bin is open
STEADY_CURVATURE_RANGE = 0.05  # 1/m: max - min of the step curvatures inside the window
STEADY_SPEED_RANGE = 0.05  # m/s: max - min of the four quarter-window mean speeds
COMMON_CELL_TRAVEL_M = 0.3


def wheel_curvatures(delta_l: np.ndarray, delta_r: np.ndarray, wheelbase: float, track: float) -> tuple[np.ndarray, np.ndarray]:
    """Each front wheel's own Ackermann curvature (their mean is M.kinematic_curvature)."""
    tl, tr = np.tan(delta_l), np.tan(delta_r)
    return tl / (wheelbase + track * tl / 2.0), tr / (wheelbase - track * tr / 2.0)


def bin_index(value: float, edges: tuple[float, ...]) -> int | None:
    if value < edges[0]:
        return None
    return int(np.searchsorted(edges, value, side="right") - 1)


def transport_windows(log: dict, control: np.ndarray, window, wheelbase: float, track: float,
                      lift: tuple[np.ndarray, np.ndarray] | None = None) -> dict:
    """S2 windows: fixed WINDOW_S windows over the transport (the last partial window dropped), each with the per-sample
    D3 quantities of #321 (trapezoid predicted rotation from the actual and the measured steering, truth rotation) and
    the conditions the bins and the balance need. Steps belong to the window of their end sample, as in #321 D3.
    Returns the windows and the rolled distance of the whole transport."""
    empty = {"windows": [], "rolled_m": 0.0, "steps": 0}
    if window is None:
        return empty
    t = np.asarray(log["joint_stamps_s"], float)
    sel = np.nonzero((t >= window[0]) & (t <= window[1]))[0]
    if len(sel) < 3:
        return empty
    ts = t[sel]
    ds = M.signed_steps(control, ts)
    yaw = np.interp(ts, control[:, 0], np.unwrap(control[:, 6]))
    dpsi = np.diff(yaw)
    dt = np.diff(ts)
    st = np.asarray(log["steering_rad"], float)[sel]
    k = M.kinematic_curvature(st[:, 0], st[:, 1], wheelbase, track)
    k_mid = 0.5 * (k[1:] + k[:-1])
    pred = k_mid * ds
    if "steering_measured_rad" in log:
        sm = np.asarray(log["steering_measured_rad"], float)[sel]
        km = M.kinematic_curvature(sm[:, 0], sm[:, 1], wheelbase, track)
        pred_m = 0.5 * (km[1:] + km[:-1]) * ds
    else:
        pred_m = None
    kl, kr = wheel_curvatures(st[:, 0], st[:, 1], wheelbase, track)
    lr = 0.5 * ((kl - kr)[1:] + (kl - kr)[:-1])
    win = np.floor((ts[1:] - window[0]) / WINDOW_S).astype(int)
    complete = int(math.floor((window[1] - window[0]) / WINDOW_S + 1e-9))  # windows [0, complete) end inside the transport
    out = []
    for w in range(complete):
        m = win == w
        n = int(np.count_nonzero(m))
        if n < 2:
            continue
        dsw, dtw = ds[m], dt[m]
        fwd, rev = float(np.sum(dsw[dsw > 0])), float(-np.sum(dsw[dsw < 0]))
        travel = fwd + rev
        length = float(np.sum(dtw))
        p = float(np.sum(pred[m]))
        res = float(np.sum(dpsi[m] - pred[m]))
        speed = np.abs(dsw) / np.where(dtw > 0, dtw, np.nan)
        q = max(1, n // 4)
        quarters = [float(np.nanmean(part)) for part in np.array_split(speed, 4) if len(part)]
        row = {"t0": float(window[0] + w * WINDOW_S), "length_s": length, "fwd_m": fwd, "rev_m": rev, "travel_m": travel,
               "kappa_mean": float(np.mean(k_mid[m])), "kappa_change": float(k_mid[m][-1] - k_mid[m][0]),
               "speed_mean": travel / length if length > 0 else 0.0,
               "speed_change": float(np.nanmean(speed[-q:]) - np.nanmean(speed[:q])),
               "kappa_range": float(np.max(k_mid[m]) - np.min(k_mid[m])),
               "speed_range": float(max(quarters) - min(quarters)) if quarters else 0.0,
               "lr_mismatch": float(np.mean(lr[m])), "predicted_rad": p, "residual_rad": res}
        if pred_m is not None:
            row["predicted_measured_rad"] = float(np.sum(pred_m[m]))
            row["residual_measured_rad"] = float(np.sum(dpsi[m] - pred_m[m]))
        if lift is not None and len(lift[0]):
            mid = ts[1:][m]
            ok = np.isfinite(lift[1])
            row["lift_m"] = float(np.mean(np.interp(mid, lift[0][ok], lift[1][ok]))) if np.any(ok) else None
        else:
            row["lift_m"] = None
        row["direction"] = "fwd" if travel > 0 and fwd >= ONE_WAY_SHARE * travel else (
            "rev" if travel > 0 and rev >= ONE_WAY_SHARE * travel else "mixed")
        row["signed"] = abs(p) >= MIN_PREDICTED_ROTATION
        row["included"] = (travel >= MIN_ABS_TRAVEL_M and row["direction"] != "mixed"
                           and abs(row["kappa_mean"]) >= MIN_MEAN_CURVATURE and row["signed"])
        if row["signed"]:
            row["over_rad"] = res * math.copysign(1.0, p)
            if pred_m is not None and abs(row["predicted_measured_rad"]) >= MIN_PREDICTED_ROTATION:
                row["over_measured_rad"] = row["residual_measured_rad"] * math.copysign(1.0, row["predicted_measured_rad"])
        row["turn"] = "left" if p > 0 else "right"
        # small range inside the window, not only a small net change (an excursion and back has a net change of 0)
        row["steady"] = row["kappa_range"] < STEADY_CURVATURE_RANGE and row["speed_range"] < STEADY_SPEED_RANGE
        row["cell"] = cell_key(row) if row["included"] else None
        out.append(row)
    return {"windows": out, "rolled_m": float(np.sum(np.abs(ds))), "steps": int(len(ds)),
            "residual_signed_sum_rad": float(np.sum(dpsi - pred))}


def cell_key(w: dict) -> str | None:
    kb, vb = bin_index(abs(w["kappa_mean"]), CURVATURE_BINS), bin_index(w["speed_mean"], SPEED_BINS)
    if kb is None or vb is None:
        return None
    return f"{w['direction']}|{w['turn']}|k{kb}|v{vb}|{'steady' if w['steady'] else 'transient'}"


BALANCE = ("kappa_mean", "speed_mean", "kappa_change", "lr_mismatch", "lift_m")


def cells(windows: list[dict]) -> dict[str, dict]:
    """Per cell: travel, over-rotation sum, the index per metre and the travel-weighted means of the balance values."""
    out: dict[str, dict] = {}
    for w in windows:
        if not w.get("included") or w.get("cell") is None:
            continue
        c = out.setdefault(w["cell"], {"travel_m": 0.0, "over_rad": 0.0, "windows": 0, **{f"_{b}": 0.0 for b in BALANCE},
                                       **{f"_{b}_w": 0.0 for b in BALANCE}})
        c["travel_m"] += w["travel_m"]
        c["over_rad"] += w["over_rad"]
        c["windows"] += 1
        for b in BALANCE:
            if w.get(b) is not None:
                c[f"_{b}"] += w[b] * w["travel_m"]
                c[f"_{b}_w"] += w["travel_m"]
    for c in out.values():
        c["index_rad_per_m"] = c["over_rad"] / c["travel_m"] if c["travel_m"] > 0 else None
        for b in BALANCE:
            wsum = c.pop(f"_{b}_w")
            tot = c.pop(f"_{b}")
            c[b] = tot / wsum if wsum > 0 else None
    return out


def per_metre(windows: list[dict], pick) -> tuple[float, float | None]:
    sel = [w for w in windows if pick(w)]
    d = sum(w["travel_m"] for w in sel)
    return d, (sum(w["over_rad"] for w in sel) / d if d > 0 else None)


def compare_arms(a: dict, b: dict, steady_only: bool = False) -> dict:
    """One pair: common cells (both arms at least COMMON_CELL_TRAVEL_M), the standardized difference (cell differences
    weighted by the smaller travel), the per-metre index restricted to the common cells, the same-index whole
    difference over every included window, and the staged retention per arm."""
    keep = (lambda w: w.get("steady")) if steady_only else (lambda w: True)
    wa = [w for w in a["windows"] if keep(w)]
    wb = [w for w in b["windows"] if keep(w)]
    ca, cb = cells(wa), cells(wb)
    common = sorted(c for c in set(ca) & set(cb)
                    if ca[c]["travel_m"] >= COMMON_CELL_TRAVEL_M and cb[c]["travel_m"] >= COMMON_CELL_TRAVEL_M)
    out = {"common_cells": common}
    if common:
        weights = [min(ca[c]["travel_m"], cb[c]["travel_m"]) for c in common]
        diffs = [cb[c]["index_rad_per_m"] - ca[c]["index_rad_per_m"] for c in common]
        out["standardized_d"] = float(np.average(diffs, weights=weights))
        out["cell_rows"] = [{"cell": c, "A": ca[c], "B": cb[c], "d": d, "weight_m": wt}
                            for c, d, wt in zip(common, diffs, weights)]
    else:
        out["standardized_d"] = None  # missing, not zero
    cs = set(common)
    overlap = sum(min(ca[c]["travel_m"], cb[c]["travel_m"]) for c in common)
    for arm, ws, src in (("A", wa, a), ("B", wb, b)):
        signed_d, signed_i = per_metre(ws, lambda w: w.get("signed"))
        inc_d, inc_i = per_metre(ws, lambda w: w.get("included"))
        com_d, com_i = per_metre(ws, lambda w: w.get("included") and w.get("cell") in cs)
        out_d, out_i = per_metre(ws, lambda w: w.get("included") and w.get("cell") not in cs)
        out[arm] = {"rolled_m": src["rolled_m"], "signed_m": signed_d, "signed_index": signed_i,
                    "included_m": inc_d, "included_index": inc_i, "common_m": com_d, "common_index": com_i,
                    "outside_m": out_d, "outside_index": out_i,
                    "common_share_of_rolled": com_d / src["rolled_m"] if src["rolled_m"] > 0 else None,
                    "overlap_share_of_rolled": overlap / src["rolled_m"] if src["rolled_m"] > 0 else None}
    out["overlap_m"] = overlap
    ia, ib = out["A"]["included_index"], out["B"]["included_index"]
    out["whole_d"] = ib - ia if ia is not None and ib is not None else None
    ra, rb = out["A"]["common_index"], out["B"]["common_index"]
    out["common_aggregate_d"] = rb - ra if ra is not None and rb is not None else None
    sd, wd = out["standardized_d"], out["whole_d"]
    # the overlap sum_c min(d_A, d_B), not the common-cell share: a cell common to both arms may hold most of one
    # arm's distance and little of the other's (independent plan review P2)
    shares = [out[x]["overlap_share_of_rolled"] for x in ("A", "B")]
    out["observed_in_cells"] = bool(sd is not None and wd is not None and sd * wd > 0
                                    and all(s is not None and s >= 0.5 for s in shares))
    return out


def scene_summary(rows: list[dict], key: str) -> dict:
    vals = [r[key]["standardized_d"] for r in rows if r.get(key) and r[key]["standardized_d"] is not None]
    whole = [r[key]["whole_d"] for r in rows if r.get(key) and r[key]["whole_d"] is not None]
    agree = [r for r in rows if r.get(key) and r[key]["standardized_d"] is not None and r[key]["whole_d"] is not None]
    return {"pairs": len(rows), "with_common_cells": len(vals),
            "standardized_median": float(np.median(vals)) if vals else None,
            "standardized_range": [float(min(vals)), float(max(vals))] if vals else None,
            "standardized_positive": sum(v > 0 for v in vals),
            "whole_median": float(np.median(whole)) if whole else None,
            "whole_positive": sum(v > 0 for v in whole),
            "sign_agree": sum(r[key]["standardized_d"] * r[key]["whole_d"] > 0 for r in agree),
            "observed_in_cells": sum(bool(r[key]["observed_in_cells"]) for r in rows if r.get(key))}


# ---------------------------------------------------------------- run loading


def load_run(folder: Path, t_m: float | None) -> dict:
    run = folder / "run"
    result = json.loads((run / "result.json").read_text())
    control = np.load(run / "slam_control.npy")
    z = np.load(run / "slam_log.npz")
    log = {k: z[k] for k in z.files}
    geom = json.loads((run / "meta.json").read_text())["odometry_geometry"]
    reached = t_m is not None
    if t_m is None:
        r1 = J.first_round1(result)
        t_m = float(r1["time_s"]) if r1 else None
        reached = t_m is not None
    windows = M.phase_windows(result, t_m, float(control[-1, 0]))
    samples = result.get("samples") or []
    lift = (np.array([float(s["time_s"]) for s in samples]),
            np.array([float(s["lift_m"]) if s.get("lift_m") is not None else np.nan for s in samples])) if samples else None
    s2 = transport_windows(log, control, windows["transport"], float(geom["wheelbase_m"]), float(geom["track_m"]), lift)
    return {"run": folder.name, "reached": reached, "failure": result.get("failure_reason"),
            "transport": windows["transport"], "s2": s2, "_folder": folder, "_t_m": t_m}


def followup(out_dir: Path, verdict: dict, with_map: bool = True, precision: dict | None = None) -> dict:
    rows, runs = [], {}
    for p in verdict["pairs"]:
        sp = p.get("split") or {}
        got = {}
        for arm in J.ARMS:
            name = f"mae_{arm}_{p['scene']}_s{p['seed']}"
            folder = out_dir / name
            if not (folder / "run" / "result.json").exists():
                continue
            runs[name] = got[arm] = load_run(folder, (sp.get(arm) or {}).get("t_m"))
        if set(got) != set(J.ARMS):
            continue
        a, b = got["A"], got["B"]
        row = {"scene": p["scene"], "seed": p["seed"], "primary": bool(p.get("primary")),
               "reached": {"A": a["reached"], "B": b["reached"]},
               "s2": compare_arms(a["s2"], b["s2"]), "s2_steady": compare_arms(a["s2"], b["s2"], steady_only=True)}
        if with_map:
            import mass_along_map as MAP  # noqa: PLC0415 -- S1, kept apart so S2 runs without the map stack

            row["s1"] = MAP.pair_alignment(a, b, sp, precision)
        rows.append(row)
    summary = {}
    for scene in J.SCENES:
        both = [r for r in rows if r["scene"] == scene and r["reached"]["A"] and r["reached"]["B"]]
        partial = [r for r in rows if r["scene"] == scene and not (r["reached"]["A"] and r["reached"]["B"])]
        summary[scene] = {"s2": scene_summary(both, "s2"), "s2_steady": scene_summary(both, "s2_steady"),
                          "partial_pairs": [{"seed": r["seed"], "reached": r["reached"], "standardized_d": r["s2"]["standardized_d"],
                                             "whole_d": r["s2"]["whole_d"]} for r in partial]}
    return {"pairs": rows, "summary": summary,
            "_windows": {k: v["s2"]["windows"] for k, v in runs.items()},
            "runs": {k: {"reached": v["reached"], "failure": v["failure"], "transport": v["transport"],
                         "rolled_m": v["s2"]["rolled_m"], "residual_signed_sum_rad": v["s2"].get("residual_signed_sum_rad")}
                     for k, v in runs.items()}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("verdict", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--csv-windows", type=Path)
    ap.add_argument("--no-map", action="store_true", help="S2 only")
    ap.add_argument("--precision", type=Path, help="tools/mass_along_map.py precision output (delta, method floor)")
    args = ap.parse_args(argv)
    if not args.no_map and args.precision is None:
        ap.error("S1 needs --precision (run tools/mass_along_map.py precision first)")
    prec = json.loads(args.precision.read_text()) if args.precision else None
    res = followup(args.out, json.loads(args.verdict.read_text()), with_map=not args.no_map, precision=prec)
    windows = res.pop("_windows")
    args.json.write_text(json.dumps(res, indent=1, default=str) + "\n")
    if args.csv_windows:
        keys = sorted({k for ws in windows.values() for w in ws for k in w})
        with args.csv_windows.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["run", *keys])
            w.writeheader()
            for run, ws in sorted(windows.items()):
                for x in ws:
                    w.writerow({"run": run, **x})
    print(json.dumps(res["summary"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
