"""Pixel lag of the live runner's camera (M-c plan C5), from the marker run: the lift (carrying the camera) follows a
known wave, so the picture moves up and down with the lift height at the time its pixels were drawn. Each frame's
vertical shift against the first (row-brightness profile, best correlation) is fitted linearly to the logged lift
height at (stamp - tau) for every tau on a grid, and the tau with the smallest residual is the pixel lag behind
the frame's stamp (the render metadata's sim time). A bootstrap over the frames gives its spread, and the frames on the
steep half of the wave give a per-frame lag (each frame's own best shift against the fitted line).

  python tools/live_marker_lag.py --report RUN/probe.json --frames-dir RUN/frames --live-log RUN/run/live_log.jsonl \
      --factor 0.1
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


def row_profile(path: Path) -> np.ndarray:
    """The picture's mean brightness per row, as a vertical gradient (the parts that move with the camera)."""
    grey = np.asarray(Image.open(path).convert("L"), dtype=float)
    return np.diff(grey.mean(axis=1))


def vertical_shift(profile: np.ndarray, reference: np.ndarray, max_shift: int = 40) -> float:
    """The rows the picture moved down against the reference (best correlation, sub-row by a parabola)."""
    n = len(reference)
    scores = []
    for k in range(-max_shift, max_shift + 1):
        a = reference[max(0, -k): n - max(0, k)]
        b = profile[max(0, k): n - max(0, -k)]
        a, b = a - a.mean(), b - b.mean()
        den = math.sqrt(float(np.dot(a, a) * np.dot(b, b)))
        scores.append(float(np.dot(a, b)) / den if den > 0 else -1.0)
    i = int(np.argmax(scores))
    shift = float(i - max_shift)
    if 0 < i < len(scores) - 1:
        a, b, c = scores[i - 1], scores[i], scores[i + 1]
        den = a - 2 * b + c
        shift += 0.5 * (a - c) / den if den != 0 else 0.0
    return shift


def best_lag(stamps, rows, lift_t, lift_m, taus):
    residuals = []
    for tau in taus:
        x = np.interp(stamps - tau, lift_t, lift_m)
        a = np.vstack([x, np.ones_like(x)]).T
        coef, res, *_ = np.linalg.lstsq(a, rows, rcond=None)
        residuals.append(float(np.sum((a @ coef - rows) ** 2)))
    k = int(np.argmin(residuals))
    return float(taus[k]), residuals


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--frames-dir", type=Path, required=True)
    ap.add_argument("--live-log", type=Path, required=True)
    ap.add_argument("--factor", type=float, required=True)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    report = json.loads(args.report.read_text())
    lift = [json.loads(line) for line in args.live_log.read_text().splitlines() if '"lift_sim_t"' in line]
    lift_t = np.array([e["lift_sim_t"] for e in lift])
    lift_m = np.array([e["lift_m"] for e in lift])
    stamps, rows, reference = [], [], None
    for f in report["frames"]:
        path = args.frames_dir / f"frame_{f['frame_id']:06d}.jpg"
        if path.exists() and f.get("stamp_ns") is not None:
            profile = row_profile(path)
            if reference is None:
                reference = profile
            stamps.append(f["stamp_ns"] / 1e9)
            rows.append(vertical_shift(profile, reference))
    if len(rows) < 20:
        raise SystemExit(f"{len(rows)} frames with a stamp in {args.frames_dir}: too few to fit")
    stamps, rows = np.array(stamps), np.array(rows)
    keep = (stamps > lift_t.min() + 0.5) & (stamps < lift_t.max())
    stamps, rows = stamps[keep], rows[keep]
    taus = np.arange(-0.2, 0.6 + 1e-9, 1 / 120)
    tau, residuals = best_lag(stamps, rows, lift_t, lift_m, taus)
    x = np.interp(stamps - tau, lift_t, lift_m)
    corr = float(np.corrcoef(x, rows)[0, 1])
    slope, intercept = np.polyfit(x, rows, 1)
    rng = np.random.default_rng(0)
    boot = []
    for _ in range(200):  # the frames resampled: how sharply the data pin tau down
        k = rng.integers(0, len(rows), len(rows))
        boot.append(best_lag(stamps[k], rows[k], lift_t, lift_m, taus)[0])
    # Per frame, where the wave is steep: the shift tau that puts the fitted row closest to the observed one.
    rate = np.gradient(lift_m, lift_t)
    per_frame, cand = [], np.arange(-0.2, 0.4 + 1e-9, 1 / 240)
    for s_, row_ in zip(stamps, rows):
        if abs(slope * np.interp(s_ - tau, lift_t, rate)) < 0.5 * abs(slope) * np.percentile(np.abs(rate), 95):
            continue
        per_frame.append(float(cand[np.argmin(np.abs(slope * np.interp(s_ - cand, lift_t, lift_m) + intercept - row_))]))
    residual_rows = float(np.std(rows - (slope * x + intercept)))
    result = {"frames": int(len(rows)), "pixel_lag_sim_s": tau, "pixel_lag_wall_s": tau / args.factor,
              "fit_correlation": corr, "lift_range_m": [float(lift_m.min()), float(lift_m.max())],
              "row_range": [float(rows.min()), float(rows.max())], "rows_per_m": float(slope),
              "residual_rows_rms": residual_rows,
              "bootstrap_tau_sim_s": {"p5": float(np.percentile(boot, 5)), "p95": float(np.percentile(boot, 95))},
              "per_frame_lag_sim_s": None if not per_frame else {
                  "n": len(per_frame), **{f"p{q}": float(np.percentile(per_frame, q)) for q in (5, 50, 95)},
                  "max": float(np.max(per_frame))},
              "factor": args.factor,
              "tau_grid_s": [float(taus[0]), float(taus[-1]), 1 / 120]}
    print(json.dumps(result, indent=1))
    if args.out is not None:
        args.out.write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
