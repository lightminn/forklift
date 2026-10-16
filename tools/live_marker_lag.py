"""Pixel lag of the live runner's camera (M-c plan C5), from the marker run: the lift (carrying the camera) follows a
known non-periodic wave, so the picture moves up and down with the lift height at the time its pixels were drawn.

Each frame's vertical shift against the first (row-brightness profile, best correlation) is fitted linearly to the
logged lift height at (stamp - tau) for every tau on a grid; the tau with the smallest residual is the pixel lag behind
the frame's stamp (the render metadata's sim time). A bootstrap over the frames gives its spread. A
frame-by-frame lag is not inverted -- one frame's height does not name one time -- but a frame off the fitted line by
more than three times the noise (MAD) is listed as nonconforming: the frames agree with one constant lag or they do
not (Codex M-c rounds 16-17 P2).

Sim to wall goes through the runner's step table (each physics step's sim time and the monotonic time it started), not
through a factor: the pixel-inclusive latency of a frame is acquired_mono - start_mono(step at stamp - tau), with
acquired_mono = start_mono(step at stamp) + the runner's metadata latency (round 16 P1).

  python tools/live_marker_lag.py --report RUN/probe.json --frames-dir RUN/frames --live-log RUN/run/live_log.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

TAU_GRID = np.arange(-0.5, 3.0 + 1e-9, 1 / 120)  # wide enough that a whole-period alias would show as its own minimum


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


def residuals(stamps, rows, lift_t, lift_m, taus):
    out = []
    for tau in taus:
        x = np.interp(stamps - tau, lift_t, lift_m)
        a = np.vstack([x, np.ones_like(x)]).T
        coef, *_ = np.linalg.lstsq(a, rows, rcond=None)
        out.append(float(np.sum((a @ coef - rows) ** 2)))
    return np.array(out)


def best_lag(stamps, rows, lift_t, lift_m, taus):
    res = residuals(stamps, rows, lift_t, lift_m, taus)
    return float(taus[int(np.argmin(res))]), res


def quantiles(values) -> dict | None:
    if len(values) == 0:
        return None
    v = np.asarray(values, dtype=float)
    return {"n": int(len(v)), "median": float(np.median(v)), "p99": float(np.percentile(v, 99)), "max": float(v.max())}


def analyse(report: dict, frames_dir: Path, log: list[dict]) -> dict:
    lift = [e for e in log if "lift_sim_t" in e]
    lift_t = np.array([e["lift_sim_t"] for e in lift])
    lift_m = np.array([e["lift_m"] for e in lift])
    start_mono = np.array([e["step_start_mono"] for e in lift])  # the step that ended at lift_t started then
    latency = {e["camera"]: e["latency_s"] for e in log if "camera" in e and e.get("latency_s") is not None}
    stamps, rows, ids, reference = [], [], [], None
    for f in report["frames"]:
        path = frames_dir / f"frame_{f['frame_id']:06d}.jpg"
        if path.exists() and f.get("stamp_ns") is not None:
            profile = row_profile(path)
            if reference is None:
                reference = profile
            stamps.append(f["stamp_ns"] / 1e9)
            rows.append(vertical_shift(profile, reference))
            ids.append(f["frame_id"])
    stamps, rows, ids = np.array(stamps), np.array(rows), np.array(ids)
    keep = (stamps > lift_t.min() + TAU_GRID[-1]) & (stamps < lift_t.max() + TAU_GRID[0])
    stamps, rows, ids = stamps[keep], rows[keep], ids[keep]
    if len(rows) < 20:
        raise SystemExit(f"{len(rows)} frames with a stamp inside the lift log: too few to fit")
    # Trimmed fit: frames off the line by more than three times the noise (MAD) are left out and the fit is redone, so a
    # few late frames neither move the lag nor hide among the rest (they are listed below).
    inlier = np.ones(len(rows), dtype=bool)
    for _ in range(4):
        tau, res = best_lag(stamps[inlier], rows[inlier], lift_t, lift_m, TAU_GRID)
        x = np.interp(stamps - tau, lift_t, lift_m)
        slope, intercept = np.polyfit(x[inlier], rows[inlier], 1)
        resid = rows - (slope * x + intercept)
        noise = 1.4826 * float(np.median(np.abs(resid[inlier] - np.median(resid[inlier]))))
        inlier = np.abs(resid) <= max(3 * noise, 1.0)
    # The second-best separate minimum: how much worse the best alias is (the wave is not periodic within the run).
    far = np.abs(TAU_GRID - tau) > 0.25
    rng = np.random.default_rng(0)
    boot = []
    for _ in range(200):
        k = rng.choice(np.flatnonzero(inlier), int(inlier.sum()))
        boot.append(best_lag(stamps[k], rows[k], lift_t, lift_m, TAU_GRID)[0])
    # A single frame's height does not name one time (the wave passes each height many times), so no per-frame lag is
    # inverted (Codex M-c rounds 16-17 P2). What the frames can show is whether they all agree with one lag.
    nonconforming = [int(i_) for i_ in ids[~inlier]]

    def start_at(sim_t):
        return float(np.interp(sim_t, lift_t, start_mono))

    meta, total = [], []
    for i_, s_, ok_ in zip(ids, stamps, inlier):
        if int(i_) not in latency:
            continue
        meta.append(latency[int(i_)])
        if ok_:  # a nonconforming frame's own lag is unknown: it is listed, not given the fitted one
            acquired = start_at(s_) + latency[int(i_)]
            total.append(acquired - start_at(s_ - tau))
    return {
        "frames": int(len(rows)), "pixel_lag_sim_s": tau, "physics_steps": tau * 120,
        "bootstrap_tau_sim_s": {"p5": float(np.percentile(boot, 5)), "p95": float(np.percentile(boot, 95))},
        "residual_rows_rms": float(np.std(resid[inlier])), "rows_per_m": float(slope),
        "fit_correlation": float(np.corrcoef(x[inlier], rows[inlier])[0, 1]),
        "alias_residual_ratio": float(res[far].min() / max(res.min(), 1e-12)),
        "lift_range_m": [float(lift_m.min()), float(lift_m.max())],
        "noise_rows": noise, "nonconforming_frames": nonconforming,
        "metadata_latency_s": quantiles(meta),
        "pixel_latency_wall_s_conforming_frames": quantiles(total),
        "tau_grid_s": [float(TAU_GRID[0]), float(TAU_GRID[-1]), 1 / 120],
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--frames-dir", type=Path, required=True)
    ap.add_argument("--live-log", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    log = [json.loads(line) for line in args.live_log.read_text().splitlines()
           if '"lift_sim_t"' in line or '"camera"' in line]
    result = analyse(json.loads(args.report.read_text()), args.frames_dir, log)
    print(json.dumps(result, indent=1))
    if args.out is not None:
        args.out.write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
