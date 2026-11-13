"""Baseline fragility priority 1, descriptive: does a rotation-gain error of the odometry
explain the yaw of the bias change over the capture -> lift leg? (#332 measured the held legs' net turn ratio
odometry/truth; this asks whether one gain carries over.)

    python gain_check.py DECOMP.json --raw-root DIR --out GAIN.json

DECOMP.json is seat_decompose.py's output (its runs keep root and name, so the raw records are read again). For every
run that reached lift and was not held or a duplicate, dpsi (the bias change's yaw, est turn - true turn from the
capture to lift) is fitted by least squares against the true rotation over the same rows: one gain (dpsi = k * net
turn), left/right gains (dpsi = kL * left turn + kR * right turn), and the left/right fit refitted with each scene held
out and scored on it. A rotation-gain error would show as a stable k with a high R^2 and a fit that carries across
scenes. Reads only; no judgement beyond the numbers.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

TIME_TOL_S = 1e-6


def true_rotation(result: dict, sc: np.ndarray, t_lift: float) -> tuple[float, float, float]:
    """Net, left (+) and right (-) true yaw change from the capture's truth pose through the slam_control rows to
    lift."""
    near = result["near_capture"]
    cap = result["observation_attempts"][near["near_attempt"] - 1]["capture_control"]
    rows = sc[(sc[:, 0] >= cap["time_s"] - TIME_TOL_S) & (sc[:, 0] <= t_lift + TIME_TOL_S)]
    yaw = np.concatenate([[cap["truth_rear"][2]], rows[:, 6]])
    d = np.array([math.remainder(b - a, math.tau) for a, b in zip(yaw[:-1], yaw[1:])])
    return float(d.sum()), float(d[d > 0].sum()), float(d[d < 0].sum())


def fit(A: np.ndarray, y: np.ndarray) -> tuple[list[float], float]:
    c = np.linalg.lstsq(A, y, rcond=None)[0]
    return [float(v) for v in c], r2(y, A @ c)


def r2(y: np.ndarray, pred: np.ndarray) -> float:
    return float(1 - ((y - pred) ** 2).sum() / ((y - y.mean()) ** 2).sum())


def analyse(runs: list[dict], raw_root: Path) -> dict:
    X, y, scenes = [], [], []
    for r in runs:
        run = raw_root / r["root"] / r["name"] / "run"
        result = json.loads((run / "result.json").read_text())
        net, left, right = true_rotation(result, np.load(run / "slam_control.npy"), r["t_lift"])
        X.append([net, left, right])
        y.append(r["bias_course"]["dpsi_rad"])
        scenes.append(r["scene"])
    X, y = np.array(X), np.array(y)
    single, single_r2 = fit(X[:, :1], y)
    split, split_r2 = fit(X[:, 1:], y)
    held_out = {}
    for scene in sorted(set(scenes)):
        mask = np.array([s == scene for s in scenes])
        c, _ = fit(X[~mask, 1:], y[~mask])
        held_out[scene] = {"n": int(mask.sum()), "kL": c[0], "kR": c[1],
                           "r2_on_held_out": r2(y[mask], X[mask, 1:] @ np.array(c))}
    per_scene = {}
    for scene in sorted(set(scenes)):
        mask = np.array([s == scene for s in scenes])
        per_scene[scene] = {"n": int(mask.sum()), "corr_dpsi_net_turn": float(np.corrcoef(y[mask], X[mask, 0])[0, 1])}
    return {"n": len(y), "single_gain": {"k": single[0], "r2": single_r2},
            "left_right": {"kL": split[0], "kR": split[1], "r2": split_r2}, "scene_held_out": held_out,
            "per_scene": per_scene, "corr_dpsi_net_turn": float(np.corrcoef(y, X[:, 0])[0, 1]),
            "net_turn_abs_median_rad": float(np.median(np.abs(X[:, 0]))),
            "dpsi_abs_median_rad": float(np.median(np.abs(y)))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("decomp", type=Path)
    ap.add_argument("--raw-root", type=Path, required=True, help="the folder holding the copies named in DECOMP.json")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    runs = [r for r in json.loads(args.decomp.read_text())["runs"]
            if r["judgement"]["status"] in ("judged", "not_selected") and r.get("bias_course")]
    args.out.write_text(json.dumps(analyse(runs, args.raw_root), indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
