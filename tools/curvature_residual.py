"""How far the Isaac truck's true path curvature strays from the curvature its steering joints imply.

The teleoperated arc bundle (``forklift_core.safety.teleop_drive``) is built from the steering joints through the
Ackermann geometry; the M-c approach run found the true rear-axle path curving about 0.02 1/m away from it
(docs/validation/2026-10-17-m-c-evidence.md 1.5). This tool measures that residual over recorded runs, at the
120 Hz of ``slam_log.npz`` (true base pose and steering joints, recorded with ``--record-slam``):

- the true rear axle is the base pose moved back by the run's rear-axle offset along its yaw;
- over a window of at least ``--min-travel-m`` of true travel (and at most ``--max-window-s``), the true curvature
  is the yaw change over the signed travel along the mid heading; the joint curvature is the window's
  distance-weighted mean of ``held_kappa`` (mean of the two steering joints' Ackermann curvatures, as the runner
  computes it);
- a window is **steady** when the joint curvature moved by at most ``--steady-inv-m`` across it;
- the lateral slip is the rear axle's sideways travel over its forward travel (zero for a rolling, unslipped axle).

Runs are grouped by plant (the result's plant settings, tyre contact and chassis URDF hash), since the residual
belongs to the plant, not to the code. Prints JSON: per plant, per (direction, loaded, steady), the count and the
residual quantiles, and the per-run worst cases. Ground truth only: an Isaac measurement, not a real-truck one.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

LOADED_PHASES = ("lift", "extract", "transport", "lower")


def yaw_of(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = (quat_wxyz[:, i] for i in range(4))
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def joint_curvature(steering: np.ndarray, wheelbase_m: float, track_m: float) -> np.ndarray:
    """The runner's held_kappa: the mean of the two steering joints' Ackermann curvatures."""
    half = track_m / 2
    left, right = np.tan(steering[:, 0]), np.tan(steering[:, 1])
    return 0.5 * (left / (wheelbase_m + left * half) + right / (wheelbase_m - right * half))


def windows(stamps, rear, yaw, kappa_joint, *, min_travel_m, max_window_s, steady_inv_m):
    """Rows (t, direction, joint kappa, true kappa, residual, slip, speed, steady) over consecutive windows."""
    step = np.hypot(np.diff(rear[:, 0]), np.diff(rear[:, 1]))
    travel = np.concatenate(([0.0], np.cumsum(step)))
    rows = []
    i, n = 0, len(stamps)
    while i < n - 1:
        j = int(np.searchsorted(travel, travel[i] + min_travel_m))
        if j >= n:
            break
        if stamps[j] - stamps[i] > max_window_s:
            i += 1
            continue
        dx, dy = rear[j, 0] - rear[i, 0], rear[j, 1] - rear[i, 1]
        dyaw = math.remainder(float(yaw[j] - yaw[i]), 2 * math.pi)
        mid = float(yaw[i]) + dyaw / 2
        lon = dx * math.cos(mid) + dy * math.sin(mid)
        lat = -dx * math.sin(mid) + dy * math.cos(mid)
        if abs(lon) < 0.5 * min_travel_m:  # turning on the spot or a reversal inside the window
            i = j
            continue
        k_true = dyaw / lon
        seg = kappa_joint[i:j + 1]
        # Weighted by the distance each step drove, as the yaw change integrates curvature over distance (a time
        # mean misreads a window that brakes or accelerates while the steering moves).
        ds = step[i:j]
        k_joint = float(np.sum(0.5 * (seg[:-1] + seg[1:]) * ds) / np.sum(ds))
        steady = float(np.max(seg) - np.min(seg)) <= steady_inv_m
        rows.append((float(stamps[i]), 1 if lon > 0 else -1, k_joint, k_true, k_true - k_joint, lat / abs(lon),
                     abs(lon) / float(stamps[j] - stamps[i]), steady))
        i = j
    return rows


def plant_key(result: dict) -> str:
    parts = {k: result.get(k) for k in ("plant", "plant_tyre_contact", "forklift_urdf_sha256", "plant_backend")}
    return hashlib.sha256(json.dumps(parts, sort_keys=True, default=str).encode()).hexdigest()[:12]


def loaded_at(result: dict, stamps: np.ndarray) -> np.ndarray:
    samples = result.get("samples") or []
    if not samples:
        return np.zeros(len(stamps), bool)
    t = np.array([s["time_s"] for s in samples], float)
    loaded = np.array([s.get("phase") in LOADED_PHASES for s in samples], bool)
    index = np.clip(np.searchsorted(t, stamps, side="right") - 1, 0, len(t) - 1)
    return loaded[index]


def run_rows(run: Path, args) -> tuple[str, list]:
    result = json.loads((run / "result.json").read_text())
    log = np.load(run / "slam_log.npz")
    a = result.get("arguments", {})
    geom = a.get("drive_geometry") or {}
    wheelbase = float(geom.get("wheelbase_m", args.wheelbase_m)) if isinstance(geom, dict) else args.wheelbase_m
    track = float(geom.get("track_m", args.track_m)) if isinstance(geom, dict) else args.track_m
    offset = abs(float(a.get("rear_axle_offset_m", args.rear_axle_offset_m)))
    stamps = np.asarray(log["joint_stamps_s"], float)
    base = np.asarray(log["base_pose_world"], float)
    yaw = np.unwrap(yaw_of(base[:, 3:7]))
    rear = np.column_stack((base[:, 0] - offset * np.cos(yaw), base[:, 1] - offset * np.sin(yaw)))
    kj = joint_curvature(np.asarray(log["steering_rad"], float), wheelbase, track)
    rows = windows(stamps, rear, yaw, kj, min_travel_m=args.min_travel_m, max_window_s=args.max_window_s,
                   steady_inv_m=args.steady_inv_m)
    loaded = loaded_at(result, np.array([r[0] for r in rows])) if rows else []
    return plant_key(result), [(*r, bool(l)) for r, l in zip(rows, loaded)], (wheelbase, track, offset)


def quantiles(values: np.ndarray) -> dict:
    if not len(values):
        return {"n": 0}
    q = np.percentile(values, [0.1, 1, 50, 99, 99.9])
    return {"n": int(len(values)), "min": float(values.min()), "p0.1": float(q[0]), "p1": float(q[1]),
            "median": float(q[2]), "p99": float(q[3]), "p99.9": float(q[4]), "max": float(values.max()),
            "max_abs": float(np.abs(values).max())}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("runs", nargs="+", type=Path, help="run directories holding result.json and slam_log.npz")
    p.add_argument("--min-travel-m", type=float, default=0.03)
    p.add_argument("--max-window-s", type=float, default=0.5)
    p.add_argument("--steady-inv-m", type=float, default=0.005)
    p.add_argument("--min-speed-mps", type=float, default=0.02, help="windows slower than this are left out")
    p.add_argument("--wheelbase-m", type=float, default=0.66, help="when the result does not record it")
    p.add_argument("--track-m", type=float, default=0.5)
    p.add_argument("--rear-axle-offset-m", type=float, default=0.34)
    args = p.parse_args()
    plants: dict = {}
    for run in args.runs:
        try:
            key, rows, geom = run_rows(run, args)
        except (OSError, KeyError, ValueError) as error:
            print(json.dumps({"skipped": str(run), "why": repr(error)}))
            continue
        rows = [r for r in rows if r[6] >= args.min_speed_mps]
        entry = plants.setdefault(key, {"runs": [], "geometry": set(), "rows": []})
        entry["geometry"].add(geom)
        entry["rows"] += rows
        if rows:
            res = np.array([r[4] for r in rows])
            worst = int(np.argmax(np.abs(res)))
            entry["runs"].append({"run": str(run), "windows": len(rows), "max_abs_residual": float(abs(res[worst])),
                                  "worst": dict(zip(("t", "direction", "k_joint", "k_true", "residual", "slip",
                                                     "speed", "steady", "loaded"), rows[worst]))})
    out = {}
    for key, entry in plants.items():
        R = entry["rows"]
        if not R:
            continue
        A = np.array([r[:8] for r in R], float)
        loaded = np.array([r[8] for r in R], bool)
        groups = {}
        for d in (1, -1):
            for ld in (False, True):
                for st in (True, False):
                    m = (A[:, 1] == d) & (loaded == ld) & (A[:, 7] == (1.0 if st else 0.0))
                    if m.any():
                        groups[f"{'fwd' if d > 0 else 'rev'}_{'loaded' if ld else 'empty'}_"
                               f"{'steady' if st else 'steering'}"] = {
                            "residual": quantiles(A[m, 4]), "abs_slip": quantiles(np.abs(A[m, 5])),
                            "joint_kappa": [float(A[m, 2].min()), float(A[m, 2].max())],
                            "speed_max": float(A[m, 6].max())}
        runs = sorted(entry["runs"], key=lambda r: -r["max_abs_residual"])
        out[key] = {"run_count": len(entry["runs"]), "geometry": sorted(entry["geometry"]),
                    "windows": len(R), "all": quantiles(A[:, 4]), "groups": groups, "worst_runs": runs[:5]}
    print(json.dumps({"settings": {k: v for k, v in vars(args).items() if k != "runs"}, "plants": out}, indent=1,
                     default=str))


if __name__ == "__main__":
    main()
