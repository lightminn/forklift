"""New plant candidate gates V2 and V3 measured from joint angles (EPAL 6 fork spacing plan, 13th draft
gates as revised by the user on 2026-10-15: the engine-reported joint velocity under TGS is not the
derivative of the joint angle, so the steady window and the rolling ratio are read from angles).

V2 steady window: in a segment's second half, the longest run of consecutive samples in which every
adjacent pair has, for both steering joints, |dq| / dt < 0.02 rad/s and |d(q - target)| / dt < 0.005
rad/s (adjacent differences -- a central difference hides a tick-to-tick alternation); used when it
lasts >= 0.3 s, otherwise the segment is unverified. In the window: the body curvature from the truth
rear pose, the curvature implied by the two steering joints' mean actual angles, their residual, and the
left/right steering error.

V3 rolling ratio: in the same kind of window on a straight, the rear axle's longitudinal travel (truth)
over the two rear wheels' angle advance x wheel radius.

    python tools/plant_gates.py probe_dir [probe_dir ...] --output gates.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

QDOT_MAX_RAD_S = 0.02
SLOPE_MAX_RAD_S = 0.005
MIN_STEADY_S = 0.3


def steady_span(q: np.ndarray, target: np.ndarray, dt: float, *, qdot_max: float = QDOT_MAX_RAD_S,
                slope_max: float = SLOPE_MAX_RAD_S, min_s: float = MIN_STEADY_S) -> tuple[int, int] | None:
    """(start, stop) sample indices of the longest steady run in rows q (N, 2) / target (N, 2), or None.
    A run of k samples holds k - 1 adjacent pairs, all of which must pass."""
    q, target = np.asarray(q, float), np.asarray(target, float)
    if len(q) < 2:
        return None
    rate = np.abs(np.diff(q, axis=0)) / dt
    slope = np.abs(np.diff(q - target, axis=0)) / dt
    ok = np.all((rate < qdot_max) & (slope < slope_max), axis=1)  # one flag per adjacent pair
    best, start, best_span = 0, None, None
    for i, flag in enumerate(np.r_[ok, False]):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if i - start > best:
                best, best_span = i - start, (start, i + 1)  # pairs start..i-1 -> samples start..i
            start = None
    if best_span is None or (best_span[1] - best_span[0] - 1) * dt < min_s:
        return None
    return best_span


def wheel_curvatures(q_left: float, q_right: float, wheelbase_m: float, track_m: float) -> tuple[float, float]:
    tl, tr = math.tan(q_left), math.tan(q_right)
    half = track_m / 2
    return tl / (wheelbase_m + tl * half), tr / (wheelbase_m - tr * half)


def window_metrics(rows: np.ndarray, c: dict, dt: float, *, wheelbase_m: float, track_m: float, radius_m: float,
                   direction: float) -> dict:
    yaw = np.unwrap(rows[:, c["rear_yaw"]])
    dx, dy = np.diff(rows[:, c["rear_x"]]), np.diff(rows[:, c["rear_y"]])
    travel = float(np.sum(np.hypot(dx, dy)))
    k_true = direction * float(yaw[-1] - yaw[0]) / travel if travel > 0.02 else None
    q_l, q_r = float(np.mean(rows[:, c["q_FL"]])), float(np.mean(rows[:, c["q_FR"]]))
    k_l, k_r = wheel_curvatures(q_l, q_r, wheelbase_m, track_m)
    out = {
        "steady_s": (len(rows) - 1) * dt,
        "kappa_true": k_true,
        "kappa_FL": k_l, "kappa_FR": k_r, "kappa_avg": (k_l + k_r) / 2,
        "residual": None if k_true is None else k_true - (k_l + k_r) / 2,
        "err_FL_mrad": float(np.mean(rows[:, c["q_FL"]] - rows[:, c["target_FL"]])) * 1e3,
        "err_FR_mrad": float(np.mean(rows[:, c["q_FR"]] - rows[:, c["target_FR"]])) * 1e3,
    }
    if "p_RL" in c:
        lon = float(np.sum(dx * np.cos(yaw[1:]) + dy * np.sin(yaw[1:])))
        advance = float(np.mean([np.unwrap(rows[:, c[k]])[-1] - np.unwrap(rows[:, c[k]])[0] for k in ("p_RL", "p_RR")]))
        out["rolling_ratio"] = lon / (advance * radius_m) if abs(advance) > 1e-6 else None
    return out


def evaluate(run: Path) -> dict:
    run = Path(run)
    if (run / "run").is_dir():
        run = run / "run"
    result = json.loads((run / "result.json").read_text())
    z = np.load(run / "steering_probe.npz")
    rec = z["record"]
    c = {str(k): i for i, k in enumerate(z["columns"])}
    dt = float(result["steering_probe"]["dt_s"])
    geom = (result.get("chassis_model") or {}).get("drive_geometry") or {}
    wheelbase, track, radius = float(geom["wheelbase_m"]), float(geom["track_m"]), float(geom["wheel_radius_m"])
    segments = []
    for s in result["steering_probe"]["segments"]:
        if s["speed_mps"] == 0:
            continue
        rows = rec[rec[:, 1] == s["segment"]]
        late = rows[len(rows) // 2:]
        span = steady_span(late[:, [c["q_FL"], c["q_FR"]]], late[:, [c["target_FL"], c["target_FR"]]], dt)
        entry = {"segment": s["segment"], "label": s["label"], "speed_mps": s["speed_mps"], "kappa_cmd": s["kappa_cmd"],
                 "steady": None}
        if span is not None:
            entry["steady"] = window_metrics(late[span[0]:span[1]], c, dt, wheelbase_m=wheelbase, track_m=track,
                                             radius_m=radius, direction=1.0 if s["speed_mps"] > 0 else -1.0)
        segments.append(entry)
    return {"run": str(run), "settings": result.get("settings_synthetic"), "has_wheel_angles": "p_RL" in c,
            "segments": segments}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    out = [evaluate(r) for r in args.runs]
    args.output.write_text(json.dumps(out, indent=1) + "\n")
    for e in out:
        print("==", e["run"], "wheel angles" if e["has_wheel_angles"] else "no wheel angles")
        for s in e["segments"]:
            st = s["steady"]
            print("  %2d %-9s v %+.2f k %+.2f  %s" % (
                s["segment"], s["label"], s["speed_mps"], s["kappa_cmd"],
                "unverified" if st is None else "steady %.2fs k_true %s res %s roll %s errL/R %.1f/%.1f" % (
                    st["steady_s"], None if st["kappa_true"] is None else round(st["kappa_true"], 4),
                    None if st["residual"] is None else round(st["residual"], 4),
                    None if st.get("rolling_ratio") is None else round(st["rolling_ratio"], 4),
                    st["err_FL_mrad"], st["err_FR_mrad"])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
