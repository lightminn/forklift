"""Where the raw odometry's heading error during the hall transport comes from (lead assignment 2026-11-10, after #409),
from the live runs' joint logs only (spool/slam_log.joint: per physics tick the true and measured steering angles, the
four wheels' true joint angles, the rear wheels' quantized increments, the base pose's truth).

Per session and phase (phases from spool/slam.records by stamp):
1. The odometry's heading change integrated as the runner does (IncrementalWheelOdometry.update_increments: the rear
   wheels' mean increment x the wheel radius, the Ackermann curvature of the two steering angles, mid-interval) with
   each input measured or true, against the truth's heading change:
   A measured steering + quantized increments (the run's own odometry), B measured steering + true increments (the
   wheel encoders' quantization out), C true steering + quantized increments (the steering encoders out), D true
   steering + true increments (both out: what is left is the kinematic model against the plant).
2. The wheel travel (the rear wheels' true rotation x radius) against the truth path length (longitudinal slip, radius).
3. Over forward turning ticks (|curvature| > 0.15 1/m): the rear wheels' rotation difference against track x curvature
   (is the rear axle driven as the kinematics needs), and three heading changes -- the truth's, the one the rear wheels'
   difference implies if they rolled without slip ((R - L) x radius / track), and the model's (D).
4. A least-squares split of the truth's per-tick heading change into a x the model's (true steering, true travel) plus
   b x travel (a scale and a curvature offset).
The geometry is the run's (result.json chassis_model.drive_geometry: wheelbase, track, wheel radius).

    python odom_yaw_diag.py ROOT SESSION... [--json out.json]   (ROOT/<batch>/<session>/live/epoch_1/run)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

PHASES = ("approach", "insert", "extract", "transport", "return_home")


def yaw_of(q):  # wxyz
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_joint(run: Path) -> dict:
    files = sorted((run / "spool" / "slam_log.joint").glob("*.npz"), key=lambda f: int(f.stem))
    parts = [np.load(f) for f in files]
    out = {k: np.concatenate([p[k] for p in parts]) for k in parts[0].files}
    order = np.argsort(out["joint_stamps_s"], kind="stable")
    return {k: v[order] for k, v in out.items()}


def phase_per_tick(run: Path, stamps: np.ndarray) -> np.ndarray:
    recs = []
    for f in sorted((run / "spool" / "slam.records").glob("*.jsonl")):
        recs += [json.loads(line) for line in f.read_text().splitlines() if line.strip()]
    recs.sort(key=lambda r: r["stamp_s"])
    ps = np.array([r["stamp_s"] for r in recs])
    pp = [r.get("phase") for r in recs]
    return np.array([pp[max(0, int(np.searchsorted(ps, x, side="right")) - 1)] for x in stamps])


def session(run: Path) -> dict:
    g = json.loads((run / "result.json").read_text())["chassis_model"]["drive_geometry"]
    wb, tr, rad = float(g["wheelbase_m"]), float(g["track_m"]), float(g["wheel_radius_m"])

    def curv(st):
        lt, rt = np.tan(st[:, 0]), np.tan(st[:, 1])
        return (lt / (wb + lt * tr / 2) + rt / (wb - rt * tr / 2)) / 2

    J = load_joint(run)
    t = J["joint_stamps_s"]
    ph = phase_per_tick(run, t)
    truth_yaw = np.unwrap(yaw_of(J["base_pose_world"][:, 3:7].astype(float)))
    dy_truth = np.zeros(len(t))
    dy_truth[1:] = np.diff(truth_yaw)
    true_inc = np.zeros((len(t), 4))
    true_inc[1:] = np.diff(J["wheel_angles_rad"], axis=0)
    pos = J["base_pose_world"][:, :2].astype(float)

    def model_dyaw(inc, st):
        k = curv(st)
        d = inc.mean(axis=1) * rad
        out = np.zeros(len(d))
        out[1:] = d[1:] * (k[:-1] + k[1:]) / 2
        return out

    variants = {"A": (J["wheel_increments_rad"], J["steering_measured_rad"]),
                "B": (true_inc[:, 2:4], J["steering_measured_rad"]),
                "C": (J["wheel_increments_rad"], J["steering_rad"]),
                "D": (true_inc[:, 2:4], J["steering_rad"])}
    mdy = {k: model_dyaw(*v) for k, v in variants.items()}
    k_true = curv(J["steering_rad"])
    ds = true_inc[:, 2:4].mean(axis=1) * rad
    out = {"geometry": g, "phases": {}}
    for phase in PHASES:
        m = ph == phase
        if m.sum() < 10:
            continue
        idx = np.nonzero(m)[0][1:]
        row = {"ticks": int(m.sum()), "truth_dyaw_rad": float(dy_truth[idx].sum()),
               "error_rad": {k: float(v[idx].sum() - dy_truth[idx].sum()) for k, v in mdy.items()},
               "truth_path_m": float(np.sum(np.hypot(*np.diff(pos[np.nonzero(m)[0]], axis=0).T))),
               "wheel_travel_m": float(np.sum(np.abs(ds[idx])))}
        mv = m & (np.abs(ds) > 1e-4)
        if mv.sum() > 10:
            X = np.column_stack((k_true[mv] * ds[mv], ds[mv]))
            a, b = np.linalg.lstsq(X, dy_truth[mv], rcond=None)[0]
            row["fit"] = {"scale": float(a), "offset_per_m": float(b)}
        tm = m & (np.abs(k_true) > 0.15) & (ds > 1e-4)
        if tm.sum() > 10:
            rl, rr = true_inc[tm, 2], true_inc[tm, 3]
            row["turning_forward"] = {
                "ticks": int(tm.sum()),
                "rear_diff_over_mean": float(((rr - rl) / ((rr + rl) / 2)).mean()),
                "track_x_curvature": float((tr * k_true[tm]).mean()),
                "truth_dyaw_rad": float(dy_truth[tm].sum()),
                "rear_wheels_dyaw_rad": float(((rr - rl) * rad / tr).sum()),
                "model_dyaw_rad": float(mdy["D"][tm].sum())}
        out["phases"][phase] = row
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    res = {s: session(a.root / s / "live" / "epoch_1" / "run") for s in a.sessions}
    if a.json:
        a.json.write_text(json.dumps(res, indent=1))
    for s, r in res.items():
        print(s)
        for ph, row in r["phases"].items():
            e = row["error_rad"]
            tf = row.get("turning_forward") or {}
            print(f"  {ph:12s} truth {row['truth_dyaw_rad']:+.3f} err A {e['A']:+.4f} B {e['B']:+.4f} C {e['C']:+.4f} "
                  f"D {e['D']:+.4f} | path {row['truth_path_m']:.2f}/{row['wheel_travel_m']:.2f} m | "
                  f"fit {row.get('fit')} | turning truth {tf.get('truth_dyaw_rad')} rear {tf.get('rear_wheels_dyaw_rad')} "
                  f"model {tf.get('model_dyaw_rad')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
