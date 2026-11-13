"""Baseline fragility priority 1, descriptive: where the odometry's heading error over the held capture -> lift leg
comes from (plan 3.2, the L/C route).

    python odometry_split.py DECOMP.json --raw-root DIR --out SPLIT.json

DECOMP.json is seat_decompose.py's output; each run's folder must also hold slam_log.npz (joint_stamps_s,
wheel_angles_rad, wheel_increments_rad, steering_rad, steering_measured_rad; the same clock as slam_control). The
truck's odometry geometry is the run's chassis_model.drive_geometry (wheelbase, track, wheel radius; the runs here set
no odometry-only radius).

Per run that reached lift (not held, not a duplicate), from the joint sample at the capture to the lift transition:
- the odometry integrated with the recorded inputs (quantised wheel increments, measured steering) is checked against
  the estimate's own relative motion over the same rows (they agree while SLAM is held);
- the same integration with ideal inputs (continuous wheel angles, true steering) and with one input quantised at a
  time gives the quantisation's share of the heading change (recorded - ideal, wheels, steering);
- ideal - truth is the kinematic model's heading error against the plant (Isaac), which the quantisation does not
  cause.
Pooled over the runs, a curvature-lag model is fitted: the plant's heading change per tick is taken as
gain * (this tick's rolling distance) * (the kinematic curvature `lag` ticks earlier), i.e. the curvature from the
measured joint angles reaches the chassis late while the distance does not. (gain and lag) are chosen on a grid by the
median over runs of |the run's end-to-end net-turn residual| -- not a per-tick fit -- on all runs and with each scene
(and the B0 arm) held out. The steering input is the actual joint angle, so a lag here is not the command -> joint
lag. This is a descriptive check of one hypothesis, not a plant identification; the best lag is not a measured delay.
Reads only.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from forklift_core.localization.slam_pose import IncrementalWheelOdometry, compose, invert
from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry, _curvature_inv_m

TIME_TOL_S = 1e-6
HISTORY_TICKS = 60  # kinematics before the capture, so a lagged prediction starts from real samples
LAGS = range(0, 25)
GAINS = np.round(np.linspace(0.85, 1.02, 35), 4)


def window(log, sc: np.ndarray, t_cap: float, t_lift: float) -> tuple[int, int, int]:
    """Joint-sample indices of the capture (its own stamp: the capture ends between control ticks, so slam_control
    has no row there and the capture's poses come from capture_control) and of lift, and lift's slam_control row."""
    stamps = np.asarray(log["joint_stamps_s"], dtype=float)
    i0 = int(np.argmin(np.abs(stamps - t_cap)))
    i1 = int(np.argmin(np.abs(stamps - t_lift)))
    j1 = int(np.argmin(np.abs(sc[:, 0] - t_lift)))
    if abs(stamps[i0] - t_cap) > TIME_TOL_S or abs(stamps[i1] - t_lift) > TIME_TOL_S or i1 <= i0:
        raise ValueError("the slam log has no samples at the capture and at lift")
    if abs(sc[j1, 0] - t_lift) > TIME_TOL_S:
        raise ValueError("slam_control has no row at lift")
    return i0, i1, j1


def base_yaw(log) -> np.ndarray:
    """The true base_link yaw at every joint sample (base_pose_world: x, y, z, qw, qx, qy, qz)."""
    q = np.asarray(log["base_pose_world"], dtype=float)
    w, x, y, z = q[:, 3], q[:, 4], q[:, 5], q[:, 6]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def integrate(geometry, stamps, increments, steering, i0: int, i1: int) -> tuple[float, float, float]:
    """The odometry's relative pose from sample i0 to i1 (the first sample only sets the curvature)."""
    odo = IncrementalWheelOdometry(geometry)
    pose = odo.update_increments(stamps[i0], increments[i0], steering[i0])
    for k in range(i0 + 1, i1 + 1):
        pose = odo.update_increments(stamps[k], increments[k], steering[k])
    return pose


def continuous_increments(log) -> np.ndarray:
    angles = np.asarray(log["wheel_angles_rad"], dtype=float)[:, 2:4]
    return np.vstack([np.zeros((1, 2)), np.diff(angles, axis=0)])


def run_split(result: dict, sc: np.ndarray, log, t_lift: float) -> dict:
    g = result["chassis_model"]["drive_geometry"]
    if (result.get("arguments") or {}).get("odometry_wheel_radius_m") is not None:
        raise ValueError("an odometry-only wheel radius is not handled here")
    geometry = AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"])
    cap = result["observation_attempts"][result["near_capture"]["near_attempt"] - 1]["capture_control"]
    i0, i1, j1 = window(log, sc, float(cap["time_s"]), t_lift)
    stamps = np.asarray(log["joint_stamps_s"], dtype=float)
    rec_inc, rec_steer = np.asarray(log["wheel_increments_rad"]), np.asarray(log["steering_measured_rad"])
    ideal_inc, ideal_steer = continuous_increments(log), np.asarray(log["steering_rad"], dtype=float)
    recorded = integrate(geometry, stamps, rec_inc, rec_steer, i0, i1)
    ideal = integrate(geometry, stamps, ideal_inc, ideal_steer, i0, i1)
    wheels_only = integrate(geometry, stamps, rec_inc, ideal_steer, i0, i1)  # quantised wheels, true steering
    steering_only = integrate(geometry, stamps, ideal_inc, rec_steer, i0, i1)  # continuous wheels, measured steering
    est = compose(invert(cap["control_rear"]), sc[j1, 1:4])
    truth = compose(invert(cap["truth_rear"]), sc[j1, 4:7])
    return {
        "capture_sample_t": float(stamps[i0]), "lift_sample_t": float(stamps[i1]), "ticks": i1 - i0,
        "recorded_vs_estimate_m": math.hypot(recorded[0] - est[0], recorded[1] - est[1]),
        "recorded_vs_estimate_rad": recorded[2] - est[2],
        "estimate_minus_truth_yaw_rad": est[2] - truth[2],
        "quantisation_yaw_rad": recorded[2] - ideal[2],
        "wheel_quantisation_yaw_rad": wheels_only[2] - ideal[2],
        "steering_quantisation_yaw_rad": steering_only[2] - ideal[2],
        "kinematic_minus_truth_yaw_rad": ideal[2] - truth[2],
        "truth_turn_rad": truth[2], "kinematic_turn_rad": ideal[2],
    }


def kinematic_series(result: dict, sc: np.ndarray, log, t_lift: float) -> dict:
    """Per-tick true heading change and kinematic inputs from HISTORY_TICKS before the capture to lift."""
    g = result["chassis_model"]["drive_geometry"]
    geometry = AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"])
    cap = result["observation_attempts"][result["near_capture"]["near_attempt"] - 1]["capture_control"]
    i0, i1, _ = window(log, sc, float(cap["time_s"]), t_lift)
    h = min(HISTORY_TICKS, i0)
    yaw = base_yaw(log)[i0 - h: i1 + 1]
    true_dyaw = np.array([math.remainder(b - a, math.tau) for a, b in zip(yaw[:-1], yaw[1:])])
    k = _curvature_inv_m(np.asarray(log["steering_rad"], dtype=float)[i0 - h: i1 + 1], geometry)
    ds = continuous_increments(log)[i0 - h + 1: i1 + 1].mean(axis=1) * g["wheel_radius_m"]
    return {"true_dyaw": true_dyaw, "ds": ds, "k": (k[1:] + k[:-1]) / 2, "history": h}


def lag_gain_residual(series: list[dict], lag: int, gain: float) -> np.ndarray:
    """Per run, over the window: sum over ticks t of (gain * ds[t] * k[t - lag] - true_dyaw[t]), the curvature-lag
    model's end-to-end net-turn residual (the distance is not delayed)."""
    out = []
    for s in series:
        k = s["k"]
        lagged = np.concatenate([np.full(lag, k[0]), k[: len(k) - lag]]) if lag else k
        pred = gain * s["ds"] * lagged
        out.append(float((pred - s["true_dyaw"])[s["history"]:].sum()))
    return np.array(out)


def tick_gain(series: list[dict]) -> dict:
    """Pooled per-tick least squares over the windows: plant heading change = g * kinematic heading change (no lag).
    1 - g is how much less the plant turns than the kinematics, tick by tick."""
    kin = np.concatenate([(s["ds"] * s["k"])[s["history"]:] for s in series])
    true = np.concatenate([s["true_dyaw"][s["history"]:] for s in series])
    g = float(np.dot(kin, true) / np.dot(kin, kin))
    return {"gain": g, "r2": float(1 - ((true - g * kin) ** 2).sum() / ((true - true.mean()) ** 2).sum()),
            "ticks": int(len(kin))}


def best_fit(series: list[dict]) -> dict:
    best = min(((lag, gain) for lag in LAGS for gain in GAINS),
               key=lambda p: float(np.median(np.abs(lag_gain_residual(series, *p)))))
    return {"lag_ticks": best[0], "gain": float(best[1])}


def analyse(runs: list[dict], raw_root: Path) -> dict:
    per_run, series, labels = [], [], []
    for r in runs:
        run = raw_root / r["root"] / r["name"] / "run"
        result = json.loads((run / "result.json").read_text())
        sc = np.load(run / "slam_control.npy")
        with np.load(run / "slam_log.npz") as log:
            split = run_split(result, sc, log, r["t_lift"])
            series.append(kinematic_series(result, sc, log, r["t_lift"]))
        split.update(name=r["name"], scene=r["scene"], arm=r["arm"], dpsi_rad=r["bias_course"]["dpsi_rad"])
        per_run.append(split)
        labels.append((r["scene"], r["arm"]))

    def med(key):
        return float(np.median(np.abs([p[key] for p in per_run])))

    plain = np.array([p["kinematic_minus_truth_yaw_rad"] for p in per_run])
    fits = {"all": {**best_fit(series)}}
    fits["all"]["residual_median_rad"] = float(np.median(np.abs(lag_gain_residual(series, fits["all"]["lag_ticks"],
                                                                                   fits["all"]["gain"]))))
    for name, test in [(s, lambda lab, s=s: lab[0] == s) for s in sorted({lab[0] for lab in labels})] + \
                      [("B0 arm", lambda lab: lab[1] == "B0")]:
        mask = np.array([test(lab) for lab in labels])
        if mask.all() or not mask.any():  # nothing to fit on, or nothing held out
            continue
        fit = best_fit([s for s, m in zip(series, mask) if not m])
        held = lag_gain_residual([s for s, m in zip(series, mask) if m], fit["lag_ticks"], fit["gain"])
        fits[f"held_out:{name}"] = {**fit, "n": int(mask.sum()), "residual_median_rad": float(np.median(np.abs(held))),
                                    "no_model_median_rad": float(np.median(np.abs(plain[mask])))}
    return {
        "n": len(per_run), "tick_s": 1 / 120,
        "recorded_vs_estimate_max_m": max(p["recorded_vs_estimate_m"] for p in per_run),
        "recorded_vs_estimate_max_rad": max(abs(p["recorded_vs_estimate_rad"]) for p in per_run),
        "quantisation_abs_median_rad": med("quantisation_yaw_rad"),
        "quantisation_abs_max_rad": float(max(abs(p["quantisation_yaw_rad"]) for p in per_run)),
        "kinematic_minus_truth_abs_median_rad": med("kinematic_minus_truth_yaw_rad"),
        "kinematic_minus_truth_abs_max_rad": float(np.abs(plain).max()),
        "corr_dpsi_kinematic_minus_truth": float(np.corrcoef([p["dpsi_rad"] for p in per_run], plain)[0, 1]),
        "lag_gain_fits": fits, "tick_gain": tick_gain(series), "runs": per_run,
    }


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
