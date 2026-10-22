"""Split a run's wheel-odometry drift over a window into its terms (plan 2026-10-21 task A).

From the run's ``slam_log.npz`` (the 120 Hz wheel rates, wheel joint angles, steering joint angles and the truth
base pose) and its ``result.json`` (the drive geometry, the rear-axle offset, the odometry start and noise seed):

1. the recorded odometry is rebuilt -- the reported rear-wheel rates and steering angles with the run's noise
   drawn from the first sample on -- and checked against every recorded ``odom_base`` (``slam_records.json``);
2. three more trajectories are integrated over the same samples: the same without noise, and the wheel joint
   angles' increments (the rotation the wheels really made) with the true steering;
3. over a window (default: the approach transition to the insert transition) each trajectory's displacement in
   its own start frame is compared with the truth rear axle's. The telescoping differences are the terms:

   - ``noise``        recorded - noiseless (the injected wheel-rate and steering noise)
   - ``rate_angle``   noiseless - joint-angle (the reported wheel rates against the joint angles' change)
   - ``angle_truth``  joint-angle - truth (rolling distance and steering geometry against the body's motion)

   They sum to the recorded odometry's drift exactly. Forward, lateral (the window's start frame) and yaw.

A team-spec run (meta.json ``encoder_model`` other than reported, plan 2026-10-22 joint-angle odometry) has no
wheel-rate noise: its recorded odometry is ``update_increments`` over the MT6701 counted increments and the
KTH7801 measured steering (regenerated and checked by ``encoder_odometry_inputs``). Its terms are ``noise`` (0),
``quantize`` (recorded - joint-angle: the counting, its hysteresis and the steering quantisation) and
``angle_truth``.

Each run's window gets a class -- ``complete`` (the end phase was left), ``mid_failure`` (the start was reached,
the end phase never left), ``no_window`` or ``reproduction_failure`` -- and only complete, reproduced windows
enter ``summary``. ``--hold`` uses the hold window instead: the accepted near capture to the insert's end.

The terms are a bookkeeping split, not a causal one: a term is where the difference appears, and the physics
behind a large ``rate_angle`` or ``angle_truth`` (joint velocity reporting, tyre contact, steering lag) needs
its own evidence.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from forklift_core.localization.encoders import encoder_odometry_inputs  # noqa: E402
from forklift_core.localization.slam_pose import IncrementalWheelOdometry, OdometryNoise  # noqa: E402
from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry  # noqa: E402


def yaw_of(q_wxyz) -> float:
    w, x, y, z = (float(v) for v in q_wxyz)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def relative(a, b) -> np.ndarray:
    """b in a's frame (x forward, y left, yaw)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))])


def rear_from_base(base, offset_m: float):
    """The rear axle pose from the base pose (the rear axle sits ``offset_m`` along the base x, negative behind)."""
    x, y, yaw = base
    return (x + offset_m * math.cos(yaw), y + offset_m * math.sin(yaw), yaw)


def base_from_rear(rear, offset_m: float):
    x, y, yaw = rear
    return (x - offset_m * math.cos(yaw), y - offset_m * math.sin(yaw), yaw)


def trajectories(log: dict, result: dict, meta: dict | None = None) -> dict:
    """The four rear-axle trajectories on the log's stamps: recorded (rebuilt with noise, or from the counted
    increments on a team-spec run), noiseless, joint-angle and truth. ``meta`` is the run's meta.json (None or
    no ``encoder_model``: reported)."""
    args = result["arguments"]
    drive = args["drive_geometry"]
    radius = (float(args["odometry_wheel_radius_m"]) if args.get("odometry_wheel_radius_m") is not None
              else float(drive["wheel_radius_m"]))
    geometry = AckermannOdometryGeometry(float(drive["wheelbase_m"]), float(drive["track_m"]), radius)
    offset = float(args["rear_axle_offset_m"])
    start = tuple(float(v) for v in result["slam_feedback"]["start_rear"])
    seed = args.get("slam_noise_seed")
    if args.get("slam_noise_spec"):
        raise ValueError("a --slam-noise-spec run is not rebuilt here (its steering sigma differs)")
    inputs = encoder_odometry_inputs(log, meta or {})
    counted = inputs["kind"] == "increments"
    noise = OdometryNoise(seed=seed or 0, enabled=seed is not None and not counted)
    stamps = np.asarray(log["joint_stamps_s"], float)
    rates = np.asarray(log["wheel_rates_rad_s"])
    steer = np.asarray(log["steering_rad"], float)
    angles = np.asarray(log["wheel_angles_rad"], float)
    poses = np.asarray(log["base_pose_world"], float)
    recorded = IncrementalWheelOdometry(geometry, initial_pose=start)
    clean = IncrementalWheelOdometry(geometry, initial_pose=start)
    joint = IncrementalWheelOdometry(geometry, initial_pose=start)
    out = {k: np.empty((len(stamps), 3)) for k in ("recorded", "noiseless", "joint_angle", "truth")}
    for i, t in enumerate(stamps):
        if counted:
            out["recorded"][i] = recorded.update_increments(t, inputs["increments"][i], inputs["steering"][i])
            out["noiseless"][i] = out["recorded"][i]
        else:
            out["recorded"][i] = recorded.update(t, noise.wheel_rates(rates[i, 2:4]), noise.steering(steer[i]))
            out["noiseless"][i] = clean.update(t, rates[i, 2:4], steer[i])
        inc = angles[i, 2:4] - angles[i - 1, 2:4] if i else np.zeros(2)
        out["joint_angle"][i] = joint.update_increments(t, inc, steer[i])
        base = (poses[i, 0], poses[i, 1], yaw_of(poses[i, 3:7]))
        out["truth"][i] = rear_from_base(base, offset)
    out["stamps"] = stamps
    out["offset_m"] = offset
    out["encoder_model"] = (meta or {}).get("encoder_model", {"name": "reported"})["name"]
    out["input_term"] = "quantize" if counted else "rate_angle"
    # The rolled distances themselves (mean of the two rear wheels): the reported rates' trapezoid and the joint
    # angles' change, cumulative on the stamps; the truth rear axle's path length.
    rate_mean = rates[:, 2:4].astype(float).mean(axis=1) * radius
    dt = np.diff(stamps)
    out["reported_rate_dist"] = np.concatenate([[0.0], np.cumsum((rate_mean[1:] + rate_mean[:-1]) / 2 * dt)])
    out["joint_angle_dist"] = (angles[:, 2:4].mean(axis=1) - angles[0, 2:4].mean()) * radius
    step = np.hypot(np.diff(out["truth"][:, 0]), np.diff(out["truth"][:, 1]))
    out["truth_path"] = np.concatenate([[0.0], np.cumsum(step)])
    return out


def reproduction(traj: dict, records: list, *, tolerance_m: float = 1e-9, tolerance_rad: float = 1e-9) -> dict:
    """The rebuilt recorded odometry against every recorded ``odom_base``: how many were compared, how many had no
    log sample at their stamp, the largest position and yaw differences. ``ok`` needs at least one comparison, no
    unmatched record and both differences within tolerance."""
    stamps = traj["stamps"]
    compared = unmatched = 0
    worst_m = worst_rad = 0.0
    for r in records:
        if r.get("odom_base") is None:
            continue
        i = int(np.argmin(np.abs(stamps - r["stamp_s"])))
        if abs(stamps[i] - r["stamp_s"]) > 1e-6:
            unmatched += 1
            continue
        compared += 1
        b = base_from_rear(traj["recorded"][i], traj["offset_m"])
        worst_m = max(worst_m, math.hypot(b[0] - r["odom_base"][0], b[1] - r["odom_base"][1]))
        worst_rad = max(worst_rad, abs(math.atan2(math.sin(b[2] - r["odom_base"][2]), math.cos(b[2] - r["odom_base"][2]))))
    return {"compared": compared, "unmatched": unmatched, "max_position_m": worst_m, "max_yaw_rad": worst_rad,
            "ok": compared > 0 and unmatched == 0 and worst_m <= tolerance_m and worst_rad <= tolerance_rad}


def terms(traj: dict, t0: float, t1: float) -> dict:
    """The window's displacement terms (forward, lateral in the start frame; yaw), in metres and radians."""
    stamps = traj["stamps"]
    i0, i1 = int(np.searchsorted(stamps, t0)), int(np.searchsorted(stamps, t1))
    i1 = min(i1, len(stamps) - 1)
    disp = {k: relative(traj[k][i0], traj[k][i1]) for k in ("recorded", "noiseless", "joint_angle", "truth")}
    split = {"noise": disp["recorded"] - disp["noiseless"],
             traj.get("input_term", "rate_angle"): disp["noiseless"] - disp["joint_angle"],
             "angle_truth": disp["joint_angle"] - disp["truth"]}
    total = disp["recorded"] - disp["truth"]
    dist = {"reported_rate_m": float(traj["reported_rate_dist"][i1] - traj["reported_rate_dist"][i0]),
            "joint_angle_m": float(traj["joint_angle_dist"][i1] - traj["joint_angle_dist"][i0]),
            "truth_path_m": float(traj["truth_path"][i1] - traj["truth_path"][i0])}
    return {"window_s": [float(stamps[i0]), float(stamps[i1])],
            "truth_forward_m": float(disp["truth"][0]), "distance": dist,
            "drift": {"forward_m": float(total[0]), "lateral_m": float(total[1]), "yaw_rad": float(total[2])},
            "terms": {k: {"forward_m": float(v[0]), "lateral_m": float(v[1]), "yaw_rad": float(v[2])}
                      for k, v in split.items()},
            "sum_residual_m": float(np.abs(total[:2] - sum(split.values())[:2]).max())}


def worst_window(traj: dict, t0: float, t1: float, width_s: float = 0.3) -> dict:
    """The width_s window inside [t0, t1] with the largest forward input term (rate_angle or quantize)."""
    best, key = None, traj.get("input_term", "rate_angle")
    for a in np.arange(t0, max(t0, t1 - width_s) + 1e-9, 0.05):
        w = terms(traj, a, a + width_s)
        v = w["terms"][key]["forward_m"]
        if best is None or abs(v) > abs(best["terms"][key]["forward_m"]):
            best = w
    return best


def phase_window(result: dict, start_phase: str = "approach", end_phase: str = "insert") -> tuple[float, float]:
    """From the start phase's transition to the end of the end phase (the next transition after it)."""
    tr = result["transitions"]
    t0 = next(x["time_s"] for x in tr if x["to"] == start_phase)
    after = [x for x in tr if x["from"] == end_phase]
    t1 = after[0]["time_s"] if after else next(x["time_s"] for x in tr if x["to"] == end_phase)
    return float(t0), float(t1)


def near_capture_time(result: dict) -> float | None:
    """The accepted near capture's control time (its observation attempt), None without one."""
    near = result.get("near_capture") or {}
    attempt = near.get("near_attempt")
    attempts = result.get("observation_attempts") or []
    if near.get("status") != "done" or not attempt or not 1 <= int(attempt) <= len(attempts):
        return None
    control = attempts[int(attempt) - 1].get("capture_control") or {}
    return None if control.get("time_s") is None else float(control["time_s"])


def window_status(result: dict, start_phase: str = "approach", end_phase: str = "insert",
                  *, hold: bool = False) -> tuple[str, float | None, float | None]:
    """(class, t0, t1). ``complete``: the start was reached and the end phase left; ``mid_failure``: the start was
    reached but the end phase never left (phase_window would end it at the insert's start, plan 2026-10-22 P2);
    ``no_window``: the start never came. ``hold``: the start is the accepted near capture instead."""
    tr = result.get("transitions") or []
    if hold:
        t0 = near_capture_time(result)
    else:
        t0 = next((float(x["time_s"]) for x in tr if x["to"] == start_phase), None)
    if t0 is None:
        return "no_window", None, None
    t1 = next((float(x["time_s"]) for x in tr if x["from"] == end_phase), None)
    if t1 is None:
        return "mid_failure", t0, None
    return "complete", t0, t1


def analyse(run_dir: Path, start_phase: str = "approach", end_phase: str = "insert") -> dict:
    """The run's class, its approach-to-insert window and its hold window (each None unless complete)."""
    result = json.loads((run_dir / "result.json").read_text())
    meta_path = run_dir / "meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else None
    log = dict(np.load(run_dir / "slam_log.npz", allow_pickle=False))
    row = {"run": str(run_dir), "encoder_model": (meta or {}).get("encoder_model", {"name": "reported"})["name"],
           "window": None, "worst_0p3s": None, "hold": None}
    status, t0, t1 = window_status(result, start_phase, end_phase)
    hold_status, h0, h1 = window_status(result, start_phase, end_phase, hold=True)
    row["hold_status"] = hold_status
    if status == "no_window":
        return {**row, "status": status, "reproduction": None}
    traj = trajectories(log, result, meta)
    records = json.loads((run_dir / "slam_records.json").read_text())
    row["reproduction"] = reproduction(traj, records)
    if not row["reproduction"]["ok"]:
        return {**row, "status": "reproduction_failure"}
    if status == "complete":
        row["window"], row["worst_0p3s"] = terms(traj, t0, t1), worst_window(traj, t0, t1)
    if hold_status == "complete":
        row["hold"] = terms(traj, h0, h1)
    return {**row, "status": status}


def summary(rows: list) -> dict:
    """Per encoder model: the window classes counted, and over the complete, reproduced runs with a hold window the
    hold forward drift's |median| and its largest magnitude (signed values would hide a negative worst case)."""
    out = {}
    for name in sorted({r["encoder_model"] for r in rows}):
        group = [r for r in rows if r["encoder_model"] == name]
        counts: dict = {}
        for r in group:
            counts[r["status"]] = counts.get(r["status"], 0) + 1
        drifts = [r["hold"]["drift"]["forward_m"] for r in group if r["status"] == "complete" and r.get("hold")]
        out[name] = {"counts": counts, "valid": len(drifts),
                     "hold_forward_abs_median_m": float(abs(np.median(drifts))) if drifts else None,
                     "hold_forward_max_abs_m": float(max(abs(d) for d in drifts)) if drifts else None}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("runs", nargs="+", type=Path, help="run directories (each with result.json and slam_log.npz)")
    ap.add_argument("--start-phase", default="approach")
    ap.add_argument("--end-phase", default="insert")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args(argv)
    rows = []
    for run in args.runs:
        name = run.parent.name if run.name == "run" else run.name
        try:
            row = analyse(run, args.start_phase, args.end_phase)
        except (FileNotFoundError, KeyError, ValueError) as exc:
            # A run whose record cannot be read or rebuilt counts as a reproduction failure, never as a pass.
            row = {"run": str(run), "encoder_model": "unknown", "status": "reproduction_failure",
                   "error": f"{type(exc).__name__}: {exc}", "window": None, "hold": None}
        rows.append(row)
        if row["status"] != "complete":
            print(f"{name:18s} {row['status']} {row.get('error', '')}")
            continue
        w, rp, d = row["window"], row["reproduction"], row["window"]["distance"]
        key = next(k for k in w["terms"] if k not in ("noise", "angle_truth"))
        k = w["terms"]
        hold = row["hold"]
        print(f"{name:18s} {row['encoder_model']} repro=ok({rp['compared']}, {rp['max_position_m']:.1e} m) "
              f"rolled rate {(d['reported_rate_m'] - d['truth_path_m'])*1000:+6.1f} / angle "
              f"{(d['joint_angle_m'] - d['truth_path_m'])*1000:+6.1f} mm vs path | "
              f"truth={w['truth_forward_m']:.3f} m drift={w['drift']['forward_m']*1000:+6.1f} mm = "
              f"noise {k['noise']['forward_m']*1000:+6.1f} + {key} {k[key]['forward_m']*1000:+6.1f} "
              f"+ angle_truth {k['angle_truth']['forward_m']*1000:+6.1f} | hold "
              + (f"{hold['drift']['forward_m']*1000:+6.1f} mm" if hold else row["hold_status"]))
    print(json.dumps(summary(rows), indent=1))
    if args.json is not None:
        args.json.write_text(json.dumps({"rows": rows, "summary": summary(rows)}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
