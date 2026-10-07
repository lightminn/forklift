"""Measured-chassis gate (plan v10, P0a v10) from recorded truth-controlled runs.

    python tools/gate_measure.py estop --run <run dir> [--run ...]
    python tools/gate_measure.py tilt --run <run dir> [--run ...]
    python tools/gate_measure.py capture --run <run dir> [--run ...]
    python tools/gate_measure.py withdraw --run <run dir> [--run ...] [--draws 5]

estop: every emergency-stop probe against the fixed stopping model (latency
0.15 s, deceleration 1.5 m/s^2, margin 0.05 m) and the stopping envelope
(0.01 m): the stop distance, the command-to-deceleration latency, and how far
each corner of the outline (unloaded or loaded) leaves the arc it was on.

tilt: the largest body tilt over every recorded physics tick (the slam_log
base pose, 120 Hz), overall and while a probe was braking (h_lo = 1.05 - 5 m x
tan(max tilt), plan D0 v10).

capture: the recognition's relative error (r_fix of the pickup pallet): the
largest distance between the estimated and the true pallet corners, in the
truck frame at the capture. The runs are truth-controlled, so the control pose
is the truth and the whole error is perception's.

withdraw: at the drop the delivered pallet's relative pose is the pickup
estimate carried on the forks; b_w is the largest corner distance between
that belief and the truth at the release. From the release to the end of the
withdrawal the wheel odometry (the OdometryNoise model redrawn, as in
p0a_measure) drifts against the truth; e_w is the largest lateral error at the
fork blades, e_w_full the largest corner error of the pallet, in the truck
frame. Samples are this run's, not a bound beyond them.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p0a_measure import rear_truth  # noqa: E402

from forklift_core.localization.slam_pose import OdometryNoise  # noqa: E402
from forklift_core.localization.wheel_odometry import (  # noqa: E402
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
)

LATENCY_S, DECEL_MPS2, MARGIN_M, ENVELOPE_M = 0.15, 1.5, 0.05, 0.01
# Fallback rear-axle outlines (front, rear, half width); each run's own geometry is read when
# recorded (the loaded front follows the insertion target: 1.56 m at 16 mm, Codex stage-1 3rd P2).
UNLOADED = (1.29, 0.17, 0.36)
LOADED = (1.56, 0.17, 0.40)


def outlines_of(result):
    g = result.get("geometry") or {}
    def fp(key, default):
        f = g.get(key)
        return (float(f["front_m"]), float(f["rear_m"]), float(f["half_width_m"])) if f else default
    return fp("unloaded_footprint", UNLOADED), fp("loaded_footprint", LOADED)
# dls08_measured blades in the rear-axle frame (x from the carriage face to the tips, y centre).
BLADE_X = (0.944, 1.29)
BLADE_Y = (0.145, -0.145)


def compose(a, b):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], t + b[2])


def invert(p):
    x, y, t = p
    c, s = math.cos(t), math.sin(t)
    return (-(c * x + s * y), s * x - c * y, -t)


def corners(rect_pose, length_m: float, width_m: float):
    return [compose(rect_pose, (u, v, 0.0))[:2] for u in (-length_m / 2, length_m / 2) for v in (-width_m / 2, width_m / 2)]


def corner_error_m(a, b, length_m: float, width_m: float) -> float:
    """Largest distance between matching corners of two poses of one rectangle."""
    return max(math.dist(p, q) for p, q in zip(corners(a, length_m, width_m), corners(b, length_m, width_m)))


def model_distance_m(speed_mps: float) -> float:
    v = abs(speed_mps)
    return v * LATENCY_S + v * v / (2 * DECEL_MPS2) + MARGIN_M


def outline_points(outline):
    front, rear, half = outline
    return [(front, half), (front, -half), (-rear, half), (-rear, -half)]


def corner_envelope_m(trigger_rear, curvature_inv_m: float, trace, outline) -> float:
    """How far each outline corner leaves the circle (or line) it was on at the trigger."""
    worst = 0.0
    for px, py in outline_points(outline):
        p0 = compose(trigger_rear, (px, py, 0.0))
        if abs(curvature_inv_m) < 1e-9:
            x0, y0, yaw = trigger_rear
            for row in trace:
                q = compose((row[2], row[3], row[4]), (px, py, 0.0))
                worst = max(worst, abs(-math.sin(yaw) * (q[0] - p0[0]) + math.cos(yaw) * (q[1] - p0[1])))
            continue
        x0, y0, yaw = trigger_rear
        r = 1.0 / curvature_inv_m
        centre = (x0 - r * math.sin(yaw), y0 + r * math.cos(yaw))
        radius = math.dist(centre, p0[:2])
        for row in trace:
            q = compose((row[2], row[3], row[4]), (px, py, 0.0))
            worst = max(worst, abs(math.dist(centre, q[:2]) - radius))
    return worst


def estop_command(args) -> dict:
    rows = []
    for run in args.run:
        result = json.loads((run / "result.json").read_text())
        unloaded, loaded = outlines_of(result)
        for probe in result.get("estop_probes") or []:
            v = float(probe["trigger_speed_mps"])
            outline = loaded if probe["loaded"] else unloaded
            rows.append({
                "run": run.name if run.name != "run" else run.parent.name,
                "phase": probe["phase"], "direction": probe["direction"], "loaded": bool(probe["loaded"]),
                "curvature_inv_m": float(probe["curvature_inv_m"]), "speed_mps": v,
                "latency_s": probe["decel_start_s"], "stop_distance_m": float(probe["stop_distance_m"]),
                "stop_time_s": float(probe["stop_time_s"]),
                "model_distance_m": model_distance_m(v),
                "rear_arc_offset_m": float(probe["max_arc_offset_m"]),
                "corner_offset_m": corner_envelope_m(probe["trigger_rear"], float(probe["curvature_inv_m"]),
                                                     probe["trace"], outline),
            })
    moving = [r for r in rows if abs(r["speed_mps"]) >= 0.05]
    if not moving:
        # No measurement is not a pass (Codex stage-1 4th P2).
        return {"model": {"latency_s": LATENCY_S, "decel_mps2": DECEL_MPS2, "margin_m": MARGIN_M,
                          "envelope_m": ENVELOPE_M}, "probes": len(rows), "model_holds": None, "rows": rows}
    return {
        "model": {"latency_s": LATENCY_S, "decel_mps2": DECEL_MPS2, "margin_m": MARGIN_M, "envelope_m": ENVELOPE_M},
        "probes": len(rows),
        "max_latency_s": max((r["latency_s"] for r in rows if r["latency_s"] is not None), default=None),
        "worst_model_slack_m": min((r["model_distance_m"] - r["stop_distance_m"] for r in moving), default=None),
        "max_corner_offset_m": max((r["corner_offset_m"] for r in rows), default=None),
        "model_holds": all(r["stop_distance_m"] <= r["model_distance_m"] for r in rows)
        and all(r["corner_offset_m"] <= ENVELOPE_M for r in rows),
        "classes": sorted({f"{'loaded' if r['loaded'] else 'empty'}/{r['direction']}/"
                           f"{'curve' if abs(r['curvature_inv_m']) > 0.05 else 'straight'}" for r in moving}),
        "rows": rows,
    }


def body_tilt_rad(quaternion_wxyz) -> np.ndarray:
    """Angle between the body z axis and the vertical, per row of (w, x, y, z)."""
    q = np.asarray(quaternion_wxyz, float)
    z_up = 1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2)
    return np.arccos(np.clip(z_up, -1.0, 1.0))


def tilt_command(args) -> dict:
    """Every physics tick of the recorded base pose (120 Hz), not the 0.1 s samples
    (Codex stage-1 4th P2: a peak between samples was missed)."""
    overall, braking, per_run = 0.0, 0.0, []
    for run in args.run:
        result = json.loads((run / "result.json").read_text())
        log_path = run / "slam_log.npz"
        if not log_path.exists():
            raise SystemExit(f"{run}: no slam_log.npz -- the tilt needs every tick")
        log = np.load(log_path)
        stamps = log["joint_stamps_s"]
        quats = np.asarray(log["base_pose_world"], float)[:, 3:7]
        tilt = body_tilt_rad(quats)
        if not len(tilt):
            raise SystemExit(f"{run}: empty pose log")
        if not (np.isfinite(quats).all() and np.isfinite(tilt).all()):
            raise SystemExit(f"{run}: non-finite pose in the log (Codex stage-1 5th P2)")
        windows = [(p["trigger_time_s"], p["trigger_time_s"] + p["stop_time_s"] + 0.5)
                   for p in result.get("estop_probes") or []]
        mask = np.zeros(len(stamps), bool)
        for a, b in windows:
            mask |= (stamps >= a) & (stamps <= b)
        run_max = float(tilt.max())
        run_brake = float(tilt[mask].max()) if mask.any() else None
        overall = max(overall, run_max)
        braking = max(braking, run_brake) if run_brake is not None else braking
        per_run.append({"run": str(run), "ticks": int(len(tilt)), "max_tilt_rad": run_max,
                        "braking_tilt_rad": run_brake})
    if not per_run:
        raise SystemExit("no runs")
    h_lo = 1.05 - 5.0 * math.tan(overall)
    return {"max_tilt_rad": overall, "max_tilt_deg": math.degrees(overall), "braking_tilt_rad": braking,
            "h_lo_m": h_lo, "runs": per_run}


def _pallet_dims(result):
    pg = result["arguments"]["pallet_geometry_loaded"]
    return float(pg["overall_depth_m"]), float(pg["overall_width_m"])


def _truth_pallet(result):
    p = result["scenario"]["pickup"]
    return (float(p["x_m"]), float(p["y_m"]), float(p["yaw_rad"]))


def _estimate(result):
    e = result["perception"]["perception_pickup_estimate_m"]
    return (float(e["x_m"]), float(e["y_m"]), float(e["yaw_rad"]))


def capture_command(args) -> dict:
    rows = []
    for run in args.run:
        result = json.loads((run / "result.json").read_text())
        depth, width = _pallet_dims(result)
        truth, est = _truth_pallet(result), _estimate(result)
        # Truth-controlled: the estimate is in the world frame; the corner error is frame-free.
        rows.append({"run": str(run), "corner_error_m": corner_error_m(est, truth, depth, width),
                     "centre_error_m": math.dist(est[:2], truth[:2]),
                     "yaw_error_rad": math.atan2(math.sin(est[2] - truth[2]), math.cos(est[2] - truth[2]))})
    return {"r_fix_sample_max_m": max((r["corner_error_m"] for r in rows), default=None), "runs": rows}


def _transition(result, to_phase: str) -> float:
    for t in result["transitions"]:
        if t["to"] == to_phase:
            return float(t["time_s"])
    raise ValueError(f"no transition to {to_phase}")


def _sample_at(result, t: float):
    samples = result["samples"]
    s = min(samples, key=lambda row: abs(row["time_s"] - t))
    return s


def withdraw_command(args) -> dict:
    if args.draws < 1:
        raise SystemExit("--draws must be at least 1 (no redraw is no measurement, Codex stage-1 5th P2)")
    rows = []
    for run in args.run:
        result = json.loads((run / "result.json").read_text())
        depth, width = _pallet_dims(result)
        t_lift, t_release = _transition(result, "lift"), _transition(result, "withdraw")
        t_end = next(float(t["time_s"]) for t in result["transitions"] if t["from"] == "withdraw")
        log = np.load(run / "slam_log.npz")
        meta = json.loads((run / "meta.json").read_text())
        g = meta["odometry_geometry"]
        geom = AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"])
        stamps = log["joint_stamps_s"]
        all_truth = rear_truth(log["base_pose_world"].astype(float), float(g["rear_axle_x_in_base_m"]))
        sample_t = np.array([row["time_s"] for row in result["samples"]])
        sample_p = np.array([[row["pallet_position_m"][0], row["pallet_position_m"][1],
                              row["pallet_yaw_rad"]] for row in result["samples"]])

        def truth_rear_at(t):  # every tick, at the transition itself (Codex stage-1 5th P2)
            return tuple(float(np.interp(t, stamps, all_truth[:, k])) for k in range(3))

        def pallet_at(t):
            return (float(np.interp(t, sample_t, sample_p[:, 0])), float(np.interp(t, sample_t, sample_p[:, 1])),
                    float(np.interp(t, sample_t, np.unwrap(sample_p[:, 2]))))

        believed = compose(invert(truth_rear_at(t_lift)), _estimate(result))
        actual = compose(invert(truth_rear_at(t_release)), pallet_at(t_release))
        b_w = corner_error_m(believed, actual, depth, width)
        i0, i1 = int(np.searchsorted(stamps, t_release)), int(np.searchsorted(stamps, t_end + 1.0))
        truth = all_truth[i0:i1]
        rates = log["wheel_rates_rad_s"].astype(float)[i0:i1, 2:4]
        steering = log["steering_rad"].astype(float)[i0:i1]
        blade = [(x, y) for x in BLADE_X for y in BLADE_Y]
        # The real pallet after the release (it may slide as the forks leave): 0.1 s samples
        # interpolated to every tick (Codex stage-1 4th P2: a 50 mm slide read as 0).
        ticks = stamps[i0:i1]
        pallet_world = np.column_stack([np.interp(ticks, sample_t, sample_p[:, 0]),
                                        np.interp(ticks, sample_t, sample_p[:, 1]),
                                        np.interp(ticks, sample_t, np.unwrap(sample_p[:, 2]))])
        slide = max(math.dist(pallet_world[k, :2], pallet_world[0, :2]) for k in range(len(pallet_world)))
        e_w = e_full = 0.0
        for draw in range(args.draws):
            noise = OdometryNoise(seed=30_000 + draw)
            odom = integrate_wheel_odometry(
                stamps[i0:i1], rates + noise._wheels.normal(0, noise.wheel_rate_std_rad_s, rates.shape),
                steering + noise._steering.normal(0, noise.steering_std_rad, steering.shape), geom,
                initial_pose=tuple(truth[0]))
            t0, o0 = tuple(truth[0]), tuple(odom[0])
            actual0 = compose(invert(t0), tuple(pallet_world[0]))  # the pallet in the release truck frame
            for k in range(len(truth)):
                tr = compose(invert(t0), tuple(truth[k]))
                od = compose(invert(o0), tuple(odom[k]))
                # A point fixed at the release, seen from the current truck: believed vs actual.
                for p in blade:
                    pb, pa = compose(invert(od), (*p, 0.0)), compose(invert(tr), (*p, 0.0))
                    e_w = max(e_w, abs(pb[1] - pa[1]))
                # Believed: the release pose carried by odometry; actual: the true pallet now,
                # both in the truck frame of this tick (the release frame t0 for truth).
                believed = compose(invert(od), actual0)
                now_pallet = compose(invert(t0), tuple(pallet_world[k]))
                actual_now = compose(invert(tr), now_pallet)
                e_full = max(e_full, corner_error_m(believed, actual_now, depth, width))
        clear = result.get("lateral_clearance_min_m", {}).get("withdraw")
        rows.append({"run": str(run), "b_w_m": b_w, "e_w_m": e_w, "e_w_full_m": e_full, "pallet_slide_m": slide,
                     "withdraw_s": t_end - t_release, "truth_clearance_m": clear})
    if not rows:
        raise SystemExit("no runs")
    return {"b_w_sample_max_m": max((r["b_w_m"] for r in rows), default=None),
            "e_w_sample_max_m": max((r["e_w_m"] for r in rows), default=None),
            "e_w_full_sample_max_m": max((r["e_w_full_m"] for r in rows), default=None),
            "draws": args.draws, "runs": rows}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("estop", "tilt", "capture", "withdraw"):
        p = sub.add_parser(name)
        p.add_argument("--run", type=Path, action="append", required=True)
        if name == "withdraw":
            p.add_argument("--draws", type=int, default=5)
    args = parser.parse_args(argv)
    out = {"estop": estop_command, "tilt": tilt_command, "capture": capture_command,
           "withdraw": withdraw_command}[args.command](args)
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
