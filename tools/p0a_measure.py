"""P0a measurements for the priority-5 plan (docs/plans/2026-10-04-lidar-obstacle-map.md).

    python tools/p0a_measure.py odometry --run <run dir> [--run ...] [--draws 10]
    python tools/p0a_measure.py age --run <run dir> [--run ...] [--draws 5]
    python tools/p0a_measure.py lift --run <run dir> [--run ...]
    python tools/p0a_measure.py standoff [--depth-m 0.36] [--band-m 0.10]

odometry: the relative wheel-odometry error after travelling d metres, at the
rear axle (the reference the plan's r = e(d) + 2 rho sin(psi(d)/2) uses). The
recorded joint rates are the true ones (noise is drawn online), so the online
noise model (OdometryNoise: 0.2 rad/s per wheel sample, 0.005 rad steering) is
drawn again here several times; truth is the recorded base_link pose moved to
the rear axle. Reports per-distance-bin maxima and the cheapest line
e0 + k d above all of them.

lift: pallet origin height and lift joint per phase from result.json samples,
which explains the gap between the commanded lift_target_m and the carried
pallet's height.

standoff: how deep into an EPAL 6 fork opening a carriage camera can see
under the deck (L(D)), for the mount the S3 runs used, and the camera-to-face
distance D needed to see the whole fork volume of the insertion before it
starts.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from forklift_core.localization.slam_pose import OdometryNoise
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
)

DISTANCE_BINS_M = np.arange(0.0, 12.01, 0.5)


def upper_line(x, y) -> tuple[float, float]:
    """Cheapest e0 + k x (e0, k >= 0) on or above every (x, y): least summed height."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    best = (float(y.max()), 0.0)
    best_cost = best[0] * len(x)
    for i in range(len(x)):
        for j in range(i + 1, len(x)):
            if x[j] == x[i]:
                continue
            k = (y[j] - y[i]) / (x[j] - x[i])
            e0 = y[i] - k * x[i]
            if k < 0 or e0 < 0 or np.any(e0 + k * x < y - 1e-12):
                continue
            cost = float(np.sum(e0 + k * x))
            if cost < best_cost:
                best, best_cost = (float(e0), float(k)), cost
    return best


def _yaw(quaternion_wxyz) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion_wxyz, float).T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rear_truth(base_pose_world, rear_axle_x_in_base_m: float) -> np.ndarray:
    yaw = np.unwrap(_yaw(base_pose_world[:, 3:7]))
    x = base_pose_world[:, 0] + rear_axle_x_in_base_m * np.cos(yaw)
    y = base_pose_world[:, 1] + rear_axle_x_in_base_m * np.sin(yaw)
    return np.column_stack((x, y, yaw))


def relative_errors(truth, odometry, *, start_every: int, max_distance_m: float, stamps=None, max_age_s=None):
    """(distance travelled, position error, |yaw error|) for windows from every start_every-th sample.

    With stamps and max_age_s a window also ends max_age_s after its start: the
    grid keeps an observation for T seconds, so older windows do not matter.
    """
    step = np.hypot(*np.diff(truth[:, :2], axis=0).T)
    s = np.concatenate(([0.0], np.cumsum(step)))
    out_d, out_p, out_y = [], [], []
    for i in range(0, len(truth) - 1, start_every):
        j_end = int(np.searchsorted(s, s[i] + max_distance_m, side="right"))
        if stamps is not None and max_age_s is not None:
            j_end = min(j_end, int(np.searchsorted(stamps, stamps[i] + max_age_s, side="right")))
        j = np.arange(i + 1, min(j_end, len(truth)))
        if len(j) == 0:
            continue
        rel = []
        for poses in (truth, odometry):
            c, sn = math.cos(poses[i, 2]), math.sin(poses[i, 2])
            dx, dy = poses[j, 0] - poses[i, 0], poses[j, 1] - poses[i, 1]
            rel.append((c * dx + sn * dy, -sn * dx + c * dy, poses[j, 2] - poses[i, 2]))
        out_d.append(s[j] - s[i])
        out_p.append(np.hypot(rel[0][0] - rel[1][0], rel[0][1] - rel[1][1]))
        out_y.append(np.abs(np.angle(np.exp(1j * (rel[0][2] - rel[1][2])))))
    return np.concatenate(out_d), np.concatenate(out_p), np.concatenate(out_y)


def binned_max(d, value, bins=DISTANCE_BINS_M):
    index = np.digitize(d, bins) - 1
    upper = bins[1:]
    maxima = np.full(len(upper), np.nan)
    for b in range(len(upper)):
        sel = index == b
        if sel.any():
            maxima[b] = float(value[sel].max())
    return upper, maxima


def odometry_command(args) -> dict:
    per_run = []
    all_d, all_p, all_y = [], [], []
    for run in args.run:
        log = np.load(run / "slam_log.npz")
        meta = json.loads((run / "meta.json").read_text())
        g = meta["odometry_geometry"]
        geometry = AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"])
        truth = rear_truth(log["base_pose_world"].astype(float), float(g["rear_axle_x_in_base_m"]))
        stamps = log["joint_stamps_s"]
        rates = log["wheel_rates_rad_s"].astype(float)[:, 2:4]
        steering = log["steering_rad"].astype(float)
        for draw in range(args.draws):
            noise = OdometryNoise(seed=10_000 + draw)
            noisy_rates = rates + noise._wheels.normal(0, noise.wheel_rate_std_rad_s, rates.shape)
            noisy_steer = steering + noise._steering.normal(0, noise.steering_std_rad, steering.shape)
            odom = integrate_wheel_odometry(
                stamps, noisy_rates, noisy_steer, geometry, initial_pose=tuple(truth[0])
            )
            d, p, y = relative_errors(
                truth, odom, start_every=args.start_every, max_distance_m=12.0,
                stamps=stamps, max_age_s=args.max_age_s,
            )
            all_d.append(d), all_p.append(p), all_y.append(y)
        per_run.append({"run": str(run), "samples": int(len(stamps)), "path_m": float(
            np.hypot(*np.diff(truth[:, :2], axis=0).T).sum())})
    d, p, y = np.concatenate(all_d), np.concatenate(all_p), np.concatenate(all_y)
    upper, p_max = binned_max(d, p)
    _, y_max = binned_max(d, y)
    keep = np.isfinite(p_max)
    e0, k = upper_line(upper[keep], p_max[keep])
    y0, ky = upper_line(upper[keep], y_max[keep])
    return {
        "runs": per_run,
        "draws": args.draws,
        "noise": {"wheel_rate_std_rad_s": 0.2, "steering_std_rad": 0.005},
        "reference": "rear axle centre",
        "max_age_s": args.max_age_s,
        "bins_upper_m": upper.tolist(),
        "position_error_max_m": p_max.tolist(),
        "yaw_error_max_rad": y_max.tolist(),
        "e_line": {"e0_m": e0, "k_m_per_m": k},
        "psi_line": {"psi0_rad": y0, "k_rad_per_m": ky},
        "windows": int(len(d)),
    }


AGES_S = (0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 3.0, 5.0, 10.0)


def relative_error_at(t0_index, ages_s, stamps, truth, estimate):
    """Largest (position, |yaw|) error of the pose at t0 re-projected from any
    tick up to each age -- every tick, not only the listed ages, so the bound
    holds between them (Codex checkpoint 6: a 0.19 s age beat the 0.2 s value).

    The grid places an old scan with the correction of the present (Codex L0
    P1): C = truth(now) o odom(now)^-1, then C o odom(then). This is compared
    with truth(then) -- not the relative motion seen from the old pose, which
    hides a heading error that swings the old pose around the present one.
    NaN for an age whose end lies past the record.
    """
    out = np.full((len(ages_s), 2), np.nan)
    i = t0_index
    last = int(np.searchsorted(stamps, stamps[i] + ages_s[-1], side="right"))
    j = np.arange(i, min(last, len(stamps)))
    c_yaw = truth[j, 2] - estimate[j, 2]
    c, sn = np.cos(c_yaw), np.sin(c_yaw)
    dx, dy = estimate[i, 0] - estimate[j, 0], estimate[i, 1] - estimate[j, 1]
    px = truth[j, 0] + c * dx - sn * dy
    py = truth[j, 1] + sn * dx + c * dy
    pos = np.hypot(px - truth[i, 0], py - truth[i, 1])
    e_yaw = c_yaw + estimate[i, 2] - truth[i, 2]
    yaw = np.abs(np.arctan2(np.sin(e_yaw), np.cos(e_yaw)))
    for k, a in enumerate(ages_s):
        end = int(np.searchsorted(stamps, stamps[i] + a))  # the tick the age reads
        if end >= len(stamps):
            break
        m = j <= end
        out[k] = (float(np.max(pos[m])), float(np.max(yaw[m])))
    return out


def cumulative_table(maxima):
    """Per age, the largest error of any age up to it (a later age bounds an earlier one)."""
    return np.maximum.accumulate(np.nan_to_num(maxima, nan=0.0), axis=0)


def age_command(args) -> dict:
    """Odometry error by observation age, from every recorded scan stamp (Codex P0a)."""
    maxima = np.zeros((len(AGES_S), 2))
    per_run = []
    for run in args.run:
        log = np.load(run / "slam_log.npz")
        meta = json.loads((run / "meta.json").read_text())
        g = meta["odometry_geometry"]
        geometry = AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"])
        truth = rear_truth(log["base_pose_world"].astype(float), float(g["rear_axle_x_in_base_m"]))
        stamps = log["joint_stamps_s"]
        starts = np.searchsorted(stamps, log["scan_stamps_s"])
        starts = starts[starts < len(stamps)]
        rates = log["wheel_rates_rad_s"].astype(float)[:, 2:4]
        steering = log["steering_rad"].astype(float)
        run_max = np.zeros_like(maxima)
        for draw in range(args.draws):
            noise = OdometryNoise(seed=20_000 + draw)
            odom = integrate_wheel_odometry(
                stamps,
                rates + noise._wheels.normal(0, noise.wheel_rate_std_rad_s, rates.shape),
                steering + noise._steering.normal(0, noise.steering_std_rad, steering.shape),
                geometry,
                initial_pose=tuple(truth[0]),
            )
            for i in starts:
                run_max = np.fmax(run_max, relative_error_at(i, AGES_S, stamps, truth, odom))
        maxima = np.fmax(maxima, run_max)
        per_run.append({"run": str(run), "scan_starts": int(len(starts)), "max": run_max.tolist()})
    table = cumulative_table(maxima)
    return {
        "source": "wheel odometry (rear wheel rates + steering), OdometryNoise model redrawn",
        "reference": "rear axle centre",
        "draws": args.draws,
        "ages_s": list(AGES_S),
        "position_max_m": maxima[:, 0].tolist(),
        "yaw_max_rad": maxima[:, 1].tolist(),
        "cumulative_position_m": table[:, 0].tolist(),
        "cumulative_yaw_rad": table[:, 1].tolist(),
        "runs": per_run,
    }


def lift_command(args) -> dict:
    out = []
    for run in args.run:
        result = json.loads((run / "result.json").read_text())
        rows = {}
        for sample in result["samples"]:
            rows.setdefault(sample["phase"], []).append(
                (sample["lift_m"], sample["pallet_position_m"][2])
            )
        phases = {}
        for phase, values in rows.items():
            v = np.asarray(values, float)
            phases[phase] = {
                "lift_m": [float(v[:, 0].min()), float(np.median(v[:, 0])), float(v[:, 0].max())],
                "pallet_z_m": [float(v[:, 1].min()), float(np.median(v[:, 1])), float(v[:, 1].max())],
            }
        peak = result.get("lift_tilt_peak", {})
        out.append({"run": str(run), "phases": phases, "pallet_contact_lift_m": peak.get("lift_m")})
    return {"runs": out}


def max_visible_depth_m(standoff_m, *, camera_z_m, deck_bottom_m, voxel_z_m) -> float:
    """Deepest point at voxel_z_m a straight ray from camera_z_m reaches under the deck edge.

    The ray grazes the deck underside at the face (standoff_m ahead of the
    camera) and comes down to voxel_z_m; past that it would have to bend.
    """
    drop_to_deck = camera_z_m - deck_bottom_m
    drop_to_voxel = camera_z_m - voxel_z_m
    if drop_to_deck <= 0:
        return math.inf
    return standoff_m * (drop_to_voxel / drop_to_deck - 1.0)


def standoff_command(args) -> dict:
    deck = args.deck_bottom_m
    rows = []
    for standoff in np.arange(0.4, 3.01, 0.1):
        rows.append(
            {
                "standoff_m": round(float(standoff), 2),
                "visible_depth_m": max_visible_depth_m(
                    standoff, camera_z_m=args.camera_z_m, deck_bottom_m=deck,
                    voxel_z_m=args.voxel_top_m,
                ),
            }
        )
    need = args.depth_m + args.band_m
    ratio = (args.camera_z_m - args.voxel_top_m) / (args.camera_z_m - deck) - 1.0
    required = need / ratio if ratio > 0 else math.inf
    lower_edge = args.tilt_rad + math.radians(args.vfov_deg) / 2
    return {
        "camera_z_m": args.camera_z_m,
        "tilt_rad": args.tilt_rad,
        "deck_bottom_m": deck,
        "voxel_top_m": args.voxel_top_m,
        "depth_needed_m": need,
        "standoff_required_m": required,
        "lower_fov_edge_rad": lower_edge,
        "ray_angle_at_required_rad": math.atan2(
            args.camera_z_m - args.voxel_top_m, required + need
        ),
        "table": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    o = sub.add_parser("odometry")
    o.add_argument("--run", type=Path, action="append", required=True)
    o.add_argument("--draws", type=int, default=10)
    o.add_argument("--start-every", type=int, default=60)
    o.add_argument("--max-age-s", type=float, default=10.0)
    age = sub.add_parser("age")
    age.add_argument("--run", type=Path, action="append", required=True)
    age.add_argument("--draws", type=int, default=5)
    lift = sub.add_parser("lift")
    lift.add_argument("--run", type=Path, action="append", required=True)
    s = sub.add_parser("standoff")
    # carriage_low (sim/isaac/perception_adapter.py), at lift 0 where carriage == base.
    s.add_argument("--camera-z-m", type=float, default=0.27)
    s.add_argument("--tilt-rad", type=float, default=0.10)
    s.add_argument("--vfov-deg", type=float, default=58.0)
    # EPAL 6 openings run floor to the stringer underside (config/pallet_geometry_epal6.yaml).
    s.add_argument("--deck-bottom-m", type=float, default=0.100)
    # Fork blade top at lift 0 (fork_carriage blade box z 0.04 + 0.012) + 0.010 vertical margin.
    s.add_argument("--voxel-top-m", type=float, default=0.062)
    s.add_argument("--depth-m", type=float, default=0.36)
    s.add_argument("--band-m", type=float, default=0.10)
    s.add_argument("--output", type=Path)
    for p in (o, age, lift):
        p.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = {"odometry": odometry_command, "age": age_command, "lift": lift_command, "standoff": standoff_command}[
        args.command
    ](args)
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
