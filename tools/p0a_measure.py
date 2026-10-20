"""P0a measurements for the priority-5 plan (docs/plans/2026-10-04-lidar-obstacle-map.md).

    python tools/p0a_measure.py odometry --run <run dir> [--run ...] [--draws 10]
    python tools/p0a_measure.py age --run <run dir> [--run ...] [--draws 5]
        [--odometry-heading gyro --gyro-model config/gyro_model_spec_typical.yaml [--gyro-mount NAME]]
        [--regenerate-encoder team_spec_quantized]
    python tools/p0a_measure.py lift --run <run dir> [--run ...]
    python tools/p0a_measure.py standoff [--depth-m 0.36] [--band-m 0.10]

odometry: the relative wheel-odometry error after travelling d metres, at the
rear axle (the reference the plan's r = e(d) + 2 rho sin(psi(d)/2) uses). The
recorded joint rates are the true ones (noise is drawn online), so the online
noise model (OdometryNoise: 0.2 rad/s per wheel sample, 0.005 rad steering) is
drawn again here several times; truth is the recorded base_link pose moved to
the rear axle. Reports per-distance-bin maxima and the cheapest line
e0 + k d above all of them.

age --odometry-heading gyro (gyro-heading plan G2 (A), docs/plans/2026-10-21-gyro-g2-runtime-safety.md): the
table for runs whose SLAM odometry takes its heading from the simulated gyro. From a team_spec_quantized run, each
draw regenerates the counts as another sensor unit (as the steering table does) and draws another simulated gyro
(forklift_core.localization.gyro, seed 20000 + draw) on the recorded body attitude, the yaw rate from the logged
orientations' yaw differences (the log has no body angular velocity; plan V-G0's planar approximation); the heading
then runs the runner's own algorithm tick for tick (heading_step, the steering increments until the first standstill
bias) on the log's own per-tick command condition (drive_command_zero: a run recorded before it is refused). A run
whose truck moves before the first bias, or that never gets one, is refused (the runner does not drive before it), as
is a sample fault; the run is named.

age --regenerate-encoder team_spec_quantized (V-G2 bootstrap, 2026-10-21): a table for team-spec runs from runs that
read the reported rates. The team's encoders read the engine's continuous joint angles, which every run records
(wheel_angles_rad), so each draw counts the recorded rear wheel angles and true steering through the runner's own
TeamEncoderModel (that sensor's phases) and integrates them as a team-spec run would: the error team-spec odometry
would have shown on those trajectories. The table records the source model; a run already recorded with team-spec
encoders is refused (it has its own counts). team_spec_quantized has no INL, so its draws repeat the same counts (they
are not independent sensor samples); the gyro heading still redraws its gyro per draw.

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

from forklift_core.live.spool import load_slam_log
from forklift_core.localization.encoders import (
    TeamEncoderModel,
    encoder_model_spec,
    encoder_odometry_inputs,
    sensor_phases,
)
from forklift_core.localization.slam_pose import OdometryNoise
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
    integrate_wheel_odometry_increments,
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


def refuse_recorded(runs) -> None:
    """The error tables are measurement: no boundary-recorded run among their inputs (M-a0)."""
    from forklift_core.mission.boundary_record import measurement_refusal

    for run in runs:
        # The tables read meta.json and slam_log.npz, so the flag the runner writes into meta.json is checked;
        # a meta without it was written before the flag (possibly by a recording runner), so result.json's
        # arguments decide, and with neither the input cannot be verified and is refused.
        meta = json.loads((run / "meta.json").read_text())
        path = run / "result.json"
        result = json.loads(path.read_text()) if path.exists() else None
        if "record_boundary" not in meta and result is None:
            raise SystemExit(f"{run}: no record_boundary in meta.json and no result.json -- cannot verify the run "
                             "was not boundary-recorded")
        reason = measurement_refusal({"arguments": {"record_boundary": meta.get("record_boundary", False)}})
        if reason is None and result is not None:
            reason = measurement_refusal(result)
        if reason is not None:
            raise SystemExit(f"{run}: {reason}")


def encoder_model_of(runs) -> str:
    """The one encoder model every run was recorded with (meta.json; none recorded = reported): a
    table mixes no two models (2026-10-15)."""
    names = {(json.loads((run / "meta.json").read_text()).get("encoder_model") or {"name": "reported"})["name"]
             for run in runs}
    if len(names) != 1:
        raise SystemExit(f"runs recorded with different encoder models: {sorted(names)}")
    return names.pop()


def odometry_radius_of(runs) -> float:
    """The one wheel radius every run's odometry integrated with (meta.json odometry_geometry; a
    calibrated radius from --odometry-wheel-radius-m or the geometric one): a table mixes no two
    (2026-10-15)."""
    radii = {float(json.loads((run / "meta.json").read_text())["odometry_geometry"]["wheel_radius_m"]) for run in runs}
    if len(radii) != 1:
        raise SystemExit(f"runs recorded with different odometry wheel radii: {sorted(radii)}")
    return radii.pop()


def odometry_noise_record(model: str) -> dict:
    if model == "reported":
        return {"wheel_rate_std_rad_s": 0.2, "steering_std_rad": 0.005}
    return {"encoder_model": model, "gaussian": None,
            "draws": "each draw regenerates the counts as another sensor unit (INL phase); no Gaussian"}


def odometry_command(args) -> dict:
    refuse_recorded(args.run)
    model = encoder_model_of(args.run)
    radius = odometry_radius_of(args.run)
    per_run = []
    all_d, all_p, all_y = [], [], []
    for run in args.run:
        log = load_slam_log(run)  # a batch run's slam_log.npz or a live run's complete spool
        meta = json.loads((run / "meta.json").read_text())
        g = meta["odometry_geometry"]
        geometry = AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"])
        truth = rear_truth(log["base_pose_world"].astype(float), float(g["rear_axle_x_in_base_m"]))
        stamps = log["joint_stamps_s"]
        rates = log["wheel_rates_rad_s"].astype(float)[:, 2:4]
        steering = log["steering_rad"].astype(float)
        encoders = encoder_odometry_inputs(log, meta)["kind"]  # validates a team-spec record
        for draw in range(args.draws):
            if encoders == "increments":
                # The team's encoders: counts regenerated per draw as another sensor unit, no Gaussian.
                drawn = encoder_odometry_inputs(log, meta, phase_seed=10_000 + draw)
                odom = integrate_wheel_odometry_increments(
                    stamps, drawn["increments"], drawn["steering"], geometry, initial_pose=tuple(truth[0])
                )
            else:
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
        "noise": odometry_noise_record(model),
        "odometry_wheel_radius_m": radius,
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


def gyro_mount_rotation(name: str) -> np.ndarray:
    """base_link <- optical (the D435i IMU axes are the depth axes) for a named mount, as the runner builds it."""
    import sys

    root = str(Path(__file__).resolve().parents[1])
    if root not in sys.path:
        sys.path.insert(0, root)
    from sim.isaac import perception_adapter

    return np.asarray(perception_adapter.mount_base_from_optical(name).rotation, float)


def gyro_mount_of(runs, mount: str) -> str:
    """The mount every run with a result.json recorded must be the one asked for (the gyro sits in the camera)."""
    for run in runs:
        path = run / "result.json"
        if path.exists():
            recorded = json.loads(path.read_text()).get("arguments", {}).get("perception_mount")
            if recorded is not None and recorded != mount:
                raise SystemExit(f"{run}: recorded with --perception-mount {recorded}, the table is for {mount}")
    return mount


def gyro_rear_odometry(stamps, increments, steering, base_pose, command_zero, geometry, initial_pose, model, mount,
                       seed: int):
    """Rear-axle odometry (N, 3) on the gyro heading, through the runner's sampler and heading_step, tick for tick."""
    from forklift_core.localization import gyro as G
    from forklift_core.localization.slam_pose import IncrementalWheelOdometry

    qw, qx, qy, qz = np.asarray(base_pose, float)[:, 3:7].T
    yaw = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    rates = np.concatenate(([0.0], np.diff(yaw) / np.diff(stamps)))
    sampler = G.GyroSampler(model, mount, seed)
    state = G.new_heading_state()
    odometry = IncrementalWheelOdometry(geometry, initial_pose=tuple(initial_pose))
    out = np.empty((len(stamps), 3))
    for i in range(len(stamps)):
        g_stamp, w_imu, g_seq = sampler.sample(stamps[i], (0.0, 0.0, rates[i]), base_pose[i, 3:7])
        still = increments[i][0] == 0 and increments[i][1] == 0
        dyaw = G.heading_step(state, model, stamp_s=g_stamp, seq=g_seq, raw_rate=G.vertical_rate(sampler.known, w_imu),
                              wheels_still=still, command_zero=bool(command_zero[i]), dt=1 / 120, w_imu=w_imu)
        if state["bias"] is None and i > 0 and not still:
            raise RuntimeError(f"the truck moved at {stamps[i]:.3f} s before the first standstill bias (the runner holds it)")
        if state["bias"] is None and g_stamp > G.FIRST_BIAS_LIMIT_S:
            raise RuntimeError(f"no standstill bias by {G.FIRST_BIAS_LIMIT_S:.0f} s (the runner ends the session there)")
        if dyaw is not None:
            out[i] = odometry.update_increments_heading(stamps[i], increments[i], dyaw)
        else:
            out[i] = odometry.update_increments(stamps[i], increments[i], steering[i])
    if state["updates"] == 0:
        raise RuntimeError("no standstill bias update in the whole record")
    return out, state["updates"]


def regenerated_inputs(log, name: str, phase_seed: int) -> dict:
    """encoder_odometry_inputs' team-spec counts for a log recorded on the reported rates: the recorded continuous rear
    wheel angles and true steering read through TeamEncoderModel(name, sensor_phases(phase_seed))."""
    stamps = np.asarray(log["joint_stamps_s"], dtype=float)
    angles = np.asarray(log["wheel_angles_rad"], dtype=float)
    steering_true = np.asarray(log["steering_rad"], dtype=float)
    if angles.shape != (len(stamps), 4) or not np.isfinite(angles).all():
        raise ValueError("wheel_angles_rad must be finite and (N, 4)")
    if steering_true.shape != (len(stamps), 2) or not np.isfinite(steering_true).all():
        raise ValueError("steering_rad must be finite and (N, 2)")
    model = TeamEncoderModel(encoder_model_spec(name), sensor_phases(phase_seed))
    rows = [model.read(angles[k, 2:4], steering_true[k]) for k in range(len(stamps))]
    return {"kind": "increments", "increments": np.array([r[0] for r in rows], dtype=float),
            "steering": np.array([r[1] for r in rows], dtype=float)}


def age_command(args) -> dict:
    """Odometry error by observation age, from every recorded scan stamp (Codex P0a)."""
    refuse_recorded(args.run)
    model = encoder_model_of(args.run)
    regenerate = getattr(args, "regenerate_encoder", None)
    if regenerate is not None:
        if model != "reported":
            raise SystemExit(f"--regenerate-encoder reads runs recorded on the reported rates, these are {model}")
        if encoder_model_spec(regenerate) is None:
            raise SystemExit(f"--regenerate-encoder {regenerate}: not a team-spec encoder model")
        source_model, model = model, regenerate
    radius = odometry_radius_of(args.run)
    heading = getattr(args, "odometry_heading", "steering")
    gyro = None
    if heading == "gyro":
        from forklift_core.localization.gyro import GyroModel

        if model != "team_spec_quantized":
            raise SystemExit(f"--odometry-heading gyro reads team_spec_quantized runs, these are {model}")
        if args.gyro_model is None:
            raise SystemExit("--odometry-heading gyro requires --gyro-model")
        gyro_model, gyro_sha = GyroModel.load(args.gyro_model)
        mount_name = gyro_mount_of(args.run, args.gyro_mount)
        gyro = (gyro_model, gyro_mount_rotation(mount_name))
    elif getattr(args, "gyro_model", None) is not None:
        raise SystemExit("--gyro-model requires --odometry-heading gyro")
    maxima = np.zeros((len(AGES_S), 2))
    per_run = []
    for run in args.run:
        log = load_slam_log(run)  # a batch run's slam_log.npz or a live run's complete spool
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
        encoders = "increments" if regenerate is not None else encoder_odometry_inputs(log, meta)["kind"]

        def drawn_inputs(draw_):
            if regenerate is not None:
                return regenerated_inputs(log, regenerate, 20_000 + draw_)
            return encoder_odometry_inputs(log, meta, phase_seed=20_000 + draw_)

        bias_updates = []
        if gyro is not None and "drive_command_zero" not in log:
            raise SystemExit(f"{run}: no drive_command_zero in slam_log.npz (recorded before the gyro build): the bias "
                             "block's command condition cannot be replayed")
        for draw in range(args.draws):
            if gyro is not None:
                drawn = drawn_inputs(draw)
                try:
                    odom, updates = gyro_rear_odometry(stamps, drawn["increments"], drawn["steering"],
                                                       log["base_pose_world"].astype(float), log["drive_command_zero"],
                                                       geometry, truth[0], gyro[0], gyro[1], 20_000 + draw)
                except Exception as exc:  # a gyro fault (a log gap) or an odometry refusal: the run named
                    raise SystemExit(f"{run}: {exc}") from exc
                bias_updates.append(updates)
            elif encoders == "increments":
                drawn = drawn_inputs(draw)
                odom = integrate_wheel_odometry_increments(
                    stamps, drawn["increments"], drawn["steering"], geometry, initial_pose=tuple(truth[0])
                )
            else:
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
        per_run.append({"run": str(run), "scan_starts": int(len(starts)), "max": run_max.tolist(),
                        **({"gyro_bias_updates": bias_updates} if gyro is not None else {})})
    table = cumulative_table(maxima)
    noise = odometry_noise_record(model)
    if regenerate is not None:
        noise.update(regenerated_from=source_model,
                     draws="each draw counts the recorded wheel angles through another sensor unit's phases; "
                     f"{regenerate} has no INL, so the draws repeat the same counts")
    if gyro is not None:
        noise.update(odometry_heading="gyro", gyro_model_sha256=gyro_sha, gyro_mount=mount_name,
                     gyro_draws="each draw also draws another simulated gyro (seed 20000 + draw)")
    return {
        "source": ("wheel odometry (rear wheel rates + steering), OdometryNoise model redrawn" if model == "reported"
                   else "wheel odometry from the team-spec encoders (rear wheel counts) on the simulated gyro's heading"
                   if gyro is not None else "wheel odometry from the team-spec encoders (rear wheel counts + steering)")
        + (f", counted from the {source_model} runs' recorded wheel angles" if regenerate is not None else ""),
        "noise": noise,
        "odometry_wheel_radius_m": radius,
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
    age.add_argument("--odometry-heading", choices=("steering", "gyro"), default="steering")
    age.add_argument("--regenerate-encoder", choices=("team_spec_quantized", "team_spec_typical_inl"), default=None,
                     help="count a reported run's recorded wheel angles as this team-spec encoder (V-G2 bootstrap)")
    age.add_argument("--gyro-model", type=Path, default=None)
    age.add_argument("--gyro-mount", default="carriage_low_measured",
                     help="the runs' --perception-mount (the gyro is the camera's IMU)")
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
