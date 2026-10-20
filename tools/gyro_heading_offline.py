"""Gyro-heading odometry, offline (gyro-heading plan docs/plans/2026-10-21-gyro-heading-odometry.md V-G0).

From a run's SLAM log, the odometry two arms would have sent the SLAM bridge, both on the same team-spec encoder counts:

- ``steering``: the rear wheels' count increments and the measured steering through ``IncrementalWheelOdometry
  .update_increments`` -- the runner's team-encoder path, function for function;
- ``gyro``: the same increments for distance, the heading from a modelled D435i gyro (plan G1) corrected by the
  standstill bias update (plan G2's offline part).

The gyro model: the vehicle's angular velocity (0, 0, yaw rate) -- the yaw rate from the logged quaternions' yaw
differences, the plan's stated approximation (the log has no body angular velocity) -- rotated into the IMU axes by the
*true* mount (the camera's extrinsic rotation, then the injected mount error about the vehicle's y axis); on each of the
three axes (1 + scale) x rate + initial bias + temperature ramp + bias random walk + white noise, quantised. The
algorithm knows only the extrinsic rotation: it rotates the three measured axes back to the vehicle and takes the
vertical component.

Standstill bias update (plan G2): consecutive physics ticks with neither rear wheel's count changing make a block; every
full block of ``zupt_s`` whose raw vertical rate's variance is at most ``k`` x the white-plus-quantisation variance
replaces the bias with the block's mean (non-overlapping blocks). The heading integrates (raw - bias) and stays at the
start until the first block -- a truck that moves before it fails the arm. The offline approximations the plan states:
no command condition, no monitoring, hold or fault feedback.

    python tools/gyro_heading_offline.py RUN_DIR OUT.npz --arm {steering,gyro} --bundle NAME --seed S
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.live.spool import load_list, load_slam_log  # noqa: E402
from forklift_core.localization.encoders import TeamEncoderModel, encoder_model_spec  # noqa: E402
from forklift_core.localization.slam_pose import IncrementalWheelOdometry  # noqa: E402
from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry  # noqa: E402

ENCODER_MODEL = "team_spec_quantized"
CAMERA = Path(__file__).resolve().parents[1] / "config" / "isaac_perception_camera.yaml"
REAR_AXLE_OFFSET_M = 0.34


@dataclass(frozen=True)
class GyroModel:
    """Plan V-G0's error bundle, degrees and seconds as the datasheet states them."""

    scale: float = 0.0  # (1 + scale) on every axis
    initial_bias_dps: float = 1.0  # each axis' b0 drawn from U(-x, x)
    temperature_dps: float = 0.15  # a linear ramp reaching this over temperature_s, sign drawn per axis
    temperature_s: float = 1000.0
    random_walk_dps_rts: float = 0.0005  # bias random walk, deg/s/sqrt(s)
    noise_dps_rthz: float = 0.014
    lsb_dps: float = 1.0 / 32.8  # +-1000 deg/s range
    mount_error_deg: float = 1.0  # about the vehicle's y axis, unknown to the algorithm
    zupt_s: float = 0.5
    zupt_k: float = 4.0


BUNDLES = {
    "A+": GyroModel(scale=0.01),
    "A-": GyroModel(scale=-0.01),
    "B1+": GyroModel(scale=0.03),
    "B1-": GyroModel(scale=-0.03),
    "B2": GyroModel(scale=0.01, temperature_dps=0.3),
    "B3": GyroModel(scale=0.01, zupt_s=0.3),
}


def quat_matrix(wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def camera_rotation() -> np.ndarray:
    """vehicle <- IMU: the camera's optical frame (D435i IMU axes are the depth axes: x right, y down, z forward)."""
    return quat_matrix(yaml.safe_load(CAMERA.read_text())["quaternion_wxyz"])


def yaw_rates(stamps: np.ndarray, quaternions_wxyz: np.ndarray) -> np.ndarray:
    """Interval yaw rates (N - 1) from the logged base orientations (planar approximation)."""
    qw, qx, qy, qz = np.asarray(quaternions_wxyz, float).T
    yaw = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    return np.diff(yaw) / np.diff(stamps)


def gyro_samples(stamps: np.ndarray, rates: np.ndarray, model: GyroModel, rng: np.random.Generator,
                 mount_true: np.ndarray) -> np.ndarray:
    """(N - 1, 3) measured IMU-axis rates, rad/s, one per interval."""
    dt = np.diff(stamps)
    t_mid = (stamps[:-1] + stamps[1:]) / 2 - stamps[0]
    true_imu = (np.column_stack((np.zeros_like(rates), np.zeros_like(rates), rates))) @ mount_true  # (R^T w)^T
    d2r = math.pi / 180
    b0 = rng.uniform(-model.initial_bias_dps, model.initial_bias_dps, 3) * d2r
    signs = rng.choice((-1.0, 1.0), 3)
    temp = np.outer(np.clip(t_mid / model.temperature_s, 0, None), signs * model.temperature_dps * d2r)
    walk = np.cumsum(rng.normal(0, model.random_walk_dps_rts * d2r, (len(dt), 3)) * np.sqrt(dt)[:, None], axis=0)
    white = rng.normal(0, 1, (len(dt), 3)) * (model.noise_dps_rthz * d2r / np.sqrt(dt))[:, None]
    measured = (1 + model.scale) * true_imu + b0 + temp + walk + white
    lsb = model.lsb_dps * d2r
    return np.round(measured / lsb) * lsb


def encoder_reads(wheel_angles: np.ndarray, steering: np.ndarray):
    """Per tick (increments (N, 2), measured steering (N, 2)) through the team-spec model, fixed sensor."""
    model = TeamEncoderModel(encoder_model_spec(ENCODER_MODEL), (0.0, 0.0, 0.0, 0.0))
    reads = [model.read(wheel_angles[i], steering[i]) for i in range(len(wheel_angles))]
    return np.array([r[0] for r in reads], float), np.array([r[1] for r in reads], float)


def standstill_bias(vertical: np.ndarray, dt: np.ndarray, still: np.ndarray, model: GyroModel):
    """Per interval the bias in force (NaN before the first update) and the update count. ``still[i]``: interval i had
    no count change on either rear wheel."""
    block = max(1, int(round(model.zupt_s / float(np.median(dt)))))
    d2r = math.pi / 180
    white_var = (model.noise_dps_rthz * d2r) ** 2 / float(np.median(dt)) + (model.lsb_dps * d2r) ** 2 / 12
    bias = np.full(len(vertical), np.nan)
    current, run, updates = math.nan, 0, 0
    for i in range(len(vertical)):
        bias[i] = current  # the estimate in force over this interval (updated after it)
        run = run + 1 if still[i] else 0
        if run and run % block == 0:
            window = vertical[i - block + 1:i + 1]
            if np.var(window) <= model.zupt_k * white_var:
                current = float(np.mean(window))
                updates += 1
    return bias, updates


def gyro_odometry(stamps, increments, vertical, still, model: GyroModel, start_rear, wheel_radius_m):
    """Rear-axle poses (N, 3): distance from the increments, heading from (vertical - bias)."""
    dt = np.diff(stamps)
    bias, updates = standstill_bias(vertical, dt, still, model)
    pose = np.empty((len(stamps), 3))
    x, y, yaw = (float(v) for v in start_rear)
    pose[0] = (x, y, yaw)
    for i in range(len(dt)):
        distance = float(np.mean(increments[i + 1])) * wheel_radius_m
        if math.isnan(bias[i]):
            if distance != 0.0:
                raise RuntimeError(f"the truck moved at {stamps[i + 1]:.3f} s before the first standstill bias update")
            dyaw = 0.0
        else:
            dyaw = (vertical[i] - bias[i]) * dt[i]
        mid = yaw + dyaw / 2
        x, y, yaw = x + distance * math.cos(mid), y + distance * math.sin(mid), yaw + dyaw
        pose[i + 1] = (x, y, yaw)
    return pose, updates


def run_inputs(run: Path) -> dict:
    log = load_slam_log(run)
    records = load_list(run / "spool", "slam.records")
    result = json.loads((run / "result.json").read_text())
    g = result["arguments"]["drive_geometry"]
    return {"t": np.asarray(log["joint_stamps_s"], float),
            "wheel_angles": np.asarray(log["wheel_angles_rad"], float)[:, 2:4],
            "steering": np.asarray(log["steering_rad"], float),
            "base": np.asarray(log["base_pose_world"], float),
            "records": records,
            "geometry": AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"])}


def build(run: Path, arm: str, bundle: str, seed: int) -> dict:
    inputs = run_inputs(run)
    t, records, geometry = inputs["t"], inputs["records"], inputs["geometry"]
    start_base = records[0]["odom_base"]
    start_rear = (start_base[0] - REAR_AXLE_OFFSET_M * math.cos(start_base[2]),
                  start_base[1] - REAR_AXLE_OFFSET_M * math.sin(start_base[2]), start_base[2])
    increments, measured_steering = encoder_reads(inputs["wheel_angles"], inputs["steering"])
    info = {"arm": arm, "bundle": bundle, "seed": seed, "encoder_model": ENCODER_MODEL}
    if arm == "steering":
        odometry = IncrementalWheelOdometry(geometry, initial_pose=start_rear)
        rear = np.array([odometry.update_increments(t[i], increments[i], measured_steering[i]) for i in range(len(t))])
    else:
        model = BUNDLES[bundle]
        rng = np.random.default_rng([seed, 11])
        known = camera_rotation()
        true = rot_y(math.radians(model.mount_error_deg)) @ known
        samples = gyro_samples(t, yaw_rates(t, inputs["base"][:, 3:7]), model, rng, true)
        vertical = (samples @ known.T)[:, 2]
        still = np.all(increments[1:] == 0, axis=1)
        rear, updates = gyro_odometry(t, increments, vertical, still, model, start_rear, geometry.wheel_radius_m)
        info.update(model=asdict(model), bias_updates=updates)
    stamps = np.array([r["stamp_s"] for r in records])
    j = np.searchsorted(t, stamps - 1e-6)
    if not np.allclose(t[j], stamps, atol=1e-6):
        raise ValueError("scan stamps are not joint stamps")
    p = rear[j]
    base = np.column_stack((p[:, 0] + REAR_AXLE_OFFSET_M * np.cos(p[:, 2]), p[:, 1] + REAR_AXLE_OFFSET_M * np.sin(p[:, 2]),
                            p[:, 2]))
    truth = np.array([r["truth_base"] for r in records])
    info["odom_error_m"] = {"median": float(np.median(np.hypot(*(base[:, :2] - truth[:, :2]).T))),
                            "max": float(np.max(np.hypot(*(base[:, :2] - truth[:, :2]).T)))}
    return {"odom_base": base, "scan_id": np.array([r["scan_id"] for r in records]), "stamp_s": stamps,
            "run": str(run.resolve()), "spool_manifest_sha256": hashlib.sha256((run / "spool" / "manifest.json")
                                                                               .read_bytes()).hexdigest(),
            "info": info}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--arm", choices=("steering", "gyro"), required=True)
    ap.add_argument("--bundle", choices=tuple(BUNDLES), required=True)
    ap.add_argument("--seed", type=int, required=True)
    a = ap.parse_args(argv)
    built = build(a.run, a.arm, a.bundle, a.seed)
    np.savez(a.out, odom_base=built["odom_base"], scan_id=built["scan_id"], stamp_s=built["stamp_s"],
             run=np.asarray(built["run"]), spool_manifest_sha256=np.asarray(built["spool_manifest_sha256"]),
             info=np.asarray(json.dumps(built["info"])))
    print(json.dumps(built["info"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
