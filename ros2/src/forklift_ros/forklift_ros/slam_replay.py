"""Turn an Isaac SLAM record (sim/isaac/run_slam_drive.py) into a rosbag2.

Topics, all stamped in simulation time plus CLOCK_OFFSET_S:
  /scan                    sensor_msgs/LaserScan, frame laser, REP-117 ranges
  /tf                      odom -> base_link from wheel odometry, and the
                           fixed base_link -> laser mount, at every joint sample
  /odom                    nav_msgs/Odometry, the same odometry
  /ground_truth/base_pose  geometry_msgs/PoseStamped in frame world, for
                           evaluation only; a mapper must not subscribe to it
Bag receive times equal the header stamps, so `ros2 bag play --clock` drives
/clock with simulation time. Odometry is integrated here, from the recorded
rear wheel rates and front steering angles, and written to odometry.csv as
well. Optional noise is Gaussian and seeded; the Isaac record stays noise-free.
The arithmetic below imports no ROS; write_bag does.

It needs forklift_core as well as rclpy, so run it with the development
container's venv interpreter (colcon's console scripts use /usr/bin/python3):

    python3 -m forklift_ros.slam_replay --record <run> --output <new dir>
"""

import argparse
import csv
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    integrate_wheel_odometry,
)

LOG_FORMAT = "forklift_slam_log_v1"
# tf2 reads stamp 0 as "latest available"; keep every stamp well clear of it.
CLOCK_OFFSET_S = 10.0
_ARRAYS = (
    "joint_stamps_s",
    "wheel_rates_rad_s",
    "steering_rad",
    "base_pose_world",
    "scan_stamps_s",
    "scan_ranges_m",
    "laser_pose_world",
)


@dataclass(frozen=True)
class ReplayNoise:
    """Standard deviations added at replay time; zero means none."""

    range_std_m: float = 0.0
    wheel_rate_std_rad_s: float = 0.0
    steering_std_rad: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        for name in ("range_std_m", "wheel_rate_std_rad_s", "steering_std_rad"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise ValueError("seed must be an integer")
        if self.seed < 0:
            raise ValueError("seed must be nonnegative")


def load_slam_log(directory: Path) -> tuple[dict[str, np.ndarray], dict]:
    """Read slam_log.npz and meta.json, refusing another format or bad shapes."""
    directory = Path(directory)
    meta = json.loads((directory / "meta.json").read_text(encoding="utf-8"))
    if meta.get("format") != LOG_FORMAT:
        raise ValueError(f"unsupported record format: {meta.get('format')!r}")
    with np.load(directory / "slam_log.npz") as data:
        log = {name: np.asarray(data[name]) for name in _ARRAYS}
    joints = len(log["joint_stamps_s"])
    scans = len(log["scan_stamps_s"])
    expected = {
        "wheel_rates_rad_s": (joints, 4),
        "steering_rad": (joints, 2),
        "base_pose_world": (joints, 7),
        "scan_ranges_m": (scans, meta["laser"]["beam_count"]),
        "laser_pose_world": (scans, 3),
    }
    for name, shape in expected.items():
        if log[name].shape != shape:
            raise ValueError(f"{name} has shape {log[name].shape}, expected {shape}")
    if not np.isin(log["scan_stamps_s"], log["joint_stamps_s"]).all():
        raise ValueError("every scan stamp must be a joint sample stamp")
    return log, meta


def _rng(noise: ReplayNoise, stream: int) -> np.random.Generator:
    return np.random.default_rng([noise.seed, stream])


def odometry_base_poses(
    log: dict[str, np.ndarray], meta: dict, noise: ReplayNoise | None = None
) -> np.ndarray:
    """base_link x, y, yaw in odom at every joint sample; odom = start pose."""
    noise = noise if noise is not None else ReplayNoise()
    geometry_record = meta["odometry_geometry"]
    geometry = AckermannOdometryGeometry(
        geometry_record["wheelbase_m"],
        geometry_record["track_m"],
        geometry_record["wheel_radius_m"],
    )
    rear_x = geometry_record["rear_axle_x_in_base_m"]
    rates = log["wheel_rates_rad_s"][:, 2:4].astype(float)
    steering = log["steering_rad"].astype(float)
    if noise.wheel_rate_std_rad_s:
        rates = rates + _rng(noise, 1).normal(
            0, noise.wheel_rate_std_rad_s, rates.shape
        )
    if noise.steering_std_rad:
        steering = steering + _rng(noise, 2).normal(
            0, noise.steering_std_rad, steering.shape
        )
    rear = integrate_wheel_odometry(
        log["joint_stamps_s"], rates, steering, geometry, initial_pose=(rear_x, 0, 0)
    )
    base = rear.copy()
    base[:, 0] -= rear_x * np.cos(rear[:, 2])
    base[:, 1] -= rear_x * np.sin(rear[:, 2])
    return base


def ground_truth_base_poses(log: dict[str, np.ndarray]) -> np.ndarray:
    """World base_link x, y, yaw at every joint sample (from x,y,z,qw,qx,qy,qz)."""
    pose = log["base_pose_world"].astype(float)  # Isaac hands back float32
    w, x, y, z = pose[:, 3], pose[:, 4], pose[:, 5], pose[:, 6]
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.column_stack((pose[:, 0], pose[:, 1], yaw))


def noisy_ranges(ranges: np.ndarray, meta: dict, noise: ReplayNoise) -> np.ndarray:
    """Add range noise to measured beams only, kept inside [min, max]."""
    if not noise.range_std_m:
        return ranges
    out = ranges.astype(float).copy()
    measured = np.isfinite(out)
    out[measured] += _rng(noise, 0).normal(0, noise.range_std_m, measured.sum())
    laser = meta["laser"]
    out[measured] = np.clip(out[measured], laser["range_min_m"], laser["range_max_m"])
    return out.astype(ranges.dtype)


def ros_nanoseconds(stamp_s: float) -> int:
    return int(round((stamp_s + CLOCK_OFFSET_S) * 1e9))


def ros_time(stamp_s: float) -> tuple[int, int]:
    return divmod(ros_nanoseconds(stamp_s), 10**9)


def _quaternion_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(float(yaw) / 2), math.cos(float(yaw) / 2)


def write_bag(
    record: Path, bag: Path, noise: ReplayNoise, storage_id: str = "mcap"
) -> dict:
    """Write the bag and odometry.csv next to it; return a small manifest."""
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    log, meta = load_slam_log(record)
    laser = meta["laser"]
    odometry = odometry_base_poses(log, meta, noise)
    truth = ground_truth_base_poses(log)
    ranges = noisy_ranges(log["scan_ranges_m"], meta, noise)
    stamps = log["joint_stamps_s"]
    scan_rows = {float(t): i for i, t in enumerate(log["scan_stamps_s"])}

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {
        "/scan": "sensor_msgs/msg/LaserScan",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/odom": "nav_msgs/msg/Odometry",
        "/ground_truth/base_pose": "geometry_msgs/msg/PoseStamped",
    }
    for index, (name, kind) in enumerate(topics.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(index, name, kind, "cdr"))

    def header(message, stamp_s, frame):
        message.header.stamp.sec, message.header.stamp.nanosec = ros_time(stamp_s)
        message.header.frame_id = frame

    mount = laser["mount_xyz_m"]
    counts = dict.fromkeys(topics, 0)
    for k, stamp in enumerate(stamps):
        when = ros_nanoseconds(float(stamp))
        odom_tf, laser_tf = TransformStamped(), TransformStamped()
        header(odom_tf, stamp, "odom")
        odom_tf.child_frame_id = "base_link"
        # Message fields take Python floats only; numpy scalars can crash the
        # C conversion when field checks are off.
        x, y, yaw = map(float, odometry[k])
        odom_tf.transform.translation.x, odom_tf.transform.translation.y = x, y
        (
            odom_tf.transform.rotation.x,
            odom_tf.transform.rotation.y,
            odom_tf.transform.rotation.z,
            odom_tf.transform.rotation.w,
        ) = _quaternion_xyzw(yaw)
        header(laser_tf, stamp, "base_link")
        laser_tf.child_frame_id = "laser"
        (
            laser_tf.transform.translation.x,
            laser_tf.transform.translation.y,
            laser_tf.transform.translation.z,
        ) = map(float, mount)
        (
            laser_tf.transform.rotation.x,
            laser_tf.transform.rotation.y,
            laser_tf.transform.rotation.z,
            laser_tf.transform.rotation.w,
        ) = _quaternion_xyzw(laser["mount_yaw_rad"])
        writer.write(
            "/tf", serialize_message(TFMessage(transforms=[odom_tf, laser_tf])), when
        )
        counts["/tf"] += 1

        odom = Odometry()
        header(odom, stamp, "odom")
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ) = _quaternion_xyzw(yaw)
        writer.write("/odom", serialize_message(odom), when)
        counts["/odom"] += 1

        truth_pose = PoseStamped()
        header(truth_pose, stamp, "world")
        tx, ty, tyaw = map(float, truth[k])
        truth_pose.pose.position.x, truth_pose.pose.position.y = tx, ty
        (
            truth_pose.pose.orientation.x,
            truth_pose.pose.orientation.y,
            truth_pose.pose.orientation.z,
            truth_pose.pose.orientation.w,
        ) = _quaternion_xyzw(tyaw)
        writer.write("/ground_truth/base_pose", serialize_message(truth_pose), when)
        counts["/ground_truth/base_pose"] += 1

        row = scan_rows.get(float(stamp))
        if row is not None:
            scan = LaserScan()
            header(scan, stamp, "laser")
            scan.angle_min = float(laser["angle_min_rad"])
            scan.angle_increment = float(laser["angle_increment_rad"])
            scan.angle_max = float(
                laser["angle_min_rad"]
                + (laser["beam_count"] - 1) * laser["angle_increment_rad"]
            )
            scan.time_increment = 0.0  # every beam cast at one instant
            scan.scan_time = 1.0 / laser["rate_hz"]
            scan.range_min = float(laser["range_min_m"])
            scan.range_max = float(laser["range_max_m"])
            scan.ranges = ranges[row].astype(np.float32).tolist()
            writer.write("/scan", serialize_message(scan), when)
            counts["/scan"] += 1
    del writer

    with (Path(bag).parent / "odometry.csv").open("w", newline="") as stream:
        table = csv.writer(stream)
        table.writerow(["stamp_s", "x_m", "y_m", "yaw_rad"])
        for stamp, (x, y, yaw) in zip(stamps, odometry, strict=True):
            table.writerow([f"{stamp:.9f}", f"{x:.9f}", f"{y:.9f}", f"{yaw:.9f}"])
    return {
        "record": str(record),
        "record_sha256": {
            name: hashlib.sha256((Path(record) / name).read_bytes()).hexdigest()
            for name in ("slam_log.npz", "meta.json")
        },
        "bag": str(bag),
        "storage_id": storage_id,
        "clock_offset_s": CLOCK_OFFSET_S,
        "noise": asdict(noise),
        "messages": counts,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="new directory")
    parser.add_argument("--storage", default="mcap", choices=("mcap", "sqlite3"))
    parser.add_argument("--range-noise-std-m", type=float, default=0.0)
    parser.add_argument("--wheel-rate-noise-std-rad-s", type=float, default=0.0)
    parser.add_argument("--steering-noise-std-rad", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    noise = ReplayNoise(
        args.range_noise_std_m,
        args.wheel_rate_noise_std_rad_s,
        args.steering_noise_std_rad,
        args.seed,
    )
    manifest = write_bag(args.record, args.output / "bag", noise, args.storage)
    (args.output / "replay_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest["messages"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
