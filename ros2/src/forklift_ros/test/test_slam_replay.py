"""Record-to-rosbag arithmetic, without ROS: odometry frames, noise, stamps."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).parents[1] / "forklift_ros" / "slam_replay.py"
spec = importlib.util.spec_from_file_location("slam_replay_under_test", SCRIPT)
replay = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = replay
spec.loader.exec_module(replay)

REAR_X = -0.34
RADIUS = 0.135


def write_record(directory, *, steps=241, speed=0.5, yaw0=0.3, start=(4.0, -2.0)):
    """Straight drive along yaw0: 120 Hz joints, scans every 12th step."""
    t = np.arange(steps) / 120
    forward = np.array([math.cos(yaw0), math.sin(yaw0)])
    base_xy = np.asarray(start) + np.outer(speed * t, forward)
    quaternion = np.tile([math.cos(yaw0 / 2), 0, 0, math.sin(yaw0 / 2)], (steps, 1))
    base = np.column_stack((base_xy, np.full(steps, 0.03), quaternion))
    scan_index = np.arange(0, steps, 12)
    ranges = np.tile(
        np.array([5.0, np.inf, -np.inf, 11.9], np.float32), (len(scan_index), 1)
    )
    np.savez_compressed(
        directory / "slam_log.npz",
        joint_stamps_s=t,
        wheel_rates_rad_s=np.full((steps, 4), speed / RADIUS),
        steering_rad=np.zeros((steps, 2)),
        base_pose_world=base,
        scan_stamps_s=t[scan_index],
        scan_ranges_m=ranges,
        laser_pose_world=np.column_stack(
            (base_xy[scan_index], np.full(len(scan_index), yaw0))
        ),
    )
    meta = {
        "format": replay.LOG_FORMAT,
        "laser": {
            "beam_count": 4,
            "angle_min_rad": -math.pi,
            "angle_increment_rad": math.pi / 2,
            "range_min_m": 0.2,
            "range_max_m": 12.0,
            "rate_hz": 10,
            "mount_xyz_m": [-0.12, 0.0, 1.05],
            "mount_yaw_rad": 0.0,
        },
        "odometry_geometry": {
            "wheelbase_m": 0.64,
            "track_m": 0.51,
            "wheel_radius_m": RADIUS,
            "rear_axle_x_in_base_m": REAR_X,
        },
    }
    (directory / "meta.json").write_text(json.dumps(meta))
    return t, base_xy


def test_record_loads_and_rejects_another_format(tmp_path):
    write_record(tmp_path)
    log, meta = replay.load_slam_log(tmp_path)
    assert log["scan_ranges_m"].shape == (21, 4)
    meta["format"] = "something_else"
    (tmp_path / "meta.json").write_text(json.dumps(meta))
    with pytest.raises(ValueError, match="format"):
        replay.load_slam_log(tmp_path)


def test_odometry_starts_at_the_odom_origin_and_follows_the_wheels(tmp_path):
    t, _ = write_record(tmp_path)
    log, meta = replay.load_slam_log(tmp_path)
    poses = replay.odometry_base_poses(log, meta)
    # base_link starts at the odom origin facing +x, and 0.5 m/s straight
    # driving moves it along +x by 0.5 t, whatever the world heading was.
    np.testing.assert_allclose(poses[0], [0, 0, 0], atol=1e-12)
    np.testing.assert_allclose(poses[:, 0], 0.5 * t, atol=1e-12)
    np.testing.assert_allclose(poses[:, 1:], 0, atol=1e-12)


def test_ground_truth_is_the_world_base_pose(tmp_path):
    t, base_xy = write_record(tmp_path)
    log, _ = replay.load_slam_log(tmp_path)
    truth = replay.ground_truth_base_poses(log)
    np.testing.assert_allclose(truth[:, :2], base_xy)
    np.testing.assert_allclose(truth[:, 2], 0.3)


def test_noise_is_seeded_and_never_turns_a_miss_into_a_distance(tmp_path):
    write_record(tmp_path)
    log, meta = replay.load_slam_log(tmp_path)
    noise = replay.ReplayNoise(range_std_m=0.05, seed=7)
    first = replay.noisy_ranges(log["scan_ranges_m"], meta, noise)
    again = replay.noisy_ranges(log["scan_ranges_m"], meta, noise)
    np.testing.assert_array_equal(first, again)
    assert np.isposinf(first[:, 1]).all() and np.isneginf(first[:, 2]).all()
    assert not np.array_equal(first[:, 0], log["scan_ranges_m"][:, 0])
    assert first[:, 3].max() <= 12.0  # clipped to the configured maximum


def test_zero_noise_leaves_the_record_untouched(tmp_path):
    write_record(tmp_path)
    log, meta = replay.load_slam_log(tmp_path)
    quiet = replay.ReplayNoise()
    np.testing.assert_array_equal(
        replay.noisy_ranges(log["scan_ranges_m"], meta, quiet), log["scan_ranges_m"]
    )
    np.testing.assert_array_equal(
        replay.odometry_base_poses(log, meta, quiet),
        replay.odometry_base_poses(log, meta),
    )


def test_ros_stamps_stay_clear_of_time_zero():
    sec, nanosec = replay.ros_time(0.0)
    assert (sec, nanosec) == (int(replay.CLOCK_OFFSET_S), 0)
    assert replay.ros_time(1.25) == (int(replay.CLOCK_OFFSET_S) + 1, 250_000_000)
    assert (
        replay.ros_nanoseconds(1.25)
        == (int(replay.CLOCK_OFFSET_S) + 1) * 10**9 + 250_000_000
    )


@pytest.mark.parametrize(
    "kwargs", [dict(range_std_m=-0.1), dict(steering_std_rad=math.nan), dict(seed=-1)]
)
def test_invalid_noise_settings_raise(kwargs):
    with pytest.raises(ValueError):
        replay.ReplayNoise(**kwargs)


def test_occupancy_grid_becomes_a_trinary_pgm_with_row_zero_on_top():
    path = Path(__file__).parents[1] / "forklift_ros" / "slam_recorder.py"
    spec = importlib.util.spec_from_file_location("slam_recorder_under_test", path)
    recorder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recorder)
    # Two rows of three cells; ROS row 0 is the bottom of the map.
    data = [-1, 0, 100, 25, 50, 65]
    pgm = recorder.occupancy_to_pgm(data, 3, 2)
    header, pixels = pgm[:11], np.frombuffer(pgm[11:], dtype=np.uint8)
    assert header == b"P5\n3 2\n255\n"
    np.testing.assert_array_equal(pixels.reshape(2, 3), [[254, 205, 0], [205, 254, 0]])


def test_map_history_packs_each_update_with_its_origin():
    path = Path(__file__).parents[1] / "forklift_ros" / "slam_recorder.py"
    spec = importlib.util.spec_from_file_location("slam_recorder_history", path)
    recorder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recorder)
    entries = [
        (10_000_000_000, -1.0, -2.0, 0.0, 0.05, 3, 2, [-1, 0, 100, 0, 0, 0]),
        (11_000_000_000, -1.5, -2.0, 0.0, 0.05, 4, 1, [0, 0, 100, -1]),
    ]
    packed = recorder.pack_map_history(entries)
    np.testing.assert_array_equal(packed["stamps_ns"], [10**10, 11 * 10**9])
    np.testing.assert_allclose(packed["origins"], [[-1, -2, 0], [-1.5, -2, 0]])
    assert packed["map_0000"].shape == (2, 3)
    assert packed["map_0001"].tolist() == [[0, 0, 100, -1]]
