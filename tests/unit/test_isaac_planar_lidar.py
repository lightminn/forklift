"""Ray geometry and overlay of the Isaac planar LiDAR, on CPU (no PhysX)."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "sim/isaac/planar_lidar.py"
spec = importlib.util.spec_from_file_location("planar_lidar_under_test", SCRIPT)
lidar = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = lidar
spec.loader.exec_module(lidar)

MOUNT = lidar.LaserMount(xyz_m=(-0.12, 0.0, 1.05), yaw_rad=0.0)
LEVEL = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz


def yaw_quaternion(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def test_level_base_at_origin_puts_rays_at_the_mount_in_the_scan_plane():
    angles = np.array([0.0, math.pi / 2, -math.pi])
    origin, directions = lidar.laser_rays_world(np.zeros(3), LEVEL, MOUNT, angles)
    np.testing.assert_allclose(origin, [-0.12, 0.0, 1.05])
    np.testing.assert_allclose(
        directions, [[1, 0, 0], [0, 1, 0], [-1, 0, 0]], atol=1e-15
    )


def test_base_yaw_rotates_both_the_mount_offset_and_the_beams():
    origin, directions = lidar.laser_rays_world(
        np.array([2.0, 3.0, 0.03]), yaw_quaternion(math.pi / 2), MOUNT, np.array([0.0])
    )
    np.testing.assert_allclose(origin, [2.0, 3.0 - 0.12, 1.08], atol=1e-15)
    np.testing.assert_allclose(directions, [[0, 1, 0]], atol=1e-15)


def test_mount_yaw_adds_to_base_yaw_in_the_laser_pose():
    mount = lidar.LaserMount(xyz_m=(0.5, 0.0, 1.0), yaw_rad=0.3)
    x, y, yaw = lidar.laser_pose_2d(
        np.array([1.0, 1.0, 0.0]), yaw_quaternion(0.2), mount
    )
    assert yaw == pytest.approx(0.5)
    assert (x, y) == pytest.approx((1 + 0.5 * math.cos(0.2), 1 + 0.5 * math.sin(0.2)))


def test_a_tilted_base_tilts_the_beams_but_keeps_them_unit_length():
    pitch = 0.05
    quaternion = np.array([math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0])
    _, directions = lidar.laser_rays_world(
        np.zeros(3), quaternion, MOUNT, np.linspace(-math.pi, math.pi, 7)
    )
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1.0)
    assert directions[3, 2] == pytest.approx(-math.sin(pitch))


def test_scan_points_keep_only_measured_ranges():
    origin = np.array([0.0, 0.0, 1.0])
    directions = np.array([[1.0, 0, 0], [0, 1.0, 0], [-1.0, 0, 0]])
    points = lidar.scan_points_world(
        origin, directions, np.array([2.0, np.inf, -np.inf])
    )
    np.testing.assert_allclose(points, [[2.0, 0.0, 1.0]])


def test_overlay_draws_inside_the_frame_and_ignores_the_rest():
    frame = np.zeros((10, 20, 3), dtype=np.uint8)
    out = lidar.draw_points(
        frame,
        np.array([[5.0, 4.0], [-3.0, 2.0], [25.0, 1.0], [np.nan, 1.0]]),
        (255, 0, 0),
    )
    assert out is not frame
    assert out[4, 5].tolist() == [255, 0, 0]
    assert out.sum() == 255
    assert frame.sum() == 0
