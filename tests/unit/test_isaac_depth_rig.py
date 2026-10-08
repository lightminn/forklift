"""SDK-free parts of the Isaac RGB-D rig (plan 2026-10-07 D1), on CPU."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from forklift_core.sensors.camera_rig import rig_from_config

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "depth_rig_under_test", ROOT / "sim/isaac/depth_rig.py"
)
depth_rig = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = depth_rig
spec.loader.exec_module(depth_rig)

RIG = rig_from_config(
    yaml.safe_load((ROOT / "config/isaac_depth_rig.yaml").read_text())
)


def _yaw_q(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def test_quaternion_round_trips_every_mount():
    for camera in RIG.cameras:
        q = depth_rig.quaternion_wxyz(camera.rotation_base_from_optical)
        np.testing.assert_allclose(
            depth_rig._rotation_wxyz(q), camera.rotation_base_from_optical, atol=1e-12
        )


def test_world_rays_follow_the_truck_yaw():
    camera = RIG.camera("front")
    centre = np.array([[319.5, 239.5]])
    origin, rays = depth_rig.world_rays(
        camera, [1.0, 2.0, 0.0], _yaw_q(math.pi / 2), centre
    )
    # Truck faces +y: the front camera sits 0.17 m ahead (+y), looks +y and down 15 deg.
    np.testing.assert_allclose(origin, [1.0, 2.17, 1.03], atol=1e-12)
    np.testing.assert_allclose(
        rays[0],
        [0.0, math.cos(math.radians(15)), -math.sin(math.radians(15))],
        atol=1e-12,
    )


def test_floor_depth_from_a_cast_range_matches_the_axial_depth():
    camera = RIG.camera("left")
    uv = np.array([[100.0, 400.0], [319.5, 239.5]])
    origin, rays = depth_rig.world_rays(camera, [0, 0, 0], _yaw_q(0.0), uv)
    ranges = -origin[2] / rays[:, 2]  # flat floor at z = 0
    axial = depth_rig.axial_from_range(camera, uv, ranges)
    points_optical = (rays * ranges[:, None]) @ camera.rotation_base_from_optical
    np.testing.assert_allclose(axial, points_optical[:, 2], rtol=1e-12)


def _scene(lag_offset_m):
    """Cast depth for 40 pixels: 25 floor (same from both poses), 15 that move."""
    now = np.concatenate((np.full(25, 2.0), np.linspace(1.0, 3.0, 15)))
    earlier = now.copy()
    earlier[25:] += lag_offset_m
    return now, earlier


def test_a_fresh_frame_matches_the_current_pose():
    now, earlier = _scene(0.05)
    record = depth_rig.freshness_residuals(now + 1e-6, {0: now, 12: earlier}, far_m=8.0)
    assert record["judged"] and record["fresh"]
    assert record["lags_steps"]["12"]["moved"] == 15


def test_a_stale_frame_is_caught_even_when_the_floor_agrees_with_both():
    now, earlier = _scene(0.05)
    record = depth_rig.freshness_residuals(earlier, {0: now, 12: earlier}, far_m=8.0)
    # Most pixels (floor) match either pose: the overall median cannot tell.
    assert record["lag0_median_abs_m"] <= depth_rig.LAG0_LIMIT_M
    assert record["judged"] and not record["fresh"]


def test_standing_still_is_not_judged_but_still_bounded():
    now, _ = _scene(0.0)
    record = depth_rig.freshness_residuals(now, {0: now, 12: now}, far_m=8.0)
    assert not record["judged"] and record["fresh"]
    bad = depth_rig.freshness_residuals(now + 0.02, {0: now, 12: now}, far_m=8.0)
    assert not bad["fresh"]
    summary = depth_rig.summarise_freshness(
        [{"index": 0, "cameras": {"front": {**bad, "self_hits": 0}}}]
    )
    assert summary["failures"] == [{"index": 0, "camera": "front"}]


def test_clean_depth_is_millimetres_with_zero_outside_range():
    out = depth_rig.clean_depth_mm(
        np.array([[0.2, 0.28, 7.9996, 8.1, np.inf]]), 0.28, 8.0
    )
    np.testing.assert_array_equal(out, [[0, 280, 8000, 0, 0]])
    assert out.dtype == np.uint16


def test_frame_writer_round_trips(tmp_path):
    from PIL import Image

    writer = depth_rig.FrameWriter(tmp_path, ["front"])
    rgb = np.full((4, 6, 3), 128, np.uint8)
    depth = np.arange(24, dtype=np.uint16).reshape(4, 6) * 300
    writer.put("front", 7, rgb, depth)
    writer.close()
    np.testing.assert_array_equal(
        np.asarray(Image.open(tmp_path / "front/000007.png")), depth
    )
    assert (
        np.abs(
            np.asarray(Image.open(tmp_path / "front/000007.jpg")).astype(int) - 128
        ).max()
        <= 2
    )


def test_pose_history_returns_lagged_poses():
    history = depth_rig.PoseHistory(depth=3)
    for k in range(5):
        history.push([k, 0, 0], [1, 0, 0, 0])
    assert history.lagged(0)[0][0] == 4 and history.lagged(2)[0][0] == 2
    assert history.lagged(3) is None


def test_chase_view_sits_behind_and_above():
    eye, target = depth_rig.ChaseView().eye_and_target([0, 0, 0], 0.0)
    assert eye[0] < 0 < target[0] and eye[2] > target[2]
    with pytest.raises(KeyError):
        RIG.camera("top")
