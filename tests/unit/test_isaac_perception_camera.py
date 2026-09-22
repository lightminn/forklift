"""Camera acquisition contracts at the lazy Isaac SDK boundary."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from forklift_core.geometry import FramePoints


def load_camera_module():
    path = Path(__file__).resolve().parents[2] / "sim/isaac/perception_camera.py"
    spec = importlib.util.spec_from_file_location("isaac_perception_camera_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


camera_adapter = load_camera_module()


class RobotBoundary:
    def __init__(self):
        self.position = np.array([1.0, 2.0, 0.1])
        self.quaternion = np.array([2**-0.5, 0, 0, 2**-0.5])
        self.linear = np.zeros(3)
        self.angular = np.zeros(3)

    def get_world_pose(self):
        # SDK-owned buffers deliberately remain aliased across calls.
        return self.position, self.quaternion

    def get_linear_velocity(self):
        return self.linear

    def get_angular_velocity(self):
        return self.angular


class WorldBoundary:
    def __init__(self):
        self.current_time = 1.25
        self.render_count = 0
        self.on_render = lambda: None

    def render(self):
        self.render_count += 1
        self.on_render()


class CameraBoundary:
    def __init__(self):
        self.rgba = np.full((480, 640, 4), 73, np.uint8)
        self.depth = np.full((480, 640), 1.23456789, np.float32)
        self.on_depth = lambda: None

    def get_rgba(self):
        return self.rgba

    def get_depth(self):
        self.on_depth()
        return self.depth

    def get_world_pose(self, *, camera_axes):
        assert camera_axes == "ros"
        # Base yaw +90 deg combined with optical +x right, +y down, +z forward.
        return np.array([1.0, 2.75, 0.37]), np.array([2**-0.5, -(2**-0.5), 0, 0])

    def get_intrinsics_matrix(self):
        return np.array([[465.75, 0, 320], [0, 465.75, 240], [0, 0, 1]])


@pytest.mark.parametrize(
    "acquire", ["acquire_stationary_snapshot", "acquire_frozen_snapshot"]
)
def test_frozen_snapshot_pairs_optical_mount_with_yawed_base_acquisition(acquire):
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()
    if acquire == "acquire_frozen_snapshot":
        robot.linear = np.array([0.3, -0.1, 0])
        robot.angular = np.array([0, 0, 0.4])
    expected_depth = float(camera.depth[240, 320])
    scene, world_from_base = getattr(camera_adapter, acquire)(camera, world, robot)
    np.testing.assert_allclose(
        scene.base_from_optical.translation_m, [0.75, 0, 0.27], atol=1e-14
    )
    np.testing.assert_allclose(
        scene.base_from_optical.rotation,
        [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
        atol=1e-14,
    )
    optical = FramePoints("camera_optical_frame", [[0, 0, 2], [1, 0, 2], [0, 1, 2]])
    world_points = world_from_base.apply(scene.base_from_optical.apply(optical))
    np.testing.assert_allclose(
        world_points.xyz_m,
        [[1, 4.75, 0.37], [2, 4.75, 0.37], [1, 4.75, -0.63]],
        atol=1e-14,
    )
    assert scene.stamp_ns == 1_250_000_000
    assert scene.clock_domain == "synthetic" and scene.source_provenance == "synthetic"
    assert scene.depth_m[240, 320] == expected_depth
    assert scene.intrinsics.fx == 465.75
    camera.rgba.fill(0)
    camera.depth.fill(0)
    robot.position.fill(0)
    assert scene.rgb[240, 320, 0] == 73
    assert scene.depth_m[240, 320] == expected_depth
    np.testing.assert_array_equal(world_from_base.translation_m, [1, 2, 0.1])


@pytest.mark.parametrize(
    "field,value",
    [
        ("linear", [0.013, 0, 0]),
        ("angular", [0, 0, 0.013]),
        ("linear", [np.nan, 0, 0]),
        ("angular", [0, 0, np.nan]),
    ],
)
def test_snapshot_rejects_motion_or_unknown_velocity_before_render(field, value):
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()
    setattr(robot, field, np.array(value))
    with pytest.raises(RuntimeError):
        camera_adapter.acquire_stationary_snapshot(camera, world, robot)
    assert world.render_count == 0


def test_snapshot_rejects_physics_advancing_during_render():
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()
    world.on_render = lambda: setattr(world, "current_time", world.current_time + 0.01)
    with pytest.raises(RuntimeError, match="Physics advanced"):
        camera_adapter.acquire_stationary_snapshot(camera, world, robot)


def test_snapshot_rejects_pose_change_even_when_sdk_reuses_pose_buffer():
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()
    world.on_render = lambda: robot.position.__setitem__(0, robot.position[0] + 0.01)
    with pytest.raises(RuntimeError, match="pose changed"):
        camera_adapter.acquire_stationary_snapshot(camera, world, robot)


@pytest.mark.parametrize("change", ["time", "pose"])
def test_snapshot_rejects_changes_during_annotator_read(change):
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()
    if change == "time":
        camera.on_depth = lambda: setattr(
            world, "current_time", world.current_time + 0.01
        )
    else:
        camera.on_depth = lambda: robot.position.__setitem__(
            0, robot.position[0] + 0.01
        )
    with pytest.raises(RuntimeError):
        camera_adapter.acquire_stationary_snapshot(camera, world, robot)


@pytest.mark.parametrize(
    "buffer,shape",
    [
        ("rgba", (480, 640, 3)),
        ("rgba", (240, 320, 4)),
        ("depth", (480, 640, 1)),
    ],
)
def test_snapshot_rejects_unready_or_wrong_grid(buffer, shape):
    camera = CameraBoundary()
    setattr(
        camera, buffer, np.zeros(shape, np.uint8 if buffer == "rgba" else np.float32)
    )
    with pytest.raises(RuntimeError, match="annotators not ready"):
        camera_adapter.acquire_stationary_snapshot(
            camera, WorldBoundary(), RobotBoundary()
        )


def camera_settings():
    return {
        "mount_calibration": "synthetic_fixed_base_mount",
        "translation_m": [0.75, 0, 0.27],
        "quaternion_wxyz": [0.5, -0.5, 0.5, -0.5],
        "resolution": [640, 480],
        "focal_px": 465.741156,
        "aperture_cm": 2.0955,
        "clipping_range_m": [0.03, 100],
    }


def test_camera_settings_load_explicit_synthetic_calibration(tmp_path):
    path = tmp_path / "camera.yaml"
    settings = camera_settings()
    # Values come from the file rather than a hidden mount constant.
    settings["translation_m"] = [0.71, 0.02, 0.32]
    settings["focal_px"] = 470.25
    path.write_text(yaml.safe_dump(settings))
    loaded = camera_adapter.load_camera_settings(path)
    np.testing.assert_array_equal(loaded["translation_m"], [0.71, 0.02, 0.32])
    assert loaded["focal_px"] == 470.25
    assert loaded["mount_calibration"] == "synthetic_fixed_base_mount"


@pytest.mark.parametrize(
    "key,value",
    [
        ("mount_calibration", "measured"),
        ("translation_m", [0, 0]),
        ("translation_m", [0.75, np.nan, 0.27]),
        ("quaternion_wxyz", [0.5, 0.5, 0.5, 0.6]),
        ("quaternion_wxyz", [1, 0, 0]),
        ("resolution", [320, 240]),
        ("resolution", [640.0, 480]),
        ("focal_px", 0),
        ("focal_px", np.inf),
        ("aperture_cm", True),
        ("clipping_range_m", [1, 0.03]),
        ("clipping_range_m", [0, 100]),
        ("clipping_range_m", [0.03, np.inf]),
    ],
)
def test_camera_settings_reject_invalid_calibration(tmp_path, key, value):
    path = tmp_path / "camera.yaml"
    settings = camera_settings()
    settings[key] = value
    path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError):
        camera_adapter.load_camera_settings(path)


@pytest.mark.parametrize("invalid", ["missing", "extra", "not_mapping", "malformed"])
def test_camera_settings_require_exact_schema(tmp_path, invalid):
    path = tmp_path / "camera.yaml"
    settings = camera_settings()
    if invalid == "missing":
        del settings["focal_px"]
    elif invalid == "extra":
        settings["guessed_mount"] = True
    elif invalid == "not_mapping":
        settings = []
    path.write_text(
        "invalid: [" if invalid == "malformed" else yaml.safe_dump(settings)
    )
    with pytest.raises(ValueError):
        camera_adapter.load_camera_settings(path)


@pytest.mark.parametrize("field", ["linear", "angular"])
@pytest.mark.parametrize(
    "value", [[np.nan, 0, 0], [0, np.inf, 0], [0, 0], ["x", "y", "z"]]
)
def test_moving_snapshot_rejects_invalid_velocity_before_render(field, value):
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()
    setattr(robot, field, np.asarray(value))
    with pytest.raises(RuntimeError, match="velocity"):
        camera_adapter.acquire_frozen_snapshot(camera, world, robot)
    assert world.render_count == 0


@pytest.mark.parametrize("change", ["time", "position", "orientation", "velocity"])
@pytest.mark.parametrize(
    "boundary",
    ["render", "get_rgba", "get_depth", "get_world_pose", "get_intrinsics_matrix"],
)
def test_moving_snapshot_rejects_changes_throughout_capture(
    monkeypatch, change, boundary
):
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()
    robot.linear[0] = 0.3
    robot.angular[2] = 0.4

    def mutate():
        if change == "time":
            world.current_time += 0.01
        elif change == "position":
            robot.position[0] += 0.01
        elif change == "orientation":
            robot.quaternion[:] = [1, 0, 0, 0]
        else:
            robot.angular[0] = np.nan

    if boundary == "render":
        world.on_render = mutate
    else:
        original = getattr(camera, boundary)

        def read(*args, **kwargs):
            value = original(*args, **kwargs)
            mutate()
            return value

        monkeypatch.setattr(camera, boundary, read)
    with pytest.raises(RuntimeError):
        camera_adapter.acquire_frozen_snapshot(camera, world, robot)


def test_moving_snapshot_catches_pose_change_before_later_render_restores_it():
    robot, world, camera = RobotBoundary(), WorldBoundary(), CameraBoundary()

    def render():
        robot.position[0] = 1.1 if world.render_count == 1 else 1.0

    world.on_render = render
    with pytest.raises(RuntimeError, match="pose changed"):
        camera_adapter.acquire_frozen_snapshot(camera, world, robot)


def test_camera_creation_preserves_distinct_prim_paths(tmp_path, monkeypatch):
    # Camera construction needs Isaac's external SDK; retain its configured state.
    class Camera:
        def __init__(self, **kwargs):
            self.prim_path = kwargs["prim_path"]

        def set_local_pose(self, **kwargs):
            pass

        def set_horizontal_aperture(self, value):
            pass

        def set_vertical_aperture(self, value):
            pass

        def set_focal_length(self, value):
            pass

        def set_clipping_range(self, near, far):
            pass

    monkeypatch.setitem(
        sys.modules, "isaacsim.sensors.camera", SimpleNamespace(Camera=Camera)
    )
    path = tmp_path / "camera.yaml"
    path.write_text(yaml.safe_dump(camera_settings()))
    acquisition = camera_adapter.create_perception_camera(path)
    near = camera_adapter.create_perception_camera(
        path, prim_path="/World/Forklift/base_link/NearCamera"
    )
    assert acquisition.prim_path == "/World/Forklift/base_link/PerceptionCamera"
    assert near.prim_path == "/World/Forklift/base_link/NearCamera"
