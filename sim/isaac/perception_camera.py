"""Isaac RGB-D acquisition while physics is frozen, without pallet labels."""

from dataclasses import asdict
from math import isfinite
from numbers import Real
from pathlib import Path
from typing import Any

import numpy as np

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.rgbd_snapshot import scene_input_from_rgbd_snapshot
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import PinholeIntrinsics


def load_camera_settings(settings_path: Path) -> dict:
    """Read the explicit synthetic pinhole/mount contract without loading Isaac."""
    import yaml

    try:
        settings = yaml.safe_load(Path(settings_path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError("Malformed perception camera YAML") from exc
    fields = {
        "mount_calibration",
        "translation_m",
        "quaternion_wxyz",
        "resolution",
        "focal_px",
        "aperture_cm",
        "clipping_range_m",
    }
    if not isinstance(settings, dict) or set(settings) != fields:
        raise ValueError("Camera settings must contain exactly the documented fields")
    if settings["mount_calibration"] != "synthetic_fixed_base_mount":
        raise ValueError("Camera mount must be explicitly synthetic_fixed_base_mount")
    RigidTransform(
        "camera_optical_frame",
        "base_link",
        rotation_matrix_from_quaternion_xyzw(np.roll(settings["quaternion_wxyz"], -1)),
        settings["translation_m"],
    )
    resolution = np.asarray(settings["resolution"])
    if resolution.dtype.kind not in "ui" or not np.array_equal(resolution, [640, 480]):
        raise ValueError("This capture adapter requires integer resolution [640, 480]")
    for field in ("focal_px", "aperture_cm"):
        value = settings[field]
        if (
            isinstance(value, bool)
            or not isinstance(value, Real)
            or not isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{field} must be a finite positive number")
    clipping = np.asarray(settings["clipping_range_m"])
    if (
        clipping.shape != (2,)
        or clipping.dtype.kind not in "uif"
        or not np.isfinite(clipping).all()
        or not 0 < clipping[0] < clipping[1]
    ):
        raise ValueError("clipping_range_m must be finite with 0 < near < far")
    return settings


def create_perception_camera(
    settings_path: Path,
    *,
    prim_path: str = "/World/Forklift/base_link/PerceptionCamera",
) -> Any:
    """Synthetic fixed base mount, not a measured D435i calibration."""
    settings = load_camera_settings(settings_path)
    from isaacsim.sensors.camera import Camera

    camera = Camera(
        prim_path=prim_path,
        frequency=-1,
        resolution=tuple(settings["resolution"]),
    )
    camera.set_local_pose(
        translation=np.asarray(settings["translation_m"]),
        orientation=np.asarray(settings["quaternion_wxyz"]),
        camera_axes="ros",
    )
    width, height = settings["resolution"]
    camera.set_horizontal_aperture(settings["aperture_cm"])
    camera.set_vertical_aperture(settings["aperture_cm"] * height / width)
    camera.set_focal_length(settings["aperture_cm"] * settings["focal_px"] / width)
    camera.set_clipping_range(*settings["clipping_range_m"])
    return camera


def _require_robot_velocity(robot, *, require_stopped):
    for value, label in (
        (robot.get_linear_velocity(), "translation"),
        (robot.get_angular_velocity(), "rotation"),
    ):
        velocity = np.asarray(value)
        if (
            velocity.shape != (3,)
            or velocity.dtype.kind not in "uif"
            or not np.isfinite(velocity).all()
        ):
            raise RuntimeError(f"Snapshot acquisition requires finite {label} velocity")
        if require_stopped and np.linalg.norm(velocity) > 0.012:
            raise RuntimeError(f"Snapshot acquisition requires stopped {label}")


def acquire_stationary_snapshot(
    camera: Any, world: Any, robot: Any
) -> tuple[SceneInput, RigidTransform]:
    """Return SceneInput and acquisition-time world_from_base for a stopped robot.

    Linear and angular speeds must remain below the stationary threshold for
    the entire frozen capture, preserving the stationary acquisition contract.
    """
    return _acquire_snapshot(camera, world, robot, require_stopped=True)


def acquire_frozen_snapshot(
    camera: Any, world: Any, robot: Any
) -> tuple[SceneInput, RigidTransform]:
    """Capture instantaneous RGB-D and world_from_base with physics paused.

    Nonzero finite robot velocities are allowed; physics must not advance during
    any render or getter call. This function only renders and reads state. The
    caller advances physics between captures, never during acquisition.
    """
    return _acquire_snapshot(camera, world, robot, require_stopped=False)


def _acquire_snapshot(camera, world, robot, *, require_stopped):
    """Pair same-product RGB-D with frozen simulation time and copied base pose.

    Warm-up rendering does not advance physics. SDK cached rendering_time is
    not used as a sensor timestamp.
    """
    stamp_s = world.current_time
    base, base_q_wxyz = robot.get_world_pose()
    # Pose accessors may reuse the same buffers on subsequent simulation calls.
    base = np.array(base, copy=True)
    base_q_wxyz = np.array(base_q_wxyz, copy=True)
    _require_robot_velocity(robot, require_stopped=require_stopped)

    def require_frozen_pose():
        if world.current_time != stamp_s:
            raise RuntimeError("Physics advanced during frozen RGB-D acquisition")
        after, after_q = robot.get_world_pose()
        if not np.allclose(after, base, atol=1e-8, rtol=0) or not np.allclose(
            after_q, base_q_wxyz, atol=1e-8, rtol=0
        ):
            raise RuntimeError("Robot pose changed during RGB-D acquisition")
        _require_robot_velocity(robot, require_stopped=require_stopped)

    for _ in range(8):
        world.render()
        require_frozen_pose()
    rgba = np.array(camera.get_rgba(), copy=True)
    require_frozen_pose()
    depth = np.array(camera.get_depth(), copy=True)
    require_frozen_pose()
    if rgba.shape != (480, 640, 4) or depth.shape != (480, 640):
        raise RuntimeError(f"RGB-D annotators not ready: {rgba.shape}, {depth.shape}")
    camera_pos, camera_q_wxyz = camera.get_world_pose(camera_axes="ros")
    camera_pos = np.array(camera_pos, copy=True)
    camera_q_wxyz = np.array(camera_q_wxyz, copy=True)
    require_frozen_pose()
    base_rotation = rotation_matrix_from_quaternion_xyzw(np.roll(base_q_wxyz, -1))
    camera_rotation = rotation_matrix_from_quaternion_xyzw(np.roll(camera_q_wxyz, -1))
    world_from_base = RigidTransform("base_link", "world", base_rotation, base)
    base_from_optical = RigidTransform(
        "camera_optical_frame",
        "base_link",
        base_rotation.T @ camera_rotation,
        base_rotation.T @ (camera_pos - base),
    )
    k = np.array(camera.get_intrinsics_matrix(), copy=True)
    require_frozen_pose()
    intrinsics = PinholeIntrinsics(
        640, 480, k[0, 0], k[1, 1], k[0, 2], k[1, 2], "camera_optical_frame"
    )
    scene = scene_input_from_rgbd_snapshot(
        rgb=rgba[:, :, :3],
        depth_m=depth,
        intrinsics=intrinsics,
        base_from_optical=base_from_optical,
        pixel_frame="camera_optical_frame",
        stamp_ns=round(stamp_s * 1e9),
        clock_domain="synthetic",
        source_provenance="synthetic",
        rectified=True,
        rgb_registered_to_depth_grid=True,
        depth_kind="optical_axis_z",
        depth_unit="m",
    )
    # Include annotator/calibration reads in the frozen acquisition boundary.
    require_frozen_pose()
    return scene, world_from_base


def save_snapshot(output: Path, scene, world_from_base) -> dict:
    """Persist raw metric depth and all reconstruction metadata."""
    from PIL import Image
    from run_transport import record_json

    output.mkdir(parents=True, exist_ok=False)
    Image.fromarray(scene.rgb).save(output / "rgb.png")
    np.save(output / "depth_m.npy", scene.depth_m)
    metadata = {
        "stamp_ns": scene.stamp_ns,
        "clock_domain": scene.clock_domain,
        "source_provenance": scene.source_provenance,
        "intrinsics": asdict(scene.intrinsics),
        "base_from_optical": asdict(scene.base_from_optical),
        "world_from_base": asdict(world_from_base),
        "depth_kind": "optical_axis_z",
        "depth_unit": "m",
        "rectified": True,
        "rgb_registered_to_depth_grid": True,
        "acquisition": "same_render_product_physics_frozen",
        "mount_calibration": "synthetic_fixed_base_mount",
    }
    (output / "metadata.json").write_text(record_json(metadata, indent=2) + "\n")
    return metadata
