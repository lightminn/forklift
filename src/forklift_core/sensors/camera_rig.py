"""A rig of RGB-D cameras on base_link: mounts, intrinsics and depth noise.

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md. The rig description is a
plain mapping (the adapters read config/isaac_depth_rig.yaml); this module
turns it into per-camera intrinsics and base_link <- optical transforms, and
holds the assumed stereo depth noise. Optical axes: x right, y down, z forward.
Nothing here touches a simulator, ROS or a camera SDK.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from forklift_core._validation import _finite_scalar
from forklift_core.sensors.rgbd import PinholeIntrinsics

# Columns are the optical x, y, z axes in base_link for a camera looking along
# base +x: x_opt = -y_base, y_opt = -z_base, z_opt = +x_base.
_BASE_FROM_OPTICAL_FORWARD = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
)
_CAMERA_KEYS = {"xyz_m", "yaw_deg", "pitch_down_deg"}
_RIG_KEYS = {
    "rate_hz",
    "width",
    "height",
    "fx_px",
    "depth_range_m",
    "clipping_range_m",
    "depth_noise_coefficient_per_m",
    "frozen_renders",
    "cameras",
}


def base_from_optical_rotation(yaw_rad: float, pitch_down_rad: float) -> NDArray:
    """base_link <- optical rotation for a camera yawed, then pitched down."""
    yaw = _finite_scalar(yaw_rad, "yaw_rad")
    pitch = _finite_scalar(pitch_down_rad, "pitch_down_rad")
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    about_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    # Positive rotation about base y turns +x towards -z: looking down.
    about_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return about_z @ about_y @ _BASE_FROM_OPTICAL_FORWARD


@dataclass(frozen=True)
class RigCamera:
    """One camera: integer-index-centre intrinsics and its fixed mount.

    ``intrinsics`` uses the OpenCV convention (pixel centres at integer
    indices), the K a SLAM front end expects. ``rotation_base_from_optical``
    (3, 3) and ``translation_base_m`` (3,) place the optical frame in base_link.
    """

    name: str
    intrinsics: PinholeIntrinsics
    rotation_base_from_optical: NDArray
    translation_base_m: NDArray

    @property
    def frame_id(self) -> str:
        return f"{self.name}_optical"

    def ray_directions_base(self, pixels_uv: NDArray) -> NDArray:
        """Unit ray directions (N, 3) in base_link through pixel centres (N, 2)."""
        uv = np.asarray(pixels_uv, dtype=float).reshape(-1, 2)
        k = self.intrinsics
        optical = np.column_stack(
            ((uv[:, 0] - k.cx) / k.fx, (uv[:, 1] - k.cy) / k.fy, np.ones(len(uv)))
        )
        optical /= np.linalg.norm(optical, axis=1, keepdims=True)
        return optical @ np.asarray(self.rotation_base_from_optical).T


@dataclass(frozen=True)
class CameraRig:
    rate_hz: int
    cameras: tuple[RigCamera, ...]
    depth_range_m: tuple[float, float]
    depth_noise_coefficient_per_m: float
    frozen_renders: int
    clipping_range_m: tuple[float, float]

    def camera(self, name: str) -> RigCamera:
        for camera in self.cameras:
            if camera.name == name:
                return camera
        raise KeyError(name)

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(camera.name for camera in self.cameras)


def rig_from_config(config: dict) -> CameraRig:
    """Validate a rig mapping (config/isaac_depth_rig.yaml) and build the rig.

    Unknown or missing keys are refused rather than defaulted. Every camera
    shares the rig's resolution and focal length; the principal point is the
    image centre in integer-index coordinates ((w - 1) / 2, (h - 1) / 2).
    """
    if not isinstance(config, dict):
        raise ValueError("rig config must be a mapping")
    if set(config) != _RIG_KEYS:
        raise ValueError(
            f"rig config keys must be exactly {sorted(_RIG_KEYS)}, got {sorted(config)}"
        )
    rate = config["rate_hz"]
    if isinstance(rate, bool) or not isinstance(rate, int) or rate <= 0:
        raise ValueError("rate_hz must be a positive integer")
    renders = config["frozen_renders"]
    if isinstance(renders, bool) or not isinstance(renders, int) or renders < 1:
        raise ValueError("frozen_renders must be a positive integer")
    width, height = config["width"], config["height"]
    fx = _finite_scalar(config["fx_px"], "fx_px")
    near, far = (_finite_scalar(v, "depth_range_m") for v in config["depth_range_m"])
    if not 0 < near < far:
        raise ValueError("depth_range_m must be 0 < near < far")
    clip_near, clip_far = (
        _finite_scalar(v, "clipping_range_m") for v in config["clipping_range_m"]
    )
    if not 0 < clip_near < near or clip_far <= far:
        raise ValueError(
            "clipping_range_m must open before and close after depth_range_m"
        )
    noise = _finite_scalar(
        config["depth_noise_coefficient_per_m"], "depth_noise_coefficient_per_m"
    )
    if noise < 0:
        raise ValueError("depth_noise_coefficient_per_m must be nonnegative")
    cameras = config["cameras"]
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError("cameras must be a nonempty mapping")
    built = []
    for name, mount in cameras.items():
        if not isinstance(name, str) or not name.isidentifier():
            raise ValueError(f"camera name {name!r} must be an identifier")
        if not isinstance(mount, dict) or set(mount) != _CAMERA_KEYS:
            raise ValueError(
                f"camera {name} keys must be exactly {sorted(_CAMERA_KEYS)}"
            )
        xyz = np.array([_finite_scalar(v, "xyz_m") for v in mount["xyz_m"]])
        if xyz.shape != (3,):
            raise ValueError(f"camera {name} xyz_m must hold three values")
        rotation = base_from_optical_rotation(
            math.radians(_finite_scalar(mount["yaw_deg"], "yaw_deg")),
            math.radians(_finite_scalar(mount["pitch_down_deg"], "pitch_down_deg")),
        )
        intrinsics = PinholeIntrinsics(
            width, height, fx, fx, (width - 1) / 2, (height - 1) / 2, f"{name}_optical"
        )
        rotation.setflags(write=False)
        xyz.setflags(write=False)
        built.append(RigCamera(name, intrinsics, rotation, xyz))
    return CameraRig(
        rate, tuple(built), (near, far), noise, renders, (clip_near, clip_far)
    )


def pixel_grid(intrinsics: PinholeIntrinsics, columns: int, rows: int) -> NDArray:
    """(columns * rows, 2) pixel centres of an evenly spaced interior grid."""
    if columns < 1 or rows < 1:
        raise ValueError("grid needs at least one column and row")
    u = (np.arange(columns) + 0.5) * intrinsics.width / columns - 0.5
    v = (np.arange(rows) + 0.5) * intrinsics.height / rows - 0.5
    uu, vv = np.meshgrid(u, v)
    return np.column_stack((uu.ravel(), vv.ravel()))


class DepthNoise:
    """Assumed stereo depth error, then millimetre quantisation (plan v1).

    sigma_z = coefficient * z^2, independent Gaussian per pixel, drawn from a
    stream keyed by (seed, camera index, frame index) -- so an offline replay
    and an online run of the same frame draw the same noise. Depth outside
    [near, far] (and non-finite depth) becomes 0, the z16 "no data" value.
    ``enabled=False`` keeps only the range limits and the quantisation.
    """

    def __init__(
        self,
        *,
        coefficient_per_m: float,
        range_m: tuple[float, float],
        seed: int,
        enabled: bool = True,
    ):
        self.coefficient = _finite_scalar(coefficient_per_m, "coefficient_per_m")
        self.near, self.far = (float(v) for v in range_m)
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError("seed must be a nonnegative integer")
        self.seed, self.enabled = seed, bool(enabled)

    def depth_mm(
        self, depth_m: NDArray, camera_index: int, frame_index: int
    ) -> NDArray:
        """uint16 millimetre depth of the same shape; 0 marks no measurement."""
        depth = np.asarray(depth_m, dtype=float)
        finite = np.isfinite(depth)
        z = np.where(finite, depth, 0.0)
        if self.enabled and self.coefficient > 0:
            rng = np.random.default_rng(
                [self.seed, 10 + int(camera_index), int(frame_index)]
            )
            z = z + rng.standard_normal(z.shape) * self.coefficient * z * z
        valid = finite & (z >= self.near) & (z <= self.far)
        return np.where(valid, np.round(z * 1000.0), 0).astype(np.uint16)


__all__ = [
    "CameraRig",
    "DepthNoise",
    "RigCamera",
    "base_from_optical_rotation",
    "pixel_grid",
    "rig_from_config",
]
