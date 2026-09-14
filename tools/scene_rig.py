"""Synthetic depth rig for pocket-evidence measurement.

This is the authoritative rig for the numbers in
``docs/validation/2026-09-14-pocket-evidence-measurements.md``. It renders
first-hit depth for axis-aligned or yaw-rotated boxes over a floor and a back
wall, using the same slab intersection as the frozen test fixture. A unit test
cross-checks the two, because a rig that silently drifts from the fixture has
already produced wrong conclusions once: an earlier scratch rig rotated each
slab in place without moving its centre, which is not a rigid rotation and
changed a 60-pose result from 16 to 1.

Two axes are explicit here because leaving them implicit has cost real time:

``quantize``
    Depth is rounded to 1 mm by default, as the fixture does. Turning it off
    changes lower-deck evidence counts materially, so every table must say
    which setting produced it.

``camera``
    The camera pose is an argument, not a constant. The detector's usable
    range is a function of it -- the vertical field of view, not any gate,
    is what makes the pallet undetectable below about 2 m at the default pose.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Iterable, NamedTuple, Sequence

import numpy as np

from forklift_core.geometry import (
    RigidTransform,
    rotation_matrix_from_quaternion_xyzw,
)
from forklift_core.perception.pallet_geometry import PalletGeometry
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import PinholeIntrinsics

# The catalogue v1 capture geometry. Shared with tests/fixtures/synthetic_scene.py;
# the cross-check test fails if they diverge.
WIDTH, HEIGHT = 640, 480
FOCAL_PX = 465.741156
OPTICAL_QUATERNION_XYZW = (-0.5, 0.5, -0.5, 0.5)
DEFAULT_CAMERA_XYZ_M = (0.75, 0.0, 0.5)
BACK_WALL_X_M = 6.0
OPTICAL_FRAME_ID = "camera_optical_frame"
BASE_FRAME_ID = "base_link"
QUANTIZE_STEP_M = 0.001


class Box(NamedTuple):
    """An oriented box: half-open slab, size along its own axes."""

    centre_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    yaw_rad: float = 0.0


class Camera(NamedTuple):
    """Camera placement. ``tilt_rad`` is positive downwards."""

    xyz_m: tuple[float, float, float] = DEFAULT_CAMERA_XYZ_M
    tilt_rad: float = 0.0

    def base_from_optical(self) -> RigidTransform:
        rotation = rotation_matrix_from_quaternion_xyzw(OPTICAL_QUATERNION_XYZW)
        if self.tilt_rad:
            c, s = math.cos(self.tilt_rad), math.sin(self.tilt_rad)
            # Pitch about the base y axis; positive tilt looks downwards.
            pitch = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
            rotation = pitch @ rotation
        return RigidTransform(
            OPTICAL_FRAME_ID,
            BASE_FRAME_ID,
            rotation,
            np.asarray(self.xyz_m, dtype=float),
        )


def intrinsics() -> PinholeIntrinsics:
    return PinholeIntrinsics(
        width=WIDTH,
        height=HEIGHT,
        fx=FOCAL_PX,
        fy=FOCAL_PX,
        cx=WIDTH / 2,
        cy=HEIGHT / 2,
        frame_id=OPTICAL_FRAME_ID,
    )


def _rays(spec: PinholeIntrinsics, rotation: np.ndarray) -> np.ndarray:
    v, u = np.indices((spec.height, spec.width))
    optical = np.stack(
        (
            (u - spec.cx) / spec.fx,
            (v - spec.cy) / spec.fy,
            np.ones_like(u),
        ),
        axis=-1,
    )
    return optical @ rotation.T


def _box_depth(
    origin: np.ndarray,
    directions: np.ndarray,
    centre: np.ndarray,
    size: Sequence[float],
    rotation: np.ndarray,
) -> np.ndarray:
    """Slab intersection, retaining optical-z ray parametrization."""
    local_origin = (origin - centre) @ rotation
    local_rays = directions @ rotation
    lower, upper = -np.asarray(size) / 2, np.asarray(size) / 2
    entry = np.full(directions.shape[:2], -np.inf)
    leave = np.full(directions.shape[:2], np.inf)
    for axis in range(3):
        ray = local_rays[..., axis]
        parallel = np.abs(ray) < 1e-12
        low, high = np.full(ray.shape, -np.inf), np.full(ray.shape, np.inf)
        np.divide(lower[axis] - local_origin[axis], ray, out=low, where=~parallel)
        np.divide(upper[axis] - local_origin[axis], ray, out=high, where=~parallel)
        entry = np.maximum(entry, np.minimum(low, high))
        leave = np.minimum(leave, np.maximum(low, high))
        if not lower[axis] <= local_origin[axis] <= upper[axis]:
            leave[parallel] = -np.inf
    depth = np.where(entry > 0, entry, leave)
    return np.where((leave >= entry) & (depth > 0), depth, np.inf)


def _yaw_matrix(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def render(
    boxes: Iterable[Box],
    *,
    camera: Camera = Camera(),
    quantize: bool = True,
    floor: bool = True,
    back_wall: bool = True,
) -> SceneInput:
    """First-hit depth of ``boxes`` over the floor and back wall.

    Each box carries its own yaw; the caller has already placed its centre.
    Use :func:`place` to put an assembly at a pose -- it rotates the centres,
    which is what makes the rotation rigid.
    """
    spec = intrinsics()
    transform = camera.base_from_optical()
    origin = np.asarray(transform.translation_m, dtype=float)
    rays = _rays(spec, transform.rotation)
    depth = np.full((spec.height, spec.width), np.inf)
    if floor:
        # z = 0 plane, hit where the ray descends.
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -origin[2] / rays[..., 2]
        depth = np.fmin(depth, np.where((rays[..., 2] < 0) & (t > 0), t, np.inf))
    if back_wall:
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (BACK_WALL_X_M - origin[0]) / rays[..., 0]
        depth = np.fmin(depth, np.where((rays[..., 0] > 0) & (t > 0), t, np.inf))
    for box in boxes:
        hit = _box_depth(
            origin,
            rays,
            np.asarray(box.centre_m, dtype=float),
            box.size_m,
            _yaw_matrix(box.yaw_rad),
        )
        depth = np.fmin(depth, hit)
    depth = np.where(np.isfinite(depth), depth, np.nan)
    if quantize:
        depth = np.round(depth / QUANTIZE_STEP_M) * QUANTIZE_STEP_M
    rgb = np.zeros((spec.height, spec.width, 3), dtype=np.uint8)
    return SceneInput(
        rgb=rgb,
        depth_m=depth,
        intrinsics=spec,
        base_from_optical=transform,
        stamp_ns=2_000_000_000,
        clock_domain="ros_sim",
        source_provenance="synthetic",
    )


def place(
    boxes: Iterable[Box], *, x_m: float, y_m: float = 0.0, yaw_rad: float = 0.0
) -> list[Box]:
    """Rigidly move an origin-centred assembly to a pose.

    Both the centres and the boxes rotate. Rotating only the boxes leaves the
    assembly's shape wrong by up to half its depth, which is the drift this
    module exists to prevent.
    """
    rotation = _yaw_matrix(yaw_rad)
    placed = []
    for box in boxes:
        cx, cy, cz = box.centre_m
        rx, ry, _ = rotation @ np.array([cx, cy, 0.0])
        placed.append(
            Box((x_m + rx, y_m + ry, cz), box.size_m, box.yaw_rad + yaw_rad)
        )
    return placed


def pallet(geometry: PalletGeometry) -> list[Box]:
    """The authoritative 22-box assembly, centred on the origin."""
    from tools.build_pallet_model import pallet_boxes

    return [Box(tuple(b.centre_m), tuple(b.size_m)) for b in pallet_boxes(geometry)]


def true_pockets(
    geometry: PalletGeometry, *, x_m: float, y_m: float = 0.0, yaw_rad: float = 0.0
) -> np.ndarray:
    """Pocket centres on the approach face, in the base frame.

    The approach face is half the depth in front of the placement centre. A
    metric that compares a reported pocket against the placement point instead
    reads half the pallet depth on a perfect detection.
    """
    offset = geometry.opening_centre_offset_m
    half_depth = geometry.overall_depth_m / 2
    z = geometry.opening_centre_height_m
    rotation = _yaw_matrix(yaw_rad)
    out = []
    for sign in (1.0, -1.0):
        local = np.array([-half_depth, sign * offset, 0.0])
        rx, ry, _ = rotation @ local
        out.append((x_m + rx, y_m + ry, z))
    return np.asarray(out, dtype=float)


def position_error_m(reported: Sequence[Sequence[float]], truth: np.ndarray) -> float:
    """Worst over reported pockets of the distance to the nearer true pocket."""
    worst = 0.0
    for pocket in reported:
        p = np.asarray(pocket, dtype=float)[:2]
        worst = max(worst, float(np.min(np.linalg.norm(truth[:, :2] - p, axis=1))))
    return worst


def quantized(scene: SceneInput, *, step_m: float = QUANTIZE_STEP_M) -> SceneInput:
    depth = np.round(np.asarray(scene.depth_m, dtype=float) / step_m) * step_m
    return dataclasses.replace(scene, depth_m=depth)
