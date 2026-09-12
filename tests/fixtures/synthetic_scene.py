"""Independent ray tracing of the five-box synthetic pallet fixture."""

import math

import numpy as np

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.scene_dataset import SceneInput
from forklift_core.sensors.rgbd import PinholeIntrinsics

CAMERA = {
    "width": 640,
    "height": 480,
    "fx": 465.741156,
    "fy": 465.741156,
    "cx": 320.0,
    "cy": 240.0,
    "translation_m": (0.75, 0.0, 0.5),
    "quaternion_xyzw": (-0.5, 0.5, -0.5, 0.5),
}
BACK_WALL_X_M = 6.0


def _rays(intrinsics, rotation):
    v, u = np.indices((intrinsics.height, intrinsics.width))
    optical = np.stack(
        (
            (u - intrinsics.cx) / intrinsics.fx,
            (v - intrinsics.cy) / intrinsics.fy,
            np.ones_like(u),
        ),
        axis=-1,
    )
    return optical @ rotation.T


def _box_depth(origin, directions, centre, size, rotation):
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


def _opening_intersection(scene, truth, side):
    rays = _rays(scene.intrinsics, scene.base_from_optical.rotation)
    origin = scene.base_from_optical.translation_m
    normal = np.asarray(truth["outward_normal"])
    point = np.asarray(truth[f"{side}_centre_m"])
    denominator = rays @ normal
    distance = np.full(denominator.shape, np.nan)
    np.divide(
        (point - origin) @ normal,
        denominator,
        out=distance,
        where=np.abs(denominator) > 1e-12,
    )
    intersections = origin + distance[..., None] * rays
    left_axis = np.array((-math.sin(truth["yaw_rad"]), math.cos(truth["yaw_rad"]), 0))
    lateral = (intersections - point) @ left_axis
    inside = (
        (distance > 0)
        & (np.abs(lateral) < truth["opening_width_m"] / 2)
        & (intersections[..., 2] >= 0.06)
        & (intersections[..., 2] <= 0.24)
    )
    return inside, distance, rays


def make_pallet_scene(
    *,
    centre_xy_m: tuple[float, float] = (2.5, 0.0),
    yaw_rad: float = 0.0,
    opening_width_m: float = 0.24,
    pallet: bool = True,
    lookalike: bool = False,
    occluder: dict | None = None,
    openings_unknown: bool = False,
    unknown_patch: tuple[int, int, int, int] | None = None,
    extra_box: dict | None = None,
    stamp_ns: int = 2_000_000_000,
) -> tuple[SceneInput, dict]:
    """Return float64 axial depth in m, uint8 RGB, and exact base-frame truth.

    Missing intersections and explicit unknown patches are NaN. Occluder
    width_frac is relative to opening width, gap_m locates its centre ahead
    of the front plane, and boxes rest on the z=0 floor.
    """
    intrinsics = PinholeIntrinsics(
        **{name: CAMERA[name] for name in ("width", "height", "fx", "fy", "cx", "cy")},
        frame_id="camera_optical_frame",
    )
    transform = RigidTransform(
        "camera_optical_frame",
        "base_link",
        rotation_matrix_from_quaternion_xyzw(CAMERA["quaternion_xyzw"]),
        CAMERA["translation_m"],
    )
    rays, origin = _rays(intrinsics, transform.rotation), transform.translation_m
    depth = np.full((intrinsics.height, intrinsics.width), np.inf)
    rgb = np.zeros((*depth.shape, 3), dtype=np.uint8)

    def surface(values, colour):
        nearer = (values > 0) & (values < depth)
        depth[nearer], rgb[nearer] = values[nearer], colour

    floor = np.full(depth.shape, np.inf)
    np.divide(-origin[2], rays[..., 2], out=floor, where=rays[..., 2] < -1e-12)
    surface(floor, (90, 90, 90))
    wall = np.full(depth.shape, np.inf)
    np.divide(
        BACK_WALL_X_M - origin[0], rays[..., 0], out=wall, where=rays[..., 0] > 1e-12
    )
    surface(wall, (180, 180, 190))
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    rotation = np.array(((cosine, -sine, 0), (sine, cosine, 0), (0, 0, 1)))
    centre = np.array((*centre_xy_m, 0.0))
    axis, left_axis = rotation[:, 0], rotation[:, 1]
    front = centre - 0.3 * axis
    offset = 0.05 + opening_width_m / 2
    truth = {
        "left_centre_m": tuple(front + offset * left_axis + (0, 0, 0.15)),
        "right_centre_m": tuple(front - offset * left_axis + (0, 0, 0.15)),
        "yaw_rad": yaw_rad,
        "opening_width_m": opening_width_m,
        "front_plane_point_m": tuple(front),
        "outward_normal": tuple(-axis),
    }

    def box(local_centre, size, colour=(190, 130, 65)):
        surface(
            _box_depth(origin, rays, centre + rotation @ local_centre, size, rotation),
            colour,
        )

    if pallet:
        outer_width = (0.8 - 2 * opening_width_m - 0.1) / 2
        box((0, 0, 0.025), (0.6, 0.8, 0.05))
        box((0, 0, 0.275), (0.6, 0.8, 0.05))
        box((0, 0, 0.15), (0.6, 0.1, 0.20))
        for side in (-1, 1):
            box((0, side * (0.4 - outer_width / 2), 0.15), (0.6, outer_width, 0.20))
    if lookalike:
        box((0, 0, 0.15), (0.6, 0.8, 0.30))
    if occluder is not None:
        side = 1 if occluder["side"] == "left" else -1
        box(
            (-0.3 - occluder["gap_m"], side * offset, occluder["height_m"] / 2),
            (
                occluder["depth_m"],
                opening_width_m * occluder["width_frac"],
                occluder["height_m"],
            ),
            (160, 55, 55),
        )
    if extra_box is not None:
        size = extra_box["size_m"]
        surface(
            _box_depth(
                origin,
                rays,
                np.array((*extra_box["centre_xy_m"], size[2] / 2)),
                size,
                np.eye(3),
            ),
            (50, 80, 160),
        )
    depth[~np.isfinite(depth)] = np.nan
    scene = SceneInput(
        rgb, depth, intrinsics, transform, stamp_ns, "synthetic", "synthetic"
    )
    if openings_unknown:
        for side in ("left", "right"):
            mask, _, _ = _opening_intersection(scene, truth, side)
            depth[mask] = np.nan
    if unknown_patch is not None:
        u0, v0, u1, v1 = unknown_patch
        depth[v0:v1, u0:u1] = np.nan
    return scene, truth


def opening_ray_fractions(scene: SceneInput, truth: dict, side: str) -> dict:
    """Classify all opening rays independently using signed plane distance."""
    mask, distance, rays = _opening_intersection(scene, truth, side)
    observed = scene.depth_m[mask]
    signed = (observed - distance[mask]) * (rays[mask] @ truth["outward_normal"])
    front = np.isfinite(observed) & (signed > 0.05)
    behind = np.isfinite(observed) & (signed < -0.05)
    unknown = ~np.isfinite(observed)
    total = int(mask.sum())
    counts = {
        "front": int(front.sum()),
        "behind": int(behind.sum()),
        "unknown": int(unknown.sum()),
        "near_plane": int((~(front | behind | unknown)).sum()),
    }
    return {
        **{name: count / total if total else 0.0 for name, count in counts.items()},
        "total": total,
    }
