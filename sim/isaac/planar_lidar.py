"""Planar 2D LiDAR by PhysX ray casts, plus the SDK-free geometry around it.

Rays start at the laser origin and run in the laser's scan plane, which tilts
with the truck body. Only cast_scan touches Isaac (omni.physx, carb); the rest
runs on CPU and is tested there. Every beam of one scan is cast against the
same physics state, so a scan has no rotation skew. The ray hits collision
geometry: a surface without a collider is invisible to it.
"""

from dataclasses import dataclass

import numpy as np

SELF_PREFIX = "/World/Forklift"


@dataclass(frozen=True)
class LaserMount:
    """base_link -> laser: translation in metres and yaw about base_link z."""

    xyz_m: tuple[float, float, float]
    yaw_rad: float


def _rotation_wxyz(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion_wxyz, dtype=float) / np.linalg.norm(
        quaternion_wxyz
    )
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def laser_rays_world(
    base_position: np.ndarray,
    base_quaternion_wxyz: np.ndarray,
    mount: LaserMount,
    beam_angles_rad: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """World ray origin (3,) and unit directions (N, 3) for base_link's pose."""
    world_from_base = _rotation_wxyz(base_quaternion_wxyz)
    origin = np.asarray(base_position, float) + world_from_base @ np.asarray(
        mount.xyz_m, float
    )
    angles = np.asarray(beam_angles_rad, float) + mount.yaw_rad
    in_base = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    return origin, in_base @ world_from_base.T


def laser_pose_2d(
    base_position: np.ndarray, base_quaternion_wxyz: np.ndarray, mount: LaserMount
) -> tuple[float, float, float]:
    """Ground-truth laser x, y and yaw in the world, projected to the floor."""
    origin, directions = laser_rays_world(
        base_position, base_quaternion_wxyz, mount, np.array([0.0])
    )
    return (
        float(origin[0]),
        float(origin[1]),
        float(np.arctan2(directions[0, 1], directions[0, 0])),
    )


def scan_points_world(
    origin: np.ndarray, directions: np.ndarray, ranges_m: np.ndarray
) -> np.ndarray:
    """World points of the measured beams only; +inf and -inf give no point."""
    measured = np.isfinite(ranges_m)
    return origin + directions[measured] * ranges_m[measured, None]


def draw_points(
    rgb: np.ndarray,
    pixels_uv: np.ndarray,
    colour: tuple[int, int, int],
    radius: int = 0,
) -> np.ndarray:
    """Copy of an (H, W, 3) image with square dots at the pixels inside it."""
    out = np.array(rgb, copy=True)
    height, width = out.shape[:2]
    uv = np.asarray(pixels_uv, float)
    uv = uv[np.isfinite(uv).all(axis=1)]
    columns, rows = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
    for dv in range(-radius, radius + 1):
        for du in range(-radius, radius + 1):
            u, v = columns + du, rows + dv
            inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
            out[v[inside], u[inside]] = colour
    return out


def cast_scan(
    origin: np.ndarray, directions: np.ndarray, range_max_m: float
) -> tuple[np.ndarray, np.ndarray, int]:
    """PhysX closest-hit distance per ray; returns (distances, hit, self_hits).

    A hit on the truck's own colliders is kept as a hit -- a real body blocks
    the beam too -- and counted, so a mount that sees its own truck shows up.
    """
    import carb
    from omni.physx import get_physx_scene_query_interface

    query = get_physx_scene_query_interface()
    start = carb.Float3(*map(float, origin))
    distances = np.full(len(directions), np.nan)
    hits = np.zeros(len(directions), dtype=bool)
    self_hits = 0
    for index, direction in enumerate(directions):
        result = query.raycast_closest(
            start, carb.Float3(*map(float, direction)), float(range_max_m), False
        )
        if not result.get("hit", False):
            continue
        hits[index] = True
        distances[index] = float(result["distance"])
        body = str(result.get("rigidBody", "")) + str(result.get("collision", ""))
        self_hits += SELF_PREFIX in body
    return distances, hits, self_hits


__all__ = [
    "LaserMount",
    "cast_scan",
    "draw_points",
    "laser_pose_2d",
    "laser_rays_world",
    "scan_points_world",
]
