"""Planar 2D LiDAR by PhysX ray casts, plus the SDK-free geometry around it.

Rays start at the laser origin and run in the laser's scan plane, which tilts
with the truck body. Only cast_scan touches Isaac (omni.physx, carb); the rest
runs on CPU and is tested there. Every beam of one scan is cast against the
same physics state, so a scan has no rotation skew. The ray hits collision
geometry: a surface without a collider is invisible to it.
"""

import os
import time
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
    origin: np.ndarray,
    directions: np.ndarray,
    range_max_m: float,
    *,
    ignore_self: bool = False,
) -> tuple[np.ndarray, np.ndarray, int]:
    """PhysX closest-hit distance per ray; returns (distances, hit, self_hits).

    A hit on the truck's own colliders is kept as a hit -- a real body blocks
    the beam too -- and counted, so a mount that sees its own truck shows up.
    ignore_self drops those beams instead (no hit): a reference scan cast from
    a pose the truck is not at must not contain the truck (plan v3.8).
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
        body = str(result.get("rigidBody", "")) + str(result.get("collision", ""))
        own = SELF_PREFIX in body
        self_hits += own
        if own and ignore_self:
            continue
        hits[index] = True
        distances[index] = float(result["distance"])
    return distances, hits, self_hits


def cast_scan_flags(
    origin: np.ndarray,
    directions: np.ndarray,
    range_max_m: float,
    *,
    own_prefixes: tuple = (SELF_PREFIX,),
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """PhysX closest hit per ray with a per-beam own flag (priority-5 plan D2).

    Returns (distances, hit, own): own marks beams whose closest hit is on a
    prim under any of own_prefixes -- the truck, and the pallet while it is
    carried. Those beams stay hits (the body blocks them) but the obstacle
    grid neither marks nor clears with them.
    """
    import carb
    from omni.physx import get_physx_scene_query_interface

    query = get_physx_scene_query_interface()
    start = carb.Float3(*map(float, origin))
    distances = np.full(len(directions), np.nan)
    hits = np.zeros(len(directions), dtype=bool)
    own = np.zeros(len(directions), dtype=bool)
    if _profile_due():
        return _cast_scan_flags_profiled(query, start, directions, range_max_m, own_prefixes, distances, hits, own)
    for index, direction in enumerate(directions):
        result = query.raycast_closest(
            start, carb.Float3(*map(float, direction)), float(range_max_m), False
        )
        if not result.get("hit", False):
            continue
        body = str(result.get("rigidBody", "")) + str(result.get("collision", ""))
        hits[index] = True
        distances[index] = float(result["distance"])
        own[index] = any(prefix in body for prefix in own_prefixes)
    return distances, hits, own


# Real-time plan 5.20 G3-1a: every N-th scan (FORKLIFT_LIVE_LIDAR_PROFILE_EVERY, 0 = off) is cast through the same
# steps with its time split -- the PhysX query, the direction's conversion, and the own-body test -- into the live
# timers (lidar.split.*; the scan count in lidar.split.scans). Only those scans pay for the clocks; the results are the
# loop's own, step for step.
_PROFILE_EVERY = int(os.environ.get("FORKLIFT_LIVE_LIDAR_PROFILE_EVERY", "0") or 0)
_profile_count = [0]


def _profile_due() -> bool:
    if _PROFILE_EVERY <= 0:
        return False
    from forklift_core.perf import TIMERS

    if not TIMERS.enabled:  # a batch run: nothing would book the split (Codex review)
        return False
    _profile_count[0] += 1
    return _profile_count[0] % _PROFILE_EVERY == 0


def _cast_scan_flags_profiled(query, start, directions, range_max_m, own_prefixes, distances, hits, own):
    import carb
    from forklift_core.perf import TIMERS

    clock = time.perf_counter
    t_conv = t_query = t_own = 0.0
    t0 = clock()
    for index, direction in enumerate(directions):
        a = clock()
        d = carb.Float3(*map(float, direction))
        b = clock()
        result = query.raycast_closest(start, d, float(range_max_m), False)
        c = clock()
        t_conv += b - a
        t_query += c - b
        if not result.get("hit", False):
            continue
        e = clock()
        body = str(result.get("rigidBody", "")) + str(result.get("collision", ""))
        hits[index] = True
        distances[index] = float(result["distance"])
        own[index] = any(prefix in body for prefix in own_prefixes)
        t_own += clock() - e
    total = clock() - t0
    if TIMERS.enabled:
        TIMERS.add("lidar.split.scans", total)
        TIMERS.add("lidar.split.convert", t_conv)
        TIMERS.add("lidar.split.query", t_query)
        TIMERS.add("lidar.split.hit_and_own", t_own)
    return distances, hits, own


__all__ = [
    "LaserMount",
    "cast_scan",
    "cast_scan_flags",
    "draw_points",
    "laser_pose_2d",
    "laser_rays_world",
    "scan_points_world",
]
