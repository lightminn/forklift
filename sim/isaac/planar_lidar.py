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

# Real-time plan G3-1d (bench #388 adopted; docs/validation/2026-11-09-g3-1d-lidar-bench.md): how cast_scan_flags
# casts -- "loop" (the original loop; the default, every earlier run), "fast" (cast_scan_flags_fast, same results) or
# "oracle" (both on the same physics state, compared ray by ray; the loop's result is returned). Set once by the runner
# (--lidar-cast); the own cache and the oracle's counts live here for the process.
CAST_MODE = "loop"
_OWN_CACHE: dict = {}
_OWN_CACHE_MAX = 200_000
ORACLE = {"scans": 0, "mismatched_scans": 0, "mismatched_rays": 0, "own_true_rays": 0, "prefix_sets": [],
          "first_mismatch": None}


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
    if CAST_MODE != "loop":
        return _cast_by_mode(origin, directions, range_max_m, own_prefixes)
    return _loop_cast(origin, directions, range_max_m, own_prefixes)


def _loop_cast(origin, directions, range_max_m, own_prefixes):
    """cast_scan_flags' original loop (a module-private name: the live timers wrap cast_scan_flags, and the oracle's
    reference cast must not go through that wrapper again)."""
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


def cast_scan_flags_fast(
    origin: np.ndarray,
    directions: np.ndarray,
    range_max_m: float,
    *,
    own_prefixes: tuple = (SELF_PREFIX,),
    query=None,
    own_cache: dict | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Real-time plan G3-1d candidate (the microbench's arm B; the runner does not call it yet): cast_scan_flags'
    rays, results and own test, with the Python around each query trimmed -- the directions converted to floats once
    (``tolist``), the query and the vector type bound once, and the own test done once per distinct
    (rigidBody, collision) pair (``own_cache``, keyed also by the prefixes: they change with the carried pallet).
    The own test keeps cast_scan_flags' meaning -- a prefix anywhere in rigidBody + collision (Codex P1-2, plan 5.20:
    not startswith)."""
    import carb

    if query is None:
        from omni.physx import get_physx_scene_query_interface

        query = get_physx_scene_query_interface()
    cast, vec, reach = query.raycast_closest, carb.Float3, float(range_max_m)
    start = vec(*map(float, origin))
    n = len(directions)
    distances = np.full(n, np.nan)
    hits = np.zeros(n, dtype=bool)
    own = np.zeros(n, dtype=bool)
    cache = {} if own_cache is None else own_cache
    prefixes = tuple(own_prefixes)
    for index, (x, y, z) in enumerate(np.asarray(directions, dtype=float).tolist()):
        result = cast(start, vec(x, y, z), reach, False)
        if not result.get("hit", False):
            continue
        hits[index] = True
        distances[index] = float(result["distance"])
        key = (prefixes, result.get("rigidBody", ""), result.get("collision", ""))
        flag = cache.get(key)
        if flag is None:
            body = str(key[1]) + str(key[2])
            flag = cache[key] = any(prefix in body for prefix in prefixes)
        own[index] = flag
    return distances, hits, own


def _cast_by_mode(origin, directions, range_max_m, own_prefixes):
    if len(_OWN_CACHE) > _OWN_CACHE_MAX:  # bounded: prefix sets x bodies met; dropped whole, rebuilt on use
        _OWN_CACHE.clear()
    if CAST_MODE == "fast":
        return cast_scan_flags_fast(origin, directions, range_max_m, own_prefixes=own_prefixes, own_cache=_OWN_CACHE)
    if CAST_MODE != "oracle":
        raise ValueError(f"CAST_MODE {CAST_MODE!r}: loop, fast or oracle")
    ref = _loop_cast(origin, directions, range_max_m, own_prefixes)
    got = cast_scan_flags_fast(origin, directions, range_max_m, own_prefixes=own_prefixes, own_cache=_OWN_CACHE)
    ORACLE["scans"] += 1
    ORACLE["own_true_rays"] += int(ref[2].sum())
    prefixes = list(own_prefixes)
    if prefixes not in ORACLE["prefix_sets"]:
        ORACLE["prefix_sets"].append(prefixes)
    bad = ~(np.isclose(ref[0], got[0], rtol=0.0, atol=0.0, equal_nan=True) & (ref[1] == got[1]) & (ref[2] == got[2]))
    if bad.any():
        ORACLE["mismatched_scans"] += 1
        ORACLE["mismatched_rays"] += int(bad.sum())
        if ORACLE["first_mismatch"] is None:
            i = int(np.argmax(bad))
            ORACLE["first_mismatch"] = {"ray": i, "loop": [float(ref[0][i]), bool(ref[1][i]), bool(ref[2][i])],
                                        "fast": [float(got[0][i]), bool(got[1][i]), bool(got[2][i])],
                                        "prefixes": prefixes}
    return ref


def oracle_record() -> dict:
    return {"mode": CAST_MODE, **{k: (list(v) if isinstance(v, list) else v) for k, v in ORACLE.items()},
            "own_cache_entries": len(_OWN_CACHE)}


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
        # the profiled scan's ray count, booked in the seconds field (plan 5.22 M7's accounting: its extra clock reads)
        TIMERS.add("lidar.split.rays", float(len(directions)))
    return distances, hits, own


__all__ = [
    "LaserMount",
    "cast_scan",
    "cast_scan_flags",
    "cast_scan_flags_fast",
    "draw_points",
    "laser_pose_2d",
    "laser_rays_world",
    "scan_points_world",
]
