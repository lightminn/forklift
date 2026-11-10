"""Top-down video cameras that do not show the ceiling (user request via the lead, 2026-11-10).

The warehouse keeps its ceiling lamps visible even after ``scene.hide_overhead`` (anything holding a light stays, so
the hall is lit), and the pillar assemblies' roof beams belong to parts that reach the floor. In a top-down video they
read as obstacles on an empty floor. This clips them in the video camera only: a camera looking straight down sees
the plane at height z at the distance ``eye_z - z``, so a near clipping distance of ``eye_z - clip_z`` cuts everything
above ``clip_z`` from that camera's picture. Lights, the perception cameras, the LiDAR and physics are untouched.

``clip_z`` is read from the stage: the lowest bottom of the environment's light-holding top-level parts, less a margin
(full_warehouse.usd, 2026-11-10: the SM_LampCeilingA lamps' bottoms at 5.82 m, the pillar roof beams at 7.88 m with
braces from 6.01 m; floor-standing racks, walls and pillars keep their parts below the plane).
No Isaac imports at module level.
"""

from __future__ import annotations

MARGIN_M = 0.3
FLOOR_M = 3.0  # never below scene.hide_overhead's line: floor-level props stay whole


def clip_z_from(parts, margin_m: float = MARGIN_M, floor_m: float = FLOOR_M) -> float | None:
    """``parts``: (bottom z, holds a light) of the environment's top-level parts. The clip height: the lowest
    light-holding bottom less ``margin_m``, not below ``floor_m``; None without a light-holding part."""
    lit = [float(z) for z, light in parts if light]
    if not lit:
        return None
    return max(min(lit) - margin_m, floor_m)


def top_down_near(eye_z: float, clip_z: float | None, near_now: float) -> float | None:
    """The near clipping distance that cuts everything above ``clip_z`` for a camera at ``eye_z`` looking straight
    down, or None when it would not cut anything new (no clip height, or the plane at or above the camera's
    present near distance)."""
    if clip_z is None:
        return None
    near = float(eye_z) - float(clip_z)
    return near if near > float(near_now) else None


def read_parts(stage, root: str = "/World/Environment") -> list:
    """(bottom z, holds a light) of every top-level part under ``root`` with bounds (pxr, at call time)."""
    from pxr import Usd, UsdGeom, UsdLux

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    top = stage.GetPrimAtPath(root)
    out = []
    for prim in (top.GetChildren() if top else []):
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        light = any(item.HasAPI(UsdLux.LightAPI) for item in Usd.PrimRange(prim))
        out.append((float(box.GetMin()[2]), bool(light)))
    return out


def apply(camera, stage, eye_z: float, root: str = "/World/Environment") -> dict:
    """Clip a straight-down Isaac Camera above the stage's clip height; the record for the run's camera entry."""
    clip_z = clip_z_from(read_parts(stage, root))
    near_now, far_now = (float(v) for v in camera.get_clipping_range())
    near = top_down_near(eye_z, clip_z, near_now)
    if near is not None:
        camera.set_clipping_range(near_distance=near, far_distance=far_now)
    return {"clip_z_m": clip_z, "eye_z_m": float(eye_z), "near_m": near,
            "clipping_range_m": [float(v) for v in camera.get_clipping_range()]}
