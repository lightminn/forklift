"""Dress a random factory floor with NVIDIA Isaac props -- the look only.

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W6). User request
(2026-10-09): NVIDIA factory assets instead of coloured boxes. world_scene's
boxes stay what they were -- the colliders the LiDAR casts against, matching
the planner's map -- but are hidden; props are fitted inside them without any
physics, so planning, traffic, the protective field and the safety checks see
the same world as the undressed scene. Every prop's bounds are measured on
the stage at start-up and recorded; nothing here assumes an asset's size.

Only USD (pxr) here, plus ``app.update()`` while references load.
"""

from __future__ import annotations

import math

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/"
_WH = "Environments/Simple_Warehouse/Props/"
PROPS = {
    "floor": _WH + "SM_floor02.usd",
    "wall": _WH + "SM_WallA_6M.usd",
    "rack_frame": _WH + "SM_RackFrame_03.usd",  # one upright pair, 1 m deep, 3 m high
    "rack_shelf": _WH + "SM_RackShelf_01.usd",  # one 4 m beam level with its deck
    "rack_pile": _WH + "SM_RackPile_04.usd",
    "rack_pile_b": _WH + "SM_RackPile_06.usd",
    "column": _WH + "SM_PillarA_9M.usd",
    "machine": "Props/Conveyors/ConveyorBelt_A05.usd",  # a straight 2 m module
    "pallet": _WH + "SM_PaletteA_01.usd",
    "load": _WH + "SM_CardBoxA_02.usd",
    "crate": _WH + "SM_CratePlastic_B_01.usd",
    "barrel": _WH + "SM_BarelPlastic_A_01.usd",
    "box": _WH + "SM_CardBoxC_01.usd",
    "wet_sign": _WH + "S_WetFloorSign.usd",
    "cone": _WH + "S_TrafficCone.usd",
    "decal_hold": _WH + "SM_FloorDecal_QuadRed2x1.usd",
    "decal_dock": _WH + "SM_FloorDecal_RecRed1X1.usd",
}
PEOPLE = tuple(
    f"People/Characters/{name}/{name}.usd"
    for name in ("male_adult_construction_01_new", "male_adult_construction_03", "male_adult_construction_05_new")
)
WALL_HEIGHT_M = 3.0
COLUMN_HEIGHT_M = 5.0
RACK_HEIGHT_M = 3.2
RACK_LEVELS_M = (1.15, 2.3)  # shelf tops; floor level stays clear like the collider box
RACK_BAY_M = 4.0
MACHINE_GAP_M = 0.5
MACHINE_HEIGHT_M = 1.0
DECAL_LIFT_M = 0.003
# The warehouse materials are darker than world_scene's plain colours: in a
# 400 / 1200 / 2500 comparison (2026-10-09) 1200 kept the floor readable
# under the overlays without washing the racks out.
DOME_INTENSITY = 1200.0


def _bounds(prim) -> tuple[np.ndarray, np.ndarray]:
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    )
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return np.array(box.GetMin(), float), np.array(box.GetMax(), float)


def measure(stage, app, names, root: str = ROOT) -> dict:
    """{relative name: (low, high, has_physics)} read from the referenced assets."""
    out, prims = {}, {}
    for i, name in enumerate(names):
        prim = stage.DefinePrim(f"/World/DressCatalogue/A{i}", "Xform")
        prim.GetReferences().AddReference(root + name)
        prims[name] = prim
    for _ in range(30):
        app.update()
    for name, prim in prims.items():
        low, high = _bounds(prim)
        physics = any(
            p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI)
            for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
        )
        if not np.isfinite([low, high]).all() or np.any((high - low)[:2] <= 0):  # decals are flat
            raise RuntimeError(f"prop has no usable bounds: {name}")
        out[name] = (low, high, physics)
        prim.SetActive(False)
    return out


class Dresser:
    """Places measured props into world boxes; every copy is visual only."""

    def __init__(self, stage, catalogue: dict, root: str = ROOT):
        self.stage, self.cat, self.root = stage, catalogue, root
        self.count = 0
        self.stripped = 0

    def place(self, parent: str, name: str, centre, size_xy, z0: float, height: float | None,
              yaw: float | None = None, scale: bool = True) -> str:
        """Fit ``name`` into an axis box: centre (x, y), size (sx, sy) in world axes.

        yaw None turns the asset so its long side runs along the box's long
        side (0 or 90 degrees). scale False keeps the asset's own size.
        """
        low, high, physics = self.cat[name]
        ext = high - low
        if yaw is None:
            yaw = 0.0 if (ext[0] >= ext[1]) == (size_xy[0] >= size_xy[1]) else math.pi / 2
        c, s = math.cos(yaw), math.sin(yaw)
        if scale:
            # Scale is applied in the asset frame, before the turn.
            along_x = abs(c) >= abs(s)
            lx, ly = (size_xy[0], size_xy[1]) if along_x else (size_xy[1], size_xy[0])
            sx, sy = lx / ext[0], ly / ext[1]
            if ext[2] < 1e-3:  # a flat decal keeps its own thickness
                sz = 1.0
            else:
                sz = (height / ext[2]) if height else min(sx, sy)
        else:
            sx = sy = sz = 1.0
        mid = (low + high) / 2
        ox, oy = mid[0] * sx, mid[1] * sy
        path = f"{parent}/P{self.count}"
        self.count += 1
        # Our transform on a wrapper; the asset keeps its own root transform
        # (measured with it) on the child that holds the reference.
        if not self.stage.GetPrimAtPath(parent).IsValid():
            UsdGeom.Xform.Define(self.stage, parent)
        xf = UsdGeom.Xform.Define(self.stage, path)
        prim = self.stage.DefinePrim(path + "/asset", "Xform")
        prim.GetReferences().AddReference(self.root + name)
        xf.AddTranslateOp().Set(
            Gf.Vec3d(centre[0] - (c * ox - s * oy), centre[1] - (s * ox + c * oy), z0 - low[2] * sz)
        )
        xf.AddOrientOp().Set(Gf.Quatf(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
        xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
        if physics:
            self._strip(prim)
        else:
            prim.SetInstanceable(True)
        return path

    def _strip(self, prim) -> None:
        """A prop that brings its own physics loses it (looks only)."""
        for _ in range(8):
            inst = [p for p in Usd.PrimRange(prim) if p.IsInstance()]
            if not inst:
                break
            for p in inst:
                p.SetInstanceable(False)
        for p in Usd.PrimRange(prim):
            for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI, UsdPhysics.MassAPI, UsdPhysics.ArticulationRootAPI):
                if p.HasAPI(api):
                    p.RemoveAPI(api)
        self.stripped += 1


def _hide(prim) -> None:
    UsdGeom.Imageable(prim).MakeInvisible()


def _tiles(lo: float, hi: float, unit: float) -> list[tuple[float, float]]:
    n = max(1, int(round((hi - lo) / unit)))
    step = (hi - lo) / n
    return [(lo + i * step, lo + (i + 1) * step) for i in range(n)]


def dress_world(stage, app, world, geometry, scene_info: dict, rng, root: str = ROOT) -> dict:
    """Hide world_scene's boxes and put measured NVIDIA props in their place."""
    from pxr import UsdLux

    UsdLux.DomeLight(stage.GetPrimAtPath("/World/Factory/Sky")).GetIntensityAttr().Set(DOME_INTENSITY)
    names = sorted(set(PROPS.values()) | set(PEOPLE))
    cat = measure(stage, app, names, root)
    d = Dresser(stage, cat, root)
    P = PROPS
    base = "/World/Dressing"
    stage.DefinePrim(base, "Xform")
    # Floor: tiles of the warehouse floor over the whole hall bounds.
    b = world.bounds
    low, high, _ = cat[P["floor"]]
    tile = max(float(max(high[0] - low[0], high[1] - low[1])), 4.0)  # small tiles stretched: fewer prims
    for x0, x1 in _tiles(b.x_min_m - 0.4, b.x_max_m + 0.4, tile):
        for y0, y1 in _tiles(b.y_min_m - 0.4, b.y_max_m + 0.4, tile):
            d.place(base + "/Floor", P["floor"], ((x0 + x1) / 2, (y0 + y1) / 2), (x1 - x0, y1 - y0), 0.0, None, yaw=0.0)
    # The dark ground box stays under the tiles: it shows beyond the hall.
    statics = stage.GetPrimAtPath("/World/Factory/Static")
    counts: dict[str, int] = {}
    for i, (kind, box) in enumerate(world.obstacles):
        prim = statics.GetChild(f"{kind}_{i}")
        cx, cy = (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2
        sx, sy = box.x1 - box.x0, box.y1 - box.y0
        along_x = sx >= sy
        length, depth = (sx, sy) if along_x else (sy, sx)
        lo_along = box.x0 if along_x else box.y0

        def at(a0, a1, c0=None, c1=None):
            c0 = (box.y0 if along_x else box.x0) if c0 is None else c0
            c1 = (box.y1 if along_x else box.x1) if c1 is None else c1
            if along_x:
                return ((a0 + a1) / 2, (c0 + c1) / 2), (a1 - a0, c1 - c0)
            return ((c0 + c1) / 2, (a0 + a1) / 2), (c1 - c0, a1 - a0)

        if kind == "wall":
            unit = float(np.max((cat[P["wall"]][1] - cat[P["wall"]][0])[:2]))
            for a0, a1 in _tiles(lo_along, lo_along + length, unit):
                centre, size = at(a0, a1)
                d.place(base + "/Walls", P["wall"], centre, size, 0.0, WALL_HEIGHT_M)
        elif kind == "rack":
            # Upright frames at every bay end, two shelf levels, cartons on them.
            fext = cat[P["rack_frame"]][1] - cat[P["rack_frame"]][0]
            sext = cat[P["rack_shelf"]][1] - cat[P["rack_shelf"]][0]
            rows = max(1, int(round(depth / float(fext[1]))))
            across_lo = box.y0 if along_x else box.x0
            row_w = depth / rows
            frame_yaw = 0.0 if along_x else math.pi / 2  # the frame's depth runs across the rack
            shelf_yaw = 0.0 if along_x else math.pi / 2  # the shelf's 4 m runs along it
            bays = _tiles(lo_along, lo_along + length, RACK_BAY_M)
            for r in range(rows):
                c0, c1 = across_lo + r * row_w, across_lo + (r + 1) * row_w
                ends = [bays[0][0]] + [b1 for _, b1 in bays]
                for e in ends:
                    a0, a1 = e - fext[0] / 2, e + fext[0] / 2
                    a0, a1 = max(a0, lo_along), min(a1, lo_along + length)
                    centre, size = at(a0, a1, c0, c1)
                    d.place(base + "/Racks", P["rack_frame"], centre, size, 0.0, RACK_HEIGHT_M, yaw=frame_yaw)
                for k, (a0, a1) in enumerate(bays):
                    centre, size = at(a0, a1, c0, c1)
                    for level in RACK_LEVELS_M:
                        d.place(base + "/Racks", P["rack_shelf"], centre, size, level - float(sext[2]), None, yaw=shelf_yaw)
                        pile = P["rack_pile_b"] if (k + r + i + int(level)) % 3 == 0 else P["rack_pile"]
                        pext = cat[pile][1] - cat[pile][0]
                        span = (a1 - a0) * 0.92
                        for b0, b1 in _tiles(-span / 2, span / 2, float(max(pext[0], pext[1]))):
                            mid_a = (a0 + a1) / 2
                            centre_p, size_p = at(mid_a + b0, mid_a + b1, c0 + 0.08 * row_w, c1 - 0.08 * row_w)
                            d.place(base + "/Racks", pile, centre_p, size_p, level, 0.8)
        elif kind == "column":
            d.place(base + "/Columns", P["column"], (cx, cy), (sx, sy), 0.0, COLUMN_HEIGHT_M)
        elif kind == "machine":
            # Lanes of straight conveyor modules along the machine's long side.
            ext = cat[P["machine"]][1] - cat[P["machine"]][0]
            lane, unit = float(min(ext[0], ext[1])), float(max(ext[0], ext[1]))
            lanes = max(1, int((depth + MACHINE_GAP_M) // (lane + MACHINE_GAP_M)))
            span = lanes * lane + (lanes - 1) * MACHINE_GAP_M
            across_lo = (box.y0 if along_x else box.x0) + (depth - span) / 2
            for r in range(lanes):
                c0 = across_lo + r * (lane + MACHINE_GAP_M)
                for a0, a1 in _tiles(lo_along, lo_along + length, unit):
                    centre, size = at(a0, a1, c0, c0 + lane)
                    d.place(base + "/Machines", P["machine"], centre, size, 0.0, MACHINE_HEIGHT_M)
        elif kind == "store":
            for k, (a0, a1) in enumerate(_tiles(lo_along, lo_along + length, 0.75)):
                centre, size = at(a0, a1)
                d.place(base + "/Store", P["crate"] if k % 2 else P["barrel"], centre, size, 0.0, 0.9)
        else:
            continue  # "outside" keeps its dark box
        _hide(prim)
        counts[kind] = counts.get(kind, 0) + 1
    # Standing pallets: a real pallet and a box stack inside each stand-in.
    pallet_depth, pallet_width = geometry.pallet_depth_m, geometry.pallet_width_m
    for slot in world.slots:
        root_path = scene_info["stands"][slot.name]
        stand = stage.GetPrimAtPath(root_path)
        for child in ("pallet", "load"):
            _hide(stand.GetChild(child))
        yaw = slot.site.yaw_rad
        along = abs(math.cos(yaw)) > 0.5
        size = (pallet_depth, pallet_width) if along else (pallet_width, pallet_depth)
        d.place(root_path + "/Look", P["pallet"], (0.0, 0.0), size, 0.0, 0.15)
        for z in (0.15, 0.65):  # two cartons, the stand-in's 1 m load
            d.place(root_path + "/Look", P["load"], (0.0, 0.0), (size[0] * 0.95, size[1] * 0.95), z, 0.5)
        # The place itself, marked on the floor.
        d.place(base + "/Marks", P["decal_dock"], (slot.site.x_m, slot.site.y_m), (size[0] + 0.3, size[1] + 0.3), DECAL_LIFT_M, 0.002)
    front, rear = geometry.unloaded_footprint.front_m, geometry.unloaded_footprint.rear_m
    half = geometry.unloaded_footprint.half_width_m
    for pose in world.holding:
        c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        mid = (front - rear) / 2
        centre = (pose.x_m + c * mid, pose.y_m + s * mid)
        along = abs(c) > 0.5
        size = (front + rear + 0.3, 2 * half + 0.3) if along else (2 * half + 0.3, front + rear + 0.3)
        d.place(base + "/Marks", P["decal_hold"], centre, size, DECAL_LIFT_M, 0.002)
    for _ in range(20):
        app.update()
    return {
        "props": d.count,
        "props_stripped_of_physics": d.stripped,
        "hidden_boxes": counts,
        "catalogue": {k: {"low": v[0].round(4).tolist(), "high": v[1].round(4).tolist(), "physics": v[2]} for k, v in cat.items()},
        "dresser": d,
    }


def dress_spill(dresser: Dresser, stage, path: str, box) -> None:
    """Spilled cartons and a wet-floor sign inside a spill's (hidden) box."""
    _hide(stage.GetPrimAtPath(path))
    cx, cy = (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2
    w, h = box.x1 - box.x0, box.y1 - box.y0
    for k, (dx, dy, yaw) in enumerate(((-0.25, -0.2, 0.4), (0.2, 0.15, -0.7), (0.05, -0.3, 1.2))):
        dresser.place(path + "_look", PROPS["box"], (cx + dx * w, cy + dy * h), (0.5, 0.5), 0.0, 0.25, yaw=yaw)
    dresser.place(path + "_look", PROPS["wet_sign"], (cx + 0.3 * w, cy + 0.3 * h), (0.3, 0.3), 0.0, None, yaw=0.0, scale=False)


class Workers:
    """One worker figure per walking person; their capsules stay the colliders
    and are hidden. A figure faces the way its person last moved."""

    def __init__(self, dresser: Dresser, stage, capsules: list[str], rng):
        self.stage, self.figures = stage, []
        for path in capsules:
            _hide(stage.GetPrimAtPath(path))
            name = PEOPLE[int(rng.integers(len(PEOPLE)))]
            low, high, _ = dresser.cat[name]
            fig = dresser.place("/World/PeopleLook", name, (0.0, 0.0), (0.5, 0.5), -20.0, None, yaw=0.0, scale=False)
            self.figures.append([fig, (low + high) / 2, low[2], None, 0.0])

    def move(self, k: int, x: float, y: float) -> None:
        fig, mid, floor, last, yaw = self.figures[k]
        if last is not None and math.hypot(x - last[0], y - last[1]) > 1e-3:
            # The figures stand in a T pose, arms along their x: turn the arms
            # across the way they walk.
            yaw = math.atan2(y - last[1], x - last[0]) - math.pi / 2
        c, s = math.cos(yaw), math.sin(yaw)
        ops = UsdGeom.Xformable(self.stage.GetPrimAtPath(fig)).GetOrderedXformOps()
        ops[0].Set(Gf.Vec3d(x - (c * mid[0] - s * mid[1]), y - (s * mid[0] + c * mid[1]), -floor))
        ops[1].Set(Gf.Quatf(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
        self.figures[k][3:] = [(x, y), yaw]
