"""Build a random factory floor (forklift_core.planning.world_gen) as an Isaac stage.

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W6). Everything static is
a coloured box with a collider: walls, racks, machines, columns, stands and
the solid fill outside the halls. A pallet standing at a slot is a static
stand-in (pallet plus load) that is moved under the floor while the slot is
empty; the pallet a truck handles is a real pallet (scene.create_pallet).
People are kinematic capsules. Only USD (pxr) here; the runner owns physics.
"""

from __future__ import annotations

import math

from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

HEIGHTS = {
    "wall": 3.0,
    "outside": 3.0,
    "rack": 3.2,
    "machine": 1.8,
    "column": 5.0,
    "store": 1.0,
    "spill": 0.4,
    "broken": 0.0,
}
COLOURS = {
    "wall": (0.36, 0.38, 0.42),
    "outside": (0.16, 0.17, 0.2),
    "rack": (0.86, 0.52, 0.12),
    "machine": (0.24, 0.45, 0.78),
    "column": (0.62, 0.64, 0.68),
    "store": (0.66, 0.56, 0.38),
    "spill": (0.9, 0.85, 0.1),
    "floor": (0.2, 0.22, 0.25),
    "pallet": (0.72, 0.55, 0.33),
    "load": (0.83, 0.71, 0.5),
    "person": (0.95, 0.3, 0.45),
}
HIDDEN_Z_M = -20.0
PERSON_HEIGHT_M = 1.7


def _box(stage, path, centre, size, colour, collide=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def build_world(stage, world, geometry, *, ground_margin_m: float = 25.0) -> dict:
    """Ground, lights, static boxes and slot stand-ins; returns prim paths."""
    if stage.GetPrimAtPath("/World/Environment").IsValid():
        stage.RemovePrim("/World/Environment")
    b = world.bounds
    cx, cy = (b.x_min_m + b.x_max_m) / 2, (b.y_min_m + b.y_max_m) / 2
    w = b.x_max_m - b.x_min_m + 2 * ground_margin_m
    h = b.y_max_m - b.y_min_m + 2 * ground_margin_m
    # The floor collides as a plane: on a 100 m box collider the parked trucks
    # rocked by up to 0.27 rad (smoke run, 2026-10-09); the box stays as the look.
    _box(stage, "/World/Factory/Ground", (cx, cy, -0.051), (w, h, 0.1), COLOURS["floor"], collide=False)
    plane = UsdGeom.Plane.Define(stage, "/World/Factory/GroundCollider")
    plane.CreateAxisAttr("Z")
    plane.CreateWidthAttr(w)
    plane.CreateLengthAttr(h)
    UsdGeom.Xformable(plane).AddTranslateOp().Set(Gf.Vec3d(cx, cy, 0.0))
    plane.MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    # Dome light only: in a four-setting render comparison (2026-10-09) every
    # setting with a distant light left glare spots in the top view, and 700
    # dome + 2500 sun washed the colours out.
    dome = UsdLux.DomeLight.Define(stage, "/World/Factory/Sky")
    dome.CreateIntensityAttr(400.0)
    for i, (kind, box) in enumerate(world.obstacles):
        height = HEIGHTS.get(kind, 2.0)
        _box(
            stage,
            f"/World/Factory/Static/{kind}_{i}",
            ((box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2, height / 2),
            (box.x1 - box.x0, box.y1 - box.y0, height),
            COLOURS.get(kind, (0.5, 0.5, 0.5)),
        )
    stands = {}
    for slot in world.slots:
        along = abs(math.cos(slot.site.yaw_rad)) > 0.5
        dx, dy = (
            (geometry.pallet_depth_m, geometry.pallet_width_m)
            if along
            else (geometry.pallet_width_m, geometry.pallet_depth_m)
        )
        root = f"/World/Factory/Slots/{slot.name}"
        xf = UsdGeom.Xform.Define(stage, root)
        xf.AddTranslateOp().Set(Gf.Vec3d(slot.site.x_m, slot.site.y_m, HIDDEN_Z_M))
        _box(stage, f"{root}/pallet", (0, 0, 0.075), (dx, dy, 0.15), COLOURS["pallet"])
        _box(stage, f"{root}/load", (0, 0, 0.65), (dx * 0.95, dy * 0.95, 1.0), COLOURS["load"])
        stands[slot.name] = root
    return {"stands": stands}


def set_stand(stage, path: str, x: float, y: float, present: bool) -> None:
    """Show a standing pallet at its slot, or put it away under the floor."""
    op = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()[0]
    op.Set(Gf.Vec3d(x, y, 0.0 if present else HIDDEN_Z_M))


def add_box(stage, path: str, kind: str, box) -> None:
    """A static box that appears during the run (a spill)."""
    height = HEIGHTS.get(kind, 0.5)
    _box(
        stage,
        path,
        ((box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2, height / 2),
        (box.x1 - box.x0, box.y1 - box.y0, height),
        COLOURS.get(kind, (0.9, 0.9, 0.1)),
    )


def add_people(stage, count: int, radius_m: float) -> list[str]:
    """Kinematic capsules; the runner moves them by their translate op."""
    paths = []
    for k in range(count):
        path = f"/World/People/P_{k}"
        capsule = UsdGeom.Capsule.Define(stage, path)
        capsule.CreateRadiusAttr(radius_m)
        capsule.CreateHeightAttr(PERSON_HEIGHT_M - 2 * radius_m)
        capsule.CreateAxisAttr("Z")
        capsule.CreateDisplayColorAttr([Gf.Vec3f(*COLOURS["person"])])
        xf = UsdGeom.Xformable(capsule)
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, HIDDEN_Z_M))
        prim = capsule.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(True)
        paths.append(path)
    return paths


def move_person(stage, path: str, x: float, y: float) -> None:
    op = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()[0]
    op.Set(Gf.Vec3d(x, y, PERSON_HEIGHT_M / 2))
