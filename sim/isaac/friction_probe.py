#!/usr/bin/env python3
"""T0 of the contact materials plan (docs/plans/2026-11-01-contact-materials.md): the friction a contact pair really
gets in the operating scene, measured rather than read from USD -- the floor of the base scene carries no physics
material, and PhysX's default material is not on the stage to read.

Test boxes (0.20 x 0.20 x 0.04 m, 2 kg, contact offsets as the runner sets them) stand on the base scene's floor and
on fixed (kinematic) plates high above it, nothing else touching them. The truck is deactivated so the floor around
its spawn is free.

  static   gravity tilts about the y axis at --tilt-rate-deg-s from --tilt-start-deg; a box has slipped once it has
           moved --slip-m along x relative to its support since the start: mu_s = tan(angle at that tick)
  dynamic  gravity back to vertical, every box re-placed at rest, then given --slide-speed along +x: mu_d = a / g from
           a straight-line fit of its speed while it is above a fifth of the start speed

Boxes on the floor: the settings' contact material (as configure_drives binds it), no material at all, the same values
under combine "min" and under "multiply" (the last two read the floor's own material from the pair), and with
--split the step-A copies. A box of the contact material on a plate of the contact material is the calibration pair:
its expected value is the material's own.

  /isaac/python.sh sim/isaac/friction_probe.py --base-scene scene.usda --settings config/isaac_transport_measured.yaml \
      --output OUT [--split]

Writes OUT/friction_probe.json; touches nothing in the repository.
"""

import argparse
import json
import math
import sys
import traceback
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--base-scene", required=True)
ap.add_argument("--settings", type=Path, required=True)
ap.add_argument("--output", type=Path, required=True)
ap.add_argument("--split", action="store_true", help="also measure the step-A copies (contact_materials)")
ap.add_argument("--tilt-start-deg", type=float, default=15.0)
ap.add_argument("--tilt-end-deg", type=float, default=50.0)
ap.add_argument("--tilt-rate-deg-s", type=float, default=0.5)
ap.add_argument("--slip-m", type=float, default=0.002)
ap.add_argument("--slide-speed", type=float, default=1.0,
                help="at most 1.5 m/s: the plate pairs must stop on their plate (Codex A review P2-4)")
args = ap.parse_args()
if not 0.2 <= args.slide_speed <= 1.5:
    ap.error("--slide-speed must be in [0.2, 1.5] m/s")
args.output.mkdir(parents=True, exist_ok=False)

from isaacsim import SimulationApp  # noqa: E402 -- before any other Isaac import

app = SimulationApp({"headless": True, "multi_gpu": False})
G = 9.81
DT = 1 / 120
BOX = (0.20, 0.20, 0.04)
PLATE_HALF_X = 0.3
# a box on a plate counts while it is still on the plate (its centre at most this far from where it started)
ON_PLATE_M = PLATE_HALF_X - BOX[0] / 2
record = {"settings": str(args.settings), "base_scene": args.base_scene, "boxes": [], "errors": [],
          "conditions": {k: v for k, v in vars(args).items() if k != "output"} | {"box_m": BOX, "box_kg": 2.0,
                                                                                 "dt_s": DT, "g": G}}


def main():
    import hashlib

    import omni.usd
    import yaml
    from isaacsim.core.api import World
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import contact_materials

    settings = yaml.safe_load(args.settings.read_text())
    # the probe builds the contact material from the two friction values alone: a settings file whose operating
    # materials differ (a diagnostic friction or compliance) would be measured as something it is not (Codex A P2-2)
    diagnostic = [k for k in ("pallet_contact_friction", "contact_stiffness_n_per_m", "tyre_contact_stiffness_n_per_m")
                  if k in settings]
    if diagnostic:
        raise ValueError(f"{', '.join(diagnostic)}: the probe measures the plain contact material only")
    here = Path(__file__).resolve().parent
    record["sources_sha256"] = {name: hashlib.sha256((here / name).read_bytes()).hexdigest()
                                for name in ("friction_probe.py", "contact_materials.py")}
    record["settings_sha256"] = hashlib.sha256(args.settings.read_bytes()).hexdigest()
    record["base_scene_sha256"] = hashlib.sha256(Path(args.base_scene).read_bytes()).hexdigest()
    record["python"] = sys.executable
    if not omni.usd.get_context().open_stage(args.base_scene):
        raise RuntimeError("Cannot open base scene")
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)
    truck = stage.GetPrimAtPath("/World/Forklift")
    if truck:
        truck.SetActive(False)

    def material(path, mu_s, mu_d, combine=None):
        m = UsdShade.Material.Define(stage, path)
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr(float(mu_s))
        api.CreateDynamicFrictionAttr(float(mu_d))
        api.CreateRestitutionAttr(0.0)
        if combine is not None:
            PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim()).CreateFrictionCombineModeAttr(combine)
        return m

    mu_s, mu_d = settings["static_friction"], settings["dynamic_friction"]
    # the contact material exactly as scene.configure_drives authors it (values, restitution, no combine rule)
    contact = material("/World/ContactMaterial", mu_s, mu_d)
    mats = {"contact": contact, "none": None,
            "contact_min": material("/World/ProbeMin", mu_s, mu_d, "min"),
            "contact_multiply": material("/World/ProbeMultiply", mu_s, mu_d, "multiply")}
    if args.split:
        # step A's copies, made from /World/ContactMaterial as the runner makes them
        for role in contact_materials.ROLES:
            mats[f"split_{role}"] = UsdShade.Material(contact_materials._copy_material(
                stage, contact, f"{contact_materials.ROOT}/{role.capitalize()}").GetPrim())

    def collider(prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        c.CreateContactOffsetAttr(0.002)
        c.CreateRestOffsetAttr(0.0)

    def bind(prim, m):
        if m is not None:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(m, UsdShade.Tokens.strongerThanDescendants, "physics")

    def box(path, xyz, m):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        xf.AddScaleOp().Set(Gf.Vec3f(*BOX))
        prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(2.0)
        collider(prim)
        bind(prim, m)
        return prim

    def plate(path, xyz, m):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        xf.AddScaleOp().Set(Gf.Vec3f(2 * PLATE_HALF_X, 0.4, 0.02))
        prim = cube.GetPrim()
        body = UsdPhysics.RigidBodyAPI.Apply(prim)
        body.CreateKinematicEnabledAttr(True)
        collider(prim)
        bind(prim, m)
        return prim

    # floor boxes along y around the truck's spawn (x 0); plate pairs 1.5 m up, each on its own plate
    cases = [(f"floor_{name}", name, None) for name in mats]
    cases.append(("plate_contact_on_contact", "contact", "contact"))
    if args.split:
        cases += [("plate_split_fork_on_split_pallet", "split_fork", "split_pallet"),
                  ("plate_split_tyre_on_split_chassis", "split_tyre", "split_chassis")]
    floor_z = 0.0
    boxes = []
    for i, (label, box_mat, plate_mat) in enumerate(cases):
        y = -1.5 + 0.35 * i
        if plate_mat is None:
            z = floor_z + BOX[2] / 2 + 0.001
            support = None
        else:
            z = 1.5
            support = plate(f"/World/Probe/Plate_{i}", (0.0, y, z), mats[plate_mat])
            z += 0.01 + BOX[2] / 2 + 0.001
        boxes.append({"label": label, "box_material": box_mat, "support_material": plate_mat or "floor",
                      "prim": box(f"/World/Probe/Box_{i}", (0.0, y, z), mats[box_mat]), "start": (0.0, y, z),
                      "support": support})
    inventory = contact_materials.material_inventory(stage)
    record["inventory_outside_probe"] = {
        "colliders": {m: {g: n for g, n in groups.items() if not g.startswith("/World/Probe")}
                      for m, groups in inventory["colliders"].items()},
        "materials": inventory["materials"]}
    # the probe's region must be free: any other collider whose bounds reach into it is recorded (and the run is then
    # not to be read)
    lo, hi = Gf.Vec3d(-0.4, -1.8, -0.05), Gf.Vec3d(0.6, -1.5 + 0.35 * len(cases) + 0.2, 1.8)
    # collision geometry is often authored with the guide or proxy purpose (Codex A P2-1): every purpose counts
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide,
                                  UsdGeom.Tokens.render], useExtentsHint=True)
    intruders = []
    for prim in contact_materials.colliders(stage):
        path = str(prim.GetPath())
        if path.startswith("/World/Probe"):
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty() or "GroundPlane" in path:
            continue
        mn, mx = r.GetMin(), r.GetMax()
        if all(mn[k] < hi[k] and mx[k] > lo[k] for k in range(3)):
            intruders.append(path)
    record["region_intruders"] = intruders
    scene_prim = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
    record["physics_scene"] = str(scene_prim.GetPath()) if scene_prim else None
    record["physics_scene_material"] = (contact_materials._bound_path(scene_prim) if scene_prim else None)
    scene_api = UsdPhysics.Scene(scene_prim)

    from isaacsim.core.prims import SingleRigidPrim

    world.reset()
    handles = [SingleRigidPrim(str(b["prim"].GetPath())) for b in boxes]
    for _ in range(60):
        world.step(render=False)

    def positions():
        return np.array([h.get_world_pose()[0] for h in handles])

    # each box settled where it was put (on its own support, not on something else)
    record["settled_offsets_m"] = {b["label"]: [round(float(v), 4) for v in p - np.array(b["start"])]
                                   for b, p in zip(boxes, positions(), strict=True)}

    # static: tilt gravity about y (pull along +x)
    start = positions()
    slipped = [None] * len(boxes)
    angle = args.tilt_start_deg
    while angle <= args.tilt_end_deg and any(s is None for s in slipped):
        a = math.radians(angle)
        scene_api.CreateGravityDirectionAttr().Set(Gf.Vec3f(math.sin(a), 0.0, -math.cos(a)))
        world.step(render=False)
        now = positions()
        for i in range(len(boxes)):
            if slipped[i] is None and (now[i][0] - start[i][0]) > args.slip_m:
                slipped[i] = angle
        angle += args.tilt_rate_deg_s * DT
    # dynamic: vertical gravity, every box back at rest at its start, then a push along +x
    scene_api.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    for h, b in zip(handles, boxes, strict=True):
        h.set_world_pose(np.array(b["start"]), np.array([1.0, 0.0, 0.0, 0.0]))
        h.set_linear_velocity(np.zeros(3))
        h.set_angular_velocity(np.zeros(3))
    for _ in range(60):
        world.step(render=False)
    for h in handles:
        h.set_linear_velocity(np.array([args.slide_speed, 0.0, 0.0]))
    speeds = [[] for _ in boxes]
    starts = positions()
    for k in range(240):
        world.step(render=False)
        now = positions()
        for i, h in enumerate(handles):
            speeds[i].append((k * DT, float(h.get_linear_velocity()[0]), float(now[i][0] - starts[i][0])))
    for i, b in enumerate(boxes):
        on_floor = b["support_material"] == "floor"
        # the fit keeps a plate box's samples while it is still on its plate, and every box's while it slides
        pts = np.array([(t, v) for t, v, dx in speeds[i]
                        if v > args.slide_speed / 5 and (on_floor or dx < ON_PLATE_M)])
        mu_dyn = None
        if len(pts) >= 6:
            slope = np.polyfit(pts[1:, 0], pts[1:, 1], 1)[0]
            mu_dyn = -slope / G
        mu_st = None if slipped[i] is None else math.tan(math.radians(slipped[i]))
        record["boxes"].append({"label": b["label"], "box_material": b["box_material"],
                                "support": b["support_material"], "slip_angle_deg": slipped[i],
                                "mu_static": mu_st, "mu_dynamic": mu_dyn, "fit_points": int(len(pts)),
                                "slide_series": [[round(t, 4), round(v, 5), round(dx, 5)] for t, v, dx in speeds[i]]})
    record["materials"] = {k: (None if m is None else contact_materials.material_record(m.GetPrim()))
                           for k, m in mats.items()}


try:
    main()
    record["ok"] = True
except Exception:  # noqa: BLE001 -- recorded
    record["ok"] = False
    record["errors"].append(traceback.format_exc())
finally:
    (args.output / "friction_probe.json").write_text(json.dumps(record, indent=1, default=str))
    print(json.dumps({"ok": record.get("ok"), "boxes": record["boxes"]}, indent=1, default=str))
    app.close()
