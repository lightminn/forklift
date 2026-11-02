#!/usr/bin/env python3
"""B0 of the contact materials plan (docs/plans/2026-11-01-contact-materials.md section 5 B): the effective friction a
real pallet gets on the two fork blades, one arm per fork material -- measured, so that the B arms are defined by the
friction they really give (lead 2026-11-02), not by the values a settings file names.

The base scene is opened for its physics scene alone (the warehouse and the truck are deactivated). Each arm is a pair
of kinematic blades (the measured chassis' 0.055 x 0.025 m section, the run's fork spacing, 0.33 m under the pallet as
a full insertion leaves them) carrying a free pallet imported as the runner imports it (its boxes, synthetic
uniform-volume inertia, the settings' mass for its shape), its stringers resting on the blades. The pallet carries the
contact material as configure_drives binds it; the blades carry the arm's fork material. Arms stand 2 m apart in the
air and touch nothing else.

  step     gravity tilted at once about y (a pull along the blades) to each --step-angle-deg, held --hold-s: stuck or
           slid (the pallet's travel along the blades); the pallet put back between angles
  ramp     gravity tilted from --ramp-start-deg at --ramp-rate-deg-s until the pallet has moved --slip-m along the
           blades: mu_s(onset) = tan(angle)
  slide    gravity vertical, the pallet given --slide-speed along +x: mu_d = a / g from the speed's straight-line fit
           while it is above a fifth of the start speed and still on the blades

  python sim/isaac/fork_friction_probe.py --base-scene scene.usda --settings config/isaac_transport_measured.yaml \
      --pallet-urdf sim/models/epal6_pallet/pallet.urdf --pallet-geometry config/pallet_geometry_epal6.yaml \
      --fork-spacing-m 0.3725 --arms ARMS.json --output OUT

ARMS.json: [{"name": "ctl"}, {"name": "m05", "mu_s": 0.5714, "mu_d": 0.5, "combine": "min"}, ...] -- an arm without
mu_s / mu_d takes the contact material itself (today's single material). Writes OUT/fork_friction_probe.json.
"""

import argparse
import hashlib
import json
import math
import sys
import traceback
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--base-scene", required=True)
ap.add_argument("--settings", type=Path, required=True)
ap.add_argument("--pallet-urdf", type=Path, required=True)
ap.add_argument("--pallet-geometry", type=Path, required=True)
ap.add_argument("--fork-spacing-m", type=float, required=True)
ap.add_argument("--arms", type=Path, required=True)
ap.add_argument("--output", type=Path, required=True)
ap.add_argument("--step-angle-deg", type=float, nargs="+", default=[])
ap.add_argument("--hold-s", type=float, default=3.0)
ap.add_argument("--ramp-start-deg", type=float, default=5.0)
ap.add_argument("--ramp-end-deg", type=float, default=45.0)
ap.add_argument("--ramp-rate-deg-s", type=float, default=0.5)
ap.add_argument("--slip-m", type=float, default=0.002)
ap.add_argument("--slide-speed", type=float, default=0.5)
args = ap.parse_args()
if not 0.1 <= args.slide_speed <= 1.0:
    ap.error("--slide-speed must be in [0.1, 1.0] m/s: the pallet must stop on the blades")
args.output.mkdir(parents=True, exist_ok=False)

from isaacsim import SimulationApp  # noqa: E402 -- before any other Isaac import

app = SimulationApp({"headless": True, "multi_gpu": False})
G, DT = 9.81, 1 / 120
BLADE_W, BLADE_T, BLADE_L = 0.055, 0.025, 0.33  # dls08_measured fork section; the 0.330 m insertion target
Z0 = 1.5  # the pallet's floor plane, in the air
ARM_GAP_M = 2.0
record = {"conditions": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items() if k != "output"},
          "arms": [], "errors": []}


def main():
    import omni.usd
    import yaml
    from isaacsim.core.api import World
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    import contact_materials
    from pallet_mass import resolve_pallet_mass
    from scene import import_pallet_usd, place_pallet

    from forklift_core.perception.pallet_geometry import load_pallet_geometry
    from forklift_core.perception.pallet_model import (
        choose_pocket_pair,
        face_layout,
        spec_from_geometry,
    )

    settings = yaml.safe_load(args.settings.read_text())
    geometry = load_pallet_geometry(args.pallet_geometry)
    settings, mass = resolve_pallet_mass(settings, geometry.geometry_version)
    arms = json.loads(args.arms.read_text())
    record.update(
        sources_sha256={n: hashlib.sha256((here / n).read_bytes()).hexdigest()
                        for n in ("fork_friction_probe.py", "contact_materials.py", "pallet_mass.py", "scene.py")},
        settings_sha256=hashlib.sha256(args.settings.read_bytes()).hexdigest(),
        base_scene_sha256=hashlib.sha256(Path(args.base_scene).read_bytes()).hexdigest(),
        pallet_mass=mass, python=sys.executable, arms_requested=arms)

    # the pockets the blades sit in: the x face, the pair the runner's fork spacing takes
    spec = spec_from_geometry(geometry, entry_axes=("x",))
    layout = face_layout(spec, "x")
    pair = choose_pocket_pair(layout, args.fork_spacing_m, BLADE_W)
    centres = [sum(layout.opening_intervals_m[i]) / 2 for i in pair]
    blade_y = [sum(centres) / 2 - args.fork_spacing_m / 2, sum(centres) / 2 + args.fork_spacing_m / 2]
    ceiling = layout.lane_ceiling_m  # the stringers' underside: what the blades lift
    front_x = -geometry.overall_depth_m / 2
    record["geometry"] = {"pair": list(pair), "pocket_centres_y_m": centres, "blade_y_m": blade_y,
                          "lift_face_z_m": ceiling, "blade_x_m": [front_x, front_x + BLADE_L]}

    if not omni.usd.get_context().open_stage(args.base_scene):
        raise RuntimeError("Cannot open base scene")
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    for path in ("/World/Forklift", "/World/Environment"):
        if stage.GetPrimAtPath(path):
            stage.GetPrimAtPath(path).SetActive(False)
    world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)

    def material(path, mu_s, mu_d, combine=None):
        m = UsdShade.Material.Define(stage, path)
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr(float(mu_s))
        api.CreateDynamicFrictionAttr(float(mu_d))
        api.CreateRestitutionAttr(0.0)
        if combine is not None:
            PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim()).CreateFrictionCombineModeAttr(combine)
        return m

    def collider(prim, m):
        UsdPhysics.CollisionAPI.Apply(prim)
        c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        c.CreateContactOffsetAttr(0.002)  # as configure_drives sets every truck and pallet collider
        c.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(m, UsdShade.Tokens.strongerThanDescendants, "physics")

    contact = material("/World/ContactMaterial", settings["static_friction"], settings["dynamic_friction"])
    usd = import_pallet_usd(args.pallet_urdf, args.output, settings["pallet_mass_kg"])

    class Site:
        def __init__(self, x, y):
            self.x_m, self.y_m, self.yaw_rad = x, y, 0.0

    handles = []
    for k, arm in enumerate(arms):
        y0 = k * ARM_GAP_M
        if arm.get("mu_s") is None:
            fork = contact  # today's single material on the blades too
        else:
            fork = material(f"/World/ProbeFork_{k}", arm["mu_s"], arm["mu_d"], arm.get("combine"))
        for side, by in zip(("L", "R"), blade_y, strict=True):
            cube = UsdGeom.Cube.Define(stage, f"/World/ProbeArm_{k}/Blade{side}")
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube)
            xf.AddTranslateOp().Set(Gf.Vec3d(front_x + BLADE_L / 2, y0 + by, Z0 + ceiling - BLADE_T / 2))
            xf.AddScaleOp().Set(Gf.Vec3f(BLADE_L, BLADE_W, BLADE_T))
            prim = cube.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(True)
            collider(prim, fork)
        pallet = place_pallet(world, stage, app, usd, Site(0.0, y0), prim_path=f"/World/ProbePallet_{k}",
                              name=f"probe_pallet_{k}")
        for prim in [p for p in stage.Traverse() if str(p.GetPath()).startswith(f"/World/ProbePallet_{k}/")]:
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collider(prim, contact)
        handles.append((arm, pallet, y0))
    scene_prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    scene_api = UsdPhysics.Scene(scene_prim)
    world.reset()

    def rest_pose(y0):
        return np.array([0.0, y0, Z0 + 0.001]), np.array([1.0, 0.0, 0.0, 0.0])

    def put_back():
        for _, pallet, y0 in handles:
            p, q = rest_pose(y0)
            pallet.set_world_pose(p, q)
            pallet.set_linear_velocity(np.zeros(3))
            pallet.set_angular_velocity(np.zeros(3))

    def gravity(angle_deg):
        a = math.radians(angle_deg)
        scene_api.CreateGravityDirectionAttr().Set(Gf.Vec3f(math.sin(a), 0.0, -math.cos(a)))

    def xs():
        return np.array([pallet.get_world_pose()[0][0] for _, pallet, _ in handles])

    def settle(steps=120):
        for _ in range(steps):
            world.step(render=False)

    gravity(0.0)
    put_back()
    settle()
    settled = [pallet.get_world_pose()[0].tolist() for _, pallet, _ in handles]
    out = [{"name": arm["name"], "fork_material": None if arm.get("mu_s") is None else
            {"mu_s": arm["mu_s"], "mu_d": arm["mu_d"], "combine": arm.get("combine")},
            "settled_position_m": pos, "step": [], "ramp_onset_deg": None} for (arm, _, _), pos in
           zip(handles, settled, strict=True)]
    # step: a sudden tilt held; a pallet that travels more than --slip-m along the blades slid
    for angle in args.step_angle_deg:
        gravity(0.0)
        put_back()
        settle()
        start = xs()
        gravity(angle)
        settle(int(args.hold_s / DT))
        moved = xs() - start
        for o, m in zip(out, moved, strict=True):
            o["step"].append({"angle_deg": angle, "tan": math.tan(math.radians(angle)), "travel_m": float(m),
                              "slid": bool(m > args.slip_m)})
    # ramp
    gravity(0.0)
    put_back()
    settle()
    start = xs()
    angle = args.ramp_start_deg
    while angle <= args.ramp_end_deg and any(o["ramp_onset_deg"] is None for o in out):
        gravity(angle)
        world.step(render=False)
        for o, m in zip(out, xs() - start, strict=True):
            if o["ramp_onset_deg"] is None and m > args.slip_m:
                o["ramp_onset_deg"] = angle
        angle += args.ramp_rate_deg_s * DT
    # slide
    gravity(0.0)
    put_back()
    settle()
    for _, pallet, _ in handles:
        pallet.set_linear_velocity(np.array([args.slide_speed, 0.0, 0.0]))
    start = xs()
    series = [[] for _ in handles]
    for k in range(180):
        world.step(render=False)
        now = xs()
        for i, (_, pallet, _) in enumerate(handles):
            series[i].append((k * DT, float(pallet.get_linear_velocity()[0]), float(now[i] - start[i])))
    for o, s in zip(out, series, strict=True):
        pts = np.array([(t, v) for t, v, dx in s if v > args.slide_speed / 5 and dx < BLADE_L / 2])
        o["mu_dynamic"] = None if len(pts) < 6 else float(-np.polyfit(pts[1:, 0], pts[1:, 1], 1)[0] / G)
        o["mu_onset"] = None if o["ramp_onset_deg"] is None else math.tan(math.radians(o["ramp_onset_deg"]))
        o["slide_series"] = [[round(t, 4), round(v, 5), round(dx, 5)] for t, v, dx in s]
    record["arms"] = out
    record["contact_material"] = contact_materials.material_record(contact.GetPrim())


try:
    main()
    record["ok"] = True
except Exception:  # noqa: BLE001 -- recorded
    record["ok"] = False
    record["errors"].append(traceback.format_exc())
finally:
    (args.output / "fork_friction_probe.json").write_text(json.dumps(record, indent=1, default=str))
    print(json.dumps({"ok": record.get("ok"), "arms": [{k: a.get(k) for k in ("name", "mu_onset", "mu_dynamic", "step")}
                                                        for a in record["arms"]]}, indent=1, default=str))
    app.close()
