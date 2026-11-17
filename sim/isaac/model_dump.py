"""The steering probe's model dump (docs/plans/2026-11-17-model-symmetry-check.md 1): what Isaac actually uses for the
chassis, beside what the stage authors, so a left/right or model-to-URDF difference inside the engine can be found.

- engine_bodies(view): each articulation body's mass, centre of mass and inertia tensor as the physics view returns
  them; a query that fails is recorded as such (never replaced by the authored value);
- authored_masses(stage): each rigid body's UsdPhysics.MassAPI values (mass, centre of mass, diagonal inertia,
  principal axes) as authored;
- joints(stage): every revolute/continuous joint under the robot -- frames on both bodies, axis, limits, the angular
  drive (type, stiffness, damping, max force) and PhysxJointAPI armature, friction, max velocity;
- colliders(stage): every collision prim under the robot -- the shape (a cylinder's radius, height and axis, a box's
  size, a mesh's approximation), its world-axis-aligned bounds and lowest z, contact and rest offsets.

Reads only. No clock, no Isaac import here: the runner passes the stage and the view.
"""

from __future__ import annotations

import math

import numpy as np

ROOT = "/World/Forklift"


def _num(v):
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    return f if math.isfinite(f) else None


def _vec(v):
    return None if v is None else [float(x) for x in v]


def _quat(q):
    return None if q is None else [float(q.GetReal()), *map(float, q.GetImaginary())]


def _attr(prim, name):
    """A PhysX schema attribute by its name (no PhysxSchema import: the readers also run on a plain USD build)."""
    a = prim.GetAttribute(name)
    return _num(a.Get()) if a and a.HasAuthoredValue() else None


def engine_bodies(view) -> dict:
    """{"bodies": {name: {mass_kg, com_m, com_quat_wxyz, inertia}}, "errors": {...}} from the articulation view."""
    out, errors = {}, {}
    try:
        names = list(view.body_names)
    except Exception as exc:  # noqa: BLE001 -- recorded: this item stays undecided
        return {"bodies": {}, "errors": {"body_names": f"{type(exc).__name__}: {exc}"}}

    def read(label, fn):
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001
            errors[label] = f"{type(exc).__name__}: {exc}"
            return None

    masses = read("masses", lambda: np.asarray(view.get_body_masses(), float).reshape(-1))
    coms = read("coms", lambda: view.get_body_coms())
    inertias = read("inertias", lambda: np.asarray(view.get_body_inertias(), float).reshape(-1, 9))
    com_p = None if coms is None else np.asarray(coms[0], float).reshape(-1, 3)
    com_q = None if coms is None else np.asarray(coms[1], float).reshape(-1, 4)
    for i, n in enumerate(names):
        out[n] = {"mass_kg": None if masses is None else float(masses[i]),
                  "com_m": None if com_p is None else com_p[i].tolist(),
                  "com_quat_wxyz": None if com_q is None else com_q[i].tolist(),
                  "inertia": None if inertias is None else inertias[i].tolist()}
    return {"bodies": out, "errors": errors}


def authored_masses(stage, root: str = ROOT) -> dict:
    from pxr import Usd, UsdPhysics

    out = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.HasAPI(UsdPhysics.MassAPI):
            continue
        api = UsdPhysics.MassAPI(prim)
        pa = api.GetPrincipalAxesAttr().Get()
        out[prim.GetName()] = {
            "mass_kg": _num(api.GetMassAttr().Get()),
            "com_m": _vec(api.GetCenterOfMassAttr().Get()),
            "diagonal_inertia": _vec(api.GetDiagonalInertiaAttr().Get()),
            "principal_axes_wxyz": None if pa is None else [float(pa.GetReal()), *map(float, pa.GetImaginary())],
        }
    return out


def joints(stage, root: str = ROOT) -> dict:
    from pxr import Usd, UsdPhysics

    out = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        revolute, prismatic = prim.IsA(UsdPhysics.RevoluteJoint), prim.IsA(UsdPhysics.PrismaticJoint)
        j = UsdPhysics.RevoluteJoint(prim) if revolute else (UsdPhysics.PrismaticJoint(prim) if prismatic
                                                             else UsdPhysics.Joint(prim))
        drive = UsdPhysics.DriveAPI(prim, "angular" if revolute else "linear")

        def get(attr):
            return _num(attr.Get()) if attr else None

        out[prim.GetName()] = {
            "body0": [str(t) for t in j.GetBody0Rel().GetTargets()],
            "body1": [str(t) for t in j.GetBody1Rel().GetTargets()],
            "local_pos0": _vec(j.GetLocalPos0Attr().Get()), "local_pos1": _vec(j.GetLocalPos1Attr().Get()),
            "local_rot0": _quat(j.GetLocalRot0Attr().Get()), "local_rot1": _quat(j.GetLocalRot1Attr().Get()),
            "kind": "revolute" if revolute else ("prismatic" if prismatic else prim.GetTypeName()),
            "axis": str(j.GetAxisAttr().Get()) if (revolute or prismatic) else None,
            "lower_deg": get(j.GetLowerLimitAttr()) if revolute else None,
            "upper_deg": get(j.GetUpperLimitAttr()) if revolute else None,
            "drive": None if not prim.HasAPI(UsdPhysics.DriveAPI, "angular" if revolute else "linear") else {
                "type": str(drive.GetTypeAttr().Get()), "stiffness": get(drive.GetStiffnessAttr()),
                "damping": get(drive.GetDampingAttr()), "max_force": get(drive.GetMaxForceAttr())},
            "armature": _attr(prim, "physxJoint:armature"), "joint_friction": _attr(prim, "physxJoint:jointFriction"),
            "max_joint_velocity": _attr(prim, "physxJoint:maxJointVelocity"),
        }
    return out


def colliders(stage, root: str = ROOT) -> dict:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy,
                                                        UsdGeom.Tokens.guide], useExtentsHint=False)
    out = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        shape = {"type": prim.GetTypeName()}
        if prim.IsA(UsdGeom.Cylinder):
            c = UsdGeom.Cylinder(prim)
            shape.update(radius=_num(c.GetRadiusAttr().Get()), height=_num(c.GetHeightAttr().Get()),
                         axis=str(c.GetAxisAttr().Get()))
        elif prim.IsA(UsdGeom.Cube):
            shape.update(size=_num(UsdGeom.Cube(prim).GetSizeAttr().Get()))
        elif prim.IsA(UsdGeom.Mesh):
            mc = UsdPhysics.MeshCollisionAPI(prim)
            shape.update(approximation=str(mc.GetApproximationAttr().Get()) if prim.HasAPI(UsdPhysics.MeshCollisionAPI)
                         else None)
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lo, hi = (None, None) if box.IsEmpty() else (_vec(box.GetMin()), _vec(box.GetMax()))
        # the collider's centre in its rigid body's own frame: the zero-pose geometry, independent of the stand
        body = prim
        while body and body.GetPath() != body.GetStage().GetPseudoRoot().GetPath() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        in_body = in_lo = in_hi = body_scale = None
        if body and body.HasAPI(UsdPhysics.RigidBodyAPI):
            # the body's own world scale (the relative bound leaves it out; Codex review P2): 1 for an unscaled body
            m = UsdGeom.XformCache().GetLocalToWorldTransform(body)
            body_scale = [float(m.GetRow3(i).GetLength()) for i in range(3)]
            # the bounds in the body's frame carry the collider's own rotation and scale (Codex review P2)
            rel = cache.ComputeRelativeBound(prim, body).ComputeAlignedRange()
            if not rel.IsEmpty():
                in_lo, in_hi = _vec(rel.GetMin()), _vec(rel.GetMax())
                in_body = [(a + b) / 2 for a, b in zip(in_lo, in_hi, strict=True)]
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
        mat_prim = material.GetPrim() if material else None
        mat = None
        if mat_prim and mat_prim.IsValid():
            mapi = UsdPhysics.MaterialAPI(mat_prim)
            mat = {"path": str(mat_prim.GetPath()),
                   "static_friction": _num(mapi.GetStaticFrictionAttr().Get()) if mapi.GetStaticFrictionAttr() else None,
                   "dynamic_friction": _num(mapi.GetDynamicFrictionAttr().Get()) if mapi.GetDynamicFrictionAttr() else None,
                   "restitution": _num(mapi.GetRestitutionAttr().Get()) if mapi.GetRestitutionAttr() else None,
                   "friction_combine": (str(mat_prim.GetAttribute("physxMaterial:frictionCombineMode").Get())
                                        if mat_prim.GetAttribute("physxMaterial:frictionCombineMode") else None)}
        out[str(prim.GetPath())] = {
            **shape,
            "world_min_m": lo, "world_max_m": hi,
            "world_center_m": None if lo is None else [(a + b) / 2 for a, b in zip(lo, hi, strict=True)],
            "body": None if in_body is None else str(body.GetPath()), "in_body_center_m": in_body,
            "in_body_min_m": in_lo, "in_body_max_m": in_hi, "body_scale": body_scale, "material": mat,
            "contact_offset": _attr(prim, "physxCollision:contactOffset"),
            "rest_offset": _attr(prim, "physxCollision:restOffset"),
        }
    return out


def floor_colliders(stage, root: str = "/World") -> dict:
    """The floor's collider(s) (a path holding GroundPlane) with their bound material, as colliders() reads them."""
    from pxr import Usd, UsdPhysics

    out = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if "GroundPlane" in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI):
            out.update({k: v for k, v in colliders(stage, str(prim.GetPath())).items()})
    return out


def articulation(stage, root: str = ROOT) -> dict:
    """The articulation root's solver iterations as authored (PhysxArticulationAPI attributes by name)."""
    from pxr import Usd, UsdPhysics

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return {"root": str(prim.GetPath()),
                    "solver_position_iterations": _attr(prim, "physxArticulation:solverPositionIterationCount"),
                    "solver_velocity_iterations": _attr(prim, "physxArticulation:solverVelocityIterationCount")}
    return {"root": None}


def settled_summary(tyre_rows: list, tyre_names: tuple, dt: float, wheel_z: dict, roll_pitch: list) -> dict:
    """The stand's mean floor force per tyre (N, the floor's push: the negative of the reported impulse / dt, as
    tyre_floor_forces' docstring says), mean wheel-centre z (m) and mean base roll/pitch (rad)."""
    rows = np.asarray(tyre_rows, float)
    fz = None if rows.size == 0 else (-np.nanmean(rows[:, :, 2], axis=0) / dt).tolist()
    rp = np.asarray(roll_pitch, float)
    return {"tyres": list(tyre_names), "floor_fz_n": fz,
            "wheel_center_z_m": {k: float(np.mean(v)) for k, v in wheel_z.items()},
            "base_roll_rad": None if rp.size == 0 else float(np.mean(rp[:, 0])),
            "base_pitch_rad": None if rp.size == 0 else float(np.mean(rp[:, 1])),
            "samples": int(rows.shape[0]) if rows.size else 0}
