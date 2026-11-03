"""The --steering-probe arms of the steering left-asymmetry plan (docs/plans/2026-11-03-steering-left-asymmetry.md,
stage P, sections 4.1-4.2). Every arm is a settings key that only a --steering-probe run accepts (the runner refuses
them otherwise, as DIAGNOSTIC_PROBE_SETTINGS); with none of them set the probe runs as before.

  A3  probe_fix_base_z_m              base_link held by a fixed joint to the world at the spawn x, y, yaw and this z
  A4  probe_load_kg, probe_load_dx_m, probe_load_dz_m, probe_lift_m
                                      a pallet-equivalent load merged into fork_carriage's mass properties (the scene
                                      pallet's inertia scaled to the mass), its origin probe_load_dx/dz ahead of / above
                                      base_link at the actual lift probe_lift_m; the lift target is that lift plus the
                                      drive's gravity sag with the load
  A6  probe_front_tyre_frictionless   the two front tyre colliders get a mu 0 material combined by "min"
  A7  probe_shim_wheel, probe_shim_height_m
                                      a static plate of this height under the front-left (1) or front-right (2) wheel
  A9  probe_spawn_yaw_offset_rad      the truck spawned with this yaw added (same place)
  A10 probe_assign_by_y               commands and records assigned to joints by their physical side (y sign in the
                                      base frame) instead of their names (a name-swapped URDF; the drive geometry then
                                      comes from --probe-geometry-urdf)
  A11 probe_front_spin_free           the two front spin drives without gains or effort (rear-wheel drive)
  all probe_clear_area                the random props deactivated before the reset and the scene pallet made
                                      kinematic and lifted out of the way after the spawn checks, so the probe's
                                      arcs touch nothing but the floor (the smoke runs' fast arcs reached a prop)

Values are positive numbers, as every settings value must be: a flag is 1 (set) or absent; the base height and the
yaw offset are positive (A3's 'touching' height is the smallest positive value next to the settled height).
"""

import math

import numpy as np

PROBE_ARM_SETTINGS = (
    "probe_fix_base_z_m",
    "probe_load_kg", "probe_load_dx_m", "probe_load_dz_m", "probe_lift_m",
    "probe_front_tyre_frictionless",
    "probe_shim_wheel", "probe_shim_height_m",
    "probe_spawn_yaw_offset_rad",
    "probe_assign_by_y",
    "probe_front_spin_free",
    "probe_clear_area",
)
LOAD_KEYS = ("probe_load_kg", "probe_load_dx_m", "probe_load_dz_m", "probe_lift_m")
SHIM_KEYS = ("probe_shim_wheel", "probe_shim_height_m")
FLAG_KEYS = ("probe_front_tyre_frictionless", "probe_assign_by_y", "probe_front_spin_free", "probe_clear_area")
LIFT_SETTLE_S = 1.5  # plan 4.2 A4: the load and the lift settle before the segments
FRONT_TYRES = ("front_left_tire_collision", "front_right_tire_collision")
SHIM_PATH = "/World/ProbeShim"
BASE_FIX_PATH = "/World/ProbeBaseFix"
FRICTIONLESS_PATH = "/World/ProbeFrontTyreMaterial"
SHIM_SIZE_FACTOR = 3.0  # plan 4.2 A7: the plate is three times the tyre's width and its contact length


def arm_problems(settings: dict) -> list[str]:
    """What is wrong with the probe arm keys a settings file sets (an empty list when none or all is well)."""
    out = []
    present = [k for k in LOAD_KEYS if k in settings]
    if present and len(present) != len(LOAD_KEYS):
        out.append(f"probe load needs all of {', '.join(LOAD_KEYS)}")
    if "probe_load_kg" in settings and not settings["probe_load_kg"] > 0:
        out.append("probe_load_kg must be positive")
    if "probe_lift_m" in settings and not 0 < settings["probe_lift_m"] < 0.28:
        out.append("probe_lift_m must be inside the lift travel (0, 0.28) m")
    present = [k for k in SHIM_KEYS if k in settings]
    if present and len(present) != len(SHIM_KEYS):
        out.append(f"probe shim needs both of {', '.join(SHIM_KEYS)}")
    if "probe_shim_wheel" in settings and settings["probe_shim_wheel"] not in (1, 2):
        out.append("probe_shim_wheel is 1 (front left) or 2 (front right)")
    if "probe_shim_height_m" in settings and not 0 < settings["probe_shim_height_m"] <= 0.01:
        out.append("probe_shim_height_m must be in (0, 0.01] m")
    for key in FLAG_KEYS:
        if key in settings and settings[key] != 1:
            out.append(f"{key} is 1 or absent")
    if "probe_fix_base_z_m" in settings and not 0 < settings["probe_fix_base_z_m"] <= 0.2:
        out.append("probe_fix_base_z_m must be in (0, 0.2] m")
    if "probe_spawn_yaw_offset_rad" in settings and not 0 < settings["probe_spawn_yaw_offset_rad"] <= math.pi:
        out.append("probe_spawn_yaw_offset_rad must be in (0, pi]")
    return out


def yaw_quat_wxyz(yaw: float) -> tuple:
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def quat_matrix(q_wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in q_wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def physical_assignment(body_positions: dict) -> dict:
    """Plan 4.2 A10: the names standing at each physical place, from the wheel links' positions in the base frame
    (x forward, y > 0 the truck's left). ``body_positions``: body name -> [x, y, z] in the base frame for the four
    ``*_wheel`` links. The model's naming ties a wheel ``<p>_wheel`` to its spin joint ``<p>_spin`` and a front wheel
    ``front_<s>_wheel`` to its steering joint ``<s>_steer``, so a name swap moves all three together. Returns
    {"steer": [left, right], "spin": [FL, FR, RL, RR], "tyre_links": [FL, FR, RL, RR]} as names; raises unless there
    is one wheel per corner."""
    wheels = {n: [float(v) for v in p] for n, p in body_positions.items() if n.endswith("_wheel")}
    if len(wheels) != 4:
        raise ValueError(f"expected four wheel links, got {sorted(wheels)}")
    xs = sorted(p[0] for p in wheels.values())
    mid = (xs[1] + xs[2]) / 2
    order = []
    for front in (True, False):
        axle = sorted(((p[1], n) for n, p in wheels.items() if (p[0] > mid) == front), reverse=True)
        if len(axle) != 2 or not (axle[0][0] > 0 > axle[1][0]):
            raise ValueError(f"{'front' if front else 'rear'} wheels are not one per side: {axle}")
        order += [axle[0][1], axle[1][1]]
    if not all(n.startswith("front_") for n in order[:2]):
        raise ValueError(f"the front wheels are not the front_* links: {order[:2]}")
    stem = [n[: -len("_wheel")] for n in order]
    return {"steer": [stem[0][len("front_"):] + "_steer", stem[1][len("front_"):] + "_steer"],
            "spin": [s_ + "_spin" for s_ in stem], "tyre_links": order}


def composite_body(m1: float, c1, inertia1, m2: float, c2, inertia2):
    """Two rigid masses as one: total mass, centre of mass and the 3 x 3 inertia about that centre (each inertia
    given about its own centre, all in one frame) -- the parallel-axis theorem."""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    i1, i2 = np.asarray(inertia1, float).reshape(3, 3), np.asarray(inertia2, float).reshape(3, 3)
    m = m1 + m2
    c = (m1 * c1 + m2 * c2) / m

    def shift(mass, r):
        r = np.asarray(r, float)
        return mass * (float(r @ r) * np.eye(3) - np.outer(r, r))

    return m, c, i1 + shift(m1, c1 - c) + i2 + shift(m2, c2 - c)


def load_in_carriage(base_to_carriage, load_origin_in_base, load_com_in_load, load_rot_in_base=np.eye(3)):
    """The load's centre in the carriage frame. ``base_to_carriage``: (R, t) of the carriage frame in the base frame
    at the lift the load offset was given for; the load origin and its own centre offset as given."""
    rotation, translation = base_to_carriage
    com_base = np.asarray(load_origin_in_base, float) + np.asarray(load_rot_in_base, float) @ np.asarray(load_com_in_load, float)
    return np.asarray(rotation, float).T @ (com_base - np.asarray(translation, float))


def steer_axis_moment_z(points, impulses, axis_point) -> float:
    """Sum over contact points of ((p - a) x J)_z about a vertical axis through ``axis_point`` (world frame)."""
    total = 0.0
    ax, ay = float(axis_point[0]), float(axis_point[1])
    for p, j in zip(points, impulses):
        total += (float(p[0]) - ax) * float(j[1]) - (float(p[1]) - ay) * float(j[0])
    return total


# ---- Isaac-side helpers (pxr / omni imported inside) ----

def author_stage(stage, settings: dict, *, base_xyz, base_yaw: float, steer_positions: dict,
                 floor_material_path: str = "/World/ContactMaterial") -> dict:
    """The stage-side arms, authored after configure_drives and before the world reset (A3, A6, A7). ``steer_positions``:
    each steering joint's position in the base frame by name (the scene record). Returns what was authored."""
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    record = {}
    if settings.get("probe_clear_area"):
        props = stage.GetPrimAtPath("/World/RandomProps")
        if props and props.IsValid():
            props.SetActive(False)
        record["cleared_props"] = bool(props and props.IsValid())
    if "probe_fix_base_z_m" in settings:
        joint = UsdPhysics.FixedJoint.Define(stage, BASE_FIX_PATH)
        joint.CreateBody1Rel().SetTargets(["/World/Forklift/base_link"])
        z = float(settings["probe_fix_base_z_m"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(float(base_xyz[0]), float(base_xyz[1]), z))
        w, x, y, zq = yaw_quat_wxyz(base_yaw)
        joint.CreateLocalRot0Attr(Gf.Quatf(w, x, y, zq))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        record["base_fix"] = {"path": BASE_FIX_PATH, "xyz": [float(base_xyz[0]), float(base_xyz[1]), z],
                              "yaw_rad": float(base_yaw)}
    if settings.get("probe_front_tyre_frictionless"):
        material = UsdShade.Material.Define(stage, FRICTIONLESS_PATH)
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(0.0)
        api.CreateDynamicFrictionAttr(0.0)
        api.CreateRestitutionAttr(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
        bound = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Forklift")):
            if prim.HasAPI(UsdPhysics.CollisionAPI) and any(t in str(prim.GetPath()) for t in FRONT_TYRES):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
                bound.append(str(prim.GetPath()))
        if len(bound) != 2:
            raise ValueError(f"front tyre colliders bound: {bound}")
        record["front_tyre_frictionless"] = bound
    if "probe_shim_wheel" in settings:
        wheel = ("front_left", "front_right")[int(settings["probe_shim_wheel"]) - 1]
        cylinder = UsdGeom.Cylinder(stage.GetPrimAtPath(f"/World/Forklift/{wheel}_wheel/collisions/{wheel}_tire_collision/cylinder"))
        tyre_radius_m, tyre_width_m = float(cylinder.GetRadiusAttr().Get()), float(cylinder.GetHeightAttr().Get())
        # the wheel centre is its steering joint's origin in the base frame (the measured model: spin joint and tyre
        # collider at the carrier's origin), carried to the spawn pose -- the stage's wheel transforms do not follow the
        # articulation's placement before the reset (Codex review)
        relative = [float(v) for v in steer_positions[("left_steer", "right_steer")[int(settings["probe_shim_wheel"]) - 1]]]
        c_, s_ = math.cos(base_yaw), math.sin(base_yaw)
        cx = float(base_xyz[0]) + c_ * relative[0] - s_ * relative[1]
        cy = float(base_xyz[1]) + s_ * relative[0] + c_ * relative[1]
        h = float(settings["probe_shim_height_m"])
        size_xy = (SHIM_SIZE_FACTOR * 2 * math.sqrt(2 * tyre_radius_m * 0.002), SHIM_SIZE_FACTOR * tyre_width_m)
        # at least the tyre's diameter long, so the wheel's steering sweep stays on the plate
        size_xy = (max(size_xy[0], 2 * tyre_radius_m), size_xy[1])
        cube = UsdGeom.Cube.Define(stage, SHIM_PATH)
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.XformCommonAPI(cube.GetPrim())
        xform.SetTranslate(Gf.Vec3d(cx, cy, h / 2))
        xform.SetRotate(Gf.Vec3f(0, 0, math.degrees(base_yaw)))
        xform.SetScale(Gf.Vec3f(size_xy[0], size_xy[1], h))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        floor_material = UsdShade.Material(stage.GetPrimAtPath(floor_material_path))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(floor_material, UsdShade.Tokens.strongerThanDescendants,
                                                              "physics")
        record["shim"] = {"path": SHIM_PATH, "wheel": wheel, "centre_xy": [cx, cy], "height_m": h,
                          "wheel_in_base_m": relative,
                          "size_xy_m": list(size_xy), "material": floor_material_path}
    return record


def joint_readback(stage) -> dict:
    """Plan 4.1 record 5: the steering joints' and every tyre collider's engine-facing settings as the stage holds them
    (armature, joint friction, max joint velocity, frames; contact and rest offsets), left and right side by side."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    def get(api, name):
        attr = getattr(api, name)()
        value = attr.Get() if attr else None
        return None if value is None else (float(value) if isinstance(value, (int, float)) else str(value))

    out = {"joints": {}, "colliders": {}}
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Forklift")):
        name = prim.GetName()
        if prim.IsA(UsdPhysics.RevoluteJoint) and name.endswith("_steer"):
            physx = PhysxSchema.PhysxJointAPI(prim)
            joint = UsdPhysics.RevoluteJoint(prim)
            out["joints"][name] = {
                "armature": get(physx, "GetArmatureAttr"), "joint_friction": get(physx, "GetJointFrictionAttr"),
                "max_joint_velocity": get(physx, "GetMaxJointVelocityAttr"),
                "local_pos0": str(joint.GetLocalPos0Attr().Get()), "local_rot0": str(joint.GetLocalRot0Attr().Get()),
                "local_pos1": str(joint.GetLocalPos1Attr().Get()), "local_rot1": str(joint.GetLocalRot1Attr().Get()),
                "axis": str(joint.GetAxisAttr().Get()),
            }
        if prim.HasAPI(UsdPhysics.CollisionAPI) and "_tire_collision" in str(prim.GetPath()):
            collision = PhysxSchema.PhysxCollisionAPI(prim)
            out["colliders"][str(prim.GetPath())] = {"contact_offset": get(collision, "GetContactOffsetAttr"),
                                                     "rest_offset": get(collision, "GetRestOffsetAttr")}
    return out


FULL_FIELDS = ("Jz", "tangent_impulse", "steer_moment_z", "contacts", "anchors", "shim_Jz", "partners",
               "anchors_missing")


ANCHOR_INDEX_PROBLEM = {}


FLOOR_TOKEN = "/GroundPlane/"  # the warehouse scene's floor collider (/World/Environment/GroundPlane/CollisionPlane)


def tyre_full_contacts(tyre_links, axis_points, partners_seen=None, env=None) -> list[list[float]]:
    """Per tyre link, from this step's full contact report (friction anchors included -- plan 4.1 record 4), against
    everything outside the robot: the summed vertical impulse of its contact points except the shim's, the summed
    tangential impulse magnitude (contact points and friction anchors), the z moment of every point and anchor
    impulse about the vertical axis through its ``axis_points`` entry (None: about the mean contact point), the
    contact-point and anchor counts, the shim's vertical impulse, and the number of distinct partner actors this step.
    Impulses carry tyre_floor_forces' sign (the report's impulse acts on actor1: + when the tyre is actor1, - when it
    is actor0): standing, the floor impulses sum to about -weight x dt, so the floor's push on the truck is the
    negative (Jz, moment alike). Divide by the step for
    force. ``partners_seen``: a dict of sets to collect each tyre's partner paths over the run (actor-path
    classification, plan 4.1). NaN rows when the report is unavailable."""
    nan_row = [math.nan] * len(FULL_FIELDS)
    try:
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        headers, data, anchors = get_physx_simulation_interface().get_full_contact_report()
    except Exception:  # noqa: BLE001 -- recorded as missing
        return [nan_row for _ in tyre_links]
    out = [[0.0] * len(FULL_FIELDS) for _ in tyre_links]
    points = [[] for _ in tyre_links]
    impulses = [[] for _ in tyre_links]
    partners = [set() for _ in tyre_links]
    for header in headers:
        paths = (str(PhysicsSchemaTools.intToSdfPath(header.actor0)), str(PhysicsSchemaTools.intToSdfPath(header.actor1)))
        if env is not None:
            # any robot body against anything but the floor or the shim under a tyre: the window is not clean
            robot = [p for p in paths if p.startswith("/World/Forklift")]
            if len(robot) == 1:
                other = paths[1] if paths[0] == robot[0] else paths[0]
                tyre_on_floor = robot[0] in tyre_links and (FLOOR_TOKEN in other or other.startswith(SHIM_PATH))
                if not tyre_on_floor:
                    env["count"] = env.get("count", 0) + max(1, header.num_contact_data)
                    env.setdefault("pairs", set()).add((robot[0], other))
        for w, link in enumerate(tyre_links):
            if link not in paths:
                continue
            other = paths[1] if paths[0] == link else paths[0]
            if other.startswith("/World/Forklift"):
                continue
            partners[w].add(other)
            shim = other.startswith(SHIM_PATH)
            sign = 1.0 if paths[1] == link else -1.0  # as tyre_floor_forces (Codex review: actor order)
            for i in range(header.num_contact_data):
                c = data[header.contact_data_offset + i]
                j = [sign * float(c.impulse[k]) for k in range(3)]
                out[w][5 if shim else 0] += j[2]
                out[w][1] += math.hypot(j[0], j[1])
                out[w][3] += 1
                points[w].append([float(c.position[k]) for k in range(3)])
                impulses[w].append(j)
            for i in range(header.num_friction_anchors_data):
                k_ = header.friction_anchors_offset + i
                if not 0 <= k_ < len(anchors):
                    # an anchor index outside the vector the report returned (seen in the first smoke runs): counted,
                    # and the first case kept for the record; the steer-axis moment then lacks those anchors
                    out[w][7] += 1
                    if ANCHOR_INDEX_PROBLEM.get("first") is None:
                        ANCHOR_INDEX_PROBLEM["first"] = {"tyre": link, "partner": other, "offset": header.friction_anchors_offset,
                                                         "count": header.num_friction_anchors_data, "vector": len(anchors),
                                                         "contacts": header.num_contact_data}
                    continue
                a = anchors[k_]
                j = [sign * float(a.impulse[k]) for k in range(3)]
                out[w][1] += math.hypot(j[0], j[1])
                out[w][4] += 1
                points[w].append([float(a.position[k]) for k in range(3)])
                impulses[w].append(j)
    for w, link in enumerate(tyre_links):
        out[w][6] = float(len(partners[w]))
        if partners_seen is not None:
            partners_seen.setdefault(link, set()).update(partners[w])
        if points[w]:
            axis = axis_points[w] if axis_points[w] is not None else np.mean(np.asarray(points[w]), axis=0)
            out[w][2] = steer_axis_moment_z(points[w], impulses[w], axis)
    return out


def _link_poses(robot) -> dict:
    """Every articulation body's world pose (position, wxyz) by name, from the physics view."""
    view = robot._articulation_view
    poses = np.asarray(view._physics_view.get_link_transforms(), dtype=float).reshape(-1, 7)  # x y z qx qy qz qw
    return {n: (poses[i, :3], np.array([poses[i, 6], *poses[i, 3:6]])) for i, n in enumerate(view.body_names)}


def _in_base(poses: dict) -> dict:
    """Body positions in the base_link frame."""
    p_base, q_base = poses["base_link"]
    rotation = quat_matrix(q_base)
    return {n: rotation.T @ (p - p_base) for n, (p, _) in poses.items()}


class ProbeRuntime:
    """The run-time arms (A4 load and lift, A10 assignment, A11 front spin free) and the extra per-tick records
    (plan 4.1: drive gains and effort limits per tick, lift, base pose, the full contact report per tyre)."""

    def __init__(self, robot, settings: dict, scene_joints: dict, stage, lift_sag):
        """``lift_sag(extra_mass_kg)``: the runner's lift sag rule on this run's plant record (plan D14)."""
        self.robot, self.settings, self.stage = robot, settings, stage
        self.scene_joints, self.lift_sag = scene_joints, lift_sag
        self.record = {"arms": {k: settings[k] for k in PROBE_ARM_SETTINGS if k in settings}}
        self.rows = {"gains": [], "damping": [], "max_effort": [], "lift_q": [], "base_pose": [], "wheel_z": [],
                     "full": [], "env_contacts": []}
        self.env_pairs = set()
        self.partners = {}

    def assign(self, names: list, steers, wheels, tyre_links):
        """The joint indices and tyre links the probe drives and records, in physical order (A10), or as given."""
        poses = _link_poses(self.robot)
        place = physical_assignment(_in_base(poses))
        self.record["assignment"] = place
        if self.settings.get("probe_assign_by_y"):
            steers = np.array([names.index(n) for n in place["steer"]])
            wheels = np.array([names.index(n) for n in place["spin"]])
            tyre_links = tuple(f"/World/Forklift/{n}" for n in place["tyre_links"])
        elif place["steer"] != ["left_steer", "right_steer"]:
            raise ValueError(f"the steering joints are not at their named sides: {place} (a name-swapped model needs "
                             "probe_assign_by_y)")
        self.steer_names = [names[i] for i in steers]
        self.steers, self.wheels, self.tyre_links = steers, wheels, tyre_links
        return steers, wheels, tyre_links

    def setup(self, world, names: list, action_type, lift_index, dt: float, scene_pallet=None) -> None:
        """The clear area, A11, then A4 (the load merged into the carriage, the lift raised and settled), before the
        segments. ``scene_pallet``: the scene's pallet handle (a rigid prim with set_world_pose)."""
        view = self.robot._articulation_view
        if self.settings.get("probe_clear_area") and scene_pallet is not None:
            from pxr import Usd, UsdPhysics

            bodies = [p for p in Usd.PrimRange(self.stage.GetPrimAtPath("/World/Pallet"))
                      if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            for body in bodies:
                UsdPhysics.RigidBodyAPI(body).CreateKinematicEnabledAttr(True)
            position, orientation = scene_pallet.get_world_pose()
            lifted = np.asarray(position, float) + np.array([0.0, 0.0, 10.0])
            scene_pallet.set_world_pose(position=lifted, orientation=orientation)
            world.step(render=False)
            self.record["cleared_pallet"] = {"bodies": [str(p.GetPath()) for p in bodies],
                                             "lifted_to_m": np.asarray(scene_pallet.get_world_pose()[0], float).tolist()}
        if self.settings.get("probe_front_spin_free"):
            front = np.array([names.index(n) for n in self.record["assignment"]["spin"][:2]])
            view.set_gains(kps=np.zeros((1, 2)), kds=np.zeros((1, 2)), joint_indices=front)
            view.set_max_efforts(values=np.zeros((1, 2)), joint_indices=front)
            kps, kds = view.get_gains(joint_indices=front)
            self.record["front_spin_free"] = {"joints": [names[i] for i in front],
                                              "stiffness": np.asarray(kps).reshape(-1).tolist(),
                                              "damping": np.asarray(kds).reshape(-1).tolist(),
                                              "max_effort": np.asarray(view.get_max_efforts(joint_indices=front)).reshape(-1).tolist()}
        if "probe_load_kg" in self.settings:
            self.record["load"] = self._merge_load(view)
            sag = self.lift_sag(float(self.settings["probe_load_kg"]))
            target = float(self.settings["probe_lift_m"]) + float(sag)
            self.robot.apply_action(action_type(joint_positions=np.array([target]), joint_indices=lift_index))
            for _ in range(int(round(LIFT_SETTLE_S / dt))):
                world.step(render=False)
            self.record["lift"] = {"target_m": target, "sag_m": float(sag),
                                   "settled_q_m": float(self.robot.get_joint_positions()[lift_index[0]])}

    def _merge_load(self, view) -> dict:
        from pxr import UsdPhysics

        names = list(view.body_names)
        k = names.index("fork_carriage")
        mass_kg = float(self.settings["probe_load_kg"])
        pallet = self.stage.GetPrimAtPath("/World/Pallet/pallet")
        api = UsdPhysics.MassAPI(pallet)
        pallet_mass = float(api.GetMassAttr().Get())
        com_pallet = np.asarray(api.GetCenterOfMassAttr().Get(), dtype=float)
        principal = quat_matrix([api.GetPrincipalAxesAttr().Get().GetReal(), *api.GetPrincipalAxesAttr().Get().GetImaginary()])
        inertia_load = principal @ np.diag(np.asarray(api.GetDiagonalInertiaAttr().Get(), float)) @ principal.T
        inertia_load *= mass_kg / pallet_mass
        poses = _link_poses(self.robot)
        p_base, q_base = poses["base_link"]
        p_carr, q_carr = poses["fork_carriage"]
        r_base = quat_matrix(q_base)
        r_bc = r_base.T @ quat_matrix(q_carr)
        t_bc0 = r_base.T @ (p_carr - p_base)
        lift_now = float(self.robot.get_joint_positions()[list(self.robot.dof_names).index("fork_lift")])
        # the carriage frame at the lift the load offset was taken at (prismatic along the carriage z)
        t_bc = t_bc0 + (float(self.settings["probe_lift_m"]) - lift_now) * r_bc[:, 2]
        com_c = load_in_carriage((r_bc, t_bc), [float(self.settings["probe_load_dx_m"]), 0.0,
                                                float(self.settings["probe_load_dz_m"])], com_pallet)
        masses = np.asarray(view.get_body_masses(), float).reshape(-1)
        coms_p, coms_q = view.get_body_coms()
        coms_p = np.asarray(coms_p, float).reshape(-1, 3)
        coms_q = np.asarray(coms_q, float).reshape(-1, 4)
        inertias = np.asarray(view.get_body_inertias(), float).reshape(-1, 9)
        # about the COM, already expressed in the link frame (Isaac 5.1 get_body_inertias; Codex review)
        inertia_carr = inertias[k].reshape(3, 3)
        rotation_load = r_bc.T  # the load's axes (aligned with the base) in the carriage frame
        m, c, inertia = composite_body(masses[k], coms_p[k], inertia_carr, mass_kg, com_c,
                                       rotation_load @ inertia_load @ rotation_load.T)
        body = np.array([k])
        view.set_body_masses(np.array([[m]]), body_indices=body)
        view.set_body_coms(positions=c.reshape(1, 1, 3), orientations=np.array([[[1.0, 0.0, 0.0, 0.0]]]),
                           body_indices=body)
        view.set_body_inertias(inertia.reshape(1, 1, 9), body_indices=body)
        return {"carriage_before": {"mass_kg": float(masses[k]), "com_m": coms_p[k].tolist(),
                                    "inertia": inertia_carr.reshape(-1).tolist()},
                "load": {"mass_kg": mass_kg, "com_in_carriage_m": com_c.tolist(),
                         "scene_pallet_mass_kg": pallet_mass, "scene_pallet_com_m": com_pallet.tolist()},
                "carriage_after_set": {"mass_kg": float(m), "com_m": c.tolist(), "inertia": inertia.reshape(-1).tolist()},
                "carriage_readback": {"mass_kg": float(np.asarray(view.get_body_masses(), float).reshape(-1)[k]),
                                      "com_m": np.asarray(view.get_body_coms()[0], float).reshape(-1, 3)[k].tolist()}}

    def tick(self) -> None:
        view = self.robot._articulation_view
        kps, kds = view.get_gains(joint_indices=self.steers)
        self.rows["gains"].append(np.asarray(kps, float).reshape(-1)[:2])
        self.rows["damping"].append(np.asarray(kds, float).reshape(-1)[:2])
        self.rows["max_effort"].append(np.asarray(view.get_max_efforts(joint_indices=self.steers), float).reshape(-1)[:2])
        names = list(self.robot.dof_names)
        self.rows["lift_q"].append(float(self.robot.get_joint_positions()[names.index("fork_lift")]))
        position, orientation = self.robot.get_world_pose()
        self.rows["base_pose"].append([*np.asarray(position, float), *np.asarray(orientation, float)])
        # the four wheel link centres' heights, physical FL FR RL RR (plan 4.2 A8: the sag check)
        poses = _link_poses(self.robot)
        self.rows["wheel_z"].append([float(poses[n.rsplit("/", 1)[-1]][0][2]) for n in self.tyre_links])
        rotation = quat_matrix(orientation)
        axes = [np.asarray(position, float) + rotation @ np.asarray(self.scene_joints[n]["position_m"], float)
                for n in self.steer_names] + [None, None]
        env = {}
        self.rows["full"].append(tyre_full_contacts(self.tyre_links, axes, self.partners, env))
        self.rows["env_contacts"].append(float(env.get("count", 0)))
        self.env_pairs |= env.get("pairs", set())

    def save(self, out_dir) -> dict:
        np.savez(out_dir / "steering_probe_arms.npz", steer_names=np.array(self.steer_names),
                 tyre_links=np.array(self.tyre_links), full_fields=np.array(FULL_FIELDS),
                 **{k: np.asarray(v, dtype=float) for k, v in self.rows.items()})
        self.record["partners"] = {k: sorted(v) for k, v in self.partners.items()}
        self.record["anchor_index_problem"] = ANCHOR_INDEX_PROBLEM.get("first")
        self.record["env_contact_pairs"] = sorted(list(p) for p in self.env_pairs)
        return self.record
