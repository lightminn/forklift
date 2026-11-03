"""Chassis model contract checks for Isaac runs, without an Isaac SDK."""

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))
SPEC = importlib.util.spec_from_file_location(
    "chassis_contract", ROOT / "sim/isaac/chassis_contract.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
from insertion_geometry import read_drive_geometry_m  # noqa: E402

PROVISIONAL = ROOT / "sim/models/dls08_provisional/forklift.urdf"
MEASURED = ROOT / "sim/models/dls08_measured/forklift.urdf"


def settings(name):
    return yaml.safe_load((ROOT / "config" / name).read_text())


def test_kinematic_curvature_limit_matches_the_drive_clamp():
    measured = read_drive_geometry_m(MEASURED, 8.0)
    t = math.tan(math.radians(15))
    assert MODULE.kinematic_curvature_limit_inv_m(measured) == pytest.approx(
        t / (0.66 + t * 0.265)
    )
    assert 1 / MODULE.kinematic_curvature_limit_inv_m(measured) == pytest.approx(
        2.728, abs=5e-4
    )


@pytest.mark.parametrize(
    ("urdf", "config"),
    [
        (PROVISIONAL, "isaac_transport.yaml"),
        (PROVISIONAL, "isaac_transport_fast.yaml"),
        (MEASURED, "isaac_transport_measured.yaml"),
    ],
)
def test_shipped_settings_fit_their_model(urdf, config):
    MODULE.require_curvature_within_model(
        settings(config), read_drive_geometry_m(urdf, 8.0)
    )


def test_provisional_settings_are_refused_on_the_measured_model():
    with pytest.raises(ValueError, match="tracker_curvature_inv_m"):
        MODULE.require_curvature_within_model(
            settings("isaac_transport.yaml"), read_drive_geometry_m(MEASURED, 8.0)
        )


def test_measured_settings_differ_from_the_default_only_in_curvature_and_the_pallet_mass():
    default = settings("isaac_transport.yaml")
    measured = settings("isaac_transport_measured.yaml")
    changed = {k for k in default if default[k] != measured.get(k)}
    # pallet-mass plan (2026-11-01): the measured settings carry a mass by pallet shape, the provisional ones 10 kg
    assert changed == {"planner_curvature_inv_m", "tracker_curvature_inv_m", "pallet_mass_kg"}
    assert set(measured) == set(default)


def test_scene_matching_the_model_passes():
    chassis = MODULE.urdf_chassis(MEASURED)
    assert set(chassis.joints) == set(MODULE.SCENE_JOINTS)
    assert len(chassis.tyre_radii_m) == 4
    assert {n for n in chassis.carriage_boxes if n.startswith("carriage_cross")} == {
        f"carriage_cross_{i}_collision" for i in range(3)
    }
    MODULE.require_scene_matches_model(chassis, MEASURED)


def test_a_scene_built_from_the_other_model_is_refused():
    with pytest.raises(ValueError, match="left_steer is 24.5 mm"):
        MODULE.require_scene_matches_model(MODULE.urdf_chassis(PROVISIONAL), MEASURED)


def test_a_missing_joint_is_refused():
    chassis = MODULE.urdf_chassis(MEASURED)
    missing = dict(chassis.joints)
    del missing["right_steer"]
    with pytest.raises(ValueError, match="right_steer"):
        MODULE.require_scene_matches_model(chassis._replace(joints=missing), MEASURED)


def principal(tensor):
    """(principal moments, principal axes as a (w, x, y, z) quaternion) of a link-frame tensor."""
    import numpy as np

    values, vectors = np.linalg.eigh(np.array(tensor))
    if np.linalg.det(vectors) < 0:
        vectors[:, 0] = -vectors[:, 0]
    m = vectors
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-6:
        q = (w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w))
    else:  # a half turn: the axis from the largest diagonal term
        i = int(np.argmax(np.diag(m)))
        axis = np.sqrt(np.clip((np.diag(m) + 1) / 2, 0, None))
        axis = axis * np.sign(np.where(m[i] != 0, m[i], 1.0))
        q = (0.0, *axis)
    return tuple(float(v) for v in values), tuple(float(v) for v in q)


def importer_like_stage(
    urdf,
    *,
    tyre_radius=None,
    cross_x=None,
    steer_mount_x=0.0,
    root_scale=(1.0, 1.0, 1.0),
    steer_rot0=(1.0, 0.0, 0.0, 0.0),
    steer_axis="Z",
    tyre_upright=False,
    steer_limit_deg=None,
    base_mass_scale=1.0,
    lift_upper=None,
    drop_masses=False,
    base_axes=None,
    swap_base_moments=False,
):
    """USD laid out like Isaac 5.1's URDF importer output (checked 2026-10-01).

    Joints under <root>/joints with body0/localPos0; collisions as scaled unit
    Cubes and Z-axis Cylinders under <link>/collisions/<name>_collision.
    `steer_mount_x` puts the steer joints' body0 on an offset mount link.
    """
    pytest.importorskip("pxr")
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    expected = MODULE.urdf_chassis(urdf)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = "/World/Forklift"
    UsdGeom.Xform.Define(stage, root).AddScaleOp().Set(Gf.Vec3f(*root_scale))
    base = UsdGeom.Xform.Define(stage, f"{root}/base_link")
    # The truck is not at the origin.
    base.AddTranslateOp().Set(Gf.Vec3d(2.0, -1.0, 0.0))
    mount = UsdGeom.Xform.Define(stage, f"{root}/base_link/steer_mount")
    mount.AddTranslateOp().Set(Gf.Vec3d(steer_mount_x, 0, 0))
    for name, joint in expected.joints.items():
        prim = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/joints/{name}")
        body0 = f"{root}/base_link"
        x, y, z = joint.position_m
        if name.endswith("_steer"):
            body0 = f"{root}/base_link/steer_mount"
            x -= steer_mount_x
        prim.CreateBody0Rel().SetTargets([body0])
        prim.CreateLocalPos0Attr().Set(Gf.Vec3f(x, y, z))
        rot0 = steer_rot0 if name.endswith("_steer") else (1.0, 0.0, 0.0, 0.0)
        prim.CreateLocalRot0Attr().Set(Gf.Quatf(rot0[0], Gf.Vec3f(*rot0[1:])))
        prim.CreateAxisAttr().Set(steer_axis if name.endswith("_steer") else "Y")
        if joint.limit_rad is not None:
            lower, upper = (math.degrees(v) for v in joint.limit_rad)
            if steer_limit_deg is not None:
                lower, upper = -steer_limit_deg, steer_limit_deg
            prim.CreateLowerLimitAttr().Set(lower)
            prim.CreateUpperLimitAttr().Set(upper)
    for wheel, radius in expected.tyre_radii_m.items():
        link = UsdGeom.Xform.Define(stage, f"{root}/{wheel}_wheel")
        link.AddTranslateOp().Set(Gf.Vec3d(2.0, -1.0, 0.0))
        holder = UsdGeom.Xform.Define(
            stage, f"{root}/{wheel}_wheel/collisions/{wheel}_tire_collision"
        )
        if not tyre_upright:  # the importer turns the Z-axis cylinder onto -Y
            holder.AddOrientOp().Set(Gf.Quatf(0.5**0.5, Gf.Vec3f(0.5**0.5, 0, 0)))
        cylinder = UsdGeom.Cylinder.Define(
            stage, f"{root}/{wheel}_wheel/collisions/{wheel}_tire_collision/cylinder"
        )
        cylinder.CreateRadiusAttr().Set(tyre_radius or radius)
        cylinder.CreateAxisAttr().Set("Z")
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    carriage = UsdGeom.Xform.Define(stage, f"{root}/fork_carriage")
    carriage.AddTranslateOp().Set(Gf.Vec3d(2.0, -1.0, 0.0))
    for name, (center, half) in expected.carriage_boxes.items():
        holder = UsdGeom.Xform.Define(stage, f"{root}/fork_carriage/collisions/{name}")
        cx = (
            cross_x if (cross_x is not None and "carriage_cross" in name) else center[0]
        )
        holder.AddTranslateOp().Set(Gf.Vec3d(cx, center[1], center[2]))
        holder.AddScaleOp().Set(Gf.Vec3f(*(2 * h for h in half)))
        cube = UsdGeom.Cube.Define(stage, f"{root}/fork_carriage/collisions/{name}/box")
        cube.CreateSizeAttr().Set(1.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    # every link as the importer writes it: a rigid body with its mass, centre of mass, principal moments and
    # principal axes (the URDF tensor's eigen-decomposition), and the lift joint
    for link, inertia in expected.inertials.items():
        prim = stage.GetPrimAtPath(f"{root}/{link}")
        if not prim.IsValid():
            prim = UsdGeom.Xform.Define(stage, f"{root}/{link}").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        if drop_masses:
            continue
        api = UsdPhysics.MassAPI.Apply(prim)
        if inertia is None:
            api.CreateMassAttr().Set(0.0)
            continue
        scale = base_mass_scale if link == "base_link" else 1.0
        moments, quat = principal(inertia.tensor_kg_m2)
        if link == "base_link" and swap_base_moments:
            moments = (moments[2], moments[1], moments[0])
        if link == "base_link" and base_axes is not None:
            quat = base_axes
        api.CreateMassAttr().Set(inertia.mass_kg * scale)
        api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*inertia.com_m))
        api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*(v * scale for v in moments)))
        api.CreatePrincipalAxesAttr().Set(Gf.Quatf(quat[0], Gf.Vec3f(*quat[1:])))
    lift = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/joints/fork_lift")
    lower, upper = expected.lift_limit_m
    lift.CreateLowerLimitAttr().Set(lower)
    lift.CreateUpperLimitAttr().Set(upper if lift_upper is None else lift_upper)
    return stage


def test_an_importer_like_scene_of_the_model_passes_through_a_mount_link():
    stage = importer_like_stage(MEASURED, steer_mount_x=0.10)
    MODULE.require_scene_matches_model(
        MODULE.stage_chassis(stage, "/World/Forklift"), MEASURED
    )


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"tyre_radius": 0.135}, "tyre"),
        ({"cross_x": 0.53}, "carriage_cross"),
        ({"root_scale": (2.0, 2.0, 2.0)}, "rigid"),
        ({"root_scale": (-1.0, 1.0, 1.0)}, "rigid"),
        ({"steer_rot0": (0.0, 1.0, 0.0, 0.0)}, "axis"),
        ({"steer_axis": "X"}, "axis"),
        ({"tyre_upright": True}, "tyre"),
        ({"tyre_radius": math.nan}, "finite"),
        ({"steer_limit_deg": math.nan}, "finite"),
        # model audit (lead 2026-11-01): Codex's counterexamples -- a 26 kg truck, a 0.028 m lift
        ({"base_mass_scale": 26.0 / 24.0}, "base_link mass"),
        ({"lift_upper": 0.028}, "fork_lift travel"),
        ({"drop_masses": True}, "no mass of its own"),
        # Codex chassis review P2-1: a rotated or reordered tensor with the same moments
        ({"base_axes": (0.5**0.5, 0.0, 0.5**0.5, 0.0)}, "base_link inertia tensor"),
        ({"swap_base_moments": True}, "base_link inertia tensor"),
    ],
)
def test_same_joints_but_other_collision_geometry_is_refused(change, message):
    stage = importer_like_stage(MEASURED, **change)
    with pytest.raises(ValueError, match=message):
        MODULE.require_scene_matches_model(
            MODULE.stage_chassis(stage, "/World/Forklift"), MEASURED
        )


def test_the_tolerance_is_one_millimetre_exclusive():
    joints = MODULE.urdf_chassis(MEASURED)
    for offset, refused in [(0.0009, False), (0.0011, True)]:
        x, y, z = joints.joints["left_steer"].position_m
        moved = dict(joints.joints)
        moved["left_steer"] = moved["left_steer"]._replace(
            position_m=(x + offset, y, z)
        )
        scene = joints._replace(joints=moved)
        if refused:
            with pytest.raises(ValueError, match="left_steer"):
                MODULE.require_scene_matches_model(scene, MEASURED)
        else:
            MODULE.require_scene_matches_model(scene, MEASURED)


def test_a_tyre_that_does_not_roll_about_y_is_refused_in_the_urdf(tmp_path):
    import xml.etree.ElementTree as ET

    tree = ET.parse(MEASURED)
    for origin in tree.getroot().findall("link/collision/origin"):
        if origin.get("rpy", "").startswith("1.57"):
            origin.set("rpy", "0 0 0")
    path = tmp_path / "forklift.urdf"
    tree.write(path)
    with pytest.raises(ValueError, match="tyre"):
        read_drive_geometry_m(path, 8.0)


def test_the_record_carries_what_the_scene_check_read():
    stage = importer_like_stage(MEASURED, steer_mount_x=0.10)
    scene = MODULE.stage_chassis(stage, "/World/Forklift")
    record = MODULE.chassis_record(scene)
    assert (
        record["joints"]["left_steer"]["body0"]
        == "/World/Forklift/base_link/steer_mount"
    )
    assert record["joints"]["left_steer"]["local_pos0_m"][0] == pytest.approx(0.22)
    assert record["joints"]["left_steer"]["position_m"][0] == pytest.approx(0.32)
    assert record["tyre_radii_m"]["rear_left"] == pytest.approx(0.125)
    assert record["carriage_limit_m"] == pytest.approx(0.346)
    import json

    json.dumps(record, allow_nan=False)


def test_a_frame_link_without_a_mass_of_its_own_matches_a_urdf_link_without_an_inertial(tmp_path):
    # the operating base scene (2026-10-13) was built from a URDF whose fork-tip frames carried no inertial: the
    # importer leaves them at mass 0, which the check reads as "none of its own" on both sides
    truck = ET.parse(MEASURED)
    for link in truck.getroot().findall("link"):
        if link.get("name").endswith("_fork_tip") and link.find("inertial") is not None:
            link.remove(link.find("inertial"))
    old = tmp_path / "forklift.urdf"
    truck.write(old)
    stage = importer_like_stage(old)
    chassis = MODULE.stage_chassis(stage, "/World/Forklift")
    MODULE.require_scene_matches_model(chassis, old)
    assert MODULE.chassis_record(chassis)["links_without_mass"] == ["left_fork_tip", "right_fork_tip"]
    with pytest.raises(ValueError, match="left_fork_tip mass"):
        MODULE.require_scene_matches_model(chassis, MEASURED)  # the repo URDF gives them 0.0001 kg


def _stage_with(extra):
    """An importer-like scene of the measured model with ``extra(stage, root)`` applied (Codex chassis review P2-2)."""
    stage = importer_like_stage(MEASURED)
    extra(stage, "/World/Forklift")
    return MODULE.stage_chassis(stage, "/World/Forklift")


def test_mass_from_anywhere_but_a_links_own_explicit_mass_is_refused():
    pytest.importorskip("pxr")
    from pxr import UsdGeom, UsdPhysics

    def nested_body(stage, root):  # a 26 kg body fixed below base_link
        prim = UsdGeom.Cube.Define(stage, f"{root}/base_link/ballast").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(26.0)

    def child_mass(stage, root):  # a 2 kg collider mass below a link
        prim = UsdGeom.Cube.Define(stage, f"{root}/fork_carriage/collisions/weight").GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(2.0)

    def link_density(stage, root):
        UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{root}/base_link")).CreateDensityAttr().Set(1000.0)

    for extra, message in ((nested_body, "rigid body"), (child_mass, "mass authored below a link"),
                           (link_density, "density")):
        with pytest.raises(ValueError, match=message):
            _stage_with(extra)


def test_a_link_without_a_mass_of_its_own_may_not_carry_colliders(tmp_path):
    pytest.importorskip("pxr")
    from pxr import UsdGeom, UsdPhysics

    truck = ET.parse(MEASURED)
    for link in truck.getroot().findall("link"):
        if link.get("name").endswith("_fork_tip") and link.find("inertial") is not None:
            link.remove(link.find("inertial"))
    old = tmp_path / "forklift.urdf"
    truck.write(old)
    stage = importer_like_stage(old)
    cube = UsdGeom.Cube.Define(stage, "/World/Forklift/left_fork_tip/collisions/stub").GetPrim()
    UsdPhysics.CollisionAPI.Apply(cube)
    with pytest.raises(ValueError, match="left_fork_tip has no mass of its own but has colliders"):
        MODULE.stage_chassis(stage, "/World/Forklift")


def test_an_inertial_rpy_rotates_the_urdf_tensor(tmp_path):
    truck = ET.parse(MEASURED)
    base = truck.getroot().find("link[@name='base_link']/inertial/origin")
    base.set("rpy", "0 1.5707963267948966 0")
    turned = tmp_path / "forklift.urdf"
    truck.write(turned)
    straight = MODULE.urdf_chassis(MEASURED).inertials["base_link"].tensor_kg_m2
    rotated = MODULE.urdf_chassis(turned).inertials["base_link"].tensor_kg_m2
    assert rotated[0][0] == pytest.approx(straight[2][2]) and rotated[2][2] == pytest.approx(straight[0][0])
    with pytest.raises(ValueError, match="base_link inertia tensor"):
        MODULE.require_scene_matches_model(MODULE.urdf_chassis(turned), MEASURED)


def test_the_manifest_names_the_urdf_a_scene_was_built_from(tmp_path):
    import hashlib
    import json

    scene = tmp_path / "scene.usda"
    scene.write_text("#usda 1.0\n")
    sha = hashlib.sha256(MEASURED.read_bytes()).hexdigest()
    assert MODULE.scene_manifest_urdf(scene, MEASURED) == (None, sha)  # no manifest
    (tmp_path / "base_scene_manifest.json").write_text(json.dumps({"forklift_urdf_sha256": "0" * 64}))
    built_from, running = MODULE.scene_manifest_urdf(scene, MEASURED)
    assert built_from == "0" * 64 and running == sha


def test_every_liftable_pallets_body_mass_is_read():
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics

    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/isaac"))
    import run_transport

    stage = Usd.Stage.CreateInMemory()
    for root, mass in (("/World/SessionPallets/P_1", 4.93), ("/World/SessionPallets/P_2", 2.49)):
        body = UsdGeom.Xform.Define(stage, f"{root}/pallet").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr().Set(mass)
    got = run_transport.pallet_body_masses_kg(stage, ("/World/SessionPallets/P_1", "/World/SessionPallets/P_2"))
    assert got == pytest.approx({"/World/SessionPallets/P_1/pallet": 4.93, "/World/SessionPallets/P_2/pallet": 2.49})
