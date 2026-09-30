"""Chassis model contract checks for Isaac runs, without an Isaac SDK."""

import importlib.util
import math
import sys
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


def test_measured_settings_differ_from_the_default_only_in_curvature():
    default = settings("isaac_transport.yaml")
    measured = settings("isaac_transport_measured.yaml")
    changed = {k for k in default if default[k] != measured.get(k)}
    assert changed == {"planner_curvature_inv_m", "tracker_curvature_inv_m"}
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
