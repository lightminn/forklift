"""The steering probe's model dump (sim/isaac/model_dump.py, plan 2026-11-17-model-symmetry-check 1)."""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

import model_dump as M  # noqa: E402

pxr = pytest.importorskip("pxr")


class View:
    body_names = ["base_link", "front_left_wheel"]

    def get_body_masses(self):
        return np.array([[20.1, 0.55]])

    def get_body_coms(self):
        return np.array([[[-0.09, 0, 0.155], [0, 0, 0]]]), np.array([[[1, 0, 0, 0], [1, 0, 0, 0]]])

    def get_body_inertias(self):
        return np.array([[np.eye(3).reshape(-1), (2 * np.eye(3)).reshape(-1)]])


def test_engine_bodies_reads_masses_coms_and_inertias():
    out = M.engine_bodies(View())
    assert out["errors"] == {}
    assert out["bodies"]["front_left_wheel"]["mass_kg"] == pytest.approx(0.55)
    assert out["bodies"]["base_link"]["com_m"] == pytest.approx([-0.09, 0, 0.155])
    assert out["bodies"]["front_left_wheel"]["inertia"][0] == 2.0


def test_a_failed_engine_query_stays_undecided():
    class Bad(View):
        def get_body_inertias(self):
            raise RuntimeError("no physics view")

    out = M.engine_bodies(Bad())
    assert "inertias" in out["errors"] and out["bodies"]["base_link"]["inertia"] is None
    assert out["bodies"]["base_link"]["mass_kg"] == pytest.approx(20.1)  # the others still read


def _stage():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    s = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    UsdGeom.Xform.Define(s, "/World/Forklift")
    base = UsdGeom.Xform.Define(s, "/World/Forklift/base_link")
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    m = UsdPhysics.MassAPI.Apply(base.GetPrim())
    m.CreateMassAttr(20.1)
    m.CreateCenterOfMassAttr(Gf.Vec3f(-0.09, 0, 0.155))
    for side, y in (("left", 0.265), ("right", -0.265)):
        w = UsdGeom.Xform.Define(s, f"/World/Forklift/front_{side}_wheel")
        w.AddTranslateOp().Set(Gf.Vec3d(0.32, y, 0.125))
        c = UsdGeom.Cylinder.Define(s, f"/World/Forklift/front_{side}_wheel/tire")
        c.CreateRadiusAttr(0.125)
        c.CreateHeightAttr(0.08)
        c.CreateAxisAttr("Y")
        c.CreateExtentAttr([(-0.125, -0.04, -0.125), (0.125, 0.04, 0.125)])
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        c.GetPrim().CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.02)
        j = UsdPhysics.RevoluteJoint.Define(s, f"/World/Forklift/{side}_steer")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets(["/World/Forklift/base_link"])
        j.CreateBody1Rel().SetTargets([f"/World/Forklift/front_{side}_wheel"])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.32, y, 0.125))
        j.CreateLowerLimitAttr(-15.0)
        j.CreateUpperLimitAttr(15.0)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateStiffnessAttr(57295.8)
        d.CreateDampingAttr(5729.6)
        d.CreateMaxForceAttr(20.0)
        j.GetPrim().CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(0.0)
    return s


def test_stage_readers_report_both_sides():
    s = _stage()
    am = M.authored_masses(s)
    assert am["base_link"]["mass_kg"] == pytest.approx(20.1) and am["base_link"]["com_m"][2] == pytest.approx(0.155)
    js = M.joints(s)
    assert set(js) == {"left_steer", "right_steer"}
    assert js["left_steer"]["drive"]["stiffness"] == pytest.approx(57295.8)
    assert js["left_steer"]["local_pos0"][1] == pytest.approx(0.265) and js["right_steer"]["local_pos0"][1] == pytest.approx(-0.265)
    assert js["right_steer"]["lower_deg"] == -15.0 and js["right_steer"]["armature"] == 0.0
    cs = M.colliders(s)
    left = cs["/World/Forklift/front_left_wheel/tire"]
    assert left["type"] == "Cylinder" and left["radius"] == pytest.approx(0.125) and left["axis"] == "Y"
    assert left["world_min_m"][2] == pytest.approx(0.0, abs=1e-6) and left["contact_offset"] == pytest.approx(0.02)
    assert cs["/World/Forklift/front_right_wheel/tire"]["world_center_m"][1] == pytest.approx(-0.265)


def test_settled_summary_turns_impulses_into_floor_force():
    dt = 1 / 120
    rows = [[[0, 0, -120 * dt] + [0] * 7, [0, 0, -30 * dt] + [0] * 7] for _ in range(4)]
    out = M.settled_summary(rows, ("FL", "FR"), dt, {"FL": [0.125, 0.125]}, [[0.001, -0.002]] * 4)
    assert out["floor_fz_n"] == pytest.approx([120.0, 30.0]) and out["samples"] == 4
    assert out["wheel_center_z_m"]["FL"] == pytest.approx(0.125) and out["base_pitch_rad"] == pytest.approx(-0.002)


def test_the_runner_option_is_off_by_default_and_needs_the_probe():
    import argparse

    import near_tracking
    import run_transport

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["steering_probe_dump"] is False
    off = run_transport.recorded_arguments(argparse.Namespace(steering_probe_dump=False, seed=1))
    assert "steering_probe_dump" not in off
    assert run_transport.recorded_arguments(argparse.Namespace(steering_probe_dump=True, seed=1))["steering_probe_dump"]
    src = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'parser.error("--steering-probe-dump goes with --steering-probe")' in src
    # the dump comes after the arms' setup (D1's merged load), before the segments
    i, j, k = (src.index("arms.setup(world, names, action_type, lift_index, dt, scene_pallet=scene_pallet)"),
               src.index("        probe_model_dump(world, robot, wheels, steers, steering"),
               src.index("for index, (label, target_speed, kappa, seconds) in enumerate(segments):",
                         src.index("def run_steering_probe(")))
    assert i < j < k


def _compare():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "compare_lr", ROOT / "docs/validation/2026-11-17-model-symmetry-check/compare_lr.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _dump(fr_mass=0.55, fr_z=0.125):
    ident = [1.0, 0.0, 0.0, 0.0]
    eye = np.eye(3).reshape(-1).tolist()
    bodies = {"base_link": {"mass_kg": 20.1, "com_m": [-0.09, 0, 0.155], "inertia": eye},
              "front_left_wheel": {"mass_kg": 0.55, "com_m": [0, 0, 0], "inertia": eye},
              "front_right_wheel": {"mass_kg": fr_mass, "com_m": [0, 0, 0], "inertia": eye}}
    poses = {"base_link": {"position_m": [0, 0, 0.125], "quat_wxyz": ident},
             "front_left_wheel": {"position_m": [0.32, 0.265, 0.125], "quat_wxyz": ident},
             "front_right_wheel": {"position_m": [0.32, -0.265, fr_z], "quat_wxyz": ident}}
    cyl = {"type": "Cylinder", "radius": 0.125, "height": 0.08, "axis": "Y", "contact_offset": 0.02, "rest_offset": 0.0}
    colliders = {"/World/Forklift/front_left_wheel/tire": {**cyl, "world_center_m": [0.32, 0.265, 0.125]},
                 "/World/Forklift/front_right_wheel/tire": {**cyl, "world_center_m": [0.32, -0.265, fr_z]}}
    drive = {"type": "force", "stiffness": 57295.8, "damping": 5729.6, "max_force": 20.0}
    joints = {s: {"lower_deg": -15.0, "upper_deg": 15.0, "armature": 0.0, "joint_friction": 0.0,
                  "max_joint_velocity": None, "drive": drive, "local_pos0": [0.32, y, 0.0], "local_pos1": [0, 0, 0]}
              for s, y in (("left_steer", 0.265), ("right_steer", -0.265))}
    return {"engine": {"bodies": bodies, "errors": {}}, "body_poses": poses, "colliders": colliders, "joints": joints,
            "settled": {"floor_fz_n": [120, 29, 26, 80]}}


URDF_IN = {"base_link": {"mass_kg": 20.1, "com_m": [-0.09, 0, 0.155], "inertia": np.eye(3).tolist()},
           "front_left_wheel": {"mass_kg": 0.55, "com_m": [0, 0, 0], "inertia": np.eye(3).tolist()},
           "front_right_wheel": {"mass_kg": 0.55, "com_m": [0, 0, 0], "inertia": np.eye(3).tolist()}}


def test_a_symmetric_dump_has_no_mismatch_even_with_unequal_loads():
    C = _compare()
    out = C.check(_dump(), URDF_IN)
    assert out["verdict"] == "undecided"  # the rear wheels and spin joints are missing from this small dump
    assert not out["crossing"] and out["settled"]["floor_fz_n"] == [120, 29, 26, 80]  # results are not a defect


def test_a_heavier_right_wheel_or_an_offset_tyre_is_a_defect():
    C = _compare()
    heavy = C.check(_dump(fr_mass=0.60), URDF_IN)
    assert heavy["verdict"] == "defect"
    assert {i["item"] for i in heavy["crossing"]} >= {"mass front_left_wheel~front_right_wheel",
                                                       "mass front_right_wheel vs expected"}
    low = C.check(_dump(fr_z=0.124), URDF_IN)
    assert any(i["item"].startswith("tyre centre mirror") for i in low["crossing"])


def test_a_failed_engine_query_leaves_the_verdict_undecided():
    C = _compare()
    d = _dump()
    d["engine"]["errors"] = {"inertias": "RuntimeError"}
    assert C.check(d, URDF_IN)["verdict"] in ("undecided", "defect")
    assert "engine:inertias" in C.check(d, URDF_IN)["undecided"]


def test_the_merged_carriage_is_checked_against_the_arm_record():
    C = _compare()
    d = _dump()
    d["engine"]["bodies"]["fork_carriage"] = {"mass_kg": 6.6, "com_m": [0.3, 0, 0.1], "inertia": None}
    d["body_poses"]["fork_carriage"] = {"position_m": [0.65, 0, 0.15], "quat_wxyz": [1, 0, 0, 0]}
    urdf = {**URDF_IN, "fork_carriage": {"mass_kg": 1.6, "com_m": [0, 0, 0], "inertia": None}}
    plain = C.check(d, urdf)
    assert any(i["item"] == "mass fork_carriage vs expected" for i in plain["crossing"])
    merged = C.check(d, urdf, {"mass_kg": 6.6, "com_m": [0.3, 0, 0.1], "inertia": None})
    assert not any(i["item"].startswith("mass fork_carriage") or i["item"].startswith("com fork_carriage")
                   for i in merged["crossing"])
