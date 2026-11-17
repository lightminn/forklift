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
        UsdPhysics.RigidBodyAPI.Apply(w.GetPrim())
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
    from pxr import UsdShade
    mat = UsdShade.Material.Define(s, "/World/TyreMaterial")
    mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mapi.CreateStaticFrictionAttr(0.8)
    mapi.CreateDynamicFrictionAttr(0.7)
    for side in ("left", "right"):
        tire = s.GetPrimAtPath(f"/World/Forklift/front_{side}_wheel/tire")
        UsdShade.MaterialBindingAPI.Apply(tire).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")
    UsdPhysics.ArticulationRootAPI.Apply(s.GetPrimAtPath("/World/Forklift"))
    s.GetPrimAtPath("/World/Forklift").CreateAttribute(
        "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
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
    assert left["in_body_center_m"] == pytest.approx([0, 0, 0]) and left["body"] == "/World/Forklift/front_left_wheel"
    assert left["body_scale"] == pytest.approx([1.0, 1.0, 1.0])
    assert left["material"]["path"] == "/World/TyreMaterial" and left["material"]["static_friction"] == pytest.approx(0.8)
    art = M.articulation(s)
    assert art["solver_position_iterations"] == 32 and art["solver_velocity_iterations"] is None


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


SETTINGS = {"steering_stiffness": 1000.0, "steering_damping": 100.0, "steering_torque_nm": 20.0, "wheel_torque_nm": 3.0,
            "static_friction": 0.8, "dynamic_friction": 0.7}
Q0 = [1.0, 0.0, 0.0, 0.0]
WHEELS = {"front_left_wheel": 0.265, "front_right_wheel": -0.265, "rear_left_wheel": 0.265, "rear_right_wheel": -0.265}


def _inertia(a, b, c):
    return [a, 0, 0, 0, b, 0, 0, 0, c]


def _full_dump():
    bodies = {"base_link": {"mass_kg": 20.1, "com_m": [-0.09, 0, 0.155], "inertia": _inertia(1, 2, 3)},
              "fork_carriage": {"mass_kg": 1.6, "com_m": [0, 0, 0], "inertia": _inertia(0.1, 0.1, 0.1)},
              "left_steering_carrier": {"mass_kg": 0.05, "com_m": [0, 0, 0], "inertia": _inertia(1e-5, 1e-5, 1e-5)},
              "right_steering_carrier": {"mass_kg": 0.05, "com_m": [0, 0, 0], "inertia": _inertia(1e-5, 1e-5, 1e-5)}}
    colliders = {}
    for w in WHEELS:
        bodies[w] = {"mass_kg": 0.55, "com_m": [0, 0, 0], "inertia": _inertia(0.002, 0.004, 0.002)}
        colliders[f"/World/Forklift/{w}/tire"] = {
            "type": "Cylinder", "radius": 0.125, "height": 0.08, "axis": "Y", "in_body_center_m": [0, 0, 0],
            "in_body_min_m": [-0.125, -0.04, -0.125], "in_body_max_m": [0.125, 0.04, 0.125], "body_scale": [1.0, 1.0, 1.0],
            "contact_offset": 0.002, "rest_offset": 0.0,
            "material": {"path": "/World/TyreMaterial", "static_friction": 0.8, "dynamic_friction": 0.7}}
    F = "/World/Forklift/"
    joints = {"lift": {"kind": "prismatic", "body0": [F + "base_link"], "body1": [F + "fork_carriage"],
                       "local_pos0": [0.74, 0.0, 0.025], "local_pos1": [0, 0, 0], "local_rot0": Q0, "local_rot1": Q0}}
    for side, y in (("left", 0.265), ("right", -0.265)):
        joints[f"{side}_steer"] = {"lower_deg": -15.0, "upper_deg": 15.0, "axis": "Z", "armature": 0.0,
                                   "body0": [F + "base_link"], "body1": [F + f"{side}_steering_carrier"],
                                   "local_pos0": [0.41, y, 0.0], "local_pos1": [0, 0, 0], "local_rot0": Q0, "local_rot1": Q0,
                                   "drive": {"type": "force", "stiffness": 1000.0, "damping": 100.0, "max_force": 20.0}}
    for w in ("front_left", "front_right", "rear_left", "rear_right"):
        y = 0.265 if "left" in w else -0.265
        parent = f"{'left' if 'left' in w else 'right'}_steering_carrier" if "front" in w else "base_link"
        joints[f"{w}_spin"] = {"axis": "Y", "body0": [F + parent], "body1": [F + f"{w}_wheel"],
                               "local_pos0": [0.0, 0.0, 0.0] if "front" in w else [-0.25, y, 0.0], "local_pos1": [0, 0, 0],
                               "local_rot0": Q0, "local_rot1": Q0,
                               "drive": {"type": "force", "stiffness": 0.0, "damping": 20.0, "max_force": 3.0}}
    return {"engine": {"bodies": bodies, "errors": {}}, "colliders": colliders, "joints": joints,
            "articulation": {"solver_position_iterations": 32, "solver_velocity_iterations": 8},
            "settled": {"floor_fz_n": [120, 29, 26, 80]}}


URDF_MODEL = {"inertials": {n: {"mass_kg": b["mass_kg"], "com_m": b["com_m"],
                                "inertia": np.asarray(b["inertia"]).reshape(3, 3).tolist()}
                            for n, b in _full_dump()["engine"]["bodies"].items()},
              "limits": {"left_steer": (-15.0, 15.0), "right_steer": (-15.0, 15.0)}}


def test_a_symmetric_full_dump_has_no_mismatch_even_with_unequal_loads():
    out = _compare().check(_full_dump(), URDF_MODEL, SETTINGS)
    assert out["verdict"] == "no_mismatch_in_scope", (out["crossing"], out["undecided"])
    assert out["settled"]["floor_fz_n"] == [120, 29, 26, 80]  # results are never a defect


@pytest.mark.parametrize("mutate, item", [
    (lambda d: d["engine"]["bodies"]["front_right_wheel"].update(mass_kg=0.60), "mass front_left_wheel~front_right_wheel"),
    (lambda d: d["colliders"]["/World/Forklift/front_right_wheel/tire"].update(in_body_center_m=[0, 0, -0.001]),
     "tyre centre in body mirror front_left_wheel~front_right_wheel (m)"),
    (lambda d: [j["drive"].update(stiffness=100000.0) for n, j in d["joints"].items() if "steer" in n],
     "drive stiffness left_steer vs settings"),  # both sides wrong alike
    (lambda d: d["joints"]["right_steer"].update(axis="X"), "joint axis left_steer~right_steer"),
    (lambda d: d["joints"]["right_steer"].update(local_rot0=[0.7071, 0, 0, 0.7071]), "joint local_rot0 left_steer~right_steer"),
    (lambda d: d["joints"]["lift"].update(local_pos0=[0.74, 0.03, 0.025]), "composite com |y| at zero pose (m)"),
    (lambda d: d["colliders"]["/World/Forklift/front_right_wheel/tire"].update(
        in_body_min_m=[-0.1375, -0.044, -0.1375], in_body_max_m=[0.1375, 0.044, 0.1375]),
     "tyre extent in body front_left_wheel~front_right_wheel (m)"),  # scaled 1.1 with the raw radius unchanged
    (lambda d: d["colliders"]["/World/Forklift/front_right_wheel/tire"].update(body_scale=[1.0, 1.1, 1.0]),
     "wheel body scale front_right_wheel vs 1"),
    (lambda d: [c["material"].update(static_friction=0.1) for c in d["colliders"].values()],
     "tyre static_friction front_left_wheel vs settings"),  # all four wrong alike
    (lambda d: d["joints"]["right_steer"]["drive"].update(type="acceleration"), "drive type right_steer vs settings"),
    (lambda d: d["colliders"]["/World/Forklift/rear_right_wheel/tire"]["material"].update(static_friction=0.5),
     "tyre material rear_left_wheel~rear_right_wheel"),
    (lambda d: d["articulation"].update(solver_position_iterations=128), "solver iterations vs settings"),
])
def test_each_cause_is_caught(mutate, item):
    d = _full_dump()
    mutate(d)
    out = _compare().check(d, URDF_MODEL, SETTINGS)
    assert out["verdict"] == "defect" and item in {i["item"] for i in out["crossing"]}, out["crossing"]


def test_missing_or_non_finite_engine_values_are_undecided():
    d = _full_dump()
    d["engine"]["bodies"]["base_link"]["inertia"] = [float("nan")] * 9
    out = _compare().check(d, URDF_MODEL, SETTINGS)
    assert out["verdict"] == "undecided" and any("base_link" in u for u in out["undecided"])
    d = _full_dump()
    d["engine"] = {"bodies": {}, "errors": {"body_names": "RuntimeError"}}
    assert _compare().check(d, URDF_MODEL, SETTINGS)["verdict"] == "undecided"


def test_the_merged_carriage_is_checked_against_the_arm_record_with_its_flat_inertia():
    d = _full_dump()
    d["engine"]["bodies"]["fork_carriage"] = {"mass_kg": 6.6, "com_m": [0.3, 0, 0.1], "inertia": _inertia(0.5, 0.6, 0.7)}
    plain = _compare().check(d, URDF_MODEL, SETTINGS)
    assert "mass fork_carriage vs expected" in {i["item"] for i in plain["crossing"]}
    merged = _compare().check(d, URDF_MODEL, SETTINGS,
                              {"mass_kg": 6.6, "com_m": [0.3, 0, 0.1], "inertia": _inertia(0.5, 0.6, 0.7)})
    assert merged["verdict"] == "no_mismatch_in_scope", merged["crossing"]
