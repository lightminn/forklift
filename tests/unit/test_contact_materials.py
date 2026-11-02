"""Contact materials by part, step A (docs/plans/2026-11-01-contact-materials.md, sim/isaac/contact_materials.py)."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import contact_materials as cm  # noqa: E402


def test_the_split_is_off_unless_the_settings_name_it():
    assert not cm.split_enabled({"static_friction": 0.8})
    assert cm.split_enabled({cm.SPLIT_KEY: 1})


def test_truck_colliders_take_their_part_from_the_path():
    base = "/World/Forklift"
    assert cm.truck_role(f"{base}/rear_left_wheel/collisions/rear_left_tire_collision") == "tyre"
    assert cm.truck_role(f"{base}/fork_carriage/collisions/left_fork_collision") == "fork"
    assert cm.truck_role(f"{base}/fork_carriage/collisions/right_fork_heel_collision/mesh_0") == "fork"
    for chassis in ("fork_carriage/collisions/carriage_cross_0_collision", "base_link/collisions/rear_hood_collision",
                    "base_link/collisions/mast_rail_1_collision"):
        assert cm.truck_role(f"{base}/{chassis}") == "chassis"


def _stage(tyre_material=False):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    stage = Usd.Stage.CreateInMemory()
    contact = UsdShade.Material.Define(stage, cm.SOURCE_PATH)
    api = UsdPhysics.MaterialAPI.Apply(contact.GetPrim())
    api.CreateStaticFrictionAttr(0.8)
    api.CreateDynamicFrictionAttr(0.7)
    api.CreateRestitutionAttr(0.0)
    tyre = None
    if tyre_material:
        tyre = UsdShade.Material.Define(stage, "/World/TyreContactMaterial")
        UsdPhysics.MaterialAPI.Apply(tyre.GetPrim()).CreateStaticFrictionAttr(0.8)
    paths = ["/World/Forklift/rear_left_wheel/collisions/rear_left_tire_collision",
             "/World/Forklift/fork_carriage/collisions/left_fork_collision",
             "/World/Forklift/fork_carriage/collisions/carriage_cross_0_collision",
             "/World/Forklift/base_link/collisions/rear_hood_collision",
             "/World/Pallet/collisions/block_0", "/World/Pallet/collisions/block_1",
             "/World/Environment/Root/GroundPlane/CollisionPlane", "/World/Environment/SM_floor28/SM_floor02"]
    # a render material with no physics API, bound for all purposes, as the warehouse's floor tiles are
    render = UsdShade.Material.Define(stage, "/World/Environment/Looks/MI_Floor_02b")
    for path in paths:
        prim = UsdGeom.Cube.Define(stage, path).GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        if "SM_floor" in path:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(render)
        if path.startswith(("/World/Forklift", "/World/Pallet")):
            material = tyre if (tyre is not None and "_tire_" in path) else contact
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
    return stage


def test_every_part_gets_a_copy_with_the_same_values_and_the_floor_stays_unbound():
    stage = _stage()
    record = cm.split_contact_materials(stage)
    assert record["colliders"] == {"tyre": 1, "chassis": 2, "fork": 1, "pallet": 2}
    assert record["kept_on_other_material"] == {}
    for m in record["materials"].values():
        assert m["physics:staticFriction"] == pytest.approx(0.8) and m["physics:dynamicFriction"] == pytest.approx(0.7)
        assert m["physics:restitution"] == 0.0 and "physxMaterial:frictionCombineMode" not in m  # average, as before
    assert cm._bound_path(stage.GetPrimAtPath(
        "/World/Forklift/fork_carriage/collisions/left_fork_collision")) == f"{cm.ROOT}/Fork"
    # the render-bound floor tile counts as unbound: it carries no physics material
    assert cm.unbound_colliders(stage) == {"/World/Environment": 2}
    inventory = cm.material_inventory(stage)
    assert inventory["colliders"][f"{cm.ROOT}/Fork"] == {"/World/Forklift": 1}
    assert inventory["colliders"][f"{cm.ROOT}/Pallet"] == {"/World/Pallet": 2}


def test_a_diagnostic_tyre_material_is_left_where_it_is():
    stage = _stage(tyre_material=True)
    record = cm.split_contact_materials(stage)
    assert record["colliders"]["tyre"] == 0 and record["kept_on_other_material"] == {"/World/TyreContactMaterial": 1}


def test_a_stage_without_the_contact_material_is_refused():
    pytest.importorskip("pxr")
    from pxr import Usd

    with pytest.raises(ValueError, match="no /World/ContactMaterial"):
        cm.split_contact_materials(Usd.Stage.CreateInMemory())


def test_the_fork_override_goes_together_with_the_split_and_keeps_dynamic_at_most_static():
    assert cm.fork_override({cm.SPLIT_KEY: 1}) is None
    ok = {cm.SPLIT_KEY: 1, "fork_static_friction": 0.5714, "fork_dynamic_friction": 0.5, "fork_friction_combine_min": 1}
    assert cm.fork_override(ok) == {"static_friction": 0.5714, "dynamic_friction": 0.5, "combine": "min"}
    for bad in ({k: v for k, v in ok.items() if k != cm.SPLIT_KEY},
                {k: v for k, v in ok.items() if k != "fork_friction_combine_min"},
                {**ok, "fork_dynamic_friction": 0.6}):
        with pytest.raises(ValueError):
            cm.fork_override(bad)


def test_the_fork_override_gives_only_the_fork_copy_its_values_and_the_min_rule():
    stage = _stage()
    fork = {"static_friction": 0.3429, "dynamic_friction": 0.3, "combine": "min"}
    record = cm.split_contact_materials(stage, fork=fork)
    m = record["materials"]
    assert m["fork"]["physics:staticFriction"] == pytest.approx(0.3429)
    assert m["fork"]["physics:dynamicFriction"] == pytest.approx(0.3)
    assert m["fork"]["physxMaterial:frictionCombineMode"] == "min"
    for role in ("tyre", "chassis", "pallet"):
        assert m[role]["physics:staticFriction"] == pytest.approx(0.8)
        assert "physxMaterial:frictionCombineMode" not in m[role]
