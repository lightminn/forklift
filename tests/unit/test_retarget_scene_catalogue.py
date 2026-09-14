"""Retargeting must move the shape and nothing else.

Re-drawing the scenes for a second pallet would change the population as well
as the geometry, and no downstream number could separate the two effects. So
the poses are asserted identical -- exactly, not approximately -- and each
recomputed field is checked against the rule that decides it.
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
V1 = ROOT / "sim/gazebo/scenes/catalogue_v1.yaml"
EPAL6_GEOMETRY = ROOT / "config/pallet_geometry_epal6.yaml"
T11_GEOMETRY = ROOT / "config/pallet_geometry_t11_06.yaml"


def _module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


retarget_module = _module("retarget_catalogue_under_test", "tools/retarget_scene_catalogue.py")
world_module = _module("scene_world_under_test", "sim/gazebo/build_scene_world.py")

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402


@pytest.fixture(scope="module")
def source():
    return yaml.safe_load(V1.read_text())


@pytest.fixture(scope="module")
def geometry():
    return load_pallet_geometry(EPAL6_GEOMETRY)


@pytest.fixture(scope="module")
def retargeted(source, geometry):
    return retarget_module.retarget(source, geometry)


def test_every_pose_is_copied_exactly(source, retargeted):
    """Exactly, not approximately: a rounded pose is a different scene."""
    assert len(retargeted["scenes"]) == len(source["scenes"])
    for before, after in zip(source["scenes"], retargeted["scenes"]):
        assert before["scene_id"] == after["scene_id"]
        for field in ("category", "split", "lighting", "surfaces", "distractors"):
            assert before[field] == after[field], (after["scene_id"], field)
        if before["pallet"] is None:
            assert after["pallet"] is None
            continue
        for axis in ("x_m", "y_m", "yaw_rad"):
            assert before["pallet"][axis] == after["pallet"][axis], (
                after["scene_id"],
                axis,
            )


def test_the_lookalike_placements_are_untouched(source, retargeted):
    for before, after in zip(source["scenes"], retargeted["scenes"]):
        assert before["lookalike"] == after["lookalike"], after["scene_id"]


def test_the_header_matches_what_the_world_generator_approves(retargeted):
    assert retargeted["catalogue_version"] == "epal6"
    assert retargeted["pallet"] == world_module.APPROVED_PALLETS["epal6"]
    # Split decks: no single symmetric thickness closes a real pallet.
    assert "deck_m" not in retargeted["pallet"]


def test_the_result_passes_the_world_generator_validator(retargeted, tmp_path):
    path = tmp_path / "catalogue_epal6.yaml"
    path.write_text(yaml.safe_dump(retargeted, sort_keys=False))
    loaded = world_module.load_catalogue(path)
    assert len(loaded["scenes"]) == len(retargeted["scenes"])


def test_one_opening_width_replaces_v1_drawn_spread(source, geometry, retargeted):
    drawn = {
        s["pallet"]["opening_width_m"] for s in source["scenes"] if s["pallet"]
    }
    assert len(drawn) > 50, "v1 is supposed to have drawn a wide spread"
    widths = {s["pallet"]["opening_width_m"] for s in retargeted["scenes"] if s["pallet"]}
    assert widths == {round(geometry.opening_width_m, 6)}
    assert retargeted["ranges"]["opening_width_m"] == [
        round(geometry.opening_width_m, 6)
    ] * 2


def test_the_pocket_height_moves_from_the_v1_constant(source, geometry, retargeted):
    for before, after in zip(source["scenes"], retargeted["scenes"]):
        if before["pallet"] is None:
            continue
        for side in ("left", "right"):
            assert before["ground_truth"][side]["center_m"][2] == 0.15
            assert after["ground_truth"][side]["center_m"][2] == pytest.approx(
                geometry.opening_centre_height_m, abs=1e-9
            )
            assert after["ground_truth"][side]["height_m"] == pytest.approx(
                geometry.block_height_m
            )


def test_the_pocket_x_moves_too_wherever_the_pallet_is_turned(source, retargeted):
    """The lateral offset differs, so a turned pallet puts its pockets elsewhere
    along x as well -- copying only y and z would be wrong."""
    moved = 0
    for before, after in zip(source["scenes"], retargeted["scenes"]):
        if before["pallet"] is None or abs(before["pallet"]["yaw_rad"]) < 1e-6:
            continue
        for side in ("left", "right"):
            if before["ground_truth"][side]["center_m"][0] != after["ground_truth"][
                side
            ]["center_m"][0]:
                moved += 1
    assert moved > 0, "no turned scene moved its pockets along x"


def test_the_pocket_centres_follow_the_stated_rule(geometry, retargeted):
    for scene in retargeted["scenes"]:
        if scene["pallet"] is None:
            continue
        pose = (
            scene["pallet"]["x_m"],
            scene["pallet"]["y_m"],
            scene["pallet"]["yaw_rad"],
        )
        left, right = retarget_module.pocket_centres(geometry, *pose)
        for side, expected in (("left", left), ("right", right)):
            got = scene["ground_truth"][side]["center_m"]
            assert got == pytest.approx(list(expected), abs=1e-6), scene["scene_id"]


def test_the_occluder_width_is_a_share_of_the_opening_not_of_the_image(
    geometry, retargeted
):
    seen = 0
    for scene in retargeted["scenes"]:
        occluder = scene["occluder"]
        if occluder is None:
            continue
        seen += 1
        assert occluder["size_m"][1] == pytest.approx(
            occluder["fraction"] * geometry.opening_width_m, abs=1e-6
        )
    assert seen > 0, "no occluded scene to check"


def test_visibility_is_recomputed_rather_than_carried_over(source, retargeted):
    changed = 0
    for before, after in zip(source["scenes"], retargeted["scenes"]):
        if before["pallet"] is None:
            continue
        if before["visibility"]["corners_px"] != after["visibility"]["corners_px"]:
            changed += 1
    assert changed == sum(1 for s in source["scenes"] if s["pallet"]), (
        "a shorter pallet must project different corners in every posed scene"
    )


def test_a_second_geometry_retargets_without_special_casing(source):
    """The tool takes a geometry, not a hardcoded second pallet."""
    t11 = load_pallet_geometry(T11_GEOMETRY)
    out = retarget_module.retarget(source, t11)
    assert out["catalogue_version"] == "t11_06"
    assert out["pallet"]["height_m"] == pytest.approx(0.090)
    widths = {s["pallet"]["opening_width_m"] for s in out["scenes"] if s["pallet"]}
    assert widths == {round(t11.opening_width_m, 6)}


def test_retargeting_is_idempotent_on_its_own_output(source, geometry, retargeted):
    again = retarget_module.retarget(retargeted, geometry)
    assert again == retargeted
