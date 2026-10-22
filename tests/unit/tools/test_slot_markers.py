"""Slot markers for the open delivery zones (docs/plans/2026-10-22-open-zone-docking.md D3): per unloading slot two
plates behind it, 1.50 m to either side of its axis, in the planner's props and Isaac's visuals of a `_m` copy; the
scenes without markers stay as they were."""

import json
import math
import sys
from pathlib import Path

import pytest

from forklift_core.planning import Footprint, Pose2D
from forklift_core.planning.geometry import Rectangle, collision_free_pose
from forklift_core.planning.pallet_mission import PalletSite

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from tools import riibotics_isaac_scenes as tool  # noqa: E402


def test_each_slot_gets_two_plates_behind_it_on_either_side():
    site = PalletSite(3.0, -8.946, 0.0)
    plates = tool.slot_markers([site])
    assert len(plates) == 2
    for r in plates:
        assert r.length_m == pytest.approx(0.90) and r.width_m == pytest.approx(0.03)
        assert r.x_m == pytest.approx(3.0 + 0.30 + 0.05 + 0.45)  # far edge + 0.05, centre of a 0.90 m plate
        assert r.yaw_rad == pytest.approx(0.0)
    assert sorted(round(r.y_m - site.y_m, 6) for r in plates) == [-1.5, 1.5]


def test_plates_turn_with_the_slot():
    site = PalletSite(1.0, 2.0, math.pi / 2)  # entering along +y
    plates = tool.slot_markers([site])
    for r in plates:
        assert r.y_m == pytest.approx(2.0 + 0.80) and r.yaw_rad == pytest.approx(math.pi / 2)
    assert sorted(round(r.x_m, 6) for r in plates) == [-0.5, 2.5]


def test_plates_of_neighbouring_slots_closer_than_0_3_m_merge():
    a, b = PalletSite(3.4, 0.0, 0.0), PalletSite(3.4, -2.8, 0.0)  # a's right plate at -1.5, b's left plate at -1.3
    plates = tool.slot_markers([a, b])
    assert len(plates) == 3
    assert any(r.y_m == pytest.approx(-1.4) for r in plates)


def test_marked_copies_keep_the_originals_and_add_the_same_plates_to_props_and_visuals(tmp_path):
    built = tool.scenes()
    plain = tool.scene_record("line_3", built["line_3"])
    marked = tool.scene_record("line_3_m", tool.with_slot_markers("line", built["line_3"]))
    assert plain == tool.scene_record("line_3", tool.scenes()["line_3"])  # the original is unchanged
    props = [p for p in marked["scenario"]["props"] if p["asset"]["uri"] == tool.MARKER_URI]
    visuals = [v for v in marked["visuals"] if v.get("role") == "slot_marker"]
    assert len(props) == len(visuals) == 6  # three unloading slots, two plates each
    for p, v in zip(sorted(props, key=lambda p: (p["rectangle"]["x_m"], p["rectangle"]["y_m"])),
                    sorted(visuals, key=lambda v: (v["x_m"], v["y_m"]))):
        assert (p["rectangle"]["x_m"], p["rectangle"]["y_m"]) == pytest.approx((v["x_m"], v["y_m"]))
        assert v["kind"] == "wall" and v["height_m"] == pytest.approx(tool.MARKER_H_M) == p["asset"]["height_m"]
    assert marked["name"] == "line_3_m" and marked["source"]["slot_markers"] is True
    path = tmp_path / "line_3_m.json"
    path.write_text(json.dumps(marked))
    from scene_file import load_scene

    assert sum(v.kind == "wall" for v in load_scene(path).visuals) == 6


@pytest.mark.parametrize("name", ["line_1", "line_2", "line_3", "random_1", "random_2", "random_3"])
def test_plates_stay_clear_of_every_slot_and_inside_the_hall(name):
    kind = name.split("_")[0]
    record = tool.scene_record(f"{name}_m", tool.with_slot_markers(kind, tool.scenes()[name]))
    plates = [p["rectangle"] for p in record["scenario"]["props"] if p["asset"]["uri"] == tool.MARKER_URI]
    slots = tool.unloading_slots_hall(kind, tool.scenes()[name])
    for site in slots:
        pallet = Rectangle(site.x_m, site.y_m, 0.60, 0.80, site.yaw_rad)
        # the pallet standing in the slot and the truck's forks reaching its far edge touch no plate
        assert collision_free_pose(Pose2D(site.x_m, site.y_m, site.yaw_rad), [Rectangle(**r) for r in plates],
                                   Footprint(0.30, 0.30, 0.40), tool.HALL), (name, site, pallet)
    for r in plates:
        assert tool.HALL.x_min_m < r["x_m"] < tool.HALL.x_max_m and tool.HALL.y_min_m < r["y_m"] < tool.HALL.y_max_m


def test_only_scenes_with_unloading_slots_get_markers():
    assert set(tool.MARKED_KINDS) == {"line", "random"}
    with pytest.raises(ValueError):
        tool.with_slot_markers("trailer", tool.scenes()["trailer"])
