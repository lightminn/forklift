"""Rack copies of the narrow aisle (docs/plans/2026-10-22-riibotics-aisle.md, user decisions D-A A1 and D-C C1a): the
0.20 m back gap and a rack frame at every slot boundary, with the slot gap widened so the delivery and extraction
straights clear the posts; the plain aisle scenes stay as they were."""

import json
import math
import sys
from pathlib import Path

import pytest

from forklift_core.planning.pallet_mission import PalletSite, _straight_plan, site_poses

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from tools import riibotics_isaac_scenes as tool  # noqa: E402

PALLET = tool.load_pallet_geometry(tool.PALLETS[tool.PALLET])
GEOMETRY = tool.mission_geometry(tool.URDF, PALLET, tool.RESERVE_M, **tool.STRAIGHTS)


def record(name, built=None):
    return tool.scene_record(name, built if built is not None else tool.rack_scene())


def straights(rec, pickup_shift=0.0, drop_shift=0.0):
    sc = tool.scenario_from_record(rec["scenario"])
    rects = [p.rectangle for p in sc.props]
    pick = site_poses(PalletSite(sc.pickup.x_m, sc.pickup.y_m + pickup_shift, sc.pickup.yaw_rad), GEOMETRY)
    drop = site_poses(PalletSite(sc.destination.x_m, sc.destination.y_m + drop_shift, sc.destination.yaw_rad), GEOMETRY)
    c = tool.CLEARANCE_M
    return {
        # the scene planning's insertion is checked at the full clearance (no grid: pallet_mission.py, insert)
        "insert": _straight_plan(pick["approach"], pick["inserted"], 1, rects, GEOMETRY.unloaded_footprint, sc.bounds,
                                 c).status,
        "extract": _straight_plan(pick["inserted"], pick["extracted"], -1, rects, GEOMETRY.loaded_footprint, sc.bounds,
                                  c).status,
        "deliver": _straight_plan(drop["predelivery"], drop["delivery"], 1, rects, GEOMETRY.loaded_footprint, sc.bounds,
                                  c).status,
        "withdraw": _straight_plan(drop["delivery"], drop["withdrawn"], -1, rects, GEOMETRY.unloaded_footprint,
                                   sc.bounds, c).status,
    }


def uprights(rec):
    return [p for p in rec["scenario"]["props"] if p["asset"]["uri"] == tool.UPRIGHT_URI]


def test_every_slot_boundary_gets_a_front_and_a_back_upright_in_props_and_visuals():
    rec = record("aisle_rack")
    props = uprights(rec)
    visuals = [v for v in rec["visuals"] if v.get("role") == "rack_upright"]
    assert len(props) == len(visuals) == 2 * (tool.SLOTS + 1)
    for p, v in zip(sorted(props, key=lambda p: (p["rectangle"]["x_m"], p["rectangle"]["y_m"])),
                    sorted(visuals, key=lambda v: (v["x_m"], v["y_m"]))):
        assert (p["rectangle"]["x_m"], p["rectangle"]["y_m"]) == pytest.approx((v["x_m"], v["y_m"]))
        assert v["kind"] == "wall" and v["height_m"] == pytest.approx(tool.RACK_H_M) and v["height_m"] > 1.05  # LiDAR plane
        assert v["length_m"] == v["width_m"] == pytest.approx(tool.UPRIGHT_M)  # Isaac draws every post at its size
    assert rec["source"]["back_gap_m"] == pytest.approx(0.20) and rec["source"]["slot_gap_m"] == pytest.approx(0.58)


def test_the_back_upright_stands_against_the_back_frame_and_the_front_one_at_the_entry_face():
    rec = record("aisle_rack")
    bound = rec["scenario"]["bounds"]["x_max_m"]  # the hall's east edge: the rack back
    xs = sorted({round(p["rectangle"]["x_m"], 6) for p in uprights(rec)})
    assert xs[1] == pytest.approx(bound - tool.UPRIGHT_M / 2)
    nominal = rec["pickup_nominal"]["x_m"]
    assert xs[0] == pytest.approx(nominal - PALLET.overall_depth_m / 2 + tool.UPRIGHT_M / 2)


def test_the_posts_are_their_size_by_default_now_that_the_planner_keeps_out_of_them():
    # P1c replaced PR #106's scene keep-out: the rack copy's posts are their size in the planner's map too
    assert tool.RACK_KEEPOUT_M == 0.0
    sizes = {round(p["rectangle"]["length_m"], 6) for p in uprights(record("aisle_rack"))}
    assert sizes == {round(tool.UPRIGHT_M, 6)}


def test_the_pr106_keep_out_copy_still_widens_every_post_but_the_working_slots_frames():
    # kept for comparison: all but the four posts of the pickup and drop slots' frames widened by the keep-out
    rec = record("aisle_rack", tool.rack_scene(keepout=0.20))
    sizes = sorted(round(p["rectangle"]["length_m"], 6) for p in uprights(rec))
    plain, grown = tool.UPRIGHT_M, tool.UPRIGHT_M + 2 * 0.20
    assert sizes == [pytest.approx(plain)] * 8 + [pytest.approx(grown)] * (len(sizes) - 8)  # 2 slots x 2 edges x front/back
    lateral = lambda p: p["rectangle"]["y_m"]  # hall frame: along the aisle
    work = [rec["pickup_nominal"]["y_m"], rec["scenario"]["destination"]["y_m"]]
    pitch = PALLET.overall_width_m + tool.RACK_SLOT_GAP_M
    for p in uprights(rec):
        edge = any(abs(abs(lateral(p) - w) - pitch / 2) < 1e-6 for w in work)
        assert (p["rectangle"]["length_m"] == pytest.approx(plain)) == edge


def test_the_rack_copy_plans_its_whole_mission_with_the_planners_lidar_keepout(tmp_path):
    # the CPU check with the Isaac mission settings, as --plan --lidar-keepout 0.15 runs it (the launcher's default)
    path = tmp_path / "aisle_rack.json"
    path.write_text(json.dumps(record("aisle_rack")))
    tool.plan_scene(path, tmp_path, keepout_m=0.15)
    result = json.loads((tmp_path / "aisle_rack.keep0.15.plan.json").read_text())  # repr(0.15) == "0.15"
    assert result["outcome"] == "success" and result["lidar_keepout_m"] == 0.15


def test_the_plain_aisle_scenes_are_unchanged():
    for name in ("aisle", "aisle_out"):
        rec = tool.scene_record(name, tool.scenes()[name])
        assert not uprights(rec) and rec["source"]["back_gap_m"] == pytest.approx(0.15)
        assert rec["source"]["slot_gap_m"] == pytest.approx(0.20)


@pytest.mark.parametrize("name", ["aisle_rack", "aisle_out_rack"])
def test_the_rack_copies_pick_as_their_plain_scene_and_every_straight_clears_the_posts(name):
    rec = record(name)
    plain = tool.scene_record(name.removesuffix("_rack"), tool.scenes()[name.removesuffix("_rack")])
    offset = [rec["scenario"]["pickup"][k] - rec["pickup_nominal"][k] for k in ("x_m", "y_m")]
    assert offset == pytest.approx([plain["scenario"]["pickup"][k] - plain["pickup_nominal"][k] for k in ("x_m", "y_m")])
    assert set(straights(rec).values()) == {"success"}


@pytest.mark.parametrize("shift", [-0.06, 0.06])
def test_the_straights_keep_six_centimetres_either_side(shift):
    # an estimate off sideways (recognition at pickup, SLAM at delivery) still plans
    assert set(straights(record("aisle_rack"), pickup_shift=shift, drop_shift=shift).values()) == {"success"}


def test_posts_at_the_plain_slot_gap_block_the_loaded_straights():
    # the reason for the wider gap: pitch 0.80 + 0.20 m = the loaded footprint 0.80 m plus 2 x 0.10 m clearance
    rec = record("aisle_rack", tool.aisle_scene(slot_gap=0.20, back_gap=0.20, uprights=True))
    s = straights(rec)
    assert s["extract"] == "straight_collision" and s["deliver"] == "straight_collision"


def test_the_rack_back_gap_lets_the_aisle_extraction_through_where_the_plain_one_refuses():
    # r4's estimate (7 mm deeper than truth): the plain scene refuses the extraction, the rack copy plans it
    for name, built, expected in (("aisle", tool.scenes()["aisle"], "straight_collision"),
                                  ("aisle_rack", tool.rack_scene(), "success")):
        rec = tool.scene_record(name, built)
        sc = tool.scenario_from_record(rec["scenario"])
        site = PalletSite(sc.pickup.x_m + 0.00702, sc.pickup.y_m, 0.034556)
        p = site_poses(site, GEOMETRY)
        assert _straight_plan(p["inserted"], p["extracted"], -1, [q.rectangle for q in sc.props],
                              GEOMETRY.loaded_footprint, sc.bounds, tool.CLEARANCE_M).status == expected


def test_the_rack_copy_loads_as_a_scene_file(tmp_path):
    path = tmp_path / "aisle_rack.json"
    path.write_text(json.dumps(record("aisle_rack")))
    from scene_file import load_scene

    assert sum(v.kind == "wall" for v in load_scene(path).visuals) == 2 * (tool.SLOTS + 1) + 1  # posts + back frame


def _wall_segments(rec):
    segs = []
    for v in rec["visuals"]:
        if v["kind"] == "wall" and v["height_m"] > 1.05:
            c, s = math.cos(v["yaw_rad"]), math.sin(v["yaw_rad"])
            hl, hw = v["length_m"] / 2, v["width_m"] / 2
            pts = [(v["x_m"] + c * a - s * b, v["y_m"] + s * a + c * b) for a, b in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]
            segs += [(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    return segs


@pytest.mark.parametrize("name, built, reason", [("aisle_out", None, "start_rejected:degenerate"),
                                                 ("aisle_out_rack", "rack", "ok")])
def test_the_uprights_give_the_delivery_match_its_sideways_information(name, built, reason):
    # synthetic scans of what stands through the LiDAR plane (the base hall left out); SLAM 11 cm off sideways as in
    # ws1 8723: the plain aisle is refused as degenerate, the rack copy is taken and lands within millimetres
    import numpy as np

    from forklift_core.localization.scan_docking import dock, laser_points, ray_scan
    from forklift_core.localization.slam_pose import compose, invert

    laser_offset = (0.22, 0.0, 0.0)
    angles = -math.pi + np.arange(1600) * (2 * math.pi / 1600)
    rec = tool.scene_record(name, tool.rack_scene() if built else tool.scenes()[name])
    sc = tool.scenario_from_record(rec["scenario"])
    poses = site_poses(sc.destination, GEOMETRY)
    delivery = (poses["delivery"].x_m, poses["delivery"].y_m, poses["delivery"].yaw_rad)
    truth = (poses["predelivery"].x_m, poses["predelivery"].y_m, poses["predelivery"].yaw_rad)
    segs = _wall_segments(rec)

    def scan(rear, noise=0.0):
        laser = compose(rear, laser_offset)
        local = [(compose(invert(laser), (*a, 0.0))[:2], compose(invert(laser), (*b, 0.0))[:2]) for a, b in segs]
        ranges = ray_scan(local, (0.0, 0.0, 0.0), angles, 12.0)
        return laser_points(ranges + np.random.default_rng(1).normal(0, noise, ranges.shape), angles)

    estimate = compose(truth, (0.016, 0.111, 0.0013))
    result = dock(scan(delivery), scan(truth, 0.02), rear_from_laser=laser_offset, estimate_rear=estimate,
                  delivery_rear=delivery)
    assert result.reason == reason
    if result.accepted:
        expected = compose(estimate, compose(invert(truth), delivery))
        assert math.hypot(result.goal_estimate[0] - expected[0], result.goal_estimate[1] - expected[1]) < 0.01
