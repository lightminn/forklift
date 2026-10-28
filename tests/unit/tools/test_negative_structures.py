import math

import numpy as np
import pytest

from tools import negative_structures as ns
from tools import scene_rig
from tools.measure_pocket_evidence import structure
from tools.recognition_scenes import _geometry

G = _geometry("t11_06")
H, T = G.overall_height_m, G.top_board_thickness_m
CEIL = G.deck_bottom_m + G.block_height_m  # the pallet's lane ceiling


def _z(b):
    return b.centre_m[2] - b.size_m[2] / 2, b.centre_m[2] + b.size_m[2] / 2


def _covering(boxes, y, z):
    """Boxes whose y-z footprint holds (y, z) -- what a ray at that lateral offset and height meets."""
    return [b for b in boxes if abs(b.centre_m[1] - y) <= b.size_m[1] / 2 + 1e-9 and _z(b)[0] - 1e-9 <= z <= _z(b)[1] + 1e-9]


def test_every_kind_builds_and_sits_on_the_floor():
    rng = np.random.default_rng(0)
    for kind in ns.KINDS:
        boxes = ns.build(G, kind, rng)
        assert boxes, kind
        assert min(_z(b)[0] for b in boxes) == pytest.approx(0.0, abs=1e-9), kind


def test_the_closed_shelf_has_no_floating_deck():
    boxes = ns.build(G, "shelf_closed", np.random.default_rng(0))
    deck = max(boxes, key=lambda b: b.size_m[1])
    columns = [b for b in boxes if b is not deck]
    assert _z(deck)[0] == pytest.approx(H - T)
    assert all(_z(c)[1] == pytest.approx(H - T) for c in columns)
    # the legacy shelf leaves a stringer-height gap under its deck
    legacy = structure(G, "shelf")
    assert max(_z(b)[1] for b in legacy if b.size_m[1] < G.overall_width_m) == pytest.approx(CEIL)


def test_the_rail_shelf_shows_the_pallet_entry_face():
    # Across a fork lane, from the front: pallet and rail shelf both are solid from the lane ceiling to the top.
    lane_y = (G.block_centres_y_m()[0] + G.block_centres_y_m()[1]) / 2
    rail = ns.build(G, "shelf_rail", np.random.default_rng(0))
    pallet = structure(G, "pallet")
    # the stringer band: solid across the lane in both
    for z in (CEIL + 0.002, H - T - 0.002):
        assert _covering(rail, lane_y, z) and _covering(pallet, lane_y, z), z
    # the top band: the rail shelf's deck is solid (the pallet's top boards have slots between them)
    assert _covering(rail, lane_y, H - 0.002)
    # and the lane below the ceiling is open in both
    assert not _covering(rail, lane_y, CEIL / 2)


def test_the_raised_shelf_closes_the_lane_floor():
    boxes = ns.build(G, "shelf_raised", np.random.default_rng(1))
    lane_y = (G.block_centres_y_m()[0] + G.block_centres_y_m()[1]) / 2
    plate = _covering(boxes, lane_y, 0.001)
    assert plate and 0.010 <= _z(plate[0])[1] <= 0.030


def test_the_legged_shelf_has_three_rows_of_legs_and_a_solid_deck():
    boxes = ns.build(G, "shelf_legs", np.random.default_rng(0))
    legs = [b for b in boxes if _z(b)[0] == pytest.approx(0.0)]
    assert len(legs) == 9
    assert len({round(b.centre_m[0], 6) for b in legs}) == 3
    deck = [b for b in boxes if b not in legs]
    assert len(deck) == 1 and deck[0].size_m[1] == pytest.approx(G.overall_width_m)


def test_the_rack_uprights_cross_the_lidar_plane():
    boxes = ns.build(G, "shelf_uprights", np.random.default_rng(0))
    assert max(_z(b)[1] for b in boxes) > ns.LIDAR_PLANE_M


def test_the_box_stand_has_four_to_nine_legs_under_a_closed_box():
    for seed in range(20):
        boxes = ns.build(G, "box_stand", np.random.default_rng(seed))
        top = max(boxes, key=lambda b: b.size_m[2])
        legs = [b for b in boxes if b is not top]
        assert 4 <= len(legs) <= 9
        assert _z(top)[0] == pytest.approx(max(_z(b)[1] for b in legs))
        assert top.size_m[0] == pytest.approx(G.overall_depth_m) and top.size_m[1] == pytest.approx(G.overall_width_m)


def test_a_block_pile_is_irregular_and_inside_the_footprint():
    a = ns.build(G, "block_pile", np.random.default_rng(3))
    b = ns.build(G, "block_pile", np.random.default_rng(4))
    assert 4 <= len(a) <= 8 and a != b
    for box in a:
        assert abs(box.centre_m[0]) <= G.overall_depth_m / 2 and abs(box.centre_m[1]) <= G.overall_width_m / 2


def test_no_kind_interpenetrates_itself():
    rng = np.random.default_rng(7)
    for kind in ns.KINDS:
        boxes = ns.build(G, kind, rng)
        for i, p in enumerate(boxes):
            for q in boxes[i + 1:]:
                assert not scene_rig.boxes_interpenetrate(p, q), kind


def test_hard_lookalikes_are_at_least_two_thirds_of_the_cycle():
    hard = sum(1 for k in ns.N2_CYCLE if k in ns.HARD)
    assert hard / len(ns.N2_CYCLE) >= 2 / 3
    assert set(ns.N2_CYCLE) <= set(ns.KINDS) | {"crates"}
    assert math.isclose(len(ns.N2_CYCLE), 17)
