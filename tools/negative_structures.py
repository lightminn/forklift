"""Non-pallet structures for the redefined negative population N2 (docs/plans/2026-10-27-bottomless-and-bundled-pallets.md §4.2).

Pallets without a bottom deck are pick targets now (user decision 2026-10-27), so the negatives are things that are
not pallets. Most of them are made to look like one from the entry face -- the hard lookalikes -- because a
population of easy boxes would make the false-positive bound say nothing (plan §4.2). Every structure is built in the
pallet frame of ``measure_pocket_evidence.structure`` (centre at the origin, x along the depth, z up from the floor)
from a pallet geometry, so it shares that pallet's face dimensions; ``rng`` draws the free parameters.

    shelf           measure_pocket_evidence's shelf: full-depth columns to the lane ceiling, a solid deck floating a
                    stringer's height above them (a box-model artefact kept for continuity with the old N set)
    shelf_closed    the same with the columns up to the deck: no gap, taller lanes
    shelf_rail      columns to the lane ceiling, a front and a back rail at stringer height, a solid deck: from the
                    entry face it shows the pallet's bands (columns, stringer, top)
    shelf_raised    the closed shelf on a full plate 10-30 mm thick: the lane floor raised
    shelf_legs      legs in three rows (a bottomless pallet's blocks) under one solid deck, no stringer
    shelf_uprights  the rail shelf with rack uprights at the corners up to 1.15 m (through the LiDAR plane) and an
                    upper level
    box_stand       a closed box of the pallet's footprint on 4, 6 or 9 legs
    block_pile      four to eight free blocks of random size, on the floor
    blocks          nine free blocks of the pallet's block size, on the floor
"""

from __future__ import annotations

import math

import numpy as np

from forklift_core.perception.pallet_geometry import PalletGeometry
from tools.measure_pocket_evidence import structure
from tools.scene_rig import Box, boxes_interpenetrate

LIDAR_PLANE_M = 1.05  # RPLIDAR plane above the floor (forklift_core.safety.pallet_permit)
KINDS = ("shelf", "shelf_closed", "shelf_rail", "shelf_raised", "shelf_legs", "shelf_uprights", "box_stand",
         "block_pile", "blocks")
# "crates" (a stack of crates) is drawn by tools/recognition_scenes itself, as in the old N set.
HARD = ("shelf", "shelf_closed", "shelf_rail", "shelf_raised", "shelf_legs", "shelf_uprights", "box_stand")
# The N2 composition, by a scene's rank inside its split (17 slots: 14 hard lookalikes, 3 easy negatives).
N2_CYCLE = ("shelf_rail", "shelf_legs", "shelf_closed", "box_stand", "shelf", "shelf_rail", "shelf_raised",
            "block_pile", "shelf_legs", "shelf_uprights", "box_stand", "shelf_rail", "crates", "shelf_closed",
            "shelf_raised", "blocks", "shelf_legs")
RAIL_DEPTH_M = 0.02


def _columns(g: PalletGeometry, top_m: float, bottom_m: float = 0.0) -> list[Box]:
    h = top_m - bottom_m
    return [Box((0.0, y, bottom_m + h / 2), (g.overall_depth_m, w, h))
            for y, w in zip(g.block_centres_y_m(), g.block_widths_m, strict=True)]


def _deck(g: PalletGeometry, bottom_m: float) -> Box:
    t = g.top_board_thickness_m
    return Box((0.0, 0.0, bottom_m + t / 2), (g.overall_depth_m, g.overall_width_m, t))


def build(g: PalletGeometry, kind: str, rng: np.random.Generator) -> list[Box]:
    height, top = g.overall_height_m, g.top_board_thickness_m
    ceiling = g.deck_bottom_m + g.block_height_m  # the pallet's lane ceiling
    deck_bottom = height - top
    if kind == "shelf":
        return structure(g, kind)
    if kind == "blocks":
        # measure_pocket_evidence's "blocks" keep their pallet height (z from deck_bottom up): they float a bottom
        # board's thickness above the floor. Here the nine blocks stand on the floor.
        return [Box((x, y, g.block_height_m / 2), (g.block_depth_m, w, g.block_height_m))
                for x in g.block_centres_x_m()
                for y, w in zip(g.block_centres_y_m(), g.block_widths_m, strict=True)]
    if kind == "shelf_closed":
        return _columns(g, deck_bottom) + [_deck(g, deck_bottom)]
    if kind in ("shelf_rail", "shelf_uprights"):
        rail_h = deck_bottom - ceiling
        rails = [Box((sign * (g.overall_depth_m - RAIL_DEPTH_M) / 2, 0.0, ceiling + rail_h / 2),
                     (RAIL_DEPTH_M, g.overall_width_m, rail_h)) for sign in (-1, 1)]
        boxes = _columns(g, ceiling) + rails + [_deck(g, deck_bottom)]
        if kind == "shelf_uprights":
            u, tall = 0.04, 1.15
            for sx in (-1, 1):
                for sy in (-1, 1):
                    boxes.append(Box((sx * (g.overall_depth_m - u) / 2, sy * (g.overall_width_m + u) / 2, tall / 2),
                                     (u, u, tall)))
            level = 0.55
            boxes.append(Box((0.0, 0.0, level + top / 2), (g.overall_depth_m, g.overall_width_m, top)))
        return boxes
    if kind == "shelf_raised":
        plate = float(rng.uniform(0.010, 0.030))
        return ([Box((0.0, 0.0, plate / 2), (g.overall_depth_m, g.overall_width_m, plate))]
                + _columns(g, deck_bottom, plate) + [_deck(g, deck_bottom)])
    if kind == "shelf_legs":
        legs = [Box((x, y, deck_bottom / 2), (g.block_depth_m, w, deck_bottom))
                for x in g.block_centres_x_m()
                for y, w in zip(g.block_centres_y_m(), g.block_widths_m, strict=True)]
        return legs + [_deck(g, deck_bottom)]
    if kind == "box_stand":
        nx, ny = (2, 2) if rng.random() < 1 / 3 else ((2, 3) if rng.random() < 0.5 else (3, 3))
        s = float(rng.uniform(0.05, 0.09))
        leg_h = float(rng.uniform(ceiling, 1.5 * ceiling))
        box_h = float(rng.uniform(0.20, 0.45))
        xs = np.linspace(-(g.overall_depth_m - s) / 2, (g.overall_depth_m - s) / 2, nx)
        ys = np.linspace(-(g.overall_width_m - s) / 2, (g.overall_width_m - s) / 2, ny)
        legs = [Box((float(x), float(y), leg_h / 2), (s, s, leg_h)) for x in xs for y in ys]
        return legs + [Box((0.0, 0.0, leg_h + box_h / 2), (g.overall_depth_m, g.overall_width_m, box_h))]
    if kind == "block_pile":
        n = int(rng.integers(4, 9))
        out: list[Box] = []
        for _ in range(200):
            if len(out) == n:
                break
            size = (float(rng.uniform(0.06, 0.20)), float(rng.uniform(0.06, 0.20)), float(rng.uniform(0.05, 0.15)))
            half = max(size[0], size[1]) / 2 * math.sqrt(2)
            cx = float(rng.uniform(-g.overall_depth_m / 2 + half, g.overall_depth_m / 2 - half))
            cy = float(rng.uniform(-g.overall_width_m / 2 + half, g.overall_width_m / 2 - half))
            cand = Box((cx, cy, size[2] / 2), size, float(rng.uniform(-math.pi, math.pi)))
            if not any(boxes_interpenetrate(cand, b) for b in out):
                out.append(cand)
        return out
    raise ValueError(f"unknown negative structure: {kind}")
