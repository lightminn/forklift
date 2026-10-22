"""Random factory floors for fleet tests: three structures, seeded.

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W0). A seed gives a
WorldMap: planning bounds, static obstacles, pallet slots (sources and sinks,
each a pallet centre and the heading the forks enter along), holding places
where trucks start and wait, and labelled zones for drawing.

- ``rack``: rack blocks whose rows run along x or y, pick aisles with floor
  pallets at the rack faces, main aisles around the blocks (some blocks reach
  a wall, so their aisles are dead ends), docks on one wall.
- ``shop``: a column grid with machines scattered over it, each with an
  output place and one or two input places, a raw-material store and docks.
- ``multi``: two or three halls of the other two kinds side by side or in an
  L, joined by doors in a partition wall or by a one-lane corridor.

Every dimension is drawn from a range written here (Claude's choices, not a
plant drawing); ``scale`` stretches the halls' ranges (not the doors,
corridors, aisles or the things in a hall, of which a bigger hall holds more). Everything is axis-aligned. Candidates that fail the checks --
a pallet or its run-up on an obstacle, a place the truck cannot reach on a
coarse grid -- are dropped and counted in ``WorldMap.dropped``.
Metres and radians; x east, y north, origin at the south-west inner corner.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
from math import cos, pi, sin

import numpy as np

from .geometry import Bounds, Pose2D, Rectangle
from .pallet_mission import PalletSite, SyntheticMissionGeometry

STRUCTURES = ("rack", "shop", "multi")
WALL_M = 0.3
BOUND_INSET_M = 0.4  # planning bounds inside the wall faces
PALLET_KEEP_M = 0.3  # slot keep-out around a pallet (covers pose error)
TRUCK_GAP_M = 0.6  # two traffic margins: a waiting truck's keep-out
RUN_UP_M = 2.0  # clear floor behind the pre-alignment pose
HOLD_FRONT_M = 1.5  # holding pose (rear axle) to the bound it faces
HOLD_STRAIGHT_M = 2.2  # straight run out of and into a holding place
HOLD_PITCH_M = 1.6
DOCK_PITCH_M = (2.4, 3.2)
GRID_M = 0.5
# Pallet to the rack, machine or stand behind it. The loaded truck reaches
# the pallet's far edge and plans with 0.10 m clearance, so 0.1 m failed
# every extraction (first CPU smoke, 2026-10-09).
BACK_GAP_M = 0.25
EAST, NORTH, WEST, SOUTH = 0.0, pi / 2, pi, -pi / 2


@dataclass(frozen=True)
class Box:
    """Axis-aligned rectangle by its corners."""

    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def w(self) -> float:
        return self.x1 - self.x0

    @property
    def h(self) -> float:
        return self.y1 - self.y0

    def rect(self) -> Rectangle:
        return Rectangle(
            (self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2, self.w, self.h, 0.0
        )

    def grow(self, d: float) -> Box:
        return Box(self.x0 - d, self.y0 - d, self.x1 + d, self.y1 + d)

    def overlaps(self, other: Box) -> bool:
        return (
            self.x0 < other.x1
            and other.x0 < self.x1
            and self.y0 < other.y1
            and other.y0 < self.y1
        )

    def inside(self, other: Box) -> bool:
        return (
            other.x0 <= self.x0
            and self.x1 <= other.x1
            and other.y0 <= self.y0
            and self.y1 <= other.y1
        )


@dataclass(frozen=True)
class Slot:
    name: str
    role: str  # "source" (pallets appear) or "sink" (pallets are delivered)
    site: PalletSite  # pallet centre; yaw = the heading the forks enter along
    zone: str


@dataclass(frozen=True)
class WorldMap:
    structure: str
    seed: int
    size_m: tuple[float, float]
    bounds: Bounds
    obstacles: tuple[tuple[str, Box], ...]  # (kind, box)
    slots: tuple[Slot, ...]
    holding: tuple[Pose2D, ...]
    zones: tuple[tuple[str, Box], ...]
    doors: tuple[Box, ...]
    dropped: dict = field(default_factory=dict)

    def obstacle_rects(self) -> list[Rectangle]:
        return [b.rect() for _, b in self.obstacles]

    def sources(self) -> list[Slot]:
        return [s for s in self.slots if s.role == "source"]

    def sinks(self) -> list[Slot]:
        return [s for s in self.slots if s.role == "sink"]


def pallet_box(site: PalletSite, geometry: SyntheticMissionGeometry) -> Box:
    """The pallet's footprint (axis-aligned headings only)."""
    along = abs(cos(site.yaw_rad)) > 0.5
    dx, dy = (
        (geometry.pallet_depth_m, geometry.pallet_width_m)
        if along
        else (geometry.pallet_width_m, geometry.pallet_depth_m)
    )
    return Box(site.x_m - dx / 2, site.y_m - dy / 2, site.x_m + dx / 2, site.y_m + dy / 2)


def run_up_box(site: PalletSite, geometry: SyntheticMissionGeometry) -> Box:
    """Floor a truck needs clear to line up on the slot and insert."""
    near = geometry.pallet_depth_m / 2
    far = geometry.prealign_offset_m + geometry.loaded_footprint.rear_m + RUN_UP_M
    half = geometry.loaded_footprint.half_width_m + PALLET_KEEP_M
    c, s = cos(site.yaw_rad), sin(site.yaw_rad)
    xs = [site.x_m - c * near, site.x_m - c * far]
    ys = [site.y_m - s * near, site.y_m - s * far]
    if abs(c) > 0.5:
        return Box(min(xs), site.y_m - half, max(xs), site.y_m + half)
    return Box(site.x_m - half, min(ys), site.x_m + half, max(ys))


def holding_box(pose: Pose2D, geometry: SyntheticMissionGeometry, grow: float) -> Box:
    """An unloaded truck at a holding place, grown by ``grow``."""
    fp = geometry.unloaded_footprint
    c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
    xs = [pose.x_m + c * fp.front_m, pose.x_m - c * fp.rear_m]
    ys = [pose.y_m + s * fp.front_m, pose.y_m - s * fp.rear_m]
    half = fp.half_width_m
    if abs(c) > 0.5:
        return Box(min(xs), pose.y_m - half, max(xs), pose.y_m + half).grow(grow)
    return Box(pose.x_m - half, min(ys), pose.x_m + half, max(ys)).grow(grow)


def holding_exit(pose: Pose2D, distance_m: float = HOLD_STRAIGHT_M) -> Pose2D:
    """Where a truck parked at pose (facing its wall) is after backing out."""
    return Pose2D(
        pose.x_m - distance_m * cos(pose.yaw_rad),
        pose.y_m - distance_m * sin(pose.yaw_rad),
        pose.yaw_rad,
    )


# -- building blocks -------------------------------------------------------------


class _Floor:
    """What a generator has placed so far."""

    def __init__(self, inner: Box):
        self.inner = inner  # planning bounds
        self.obstacles: list[tuple[str, Box]] = []
        self.slots: list[tuple[str, PalletSite, str]] = []  # role, site, zone
        self.holding: list[Pose2D] = []
        self.zones: list[tuple[str, Box]] = []
        self.doors: list[Box] = []
        self.keep_clear: list[Box] = []

    def free(self, box: Box) -> bool:
        return box.inside(self.inner) and not any(
            box.overlaps(b) for _, b in self.obstacles
        ) and not any(box.overlaps(k) for k in self.keep_clear)

    def add(self, kind: str, box: Box) -> None:
        self.obstacles.append((kind, box))


def _hall_walls(floor: _Floor, hall: Box, gaps: dict[str, list[tuple[float, float]]]):
    """Walls just outside a hall, with door gaps (along-wall intervals) per side."""
    t = WALL_M
    sides = {
        "south": (hall.x0 - t, hall.x1 + t, hall.y0 - t, hall.y0, "x"),
        "north": (hall.x0 - t, hall.x1 + t, hall.y1, hall.y1 + t, "x"),
        "west": (hall.x0 - t, hall.x0, hall.y0, hall.y1, "y"),
        "east": (hall.x1, hall.x1 + t, hall.y0, hall.y1, "y"),
    }
    for side, (x0, x1, y0, y1, axis) in sides.items():
        lo, hi = (x0, x1) if axis == "x" else (y0, y1)
        cuts = sorted(gaps.get(side, []))
        start = lo
        for a, b in cuts + [(hi, hi)]:
            if a > start + 0.05:
                if axis == "x":
                    floor.add("wall", Box(start, y0, a, y1))
                else:
                    floor.add("wall", Box(x0, start, x1, a))
            start = max(start, b)
        for a, b in cuts:
            floor.doors.append(Box(a, y0, b, y1) if axis == "x" else Box(x0, a, x1, b))


def _band(hall: Box, side: str, depth: float) -> Box:
    """The strip of a hall along one side."""
    if side == "south":
        return Box(hall.x0, hall.y0, hall.x1, hall.y0 + depth)
    if side == "north":
        return Box(hall.x0, hall.y1 - depth, hall.x1, hall.y1)
    if side == "west":
        return Box(hall.x0, hall.y0, hall.x0 + depth, hall.y1)
    return Box(hall.x1 - depth, hall.y0, hall.x1, hall.y1)


_FACING = {"south": SOUTH, "north": NORTH, "west": WEST, "east": EAST}


def _docks(rng, floor: _Floor, hall: Box, side: str, geometry, zone: str) -> None:
    """Drop places along a wall: forks enter towards the wall."""
    inner = _clip(hall, floor.inner)
    yaw = _FACING[side]
    lo, hi = (inner.x0, inner.x1) if side in ("south", "north") else (inner.y0, inner.y1)
    span = rng.uniform(0.35, 0.7) * (hi - lo)
    start = rng.uniform(lo + 3.0, max(lo + 3.0, hi - span - 3.0))
    pitch = rng.uniform(*DOCK_PITCH_M)
    depth = 0.3 + geometry.pallet_depth_m / 2
    along = start
    while along <= start + span:
        if side == "south":
            site = PalletSite(along, inner.y0 + depth, yaw)
        elif side == "north":
            site = PalletSite(along, inner.y1 - depth, yaw)
        elif side == "west":
            site = PalletSite(inner.x0 + depth, along, yaw)
        else:
            site = PalletSite(inner.x1 - depth, along, yaw)
        floor.slots.append(("sink", site, zone))
        along += pitch
    floor.zones.append((zone, _band(inner, side, 7.0)))
    floor.keep_clear.append(_band(inner, side, 9.0))


def _parking(rng, floor: _Floor, hall: Box, side: str, count: int, zone: str) -> None:
    """A row of holding places facing a wall (drive in forwards, back out)."""
    inner = _clip(hall, floor.inner)
    yaw = _FACING[side]
    lo, hi = (inner.x0, inner.x1) if side in ("south", "north") else (inner.y0, inner.y1)
    length = min(count * HOLD_PITCH_M, hi - lo - 6.0)
    start = rng.uniform(lo + 3.0, hi - length - 3.0)
    n = int(length // HOLD_PITCH_M)
    for i in range(n):
        along = start + (i + 0.5) * HOLD_PITCH_M
        if side == "south":
            pose = Pose2D(along, inner.y0 + HOLD_FRONT_M, yaw)
        elif side == "north":
            pose = Pose2D(along, inner.y1 - HOLD_FRONT_M, yaw)
        elif side == "west":
            pose = Pose2D(inner.x0 + HOLD_FRONT_M, along, yaw)
        else:
            pose = Pose2D(inner.x1 - HOLD_FRONT_M, along, yaw)
        floor.holding.append(pose)
    a, b = start, start + n * HOLD_PITCH_M
    band = _band(inner, side, 5.5)
    row = (
        Box(a, band.y0, b, band.y1)
        if side in ("south", "north")
        else Box(band.x0, a, band.x1, b)
    )
    floor.zones.append((zone, row))
    # The turn out of the row needs open floor in front of it.
    front = _band(inner, side, 11.0)
    floor.keep_clear.append(
        Box(a - 2.0, front.y0, b + 2.0, front.y1)
        if side in ("south", "north")
        else Box(front.x0, a - 2.0, front.x1, b + 2.0)
    )


def _clip(a: Box, b: Box) -> Box:
    return Box(max(a.x0, b.x0), max(a.y0, b.y0), min(a.x1, b.x1), min(a.y1, b.y1))


def _rack_block(rng, floor: _Floor, block: Box, zone: str, geometry) -> None:
    """Rack rows with pick aisles between them; floor pallets at the faces."""
    along_x = bool(rng.random() < 0.5)
    across_lo, across_hi = (block.y0, block.y1) if along_x else (block.x0, block.x1)
    along_lo, along_hi = (block.x0, block.x1) if along_x else (block.y0, block.y1)
    aisle = rng.uniform(7.0, 8.5)
    depth = float(rng.choice([1.2, 2.4]))
    n = int((across_hi - across_lo + aisle) // (depth + aisle))
    if n < 1:
        return
    used = n * depth + (n - 1) * aisle
    pos = across_lo + (across_hi - across_lo - used) / 2
    split = rng.random() < 0.35 and along_hi - along_lo > 24.0
    cut = rng.uniform(along_lo + 8.0, along_hi - 12.0) if split else None
    pitch = rng.uniform(1.8, 2.4)
    for _ in range(n):
        lo, hi = pos, pos + depth
        pieces = [(along_lo, along_hi)] if cut is None else [
            (along_lo, cut),
            (cut + rng.uniform(4.5, 6.0), along_hi),
        ]
        for a, b in [q for piece in pieces for q in _trim(floor, piece, lo, hi, along_x)]:
            box = Box(a, lo, b, hi) if along_x else Box(lo, a, hi, b)
            floor.add("rack", box)
            # Floor pallets in front of both faces, forks entering towards the rack.
            for face, normal in ((lo, -1.0), (hi, 1.0)):
                if rng.random() > 0.75:
                    continue
                centre = face + normal * (BACK_GAP_M + geometry.pallet_depth_m / 2)
                s = a + 1.0
                while s < b - 1.0:
                    if rng.random() < 0.6:
                        site = (
                            PalletSite(s, centre, NORTH if normal < 0 else SOUTH)
                            if along_x
                            else PalletSite(centre, s, EAST if normal < 0 else WEST)
                        )
                        floor.slots.append(("source", site, zone))
                    s += pitch
        pos += depth + aisle


def _trim(floor: _Floor, piece, lo: float, hi: float, along_x: bool, gap: float = 1.0):
    """Parts (at least 4 m) of a rack piece left after cutting out kept-clear areas."""
    parts = [piece]
    for k in floor.keep_clear:
        k_lo, k_hi = (k.y0, k.y1) if along_x else (k.x0, k.x1)
        if k_hi <= lo or hi <= k_lo:
            continue
        a_cut, b_cut = ((k.x0, k.x1) if along_x else (k.y0, k.y1))
        a_cut, b_cut = a_cut - gap, b_cut + gap
        out = []
        for a, b in parts:
            if b <= a_cut or b_cut <= a:
                out.append((a, b))
                continue
            if a_cut - a >= 4.0:
                out.append((a, a_cut))
            if b - b_cut >= 4.0:
                out.append((b_cut, b))
        parts = out
    return [(a, b) for a, b in parts if b - a >= 4.0]


def _rack_hall(rng, floor: _Floor, hall: Box, zone: str, geometry) -> None:
    inner = _clip(hall, floor.inner)
    main = rng.uniform(6.0, 8.0)
    # Main aisle round the blocks on some sides; elsewhere blocks reach the wall.
    pad = {
        s: (main if rng.random() < 0.65 else 0.6)
        for s in ("south", "north", "west", "east")
    }
    for side, depth in _reserved_depths(floor, inner).items():
        pad[side] = max(pad[side], depth + main)
    area = Box(
        inner.x0 + pad["west"],
        inner.y0 + pad["south"],
        inner.x1 - pad["east"],
        inner.y1 - pad["north"],
    )
    if area.w < 12 or area.h < 12:
        return
    bx = max(1, min(3, int(rng.integers(1, 4)) if area.w > 40 else 1))
    by = max(1, min(2, int(rng.integers(1, 3)) if area.h > 32 else 1))
    widths = _split(rng, area.x0, area.x1, bx, main)
    heights = _split(rng, area.y0, area.y1, by, main)
    for i, (x0, x1) in enumerate(widths):
        for j, (y0, y1) in enumerate(heights):
            _rack_block(
                rng, floor, Box(x0, y0, x1, y1), f"{zone} 블록 {i + 1}-{j + 1}", geometry
            )


def _reserved_depths(floor: _Floor, inner: Box) -> dict[str, float]:
    """How deep the kept-clear areas reach in from each side of a hall."""
    depths = {}
    for k in floor.keep_clear:
        if not k.overlaps(inner):
            continue
        if k.w < 0.6 * inner.w and k.h < 0.6 * inner.h:
            continue  # partial: the racks are trimmed round it instead
        if abs(k.y0 - inner.y0) < 0.5:
            depths["south"] = max(depths.get("south", 0.0), k.y1 - inner.y0)
        if abs(k.y1 - inner.y1) < 0.5:
            depths["north"] = max(depths.get("north", 0.0), inner.y1 - k.y0)
        if abs(k.x0 - inner.x0) < 0.5:
            depths["west"] = max(depths.get("west", 0.0), k.x1 - inner.x0)
        if abs(k.x1 - inner.x1) < 0.5:
            depths["east"] = max(depths.get("east", 0.0), inner.x1 - k.x0)
    return {s: d for s, d in depths.items() if d < 0.5 * min(inner.w, inner.h)}


def _split(rng, lo: float, hi: float, n: int, gap: float) -> list[tuple[float, float]]:
    """n intervals over [lo, hi] separated by gaps, sizes jittered."""
    if n == 1:
        return [(lo, hi)]
    weights = rng.uniform(0.7, 1.3, n)
    usable = hi - lo - gap * (n - 1)
    sizes = usable * weights / weights.sum()
    out, x = [], lo
    for size in sizes:
        out.append((x, x + size))
        x += size + gap
    return out


def _shop_hall(rng, floor: _Floor, hall: Box, zone: str, geometry) -> None:
    inner = _clip(hall, floor.inner)
    cell = rng.uniform(15.0, 19.0)
    nx, ny = int(inner.w // cell), int(inner.h // cell)
    if nx < 1 or ny < 1:
        return
    ox = inner.x0 + (inner.w - nx * cell) / 2
    oy = inner.y0 + (inner.h - ny * cell) / 2
    # Columns at the cell corners (structural, they sit in the corridors).
    for i in range(nx + 1):
        for j in range(ny + 1):
            cx, cy = ox + i * cell, oy + j * cell
            col = Box(cx - 0.3, cy - 0.3, cx + 0.3, cy + 0.3)
            if floor.free(col.grow(0.5)):
                floor.add("column", col)
    store = (int(rng.integers(nx)), int(rng.integers(ny)))
    for i in range(nx):
        for j in range(ny):
            x0, y0 = ox + i * cell, oy + j * cell
            if (i, j) == store:
                _store(rng, floor, Box(x0 + 3.5, y0 + 3.5, x0 + cell - 3.5, y0 + cell - 3.5), f"{zone} 원자재", geometry)
                continue
            if rng.random() > 0.8:
                continue
            w = rng.uniform(3.0, min(9.0, cell - 7.5))
            h = rng.uniform(3.0, min(9.0, cell - 7.5))
            mx = rng.uniform(x0 + 3.6, x0 + cell - 3.6 - w)
            my = rng.uniform(y0 + 3.6, y0 + cell - 3.6 - h)
            box = Box(mx, my, mx + w, my + h)
            if not floor.free(box.grow(0.2)):
                continue
            floor.add("machine", box)
            sides = list(rng.permutation(["south", "north", "west", "east"]))
            for k, side in enumerate(sides[: int(rng.integers(2, 4))]):
                role = "source" if k == 0 else "sink"
                floor.slots.append((role, _face_site(rng, box, side, geometry), f"{zone} 기계"))


def _face_site(rng, box: Box, side: str, geometry) -> PalletSite:
    """A floor pallet in front of a machine side, forks entering towards it."""
    off = BACK_GAP_M + geometry.pallet_depth_m / 2
    if side in ("south", "north"):
        x = rng.uniform(box.x0 + 0.6, box.x1 - 0.6)
        return PalletSite(x, box.y0 - off, NORTH) if side == "south" else PalletSite(x, box.y1 + off, SOUTH)
    y = rng.uniform(box.y0 + 0.6, box.y1 - 0.6)
    return PalletSite(box.x0 - off, y, EAST) if side == "west" else PalletSite(box.x1 + off, y, WEST)


def _store(rng, floor: _Floor, area: Box, zone: str, geometry) -> None:
    """Block-stacked raw material: a back-to-back double row of pallet places."""
    if area.w < 4 or area.h < 4 or not floor.free(area):
        return
    along_x = area.w >= area.h
    mid = (area.y0 + area.y1) / 2 if along_x else (area.x0 + area.x1) / 2
    floor.add("store", Box(area.x0, mid - 0.35, area.x1, mid + 0.35) if along_x else Box(mid - 0.35, area.y0, mid + 0.35, area.y1))
    pitch = rng.uniform(1.8, 2.2)
    lo, hi = (area.x0, area.x1) if along_x else (area.y0, area.y1)
    s = lo + 0.8
    while s < hi - 0.8:
        for normal in (-1.0, 1.0):
            c = mid + normal * (0.35 + BACK_GAP_M + geometry.pallet_depth_m / 2)
            if along_x:
                site = PalletSite(s, c, NORTH if normal < 0 else SOUTH)
            else:
                site = PalletSite(c, s, EAST if normal < 0 else WEST)
            floor.slots.append(("source", site, zone))
        s += pitch
    floor.zones.append((zone, area))


# -- structures ------------------------------------------------------------------


def _single(rng, structure: str, geometry, trucks: int, scale: float):
    w = scale * (rng.uniform(70.0, 110.0) if structure == "rack" else rng.uniform(60.0, 100.0))
    h = scale * (rng.uniform(45.0, 70.0) if structure == "rack" else rng.uniform(50.0, 80.0))
    hall = Box(0.0, 0.0, w, h)
    floor = _Floor(hall.grow(-BOUND_INSET_M))
    _hall_walls(floor, hall, {})
    sides = list(rng.permutation(["south", "north", "west", "east"]))
    _docks(rng, floor, hall, sides[0], geometry, "출하장")
    _parking(rng, floor, hall, sides[1], trucks + 4, "대기 1")
    if trucks > 8:
        _parking(rng, floor, hall, sides[2], trucks // 2 + 2, "대기 2")
    floor.zones.append(("공장동", hall))
    if structure == "rack":
        _rack_hall(rng, floor, hall, "랙", geometry)
    else:
        _shop_hall(rng, floor, hall, "공정", geometry)
    return floor, (w, h)


def _multi(rng, geometry, trucks: int, scale: float):
    shape = str(rng.choice(["line", "line3", "L"]))
    kinds = list(rng.permutation(["rack", "shop"]))
    if shape == "line3":
        kinds.append(str(rng.choice(["rack", "shop"])))
    halls, joins = [], []  # joins: (hall a, hall b, axis, coordinate, overlap lo, hi, connector length)
    x, base_h = 0.0, scale * rng.uniform(42.0, 58.0)
    if shape in ("line", "line3"):
        for k, kind in enumerate(kinds):
            w = scale * rng.uniform(38.0, 55.0)
            h = base_h * rng.uniform(0.8, 1.15)
            y0 = rng.uniform(0.0, max(0.0, base_h * 1.15 - h)) if k else 0.0
            halls.append((kind, Box(x, y0, x + w, y0 + h)))
            gap = WALL_M if rng.random() < 0.5 else rng.uniform(10.0, 18.0)
            x += w + gap
            if k + 1 < len(kinds):
                joins.append((k, k + 1, gap))
    else:
        w1, h1 = scale * rng.uniform(55.0, 80.0), scale * rng.uniform(38.0, 50.0)
        halls.append((kinds[0], Box(0.0, 0.0, w1, h1)))
        w2, h2 = scale * rng.uniform(36.0, 48.0), scale * rng.uniform(34.0, 48.0)
        gap = WALL_M if rng.random() < 0.5 else rng.uniform(10.0, 18.0)
        halls.append((kinds[1], Box(0.0, h1 + gap, w2, h1 + gap + h2)))
        joins.append((0, 1, gap))
    xs = [b.x1 for _, b in halls] + [b.x0 for _, b in halls]
    ys = [b.y1 for _, b in halls] + [b.y0 for _, b in halls]
    world = Box(min(xs), min(ys), max(xs), max(ys))
    floor = _Floor(world.grow(-BOUND_INSET_M))
    gaps: dict[int, dict[str, list[tuple[float, float]]]] = {k: {} for k in range(len(halls))}
    corridors = []
    for a, b, gap in joins:
        ha, hb = halls[a][1], halls[b][1]
        vertical = shape == "L"  # b is north of a
        lo = max(ha.x0, hb.x0) if vertical else max(ha.y0, hb.y0)
        hi = min(ha.x1, hb.x1) if vertical else min(ha.y1, hb.y1)
        if gap <= WALL_M + 1e-9:
            # Partition with one or two doors.
            for _ in range(int(rng.integers(1, 3))):
                width = rng.uniform(5.0, 8.0)
                c = rng.uniform(lo + width / 2 + 2.0, hi - width / 2 - 2.0)
                cut = (c - width / 2, c + width / 2)
                if vertical:
                    gaps[a].setdefault("north", []).append(cut)
                    gaps[b].setdefault("south", []).append(cut)
                    corridors.append(Box(cut[0], ha.y1, cut[1], hb.y0))
                    floor.keep_clear += [Box(cut[0] - 1, ha.y1 - 8.0, cut[1] + 1, ha.y1), Box(cut[0] - 1, hb.y0, cut[1] + 1, hb.y0 + 8.0)]
                else:
                    gaps[a].setdefault("east", []).append(cut)
                    gaps[b].setdefault("west", []).append(cut)
                    corridors.append(Box(ha.x1, cut[0], hb.x0, cut[1]))
                    floor.keep_clear += [Box(ha.x1 - 8.0, cut[0] - 1, ha.x1, cut[1] + 1), Box(hb.x0, cut[0] - 1, hb.x0 + 8.0, cut[1] + 1)]
        else:
            # A one-lane corridor across the gap; the rest of the gap is solid.
            width = rng.uniform(4.0, 4.5)
            c = rng.uniform(lo + width / 2 + 3.0, hi - width / 2 - 3.0)
            cut = (c - width / 2, c + width / 2)
            if vertical:
                gaps[a].setdefault("north", []).append(cut)
                gaps[b].setdefault("south", []).append(cut)
                corridor = Box(cut[0], ha.y1, cut[1], hb.y0)
                floor.keep_clear += [Box(cut[0] - 2, ha.y1 - 10.0, cut[1] + 2, ha.y1), Box(cut[0] - 2, hb.y0, cut[1] + 2, hb.y0 + 10.0)]
            else:
                gaps[a].setdefault("east", []).append(cut)
                gaps[b].setdefault("west", []).append(cut)
                corridor = Box(ha.x1, cut[0], hb.x0, cut[1])
                floor.keep_clear += [Box(ha.x1 - 10.0, cut[0] - 2, ha.x1, cut[1] + 2), Box(hb.x0, cut[0] - 2, hb.x0 + 10.0, cut[1] + 2)]
            corridors.append(corridor)
            floor.zones.append(("한 차로 연결 통로", corridor))
    for k, (_, hall) in enumerate(halls):
        _hall_walls(floor, hall, gaps[k])
    # Everything inside the world box that is neither a hall nor a corridor is solid.
    for box in _complement(world, [h for _, h in halls] + corridors):
        floor.add("outside", box)
    outer = {k: _outer_sides(hall, world) for k, (_, hall) in enumerate(halls)}
    dock_hall = int(rng.integers(len(halls)))
    _docks(rng, floor, halls[dock_hall][1], str(rng.choice(outer[dock_hall])), geometry, "출하장")
    park_halls = [k for k in range(len(halls)) if k != dock_hall] or [dock_hall]
    for n, k in enumerate(park_halls[:2]):
        sides = [s for s in outer[k]]
        _parking(rng, floor, halls[k][1], str(rng.choice(sides)), (trucks + 4) // len(park_halls[:2]) + 2, f"대기 {n + 1}")
    for k, (kind, hall) in enumerate(halls):
        name = f"{k + 1}동 {'랙' if kind == 'rack' else '공정'}"
        floor.zones.append((name, hall))
        if kind == "rack":
            _rack_hall(rng, floor, hall, name, geometry)
        else:
            _shop_hall(rng, floor, hall, name, geometry)
    return floor, (world.w, world.h)


def _outer_sides(hall: Box, world: Box) -> list[str]:
    sides = []
    if abs(hall.y0 - world.y0) < 1e-6:
        sides.append("south")
    if abs(hall.y1 - world.y1) < 1e-6:
        sides.append("north")
    if abs(hall.x0 - world.x0) < 1e-6:
        sides.append("west")
    if abs(hall.x1 - world.x1) < 1e-6:
        sides.append("east")
    return sides or ["south"]


def _complement(world: Box, holes: list[Box]) -> list[Box]:
    """Axis-aligned boxes covering world minus the holes (grid decomposition)."""
    xs = sorted({world.x0, world.x1, *[h.x0 for h in holes], *[h.x1 for h in holes]})
    ys = sorted({world.y0, world.y1, *[h.y0 for h in holes], *[h.y1 for h in holes]})
    out = []
    for x0, x1 in zip(xs[:-1], xs[1:], strict=True):
        for y0, y1 in zip(ys[:-1], ys[1:], strict=True):
            cell = Box(x0, y0, x1, y1)
            if cell.w < 1e-6 or cell.h < 1e-6:
                continue
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            if not any(h.x0 < cx < h.x1 and h.y0 < cy < h.y1 for h in holes):
                out.append(cell)
    return out


# -- checks ----------------------------------------------------------------------


def _reachable(
    inner: Box, blocked_boxes: list[Box], grow: float, points: list, seed_point
) -> list[bool]:
    """Coarse-grid connectivity for a disc of radius ``grow``."""
    nx, ny = int(np.ceil(inner.w / GRID_M)), int(np.ceil(inner.h / GRID_M))
    blocked = np.zeros((nx, ny), dtype=bool)
    for b in blocked_boxes:
        g = b.grow(grow)
        i0 = max(int(np.floor((g.x0 - inner.x0) / GRID_M)), 0)
        i1 = min(int(np.ceil((g.x1 - inner.x0) / GRID_M)), nx)
        j0 = max(int(np.floor((g.y0 - inner.y0) / GRID_M)), 0)
        j1 = min(int(np.ceil((g.y1 - inner.y0) / GRID_M)), ny)
        if i0 < i1 and j0 < j1:
            blocked[i0:i1, j0:j1] = True

    def cell(p):
        return (
            int(np.clip((p[0] - inner.x0) / GRID_M, 0, nx - 1)),
            int(np.clip((p[1] - inner.y0) / GRID_M, 0, ny - 1)),
        )

    seen = np.zeros_like(blocked)
    start = cell(seed_point)
    if blocked[start]:
        return [False] * len(points)
    queue = deque([start])
    seen[start] = True
    while queue:
        i, j = queue.popleft()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < nx and 0 <= b < ny and not blocked[a, b] and not seen[a, b]:
                seen[a, b] = True
                queue.append((a, b))
    return [bool(seen[cell(p)]) for p in points]


def make_world(
    structure: str,
    seed: int,
    *,
    geometry: SyntheticMissionGeometry,
    trucks: int = 10,
    scale: float = 1.0,
) -> WorldMap:
    """One random factory floor of the given structure for this seed; ``scale``
    stretches the halls (1: the floors every earlier run used)."""
    if structure not in STRUCTURES:
        raise ValueError(f"structure must be one of {STRUCTURES}")
    rng = np.random.default_rng([seed, STRUCTURES.index(structure), 0x574F524C])
    if structure == "multi":
        floor, size = _multi(rng, geometry, trucks, scale)
    else:
        floor, size = _single(rng, structure, geometry, trucks, scale)
    inner = floor.inner
    walls = [b for _, b in floor.obstacles]
    dropped: Counter = Counter()

    def clear(box: Box) -> bool:
        return box.inside(inner) and not any(box.overlaps(b) for b in walls)

    # Holding places: the parked body and the straight run out, on clear floor.
    holding = []
    for pose in floor.holding:
        a = holding_box(pose, geometry, 0.1)
        b = holding_box(holding_exit(pose), geometry, 0.1)
        run = Box(min(a.x0, b.x0), min(a.y0, b.y0), max(a.x1, b.x1), max(a.y1, b.y1))
        if clear(run):
            holding.append(pose)
        else:
            dropped["holding_blocked"] += 1
    holding_keep = [holding_box(p, geometry, TRUCK_GAP_M) for p in holding]
    # Pallet places: pallet and run-up clear of obstacles, holding places and
    # each other's keep-outs (the keep-outs are planning obstacles later).
    kept: list[tuple[str, PalletSite, str]] = []
    for role, site, zone in floor.slots:
        pallet = pallet_box(site, geometry)
        run = run_up_box(site, geometry)
        keep = pallet.grow(PALLET_KEEP_M)
        if not clear(pallet.grow(0.05)):
            dropped["slot_on_obstacle"] += 1
        elif not clear(run):
            dropped["slot_run_up_blocked"] += 1
        elif any(keep.overlaps(h) or run.overlaps(h) for h in holding_keep):
            dropped["slot_on_holding"] += 1
        elif any(
            keep.overlaps(pallet_box(s, geometry).grow(PALLET_KEEP_M))
            or run.overlaps(pallet_box(s, geometry).grow(PALLET_KEEP_M))
            or keep.overlaps(run_up_box(s, geometry))
            for _, s, _ in kept
        ):
            dropped["slot_crowded"] += 1
        else:
            kept.append((role, site, zone))
    if not holding:
        raise ValueError(f"{structure} seed {seed}: no holding place survived")
    # Everything reachable from the first holding place on a coarse grid that
    # also blocks every pallet keep-out and every other holding place.
    blocked = walls + [pallet_box(s, geometry).grow(PALLET_KEEP_M) for _, s, _ in kept]
    blocked += [holding_box(p, geometry, 0.1) for p in holding]
    grow = geometry.loaded_footprint.half_width_m + 0.1
    start = holding_exit(holding[0])
    exits = [holding_exit(h) for h in holding]
    ok = _reachable(inner, blocked, grow, [(e.x_m, e.y_m) for e in exits], (start.x_m, start.y_m))
    dropped["holding_unreachable"] += ok.count(False)
    holding = [h for h, good in zip(holding, ok, strict=True) if good]
    lined_up = [
        (
            s.x_m - geometry.prealign_offset_m * cos(s.yaw_rad),
            s.y_m - geometry.prealign_offset_m * sin(s.yaw_rad),
        )
        for _, s, _ in kept
    ]
    ok = _reachable(inner, blocked, grow, lined_up, (start.x_m, start.y_m))
    dropped["slot_unreachable"] += ok.count(False)
    kept = [k for k, good in zip(kept, ok, strict=True) if good]
    counts = Counter(role for role, _, _ in kept)
    if counts["source"] < 2 or counts["sink"] < 2:
        raise ValueError(f"{structure} seed {seed}: too few slots {dict(counts)}")
    names = Counter()
    slots = []
    for role, site, zone in kept:
        names[role] += 1
        slots.append(Slot(f"{'S' if role == 'source' else 'D'}{names[role]}", role, site, zone))
    return WorldMap(
        structure=structure,
        seed=seed,
        size_m=(round(size[0], 2), round(size[1], 2)),
        bounds=Bounds(inner.x0, inner.x1, inner.y0, inner.y1),
        obstacles=tuple(floor.obstacles),
        slots=tuple(slots),
        holding=tuple(holding),
        zones=tuple(floor.zones),
        doors=tuple(floor.doors),
        dropped={k: v for k, v in dropped.items() if v},
    )


def world_record(world: WorldMap, geometry: SyntheticMissionGeometry, extra=()) -> dict:
    """JSON-ready description of a floor (and boxes added during a run), for drawing."""
    return {
        "structure": world.structure,
        "seed": world.seed,
        "size_m": list(world.size_m),
        "bounds": [
            world.bounds.x_min_m,
            world.bounds.x_max_m,
            world.bounds.y_min_m,
            world.bounds.y_max_m,
        ],
        "obstacles": [[k, b.x0, b.y0, b.x1, b.y1] for k, b in world.obstacles],
        "extra": [[k, b.x0, b.y0, b.x1, b.y1] for k, b in extra],
        "slots": [
            [s.name, s.role, s.site.x_m, s.site.y_m, s.site.yaw_rad, s.zone]
            for s in world.slots
        ],
        "holding": [[p.x_m, p.y_m, p.yaw_rad] for p in world.holding],
        "zones": [[n, b.x0, b.y0, b.x1, b.y1] for n, b in world.zones],
        "doors": [[b.x0, b.y0, b.x1, b.y1] for b in world.doors],
        "footprints": {
            "unloaded": [
                geometry.unloaded_footprint.front_m,
                geometry.unloaded_footprint.rear_m,
                geometry.unloaded_footprint.half_width_m,
            ],
            "loaded": [
                geometry.loaded_footprint.front_m,
                geometry.loaded_footprint.rear_m,
                geometry.loaded_footprint.half_width_m,
            ],
            "pallet": [geometry.pallet_depth_m, geometry.pallet_width_m],
        },
    }
