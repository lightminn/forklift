"""Scene files for run_transport.py --layout scene: the Riibotics demo scenarios in the south hall.

Riibotics sent five demo scenarios on 2026-10-19 (local reference, not in the repository). Four
are built here, each as one transport mission (pick one pallet, deliver it, return to the start):

  aisle    (1) narrow aisle: pick a floor pallet from the rack row, put it into another slot
  line     (3) three pallets in a column, carried one after another to the unloading column
  random   (4) five pallets placed loosely (yaw within 10 deg), three carried to the unloading row
  trailer  (5) a trailer walled except for its entrance: take the front pallet out
  conveyor (2) take the pallet at the end of conveyor A and put it at the end of conveyor B

The conveyor's height is not modelled: its pallets stand higher (0.37 m scaled) than the forks lift
(0.32 m), so the pallets stand on the floor at the conveyor's edge, in a cell cleared in it; the
rest of each conveyor is a 0.37 m box (below the LiDAR plane, as a real conveyor would be).
A sequence (line, random) is one scene per pallet: scene k holds the pallets still waiting and
the ones already delivered, and picks pallet k.

The aisle and the trailer come from riibotics_scenario_paths (scale 0.521 by length, EPAL 6,
pick and drop slots over the gaps between the blocks, one trailer column); line and random are
read off the scenario drawings, which are drawn at about our truck's scale (its 0.63 m width is
70 px, 9.0 mm/px). Every scene is turned so the forks enter the pickup along +x and moved so
its start stands where the original bay's does (y = 0) with its far edge at x = 4.6, inside
the empty south hall (config/factory_south_hall.yaml: hall x <= 4.7, y in [-22.9, 8.3]).

What the planner avoids is scenario.props (the scene's layout as a prior map: the slots,
blocks, walls and the pallets standing in them); what Isaac shows and collides with is
"visuals". The pickup pallet stands a few centimetres and degrees off its nominal slot (the random
scene's by the drawing's tilt); the pickup zone (the operator's "the pallet around here") is the
nominal slot +- 0.25 m, yaw within 12 deg, and the observation candidates face the nominal slot --
neither is built from where the pallet really stands. The runner plans on the zone before
recognition and on the estimate after it; the CPU check here does the same with the true pallet
standing in for the estimate.

    python tools/riibotics_isaac_scenes.py --output config/riibotics_scenes
    python tools/riibotics_isaac_scenes.py --output <dir> --plan [--only aisle] [--jobs 4]
    python tools/riibotics_isaac_scenes.py --output <dir> --slot-markers   # also <name>_m.json for line / random
    python tools/riibotics_isaac_scenes.py --output <dir> --rack           # also aisle_rack / aisle_out_rack

Slot markers (docs/plans/2026-10-22-open-zone-docking.md D3): the open unloading zones of line and random leave the
delivery docking match a single far wall, so it refuses (degenerate). A `_m` copy puts two plates behind every
unloading slot (slot_markers) -- in the planner's props and in what Isaac draws -- and the plain scene stays for
regression.

Rack copies (docs/plans/2026-10-22-riibotics-aisle.md, user decisions D-A A1 and D-C C1a): `--rack` also writes
aisle_rack / aisle_out_rack -- the 0.20 m back gap (assumed, the plain scenes keep 0.15 m) and a rack frame at every
slot boundary with the slot gap widened to clear it. The plain aisle scenes stay for regression.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import multiprocessing
import os
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "sim/isaac"))

from bay_planning_sweep import mission_geometry  # noqa: E402
from riibotics_scenario_paths import (  # noqa: E402
    PALLETS,
    SCALES,
    SLOTS,
    STAGES,
    UP,
    aisle_case,
    classify,
    gear_changes,
    pallet_prop,
    prop,
    trailer_case,
)

from scene_file import FORMAT, SESSION_FORMAT, scenario_from_record  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning import Pose2D  # noqa: E402
from forklift_core.planning.geometry import Bounds, Rectangle  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    AssetSpec,
    D7_PLANNER_OPTIONS,
    PalletSite,
    PlacedProp,
    TransportScenario,
    make_transport_planner_config,
    plan_observation_leg,
    plan_transport,
)

URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
PALLET = "epal6"
# The mission settings of the Isaac runs (test_runs/recog/recog.sbatch on ws1, 2026-10-19)
STRAIGHTS = {"alignment_straight_m": 2.1, "delivery_straight_m": 1.5, "withdrawal_m": 0.55}
# KAPPA: the planner curvature limit; --kappa sets it through the environment, so the spawned planners see it too
# (docs/plans/2026-10-24-curvature-response.md arm B)
RESERVE_M, KAPPA, CLEARANCE_M = 0.016, float(os.environ.get("FORKLIFT_PLAN_KAPPA", "0.29")), 0.10
REAR_TO_CAMERA_M = 0.34 + 0.619   # rear axle -> base_link, base_link -> carriage_low_measured
# Placement in the south hall
EAST_EDGE_M = 4.6
HALL = Bounds(-25.6, 4.7, -22.9, 8.3)
# Heights of what Isaac draws (scaled 0.521 from typical real sizes; all above the 1.05 m LiDAR plane)
WALL_H_M = 1.36          # trailer walls: 2.6 m inside height
RACK_H_M = 1.50          # rack back frame
CONVEYOR_H_M = round(0.70 * 1.46 / 2.8, 3)   # scenario 2: about 0.7 m, scaled by length (0.365 m)
CONVEYOR_BAND_M, CONVEYOR_BACK_M = 0.25, 0.25  # conveyor cleared beside / behind a pallet (fleet demo p4)
WALL_T_M = 0.08
ZONE_GROW_M = 0.25
ZONE_YAW_RAD = math.radians(12)
PX_M = 0.63 / 70         # drawing scale of scenarios 3 and 4 (truck width 70 px)
TRUCK_FRONT_M = 1.29     # rear axle to fork tip (measured chassis)


def _visual(kind: str, x: float, y: float, length: float, width: float, yaw: float = 0.0,
            height: float = 0.0, colour=(0.5, 0.5, 0.5)) -> dict:
    return {"kind": kind, "x_m": x, "y_m": y, "length_m": length, "width_m": width, "yaw_rad": yaw,
            "height_m": height, "colour": list(colour)}


ZONE_GREY = (0.62, 0.62, 0.60)
UNLOAD_BLUE = (0.35, 0.50, 0.70)
WALL_GREY = (0.70, 0.72, 0.75)
RACK_ORANGE = (0.85, 0.45, 0.12)


# Slot markers (plan 2026-10-22 open-zone docking D3): per unloading slot two plates parallel to its entry, from
# MARKER_BEHIND_M to MARKER_BEHIND_M + MARKER_L_M beyond its far edge (the pallet's front face), MARKER_SIDE_M to
# either side of its axis (halfway to a slot beside it in the same row when nearer: a plate 1.5 m out would stand
# 0.21 m off the neighbour's axis, inside its planning clearance); they cover the 1.05 m LiDAR plane by 0.25 m. Sized against the 13 refused
# delivery matches (ws1 test_runs/riibotics/marker_sizing.py): side panels beside the slot hide the far wall from
# the station's reference scan, plates nearer than 1.5 m to the axis left the middle line slot short of inliers.
MARKER_URI = "riibotics:slot_marker"
MARKER_L_M, MARKER_T_M, MARKER_H_M = 0.90, 0.03, 1.30
MARKER_BEHIND_M, MARKER_SIDE_M, MARKER_MERGE_M = 0.05, 1.50, 0.30
MARKER_COLOUR = (0.95, 0.80, 0.10)
SLOT_DEPTH_M = 0.60  # EPAL 6 along the forks: a slot's far edge is half of it ahead of its centre
MARKED_KINDS = ("line", "random")

# Rack copies of the aisle (plan 2026-10-22 riibotics-aisle, user decisions D-A A1 and D-C C1a): the back gap is
# assumed 0.20 m instead of 0.15 m (the plain scenes leave the truth extraction 2.8 mm to spare, so a few millimetres
# of recognition error decide it), and every slot boundary gets a rack frame -- an upright at the entry face and one
# at the back -- standing through the LiDAR plane. An upright is assumed 0.10 m square real (0.052 m scaled); the
# slot gap widens to RACK_SLOT_GAP_M because at the plain 0.20 m the slot pitch equals the loaded footprint plus
# its clearance, so a post of any thickness blocks the delivery and the extraction straights.
# Neither the gap, the post nor the back gap comes from Riibotics (questions in the plan, section 6).
UPRIGHT_URI = "riibotics:rack_upright"
UPRIGHT_M = round(0.10 * 1.46 / 2.8, 3)
RACK_BACK_GAP_M = 0.20
RACK_SLOT_GAP_M = 0.58
# The live obstacle layer marks a LiDAR hit with its error radius (config/obstacle_layer_single.yaml; a 0.052 m post
# reads as about 0.35-0.40 m of OCCUPIED cells -- ws1 riib_901632b3 runs), while the planner keeps 0.05-0.10 m from a
# prop. PR #106 widened every post but the working slots' frames in the scene file (a scene-only device the real
# truck does not have); the planner's own LiDAR keepout (P1c, --lidar-keepout-m, docs/plans/2026-10-23-planner-
# permission-clearance.md) replaces it, so the posts are their size by default. ``keepout`` > 0 still builds the
# PR #106 copy for comparison.
RACK_KEEPOUT_M = 0.0
RACK_KINDS = ("aisle", "aisle_out")


def _beside(a: PalletSite, b: PalletSite) -> float | None:
    """b's lateral offset from a when b stands beside a in the same row (same entry yaw, same depth), else None."""
    if abs(math.atan2(math.sin(a.yaw_rad - b.yaw_rad), math.cos(a.yaw_rad - b.yaw_rad))) > 1e-6:
        return None
    c, sn = math.cos(a.yaw_rad), math.sin(a.yaw_rad)
    dx, dy = b.x_m - a.x_m, b.y_m - a.y_m
    along, lateral = dx * c + dy * sn, -dx * sn + dy * c
    return lateral if abs(along) < 0.05 and abs(lateral) > 1e-6 else None


def slot_markers(slots: list[PalletSite]) -> list[Rectangle]:
    """The plates of a set of unloading slots (each a PalletSite: centre and entry yaw): MARKER_SIDE_M to either
    side of a slot's axis, or halfway to the slot beside it in the same row when that is nearer (shared plates
    then coincide); plates whose centres are closer than MARKER_MERGE_M become one at their midpoint."""
    plates: list[list[float]] = []
    for s in slots:
        c, sn = math.cos(s.yaw_rad), math.sin(s.yaw_rad)
        ahead = SLOT_DEPTH_M / 2 + MARKER_BEHIND_M + MARKER_L_M / 2
        offsets = [o for o in (_beside(s, other) for other in slots if other is not s) if o is not None]
        for sign in (1.0, -1.0):
            same_side = [abs(o) for o in offsets if o * sign > 0]
            side = sign * min([MARKER_SIDE_M] + [o / 2 for o in same_side])
            x, y = s.x_m + ahead * c - side * sn, s.y_m + ahead * sn + side * c
            near = [q for q in plates if math.hypot(q[0] - x, q[1] - y) < MARKER_MERGE_M]
            if near:
                q = near[0]
                q[0], q[1] = (q[0] + x) / 2, (q[1] + y) / 2
            else:
                plates.append([x, y, s.yaw_rad])
    return [Rectangle(x, y, MARKER_L_M, MARKER_T_M, yaw) for x, y, yaw in plates]


def pallet_visuals(scenario: TransportScenario) -> list[dict]:
    """Every pallet prop is drawn as a pallet (a static copy of the mission pallet)."""
    out = []
    for p in scenario.props:
        if p.asset.uri.startswith("riibotics:") and "pallet" in p.asset.uri:
            r = p.rectangle
            out.append(_visual("pallet", r.x_m, r.y_m, r.length_m, r.width_m, r.yaw_rad))
    return out


def rack_uprights(slot_x: list[float], pitch: float, aisle: float, depth: float, back_gap: float) -> list[Rectangle]:
    """A rack frame at every slot boundary (scene frame): a front upright whose aisle face is flush with the pallets'
    entry face and a back upright against the back frame."""
    front = aisle / 2 + UPRIGHT_M / 2
    back = aisle / 2 + depth + back_gap - UPRIGHT_M / 2
    edges = [slot_x[0] - pitch / 2] + [x + pitch / 2 for x in slot_x]
    return [Rectangle(x, y, UPRIGHT_M, UPRIGHT_M, 0.0) for x in edges for y in (front, back)]


def aisle_scene(scale_name: str = "length", slot_gap: float = 0.20, back_gap: float = 0.15, uprights: bool = False,
                keepout: float = 0.0):
    pallet = load_pallet_geometry(PALLETS[PALLET])
    scenario, info = aisle_case(SCALES[scale_name], pallet, "gap", "gap", slot_gap, back_gap)
    if uprights:
        pitch = info["pitch_m"]
        posts = rack_uprights([slot_gap / 2 + pitch * (i + 0.5) for i in range(SLOTS)], pitch, info["aisle_m"],
                              pallet.overall_depth_m, back_gap)
        work = (scenario.pickup.x_m, scenario.destination.x_m)

        def planned(r: Rectangle) -> Rectangle:
            if any(abs(abs(r.x_m - w) - pitch / 2) < 1e-9 for w in work):
                return r
            return Rectangle(r.x_m, r.y_m, r.length_m + 2 * keepout, r.width_m + 2 * keepout, r.yaw_rad)

        asset = AssetSpec(UPRIGHT_URI, UPRIGHT_M, UPRIGHT_M, RACK_H_M)
        scenario = replace(scenario, props=scenario.props + tuple(PlacedProp(asset, planned(r)) for r in posts))
        info = {**info, "rack_uprights": len(posts), "upright_m": UPRIGHT_M, "upright_keepout_m": keepout}
    visuals = pallet_visuals(scenario)
    for r in posts if uprights else ():
        visuals.append({**_visual("wall", r.x_m, r.y_m, r.length_m, r.width_m, 0.0, RACK_H_M, RACK_ORANGE),
                        "role": "rack_upright"})
    b = scenario.bounds
    # The rack's back frame stands just past the planner's bound (the rack row is the bound).
    visuals.append(_visual("wall", (b.x_min_m + b.x_max_m) / 2, b.y_max_m + WALL_T_M / 2,
                           b.x_max_m - b.x_min_m, WALL_T_M, 0.0, RACK_H_M, RACK_ORANGE))
    # Blocks are floor storage areas (drawn grey, holding pallets): a floor patch and four pallets each.
    for p in scenario.props:
        if p.asset.uri == "riibotics:block":
            r = p.rectangle
            visuals.append(_visual("floor", r.x_m, r.y_m, r.length_m, r.width_m, 0.0, 0.0, ZONE_GREY))
            top = r.y_m + r.width_m / 2
            for dx in (-0.43, 0.43):
                for row in (0, 1):
                    cx, cy = r.x_m + dx, top - 0.5 - row * (pallet.overall_depth_m + 0.25)
                    if abs(dx) + pallet.overall_width_m / 2 <= r.length_m / 2:
                        visuals.append(_visual("pallet", cx, cy, pallet.overall_depth_m, pallet.overall_width_m, UP))
    return scenario, visuals, {"scale": scale_name, **info}


def trailer_scene(case: str = "a_front", row_gap: float = 0.20):
    pallet = load_pallet_geometry(PALLETS[PALLET])
    scenario, info = trailer_case(SCALES["length"], pallet, case, row_gap)
    inner, length = info["inner_m"], info["length_m"]
    visuals = pallet_visuals(scenario)
    # The trailer itself: two side walls and its front wall (the planner's blocks beside it and
    # its bound are wider; the floor past the walls is simply out of bounds for the planner).
    for sign in (-1, 1):
        visuals.append(_visual("wall", sign * (inner / 2 + WALL_T_M / 2), (length + WALL_T_M) / 2,
                               WALL_T_M, length + WALL_T_M, 0.0, WALL_H_M, WALL_GREY))
    visuals.append(_visual("wall", 0.0, length + WALL_T_M / 2, inner + 2 * WALL_T_M, WALL_T_M, 0.0,
                           WALL_H_M, WALL_GREY))
    visuals.append(_visual("floor", 0.0, length / 2, inner, length, 0.0, 0.0, ZONE_GREY))
    yard = scenario.destination
    visuals.append(_visual("floor", yard.x_m, yard.y_m, 1.4, 1.4, 0.0, 0.0, UNLOAD_BLUE))
    return scenario, visuals, {"case": case, **info}


def _drawing(px: float, py: float) -> tuple[float, float]:
    """Drawing pixels (y down) to the scene frame (y up, forks along +y), metres."""
    return px * PX_M, -py * PX_M


def _site(px: float, py: float, tilt_deg: float = 0.0) -> PalletSite:
    x, y = _drawing(px, py)
    return PalletSite(x, y, UP + math.radians(tilt_deg))


def _sequence_scene(loading: list[PalletSite], unloading: list[PalletSite], order: list[tuple[int, int]],
                    k: int, start: Pose2D, zones: list[tuple[tuple[float, float, float, float], tuple]],
                    margin_m: float):
    """Scene k of a sequence: order[k] = (loading index, unloading index) is carried now; the
    earlier ones stand at their unloading sites, the rest wait at their loading sites."""
    pallet = load_pallet_geometry(PALLETS[PALLET])
    pick, drop = order[k]
    done = {li: ui for li, ui in order[:k]}
    props = []
    for i, site in enumerate(loading):
        if i != pick and i not in done:
            props.append(pallet_prop(f"loading_pallet_{i}", site.x_m, site.y_m, pallet, site.yaw_rad))
    for li, ui in done.items():
        site = unloading[ui]
        props.append(pallet_prop(f"delivered_pallet_{li}", site.x_m, site.y_m, pallet, site.yaw_rad))
    xs = [s.x_m for s in loading + unloading] + [start.x_m]
    ys = [s.y_m for s in loading + unloading] + [start.y_m]
    bounds = Bounds(min(xs) - margin_m, max(xs) + margin_m, start.y_m - 2.0, max(ys) + margin_m)
    scenario = TransportScenario(seed=0, start_rear=start, pickup=loading[pick], destination=unloading[drop],
                                 props=tuple(props), bounds=bounds)
    visuals = [_visual("floor", (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.0, 0.0, colour)
               for (x0, x1, y0, y1), colour in zones] + pallet_visuals(scenario)
    return scenario, visuals, {"pick": pick, "drop": drop, "delivered": sorted(done)}


def line_unloading() -> list[PalletSite]:
    return [_site(1118, py) for py in (87, 205, 323)]       # deepest first


def random_unloading() -> list[PalletSite]:
    return [_site(px, 48) for px in (1125, 1268, 1411)]


def line_defs():
    """Scenario 3: three pallets in a column, the unloading column 8.9 m to the right; the
    deepest unloading slot is filled first. Drawn: pallets at y 95 / 215 / 333 px, columns at
    x 124 and 1118 px, fork tips at y 605 px. (loading, unloading, order, start, zones, margin)"""
    loading = [_site(124, py) for py in (333, 215, 95)]          # front first
    unloading = line_unloading()
    tip_x, tip_y = _drawing(124, 605)
    start = Pose2D(tip_x, tip_y - TRUCK_FRONT_M, UP)
    zones = [((25 * PX_M, 223 * PX_M, -600 * PX_M, -15 * PX_M), ZONE_GREY),
             ((1020 * PX_M, 1217 * PX_M, -600 * PX_M, 0.0), UNLOAD_BLUE)]
    return loading, unloading, [(0, 0), (1, 1), (2, 2)], start, zones, 1.6


def random_defs():
    """Scenario 4: five pallets in the loading area, three carried to the unloading row.
    Drawn centres (px) and tilts; the drawing's tilts are capped at the scenario's 10 deg."""
    loading = [_site(121, 68), _site(127, 189), _site(343, 213, -8.0), _site(185, 356, 10.0),
               _site(495, 312, -6.0)]
    unloading = random_unloading()
    tip_x, tip_y = _drawing(327, 515)
    start = Pose2D(tip_x, tip_y - TRUCK_FRONT_M, UP)
    zones = [((28 * PX_M, 670 * PX_M, -485 * PX_M, 0.5), ZONE_GREY),
             ((1043 * PX_M, 1684 * PX_M, -485 * PX_M, 0.5), UNLOAD_BLUE)]
    return loading, unloading, [(3, 0), (4, 1), (2, 2)], start, zones, 1.2


SEQUENCES = {"line": line_defs, "random": random_defs}


def line_scene(k: int):
    scenario, visuals, info = _sequence_scene(*line_defs()[:3], k, *line_defs()[3:])
    return scenario, visuals, {"sequence_index": k, **info}


def random_scene(k: int):
    scenario, visuals, info = _sequence_scene(*random_defs()[:3], k, *random_defs()[3:])
    return scenario, visuals, {"sequence_index": k, **info}


CONVEYOR_GREY = (0.79, 0.81, 0.83)


def _rotate(scenario: TransportScenario, visuals: list[dict]) -> tuple[TransportScenario, list[dict]]:
    """The scene frame turned by -90 deg about its origin (x, y -> y, -x), headings with it."""
    def point(x, y):
        return y, -x

    def rect(r: Rectangle) -> Rectangle:
        return Rectangle(*point(r.x_m, r.y_m), r.length_m, r.width_m, r.yaw_rad - UP)

    b = scenario.bounds
    turned = TransportScenario(
        seed=scenario.seed,
        start_rear=Pose2D(*point(scenario.start_rear.x_m, scenario.start_rear.y_m), scenario.start_rear.yaw_rad - UP),
        pickup=PalletSite(*point(scenario.pickup.x_m, scenario.pickup.y_m), scenario.pickup.yaw_rad - UP),
        destination=PalletSite(*point(scenario.destination.x_m, scenario.destination.y_m),
                               scenario.destination.yaw_rad - UP),
        props=tuple(PlacedProp(p.asset, rect(p.rectangle)) for p in scenario.props),
        bounds=Bounds(b.y_min_m, b.y_max_m, -b.x_max_m, -b.x_min_m))
    moved = [{**v, "x_m": v["y_m"], "y_m": -v["x_m"], "yaw_rad": v["yaw_rad"] - UP} for v in visuals]
    return turned, moved


def conveyor_scene():
    """Scenario 2, read off its drawing (9.0 mm/px): the forks point left into conveyor A for the
    pickup and up into conveyor B for the drop. As in the fleet demo scene (p4): each pallet's entry
    face is flush with its conveyor's edge (the carriage cannot reach in further), the conveyor is
    cleared CONVEYOR_BAND_M beside the pallet and CONVEYOR_BACK_M behind it. The drawing's pickup
    faces -x, so the scene is turned by -90 deg first (forks along +y, as the others)."""
    pallet = load_pallet_geometry(PALLETS[PALLET])
    m, d = PX_M, pallet.overall_depth_m
    a = (263 * m, 415 * m, -470 * m, -12 * m)               # conveyor A: x0, x1, y0, y1
    b = (1414 * m, 1872 * m, -257 * m, -113 * m)            # conveyor B
    pick = PalletSite(a[1] - d / 2, -400 * m, math.pi)      # entry face on A's east edge
    drop = PalletSite(1490 * m, b[2] + d / 2, UP)            # entry face on B's south edge
    tip_x, tip_y = _drawing(697, 637)
    start = Pose2D(tip_x, tip_y - TRUCK_FRONT_M, UP)
    band = pallet.overall_width_m / 2 + CONVEYOR_BAND_M
    h = CONVEYOR_H_M
    boxes = [(a[0], a[1], pick.y_m + band, a[3]),                                   # A upstream
             (a[0], pick.x_m - d / 2 - CONVEYOR_BACK_M, a[2], pick.y_m + band),     # A behind the pallet
             (drop.x_m + band, b[1], b[2], b[3]),                                   # B downstream
             (drop.x_m - band, drop.x_m + band, drop.y_m + d / 2 + CONVEYOR_BACK_M, b[3])]  # B behind
    if drop.x_m - band - b[0] > 0.05:
        boxes.append((b[0], drop.x_m - band, b[2], b[3]))                         # B upstream
    pieces = [((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0) for x0, x1, y0, y1 in boxes]
    props = tuple(prop("conveyor", x, y, length, width, 0.0, h) for x, y, length, width in pieces)
    bounds = Bounds(a[0] - 1.5, b[1] + 1.0, start.y_m - 2.0, a[3] + 1.0)
    scenario = TransportScenario(seed=0, start_rear=start, pickup=pick, destination=drop, props=props, bounds=bounds)
    visuals = [_visual("wall", x, y, length, width, 0.0, h, CONVEYOR_GREY) for x, y, length, width in pieces]
    scenario, visuals = _rotate(scenario, visuals)
    return scenario, visuals, {"conveyor_height_m": h, "band_m": CONVEYOR_BAND_M, "back_m": CONVEYOR_BACK_M}


def with_slot_markers(kind: str, built):
    """A scene with the plates of every unloading slot of its zone (the slots of the whole sequence, filled or not)
    added to the planner's props and to the visuals (walls Isaac collides with, role slot_marker)."""
    if kind not in MARKED_KINDS:
        raise ValueError(f"no unloading slots to mark in a {kind} scene")
    scenario, visuals, info = built
    plates = slot_markers({"line": line_unloading, "random": random_unloading}[kind]())
    asset = AssetSpec(MARKER_URI, MARKER_L_M, MARKER_T_M, MARKER_H_M)
    props = scenario.props + tuple(PlacedProp(asset, r) for r in plates)
    marks = [{**_visual("wall", r.x_m, r.y_m, r.length_m, r.width_m, r.yaw_rad, MARKER_H_M, MARKER_COLOUR),
              "role": "slot_marker"} for r in plates]
    return replace(scenario, props=props), visuals + marks, {**info, "slot_markers": True}


def rack_scene(slot_gap: float = RACK_SLOT_GAP_M, back_gap: float = RACK_BACK_GAP_M, keepout: float = RACK_KEEPOUT_M):
    """The aisle with rack frames and the assumed 0.20 m back gap (RACK_* above); aisle and aisle_out stay as they
    were for regression."""
    return aisle_scene(slot_gap=slot_gap, back_gap=back_gap, uprights=True, keepout=keepout)


def unloading_slots_hall(kind: str, built) -> list[PalletSite]:
    """The unloading slots of a scene in the hall frame (tests: the plates against the slots)."""
    scenario, _, _ = built
    probe = replace(scenario, props=())
    sites = {"line": line_unloading, "random": random_unloading}[kind]()
    out = []
    for s in sites:
        moved, _, _ = to_hall(replace(probe, pickup=s), [])
        out.append(moved.pickup)
    return out


# ---- placement in the hall ----

def _turn(x: float, y: float) -> tuple[float, float]:
    """Scene frame (forks along +y) to the hall frame's axes (forks along +x): -90 deg."""
    return y, -x


def to_hall(scenario: TransportScenario, visuals: list[dict]) -> tuple[TransportScenario, list[dict], dict]:
    b = scenario.bounds
    # Turned bounds: x from y_min..y_max, y from -x_max..-x_min. Then x_max -> EAST_EDGE_M, start y -> 0.
    sx, sy = _turn(scenario.start_rear.x_m, scenario.start_rear.y_m)
    tx, ty = EAST_EDGE_M - b.y_max_m, -sy

    def point(x, y):
        u, v = _turn(x, y)
        return u + tx, v + ty

    def pose(p: Pose2D) -> Pose2D:
        return Pose2D(*point(p.x_m, p.y_m), p.yaw_rad - UP)

    def site(s: PalletSite) -> PalletSite:
        return PalletSite(*point(s.x_m, s.y_m), s.yaw_rad - UP)

    def rect(r: Rectangle) -> Rectangle:
        return Rectangle(*point(r.x_m, r.y_m), r.length_m, r.width_m, r.yaw_rad - UP)

    bounds = Bounds(b.y_min_m + tx, b.y_max_m + tx, -b.x_max_m + ty, -b.x_min_m + ty)
    if not (HALL.x_min_m <= bounds.x_min_m and bounds.x_max_m <= HALL.x_max_m
            and HALL.y_min_m <= bounds.y_min_m and bounds.y_max_m <= HALL.y_max_m):
        raise ValueError(f"scene bounds {bounds} leave the south hall {HALL}")
    hall = TransportScenario(
        seed=scenario.seed, start_rear=pose(scenario.start_rear), pickup=site(scenario.pickup),
        destination=site(scenario.destination),
        props=tuple(PlacedProp(p.asset, rect(p.rectangle)) for p in scenario.props), bounds=bounds)
    moved = []
    for v in visuals:
        x, y = point(v["x_m"], v["y_m"])
        moved.append({**v, "x_m": x, "y_m": y, "yaw_rad": v["yaw_rad"] - UP})
    return hall, moved, {"turn_rad": -UP, "translate_m": [tx, ty]}


# Where the pickup pallet really stands against its nominal slot (hall frame: dx, dy, dyaw deg). The zone and
# the observation candidates come from the nominal slot (heading +x), never from the pallet itself (Codex
# 2026-10-20 P2); the random scene's pallets carry the drawing's tilt as well.
# aisle_out: the same slot with the pallet 3 cm out of the rack instead of 3 cm in -- 3 cm in leaves the
# extraction 2.8 mm inside the rack-back bound (back gap 0.15 m, an assumption; Isaac r4 and Codex 2026-10-20).
PICKUP_ERRORS = {"aisle": (0.03, -0.05, 2.0), "aisle_out": (-0.03, -0.05, 2.0), "trailer": (-0.04, 0.05, -2.0),
                 "conveyor": (0.04, -0.05, 2.0),
                 "line_1": (0.05, -0.06, 3.0), "line_2": (-0.04, 0.05, -2.0), "line_3": (0.03, 0.04, 2.0),
                 "random_1": (0.06, -0.08, 0.0), "random_2": (-0.07, 0.05, 0.0), "random_3": (0.05, 0.07, 0.0)}


def observation_waypoints(site: PalletSite) -> list[list[float]]:
    """Rear-axle poses facing the pickup: on its axis at 3.4 / 3.0 / 3.8 / 2.6 m from its centre
    (camera to entry face about 2.1-2.5 m), then 0.4 m to either side looking at its centre."""
    out = []
    c, s = math.cos(site.yaw_rad), math.sin(site.yaw_rad)
    for d in (3.4, 3.0, 3.8, 2.6):
        out.append([site.x_m - d * c, site.y_m - d * s, site.yaw_rad])
    for side in (0.4, -0.4):
        x, y = site.x_m - 3.2 * c - side * s, site.y_m - 3.2 * s + side * c
        out.append([x, y, math.atan2(site.y_m - y, site.x_m - x)])
    return [[round(v, 4) for v in w] for w in out]


def pickup_zone(site: PalletSite) -> dict:
    return {"x_min_m": site.x_m - ZONE_GROW_M, "x_max_m": site.x_m + ZONE_GROW_M,
            "y_min_m": site.y_m - ZONE_GROW_M, "y_max_m": site.y_m + ZONE_GROW_M, "yaw_max_rad": ZONE_YAW_RAD}


def views(s: TransportScenario) -> dict:
    """A quarter view from behind the start's left, looking between the pickup and the destination."""
    tx = (s.pickup.x_m + s.destination.x_m + s.start_rear.x_m) / 3
    ty = (s.pickup.y_m + s.destination.y_m + s.start_rear.y_m) / 3
    return {"quarter_target": [round(tx, 2), round(ty, 2), 0.2],
            "quarter_eye": [round(s.start_rear.x_m - 3.5, 2), round(ty + 4.5, 2), 4.2]}


def scenes() -> dict:
    out = {"aisle": aisle_scene(), "aisle_out": aisle_scene(), "trailer": trailer_scene(),
           "conveyor": conveyor_scene()}
    for k in range(3):
        out[f"line_{k + 1}"] = line_scene(k)
        out[f"random_{k + 1}"] = random_scene(k)
    return out


def _heights(scenario: TransportScenario) -> TransportScenario:
    """Prop heights as Isaac draws them: trailer walls WALL_H_M, blocks (floor storage) a pallet's height."""
    pallet = load_pallet_geometry(PALLETS[PALLET])
    height = {"riibotics:trailer_wall": WALL_H_M, "riibotics:block": pallet.overall_height_m}
    return replace(scenario, props=tuple(
        PlacedProp(replace(p.asset, height_m=height.get(p.asset.uri, p.asset.height_m)), p.rectangle)
        for p in scenario.props))


def scene_record(name: str, built) -> dict:
    scenario, visuals, info = built
    scenario = _heights(scenario)
    hall, visuals, placement = to_hall(scenario, visuals)
    nominal = PalletSite(hall.pickup.x_m, hall.pickup.y_m, 0.0)
    dx, dy, dyaw = PICKUP_ERRORS[name.removesuffix("_m").removesuffix("_rack")]  # a copy picks as its plain scene
    hall = replace(hall, pickup=PalletSite(nominal.x_m + dx, nominal.y_m + dy,
                                           hall.pickup.yaw_rad + math.radians(dyaw)))
    return {"format": FORMAT, "name": name, "pallet": PALLET, "scenario": asdict(hall),
            "pickup_nominal": asdict(nominal), "pickup_zone": pickup_zone(nominal),
            "observation_waypoints": observation_waypoints(nominal),
            "visuals": visuals, "views": views(hall), "source": {**info, "placement": placement}}


# Pallets a session never carries still stand off their slot (component <= 0.08 m, as the carried ones' errors).
SESSION_EXTRA_ERRORS = {"random": {0: (0.04, -0.03, 1.0), 1: (-0.03, 0.04, -1.0)}}


def session_record(kind: str, markers: bool = False) -> dict:
    """One session scene (docs/plans/2026-10-22-session-scene-pallet-props.md 2.1-2.2): every pallet of the sequence
    scenario with its nominal slot (the drawing's site and tilt, in the hall), its truth (nominal + its own error -- the
    carried ones keep the error their v1 scene k gave them) and the zone and candidates made from its nominal; the
    unloading slots as destinations D-1..; the drawing's order. Its pallets are not visuals: Isaac makes them from
    pallets[].truth (PR 3). ``markers``: the slot markers of the marked v1 copies (``with_slot_markers``) as the
    session's static props and walls, the scene named ``<kind>_m``."""
    loading, unloading, order, start, zones, margin = SEQUENCES[kind]()
    pallet = load_pallet_geometry(PALLETS[PALLET])
    # The hall placement is the v1 scenes' (their bounds come from every site and the start, the same for each k).
    probe = [pallet_prop(f"site_{i}", s.x_m, s.y_m, pallet, s.yaw_rad) for i, s in enumerate(loading + unloading)]
    xs = [s.x_m for s in loading + unloading] + [start.x_m]
    ys = [s.y_m for s in loading + unloading] + [start.y_m]
    bounds = Bounds(min(xs) - margin, max(xs) + margin, start.y_m - 2.0, max(ys) + margin)
    scenario = TransportScenario(seed=0, start_rear=start, pickup=loading[order[0][0]],
                                 destination=unloading[order[0][1]], props=tuple(probe), bounds=bounds)
    visuals = [_visual("floor", (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.0, 0.0, colour)
               for (x0, x1, y0, y1), colour in zones]
    if markers:
        scenario, visuals, _ = with_slot_markers(kind, (scenario, visuals, {}))
    hall, visuals, placement = to_hall(scenario, visuals)
    sites = [PalletSite(p.rectangle.x_m, p.rectangle.y_m, p.rectangle.yaw_rad) for p in hall.props[:len(probe)]]
    nominal, slots = sites[:len(loading)], sites[len(loading):]
    errors = {li: PICKUP_ERRORS[f"{kind}_{k + 1}"] for k, (li, _) in enumerate(order)}
    errors.update(SESSION_EXTRA_ERRORS.get(kind, {}))
    pallets = []
    for i, n in enumerate(nominal):
        dx, dy, dyaw = errors[i]
        pallets.append({
            "pallet_id": f"P-{i + 1}", "nominal": asdict(n),
            "truth": asdict(PalletSite(n.x_m + dx, n.y_m + dy, n.yaw_rad + math.radians(dyaw))),
            "zone": {"x_min_m": n.x_m - ZONE_GROW_M, "x_max_m": n.x_m + ZONE_GROW_M, "y_min_m": n.y_m - ZONE_GROW_M,
                     "y_max_m": n.y_m + ZONE_GROW_M, "yaw_centre_rad": n.yaw_rad, "yaw_half_rad": ZONE_YAW_RAD},
            "observation_waypoints": observation_waypoints(n)})
    first = replace(hall, props=())
    return {"format": SESSION_FORMAT, "name": f"{kind}_m" if markers else kind, "pallet": PALLET,
            "start_rear": asdict(hall.start_rear), "bounds": asdict(hall.bounds),
            "static_props": [asdict(p) for p in hall.props[len(probe):]],
            "pallets": pallets,
            "destinations": [{"destination_id": f"D-{j + 1}", **asdict(s)} for j, s in enumerate(slots)],
            "sequence": [[f"P-{li + 1}", f"D-{ui + 1}"] for li, ui in order],
            "visuals": visuals, "views": views(first),
            "source": {"placement": placement, "errors": errors, **({"slot_markers": True} if markers else {})}}


# ---- CPU planning check with the Isaac mission settings ----

def grown_pallets(scenario: TransportScenario, grow_m: float) -> TransportScenario:
    """Every pallet prop (a neighbour) grown by grow_m on each side: the keep-out a planner puts around
    a pallet it does not handle (the fleet planner's PALLET_KEEP_M is 0.3 m)."""
    def grow(p):
        if "pallet" not in p.asset.uri:
            return p
        r = p.rectangle
        return PlacedProp(p.asset, Rectangle(r.x_m, r.y_m, r.length_m + 2 * grow_m, r.width_m + 2 * grow_m, r.yaw_rad))
    return replace(scenario, props=tuple(grow(p) for p in scenario.props))


def plan_scene(path: Path, out: Path, grow_m: float = 0.0, keepout_m: float = 0.0) -> None:
    """keepout_m: the runner's --lidar-keepout-m (P1c, docs/plans/2026-10-23-planner-permission-clearance.md)."""
    record = json.loads(path.read_text())
    scenario = grown_pallets(scenario_from_record(record["scenario"]), grow_m)
    pallet = load_pallet_geometry(PALLETS[record["pallet"]])
    geometry = replace(mission_geometry(URDF, pallet, RESERVE_M, **STRAIGHTS), lidar_keepout_m=keepout_m)
    config = make_transport_planner_config(curvature_limit_inv_m=KAPPA, clearance_m=CLEARANCE_M,
                                           max_expansions=30000, **D7_PLANNER_OPTIONS)
    result = {"scene": record["name"], "scene_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "grow_pallets_m": grow_m, "lidar_keepout_m": keepout_m, "kappa": KAPPA, "candidates": []}
    started = time.monotonic()
    # As the runner plans a scene: the zone grown by the pallet's half diagonal before recognition, the
    # (here: true) pallet as the estimate after it -- never scenario.pickup by default.
    z, grow = record["pickup_zone"], math.hypot(pallet.overall_depth_m, pallet.overall_width_m) / 2
    zone = Rectangle((z["x_min_m"] + z["x_max_m"]) / 2, (z["y_min_m"] + z["y_max_m"]) / 2,
                     z["x_max_m"] - z["x_min_m"] + 2 * grow, z["y_max_m"] - z["y_min_m"] + 2 * grow, 0.0)
    estimate = Rectangle(scenario.pickup.x_m, scenario.pickup.y_m, pallet.overall_depth_m,
                         pallet.overall_width_m, scenario.pickup.yaw_rad)
    observe = None
    for extended in (False, True):
        for index, w in enumerate(record["observation_waypoints"]):
            leg = plan_observation_leg(scenario, Pose2D(*w), config, geometry=geometry, extended=extended,
                                       pickup_obstacle=zone)
            result["candidates"].append({"index": index, "extended": extended, "status": leg.status})
            if leg.success:
                observe = leg
                result["observation"] = {"index": index, "extended": extended, "pose": w}
                break
        if observe is not None:
            break
    if observe is None:
        result.update(outcome="observe_failed", wall_s=round(time.monotonic() - started, 1))
    else:
        end = observe.poses[-1]
        trace: list = []
        plan = plan_transport(scenario, config, geometry=geometry, target_pickup=scenario.pickup,
                              start_rear=Pose2D(float(end[0]), float(end[1]), float(end[2])),
                              return_to=scenario.start_rear, trace=trace, pickup_obstacle=estimate)
        result.update(outcome=classify(plan.status), status=plan.status, trace=trace,
                      gear_changes=gear_changes(plan) if plan.success else None,
                      wall_s=round(time.monotonic() - started, 1),
                      lengths_m={n: round(float(getattr(plan, n).length_m), 2)
                                 for n in STAGES if getattr(plan, n, None) is not None})
    (out / _result_name(record["name"], grow_m, keepout_m)).write_text(json.dumps(result, indent=1, default=str))


def _result_name(name: str, grow_m: float, keepout_m: float = 0.0) -> str:
    keep = "" if keepout_m == 0 else f".keep{keepout_m!r}"  # exact: two values never share a file (Codex P2)
    return f"{name}{keep}.plan.json" if grow_m == 0 else f"{name}.grow{grow_m:.2f}{keep}.plan.json"


def _plan_worker(args):
    """A result counts only when this run's child wrote it for this scene file (Codex 2026-10-20 P2)."""
    path, out, timeout, grow_m, keepout_m = (*args, 0.0) if len(args) == 4 else args
    result = out / _result_name(path.stem, grow_m, keepout_m)
    result.unlink(missing_ok=True)
    proc = multiprocessing.get_context("spawn").Process(target=plan_scene, args=(path, out, grow_m, keepout_m))
    proc.start()
    proc.join(timeout)
    if proc.is_alive():
        proc.kill()
        proc.join()
        return result.name, "case_timeout"
    if proc.exitcode != 0 or not result.exists():
        return result.name, f"crash {proc.exitcode}"
    record = json.loads(result.read_text())
    if record.get("scene_sha256") != hashlib.sha256(path.read_bytes()).hexdigest():
        return result.name, "stale_result"
    if record.get("lidar_keepout_m", 0.0) != keepout_m:
        return result.name, "stale_result"
    return result.name, record["outcome"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plan", action="store_true", help="also plan every scene on the CPU")
    parser.add_argument("--only", help="comma-separated scene names")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--kappa", type=float, default=None, help="the planner curvature limit (1/m; default 0.29)")
    parser.add_argument("--timeout-s", type=float, default=1200)
    parser.add_argument("--grow-pallets", default="0", help="comma-separated keep-outs (m) around neighbour pallets")
    parser.add_argument("--lidar-keepout", default="0",
                        help="comma-separated --lidar-keepout-m values (m) to plan with (P1c, plan 2026-10-23)")
    parser.add_argument("--slot-markers", action="store_true",
                        help="also write <name>_m.json with slot markers for the line / random scenes (plan 2026-10-22)")
    parser.add_argument("--rack", action="store_true",
                        help="also write aisle_rack.json / aisle_out_rack.json with rack frames (plan 2026-10-22 aisle)")
    parser.add_argument("--session", choices=tuple(SEQUENCES), action="append", default=[],
                        help="also write this sequence as one session scene (<name>_session.json)")
    args = parser.parse_args()
    if args.kappa is not None:
        global KAPPA
        KAPPA = float(args.kappa)
        os.environ["FORKLIFT_PLAN_KAPPA"] = repr(KAPPA)  # the spawned planners import it from here
    args.output.mkdir(parents=True, exist_ok=True)
    for kind in args.session:
        path = args.output / f"{kind}_session.json"
        path.write_text(json.dumps(session_record(kind), indent=1) + "\n")
        print("session", path)
        if args.slot_markers and kind in MARKED_KINDS:  # the marked copy beside it (<kind>_m_session.json)
            path = args.output / f"{kind}_m_session.json"
            path.write_text(json.dumps(session_record(kind, markers=True), indent=1) + "\n")
            print("session", path)
    only = set(args.only.split(",")) if args.only else None
    paths = []
    built_scenes = scenes()
    if args.slot_markers:
        for name in list(built_scenes):
            kind = name.split("_")[0]
            if kind in MARKED_KINDS:
                built_scenes[f"{name}_m"] = with_slot_markers(kind, built_scenes[name])
    if args.rack:
        for name in RACK_KINDS:
            built_scenes[f"{name}_rack"] = rack_scene()
    for name, built in built_scenes.items():
        if only and name not in only:
            continue
        path = args.output / f"{name}.json"
        path.write_text(json.dumps(scene_record(name, built), indent=1) + "\n")
        paths.append(path)
        print("scene", path)
    if args.plan:
        with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
            grows = [float(g) for g in args.grow_pallets.split(",")]
            keeps = [float(k) for k in args.lidar_keepout.split(",")]
            jobs = [(p, args.output, args.timeout_s, g, k) for p in paths for g in grows for k in keeps]
            for name, outcome in pool.map(_plan_worker, jobs):
                print("plan", name, outcome, flush=True)


if __name__ == "__main__":
    main()
