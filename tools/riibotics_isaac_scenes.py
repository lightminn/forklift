"""Scene files for run_transport.py --layout scene: the Riibotics demo scenarios in the south hall.

Riibotics sent five demo scenarios on 2026-10-19 (local reference, not in the repository). Four
are built here, each as one transport mission (pick one pallet, deliver it, return to the start):

  aisle    (1) narrow aisle: pick a floor pallet from the rack row, put it into another slot
  line     (3) three pallets in a column, carried one after another to the unloading column
  random   (4) five pallets placed loosely (yaw within 10 deg), three carried to the unloading row
  trailer  (5) a trailer walled except for its entrance: take the front pallet out

The conveyor (2) is left out: its pallets stand higher than the forks lift (week-8 analysis).
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
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import multiprocessing
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
    STAGES,
    UP,
    aisle_case,
    classify,
    gear_changes,
    pallet_prop,
    prop,
    trailer_case,
)

from scene_file import FORMAT, scenario_from_record  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning import Pose2D  # noqa: E402
from forklift_core.planning.geometry import Bounds, Rectangle  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
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
RESERVE_M, KAPPA, CLEARANCE_M = 0.016, 0.29, 0.10
REAR_TO_CAMERA_M = 0.34 + 0.619   # rear axle -> base_link, base_link -> carriage_low_measured
# Placement in the south hall
EAST_EDGE_M = 4.6
HALL = Bounds(-25.6, 4.7, -22.9, 8.3)
# Heights of what Isaac draws (scaled 0.521 from typical real sizes; all above the 1.05 m LiDAR plane)
WALL_H_M = 1.36          # trailer walls: 2.6 m inside height
RACK_H_M = 1.50          # rack back frame
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


def pallet_visuals(scenario: TransportScenario) -> list[dict]:
    """Every pallet prop is drawn as a pallet (a static copy of the mission pallet)."""
    out = []
    for p in scenario.props:
        if p.asset.uri.startswith("riibotics:") and "pallet" in p.asset.uri:
            r = p.rectangle
            out.append(_visual("pallet", r.x_m, r.y_m, r.length_m, r.width_m, r.yaw_rad))
    return out


def aisle_scene(scale_name: str = "length", slot_gap: float = 0.20):
    pallet = load_pallet_geometry(PALLETS[PALLET])
    scenario, info = aisle_case(SCALES[scale_name], pallet, "gap", "gap", slot_gap, 0.15)
    visuals = pallet_visuals(scenario)
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


def line_scene(k: int):
    """Scenario 3: three pallets in a column, the unloading column 8.9 m to the right; the
    deepest unloading slot is filled first. Drawn: pallets at y 95 / 215 / 333 px, columns at
    x 124 and 1118 px, fork tips at y 605 px."""
    loading = [_site(124, py) for py in (333, 215, 95)]          # front first
    unloading = [_site(1118, py) for py in (87, 205, 323)]       # deepest first
    tip_x, tip_y = _drawing(124, 605)
    start = Pose2D(tip_x, tip_y - TRUCK_FRONT_M, UP)
    zones = [((25 * PX_M, 223 * PX_M, -600 * PX_M, -15 * PX_M), ZONE_GREY),
             ((1020 * PX_M, 1217 * PX_M, -600 * PX_M, 0.0), UNLOAD_BLUE)]
    scenario, visuals, info = _sequence_scene(loading, unloading, [(0, 0), (1, 1), (2, 2)], k, start, zones, 1.6)
    return scenario, visuals, {"sequence_index": k, **info}


def random_scene(k: int):
    """Scenario 4: five pallets in the loading area, three carried to the unloading row.
    Drawn centres (px) and tilts; the drawing's tilts are capped at the scenario's 10 deg."""
    loading = [_site(121, 68), _site(127, 189), _site(343, 213, -8.0), _site(185, 356, 10.0),
               _site(495, 312, -6.0)]
    unloading = [_site(px, 48) for px in (1125, 1268, 1411)]
    tip_x, tip_y = _drawing(327, 515)
    start = Pose2D(tip_x, tip_y - TRUCK_FRONT_M, UP)
    zones = [((28 * PX_M, 670 * PX_M, -485 * PX_M, 0.5), ZONE_GREY),
             ((1043 * PX_M, 1684 * PX_M, -485 * PX_M, 0.5), UNLOAD_BLUE)]
    scenario, visuals, info = _sequence_scene(loading, unloading, [(3, 0), (4, 1), (2, 2)], k, start, zones, 1.2)
    return scenario, visuals, {"sequence_index": k, **info}


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
PICKUP_ERRORS = {"aisle": (0.03, -0.05, 2.0), "trailer": (-0.04, 0.05, -2.0),
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
    out = {"aisle": aisle_scene(), "trailer": trailer_scene()}
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
    dx, dy, dyaw = PICKUP_ERRORS[name]
    hall = replace(hall, pickup=PalletSite(nominal.x_m + dx, nominal.y_m + dy,
                                           hall.pickup.yaw_rad + math.radians(dyaw)))
    return {"format": FORMAT, "name": name, "pallet": PALLET, "scenario": asdict(hall),
            "pickup_nominal": asdict(nominal), "pickup_zone": pickup_zone(nominal),
            "observation_waypoints": observation_waypoints(nominal),
            "visuals": visuals, "views": views(hall), "source": {**info, "placement": placement}}


# ---- CPU planning check with the Isaac mission settings ----

def plan_scene(path: Path, out: Path) -> None:
    record = json.loads(path.read_text())
    scenario = scenario_from_record(record["scenario"])
    pallet = load_pallet_geometry(PALLETS[record["pallet"]])
    geometry = mission_geometry(URDF, pallet, RESERVE_M, **STRAIGHTS)
    config = make_transport_planner_config(curvature_limit_inv_m=KAPPA, clearance_m=CLEARANCE_M,
                                           max_expansions=30000, **D7_PLANNER_OPTIONS)
    result = {"scene": record["name"], "scene_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "candidates": []}
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
    (out / f"{record['name']}.plan.json").write_text(json.dumps(result, indent=1, default=str))


def _plan_worker(args):
    """A result counts only when this run's child wrote it for this scene file (Codex 2026-10-20 P2)."""
    path, out, timeout = args
    result = out / f"{path.stem}.plan.json"
    result.unlink(missing_ok=True)
    proc = multiprocessing.get_context("spawn").Process(target=plan_scene, args=(path, out))
    proc.start()
    proc.join(timeout)
    if proc.is_alive():
        proc.kill()
        proc.join()
        return path.stem, "case_timeout"
    if proc.exitcode != 0 or not result.exists():
        return path.stem, f"crash {proc.exitcode}"
    record = json.loads(result.read_text())
    if record.get("scene_sha256") != hashlib.sha256(path.read_bytes()).hexdigest():
        return path.stem, "stale_result"
    return path.stem, record["outcome"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plan", action="store_true", help="also plan every scene on the CPU")
    parser.add_argument("--only", help="comma-separated scene names")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--timeout-s", type=float, default=1200)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    only = set(args.only.split(",")) if args.only else None
    paths = []
    for name, built in scenes().items():
        if only and name not in only:
            continue
        path = args.output / f"{name}.json"
        path.write_text(json.dumps(scene_record(name, built), indent=1) + "\n")
        paths.append(path)
        print("scene", path)
    if args.plan:
        with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
            for name, outcome in pool.map(_plan_worker, [(p, args.output, args.timeout_s) for p in paths]):
                print("plan", name, outcome, flush=True)


if __name__ == "__main__":
    main()
