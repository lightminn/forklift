"""Plan five fleet missions and run them kinematically under traffic control (CPU).

    python tools/fleet_cpu_sim.py --seeds 0-4 --output <dir> [--margin-m 0.3 ...]

Plan: docs/plans/2026-10-08-fleet-five-forklifts.md, stage F1. For each seed:
the fleet layout (config/fleet_full_warehouse.yaml on the south hall
generator), every truck's mission planned with forklift_core.planning
(measured chassis geometry and planner settings, as run_transport builds
them), then all trucks moved at once along their paths -- perfect tracking,
0.1 s steps, a stop at every gear change, 4.5 s to lift and to lower -- with
fleet_traffic granting each one the distance it may drive under its fixed
rank and safe-stop rule; nothing is replanned. Locks the traffic manager
reports are recorded, not resolved. True footprints are checked pairwise
every step; any overlap is a failure of the traffic rules.

Writes <output>/seed_<n>/result.json and trajectory.npz (1 Hz), and
<output>/summary.json. Failures are results; no seed is dropped.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.planning.fleet_layout import load_fleet_layout, make_fleet_scenario
from forklift_core.planning.fleet_mission import (
    LOADED_STAGES,
    STAGES,
    plan_fleet_mission,
)
from forklift_core.planning.fleet_traffic import (
    TrafficManager,
    arc_lengths,
    footprint_rectangle,
    pose_at,
    rectangles_overlap,
)
from forklift_core.planning.geometry import Footprint
from forklift_core.planning.pallet_mission import (
    SyntheticMissionGeometry,
    make_transport_planner_config,
)

ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
)
BAY_ASSETS = (
    "SM_BarelPlastic_A_01.usd",
    "SM_CratePlastic_D_01.usd",
    "SM_CardBoxA_02.usd",
)
LIFT_S = 4.5
DT_S = 0.1


def _module(name: str):
    path = ROOT / "sim/isaac" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _seeds(text: str) -> list[int]:
    seeds = []
    for part in text.split(","):
        first, _, last = part.partition("-")
        seeds.extend(range(int(first), int(last or first) + 1))
    return seeds


def mission_geometry(args) -> SyntheticMissionGeometry:
    """As run_transport: chassis reference and carriage limit from the URDF."""
    insertion = _module("insertion_geometry")
    axle_to_fork_tip_m, _ = insertion.read_chassis_reference_m(args.forklift_urdf)
    pallet = load_pallet_geometry(args.pallet_geometry)
    return SyntheticMissionGeometry(
        unloaded_footprint=Footprint(axle_to_fork_tip_m, 0.17, 0.36),
        pallet_depth_m=pallet.overall_depth_m,
        pallet_width_m=pallet.overall_width_m,
        axle_to_fork_tip_m=axle_to_fork_tip_m,
        carriage_limit_m=insertion.read_carriage_limit_m(args.forklift_urdf),
        insertion_reserve_m=args.insertion_reserve_m,
        delivery_straight_m=0.70,
    )


def _segments(result) -> list[tuple[np.ndarray, int, float]]:
    """Split a path at every gear change; the truck stops at each cusp.

    Each segment: its poses, its direction and where it starts on the path.
    """
    poses, directions = np.asarray(result.poses), np.asarray(result.directions)
    if len(poses) < 2:
        return [(poses, int(directions[0]) if len(directions) else 1, 0.0)]
    s = arc_lengths(poses)
    cuts = [0] + [
        i for i in range(1, len(poses) - 1) if directions[i + 1] != directions[i]
    ]
    cuts.append(len(poses) - 1)
    out = []
    for a, b in zip(cuts[:-1], cuts[1:], strict=True):
        if b > a:
            out.append((poses[a : b + 1], int(directions[min(a + 1, b)]), s[a]))
    return out


class Truck:
    """A kinematic follower working through its stage list."""

    def __init__(self, k, mission, plan, geometry, settings):
        self.k, self.name = k, f"truck_{k}"
        self.mission = mission
        self.geometry = geometry
        speeds = {
            "observe": settings["approach_speed_mps"],
            "approach": settings["approach_speed_mps"],
            "insert": settings["insert_speed_mps"],
            "extract": settings["extract_speed_mps"],
            "aisle_exit": settings["transport_speed_mps"],
            "transport": settings["transport_speed_mps"],
            "withdraw": settings["withdraw_speed_mps"],
            "return_home": settings["approach_speed_mps"],
        }
        self.accel = settings["drive_acceleration_mps2"]
        self.items = []
        for stage in STAGES:
            self.items.append(("path", stage, plan.stages[stage], speeds[stage]))
            if stage == "insert":
                self.items.append(("wait", "lift", LIFT_S, None))
            if stage == "transport":
                self.items.append(("wait", "lower", LIFT_S, None))
        self.item = -1
        self.loaded = False
        self.pose = np.array(
            [mission.parking.x_m, mission.parking.y_m, mission.parking.yaw_rad]
        )
        self.v = 0.0
        self.progress = 0.0
        self.segments = []
        self.segment = 0
        self.wait_left = 0.0
        self.done = False
        self.finished_s = None
        self.waiting_s = 0.0
        self.stage_times = {}
        self.offsets = {}
        self.route_end_m = 0.0

    @property
    def stage(self):
        return (
            self.items[self.item][1] if 0 <= self.item < len(self.items) else "parked"
        )

    def footprint(self) -> Footprint:
        g = self.geometry
        return g.loaded_footprint if self.loaded else g.unloaded_footprint

    def set_route(self, manager):
        """Hand the whole mission to the traffic manager, stage by stage."""
        g = self.geometry
        paths = [(stage, payload) for kind, stage, payload, _ in self.items if kind == "path"]
        offsets = manager.set_route(
            self.name,
            [
                (
                    np.asarray(result.poses),
                    g.loaded_footprint if stage in LOADED_STAGES else g.unloaded_footprint,
                )
                for stage, result in paths
            ],
        )
        self.offsets = dict(zip((stage for stage, _ in paths), offsets, strict=True))
        self.route_end_m = offsets[-1] + float(arc_lengths(paths[-1][1].poses)[-1])

    def route_progress(self) -> float:
        """Arc length along the whole route, as the traffic manager counts it."""
        if self.done:
            return self.route_end_m
        kind, stage, _, _ = self.items[self.item]
        if kind == "wait":  # lift after insert, lower after transport
            return self.offsets[self.items[self.item + 1][1]]
        return self.offsets[stage] + self.segments[self.segment][2] + self.progress

    def start_next(self, now, manager):
        self.item += 1
        if self.item >= len(self.items):
            self.done, self.finished_s = True, now
            return
        kind, stage, payload, _ = self.items[self.item]
        self.stage_times.setdefault(stage, now)
        if stage == "lift":
            self.loaded = True
        if kind == "wait":
            self.wait_left = payload
        else:
            self.loaded = stage in LOADED_STAGES
            self.segments, self.segment, self.progress, self.v = (
                _segments(payload),
                0,
                0.0,
                0.0,
            )
        manager.set_footprint(self.name, self.footprint())

    def step(self, now, permit, manager):
        if self.done:
            return
        kind, stage, payload, speed = self.items[self.item]
        if kind == "wait":
            self.wait_left -= DT_S
            if self.wait_left <= 1e-9:
                if stage == "lower":
                    self.loaded = False
                self.start_next(now, manager)
            return
        poses = self.segments[self.segment][0]
        s = arc_lengths(poses)
        remaining = s[-1] - self.progress
        room = min(remaining, permit.allowed_m)
        # Brake 1 cm short of a permit's end; at the path's own end (a cusp or
        # the goal, which the permit already covers) creep in and stop on it.
        end_limited = permit.allowed_m >= remaining - 1e-9
        target = min(speed, math.sqrt(2 * self.accel * max(room - 0.01, 0.0)))
        if end_limited:
            target = max(target, min(speed, 0.05))
        self.v = min(self.v + self.accel * DT_S, target)
        advance = min(self.v * DT_S, room)
        if end_limited and remaining - advance < 0.002:
            advance = remaining
        if advance <= 1e-6 and room < remaining:
            self.waiting_s += DT_S
        self.progress += max(advance, 0.0)
        self.pose = pose_at(poses, s, self.progress)
        if self.progress >= s[-1] - 1e-6:
            self.v = 0.0
            if self.segment + 1 < len(self.segments):
                self.segment += 1
                self.progress = 0.0
            else:
                self.start_next(now, manager)


def run_seed(seed, args, layout, assets, geometry, settings, configs) -> dict:
    config, travel = configs
    factory_assets_module, specs = assets
    fleet = make_fleet_scenario(
        seed,
        layout,
        [specs[n] for n in BAY_ASSETS],
        factory_assets_module.factory_assets(specs),
        geometry=geometry,
        trucks=args.trucks,
        parked_gap_m=2 * args.margin_m,
    )
    plans, planning = [], []
    for mission in fleet.missions:
        start = time.perf_counter()
        plan = plan_fleet_mission(mission, config, travel, geometry)
        planning.append(
            {
                "truck": mission.truck,
                "aisle": mission.aisle,
                "status": plan.status,
                "wall_s": round(time.perf_counter() - start, 2),
                "lengths_m": {
                    k: round(float(v.length_m), 2) for k, v in plan.stages.items()
                },
            }
        )
        plans.append(plan)
    out = {
        "seed": seed,
        "planning": planning,
        "assignment": [
            {
                "truck": m.truck,
                "aisle": m.aisle,
                "pickup": [m.pickup.x_m, m.pickup.y_m],
                "dock": [round(m.destination.x_m, 3), round(m.destination.y_m, 3)],
            }
            for m in fleet.missions
        ],
    }
    if not all(p.success for p in plans):
        out["status"] = "planning_failed"
        return out
    manager = TrafficManager(
        margin_m=args.margin_m,
        lookahead_m=args.lookahead_m,
        cycle_s=args.cycle_s,
        stall_s=args.stall_s,
    )
    trucks = [
        Truck(m.truck, m, p, geometry, settings)
        for m, p in zip(fleet.missions, plans, strict=True)
    ]
    for t in trucks:
        manager.add(t.name, t.k, t.pose, t.footprint())
    for t in trucks:
        t.set_route(manager)
        t.start_next(0.0, manager)
    manager.update(0.0, {t.name: t.pose for t in trucks}, {t.name: 0.0 for t in trucks})
    out["route_conflicts"] = manager.route_conflicts()
    overlaps, min_gap, events, trajectory = [], math.inf, [], []
    near = {m: 0 for m in (0.05, 0.1, 0.2)}
    now, idle_s, last_progress = 0.0, 0.0, None
    status = "timeout"
    reported = set()
    while now < args.max_sim_s:
        poses = {t.name: t.pose for t in trucks}
        progress = {t.name: t.route_progress() for t in trucks}
        permits = manager.update(now, poses, progress)
        for name, blocker in manager.deadlocks(now):
            episode = (name, blocker, manager.trucks[name].waiting_since_s)
            if episode in reported:
                continue
            reported.add(episode)
            t = next(x for x in trucks if x.name == name)
            cycle = manager.trucks[blocker].waiting_on == name
            events.append(
                {
                    "t_s": round(now, 1),
                    "truck": name,
                    "blocker": blocker,
                    "stage": t.stage,
                    "kind": "cycle" if cycle else "stall",
                }
            )
        for t in trucks:
            t.step(now, permits[t.name], manager)
        for i, a in enumerate(trucks):
            ra = footprint_rectangle(a.pose, a.footprint())
            for b in trucks[i + 1 :]:
                rb = footprint_rectangle(b.pose, b.footprint())
                min_gap = min(min_gap, math.hypot(ra.x_m - rb.x_m, ra.y_m - rb.y_m))
                if rectangles_overlap(ra, rb):
                    overlaps.append({"t_s": round(now, 1), "a": a.name, "b": b.name})
                for m in near:
                    if rectangles_overlap(
                        footprint_rectangle(a.pose, a.footprint(), m), rb
                    ):
                        near[m] += 1
        if abs(now - round(now)) < DT_S / 2:
            trajectory.append(
                [
                    [
                        *t.pose,
                        t.loaded,
                        STAGES.index(t.stage) if t.stage in STAGES else -1,
                    ]
                    for t in trucks
                ]
            )
        total = sum(t.route_progress() + t.item * 1000 for t in trucks)
        idle_s = (
            idle_s + DT_S
            if last_progress is not None and total == last_progress
            else 0.0
        )
        last_progress = total
        now += DT_S
        if all(t.done for t in trucks):
            status = "complete"
            break
        if idle_s > args.gridlock_s:
            status = "gridlock"
            break
    out.update(
        status=status,
        sim_s=round(now, 1),
        overlaps=overlaps[:50],
        overlap_steps=len(overlaps),
        near_steps={str(k): v for k, v in near.items()},
        min_centre_gap_m=round(min_gap, 3),
        deadlock_events=events,
        trucks=[
            {
                "truck": t.name,
                "finished_s": None if t.finished_s is None else round(t.finished_s, 1),
                "stage": t.stage,
                "waiting_s": round(t.waiting_s, 1),
                "stage_start_s": {k: round(v, 1) for k, v in t.stage_times.items()},
            }
            for t in trucks
        ],
    )
    out["_trajectory"] = np.asarray(trajectory, dtype=float)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=_seeds, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--layout", type=Path, default=ROOT / "config/fleet_full_warehouse.yaml"
    )
    parser.add_argument(
        "--settings", type=Path, default=ROOT / "config/isaac_transport_measured.yaml"
    )
    parser.add_argument(
        "--forklift-urdf",
        type=Path,
        default=ROOT / "sim/models/dls08_measured/forklift.urdf",
    )
    parser.add_argument(
        "--pallet-geometry",
        type=Path,
        default=ROOT / "config/pallet_geometry_epal6.yaml",
    )
    parser.add_argument("--insertion-reserve-m", type=float, default=0.016)
    parser.add_argument("--trucks", type=int, default=5)
    parser.add_argument("--expansions", type=int, default=30000)
    parser.add_argument("--heuristic-resolution-m", type=float, default=0.25)
    parser.add_argument("--margin-m", type=float, default=0.3)
    parser.add_argument("--lookahead-m", type=float, default=3.0)
    parser.add_argument("--cycle-s", type=float, default=5.0)
    parser.add_argument("--stall-s", type=float, default=20.0)
    parser.add_argument("--gridlock-s", type=float, default=300.0)
    parser.add_argument("--max-sim-s", type=float, default=2400.0)
    args = parser.parse_args(argv)
    settings = yaml.safe_load(args.settings.read_text())
    config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=args.expansions,
    )
    travel = replace(
        config, obstacle_heuristic_resolution_m=args.heuristic_resolution_m
    )
    geometry = mission_geometry(args)
    layout = load_fleet_layout(args.layout)
    factory_assets_module = _module("factory_assets")
    specs = factory_assets_module.offline_specs(ASSET_ROOT)
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for seed in args.seeds:
        start = time.perf_counter()
        result = run_seed(
            seed,
            args,
            layout,
            (factory_assets_module, specs),
            geometry,
            settings,
            (config, travel),
        )
        result["wall_s"] = round(time.perf_counter() - start, 1)
        folder = args.output / f"seed_{seed}"
        folder.mkdir(exist_ok=True)
        trajectory = result.pop("_trajectory", None)
        if trajectory is not None:
            np.savez_compressed(folder / "trajectory.npz", trajectory=trajectory)
        result["parameters"] = {
            k: (str(v) if isinstance(v, Path) else v)
            for k, v in vars(args).items()
            if k != "seeds"
        }
        (folder / "result.json").write_text(json.dumps(result, indent=1) + "\n")
        rows.append(
            {
                k: result.get(k)
                for k in (
                    "seed",
                    "status",
                    "sim_s",
                    "overlap_steps",
                    "min_centre_gap_m",
                    "wall_s",
                )
            }
        )
        print(json.dumps(rows[-1]), flush=True)
    (args.output / "summary.json").write_text(json.dumps(rows, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
