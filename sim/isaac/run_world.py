"""A fleet with continuous jobs on a random factory floor, in Isaac Sim.

    python sim/isaac/run_world.py --base-scene <measured base scene>/scene.usda \
        --structure rack --seed 200 --trucks 8 --people 8 --output <dir> [--faults] [--video]

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W6). The floor comes from
forklift_core.planning.world_gen (built by world_scene.py on the measured base
scene, whose warehouse is removed), jobs from world_mission's Dispatcher,
traffic from fleet_traffic -- the same modules tools/world_cpu_sim.py runs on
CPU. Here the trucks are the measured chassis under PhysX with the path
tracker; poses are the simulator's (no SLAM: cost, and the plan's choice).

- A pallet standing at a slot is a static stand-in. The truck given a job
  gets its own real pallet put at the source; after delivery that pallet is
  swapped back for a stand-in at the sink.
- People are kinematic capsules walking as in world_people. Each truck casts
  its 2D LiDAR (planar_lidar) at 10 Hz; returns off the known map (static
  boxes, slots, other trucks and pallets) fall in its protective field.
- --faults: one truck pauses for 60 s, and a spill (static box) appears on a
  truck's route; the trucks whose remaining route it touches stop and
  re-plan round it (rank kept).
- TA/RA (docs/plans/2026-10-10-fleet-ta-ra.md): traffic control's advice is
  carried out by world_resolution once the truck stands still -- a TA goes
  round a standing truck, an RA clears a higher-ranked route (detour round it
  as a wall, or back off along the stage driven so far). --ta-s/--ra-s 0
  turn them off.

Safety is judged on the truth every 0.1 s (truck-truck, truck-floor,
truck-standing pallet, truck-person) and recorded; nothing is stopped by it.
Writes result.json, world_log.npz (field: the distance the protective field
still allows, -1 when nothing is in it; pallets: each truck's real pallet,
x, y, yaw; route/progress/granted/rank: what traffic control holds per truck;
alert: 0 clear, 1 waiting for a truck (TA), 2 in RA,
with the routes themselves in routes.npz -- the claim, the "tablet", is a
route's samples from progress to granted), world.json, events.json,
camera.json and (--video) overview.mp4.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
import traceback
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import yaml
from run_fleet import native, require

ROOT = Path(__file__).resolve().parents[2]
IDLE_RANK = 10**9
LIFT_TIMEOUT_S = 30.0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--structure", required=True, choices=("rack", "shop", "multi"))
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--trucks", type=int, default=8)
    parser.add_argument("--people", type=int, default=8)
    parser.add_argument("--faults", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--forklift-urdf", type=Path, default=ROOT / "sim/models/dls08_measured/forklift.urdf"
    )
    parser.add_argument(
        "--pallet-urdf", type=Path, default=ROOT / "sim/models/epal6_pallet/pallet.urdf"
    )
    parser.add_argument(
        "--pallet-geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml"
    )
    parser.add_argument(
        "--settings", type=Path, default=ROOT / "config/isaac_transport_measured.yaml"
    )
    parser.add_argument("--lidar", type=Path, default=ROOT / "config/isaac_slam_lidar.yaml")
    parser.add_argument("--insertion-reserve-m", type=float, default=0.016)
    parser.add_argument("--expansions", type=int, default=30000)
    parser.add_argument("--margin-m", type=float, default=0.3)
    parser.add_argument("--lookahead-m", type=float, default=3.0)
    parser.add_argument("--stall-s", type=float, default=20.0)
    parser.add_argument("--ta-s", type=float, default=20.0)
    parser.add_argument("--ra-s", type=float, default=60.0)
    parser.add_argument("--cooldown-s", type=float, default=60.0)
    parser.add_argument("--wall-m", type=float, default=20.0)
    parser.add_argument("--retreat-max-m", type=float, default=20.0)
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--dress", action="store_true", help="NVIDIA props over the (hidden) collider boxes; looks only"
    )
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1440)
    parser.add_argument("--height", type=int, default=810)
    parser.add_argument("--max-sim-seconds", type=float, default=1200.0)
    args, unknown = parser.parse_known_args()
    if 120 % args.fps:
        parser.error("--fps must divide 120")
    args.kit_arguments = unknown
    return args


class Truck:
    """One truck's job stage machine and control state."""

    def __init__(self, k, robot, place, ctx):
        self.k, self.name = k, f"truck_{k}"
        self.prim = f"/World/Fleet/Truck_{k}"
        self.robot, self.ctx = robot, ctx
        names = list(robot.dof_names)
        self.wheels = np.array(
            [
                names.index(n)
                for n in (
                    "front_left_spin",
                    "front_right_spin",
                    "rear_left_spin",
                    "rear_right_spin",
                )
            ]
        )
        self.steers = np.array([names.index("left_steer"), names.index("right_steer")])
        self.lift_index = np.array([names.index("fork_lift")])
        self.place = place
        self.state = "idle"
        self.idle_until = 0.0
        self.job = None
        self.items: list = []
        self.item = -1
        self.loaded = False
        self.lift_command = 0.0
        self.steering_command = np.zeros(2)
        self.tracker = None
        self.legs: list = []
        self.leg = 0
        self.last_command = None
        self.offsets: dict = {}
        self.stage_started = 0.0
        self.waited_s = 0.0
        self.failed = None
        self.jobs_done = 0
        self.job_started = 0.0
        self.requested = 0.0
        self.paused_until = 0.0
        self.replan_pending = False
        self.person_stop_s = 0.0
        self.abandoned = False
        self.route_id = -1  # index into routes.npz; -1 before the first route
        self.advice = None  # TA/RA waiting for the truck to stand still

    @property
    def kind(self):
        return self.items[self.item][0] if self.state == "job" else "idle"

    @property
    def stage(self):
        return self.items[self.item][1] if self.state == "job" else "idle"

    def footprint(self):
        g = self.ctx["geometry"]
        return g.loaded_footprint if self.loaded else g.unloaded_footprint

    def log_stage(self) -> int:
        stages = self.ctx["log_stages"]
        return stages.index("broken" if self.failed else self.stage)

    def start_job(self, job, plan, now, rank):
        from forklift_core.planning.world_mission import STAGES

        self.job, self.state, self.job_started = job, "job", now
        self.items = []
        for stage in STAGES:
            self.items.append(("path", stage, plan.stages[stage]))
            if stage == "insert":
                self.items.append(("lift", "lift", None))
            if stage == "to_drop":
                self.items.append(("lower", "lower", None))
        self.item = -1
        self.set_route(0, rank=rank)
        self.next_item(now)

    def set_route(self, first_item, rank=None):
        g, loaded = self.ctx["geometry"], self.ctx["loaded_stages"]
        paths = [
            (stage, result)
            for kind, stage, result in self.items[first_item:]
            if kind == "path"
        ]
        offsets = self.ctx["traffic"].set_route(
            self.name,
            [
                (
                    np.asarray(result.poses),
                    g.loaded_footprint if stage in loaded else g.unloaded_footprint,
                )
                for stage, result in paths
            ],
            rank=rank,
        )
        self.offsets = dict(zip((s for s, _ in paths), offsets, strict=True))
        self.record_route()

    def record_route(self):
        """Keep the route traffic control now holds for this truck (routes.npz)."""
        held = self.ctx["traffic"].trucks[self.name]
        self.ctx["routes"].append(
            (self.k, held.rank, held.rects.astype(np.float32), held.g.astype(np.float32))
        )
        self.route_id = len(self.ctx["routes"]) - 1

    def go_idle(self, now, rng, pose):
        self.state, self.job, self.items, self.item = "idle", None, [], -1
        self.idle_until = now + float(rng.uniform(2.0, 10.0))
        self.ctx["traffic"].set_route(
            self.name,
            [(np.asarray(pose).reshape(1, 3), self.footprint())],
            rank=IDLE_RANK + self.k,
        )
        self.record_route()

    def next_item(self, now):
        self.item += 1
        if self.item >= len(self.items):
            return "done"
        self.stage_started = now
        kind, stage, payload = self.items[self.item]
        if kind == "path":
            self.loaded = stage in self.ctx["loaded_stages"]
            self.load_path(payload)
        self.ctx["traffic"].set_footprint(self.name, self.footprint())
        return "next"

    def load_path(self, result):
        from forklift_core.control import RearAxlePathTracker

        self.tracker = RearAxlePathTracker(
            result.poses,
            result.directions,
            result.curvatures_inv_m,
            self.ctx["tracker_config"](self.stage),
        )
        directions = np.asarray(result.directions)
        poses = np.asarray(result.poses)
        cuts = [0] + [
            i for i in range(1, len(poses) - 1) if directions[i + 1] != directions[i]
        ]
        cuts.append(len(poses) - 1)
        s = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T))))
        self.legs = [
            (a, b, s[b], int(directions[min(a + 1, b)]))
            for a, b in zip(cuts[:-1], cuts[1:], strict=True)
            if b > a
        ]
        self.leg = 0
        self.last_command = None

    def route_progress(self) -> float:
        if self.state != "job":
            return 0.0
        if self.kind != "path":  # lift after insert, lower after to_drop
            return self.offsets[self.items[self.item + 1][1]]
        along = 0.0 if self.last_command is None else self.last_command.progress_m
        return self.offsets[self.stage] + along

    def leg_end_m(self, command) -> float:
        while self.leg + 1 < len(self.legs) and command.segment_index >= self.legs[self.leg][1]:
            self.leg += 1
        return self.offsets[self.stage] + self.legs[self.leg][2]

    def direction(self) -> int:
        return self.legs[self.leg][3] if self.kind == "path" and self.legs else 1


def run(app, args, settings, state) -> None:
    import omni.usd
    import planar_lidar
    from chassis_contract import (
        chassis_record,
        require_scene_matches_model,
        stage_chassis,
    )
    from insertion_geometry import (
        InsertionGeometry,
        read_carriage_limit_m,
        read_chassis_reference_m,
        read_drive_geometry_m,
    )
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.sensors.camera import Camera
    from pxr import Gf
    from scene import configure_drives, create_pallet
    from world_dressing import Workers, dress_spill, dress_world
    from world_scene import add_box, add_people, build_world, move_person, set_stand

    from forklift_core.control import TrackerConfig, ackermann_command
    from forklift_core.perception.pallet_geometry import load_pallet_geometry
    from forklift_core.planning import Footprint, Pose2D
    from forklift_core.planning.fleet_traffic import (
        TrafficManager,
        oriented_rects,
        overlap_matrix,
    )
    from forklift_core.planning.pallet_mission import (
        PalletSite,
        SyntheticMissionGeometry,
        make_transport_planner_config,
    )
    from forklift_core.planning.world_gen import (
        PALLET_KEEP_M,
        Box,
        make_world,
        pallet_box,
        world_record,
    )
    from forklift_core.planning.world_mission import (
        LOADED_STAGES,
        STAGES,
        TRAVEL_STAGES,
        Dispatcher,
        plan_job,
    )
    from forklift_core.planning.world_people import (
        PERSON_RADIUS,
        People,
        protective_block_m,
    )
    from forklift_core.planning.world_resolution import (
        own_pallets_keepout,
        rectangles,
        resolution_counts,
        resolve,
        wait_stats,
    )
    from forklift_core.sensors.lidar import PlanarScanPattern

    rng = np.random.default_rng([args.seed, args.trucks, 0x15AAC])
    drive_geometry = read_drive_geometry_m(args.forklift_urdf, settings["max_wheel_rate_rad_s"])
    axle_to_fork_tip_m, rear_offset_m = read_chassis_reference_m(args.forklift_urdf)
    rear_offset_m = abs(rear_offset_m)
    pallet_geometry = load_pallet_geometry(args.pallet_geometry)
    geometry = SyntheticMissionGeometry(
        unloaded_footprint=Footprint(axle_to_fork_tip_m, 0.17, 0.36),
        pallet_depth_m=pallet_geometry.overall_depth_m,
        pallet_width_m=pallet_geometry.overall_width_m,
        axle_to_fork_tip_m=axle_to_fork_tip_m,
        carriage_limit_m=read_carriage_limit_m(args.forklift_urdf),
        insertion_reserve_m=args.insertion_reserve_m,
        delivery_straight_m=0.70,
    )
    state["geometry"] = asdict(geometry)
    config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=args.expansions,
    )
    travel = replace(config, obstacle_heuristic_resolution_m=0.25)
    wm = make_world(args.structure, args.seed, geometry=geometry, trucks=args.trucks)
    state["world"] = {
        "size_m": wm.size_m,
        "sources": len(wm.sources()),
        "sinks": len(wm.sinks()),
        "holding": len(wm.holding),
        "obstacles": len(wm.obstacles),
        "dropped": wm.dropped,
    }

    state["phase"] = "scene"
    enable_extension("isaacsim.asset.importer.urdf")
    require(omni.usd.get_context().open_stage(args.base_scene), "Cannot open base scene")
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    template = stage.GetPrimAtPath("/World/Forklift")
    require(template.IsValid(), "base scene has no /World/Forklift")
    truck_usd = list(template.GetMetadata("references").GetAddedOrExplicitItems())[0].assetPath
    stage.RemovePrim("/World/Forklift")
    scene_info = build_world(stage, wm, geometry)
    dresser = None
    if args.dress:
        state["phase"] = "dressing"
        dressed = dress_world(stage, app, wm, geometry, scene_info, np.random.default_rng([args.seed, 0xD7E55]))
        dresser = dressed.pop("dresser")
        state["dressing"] = dressed
        state["phase"] = "scene"
    for k in range(args.trucks):
        stage.DefinePrim(f"/World/Fleet/Truck_{k}", "Xform").GetReferences().AddReference(truck_usd)
    for _ in range(10):
        app.update()
    for k in range(args.trucks):
        chassis = stage_chassis(stage, f"/World/Fleet/Truck_{k}")
        require_scene_matches_model(chassis, args.forklift_urdf)
    state["scene_chassis"] = chassis_record(chassis)

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    dispatcher = Dispatcher(wm, np.random.default_rng([args.seed, args.trucks, 11]))
    places = dispatcher.spawn(args.trucks)
    b = wm.bounds
    pool = [
        PalletSite(b.x_max_m + 6.0 + 1.6 * (k % 8), b.y_min_m - 6.0 - 1.6 * (k // 8), 0.0)
        for k in range(args.trucks)
    ]
    pallets = [
        create_pallet(
            world,
            stage,
            app,
            args.pallet_urdf,
            args.output,
            pool[k],
            settings["pallet_mass_kg"],
            prim_path=f"/World/Fleet/Pallet_{k}",
            name=f"pallet_{k}",
        )
        for k in range(args.trucks)
    ]
    robots = []
    for k, place in enumerate(places):
        p = wm.holding[place]
        robots.append(
            world.scene.add(
                Robot(
                    prim_path=f"/World/Fleet/Truck_{k}",
                    name=f"truck_{k}",
                    position=np.array(
                        [
                            p.x_m + rear_offset_m * math.cos(p.yaw_rad),
                            p.y_m + rear_offset_m * math.sin(p.yaw_rad),
                            0.015,
                        ]
                    ),
                    orientation=np.array([math.cos(p.yaw_rad / 2), 0, 0, math.sin(p.yaw_rad / 2)]),
                )
            )
        )
    boxes = len(InsertionGeometry.from_urdfs(args.forklift_urdf, args.pallet_urdf).pallet_boxes)
    state["collision_counts"] = configure_drives(
        stage,
        settings,
        expected_pallet_box_count=boxes,
        forklift_roots=tuple(f"/World/Fleet/Truck_{k}" for k in range(args.trucks)),
        pallet_roots=tuple(f"/World/Fleet/Pallet_{k}" for k in range(args.trucks)),
    )
    person_paths = add_people(stage, args.people, PERSON_RADIUS)
    people = People(wm, geometry, args.people, rng)
    workers = (
        Workers(dresser, stage, person_paths, np.random.default_rng([args.seed, 0x3E0]))
        if dresser is not None
        else None
    )

    camera = None
    if args.video:
        camera = Camera(
            prim_path="/World/WorldCamera", frequency=-1, resolution=(args.width, args.height)
        )
        centre = np.array([(b.x_min_m + b.x_max_m) / 2, (b.y_min_m + b.y_max_m) / 2, 0.0])
        width_m, height_m = b.x_max_m - b.x_min_m + 4, b.y_max_m - b.y_min_m + 4
        wide = width_m >= height_m
        across, down = (width_m, height_m) if wide else (height_m, width_m)
        focal, aperture = 0.85, 2.0955
        tan_h = aperture / 2 / focal
        tan_v = tan_h * args.height / args.width
        eye_z = max(across / 2 / tan_h, down / 2 / tan_v) * 1.04 + 3.2
        eye = centre + np.array([0.0, 0.0, eye_z])
        up = (0, 1, 0) if wide else (-1, 0, 0)
        look = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*centre), Gf.Vec3d(*up))
        q = look.GetInverse().ExtractRotationQuat()
        camera.set_world_pose(
            position=eye,
            orientation=np.array([q.GetReal(), *q.GetImaginary()]),
            camera_axes="usd",
        )
        camera.set_focal_length(focal)
        state["camera"] = {
            "eye_m": eye.tolist(),
            "target_m": centre.tolist(),
            "up": list(up),
            "focal_length": focal,
            "resolution": [args.width, args.height],
            "fps": args.fps,
        }
    world.reset()
    if camera is not None:
        camera.initialize()
    for _ in range(120):
        world.step(render=args.video)
    if camera is not None:
        (args.output / "camera.json").write_text(
            json.dumps(
                {
                    **state["camera"],
                    "horizontal_aperture": float(camera.get_horizontal_aperture()),
                    "vertical_aperture": float(camera.get_vertical_aperture()),
                    "focal_length_read": float(camera.get_focal_length()),
                },
                indent=2,
            )
            + "\n"
        )
    pallet_z = float(pallets[0].get_world_pose()[0][2])
    slots = {s.name: s for s in wm.slots}
    handled: dict[str, int] = {}  # slot -> truck whose real pallet stands there
    shown = {}

    def sync_stands():
        for name, slot in slots.items():
            want = dispatcher.occupied[name] and name not in handled
            if shown.get(name) != want:
                set_stand(stage, scene_info["stands"][name], slot.site.x_m, slot.site.y_m, want)
                shown[name] = want

    def put_pallet(k, site):
        yaw = site.yaw_rad
        pallets[k].set_world_pose(
            position=np.array([site.x_m, site.y_m, pallet_z]),
            orientation=np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]),
        )
        pallets[k].set_linear_velocity(np.zeros(3))
        pallets[k].set_angular_velocity(np.zeros(3))

    sync_stands()

    def tracker_config(stage_name):
        speeds = {
            "leave": "withdraw_speed_mps",
            "to_pickup": "approach_speed_mps",
            "insert": "insert_speed_mps",
            "extract": "extract_speed_mps",
            "to_drop": "transport_speed_mps",
            "withdraw": "withdraw_speed_mps",
            "to_holding": "approach_speed_mps",
        }
        travel_leg = stage_name in TRAVEL_STAGES
        return TrackerConfig(
            cruise_speed_mps=settings[speeds[stage_name]],
            max_curvature_inv_m=settings["tracker_curvature_inv_m"],
            max_acceleration_mps2=settings["drive_acceleration_mps2"],
            lookahead_m=0.28,
            position_tolerance_m=0.03 if stage_name == "to_holding" else 0.008,
            yaw_tolerance_rad=0.03 if stage_name == "to_holding" else 0.02,
            cusp_position_tolerance_m=0.10 if travel_leg else 0.03,
            cusp_yaw_tolerance_rad=0.08 if travel_leg else 0.05,
            cusp_brake_window_m=0.008,
            max_lateral_acceleration_mps2=settings.get("max_lateral_acceleration_mps2"),
            max_reverse_speed_mps=settings.get("max_reverse_speed_mps"),
            overshoot_tolerance_m=None if stage_name == "insert" else 0.03,
            stop_speed_mps=0.012,
            max_cross_track_error_m=0.35,
        )

    traffic = TrafficManager(
        margin_m=args.margin_m,
        lookahead_m=args.lookahead_m,
        stall_s=args.stall_s,
        ta_s=args.ta_s if args.ta_s > 0 else None,
        ra_s=args.ra_s if args.ra_s > 0 else None,
        cooldown_s=args.cooldown_s,
    )
    log_stages = ("idle", *STAGES, "lift", "lower", "broken")
    ctx = {
        "geometry": geometry,
        "traffic": traffic,
        "tracker_config": tracker_config,
        "loaded_stages": LOADED_STAGES,
        "log_stages": log_stages,
        "routes": [],
    }
    trucks = [Truck(k, robots[k], places[k], ctx) for k in range(args.trucks)]
    named = {t.name: t for t in trucks}

    def truth(t):
        base, q = t.robot.get_world_pose()
        w, x, y, z = q
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        rear = np.array(
            [base[0] - rear_offset_m * math.cos(yaw), base[1] - rear_offset_m * math.sin(yaw), yaw]
        )
        velocity = t.robot.get_linear_velocity()
        speed = float(velocity[0] * math.cos(yaw) + velocity[1] * math.sin(yaw))
        return rear, speed

    for t in trucks:
        traffic.add(t.name, IDLE_RANK + t.k, truth(t)[0], t.footprint())
        t.idle_until = float(rng.uniform(0.0, 5.0))

    lidar = yaml.safe_load(args.lidar.read_text())
    pattern = PlanarScanPattern(
        lidar["beam_count"], float(lidar["range_min_m"]), float(lidar["range_max_m"])
    )
    mount = planar_lidar.LaserMount(tuple(map(float, lidar["mount_xyz_m"])), float(lidar["mount_yaw_rad"]))
    # The field needs people, not a map: every 4th beam (0.9 deg), 8 m.
    beam_angles = pattern.beam_angles_rad()[::4]
    field_range_m = min(8.0, pattern.range_max_m)
    scan_every = 120 // int(lidar["rate_hz"])
    static_rects = oriented_rects(
        np.array([[(x.x0 + x.x1) / 2, (x.y0 + x.y1) / 2, 0.0] for _, x in wm.obstacles]),
        [Footprint((x.x1 - x.x0) / 2, (x.x1 - x.x0) / 2, (x.y1 - x.y0) / 2) for _, x in wm.obstacles],
        0.0,
    )
    slot_names = list(slots)
    slot_rects = oriented_rects(
        np.array([[s.site.x_m, s.site.y_m, s.site.yaw_rad] for s in wm.slots]),
        Footprint(geometry.pallet_depth_m / 2, geometry.pallet_depth_m / 2, geometry.pallet_width_m / 2),
        0.0,
    )
    pallet_fp = Footprint(geometry.pallet_depth_m / 2, geometry.pallet_depth_m / 2, geometry.pallet_width_m / 2)
    slot_keep = {s.name: pallet_box(s.site, geometry).grow(PALLET_KEEP_M) for s in wm.slots}
    extra_boxes: list[tuple[str, Box]] = []

    def box_rects(items):
        if not items:
            return np.zeros((0, 6))
        return oriented_rects(
            np.array([[(x.x0 + x.x1) / 2, (x.y0 + x.y1) / 2, 0.0] for _, x in items]),
            [Footprint((x.x1 - x.x0) / 2, (x.x1 - x.x0) / 2, (x.y1 - x.y0) / 2) for _, x in items],
            0.0,
        )

    def plan(job, first_stage="leave", start=None, avoid=()):
        started = time.monotonic()
        result = plan_job(
            job, wm, config, travel, geometry,
            first_stage=first_stage, start=start, extra=[x.rect() for _, x in extra_boxes] + list(avoid),
        )
        plans.append({"job": job.number, "truck": job.truck, "from": first_stage,
                      "status": result.status, "wall_s": round(time.monotonic() - started, 2)})
        return result

    faults = []
    if args.faults:
        faults = sorted([(float(rng.uniform(150, 400)), "pause"), (float(rng.uniform(200, 600)), "spill")])
    encoder = None
    if args.video:
        encoder = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{args.width}x{args.height}", "-r", str(args.fps), "-i", "-",
             "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", str(args.output / "overview.mp4")],
            stdin=subprocess.PIPE,
        )
    log = {
        k: []
        for k in (
            "t_s", "truth", "stage", "loaded", "blocked", "allowed", "field", "pallets",
            "route", "progress", "granted", "rank", "alert", "people", "jobs",
        )
    }
    events, contacts, plans, job_log = [], [], [], []
    reported = set()
    frames, tick = 0, 0
    initial_time = world.current_time
    frame_every = 120 // args.fps
    people_every = 4
    dt = 1 / 120
    permits = {t.name: None for t in trucks}
    blocks = {t.name: None for t in trucks}
    last_seen, last_motion_s = {}, 0.0
    state["phase"] = "running"

    def unknown_points(t, truths):
        """LiDAR returns of truck t that the known map does not explain."""
        base, q = t.robot.get_world_pose()
        origin, directions = planar_lidar.laser_rays_world(base, q, mount, beam_angles)
        distances, hits, _ = planar_lidar.cast_scan(
            origin, directions, field_range_m, self_prefix=t.prim + "/"
        )
        points = planar_lidar.scan_points_world(origin, directions, np.where(hits, distances, np.inf))[:, :2]
        if not len(points):
            return points
        known = [static_rects.copy(), slot_rects.copy(), box_rects(extra_boxes)]
        others = [u for u in trucks if u is not t]
        if others:
            known.append(oriented_rects(np.array([truths[u.name][0] for u in others]), [u.footprint() for u in others], 0.0))
        pall = np.array([[*pallets[k].get_world_pose()[0][:2], 0.0] for k in range(len(pallets))])
        known.append(oriented_rects(pall, pallet_fp, 0.0))
        known = np.vstack(known)
        known[:, 2:4] += 0.2  # grow every known box by 0.2 m
        dots = np.column_stack((points, np.full(len(points), 0.01), np.full(len(points), 0.01), np.ones(len(points)), np.zeros(len(points))))
        explained = overlap_matrix(dots, known).any(axis=1)
        return points[~explained]

    def replan_now(t, rear, why):
        kind, stage_name, _ = t.items[t.item]
        if kind == "path":
            first, keep = stage_name, t.item
        else:
            keep = t.item + 1
            first = t.items[keep][1]
        # Round where the lower-ranked trucks may stop (plan 2026-10-10); with
        # no way round them, as before.
        start = Pose2D(*map(float, rear))
        stops = rectangles(traffic.stop_keepout(t.name, config.clearance_m + 0.05))
        own = own_pallets_keepout(traffic, t.name, t.job, stage_name, dispatcher.occupied, geometry, config.clearance_m + 0.05)
        result = plan(t.job, first, start, stops + own)
        kept = result.success or not stops
        if not result.success and stops:
            result = plan(t.job, first, start, own)
        events.append({"t_s": round(now, 2), "kind": "replan", "truck": t.name, "from": first, "why": why, "status": result.status, "stops_kept": kept})
        if not result.success:
            return False
        for i in range(keep, len(t.items)):
            k_i, s_i, _ = t.items[i]
            if k_i == "path":
                t.items[i] = (k_i, s_i, result.stages[s_i])
        if keep == t.item:
            t.load_path(t.items[t.item][2])
            t.stage_started = now  # the stage's time limit counts from the new path
        # Rank kept (plan 2026-10-10): sending it to the back left it standing on
        # higher-ranked routes; a lock that the new route makes, RA clears.
        t.set_route(keep)
        return True

    def carry_out(t, advice, rear):
        """A TA or RA for a truck standing still: its stage again, rank kept."""
        record = {"t_s": round(now, 2), "kind": advice.kind, "truck": t.name, "other": advice.other, "why": advice.why}
        if t.kind != "path":
            events.append({**record, "how": "skipped"})
            return
        # Given when the wait was seen; the truck may have taken a while to stop.
        if advice.why == "in_the_way":
            still = traffic.trucks[advice.other].waiting_on == t.name
        else:
            still = traffic.trucks[t.name].waiting_on is not None
        if not still:
            events.append({**record, "how": "stale"})
            return
        stage_name = t.stage
        res = resolve(
            advice,
            traffic,
            stage=stage_name,
            path=t.items[t.item][2],
            driven_m=0.0 if t.last_command is None else t.last_command.progress_m,
            stage_start_m=t.offsets[stage_name],
            pose=rear,
            job=t.job,
            world=wm,
            config=config,
            travel_config=travel,
            geometry=geometry,
            extra=[x.rect() for _, x in extra_boxes]
            + own_pallets_keepout(traffic, t.name, t.job, stage_name, dispatcher.occupied, geometry, config.clearance_m + 0.05),
            wall_m=args.wall_m,
            retreat_max_m=args.retreat_max_m,
            clock=time.monotonic,
        )
        for what, status, wall_s in res.plans:
            plans.append({"job": t.job.number, "truck": t.k, "from": f"{what}:{stage_name}", "status": status, "wall_s": wall_s})
        events.append({**record, **res.record()})
        if res.path is None:
            if res.how == "failed":
                traffic.failed(advice)
            return
        t.items[t.item] = ("path", stage_name, res.path)
        t.load_path(res.path)
        t.stage_started = now
        t.set_route(t.item)
        if advice.kind == "ra":
            traffic.resolved(t.name, advice.other)

    def affected(t, box):
        if t.state != "job":
            return False
        mine = traffic.trucks[t.name]
        ahead = mine.rects[mine.g >= t.route_progress() - 0.2]
        return bool(overlap_matrix(ahead, box_rects([("spill", box.grow(args.margin_m))])).any())

    now = 0.0
    while world.current_time - initial_time < args.max_sim_seconds:
        now = world.current_time - initial_time
        truths = {t.name: truth(t) for t in trucks}
        scan_tick = tick % scan_every == 0
        if scan_tick:
            occupied_before = dict(dispatcher.occupied)
            # No pallet comes back under a truck (its slot keep-out).
            bodies_now = oriented_rects(np.array([truths[t.name][0] for t in trucks]), [t.footprint() for t in trucks], 0.0)
            dispatcher.tick(now, lambda name, bodies=bodies_now: bool(overlap_matrix(box_rects([("slot", slot_keep[name])]), bodies).any()))
            if occupied_before != dispatcher.occupied:
                sync_stands()
            # -- faults ------------------------------------------------------
            while faults and faults[0][0] <= now:
                _, kind = faults.pop(0)
                moving = [t for t in trucks if t.state == "job" and abs(truths[t.name][1]) > 0.05]
                if kind == "pause" and moving:
                    t = moving[int(rng.integers(len(moving)))]
                    t.paused_until = now + 60.0
                    events.append({"t_s": round(now, 2), "kind": "pause", "truck": t.name, "until": round(t.paused_until, 1)})
                elif kind == "spill":
                    box = None
                    for t in rng.permutation(trucks):
                        if t.state != "job":
                            continue
                        mine = traffic.trucks[t.name]
                        idx = np.nonzero(mine.g > t.route_progress() + 8.0)[0]
                        if not len(idx):
                            continue
                        r = mine.rects[idx[int(rng.integers(len(idx)))]]
                        size = float(rng.uniform(1.0, 1.8))
                        cand = Box(r[0] - size / 2, r[1] - size / 2, r[0] + size / 2, r[1] + size / 2)
                        rect = box_rects([("spill", cand.grow(0.3))])
                        if any(overlap_matrix(rect, traffic.trucks[u.name].claim).any() for u in trucks):
                            continue
                        if overlap_matrix(rect, slot_rects).any():
                            continue
                        box = cand
                        break
                    if box is not None:
                        extra_boxes.append(("spill", box))
                        add_box(stage, f"/World/Factory/Spill_{len(extra_boxes)}", "spill", box)
                        if dresser is not None:
                            dress_spill(dresser, stage, f"/World/Factory/Spill_{len(extra_boxes)}", box)
                        events.append({"t_s": round(now, 2), "kind": "spill", "box": [round(v, 2) for v in (box.x0, box.y0, box.x1, box.y1)]})
                        for t in trucks:
                            if affected(t, box):
                                t.replan_pending = True
            # -- idle trucks ask for work ------------------------------------
            for t in trucks:
                if t.state == "idle" and now >= t.idle_until and not t.failed:
                    job = dispatcher.next_job(t.k, t.place)
                    if job is None:
                        t.idle_until = now + 5.0
                        continue
                    result = plan(job)
                    if not result.success:
                        dispatcher.abandon(job, reached_pickup=False, unreachable=True)
                        t.idle_until = now + 5.0
                        job_log.append({"job": job.number, "truck": t.k, "status": "plan_failed", "why": result.status, "t_s": round(now, 1)})
                        continue
                    put_pallet(t.k, job.source.site)
                    handled[job.source.name] = t.k
                    sync_stands()
                    t.place = job.home
                    t.start_job(job, result, now, dispatcher.next_rank())
                    for c in traffic.route_conflicts():
                        if t.name in (c["truck"], c["route_of"]):
                            events.append({"t_s": round(now, 2), "kind": "precondition", **c})
            # -- traffic -------------------------------------------------------
            poses = {t.name: truths[t.name][0] for t in trucks}
            progress = {t.name: t.route_progress() for t in trucks}
            permits = traffic.update(now, poses, progress)
            for name, blocker in traffic.deadlocks(now):
                episode = (name, blocker, traffic.trucks[name].waiting_since_s)
                if episode not in reported:
                    reported.add(episode)
                    cycle = traffic.trucks[blocker].waiting_on == name
                    events.append({"t_s": round(now, 2), "kind": "cycle" if cycle else "stall", "truck": name, "blocker": blocker})
            for advice in traffic.advisories(now):
                t = named[advice.truck]
                if advice.kind == "cc":
                    events.append({"t_s": round(now, 2), "kind": "cc", "truck": t.name, "other": advice.other})
                elif t.state == "job" and not t.failed and t.advice is None:
                    t.advice = advice  # carried out once the truck stands still
                else:
                    events.append({"t_s": round(now, 2), "kind": advice.kind, "truck": t.name, "other": advice.other, "why": advice.why, "how": "skipped"})
            # -- protective fields from the LiDAR ---------------------------
            for t in trucks:
                blocks[t.name] = None
                if args.people and t.state == "job" and t.kind == "path" and not t.failed:
                    points = unknown_points(t, truths)
                    blocks[t.name] = protective_block_m(
                        truths[t.name][0], t.footprint(), t.direction(), abs(truths[t.name][1]),
                        settings["drive_acceleration_mps2"], points,
                    )
            # -- safety on the truth ----------------------------------------
            bodies = oriented_rects(np.array([truths[t.name][0] for t in trucks]), [t.footprint() for t in trucks], 0.0)
            touch = overlap_matrix(bodies, bodies)
            for i in range(len(trucks)):
                for j in range(i + 1, len(trucks)):
                    if touch[i, j]:
                        contacts.append({"t_s": round(now, 2), "kind": "truck", "a": trucks[i].name, "b": trucks[j].name})
            floor = static_rects if not extra_boxes else np.vstack((static_rects, box_rects(extra_boxes)))
            hit_floor = overlap_matrix(bodies, floor).any(axis=1)
            standing = np.array([shown.get(n, False) for n in slot_names])
            on_pallet = overlap_matrix(bodies, slot_rects) & standing[None, :]
            pallet_poses = np.array([[*pallets[k].get_world_pose()[0][:2], yaw_of(pallets[k])] for k in range(len(pallets))])
            pall = oriented_rects(pallet_poses, pallet_fp, 0.0)
            on_real = overlap_matrix(bodies, pall)
            for i, t in enumerate(trucks):
                if hit_floor[i]:
                    contacts.append({"t_s": round(now, 2), "kind": "floor", "a": t.name, "stage": t.stage})
                if on_pallet[i].any():
                    contacts.append({"t_s": round(now, 2), "kind": "pallet", "a": t.name, "stage": t.stage})
                others_hit = [k for k in range(len(pallets)) if k != t.k and on_real[i, k]]
                if others_hit:
                    contacts.append({"t_s": round(now, 2), "kind": "pallet", "a": t.name, "stage": t.stage, "pallet_of": others_hit})
            if len(people.pos):
                discs = np.column_stack((people.pos, np.full(len(people.pos), PERSON_RADIUS), np.full(len(people.pos), PERSON_RADIUS), np.ones(len(people.pos)), np.zeros(len(people.pos))))
                hit = overlap_matrix(bodies, discs)
                for i, j in zip(*np.nonzero(hit), strict=True):
                    contacts.append({"t_s": round(now, 2), "kind": "person", "a": trucks[i].name, "person": int(j), "truck_moving": bool(abs(truths[trucks[i].name][1]) > 0.02)})
            # -- log -------------------------------------------------------
            log["t_s"].append(now)
            log["truth"].append([truths[t.name][0] for t in trucks])
            log["stage"].append([t.log_stage() for t in trucks])
            log["loaded"].append([t.loaded for t in trucks])
            log["blocked"].append([
                -1 if permits[t.name] is None or permits[t.name].blocked_by is None or permits[t.name].allowed_m >= 0.05
                else int(permits[t.name].blocked_by[6:]) for t in trucks
            ])
            log["allowed"].append([0.0 if permits[t.name] is None else permits[t.name].allowed_m for t in trucks])
            log["field"].append([-1.0 if blocks[t.name] is None else max(blocks[t.name], 0.0) for t in trucks])
            log["pallets"].append(pallet_poses.tolist())
            log["route"].append([t.route_id for t in trucks])
            log["progress"].append([traffic.trucks[t.name].progress_m for t in trucks])
            log["granted"].append([traffic.trucks[t.name].granted_m for t in trucks])
            log["rank"].append([float(traffic.trucks[t.name].rank) for t in trucks])
            log["alert"].append([
                2 if t.name in traffic.ra_against else 1 if log["blocked"][-1][k] >= 0 else 0
                for k, t in enumerate(trucks)
            ])
            log["people"].append(people.pos.copy() if len(people.pos) else np.zeros((0, 2)))
            log["jobs"].append([t.jobs_done for t in trucks])
            moved = [float(np.hypot(*(truths[t.name][0][:2] - last_seen.get(t.name, truths[t.name][0])[:2]))) for t in trucks]
            if any(m > 0.01 for m in moved) or not any(t.state == "job" for t in trucks):
                last_motion_s = now
            last_seen = {t.name: truths[t.name][0].copy() for t in trucks}
            if now - last_motion_s > 180.0:
                state["stopped"] = f"gridlock: no truck moved for 180 s at {now:.1f} s"
                break
        if tick % people_every == 0 and len(people.pos):
            bodies_now = oriented_rects(np.array([truths[t.name][0] for t in trucks]), [t.footprint() for t in trucks], 0.0)
            people.step(dt * people_every, bodies_now)
            for path, (x, y) in zip(person_paths, people.pos, strict=True):
                move_person(stage, path, float(x), float(y))
            if workers is not None:
                for k, (x, y) in enumerate(people.pos):
                    workers.move(k, float(x), float(y))
        # -- control ---------------------------------------------------------
        for t in trucks:
            rear, speed = truths[t.name]
            requested, curvature = 0.0, 0.0
            if t.state != "job" or t.failed or now < t.paused_until:
                pass
            elif t.replan_pending and (t.kind != "path" or t.stage in TRAVEL_STAGES):
                if abs(speed) < 0.02:
                    t.replan_pending = False
                    if not replan_now(t, rear, "spill"):
                        t.failed = "no route round the spill"  # plan W3: the job fails
            elif t.advice is not None and abs(speed) < 0.02:
                advice, t.advice = t.advice, None
                carry_out(t, advice, rear)
            elif t.kind == "path":
                command = t.tracker.update(rear, speed, dt)
                t.last_command = command
                if command.status == "failed":
                    if t.stage in TRAVEL_STAGES and abs(speed) < 0.02:
                        if not replan_now(t, rear, f"tracking {command.failure}"):
                            t.failed = f"tracking {command.failure} in {t.stage}"
                    elif t.stage not in TRAVEL_STAGES:
                        t.failed = f"tracking {command.failure} in {t.stage}"
                elif command.status == "arrived":
                    finished = t.stage
                    if finished == "leave":
                        dispatcher.left(t.job)
                    if finished == "withdraw":
                        # The delivered pallet becomes a stand-in; the real one goes back.
                        handled.pop(t.job.sink.name, None)
                        put_pallet(t.k, pool[t.k])
                        sync_stands()
                    if t.next_item(now) == "done":
                        t.jobs_done += 1
                        job_log.append({"job": t.job.number, "truck": t.k, "status": "done", "t_s": round(now, 1), "took_s": round(now - t.job_started, 1)})
                        t.place = t.job.home
                        t.go_idle(now, rng, rear)
                else:
                    requested, curvature = command.speed_mps, command.curvature_inv_m
                    permit = permits.get(t.name)
                    leg_end = t.leg_end_m(command)
                    if permit is not None and permit.granted_m < leg_end - 1e-6:
                        allowed = permit.granted_m - t.route_progress()
                        cap = math.sqrt(2 * settings["drive_acceleration_mps2"] * max(allowed - 0.02, 0.0))
                        if abs(requested) > cap:
                            requested = math.copysign(cap, requested)
                            t.waited_s += dt if cap < 0.02 else 0.0
                    block = blocks.get(t.name)
                    if block is not None:
                        cap = math.sqrt(2 * settings["drive_acceleration_mps2"] * max(block - 0.02, 0.0))
                        if abs(requested) > cap:
                            requested = math.copysign(cap, requested)
                            t.person_stop_s += dt if cap < 0.02 else 0.0
                if now - t.stage_started > 3 * max(t.tracker.nominal_duration_s(), 10.0) + t.waited_s + t.person_stop_s + 120:
                    t.failed = t.failed or f"timeout in {t.stage}"
            elif t.kind == "lift":
                t.lift_command = min(settings["lift_target_m"], t.lift_command + settings["lift_rate_mps"] * dt)
                pos, _ = pallets[t.k].get_world_pose()
                if t.lift_command >= settings["lift_target_m"] and pos[2] > pallet_z + 0.05:
                    t.loaded = True
                    dispatcher.lifted(t.job, now)
                    handled.pop(t.job.source.name, None)
                    sync_stands()
                    t.next_item(now)
                elif now - t.stage_started > LIFT_TIMEOUT_S:
                    t.failed = "lift_timeout"
            elif t.kind == "lower":
                t.lift_command = max(0.0, t.lift_command - settings["lift_rate_mps"] * dt)
                if t.lift_command <= 0.0 and now - t.stage_started > settings["lift_target_m"] / settings["lift_rate_mps"] + 1.0:
                    t.loaded = False
                    dispatcher.lowered(t.job, now)
                    handled[t.job.sink.name] = t.k
                    sync_stands()
                    t.next_item(now)
            if t.failed and t.job is not None and not t.abandoned:
                t.abandoned = True
                dispatcher.abandon(t.job, reached_pickup=t.loaded, unreachable=False)
                events.append({"t_s": round(now, 2), "kind": "truck_failed", "truck": t.name, "why": t.failed})
            t.requested = requested
            drive = ackermann_command(requested, curvature, drive_geometry)
            target = np.asarray(drive.steering_rad)
            actual = t.robot.get_joint_positions()[t.steers]
            if float(np.max(np.abs(target - actual))) > 0.05:
                drive = ackermann_command(requested * 0.25, curvature, drive_geometry)
            rate = settings["steering_command_rate_rad_s"] * dt
            t.steering_command += np.clip(target - t.steering_command, -rate, rate)
            t.robot.apply_action(ArticulationAction(joint_velocities=np.asarray(drive.wheel_rates_rad_s), joint_indices=t.wheels))
            t.robot.apply_action(ArticulationAction(joint_positions=t.steering_command, joint_indices=t.steers))
            t.robot.apply_action(ArticulationAction(joint_positions=np.array([t.lift_command]), joint_indices=t.lift_index))
        frame_due = args.video and tick % frame_every == 0
        world.step(render=bool(frame_due))
        tick += 1
        if frame_due:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.size:
                encoder.stdin.write(np.ascontiguousarray(rgba[:, :, :3]).astype(np.uint8).tobytes())
                frames += 1
    if encoder is not None:
        encoder.stdin.close()
        encoder.wait()
    done = [j for j in job_log if j["status"] == "done"]
    kinds = {}
    for c in contacts:
        key = c["kind"] + ("_moving" if c.get("truck_moving") else "")
        kinds[key] = kinds.get(key, 0) + 1
    state.update(
        frames=frames,
        sim_time_s=world.current_time - initial_time,
        jobs_done=len(done),
        jobs_per_hour=round(len(done) / max(now, 1e-9) * 3600, 1),
        plan_failed=sum(j["status"] == "plan_failed" for j in job_log),
        contacts=kinds,
        contact_events=contacts[:300],
        cycles=sum(e["kind"] == "cycle" for e in events),
        stalls=sum(e["kind"] == "stall" for e in events),
        preconditions=sum(e["kind"] == "precondition" for e in events),
        replans=sum(e["kind"] == "replan" for e in events),
        fills_held=len(dispatcher.held_back),
        resolution=resolution_counts(events),
        waits=wait_stats(_per_second(log["t_s"], log["blocked"], len(trucks))),
        trucks=[
            {"truck": t.name, "jobs_done": t.jobs_done, "failed": t.failed, "stage": t.stage,
             "waited_s": round(t.waited_s, 1), "person_stop_s": round(t.person_stop_s, 1)}
            for t in trucks
        ],
        jobs=job_log,
        plans=plans,
    )
    state["success"] = not contacts and not any(t.failed for t in trucks) and state["cycles"] == 0
    (args.output / "events.json").write_text(json.dumps(events, default=native, indent=1, ensure_ascii=False) + "\n")
    (args.output / "world.json").write_text(json.dumps(world_record(wm, geometry, extra_boxes), ensure_ascii=False) + "\n")
    routes = ctx["routes"]
    np.savez_compressed(
        args.output / "routes.npz",
        truck=np.array([r[0] for r in routes], dtype=int),
        rank=np.array([float(r[1]) for r in routes]),
        start=np.cumsum([0] + [len(r[3]) for r in routes])[:-1].astype(int),
        rects=np.concatenate([r[2] for r in routes]) if routes else np.zeros((0, 6), np.float32),
        g=np.concatenate([r[3] for r in routes]) if routes else np.zeros(0, np.float32),
    )
    people_log = log.pop("people")
    np.savez_compressed(
        args.output / "world_log.npz",
        people=np.asarray(people_log, dtype=float),
        **{k: np.asarray(v, dtype=float) for k, v in log.items()},
    )


def _per_second(times, blocked, trucks) -> np.ndarray:
    """The blocked log at its first sample of each whole second (wait_stats is 1 Hz)."""
    t = np.asarray(times, dtype=float)
    if not len(t):
        return np.zeros((0, trucks), dtype=int)
    first = np.nonzero(np.diff(np.floor(t), prepend=-1.0) > 0)[0]
    return np.asarray(blocked, dtype=int)[first]


def yaw_of(prim) -> float:
    w, x, y, z = prim.get_world_pose()[1]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main() -> None:
    args = arguments()
    settings = yaml.safe_load(args.settings.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    state = {
        "success": False,
        "phase": "startup",
        "structure": args.structure,
        "seed": args.seed,
        "arguments": {k: v for k, v in vars(args).items() if k != "kit_arguments"},
        "settings_synthetic": settings,
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "python": sys.version,
    }
    started = time.monotonic()
    app = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": True,
                "width": args.width,
                "height": args.height,
                "renderer": "RaytracedLighting",
                "multi_gpu": False,
            }
        )
        run(app, args, settings, state)
    except BaseException as exc:  # noqa: BLE001 - the record must be written
        state["success"] = False
        state["failure_reason"] = str(exc)
        (args.output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        state["wall_time_s"] = time.monotonic() - started
        (args.output / "result.json").write_text(
            json.dumps(state, default=native, indent=2, ensure_ascii=False) + "\n"
        )
        print(
            "WORLD_RESULT",
            json.dumps(
                {
                    k: state.get(k)
                    for k in ("success", "jobs_done", "contacts", "phase", "failure_reason", "sim_time_s")
                },
                default=native,
                ensure_ascii=False,
            ),
            flush=True,
        )
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
