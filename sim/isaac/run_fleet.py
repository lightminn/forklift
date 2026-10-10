"""Five forklifts at once on the whole NVIDIA warehouse, each on its own slam_toolbox.

    python sim/isaac/run_fleet.py --base-scene <measured base scene>/scene.usda \
        --seed 0 --output <dir> [--slam-sockets <dir>] [--slam-noise-seed N] [--video]

Plan: docs/plans/2026-10-08-fleet-five-forklifts.md. The layout and every
mission come from forklift_core.planning.fleet_layout / fleet_mission; the
trucks are kept apart by forklift_core.planning.fleet_traffic, which sees only
the poses the trucks believe.

Pose sources, per stage:
- travel (observe, aisle_exit, transport, return_home): with --slam-sockets the
  truck's own SLAM estimate (map frame = its known start, as in the one-truck
  closed loop); without it the simulator pose (stage F2 smoke).
- at the pallet (approach in the aisle, insert, lift, extract, lower,
  withdraw): the simulator pose -- it stands in for the pocket perception and
  docking a real truck would use for the relative pose (the user did not ask
  for perception; plan "위치·인식").

Safety is judged on the truth every 0.1 s: truck-truck footprint overlap,
truck-floor overlap and a truck over another truck's pallet stop the trucks
involved and count as failures. Writes result.json, fleet_log.npz,
traffic_events.json, camera.json and (--video) overview.mp4.
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
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
TRAVEL_STAGES = ("observe", "aisle_exit", "transport", "return_home")
LIFT_TIMEOUT_S = 30.0
# fleet_log.npz "status": what each truck's control branch did last (diagnosis).
STATUS = ("idle", "done", "failed", "slam_warmup", "tracking", "braking", "arrived_wait_stop", "lift", "lower")


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise RuntimeError(reason)


def native(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def yaw_and_tilt(q) -> tuple[float, float]:
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw, math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base-scene", required=True)
    parser.add_argument(
        "--forklift-urdf",
        type=Path,
        default=ROOT / "sim/models/dls08_measured/forklift.urdf",
    )
    parser.add_argument(
        "--pallet-urdf", type=Path, default=ROOT / "sim/models/epal6_pallet/pallet.urdf"
    )
    parser.add_argument(
        "--pallet-geometry",
        type=Path,
        default=ROOT / "config/pallet_geometry_epal6.yaml",
    )
    parser.add_argument(
        "--settings", type=Path, default=ROOT / "config/isaac_transport_measured.yaml"
    )
    parser.add_argument(
        "--layout", type=Path, default=ROOT / "config/fleet_full_warehouse.yaml"
    )
    parser.add_argument(
        "--lidar", type=Path, default=ROOT / "config/isaac_slam_lidar.yaml"
    )
    parser.add_argument(
        "--asset-root",
        default=(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
            "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
        ),
    )
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--trucks", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--insertion-reserve-m", type=float, default=0.016)
    parser.add_argument("--expansions", type=int, default=30000)
    parser.add_argument("--margin-m", type=float, default=0.3)
    parser.add_argument("--lookahead-m", type=float, default=3.0)
    parser.add_argument("--cycle-s", type=float, default=5.0)
    parser.add_argument("--stall-s", type=float, default=20.0)
    parser.add_argument(
        "--slam-sockets", type=Path, help="directory with truck_<k>.sock"
    )
    parser.add_argument("--slam-noise-seed", type=int)
    parser.add_argument("--slam-reply-timeout", type=float, default=90.0)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--max-sim-seconds", type=float, default=1500.0)
    args, unknown = parser.parse_known_args()
    if args.slam_noise_seed is not None and args.slam_sockets is None:
        parser.error("--slam-noise-seed requires --slam-sockets")
    if 120 % args.fps:
        parser.error("--fps must divide 120")
    args.kit_arguments = unknown
    return args


class Truck:
    """One truck's stage machine, control and (optional) SLAM state."""

    def __init__(self, k, robot, mission, plan, ctx):
        from forklift_core.planning.fleet_mission import STAGES

        self.k, self.name = k, f"truck_{k}"
        self.prim = f"/World/Fleet/Truck_{k}"
        self.robot, self.mission, self.plan, self.ctx = robot, mission, plan, ctx
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
        self.items = []
        for stage in STAGES:
            self.items.append(("path", stage))
            if stage == "insert":
                self.items.append(("lift", "lift"))
            if stage == "transport":
                self.items.append(("lower", "lower"))
        self.item = -1
        self.loaded = False
        self.lift_command = 0.0
        self.steering_command = np.zeros(2)
        self.tracker = None
        self.legs = []
        self.leg = 0
        self.paths = dict(plan.stages)
        self.offsets = {}
        self.route_end_m = 0.0
        self.stage_started = 0.0
        self.waited_s = 0.0
        self.done = False
        self.failed = None
        self.finished_s = None
        self.replans = []
        self.stage_log = []
        self.slam = None
        self.requested = 0.0
        self.status = "idle"
        self.last_command = None

    # -- stage machine ---------------------------------------------------
    @property
    def kind(self):
        return (
            self.items[self.item][0] if 0 <= self.item < len(self.items) else "parked"
        )

    @property
    def stage(self):
        return (
            self.items[self.item][1] if 0 <= self.item < len(self.items) else "parked"
        )

    def footprint(self):
        g = self.ctx["geometry"]
        return g.loaded_footprint if self.loaded else g.unloaded_footprint

    def uses_slam(self):
        return self.slam is not None and self.stage in TRAVEL_STAGES

    def start_next(self, t, pose):
        self.item += 1
        if self.item >= len(self.items):
            self.done, self.finished_s = True, t
            self.stage_log.append({"stage": "parked", "t_s": t})
            return
        self.stage_started = t
        self.stage_log.append({"stage": self.stage, "t_s": t})
        if self.kind == "path":
            self.loaded = self.stage in self.ctx["loaded_stages"]
            self.load_path(self.paths[self.stage], t)
        elif self.kind == "lift":
            self.loaded = True
        self.ctx["traffic"].set_footprint(self.name, self.footprint())

    def set_route(self, first_item):
        """Hand the traffic manager the route from this item's stage to the end."""
        g, loaded = self.ctx["geometry"], self.ctx["loaded_stages"]
        stages = [
            stage
            for kind, stage in self.items[max(first_item, 0) :]
            if kind == "path"
        ]
        offsets = self.ctx["traffic"].set_route(
            self.name,
            [
                (
                    np.asarray(self.paths[stage].poses),
                    g.loaded_footprint if stage in loaded else g.unloaded_footprint,
                )
                for stage in stages
            ],
        )
        self.offsets = dict(zip(stages, offsets, strict=True))
        last = np.asarray(self.paths[stages[-1]].poses)
        self.route_end_m = offsets[-1] + float(
            np.sum(np.hypot(*np.diff(last[:, :2], axis=0).T))
        )

    def route_progress(self) -> float:
        """Arc length along the route, as the traffic manager counts it."""
        if self.done:
            return self.route_end_m
        if self.kind != "path":  # lift after insert, lower after transport
            return self.offsets[self.items[self.item + 1][1]]
        along = 0.0 if self.last_command is None else self.last_command.progress_m
        return self.offsets[self.stage] + along

    def leg_end_m(self, command) -> float:
        """Route arc length where the current gear leg ends."""
        while (
            self.leg + 1 < len(self.legs)
            and command.segment_index >= self.legs[self.leg][1]
        ):
            self.leg += 1
        return self.offsets[self.stage] + self.legs[self.leg][2]

    def load_path(self, result, t):
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
        s = np.concatenate(
            ([0.0], np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T)))
        )
        # (first sample, last sample, arc length at the leg's end)
        self.legs = [
            (a, b, s[b]) for a, b in zip(cuts[:-1], cuts[1:], strict=True) if b > a
        ]
        self.leg = 0
        self.last_command = None


def run(app, args, settings, state) -> None:
    import factory_assets
    import omni.usd
    import planar_lidar
    from chassis_contract import (
        chassis_record,
        require_scene_matches_model,
        stage_chassis,
    )
    from insertion_geometry import (
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
    from scene import (
        add_factory_items,
        configure_drives,
        create_pallet,
        hide_overhead,
        read_catalogue,
    )

    from forklift_core.control import TrackerConfig, ackermann_command
    from forklift_core.perception.pallet_geometry import load_pallet_geometry
    from forklift_core.planning import Footprint, Pose2D
    from forklift_core.planning.fleet_layout import (
        load_fleet_layout,
        make_fleet_scenario,
    )
    from forklift_core.planning.fleet_mission import (
        LOADED_STAGES,
        REPLANNABLE,
        plan_fleet_mission,
        replan_stage,
    )
    from forklift_core.planning.fleet_traffic import (
        TrafficManager,
        footprint_rectangle,
        rectangles_overlap,
    )
    from forklift_core.planning.geometry import FootprintCollisionChecker
    from forklift_core.planning.pallet_mission import (
        SyntheticMissionGeometry,
        make_transport_planner_config,
    )
    from forklift_core.sensors.lidar import PlanarScanPattern

    drive_geometry = read_drive_geometry_m(
        args.forklift_urdf, settings["max_wheel_rate_rad_s"]
    )
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
    state["drive_geometry"] = asdict(drive_geometry)
    config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=args.expansions,
    )
    travel = replace(config, obstacle_heuristic_resolution_m=0.25)

    state["phase"] = "scene"
    enable_extension("isaacsim.asset.importer.urdf")
    require(
        omni.usd.get_context().open_stage(args.base_scene), "Cannot open base scene"
    )
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    template = stage.GetPrimAtPath("/World/Forklift")
    require(template.IsValid(), "base scene has no /World/Forklift")
    refs = template.GetMetadata("references")
    truck_usd = list(refs.GetAddedOrExplicitItems())[0].assetPath
    stage.RemovePrim("/World/Forklift")
    for k in range(args.trucks):
        stage.DefinePrim(
            f"/World/Fleet/Truck_{k}", "Xform"
        ).GetReferences().AddReference(truck_usd)
    for _ in range(10):
        app.update()
    for k in range(args.trucks):
        chassis = stage_chassis(stage, f"/World/Fleet/Truck_{k}")
        require_scene_matches_model(chassis, args.forklift_urdf)
    state["scene_chassis"] = chassis_record(chassis)
    state["truck_usd"] = truck_usd

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    catalogue, offsets = read_catalogue(stage, app, args.asset_root)
    factory_catalogue, factory_offsets = read_catalogue(
        stage,
        app,
        args.asset_root,
        filenames=factory_assets.ALL,
        prim_prefix="/World/FactoryCatalogue_",
    )
    offsets.update(factory_offsets)
    layout = load_fleet_layout(args.layout)
    fleet = make_fleet_scenario(
        args.seed,
        layout,
        catalogue,
        factory_assets.factory_assets(
            dict(zip(factory_assets.ALL, factory_catalogue, strict=True))
        ),
        geometry=geometry,
        trucks=args.trucks,
        parked_gap_m=2 * args.margin_m,
    )
    state["layout"] = {
        "file": str(args.layout),
        "sha256": hashlib.sha256(args.layout.read_bytes()).hexdigest(),
        "version": fleet.layout_version,
        "work_items": len(fleet.work_items),
        "loads": len(fleet.loads),
    }
    state["missions"] = [
        {
            "truck": m.truck,
            "aisle": m.aisle,
            "parking": asdict(m.parking),
            "pickup": asdict(m.pickup),
            "destination": asdict(m.destination),
        }
        for m in fleet.missions
    ]
    state["factory_items"] = add_factory_items(
        stage, app, fleet.work_items, fleet.loads, offsets
    )
    removed = []
    for name in layout.removed_prims:
        prim = stage.GetPrimAtPath(f"/World/Environment/{name}")
        require(prim.IsValid(), f"warehouse has no {name} to remove")
        prim.SetActive(False)
        removed.append(str(prim.GetPath()))
    state["removed_warehouse_prims"] = removed
    state["hidden_overhead_prims"] = len(hide_overhead(stage))
    pallets = [
        create_pallet(
            world,
            stage,
            app,
            args.pallet_urdf,
            args.output,
            m.pickup,
            settings["pallet_mass_kg"],
            prim_path=f"/World/Fleet/Pallet_{m.truck}",
            name=f"pallet_{m.truck}",
        )
        for m in fleet.missions
    ]
    robots = []
    for m in fleet.missions:
        yaw = m.parking.yaw_rad
        robots.append(
            world.scene.add(
                Robot(
                    prim_path=f"/World/Fleet/Truck_{m.truck}",
                    name=f"truck_{m.truck}",
                    position=np.array(
                        [
                            m.parking.x_m + rear_offset_m * math.cos(yaw),
                            m.parking.y_m + rear_offset_m * math.sin(yaw),
                            0.015,
                        ]
                    ),
                    orientation=np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]),
                )
            )
        )
    from insertion_geometry import InsertionGeometry

    boxes = len(
        InsertionGeometry.from_urdfs(args.forklift_urdf, args.pallet_urdf).pallet_boxes
    )
    state["collision_counts"] = configure_drives(
        stage,
        settings,
        expected_pallet_box_count=boxes,
        forklift_roots=tuple(f"/World/Fleet/Truck_{k}" for k in range(args.trucks)),
        pallet_roots=tuple(f"/World/Fleet/Pallet_{k}" for k in range(args.trucks)),
    )
    camera = None
    if args.video:
        camera = Camera(
            prim_path="/World/FleetCamera", frequency=-1, resolution=(1280, 720)
        )
        b = fleet.bounds
        centre = np.array(
            [(b.x_min_m + b.x_max_m) / 2, (b.y_min_m + b.y_max_m) / 2, 0.0]
        )
        eye = centre + np.array([0.0, 0.0, 27.0])
        # North to the right: the warehouse's long axis runs across the frame.
        look = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye), Gf.Vec3d(*centre), Gf.Vec3d(-1, 0, 0)
        )
        q = look.GetInverse().ExtractRotationQuat()
        camera.set_world_pose(
            position=eye,
            orientation=np.array([q.GetReal(), *q.GetImaginary()]),
            camera_axes="usd",
        )
        camera.set_focal_length(0.85)
        state["camera"] = {
            "eye_m": eye.tolist(),
            "target_m": centre.tolist(),
            "up": [-1, 0, 0],
            "focal_length": 0.85,
            "resolution": [1280, 720],
            "fps": args.fps,
        }
    world.reset()
    if camera is not None:
        camera.initialize()
    for _ in range(120):
        world.step(render=args.video)
    for m, p in zip(fleet.missions, pallets, strict=True):
        pos, _ = p.get_world_pose()
        require(
            np.linalg.norm(pos[:2] - [m.pickup.x_m, m.pickup.y_m]) < 0.005,
            f"pallet {m.truck} displaced",
        )
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

    state["phase"] = "planning"
    plans, planning = [], []
    for m in fleet.missions:
        start = time.monotonic()
        plan = plan_fleet_mission(m, config, travel, geometry)
        planning.append(
            {
                "truck": m.truck,
                "status": plan.status,
                "wall_s": time.monotonic() - start,
                "lengths_m": {k: float(v.length_m) for k, v in plan.stages.items()},
            }
        )
        plans.append(plan)
    state["planning"] = planning
    require(
        all(p.success for p in plans),
        "planning_failed: " + ", ".join(p.status for p in plans),
    )
    (args.output / "plans.json").write_text(
        json.dumps(
            {
                f"truck_{m.truck}": {
                    k: np.asarray(v.poses).round(4).tolist()
                    for k, v in p.stages.items()
                }
                for m, p in zip(fleet.missions, plans, strict=True)
            }
        )
        + "\n"
    )

    def tracker_config(stage_name):
        speeds = {
            "observe": "approach_speed_mps",
            "approach": "approach_speed_mps",
            "insert": "insert_speed_mps",
            "extract": "extract_speed_mps",
            "aisle_exit": "transport_speed_mps",
            "transport": "transport_speed_mps",
            "withdraw": "withdraw_speed_mps",
            "return_home": "approach_speed_mps",
        }
        travel_stage = stage_name in ("observe", "return_home")
        return TrackerConfig(
            cruise_speed_mps=settings[speeds[stage_name]],
            max_curvature_inv_m=settings["tracker_curvature_inv_m"],
            max_acceleration_mps2=settings["drive_acceleration_mps2"],
            lookahead_m=0.28,
            position_tolerance_m=0.03 if travel_stage else 0.008,
            yaw_tolerance_rad=0.05
            if stage_name == "observe"
            else 0.03
            if stage_name == "return_home"
            else 0.02,
            # A gear change on a travel leg is not a docking point; under SLAM
            # the truck stops 4-5 cm off it sideways, and with the 3 cm cusp
            # tolerance it then waits there for good (F3 seed 5, 2026-10-09).
            cusp_position_tolerance_m=0.10 if stage_name in TRAVEL_STAGES else 0.03,
            cusp_yaw_tolerance_rad=0.08 if stage_name in TRAVEL_STAGES else 0.05,
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
        cycle_s=args.cycle_s,
        stall_s=args.stall_s,
    )
    ctx = {
        "geometry": geometry,
        "settings": settings,
        "traffic": traffic,
        "tracker_config": tracker_config,
        "loaded_stages": LOADED_STAGES,
    }
    trucks = [
        Truck(m.truck, r, m, p, ctx)
        for m, r, p in zip(fleet.missions, robots, plans, strict=True)
    ]
    static = [p.rectangle for p in fleet.fixed] + [
        p.rectangle for p in fleet.work_items
    ]

    lidar = yaml.safe_load(args.lidar.read_text())
    pattern = PlanarScanPattern(
        lidar["beam_count"], float(lidar["range_min_m"]), float(lidar["range_max_m"])
    )
    mount = planar_lidar.LaserMount(
        tuple(map(float, lidar["mount_xyz_m"])), float(lidar["mount_yaw_rad"])
    )
    beam_angles = pattern.beam_angles_rad()
    scan_every = 120 // int(lidar["rate_hz"])
    if args.slam_sockets is not None:
        from forklift_core.localization import slam_link
        from forklift_core.localization.slam_pose import (
            IncrementalWheelOdometry,
            LocalizationStale,
            OdometryNoise,
            SlamPoseTracker,
            StopDetector,
        )
        from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry

        for t in trucks:
            start = (
                t.mission.parking.x_m,
                t.mission.parking.y_m,
                t.mission.parking.yaw_rad,
            )
            t.slam = {
                "link": slam_link.SlamLinkClient(
                    str(args.slam_sockets / f"truck_{t.k}.sock"),
                    timeout_s=args.slam_reply_timeout,
                ),
                "noise": OdometryNoise(
                    seed=(args.slam_noise_seed or 0) * 100 + t.k,
                    enabled=args.slam_noise_seed is not None,
                ),
                "odometry": IncrementalWheelOdometry(
                    AckermannOdometryGeometry(
                        drive_geometry.wheelbase_m,
                        drive_geometry.track_m,
                        drive_geometry.wheel_radius_m,
                    ),
                    initial_pose=start,
                ),
                "tracker": SlamPoseTracker(max_age_s=0.25, hold_limit_m=15.0),
                "stop": StopDetector(tick_s=1 / 120),
                "odom_rear": start,
                "tracker_speed": 0.0,
                "scan_id": 0,
                "holds": 0,
                "jumps": [],
                "start": start,
            }
        pool = ThreadPoolExecutor(max_workers=len(trucks))
    state["pose_source"] = (
        "slam on travel stages" if args.slam_sockets else "simulator (F2 smoke)"
    )

    def odom_base(t):
        x, y, yaw = t.slam["odom_rear"]
        return (
            x + rear_offset_m * math.cos(yaw),
            y + rear_offset_m * math.sin(yaw),
            yaw,
        )

    def truth(t):
        base, q = t.robot.get_world_pose()
        yaw, tilt = yaw_and_tilt(q)
        rear = np.array(
            [
                base[0] - rear_offset_m * math.cos(yaw),
                base[1] - rear_offset_m * math.sin(yaw),
                yaw,
            ]
        )
        speed = float(
            np.dot(t.robot.get_linear_velocity()[:2], [math.cos(yaw), math.sin(yaw)])
        )
        return rear, speed, tilt

    def slam_rear(t, now):
        if t.slam["tracker"].applied is None:
            return np.array(t.slam["odom_rear"])
        try:
            est = t.slam["tracker"].map_from_base(now, odom_base(t))
        except LocalizationStale as exc:
            raise RuntimeError(f"{t.name} localization_stale: {exc}") from exc
        return np.array(
            [
                est[0] - rear_offset_m * math.cos(est[2]),
                est[1] - rear_offset_m * math.sin(est[2]),
                est[2],
            ]
        )

    encoder = None
    if args.video:
        encoder = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                "1280x720",
                "-r",
                str(args.fps),
                "-i",
                "-",
                "-c:v",
                "libx264",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                str(args.output / "overview.mp4"),
            ],
            stdin=subprocess.PIPE,
        )
    frames = 0
    log = {
        k: []
        for k in (
            "t_s",
            "truth",
            "control",
            "stage",
            "loaded",
            "speed",
            "allowed",
            "blocked",
            "status",
            "tracker_speed",
        )
    }
    events, violations = [], []
    scans_sent = 0
    tick = 0
    initial_time = world.current_time
    frame_every = 120 // args.fps
    dt = 1 / 120
    permits = {}
    last_seen, last_motion_s = {}, 0.0
    state["phase"] = "running"

    def scan_and_exchange(t, stamp):
        base, q = t.robot.get_world_pose()
        origin, directions = planar_lidar.laser_rays_world(base, q, mount, beam_angles)
        distances, hits, _ = planar_lidar.cast_scan(
            origin, directions, pattern.range_max_m, self_prefix=t.prim + "/"
        )
        ranges = pattern.ranges_from_hits(distances, hits)
        sent = t.slam["noise"].ranges(
            ranges, range_min_m=pattern.range_min_m, range_max_m=pattern.range_max_m
        )
        return t, slam_link.Scan(
            t.slam["scan_id"],
            float(stamp),
            odom_base(t),
            sent.astype(np.float32),
            pattern.angle_min_rad,
            pattern.angle_increment_rad,
            pattern.range_min_m,
            pattern.range_max_m,
        )

    for t in trucks:
        traffic.add(t.name, t.k, truth(t)[0], t.footprint())
    for t in trucks:
        t.set_route(0)
        t.start_next(0.0, truth(t)[0])
    traffic.update(
        0.0, {t.name: truth(t)[0] for t in trucks}, {t.name: 0.0 for t in trucks}
    )
    state["route_conflicts"] = traffic.route_conflicts()
    reported = set()

    while world.current_time - initial_time < args.max_sim_seconds:
        now = world.current_time - initial_time
        poses, speeds, truths = {}, {}, {}
        for t in trucks:
            rear, speed, tilt = truth(t)
            truths[t.name] = (rear, speed, tilt)
            if t.uses_slam():
                poses[t.name], speeds[t.name] = (
                    slam_rear(t, now),
                    t.slam["tracker_speed"],
                )
            else:
                poses[t.name], speeds[t.name] = rear, speed
        if tick % scan_every == 0:
            progress = {t.name: t.route_progress() for t in trucks}
            permits = traffic.update(now, poses, progress)
            # Recorded, not resolved: under the rank and safe-stop rules a
            # lock means a broken precondition or a bug.
            for name, blocker in traffic.deadlocks(now):
                episode = (name, blocker, traffic.trucks[name].waiting_since_s)
                if episode not in reported:
                    reported.add(episode)
                    events.append(
                        {
                            "t_s": now,
                            "truck": name,
                            "blocker": blocker,
                            "stage": next(x for x in trucks if x.name == name).stage,
                            "kind": "cycle"
                            if traffic.trucks[blocker].waiting_on == name
                            else "stall",
                        }
                    )
            # Safety on the truth: trucks, floor, other trucks' pallets.
            bodies = {
                t.name: footprint_rectangle(truths[t.name][0], t.footprint())
                for t in trucks
            }
            pallet_rects = {}
            for t, p in zip(trucks, pallets, strict=True):
                pos, pq = p.get_world_pose()
                pallet_rects[t.name] = footprint_rectangle(
                    (pos[0], pos[1], yaw_and_tilt(pq)[0]),
                    Footprint(
                        geometry.pallet_depth_m / 2,
                        geometry.pallet_depth_m / 2,
                        geometry.pallet_width_m / 2,
                    ),
                )
            for i, a in enumerate(trucks):
                if a.failed:
                    continue
                checker = FootprintCollisionChecker(static, a.footprint(), fleet.bounds)
                if not checker.free(tuple(truths[a.name][0])):
                    a.failed = f"floor_contact in {a.stage}"
                    violations.append(
                        {"t_s": now, "truck": a.name, "kind": "floor", "stage": a.stage}
                    )
                for b in trucks[i + 1 :]:
                    if rectangles_overlap(bodies[a.name], bodies[b.name]):
                        for x in (a, b):
                            x.failed = (
                                x.failed
                                or f"truck_contact with {(b if x is a else a).name}"
                            )
                        violations.append(
                            {
                                "t_s": now,
                                "truck": a.name,
                                "other": b.name,
                                "kind": "truck",
                            }
                        )
                for other, rect in pallet_rects.items():
                    if other != a.name and rectangles_overlap(bodies[a.name], rect):
                        a.failed = a.failed or f"pallet_contact {other}"
                        violations.append(
                            {
                                "t_s": now,
                                "truck": a.name,
                                "pallet": other,
                                "kind": "pallet",
                            }
                        )
            log["t_s"].append(now)
            log["truth"].append([truths[t.name][0] for t in trucks])
            log["control"].append([poses[t.name] for t in trucks])
            log["stage"].append([t.item for t in trucks])
            log["loaded"].append([t.loaded for t in trucks])
            log["speed"].append([t.requested for t in trucks])
            log["allowed"].append([permits[t.name].allowed_m for t in trucks])
            log["blocked"].append(
                [
                    -1
                    if permits[t.name].blocked_by is None
                    else int(permits[t.name].blocked_by[6:])
                    for t in trucks
                ]
            )
            log["status"].append([STATUS.index(t.status) for t in trucks])
            log["tracker_speed"].append([speeds[t.name] for t in trucks])

        for t in trucks:
            rear, speed = poses[t.name], speeds[t.name]
            requested, curvature = 0.0, 0.0
            if t.done or t.failed:
                t.status = "failed" if t.failed else "done"
            elif t.slam is not None and not t.slam["tracker"].may_drive():
                t.status = "slam_warmup"
            elif t.kind == "path":
                command = t.tracker.update(rear, speed, dt)
                t.last_command = command
                t.status = (
                    "arrived_wait_stop" if command.status == "arrived" else command.status
                )
                if command.status == "failed":
                    if (
                        t.stage in REPLANNABLE
                        and len(t.replans) < 6
                        and abs(speed) < 0.02
                    ):
                        result = replan_stage(
                            t.mission,
                            t.stage,
                            Pose2D(*map(float, rear)),
                            [],
                            config,
                            travel,
                            geometry,
                        )
                        events.append(
                            {
                                "t_s": now,
                                "truck": t.name,
                                "stage": t.stage,
                                "tracking_failure": command.failure,
                                "replan": result.status,
                            }
                        )
                        if result.success:
                            t.replans.append(
                                {
                                    "t_s": now,
                                    "stage": t.stage,
                                    "reason": command.failure,
                                }
                            )
                            t.paths[t.stage] = result
                            t.load_path(result, now)
                            t.set_route(t.item)
                        else:
                            t.failed = f"tracking {command.failure} in {t.stage}, replan {result.status}"
                    elif t.stage not in REPLANNABLE:
                        t.failed = f"tracking {command.failure} in {t.stage}"
                elif command.status == "arrived" and (
                    t.slam is None or t.slam["stop"].stopped or not t.uses_slam()
                ):
                    if t.slam is not None and t.slam["tracker"].mode == "holding":
                        jump = t.slam["tracker"].release(odom_from_base=odom_base(t))
                        t.slam["jumps"].append(
                            {"t_s": now, "stage": t.stage, "m": jump[0], "rad": jump[1]}
                        )
                    t.start_next(now, rear)
                else:
                    requested, curvature = command.speed_mps, command.curvature_inv_m
                    permit = permits.get(t.name)
                    # Cap only where the permit, not the leg's own end, binds:
                    # the tracker brakes to its goal (8 mm) by itself.
                    leg_end = t.leg_end_m(command)
                    if permit is not None and permit.granted_m < leg_end - 1e-6:
                        cap = math.sqrt(
                            2
                            * settings["drive_acceleration_mps2"]
                            * max(permit.allowed_m - 0.02, 0.0)
                        )
                        if abs(requested) > cap:
                            requested = math.copysign(cap, requested)
                            t.waited_s += dt if cap < 0.02 else 0.0
                    if (
                        t.uses_slam()
                        and t.slam["tracker"].mode == "tracking"
                        and t.tracker.remaining_to_goal_m()
                        <= 0.5 + speed**2 / (2 * settings["drive_acceleration_mps2"])
                    ):
                        t.slam["tracker"].hold(odom_from_base=odom_base(t))
                        t.slam["holds"] += 1
                if (
                    now - t.stage_started
                    > 3 * max(t.tracker.nominal_duration_s(), 10.0) + t.waited_s + 60
                ):
                    t.failed = t.failed or f"timeout in {t.stage}"
            elif t.kind == "lift":
                t.status = "lift"
                t.lift_command = min(
                    settings["lift_target_m"],
                    t.lift_command + settings["lift_rate_mps"] * dt,
                )
                pos, _ = pallets[t.k].get_world_pose()
                if t.lift_command >= settings["lift_target_m"] and pos[2] > 0.06:
                    t.start_next(now, rear)
                elif now - t.stage_started > LIFT_TIMEOUT_S:
                    t.failed = "lift_timeout"
            elif t.kind == "lower":
                t.status = "lower"
                t.lift_command = max(
                    0.0, t.lift_command - settings["lift_rate_mps"] * dt
                )
                if (
                    t.lift_command <= 0.0
                    and now - t.stage_started
                    > settings["lift_target_m"] / settings["lift_rate_mps"] + 1.0
                ):
                    t.loaded = False
                    t.start_next(now, rear)
            t.requested = requested
            drive = ackermann_command(requested, curvature, drive_geometry)
            target = np.asarray(drive.steering_rad)
            actual = t.robot.get_joint_positions()[t.steers]
            if float(np.max(np.abs(target - actual))) > 0.05:
                drive = ackermann_command(requested * 0.25, curvature, drive_geometry)
            rate = settings["steering_command_rate_rad_s"] * dt
            t.steering_command += np.clip(target - t.steering_command, -rate, rate)
            t.robot.apply_action(
                ArticulationAction(
                    joint_velocities=np.asarray(drive.wheel_rates_rad_s),
                    joint_indices=t.wheels,
                )
            )
            t.robot.apply_action(
                ArticulationAction(
                    joint_positions=t.steering_command, joint_indices=t.steers
                )
            )
            t.robot.apply_action(
                ArticulationAction(
                    joint_positions=np.array([t.lift_command]),
                    joint_indices=t.lift_index,
                )
            )
            if t.slam is not None:
                t.slam["last_command"] = requested

        frame_due = args.video and tick % frame_every == 0
        world.step(render=bool(frame_due))
        tick += 1
        stamp = world.current_time - initial_time
        if args.slam_sockets is not None:
            for t in trucks:
                rates_true = t.robot.get_joint_velocities()[t.wheels]
                steer_true = t.robot.get_joint_positions()[t.steers]
                rates = t.slam["noise"].wheel_rates(rates_true[2:4])
                angles = t.slam["noise"].steering(steer_true)
                yaw_before = t.slam["odom_rear"][2]
                t.slam["odom_rear"] = t.slam["odometry"].update(stamp, rates, angles)
                speed = float(np.mean(rates)) * drive_geometry.wheel_radius_m
                t.slam["stop"].update(
                    commanded_speed=t.slam.get("last_command", 0.0),
                    speed=speed,
                    yaw_rate=(t.slam["odom_rear"][2] - yaw_before) * 120.0,
                )
                t.slam["tracker_speed"] = t.slam["stop"].tracker_speed(0.012)
            if tick % scan_every == 0:
                prepared = [scan_and_exchange(t, stamp) for t in trucks]
                replies = list(
                    pool.map(
                        lambda item: (item[0], item[0].slam["link"].exchange(item[1])),
                        prepared,
                    )
                )
                for t, reply in replies:
                    t.slam["tracker"].receive(
                        reply.scan_id, reply.stamp_s, reply.status, reply.map_from_odom
                    )
                    t.slam["scan_id"] += 1
                scans_sent += len(trucks)
        if frame_due:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.size:
                encoder.stdin.write(
                    np.ascontiguousarray(rgba[:, :, :3]).astype(np.uint8).tobytes()
                )
                frames += 1
        if tick % 120 == 0:
            moved = [
                float(
                    np.hypot(
                        *(
                            truths[t.name][0][:2]
                            - last_seen.get(t.name, truths[t.name][0])[:2]
                        )
                    )
                )
                for t in trucks
                if not (t.done or t.failed)
            ]
            if any(m > 0.01 for m in moved) or not moved:
                last_motion_s = now
            last_seen = {t.name: truths[t.name][0].copy() for t in trucks}
            if now - last_motion_s > 120.0:
                state["stopped"] = f"gridlock: no truck moved for 120 s at {now:.1f} s"
                break
        if all(t.done or t.failed for t in trucks):
            break

    if encoder is not None:
        encoder.stdin.close()
        encoder.wait()
    state["frames"] = frames
    state["sim_time_s"] = world.current_time - initial_time
    state["scans_exchanged"] = scans_sent
    state["violations"] = violations[:200]
    state["violation_count"] = len(violations)
    state["traffic_events"] = len(events)
    state["trucks"] = [
        {
            "truck": t.name,
            "aisle": t.mission.aisle,
            "finished_s": t.finished_s,
            "failed": t.failed,
            "stage": t.stage,
            "waited_s": t.waited_s,
            "replans": t.replans,
            "stages": t.stage_log,
            "slam": None
            if t.slam is None
            else {
                "holds": t.slam["holds"],
                "jumps": t.slam["jumps"],
                "scans": t.slam["scan_id"],
            },
        }
        for t in trucks
    ]
    state["completed"] = sum(1 for t in trucks if t.done and not t.failed)
    state["success"] = state["completed"] == len(trucks) and not violations
    (args.output / "traffic_events.json").write_text(
        json.dumps(events, default=native, indent=1) + "\n"
    )
    np.savez_compressed(
        args.output / "fleet_log.npz",
        **{k: np.asarray(v, dtype=float) for k, v in log.items()},
    )


def main() -> None:
    args = arguments()
    settings = yaml.safe_load(args.settings.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    state = {
        "success": False,
        "phase": "startup",
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
                "width": 1280,
                "height": 720,
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
            json.dumps(state, default=native, indent=2) + "\n"
        )
        print(
            "FLEET_RESULT",
            json.dumps(
                {
                    k: state.get(k)
                    for k in (
                        "success",
                        "completed",
                        "phase",
                        "failure_reason",
                        "sim_time_s",
                    )
                }
            ),
            flush=True,
        )
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
