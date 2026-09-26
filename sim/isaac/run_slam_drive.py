"""Drive the factory survey loop in Isaac and record 2D LiDAR SLAM input.

Records, from one seeded factory hall: planar LiDAR scans cast by PhysX ray
queries, wheel joint rates and steering angles at the physics rate, and the
ground-truth base_link pose. Nothing here runs SLAM; ros2 forklift_ros
slam_replay turns the record into a rosbag for slam_toolbox.

The survey controller itself is driven by the simulator's ground-truth pose,
exactly like run_transport.py. The recorded wheel and scan data are what a
mapper may use; the ground truth is kept for evaluation only.
"""

import argparse
import hashlib
import importlib.util
import json
import math
import subprocess
import sys
import time
import traceback
from dataclasses import asdict
from pathlib import Path

import numpy as np

LOG_FORMAT = "forklift_slam_log_v1"
# Colour range of the robot-camera depth video; display only, not a sensor limit.
DEPTH_VIDEO_RANGE_M = (0.3, 10.0)


def record_json(value: object, indent: int | None = None) -> str:
    def native(item: object) -> object:
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(f"Unsupported record type: {type(item).__name__}")

    return json.dumps(value, default=native, indent=indent, allow_nan=False)


def require(condition: bool, reason: str) -> None:
    """A failed experimental invariant always aborts, even under Python -O."""
    if not condition:
        raise RuntimeError(reason)


def yaw_and_tilt(quaternion: np.ndarray) -> tuple[float, float]:
    w, x, y, z = quaternion
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    tilt = math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    return yaw, tilt


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def open_encoder(path: Path, width: int, height: int, fps: int) -> subprocess.Popen:
    """H.264 encoder fed raw RGB24 frames on stdin, timed by simulation frames."""
    return subprocess.Popen(
        [
            "ffmpeg",
            "-nostdin",
            "-n",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-threads",
            "2",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(path),
        ],
        stdin=subprocess.PIPE,
    )


def video_hz(value: str) -> int:
    rate = int(value)
    if rate <= 0 or 120 % rate:
        raise argparse.ArgumentTypeError("video rate must divide the 120 Hz physics")
    return rate


def arguments() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument(
        "--settings", type=Path, default=root / "config/isaac_transport.yaml"
    )
    parser.add_argument(
        "--layout", type=Path, default=root / "config/factory_south_hall.yaml"
    )
    parser.add_argument(
        "--lidar", type=Path, default=root / "config/isaac_slam_lidar.yaml"
    )
    parser.add_argument(
        "--pallet-geometry",
        type=Path,
        default=root / "config/pallet_geometry_epal6.yaml",
    )
    parser.add_argument(
        "--forklift-urdf",
        type=Path,
        default=root / "sim/models/dls08_provisional/forklift.urdf",
    )
    parser.add_argument(
        "--asset-root",
        default=(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
            "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
        ),
    )
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--fps", type=video_hz, default=30)
    parser.add_argument(
        "--robot-camera",
        action="store_true",
        help="Also record the truck's forward RGB-D camera (the synthetic "
        "perception mount) as camera_rgb.mp4 and camera_depth.mp4, frame for "
        "frame with the overview video. Requires --video.",
    )
    parser.add_argument("--max-sim-seconds", type=float, default=900)
    args, unknown = parser.parse_known_args()
    if args.max_sim_seconds <= 0:
        parser.error("--max-sim-seconds must be positive")
    if args.robot_camera and not args.video:
        parser.error("--robot-camera requires --video")
    from insertion_geometry import read_chassis_reference_m

    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    try:
        args.pallet_geometry_loaded = load_pallet_geometry(args.pallet_geometry)
        args.axle_to_fork_tip_m, args.rear_axle_offset_m = read_chassis_reference_m(
            args.forklift_urdf
        )
    except ValueError as exc:
        parser.error(str(exc))
    args.kit_arguments = unknown
    return args


def source_sha256(repo_root: Path, core_root: Path) -> dict[str, str]:
    files = sorted(core_root.rglob("*.py")) + sorted(
        (repo_root / "sim/isaac").rglob("*.py")
    )
    return {
        (
            (Path("forklift_core") / path.relative_to(core_root)).as_posix()
            if path.is_relative_to(core_root)
            else path.relative_to(repo_root).as_posix()
        ): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files
    }


def run(app, args: argparse.Namespace, state: dict) -> None:
    import factory_assets
    import omni.usd
    import planar_lidar
    import video_frames as video_frames_module
    import yaml
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.sensors.camera import Camera
    from PIL import Image
    from pxr import Gf
    from scene import (
        add_factory_items,
        add_path_display,
        add_props,
        configure_drives,
        hide_overhead,
        read_catalogue,
    )

    import forklift_core
    from forklift_core.control import (
        AckermannGeometry,
        RearAxlePathTracker,
        TrackerConfig,
        ackermann_command,
    )
    from forklift_core.planning import Footprint, Rectangle, collision_free_pose
    from forklift_core.planning.factory_layout import (
        load_factory_layout,
        make_factory_scenario,
        plan_survey_route,
    )
    from forklift_core.planning.pallet_mission import (
        SyntheticMissionGeometry,
        make_transport_planner_config,
    )
    from forklift_core.sensors.lidar import PlanarScanPattern

    root = Path(__file__).resolve().parents[2]
    state["source_sha256"] = source_sha256(root, Path(forklift_core.__file__).parent)
    settings = yaml.safe_load(args.settings.read_text())
    lidar_config = yaml.safe_load(args.lidar.read_text())
    require(settings["physics_hz"] == 120, "This adapter requires 120Hz physics")
    require(120 % lidar_config["rate_hz"] == 0, "LiDAR rate must divide 120 Hz")
    pattern = PlanarScanPattern(
        lidar_config["beam_count"],
        float(lidar_config["range_min_m"]),
        float(lidar_config["range_max_m"]),
    )
    mount = planar_lidar.LaserMount(
        tuple(map(float, lidar_config["mount_xyz_m"])),
        float(lidar_config["mount_yaw_rad"]),
    )
    layout = load_factory_layout(args.layout)
    state.update(
        {
            "settings_synthetic": settings,
            "lidar_synthetic": lidar_config,
            "layout_version": layout.layout_version,
            "layout_sha256": hashlib.sha256(args.layout.read_bytes()).hexdigest(),
            "lidar_sha256": hashlib.sha256(args.lidar.read_bytes()).hexdigest(),
        }
    )

    state["phase"] = "scene"
    require(
        omni.usd.get_context().open_stage(args.base_scene), "Cannot open base scene"
    )
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    bay_catalogue, offsets = read_catalogue(stage, app, args.asset_root)
    factory_catalogue, factory_offsets = read_catalogue(
        stage,
        app,
        args.asset_root,
        filenames=factory_assets.ALL,
        prim_prefix="/World/FactoryCatalogue_",
    )
    offsets.update(factory_offsets)
    specs = {
        name: spec
        for name, spec in zip(factory_assets.ALL, factory_catalogue, strict=True)
    }
    state["factory_asset_dimensions_m"] = {
        name: [spec.length_m, spec.width_m, spec.height_m]
        for name, spec in specs.items()
    }
    state["factory_asset_offline_difference_m"] = {
        name: float(
            np.max(
                np.abs(
                    np.subtract(
                        state["factory_asset_dimensions_m"][name],
                        factory_assets.MEASURED_DIMENSIONS_M[name],
                    )
                )
            )
        )
        for name in specs
    }
    pallet_geometry = args.pallet_geometry_loaded
    geometry = SyntheticMissionGeometry(
        unloaded_footprint=Footprint(args.axle_to_fork_tip_m, 0.17, 0.36),
        pallet_depth_m=pallet_geometry.overall_depth_m,
        pallet_width_m=pallet_geometry.overall_width_m,
        axle_to_fork_tip_m=args.axle_to_fork_tip_m,
    )
    factory = make_factory_scenario(
        args.seed,
        layout,
        bay_catalogue,
        factory_assets.factory_assets(specs),
        geometry=geometry,
    )
    transport = factory.transport
    bay_props = transport.props[: len(transport.props) - len(factory.work_items)]
    state["factory"] = {
        "work_items": len(factory.work_items),
        "loads": len(factory.loads),
        "bay_props": len(bay_props),
        "pickup_pallet_spawned": False,
    }
    state["props"] = add_props(stage, app, bay_props, offsets)
    state["factory_items"] = add_factory_items(
        stage, app, factory.work_items, factory.loads, offsets
    )
    if args.video:
        state["hidden_overhead_prims"] = len(hide_overhead(stage))

    start = layout.survey_route[0]
    rear_offset = abs(args.rear_axle_offset_m)
    base_start = np.array(
        [
            start.x_m + rear_offset * math.cos(start.yaw_rad),
            start.y_m + rear_offset * math.sin(start.yaw_rad),
            0.015,
        ]
    )
    robot = world.scene.add(
        Robot(
            prim_path="/World/Forklift",
            name="forklift",
            position=base_start,
            orientation=np.array(
                [math.cos(start.yaw_rad / 2), 0, 0, math.sin(start.yaw_rad / 2)]
            ),
        )
    )
    state["collision_counts"] = configure_drives(
        stage, settings, expected_pallet_box_count=None
    )
    hall = transport.bounds
    camera = None
    if args.video:
        camera = Camera(
            prim_path="/World/SurveyCamera", frequency=-1, resolution=(1280, 720)
        )
        centre = np.array(
            [(hall.x_min_m + hall.x_max_m) / 2, (hall.y_min_m + hall.y_max_m) / 2, 0.0]
        )
        eye = centre + np.array([0.0, 0.0, 26.0])
        look = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye), Gf.Vec3d(*centre), Gf.Vec3d(0, 1, 0)
        )
        q = look.GetInverse().ExtractRotationQuat()
        camera.set_world_pose(
            position=eye,
            orientation=np.array([q.GetReal(), *q.GetImaginary()]),
            camera_axes="usd",
        )
        camera.set_focal_length(0.85)
        state["camera"] = {
            "type": "fixed_hall_overview_with_scan_overlay",
            "eye_m": eye.tolist(),
            "target_m": centre.tolist(),
            "fps": args.fps,
        }
    robot_camera = None
    if args.robot_camera:
        adapter = load_module(
            "slam_drive_perception_adapter", root / "sim/isaac/perception_adapter.py"
        )
        rig = load_module("slam_drive_scene_rig", root / "tools/scene_rig.py")
        camera_mount = adapter.default_base_from_optical()
        calibration = rig.intrinsics()
        robot_camera = Camera(
            prim_path="/World/Forklift/base_link/RobotCamera",
            frequency=-1,
            resolution=(calibration.width, calibration.height),
        )
        robot_camera.set_local_pose(
            translation=np.asarray(camera_mount.translation_m),
            orientation=np.asarray(adapter.xyzw_to_wxyz(rig.OPTICAL_QUATERNION_XYZW)),
            camera_axes="ros",
        )
        robot_camera.set_projection_mode("perspective")
        robot_camera.set_lens_distortion_model("pinhole")
        robot_camera.set_focal_length(1.0)
        robot_camera.set_horizontal_aperture(
            calibration.width / calibration.fx, maintain_square_pixels=True
        )
        state["robot_camera"] = {
            "mount": "perception_adapter.default_base_from_optical (synthetic)",
            "translation_m": list(camera_mount.translation_m),
            "resolution": [calibration.width, calibration.height],
            "fx_px": calibration.fx,
            "depth_video_range_m": list(DEPTH_VIDEO_RANGE_M),
            "timing": "video only: the frame read after each render step, "
            "not a freshness-checked perception capture",
        }
    world.reset()
    if camera is not None:
        camera.initialize()
    if robot_camera is not None:
        robot_camera.initialize()
        robot_camera.add_distance_to_image_plane_to_frame()
    names = list(robot.dof_names)
    wheels = np.array(
        [
            names.index(n)
            for n in [
                "front_left_spin",
                "front_right_spin",
                "rear_left_spin",
                "rear_right_spin",
            ]
        ]
    )
    steers = np.array([names.index("left_steer"), names.index("right_steer")])
    lift_index = np.array([names.index("fork_lift")])
    for _ in range(120):
        world.step(render=args.video)
    settled, _ = robot.get_world_pose()
    require(
        np.linalg.norm(settled[:2] - base_start[:2]) < 0.005, "Truck spawn displaced"
    )

    state["phase"] = "planning"
    travel = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=30000,
        obstacle_heuristic_resolution_m=0.25,
    )
    state["planner_config"] = asdict(travel)
    planning_started = time.monotonic()
    survey = plan_survey_route(factory, layout.survey_route, travel, geometry=geometry)
    state["planning_wall_s"] = time.monotonic() - planning_started
    state["planning_status"] = survey.status
    require(survey.success, f"Survey planning failed: {survey.status}")
    state["survey_length_m"] = survey.length_m
    (args.output / "paths.json").write_text(
        record_json(
            {
                "survey": {
                    "poses": survey.poses,
                    "directions": survey.directions,
                    "curvatures_inv_m": survey.curvatures_inv_m,
                    "length_m": survey.length_m,
                    "expanded_nodes": survey.expanded_nodes,
                }
            },
            indent=2,
        )
        + "\n"
    )
    add_path_display(stage, survey, "Survey", (0.1, 0.55, 1.0))
    stage.GetRootLayer().Export(str(args.output / "scene.usda"))

    obstacles = [prop.rectangle for prop in transport.props] + [
        Rectangle(
            transport.pickup.x_m,
            transport.pickup.y_m,
            geometry.pallet_depth_m,
            geometry.pallet_width_m,
            transport.pickup.yaw_rad,
        )
    ]
    speed = float(lidar_config["survey_speed_mps"])
    tracker = RearAxlePathTracker(
        survey.poses,
        survey.directions,
        survey.curvatures_inv_m,
        TrackerConfig(
            cruise_speed_mps=speed,
            max_curvature_inv_m=settings["tracker_curvature_inv_m"],
            max_acceleration_mps2=settings["drive_acceleration_mps2"],
            lookahead_m=0.28,
            position_tolerance_m=float(lidar_config["survey_position_tolerance_m"]),
            yaw_tolerance_rad=float(lidar_config["survey_yaw_tolerance_rad"]),
            stop_speed_mps=0.012,
            max_cross_track_error_m=0.35,
        ),
    )
    drive_geometry = AckermannGeometry(0.64, 0.51, 0.135, 0.45, 8.0)
    beam_angles = pattern.beam_angles_rad()
    scan_every = 120 // int(lidar_config["rate_hz"])
    frame_every = 120 // args.fps
    time_limit = max(60.0, 3 * survey.length_m / speed + 30)
    log = {
        "joint_stamps_s": [],
        "wheel_rates_rad_s": [],
        "steering_rad": [],
        "base_pose_world": [],
        "scan_stamps_s": [],
        "scan_ranges_m": [],
        "laser_pose_world": [],
    }
    scan_wall_s = 0.0
    self_hits = 0
    latest_points = np.empty((0, 3))
    encoders = {}
    video_frames = []
    steering_command = np.zeros(2)
    state["phase"] = "survey"
    initial_time = world.current_time
    started_wall = time.monotonic()
    arrived_at = None
    try:
        if args.video:
            encoders["overview"] = open_encoder(
                args.output / "survey.mp4", 1280, 720, args.fps
            )
        if robot_camera is not None:
            size = (calibration.width, calibration.height)
            encoders["rgb"] = open_encoder(
                args.output / "camera_rgb.mp4", *size, args.fps
            )
            encoders["depth"] = open_encoder(
                args.output / "camera_depth.mp4", *size, args.fps
            )
        dt = 1 / 120
        for step in range(int(120 * args.max_sim_seconds)):
            t = world.current_time - initial_time
            base, q = robot.get_world_pose()
            yaw, tilt = yaw_and_tilt(q)
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            rear = np.array(
                [
                    base[0] - rear_offset * forward[0],
                    base[1] - rear_offset * forward[1],
                    yaw,
                ]
            )
            require(np.isfinite(base).all(), "Nonfinite body state")
            require(tilt < 0.1, "Excessive body tilt")
            require(
                collision_free_pose(rear, obstacles, geometry.unloaded_footprint, hall),
                "Actual truck footprint overlap during survey",
            )
            require(t < time_limit, "Survey tracking timeout")
            signed_speed = float(np.dot(robot.get_linear_velocity()[:2], forward))
            requested, curvature = 0.0, 0.0
            if arrived_at is None:
                tracking = tracker.update(rear, signed_speed, dt)
                require(
                    tracking.status != "failed",
                    f"Survey tracking failed: pos={tracking.position_error_m:.4f},"
                    f"yaw={tracking.yaw_error_rad:.4f}",
                )
                requested, curvature = tracking.speed_mps, tracking.curvature_inv_m
                if tracking.status == "arrived":
                    arrived_at = t
                    state["arrival_error"] = {
                        "position_m": tracking.position_error_m,
                        "yaw_rad": tracking.yaw_error_rad,
                    }
            elif t - arrived_at > 1.0:
                break
            drive = ackermann_command(requested, curvature, drive_geometry)
            target_steering = np.asarray(drive.steering_rad)
            actual_steering = robot.get_joint_positions()[steers]
            if float(np.max(np.abs(target_steering - actual_steering))) > 0.05:
                drive = ackermann_command(requested * 0.25, curvature, drive_geometry)
            rate = settings["steering_command_rate_rad_s"] * dt
            steering_command += np.clip(target_steering - steering_command, -rate, rate)
            robot.apply_action(
                ArticulationAction(
                    joint_velocities=np.asarray(drive.wheel_rates_rad_s),
                    joint_indices=wheels,
                )
            )
            robot.apply_action(
                ArticulationAction(
                    joint_positions=steering_command, joint_indices=steers
                )
            )
            robot.apply_action(
                ArticulationAction(
                    joint_positions=np.array([0.0]), joint_indices=lift_index
                )
            )
            render = args.video and step % frame_every == 0
            world.step(render=render)
            # Encoders and the scan both read the state this step produced.
            stamp = world.current_time - initial_time
            base, q = robot.get_world_pose()
            log["joint_stamps_s"].append(stamp)
            log["wheel_rates_rad_s"].append(robot.get_joint_velocities()[wheels])
            log["steering_rad"].append(robot.get_joint_positions()[steers])
            log["base_pose_world"].append(np.concatenate((base, q)))
            if step % scan_every == 0:
                cast_started = time.monotonic()
                origin, directions = planar_lidar.laser_rays_world(
                    base, q, mount, beam_angles
                )
                distances, hits, own = planar_lidar.cast_scan(
                    origin, directions, pattern.range_max_m
                )
                scan_wall_s += time.monotonic() - cast_started
                self_hits += own
                ranges = pattern.ranges_from_hits(distances, hits)
                if not log["scan_stamps_s"]:
                    require(
                        np.isfinite(ranges).sum() > pattern.beam_count // 10,
                        "First scan hit almost nothing: check the ray-cast result "
                        "format and the colliders",
                    )
                log["scan_stamps_s"].append(stamp)
                log["scan_ranges_m"].append(ranges.astype(np.float32))
                log["laser_pose_world"].append(
                    planar_lidar.laser_pose_2d(base, q, mount)
                )
                latest_points = planar_lidar.scan_points_world(
                    origin, directions, ranges
                )
            if render:
                rgba = camera.get_rgba()
                require(
                    rgba is not None and rgba.shape == (720, 1280, 4),
                    "Camera did not produce RGB",
                )
                frame = rgba[:, :, :3]
                if len(latest_points):
                    uv = camera.get_image_coords_from_world_points(latest_points)
                    frame = planar_lidar.draw_points(frame, uv, (255, 40, 40), 1)
                encoders["overview"].stdin.write(
                    np.ascontiguousarray(frame, np.uint8).tobytes()
                )
                record = {"time_s": stamp}
                if not video_frames:
                    # Floor points to pixels, for placing the overview offline.
                    floor = np.array(
                        [
                            [hall.x_min_m, hall.y_min_m, 0.0],
                            [hall.x_max_m, hall.y_min_m, 0.0],
                            [hall.x_max_m, hall.y_max_m, 0.0],
                            [hall.x_min_m, hall.y_max_m, 0.0],
                            [*centre[:2], 0.0],
                        ]
                    )
                    state["overview_floor_points"] = {
                        "world_m": floor.tolist(),
                        "pixels_uv": np.asarray(
                            camera.get_image_coords_from_world_points(floor)
                        ).tolist(),
                    }
                if robot_camera is not None:
                    width, height = calibration.width, calibration.height
                    rgb = robot_camera.get_rgba()
                    depth = robot_camera.get_depth()
                    ready = (
                        rgb is not None
                        and np.shape(rgb)[:2] == (height, width)
                        and depth is not None
                        and np.shape(depth)[:2] == (height, width)
                    )
                    record["camera_ready"] = bool(ready)
                    if ready:
                        colour = np.asarray(rgb)[:, :, :3]
                        shaded = video_frames_module.colorize_depth(
                            np.asarray(depth).reshape(height, width),
                            *DEPTH_VIDEO_RANGE_M,
                        )
                    else:
                        colour = shaded = np.zeros((height, width, 3), np.uint8)
                    encoders["rgb"].stdin.write(
                        np.ascontiguousarray(colour, np.uint8).tobytes()
                    )
                    encoders["depth"].stdin.write(
                        np.ascontiguousarray(shaded, np.uint8).tobytes()
                    )
                video_frames.append(record)
                if step == 0:
                    Image.fromarray(np.asarray(frame, np.uint8)).save(
                        args.output / "start.png"
                    )
        require(arrived_at is not None, "Survey exceeded the simulation time budget")
        if args.video:
            Image.fromarray(np.asarray(frame, np.uint8)).save(args.output / "end.png")
    finally:
        for name, encoder in encoders.items():
            encoder.stdin.close()
            require(encoder.wait(timeout=120) == 0, f"{name} video encoding failed")
        if video_frames:
            (args.output / "video_frames.json").write_text(
                record_json(
                    {
                        "fps": args.fps,
                        "clock": "Isaac simulation time since the first recorded step",
                        "videos": sorted(encoders),
                        "overview_floor_points": state.get("overview_floor_points"),
                        "robot_camera": state.get("robot_camera"),
                        "frames": video_frames,
                    },
                    indent=1,
                )
                + "\n"
            )

    ranges = np.asarray(log["scan_ranges_m"])
    finite = np.isfinite(ranges).mean(axis=1)
    np.savez_compressed(
        args.output / "slam_log.npz",
        joint_stamps_s=np.asarray(log["joint_stamps_s"]),
        wheel_rates_rad_s=np.asarray(log["wheel_rates_rad_s"]),
        steering_rad=np.asarray(log["steering_rad"]),
        base_pose_world=np.asarray(log["base_pose_world"]),
        scan_stamps_s=np.asarray(log["scan_stamps_s"]),
        scan_ranges_m=ranges,
        laser_pose_world=np.asarray(log["laser_pose_world"]),
    )
    meta = {
        "format": LOG_FORMAT,
        "clock": "Isaac simulation time since the first recorded step, seconds",
        "source": "synthetic Isaac Sim 5.1 recording, not a physical sensor",
        "frames": {"odom": "odom", "base": "base_link", "laser": "laser"},
        "base_pose_world": "x, y, z, qw, qx, qy, qz of base_link in the stage",
        "laser_pose_world": "x, y, yaw of the laser in the stage (ground truth)",
        "laser": {
            "beam_count": pattern.beam_count,
            "angle_min_rad": pattern.angle_min_rad,
            "angle_increment_rad": pattern.angle_increment_rad,
            "range_min_m": pattern.range_min_m,
            "range_max_m": pattern.range_max_m,
            "rate_hz": int(lidar_config["rate_hz"]),
            "ranges": "REP-117: +inf nothing in range, -inf too close",
            "instantaneous": True,
            "mount_xyz_m": list(mount.xyz_m),
            "mount_yaw_rad": mount.yaw_rad,
        },
        "wheel_order": ["front_left", "front_right", "rear_left", "rear_right"],
        "wheel_rate_sign": "positive rolls the truck forward (drive command sign)",
        "steering_order": ["front_left", "front_right"],
        "odometry_geometry": {
            "wheelbase_m": 0.64,
            "track_m": 0.51,
            "wheel_radius_m": 0.135,
            "rear_axle_x_in_base_m": args.rear_axle_offset_m,
            "source": "sim/models/dls08_provisional/forklift.urdf joint origins",
        },
        "seed": args.seed,
        "layout_version": layout.layout_version,
        "survey_route": [asdict(pose) for pose in layout.survey_route],
        "obstacles": [
            {**asdict(prop.rectangle), "height_m": prop.asset.height_m, "base_m": 0.0}
            for prop in transport.props
        ]
        + [
            {
                **asdict(load.rectangle),
                "height_m": load.asset.height_m,
                "base_m": load.base_height_m,
            }
            for load in factory.loads
        ],
        "hall": asdict(hall),
    }
    (args.output / "meta.json").write_text(record_json(meta, indent=2) + "\n")
    state.update(
        {
            "success": True,
            "scans": len(ranges),
            "joint_samples": len(log["joint_stamps_s"]),
            "scan_finite_fraction_mean": float(finite.mean()),
            "scan_finite_fraction_min": float(finite.min()),
            "scan_self_hits": int(self_hits),
            "scan_cast_wall_s": scan_wall_s,
            "simulated_time_s": float(log["joint_stamps_s"][-1]),
            "simulation_wall_s": time.monotonic() - started_wall,
        }
    )


def main() -> None:
    args = arguments()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    state = {
        "success": False,
        "seed": args.seed,
        "control_feedback": "simulator_ground_truth",
        "recorded_for_slam": ["scans", "wheel_rates", "steering_angles"],
        "ground_truth_recorded_for_evaluation": True,
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "python": sys.version,
        "arguments": {
            k: (str(v) if isinstance(v, Path) else v)
            for k, v in vars(args).items()
            if k != "pallet_geometry_loaded"
        },
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
        run(app, args, state)
    except BaseException as exc:
        state["success"] = False
        state["failure_reason"] = str(exc)
        (args.output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        state["wall_time_s"] = time.monotonic() - started
        (args.output / "result.json").write_text(record_json(state, indent=2) + "\n")
        print("SLAM_DRIVE_RESULT", record_json(state), flush=True)
        if app is not None:
            app.close()
    raise SystemExit(0 if state["success"] else 1)


if __name__ == "__main__":
    main()
