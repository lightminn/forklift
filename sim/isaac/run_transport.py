"""Physical Hybrid A* pallet transport using official warehouse props.

Run with Isaac Sim's Python after installing forklift-core in that environment.
All pose feedback is simulator ground truth; configuration is synthetic. Camera
output is 60fps by default, and the floor destination is marked by a green ring.
"""

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
import traceback
from dataclasses import asdict
from pathlib import Path

import numpy as np

EXIT_CLEARANCE_M = 0.08


def record_json(value: object, *, indent: int | None = None) -> str:
    """Encode simulator numerical records without coercing numbers to strings."""

    def native(value: object) -> object:
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(f"Unsupported record type: {type(value).__name__}")

    return json.dumps(value, default=native, indent=indent, allow_nan=False)


def camera_hz(value: str) -> int:
    """Camera sample period must divide the 120Hz physics time step."""
    rate = int(value)
    if rate <= 0 or 120 % rate:
        raise argparse.ArgumentTypeError(
            "camera frequency must be positive and divide 120"
        )
    return rate


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--pallet-urdf", type=Path, required=True)
    parser.add_argument("--pallet-geometry", type=Path, required=True)
    parser.add_argument(
        "--forklift-urdf",
        type=Path,
        default=Path(__file__).resolve().parent.parent
        / "models/dls08_provisional/forklift.urdf",
    )
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--obstacles", type=int, default=4)
    parser.add_argument("--fps", type=camera_hz, default=60)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--max-sim-seconds", type=float, default=300)
    parser.add_argument(
        "--asset-root",
        default=(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
            "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
        ),
    )
    parser.add_argument("--use-perception", action="store_true")
    parser.add_argument("--pallet-prior", type=Path, default=None)
    parser.add_argument(
        "--observation-waypoints",
        action="append",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW"),
        help=(
            "Ordered observation candidates in metres/radians; repeat this option "
            "for each candidate. Overrides defaults: (-0.10, 0.90, 0), "
            "(-1.20, 0.30, 0), (-0.10, -0.60, 0), (-1.50, -0.60, 0), (-2.00, -0.30, 0)."
        ),
    )
    parser.add_argument(
        "--perception-camera-axes",
        choices=("world", "usd", "ros"),
        default="ros",
    )
    parser.add_argument("--perception-max-attempts", type=int, default=200)
    args, unknown = parser.parse_known_args()
    if args.observation_waypoints is None:
        # (-1.20, 0.30) moved ahead of (-0.10, -0.60): both plan equally well
        # for every seed that can reach either, but seed 3 only detects the
        # pallet from (-1.20, 0.30) -- (-0.10, -0.60) occludes the right
        # pocket there. No seed's chosen candidate changes except seed 3's
        # (confirmed 2026-09-19: re-running the full reachability sweep with
        # this order picks the same candidate as before for every other seed).
        args.observation_waypoints = [
            [-0.10, 0.90, 0.0],
            [-1.20, 0.30, 0.0],
            [-0.10, -0.60, 0.0],
            [-1.50, -0.60, 0.0],
            [-2.00, -0.30, 0.0],
        ]
    if not args.observation_waypoints:
        parser.error("--observation-waypoints requires at least one candidate")
    if args.use_perception:
        if args.pallet_prior is None:
            parser.error("--pallet-prior is required with --use-perception")
        from forklift_core.perception.pallet_prior import load_pallet_prior

        try:
            args.pallet_prior_loaded = load_pallet_prior(args.pallet_prior)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    if args.obstacles < 1 or args.max_sim_seconds <= 0:
        parser.error("obstacles and max-sim-seconds must be positive")
    from insertion_geometry import (
        assert_pallet_urdf_matches_geometry,
        assert_pallet_urdf_matches_named_boxes,
        read_chassis_reference_m,
    )

    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    try:
        pallet_geometry = load_pallet_geometry(args.pallet_geometry)
        assert_pallet_urdf_matches_geometry(
            args.pallet_urdf,
            pallet_geometry.overall_depth_m,
            pallet_geometry.overall_width_m,
        )
        assert_pallet_urdf_matches_named_boxes(args.pallet_urdf, pallet_geometry)
        axle_to_fork_tip_m, rear_axle_offset_m = read_chassis_reference_m(
            args.forklift_urdf
        )
    except ValueError as exc:
        parser.error(str(exc))
    args.axle_to_fork_tip_m = axle_to_fork_tip_m
    args.rear_axle_offset_m = rear_axle_offset_m
    args.pallet_geometry_loaded = pallet_geometry
    # Kit's --portable-root and related application arguments are consumed by Kit.
    args.kit_arguments = unknown
    return args


def yaw_and_tilt(quaternion: np.ndarray) -> tuple[float, float]:
    """Return world yaw and z-axis tilt from scalar-first quaternion."""
    w, x, y, z = quaternion
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    tilt = math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    return yaw, tilt


def require(condition: bool, reason: str) -> None:
    """A failed experimental invariant always aborts, even under Python -O."""
    if not condition:
        raise RuntimeError(reason)


def path_record(path) -> dict:
    return {
        "success": path.success,
        "status": path.status,
        "poses": path.poses.tolist(),
        "directions": path.directions.tolist(),
        "curvatures_inv_m": path.curvatures_inv_m.tolist(),
        "length_m": path.length_m,
        "expanded_nodes": path.expanded_nodes,
    }


def run(app, args: argparse.Namespace, settings: dict, state: dict) -> None:
    """Construct and execute one immutable seeded scenario; state keeps evidence."""
    import omni.usd
    from insertion_geometry import InsertionGeometry
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.sensors.camera import Camera
    from PIL import Image
    from pxr import Gf
    from scene import (
        add_destination,
        add_path_display,
        add_props,
        configure_drives,
        create_pallet,
        read_catalogue,
    )

    import forklift_core
    from forklift_core.control import (
        AckermannGeometry,
        RearAxlePathTracker,
        TrackerConfig,
        ackermann_command,
    )
    from forklift_core.planning import (
        Footprint,
        PlannerConfig,
        Rectangle,
        collision_free_pose,
    )
    from forklift_core.planning.pallet_mission import (
        SyntheticMissionGeometry,
        make_scenario,
        plan_transport,
    )

    if args.use_perception:
        import importlib.util

        from forklift_core.perception.pocket_detector import (
            DetectorParams,
            detect_pockets,
        )
        from forklift_core.planning import Pose2D
        from forklift_core.planning.pallet_mission import plan_observation_leg

        def load_perception_module(name, path):
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
            return module

        root = Path(__file__).resolve().parents[2]
        adapter = load_perception_module(
            "run_transport_perception_adapter", root / "sim/isaac/perception_adapter.py"
        )
        rig = load_perception_module(
            "run_transport_scene_rig", root / "tools/scene_rig.py"
        )

    source_files = [
        Path(__file__),
        Path(__file__).with_name("scene.py"),
        Path(__file__).with_name("insertion_geometry.py"),
    ]
    core_root = Path(forklift_core.__file__).parent
    source_files += sorted((core_root / "control").glob("*.py"))
    source_files += sorted((core_root / "planning").glob("*.py"))
    state["source_sha256"] = {
        str(
            path.relative_to(core_root.parent)
            if path.is_relative_to(core_root)
            else Path("sim/isaac") / path.name
        ): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source_files
    }
    state["phase"] = "scene"
    enable_extension("isaacsim.asset.importer.urdf")
    require(
        omni.usd.get_context().open_stage(args.base_scene), "Cannot open base scene"
    )
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    catalogue, offsets = read_catalogue(stage, app, args.asset_root)
    geometry = SyntheticMissionGeometry(
        unloaded_footprint=Footprint(args.axle_to_fork_tip_m, 0.17, 0.36),
        pallet_depth_m=args.pallet_geometry_loaded.overall_depth_m,
        pallet_width_m=args.pallet_geometry_loaded.overall_width_m,
        axle_to_fork_tip_m=args.axle_to_fork_tip_m,
    )
    insertion_geometry = InsertionGeometry.from_urdfs(
        args.forklift_urdf, args.pallet_urdf
    )
    state["pocket_clearance_margin_m"] = 0.002
    state["pocket_geometry_checks"] = 0
    state["forbidden_pocket_contacts"] = []
    scenario = make_scenario(args.seed, catalogue, args.obstacles, geometry=geometry)
    state["scenario"] = asdict(scenario)
    state["geometry"] = asdict(geometry)
    state["pallet_geometry_source"] = str(args.pallet_geometry)
    state["pallet_geometry_sha256"] = hashlib.sha256(
        args.pallet_geometry.read_bytes()
    ).hexdigest()
    state["asset_origin_offsets_m"] = offsets
    (args.output / "scenario.json").write_text(
        record_json(state["scenario"], indent=2) + "\n"
    )
    state["props"] = add_props(stage, app, scenario.props, offsets)
    state["destination_marker"] = add_destination(
        stage, scenario.destination.x_m, scenario.destination.y_m
    )
    pallet = create_pallet(
        world,
        stage,
        app,
        args.pallet_urdf,
        args.output,
        scenario.pickup,
        settings["pallet_mass_kg"],
    )
    base_start = np.array(
        [
            scenario.start_rear.x_m
            + abs(args.rear_axle_offset_m) * math.cos(scenario.start_rear.yaw_rad),
            scenario.start_rear.y_m
            + abs(args.rear_axle_offset_m) * math.sin(scenario.start_rear.yaw_rad),
            0.015,
        ]
    )
    start_yaw = scenario.start_rear.yaw_rad
    robot = world.scene.add(
        Robot(
            prim_path="/World/Forklift",
            name="forklift",
            position=base_start,
            orientation=np.array(
                [math.cos(start_yaw / 2), 0, 0, math.sin(start_yaw / 2)]
            ),
        )
    )
    state["collision_counts"] = configure_drives(
        stage,
        settings,
        expected_pallet_box_count=len(insertion_geometry.pallet_boxes),
    )
    # Physics-step scheduling sets the recording rate. Acquire every rendered
    # frame so the SDK elapsed-time threshold cannot skip frame metadata.
    camera = Camera(
        prim_path="/World/GlobalCamera", frequency=-1, resolution=(1280, 720)
    )
    b = scenario.bounds
    center = np.array([(b.x_min_m + b.x_max_m) / 2, (b.y_min_m + b.y_max_m) / 2, 0.0])
    eye = center + np.array([0.0, 0.0, 6.0])
    look = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0, 1, 0))
    q = look.GetInverse().ExtractRotationQuat()
    camera.set_world_pose(
        position=eye,
        orientation=np.array([q.GetReal(), *q.GetImaginary()]),
        camera_axes="usd",
    )
    camera.set_focal_length(1.1)
    state["camera"] = {
        "type": "fixed_global_overview",
        "eye_m": eye.tolist(),
        "target_m": center.tolist(),
        "fps": args.fps,
        "resolution": [1280, 720],
    }
    if args.use_perception:
        perception_mount = adapter.default_base_from_optical()
        perception_calibration = rig.intrinsics()
        perception_camera = Camera(
            prim_path="/World/Forklift/base_link/PerceptionCamera",
            frequency=-1,
            resolution=(perception_calibration.width, perception_calibration.height),
        )
        perception_camera.set_local_pose(
            translation=np.asarray(perception_mount.translation_m),
            orientation=np.asarray(adapter.xyzw_to_wxyz(rig.OPTICAL_QUATERNION_XYZW)),
            camera_axes=args.perception_camera_axes,
        )
        perception_camera.set_projection_mode("perspective")
        perception_camera.set_lens_distortion_model("pinhole")
        perception_camera.set_focal_length(1.0)
        perception_camera.set_horizontal_aperture(
            perception_calibration.width / perception_calibration.fx,
            maintain_square_pixels=True,
        )
    world.reset()
    camera.initialize()
    if args.use_perception:
        perception_camera.initialize()
        # Capture needs axial depth as well as RGBA (see determinism_probe.py).
        perception_camera.add_distance_to_image_plane_to_frame()
        perception_capture = adapter.SensorCapture(
            perception_camera,
            perception_mount,
            step_fn=lambda: world.step(render=True),
            physics_time_fn=lambda: world.current_time,
            pose_fn=robot.get_world_pose,
        )
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
    initial_pallet, initial_pallet_q = pallet.get_world_pose()
    initial_base, _ = robot.get_world_pose()
    require(
        np.linalg.norm(initial_pallet[:2] - [scenario.pickup.x_m, scenario.pickup.y_m])
        < 0.005
        and abs(initial_pallet[2]) < 0.005,
        "Invalid pallet spawn after settling",
    )
    require(
        np.linalg.norm(initial_base[:2] - base_start[:2]) < 0.005,
        "Truck spawn displaced",
    )
    state["initial_pallet_m"] = initial_pallet.tolist()
    state["phase"] = "planning"
    if args.use_perception:
        planning_start = time.monotonic()
        state["observation_candidates"] = []
        state["observation_attempts"] = []
        next_candidate_index = 0
        state["observation_waypoint_selected"] = None
        observe_plan = None
        for candidate_index, coordinates in enumerate(args.observation_waypoints):
            next_candidate_index = candidate_index + 1
            waypoint = Pose2D(*coordinates)
            candidate_plan = plan_observation_leg(
                scenario,
                waypoint,
                PlannerConfig(
                    curvature_limit_inv_m=settings["planner_curvature_inv_m"],
                    clearance_m=settings["planning_clearance_m"],
                    max_expansions=30000,
                    primitive_length_m=0.25,
                ),
                geometry=geometry,
            )
            state["observation_candidates"].append(
                {
                    "candidate_index": candidate_index,
                    "pose": [waypoint.x_m, waypoint.y_m, waypoint.yaw_rad],
                    "success": candidate_plan.success,
                    "status": candidate_plan.status,
                }
            )
            if candidate_plan.success:
                observe_plan = candidate_plan
                state["observation_waypoint_selected"] = [
                    waypoint.x_m,
                    waypoint.y_m,
                    waypoint.yaw_rad,
                ]
                break
        state["planning_wall_s"] = time.monotonic() - planning_start
        if observe_plan is None:
            state["planning_status"] = "all_candidates_failed"
            reasons = ";".join(
                candidate["status"] for candidate in state["observation_candidates"]
            )
            require(False, f"observe:all_candidates_failed:{reasons}")
        state["planning_status"] = observe_plan.status
        paths = {"observe": observe_plan}
    else:
        planning_start = time.monotonic()
        plans = plan_transport(
            scenario,
            PlannerConfig(
                curvature_limit_inv_m=settings["planner_curvature_inv_m"],
                clearance_m=settings["planning_clearance_m"],
                max_expansions=30000,
            ),
            geometry=geometry,
        )
        state["planning_wall_s"] = time.monotonic() - planning_start
        state["planning_status"] = plans.status
        require(plans.success, f"Mission planning failed: {plans.status}")
        paths = {
            name: getattr(plans, name)
            for name in ["approach", "insert", "extract", "transport", "withdraw"]
        }
        state["paths"] = {name: path_record(path) for name, path in paths.items()}
        (args.output / "paths.json").write_text(
            record_json(state["paths"], indent=2) + "\n"
        )
        add_path_display(stage, paths["approach"], "Approach", (0.05, 0.45, 1.0))
        add_path_display(stage, paths["transport"], "Transport", (1.0, 0.65, 0.04))
        stage.GetRootLayer().Export(str(args.output / "scene.usda"))
        print(
            "PLANNED",
            record_json(
                {
                    k: {"length_m": v.length_m, "expansions": v.expanded_nodes}
                    for k, v in paths.items()
                }
            ),
            flush=True,
        )

    drive_geometry = AckermannGeometry(0.64, 0.51, 0.135, 0.45, 8.0)
    obstacles = [item.rectangle for item in scenario.props]
    pickup_obstacle = Rectangle(
        scenario.pickup.x_m,
        scenario.pickup.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        scenario.pickup.yaw_rad,
    )
    fps_divisor = 120 // args.fps
    dt = 1 / 120
    encoder = None
    frame_audit = []
    speeds = {
        "approach": settings["approach_speed_mps"],
        "insert": settings["insert_speed_mps"],
        "extract": settings["extract_speed_mps"],
        "transport": settings["transport_speed_mps"],
        "withdraw": settings["withdraw_speed_mps"],
    }
    if args.use_perception:
        # Observation travel reuses approach speed; no new settings YAML key.
        speeds["observe"] = settings["approach_speed_mps"]
    trackers = {
        name: RearAxlePathTracker(
            path.poses,
            path.directions,
            path.curvatures_inv_m,
            TrackerConfig(
                cruise_speed_mps=speeds[name],
                max_curvature_inv_m=settings["tracker_curvature_inv_m"],
                max_acceleration_mps2=settings["drive_acceleration_mps2"],
                lookahead_m=0.28,
                position_tolerance_m=0.008,
                yaw_tolerance_rad=0.03 if name == "observe" else 0.02,
                stop_speed_mps=0.012,
                max_cross_track_error_m=0.35,
            ),
        )
        for name, path in paths.items()
    }
    phase = "approach"
    if args.use_perception:
        phase = "observe"
    phase_started = 0.0
    initial_time = world.current_time
    simulation_started_wall = time.monotonic()
    lift_command = 0.0
    steering_command = np.zeros(2)
    state["transitions"] = []
    state["samples"] = []
    state["frames"] = 0
    state["phase"] = phase

    def snapshot(label: str) -> None:
        if args.video:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.shape == (720, 1280, 4):
                Image.fromarray(rgba.astype(np.uint8)).convert("RGB").save(
                    args.output / f"{label}.png"
                )

    def transition(next_phase: str, t: float) -> None:
        nonlocal phase, phase_started
        state["transitions"].append({"from": phase, "to": next_phase, "time_s": t})
        print("TRANSITION", record_json(state["transitions"][-1]), flush=True)
        snapshot(phase)
        phase, phase_started = next_phase, t
        state["phase"] = phase

    try:
        if args.video:
            encoder = subprocess.Popen(
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
                    "1280x720",
                    "-r",
                    str(args.fps),
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
                    str(args.output / "transport.mp4"),
                ],
                stdin=subprocess.PIPE,
            )
        for step in range(int(120 * args.max_sim_seconds)):
            t = world.current_time - initial_time
            base, q = robot.get_world_pose()
            ppos, pq = pallet.get_world_pose()
            yaw, tilt = yaw_and_tilt(q)
            pallet_yaw, pallet_tilt = yaw_and_tilt(pq)
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            rear = np.array(
                [
                    base[0] - abs(args.rear_axle_offset_m) * forward[0],
                    base[1] - abs(args.rear_axle_offset_m) * forward[1],
                    yaw,
                ]
            )
            velocity = robot.get_linear_velocity()
            signed_speed = float(np.dot(velocity[:2], forward))
            require(np.isfinite([base, ppos]).all(), "Nonfinite body state")
            require(
                tilt < 0.1 and pallet_tilt < 0.15, f"Excessive body tilt in {phase}"
            )
            loaded = phase in ["lift", "extract", "transport", "lower"]
            footprint = (
                geometry.loaded_footprint if loaded else geometry.unloaded_footprint
            )
            checked_obstacles = obstacles + (
                [pickup_obstacle] if phase in ("approach", "observe") else []
            )
            # Runtime pose checks below use zero margin intentionally --
            # clearance_m is a planning-time buffer against the intended path,
            # not a re-check of the executed pose; spawn_clearance_m already
            # keeps obstacles far enough that a nominal run clears this at
            # margin 0 (docs/plans/2026-09-17-hybrid-astar-transport.md,
            # "계획 여유가 시험으로 고정돼 있지 않다").
            require(
                collision_free_pose(
                    rear, checked_obstacles, footprint, scenario.bounds
                ),
                f"Actual truck/load footprint overlap in {phase}",
            )
            require(
                collision_free_pose(
                    [ppos[0], ppos[1], pallet_yaw],
                    obstacles,
                    Footprint(
                        geometry.pallet_depth_m / 2,
                        geometry.pallet_depth_m / 2,
                        geometry.pallet_width_m / 2,
                    ),
                    scenario.bounds,
                ),
                f"Measured pallet footprint overlap in {phase}",
            )
            relative = ppos[:2] - base[:2]
            if phase in ["extract", "transport"]:
                require(ppos[2] > 0.04, "Pallet dropped during transport")
                require(
                    abs(
                        float(np.dot(relative, forward))
                        - (geometry.inserted_offset_m - abs(args.rear_axle_offset_m))
                    )
                    < 0.08
                    and abs(float(np.dot(relative, [-forward[1], forward[0]]))) < 0.04,
                    "Pallet slipped off forks",
                )
            if phase in ["approach", "insert"]:
                require(
                    np.linalg.norm(ppos[:2] - initial_pallet[:2]) < 0.018,
                    "Pallet displaced before pickup",
                )
            if phase in ["insert", "withdraw"]:
                contacts = insertion_geometry.forbidden_contacts(
                    base,
                    q,
                    float(robot.get_joint_positions()[lift_index[0]]),
                    ppos,
                    pq,
                    clearance_m=state["pocket_clearance_margin_m"],
                )
                state["pocket_geometry_checks"] += 1
                if contacts:
                    state["forbidden_pocket_contacts"] = contacts
                require(
                    not contacts,
                    f"Forbidden fork/pallet contact in {phase}: {contacts}",
                )
            requested_speed, curvature = 0.0, 0.0
            tracking = None
            if phase in trackers:
                limit = max(30.0, 3 * paths[phase].length_m / speeds[phase] + 10)
                require(t - phase_started < limit, f"Tracking timeout in {phase}")
                tracking = trackers[phase].update(rear, signed_speed, dt)
                require(
                    tracking.status != "failed",
                    f"Tracking failed in {phase}: pos={tracking.position_error_m:.4f},yaw={tracking.yaw_error_rad:.4f}",
                )
                requested_speed, curvature = (
                    tracking.speed_mps,
                    tracking.curvature_inv_m,
                )
                if tracking.status == "arrived":
                    if args.use_perception and phase == "observe":
                        attempt_number = len(state["observation_attempts"]) + 1
                        attempt = {
                            "candidate_index": next_candidate_index - 1,
                            "waypoint": list(state["observation_waypoint_selected"]),
                            "arrived_pose": rear.tolist(),
                            "pocket_observation": None,
                            "frame_diagnostics": None,
                            "detection_diagnostics": None,
                            "capture_attempts": None,
                        }
                        state["observation_attempts"].append(attempt)
                        try:
                            scene_input, frame_diagnostics, capture_attempts = (
                                perception_capture.capture(
                                    max_attempts=args.perception_max_attempts,
                                )
                            )
                        except adapter.CaptureFailure as exc:
                            attempt["capture_failure"] = exc.reason
                            attempt["capture_diagnostics"] = asdict(exc.diagnostics)
                            attempt["capture_attempts"] = exc.diagnostics.attempts
                            require(False, f"perception_capture_failed:{exc.reason}")
                        # Diagnostic dump for offline root-cause analysis; not part
                        # of the perception contract itself.
                        Image.fromarray(scene_input.rgb).save(
                            args.output / f"perception_capture_{attempt_number}_rgb.png"
                        )
                        np.save(
                            args.output
                            / f"perception_capture_{attempt_number}_depth_m.npy",
                            scene_input.depth_m,
                        )
                        finite_depth = scene_input.depth_m[
                            np.isfinite(scene_input.depth_m)
                        ]
                        if finite_depth.size:
                            depth_range = (
                                float(finite_depth.min()),
                                float(finite_depth.max()),
                            )
                            normalized = np.clip(
                                (scene_input.depth_m - depth_range[0])
                                / max(depth_range[1] - depth_range[0], 1e-6),
                                0,
                                1,
                            )
                            normalized = np.nan_to_num(normalized, nan=0.0)
                            Image.fromarray((normalized * 255).astype(np.uint8)).save(
                                args.output
                                / f"perception_capture_{attempt_number}_depth_vis.png"
                            )
                        # Use the accepted end pose as the stationary acquisition
                        # representative; no timestamp interpolation is implied.
                        capture_diagnostics = perception_capture.state.diagnostics
                        attempt["capture_diagnostics"] = asdict(capture_diagnostics)
                        base, q = map(np.asarray, capture_diagnostics.accepted_pose)
                        yaw, _ = yaw_and_tilt(q)
                        forward = np.array([math.cos(yaw), math.sin(yaw)])
                        rear = np.array(
                            [
                                base[0] - abs(args.rear_axle_offset_m) * forward[0],
                                base[1] - abs(args.rear_axle_offset_m) * forward[1],
                                yaw,
                            ]
                        )
                        attempt.update(
                            {
                                "frame_diagnostics": asdict(frame_diagnostics),
                                "capture_attempts": capture_attempts,
                            }
                        )
                        prior = args.pallet_prior_loaded
                        params = DetectorParams.derived_for(prior)
                        detection = detect_pockets(scene_input, prior, params)
                        observation = detection.observation
                        attempt.update(
                            {
                                "pocket_observation": asdict(observation),
                                "frame_diagnostics": asdict(frame_diagnostics),
                                "detection_diagnostics": asdict(detection.diagnostics),
                                "capture_attempts": capture_attempts,
                            }
                        )
                        if observation.status != "valid":
                            attempt["retry_reason"] = (
                                f"perception_{observation.status}:{observation.reason}"
                            )
                            planning_start = time.monotonic()
                            observe_plan = None
                            while next_candidate_index < len(
                                args.observation_waypoints
                            ):
                                candidate_index = next_candidate_index
                                next_candidate_index += 1
                                coordinates = args.observation_waypoints[
                                    candidate_index
                                ]
                                waypoint = Pose2D(*coordinates)
                                candidate_plan = plan_observation_leg(
                                    scenario,
                                    waypoint,
                                    PlannerConfig(
                                        curvature_limit_inv_m=settings[
                                            "planner_curvature_inv_m"
                                        ],
                                        clearance_m=settings["planning_clearance_m"],
                                        max_expansions=30000,
                                        primitive_length_m=0.25,
                                    ),
                                    geometry=geometry,
                                    start_rear=Pose2D(rear[0], rear[1], rear[2]),
                                )
                                state["observation_candidates"].append(
                                    {
                                        "candidate_index": candidate_index,
                                        "pose": list(coordinates),
                                        "start_rear": rear.tolist(),
                                        "success": candidate_plan.success,
                                        "status": candidate_plan.status,
                                    }
                                )
                                if candidate_plan.success:
                                    observe_plan = candidate_plan
                                    state["observation_waypoint_selected"] = list(
                                        coordinates
                                    )
                                    break
                            state["planning_wall_s"] += (
                                time.monotonic() - planning_start
                            )
                            if observe_plan is None:
                                state["planning_status"] = "all_candidates_exhausted"
                                reasons = ";".join(
                                    candidate["status"]
                                    for candidate in state["observation_candidates"]
                                )
                                require(
                                    False,
                                    f"observe:all_candidates_exhausted:{attempt['retry_reason']}:{reasons}",
                                )
                            state["planning_status"] = observe_plan.status
                            paths["observe"] = observe_plan
                            trackers["observe"] = RearAxlePathTracker(
                                observe_plan.poses,
                                observe_plan.directions,
                                observe_plan.curvatures_inv_m,
                                TrackerConfig(
                                    cruise_speed_mps=speeds["observe"],
                                    max_curvature_inv_m=settings[
                                        "tracker_curvature_inv_m"
                                    ],
                                    max_acceleration_mps2=settings[
                                        "drive_acceleration_mps2"
                                    ],
                                    lookahead_m=0.28,
                                    position_tolerance_m=0.008,
                                    yaw_tolerance_rad=0.03,
                                    stop_speed_mps=0.012,
                                    max_cross_track_error_m=0.35,
                                ),
                            )
                            phase_started = t
                        else:
                            state["perception"] = attempt.copy()
                            base_xy = adapter.estimate_pallet_center_m(
                                observation, geometry.pallet_depth_m
                            )
                            target_pickup = adapter.estimate_world_pallet_site(
                                base_xy,
                                observation.insertion_yaw_rad,
                                (base[0], base[1]),
                                yaw,
                            )
                            start_rear_pose = Pose2D(rear[0], rear[1], rear[2])
                            state["perception"]["perception_pickup_estimate_m"] = {
                                "x_m": target_pickup.x_m,
                                "y_m": target_pickup.y_m,
                                "yaw_rad": target_pickup.yaw_rad,
                            }
                            # Ground-truth pickup is for error evaluation here, never
                            # the planning target; the collision map still uses it.
                            state["perception"]["perception_error"] = {
                                "position_m": float(
                                    np.hypot(
                                        target_pickup.x_m - scenario.pickup.x_m,
                                        target_pickup.y_m - scenario.pickup.y_m,
                                    )
                                ),
                                "yaw_rad": float(
                                    math.atan2(
                                        math.sin(
                                            target_pickup.yaw_rad
                                            - scenario.pickup.yaw_rad
                                        ),
                                        math.cos(
                                            target_pickup.yaw_rad
                                            - scenario.pickup.yaw_rad
                                        ),
                                    )
                                ),
                            }
                            planning_start = time.monotonic()
                            plans = plan_transport(
                                scenario,
                                PlannerConfig(
                                    curvature_limit_inv_m=settings[
                                        "planner_curvature_inv_m"
                                    ],
                                    clearance_m=settings["planning_clearance_m"],
                                    max_expansions=30000,
                                ),
                                geometry=geometry,
                                target_pickup=target_pickup,
                                start_rear=start_rear_pose,
                            )
                            state["planning_wall_s"] += (
                                time.monotonic() - planning_start
                            )
                            state["planning_status"] = plans.status
                            # MissionPlan.status already contains the failing phase.
                            require(plans.success, plans.status)
                            paths.update(
                                {
                                    name: getattr(plans, name)
                                    for name in [
                                        "approach",
                                        "insert",
                                        "extract",
                                        "transport",
                                        "withdraw",
                                    ]
                                }
                            )
                            trackers.update(
                                {
                                    name: RearAxlePathTracker(
                                        path.poses,
                                        path.directions,
                                        path.curvatures_inv_m,
                                        TrackerConfig(
                                            cruise_speed_mps=speeds[name],
                                            max_curvature_inv_m=settings[
                                                "tracker_curvature_inv_m"
                                            ],
                                            max_acceleration_mps2=settings[
                                                "drive_acceleration_mps2"
                                            ],
                                            lookahead_m=0.28,
                                            position_tolerance_m=0.008,
                                            yaw_tolerance_rad=0.02,
                                            stop_speed_mps=0.012,
                                            max_cross_track_error_m=0.35,
                                        ),
                                    )
                                    for name, path in paths.items()
                                    if name != "observe"
                                }
                            )
                            state["paths"] = {
                                name: path_record(path) for name, path in paths.items()
                            }
                            (args.output / "paths.json").write_text(
                                record_json(state["paths"], indent=2) + "\n"
                            )
                            add_path_display(
                                stage, paths["approach"], "Approach", (0.05, 0.45, 1.0)
                            )
                            add_path_display(
                                stage,
                                paths["transport"],
                                "Transport",
                                (1.0, 0.65, 0.04),
                            )
                            stage.GetRootLayer().Export(str(args.output / "scene.usda"))
                            print(
                                "PLANNED",
                                record_json(
                                    {
                                        k: {
                                            "length_m": v.length_m,
                                            "expansions": v.expanded_nodes,
                                        }
                                        for k, v in paths.items()
                                    }
                                ),
                                flush=True,
                            )
                            transition("approach", t)
                    elif phase == "approach":
                        transition("insert", t)
                    elif phase == "insert":
                        state["insertion_error"] = {
                            "position_m": tracking.position_error_m,
                            "yaw_rad": tracking.yaw_error_rad,
                        }
                        transition("lift", t)
                    elif phase == "extract":
                        transition("transport", t)
                    elif phase == "transport":
                        transition("lower", t)
                    elif phase == "withdraw":
                        transition("settle", t)
            desired_lift = (
                settings["lift_target_m"]
                if phase in ["lift", "extract", "transport"]
                else 0.0
            )
            lift_command += float(
                np.clip(
                    desired_lift - lift_command,
                    -settings["lift_rate_mps"] * dt,
                    settings["lift_rate_mps"] * dt,
                )
            )
            if phase == "lift" and t - phase_started > 4.5:
                require(ppos[2] > 0.06, "Pallet was not lifted")
                transition("extract", t)
            if phase == "lower" and t - phase_started > 4.5:
                require(
                    abs(ppos[2]) < 0.008, "Pallet did not settle onto destination floor"
                )
                require(
                    robot.get_joint_positions()[lift_index[0]] < 0.012,
                    "Fork did not lower",
                )
                transition("withdraw", t)
            if phase == "settle" and t - phase_started > 1.0:
                transition("complete", t)
                break
            drive = ackermann_command(requested_speed, curvature, drive_geometry)
            target_steering = np.asarray(drive.steering_rad)
            actual_steering = robot.get_joint_positions()[steers]
            # Creep while steering catches up; log the measured physical response.
            steering_error = float(np.max(np.abs(target_steering - actual_steering)))
            if steering_error > 0.05:
                drive = ackermann_command(
                    requested_speed * 0.25, curvature, drive_geometry
                )
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
                    joint_positions=np.array([lift_command]), joint_indices=lift_index
                )
            )
            world.step(render=args.video and step % fps_divisor == 0)
            if args.video and step % fps_divisor == 0:
                frame = camera.get_current_frame()
                rgba = camera.get_rgba()
                # get_rgba reads the RGB annotator directly. The SDK's cached
                # frame metadata can lag; retain it for audit without treating
                # its timestamp as a control or image-acquisition failure.
                require(
                    rgba is not None and rgba.shape == (720, 1280, 4),
                    "Camera did not produce RGB",
                )
                encoder.stdin.write(
                    np.ascontiguousarray(rgba[:, :, :3], dtype=np.uint8).tobytes()
                )
                state["frames"] += 1
                frame_audit.append(
                    {
                        "simulation_time_s": world.current_time - initial_time,
                        "rendering_time": frame.get("rendering_time"),
                    }
                )
                if state["frames"] == 1:
                    snapshot("start")
            if step % 12 == 0:
                sample = {
                    "time_s": t,
                    "phase": phase,
                    "base_position_m": base.tolist(),
                    "rear_pose": rear.tolist(),
                    "base_tilt_rad": tilt,
                    "signed_speed_mps": signed_speed,
                    "pallet_position_m": ppos.tolist(),
                    "pallet_yaw_rad": pallet_yaw,
                    "pallet_tilt_rad": pallet_tilt,
                    "speed_command_mps": drive.speed_mps,
                    "curvature_inv_m": drive.curvature_inv_m,
                    "steering_command_rad": steering_command.tolist(),
                    "steering_actual_rad": actual_steering.tolist(),
                    "lift_m": float(robot.get_joint_positions()[lift_index[0]]),
                }
                if tracking is not None:
                    sample["tracking"] = asdict(tracking)
                state["samples"].append(sample)
                if step % 240 == 0:
                    print("SAMPLE", record_json(sample), flush=True)
        require(phase == "complete", "Mission exceeded simulation time budget")
        final_pallet, _ = pallet.get_world_pose()
        destination = np.array([scenario.destination.x_m, scenario.destination.y_m])
        delivery_error = float(np.linalg.norm(final_pallet[:2] - destination))
        require(delivery_error < 0.08, "Pallet missed the green destination center")
        require(abs(final_pallet[2]) < 0.008, "Delivered pallet is not grounded")
        final_base, final_q = robot.get_world_pose()
        final_yaw = yaw_and_tilt(final_q)[0]
        # final_base is base_link, not the rear axle -- axle_to_fork_tip_m is
        # measured from the axle, so subtract the axle-to-base_link offset first.
        tip = final_base[:2] + (
            args.axle_to_fork_tip_m - abs(args.rear_axle_offset_m)
        ) * np.array([math.cos(final_yaw), math.sin(final_yaw)])
        destination_axis = np.array(
            [
                math.cos(scenario.destination.yaw_rad),
                math.sin(scenario.destination.yaw_rad),
            ]
        )
        require(
            float(np.dot(final_pallet[:2] - tip, destination_axis))
            > geometry.pallet_depth_m / 2 + EXIT_CLEARANCE_M,
            "Fork still inside delivered pallet",
        )
        snapshot("delivered")
        state.update(
            {
                "success": True,
                "delivery_error_m": delivery_error,
                "final_pallet_m": final_pallet.tolist(),
                "simulated_time_s": world.current_time - initial_time,
                "simulation_wall_s": time.monotonic() - simulation_started_wall,
                "pallet_max_height_m": max(
                    r["pallet_position_m"][2] for r in state["samples"]
                ),
            }
        )
    finally:
        if encoder is not None:
            encoder.stdin.close()
            require(encoder.wait(timeout=60) == 0, "Video encoding failed")
        (args.output / "frame_audit.json").write_text(
            record_json(frame_audit, indent=2) + "\n"
        )


def main() -> None:
    args = arguments()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    import yaml

    settings = yaml.safe_load(args.settings.read_text())
    require(settings["physics_hz"] == 120, "This adapter requires 120Hz physics")
    require(
        all(
            isinstance(v, (int, float)) and math.isfinite(v) and v > 0
            for v in settings.values()
        ),
        "Settings must be positive finite values",
    )
    state = {
        "success": False,
        "seed": args.seed,
        "feedback": "simulator_ground_truth",
        "physical_wheel_drive": True,
        "pallet_fixed_attachment": False,
        "pose_teleportation_after_reset": False,
        "settings_synthetic": settings,
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "pallet_urdf_sha256": hashlib.sha256(args.pallet_urdf.read_bytes()).hexdigest(),
        "forklift_urdf_sha256": hashlib.sha256(
            args.forklift_urdf.read_bytes()
        ).hexdigest(),
        "python": sys.version,
        "arguments": {
            k: (
                asdict(v)
                if k == "pallet_geometry_loaded"
                else str(v)
                if isinstance(v, Path)
                else v
            )
            for k, v in vars(args).items()
        },
    }
    if args.use_perception:
        # PalletPrior is a dataclass, not a JSON-native argparse value.
        state["arguments"]["pallet_prior_loaded"] = asdict(args.pallet_prior_loaded)
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
    except BaseException as exc:
        state["success"] = False
        state["failure_reason"] = str(exc)
        (args.output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        state["wall_time_s"] = time.monotonic() - started
        (args.output / "result.json").write_text(record_json(state, indent=2) + "\n")
        print(
            "MISSION_RESULT",
            record_json(
                {k: v for k, v in state.items() if k not in ["samples", "paths"]}
            ),
            flush=True,
        )
        if app is not None:
            app.close()
    raise SystemExit(0 if state["success"] else 1)


if __name__ == "__main__":
    main()
