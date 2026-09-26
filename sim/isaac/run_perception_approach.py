"""Camera-estimated EPAL 6 alignment approach; no insertion or pallet truth target.

Run using Isaac Sim Python with forklift-core installed. Robot localization and
factory-prop obstacle bounds remain simulator inputs. All settings are synthetic.
"""

import argparse
import faulthandler
import hashlib
import math
import os
import subprocess
import time
import traceback
from dataclasses import asdict
from pathlib import Path

import numpy as np
from run_transport import path_record, record_json, require, yaw_and_tilt


def arguments():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--obstacles", type=int, default=4)
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--hide-pallet",
        action="store_true",
        help="Negative experiment: hide the pallet from rendering",
    )
    parser.add_argument("--capture-only", action="store_true")
    parser.add_argument(
        "--settings", type=Path, default=root / "config/isaac_transport.yaml"
    )
    parser.add_argument(
        "--prior", type=Path, default=root / "config/pallet_prior_epal6.yaml"
    )
    parser.add_argument(
        "--camera-settings",
        type=Path,
        default=root / "config/isaac_perception_camera.yaml",
    )
    parser.add_argument(
        "--detector-settings",
        type=Path,
        default=root / "config/isaac_perception_detector.yaml",
    )
    parser.add_argument(
        "--forklift-urdf",
        type=Path,
        default=root / "sim/models/dls08_provisional/forklift.urdf",
    )
    parser.add_argument(
        "--pallet-urdf", type=Path, default=root / "sim/models/epal6_pallet/pallet.urdf"
    )
    parser.add_argument(
        "--asset-root",
        default=(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
            "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse"
        ),
    )
    args, _ = parser.parse_known_args()
    require(args.obstacles >= 0, "obstacles must be nonnegative")
    return args


def build_scene(app, args, settings, state):
    """Place experiment labels here; consumers receive only sensor observations."""
    import omni.kit.commands
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, UsdGeom
    from scene import (
        add_destination,
        add_props,
        configure_drives,
        create_pallet,
        read_catalogue,
    )

    from forklift_core.planning.pallet_mission import make_scenario

    enable_extension("isaacsim.asset.importer.urdf")
    omni.usd.get_context().new_stage()
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    stage = omni.usd.get_context().get_stage()
    stage.DefinePrim("/World/Environment", "Xform").GetReferences().AddReference(
        args.asset_root + "/full_warehouse.usd"
    )
    for _ in range(30):
        app.update()
    ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
    require(ok, "Cannot create URDF importer")
    config.merge_fixed_joints = False
    config.import_inertia_tensor = True
    config.fix_base = False
    config.self_collision = False
    config.collision_from_visuals = False
    config.create_physics_scene = False
    config.distance_scale = 1.0
    config.make_default_prim = True
    robot_usd = args.output / "forklift.usd"
    ok, _ = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(args.forklift_urdf),
        import_config=config,
        dest_path=str(robot_usd),
    )
    require(ok, "Forklift URDF import failed")
    stage.DefinePrim("/World/Forklift", "Xform").GetReferences().AddReference(
        str(robot_usd)
    )
    catalogue, offsets = read_catalogue(stage, app, args.asset_root + "/Props")
    scenario = make_scenario(args.seed, catalogue, args.obstacles)
    state["scenario_ground_truth_for_evaluation"] = asdict(scenario)
    state["props"] = add_props(stage, app, scenario.props, offsets)
    add_destination(stage, scenario.destination.x_m, scenario.destination.y_m)
    pallet = create_pallet(
        world,
        stage,
        app,
        args.pallet_urdf,
        args.output,
        scenario.pickup,
        settings["pallet_mass_kg"],
    )
    if args.hide_pallet:
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/Pallet")).MakeInvisible()
    start = scenario.start_rear
    robot = world.scene.add(
        Robot(
            prim_path="/World/Forklift",
            name="forklift",
            position=np.array(
                [
                    start.x_m + 0.34 * math.cos(start.yaw_rad),
                    start.y_m + 0.34 * math.sin(start.yaw_rad),
                    0.015,
                ]
            ),
            orientation=np.array(
                [math.cos(start.yaw_rad / 2), 0, 0, math.sin(start.yaw_rad / 2)]
            ),
        )
    )
    state["collision_counts"] = configure_drives(stage, settings)
    overview = Camera(prim_path="/World/Overview", frequency=-1, resolution=(1280, 720))
    eye = np.array([0.85, 0.65, 6.0])
    look = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*eye), Gf.Vec3d(0.85, 0.65, 0), Gf.Vec3d(0, 1, 0)
    )
    q = look.GetInverse().ExtractRotationQuat()
    overview.set_world_pose(
        position=eye,
        orientation=np.array([q.GetReal(), *q.GetImaginary()]),
        camera_axes="usd",
    )
    overview.set_focal_length(1.1)
    return world, stage, robot, pallet, overview, scenario


def follow_approach(
    world, robot, overview, path, obstacles, bounds, settings, output, video
):
    """Execute one observation-derived path; no pallet pose is an input."""
    from isaacsim.core.utils.types import ArticulationAction
    from PIL import Image

    from forklift_core.control import (
        AckermannGeometry,
        RearAxlePathTracker,
        TrackerConfig,
        ackermann_command,
    )
    from forklift_core.planning import collision_free_pose
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

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
    tracker = RearAxlePathTracker(
        path.poses,
        path.directions,
        path.curvatures_inv_m,
        TrackerConfig(
            cruise_speed_mps=settings["approach_speed_mps"],
            max_curvature_inv_m=settings["tracker_curvature_inv_m"],
            max_acceleration_mps2=settings["drive_acceleration_mps2"],
            lookahead_m=0.28,
            position_tolerance_m=0.008,
            yaw_tolerance_rad=0.02,
            stop_speed_mps=0.012,
            max_cross_track_error_m=0.35,
        ),
    )
    geometry = AckermannGeometry(0.64, 0.51, 0.135, 0.45, 8)
    footprint = SyntheticMissionGeometry().unloaded_footprint
    steering = np.zeros(2)
    encoder = None
    samples = []
    started = world.current_time
    success = False
    try:
        if video:
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
                    "60",
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
                    str(output / "approach.mp4"),
                ],
                stdin=subprocess.PIPE,
            )
        limit_s = max(30, 3 * path.length_m / settings["approach_speed_mps"] + 10)
        for step in range(math.ceil(120 * limit_s)):
            base, q = robot.get_world_pose()
            yaw, tilt = yaw_and_tilt(q)
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            rear = np.array([*(base[:2] - 0.34 * forward), yaw])
            speed = float(robot.get_linear_velocity()[:2] @ forward)
            require(tilt < 0.1, "Truck tilt limit exceeded")
            require(
                collision_free_pose(rear, obstacles, footprint, bounds),
                "Truck overlap with mapped obstacles or observed pallet",
            )
            command = tracker.update(rear, speed, 1 / 120)
            require(command.status != "failed", f"Tracking failed: {command}")
            if step % 12 == 0:
                samples.append(
                    {
                        "time_s": world.current_time - started,
                        "rear_pose": rear.tolist(),
                        "signed_speed_mps": speed,
                        "tracking": asdict(command),
                    }
                )
            if command.status == "arrived":
                success = True
                break
            drive = ackermann_command(
                command.speed_mps, command.curvature_inv_m, geometry
            )
            desired = np.asarray(drive.steering_rad)
            if np.max(np.abs(desired - robot.get_joint_positions()[steers])) > 0.05:
                drive = ackermann_command(
                    command.speed_mps * 0.25, command.curvature_inv_m, geometry
                )
            rate = settings["steering_command_rate_rad_s"] / 120
            steering += np.clip(desired - steering, -rate, rate)
            robot.apply_action(
                ArticulationAction(
                    joint_indices=wheels,
                    joint_velocities=np.asarray(drive.wheel_rates_rad_s),
                )
            )
            robot.apply_action(
                ArticulationAction(joint_indices=steers, joint_positions=steering)
            )
            world.step(render=video and step % 2 == 0)
            if video and step % 2 == 0:
                rgba = overview.get_rgba()
                require(
                    rgba is not None and rgba.shape == (720, 1280, 4),
                    "Overview frame missing",
                )
                encoder.stdin.write(np.ascontiguousarray(rgba[:, :, :3]).tobytes())
        require(success, "Approach tracking timeout")
    finally:
        robot.apply_action(
            ArticulationAction(joint_indices=wheels, joint_velocities=np.zeros(4))
        )
        try:
            for _ in range(120):
                world.step(render=False)
            stop_speed = float(np.linalg.norm(robot.get_linear_velocity()))
            stop_record = {
                "zero_wheel_setpoint_applied": True,
                "settling_time_s": 1.0,
                "measured_speed_mps": stop_speed,
                "stopped": math.isfinite(stop_speed) and stop_speed < 0.012,
            }
            (output / "stop.json").write_text(record_json(stop_record, indent=2) + "\n")
            require(
                stop_record["stopped"], "Robot did not stop after zero wheel command"
            )
        finally:
            (output / "tracking.json").write_text(record_json(samples, indent=2) + "\n")
            if encoder:
                encoder.stdin.close()
                require(encoder.wait(timeout=60) == 0, "Video encoder failed")
    world.render()
    Image.fromarray(overview.get_rgba()[:, :, :3]).save(output / "aligned.png")
    base, q = robot.get_world_pose()
    yaw = yaw_and_tilt(q)[0]
    rear = np.array(
        [base[0] - 0.34 * math.cos(yaw), base[1] - 0.34 * math.sin(yaw), yaw]
    )
    stopped_speed = float(np.linalg.norm(robot.get_linear_velocity()))
    error = float(np.linalg.norm(rear[:2] - path.poses[-1, :2]))
    yaw_error = abs(
        math.atan2(math.sin(yaw - path.poses[-1, 2]), math.cos(yaw - path.poses[-1, 2]))
    )
    require(
        error < 0.02 and yaw_error < 0.03 and stopped_speed < 0.012,
        "Alignment changed or robot did not stop after approach",
    )
    return {
        "observed_goal_reached": True,
        "status": "observed_goal_reached",
        "final_rear_pose": rear.tolist(),
        "position_error_m": error,
        "yaw_error_rad": yaw_error,
        "stopped_speed_mps": stopped_speed,
        "simulated_time_s": world.current_time - started,
    }


def run(app, args, state):
    import yaml
    from approach_evaluation import evaluate_alignment
    from perception_camera import (
        acquire_stationary_snapshot,
        create_perception_camera,
        save_snapshot,
    )
    from scene import add_path_display

    import forklift_core
    from forklift_core.perception.pallet_prior import load_pallet_prior
    from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
    from forklift_core.planning import (
        PlannerConfig,
        Pose2D,
        Rectangle,
        plan_hybrid_astar,
    )
    from forklift_core.planning.observed_approach import observed_approach_goal
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    settings = yaml.safe_load(args.settings.read_text())
    require(settings["physics_hz"] == 120, "Expected 120Hz physics")
    require(
        all(
            isinstance(v, (int, float)) and math.isfinite(v) and v > 0
            for v in settings.values()
        ),
        "Settings must be positive finite numbers",
    )
    state["settings_synthetic"] = settings
    state["settings_sha256"] = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [
            args.settings,
            args.prior,
            args.camera_settings,
            args.detector_settings,
            args.forklift_urdf,
            args.pallet_urdf,
        ]
    }
    core_root = Path(forklift_core.__file__).parent
    state["source_sha256"].update(
        {
            str(path.relative_to(core_root.parent)): hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
            for path in core_root.rglob("*.py")
        }
    )
    world, stage, robot, pallet, overview, scenario = build_scene(
        app, args, settings, state
    )
    camera = create_perception_camera(args.camera_settings)
    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    overview.initialize()
    for _ in range(240):
        world.step(render=True)
    state["phase"] = "acquisition"
    scene, world_from_base = acquire_stationary_snapshot(camera, world, robot)
    state["acquisition"] = save_snapshot(
        args.output / "observation", scene, world_from_base
    )
    prior = load_pallet_prior(args.prior)
    state["phase"] = "detection"
    detector_settings = yaml.safe_load(args.detector_settings.read_text())
    require(
        isinstance(detector_settings, dict)
        and set(detector_settings) == {"max_plane_candidates"},
        "Detector settings require exactly max_plane_candidates",
    )
    detector_params = DetectorParams.derived_for(prior, **detector_settings)
    state["detector_parameters"] = asdict(detector_params)
    detection = detect_pockets(scene, prior, detector_params)
    state["detection"] = asdict(detection)
    (args.output / "detection.json").write_text(
        record_json(state["detection"], indent=2) + "\n"
    )
    print("DETECTION", record_json(state["detection"]), flush=True)
    target = observed_approach_goal(
        detection.observation,
        prior,
        world_from_base,
        transform_stamp_ns=scene.stamp_ns,
        transform_clock_domain=scene.clock_domain,
        now_ns=round(world.current_time * 1e9),
        now_clock_domain="synthetic",
    )
    state["target"] = asdict(target)
    if not target.success:
        state.update(
            status="stopped_no_target",
            phase="stopped",
            success=False,
            wheel_motion_commanded=False,
            reason=target.reason,
            stopped_speed_mps=float(np.linalg.norm(robot.get_linear_velocity())),
        )
        return
    if args.capture_only:
        state.update(
            status="capture_only",
            phase="stopped",
            success=False,
            wheel_motion_commanded=False,
        )
        return
    state["phase"] = "planning"
    base = world_from_base.translation_m
    yaw = math.atan2(world_from_base.rotation[1, 0], world_from_base.rotation[0, 0])
    start = Pose2D(base[0] - 0.34 * math.cos(yaw), base[1] - 0.34 * math.sin(yaw), yaw)
    site = target.pallet_site
    obstacles = [p.rectangle for p in scenario.props] + [
        Rectangle(
            site.x_m,
            site.y_m,
            prior.overall_depth_m,
            prior.overall_width_m,
            site.yaw_rad,
        )
    ]
    path = plan_hybrid_astar(
        start,
        target.approach_rear,
        obstacles,
        SyntheticMissionGeometry().unloaded_footprint,
        scenario.bounds,
        PlannerConfig(
            curvature_limit_inv_m=settings["planner_curvature_inv_m"],
            clearance_m=min(settings["planning_clearance_m"], 0.05),
            max_expansions=30000,
        ),
    )
    state["path"] = path_record(path)
    require(path.success, f"Approach planning failed: {path.status}")
    add_path_display(stage, path, "ObservedApproach", (0.05, 0.45, 1))
    stage.GetRootLayer().Export(str(args.output / "scene.usda"))
    state.update(phase="approach", wheel_motion_commanded=True)
    state.update(
        follow_approach(
            world,
            robot,
            overview,
            path,
            obstacles,
            scenario.bounds,
            settings,
            args.output,
            args.video,
        )
    )
    # Evaluation is downstream of target generation and completed execution.
    truth, truth_q = pallet.get_world_pose()
    truth_yaw = yaw_and_tilt(truth_q)[0]
    state["evaluation"] = evaluate_alignment(
        final_rear_pose=state["final_rear_pose"],
        observed_site=site,
        pallet_position_m=truth,
        pallet_yaw_rad=truth_yaw,
        initial_pallet_xy_m=[scenario.pickup.x_m, scenario.pickup.y_m],
    )
    require(state["evaluation"]["success"], "Alignment evaluation failed")
    state.update(phase="complete", success=True, status="aligned")


def main():
    args = arguments()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    state = {
        "success": False,
        "phase": "startup",
        "seed": args.seed,
        "pallet_target_source": "camera_depth_detection",
        "robot_and_obstacle_source": "simulator_ground_truth",
        "insertion_enabled": False,
        "wheel_motion_commanded": False,
        "source_sha256": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in Path(__file__).parent.glob("*.py")
        },
    }
    app = None
    started = time.monotonic()
    try:
        from isaacsim import SimulationApp

        # Respect the Slurm CPU affinity instead of creating one worker per host
        # CPU. Preserve a traceback if the renderer stalls during first startup.
        cpu_threads = len(os.sched_getaffinity(0))
        state["cpu_threads"] = cpu_threads
        faulthandler.dump_traceback_later(120, repeat=True)
        app = SimulationApp(
            {
                "headless": True,
                "width": 1280,
                "height": 720,
                "renderer": "RaytracedLighting",
                "multi_gpu": False,
                "limit_cpu_threads": cpu_threads,
                "create_new_stage": False,
                "disable_viewport_updates": True,
            }
        )
        faulthandler.cancel_dump_traceback_later()
        run(app, args, state)
    except Exception as exc:
        state.update(success=False, failure_reason=str(exc))
        (args.output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        faulthandler.cancel_dump_traceback_later()
        state["wall_time_s"] = time.monotonic() - started
        (args.output / "result.json").write_text(record_json(state, indent=2) + "\n")
        print(
            "APPROACH_RESULT",
            record_json(
                {
                    k: v
                    for k, v in state.items()
                    if k
                    not in {
                        "path",
                        "source_sha256",
                        "scenario_ground_truth_for_evaluation",
                    }
                }
            ),
            flush=True,
        )
        if app is not None:
            app.close()
    raise SystemExit(0 if state["success"] else 1)


if __name__ == "__main__":
    main()
