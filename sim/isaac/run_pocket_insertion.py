"""Continuous RGB-D pocket observation with fail-stop physical insertion.

Synthetic camera, localization and prop map. Pallet truth is confined to scene
setup and an independent abort/evaluation boundary; it never generates motion.
"""

import argparse
import faulthandler
import hashlib
import math
import os
import subprocess
import time
import traceback
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
from run_perception_approach import arguments, build_scene, follow_approach
from run_transport import record_json, require, yaw_and_tilt


def associate_observation(observation, world_from_base, reference_target):
    """Associate to the initially observed object, never a truth-labelled object.

    This only rejects object switches; it does not resolve lookalike structures.
    The frozen acquisition adapter supplies the paired transform and the servo
    separately validates observation age, clock, provenance and geometry.
    """
    require(reference_target.success, "Association requires an observed target")
    require(
        world_from_base.source_frame == "base_link"
        and world_from_base.target_frame == "world",
        "Association transform frames must be base_link to world",
    )
    if observation.status != "valid":
        return {"matched": False, "reason": "invalid_observation"}
    midpoint = (np.array(observation.left.center_m) + observation.right.center_m) / 2
    world_midpoint = world_from_base.rotation @ midpoint + world_from_base.translation_m
    heading = world_from_base.rotation @ np.array(
        [
            math.cos(observation.insertion_yaw_rad),
            math.sin(observation.insertion_yaw_rad),
            0,
        ]
    )
    error = float(
        np.linalg.norm(world_midpoint - reference_target.front_midpoint_world_m)
    )
    yaw_error = abs(
        math.atan2(
            math.sin(
                math.atan2(heading[1], heading[0])
                - reference_target.pallet_site.yaw_rad
            ),
            math.cos(
                math.atan2(heading[1], heading[0])
                - reference_target.pallet_site.yaw_rad
            ),
        )
    )
    return {
        "matched": error <= 0.03 and yaw_error <= 0.03,
        "position_error_m": error,
        "yaw_error_rad": yaw_error,
    }


def association_gate(observation, association):
    """Preserve missing evidence and reject measured object switches."""
    if observation.status != "valid" or association["matched"]:
        return observation
    return replace(
        observation,
        status="invalid",
        left=None,
        right=None,
        insertion_yaw_rad=None,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason="initial_target_association_lost",
    )


def run(app, args, state):
    import yaml
    from perception_camera import (
        acquire_stationary_snapshot,
        create_perception_camera,
        save_snapshot,
    )
    from scene import add_path_display

    import forklift_core
    from forklift_core.perception.pallet_geometry import load_pallet_geometry
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
    root = Path(__file__).resolve().parents[2]
    tracking_settings_path = root / "config/isaac_pocket_insertion.yaml"
    tracking_settings = yaml.safe_load(tracking_settings_path.read_text())
    state["tracking_settings_synthetic"] = tracking_settings
    pallet_geometry_path = root / "config/pallet_geometry_epal6.yaml"
    pallet_geometry = load_pallet_geometry(pallet_geometry_path)
    files = [
        *Path(__file__).parent.glob("*.py"),
        *Path(forklift_core.__file__).parent.rglob("*.py"),
    ]
    files += [
        args.settings,
        args.camera_settings,
        args.detector_settings,
        args.prior,
        args.forklift_urdf,
        args.pallet_urdf,
        tracking_settings_path,
        pallet_geometry_path,
    ]
    state["sha256"] = {
        str(p.relative_to(root)) if p.is_relative_to(root) else str(p): hashlib.sha256(
            p.read_bytes()
        ).hexdigest()
        for p in files
    }
    world, stage, robot, pallet, overview, scenario = build_scene(
        app, args, settings, state
    )
    camera = create_perception_camera(args.camera_settings)
    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    overview.initialize()
    for _ in range(360):
        world.step(render=True)
    scene, transform = acquire_stationary_snapshot(camera, world, robot)
    save_snapshot(args.output / "initial_observation", scene, transform)
    prior = load_pallet_prior(args.prior)
    detector_settings = yaml.safe_load(args.detector_settings.read_text())
    detector = DetectorParams.derived_for(prior, **detector_settings)
    detected = detect_pockets(scene, prior, detector)
    state["initial_detection"] = asdict(detected)
    geometry = SyntheticMissionGeometry(
        approach_offset_m=tracking_settings["handoff_rear_offset_m"]
    )
    target = observed_approach_goal(
        detected.observation,
        prior,
        transform,
        transform_stamp_ns=scene.stamp_ns,
        transform_clock_domain=scene.clock_domain,
        now_ns=scene.stamp_ns,
        now_clock_domain=scene.clock_domain,
        geometry=geometry,
    )
    state["target"] = asdict(target)
    if not target.success:
        state.update(status="stopped_no_initial_target", success=False)
        return
    if args.capture_only:
        state.update(status="capture_only", phase="stopped", success=False)
        return
    base = transform.translation_m
    yaw = math.atan2(transform.rotation[1, 0], transform.rotation[0, 0])
    start = Pose2D(base[0] - 0.34 * math.cos(yaw), base[1] - 0.34 * math.sin(yaw), yaw)
    site = target.pallet_site
    prop_obstacles = [p.rectangle for p in scenario.props]
    path = plan_hybrid_astar(
        start,
        target.approach_rear,
        prop_obstacles
        + [
            Rectangle(
                site.x_m,
                site.y_m,
                prior.overall_depth_m,
                prior.overall_width_m,
                site.yaw_rad,
            )
        ],
        geometry.unloaded_footprint,
        scenario.bounds,
        PlannerConfig(
            curvature_limit_inv_m=settings["planner_curvature_inv_m"],
            clearance_m=0.05,
            max_expansions=30000,
        ),
    )
    require(path.success, f"Observed handoff planning failed: {path.status}")
    add_path_display(stage, path, "ObservedHandoff", (0.05, 0.45, 1))
    state["approach"] = follow_approach(
        world,
        robot,
        overview,
        path,
        prop_obstacles
        + [
            Rectangle(
                site.x_m,
                site.y_m,
                prior.overall_depth_m,
                prior.overall_width_m,
                site.yaw_rad,
            )
        ],
        scenario.bounds,
        settings,
        args.output,
        args.video,
    )
    state["phase"] = "continuous_observation"
    near_detector = replace(
        detector,
        range_min_m=tracking_settings["detector_range_min_m"],
        median_plane_offset=True,
    )
    state["tracking_detector"] = asdict(near_detector)
    state.update(
        follow_pockets(
            world,
            robot,
            pallet,
            overview,
            camera,
            scenario,
            settings,
            tracking_settings,
            prior,
            near_detector,
            args,
            reference_target=target,
            pallet_geometry=pallet_geometry,
        )
    )


def follow_pockets(
    world,
    robot,
    pallet,
    overview,
    camera,
    scenario,
    settings,
    tracking_settings,
    prior,
    detector,
    args,
    *,
    reference_target,
    pallet_geometry=None,
):
    from insertion_geometry import InsertionGeometry
    from isaacsim.core.utils.types import ArticulationAction
    from perception_camera import acquire_frozen_snapshot, save_snapshot
    from PIL import Image, ImageDraw

    from forklift_core.control import AckermannGeometry, ackermann_command
    from forklift_core.control.pocket_insertion import (
        PocketInsertionConfig,
        PocketInsertionController,
    )
    from forklift_core.perception.pocket_detector import detect_pockets
    from forklift_core.planning import collision_free_pose
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    config = PocketInsertionConfig(**tracking_settings["controller"])
    servo = PocketInsertionController(config)
    roof_settings = tracking_settings.get("roof_tracking")
    handoff = None
    observation_mode = "front_pockets"
    if roof_settings is not None:
        from pocket_tracking_handoff import RoofHandoff

        from forklift_core.perception.roof_tracking import track_roof

        require(pallet_geometry is not None, "Roof tracking requires known geometry")
        handoff = RoofHandoff(**roof_settings)
    geometry = AckermannGeometry(0.64, 0.51, 0.135, 0.45, 8.0)
    guard = InsertionGeometry.from_urdfs(args.forklift_urdf, args.pallet_urdf)
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
    lift = names.index("fork_lift")
    observations = []
    started = world.current_time
    first_drive = None
    loss_pose = None
    loss_time = None
    loss_speed = None
    command = None
    scene = None
    encoder = None
    success = False
    status = "acquiring"
    last_rgb = None
    issued_speed = 0.0
    acceleration = tracking_settings.get("acceleration_mps2", 0.6)
    angular_limit = tracking_settings.get("max_angular_speed_rad_s", 0.08)
    require(math.isfinite(acceleration) and acceleration > 0, "Invalid acceleration")
    require(math.isfinite(angular_limit) and angular_limit > 0, "Invalid angular limit")
    output = args.output / "pocket_frames"
    output.mkdir()
    try:
        if args.video:
            encoder = subprocess.Popen(
                [
                    "ffmpeg",
                    "-y",
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
                    str(args.output / "pocket_tracking.mp4"),
                ],
                stdin=subprocess.PIPE,
            )
        for step in range(round(args.max_tracking_s * 120)):
            base, q = robot.get_world_pose()
            yaw, tilt = yaw_and_tilt(q)
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            linear = np.asarray(robot.get_linear_velocity())
            angular = np.asarray(robot.get_angular_velocity())
            require(
                np.isfinite(linear).all() and np.isfinite(angular).all(),
                "Invalid measured velocity",
            )
            speed = math.copysign(float(np.linalg.norm(linear)), linear[:2] @ forward)
            angular_speed = float(np.linalg.norm(angular))
            require(angular_speed < angular_limit, "Truck angular speed limit")
            require(tilt < 0.1, "Truck tilt limit")
            require(
                collision_free_pose(
                    [*(base[:2] - 0.34 * forward), yaw],
                    [p.rectangle for p in scenario.props],
                    SyntheticMissionGeometry().unloaded_footprint,
                    scenario.bounds,
                ),
                "Mapped obstacle overlap",
            )
            truth, truth_q = pallet.get_world_pose()
            require(
                not guard.forbidden_contacts(
                    base, q, float(robot.get_joint_positions()[lift]), truth, truth_q
                ),
                "Independent fork/pallet clearance guard",
            )
            now_ns = round(world.current_time * 1e9)
            if step % 12 == 0:
                scene, transform = acquire_frozen_snapshot(camera, world, robot)
                injected = (
                    first_drive is not None
                    and args.drop_after_s is not None
                    and world.current_time - first_drive >= args.drop_after_s
                )
                if injected:
                    scene = replace(scene, depth_m=np.full_like(scene.depth_m, np.nan))
                detection = detect_pockets(scene, prior, detector)
                association = associate_observation(
                    detection.observation, transform, reference_target
                )
                control_observation = association_gate(
                    detection.observation, association
                )
                roof_detection = None
                roof_association = None
                selection = None
                if handoff is not None:
                    expected_front = transform.rotation.T @ (
                        np.asarray(reference_target.front_midpoint_world_m)
                        - transform.translation_m
                    )
                    world_yaw = reference_target.pallet_site.yaw_rad
                    expected_axis = transform.rotation.T @ np.array(
                        [math.cos(world_yaw), math.sin(world_yaw), 0.0]
                    )
                    roof_detection = track_roof(
                        scene,
                        pallet_geometry,
                        expected_front,
                        math.atan2(expected_axis[1], expected_axis[0]),
                    )
                    roof_association = associate_observation(
                        roof_detection.observation, transform, reference_target
                    )
                    selection = handoff.select(
                        control_observation,
                        association_gate(roof_detection.observation, roof_association),
                    )
                    control_observation = selection.observation
                    observation_mode = selection.mode
                # Confirmations require physical rest, including rotation; a
                # sideways skid must not look stopped merely because vx is zero.
                rotation_blocks_acquisition = (
                    command is None or command.status != "tracking"
                ) and angular_speed >= 0.012
                command = servo.update(
                    None if rotation_blocks_acquisition else control_observation,
                    now_ns=now_ns,
                    clock_domain="synthetic",
                    measured_speed_mps=speed,
                )
                last_rgb = scene.rgb
                row = {
                    "time_s": world.current_time - started,
                    "detection": asdict(detection),
                    "initial_target_association": association,
                    "roof_detection": None
                    if roof_detection is None
                    else asdict(roof_detection),
                    "roof_initial_target_association": roof_association,
                    "handoff": None if selection is None else asdict(selection),
                    "observation_mode": observation_mode,
                    "control_observation": asdict(control_observation),
                    "command": asdict(command),
                    "world_from_base": asdict(transform),
                    "pallet_ground_truth_for_evaluation": {
                        "position_m": truth.tolist(),
                        "orientation_wxyz": truth_q.tolist(),
                    },
                    "base_from_optical": asdict(scene.base_from_optical),
                    "speed_mps": speed,
                    "angular_speed_rad_s": angular_speed,
                    "rotation_blocks_acquisition": rotation_blocks_acquisition,
                    "injected_depth_loss": injected,
                }
                observations.append(row)
                np.savez_compressed(
                    output / f"{len(observations):05d}.npz", depth_m=scene.depth_m
                )
                if len(observations) == 1 or (
                    loss_time is None and command.status == "lost"
                ):
                    save_snapshot(
                        output / f"capture_{len(observations):05d}", scene, transform
                    )
                if command.status == "lost" and loss_time is None:
                    loss_time = world.current_time
                    loss_pose = np.array(base, copy=True)
                    loss_speed = speed
                status = command.status
            # A simulation-time watchdog also gates held commands between images.
            fresh = (
                scene is not None
                and 0 <= now_ns - scene.stamp_ns <= config.max_observation_age_ns
            )
            if command is not None and not fresh and loss_time is None:
                command = servo.update(
                    None,
                    now_ns=now_ns,
                    clock_domain="synthetic",
                    measured_speed_mps=speed,
                )
                status = command.status
                loss_time = world.current_time
                loss_pose = np.array(base, copy=True)
                loss_speed = speed
            requested = (
                command.speed_mps
                if command is not None and fresh and loss_time is None
                else 0.0
            )
            if requested > 0 and first_drive is None:
                first_drive = world.current_time
            # Limit acceleration only: loss and lower-speed requests brake now,
            # rather than coasting through an invalid observation while ramping.
            requested = min(requested, issued_speed + acceleration / 120)
            issued_speed = requested
            curvature = (
                command.curvature_inv_m if command is not None and requested else 0.0
            )
            drive = ackermann_command(requested, curvature, geometry)
            robot.apply_action(
                ArticulationAction(
                    joint_indices=wheels,
                    joint_velocities=np.array(drive.wheel_rates_rad_s),
                )
            )
            robot.apply_action(
                ArticulationAction(
                    joint_indices=steers, joint_positions=np.array(drive.steering_rad)
                )
            )
            world.step(render=args.video and step % 2 == 0)
            if encoder and step % 2 == 0:
                frame = Image.fromarray(overview.get_rgba()[:, :, :3])
                if last_rgb is not None:
                    frame.paste(Image.fromarray(last_rgb).resize((384, 288)), (880, 12))
                draw = ImageDraw.Draw(frame)
                draw.rectangle((12, 12, 840, 76), fill=(15, 20, 28))
                draw.text(
                    (24, 22),
                    f"RGB-D {observation_mode} | t={world.current_time - started:.2f}s | {status} | command={requested:.3f} m/s",
                    fill="white",
                )
                draw.text(
                    (24, 46),
                    "Fresh depth required. Roof mode = known-model estimate. Loss = STOP.",
                    fill="white",
                )
                encoder.stdin.write(np.asarray(frame).tobytes())
            if (
                command is not None
                and command.status == "complete"
                and loss_time is None
            ):
                success = True
                status = "inserted"
                break
            if loss_time is not None and world.current_time - loss_time >= 1.0:
                status = "stopped_observation_loss"
                break
            if first_drive is None and world.current_time - started >= 3.0:
                status = "stopped_no_tracking_acquisition"
                break
        else:
            status = "tracking_timeout"
    finally:
        robot.apply_action(
            ArticulationAction(joint_indices=wheels, joint_velocities=np.zeros(4))
        )
        try:
            for _ in range(120):
                world.step(render=False)
            stop_speed = float(np.linalg.norm(robot.get_linear_velocity()))
            stop_angular_speed = float(np.linalg.norm(robot.get_angular_velocity()))
            stop_record = {
                "zero_wheel_setpoint_applied": True,
                "settling_time_s": 1.0,
                "measured_speed_mps": stop_speed,
                "angular_speed_rad_s": stop_angular_speed,
                "stopped": math.isfinite(stop_speed)
                and math.isfinite(stop_angular_speed)
                and stop_speed < 0.012
                and stop_angular_speed < 0.012,
            }
            (args.output / "pocket_braking.json").write_text(
                record_json(stop_record, indent=2) + "\n"
            )
            require(stop_record["stopped"], "Physical stop failed")
        finally:
            try:
                (args.output / "pocket_observations.json").write_text(
                    record_json(observations, indent=2) + "\n"
                )
            finally:
                if encoder:
                    encoder.stdin.close()
                    require(
                        encoder.wait(timeout=60) == 0, "Tracking video encoder failed"
                    )
    final_base, final_q = robot.get_world_pose()
    stop_speed = float(np.linalg.norm(robot.get_linear_velocity()))
    require(math.isfinite(stop_speed) and stop_speed < 0.012, "Physical stop failed")
    truth, truth_q = pallet.get_world_pose()
    pallet_axis = np.array(
        [math.cos(yaw_and_tilt(truth_q)[0]), math.sin(yaw_and_tilt(truth_q)[0])]
    )
    front_gap = float((truth[:2] - final_base[:2]) @ pallet_axis - 0.3)
    lateral_error = float(
        (truth[:2] - final_base[:2]) @ np.array([-pallet_axis[1], pallet_axis[0]])
    )
    yaw_error = abs(
        math.atan2(
            math.sin(yaw_and_tilt(final_q)[0] - yaw_and_tilt(truth_q)[0]),
            math.cos(yaw_and_tilt(final_q)[0] - yaw_and_tilt(truth_q)[0]),
        )
    )
    final_contacts = guard.forbidden_contacts(
        final_base, final_q, float(robot.get_joint_positions()[lift]), truth, truth_q
    )
    evaluation_passed = (
        abs(front_gap - config.target_front_x_m) <= 0.02
        and abs(lateral_error) <= 0.02
        and yaw_error <= 0.03
        and not final_contacts
    )
    if success and not evaluation_passed:
        success = False
        status = "insertion_evaluation_failed"
    result = {
        "success": success,
        "status": status,
        "phase": "stopped",
        "tracking_motion_commanded": first_drive is not None,
        "tracking_frames": len(observations),
        "valid_frames": sum(
            r["control_observation"]["status"] == "valid" for r in observations
        ),
        "roof_tracking_qualified": handoff is not None and handoff.qualified,
        "roof_control_frames": sum(
            r["observation_mode"] == "roof_model" for r in observations
        ),
        "stopped_speed_mps": stop_speed,
        "stopped_angular_speed_rad_s": stop_angular_speed,
        "loss_speed_mps": loss_speed,
        "stop_displacement_m": None
        if loss_pose is None
        else float(np.linalg.norm(final_base[:2] - loss_pose[:2])),
        "evaluation_actual_front_x_m": front_gap,
        "evaluation_fork_tip_to_front_m": front_gap - 0.95,
        "evaluation_lateral_error_m": lateral_error,
        "evaluation_yaw_error_rad": yaw_error,
        "evaluation_final_forbidden_contacts": final_contacts,
        "evaluation_inserted_pose_passed": evaluation_passed,
        "tracking_simulated_time_s": world.current_time - started,
        "loss_latched_for_this_trial": loss_time is not None,
        "drop_after_s": args.drop_after_s,
    }
    (args.output / "pocket_stop.json").write_text(record_json(result, indent=2) + "\n")
    return result


def main():
    args = arguments()
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--drop-after-s", type=float)
    parser.add_argument("--max-tracking-s", type=float, default=35.0)
    extra, _ = parser.parse_known_args()
    args.drop_after_s = extra.drop_after_s
    args.max_tracking_s = extra.max_tracking_s
    require(
        math.isfinite(args.max_tracking_s) and args.max_tracking_s > 0,
        "Invalid duration",
    )
    require(
        args.drop_after_s is None
        or math.isfinite(args.drop_after_s)
        and args.drop_after_s >= 0,
        "Invalid loss time",
    )
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    state = {
        "success": False,
        "phase": "startup",
        "seed": args.seed,
        "pallet_target_source": "camera_depth_detection",
        "robot_and_obstacle_source": "simulator_ground_truth",
    }
    app = None
    started = time.monotonic()
    try:
        from isaacsim import SimulationApp

        faulthandler.dump_traceback_later(120, repeat=True)
        app = SimulationApp(
            {
                "headless": True,
                "width": 1280,
                "height": 720,
                "renderer": "RaytracedLighting",
                "multi_gpu": False,
                "limit_cpu_threads": len(os.sched_getaffinity(0)),
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
            "POCKET_RESULT",
            record_json(
                {
                    k: v
                    for k, v in state.items()
                    if k not in {"sha256", "scenario_ground_truth_for_evaluation"}
                }
            ),
            flush=True,
        )
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
