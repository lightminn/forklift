"""Physical Hybrid A* pallet transport using official warehouse props.

Run with Isaac Sim's Python after installing forklift-core in that environment.
All pose feedback is simulator ground truth; configuration is synthetic. Camera
output is 60fps by default, and the floor destination is marked by a green ring.
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
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

EXIT_CLEARANCE_M = 0.08


def load_perception_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CAMERA_CALIBRATION = load_perception_module(
    "run_transport_camera_calibration",
    Path(__file__).with_name("camera_calibration.py"),
)
MISSION_VIEWS = load_perception_module(
    "run_transport_mission_views", Path(__file__).with_name("mission_views.py")
)
G2 = load_perception_module(
    "run_transport_g2_records", Path(__file__).with_name("g2_records.py")
)


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


def quarter_vector(value: str) -> tuple[float, float, float]:
    """Parse one finite world-space camera point."""
    try:
        parts = tuple(float(part) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("quarter point needs X,Y,Z floats") from exc
    if len(parts) != 3 or not all(math.isfinite(part) for part in parts):
        raise argparse.ArgumentTypeError(
            "quarter point needs three finite X,Y,Z floats"
        )
    return parts


def quarter_focal(value: str) -> float:
    """Parse a positive finite focal length for Camera.set_focal_length."""
    try:
        focal = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("quarter focal must be positive") from exc
    if not math.isfinite(focal) or focal <= 0:
        raise argparse.ArgumentTypeError("quarter focal must be positive and finite")
    return focal


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", required=True)
    parser.add_argument("--pallet-urdf", type=Path, required=True)
    parser.add_argument("--pallet-geometry", type=Path, required=True)
    # Required: the base scene's truck and this URDF are checked against each
    # other, so neither may be picked silently.
    parser.add_argument("--forklift-urdf", type=Path, required=True)
    parser.add_argument("--settings", type=Path, required=True)
    parser.add_argument(
        "--insertion-reserve-m",
        type=float,
        default=0.046,
        help="Insertion reserve behind the carriage limit (ADR 0004 D3 policy "
        "0.046). Other values are for diagnostic sweeps and are recorded.",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--obstacles", type=int, default=4)
    parser.add_argument(
        "--layout",
        choices=("bay", "factory"),
        default="bay",
        help="bay: the original 7.7 x 4.8 m transport bay (every recorded run). "
        "factory: the same bay inside the full south hall, filled with stored "
        "pallets and clutter, delivering to the shipping yard.",
    )
    parser.add_argument(
        "--factory-layout",
        type=Path,
        default=Path(__file__).resolve().parent.parent.parent
        / "config/factory_south_hall.yaml",
    )
    parser.add_argument("--fps", type=camera_hz, default=60)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--extra-views", default="", metavar="VIEWS")
    parser.add_argument("--quarter-eye", type=quarter_vector, metavar="X,Y,Z")
    parser.add_argument("--quarter-target", type=quarter_vector, metavar="X,Y,Z")
    parser.add_argument("--quarter-focal", type=quarter_focal, default=2.5, metavar="F")
    parser.add_argument("--max-sim-seconds", type=float, default=300)
    parser.add_argument(
        "--asset-root",
        default=(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
            "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props"
        ),
    )
    parser.add_argument("--use-perception", action="store_true")
    parser.add_argument(
        "--camera-inset",
        action="store_true",
        help="Draw the perception camera's live picture at the top left of the "
        "video, with the detected pockets and the turn needed to align. "
        "Requires --video and --use-perception.",
    )
    parser.add_argument(
        "--robot-camera",
        action="store_true",
        help="Also record the perception camera as camera_rgb.mp4 and "
        "camera_depth.mp4 (detected pockets drawn in), frame for frame with the "
        "overview, and write video_frames.json. Requires --video and "
        "--use-perception.",
    )
    parser.add_argument(
        "--record-slam",
        action="store_true",
        help="Record 2D LiDAR scans, wheel joints and ground truth as "
        "slam_log.npz/meta.json in the run_slam_drive.py format, for a "
        "slam_toolbox replay of the mission.",
    )
    parser.add_argument(
        "--lidar",
        type=Path,
        default=Path(__file__).resolve().parent.parent.parent
        / "config/isaac_slam_lidar.yaml",
    )
    parser.add_argument(
        "--return-home",
        action="store_true",
        help="After unloading, drive back to the rear-axle pose the mission "
        "started from. The delivered pallet becomes an obstacle for that leg.",
    )
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
            "(-1.20, 0.30, 0), (-0.10, -0.60, 0), (-1.50, -0.60, 0), (-2.00, -0.30, 0), "
            "(0.00, 2.10, -0.25), (0.40, 1.20, 0), (-0.60, 1.80, -0.25)."
        ),
    )
    parser.add_argument(
        "--perception-camera-axes",
        choices=("world", "usd", "ros"),
        default="ros",
    )
    parser.add_argument("--perception-max-attempts", type=int, default=200)
    # Online SLAM closed loop (docs/plans/2026-10-04-online-slam-closed-loop.md).
    parser.add_argument(
        "--slam-feedback",
        type=Path,
        default=None,
        metavar="SOCKET",
        help="Unix socket of forklift_ros isaac_slam_bridge: control, planning "
        "starts and the perception world transform use the SLAM estimate and "
        "wheel odometry; ground truth only checks, evaluates and renders. "
        "Requires --record-slam (the LiDAR) and --use-perception.",
    )
    parser.add_argument("--slam-noise-seed", type=int, default=None)
    parser.add_argument("--slam-reply-timeout", type=float, default=90.0)
    parser.add_argument(
        "--perception-mount",
        choices=("legacy", "carriage_low"),
        default="legacy",
        help="legacy: base (0.75, 0, 0.50), tilt 0 (every recorded run). "
        "carriage_low: on fork_carriage at base (0.559, 0, 0.27), tilt 0.10 rad, "
        "provisional chassis only, captures only at lift 0 "
        "(docs/plans/2026-10-03-carriage-mount-adoption.md).",
    )
    parser.add_argument(
        "--depth-quantize-mm",
        type=int,
        choices=(0, 1),
        default=0,
        help="1: round the detector's depth to whole millimetres, as a z16 "
        "depth stream reports it; saved depth stays raw (adoption plan, B1c).",
    )
    parser.add_argument(
        "--planning-target",
        choices=G2.PLANNING_TARGETS,
        default="perception",
        help="oracle_nominal: after a valid detection, plan to the scenario's "
        "nominal pickup instead of the estimate (G2a control, "
        "docs/plans/2026-10-01-g2-rerun.md). Detection and re-observation are "
        "unchanged.",
    )
    parser.add_argument(
        "--repeat-captures",
        type=int,
        default=0,
        help="Diagnostic run (G2r): at observation attempt --repeat-at-attempt, "
        "hold the wheels and capture this many more times, then stop.",
    )
    parser.add_argument("--repeat-at-attempt", type=int, default=None)
    parser.add_argument(
        "--tracker-profile",
        choices=("current", "20260921"),
        default="current",
        help="20260921: the G2 baseline's tracking rules -- gear-change cusps and "
        "the observation stop at the goal tolerances (8 mm), no overshoot "
        "allowance. For the G2b diagnosis; current keeps today's rules.",
    )
    args, unknown = parser.parse_known_args()
    try:
        args.extra_views = MISSION_VIEWS.parse_extra_views(
            args.extra_views, args.video, args.use_perception
        )
    except ValueError as exc:
        parser.error(str(exc))
    if args.quarter_eye is not None and args.quarter_target is not None:
        if args.quarter_eye == args.quarter_target:
            parser.error("quarter eye and target must differ")
    # Explicit waypoints reproduce earlier diagnostic runs exactly; only the
    # default list gets run-time viewpoints appended after it.
    # The placement zone is the synthetic bay's; the factory layout has its own.
    args.runtime_viewpoints = (
        args.observation_waypoints is None and args.layout != "factory"
    )
    if args.observation_waypoints is None:
        # (-1.20, 0.30) moved ahead of (-0.10, -0.60): both plan equally well
        # for every seed that can reach either, but seed 3 only detects the
        # pallet from (-1.20, 0.30) -- (-0.10, -0.60) occludes the right
        # pocket there. No seed's chosen candidate changes except seed 3's
        # (confirmed 2026-09-19: re-running the full reachability sweep with
        # this order picks the same candidate as before for every other seed).
        # The last three (G4, 2026-10-02) are tried only after all five above
        # fail to plan or detect, so seeds served earlier never reach them.
        # Chosen by tools/observation_candidate_design.py on design seeds
        # 200-399 for pallets high in the bay, hidden from the far candidates
        # (docs/plans/2026-10-02-g4-observation-candidates.md).
        args.observation_waypoints = [
            [-0.10, 0.90, 0.0],
            [-1.20, 0.30, 0.0],
            [-0.10, -0.60, 0.0],
            [-1.50, -0.60, 0.0],
            [-2.00, -0.30, 0.0],
            [0.00, 2.10, -0.25],
            [0.40, 1.20, 0.0],
            [-0.60, 1.80, -0.25],
        ]
    if not args.observation_waypoints:
        parser.error("--observation-waypoints requires at least one candidate")
    if args.slam_feedback is not None:
        if not (args.record_slam and args.use_perception):
            parser.error("--slam-feedback needs --record-slam and --use-perception")
        if args.planning_target == "oracle_nominal":
            parser.error("--slam-feedback forbids --planning-target oracle_nominal")
    if args.slam_noise_seed is not None and args.slam_feedback is None:
        parser.error("--slam-noise-seed requires --slam-feedback")
    if args.perception_mount == "carriage_low":
        if not args.use_perception:
            parser.error("--perception-mount carriage_low needs --use-perception")
        if args.perception_camera_axes != "ros":
            parser.error("--perception-mount carriage_low needs ros camera axes")
        if "dls08_provisional" not in str(args.forklift_urdf):
            parser.error(
                "--perception-mount carriage_low is defined for dls08_provisional only"
            )
    if args.use_perception:
        if args.pallet_prior is None:
            parser.error("--pallet-prior is required with --use-perception")
        from forklift_core.perception.pallet_prior import load_pallet_prior

        try:
            args.pallet_prior_loaded = load_pallet_prior(args.pallet_prior)
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
    if args.camera_inset and not (args.video and args.use_perception):
        parser.error("--camera-inset requires --video and --use-perception")
    if args.robot_camera and not (args.video and args.use_perception):
        parser.error("--robot-camera requires --video and --use-perception")
    if args.planning_target != "perception" and not args.use_perception:
        parser.error("--planning-target requires --use-perception")
    if args.repeat_captures < 0:
        parser.error("--repeat-captures must not be negative")
    if args.repeat_captures:
        if not args.use_perception or args.repeat_at_attempt is None:
            parser.error(
                "--repeat-captures requires --use-perception and --repeat-at-attempt"
            )
        if args.repeat_at_attempt < 1:
            parser.error("--repeat-at-attempt counts from 1")
        if args.planning_target != "perception":
            parser.error("a repeat-capture run stops before planning")
    elif args.repeat_at_attempt is not None:
        parser.error("--repeat-at-attempt requires --repeat-captures")
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


# Colour range of the robot-camera depth video; display only, not a sensor limit.
DEPTH_VIDEO_RANGE_M = (0.3, 10.0)
FONT_PATH = next(
    (
        str(path)
        for path in (
            Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
            Path("/usr/share/fonts/truetype/nanum/NanumSquareB.ttf"),
        )
        if path.exists()
    ),
    None,
)


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


def record_robot_camera_frame(
    camera,
    calibration,
    encoders: dict,
    frames: list,
    video_frames_module,
    inset,
    *,
    state: dict,
    stamp: float,
    phase: str,
    pose,
    estimate,
    fork_tip_x_m: float,
    overview,
    bounds,
) -> None:
    """Write one robot-camera RGB and depth frame, pockets drawn while on the floor."""
    width, height = calibration.width, calibration.height
    if not frames:
        # Floor points to overview pixels, for placing the overview offline.
        floor = np.array(
            [
                [bounds.x_min_m, bounds.y_min_m, 0.0],
                [bounds.x_max_m, bounds.y_min_m, 0.0],
                [bounds.x_max_m, bounds.y_max_m, 0.0],
                [bounds.x_min_m, bounds.y_max_m, 0.0],
                [
                    (bounds.x_min_m + bounds.x_max_m) / 2,
                    (bounds.y_min_m + bounds.y_max_m) / 2,
                    0.0,
                ],
            ]
        )
        state["overview_floor_points"] = {
            "world_m": floor.tolist(),
            "pixels_uv": np.asarray(
                overview.get_image_coords_from_world_points(floor)
            ).tolist(),
        }
    rgb, depth = camera.get_rgba(), camera.get_depth()
    ready = (
        rgb is not None
        and np.shape(rgb)[:2] == (height, width)
        and depth is not None
        and np.shape(depth)[:2] == (height, width)
    )
    if ready:
        colour = np.asarray(rgb)[:, :, :3]
        if estimate is not None and phase in inset.PALLET_ON_FLOOR_PHASES:
            base, q = pose
            current = (float(base[0]), float(base[1]), yaw_and_tilt(q)[0])
            gap = inset.pallet_front_distance_m(estimate, current) - fork_tip_x_m
            colour = video_frames_module.annotate_detection(
                colour,
                inset.projected_openings(estimate, current),
                f"팔레트 {max(gap, 0.0):.2f} m",
                font_path=FONT_PATH,
            )
        shaded = video_frames_module.colorize_depth(
            np.asarray(depth).reshape(height, width), *DEPTH_VIDEO_RANGE_M
        )
    else:
        colour = shaded = np.zeros((height, width, 3), np.uint8)
    encoders["rgb"].stdin.write(np.ascontiguousarray(colour, np.uint8).tobytes())
    encoders["depth"].stdin.write(np.ascontiguousarray(shaded, np.uint8).tobytes())
    frames.append({"time_s": stamp, "phase": phase, "camera_ready": bool(ready)})


def write_slam_record(args, state, scenario, factory, log, lidar_config) -> None:
    """slam_log.npz + meta.json in the run_slam_drive.py format, plus the mission."""
    from forklift_core.sensors.lidar import PlanarScanPattern

    pattern = PlanarScanPattern(
        lidar_config["beam_count"],
        float(lidar_config["range_min_m"]),
        float(lidar_config["range_max_m"]),
    )
    np.savez_compressed(
        args.output / "slam_log.npz", **{k: np.asarray(v) for k, v in log.items()}
    )
    obstacles = [
        {**asdict(prop.rectangle), "height_m": prop.asset.height_m, "base_m": 0.0}
        for prop in scenario.props
    ]
    if factory is not None:
        obstacles += [
            {
                **asdict(load.rectangle),
                "height_m": load.asset.height_m,
                "base_m": load.base_height_m,
            }
            for load in factory.loads
        ]
    meta = {
        "format": "forklift_slam_log_v1",
        "clock": "Isaac simulation time since the first recorded step, seconds",
        "source": "synthetic Isaac Sim 5.1 transport mission, not a physical sensor",
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
            "mount_xyz_m": list(map(float, lidar_config["mount_xyz_m"])),
            "mount_yaw_rad": float(lidar_config["mount_yaw_rad"]),
        },
        "wheel_order": ["front_left", "front_right", "rear_left", "rear_right"],
        "wheel_rate_sign": "positive rolls the truck forward (drive command sign)",
        "steering_order": ["front_left", "front_right"],
        "odometry_geometry": {
            "wheelbase_m": args.drive_geometry.wheelbase_m,
            "track_m": args.drive_geometry.track_m,
            "wheel_radius_m": args.drive_geometry.wheel_radius_m,
            "rear_axle_x_in_base_m": args.rear_axle_offset_m,
            "source": f"{args.forklift_urdf} joint origins",
        },
        "seed": args.seed,
        "layout_version": "bay" if factory is None else factory.layout_version,
        "obstacles": obstacles,
        "hall": asdict(scenario.bounds),
        "mission": {
            "start_rear": asdict(scenario.start_rear),
            "pickup_truth": asdict(scenario.pickup),
            "destination": asdict(scenario.destination),
            "pickup_estimate": state.get("perception", {}).get(
                "perception_pickup_estimate_m"
            ),
        },
    }
    (args.output / "meta.json").write_text(record_json(meta, indent=2) + "\n")


def verify_camera_intrinsics(camera, raw_sdk_calibration, state: dict) -> None:
    """G1a: record getter provenance and refuse inconsistent camera settings.

    Isaac 5.1 computes K from prim focal length/aperture and cached resolution.
    Because we set those properties from the nominal K, this checks setting
    propagation/render-product resolution only, NOT independent calibration.
    The independent render experiment is verify_perception_camera.py (G1b).
    raw_sdk_calibration is nominal K in the SDK's half-integer-centre convention.
    Compare raw getter K to raw nominal K and normalized K to integer-index K;
    both comparisons retain the original G1 tolerances.
    """
    adapter = load_perception_module(
        "g1a_perception_adapter", Path(__file__).with_name("perception_adapter.py")
    )
    nominal = adapter.normalize_isaac_intrinsics(raw_sdk_calibration)
    limits = CAMERA_CALIBRATION.G1_LIMITS
    record = {
        "status": "FAIL",
        "intrinsics_source": "Camera.get_intrinsics_matrix() (prim focal/aperture + SDK cached resolution)",
        "resolution_source": 'omni.usd.get_context().get_stage().GetPrimAtPath(Camera.get_render_product_path()).GetAttribute("resolution").Get()',
        "cached_resolution_source": "Camera.get_resolution() (SDK cache)",
        "limits": {
            key: limits[key] for key in ("focal_relative", "principal_point_px")
        },
    }
    # Record the nominal coordinate system even when getter/readback fails.
    for name, calibration in (
        ("raw_sdk", nominal.raw_sdk),
        ("integer_index", nominal.integer_index),
    ):
        record[name] = {
            "status": "FAIL",
            "coordinate_convention": nominal.to_record()[name]["coordinate_convention"],
            "nominal": {
                key: getattr(calibration, key)
                for key in ("fx", "fy", "cx", "cy", "width", "height")
            },
        }
    state["perception_camera_intrinsics"] = record
    try:
        import omni.usd

        record["render_product_path"] = str(camera.get_render_product_path())
        measured = adapter.read_isaac_intrinsics(camera)
        measured_record = measured.to_record()
        record["normalization"] = measured_record["normalization"]
        for name in ("raw_sdk", "integer_index"):
            record[name].update(measured_record[name])
        cached_resolution = (measured.raw_sdk.width, measured.raw_sdk.height)
        record["cached_resolution"] = list(cached_resolution)
        product = (
            omni.usd.get_context()
            .get_stage()
            .GetPrimAtPath(record["render_product_path"])
        )
        require(product.IsValid(), "Camera render product prim is missing")
        # Isaac 5.1 Camera.get_resolution() returns self._resolution. Read the
        # composed USD product independently before any capture or motion.
        resolution = tuple(product.GetAttribute("resolution").Get())
        record["resolution"] = list(resolution)
        require(
            resolution == (nominal.raw_sdk.width, nominal.raw_sdk.height)
            and cached_resolution == resolution,
            "Perception camera intrinsics mismatch (G1a)",
        )
        matches = []
        for name in ("raw_sdk", "integer_index"):
            actual, calibration = getattr(measured, name), getattr(nominal, name)
            record[name]["errors"] = {
                "fx_relative": abs(actual.fx / calibration.fx - 1),
                "fy_relative": abs(actual.fy / calibration.fy - 1),
                "cx_px": abs(actual.cx - calibration.cx),
                "cy_px": abs(actual.cy - calibration.cy),
            }
            # Direct authored bounds preserve the inclusive 0.1 px limit;
            # subtracting 320 from 320.1 gives 0.10000000000002274.
            matches.append(
                calibration.fx * (1 - limits["focal_relative"])
                <= actual.fx
                <= calibration.fx * (1 + limits["focal_relative"])
                and calibration.fy * (1 - limits["focal_relative"])
                <= actual.fy
                <= calibration.fy * (1 + limits["focal_relative"])
                and calibration.cx - limits["principal_point_px"]
                <= actual.cx
                <= calibration.cx + limits["principal_point_px"]
                and calibration.cy - limits["principal_point_px"]
                <= actual.cy
                <= calibration.cy + limits["principal_point_px"]
            )
            record[name]["status"] = "PASS" if matches[-1] else "FAIL"
        require(all(matches), "Perception camera intrinsics mismatch (G1a)")
    except Exception as exc:
        record["reason"] = str(exc)
        require(False, f"Perception camera intrinsics check failed: {exc}")
    record["status"] = "PASS"


def source_sha256(repo_root: Path, core_root: Path) -> dict[str, str]:
    """Hash the full installed core and Isaac source trees plus the shared rig.

    Package keys remain forklift_core/... even for a wheel outside this checkout;
    repository tools use their actual repository-relative paths, never basenames.
    Full trees deliberately include future transitive Python dependencies.
    """
    repo_root, core_root = repo_root.resolve(), core_root.resolve()
    files = sorted(core_root.rglob("*.py"))
    files += sorted((repo_root / "sim/isaac").rglob("*.py"))
    files.append(repo_root / "tools/scene_rig.py")
    return {
        (
            (Path("forklift_core") / path.relative_to(core_root)).as_posix()
            if path.is_relative_to(core_root)
            else path.relative_to(repo_root).as_posix()
        ): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files
    }


def path_record(path) -> dict:
    return {
        "success": path.success,
        "status": path.status,
        "poses": path.poses.tolist(),
        "directions": path.directions.tolist(),
        "curvatures_inv_m": path.curvatures_inv_m.tolist(),
        "length_m": path.length_m,
        "expanded_nodes": path.expanded_nodes,
        "analytic_expansion_interval": getattr(
            path, "analytic_expansion_interval", None
        ),
        "search_attempts": [
            list(entry) for entry in getattr(path, "search_attempts", ())
        ],
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
        add_factory_items,
        add_path_display,
        add_props,
        configure_drives,
        create_pallet,
        hide_overhead,
        read_catalogue,
    )

    import forklift_core
    from forklift_core.control import (
        RearAxlePathTracker,
        TrackerConfig,
        ackermann_command,
    )
    from forklift_core.planning import (
        Footprint,
        Rectangle,
        collision_free_pose,
    )
    from forklift_core.control.rollout import bicycle_rollout
    from forklift_core.planning import Pose2D as PlanningPose
    from forklift_core.planning.pallet_mission import (
        SyntheticMissionGeometry,
        make_scenario,
        make_transport_planner_config,
        plan_transport,
        final_straight_prefix,
        straight_from_pose,
        plan_return_leg,
        plan_transport_leg,
    )

    if args.use_perception:
        from forklift_core.perception.pocket_detector import (
            DetectorParams,
            detect_pockets,
        )
        from forklift_core.planning import Pose2D
        from forklift_core.planning.observation_viewpoints import runtime_viewpoints
        from forklift_core.planning.pallet_mission import plan_observation_leg

        root = Path(__file__).resolve().parents[2]
        adapter = load_perception_module(
            "run_transport_perception_adapter", root / "sim/isaac/perception_adapter.py"
        )
        rig = load_perception_module(
            "run_transport_scene_rig", root / "tools/scene_rig.py"
        )
        if args.camera_inset or args.robot_camera:
            inset = load_perception_module(
                "run_transport_camera_inset", root / "sim/isaac/camera_inset.py"
            )
            inset_font = next(
                (
                    str(path)
                    for path in (
                        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
                        Path("/usr/share/fonts/truetype/nanum/NanumSquareB.ttf"),
                    )
                    if path.exists()
                ),
                None,
            )
            state["camera_inset"] = {
                "source": "perception camera live RGB",
                "outline": "one detection carried forward by simulator pose, "
                "not continuous tracking",
                "font": inset_font,
            }

    state["source_sha256"] = source_sha256(
        Path(__file__).resolve().parents[2], Path(forklift_core.__file__).parent
    )
    state["phase"] = "scene"
    enable_extension("isaacsim.asset.importer.urdf")
    require(
        omni.usd.get_context().open_stage(args.base_scene), "Cannot open base scene"
    )
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    from chassis_contract import (
        chassis_record,
        require_scene_matches_model,
        stage_chassis,
    )

    # The truck's physics comes from the scene, its commands from the URDF.
    scene_chassis = stage_chassis(stage, "/World/Forklift")
    state["scene_chassis"] = chassis_record(scene_chassis)
    require_scene_matches_model(scene_chassis, args.forklift_urdf)
    state["scene_chassis_matches_urdf"] = True
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    catalogue, offsets = read_catalogue(stage, app, args.asset_root)
    geometry = SyntheticMissionGeometry(
        unloaded_footprint=Footprint(args.axle_to_fork_tip_m, 0.17, 0.36),
        pallet_depth_m=args.pallet_geometry_loaded.overall_depth_m,
        pallet_width_m=args.pallet_geometry_loaded.overall_width_m,
        axle_to_fork_tip_m=args.axle_to_fork_tip_m,
        carriage_limit_m=args.carriage_limit_m,
        insertion_reserve_m=args.insertion_reserve_m,
    )
    state["insertion_reserve_m"] = args.insertion_reserve_m
    planner_config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=30000,
    )
    mission_stages = ["approach", "insert", "extract", "transport", "withdraw"]
    if args.return_home:
        mission_stages.append("return_home")
    state["mission_stages"] = list(mission_stages)
    state["planner_config"] = asdict(planner_config)
    state["approach_clearance_m"] = min(
        planner_config.clearance_m, geometry.approach_gap_m / 2
    )
    insertion_geometry = InsertionGeometry.from_urdfs(
        args.forklift_urdf, args.pallet_urdf
    )
    state["pocket_clearance_margin_m"] = 0.002
    state["pocket_geometry_checks"] = 0
    state["forbidden_pocket_contacts"] = []
    # A loaded gear cusp reached on the path but out of heading: stop and plan
    # the rest of the transport again from the measured pose, at the same
    # clearance, at most this many times (third- and fourth-evaluation seeds
    # 3003 and 4007; docs/plans/2026-10-03-transport-stage-fixes.md).
    state["cusp_replans"] = []
    state["stall_replans"] = []
    slam_stall_ticks = 0
    max_cusp_replans = 2
    # Stopped = zero command, planar speed and yaw rate under these for this
    # many consecutive ticks (0.1 s), not the forward speed alone (Codex review).
    cusp_stop_ticks, cusp_stop_needed = 0, 12
    cusp_stop_yaw_rate_radps = 0.02
    # None keeps every planner call exactly as in the original bay runs.
    pickup_bounds = travel_config = factory = None
    if args.layout == "factory":
        import factory_assets

        from forklift_core.planning.factory_layout import (
            load_factory_layout,
            make_factory_scenario,
        )

        factory_catalogue, factory_offsets = read_catalogue(
            stage,
            app,
            args.asset_root,
            filenames=factory_assets.ALL,
            prim_prefix="/World/FactoryCatalogue_",
        )
        offsets.update(factory_offsets)
        factory = make_factory_scenario(
            args.seed,
            load_factory_layout(args.factory_layout),
            catalogue,
            factory_assets.factory_assets(
                dict(zip(factory_assets.ALL, factory_catalogue, strict=True))
            ),
            args.obstacles,
            geometry=geometry,
        )
        scenario = factory.transport
        pickup_bounds = factory.pickup_bounds
        # The long legs cross the hall; only they get the obstacle heuristic.
        travel_config = replace(planner_config, obstacle_heuristic_resolution_m=0.25)
        state["travel_planner_config"] = asdict(travel_config)
        state["factory"] = {
            "layout": str(args.factory_layout),
            "layout_version": factory.layout_version,
            "layout_sha256": hashlib.sha256(
                args.factory_layout.read_bytes()
            ).hexdigest(),
            "work_items": len(factory.work_items),
            "loads": len(factory.loads),
            "pickup_bounds": asdict(pickup_bounds),
            "asset_dimensions_m": {
                name: [spec.length_m, spec.width_m, spec.height_m]
                for name, spec in zip(
                    factory_assets.ALL, factory_catalogue, strict=True
                )
            },
        }
    else:
        scenario = make_scenario(
            args.seed, catalogue, args.obstacles, geometry=geometry
        )
    # The pose the mission began at, or None when no return leg was requested.
    return_to_pose = scenario.start_rear if args.return_home else None
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
    if factory is None:
        state["props"] = add_props(stage, app, scenario.props, offsets)
    else:
        bay_props = scenario.props[: len(scenario.props) - len(factory.work_items)]
        state["props"] = add_props(stage, app, bay_props, offsets)
        state["factory_items"] = add_factory_items(
            stage, app, factory.work_items, factory.loads, offsets
        )
        # The overview has to see the whole hall from above the roof line.
        state["hidden_overhead_prims"] = len(hide_overhead(stage))
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
    eye = center + np.array([0.0, 0.0, 6.0 if factory is None else 26.0])
    look = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0, 1, 0))
    q = look.GetInverse().ExtractRotationQuat()
    camera.set_world_pose(
        position=eye,
        orientation=np.array([q.GetReal(), *q.GetImaginary()]),
        camera_axes="usd",
    )
    camera.set_focal_length(1.1 if factory is None else 0.85)
    state["camera"] = {
        "type": "fixed_global_overview",
        "eye_m": eye.tolist(),
        "target_m": center.tolist(),
        "fps": args.fps,
        "resolution": [1280, 720],
    }
    extra_cameras = {}
    if args.extra_views:
        state["extra_views"] = {}
    if "quarter" in args.extra_views:
        default_quarter_eye = center + np.array(
            [
                -0.55 * (b.x_max_m - b.x_min_m),
                -0.55 * (b.y_max_m - b.y_min_m),
                3.2,
            ]
        )
        quarter_eye = np.asarray(
            args.quarter_eye if args.quarter_eye is not None else default_quarter_eye
        )
        quarter_target = np.asarray(
            args.quarter_target if args.quarter_target is not None else center
        )
        if np.array_equal(quarter_eye, quarter_target):
            raise ValueError("quarter eye and target must differ")
        quarter = Camera(
            prim_path="/World/QuarterCamera", frequency=-1, resolution=(1280, 720)
        )
        look = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*quarter_eye), Gf.Vec3d(*quarter_target), Gf.Vec3d(0, 0, 1)
        )
        quat = look.GetInverse().ExtractRotationQuat()
        quarter.set_world_pose(
            position=quarter_eye,
            orientation=np.array([quat.GetReal(), *quat.GetImaginary()]),
            camera_axes="usd",
        )
        quarter.set_focal_length(args.quarter_focal)
        extra_cameras["quarter"] = quarter
        state["extra_views"]["quarter"] = {
            "eye_m": quarter_eye.tolist(),
            "target_m": quarter_target.tolist(),
            "focal_length_mm": args.quarter_focal,
        }
    if "chase" in args.extra_views:
        chase = Camera(
            prim_path="/World/ChaseCamera", frequency=-1, resolution=(1280, 720)
        )
        chase.set_focal_length(3.0)
        extra_cameras["chase"] = chase
        state["extra_views"]["chase"] = {"focal_length_mm": 3.0}
    if args.use_perception:
        if args.perception_mount == "legacy":
            perception_mount = adapter.default_base_from_optical()
            mount_parent = "/World/Forklift/base_link"
            mount_xyzw = rig.OPTICAL_QUATERNION_XYZW
        else:
            # On the carriage, so it rises with the lift like the real camera;
            # the base<-optical transform below holds only at lift 0.
            perception_mount = adapter.mount_base_from_optical(args.perception_mount)
            mount_parent = "/World/Forklift/fork_carriage"
            mount_xyzw = adapter.quaternion_xyzw(perception_mount.rotation)
        state["perception_mount"] = {
            "name": args.perception_mount,
            "parent": mount_parent,
            "translation_m": np.asarray(perception_mount.translation_m).tolist(),
            "rotation": np.asarray(perception_mount.rotation).tolist(),
            "depth_quantize_mm": args.depth_quantize_mm,
        }
        perception_calibration = rig.intrinsics()
        perception_camera = Camera(
            prim_path=mount_parent + "/PerceptionCamera",
            frequency=-1,
            resolution=(perception_calibration.width, perception_calibration.height),
        )
        perception_camera.set_local_pose(
            translation=np.asarray(perception_mount.translation_m),
            orientation=np.asarray(adapter.xyzw_to_wxyz(mount_xyzw)),
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

    def body_velocities() -> dict:
        # Saved scene state could survive the reset; compare runs on values.
        return {
            "linear_mps": robot.get_linear_velocity(),
            "angular_radps": robot.get_angular_velocity(),
            "joint_names": list(robot.dof_names),
            "joint_radps_or_mps": robot.get_joint_velocities(),
            "joint_max_abs": float(np.max(np.abs(robot.get_joint_velocities()))),
        }

    state["velocities_after_reset"] = body_velocities()
    camera.initialize()
    import carb

    # Every run records the render mode it used (G2 rerun plan, 2026-10-01).
    state["render_mode"] = {
        "before_first_capture": carb.settings.get_settings().get("/rtx/rendermode")
    }
    for extra_camera in extra_cameras.values():
        extra_camera.initialize()
    if args.use_perception:
        perception_camera.initialize()
        verify_camera_intrinsics(perception_camera, perception_calibration, state)
        # Capture needs axial depth as well as RGBA (see determinism_probe.py).
        perception_camera.add_distance_to_image_plane_to_frame()
        if "perception" in args.extra_views:
            perception_display = Camera(
                prim_path=mount_parent + "/PerceptionDisplayCamera",
                frequency=-1,
                resolution=(
                    perception_calibration.width,
                    perception_calibration.height,
                ),
            )
            perception_display.set_local_pose(
                translation=np.asarray(perception_mount.translation_m),
                orientation=np.asarray(adapter.xyzw_to_wxyz(mount_xyzw)),
                camera_axes=args.perception_camera_axes,
            )
            perception_display.set_projection_mode("perspective")
            perception_display.set_lens_distortion_model("pinhole")
            perception_display.set_focal_length(1.0)
            perception_display.set_horizontal_aperture(
                perception_calibration.width / perception_calibration.fx,
                maintain_square_pixels=True,
            )
            perception_display.initialize()
            perception_display.add_distance_to_image_plane_to_frame()
            _, display_far_m = perception_display.get_clipping_range()
            perception_display.set_clipping_range(near_distance=0.05)
            display_clip = list(map(float, perception_display.get_clipping_range()))
            require(
                np.isclose(display_clip[0], 0.05) and display_clip[1] == display_far_m,
                "Perception display clipping range did not preserve the far plane",
            )
            extra_cameras["perception"] = perception_display
            state["extra_views"]["perception_display"] = {
                "clipping_range_m": display_clip,
                "min_depth_m": MISSION_VIEWS.DISPLAY_MIN_DEPTH_M,
            }
            state["extra_views"]["perception_mount"] = {
                "translation_m": perception_mount.translation_m.tolist(),
                "rotation": perception_mount.rotation.tolist(),
                "intrinsics": asdict(
                    adapter.read_isaac_intrinsics(perception_camera).integer_index
                ),
            }
        lift_guard = None
        if args.perception_mount != "legacy":
            lift_joint = list(robot.dof_names).index("fork_lift")

            def lift_guard():
                # Every render step of a capture, not only before and after:
                # a carriage camera's base<-optical holds at lift 0 only.
                if abs(float(robot.get_joint_positions()[lift_joint])) > 0.001:
                    raise adapter.CaptureFailure("lift_not_zero")

            # The mount as Isaac holds it must be the planned one.
            read_xyz, read_wxyz = perception_camera.get_local_pose(
                camera_axes=args.perception_camera_axes
            )
            require(
                np.allclose(read_xyz, perception_mount.translation_m, atol=1e-6)
                and min(
                    np.abs(np.asarray(read_wxyz) - adapter.xyzw_to_wxyz(mount_xyzw)).max(),
                    np.abs(np.asarray(read_wxyz) + adapter.xyzw_to_wxyz(mount_xyzw)).max(),
                )
                <= 1e-6,
                "Perception camera local pose differs from the planned mount",
            )
        perception_capture = adapter.SensorCapture(
            perception_camera,
            perception_mount,
            # Every physics step, capture included, goes through step_world
            # (SLAM plan: odometry, scans and the lockstep must see them all).
            step_fn=lambda: stepper["fn"](True),
            physics_time_fn=lambda: world.current_time,
            pose_fn=robot.get_world_pose,
            guard_fn=lift_guard,
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
    state["velocities_before_planning"] = body_velocities()
    state["phase"] = "planning"
    if args.use_perception:
        planning_start = time.monotonic()
        state["fixed_observation_waypoints"] = len(args.observation_waypoints)
        state["runtime_viewpoints"] = []
        if args.runtime_viewpoints:
            # After the fixed list, from the map alone: every occupied rectangle,
            # unlabelled, and the bay's placement zone -- never the pallet pose
            # (docs/plans/2026-10-03-runtime-observation-viewpoints.md).
            occupied = [prop.rectangle for prop in scenario.props] + [
                Rectangle(
                    scenario.pickup.x_m,
                    scenario.pickup.y_m,
                    geometry.pallet_depth_m,
                    geometry.pallet_width_m,
                    scenario.pickup.yaw_rad,
                )
            ]
            viewpoints = runtime_viewpoints(
                occupied,
                geometry.unloaded_footprint,
                scenario.bounds,
                margin_m=planner_config.clearance_m,
                pallet_depth_m=geometry.pallet_depth_m,
                pallet_width_m=geometry.pallet_width_m,
                rear_to_camera_m=abs(args.rear_axle_offset_m)
                + float(perception_mount.translation_m[0]),
                half_fov_rad=math.atan(
                    (perception_calibration.width / 2) / perception_calibration.fx
                ),
            )
            state["runtime_viewpoints"] = [
                {
                    "pose": [v.pose.x_m, v.pose.y_m, v.pose.yaw_rad],
                    "score": v.score,
                }
                for v in viewpoints
            ]
            args.observation_waypoints = list(args.observation_waypoints) + [
                [v.pose.x_m, v.pose.y_m, v.pose.yaw_rad] for v in viewpoints
            ]
        state["observation_candidates"] = []
        state["observation_attempts"] = []
        next_candidate_index = 0
        state["observation_waypoint_selected"] = None
        observe_plan = None
        # Every candidate on the earlier ladder first, then -- only if none
        # plans -- every candidate again on the extended one, so a candidate the
        # earlier ladder planned is never displaced by an earlier-numbered one
        # only the extended ladder reaches (fourth-evaluation seed 4020;
        # docs/plans/2026-10-03-transport-stage-fixes.md, T2).
        initial_candidates = [
            (index, coordinates, extended)
            for extended in (False, True)
            for index, coordinates in enumerate(args.observation_waypoints)
        ]
        for candidate_index, coordinates, extended in initial_candidates:
            next_candidate_index = candidate_index + 1
            waypoint = Pose2D(*coordinates)
            candidate_plan = plan_observation_leg(
                scenario,
                waypoint,
                planner_config,
                geometry=geometry,
                pickup_bounds=pickup_bounds,
                extended=extended,
            )
            state["observation_candidates"].append(
                {
                    "candidate_index": candidate_index,
                    "pose": [waypoint.x_m, waypoint.y_m, waypoint.yaw_rad],
                    "success": candidate_plan.success,
                    "status": candidate_plan.status,
                    "analytic_expansion_interval": (
                        candidate_plan.analytic_expansion_interval
                    ),
                    # Every search behind this candidate, failed or not: the
                    # interval alone cannot tell the base lattice from the fine one.
                    "search_attempts": [
                        list(entry) for entry in candidate_plan.search_attempts
                    ],
                    "start_rear": [
                        scenario.start_rear.x_m,
                        scenario.start_rear.y_m,
                        scenario.start_rear.yaw_rad,
                    ],
                    "extended_ladder": extended,
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
        if observe_plan is None and args.repeat_captures:
            # No observation is reached at all, so attempt K never is.
            state["planning_status"] = "all_candidates_failed"
            state["repeat_capture"] = {
                "status": "repeat_target_not_reached",
                "attempts_made": 0,
                "reason": "no_reachable_candidate",
            }
            state["phase"] = "repeat_target_not_reached"
            return
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
        # The per-stage trace survives a later stage's failure; the result does not.
        planning_trace = []
        plans = plan_transport(
            scenario,
            planner_config,
            geometry=geometry,
            return_to=return_to_pose,
            pickup_bounds=pickup_bounds,
            travel_config=travel_config,
            trace=planning_trace,
        )
        state.setdefault("planning_traces", []).append(planning_trace)
        state["planning_wall_s"] = time.monotonic() - planning_start
        state["planning_status"] = plans.status
        require(plans.success, f"Mission planning failed: {plans.status}")
        paths = {name: getattr(plans, name) for name in mission_stages}
        state["paths"] = {name: path_record(path) for name, path in paths.items()}
        (args.output / "paths.json").write_text(
            record_json(state["paths"], indent=2) + "\n"
        )
        add_path_display(stage, paths["approach"], "Approach", (0.05, 0.45, 1.0))
        add_path_display(stage, paths["transport"], "Transport", (1.0, 0.65, 0.04))
        if "return_home" in paths:
            add_path_display(stage, paths["return_home"], "Return", (0.55, 0.2, 0.85))
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

    drive_geometry = args.drive_geometry
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
    extra_encoders = {}
    frame_log = None
    chase_yaw = None
    frame_audit = []
    video_frames = []
    slam_log = None
    slam = None
    # step_world replaces this before the main loop; captures call through it.
    stepper = {"fn": lambda render: world.step(render=render), "tick": 0}
    if args.record_slam:
        import planar_lidar
        import yaml

        from forklift_core.sensors.lidar import PlanarScanPattern

        lidar_config = yaml.safe_load(args.lidar.read_text())
        scan_pattern = PlanarScanPattern(
            lidar_config["beam_count"],
            float(lidar_config["range_min_m"]),
            float(lidar_config["range_max_m"]),
        )
        laser_mount = planar_lidar.LaserMount(
            tuple(map(float, lidar_config["mount_xyz_m"])),
            float(lidar_config["mount_yaw_rad"]),
        )
        beam_angles = scan_pattern.beam_angles_rad()
        scan_every = 120 // int(lidar_config["rate_hz"])
        slam_log = {
            "joint_stamps_s": [],
            "wheel_rates_rad_s": [],
            "steering_rad": [],
            "base_pose_world": [],
            "scan_stamps_s": [],
            "scan_ranges_m": [],
            "laser_pose_world": [],
        }
        state["lidar_synthetic"] = lidar_config
    if args.slam_feedback is not None:
        from forklift_core.localization import slam_link
        from forklift_core.localization.slam_pose import (
            IncrementalWheelOdometry,
            LocalizationStale,
            OdometryNoise,
            SlamPoseTracker,
            StopDetector,
            compose,
        )
        from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry

        # The known start (odom = map = world); the plan's v3 state machine.
        slam_start = (
            scenario.start_rear.x_m,
            scenario.start_rear.y_m,
            scenario.start_rear.yaw_rad,
        )
        slam = {
            "link": slam_link.SlamLinkClient(
                str(args.slam_feedback), timeout_s=args.slam_reply_timeout
            ),
            "module": slam_link,
            "noise": OdometryNoise(
                seed=args.slam_noise_seed or 0,
                enabled=args.slam_noise_seed is not None,
            ),
            "odometry": IncrementalWheelOdometry(
                AckermannOdometryGeometry(
                    args.drive_geometry.wheelbase_m,
                    args.drive_geometry.track_m,
                    args.drive_geometry.wheel_radius_m,
                ),
                initial_pose=slam_start,
            ),
            # Holding now spans approach + insert + extract (up to ~11 m in
            # the recorded runs): the bound is on odometry since the capture.
            "tracker": SlamPoseTracker(max_age_s=0.25, hold_limit_m=15.0),
            "stale": LocalizationStale,
            "stop": StopDetector(tick_s=1 / 120),
            "odom_rear": slam_start,
            "odom_speed": 0.0,
            "odom_yaw_rate": 0.0,
            "scan_id": 0,
            "tick": 0,
            "records": [],
            "control": [],
            "holds": [],
            "pending_release": None,
            "stop_now": False,
            "tracker_speed": 0.0,
            "last_command": 0.0,
        }
        state["slam_feedback"] = {
            "socket": str(args.slam_feedback),
            "noise_seed": args.slam_noise_seed,
            "start_rear": list(slam_start),
            "max_age_s": 0.25,
            "hold_limit_m": 15.0,
            "hold_phases": [
                "approach (from the accepted capture)",
                "insert",
                "lift",
                "extract",
                "lower",
                "withdraw",
            ],
            "release_limits": {"position_m": 0.02, "yaw_rad": 0.02},
        }

    def slam_odom_base():
        x, y, yaw_o = slam["odom_rear"]
        offset = abs(args.rear_axle_offset_m)
        return (x + offset * math.cos(yaw_o), y + offset * math.sin(yaw_o), yaw_o)

    def slam_error():
        """Estimate minus truth in the true rear-axle frame (along, lateral, yaw)."""
        truth_base, truth_q = robot.get_world_pose()
        truth_yaw = yaw_and_tilt(truth_q)[0]
        offset = abs(args.rear_axle_offset_m)
        truth = np.array(
            [
                truth_base[0] - offset * math.cos(truth_yaw),
                truth_base[1] - offset * math.sin(truth_yaw),
            ]
        )
        if slam["tracker"].applied is None:
            x, y, yaw_e = slam["odom_rear"]
        else:
            bx, by, yaw_e = compose(slam["tracker"].applied[0], slam_odom_base())
            x, y = bx - offset * math.cos(yaw_e), by - offset * math.sin(yaw_e)
        dx, dy = x - truth[0], y - truth[1]
        c, s_ = math.cos(truth_yaw), math.sin(truth_yaw)
        return {
            "along_m": float(c * dx + s_ * dy),
            "lateral_m": float(-s_ * dx + c * dy),
            "yaw_rad": float(math.atan2(math.sin(yaw_e - truth_yaw), math.cos(yaw_e - truth_yaw))),
        }

    def annotation_pose():
        """World (position, wxyz) for video annotations: the truth, or under
        SLAM feedback the applied estimate (no freshness check -- drawing only)."""
        position, orientation = robot.get_world_pose()
        if slam is None or slam["tracker"].applied is None:
            return position, orientation
        x, y, yaw_e = compose(slam["tracker"].applied[0], slam_odom_base())
        return (
            np.array([x, y, float(position[2])]),
            np.array([math.cos(yaw_e / 2), 0.0, 0.0, math.sin(yaw_e / 2)]),
        )

    def slam_rear(now_s):
        if slam["tracker"].applied is None:
            # Before the first scan: odom = map at the known start (plan v3.1);
            # warming up holds the drive anyway.
            x, y, yaw_o = slam["odom_rear"]
            return np.array([x, y, yaw_o])
        try:
            est = slam["tracker"].map_from_base(now_s, slam_odom_base())
        except slam["stale"] as exc:
            require(False, f"localization_stale: {exc}")
        offset = abs(args.rear_axle_offset_m)
        return np.array(
            [est[0] - offset * math.cos(est[2]), est[1] - offset * math.sin(est[2]), est[2]]
        )
    if args.robot_camera:
        import video_frames as video_frames_module
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
    if args.return_home:
        # The return runs unloaded, so it reuses the unloaded approach speed
        # rather than introducing a settings key the recorded runs never had.
        speeds["return_home"] = settings["approach_speed_mps"]
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
                position_tolerance_m=0.03 if name == "observe" else 0.008,
                # Observation and return are repositioning moves, not docking,
                # in heading: measured seeds 18 and 20 ended the return at
                # 0.024 and 0.021 rad, inside the repositioning tolerance and
                # outside the docking one. Only observation also stops within
                # 3 cm: a 3 cm return stopped 3 cm short before the heading
                # settled (seed 23 at 0.041 rad, 2026-09-27), and the
                # overshoot tolerance already covers a stop just past the goal.
                # Insertion tolerances are unchanged. An observation stop is
                # judged like a cusp in heading (0.05 rad): the next leg and the
                # capture start from the measured pose (second-evaluation seed
                # 2018 entered the 3 cm window at 0.036 rad on a curved end;
                # docs/plans/2026-10-02-second-eval-failure-fixes.md, P3).
                yaw_tolerance_rad=(
                    0.05
                    if name == "observe"
                    else 0.03
                    if name == "return_home"
                    else 0.02
                ),
                # Gear-change cusps are not goals (2026-09-26): the next leg
                # starts from the measured pose. Final goals keep the rules above.
                cusp_position_tolerance_m=0.03,
                cusp_yaw_tolerance_rad=0.05,
                # Brake and judge the cusp at 8 mm, still accepting 30 mm / 50 mrad
                # (docs/plans/2026-10-02-planner-tracker-robustness.md, P2).
                cusp_brake_window_m=0.008,
                # Optional path speed caps; absent from the settings = off.
                max_lateral_acceleration_mps2=settings.get(
                    "max_lateral_acceleration_mps2"
                ),
                max_reverse_speed_mps=settings.get("max_reverse_speed_mps"),
                # A stop just past the goal is fine except deeper into
                # the pallet: insertion keeps the round 8 mm.
                overshoot_tolerance_m=None if name == "insert" else 0.03,
                stop_speed_mps=0.012,
                max_cross_track_error_m=0.35,
            ),
        )
        for name, path in paths.items()
    }

    def apply_tracker_profile() -> None:
        # The 2026-09-21 G2 baseline had no cusp or overshoot tolerances and
        # stopped observation at 8 mm (git show 6f9fb82:sim/isaac/run_transport.py).
        if args.tracker_profile != "20260921":
            return
        for name, tracker in trackers.items():
            tracker.config = replace(
                tracker.config,
                cusp_position_tolerance_m=None,
                cusp_yaw_tolerance_rad=None,
                cusp_brake_window_m=None,
                overshoot_tolerance_m=None,
                position_tolerance_m=0.008,
            )
            if name == "observe":
                # It also judged observation stops at 0.03 rad (P3 is newer).
                tracker.config = replace(tracker.config, yaw_tolerance_rad=0.03)

    def record_tracker_configs() -> None:
        # Every tracker's full settings as built (G2 rerun plan, 2026-10-01).
        state.setdefault("tracker_configs", {}).update(
            {name: asdict(tracker.config) for name, tracker in trackers.items()}
        )

    apply_tracker_profile()
    record_tracker_configs()
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
    inset_estimate = None
    inset_status = "팔레트 탐색 중"
    inset_detail = ""
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
        if slam is None:
            return
        # Docking phases freeze the applied correction (plan v3.1); the next
        # travel leg waits for a stop and the release below.
        # approach: the pallet estimate was made with the correction applied at
        # the accepted capture (this same instant -- the truck stands still for
        # the capture, so no keyframe lands in between). Keep that correction
        # until extract: the docking then runs on odometry relative to what was
        # seen, and a later SLAM correction cannot slide the truck against the
        # pallet estimate (plan v3.5; S2 seed 0 on v3.4 hit a block that way).
        if phase in ("approach", "insert", "lower") and slam["tracker"].mode == "tracking":
            slam["tracker"].hold(odom_from_base=slam_odom_base())
            slam["holds"].append(
                {"phase": phase, "time_s": t, "event": "hold", "error": slam_error()}
            )
        elif phase in ("transport", "settle"):
            slam["pending_release"] = phase
            slam["release_wait_from"] = t

    def slam_release(t: float) -> None:
        """Apply the received correction while stopped; replan the next leg
        from the new estimate if it moved more than 2 cm or 0.02 rad."""
        nonlocal phase_started
        leg = "transport" if slam["pending_release"] == "transport" else "return_home"
        slam["pending_release"] = None
        before = slam_rear(t)
        jump_m, jump_rad = slam["tracker"].release(odom_from_base=slam_odom_base())
        after = slam_rear(t)
        event = {
            "phase": phase,
            "time_s": t,
            "event": "release",
            "jump_m": jump_m,
            "jump_rad": jump_rad,
            "rear_before": before.tolist(),
            "rear_after": after.tolist(),
            "replanned": False,
        }
        slam["holds"].append(event)
        if leg not in trackers or (jump_m <= 0.02 and abs(jump_rad) <= 0.02):
            return
        start = PlanningPose(float(after[0]), float(after[1]), float(after[2]))
        replan_start = time.monotonic()
        if leg == "transport":
            replanned = plan_transport_leg(
                scenario, start, planner_config, geometry=geometry, travel_config=travel_config
            )
        else:
            replanned = plan_return_leg(
                scenario,
                start,
                return_to_pose,
                planner_config,
                geometry=geometry,
                travel_config=travel_config,
            )
        event.update(
            replaced_path=path_record(paths[leg]),
            replanned=True,
            replan_status=replanned.status,
            planning_wall_s=time.monotonic() - replan_start,
        )
        require(replanned.success, f"slam_release_replan_failed: {replanned.status}")
        paths[leg] = replanned
        state["paths"][leg] = path_record(replanned)
        (args.output / "paths.json").write_text(record_json(state["paths"], indent=2) + "\n")
        trackers[leg] = RearAxlePathTracker(
            replanned.poses,
            replanned.directions,
            replanned.curvatures_inv_m,
            trackers[leg].config,
        )
        add_path_display(
            stage,
            replanned,
            "Transport" if leg == "transport" else "Return",
            (1.0, 0.65, 0.04) if leg == "transport" else (0.55, 0.2, 0.85),
        )
        if leg == phase:
            phase_started = t

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
        if args.robot_camera:
            size = (perception_calibration.width, perception_calibration.height)
            for name in ("rgb", "depth"):
                extra_encoders[name] = open_encoder(
                    args.output / f"camera_{name}.mp4", *size, args.fps
                )
        if args.extra_views:
            frame_log = (args.output / "frames.jsonl").open("w", encoding="utf-8")
            for name in args.extra_views:
                height = 480 if name == "perception" else 720
                extra_encoders[name] = subprocess.Popen(
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
                        f"1280x{height}",
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
                        str(args.output / f"view_{name}.mp4"),
                    ],
                    stdin=subprocess.PIPE,
                )
        last_tracking = None

        def dump_tracking(reason: str, tracking) -> None:
            # The run aborts right after this, before the next sample; keep what
            # the G2b diagnosis needs (closeout plan, G2b ㉮).
            tracker = trackers[phase]
            path = paths[phase]
            directions = np.asarray(path.directions)
            cusps = [
                int(i)
                for i in range(1, len(directions) - 1)
                if directions[i + 1] != directions[i]
            ]
            final = int(len(path.poses) - 1)
            record = {
                "reason": reason,
                "phase": phase,
                "time_s": t,
                "phase_started_s": phase_started,
                "loop_step": step,
                "config": asdict(tracker.config),
                "rear_pose": rear.tolist(),
                "signed_speed_mps": signed_speed,
                "steering_command_rad": steering_command.tolist(),
                "steering_actual_rad": np.asarray(
                    robot.get_joint_positions()[steers], dtype=float
                ).tolist(),
                "cusp_indices": cusps,
                "final_index": final,
                "path_length_m": float(path.length_m),
                "path": path_record(path),
                "tracking": None if tracking is None else asdict(tracking),
            }
            # The leg being driven, as the tracker itself holds it (its errors
            # refer to this endpoint): private state, read only for the record.
            leg = int(tracker._leg)
            endpoint = int(tracker._leg_ends[leg])
            record.update(
                leg=leg,
                leg_end_index=endpoint,
                leg_end_is_cusp=endpoint != final,
                remaining_to_leg_end_m=float(
                    tracker._distance[endpoint] - tracker._progress
                ),
                remaining_to_path_end_m=float(
                    tracker._distance[-1] - tracker._progress
                ),
            )
            state["tracking_failure"] = record

        def write_video_frame(stamp: float) -> None:
            """One video frame per fps tick of physics, capture steps included
            (SLAM plan v3.1: frames match the run length)."""
            frame = camera.get_current_frame()
            rgba = camera.get_rgba()
            # get_rgba reads the RGB annotator directly. The SDK's cached
            # frame metadata can lag; retain it for audit without treating
            # its timestamp as a control or image-acquisition failure.
            require(
                rgba is not None and rgba.shape == (720, 1280, 4),
                "Camera did not produce RGB",
            )
            video_rgb = rgba[:, :, :3]
            if args.camera_inset:
                # Pose after this render step, so outline and picture agree;
                # under SLAM feedback the pose the robot believes.
                now_base, now_q = annotation_pose()
                now_yaw, _ = yaw_and_tilt(now_q)
                live = perception_camera.get_rgba()
                if live is None or live.ndim != 3 or live.size == 0:
                    live = None
                video_rgb = inset.compose_frame(
                    video_rgb,
                    inset.render_inset(
                        live,
                        phase=phase,
                        current_pose=(
                            float(now_base[0]),
                            float(now_base[1]),
                            now_yaw,
                        ),
                        estimate=inset_estimate,
                        status=inset_status,
                        detail=inset_detail,
                        fork_tip_x_m=args.axle_to_fork_tip_m
                        - abs(args.rear_axle_offset_m),
                        font_path=inset_font,
                    ),
                )
            encoder.stdin.write(
                np.ascontiguousarray(video_rgb, dtype=np.uint8).tobytes()
            )
            if args.robot_camera:
                record_robot_camera_frame(
                    perception_camera,
                    perception_calibration,
                    extra_encoders,
                    video_frames,
                    video_frames_module,
                    inset,
                    stamp=stamp,
                    phase=phase,
                    state=state,
                    pose=annotation_pose(),
                    estimate=inset_estimate,
                    fork_tip_x_m=args.axle_to_fork_tip_m
                    - abs(args.rear_axle_offset_m),
                    overview=camera,
                    bounds=scenario.bounds,
                )
                if slam is not None:
                    # The map panel may show only maps the bridge recorded
                    # after a scan id below this one (received before it).
                    video_frames[-1]["slam_last_scan_id"] = slam["scan_id"] - 1
                    video_frames[-1]["slam_mode"] = slam["tracker"].mode
            state["frames"] += 1
            if args.extra_views:
                for name in args.extra_views:
                    view_camera = extra_cameras[name]
                    view_rgba = view_camera.get_rgba()
                    expected = (
                        (480, 640, 4) if name == "perception" else (720, 1280, 4)
                    )
                    require(
                        view_rgba is not None and view_rgba.shape == expected,
                        f"{name} camera did not produce RGB",
                    )
                    view_rgb = np.ascontiguousarray(
                        view_rgba[:, :, :3], dtype=np.uint8
                    )
                    if name == "perception":
                        depth = view_camera.get_depth()
                        require(
                            depth is not None and depth.shape[:2] == (480, 640),
                            "Perception camera did not produce depth",
                        )
                        if depth.ndim == 3:
                            depth = depth[:, :, 0]
                        view_rgb = np.concatenate(
                            (view_rgb, MISSION_VIEWS.depth_colormap(depth)),
                            axis=1,
                        )
                    extra_encoders[name].stdin.write(view_rgb.tobytes())
                frame_base, frame_q = robot.get_world_pose()
                frame_pallet, frame_pq = pallet.get_world_pose()
                frame_log.write(
                    record_json(
                        {
                            "frame": state["frames"] - 1,
                            "simulation_time_s": world.current_time - initial_time,
                            "phase": phase,
                            "base_position_m": frame_base,
                            "base_orientation_wxyz": frame_q,
                            "pallet_position_m": frame_pallet,
                            "pallet_orientation_wxyz": frame_pq,
                            "lift_m": float(
                                robot.get_joint_positions()[lift_index[0]]
                            ),
                        }
                    )
                    + "\n"
                )
            frame_audit.append(
                {
                    "simulation_time_s": world.current_time - initial_time,
                    "rendering_time": frame.get("rendering_time"),
                }
            )
            if state["frames"] == 1:
                snapshot("start")

        def step_world(render: bool) -> None:
            """One physics step and everything that must see every step
            (SLAM plan v3.1): encoders, odometry, the 10 Hz scan by physics
            tick (not loop step) and the SLAM lockstep. Capture steps too."""
            tick = stepper["tick"]
            stepper["tick"] = tick + 1
            frame_due = args.video and tick % fps_divisor == 0
            world.step(render=render or frame_due)
            stamp_now = world.current_time - initial_time
            if slam_log is None:
                if frame_due:
                    write_video_frame(stamp_now)
                return
            now_base, now_q = robot.get_world_pose()
            rates_true = robot.get_joint_velocities()[wheels]
            steer_true = robot.get_joint_positions()[steers]
            slam_log["joint_stamps_s"].append(stamp_now)
            slam_log["wheel_rates_rad_s"].append(rates_true)
            slam_log["steering_rad"].append(steer_true)
            slam_log["base_pose_world"].append(np.concatenate((now_base, now_q)))
            if slam is not None:
                rates = slam["noise"].wheel_rates(rates_true[2:4])
                angles = slam["noise"].steering(steer_true)
                previous_yaw = slam["odom_rear"][2]
                slam["odom_rear"] = slam["odometry"].update(stamp_now, rates, angles)
                slam["odom_speed"] = float(np.mean(rates)) * args.drive_geometry.wheel_radius_m
                slam["odom_yaw_rate"] = (slam["odom_rear"][2] - previous_yaw) * 120.0
                slam["stop_now"] = slam["stop"].update(
                    commanded_speed=slam.get("last_command", 0.0),
                    speed=slam["odom_speed"],
                    yaw_rate=slam["odom_yaw_rate"],
                )
                # Every tracker in this runner stops at 0.012 m/s; its arrival
                # and gear-change checks wait for the noise-aware detector.
                slam["tracker_speed"] = slam["stop"].tracker_speed(0.012)
            if frame_due:
                # After this tick's odometry, so the annotation matches the image.
                write_video_frame(stamp_now)
            if tick % scan_every:
                return
            origin, directions = planar_lidar.laser_rays_world(
                now_base, now_q, laser_mount, beam_angles
            )
            distances, hits, _ = planar_lidar.cast_scan(
                origin, directions, scan_pattern.range_max_m
            )
            ranges = scan_pattern.ranges_from_hits(distances, hits)
            slam_log["scan_stamps_s"].append(stamp_now)
            slam_log["scan_ranges_m"].append(ranges.astype(np.float32))
            slam_log["laser_pose_world"].append(
                planar_lidar.laser_pose_2d(now_base, now_q, laser_mount)
            )
            if slam is None:
                return
            link = slam["module"]
            sent = slam["noise"].ranges(
                ranges,
                range_min_m=scan_pattern.range_min_m,
                range_max_m=scan_pattern.range_max_m,
            )
            try:
                reply = slam["link"].exchange(
                    link.Scan(
                        slam["scan_id"],
                        float(stamp_now),
                        slam_odom_base(),
                        sent.astype(np.float32),
                        scan_pattern.angle_min_rad,
                        scan_pattern.angle_increment_rad,
                        scan_pattern.range_min_m,
                        scan_pattern.range_max_m,
                    )
                )
            except link.SlamLinkFailure as exc:
                slam["records"].append(
                    {
                        "scan_id": slam["scan_id"],
                        "stamp_s": float(stamp_now),
                        "phase": phase,
                        "status": "link_failure",
                        "error": str(exc),
                    }
                )
                require(False, f"slam_link_failed: {exc}")
            slam["tracker"].receive(
                reply.scan_id, reply.stamp_s, reply.status, reply.map_from_odom
            )
            truth_yaw, _ = yaw_and_tilt(now_q)
            slam["records"].append(
                {
                    "scan_id": slam["scan_id"],
                    "stamp_s": float(stamp_now),
                    "phase": phase,
                    "status": reply.status,
                    "mode": slam["tracker"].mode,
                    "map_from_odom": list(reply.map_from_odom),
                    # What control uses (holding keeps an older one).
                    "applied_map_from_odom": list(slam["tracker"].applied[0])
                    if slam["tracker"].applied is not None
                    else None,
                    "odom_base": list(slam_odom_base()),
                    "truth_base": [float(now_base[0]), float(now_base[1]), truth_yaw],
                    "slam_wall_s": reply.slam_wall_s,
                }
            )
            slam["scan_id"] += 1

        stepper["fn"] = step_world
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
            # Ground truth checks, evaluates and renders; control below uses
            # `rear`/`signed_speed`, which --slam-feedback replaces.
            truth_rear = rear
            if slam is not None:
                rear = slam_rear(t)
                signed_speed = slam["tracker_speed"]
                slam["control"].append([t, *rear.tolist(), *truth_rear.tolist()])
            require(np.isfinite([base, ppos]).all(), "Nonfinite body state")
            if phase == "lift":
                # Every physics step, so the peak and the aborting state are
                # kept; the 0.1 s samples miss both.
                peak = state.setdefault("lift_tilt_peak", {"pallet_tilt_rad": -1.0})
                if pallet_tilt > peak["pallet_tilt_rad"]:
                    peak.update(
                        time_s=t,
                        pallet_tilt_rad=pallet_tilt,
                        pallet_position_m=ppos,
                        pallet_quaternion_wxyz=pq,
                        lift_m=float(robot.get_joint_positions()[lift_index[0]]),
                        lift_command_m=lift_command,
                    )
            if not (tilt < 0.1 and pallet_tilt < 0.15):
                state["tilt_abort_state"] = {
                    "phase": phase,
                    "time_s": t,
                    "base_tilt_rad": tilt,
                    "pallet_tilt_rad": pallet_tilt,
                    "base_position_m": base,
                    "base_quaternion_wxyz": q,
                    "pallet_position_m": ppos,
                    "pallet_quaternion_wxyz": pq,
                    "lift_m": float(robot.get_joint_positions()[lift_index[0]]),
                    "lift_command_m": lift_command,
                }
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
            if phase in ("return_home", "home_settle"):
                # The pallet is on the floor behind the forks now, so drive
                # around its measured pose rather than its planned one.
                checked_obstacles = checked_obstacles + [
                    Rectangle(
                        float(ppos[0]),
                        float(ppos[1]),
                        geometry.pallet_depth_m,
                        geometry.pallet_width_m,
                        pallet_yaw,
                    )
                ]
            # Runtime pose checks below use zero margin intentionally --
            # clearance_m is a planning-time buffer against the intended path,
            # not a re-check of the executed pose; spawn_clearance_m already
            # keeps obstacles far enough that a nominal run clears this at
            # margin 0 (docs/plans/2026-09-17-hybrid-astar-transport.md,
            # "계획 여유가 시험으로 고정돼 있지 않다").
            require(
                collision_free_pose(
                    truth_rear, checked_obstacles, footprint, scenario.bounds
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
            if phase in ["insert", "extract", "withdraw"]:
                # Smallest sideways gap per blade over every check (SLAM plan
                # S2 insertion criterion), kept per phase.
                lateral = insertion_geometry.lateral_clearances(
                    base,
                    q,
                    float(robot.get_joint_positions()[lift_index[0]]),
                    ppos,
                    pq,
                )
                minima = state.setdefault("lateral_clearance_min_m", {}).setdefault(
                    phase, {"left": None, "right": None}
                )
                for side, gap in lateral.items():
                    if gap is not None and (minima[side] is None or gap < minima[side]):
                        minima[side] = gap
            requested_speed, curvature = 0.0, 0.0
            tracking = None
            releasing = slam is not None and slam.get("pending_release") is not None
            if releasing:
                waited = t - slam["release_wait_from"]
                require(waited < 10.0, f"slam_release_no_stop after {waited:.1f} s")
                if slam["stop_now"]:
                    slam_release(t)
                    releasing = False
                    rear = slam_rear(t)  # the released estimate, this very tick
                    slam["control"][-1][1:4] = rear.tolist()
            warming = slam is not None and not slam["tracker"].may_drive()
            if phase in trackers and not releasing and not warming:
                # Three times the path time at the tracker's own speed caps
                # (length / cruise when there are none), plus 10 s.
                limit = max(30.0, 3 * trackers[phase].nominal_duration_s() + 10)
                if t - phase_started >= limit:
                    dump_tracking("timeout", last_tracking)
                require(t - phase_started < limit, f"Tracking timeout in {phase}")
                tracking = trackers[phase].update(rear, signed_speed, dt)
                last_tracking = tracking
                if (
                    slam is not None
                    and phase in ("observe", "approach", "transport", "return_home")
                    and slam["tracker"].mode == "tracking"
                    and trackers[phase].remaining_to_goal_m()
                    <= 0.5 + signed_speed**2 / (2 * settings["drive_acceleration_mps2"])
                ):
                    # The window grows with the braking distance (Codex v3.5 P2:
                    # a correction at 0.507 m and 0.55 m/s left no room to stop).
                    # A final goal is judged at 8 mm:
                    # a SLAM correction landing in the last half metre moves
                    # the estimate by centimetres and leaves the truck stopped
                    # outside the tolerance (S2 seed 1 on v3.4, transport).
                    # Freeze it there (plan v3.5; approach already holds from
                    # the capture). Released at settle as before.
                    slam["tracker"].hold(odom_from_base=slam_odom_base())
                    slam["holds"].append(
                        {"phase": phase, "time_s": t, "event": "hold", "error": slam_error()}
                    )
                stalled = (
                    slam is not None
                    and phase in ("transport", "return_home")
                    and tracking.status == "tracking"
                    and tracking.speed_mps == 0.0
                    and trackers[phase].remaining_to_goal_m() <= 1e-6
                    and slam["stop_now"]
                )
                slam_stall_ticks = slam_stall_ticks + 1 if stalled else 0
                if slam_stall_ticks >= 120 and len(state["stall_replans"]) < 2:
                    # Stopped at the end of the path but outside the goal
                    # tolerance (an estimate shift before the hold): plan the
                    # leg again from here, in the held frame (Codex v3.5 P2).
                    slam_stall_ticks = 0
                    start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
                    replan_start = time.monotonic()
                    if phase == "transport":
                        replanned = plan_transport_leg(
                            scenario, start, planner_config, geometry=geometry,
                            travel_config=travel_config,
                        )
                    else:
                        replanned = plan_return_leg(
                            scenario, start, return_to_pose, planner_config,
                            geometry=geometry, travel_config=travel_config,
                        )
                    state["stall_replans"].append(
                        {
                            "phase": phase,
                            "time_s": t,
                            "rear_pose": rear.tolist(),
                            "position_error_m": tracking.position_error_m,
                            "yaw_error_rad": tracking.yaw_error_rad,
                            "status": replanned.status,
                            "planning_wall_s": time.monotonic() - replan_start,
                        }
                    )
                    require(replanned.success, f"stall_replan_failed:{replanned.status}")
                    paths[phase] = replanned
                    state["paths"][phase] = path_record(replanned)
                    (args.output / "paths.json").write_text(
                        record_json(state["paths"], indent=2) + "\n"
                    )
                    add_path_display(
                        stage,
                        replanned,
                        "Transport" if phase == "transport" else "Return",
                        (1.0, 0.65, 0.04) if phase == "transport" else (0.55, 0.2, 0.85),
                    )
                    trackers[phase] = RearAxlePathTracker(
                        replanned.poses,
                        replanned.directions,
                        replanned.curvatures_inv_m,
                        trackers[phase].config,
                    )
                    phase_started = t
                    tracking = trackers[phase].update(rear, signed_speed, dt)
                    last_tracking = tracking
                if (
                    tracking.status == "failed"
                    and phase == "transport"
                    and tracking.failure == "endpoint_heading"
                    and tracking.at_cusp
                    and not tracking.off_path
                    and len(state["cusp_replans"]) < max_cusp_replans
                ):
                    stopped = (
                        slam["stop_now"]
                        if slam is not None
                        else (
                            tracking.speed_mps == 0.0
                            and float(np.linalg.norm(velocity[:2]))
                            <= trackers[phase].config.stop_speed_mps
                            and abs(float(robot.get_angular_velocity()[2]))
                            <= cusp_stop_yaw_rate_radps
                        )
                    )
                    cusp_stop_ticks = cusp_stop_ticks + 1 if stopped else 0
                    if cusp_stop_ticks < cusp_stop_needed:
                        # The failed tracker already commands zero; let it stop.
                        tracking = replace(tracking, status="braking")
                    else:
                        cusp_stop_ticks = 0
                        replan_start = time.monotonic()
                        replanned = plan_transport_leg(
                            scenario,
                            PlanningPose(float(rear[0]), float(rear[1]), float(rear[2])),
                            planner_config,
                            geometry=geometry,
                            travel_config=travel_config,
                        )
                        replan_wall_s = time.monotonic() - replan_start
                        state["planning_wall_s"] = (
                            state.get("planning_wall_s") or 0.0
                        ) + replan_wall_s
                        state["cusp_replans"].append(
                            {
                                "time_s": t,
                                "loop_step": step,
                                "rear_pose": rear.tolist(),
                                "position_error_m": tracking.position_error_m,
                                "yaw_error_rad": tracking.yaw_error_rad,
                                "status": replanned.status,
                                "planning_wall_s": replan_wall_s,
                                "search_attempts": [
                                    list(entry) for entry in replanned.search_attempts
                                ],
                                "path": path_record(replanned)
                                if replanned.success
                                else None,
                                "replaced_path": path_record(paths[phase]),
                            }
                        )
                        if replanned.success:
                            paths[phase] = replanned
                            # The active plan everywhere: record, file, display.
                            state["paths"][phase] = path_record(replanned)
                            (args.output / "paths.json").write_text(
                                record_json(state["paths"], indent=2) + "\n"
                            )
                            add_path_display(
                                stage, replanned, "Transport", (1.0, 0.65, 0.04)
                            )
                            trackers[phase] = RearAxlePathTracker(
                                replanned.poses,
                                replanned.directions,
                                replanned.curvatures_inv_m,
                                trackers[phase].config,
                            )
                            phase_started = t
                            tracking = trackers[phase].update(rear, signed_speed, dt)
                            last_tracking = tracking
                if tracking.status == "failed":
                    dump_tracking("failed", tracking)
                require(
                    tracking.status != "failed",
                    f"Tracking failed in {phase}: pos={tracking.position_error_m:.4f},yaw={tracking.yaw_error_rad:.4f}",
                )
                requested_speed, curvature = (
                    tracking.speed_mps,
                    tracking.curvature_inv_m,
                )
                if slam is not None and not slam["tracker"].may_drive():
                    requested_speed = 0.0  # warming up: hold still
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
                            # Main-loop step and the t the next phase starts from:
                            # together they fix the remaining time budget.
                            "loop_step": step,
                            "t_before_capture_s": t,
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
                        attempt["depth_sha256"] = G2.depth_sha256(scene_input.depth_m)
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
                        attempt["accepted_pose"] = [
                            np.asarray(v, dtype=float).tolist()
                            for v in capture_diagnostics.accepted_pose
                        ]
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
                        if slam is not None:
                            now_s = world.current_time - initial_time
                            near_pending = (
                                state.get("near_capture", {}).get("status") == "pending"
                            )
                            if slam["tracker"].mode == "holding" and not near_pending:
                                # An observe leg ends held (below). Anything but the
                                # near capture is followed by more driving, so let
                                # SLAM back in now, while the truck stands still
                                # for the capture (plan v3.6).
                                jump_m, jump_rad = slam["tracker"].release(
                                    odom_from_base=slam_odom_base()
                                )
                                slam["holds"].append(
                                    {
                                        "phase": phase,
                                        "time_s": now_s,
                                        "event": "release_at_capture",
                                        "jump_m": jump_m,
                                        "jump_rad": jump_rad,
                                    }
                                )
                            # The accepted ground-truth pose only validated the
                            # capture; control takes the estimate at this instant.
                            rear = slam_rear(now_s)
                            yaw = float(rear[2])
                            forward = np.array([math.cos(yaw), math.sin(yaw)])
                            base = np.array(
                                [
                                    rear[0] + abs(args.rear_axle_offset_m) * forward[0],
                                    rear[1] + abs(args.rear_axle_offset_m) * forward[1],
                                    float(base[2]),
                                ]
                            )
                            attempt["control_pose_source"] = "slam_estimate"
                            attempt["slam_error"] = slam_error()
                        attempt.update(
                            {
                                "frame_diagnostics": asdict(frame_diagnostics),
                                "capture_attempts": capture_attempts,
                            }
                        )
                        prior = args.pallet_prior_loaded
                        params = DetectorParams.derived_for(prior)
                        state["detector_params"] = asdict(params)
                        # B1c: the detector sees millimetre depth when asked; the
                        # saved depth above stays raw (G3 applies the same rounding).
                        detector_input = adapter.detector_input(
                            scene_input, args.depth_quantize_mm
                        )
                        attempt["base_from_optical"] = {
                            "translation_m": np.asarray(
                                scene_input.base_from_optical.translation_m
                            ).tolist(),
                            "rotation": np.asarray(
                                scene_input.base_from_optical.rotation
                            ).tolist(),
                        }
                        attempt["depth_quantize_mm"] = args.depth_quantize_mm
                        detection = detect_pockets(detector_input, prior, params)
                        observation = detection.observation
                        attempt.update(
                            {
                                "pocket_observation": asdict(observation),
                                "frame_diagnostics": asdict(frame_diagnostics),
                                "detection_diagnostics": asdict(detection.diagnostics),
                                "capture_attempts": capture_attempts,
                            }
                        )
                        if (
                            args.repeat_captures
                            and attempt_number == args.repeat_at_attempt
                        ):
                            # G2r: the original capture above is G2's; hold the
                            # wheels, capture again, then stop whatever is seen.
                            robot.apply_action(
                                ArticulationAction(
                                    joint_velocities=np.zeros(len(wheels)),
                                    joint_indices=wheels,
                                )
                            )
                            first_pose = [
                                np.asarray(v, dtype=float).tolist()
                                for v in capture_diagnostics.accepted_pose
                            ]

                            def observed(obs) -> dict:
                                valid = obs.status == "valid"
                                return {
                                    "status": obs.status,
                                    "reason": obs.reason,
                                    "left_center_m": list(obs.left.center_m)
                                    if valid
                                    else None,
                                    "right_center_m": list(obs.right.center_m)
                                    if valid
                                    else None,
                                    "insertion_yaw_rad": obs.insertion_yaw_rad,
                                }

                            def valid_fraction(depth) -> float:
                                depth = np.asarray(depth, dtype=float)
                                return float((np.isfinite(depth) & (depth > 0)).mean())

                            captures = [
                                {
                                    "index": 0,
                                    "accepted_pose": first_pose,
                                    "drift": G2.drift(first_pose, first_pose),
                                    "depth_sha256": attempt["depth_sha256"],
                                    "valid_fraction": valid_fraction(
                                        scene_input.depth_m
                                    ),
                                    **observed(observation),
                                }
                            ]
                            for index in range(1, args.repeat_captures + 1):
                                record = {"index": index}
                                try:
                                    repeat_input, _, repeat_attempts = (
                                        perception_capture.capture(
                                            max_attempts=args.perception_max_attempts,
                                        )
                                    )
                                except adapter.CaptureFailure as exc:
                                    record["capture_failure"] = exc.reason
                                    record["capture_diagnostics"] = asdict(
                                        exc.diagnostics
                                    )
                                    captures.append(record)
                                    break
                                repeat_diagnostics = asdict(
                                    perception_capture.state.diagnostics
                                )
                                pose = [
                                    np.asarray(v, dtype=float).tolist()
                                    for v in perception_capture.state.diagnostics.accepted_pose
                                ]
                                record.update(
                                    {
                                        "capture_diagnostics": repeat_diagnostics,
                                        "capture_attempts": repeat_attempts,
                                        "accepted_pose": pose,
                                        "drift": G2.drift(first_pose, pose),
                                        "depth_sha256": G2.depth_sha256(
                                            repeat_input.depth_m
                                        ),
                                        "depth_change": G2.depth_change(
                                            scene_input.depth_m, repeat_input.depth_m
                                        ),
                                        "valid_fraction": valid_fraction(
                                            repeat_input.depth_m
                                        ),
                                        **observed(
                                            detect_pockets(
                                                adapter.detector_input(
                                                    repeat_input, args.depth_quantize_mm
                                                ),
                                                prior,
                                                params,
                                            ).observation
                                        ),
                                    }
                                )
                                captures.append(record)
                            state["repeat_capture"] = {
                                "status": "repeat_capture_done",
                                "attempt_number": attempt_number,
                                "candidate_index": attempt["candidate_index"],
                                "wheels_held": True,
                                "captures": captures,
                                "summary": G2.repeat_summary(captures),
                            }
                            transition("repeat_capture_done", t)
                            break
                        if observation.status != "valid":
                            attempt["retry_reason"] = (
                                f"perception_{observation.status}:{observation.reason}"
                            )
                            if state.get("near_capture", {}).get("status") == "pending":
                                # No silent fallback to the far estimate (v3.6).
                                state["near_capture"]["status"] = "failed"
                                require(
                                    False,
                                    f"near_capture_failed:{attempt['retry_reason']}",
                                )
                            inset_status = f"인식 실패 #{attempt_number} · 재관측 이동"
                            inset_detail = f"사유: {observation.reason}"
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
                                    planner_config,
                                    geometry=geometry,
                                    start_rear=Pose2D(rear[0], rear[1], rear[2]),
                                    pickup_bounds=pickup_bounds,
                                    # Re-observation keeps the earlier ladder.
                                    extended=False,
                                )
                                state["observation_candidates"].append(
                                    {
                                        "candidate_index": candidate_index,
                                        "pose": list(coordinates),
                                        "start_rear": rear.tolist(),
                                        "success": candidate_plan.success,
                                        "status": candidate_plan.status,
                                        "analytic_expansion_interval": (
                                            candidate_plan.analytic_expansion_interval
                                        ),
                                        "search_attempts": [
                                            list(entry)
                                            for entry in candidate_plan.search_attempts
                                        ],
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
                            if observe_plan is None and args.repeat_captures:
                                state["repeat_capture"] = {
                                    "status": "repeat_target_not_reached",
                                    "attempts_made": attempt_number,
                                    "reason": "candidates_exhausted",
                                }
                                transition("repeat_target_not_reached", t)
                                break
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
                                    position_tolerance_m=0.03,
                                    # Judged like a cusp in heading (P3, above).
                                    yaw_tolerance_rad=0.05,
                                    # Gear-change cusps are not goals (2026-09-26): the next leg
                                    # starts from the measured pose. Final goals keep the rules above.
                                    cusp_position_tolerance_m=0.03,
                                    cusp_yaw_tolerance_rad=0.05,
                                    # Brake and judge the cusp at 8 mm, still accepting 30 mm / 50 mrad
                                    # (docs/plans/2026-10-02-planner-tracker-robustness.md, P2).
                                    cusp_brake_window_m=0.008,
                                    # Optional path speed caps; absent from the settings = off.
                                    max_lateral_acceleration_mps2=settings.get(
                                        "max_lateral_acceleration_mps2"
                                    ),
                                    max_reverse_speed_mps=settings.get(
                                        "max_reverse_speed_mps"
                                    ),
                                    overshoot_tolerance_m=0.03,
                                    stop_speed_mps=0.012,
                                    max_cross_track_error_m=0.35,
                                ),
                            )
                            phase_started = t
                            apply_tracker_profile()
                            record_tracker_configs()
                        else:
                            if args.repeat_captures:
                                # G2r took another path to a valid detection
                                # before the requested attempt: no batch, no plan.
                                state["repeat_capture"] = {
                                    "status": "repeat_target_not_reached",
                                    "attempts_made": attempt_number,
                                    "reason": "valid_detection_before_target",
                                }
                                transition("repeat_target_not_reached", t)
                                break
                            if args.extra_views:
                                attempt["acceptance_simulation_time_s"] = (
                                    world.current_time - initial_time
                                )
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
                            if args.camera_inset or args.robot_camera:
                                inset_estimate = inset.InsetEstimate(
                                    observation=observation,
                                    capture_pose=(float(base[0]), float(base[1]), yaw),
                                    pallet_site=target_pickup,
                                    intrinsics=scene_input.intrinsics,
                                    base_from_optical=scene_input.base_from_optical,
                                    attempt_number=attempt_number,
                                )
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
                            # Against the pallet as it stands, read the way the
                            # runner reads it; spawn checks XY and z, not yaw.
                            pallet_position, pallet_orientation = (
                                pallet.get_world_pose()
                            )
                            pallet_position = np.asarray(pallet_position, dtype=float)
                            pallet_orientation = np.asarray(
                                pallet_orientation, dtype=float
                            )
                            state["perception"]["perception_error_actual"] = (
                                G2.site_error(
                                    (
                                        target_pickup.x_m,
                                        target_pickup.y_m,
                                        target_pickup.yaw_rad,
                                    ),
                                    (
                                        pallet_position[0],
                                        pallet_position[1],
                                        yaw_and_tilt(pallet_orientation)[0],
                                    ),
                                )
                            )
                            state["handoff"] = {
                                "attempt_number": attempt_number,
                                "candidate_index": attempt["candidate_index"],
                                "depth_sha256": attempt["depth_sha256"],
                                "loop_step": step,
                                "accepted_pose": [
                                    np.asarray(v, dtype=float).tolist()
                                    for v in capture_diagnostics.accepted_pose
                                ],
                                "linear_velocity_mps": np.asarray(
                                    robot.get_linear_velocity(), dtype=float
                                ).tolist(),
                                "angular_velocity_radps": np.asarray(
                                    robot.get_angular_velocity(), dtype=float
                                ).tolist(),
                                "steering_rad": np.asarray(
                                    robot.get_joint_positions()[steers], dtype=float
                                ).tolist(),
                                "pallet_position_m": pallet_position.tolist(),
                                "pallet_orientation_wxyz": pallet_orientation.tolist(),
                                "simulation_time_s": world.current_time - initial_time,
                                "t_before_capture_s": t,
                            }
                            planning_pickup, state["planning_target_source"] = (
                                G2.planning_target(
                                    args.planning_target, target_pickup, scenario.pickup
                                )
                            )
                            planning_start = time.monotonic()
                            planning_trace = []
                            plans = plan_transport(
                                scenario,
                                planner_config,
                                geometry=geometry,
                                target_pickup=planning_pickup,
                                start_rear=start_rear_pose,
                                return_to=return_to_pose,
                                pickup_bounds=pickup_bounds,
                                travel_config=travel_config,
                                trace=planning_trace,
                            )
                            state.setdefault("planning_traces", []).append(
                                planning_trace
                            )
                            state["planning_wall_s"] += (
                                time.monotonic() - planning_start
                            )
                            state["planning_status"] = plans.status
                            # MissionPlan.status already contains the failing phase.
                            require(plans.success, plans.status)
                            paths.update(
                                {name: getattr(plans, name) for name in mission_stages}
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
                                            # Same rule as the trackers built
                                            # without perception (observe is
                                            # excluded below and keeps its own
                                            # tracker; the return is 0.03 rad).
                                            position_tolerance_m=(
                                                0.03 if name == "observe" else 0.008
                                            ),
                                            yaw_tolerance_rad=(
                                                0.05
                                                if name == "observe"
                                                else 0.03
                                                if name == "return_home"
                                                else 0.02
                                            ),
                                            # Gear-change cusps are not goals (2026-09-26): the next leg
                                            # starts from the measured pose. Final goals keep the rules above.
                                            cusp_position_tolerance_m=0.03,
                                            cusp_yaw_tolerance_rad=0.05,
                                            # Brake and judge the cusp at 8 mm, still accepting 30 mm / 50 mrad
                                            # (docs/plans/2026-10-02-planner-tracker-robustness.md, P2).
                                            cusp_brake_window_m=0.008,
                                            # Optional path speed caps; absent from the settings = off.
                                            max_lateral_acceleration_mps2=settings.get(
                                                "max_lateral_acceleration_mps2"
                                            ),
                                            max_reverse_speed_mps=settings.get(
                                                "max_reverse_speed_mps"
                                            ),
                                            # A stop just past the goal is fine except deeper into
                                            # the pallet: insertion keeps the round 8 mm.
                                            overshoot_tolerance_m=None
                                            if name == "insert"
                                            else 0.03,
                                            stop_speed_mps=0.012,
                                            max_cross_track_error_m=0.35,
                                        ),
                                    )
                                    for name, path in paths.items()
                                    if name != "observe"
                                }
                            )
                            apply_tracker_profile()
                            record_tracker_configs()
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
                            near_leg = None
                            if slam is not None and "near_capture" not in state:
                                # Plan v3.6: see the pallet again where the final
                                # straight begins, so only that straight (and the
                                # insert/extract) runs on held odometry. Curved
                                # approaches drifted 0.02-0.03 rad held (S2 v3.5).
                                near_leg = final_straight_prefix(
                                    paths["approach"], geometry.alignment_straight_m
                                )
                                # Every approach ends in the 0.8 m alignment straight;
                                # not finding it is a failure, not a skip (Codex v3.6 P1).
                                require(near_leg is not None, "near_capture_no_final_straight")
                                state["near_capture"] = {
                                    "status": "pending",
                                    "far_attempt": attempt_number,
                                    "far_estimate_m": state["perception"][
                                        "perception_pickup_estimate_m"
                                    ],
                                    "far_error": state["perception"]["perception_error"],
                                }
                            if near_leg is not None:
                                paths["observe"] = near_leg
                                trackers["observe"] = RearAxlePathTracker(
                                    near_leg.poses,
                                    near_leg.directions,
                                    near_leg.curvatures_inv_m,
                                    trackers["observe"].config,
                                )
                                state["observation_waypoint_selected"] = (
                                    near_leg.poses[-1].tolist()
                                )
                                state["near_capture"]["waypoint"] = near_leg.poses[-1].tolist()
                                phase_started = t
                                inset_status = "근접 재관측 위치로 이동"
                            else:
                                if state.get("near_capture", {}).get("status") == "pending":
                                    # The new estimate moves the straight by a few
                                    # cm; re-draw it from where the truck stands
                                    # instead of a Hybrid A* manoeuvre that would
                                    # run held (Codex v3.6 P2).
                                    new_prefix = final_straight_prefix(
                                        paths["approach"], geometry.alignment_straight_m
                                    )
                                    require(
                                        new_prefix is not None,
                                        "near_capture_no_final_straight",
                                    )
                                    line_start = Pose2D(*new_prefix.poses[-1])
                                    line_end = Pose2D(*paths["approach"].poses[-1])
                                    straight, offsets = straight_from_pose(
                                        Pose2D(float(rear[0]), float(rear[1]), float(rear[2])),
                                        line_start,
                                        line_end,
                                        max_lateral_m=0.08,
                                        max_yaw_rad=0.08,
                                        min_length_m=0.3,
                                    )
                                    state["near_capture"]["offsets_to_new_line"] = offsets
                                    require(straight is not None, f"near_capture_misaligned:{offsets}")
                                    # The box above only bounds the projection; whether
                                    # the approach tracker converges within this straight
                                    # is checked by a dry run with margin (v3.7, Codex
                                    # v3.6 re-review P2).
                                    dry = bicycle_rollout(
                                        straight.poses,
                                        straight.directions,
                                        straight.curvatures_inv_m,
                                        trackers["approach"].config,
                                        rear,
                                    )
                                    state["near_capture"]["dry_run"] = asdict(dry)
                                    require(
                                        dry.status == "arrived"
                                        and dry.position_error_m <= 0.006
                                        and abs(dry.yaw_error_rad) <= 0.015,
                                        f"near_capture_misaligned:dry_run:{asdict(dry)}",
                                    )
                                    paths["approach"] = straight
                                    state["paths"]["approach"] = path_record(straight)
                                    (args.output / "paths.json").write_text(
                                        record_json(state["paths"], indent=2) + "\n"
                                    )
                                    add_path_display(
                                        stage, straight, "Approach", (0.05, 0.45, 1.0)
                                    )
                                    trackers["approach"] = RearAxlePathTracker(
                                        straight.poses,
                                        straight.directions,
                                        straight.curvatures_inv_m,
                                        trackers["approach"].config,
                                    )
                                    state["near_capture"].update(
                                        status="done",
                                        near_attempt=attempt_number,
                                        near_estimate_m=state["perception"][
                                            "perception_pickup_estimate_m"
                                        ],
                                        near_error=state["perception"]["perception_error"],
                                    )
                                transition("approach", t)
                    elif phase == "approach":
                        transition("insert", t)
                    elif phase == "insert":
                        state["insertion_error"] = {
                            "position_m": tracking.position_error_m,
                            "yaw_rad": tracking.yaw_error_rad,
                        }
                        # What the forks actually reach, not the planned target.
                        state["insertion_measured"] = (
                            insertion_geometry.insertion_measures(
                                base,
                                q,
                                float(robot.get_joint_positions()[lift_index[0]]),
                                ppos,
                                pq,
                                geometry.pallet_depth_m,
                            )
                        )
                        # Insertion-axis yaw against the true pallet (fork axis
                        # along the pallet x axis either way round).
                        relative_yaw = yaw_and_tilt(q)[0] - yaw_and_tilt(pq)[0]
                        state["insertion_truth_yaw_rad"] = float(
                            math.atan2(math.sin(2 * relative_yaw), math.cos(2 * relative_yaw)) / 2
                        )
                        if slam is not None:
                            state["slam_error_insert_end"] = slam_error()
                        transition("lift", t)
                    elif phase == "extract":
                        transition("transport", t)
                    elif phase == "transport":
                        transition("lower", t)
                    elif phase == "withdraw":
                        transition("settle", t)
                    elif phase == "return_home":
                        transition("home_settle", t)
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
                if "return_home" in trackers:
                    transition("return_home", t)
                else:
                    transition("complete", t)
                    break
            if phase == "home_settle" and t - phase_started > 1.0:
                state["return_home_error"] = {
                    "position_m": float(
                        np.hypot(
                            truth_rear[0] - scenario.start_rear.x_m,
                            truth_rear[1] - scenario.start_rear.y_m,
                        )
                    ),
                    "yaw_rad": float(
                        math.atan2(
                            math.sin(truth_rear[2] - scenario.start_rear.yaw_rad),
                            math.cos(truth_rear[2] - scenario.start_rear.yaw_rad),
                        )
                    ),
                }
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
            if "chase" in extra_cameras and step % fps_divisor == 0:
                chase_eye, chase_target, chase_yaw = MISSION_VIEWS.chase_pose(
                    base, yaw, chase_yaw, fps_divisor / 120
                )
                chase_look = Gf.Matrix4d().SetLookAt(
                    Gf.Vec3d(*chase_eye),
                    Gf.Vec3d(*chase_target),
                    Gf.Vec3d(0, 0, 1),
                )
                chase_quat = chase_look.GetInverse().ExtractRotationQuat()
                extra_cameras["chase"].set_world_pose(
                    position=chase_eye,
                    orientation=np.array(
                        [chase_quat.GetReal(), *chase_quat.GetImaginary()]
                    ),
                    camera_axes="usd",
                )
            if slam is not None:
                slam["last_command"] = requested_speed
            stepper["fn"](False)
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
        if phase in ("repeat_capture_done", "repeat_target_not_reached"):
            return
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
        if slam is not None:
            # First, so a video shutdown failure cannot lose the SLAM record.
            slam["link"].close()
            (args.output / "slam_records.json").write_text(
                record_json(slam["records"]) + "\n"
            )
            control = np.asarray(slam["control"], dtype=float).reshape(-1, 7)
            np.save(args.output / "slam_control.npy", control)
            error = np.hypot(control[:, 1] - control[:, 4], control[:, 2] - control[:, 5])
            yaw_error = np.abs(np.angle(np.exp(1j * (control[:, 3] - control[:, 6]))))
            statuses = [record["status"] for record in slam["records"]]
            state["slam_summary"] = {
                "control_samples": int(len(control)),
                "raw_position_rmse_m": float(np.sqrt(np.mean(error**2)))
                if len(error)
                else None,
                "raw_position_max_m": float(error.max()) if len(error) else None,
                "raw_yaw_rmse_rad": float(np.sqrt(np.mean(yaw_error**2)))
                if len(error)
                else None,
                "raw_yaw_max_rad": float(yaw_error.max()) if len(error) else None,
                "scans": len(statuses),
                "processed": statuses.count("processed"),
                "skipped": statuses.count("skipped"),
                "holds": slam["holds"],
                "link_failures": statuses.count("link_failure"),
                "pending_release_at_end": slam["pending_release"],
                "final_mode": slam["tracker"].mode,
            }
        if frame_log is not None:
            frame_log.close()
        (args.output / "frame_audit.json").write_text(
            record_json(frame_audit, indent=2) + "\n"
        )
        if video_frames:
            (args.output / "video_frames.json").write_text(
                record_json(
                    {
                        "fps": args.fps,
                        "clock": "Isaac simulation time since the first recorded step",
                        "overview_video": "transport.mp4",
                        "overview_floor_points": state.get("overview_floor_points"),
                        "robot_camera": {
                            "mount": (
                                "perception camera (synthetic baseline_0p50)"
                                if args.perception_mount == "legacy"
                                else f"perception camera ({args.perception_mount})"
                            ),
                            "resolution": [
                                perception_calibration.width,
                                perception_calibration.height,
                            ],
                            "depth_video_range_m": list(DEPTH_VIDEO_RANGE_M),
                            "timing": "video only, not a freshness-checked capture",
                        },
                        "frames": video_frames,
                    },
                    indent=1,
                )
                + "\n"
            )
        if slam_log is not None and slam_log["scan_stamps_s"]:
            write_slam_record(args, state, scenario, factory, slam_log, lidar_config)
        # Records first, encoders last: a slow ffmpeg shutdown must neither
        # lose the records above nor replace the run's own failure reason.
        failing = sys.exc_info()[0] is not None
        video_results = []
        encoders = ([("transport", encoder)] if encoder is not None else []) + list(
            extra_encoders.items()
        )
        for name, process in encoders:
            try:
                process.stdin.close()
                video_results.append((name, process.wait(timeout=60)))
            except (OSError, subprocess.TimeoutExpired) as exc:
                process.kill()
                state.setdefault("video_shutdown_errors", []).append(f"{name}: {exc!r}")
                video_results.append((name, None))
        if not failing:
            for name, exit_code in video_results:
                require(exit_code == 0, f"{name} video encoding failed")


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
    from chassis_contract import require_curvature_within_model
    from insertion_geometry import read_carriage_limit_m, read_drive_geometry_m

    args.drive_geometry = read_drive_geometry_m(
        args.forklift_urdf, settings["max_wheel_rate_rad_s"]
    )
    args.carriage_limit_m = read_carriage_limit_m(args.forklift_urdf)
    require_curvature_within_model(settings, args.drive_geometry)
    state = {
        "success": False,
        # Overwritten once the scene build starts; a result still at "startup"
        # is an environment failure under the G2 rerun rule.
        "phase": "startup",
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
        "chassis_model": {
            "forklift_urdf": str(args.forklift_urdf),
            "drive_geometry": asdict(args.drive_geometry),
            "carriage_limit_m": args.carriage_limit_m,
            "axle_to_fork_tip_m": args.axle_to_fork_tip_m,
        },
        "python": sys.version,
        "arguments": {
            k: (
                asdict(v)
                if k in {"pallet_geometry_loaded", "drive_geometry"}
                else str(v)
                if isinstance(v, Path)
                else v
            )
            for k, v in vars(args).items()
            if (k != "extra_views" or v)
            and (
                k not in {"quarter_eye", "quarter_target", "quarter_focal"}
                or (v is not None and (k != "quarter_focal" or v != 2.5))
            )
        },
    }
    scene_root = Path(args.base_scene).resolve().parent
    state["scene_sha256"] = {
        str(path.relative_to(scene_root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(scene_root.rglob("*"))
        if path.is_file() and path.suffix in {".usd", ".usda", ".usdc"}
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
        if app is not None and "render_mode" in state:
            import carb

            state["render_mode"]["at_end"] = carb.settings.get_settings().get(
                "/rtx/rendermode"
            )
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
