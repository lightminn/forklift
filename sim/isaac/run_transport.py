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
import os
import subprocess
import sys
import time
import traceback
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

EXIT_CLEARANCE_M = 0.08


# Braking into the mid-approach standing frame (a planned stop).
MID_STILL_DECEL_MPS2 = 0.8
# Pull-away limit for the wheel target with the obstacle layer acting.
COMMAND_ACCEL_MPS2 = 0.8
# Plan D4 delta (2026-10-06, user decision): a leg whose blocked-path replan
# fails backs off this far, this slowly, at most this many times, then replans.
BACKOFF_M = 0.30
BACKOFF_SPEED_MPS = 0.15
BACKOFF_PER_LEG = 2
# The second docking round stops this far before the goal (dock_at_delivery_straight).
ROUND2_KEEP_M = 0.6
# An observation waypoint missed by at most this much, standing, is arrival.
OBSERVE_ARRIVAL_M = 0.10
OBSERVE_ARRIVAL_YAW_RAD = 0.10


def same_path(new, current, remaining_m: float, tol_m: float = 0.01, yaw_tol_rad: float = 0.02) -> bool:
    """True when a replan returns the path the truck is already on: every new
    pose lies within tol_m of the current path's remaining part, measured to
    its segments, not its samples (Codex checkpoint 8: the same straight
    sampled from another start read as different), with the heading within
    yaw_tol_rad and the lengths within 2 tol_m (plan D4: that is a wait). Only
    a near-identical path: a 2.5 cm detour already misses the cells that block
    the old one (Codex checkpoint 7 P2), so it must not be thrown away."""
    a = np.asarray(new.poses, dtype=float)
    b = np.asarray(current.poses, dtype=float)
    if len(a) < 2 or len(b) < 2:
        return False
    seg = np.hypot(*np.diff(b[:, :2], axis=0).T)
    from_end = np.concatenate((np.cumsum(seg[::-1])[::-1], [0.0]))
    keep = from_end <= remaining_m + 2 * tol_m
    first = max(int(np.argmax(keep)) - 1, 0)  # the segment the truck is on
    b = b[first:]
    length = float(np.sum(np.hypot(*np.diff(a[:, :2], axis=0).T)))
    if len(b) < 2 or abs(length - remaining_m) > 2 * tol_m:
        return False
    p0, p1 = b[:-1, :2], b[1:, :2]
    v = p1 - p0
    vv = np.maximum(np.sum(v * v, axis=1), 1e-12)
    rel = a[:, None, :2] - p0[None]
    u = np.clip(np.sum(rel * v[None], axis=2) / vv[None], 0.0, 1.0)
    near = p0[None] + u[..., None] * v[None]
    d = np.hypot(*(a[:, None, :2] - near).transpose(2, 0, 1))
    k = np.argmin(d, axis=1)
    yaw_b = b[:-1, 2][k]
    dyaw = np.abs(np.arctan2(np.sin(a[:, 2] - yaw_b), np.cos(a[:, 2] - yaw_b)))
    return bool(np.all(d[np.arange(len(a)), k] <= tol_m) and np.all(dyaw <= yaw_tol_rad))


def box_meets_obb(box_centre, box_half, obb_centre, obb_axes, obb_half) -> bool:
    """Closed overlap of an axis-aligned box and an oriented box (15-axis SAT):
    touching counts. obb_axes holds the oriented box's unit axes as columns."""
    a_half = np.asarray(box_half, dtype=float)
    b_half = np.asarray(obb_half, dtype=float)
    rot = np.asarray(obb_axes, dtype=float)  # A <- B
    t = np.asarray(obb_centre, dtype=float) - np.asarray(box_centre, dtype=float)
    abs_rot = np.abs(rot) + 1e-12
    for i in range(3):  # the box's axes
        if abs(t[i]) > a_half[i] + abs_rot[i] @ b_half:
            return False
    for j in range(3):  # the oriented box's axes
        if abs(t @ rot[:, j]) > a_half @ abs_rot[:, j] + b_half[j]:
            return False
    for i in range(3):  # cross products of the two
        for j in range(3):
            i1, i2 = (i + 1) % 3, (i + 2) % 3
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            ra = a_half[i1] * abs_rot[i2, j] + a_half[i2] * abs_rot[i1, j]
            rb = b_half[j1] * abs_rot[i, j2] + b_half[j2] * abs_rot[i, j1]
            if abs(t[i2] * rot[i1, j] - t[i1] * rot[i2, j]) > ra + rb:
                return False
    return True


def load_perception_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


DOCKING_RETRY = load_perception_module(
    "run_transport_docking_retry", Path(__file__).with_name("docking_retry.py")
)
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
ESTOP = load_perception_module(
    "run_transport_estop_probe", Path(__file__).with_name("estop_probe.py")
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
        "--delivery-straight-m",
        type=float,
        default=0.70,
        help="Length of the final delivery straight (plan v3.8: 1.5 for the "
        "docking runs and their ground-truth controls; 0.70 is the recorded default).",
    )
    parser.add_argument(
        "--alignment-straight-m",
        type=float,
        default=0.80,
        help="Length of the final approach straight; the near capture happens at "
        "its start (priority-5 D5: 2.1 puts the carriage camera 2.59 m from the "
        "face, enough for the depth pocket check).",
    )
    parser.add_argument(
        "--withdrawal-m",
        type=float,
        default=0.55,
        help="Reverse distance after the drop. Priority-5 L3c uses 0.75: at 0.55 the "
        "blade tips end 0.19 m from the pallet face, inside its grid swelling, and "
        "the stop envelope beside them holds every return start.",
    )
    parser.add_argument(
        "--pocket-check",
        action="store_true",
        help="Priority-5 D5 depth pocket check: the drive permission also acts on "
        "the approach straight and the insertion, waiving the grid inside the "
        "estimated pallet's region only where depth certified the truck's stop.",
    )
    parser.add_argument(
        "--insertion-reserve-m",
        type=float,
        default=0.016,
        help="Insertion reserve behind the carriage limit (ADR 0004 D3 policy "
        "0.016 since 2026-10-08, ADR 0004 D3 amendment). Other values are for diagnostic sweeps and are recorded.",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--obstacles", type=int, default=4)
    parser.add_argument(
        "--min-obstacle-height-m",
        type=float,
        default=None,
        help="Single-LiDAR operating assumption (user, 2026-10-07): remove every "
        "floor prop (bay props, storage pallets with their loads, clutter) whose "
        "top is below this height. The kept props keep their places.",
    )
    parser.add_argument(
        "--prism-colliders",
        action="store_true",
        help="Plan v10 D0: each kept floor prop collides as one invisible box of its "
        "floor rectangle up to its top (meshes stay visible, their collision off), "
        "so the single 1.05 m LiDAR meets the whole footprint at every height.",
    )
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
        "--d7-planner",
        action="store_true",
        help="Plan with the D7 CPU judgement's adopted options (Reeds-Shepp goal "
        "connection, 4 m connection cap; pallet_mission.D7_PLANNER_OPTIONS) -- the "
        "'D7 on' arm of the Isaac D7 off/on comparison.",
    )
    parser.add_argument(
        "--d7-docking",
        action="store_true",
        help="Plan D7c (delivery only): at a stalled docking stop dock in place; when "
        "the docking straight cannot be accepted, or a docked straight needs a "
        "recovery, re-approach the docking line's start (at most 2 retries) and match "
        "again. Needs --slam-feedback.",
    )
    parser.add_argument(
        "--destination-prior-error-m",
        type=float,
        default=0.0,
        help="Plan D7c δ runs: plan to (and seed docking from) a destination moved this "
        "far along its own lateral axis; the scene, the docking reference scan and the "
        "evaluation keep the true destination.",
    )
    parser.add_argument(
        "--return-home",
        action="store_true",
        help="After unloading, drive back to the rear-axle pose the mission "
        "started from. The delivered pallet becomes an obstacle for that leg.",
    )
    parser.add_argument(
        "--estop-probe",
        type=ESTOP.parse_spec,
        default=None,
        help="Priority-5 P0a: comma-separated phase@seconds. At each, once the "
        "truck is moving in that phase, every wheel target goes to zero with "
        "the steering held; the stop is recorded from ground truth and the "
        "run carries on. Measurement runs only.",
    )
    parser.add_argument(
        "--obstacle-layer",
        type=Path,
        default=None,
        help="Priority-5 obstacle layer config (config/obstacle_layer.yaml): cast "
        "the obstacle LiDARs every scan tick, keep the rolling grid and the drive "
        "permission. Alone it only records what the permission would do.",
    )
    parser.add_argument(
        "--obstacle-act",
        action="store_true",
        help="With --obstacle-layer: limit the commanded speed by the drive "
        "permission in the travel phases (not the docking straights).",
    )
    parser.add_argument(
        "--grid-planning",
        action="store_true",
        help="With --obstacle-act: plan on the LiDAR grid instead of the ground-truth "
        "props, the unrecognised pallet as the pickup-zone prior and the recognised "
        "one as its estimate; stop, replan and resume when the path is blocked.",
    )
    parser.add_argument(
        "--slam-map-dir",
        type=Path,
        default=None,
        help="Priority-5 D3 delta: the SLAM bridge's output directory; its latest_map.npz "
        "joins the planning grid as a static layer (planning only, never the permission).",
    )
    parser.add_argument(
        "--new-obstacles",
        type=Path,
        default=None,
        help="Priority-5 L4 scenario rules (sim/isaac/new_obstacles.py): boxes that "
        "appear or disappear along the active path, and obstacle sensors that go silent.",
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
    parser.add_argument(
        "--slam-noise-spec",
        nargs="?",
        const="max",
        default=None,
        choices=("max", "3sigma"),
        help="SLAM feedback noise from the sensor datasheets' maximum error -- RPLIDAR A2M12 range "
        "row (1 / 2 / 2.5 %% of range) and MT6701 1.5 deg for steering -- read as one sigma "
        "('max') or as three sigma ('3sigma'); wheel rates keep 0.2 rad/s (no datasheet value).",
    )
    parser.add_argument("--slam-reply-timeout", type=float, default=90.0)
    parser.add_argument(
        "--perception-mount",
        choices=("legacy", "carriage_low", "carriage_low_measured"),
        default="legacy",
        help="legacy: base (0.75, 0, 0.50), tilt 0 (every recorded run). "
        "carriage_low: on fork_carriage at base (0.559, 0, 0.27), tilt 0.10 rad, "
        "provisional chassis only, captures only at lift 0; carriage_low_measured: the same "
        "mount at base x 0.619 for dls08_measured "
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
        from forklift_core.planning.observation_viewpoints import DEFAULT_OBSERVATION_WAYPOINTS

        args.observation_waypoints = [list(w) for w in DEFAULT_OBSERVATION_WAYPOINTS]
    if not args.observation_waypoints:
        parser.error("--observation-waypoints requires at least one candidate")
    if args.slam_feedback is not None:
        if not (args.record_slam and args.use_perception):
            parser.error("--slam-feedback needs --record-slam and --use-perception")
        if args.planning_target == "oracle_nominal":
            parser.error("--slam-feedback forbids --planning-target oracle_nominal")
    if args.slam_noise_seed is not None and args.slam_feedback is None:
        parser.error("--slam-noise-seed requires --slam-feedback")
    if args.perception_mount in ("carriage_low", "carriage_low_measured"):
        if not args.use_perception:
            parser.error("--perception-mount carriage_low needs --use-perception")
        if args.perception_camera_axes != "ros":
            parser.error("--perception-mount carriage_low needs ros camera axes")
        if args.perception_mount == "carriage_low" and "dls08_provisional" not in str(args.forklift_urdf):
            parser.error(
                "--perception-mount carriage_low is defined for dls08_provisional only"
            )
        if args.perception_mount == "carriage_low_measured" and "dls08_measured" not in str(args.forklift_urdf):
            parser.error("--perception-mount carriage_low_measured is defined for dls08_measured only")
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
    if args.obstacle_layer is not None and not args.record_slam:
        parser.error("--obstacle-layer needs --record-slam (the scan tick lives there)")
    if args.obstacle_act and args.obstacle_layer is None:
        parser.error("--obstacle-act needs --obstacle-layer")
    if args.grid_planning and not args.obstacle_act:
        parser.error("--grid-planning needs --obstacle-act")
    if args.grid_planning and not (args.use_perception and args.planning_target == "perception"):
        # Without recognition, or with an oracle target, the planner reads the
        # pickup pallet's true pose (Codex checkpoint 14): grid planning plans
        # on perception only (plan audit table).
        parser.error("--grid-planning needs --use-perception and --planning-target perception")
    if args.grid_planning and args.runtime_viewpoints:
        # The runtime viewpoints count the pickup pallet's true rectangle as
        # occupied: a ground-truth planning input the grid plan must not have
        # (plan audit table, 2026-10-06 audit).
        parser.error("--runtime-viewpoints uses the true pallet rectangle; not with --grid-planning")
    if args.pocket_check and not args.obstacle_act:
        # Without the layer the check is never asked, without acting a zero
        # limit never reaches the wheels (Codex L3c P1).
        parser.error("--pocket-check needs --obstacle-layer and --obstacle-act")
    if args.new_obstacles is not None and args.obstacle_layer is None:
        parser.error("--new-obstacles needs --obstacle-layer")
    if args.d7_docking and args.slam_feedback is None:
        parser.error("--d7-docking retries the SLAM docking match; it needs --slam-feedback")
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


def path_stats(path) -> dict:
    """Diagnostics of a (re)plan for the priority-5 D7d record: its length, the gear
    changes on it, and every search the ladder ran (status, expansions, pruned and
    superseded nodes)."""
    directions = np.asarray(getattr(path, "directions", ()), dtype=int)
    return {
        "length_m": float(getattr(path, "length_m", 0.0)) if getattr(path, "success", False) else None,
        "gear_changes": int(np.count_nonzero(np.diff(directions))) if len(directions) > 1 else 0,
        "search_attempts": [list(entry) for entry in getattr(path, "search_attempts", ())],
    }


def run(app, args: argparse.Namespace, settings: dict, state: dict) -> None:
    """Construct and execute one immutable seeded scenario; state keeps evidence."""
    import omni.usd
    from insertion_geometry import InsertionGeometry, read_fork_blades_m
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
        add_prism_colliders,
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
        FootprintCollisionChecker,
        PlanResult,
        Rectangle,
        collision_free_pose,
    )
    from forklift_core.control.drive_permission import arc_poses as ARC_POSES
    from forklift_core.control.drive_permission import parts_of as PARTS_OF
    from forklift_core.planning.geometry import Bounds

    # The pallet's solids (the canonical assembly: boards, blocks, stringers) a
    # bar in a pocket is judged against (Codex checkpoint 10 P2: a column model
    # missed the stringers and asymmetric blocks).
    from forklift_core.perception.pallet_geometry import pallet_boxes as PALLET_BOXES_OF

    PALLET_BOXES = PALLET_BOXES_OF(args.pallet_geometry_loaded)
    from forklift_core.control.drive_permission import shape_meets as SHAPE_MEETS
    from forklift_core.control.rollout import bicycle_rollout
    from forklift_core.planning import Pose2D as PlanningPose
    from forklift_core.planning.pallet_mission import (
        SyntheticMissionGeometry,
        make_scenario,
        D7_PLANNER_OPTIONS,
        SearchBudget,
        make_transport_planner_config,
        plan_docking_reapproach,
        plan_transport,
        final_straight_prefix,
        straight_from_pose,
        plan_return_leg,
        plan_transport_leg,
        site_poses,
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
        delivery_straight_m=args.delivery_straight_m,
        alignment_straight_m=args.alignment_straight_m,
        withdrawal_m=args.withdrawal_m,
    )
    state["insertion_reserve_m"] = args.insertion_reserve_m
    # The truck unloaded is its body and two fork blades, not their hull (D4).
    unloaded_shape = [
        (Footprint(geometry.axle_to_fork_tip_m - geometry.carriage_limit_m, geometry.unloaded_footprint.rear_m,
                   geometry.unloaded_footprint.half_width_m), 0.0, 0.0)
    ] + [
        (Footprint((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2), (y0 + y1) / 2, (x0 + x1) / 2 + abs(args.rear_axle_offset_m))
        for x0, x1, y0, y1 in read_fork_blades_m(args.forklift_urdf)
    ]
    planner_config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=30000,
        **(D7_PLANNER_OPTIONS if args.d7_planner else {}),
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
    estop = ESTOP.EstopProbe(list(args.estop_probe)) if args.estop_probe else None
    state["estop_probes"] = estop.records if estop is not None else None
    state["observe_replans"] = []
    observe_stall_ticks = 0
    slam_stall_ticks = 0
    dock_stop_ticks = 0
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
    if args.min_obstacle_height_m is not None:
        from forklift_core.planning.factory_layout import drop_low_obstacles

        kept_before = len(scenario.props)
        scenario, factory, removed = drop_low_obstacles(
            scenario, factory, args.min_obstacle_height_m
        )
        state["low_obstacles_removed"] = {
            "min_top_m": args.min_obstacle_height_m,
            "props_before": kept_before,
            "props_removed": removed,
            "props_kept": len(scenario.props),
            "loads_kept": None if factory is None else len(factory.loads),
        }
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
    mesh_collision = not args.prism_colliders
    if factory is None:
        state["props"] = add_props(stage, app, scenario.props, offsets, mesh_collision=mesh_collision)
    else:
        bay_props = scenario.props[: len(scenario.props) - len(factory.work_items)]
        state["props"] = add_props(stage, app, bay_props, offsets, mesh_collision=mesh_collision)
        state["factory_items"] = add_factory_items(
            stage, app, factory.work_items, factory.loads, offsets, mesh_collision=mesh_collision
        )
    if args.prism_colliders:
        # Plan v10 D0: every kept prop collides as its floor rectangle raised to its top.
        from forklift_core.planning.factory_layout import prism_columns

        state["prism_colliders"] = add_prism_colliders(stage, prism_columns(scenario, factory))
        # The overview has to see the whole hall from above the roof line.
        state["hidden_overhead_prims"] = len(hide_overhead(stage))
    state["destination_marker"] = add_destination(
        stage, scenario.destination.x_m, scenario.destination.y_m
    )
    # The scene above and scenario.json keep the true destination; with a prior
    # error (plan D7c δ runs) everything that plans from here on, and the docking
    # match's prior, sees the moved one. The reference scan and the evaluation use
    # truth_destination.
    truth_destination = scenario.destination
    if args.destination_prior_error_m:
        scenario = replace(
            scenario,
            destination=DOCKING_RETRY.destination_with_prior_error(
                scenario.destination, args.destination_prior_error_m
            ),
        )
        state["destination_prior"] = {
            "error_m": args.destination_prior_error_m,
            "planning_destination": asdict(scenario.destination),
            "truth_destination": asdict(truth_destination),
        }
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
        if args.pocket_check:
            # The D5 depth check's own camera: same mount and intrinsics, but the
            # near clipping plane at the D435i's minimum depth (0.28 m, datasheet,
            # its highest resolution -- the conservative figure) instead of the
            # 1.0 m default the recogniser was validated with. At 1.0 m the floor
            # before the face came back empty and never certified (L3c v31 seed 2:
            # raw depth min 1.000001 m, the lower third of the image inf).
            pocket_camera = Camera(
                prim_path=mount_parent + "/PocketDepthCamera",
                frequency=-1,
                resolution=(perception_calibration.width, perception_calibration.height),
            )
            pocket_camera.set_local_pose(
                translation=np.asarray(perception_mount.translation_m),
                orientation=np.asarray(adapter.xyzw_to_wxyz(mount_xyzw)),
                camera_axes=args.perception_camera_axes,
            )
            pocket_camera.set_projection_mode("perspective")
            pocket_camera.set_lens_distortion_model("pinhole")
            pocket_camera.set_focal_length(1.0)
            pocket_camera.set_horizontal_aperture(
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
        if args.pocket_check:
            pocket_camera.initialize()
            pocket_camera.add_distance_to_image_plane_to_frame()
            _, pocket_far_m = pocket_camera.get_clipping_range()
            pocket_camera.set_clipping_range(near_distance=0.28, far_distance=pocket_far_m)
            k_pocket = adapter.read_isaac_intrinsics(pocket_camera).integer_index
            k_percep = adapter.read_isaac_intrinsics(perception_camera).integer_index
            require(
                all(abs(getattr(k_pocket, n) - getattr(k_percep, n)) <= 1e-6 for n in ("fx", "fy", "cx", "cy", "width", "height")),
                "pocket camera intrinsics differ from the perception camera's",
            )
            state["pocket_camera"] = {
                "clipping_range_m": list(map(float, pocket_camera.get_clipping_range())),
                "intrinsics": asdict(k_pocket),
            }
        perception_display = None
        if "perception" in args.extra_views or args.robot_camera:
            # The display camera: the same mount and intrinsics with a 0.05 m
            # near plane, so the picture is not cut open when the pallet comes
            # closer than the recogniser's 1.0 m (the robot-camera video showed
            # the pallet's inside, 2026-10-06 user report). Never a sensor.
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
            if "perception" in args.extra_views:
                extra_cameras["perception"] = perception_display
            state.setdefault("extra_views", {})
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
    obstacle = None  # the priority-5 obstacle layer
    if args.obstacle_layer is not None:
        import obstacle_layer as obstacle_module

        from forklift_core.perception.obstacle_grid import AgeErrorTable

        layer_config = obstacle_module.load_layer_config(args.obstacle_layer)
        age_table = json.loads(Path(layer_config["odometry_age"]).read_text())
        obstacle = {
            "layer": obstacle_module.ObstacleLayer(
                layer_config,
                hall=scenario.bounds,
                error_table=AgeErrorTable(
                    tuple(age_table["ages_s"]),
                    tuple(age_table["cumulative_position_m"]),
                    tuple(age_table["cumulative_yaw_rad"]),
                ),
                unloaded=geometry.unloaded_footprint,
                loaded=geometry.loaded_footprint,
                body_front_m=geometry.axle_to_fork_tip_m - geometry.carriage_limit_m,
                rear_axle_x_in_base_m=-abs(args.rear_axle_offset_m),
                noise_seed=args.seed,
                blades_rear_m=tuple(
                    (x0 + abs(args.rear_axle_offset_m), x1 + abs(args.rear_axle_offset_m), y0, y1)
                    for x0, x1, y0, y1 in read_fork_blades_m(args.forklift_urdf)
                ),
            ),
            "version": 0,
            "applied": None,
            "scans": [],
            "ticks": {},
            "slowed": {},
            "run_wall_start": time.time(),
            "reasons": {},
            "events": 0,
            "unpermitted": [],
            "min_allowed": {},
        }
        state["obstacle_layer"] = {
            "config": str(args.obstacle_layer),
            "act": bool(args.obstacle_act),
            "sensors": [
                {"name": sn.name, "xyz_m": sn.xyz_m, "yaw_rad": sn.yaw_rad, "may_clear": sn.may_clear}
                for sn in obstacle["layer"].sensors
            ],
        }

    grid_planning = bool(args.grid_planning)
    if obstacle is not None and obstacle["layer"].known_enabled and not grid_planning:
        # The recognised pallet reaches the layer through the grid plan's pickup
        # (Codex stage-1 P1-5); without it the permission would lose the pallet.
        raise SystemExit("known_pallets needs --grid-planning")

    # Video overlay record (display only): every path the truck was given, with
    # why, and the obstacle grid's OCCUPIED cells every 0.5 s -- so the video
    # can show the first plan and each replan on the live grid.
    overlay = {"plan_id": None, "plan_history": [], "grid_t": -1e9, "grids": []}

    def record_video_overlay(t_now: float) -> None:
        from forklift_core.perception.obstacle_grid import OCCUPIED as GRID_OCCUPIED

        current = paths.get(phase)
        if current is not None and id(current) != overlay["plan_id"]:
            overlay["plan_id"] = id(current)
            why = "plan"
            if obstacle.get("backoff", {}).get("phase") == phase:
                why = "backoff"
            elif obstacle.get("replans") and abs(obstacle["replans"][-1]["time_s"] - t_now) < 0.6:
                why = "replan"
            elif obstacle.get("live_plans") and abs(obstacle["live_plans"][-1]["time_s"] - t_now) < 0.6:
                why = "live"
            poses = np.asarray(current.poses, dtype=float)
            step_ = max(1, len(poses) // 400)
            overlay["plan_history"].append(
                {"time_s": float(t_now), "phase": phase, "why": why,
                 "poses": np.round(poses[::step_, :2], 3).tolist() + [np.round(poses[-1, :2], 3).tolist()]}
            )
        snap_ = obstacle["layer"].snapshot
        if snap_ is not None and t_now - overlay["grid_t"] >= 0.5:
            overlay["grid_t"] = t_now
            cells_ = np.argwhere(snap_.state == GRID_OCCUPIED).astype(np.int32)
            overlay["grids"].append((float(t_now), float(snap_.origin_x_m), float(snap_.origin_y_m),
                                     float(snap_.resolution_m), cells_))
    new_obstacles = None
    # Priority-5 D5 depth pocket check: built at the near capture, fed 10 Hz
    # carriage frames on the approach straight and the insertion.
    pocket = {"check": None, "history": [], "capture": None, "frames_read": 0}

    def build_pocket_check(axis_yaw: float, rear_now, base_now) -> None:
        """The D5 pocket check, at the near (stand-off) capture: the final estimate,
        the approach straight's heading, and that capture as its first frame."""
        import pocket_check as pocket_module
        from forklift_core.perception.pocket_clearance import DepthCamera

        scene = pocket["capture"]
        require(scene is not None, "pocket_check_without_capture")
        est = state["perception"]["perception_pickup_estimate_m"]
        k = scene.intrinsics
        offset = abs(args.rear_axle_offset_m)
        check = pocket_module.PocketCheck(
            estimate=(float(est["x_m"]), float(est["y_m"]), float(est["yaw_rad"])),
            axis_yaw=axis_yaw,
            pallet_geometry=args.pallet_geometry_loaded,
            blades_rear=tuple(
                (x0 + offset, x1 + offset, y0, y1) for x0, x1, y0, y1 in read_fork_blades_m(args.forklift_urdf)
            ),
            body_front_m=geometry.axle_to_fork_tip_m - geometry.carriage_limit_m,
            body_rear_m=geometry.unloaded_footprint.rear_m,
            body_half_width_m=geometry.unloaded_footprint.half_width_m,
            insertion_depth_m=geometry.axle_to_fork_tip_m + geometry.pallet_depth_m / 2 - geometry.inserted_offset_m,
            # base_link height: a fixed chassis dimension, not a pose estimate.
            base_z_m=float(base_now[2]),
            base_from_optical=(np.asarray(scene.base_from_optical.rotation, dtype=float),
                               np.asarray(scene.base_from_optical.translation_m, dtype=float)),
            rear_axle_offset_m=offset,
            camera=DepthCamera(k.fx, k.fy, k.cx, k.cy, k.width, k.height),
            stopping=obstacle["layer"].permission.config.stopping,
            noise_seed=args.seed,
        )
        now_s = world.current_time - initial_time
        first = check.add_frame(now_s, scene.depth_m, tuple(float(v) for v in rear_now))
        pocket["check"] = check
        state["pocket_check"] = {"built_s": now_s, "axis_yaw_rad": axis_yaw, "first_frame": first}
        require(check.valid, f"pocket_check_refused:{check.invalid_reason}")
        obstacle["layer"].exempt = check.region_control()
        obstacle["layer"].depth_support = check.depth_free_cells

    def read_pocket_frame() -> None:
        """One 10 Hz carriage depth frame for the pocket check, placed at the
        control pose of its rendering time (frames lag the step)."""
        check = pocket["check"]
        if new_obstacles is not None and "pocket_camera" in new_obstacles["schedule"].silenced:
            # N13: the depth camera falls silent; the check's 0.2 s freshness
            # runs out on its own (plan D6).
            pocket["silenced_frames"] = pocket.get("silenced_frames", 0) + 1
            return
        raw = pocket_camera.get_depth()
        if raw is None:
            return
        frame = pocket_camera.get_current_frame()
        rendered = frame.get("rendering_time") if isinstance(frame, dict) else None
        if not rendered or not math.isfinite(float(rendered)):
            pocket["frames_without_time"] = pocket.get("frames_without_time", 0) + 1
            return  # no acquisition time: never a certification (Codex L3c P1)
        stamp = float(rendered) - initial_time
        if stamp > world.current_time - initial_time + 1e-6 or stamp <= check.last_new_s:
            pocket["frames_bad_time"] = pocket.get("frames_bad_time", 0) + 1
            return
        history = pocket["history"]
        times = np.array([h[0] for h in history])
        k = int(np.clip(np.searchsorted(times, stamp), 1, len(history) - 1)) if len(history) > 1 else 0
        if len(history) > 1:
            (t0, p0), (t1, p1) = history[k - 1], history[k]
            a = float(np.clip((stamp - t0) / (t1 - t0), 0.0, 1.0)) if t1 > t0 else 1.0
            rear_at = tuple(p0[i] + a * (p1[i] - p0[i]) for i in range(3))
        else:
            rear_at = history[-1][1]
        k_ = pocket["capture"].intrinsics
        depth, _ = adapter.normalize_depth(np.asarray(raw).reshape(k_.height, k_.width))
        # The frame's pose uncertainty grows with the speed the truck moves at
        # (pixel lag, L3c v7): the odometry speed, never the truth -- not the
        # command, which stays up while the permission holds the truck (L3c v11:
        # a standing truck kept eroding the strip against the face).
        speed_now = abs(slam["odom_speed"]) if slam is not None else abs(requested_speed)
        yaw_rate_now = abs(slam.get("odom_yaw_rate", 0.0)) if slam is not None else 0.0
        record = check.add_frame(stamp, depth, rear_at, speed_mps=speed_now, yaw_rate_rps=yaw_rate_now,
                                 lift_m=float(robot.get_joint_positions()[lift_index[0]]))
        if pocket.get("await_still_frame") and record.get("new") and speed_now < 0.01 and yaw_rate_now < 0.01:
            pocket["await_still_frame"] = False
            pocket["still_frame_s"] = stamp
            # Diagnostics: the standing frames as the check saw them.
            k_still = pocket.setdefault("still_frames", 0) + 1
            pocket["still_frames"] = k_still
            np.savez_compressed(
                args.output / f"pocket_still_{k_still}.npz",
                depth=np.asarray(depth, dtype=np.float32), raw=np.asarray(raw, dtype=np.float32), stamp=stamp,
                rear=np.asarray(rear_at, dtype=float), estimate=np.asarray(check.estimate, dtype=float),
                axis_yaw=check.axis_yaw,
            )
        pocket["frames_read"] += 1
    if args.new_obstacles is not None:
        new_module = load_perception_module(
            "run_transport_new_obstacles", Path(__file__).with_name("new_obstacles.py")
        )
        new_obstacles = {"schedule": new_module.Schedule(new_module.load_events(args.new_obstacles)), "rects": {}}
        new_obstacles["band_overlaps"] = []
        state["new_obstacles"] = {
            "rules": str(args.new_obstacles),
            "log": new_obstacles["schedule"].log,
            # The shadow-band memory's scope excludes objects appearing inside
            # the band (D4 delta 2026-10-05): a spawn whose collider meets the
            # band's cells at spawn time is outside the scenario's validity.
            "band_overlaps": new_obstacles["band_overlaps"],
        }
    if obstacle is not None:
        import planar_lidar as obstacle_lidar

        # The first obstacle scan, from the start pose, before any plan: the
        # grid the first plan sees (priority-5 L3b). Start = map = odom.
        prime_base, prime_q = robot.get_world_pose()
        prime_raw = {}
        for sensor in obstacle["layer"].sensors:
            origin, directions = obstacle_lidar.laser_rays_world(
                prime_base, prime_q, obstacle_lidar.LaserMount(sensor.xyz_m, sensor.yaw_rad),
                obstacle["layer"].beam_angles,
            )
            prime_raw[sensor.name] = (
                *obstacle_lidar.cast_scan_flags(
                    origin, directions, obstacle["layer"].range_max_m, own_prefixes=(obstacle_lidar.SELF_PREFIX,)
                ),
                obstacle_module.beam_limits(origin, directions, band_top_m=obstacle["layer"].band_top_m,
                                             band_bottom_m=obstacle["layer"].band_bottom_m),
            )
        start_pose = (scenario.start_rear.x_m, scenario.start_rear.y_m, scenario.start_rear.yaw_rad)
        obstacle["layer"].add_scans(0.0, prime_raw, odom_rear=start_pose, loaded=False)
        obstacle["last_stamp"] = 0.0
        obstacle["applied"] = (0.0, 0.0, 0.0)
        obstacle["replans"] = []
        obstacle["plans"] = []
    # Pickup-zone prior (plan D0): where the unrecognised pallet may stand --
    # the layout's placement distribution (BAY_PICKUP_ZONE) grown by the pallet's
    # half diagonal; never built from scenario.pickup (Codex v4 P3).
    from forklift_core.planning.observation_viewpoints import BAY_PICKUP_ZONE as _ZONE

    zone_grow = math.hypot(geometry.pallet_depth_m, geometry.pallet_width_m) / 2
    pickup_zone = Rectangle(
        (_ZONE.x_min_m + _ZONE.x_max_m) / 2,
        (_ZONE.y_min_m + _ZONE.y_max_m) / 2,
        _ZONE.x_max_m - _ZONE.x_min_m + 2 * zone_grow,
        _ZONE.y_max_m - _ZONE.y_min_m + 2 * zone_grow,
        0.0,
    )

    if obstacle is not None and obstacle["layer"].known_enabled:
        # Plan v10 D5: the 1.05 m plane cannot see the pallet; before recognition
        # its possible area is unknown to the permission too (prior, not truth).
        from forklift_core.perception.known_obstacles import KnownRect

        obstacle["layer"].known = [KnownRect(
            (pickup_zone.x_m, pickup_zone.y_m, pickup_zone.length_m, pickup_zone.width_m, pickup_zone.yaw_rad),
            fix_stamp_s=0.0, r_fix_m=0.0, rho_m=0.0, frame_correction=(0.0, 0.0, 0.0), kind="zone")]
        state["known_pallets"] = [{"event": "pickup_zone", "time_s": 0.0}]

    def control_correction():
        """The correction control applies now (the one an estimate made now is expressed in)."""
        try:
            slam_ref_ = slam
        except NameError:
            slam_ref_ = None
        if slam_ref_ is not None and slam_ref_["tracker"].applied is not None:
            return tuple(float(v) for v in slam_ref_["tracker"].applied[0])
        return (0.0, 0.0, 0.0)

    def known_pallet(rect_xyyaw, r_fix_m: float, event: str) -> None:
        """Replace the known pallet with a fresh fix (plan v10 D5: the age restarts).

        Stored with the correction control applies at this instant -- the one the
        estimate is in -- not the layer's last-scan copy, which lags a capture's
        release (Codex stage-1 2nd P1-1: 0.30 m shown 0.30 m off).
        """
        from forklift_core.perception.known_obstacles import KnownRect

        stamp_ = float(obstacle.get("last_stamp", 0.0))
        applied_ = control_correction()
        obstacle["layer"].known = [KnownRect(
            (float(rect_xyyaw[0]), float(rect_xyyaw[1]), geometry.pallet_depth_m, geometry.pallet_width_m,
             float(rect_xyyaw[2])),
            fix_stamp_s=stamp_, r_fix_m=float(r_fix_m), rho_m=None, frame_correction=tuple(applied_))]
        state.setdefault("known_pallets", []).append(
            {"event": event, "time_s": stamp_, "rect": [float(v) for v in rect_xyyaw], "r_fix_m": float(r_fix_m)})

    def grid_world(sc):
        """The scenario a grid plan sees: no ground-truth props (plan audit table)."""
        return replace(sc, props=()) if grid_planning else sc

    def grid_kwargs(kind=None, target=None, own=None, applied=None) -> dict:
        """occupancy (and the pallet obstacle the plan may know) for a grid plan."""
        if not grid_planning:
            return {}
        try:  # the first plans run before the SLAM link exists
            slam_ref = slam
        except NameError:
            slam_ref = None
        stamp = obstacle.get("last_stamp", 0.0)
        layer_ = obstacle["layer"]
        # The correction the obstacle grid is projected with -- the one the
        # permission's grid shares -- unless the caller passes another: a
        # release plans with the released correction before the obstacle block
        # re-projects (Codex review P1). The tracker's own value moves between
        # scans; planning on it shifted the grid from the permission's and
        # flipped seed 1's insertion (l5_exp_A/B, 2026-10-06).
        applied_ = tuple(float(v) for v in applied) if applied is not None else (
            obstacle["applied"] or (0.0, 0.0, 0.0))
        try:
            own_now_ = tuple(float(v) for v in (own if own is not None else rear))
            loaded_now_ = bool(loaded)
        except NameError:  # the first plans, before the drive loop: nothing remembered under the truck
            own_now_, loaded_now_ = None, False
        mode_ = slam_ref["tracker"].mode if slam_ref is not None else None
        key_ = (stamp, obstacle["version"], applied_, mode_, loaded_now_,
                None if own_now_ is None else tuple(round(v, 4) for v in own_now_))
        cache_ = obstacle.get("plan_cache")
        if cache_ is not None and cache_[0] == key_:
            # One bundle per planning instant and state: retries and fallbacks
            # see the same map and memory (Codex design review P1).
            occupancy, bundle = cache_[1], cache_[2]
        else:
            use_slam = False
            map_note = None
            if layer_.memory is not None and args.slam_map_dir is not None:
                latest = args.slam_map_dir / "latest_map.npz"
                if latest.exists():
                    # One open file: its fstat and its contents go together (the
                    # bridge may swap the path between -- Codex review P2).
                    with open(latest, "rb") as fh_, np.load(fh_) as m_:
                        index_ = int(m_["index"])
                        after_ = int(m_["after_scan_id"])
                        map_stamp_ = int(m_["stamp_ns"]) / 1e9
                        replied_ = (slam_ref["scan_id"] - 1) if slam_ref is not None else None
                        loaded_index_ = (layer_.memory.slam_info or {}).get("index", -1)
                        why_not_ = None
                        if os.fstat(fh_.fileno()).st_mtime < obstacle["run_wall_start"]:
                            why_not_ = "file_before_run"  # another run's file (Codex review P2)
                        elif replied_ is not None and after_ > replied_:
                            why_not_ = "unsent_scans"
                        elif map_stamp_ > stamp + 1e-6:
                            why_not_ = "stamp_ahead"
                        elif index_ < loaded_index_:
                            why_not_ = "older_than_loaded"
                        if why_not_ is not None:
                            # Never used; recorded.
                            map_note = {"rejected_index": index_, "why": why_not_, "after_scan_id": after_,
                                        "map_stamp_s": map_stamp_, "replied": replied_}
                        elif index_ != loaded_index_:
                            # Dated by the map's own stamp, never by when this
                            # plan read it (Codex review P1).
                            layer_.memory.set_slam(
                                m_["data"], tuple(m_["origin"][:2]), float(m_["resolution_m"]),
                                float(m_["origin"][2]), stamp_s=map_stamp_, index=index_,
                                after_scan_id=after_, stamp_ns=int(m_["stamp_ns"]),
                            )
                # The map is in SLAM's frame: only while control applies SLAM's
                # correction (not holding) do the two frames agree.
                use_slam = mode_ != "holding"
            occupancy = layer_.planner_grid(stamp, applied_, obstacle["version"], use_slam=use_slam,
                                            own_pose=own_now_, loaded=loaded_now_)
            bundle = {
                "memory": layer_.memory is not None,
                "memory_revision": layer_.memory.revision if layer_.memory is not None else None,
                "slam_used": use_slam and layer_.memory is not None and layer_.memory.slam is not None,
                "slam_map": (layer_.memory.slam_info if layer_.memory is not None else None),
                "slam_map_note": map_note,
                "last_replied_scan_id": (slam_ref["scan_id"] - 1) if slam_ref is not None else None,
                "applied": list(applied_),
                "mode": mode_,
            }
            obstacle["plan_cache"] = (key_, occupancy, bundle)
        obstacle["plans"].append({"kind": kind, "stamp_s": stamp, "occupied_cells": int(occupancy.occupied.sum()),
                                  **bundle})
        out = {"occupancy": occupancy}
        if obstacle.get("deadline") is not None and kind in ("observe", None, "mission", "return"):
            out["deadline"] = obstacle["deadline"]
        if kind == "return" and target is not None:
            # After the drop (plan D5): the delivered pallet is an outside
            # obstacle by its docked rectangle, which the return plan carries;
            # its grid swelling around the forks just withdrawn from it is
            # cleared for the plan (L3c v15: invalid_start 0.19 m from its face).
            # The permission still checks the full grid every tick.
            res_r = occupancy.resolution_m
            nx_r, ny_r = occupancy.occupied.shape
            gi_r, gj_r = np.meshgrid(np.arange(nx_r), np.arange(ny_r), indexing="ij")
            dx_r = occupancy.origin_x_m + (gi_r + 0.5) * res_r - target.x_m
            dy_r = occupancy.origin_y_m + (gj_r + 0.5) * res_r - target.y_m
            c_r, s_r = math.cos(target.yaw_rad), math.sin(target.yaw_rad)
            band_r = (np.abs(dx_r * c_r + dy_r * s_r) <= geometry.pallet_depth_m / 2 + 0.425) & (
                np.abs(-dx_r * s_r + dy_r * c_r) <= geometry.pallet_width_m / 2 + 0.20
            )
            from forklift_core.planning.grid_collision import OccupancyGrid as _OccR

            # Only where the truck stands now (its hull grown by the planning
            # clearance): clearing the whole band let the plan run through the
            # swelling the permission still sees, and every replan was blocked
            # again (L3c v18 seed 1: 3 replans in 30 s on the return).
            sx, sy, syaw = (float(v) for v in rear)
            fp_r = geometry.unloaded_footprint
            grow_r = planner_config.clearance_m + res_r
            dxs, dys = occupancy.origin_x_m + (gi_r + 0.5) * res_r - sx, occupancy.origin_y_m + (gj_r + 0.5) * res_r - sy
            cs, ss = math.cos(syaw), math.sin(syaw)
            us, vs = dxs * cs + dys * ss, -dxs * ss + dys * cs
            under_r = (us <= fp_r.front_m + grow_r) & (us >= -fp_r.rear_m - grow_r) & (np.abs(vs) <= fp_r.half_width_m + grow_r)
            out["occupancy"] = _OccR(occupancy.origin_x_m, occupancy.origin_y_m, res_r,
                                     occupancy.occupied & ~(band_r & under_r), version=occupancy.version)
            return out
        if kind == "observe":
            # Before recognition the pallet is somewhere in the pickup zone; once
            # it has been recognised (the near-capture leg is still 'observe'),
            # it is where perception put it.
            known = obstacle.get("pickup_estimate")
            out["pickup_obstacle"] = pickup_zone if known is None else Rectangle(
                known.x_m, known.y_m, geometry.pallet_depth_m, geometry.pallet_width_m, known.yaw_rad
            )
        elif kind == "mission" and target is not None:
            obstacle["pickup_estimate"] = target
            out["pickup_obstacle"] = Rectangle(
                target.x_m, target.y_m, geometry.pallet_depth_m, geometry.pallet_width_m, target.yaw_rad
            )
            # The docking straights see the grid with the perceived pallet's band
            # cleared (plan D5): the estimate grown by the bias bound and the
            # grid's error radius along the insertion axis, by the radius across.
            # Along the insertion axis: the bias bound plus the largest swelling
            # a pallet face can get in the grid (about 0.33 m for a 1 s mark at
            # ~3.5 m; 0.19 m at the 0.3 s memory). Across: the pallet sides plus
            # their swelling -- the truck's body (half width 0.36 m) stays inside
            # the pallet width (0.40 m), so what the band hides there is only the
            # planning clearance, not room an outside object could occupy. The
            # body stays outside the band at the approach end (0.5 m from the
            # face); what lies in front of the face is for the pocket depth check
            # (plan D5, L3c). Until then the band is wider than the plan's
            # fork-only band.
            band_along = 0.025 + 0.40
            band_across = 0.20
            res_g = occupancy.resolution_m
            nx_g, ny_g = occupancy.occupied.shape
            gi, gj = np.meshgrid(np.arange(nx_g), np.arange(ny_g), indexing="ij")
            dx = occupancy.origin_x_m + (gi + 0.5) * res_g - target.x_m
            dy = occupancy.origin_y_m + (gj + 0.5) * res_g - target.y_m
            ct, st_ = math.cos(target.yaw_rad), math.sin(target.yaw_rad)
            band = (np.abs(dx * ct + dy * st_) <= geometry.pallet_depth_m / 2 + band_along) & (
                np.abs(-dx * st_ + dy * ct) <= geometry.pallet_width_m / 2 + band_across
            )
            from forklift_core.planning.grid_collision import OccupancyGrid as _Occ, SplitOccupancy

            # Plan D5 with the body delta (user approval 2026-10-05): the whole
            # truck sees the perceived pallet's band cleared on the docking
            # straights. The carriage must come within 46 mm of the face, inside
            # the face's grid swelling, and the insert straight is checked with
            # the planning clearance; at run time the depth pocket check stands in
            # for the grid there (its volume stops the body at the face). Transport
            # and return are replanned on the live grid when they start, so no
            # band-cleared map reaches an executed travel leg.
            cleared = _Occ(occupancy.origin_x_m, occupancy.origin_y_m, res_g, occupancy.occupied & ~band,
                           version=occupancy.version)
            out["docking_occupancy"] = SplitOccupancy(
                cleared,
                cleared,
                geometry.axle_to_fork_tip_m - geometry.carriage_limit_m,
            )
            obstacle["plans"][-1]["docking_band_cells"] = int((occupancy.occupied & band).sum())
            np.savez_compressed(
                args.output / f"grid_mission_plan_{len(obstacle['plans'])}.npz",
                occupied=occupancy.occupied, docking=out["docking_occupancy"].forks.occupied,
                origin=[occupancy.origin_x_m, occupancy.origin_y_m], res=res_g,
                target=[target.x_m, target.y_m, target.yaw_rad],
            )
        return out

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
                grid_world(scenario),
                waypoint,
                planner_config,
                geometry=geometry,
                pickup_bounds=pickup_bounds,
                extended=extended,
                **grid_kwargs("observe"),
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
            grid_world(scenario),
            planner_config,
            geometry=geometry,
            return_to=return_to_pose,
            pickup_bounds=pickup_bounds,
            travel_config=travel_config,
            trace=planning_trace,
            **grid_kwargs("mission"),
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
    shared_scan = False  # plan v10: set with --record-slam when the layer shares the SLAM LiDAR
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
        # Plan v10: with a single obstacle LiDAR that is the SLAM LiDAR, one raw
        # scan feeds both (same mount, beams, stamp and noise draw).
        shared_scan = obstacle is not None and obstacle["layer"].shared_with_slam
        if shared_scan:
            layer_ = obstacle["layer"]
            if len(layer_.sensors) != 1:
                raise SystemExit("shared_with_slam needs exactly one obstacle sensor")
            sensor_ = layer_.sensors[0]
            if not (
                np.allclose(sensor_.xyz_m, laser_mount.xyz_m, atol=1e-9)
                and math.isclose(sensor_.yaw_rad, laser_mount.yaw_rad, abs_tol=1e-9)
                and len(layer_.beam_angles) == len(beam_angles)
                and np.allclose(layer_.beam_angles, beam_angles, atol=1e-9)
                and math.isclose(layer_.range_min_m, scan_pattern.range_min_m, abs_tol=1e-9)
                and math.isclose(layer_.range_max_m, scan_pattern.range_max_m, abs_tol=1e-9)
            ):
                raise SystemExit("shared_with_slam: the obstacle sensor is not the SLAM LiDAR (mount, beams or range differ)")
            if args.slam_noise_spec:
                raise SystemExit("shared_with_slam draws one truncated noise for both; --slam-noise-spec would differ")
        state["shared_lidar_scan"] = bool(shared_scan)
        slam_log = {
            "silenced_scan_stamps_s": [],
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
            invert,
        )
        from forklift_core.localization.scan_docking import dock, laser_points
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
                **({"steering_std_rad": math.radians(1.5) / (3.0 if args.slam_noise_spec == "3sigma" else 1.0),
                    "range_model": "a2m12",
                    "range_spec_scale": 1.0 / 3.0 if args.slam_noise_spec == "3sigma" else 1.0}
                   if args.slam_noise_spec else {}),
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
            # Plan v3.8: online SLAM + pre-scanned destination docking.
            "docking": {"status": "pending"},
            "stop_now": False,
            "tracker_speed": 0.0,
            "last_command": 0.0,
        }
        state["slam_feedback"] = {
            "socket": str(args.slam_feedback),
            "noise_seed": args.slam_noise_seed,
            "noise_spec": args.slam_noise_spec,
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
            # N9 with the shared LiDAR (Codex stage-1 P1-1): the silence that stops
            # the permission also starves SLAM. Keep driving the stop on odometry
            # under the last applied correction until the truck stands; the
            # silence trace ends the run. Any other staleness still fails.
            silent_ = (
                shared_scan and new_obstacles is not None
                and obstacle["layer"].sensors[0].name in new_obstacles["schedule"].silenced
            )
            if not silent_:
                require(False, f"localization_stale: {exc}")
            slam.setdefault("silence_dead_reckoning_s", float(now_s))
            est = compose(slam["tracker"].applied[0], slam_odom_base())
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

    def known_transition(from_phase: str, next_phase: str, t: float) -> None:
        """Known pallets across the mission (plan v10 D5)."""
        from forklift_core.perception.known_obstacles import fork_pocket_gaps, invert_pose
        from forklift_core.perception.obstacle_grid import compose as grid_compose

        layer_ = obstacle["layer"]
        rear_ = obstacle.get("control_rear")
        if next_phase == "lift":
            # The pallet is on the forks now (the loaded outline covers it). Keep
            # where perception put it relative to the truck: at the drop that is
            # the delivered pallet's pose (fork geometry, no new observation).
            est_ = obstacle.get("pickup_estimate")
            if est_ is not None and rear_ is not None:
                obstacle["pallet_rel"] = grid_compose(invert_pose(rear_), (est_.x_m, est_.y_m, est_.yaw_rad))
            layer_.known = []
            state.setdefault("known_pallets", []).append({"event": "lifted", "time_s": t})
        if next_phase == "withdraw" and layer_.known_config.get("withdraw_mode", "certified") != "certified":
            # Interim (user, 2026-10-08): the gate's b_w 12.9 mm + e_w 8.4 mm + the checker's
            # 18.5 mm side width exceed the 35.6 mm smallest blade-to-pocket gap measured, so
            # the certificate cannot hold. The withdrawal runs as before v10 (no permission,
            # the planned straight); the delivered pallet joins the permission where the
            # withdrawal stops, re-fixed at b_w + e_w_full (the measured release error and
            # four-corner drift to the stop).
            if obstacle.get("pallet_rel") is not None and rear_ is not None:
                obstacle["delivered_pending"] = (grid_compose(rear_, obstacle["pallet_rel"]), control_correction(),
                                                 float(obstacle.get("last_stamp", t)))
            state.setdefault("known_pallets", []).append({"event": "withdraw_uncertified", "time_s": t})
        elif next_phase == "withdraw":
            # No recognised pallet or control pose: nothing to certify against -- refuse
            # rather than back out uncertified (Codex stage-1 P1-5).
            require(obstacle.get("pallet_rel") is not None and rear_ is not None,
                    "withdraw_not_certified:no_pallet_estimate")
            rel_ = obstacle["pallet_rel"]
            kp_ = layer_.known_config
            r_fix_ = kp_.get("b_w_m") if kp_.get("b_w_m") is not None else kp_.get("r_fix_pickup_m", 0.05)
            known_pallet(grid_compose(rear_, rel_), r_fix_, "delivered")
            blades_ = read_fork_blades_m(args.forklift_urdf)
            pg_ = args.pallet_geometry_loaded
            centre_ = sum(abs(y0 + y1) / 2 for _, _, y0, y1 in blades_) / len(blades_)
            half_ = sum((y1 - y0) / 2 for _, _, y0, y1 in blades_) / len(blades_)
            depth_ = geometry.axle_to_fork_tip_m + geometry.pallet_depth_m / 2 - geometry.inserted_offset_m
            gaps_ = fork_pocket_gaps(
                float(rel_[1]), float(rel_[2]), blade_centre_m=centre_, blade_half_width_m=half_,
                blade_length_m=depth_, pocket_inner_m=pg_.block_widths_m[1] / 2,
                pocket_outer_m=pg_.overall_width_m / 2 - pg_.block_widths_m[0],
            )
            until_ = depth_ + 0.10 + layer_.permission.config.stopping.distance_m(settings["withdraw_speed_mps"])
            # The certified straight starts where the truck stands, back along its own
            # heading: measured from the planned line, the insertion's few-mm end error
            # would already break the 6 mm tracking bound (plan v10 D5 (4)).
            length_ = float(paths["withdraw"].length_m)
            count_ = max(2, int(math.ceil(length_ / 0.02)) + 1)
            s_ = np.linspace(0.0, length_, count_)
            straight_ = np.column_stack((rear_[0] - s_ * math.cos(rear_[2]), rear_[1] - s_ * math.sin(rear_[2]),
                                         np.full(count_, rear_[2])))
            paths["withdraw"] = replace(paths["withdraw"], poses=straight_,
                                        directions=np.full(count_, -1, dtype=np.int8),
                                        curvatures_inv_m=np.zeros(count_))
            trackers["withdraw"] = RearAxlePathTracker(
                straight_, paths["withdraw"].directions, paths["withdraw"].curvatures_inv_m, trackers["withdraw"].config
            )
            obstacle["withdraw_line"] = (tuple(rear_), length_)
            ok_ = layer_.certify_withdrawal(paths["withdraw"].poses, layer_.parts_shape, until_, gaps_)
            state["withdraw_certificate"] = {
                "time_s": t, "gaps_m": [float(g) for g in gaps_], "b_w_m": kp_.get("b_w_m"), "e_w_m": kp_.get("e_w_m"),
                "corridor_until_m": until_, "certified": ok_,
            }
            # Uncertified: the truck does not back out of the pallet (plan v10 D5 (3)).
            require(ok_, f"withdraw_not_certified:{state['withdraw_certificate']}")
        if from_phase == "withdraw":
            layer_.end_withdrawal(now_s=float(obstacle.get("last_stamp", t)))
            pending_ = obstacle.pop("delivered_pending", None)
            if pending_ is not None:
                from forklift_core.perception.known_obstacles import KnownRect

                kp_ = layer_.known_config
                (dx_, dy_, dyaw_), corr_, released_s_ = pending_
                if kp_.get("b_w_m") is not None and kp_.get("e_w_full_m") is not None:
                    # Re-fixed where the withdrawal stops: release error + measured four-corner drift.
                    full_ = float(kp_["b_w_m"]) + float(kp_["e_w_full_m"])
                    stamp_ = float(obstacle.get("last_stamp", t))
                else:
                    # Either bound missing (Codex stage-1 3rd P1): no re-fix -- the pallet keeps
                    # ageing from the release, with the release error alone.
                    full_ = float(kp_["b_w_m"]) if kp_.get("b_w_m") is not None else float(kp_.get("r_fix_pickup_m", 0.05))
                    stamp_ = released_s_
                layer_.known = [KnownRect((dx_, dy_, geometry.pallet_depth_m, geometry.pallet_width_m, dyaw_),
                                          fix_stamp_s=stamp_, r_fix_m=full_, rho_m=None, frame_correction=corr_)]
                state.setdefault("known_pallets", []).append(
                    {"event": "delivered_after_withdraw", "time_s": stamp_, "rect": [dx_, dy_, dyaw_], "r_fix_m": full_})

    def transition(next_phase: str, t: float) -> None:
        nonlocal phase, phase_started
        state["transitions"].append({"from": phase, "to": next_phase, "time_s": t})
        if obstacle is not None and obstacle["layer"].known_enabled:
            known_transition(phase, next_phase, t)
        print("TRANSITION", record_json(state["transitions"][-1]), flush=True)
        snapshot(phase)
        if next_phase == "insert" and pocket["check"] is not None:
            # D5: one frame taken standing at the approach end before the forks
            # go in. From there the strip before the face is in view; driving in,
            # the frames erode by the speed and, close up, the strip leaves the
            # camera's view (L3c v22 seed 2: four voxels 6.5 cm before the face).
            pocket["await_still_frame"] = True
        if phase == "insert" and pocket["check"] is not None:
            # The insertion is over: the depth check stops answering and the
            # pallet region is no longer waived in the grid.
            obstacle["layer"].exempt = None
            obstacle["layer"].depth_support = None
            state.setdefault("pocket_check", {}).update(pocket["check"].summary(), frames_read=pocket["frames_read"])
            pocket["check"] = None
        if (
            next_phase == "extract" and obstacle is not None and obstacle["layer"].memory is not None
            and obstacle.get("pickup_estimate") is not None
        ):
            # The pallet is the truck's now: its remembered hits around the
            # recognised estimate go (D2/D3 delta; Codex review P1 -- they kept
            # the loaded start blocked). By the estimate, never the truth.
            est_ = obstacle["pickup_estimate"]
            mem_ = obstacle["layer"].memory
            # Endpoints only, within the estimate's error (0.05 m) and the range
            # noise bound (noise_cut_m): every endpoint the pallet can have goes
            # (Codex re-review 4 P2: a -0.059 m noisy one stayed and blocked the
            # loaded start); a box beyond the 0.12 m placement margin loses only
            # its endpoints with noise below -0.01 m and keeps the rest (Codex
            # review P1).
            grow_ = 0.05 + obstacle["layer"].noise_cut_m
            gone_ = mem_.retract_endpoints(est_.x_m, est_.y_m, geometry.pallet_depth_m, geometry.pallet_width_m,
                                           est_.yaw_rad, grow_)
            obstacle.setdefault("memory_events", []).append({"time_s": t, "event": "retract_pickup", "cells": gone_,
                                                                      "grow_m": grow_})
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
        # After an accepted docking match the withdraw (and so the return's
        # start) has moved: the return is replanned whatever the jump (Codex
        # 8428113 P2: a 0 jump left the old return start 366 mm away).
        docked_return = leg == "return_home" and slam["docking"].get("accepted_any")
        if leg not in trackers or (
            jump_m <= 0.02 and abs(jump_rad) <= 0.02 and not docked_return
        ):
            if leg == "transport":
                arm_docking()
            return
        start = PlanningPose(float(after[0]), float(after[1]), float(after[2]))
        released_ = (tuple(float(v) for v in slam["tracker"].applied[0])
                     if slam["tracker"].applied is not None else None)
        replan_start = time.monotonic()
        if leg == "transport":
            replanned = plan_transport_leg(
                grid_world(scenario), start, planner_config, geometry=geometry, travel_config=travel_config,
                **grid_kwargs(None, own=after, applied=released_),
            )
        else:
            # The delivered pallet sits where docking put it -- right in the
            # held frame. Carry it into the released frame with the same
            # change of frame as the truck, T = after o before^-1, so the
            # truck-pallet relation survives the release (S3 seed 1 on a351056:
            # invalid_start against the nominal pallet after a docked drop).
            return_scenario = slam.get("transport_scenario", scenario)
            if slam["docking"].get("accepted_any"):
                frame_change = compose(tuple(after), invert(tuple(before)))
                site = return_scenario.destination
                moved_site = compose(frame_change, (site.x_m, site.y_m, site.yaw_rad))
                return_scenario = replace(
                    return_scenario,
                    destination=replace(
                        site, x_m=moved_site[0], y_m=moved_site[1], yaw_rad=moved_site[2]
                    ),
                )
                event["pallet_frame_change"] = list(frame_change)
                slam["return_scenario"] = return_scenario  # later return recoveries too
            replanned = plan_return_leg(
                grid_world(return_scenario),
                start,
                return_to_pose,
                planner_config,
                geometry=geometry,
                travel_config=travel_config,
                **grid_kwargs(None, own=after, applied=released_),
            )
        if replanned.status == "invalid_start" and grid_planning:
            # The released pose may sit inside the planning clearance of a grid
            # mark (L3b seed 3: transport after the extract); as for the obstacle
            # replans, once more with no clearance -- the grid's swelling holds
            # the placement error and the permission guards every tick.
            tight_cfg = replace(planner_config, clearance_m=0.0)
            tight_travel_cfg = replace(travel_config, clearance_m=0.0) if travel_config is not None else None
            if leg == "transport":
                replanned = plan_transport_leg(
                    grid_world(scenario), start, tight_cfg, geometry=geometry, travel_config=tight_travel_cfg,
                    **grid_kwargs(None, own=after, applied=released_),
                )
            else:
                replanned = plan_return_leg(
                    grid_world(return_scenario), start, return_to_pose, tight_cfg, geometry=geometry,
                    travel_config=tight_travel_cfg, **grid_kwargs(None, own=after, applied=released_),
                )
            event["tight_retry"] = True
        event.update(
            replaced_path=path_record(paths[leg]),
            replanned=True,
            replan_status=replanned.status,
            planning_wall_s=time.monotonic() - replan_start,
        )
        require(replanned.success, f"slam_release_replan_failed: {replanned.status}")
        paths[leg] = replanned
        state.setdefault("paths", {})[leg] = path_record(replanned)
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
        if leg == "transport":
            arm_docking()

    def arm_docking() -> None:
        """Drive transport only to where its delivery straight begins (v3.8)."""
        docking = slam["docking"]
        if docking["status"] not in ("pending", "armed"):
            return
        full = paths["transport"]
        # The current round's stop: 1.5 m, then 0.4 m (Codex 5de6c3a P2).
        prefix = final_straight_prefix(full, docking.get("keep_m", geometry.delivery_straight_m))
        if prefix is None:
            docking["status"] = "no_final_straight"
            return
        docking.update(
            status="armed",
            delivery_rear=[float(v) for v in full.poses[-1]],
            predelivery_rear=[float(v) for v in prefix.poses[-1]],
        )
        # A stop to look, not a final goal: judged like an observe stop (3 cm,
        # 0.05 rad); the docked straight that follows keeps the 8 mm goal.
        slam.setdefault("transport_config", trackers["transport"].config)
        trackers["transport"] = RearAxlePathTracker(
            prefix.poses,
            prefix.directions,
            prefix.curvatures_inv_m,
            replace(
                slam["transport_config"],
                position_tolerance_m=0.03,
                yaw_tolerance_rad=0.05,
            ),
        )

    def dock_at_delivery_straight(t: float) -> None:
        """Match the live scan to the delivery reference and re-place the goal.

        Codex v3.8: goal_est = A o R^-1 with R = X o Q o X^-1; the whole rest
        of the drop (delivery straight, withdraw, any transport replan) is
        moved by the same correction E = goal_est o D^-1; an accepted match
        whose straight cannot be redrawn is re-aligned by a collision-checked
        plan, not dropped; a refused match falls back to the SLAM goal.
        """
        nonlocal phase_started
        docking = slam["docking"]
        state["delivery_docking"] = docking  # linked first: kept if a step fails
        delivery = tuple(docking["delivery_rear_prior"])
        path_end = docking["delivery_rear"]
        docking["path_end_vs_prior_m"] = float(math.hypot(path_end[0] - delivery[0], path_end[1] - delivery[1]))
        # A: the estimate at the instant of the scan being matched (Codex P3).
        # Re-expressed with the correction applied now, so a release between
        # the scan and this match cannot mix frames (Codex 5de6c3a P2).
        bx, by, byaw = compose(slam["tracker"].applied[0], slam["last_scan_odom_base"])
        estimate = (
            bx - abs(args.rear_axle_offset_m) * math.cos(byaw),
            by - abs(args.rear_axle_offset_m) * math.sin(byaw),
            byaw,
        )
        offset = abs(args.rear_axle_offset_m)
        rear_from_laser = (
            offset + float(laser_mount.xyz_m[0]),
            float(laser_mount.xyz_m[1]),
            float(laser_mount.yaw_rad),
        )
        live = laser_points(slam["last_sent_ranges"], beam_angles)
        # The station the reference scan was taken at (the prior differs only with
        # --destination-prior-error-m). After a D7c retry the first match is seeded
        # from the latest goal, not the prior (Codex D7c re-review P2-3).
        reference = tuple(docking.get("delivery_rear_reference", delivery))
        seeded = docking.get("retry_seed_goal") is not None
        seed = tuple(docking["retry_seed_goal"]) if seeded else delivery
        match_start = time.monotonic()
        result = dock(
            slam["reference_points"],
            live,
            rear_from_laser=rear_from_laser,
            estimate_rear=estimate,
            delivery_rear=seed,
        )
        # Evaluation only: where the delivery pose really is in the estimate frame.
        expected = compose(estimate, compose(invert(tuple(truth_rear)), reference))
        docking.update(
            match_wall_s=time.monotonic() - match_start,
            live_beams=int(len(live)),
            accepted=result.accepted,
            reason=result.reason,
            correction=result.correction,
            starts=[[ok, why, list(q)] for ok, why, q in result.starts],
            goal_error_vs_truth=None
            if result.goal_estimate is None
            else [
                float(result.goal_estimate[0] - expected[0]),
                float(result.goal_estimate[1] - expected[1]),
                float(math.atan2(math.sin(result.goal_estimate[2] - expected[2]), math.cos(result.goal_estimate[2] - expected[2]))),
            ],
            slam_goal_error_vs_truth=[
                float(delivery[0] - expected[0]),
                float(delivery[1] - expected[1]),
            ],
        )
        if args.d7_docking:
            docking["match_seed"] = "latest_goal" if seeded else "prior"
        retry_match = docking.pop("await_retry_match", False)
        round_number = docking.get("round", 1)
        keep_m = docking.get("keep_m", geometry.delivery_straight_m)
        previous_goal = tuple(docking.get("previous_goal", delivery))
        # A refused match keeps the best goal so far (the SLAM goal in round 1).
        goal = result.goal_estimate if result.accepted else previous_goal
        shift = compose(goal, invert(delivery))  # E: planned -> corrected (total)
        step_shift = compose(goal, invert(previous_goal))  # since the last round
        docking.setdefault("rounds", []).append(
            {
                "round": round_number,
                "keep_m": keep_m,
                "accepted": result.accepted,
                "reason": result.reason,
                "goal_error_vs_truth": docking["goal_error_vs_truth"],
                "estimate_vs_truth_at_match": [
                    float(estimate[0] - truth_rear[0]),
                    float(estimate[1] - truth_rear[1]),
                ],
            }
        )
        if args.d7_docking:
            # Plan D7c records: when, how long, and the errors in the docking line's
            # frame (along, lateral, yaw) rather than world dx/dy.
            entry_ = docking["rounds"][-1]
            entry_.update(time_s=t, match_wall_s=docking["match_wall_s"], retry_match=retry_match,
                          estimate_error_truck_frame=list(compose(invert(tuple(truth_rear)), estimate)))
            if result.goal_estimate is not None:
                entry_["goal_error_line_frame"] = list(compose(invert(expected), tuple(result.goal_estimate)))
        if retry_match:
            # Plan D7c: the first match after a retry must be accepted -- no
            # falling back to the previous goal (Codex D7c review P1-4).
            docking.pop("retry_seed_goal", None)
            require(result.accepted, f"docking_retry_match_refused:{result.reason}")
        if result.accepted:
            docking["accepted_any"] = True
        # A refused second round keeps the first round's valid correction
        # (Codex 5de6c3a P2): still "done", and its world map W is reused.
        docking["status"] = "done" if docking.get("accepted_any") else "fallback"
        committed = False
        if args.d7_docking and result.accepted:
            # Plan D7c: commit the matched goal before the straight check, so a
            # refused straight retries from it (Codex D7c review P1-3). The goal is
            # in the held frame the match ran in.
            docking["to_world"] = list(compose(compose(reference, result.relative_rear), invert(estimate)))
            move_withdraw(step_shift)
            docking["previous_goal"] = list(goal)
            docking["goal_frame"] = "held"
            committed = True
        line_start = compose(goal, (-keep_m, 0.0, 0.0))
        # Delivery corrections of ~10 cm are the point of docking (seed 0: 92 mm),
        # so the box is wider than the insertion one; what makes it safe is the
        # dry run arriving AND its swept loaded footprint staying clear.
        straight, offsets = straight_from_pose(
            PlanningPose(*estimate),
            PlanningPose(*line_start),
            PlanningPose(*goal),
            max_lateral_m=0.12,
            max_yaw_rad=0.08,
            min_length_m=0.3,
        )
        docking["offsets_to_new_line"] = offsets
        path = None
        if straight is not None:
            config = slam.get("transport_config", trackers["transport"].config)
            dry = bicycle_rollout(
                straight.poses, straight.directions, straight.curvatures_inv_m, config, rear
            )
            # The obstacles are in world; the dry run is in the estimate frame.
            # After an accepted match the truck's world pose is D o R, so
            # W = (D o R) o A^-1 carries estimate poses into world (Codex v3.8
            # impl P1). Without a match the estimate is the best world guess.
            if result.accepted:
                to_world = compose(compose(reference, result.relative_rear), invert(estimate))
                docking["to_world"] = list(to_world)  # the held frame keeps it valid
            elif docking.get("to_world") is not None:
                to_world = tuple(docking["to_world"])
            else:
                to_world = (0.0, 0.0, 0.0)
            if grid_planning:
                # The LiDAR grid, not the ground truth (plan audit table): the
                # dry run is in the estimate frame, which the grid shares.
                from forklift_core.planning.grid_collision import GridFootprintChecker

                swept_checker = GridFootprintChecker(
                    grid_kwargs("docking")["occupancy"], geometry.loaded_footprint, scenario.bounds
                )

                def pose_clear(pose):
                    return swept_checker.free(tuple(pose))
            else:

                def pose_clear(pose):
                    return collision_free_pose(
                        np.asarray(compose(to_world, tuple(pose))),
                        obstacles,
                        geometry.loaded_footprint,
                        scenario.bounds,
                    )
            swept_clear = all(pose_clear(pose) for pose in dry.trajectory)
            docking["dry_run"] = {
                **{k: v for k, v in asdict(dry).items() if k != "trajectory"},
                "trajectory_samples": len(dry.trajectory),
                "swept_clear": swept_clear,
            }
            # The tracker stops anywhere inside its own tolerance (Codex v3.8 P2:
            # a clean 0.7 m straight stops at 7.8 mm); require yaw margin only.
            if (
                dry.status == "arrived"
                and dry.position_error_m <= config.position_tolerance_m
                and abs(dry.yaw_error_rad) <= 0.75 * config.yaw_tolerance_rad
                and swept_clear
            ):
                path = straight
                if args.d7_docking and round_number == 1 and result.accepted:
                    # Plan D7c: what is driven after an accepted first round is the
                    # straight cut at the round-2 stop, judged at the stop's tolerance
                    # -- dry-run that, not only the whole straight (Codex D7 Isaac review
                    # P1: seed 1, whole straight 0.0145 rad, the 0.90 m stop path 0.086).
                    stop_ok, docking["prefix_dry_run"] = DOCKING_RETRY.round_two_stop_check(
                        straight, rear, docking_stop_config(), ROUND2_KEEP_M, clear=pose_clear
                    )
                    if not stop_ok:
                        path = None
        corrected = replace(
            scenario,
            destination=replace(
                scenario.destination,
                **dict(
                    zip(
                        ("x_m", "y_m", "yaw_rad"),
                        compose(shift, (scenario.destination.x_m, scenario.destination.y_m, scenario.destination.yaw_rad)),
                    )
                ),
            ),
        )
        slam["transport_scenario"] = corrected
        # No long held detour: a Hybrid A* re-alignment for a 9 cm correction
        # drove 13.3 m forward-only on held odometry and drifted 0.5 m (S2 v3.8
        # seed 0). A straight that cannot be accepted ends the run instead.
        if path is None and args.d7_docking:
            # Plan D7c (b): back to the line's start and match again.
            start_docking_retry(t, "b", f"docking_unaligned:{offsets}:{docking.get('dry_run')}")
            return
        require(path is not None, f"docking_unaligned:{offsets}:{docking.get('dry_run')}")
        docking.pop("reapproach", None)  # plan D7c: a straight ends the re-approach
        paths["transport"] = path
        state["paths"]["transport"] = path_record(path)
        trackers["transport"] = RearAxlePathTracker(
            path.poses, path.directions, path.curvatures_inv_m, slam.get("transport_config", trackers["transport"].config)
        )
        if not committed:
            move_withdraw(step_shift)
        (args.output / "paths.json").write_text(record_json(state["paths"], indent=2) + "\n")
        add_path_display(stage, path, "Transport", (1.0, 0.65, 0.04))
        phase_started = t
        docking["previous_goal"] = list(goal)
        if round_number == 1 and result.accepted:
            # Second round: loaded wheels slip (~5 % over the 1.5 m straight,
            # S2 v3.8c seed 3: 8 cm short), so stop again 0.6 m out and match
            # once more; the last 0.6 m then carries only that slip (~3 cm).
            # 0.4 m left the tracker too little straight to settle its yaw: a
            # 1.1 cm / 6 mrad start failed the 0.015 rad dry-run margin (L3c v15
            # seed 3: 0.016); 0.6 m settles it to 0.0045 (offline dry run, Codex).
            final_keep = ROUND2_KEEP_M
            prefix = final_straight_prefix(path, final_keep)
            if prefix is not None:
                docking.update(status="armed", round=2, keep_m=final_keep)
                trackers["transport"] = RearAxlePathTracker(
                    prefix.poses,
                    prefix.directions,
                    prefix.curvatures_inv_m,
                    replace(slam["transport_config"], position_tolerance_m=0.03, yaw_tolerance_rad=0.05),
                )

    def set_withdraw(poses) -> None:
        paths["withdraw"] = replace(paths["withdraw"], poses=np.asarray(poses, dtype=float))
        state["paths"]["withdraw"] = path_record(paths["withdraw"])
        trackers["withdraw"] = RearAxlePathTracker(
            paths["withdraw"].poses,
            paths["withdraw"].directions,
            paths["withdraw"].curvatures_inv_m,
            trackers["withdraw"].config,
        )

    def move_withdraw(frame_change) -> None:
        """The withdraw follows the docked goal: move it by the goal's change."""
        if "withdraw" in paths:
            set_withdraw(np.array([compose(tuple(frame_change), tuple(pose)) for pose in paths["withdraw"].poses]))

    def docking_stop_config():
        """A stop to look, judged like arm_docking's (3 cm, 0.05 rad)."""
        base_ = slam.get("transport_config", trackers["transport"].config)
        return replace(base_, position_tolerance_m=0.03, yaw_tolerance_rad=0.05)

    def release_and_carry(t: float) -> dict:
        """Plan D7c: release a held correction; a goal still in the held frame moves
        with everything held by T = after o before^-1, once (docking_retry.py)."""
        docking = slam["docking"]
        if slam["tracker"].mode != "holding":
            return {"released": False}
        before = slam_rear(t)
        jump_m, jump_rad = slam["tracker"].release(odom_from_base=slam_odom_base())
        after = slam_rear(t)
        site = slam.get("transport_scenario", scenario).destination
        carry = DOCKING_RETRY.carry_on_release(
            docking,
            tuple(float(v) for v in before),
            tuple(float(v) for v in after),
            withdraw_poses=paths["withdraw"].poses if "withdraw" in paths else None,
            destination=(site.x_m, site.y_m, site.yaw_rad),
        )
        if carry["moved"]:
            if "withdraw" in paths:
                set_withdraw(carry["withdraw_poses"])
            sc_ = slam.get("transport_scenario", scenario)
            x_, y_, yaw_ = carry["destination"]
            slam["transport_scenario"] = replace(
                sc_, destination=replace(sc_.destination, x_m=x_, y_m=y_, yaw_rad=yaw_)
            )
        event = {"phase": phase, "time_s": t, "event": "release_docking_retry", "jump_m": jump_m,
                 "jump_rad": jump_rad, "frame_change": carry["frame_change"], "goal_moved": carry["moved"]}
        slam["holds"].append(event)
        if obstacle is not None and slam["tracker"].applied is not None:
            # The grid is re-projected with the released correction before any
            # plan reads it, as for a stall replan's release (Codex checkpoint P1).
            obstacle["applied"] = tuple(float(v) for v in slam["tracker"].applied[0])
            obstacle["version"] += 1
            obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
            obstacle["layer"].refresh(
                t, obstacle["applied"], obstacle["version"],
                current_pose=tuple(float(v) for v in after),
                path_ahead=np.array([after]), loaded=loaded,
            )
        return {"released": True, **{k: event[k] for k in ("jump_m", "jump_rad", "frame_change", "goal_moved")},
                "rear": [float(v) for v in after]}

    def reapproach_plan(start_rear, line, config=None):
        """Plan D7c ③: the re-approach to the docking line's start, accepted only
        inside its box and length, and when the transport tracker's dry run stops
        at the line start within the stop's tolerance."""
        planned_at = time.monotonic()
        # The grid around the start pose (after a release: the released one, Codex
        # D7c impl P1-2), and an obstacle replan's shared budget when one runs (P1-1).
        grid_ = grid_kwargs(None, own=tuple(float(v) for v in start_rear))
        deadline_ = grid_.get("deadline")
        path, record = plan_docking_reapproach(
            grid_world(slam.get("transport_scenario", scenario)),
            PlanningPose(*(float(v) for v in start_rear)),
            PlanningPose(*(float(v) for v in line)),
            config if config is not None else planner_config,
            geometry=geometry,
            keep_m=geometry.delivery_straight_m,
            deadline=deadline_ if deadline_ is not None else SearchBudget(20.0),
            **({"occupancy": grid_["occupancy"]} if "occupancy" in grid_ else {}),
        )
        record["planning_wall_s"] = time.monotonic() - planned_at
        if path is not None:
            dry = bicycle_rollout(
                path.poses, path.directions, path.curvatures_inv_m, docking_stop_config(), np.asarray(start_rear)
            )
            record["dry_run"] = {k: v for k, v in asdict(dry).items() if k != "trajectory"}
            if not (dry.status == "arrived" and dry.position_error_m <= 0.03 and abs(dry.yaw_error_rad) <= 0.05):
                record["refused"] = "dry_run"
                path = None
        return path, record

    def install_reapproach(t: float, path, goal) -> None:
        """Plan D7c ④: the re-approach replaces the transport path; the docking waits
        at its end (no arm_docking: a re-approach may end on a reverse curve)."""
        nonlocal phase_started
        paths["transport"] = path
        state["paths"]["transport"] = path_record(path)
        (args.output / "paths.json").write_text(record_json(state["paths"], indent=2) + "\n")
        add_path_display(stage, path, "Transport", (1.0, 0.65, 0.04))
        trackers["transport"] = RearAxlePathTracker(
            path.poses, path.directions, path.curvatures_inv_m, docking_stop_config()
        )
        slam["docking"].update(
            status="armed", round=1, keep_m=geometry.delivery_straight_m, reapproach=True,
            await_retry_match=True, retry_seed_goal=[float(v) for v in goal],
        )
        if obstacle is not None:
            if obstacle.get("backoff", {}).get("phase") == "transport":
                obstacle.pop("backoff")
            obstacle.setdefault("replan_wait", {}).pop("transport", None)
            obstacle.setdefault("progress", {}).clear()
        phase_started = t

    def start_docking_retry(t: float, trigger: str, failure: str) -> None:
        """Plan D7c (delivery): release, plan back to the docking line's start, and
        match again on arrival. At most two per delivery; the third ends the run with
        the failure the retry replaced."""
        retries = state.setdefault("docking_retries", [])
        require(sum(1 for r in retries if r.get("counted")) < 2, failure)
        event = {"trigger": trigger, "time_s": t, "counted": True, "replaces": failure}
        retries.append(event)
        if "previous_goal" not in slam["docking"]:
            # No match committed a goal (a refused first match): the prior, a map pose
            # that a release does not move (Codex D7c impl P2-4).
            slam["docking"]["previous_goal"] = list(slam["docking"]["delivery_rear_prior"])
            slam["docking"]["goal_frame"] = "map"
        event["release"] = release_and_carry(t)
        goal = tuple(slam["docking"]["previous_goal"])
        line = DOCKING_RETRY.line_start(goal, geometry.delivery_straight_m)
        start_now = slam_rear(t)
        path, record = reapproach_plan(start_now, line)
        event.update(goal=list(goal), goal_frame=slam["docking"].get("goal_frame"), line_start=list(line),
                     start=[float(v) for v in start_now], plan=record)
        require(path is not None, f"docking_retry_no_plan:{record.get('refused')}:{record.get('status')}")
        install_reapproach(t, path, goal)

    def transport_replan(start, config, travel):
        """The transport leg -- or, during a D7c re-approach, the re-approach to the
        docking line's start (an obstacle replan must not undo it: Codex D7c P2)."""
        if slam is not None and slam["docking"].get("reapproach"):
            line = DOCKING_RETRY.line_start(slam["docking"]["previous_goal"], geometry.delivery_straight_m)
            path, record = reapproach_plan((start.x_m, start.y_m, start.yaw_rad), line, config)
            state.setdefault("docking_retries", []).append(
                {"trigger": "obstacle_replan", "time_s": world.current_time - initial_time, "counted": False,
                 "plan": record}
            )
            if path is not None:
                return path
            # A search failure keeps its own status (invalid_start / no_path drive the
            # zero-clearance retry -- Codex D7c impl P2-6); an operating refusal is named.
            status_ = record.get("status") if record.get("refused") == "no_path" else f"reapproach_{record.get('refused')}"
            return PlanResult(False, status_, np.empty((0, 3)), np.empty(0, dtype=np.int8), np.empty(0), 0.0,
                              int(record.get("expansions", 0)))
        return plan_transport_leg(
            grid_world(slam.get("transport_scenario", scenario) if slam is not None else scenario),
            start, config, geometry=geometry, travel_config=travel, **grid_kwargs(None),
        )

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
                    perception_display if perception_display is not None else perception_camera,
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
            if obstacle is not None:
                record_video_overlay(t)
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

        def obstacle_scan(stamp: float, base, q, shared_raw: dict | None = None) -> None:
            """Cast the obstacle LiDARs, feed the grid, refresh the path check.

            shared_raw: the shared scan's rays and noisy ranges (plan v10, one
            LiDAR feeding SLAM and the layer) -- no second cast, no second draw.
            """
            layer = obstacle["layer"]
            loaded_now = phase in ("lift", "extract", "transport", "lower")
            prefixes = (planar_lidar.SELF_PREFIX,) + (("/World/Pallet",) if loaded_now else ())
            started = time.monotonic()
            raw = {} if shared_raw is None else dict(shared_raw)
            for sensor in (layer.sensors if shared_raw is None else ()):
                if new_obstacles is not None and sensor.name in new_obstacles["schedule"].silenced:
                    continue
                origin, directions = planar_lidar.laser_rays_world(
                    base, q, planar_lidar.LaserMount(sensor.xyz_m, sensor.yaw_rad), layer.beam_angles
                )
                raw[sensor.name] = (
                    *planar_lidar.cast_scan_flags(origin, directions, layer.range_max_m, own_prefixes=prefixes),
                    obstacle_module.beam_limits(origin, directions, band_top_m=layer.band_top_m,
                                                band_bottom_m=layer.band_bottom_m),
                )
            cast_s = time.monotonic() - started
            yaw_now, _ = yaw_and_tilt(q)
            off = abs(args.rear_axle_offset_m)
            truth_now = (float(base[0]) - off * math.cos(yaw_now), float(base[1]) - off * math.sin(yaw_now), yaw_now)
            if slam is not None:
                odom_rear = tuple(float(v) for v in slam["odom_rear"])
                applied = slam["tracker"].applied
                correction = tuple(float(v) for v in applied[0]) if applied is not None else (0.0, 0.0, 0.0)
                current = tuple(float(v) for v in slam_rear(stamp))
            else:
                odom_rear, correction, current = truth_now, (0.0, 0.0, 0.0), truth_now
            if correction != obstacle["applied"]:
                obstacle["version"] += 1
                obstacle["applied"] = correction
            layer.add_scans(stamp, raw, odom_rear=odom_rear, loaded=loaded_now)
            obstacle["last_stamp"] = float(stamp)
            obstacle["last_raw"] = (raw, np.asarray(base, dtype=float), np.asarray(q, dtype=float))
            leg_direction = 0
            if phase in trackers:
                ahead, leg_direction = trackers[phase].leg_ahead()
            else:
                ahead = np.array([current])
            check = layer.refresh(
                stamp, correction, obstacle["version"], current_pose=current, path_ahead=ahead, loaded=loaded_now,
                direction=leg_direction,
            )
            obstacle["scans"].append(
                {
                    "stamp_s": float(stamp),
                    "phase": phase,
                    "verified_m": check.verified_m,
                    "blocked": check.blocked,
                    "own_beams": {n: int(np.count_nonzero(r[2])) for n, r in raw.items()},
                    "cast_wall_s": cast_s,
                    "total_wall_s": time.monotonic() - started,
                    "version": obstacle["version"],
                }
            )

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
            shared_raw = None
            if shared_scan:
                # Plan v10 (Codex v10 P2-8): one LiDAR, one raw scan. Cast once with
                # the self flags the layer needs, draw one truncated noise vector,
                # and give SLAM and the layer the same stamp, rays and noise. A
                # silenced "high" (N9) stops both inputs.
                layer_ = obstacle["layer"]
                sensor_ = layer_.sensors[0]
                if new_obstacles is not None and sensor_.name in new_obstacles["schedule"].silenced:
                    slam_log["silenced_scan_stamps_s"].append(stamp_now)
                    return
                loaded_scan = phase in ("lift", "extract", "transport", "lower")
                prefixes_ = (planar_lidar.SELF_PREFIX,) + (("/World/Pallet",) if loaded_scan else ())
                distances, hits, own_ = planar_lidar.cast_scan_flags(
                    origin, directions, scan_pattern.range_max_m, own_prefixes=prefixes_
                )
                noise_ = layer_.beam_noise(len(distances))
                shared_raw = {
                    sensor_.name: (
                        distances, hits, own_,
                        obstacle_module.beam_limits(origin, directions, band_top_m=layer_.band_top_m,
                                                    band_bottom_m=layer_.band_bottom_m),
                        layer_.ranges_from(distances, hits, noise_),
                    )
                }
            else:
                distances, hits, _ = planar_lidar.cast_scan(
                    origin, directions, scan_pattern.range_max_m
                )
            ranges = scan_pattern.ranges_from_hits(distances, hits)
            slam_log["scan_stamps_s"].append(stamp_now)
            slam_log["scan_ranges_m"].append(ranges.astype(np.float32))
            slam_log["laser_pose_world"].append(
                planar_lidar.laser_pose_2d(now_base, now_q, laser_mount)
            )
            if obstacle is not None and slam is None:
                obstacle_scan(stamp_now, now_base, now_q, shared_raw)
            if slam is None:
                return
            link = slam["module"]
            if shared_scan:
                sent = ranges.astype(float).copy()
                measured_ = np.isfinite(sent)
                sent[measured_] = np.clip(sent[measured_] + noise_[measured_],
                                          scan_pattern.range_min_m, scan_pattern.range_max_m)
            else:
                sent = slam["noise"].ranges(
                    ranges,
                    range_min_m=scan_pattern.range_min_m,
                    range_max_m=scan_pattern.range_max_m,
                )
            slam["last_sent_ranges"] = sent  # the noisy scan, as docking sees it
            if slam["tracker"].applied is not None:
                bx, by, byaw = compose(slam["tracker"].applied[0], slam_odom_base())
                off = abs(args.rear_axle_offset_m)
                slam["last_scan_estimate_rear"] = (
                    bx - off * math.cos(byaw), by - off * math.sin(byaw), byaw
                )
                slam["last_scan_odom_base"] = tuple(slam_odom_base())
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
            if obstacle is not None:
                # After this scan's reply: the correction control applies now.
                obstacle_scan(stamp_now, now_base, now_q, shared_raw)

        stepper["fn"] = step_world
        if slam is not None:
            # The taught station, fixed before the mission moves (Codex v3.8
            # impl P2): the planned delivery rear pose, the start's base height,
            # one ray cast with the truck left out. Truth enters here only.
            prior = site_poses(scenario.destination, geometry)["delivery"]
            prior_d = (prior.x_m, prior.y_m, prior.yaw_rad)
            # The station is taught at the true destination; the prior differs only
            # with --destination-prior-error-m (plan D7c δ runs).
            taught = site_poses(truth_destination, geometry)["delivery"]
            delivery_d = (taught.x_m, taught.y_m, taught.yaw_rad)
            offset_d = abs(args.rear_axle_offset_m)
            start_base, _ = robot.get_world_pose()
            base_d = np.array(
                [
                    delivery_d[0] + offset_d * math.cos(delivery_d[2]),
                    delivery_d[1] + offset_d * math.sin(delivery_d[2]),
                    float(start_base[2]),
                ]
            )
            quat_d = np.array([math.cos(delivery_d[2] / 2), 0.0, 0.0, math.sin(delivery_d[2] / 2)])
            ray_origin, ray_directions = planar_lidar.laser_rays_world(
                base_d, quat_d, laser_mount, beam_angles
            )
            ray_distances, ray_hits, ray_own = planar_lidar.cast_scan(
                ray_origin, ray_directions, scan_pattern.range_max_m, ignore_self=True
            )
            reference_ranges = scan_pattern.ranges_from_hits(ray_distances, ray_hits)
            slam["reference_points"] = laser_points(reference_ranges, beam_angles)
            if args.d7_docking or args.destination_prior_error_m:
                slam["docking"]["delivery_rear_reference"] = list(delivery_d)
            slam["docking"].update(
                delivery_rear_prior=list(prior_d),
                reference_beams=int(np.isfinite(reference_ranges).sum()),
                reference_self_hits_dropped=int(ray_own),
                reference_base_z_m=float(start_base[2]),
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
            # Ground truth checks, evaluates and renders; control below uses
            # `rear`/`signed_speed`, which --slam-feedback replaces.
            truth_rear = rear
            if slam is not None:
                rear = slam_rear(t)
                signed_speed = slam["tracker_speed"]
                slam["control"].append([t, *rear.tolist(), *truth_rear.tolist()])
            if obstacle is not None:
                obstacle["control_rear"] = tuple(float(v) for v in rear)  # known pallets fix against it
                if phase == "withdraw" and obstacle["layer"].corridor is not None and obstacle.get("withdraw_line"):
                    # Plan v10 D5 (4) (Codex stage-1 P1-2, 2nd P2-3): every tick, standing or
                    # arriving included, the certificate holds only while the truck stays on its
                    # straight -- lateral offset plus the yaw swing at the fork tips within the
                    # tracking bound; past it the truck stops.
                    (wx_, wy_, wyaw_), _ = obstacle["withdraw_line"]
                    lat_ = -(rear[0] - wx_) * math.sin(wyaw_) + (rear[1] - wy_) * math.cos(wyaw_)
                    dyaw_ = math.atan2(math.sin(rear[2] - wyaw_), math.cos(rear[2] - wyaw_))
                    dev_ = abs(lat_) + geometry.axle_to_fork_tip_m * abs(math.sin(dyaw_))
                    wt_ = state.setdefault("withdraw_tracking", {"max_deviation_m": 0.0})
                    wt_["max_deviation_m"] = max(wt_["max_deviation_m"], float(dev_))
                    require(dev_ <= float(obstacle["layer"].known_config.get("tracking_m", 0.006)),
                            f"withdraw_tracking_exceeded:{dev_:.4f}")
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
            # The real shape: unloaded, the body and the blades -- an object
            # between the blades is not a contact (plan D4, Codex L3c P2).
            truth_shape = footprint if loaded else unloaded_shape
            require(
                collision_free_pose(truth_rear, [], footprint, scenario.bounds)
                and not any(SHAPE_MEETS(o, truth_shape, tuple(float(v) for v in truth_rear)) for o in checked_obstacles),
                f"Actual truck/load footprint overlap in {phase}",
            )
            # A bar placed in the pallet frame inside the pallet's outline (N11,
            # N14) sits in a pocket by design: it is judged against the block
            # columns, not the outline that holds the pockets (Codex checkpoint
            # 9 P1: the outline check aborted those runs before the depth check
            # could answer).
            in_outline = []
            if new_obstacles is not None and new_obstacles.get("pallet_frame"):
                cp, sp = math.cos(pallet_yaw), math.sin(pallet_yaw)
                for r_ in new_obstacles["pallet_frame"]:
                    if r_ not in obstacles:
                        continue
                    u_ = (r_.x_m - ppos[0]) * cp + (r_.y_m - ppos[1]) * sp
                    v_ = -(r_.x_m - ppos[0]) * sp + (r_.y_m - ppos[1]) * cp
                    if abs(u_) < geometry.pallet_depth_m / 2 and abs(v_) < geometry.pallet_width_m / 2:
                        in_outline.append(r_)
            require(
                collision_free_pose(
                    [ppos[0], ppos[1], pallet_yaw],
                    [o for o in obstacles if not any(o is r_ for r_ in in_outline)],
                    Footprint(
                        geometry.pallet_depth_m / 2,
                        geometry.pallet_depth_m / 2,
                        geometry.pallet_width_m / 2,
                    ),
                    scenario.bounds,
                ),
                f"Measured pallet footprint overlap in {phase}",
            )
            if in_outline:
                # Each bar's eight corners in the pallet's frame from its full
                # pose (tilt included), boxed there and met against the canonical
                # solids as closed intervals -- touching counts (Codex checkpoint 11).
                qw_, qx_, qy_, qz_ = (float(v) for v in pq)
                rot_ = np.array([
                    [1 - 2 * (qy_ * qy_ + qz_ * qz_), 2 * (qx_ * qy_ - qz_ * qw_), 2 * (qx_ * qz_ + qy_ * qw_)],
                    [2 * (qx_ * qy_ + qz_ * qw_), 1 - 2 * (qx_ * qx_ + qz_ * qz_), 2 * (qy_ * qz_ - qx_ * qw_)],
                    [2 * (qx_ * qz_ - qy_ * qw_), 2 * (qy_ * qz_ + qx_ * qw_), 1 - 2 * (qx_ * qx_ + qy_ * qy_)],
                ])
                for r_ in in_outline:
                    h_ = new_obstacles["heights"].get(id(r_), 0.0)
                    # The bar as an oriented box in the pallet's frame; an AABB of
                    # its corners there reported a 2.9 mm gap as contact (Codex
                    # checkpoint 12), so the exact SAT decides.
                    centre_w = np.array([r_.x_m, r_.y_m, h_ / 2])
                    axes_w = np.array([[math.cos(r_.yaw_rad), -math.sin(r_.yaw_rad), 0.0],
                                       [math.sin(r_.yaw_rad), math.cos(r_.yaw_rad), 0.0],
                                       [0.0, 0.0, 1.0]])
                    centre_p = (centre_w - np.asarray(ppos, dtype=float)) @ rot_
                    axes_p = rot_.T @ axes_w
                    for box_ in PALLET_BOXES:
                        require(
                            not box_meets_obb(box_.centre_m, np.asarray(box_.size_m) / 2, centre_p, axes_p,
                                              (r_.length_m / 2, r_.width_m / 2, h_ / 2)),
                            f"Measured pallet solid overlap in {phase}: {box_.name}",
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
                # The final goal judged out of heading tolerance under SLAM:
                # brake to a stop and replan from there rather than end the
                # mission (S3 seed 3 failed its return at -0.054 rad vs 0.03).
                d7_docking_ = slam["docking"] if (args.d7_docking and slam is not None) else {}
                # Plan D7c: a docked straight or a re-approach is recovered by a
                # docking retry, judged before the stall/cusp budgets (Codex D7c P2).
                d7_owned = phase == "transport" and bool(
                    d7_docking_.get("accepted_any") or d7_docking_.get("reapproach")
                )
                dock_stop = (
                    phase == "transport"
                    and d7_docking_.get("status") == "armed"
                    and slam["tracker"].mode == "holding"
                    and not tracking.at_cusp
                    and not tracking.off_path
                    and (
                        (tracking.status == "failed" and tracking.failure == "endpoint_heading")
                        or (tracking.status == "tracking" and trackers[phase].remaining_to_goal_m() <= 1e-6)
                    )
                )
                if dock_stop:
                    # Plan D7c (a): stopped at the docking stop outside its tolerance
                    # (0.03 m, 0.05 rad) -- dock here, as an arrival would, rather
                    # than a stall replan (l8_measured seed 1 + N1: 37.5 mm lateral).
                    tracking = replace(tracking, status="braking", speed_mps=0.0)
                    dock_stop_ticks = dock_stop_ticks + 1 if slam["stop_now"] else 0
                    if dock_stop_ticks >= 120:
                        dock_stop_ticks = 0
                        state.setdefault("docking_retries", []).append(
                            {"trigger": "a", "time_s": t, "counted": False, "round": d7_docking_.get("round", 1),
                             "reapproach": bool(d7_docking_.get("reapproach")),
                             "position_error_m": tracking.position_error_m, "yaw_error_rad": tracking.yaw_error_rad}
                        )
                        dock_at_delivery_straight(t)
                        rear = slam_rear(t)  # a retry may have released the correction
                        slam["control"][-1][1:4] = rear.tolist()
                        tracking = trackers[phase].update(rear, signed_speed, dt)
                        last_tracking = tracking
                else:
                    dock_stop_ticks = 0
                goal_heading_miss = (
                    slam is not None
                    and not dock_stop
                    and phase in ("transport", "return_home")
                    and tracking.status == "failed"
                    and tracking.failure == "endpoint_heading"
                    and not tracking.at_cusp
                    and not tracking.off_path
                    and (len(state["stall_replans"]) < 2 or d7_owned)
                )
                if goal_heading_miss:
                    # Judged inside the 8 mm brake window, still creeping
                    # (S3 rerun: 6.6 mm left at -0.008 m/s): command zero and
                    # wait for the stop detector.
                    tracking = replace(tracking, status="braking", speed_mps=0.0)
                stalled = (
                    slam is not None
                    and phase in ("transport", "return_home")
                    and tracking.speed_mps == 0.0
                    and slam["stop_now"]
                    and (
                        goal_heading_miss
                        or (
                            tracking.status == "tracking"
                            and trackers[phase].remaining_to_goal_m() <= 1e-6
                        )
                        # Plan D7c (d): a re-approach stopped at a cusp short of its
                        # tolerance (status tracking, no failure -- Codex D7c impl P2-5).
                        or (
                            bool(d7_docking_.get("reapproach"))
                            and tracking.status == "tracking"
                            and tracking.at_cusp
                        )
                    )
                )
                if stalled and goal_heading_miss:
                    slam_stall_ticks = max(slam_stall_ticks, 119)
                slam_stall_ticks = slam_stall_ticks + 1 if stalled else 0
                if slam_stall_ticks >= 120 and d7_owned:
                    # Plan D7c (c)/(d): after a docked straight or during a
                    # re-approach, a retry instead of transport_recovery_after_docking.
                    slam_stall_ticks = 0
                    start_docking_retry(
                        t, "d" if d7_docking_.get("reapproach") else "c", "transport_recovery_after_docking"
                    )
                    rear = slam_rear(t)
                    slam["control"][-1][1:4] = rear.tolist()
                    tracking = trackers[phase].update(rear, signed_speed, dt)
                    last_tracking = tracking
                elif slam_stall_ticks >= 120 and len(state["stall_replans"]) < 2:
                    # Stopped at the end of the path but outside the goal
                    # tolerance (an estimate shift before the hold): plan the
                    # leg again from here, in the held frame (Codex v3.5 P2).
                    slam_stall_ticks = 0
                    start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
                    replan_start = time.monotonic()
                    # After a docking match the transport is held relative to the
                    # station: a Hybrid A* recovery there would drive a held
                    # detour in a frame mixed with world obstacles (Codex 5de6c3a
                    # P2) -- end the run instead.
                    require(
                        not (phase == "transport" and slam["docking"].get("accepted_any")),
                        "transport_recovery_after_docking",
                    )
                    if slam["tracker"].mode == "holding":
                        # No match is held against (the docking fell back): a
                        # recovery planned now may be a long manoeuvre -- a yaw
                        # fix at a 2 m radius is a full loop -- and must not run
                        # on dead reckoning (L3c v22 seed 1: 15 m held). The truck
                        # stands still: let SLAM back in first, as at a capture.
                        jump_m, jump_rad = slam["tracker"].release(odom_from_base=slam_odom_base())
                        slam["holds"].append(
                            {"phase": phase, "time_s": t, "event": "release_before_stall_replan",
                             "jump_m": jump_m, "jump_rad": jump_rad}
                        )
                        rear_now = slam_rear(world.current_time - initial_time)
                        if obstacle is not None and slam["tracker"].applied is not None:
                            # The new correction re-projects the grid before any
                            # plan reads it, and this tick goes on from the new
                            # pose (Codex checkpoint P1: no mixed frames).
                            obstacle["applied"] = tuple(float(v) for v in slam["tracker"].applied[0])
                            obstacle["version"] += 1
                            obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
                            obstacle["layer"].refresh(
                                t, obstacle["applied"], obstacle["version"],
                                current_pose=tuple(float(v) for v in rear_now),
                                path_ahead=np.array([rear_now]), loaded=loaded,
                            )
                        rear = np.asarray(rear_now, dtype=float)
                        start = PlanningPose(float(rear_now[0]), float(rear_now[1]), float(rear_now[2]))
                    if phase == "transport":
                        # After docking, the corrected drop (v3.8, Codex P1).
                        replanned = plan_transport_leg(
                            grid_world(slam.get("transport_scenario", scenario)), start, planner_config,
                            geometry=geometry, travel_config=travel_config,
                            **grid_kwargs(None),
                        )
                    else:
                        replanned = plan_return_leg(
                            grid_world(slam.get("return_scenario", scenario)), start, return_to_pose, planner_config,
                            geometry=geometry, travel_config=travel_config,
                            **grid_kwargs(None),
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
                            **path_stats(replanned),
                        }
                    )
                    require(replanned.success, f"stall_replan_failed:{replanned.status}")
                    paths[phase] = replanned
                    state.setdefault("paths", {})[phase] = path_record(replanned)
                    (args.output / "paths.json").write_text(
                        record_json(state["paths"], indent=2) + "\n"
                    )
                    add_path_display(
                        stage,
                        replanned,
                        "Transport" if phase == "transport" else "Return",
                        (1.0, 0.65, 0.04) if phase == "transport" else (0.55, 0.2, 0.85),
                    )
                    stall_config = trackers[phase].config
                    if obstacle is not None and obstacle.get("backoff", {}).get("phase") == phase:
                        # A replan accepted here ends a backoff too (Codex checkpoint 8 P2).
                        stall_config = obstacle.pop("backoff")["config"]
                        obstacle.setdefault("replan_wait", {}).pop(phase, None)
                    trackers[phase] = RearAxlePathTracker(
                        replanned.poses,
                        replanned.directions,
                        replanned.curvatures_inv_m,
                        stall_config,
                    )
                    if phase == "transport":
                        arm_docking()  # keep the stop before the delivery straight
                    if obstacle is not None:
                        # The no-progress watch compares remaining lengths on one path:
                        # a replaced path starts its own baseline next tick, after
                        # arm_docking, as obstacle replans already do (plan D7d -- a
                        # stall replan's 23 m loop was judged against the old path's
                        # near-zero remainder, l8_measured seed 1 + N1).
                        obstacle.setdefault("progress", {}).clear()
                    phase_started = t
                    tracking = trackers[phase].update(rear, signed_speed, dt)
                    last_tracking = tracking
                if (
                    tracking.status == "failed"
                    and phase == "transport"
                    and tracking.failure == "endpoint_heading"
                    and tracking.at_cusp
                    and not tracking.off_path
                    and (len(state["cusp_replans"]) < max_cusp_replans or d7_owned)
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
                    elif d7_owned:
                        # Plan D7c (c)/(d): a cusp after a docked straight or inside a
                        # re-approach is a docking retry, not a cusp replan.
                        cusp_stop_ticks = 0
                        start_docking_retry(
                            t, "d" if d7_docking_.get("reapproach") else "c", "transport_recovery_after_docking"
                        )
                        rear = slam_rear(t)
                        slam["control"][-1][1:4] = rear.tolist()
                        tracking = trackers[phase].update(rear, signed_speed, dt)
                        last_tracking = tracking
                    else:
                        cusp_stop_ticks = 0
                        require(
                            not (slam is not None and slam["docking"].get("accepted_any")),
                            "transport_recovery_after_docking",
                        )
                        replan_start = time.monotonic()
                        replanned = plan_transport_leg(
                            grid_world(slam.get("transport_scenario", scenario)
                            if slam is not None
                            else scenario),
                            PlanningPose(float(rear[0]), float(rear[1]), float(rear[2])),
                            planner_config,
                            geometry=geometry,
                            travel_config=travel_config,
                            **grid_kwargs(None),
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
                            state.setdefault("paths", {})[phase] = path_record(replanned)
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
                            if slam is not None:
                                arm_docking()
                            if obstacle is not None:
                                obstacle.setdefault("progress", {}).clear()  # plan D7d, as above
                            phase_started = t
                            tracking = trackers[phase].update(rear, signed_speed, dt)
                            last_tracking = tracking
                # Under SLAM, an observe leg judged out of heading at a cusp or
                # its end (S2 seed 1: -0.078 rad at a cusp of the near leg, three
                # versions running): brake, then plan the leg again from the
                # stop to the same target, at most twice per run.
                # Also a stop at a cusp outside its tolerance (S3 seed 1: 44.5 mm,
                # command 0, status tracking until the timeout) once it has
                # stood for a second.
                observe_stalled = (
                    slam is not None
                    and phase == "observe"
                    and tracking.status == "tracking"
                    and tracking.at_cusp
                    and tracking.speed_mps == 0.0
                    and slam["stop_now"]
                )
                observe_stall_ticks = observe_stall_ticks + 1 if observe_stalled else 0
                observe_miss = (
                    slam is not None
                    and phase == "observe"
                    and (
                        (
                            tracking.status == "failed"
                            and tracking.failure == "endpoint_heading"
                            and not tracking.off_path
                        )
                        or observe_stall_ticks >= 120
                    )
                    and len(state["observe_replans"]) < 2
                )
                if observe_miss:
                    observe_stall_ticks = 0
                    tracking = replace(tracking, status="braking", speed_mps=0.0)
                    if slam["stop_now"]:
                        target = PlanningPose(*(float(v) for v in paths["observe"].poses[-1]))
                        replan_start = time.monotonic()
                        replanned = plan_observation_leg(
                            grid_world(scenario),
                            target,
                            planner_config,
                            geometry=geometry,
                            start_rear=PlanningPose(float(rear[0]), float(rear[1]), float(rear[2])),
                            pickup_bounds=pickup_bounds,
                            extended=False,
                            **grid_kwargs("observe"),
                        )
                        state["observe_replans"].append(
                            {
                                "time_s": t,
                                "rear_pose": rear.tolist(),
                                "yaw_error_rad": last_tracking.yaw_error_rad,
                                "status": replanned.status,
                                "planning_wall_s": time.monotonic() - replan_start,
                            }
                        )
                        require(replanned.success, f"observe_replan_failed:{replanned.status}")
                        paths["observe"] = replanned
                        state.setdefault("paths", {})["observe"] = path_record(replanned)
                        (args.output / "paths.json").write_text(
                            record_json(state["paths"], indent=2) + "\n"
                        )
                        add_path_display(stage, replanned, "Observe", (0.2, 0.8, 0.8))
                        trackers["observe"] = RearAxlePathTracker(
                            replanned.poses,
                            replanned.directions,
                            replanned.curvatures_inv_m,
                            trackers["observe"].config,
                        )
                        phase_started = t
                        tracking = trackers["observe"].update(rear, signed_speed, dt)
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
                if (
                    phase == "observe" and tracking.status == "tracking"
                    and trackers[phase].remaining_to_goal_m() <= 1e-9 and abs(tracking.speed_mps) < 1e-12
                    and (slam["stop_now"] if slam is not None else abs(signed_speed) < 0.01)
                    and tracking.position_error_m <= OBSERVE_ARRIVAL_M
                    and abs(tracking.yaw_error_rad) <= OBSERVE_ARRIVAL_YAW_RAD
                ):
                    # The tracker stops at a path end it missed sideways and never
                    # arrives (L3c v38 seed 3: 6.1 cm off an arc's end). An
                    # observation waypoint is a viewpoint, not a docking pose:
                    # perception uses the pose the truck stands at. Standing 1 s
                    # within OBSERVE_ARRIVAL_M is arrival there.
                    loose = state.setdefault("observe_loose_arrival", {})
                    loose.setdefault("since_s", t)
                    if t - loose["since_s"] >= 1.0:
                        state.setdefault("observe_loose_arrivals", []).append(
                            {"time_s": t, "position_error_m": tracking.position_error_m,
                             "yaw_error_rad": tracking.yaw_error_rad}
                        )
                        state.pop("observe_loose_arrival", None)
                        tracking = replace(tracking, status="arrived")
                else:
                    state.pop("observe_loose_arrival", None)
                if (
                    tracking.status == "arrived" and obstacle is not None
                    and obstacle.get("backoff", {}).get("phase") == phase
                ):
                    # The end of a backoff is not the leg's arrival: stand and
                    # replan the leg from here, each second until a replan is
                    # accepted (D4 delta).
                    back_ = obstacle["backoff"]
                    if "end_s" not in back_:
                        back_["end_s"] = t
                        obstacle.setdefault("backoff_records", []).append(
                            {"phase": phase, "start_s": back_["time_s"], "end_s": t,
                             "rear": [float(v) for v in rear]}
                        )
                    if t - back_.get("forced_s", -1e9) >= 1.0:
                        back_["forced_s"] = t
                        obstacle["force_replan"] = phase
                    tracking = replace(tracking, status="tracking", speed_mps=0.0)
                    requested_speed = 0.0
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
                        pocket["capture"] = scene_input
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
                                    grid_world(scenario),
                                    waypoint,
                                    planner_config,
                                    geometry=geometry,
                                    start_rear=Pose2D(rear[0], rear[1], rear[2]),
                                    pickup_bounds=pickup_bounds,
                                    # Re-observation keeps the earlier ladder.
                                    extended=False,
                                    **grid_kwargs("observe"),
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
                            pocket["planning_pickup"] = planning_pickup
                            if obstacle is not None and obstacle["layer"].known_enabled:
                                # Each detection is a new relative fix: the age restarts
                                # even for the same value (Codex stage-1 P2-8).
                                known_pallet((planning_pickup.x_m, planning_pickup.y_m, planning_pickup.yaw_rad),
                                             obstacle["layer"].known_config.get("r_fix_pickup_m", 0.05),
                                             f"recognised:attempt{len(state.get('observation_attempts', []))}")
                            planning_start = time.monotonic()
                            planning_trace = []
                            plans = plan_transport(
                                grid_world(scenario),
                                planner_config,
                                geometry=geometry,
                                target_pickup=planning_pickup,
                                start_rear=start_rear_pose,
                                return_to=return_to_pose,
                                pickup_bounds=pickup_bounds,
                                travel_config=travel_config,
                                trace=planning_trace,
                                **grid_kwargs("mission", planning_pickup),
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
                                        # The box stays at 5 cm / 0.05 rad: the dry run
                                        # checks arrival only, not the swept footprint,
                                        # and a wider box admitted a collision (Codex v3.7 P1).
                                        max_lateral_m=0.05,
                                        max_yaw_rad=0.05,
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
                                    if args.pocket_check:
                                        build_pocket_check(float(straight.poses[-1][2]), rear, base)
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
                        if slam is not None and slam["docking"]["status"] == "armed":
                            dock_at_delivery_straight(t)
                            if args.d7_docking:
                                # A (b) retry may have released the correction: this
                                # tick's pose and control record follow (Codex D7c impl P2-3).
                                rear = slam_rear(t)
                                slam["control"][-1][1:4] = rear.tolist()
                        else:
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
            # The curvature the wheels actually hold: an emergency stop keeps it, so
            # the permission, the stop itself and the probe all use this one.
            steer_now = robot.get_joint_positions()[steers]
            half_track = args.drive_geometry.track_m / 2
            kappa = float(
                np.mean(
                    [
                        math.tan(steer_now[0]) / (args.drive_geometry.wheelbase_m + math.tan(steer_now[0]) * half_track),
                        math.tan(steer_now[1]) / (args.drive_geometry.wheelbase_m - math.tan(steer_now[1]) * half_track),
                    ]
                )
            )
            truth_speed = float(np.dot(velocity[:2], forward))
            obstacle_hold = False
            pocket_phase = False
            if obstacle is not None and phase in trackers and (
                requested_speed != 0.0 or abs(truth_speed) > 0.05 or obstacle.get("force_replan") == phase
            ):
                if slam is not None and slam["tracker"].applied is not None:
                    applied_now = tuple(float(v) for v in slam["tracker"].applied[0])
                else:
                    applied_now = (0.0, 0.0, 0.0)
                if applied_now != obstacle["applied"]:
                    # A correction change since the snapshot (a SLAM release): the
                    # grid is re-projected now, never mixed with the new pose
                    # (Codex L0b P1).
                    obstacle["version"] += 1
                    obstacle["applied"] = applied_now
                    obstacle["reprojections"] = obstacle.get("reprojections", 0) + 1
                    leg_now, leg_dir_now = trackers[phase].leg_ahead()
                    obstacle["layer"].refresh(
                        t, applied_now, obstacle["version"], current_pose=tuple(float(v) for v in rear),
                        path_ahead=leg_now, loaded=loaded, direction=leg_dir_now,
                    )
                if requested_speed != 0.0:
                    direction = 1 if requested_speed > 0 else -1
                else:
                    direction = 1 if truth_speed > 0 else -1
                allowed, why = obstacle["layer"].limit(
                    t, current_pose=tuple(float(v) for v in rear), curvature_inv_m=kappa,
                    direction=direction, loaded=loaded, cap_mps=max(abs(requested_speed), abs(truth_speed)),
                )
                dumped_phases = obstacle.setdefault("dumped_phases", set())
                if allowed == 0.0 and why in ("unknown", "occupied") and t > 5.0 and (
                    obstacle.get("dumps", 0) < 4 or phase not in dumped_phases
                ):
                    dumped_phases.add(phase)
                    # Diagnostics: the snapshot and the check inputs of the first unknown stops.
                    obstacle["dumps"] = obstacle.get("dumps", 0) + 1
                    snap_ = obstacle["layer"].snapshot
                    np.savez_compressed(
                        args.output / f"obstacle_dump_{obstacle['dumps']}.npz",
                        state=snap_.state, free_stamp=snap_.free_stamp,
                        origin=[snap_.origin_x_m, snap_.origin_y_m], res=snap_.resolution_m,
                        t=t, rear=np.asarray(rear, dtype=float), truth_rear=np.asarray(truth_rear, dtype=float),
                        kappa=kappa, direction=direction, loaded=loaded,
                        requested=requested_speed, truth_speed=truth_speed, phase=phase, why=why,
                        leg=(trackers[phase].leg_ahead()[0] if phase in trackers else np.zeros((0, 3))),
                        correction=np.asarray(obstacle["applied"], dtype=float), version=obstacle["version"],
                        stamp=snap_.stamp_s,
                        blocked_cells=np.asarray(
                            getattr(getattr(obstacle["layer"].permission, "last_estop", None), "blocked_cells", ()),
                            dtype=np.int64,
                        ).reshape(-1, 2),
                        path_blocked_cells=np.asarray(
                            getattr(obstacle["layer"].permission.path_check, "blocked_cells", ()),
                            dtype=np.int64,
                        ).reshape(-1, 2),
                        **{
                            f"raw_{name}_{part}": np.asarray(arr, dtype=float)
                            for name, values in obstacle.get("last_raw", ({}, None, None))[0].items()
                            for part, arr in zip(("d", "hit", "own", "limit"), values)
                        },
                        raw_base=obstacle.get("last_raw", ({}, np.zeros(3), np.zeros(4)))[1],
                        raw_q=obstacle.get("last_raw", ({}, np.zeros(3), np.zeros(4)))[2],
                    )
                ticks = obstacle["ticks"]
                ticks[phase] = ticks.get(phase, 0) + 1
                low = obstacle["min_allowed"]
                low[phase] = min(low.get(phase, float("inf")), allowed)
                docking_straight = (phase == "approach" and trackers[phase].remaining_to_goal_m() <= geometry.alignment_straight_m) or (
                    phase == "transport" and trackers[phase].remaining_to_goal_m() <= geometry.delivery_straight_m
                )
                if obstacle.get("backoff", {}).get("phase") == phase:
                    # A backoff is never the delivery straight, whatever its
                    # remaining length: the permission acts on it (Codex
                    # checkpoint 8 P1).
                    docking_straight = False
                if slam is not None and slam["docking"].get("reapproach"):
                    # Nor is a D7c re-approach (Codex D7c review P1-2): the
                    # permission, and the safety evaluation, act on all of it.
                    docking_straight = False
                acting = (
                    phase in ("observe", "approach", "transport", "return_home")
                    or (phase == "withdraw" and obstacle["layer"].known_enabled
                        and obstacle["layer"].known_config.get("withdraw_mode", "certified") == "certified")  # D5 corridor
                ) and not docking_straight
                pocket_phase = args.pocket_check and (phase == "insert" or (phase == "approach" and docking_straight))
                if pocket_phase:
                    # D5: the approach straight and the insertion run under the
                    # permission too; inside the pallet region the depth check
                    # answers (its region is waived in the grid only while set).
                    acting = True
                    if pocket["check"] is None:
                        allowed, why = 0.0, "pocket_missing"
                    elif pocket.get("await_still_frame"):
                        allowed, why = 0.0, "pocket_settle"
                    elif (
                        phase == "approach" and not pocket.get("mid_still_done")
                        and trackers[phase].remaining_to_goal_m() <= 0.45
                    ):
                        # D5: one standing frame with the camera about 1.2 m from
                        # the face. Closer, the band's sides before the face
                        # leave the view; farther, the depth noise is wider than
                        # their 5 cm to the face; passing at speed, the frame's
                        # pose uncertainty is (L3c v27 seed 2: 27 voxels never seen).
                        pocket["mid_still_done"] = True
                        pocket["await_still_frame"] = True
                        allowed, why = 0.0, "pocket_settle"
                    else:
                        lp, wp = pocket["check"].limit(
                            t, tuple(float(v) for v in rear), kappa, direction,
                            max(abs(requested_speed), abs(truth_speed)),
                            float(robot.get_joint_positions()[lift_index[0]]),
                        )
                        if lp < allowed:
                            allowed, why = lp, wp
                        if phase == "approach" and not pocket.get("mid_still_done"):
                            # The standing frame is a planned stop: brake into it at
                            # MID_STILL_DECEL_MPS2, not as a step (L3c v35 seed 4: a step
                            # from 0.53 m/s locked an odometry wheel, the held pose fell
                            # 1.85 cm behind and the face read as an obstacle).
                            cap = max(0.05, math.sqrt(2.0 * MID_STILL_DECEL_MPS2
                                                      * max(0.0, trackers[phase].remaining_to_goal_m() - 0.45)))
                            if cap < allowed:
                                allowed, why = cap, "pocket_settle_brake"
                        if wp != "ok" and pocket.get("last_reason") != wp:
                            # Kept now: a stop that ends the run must leave its evidence.
                            pocket["last_reason"] = wp
                            state.setdefault("pocket_check", {}).update(
                                pocket["check"].summary(), frames_read=pocket["frames_read"], last_reason=wp,
                                last_reason_s=t, last_reason_rear=[float(v) for v in rear],
                            )
                    # A docking segment does not replan (plan D4): held at 0 for
                    # 5 s, the check's reason ends the run.
                    if allowed == 0.0:
                        pocket.setdefault("zero_since", t)
                        require(t - pocket["zero_since"] < 5.0, f"pocket_stop in {phase}: {why}")
                    else:
                        pocket.pop("zero_since", None)
                # Plan D4: an occupied cell on the path ahead stops the truck where
                # it is, so the replan starts with room to manoeuvre -- creeping up
                # to the obstacle left a 2 m-radius truck boxed in (L3b v24-v27).
                # 0.3 s of a blocked path first: a passing mark does not stop it.
                check_now = obstacle["layer"].permission.path_check
                path_blocked = (
                    grid_planning and acting and phase in ("observe", "transport", "return_home")
                    and check_now is not None and check_now.blocked == "occupied"
                )
                obstacle["path_blocked_ticks"] = obstacle.get("path_blocked_ticks", 0) + 1 if path_blocked else 0
                # The permission's own limit, applied to the wheels as a step
                # (the stop it was measured with); the path-blocked stop below
                # is a planned one and brakes on the slew.
                obstacle["permission_cap"] = (t, float(allowed)) if (args.obstacle_act and acting) else None
                if obstacle["permission_cap"] is not None and allowed == 0.0 and why.startswith("sensor_silent"):
                    # N9 deadlines: the permission's zero reaches the wheels as a
                    # step through the final cap whatever the tracker asks (Codex
                    # checkpoint 11): the first such tick, and the stop it ends in.
                    trace = obstacle.setdefault("silence_trace", {})
                    trace.setdefault("first_zero_s", t)
                    trace.setdefault("reason", why)
                    trace.setdefault("speed_at_zero_mps", abs(truth_speed))
                    trace.setdefault("rear_at_zero", [float(v) for v in truth_rear])
                    # The stop is the start of the final standing: a slow instant followed by
                    # motion is not it (Codex stage-1 3rd P2), so moving again clears it.
                    if abs(truth_speed) < 0.01:
                        if "stopped_s" not in trace:
                            trace["stopped_s"] = t
                            trace["rear_stopped"] = [float(v) for v in truth_rear]
                    elif "stopped_s" in trace:
                        trace.setdefault("interrupted_stops_s", []).append(trace.pop("stopped_s"))
                        trace.pop("rear_stopped", None)
                    # The N9 outcome: stopped and standing 1 s (Codex stage-1 P1-1, 2nd P2-4) ends the run.
                    require(not ("stopped_s" in trace and t - trace["stopped_s"] >= 1.0), "sensor_silence_stopped")
                if obstacle["path_blocked_ticks"] >= 36 and allowed > 0.0:
                    allowed, why = 0.0, "occupied"
                if abs(requested_speed) > allowed + 1e-9:
                    obstacle["slowed"][phase] = obstacle["slowed"].get(phase, 0) + 1
                    key = f"{phase}:{why}"
                    obstacle["reasons"][key] = obstacle["reasons"].get(key, 0) + 1
                    if args.obstacle_act and acting:
                        requested_speed = math.copysign(allowed, requested_speed)
                        # Zero allowed: the emergency stop the check assumed --
                        # wheels to zero, steering held (Codex L0b P1).
                        obstacle_hold = allowed == 0.0
                # Stop, replan, resume (plan D4): blocked by an obstacle and standing.
                stopped_now = slam["stop_now"] if slam is not None else float(np.linalg.norm(velocity[:2])) < 0.01
                if (
                    grid_planning
                    and acting
                    and phase in ("observe", "transport", "return_home")
                    and allowed == 0.0
                    and why == "occupied"
                    and stopped_now
                ):
                    obstacle["blocked_ticks"] = obstacle.get("blocked_ticks", 0) + 1
                else:
                    obstacle["blocked_ticks"] = 0
                progress_left = trackers[phase].remaining_to_goal_m()
                watch = obstacle.setdefault("progress", {})
                if watch.get("phase") != phase or not acting or progress_left < watch["best_m"] - 1.0:
                    watch.update(phase=phase, best_m=progress_left, since_s=t)
                require(
                    not (grid_planning and acting and t - watch["since_s"] > 60.0),
                    f"obstacle_no_progress in {phase}",
                )
                if obstacle["blocked_ticks"] >= 120 or obstacle.pop("force_replan", None) == phase:
                    obstacle["blocked_ticks"] = 0
                    if phase == "transport" and slam is not None and slam["docking"].get("reapproach"):
                        # Plan D7c: a re-approach path is replaced released, never as
                        # a long held manoeuvre (Codex D7c re-review P2-5).
                        released_ = release_and_carry(t)
                        if released_["released"]:
                            rear = slam_rear(t)
                            slam["control"][-1][1:4] = rear.tolist()
                    start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
                    replan_start = time.monotonic()
                    tight = replace(planner_config, clearance_m=0.0)
                    tight_travel = replace(travel_config, clearance_m=0.0) if travel_config is not None else None
                    # One shared 20 s budget for the whole replan (plan D3):
                    # retries and other candidates included.
                    obstacle["deadline"] = time.monotonic() + 20.0
                    if phase == "observe" and obstacle.get("pickup_estimate") is not None:
                        # After recognition half the shared budget goes to this leg,
                        # the rest to the mission replan below if it fails
                        # (Codex checkpoint P2: one 20 s budget for the whole replan).
                        obstacle["deadline"] = replan_start + 10.0
                    if phase == "observe":
                        target = PlanningPose(*(float(v) for v in paths["observe"].poses[-1]))
                        replanned = plan_observation_leg(
                            grid_world(scenario), target, planner_config, geometry=geometry,
                            start_rear=start, pickup_bounds=pickup_bounds, extended=False,
                            **grid_kwargs("observe"),
                        )
                    elif phase == "transport":
                        replanned = transport_replan(start, planner_config, travel_config)
                    else:
                        back = scenario if slam is None else slam.get("return_scenario", slam.get("transport_scenario", scenario))
                        replanned = plan_return_leg(
                            grid_world(back), start, return_to_pose, planner_config, geometry=geometry,
                            travel_config=travel_config, **grid_kwargs("return", back.destination),
                        )
                    if replanned.status in ("invalid_start", "no_path"):
                        # The truck stopped closer to the obstacle than the planning
                        # clearance (the stop guards the volume ahead, not the
                        # sides): replan once with no clearance. The grid's swelling
                        # already holds the placement error and the permission
                        # still guards every tick.
                        if phase == "observe":
                            replanned = plan_observation_leg(
                                grid_world(scenario), target, tight, geometry=geometry,
                                start_rear=start, pickup_bounds=pickup_bounds, extended=False,
                                **grid_kwargs("observe"),
                            )
                        elif phase == "transport":
                            replanned = transport_replan(start, tight, tight_travel)
                        else:
                            replanned = plan_return_leg(
                                grid_world(back), start, return_to_pose, tight, geometry=geometry,
                                travel_config=tight_travel, **grid_kwargs("return", back.destination),
                            )
                        obstacle.setdefault("tight_replans", 0)
                        obstacle["tight_replans"] += 1
                    if not replanned.success:
                        # Diagnostics: the inputs of the failed replan, for a CPU replay.
                        failed_grid = grid_kwargs("observe" if phase == "observe" else None)
                        np.savez_compressed(
                            args.output / f"obstacle_replan_fail_{len(obstacle['replans'])}.npz",
                            occupied=failed_grid["occupancy"].occupied,
                            origin=[failed_grid["occupancy"].origin_x_m, failed_grid["occupancy"].origin_y_m],
                            res=failed_grid["occupancy"].resolution_m,
                            start=[start.x_m, start.y_m, start.yaw_rad],
                            target=[target.x_m, target.y_m, target.yaw_rad] if phase == "observe" else [np.nan] * 3,
                            phase=phase, status=replanned.status,
                            pickup_obstacle=(
                                [failed_grid["pickup_obstacle"].x_m, failed_grid["pickup_obstacle"].y_m,
                                 failed_grid["pickup_obstacle"].length_m, failed_grid["pickup_obstacle"].width_m,
                                 failed_grid["pickup_obstacle"].yaw_rad] if "pickup_obstacle" in failed_grid else []
                            ),
                        )
                    if not replanned.success and phase == "observe" and obstacle.get("pickup_estimate") is None:
                        # Before recognition only: the way to this observation point
                        # is blocked, so try the remaining observation candidates
                        # from here, as an unreachable candidate is handled at the
                        # first plan. After recognition the leg goes to the near
                        # capture and another observation point would be wrong.
                        while next_candidate_index < len(args.observation_waypoints):
                            candidate_index = next_candidate_index
                            next_candidate_index += 1
                            coordinates = args.observation_waypoints[candidate_index]
                            candidate_plan = plan_observation_leg(
                                grid_world(scenario), PlanningPose(*(float(v) for v in coordinates)),
                                tight, geometry=geometry, start_rear=start,
                                pickup_bounds=pickup_bounds, extended=False, **grid_kwargs("observe"),
                            )
                            state["observation_candidates"].append(
                                {"candidate_index": candidate_index, "pose": list(coordinates),
                                 "start_rear": [float(v) for v in rear], "success": candidate_plan.success,
                                 "status": candidate_plan.status, "after": "obstacle_block",
                                 "analytic_expansion_interval": candidate_plan.analytic_expansion_interval,
                                 "search_attempts": [list(entry) for entry in candidate_plan.search_attempts]}
                            )
                            if candidate_plan.success:
                                replanned = candidate_plan
                                state["observation_waypoint_selected"] = list(coordinates)
                                break
                    if (
                        not replanned.success and phase == "observe" and obstacle.get("pickup_estimate") is not None
                        and pocket.get("planning_pickup") is not None
                    ):
                        # After recognition: the near-capture waypoint came from a
                        # mission plan made on an earlier grid and may now sit
                        # against an obstacle that has grown in view (L3b v24: valid
                        # only at zero clearance, no path in 120 000 expansions).
                        # The pallet, not the waypoint, is the goal: plan the
                        # mission again from here, and its final straight gives the
                        # new waypoint. The rest of the shared budget.
                        obstacle["deadline"] = replan_start + 20.0
                        planning_trace = []
                        mission_again = plan_transport(
                            grid_world(scenario), planner_config, geometry=geometry,
                            target_pickup=pocket["planning_pickup"], start_rear=start,
                            return_to=return_to_pose, pickup_bounds=pickup_bounds, travel_config=travel_config,
                            trace=planning_trace, **grid_kwargs("mission", pocket["planning_pickup"]),
                        )
                        if "invalid_start" in mission_again.status or "invalid_goal" in mission_again.status:
                            # The waypoint next to an obstacle that has grown in
                            # view is valid only without the planning clearance
                            # (L3b v26); the grid's swelling holds the placement error.
                            mission_again = plan_transport(
                                grid_world(scenario), tight, geometry=geometry,
                                target_pickup=pocket["planning_pickup"], start_rear=start,
                                return_to=return_to_pose, pickup_bounds=pickup_bounds, travel_config=tight_travel,
                                trace=planning_trace, **grid_kwargs("mission", pocket["planning_pickup"]),
                            )
                        near_again = (
                            final_straight_prefix(mission_again.approach, geometry.alignment_straight_m)
                            if mission_again.success else None
                        )
                        obstacle.setdefault("mission_replans", []).append(
                            {"time_s": t, "status": mission_again.status, "near_found": near_again is not None,
                             "trace": planning_trace}
                        )
                        if near_again is not None:
                            for name in mission_stages:
                                paths[name] = getattr(mission_again, name)
                                trackers[name] = RearAxlePathTracker(
                                    paths[name].poses, paths[name].directions, paths[name].curvatures_inv_m,
                                    trackers[name].config,
                                )
                                state["paths"][name] = path_record(paths[name])
                            apply_tracker_profile()
                            replanned = near_again
                            state["near_capture"]["waypoint"] = near_again.poses[-1].tolist()
                            state["observation_waypoint_selected"] = near_again.poses[-1].tolist()
                    obstacle["deadline"] = None
                    obstacle["replans"].append(
                        {
                            "phase": phase,
                            "time_s": t,
                            "rear_pose": [float(v) for v in rear],
                            "status": replanned.status,
                            "planning_wall_s": time.monotonic() - replan_start,
                            **path_stats(replanned),
                        }
                    )
                    if not replanned.success:
                        # Standing, the grid keeps updating: a failed replan is
                        # retried when the blocked count comes round again (1 s),
                        # five failures per leg at most; the no-progress watch
                        # bounds the wait too (L3c v45 seed 5: no_path on the way
                        # home after the delivery).
                        obstacle["replans"][-1]["failed"] = True
                        failures = sum(1 for r in obstacle["replans"] if r["phase"] == phase and r.get("failed"))
                        require(failures < 5, f"obstacle_replan_failed in {phase}: {replanned.status}")
                        # Held until a replan succeeds (D4: stop, replan, resume
                        # only on a plan -- Codex checkpoint 7 P2).
                        obstacle.setdefault("replan_wait", {})[phase] = True
                        requested_speed = 0.0
                        backs = obstacle.setdefault("backoffs", {})
                        if phase in ("transport", "return_home") and backs.get(phase, 0) < BACKOFF_PER_LEG:
                            # D4 delta: stood with the outline already on the
                            # blocking cells, no forward plan starts (L3c v46 seed
                            # 5). Back off straight, against the leg's direction,
                            # under the permission every tick (the stop volume of
                            # the reverse is checked; unobserved space stops it),
                            # then replan from there.
                            backs[phase] = backs.get(phase, 0) + 1
                            # The blocked leg's direction, kept across repeated
                            # backoffs (a second one must not undo the first,
                            # Codex checkpoint 8 P2).
                            leg_dir = obstacle.get("backoff", {}).get("leg_dir")
                            if leg_dir is None:
                                leg_dir = trackers[phase].leg_ahead()[1] or 1
                            d = -1 if leg_dir >= 0 else 1
                            n_ = max(1, int(math.ceil(BACKOFF_M / 0.04)))
                            fr = np.linspace(0.0, 1.0, n_ + 1)
                            yaw0 = float(rear[2])
                            back = PlanResult(
                                True, "backoff",
                                np.column_stack((float(rear[0]) + fr * d * BACKOFF_M * math.cos(yaw0),
                                                 float(rear[1]) + fr * d * BACKOFF_M * math.sin(yaw0),
                                                 np.full(n_ + 1, yaw0))),
                                np.full(n_ + 1, d, dtype=np.int8), np.zeros(n_ + 1), BACKOFF_M, 0,
                            )
                            # The leg's own tracker config survives repeated backoffs.
                            config0 = obstacle.get("backoff", {}).get("config", trackers[phase].config)
                            obstacle["backoff"] = {"phase": phase, "config": config0, "time_s": t, "leg_dir": leg_dir}
                            obstacle["replans"][-1]["backoff"] = True
                            obstacle["replan_wait"].pop(phase, None)
                            paths[phase] = back
                            trackers[phase] = RearAxlePathTracker(
                                back.poses, back.directions, back.curvatures_inv_m,
                                replace(config0, cruise_speed_mps=BACKOFF_SPEED_MPS),
                            )
                            phase_started = t  # the backoff's own tracking timeout
                    elif same_path(replanned, paths[phase], trackers[phase].remaining_to_goal_m()):
                        # D4: the same path again is a wait, not a retry -- it
                        # neither restarts the tracker nor the no-progress watch
                        # and does not count against the retry limits (Codex P2).
                        obstacle["replans"][-1]["same_path"] = True
                        obstacle.setdefault("replan_wait", {}).pop(phase, None)
                        requested_speed = 0.0
                    else:
                        # D4: a replan may rightly be longer (a detour), so the
                        # no-progress watch restarts on the new path; the
                        # oscillation it also cut is bounded by a per-leg replan
                        # count (L3c v34 seed 4: two good detours tripped the 60 s watch).
                        # Checked once the path is known to differ: a same-path wait
                        # never counts (Codex checkpoint 6 P2).
                        recent = [r for r in obstacle["replans"]
                                  if r["phase"] == phase and t - r["time_s"] < 30.0
                                  and not r.get("same_path") and not r.get("failed")]
                        require(len(recent) <= 3, f"obstacle_blocked in {phase}: {len(recent)} replans in 30 s")
                        leg_replans = sum(1 for r in obstacle["replans"] if r["phase"] == phase and not r.get("same_path") and not r.get("failed"))
                        require(leg_replans <= 10, f"obstacle_replan_limit in {phase}: {leg_replans}")
                        obstacle.setdefault("progress", {}).clear()
                        obstacle.setdefault("replan_wait", {}).pop(phase, None)
                        paths[phase] = replanned
                        state.setdefault("paths", {})[phase] = path_record(replanned)
                        (args.output / "paths.json").write_text(record_json(state["paths"], indent=2) + "\n")
                        config_ = trackers[phase].config
                        if obstacle.get("backoff", {}).get("phase") == phase:
                            config_ = obstacle.pop("backoff")["config"]
                        trackers[phase] = RearAxlePathTracker(
                            replanned.poses, replanned.directions, replanned.curvatures_inv_m, config_
                        )
                        if phase == "transport" and slam is not None and not slam["docking"].get("reapproach"):
                            arm_docking()
                        phase_started = t
                        requested_speed = 0.0
                # Evaluation only (truth): every moving tick, each ground-truth
                # obstacle in the steering-held stopping volume (with the envelope)
                # while the permission allowed this speed (Codex L0b P2).
                if acting and abs(truth_speed) > 0.05:
                    pconf = obstacle["layer"].permission.config
                    stop_len = pconf.stopping.distance_m(truth_speed)
                    arc = ARC_POSES(tuple(float(v) for v in truth_rear), kappa, 1 if truth_speed > 0 else -1, stop_len, 0.025)[0]
                    shape_now, _ = obstacle["layer"].footprints(loaded)
                    # A bounding-circle prefilter (exact: a rectangle farther than
                    # the shape's reach plus its own half diagonal from every
                    # pose cannot meet it) -- this evaluator was a sixth of the
                    # wall time (L3c v50 profile).
                    arc_xy = np.asarray(arc, dtype=float)[1:, :2]
                    # The margin inflates length and width both: it goes inside the hypot.
                    reach_now = max(
                        math.hypot(abs(lon_) + max(fp_.front_m, fp_.rear_m) + pconf.envelope_offset_m,
                                   abs(lat_) + fp_.half_width_m + pconf.envelope_offset_m)
                        for fp_, lat_, lon_ in PARTS_OF(shape_now)
                    )
                    for oid, rect in enumerate(checked_obstacles):
                        if len(arc_xy) and float(np.min(np.hypot(arc_xy[:, 0] - rect.x_m, arc_xy[:, 1] - rect.y_m))) > (
                            reach_now + math.hypot(rect.length_m, rect.width_m) / 2 + 1e-6
                        ):
                            continue
                        if pocket_phase and rect is pickup_obstacle:
                            # Blades in the pockets meet the pallet's 2D outline by
                            # design; fork/pallet contact is the 3D insertion guard's.
                            continue
                        # The shape the permission checks (body + blades unloaded).
                        if any(SHAPE_MEETS(rect, shape_now, tuple(pose_), pconf.envelope_offset_m) for pose_ in arc[1:]):
                            obstacle["events"] += 1
                            per = obstacle.setdefault("event_objects", {})
                            per[oid] = per.get(oid, 0) + 1
                            if allowed >= abs(truth_speed) - 1e-9:
                                obstacle["unpermitted"].append(
                                    {"time_s": t, "phase": phase, "speed_mps": truth_speed, "allowed_mps": allowed,
                                     "reason": why, "object": oid}
                                )
            if (
                grid_planning
                and phase in ("transport", "return_home")
                and phase in trackers
                and not obstacle.setdefault("live_replanned", {}).get(phase)
                and t >= obstacle.get("live_retry_after_s", 0.0)
            ):
                # The mission plan's travel legs saw the pickup pallet's band
                # cleared for the carried part; the executed leg is planned now,
                # on the live grid, where the carried pallet is the truck's own.
                obstacle["live_replanned"][phase] = True
                start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))
                obstacle["deadline"] = time.monotonic() + 20.0
                if phase == "transport":
                    sc_ = slam.get("transport_scenario", scenario) if slam is not None else scenario
                    live = plan_transport_leg(grid_world(sc_), start, planner_config, geometry=geometry,
                                              travel_config=travel_config, **grid_kwargs(None))
                    if live.status in ("invalid_start", "no_path"):
                        live = plan_transport_leg(grid_world(sc_), start, replace(planner_config, clearance_m=0.0),
                                                  geometry=geometry,
                                                  travel_config=replace(travel_config, clearance_m=0.0) if travel_config is not None else None,
                                                  **grid_kwargs(None))
                else:
                    sc_ = scenario if slam is None else slam.get("return_scenario", slam.get("transport_scenario", scenario))
                    live = plan_return_leg(grid_world(sc_), start, return_to_pose, planner_config, geometry=geometry,
                                           travel_config=travel_config, **grid_kwargs("return", sc_.destination))
                    if live.status in ("invalid_start", "no_path"):
                        live = plan_return_leg(grid_world(sc_), start, return_to_pose, replace(planner_config, clearance_m=0.0),
                                               geometry=geometry,
                                               travel_config=replace(travel_config, clearance_m=0.0) if travel_config is not None else None,
                                               **grid_kwargs("return", sc_.destination))
                obstacle["deadline"] = None
                obstacle.setdefault("live_plans", []).append({"phase": phase, "time_s": t, "status": live.status})
                if not live.success:
                    # Standing, the grid keeps updating (the cells the carried
                    # pallet stood on clear once seen again): retry each second,
                    # five times, before giving up (L3c v41 seed 4: no_path at the
                    # first transport plan, which v38 planned through).
                    tries = sum(1 for lp in obstacle["live_plans"] if lp["phase"] == phase)
                    require(tries < 5, f"obstacle_live_plan_failed in {phase}: {live.status}")
                    obstacle["live_replanned"][phase] = False
                    obstacle["live_retry_after_s"] = t + 1.0
                else:
                    paths[phase] = live
                    state["paths"][phase] = path_record(live)
                    (args.output / "paths.json").write_text(record_json(state["paths"], indent=2) + "\n")
                    trackers[phase] = RearAxlePathTracker(live.poses, live.directions, live.curvatures_inv_m, trackers[phase].config)
                    if phase == "transport" and slam is not None:
                        arm_docking()
                    phase_started = t
                requested_speed = 0.0
            elif (
                grid_planning and phase in ("transport", "return_home") and phase in trackers
                and not obstacle["live_replanned"].get(phase)
            ):
                requested_speed = 0.0  # waiting to retry the live plan: stand
            if obstacle is not None and obstacle.get("replan_wait", {}).get(phase):
                check_ = obstacle["layer"].permission.path_check
                if (
                    obstacle.get("backoff", {}).get("phase") != phase
                    and check_ is not None and check_.blocked is None
                ):
                    # The path ahead verified again -- no OCCUPIED and no UNKNOWN
                    # cell (a zero blocked count is not that, Codex checkpoint 8
                    # P2): the existing path resumes.
                    obstacle["replan_wait"].pop(phase, None)
                else:
                    requested_speed = 0.0  # a blocked-path replan failed: stand until one succeeds
            if new_obstacles is not None and phase in trackers:
                leg, leg_direction = trackers[phase].leg_ahead()
                driven = float(trackers[phase]._distance[-1]) - trackers[phase].remaining_to_goal_m()
                pallet_face = None
                if "approach" in paths and phase in ("approach", "insert"):
                    # N11/N12/N14 place bars in the pallet frame: the approach
                    # face centre of the real pallet, the insertion heading of
                    # the approach's end (a scenario placement, never a planner
                    # input -- plan D6).
                    hy = float(paths["approach"].poses[-1][2])
                    half = geometry.pallet_depth_m / 2
                    pallet_face = (float(ppos[0]) - half * math.cos(hy), float(ppos[1]) - half * math.sin(hy), hy)
                if obstacle is not None and obstacle.get("backoff", {}).get("phase") == phase:
                    leg_direction = 0  # a backoff is not a leg of the phase (Codex checkpoint 9 P1)
                def n_sweep_test(box, path_, loaded_=loaded):
                    # N3: does the truck's own sweep along the path ahead (first
                    # 4 m) meet the box, and would a straight run of that length
                    # from here meet it? (Codex checkpoint 14)
                    from forklift_core.planning.geometry import Bounds as SBounds

                    bx_, by_, byaw_, bsize_ = box
                    rect_ = Rectangle(float(bx_), float(by_), float(bsize_[0]), float(bsize_[1]), float(byaw_))
                    fp_ = geometry.loaded_footprint if loaded_ else geometry.unloaded_footprint
                    chk_ = FootprintCollisionChecker([rect_], fp_, SBounds(-1e6, 1e6, -1e6, 1e6))
                    pts_ = np.asarray(path_, dtype=float)
                    seg_ = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(pts_[:, :2], axis=0).T))))
                    pts_ = pts_[seg_ <= 4.0]
                    meets_path_ = any(not chk_.free(tuple(p_)) for p_ in pts_)
                    x0_, y0_, h0_ = (float(v) for v in pts_[0])
                    # The straight runs the way the leg drives: backwards on a
                    # reverse leg (Codex checkpoint 15).
                    sgn_ = -1.0 if leg_direction < 0 else 1.0
                    line_ = [(x0_ + sgn_ * d_ * math.cos(h0_), y0_ + sgn_ * d_ * math.sin(h0_), h0_)
                             for d_ in np.arange(0.0, min(4.0, seg_[-1]) + 1e-9, 0.025)]
                    meets_line_ = any(not chk_.free(p_) for p_ in line_)
                    return meets_path_, meets_line_

                for action in new_obstacles["schedule"].update(
                    t, phase, driven, leg, leg_direction=int(leg_direction), pallet_face=pallet_face,
                    sweep_test=n_sweep_test,
                ):
                    if action[0] == "spawn":
                        from pxr import Gf as NGf, UsdGeom as NUsdGeom, UsdPhysics as NUsdPhysics

                        _, oid, ox, oy, oyaw, osize = action
                        path_ = f"/World/NewObstacle_{oid}"
                        cube = NUsdGeom.Cube.Define(stage, path_)
                        cube.CreateSizeAttr(1.0)
                        xf = NUsdGeom.XformCommonAPI(cube)
                        xf.SetTranslate(NGf.Vec3d(float(ox), float(oy), float(osize[2]) / 2))
                        xf.SetRotate(NGf.Vec3f(0.0, 0.0, float(math.degrees(oyaw))))
                        xf.SetScale(NGf.Vec3f(*(float(v) for v in osize)))
                        NUsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                        rect = Rectangle(float(ox), float(oy), float(osize[0]), float(osize[1]), float(oyaw))
                        new_obstacles["rects"][oid] = rect
                        obstacles.append(rect)  # ground truth for the evaluator only
                        if any(e_.id == oid and e_.frame == "pallet" for e_ in new_obstacles["schedule"].events):
                            # Judged against the pallet's solids, not its outline
                            # (Codex checkpoint 9 P1).
                            new_obstacles.setdefault("pallet_frame", []).append(rect)
                            new_obstacles.setdefault("heights", {})[id(rect)] = float(osize[2])
                        if obstacle is not None and obstacle["layer"].shadow is not None:
                            _, own_ = obstacle["layer"].footprints(loaded)
                            reach = obstacle["layer"].shadow.band_m + 0.05 * math.sqrt(2)
                            grown = Footprint(own_.front_m + reach, own_.rear_m + reach, own_.half_width_m + reach)
                            # Geometry only: unbounded hall, so a grown outline past
                            # the hall edge is not taken for an overlap (Codex P2).
                            from forklift_core.planning.geometry import Bounds as NBounds

                            open_hall = NBounds(-1e6, 1e6, -1e6, 1e6)
                            if not FootprintCollisionChecker([rect], grown, open_hall).free(tuple(float(v) for v in truth_rear)):
                                new_obstacles["band_overlaps"].append({"time_s": t, "event": oid})
                    elif action[0] == "remove":
                        oid = action[1]
                        stage.RemovePrim(f"/World/NewObstacle_{oid}")
                        rect = new_obstacles["rects"].pop(oid, None)
                        if rect in obstacles:
                            obstacles.remove(rect)
            estop_holding = estop is not None and estop.update(
                t=t,
                phase=phase,
                phase_elapsed_s=t - phase_started,
                rear=truth_rear,
                speed_mps=float(np.dot(velocity[:2], forward)),
                loaded=loaded,
                curvature_inv_m=kappa,
                extra=[
                    *robot.get_joint_positions()[steers],
                    float(robot.get_joint_positions()[lift_index[0]]),
                    *ppos,
                    pallet_yaw,
                ],
            )
            if estop_holding or obstacle_hold:
                # Zero every wheel target at once, steering held (plan D4).
                requested_speed = 0.0
            drive = ackermann_command(requested_speed, curvature, drive_geometry)
            target_steering = (
                steering_command.copy() if (estop_holding or obstacle_hold) else np.asarray(drive.steering_rad)
            )
            actual_steering = robot.get_joint_positions()[steers]
            # Creep while steering catches up; log the measured physical response.
            steering_error = float(np.max(np.abs(target_steering - actual_steering)))
            creeping = steering_error > 0.05
            wheel_speed = requested_speed * 0.25 if creeping else requested_speed
            if obstacle is not None and args.obstacle_act:
                # A step wheel target locked an odometry wheel (L3c v35/v36 seed 4:
                # the held pose lost 1.85 and 2.80 cm). The final wheel speed is
                # slewed -- braking at the stopping model's deceleration, pulling
                # away at COMMAND_ACCEL_MPS2 -- and the permission's own limit (a
                # sensor falling silent included) and the e-stop probe still cut
                # it as a step, the stop their distances were measured with
                # (Codex checkpoint 6 P1). The steering creep is slewed too: as a
                # step from 0.6 m/s it skidded a rear wheel and the held pose
                # fell 3 cm behind (L3c v43 seed 1, pocket_obstacle).
                previous = state.get("command_speed", 0.0)
                target_speed = wheel_speed
                if previous * target_speed < 0:
                    target_speed = 0.0
                rate = (obstacle["layer"].permission.config.stopping.decel_mps2
                        if abs(target_speed) < abs(previous) else COMMAND_ACCEL_MPS2) * dt
                wheel_speed = previous + float(np.clip(target_speed - previous, -rate, rate))
                cap = obstacle.get("permission_cap")
                if cap is not None and cap[0] == t and abs(wheel_speed) > cap[1]:
                    wheel_speed = math.copysign(cap[1], wheel_speed)
                if estop_holding:
                    wheel_speed = 0.0
                state["command_speed"] = wheel_speed
            drive = ackermann_command(wheel_speed, curvature, drive_geometry)
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
            pocket_frames = pocket["check"] is not None and phase in ("approach", "insert")
            if pocket_frames:
                pocket["history"].append((t, tuple(float(v) for v in rear)))
                del pocket["history"][:-120]
            stepper["fn"](pocket_frames and step % 12 == 0)
            if pocket_frames and step % 12 == 0:
                read_pocket_frame()
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
        destination = np.array([truth_destination.x_m, truth_destination.y_m])
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
                math.cos(truth_destination.yaw_rad),
                math.sin(truth_destination.yaw_rad),
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
        if state.get("obstacle_layer") is not None and obstacle is not None:
            scans_rec = obstacle["scans"]
            walls = [x["total_wall_s"] for x in scans_rec] or [0.0]
            state["obstacle_layer"].update(
                {
                    "scans": len(scans_rec),
                    "wall_s_per_scan": {"mean": float(np.mean(walls)), "max": float(np.max(walls))},
                    "control_ticks": obstacle["ticks"],
                    "slowed_ticks": obstacle["slowed"],
                    "slowed_reasons": obstacle["reasons"],
                    "min_allowed_mps": obstacle["min_allowed"],
                    "truth_events": obstacle["events"],
                    "unpermitted_entries": len(obstacle["unpermitted"]),
                    "unpermitted": obstacle["unpermitted"][:50],
                    "event_objects": len(obstacle.get("event_objects", {})),
                    "reprojections": obstacle.get("reprojections", 0),
                    "grid_planning": grid_planning,
                    "replans": obstacle.get("replans", []),
                    "grid_plans": obstacle.get("plans", []),
                    "live_plans": obstacle.get("live_plans", []),
                    "mission_replans": obstacle.get("mission_replans", []),
                    "silence_trace": obstacle.get("silence_trace"),
                    "backoff_records": obstacle.get("backoff_records", []),
                    "shadow_memory": None if obstacle["layer"].shadow is None else {
                        **obstacle["layer"].shadow.stats,
                        "band_m": obstacle["layer"].shadow.band_m,
                        "premise": "nothing inside the start-up band cells (outline grown by startup_reach_m)",
                    },
                }
            )
            (args.output / "obstacle_scans.json").write_text(record_json(scans_rec) + "\n")
            if overlay["plan_history"] or overlay["grids"]:
                (args.output / "plan_history.json").write_text(
                    record_json({"plans": overlay["plan_history"], "replans": obstacle.get("replans", []),
                                 "new_obstacles": new_obstacles["schedule"].log if new_obstacles is not None else []})
                    + "\n"
                )
                g_ = overlay["grids"]
                np.savez_compressed(
                    args.output / "obstacle_grid_frames.npz",
                    times=np.array([x[0] for x in g_]), origins=np.array([[x[1], x[2]] for x in g_]).reshape(-1, 2),
                    resolution=np.array([x[3] for x in g_]),
                    offsets=np.cumsum([0] + [len(x[4]) for x in g_]),
                    cells=np.concatenate([x[4] for x in g_]).reshape(-1, 2) if g_ else np.zeros((0, 2), np.int32),
                )
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
            write_slam_record(args, state, replace(scenario, destination=truth_destination), factory, slam_log, lidar_config)
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
        # The pose the controller drives on (plan v10 audit table): the SLAM
        # estimate when --slam-feedback is given, else the simulator's pose.
        # Ground truth is still used for the evaluator and safety stops only.
        "feedback": "slam_estimate" if args.slam_feedback is not None else "simulator_ground_truth",
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
