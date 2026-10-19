"""Physical Hybrid A* pallet transport using official warehouse props.

Run with Isaac Sim's Python after installing forklift-core in that environment.
All pose feedback is simulator ground truth; configuration is synthetic. Camera
output is 60fps by default, and the floor destination is marked by a green ring.
"""

import argparse
import hashlib
import importlib.util
import json
import collections
import math
import random
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
# Obstacle recovery v4 (docs/plans/2026-10-04-lidar-obstacle-map.md, "장애물 복구–허가 정합 설계 4판"):
# straight escapes per leg, a budget apart from the backoffs.
ESCAPE_PER_LEG = 2
# Plan D8b dwells: the wheel drives are velocity drives (stiffness 0, damping 20), which
# let a standing truck roll on (confirmation matrix 1968 seed 5: 0.1-0.15 mm/s through a
# 2 s stand); during a dwell they get this stiffness (N m/rad) around the angle they
# stopped at, under the same torque limit, and their own gains back on release.
DWELL_WHEEL_KP = 2000.0
# The second docking round stops this far before the goal (dock_at_delivery_straight).
ROUND2_KEEP_M = 0.6
# An observation waypoint missed by at most this much, standing, is arrival.
OBSERVE_ARRIVAL_M = 0.10
OBSERVE_ARRIVAL_YAW_RAD = 0.10
# EPAL 6 fork spacing plan, fix A: an observe cusp reached inside its window but missed in heading by
# 0.05-0.08 rad holds; once standing it is released when the next leg's judgement (cusp_stall.judge
# with these limits for that cusp only) passes, else it fails as before.
CUSP_HEADING_HOLD_RAD = 0.08
CUSP_HEADING_HOLD_POSITION_M = 0.03
# Fix A holds observe cusps; (A') adds the return (unloaded, no other cusp heading recovery).
CUSP_HEADING_HOLD_PHASES = ("observe", "return_home")
# EPAL 6 fork spacing plan, fix (1) after the S4 result: the observe replans (a leg judged out of heading
# at a cusp or its end, or stalled at a cusp) are budgeted per observe leg, not per run -- the far
# observation leg has this many, and each near-leg installation (the first near leg, the one after a
# retry's or a realignment's re-observation) opens its own. Both S4 failures of this kind were on a
# near leg whose run had spent the budget before it.
OBSERVE_REPLANS_PER_LEG = 2


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
pocket_recorder = load_perception_module(
    "run_transport_pocket_frame_recorder", Path(__file__).with_name("pocket_frame_recorder.py")
)
near_tracking = load_perception_module(
    "run_transport_near_tracking", Path(__file__).with_name("near_tracking.py")
)
CUSP_HOLD = load_perception_module(
    "run_transport_cusp_heading_hold", Path(__file__).with_name("cusp_heading_hold.py")
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


def load_mission_schedule(path: Path) -> tuple[list[tuple[float, str]], list[tuple[float, float, str]]]:
    """M-a3: the command schedule file -- (loop time, command) sorted by time, only pause, resume and cancel (the
    run submits its own mission); and the stop kinds by loop-time interval [from, to), controlled or preempt, as the
    arbiter will give them (plan E7)."""
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or set(data) - {"commands", "stop_kinds"}:
        raise ValueError("expected {\"commands\": [...], \"stop_kinds\": [...]}")
    kinds = []
    for row in data.get("stop_kinds", []):
        from_, to_, kind_ = row.get("from"), row.get("to"), row.get("kind")
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (from_, to_)) or not 0 <= from_ < to_:
            raise ValueError(f"a stop kind needs finite loop times 0 <= from < to: {row}")
        if kind_ not in ("controlled", "preempt"):
            raise ValueError(f"stop kind must be controlled or preempt: {row}")
        kinds.append((float(from_), float(to_), kind_))
    kinds.sort()
    for (_, end_a, _), (start_b, _, _) in zip(kinds, kinds[1:]):
        if start_b < end_a:
            raise ValueError("stop kind intervals must not overlap")
    out = []
    for row in data.get("commands", []):
        t_, command_ = row.get("t"), row.get("command")
        if not isinstance(t_, (int, float)) or not math.isfinite(t_) or t_ < 0:
            raise ValueError(f"command time must be a finite loop time >= 0: {row}")
        if command_ not in ("pause", "resume", "cancel"):
            raise ValueError(f"command must be pause, resume or cancel: {row}")
        out.append((float(t_), command_))
    return sorted(out, key=lambda r: r[0]), sorted(kinds)


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
        "--record-pocket-frames",
        action="store_true",
        help="Plan D8b: create the near-field depth camera (also without --pocket-check), "
        "read it on the D5 cadence after the near capture and keep every read (float32 "
        "depth, stamp, fabric frame, read time, label) with the truth of every physics "
        "tick, in <output>/pocket_frames. Needs --slam-feedback.",
    )
    parser.add_argument(
        "--record-dwell-gaps",
        type=pocket_recorder.parse_gaps,
        default=None,
        metavar="D1,D2,...",
        help="Plan D8b calibration dwells: stand 2 s at the start of the approach "
        "straight, at each of these control-estimated camera-face distances (m, "
        "decreasing) and after the approach arrives. Needs --record-pocket-frames.",
    )
    parser.add_argument(
        "--measure-safety-stops-m",
        type=pocket_recorder.parse_gaps,
        default=None,
        metavar="D1,D2,...",
        help="Plan D8 S4a-1 ⓪: scheduled safety stops on the approach straight -- command "
        "zero at once, steering held, no wheel lock, released 0.5 s after the stop detector "
        "first sees the truck stand, then a slewed restart -- at each of these "
        "control-estimated camera-face distances (m, decreasing). Needs "
        "--record-pocket-frames; not with --record-dwell-gaps.",
    )
    parser.add_argument(
        "--near-tracking",
        action="store_true",
        help="Plan D8 S4a: track the pockets from the near capture with the D8c 3판 tracker "
        "and stop on its events and the travel budget (stops before phase transitions, "
        "terminal stops end the run once standing). Needs --slam-feedback, the measured "
        "bounds' conditions (refused as near_field_contract otherwise) and "
        "--near-drive-terms. Off: the run is unchanged.",
    )
    parser.add_argument(
        "--near-tracking-stop-at",
        choices=("insert",),
        default=None,
        help="Plan D8 S4a-1: end the run where the approach arrives (before the insertion); "
        "without it the mission runs on with S4a-2's protection.",
    )
    parser.add_argument(
        "--near-inject",
        choices=("correction", "mismatch"),
        default=None,
        help="Plan D8 S4c-1: bias the far and near captures before the tracker is built -- "
        "correction: their planning targets only (the tracker starts from the raw "
        "observation); mismatch: the observations too. Needs --near-tracking and exactly one "
        "of --near-inject-lateral-m / --near-inject-yaw-rad.",
    )
    parser.add_argument(
        "--near-inject-lateral-m",
        type=float,
        default=0.0,
        help="Plan D8 S4c-1: the bias across the pallet's insertion axis (left +), m.",
    )
    parser.add_argument(
        "--near-inject-yaw-rad",
        type=float,
        default=0.0,
        help="Plan D8 S4c-1: the bias turning the pallet about its centre, rad.",
    )
    parser.add_argument(
        "--near-capture-redraw",
        choices=("on", "off"),
        default="on",
        help="Plan D8 S4c-3 (test only): off skips stage 1 so a refused near capture goes to "
        "the stage-2 realignment. Needs --near-tracking.",
    )
    parser.add_argument(
        "--near-capture-box-yaw-rad",
        type=float,
        default=0.05,
        help="Plan D8 S4c-3 (test only): the near capture's own heading box, narrower than the "
        "0.05 rad default to send near captures to the stage-1 re-draw in a smoke. (0, 0.05]. "
        "Needs --near-tracking.",
    )
    parser.add_argument(
        "--near-drop-frames-from-m",
        type=float,
        default=None,
        help="Plan D8 S4c-1: from the first tick the estimated fork tip to face is within this "
        "(m, -0.3 < D <= 0.5; negative = inside the pocket), no frame reaches the tracker "
        "(recorded as dropped). Needs --near-tracking.",
    )
    parser.add_argument(
        "--near-bounds",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "config/near_field_bounds_measured.json",
        help="Plan D8c 3판 measured bounds (tools/export_near_field_bounds.py).",
    )
    parser.add_argument(
        "--near-drive-terms",
        type=Path,
        default=None,
        help="Plan D8 S4a-1 ⓪: the confirmed drive terms (tools/s4_drive_terms.py).",
    )
    parser.add_argument(
        "--approach-straight-speed-mps",
        type=float,
        default=None,
        help="Plan D8b: cruise speed of the approach straight after the near capture; "
        "the near capture's acceptance dry run keeps the baseline cruise (that one at "
        "this speed is recorded only) and the insertion keeps insert_speed_mps.",
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
        "--recognition",
        choices=("off", "far"),
        default="off",
        help="far: the far observation's pallet comes from the new recognition (oracle instance masks from "
        "Isaac's semantic segmentation, instance fit, target selection and the far-range tracker fed every "
        "--recognition-frame-every physics steps while the observe leg drives); the detector still runs first "
        "and is kept as the reference. The near capture and the re-observation stands keep the detector "
        "(docs/plans/2026-10-18-isaac-recognition-far-capture.md).",
    )
    parser.add_argument("--recognition-frame-every", type=int, default=24,
                        help="Physics steps between driving frames for the far tracker (24 = 0.2 s).")
    parser.add_argument("--recognition-pixel-lag-steps", type=int, default=0,
                        help="Physics steps the driving frames' pixels trail their render metadata (measured "
                        "per render cadence; the smoke run records what fits it).")
    parser.add_argument("--recognition-no-drive-frames", action="store_true",
                        help="Far recognition from the stop frame only (no driving frames).")
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
        "--return-turnaround",
        action="store_true",
        help="Return-leg forward preference (docs/plans/2026-10-12-return-forward-turnaround.md): "
        "the first live return plan on the grid (after the SLAM release, which it waits for, and "
        "the grid's re-projection) also weighs full-lock reverse arcs, each followed by a search "
        "home, against the direct plan, reverse length weighted 3.0; each arc's start must be "
        "verified on the drive permission's snapshot and no known occupied cell may lie on it. "
        "Later return replans are unchanged.",
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
        "--encoder-model",
        choices=("reported", "team_spec_quantized", "team_spec_typical_inl"),
        default="reported",
        help="What the SLAM odometry reads as its encoders: the engine-reported joint velocities "
        "(reported, the runner's behaviour before 2026-10-15) or the team's selected parts -- MT6701 x4 "
        "(4,096 counts/rev, ABZ hysteresis 0.088 deg) on the rear wheels and a KTH7801-C 12-bit steering "
        "angle, counted from the joint angles each physics tick and integrated as increments; "
        "team_spec_typical_inl adds a first-harmonic INL at the datasheet typical amplitude (MT6701 1.0 deg, "
        "KTH7801 0.35 deg; shape assumed, phase per --slam-noise-seed). With a team-spec model the "
        "Gaussian wheel and steering noise is not added (the LiDAR range noise is).",
    )
    parser.add_argument(
        "--odometry-wheel-radius-m",
        type=float,
        default=None,
        help="The rear wheel radius the SLAM odometry integrates with -- an effective rolling radius from a "
        "calibration (fork spacing plan V3: r_cal = truth travel / wheel angle advance on a straight); unset = "
        "the chassis model's geometric radius. Only the odometry reads it: drive commands, speed limits and "
        "the plant keep the geometric radius. Recorded in meta.json odometry_geometry, so every replay "
        "integrates with the same value.",
    )
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
        "--steering-probe-set",
        choices=("standard", "long", "long_reverse_first"),
        default="standard",
        help="The --steering-probe segment set: standard, or long steady-state runs (new plant diagnostic D-a).",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="M-c: run live -- paced to wall time, a console/1 server inside the run (docs/plans/"
        "2026-10-17-m-c-isaac-live-runner.md), the arbiter deciding the drive; the mission is submitted by the operator. "
        "Not with --mission-commands or a boundary record.",
    )
    parser.add_argument("--live-port-file", type=Path, default=None, help="M-c: where to write host, port and epoch")
    parser.add_argument("--live-host", default="127.0.0.1", help="M-c: the console server's address")
    parser.add_argument("--live-factor", type=float, default=1.0, help="M-c: the declared real-time factor")
    parser.add_argument("--live-lease-timeout-s", type=float, default=5.0,
                        help="M-c: the control lease's timeout -- a tick cannot go out while a synchronous call holds "
                        "the control loop (the first live run lost a 1 s lease to a 1.2 s start-up tick)")
    parser.add_argument("--live-camera-latency-bound-s", type=float, default=None,
                        help="M-c: camera_latency_bound_s, wall seconds (default 0.21/f: the assumed pixel lag below "
                        "plus a physics step, at the declared factor)")
    parser.add_argument("--live-camera-pixel-lag-s", type=float, default=0.2,
                        help="M-c: sim seconds a frame's pixels are assumed to trail its render metadata time; the "
                        "arbiter also ages the frame from that step's start (2026-10-17 marker runs: 11 physics "
                        "steps, about one render of the 10 Hz camera; the default adds a render of margin)")
    parser.add_argument("--live-web-root", type=Path, default=None, help="M-c: console/web to serve")
    parser.add_argument("--live-test-step-delay-ms", type=float, default=0.0,
                        help="M-c tests only: extra wall time per physics step (the slow-pacing test)")
    parser.add_argument("--live-test-stop-in-capture", action="store_true",
                        help="M-c tests only: a STOP arrives at a physics step inside the first capture (the path a "
                        "console STOP takes, at a moment a client cannot aim for)")
    parser.add_argument("--live-test-marker", action="store_true",
                        help="M-c tests only (plan C5): the lift follows a non-periodic wave so the carriage camera's "
                        "horizon tells the sim time a frame's pixels were drawn at; the lift height is logged every "
                        "physics step -- for a run without a mission")
    parser.add_argument("--live-test-permit-compare", action="store_true",
                        help="M-c tests only (completion 5): every 12th teleoperation permission is also asked of a "
                        "fresh full check on the same grid, pose and arcs, and both are logged with the true rear-axle "
                        "pose; the truth check's hall bounds (0.76 m inside the wall) are widened so the approach "
                        "can reach the wall")
    parser.add_argument("--live-test-scan-silence", default=None,
                        help="M-c tests only (completion 5): FROM:TO sim seconds in which the obstacle layer gets no "
                        "scans (the teleoperation permission must stop the truck on the silence and the aging evidence)")
    parser.add_argument("--live-test-camera-freeze", default=None,
                        help="M-c tests only: FROM:TO sim seconds with no new camera frames (the video-stall test)")
    parser.add_argument(
        "--mission-commands",
        type=Path,
        default=None,
        help="M-a3: a JSON schedule of mission commands by loop time -- {\"commands\": [{\"t\": 30.0, \"command\": "
        "\"pause\"}, ...]}, commands pause, resume and cancel -- applied at the first control tick at or after their "
        "time (docs/plans/2026-10-16-m-a3-mission-commands.md). Needs --obstacle-act (a command stop decelerates on "
        "the stopping model); not with the near-field tracker, the measurement runs' stops and dwells, the e-stop "
        "probe or a boundary record. Off by default: the run submits its scenario's mission and nothing else.",
    )
    parser.add_argument(
        "--record-boundary",
        action="store_true",
        help="M-a0: record every control tick's boundary -- the calls the mission logic makes to the vehicle, "
        "clock, SLAM and obstacle services, capture and planners, with their requests and responses, and the "
        "tick's commands -- under OUTPUT/boundary for replaying a moved mission core "
        "(docs/plans/2026-10-15-m-a05-boundary.md section 5). Recording only; off by default.",
    )
    parser.add_argument(
        "--record-boundary-format",
        type=int,
        choices=(1, 2),
        default=1,
        help="M-a1 replay plan R3: 1 keeps the M-a0 record (comparison form, the frozen records' call sequence); "
        "2 records values a replay can hand back, the setup snapshot and the files the setup read "
        "(docs/plans/2026-10-16-m-a1-replay.md). Recording only.",
    )
    parser.add_argument(
        "--tyre-contact-log",
        action="store_true",
        help="Diagnostic: log each tyre's floor contact (PhysX contact report) every 0.1 s of the mission to "
        "tyre_contacts.npy (fork spacing plan, new plant candidate V1/V5).",
    )
    parser.add_argument(
        "--steering-probe",
        action="store_true",
        help="Diagnostic: after the spawn checks, drive scripted steering segments (standstill, "
        "forward, reverse at +-0.29 / +-0.36 1/m) and stop, recording the joints, efforts and truth "
        "pose (fork spacing plan, cause survey after the S4 rerun).",
    )
    parser.add_argument(
        "--repeat-at-near-capture",
        action="store_true",
        help="Diagnostic: take the --repeat-captures batch at the first near capture "
        "instead of at an observation attempt number (fork spacing plan, (B)).",
    )
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
        if not args.use_perception or (
            (args.repeat_at_attempt is None) == (not args.repeat_at_near_capture)
        ):
            parser.error(
                "--repeat-captures requires --use-perception and one of "
                "--repeat-at-attempt or --repeat-at-near-capture"
            )
        if args.repeat_at_attempt is not None and args.repeat_at_attempt < 1:
            parser.error("--repeat-at-attempt counts from 1")
        if args.planning_target != "perception":
            parser.error("a repeat-capture run stops before planning")
    elif args.repeat_at_attempt is not None or args.repeat_at_near_capture:
        parser.error("--repeat-at-attempt or --repeat-at-near-capture requires --repeat-captures")
    if args.obstacles < 1 or args.max_sim_seconds <= 0:
        parser.error("obstacles and max-sim-seconds must be positive")
    if args.encoder_model != "reported" and args.pocket_check:
        # Codex (encoder implementation round 2): the pocket check takes a one-tick odometry speed as
        # its still-frame test and its frame-lag bound and records it for d5_replay; with quantised
        # counts (and the MT6701 hysteresis) neither holds, and no bound is defined yet.
        parser.error("--pocket-check is not defined for a team-spec --encoder-model yet (no speed bound / replay rule)")
    if args.encoder_model != "reported" and not args.record_slam:
        parser.error("--encoder-model reads the encoders into the SLAM record and odometry; it needs --record-slam")
    if args.odometry_wheel_radius_m is not None:
        if not args.record_slam:
            parser.error("--odometry-wheel-radius-m is the SLAM odometry's radius; it needs --record-slam")
        if args.pocket_check:
            # The pocket check records a one-tick odometry speed for d5_replay, whose rule assumes the
            # geometric radius; not defined for a calibrated one yet.
            parser.error("--pocket-check is not defined for --odometry-wheel-radius-m yet")
        if not args.odometry_wheel_radius_m > 0:
            parser.error("--odometry-wheel-radius-m must be positive")
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
    if args.recognition == "far":
        # The near-field bounds were measured with the detector's far estimate: a run that changes it is
        # outside that contract, so near tracking and the pocket check are refused with it (plan v2, Codex P1).
        if not args.use_perception or args.slam_feedback is None:
            parser.error("--recognition far needs --use-perception and --slam-feedback")
        if args.near_tracking or args.pocket_check:
            parser.error("--recognition far is not defined with --near-tracking or --pocket-check")
        if getattr(args, "live", False):
            parser.error("--recognition far is a batch run (the live capture path is out of its scope)")
        if args.recognition_frame_every <= 0 or args.recognition_pixel_lag_steps < 0:
            parser.error("--recognition-frame-every must be positive, --recognition-pixel-lag-steps nonnegative")
    if args.record_pocket_frames and args.slam_feedback is None:
        # The frames start after the near capture, which only SLAM runs make.
        parser.error("--record-pocket-frames needs --slam-feedback")
    if args.record_dwell_gaps is not None and not args.record_pocket_frames:
        parser.error("--record-dwell-gaps needs --record-pocket-frames")
    if args.measure_safety_stops_m is not None and (not args.record_pocket_frames or args.record_dwell_gaps is not None):
        parser.error("--measure-safety-stops-m needs --record-pocket-frames and excludes --record-dwell-gaps")
    if args.approach_straight_speed_mps is not None and not (
        math.isfinite(args.approach_straight_speed_mps) and 0.0 < args.approach_straight_speed_mps <= 1.0
    ):
        parser.error("--approach-straight-speed-mps must be in (0, 1] m/s")
    if args.near_tracking and (args.slam_feedback is None or not args.use_perception
                               or args.near_drive_terms is None or args.approach_straight_speed_mps is None):
        parser.error("--near-tracking needs --slam-feedback, --use-perception, --near-drive-terms and "
                     "--approach-straight-speed-mps (the near-field cruise)")
    if args.near_tracking and (args.measure_safety_stops_m is not None or args.record_dwell_gaps is not None):
        parser.error("--near-tracking excludes the scheduled stops and dwells of the measurement runs")
    if args.near_tracking_stop_at is not None and not args.near_tracking:
        parser.error("--near-tracking-stop-at needs --near-tracking")
    if args.live and args.mission_commands is not None:
        parser.error("--live takes the operator's commands; not with --mission-commands")
    if args.live and args.repeat_captures:
        parser.error("--live does not take --repeat-captures (its captures bypass the stop boundary, plan C2)")
    if args.live and args.record_boundary:
        parser.error("--live is not replayed: not with --record-boundary (plan C4)")
    if not args.live and (args.live_test_permit_compare or args.live_test_scan_silence):
        parser.error("--live-test-permit-compare and --live-test-scan-silence are for --live runs only")
    if args.live and not 0 < args.live_factor <= 1:
        # The limits are divided by it (plan C4a); above 1 the 0.05 s tick would exceed a quarter of the command age.
        parser.error("--live-factor must be in (0, 1]")
    if args.live_camera_latency_bound_s is not None and not (math.isfinite(args.live_camera_latency_bound_s)
                                                           and args.live_camera_latency_bound_s > 0):
        parser.error("--live-camera-latency-bound-s must be a positive finite number")
    if not (math.isfinite(args.live_camera_pixel_lag_s) and args.live_camera_pixel_lag_s >= 0):
        parser.error("--live-camera-pixel-lag-s must be a non-negative finite number")
    if args.mission_commands is not None:
        # M-a3 (plan E4, E7, E9): command stops decelerate on the obstacle layer's stopping model; the near-field
        # bounds were measured without pauses; scheduled stops, dwells and the e-stop probe keep their own clocks;
        # a boundary record of a command run lands with the stop kinds (M-a3d).
        if not args.obstacle_act:
            parser.error("--mission-commands needs --obstacle-act")
        if args.near_tracking or args.record_pocket_frames or args.measure_safety_stops_m is not None \
                or args.estop_probe is not None or (args.record_boundary and args.record_boundary_format != 2):
            parser.error("--mission-commands excludes --near-tracking, --record-pocket-frames, "
                         "--measure-safety-stops-m, --estop-probe and a format-1 --record-boundary")
        try:
            args.mission_schedule, args.mission_stop_kinds = load_mission_schedule(args.mission_commands)
        except (OSError, ValueError) as exc:
            parser.error(f"--mission-commands: {exc}")
    inject_sizes = (args.near_inject_lateral_m, args.near_inject_yaw_rad)
    if not all(math.isfinite(v) for v in inject_sizes):
        parser.error("--near-inject-lateral-m and --near-inject-yaw-rad must be finite")
    if args.near_inject is not None:
        if not args.near_tracking:
            parser.error("--near-inject needs --near-tracking")
        if sum(1 for v in inject_sizes if v != 0.0) != 1:
            # Plan D8 S4c-1: a lateral and a yaw bias together leave the 1.0 m correction box.
            parser.error("--near-inject needs exactly one of --near-inject-lateral-m and --near-inject-yaw-rad")
    elif any(v != 0.0 for v in inject_sizes):
        parser.error("--near-inject-lateral-m / --near-inject-yaw-rad need --near-inject")
    if args.near_capture_redraw != "on" and not args.near_tracking:
        parser.error("--near-capture-redraw needs --near-tracking")
    if args.near_capture_box_yaw_rad != 0.05:
        if not args.near_tracking:
            parser.error("--near-capture-box-yaw-rad needs --near-tracking")
        if not (math.isfinite(args.near_capture_box_yaw_rad) and 0.0 < args.near_capture_box_yaw_rad <= 0.05):
            # Narrower only: the default box is the one the v3.7 collision finding fixed.
            parser.error("--near-capture-box-yaw-rad must be in (0, 0.05] rad")
    if args.near_drop_frames_from_m is not None:
        if not args.near_tracking:
            parser.error("--near-drop-frames-from-m needs --near-tracking")
        if not (math.isfinite(args.near_drop_frames_from_m) and -0.3 < args.near_drop_frames_from_m <= 0.5):
            # At most 0.5 m: the 0.6 m correction point has closed, so no correction is open.
            parser.error("--near-drop-frames-from-m must be in (-0.3, 0.5] m")
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


class SessionFailure(RuntimeError):
    """A real-time run's failure that ends the session, not only its mission: the SLAM link (a missed reply, a
    queue that does not drain, a failed or closed bridge) or its warm-up (real-time plan D9a). The reason reads as
    a ``require`` failure would; a live console that keeps the runner up across missions tells it apart by type."""


# EPAL 6 fork spacing plan, cause survey after the S4 rerun: the steering response probe. The recorded
# runs show the front-left wheel short of its command while the body follows that wheel's curvature;
# this drives scripted segments (standstill, forward, reverse at each curvature) through the runner's
# own command path (Ackermann targets, the steering rate limit, the wheel-rate slew) and records the
# joints, their efforts and the truth pose every physics tick.
# Fork spacing plan, cause separation E: scene and drive settings a settings file may set for a
# --steering-probe only (numbers, as every settings value must be: 1 switches a flag on).
DIAGNOSTIC_PROBE_SETTINGS = ("physx_friction_correlation_distance_m", "physx_solver_pgs", "physx_enable_stabilization",
                             "tyre_damping_combine_max", "wheel_drive_damping", "physx_external_forces_every_iteration")
TYRE_LINKS = tuple(f"/World/Forklift/{n}" for n in ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"))
PROBE_CURVATURES = (0.29, -0.29, 0.36, -0.36)
PROBE_SPEED_MPS = 0.25
PROBE_FAST_MPS = 0.6  # shorter arcs: the run starts facing the pallet about 5 m ahead


def odometry_wheel_radius(args) -> float:
    """The rear wheel radius the SLAM odometry integrates with: --odometry-wheel-radius-m when given,
    else the chassis model's geometric radius (fork spacing plan, 16th draft: odometry-only radius)."""
    if args.odometry_wheel_radius_m is not None:
        return float(args.odometry_wheel_radius_m)
    return float(args.drive_geometry.wheel_radius_m)


def age_table_problem(table: dict, args) -> str | None:
    """Why an odometry age-error table does not cover this run's odometry (same encoder model, same
    odometry wheel radius; forklift_core.localization.encoders.odometry_table_problem), or None."""
    from forklift_core.localization.encoders import odometry_table_problem

    return odometry_table_problem(table, encoder_model=args.encoder_model, wheel_radius_m=odometry_wheel_radius(args),
                                  geometric_radius=args.odometry_wheel_radius_m is None)


def probe_segments() -> list[tuple[str, float, float, float]]:
    """(label, speed m/s, curvature 1/m, seconds): standstill steering steps, then the same
    curvatures rolling forward and in reverse at 0.25 m/s, then at 0.6 m/s, each between straight
    stretches (a forward run is followed by its reverse, so the truck comes back)."""
    out = [("settle", 0.0, 0.0, 1.0)]
    out += [("still", 0.0, k, 2.5) for k in PROBE_CURVATURES]
    out.append(("still", 0.0, 0.0, 1.5))
    for label, speed, arc in (("fwd", PROBE_SPEED_MPS, 2.5), ("rev", -PROBE_SPEED_MPS, 2.5),
                              ("fwd_fast", PROBE_FAST_MPS, 1.0), ("rev_fast", -PROBE_FAST_MPS, 1.0)):
        out.append((label, speed, 0.0, 1.5 if arc > 2 else 1.0))
        out += [(label, speed, k, arc) for k in PROBE_CURVATURES]
        out += [(label, speed, 0.0, 1.0 if arc > 2 else 0.5), ("stop", 0.0, 0.0, 1.5)]
    return out


TYRE_FIELDS = ("Jx", "Jy", "Jz", "Mx", "My", "Mz", "floor_contacts", "min_separation_m", "other_contacts", "other_impulse")


def probe_window_metrics(rows, c: dict, dt: float, wheel_kappa, target_speed: float, radius: float) -> dict:
    """Curvature from both front wheels' actual angles and from the truth rear pose, their residual,
    and the rolling ratio (rear-axle longitudinal speed over the two rear wheels' actual rate x radius)
    over a steady window."""
    q_fl, q_fr = float(np.mean(rows[:, c["q_FL"]])), float(np.mean(rows[:, c["q_FR"]]))
    k_fl, k_fr = wheel_kappa(q_fl, q_fr)
    dx, dy = np.diff(rows[:, c["rear_x"]]), np.diff(rows[:, c["rear_y"]])
    yaw = np.unwrap(rows[:, c["rear_yaw"]])
    ds = float(np.sum(np.hypot(dx, dy)))
    sign = 1.0 if target_speed >= 0 else -1.0
    k_true = sign * float(yaw[-1] - yaw[0]) / ds if ds > 0.02 else None
    v_long = (dx * np.cos(yaw[1:]) + dy * np.sin(yaw[1:])) / dt
    w = (rows[1:, c["w_RL"]] + rows[1:, c["w_RR"]]) / 2
    ratio = float(np.mean(v_long) / np.mean(w * radius)) if abs(float(np.mean(w))) > 1e-3 else None
    return {"kappa_FL": k_fl, "kappa_FR": k_fr, "kappa_avg": (k_fl + k_fr) / 2, "kappa_true": k_true,
            "residual": None if k_true is None else k_true - (k_fl + k_fr) / 2, "rolling_ratio": ratio,
            "err_FL_mrad": float(np.mean(rows[:, c["q_FL"]] - rows[:, c["target_FL"]])) * 1e3,
            "err_FR_mrad": float(np.mean(rows[:, c["q_FR"]] - rows[:, c["target_FR"]])) * 1e3}


def probe_segments_long_reverse_first() -> list[tuple[str, float, float, float]]:
    """probe_segments_long() with each forward/reverse pair swapped (cause separation E: order and
    contact history apart from the direction)."""
    out, segs, i = [], probe_segments_long(), 0
    while i < len(segs):
        if segs[i][0].startswith("fwd") and i + 2 < len(segs) and segs[i + 2][0].startswith("rev"):
            out += [segs[i + 2], segs[i + 1], segs[i], segs[i + 3]]
            i += 4
        else:
            out.append(segs[i])
            i += 1
    return out


def probe_segments_long() -> list[tuple[str, float, float, float]]:
    """The long steady-state set (new plant candidate, diagnostic D-a): straight runs long enough to
    settle after the 0.8 m/s^2 slew, and the 0.6 m/s full-command arcs held longer; each forward run
    is followed by the same reverse run so the truck comes back."""
    out = [("settle", 0.0, 0.0, 1.0)]
    for label, speed, seconds in (("fwd", PROBE_SPEED_MPS, 4.0), ("rev", -PROBE_SPEED_MPS, 4.0),
                                  ("fwd_fast", PROBE_FAST_MPS, 3.5), ("rev_fast", -PROBE_FAST_MPS, 3.5)):
        out += [(label, speed, 0.0, seconds), ("stop", 0.0, 0.0, 1.5)]
    for kappa in (0.36, -0.36):
        out += [("fwd_fast", PROBE_FAST_MPS, kappa, 2.5), ("stop", 0.0, 0.0, 1.5),
                ("rev_fast", -PROBE_FAST_MPS, kappa, 2.5), ("stop", 0.0, 0.0, 1.5)]
    return out


def tyre_floor_forces(stage_wheels: tuple[str, ...]) -> list[list[float]]:
    """Per tyre (in stage_wheels order), from this step's PhysX contact report against everything
    outside the robot (TYRE_FIELDS): the floor contacts' (normal within 0.9 of vertical -- classified by
    the normal, not by the other actor) summed impulse (x, y, z) as the report gives it and its moment
    about the world origin summed per contact point (p x J -- not the mean point times the sum), their
    count and smallest separation; the other contacts' count and summed impulse magnitude. NaN when the
    report is unavailable. Divide impulses by the step for force. Sign: standing, the floor impulses sum
    to about -weight x dt, so the floor's push on the truck is the negative of J and of p x J (fork
    spacing plan, D-c). Friction anchor impulses, which PhysX reports apart from contact points, are
    not summed here."""
    out = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.inf, 0.0, 0.0] for _ in stage_wheels]
    try:
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        headers, data = get_physx_simulation_interface().get_contact_report()
    except Exception:  # noqa: BLE001 -- recorded as missing
        return [[math.nan] * len(TYRE_FIELDS) for _ in stage_wheels]
    for header in headers:
        paths = (str(PhysicsSchemaTools.intToSdfPath(header.actor0)), str(PhysicsSchemaTools.intToSdfPath(header.actor1)))
        for w, wheel in enumerate(stage_wheels):
            if wheel not in paths:
                continue
            other = paths[1] if paths[0] == wheel else paths[0]
            if other.startswith("/World/Forklift"):
                continue
            sign = 1.0 if paths[1] == wheel else -1.0  # the impulse acts on actor1 (+) or actor0 (-)
            row = out[w]
            for k in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                c = data[k]
                j = np.array([float(c.impulse[0]), float(c.impulse[1]), float(c.impulse[2])]) * sign
                if abs(float(c.normal[2])) >= 0.9:
                    p_ = np.array([float(c.position[0]), float(c.position[1]), float(c.position[2])])
                    m = np.cross(p_, j)
                    for i in range(3):
                        row[i] += float(j[i])
                        row[3 + i] += float(m[i])
                    row[6] += 1.0
                    row[7] = min(row[7], float(c.separation))
                else:
                    row[8] += 1.0
                    row[9] += float(np.linalg.norm(j))
    for row in out:
        if row[7] == math.inf:
            row[7] = math.nan
    return out


def run_steering_probe(world, robot, wheels, steers, settings, args, action_type, ackermann, out_dir) -> dict:
    """Drive probe_segments() (or probe_segments_long()) and return a per-segment summary; the per-tick
    record goes to steering_probe.npz. Segment summaries use the last second of each segment, plus the
    steady window when one exists."""
    segments = {"long": probe_segments_long, "long_reverse_first": probe_segments_long_reverse_first,
                "standard": probe_segments}[args.steering_probe_set]()
    dt = 1.0 / float(settings["physics_hz"])
    geometry = args.drive_geometry
    offset = abs(float(args.rear_axle_offset_m))
    steering = np.asarray(robot.get_joint_positions()[steers], dtype=float).copy()
    speed = 0.0
    rows, efforts_applied, efforts_measured, joint_forces, tyre_contacts = [], [], [], [], []
    stage_wheels = TYRE_LINKS
    names = list(robot.dof_names)
    # The 6-D incoming joint force of each wheel's support joint (front: the steering joint, rear: the
    # spin joint) -- its vertical part is the wheel's share of the truck's weight.
    try:
        body_names = list(robot._articulation_view.body_names)
    except Exception:  # noqa: BLE001 -- recorded for the analysis only
        body_names = []
    t0 = float(world.current_time)
    for index, (label, target_speed, kappa, seconds) in enumerate(segments):
        for _ in range(int(round(seconds / dt))):
            if speed * target_speed < 0:
                target = 0.0
            else:
                target = target_speed
            speed += float(np.clip(target - speed, -COMMAND_ACCEL_MPS2 * dt, COMMAND_ACCEL_MPS2 * dt))
            drive = ackermann(speed, kappa, geometry)
            rate = settings["steering_command_rate_rad_s"] * dt
            steering += np.clip(np.asarray(drive.steering_rad) - steering, -rate, rate)
            rates = np.asarray(drive.wheel_rates_rad_s, dtype=float)
            robot.apply_action(action_type(joint_velocities=rates, joint_indices=wheels))
            robot.apply_action(action_type(joint_positions=steering.copy(), joint_indices=steers))
            world.step(render=False)
            q = np.asarray(robot.get_joint_positions(), dtype=float)
            qd = np.asarray(robot.get_joint_velocities(), dtype=float)
            position, orientation = robot.get_world_pose()
            quat = np.asarray(orientation, dtype=float)
            yaw, _ = yaw_and_tilt(quat)
            w_, x_, y_, z_ = quat
            roll = math.atan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (w_ * y_ - z_ * x_))))
            rear = (float(position[0]) - offset * math.cos(yaw), float(position[1]) - offset * math.sin(yaw), yaw)
            rows.append([float(world.current_time) - t0, index, speed, kappa, *drive.steering_rad, *steering,
                         *q[steers], *qd[steers], *drive.wheel_rates_rad_s, *qd[wheels], *rear,
                         float(position[2]), roll, pitch, *q[wheels]])
            tyre_contacts.append(tyre_floor_forces(stage_wheels))
            try:
                joint_forces.append(np.asarray(robot.get_measured_joint_forces(), dtype=float).reshape(-1, 6))
            except Exception:  # noqa: BLE001
                joint_forces.append(np.full((1, 6), math.nan))
            for store, name in ((efforts_applied, "get_applied_joint_efforts"), (efforts_measured, "get_measured_joint_efforts")):
                try:
                    e = np.asarray(getattr(robot, name)(), dtype=float)
                    store.append([*e[steers], *e[wheels[:2]]])
                except Exception:  # noqa: BLE001 -- the API differs between Isaac releases; record what exists
                    store.append([math.nan] * 4)
    x = np.asarray(rows, dtype=float)
    columns = ("t", "segment", "speed_cmd", "kappa_cmd", "target_FL", "target_FR", "cmd_FL", "cmd_FR",
               "q_FL", "q_FR", "qd_FL", "qd_FR", "w_cmd_FL", "w_cmd_FR", "w_cmd_RL", "w_cmd_RR",
               "w_FL", "w_FR", "w_RL", "w_RR", "rear_x", "rear_y", "rear_yaw", "base_z", "roll", "pitch",
               # Wheel joint angles (cause separation E): a creep in the rates is a real rotation only if
               # these advance by the integrated rate.
               "p_FL", "p_FR", "p_RL", "p_RR")
    np.savez(out_dir / "steering_probe.npz", record=x, columns=np.array(columns),
             efforts_applied=np.asarray(efforts_applied), efforts_measured=np.asarray(efforts_measured),
             effort_columns=np.array(("steer_FL", "steer_FR", "spin_FL", "spin_FR")),
             joint_forces=np.asarray(joint_forces, dtype=float), body_names=np.array(body_names),
             tyre_contacts=np.asarray(tyre_contacts, dtype=float), tyre_names=np.array(stage_wheels),
             tyre_fields=np.array(TYRE_FIELDS),
             dof_names=np.array(names))
    half, wheelbase = geometry.track_m / 2, geometry.wheelbase_m

    def wheel_kappa(left, right):
        tl, tr = math.tan(left), math.tan(right)
        return tl / (wheelbase + tl * half), tr / (wheelbase - tr * half)

    summary = []
    for index, (label, target_speed, kappa, seconds) in enumerate(segments):
        tick = int(round(min(1.0, seconds / 2) / dt))  # the segment's last half, at most 1 s
        seg = x[x[:, 1] == index][-tick:]
        whole = x[x[:, 1] == index]
        c = {k: i for i, k in enumerate(columns)}
        # The 13th draft's steady window: both wheels |qdot| < 0.02 rad/s and |d error/dt| < 5 mrad/s,
        # the longest such run in the segment's second half, used when >= 0.3 s.
        late = whole[len(whole) // 2:]  # not `half`: wheel_kappa reads the half track by that name
        steady_mask = np.zeros(len(late), dtype=bool)
        if len(late) > 2:
            err_l = late[:, c["q_FL"]] - late[:, c["target_FL"]]
            err_r = late[:, c["q_FR"]] - late[:, c["target_FR"]]
            slope = np.maximum(np.abs(np.gradient(err_l, dt)), np.abs(np.gradient(err_r, dt)))
            steady_mask = (np.abs(late[:, c["qd_FL"]]) < 0.02) & (np.abs(late[:, c["qd_FR"]]) < 0.02) & (slope < 0.005)
        best, run_start, best_span = 0, None, None
        for i_, ok_ in enumerate(np.r_[steady_mask, False]):
            if ok_ and run_start is None:
                run_start = i_
            elif not ok_ and run_start is not None:
                if i_ - run_start > best:
                    best, best_span = i_ - run_start, (run_start, i_)
                run_start = None
        steady = late[best_span[0]:best_span[1]] if best_span and best * dt >= 0.3 else None
        ea = np.asarray(efforts_applied)[x[:, 1] == index][-tick:]
        em = np.asarray(efforts_measured)[x[:, 1] == index][-tick:]
        tc = np.asarray(tyre_contacts)[x[:, 1] == index][-tick:]
        c = {k: i for i, k in enumerate(columns)}
        q_fl, q_fr = float(np.mean(seg[:, c["q_FL"]])), float(np.mean(seg[:, c["q_FR"]]))
        k_fl, k_fr = wheel_kappa(q_fl, q_fr)
        ds = float(np.sum(np.hypot(np.diff(seg[:, c["rear_x"]]), np.diff(seg[:, c["rear_y"]]))))
        dyaw = float(np.unwrap(seg[:, c["rear_yaw"]])[-1] - np.unwrap(seg[:, c["rear_yaw"]])[0])
        sign = 1.0 if target_speed >= 0 else -1.0
        summary.append({
            "segment": index, "label": label, "speed_mps": target_speed, "kappa_cmd": kappa,
            "target_FL": float(np.mean(seg[:, c["target_FL"]])), "target_FR": float(np.mean(seg[:, c["target_FR"]])),
            "q_FL": q_fl, "q_FR": q_fr,
            "err_FL_mrad": (q_fl - float(np.mean(seg[:, c["target_FL"]]))) * 1e3,
            "err_FR_mrad": (q_fr - float(np.mean(seg[:, c["target_FR"]]))) * 1e3,
            "kappa_FL": k_fl, "kappa_FR": k_fr, "kappa_avg": (k_fl + k_fr) / 2,
            "kappa_true": sign * dyaw / ds if ds > 0.02 else None, "distance_m": ds,
            "joint_force_mean": np.nanmean(np.asarray(joint_forces)[x[:, 1] == index][-tick:], axis=0).tolist(),
            # Tyre-floor force (N): the mean impulse per step / dt, per tyre FL FR RL RR, and the contact count.
            "tyre_force_n": (np.nanmean(tc[:, :, :3], axis=0) / dt).tolist() if len(tc) else None,
            "tyre_moment_nm": (np.nanmean(tc[:, :, 3:6], axis=0) / dt).tolist() if len(tc) else None,
            "tyre_floor_contacts_mean": np.nanmean(tc[:, :, 6], axis=0).tolist() if len(tc) else None,
            "tyre_min_separation_m": np.nanmin(tc[:, :, 7], axis=0).tolist() if len(tc) else None,
            "tyre_other_contacts_max": np.nanmax(tc[:, :, 8], axis=0).tolist() if len(tc) else None,
            "q_rate_FL_mean": float(np.mean(seg[:, c["qd_FL"]])), "q_rate_FR_mean": float(np.mean(seg[:, c["qd_FR"]])),
            "steady_s": None if steady is None else len(steady) * dt,
            "steady": None if steady is None else probe_window_metrics(steady, c, dt, wheel_kappa, target_speed,
                                                                    geometry.wheel_radius_m),
            "effort_applied": np.nanmean(ea, axis=0).tolist() if len(ea) else None,
            "effort_measured": np.nanmean(em, axis=0).tolist() if len(em) else None,
            "wheel_rate_FL": [float(np.mean(seg[:, c["w_cmd_FL"]])), float(np.mean(seg[:, c["w_FL"]]))],
            "wheel_rate_FR": [float(np.mean(seg[:, c["w_cmd_FR"]])), float(np.mean(seg[:, c["w_FR"]]))],
        })
    return {"segments": summary, "dt_s": dt, "speeds_mps": [PROBE_SPEED_MPS, PROBE_FAST_MPS],
            "curvatures": list(PROBE_CURVATURES)}


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
        "encoder_model": state.get("encoder_model") or {"name": "reported"},
        # M-a0: a boundary-recorded run's slam log is never measurement evidence (P0a reads this file).
        "record_boundary": bool(args.record_boundary),
        "odometry_geometry": {
            "wheelbase_m": args.drive_geometry.wheelbase_m,
            "track_m": args.drive_geometry.track_m,
            # What the odometry integrated with (replays read this); the geometric one beside it.
            "wheel_radius_m": odometry_wheel_radius(args),
            "geometric_wheel_radius_m": args.drive_geometry.wheel_radius_m,
            "wheel_radius_source": "geometric" if args.odometry_wheel_radius_m is None else "--odometry-wheel-radius-m",
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
        "turnaround": getattr(path, "turnaround", None),
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


class TrackerBook(dict):
    """The runner's path trackers by owner phase (fork spacing plan, V5 cusp arrivals; Codex leg-switch
    review). Installing a tracker stamps it with a serial (its path version), its owner phase, the time
    and its cusp endpoints; its leg switches go into the record each tick (collect), when it is replaced
    and at the end (close), so a switch is not lost to a replacement or an exception."""

    def __init__(self, record: list, versions: list, clock: dict, initial=()) -> None:
        super().__init__()
        self._record, self._versions, self._clock, self._serial = record, versions, clock, 0
        for phase, tracker in dict(initial).items():
            self[phase] = tracker

    def __setitem__(self, phase, tracker) -> None:
        old = self.get(phase)
        if old is not None and old is not tracker:
            self._drain(old, "replaced")
            if getattr(old, "_book", None) is not None:
                old._book["replaced_s"] = self._clock.get("t")
        if getattr(tracker, "_book", None) is None:
            self._serial += 1
            ends = list(getattr(tracker, "_leg_ends", []))[:-1]
            poses = getattr(tracker, "_poses", None)
            tracker._book = {
                "serial": self._serial, "phase": phase, "installed_s": self._clock.get("t"), "replaced_s": None,
                "cusps": [{"index": int(i), "pose": [float(v) for v in poses[i]]} for i in ends] if poses is not None else [],
            }
            tracker._switches_kept = 0
            self._versions.append(tracker._book)
        super().__setitem__(phase, tracker)

    def update(self, other=(), **more) -> None:  # dict.update would bypass __setitem__
        for phase, tracker in dict(other, **more).items():
            self[phase] = tracker

    def _drain(self, tracker, how: str) -> None:
        switches = getattr(tracker, "leg_switches", None) or []
        book = getattr(tracker, "_book", None) or {}
        for event in switches[getattr(tracker, "_switches_kept", 0):]:
            self._record.append({**event, "serial": book.get("serial"), "phase": book.get("phase"),
                                 "time_s": self._clock.get("t"), "stamped": how})
        tracker._switches_kept = len(switches)

    def collect(self) -> None:
        for tracker in list(self.values()):
            self._drain(tracker, "tick")

    def close(self) -> None:
        for tracker in list(self.values()):
            self._drain(tracker, "end")


# The M-a0 boundary record of the running mission, closed by whichever exit comes first (the run's own
# finally, or main()'s for failures before the mission loop's try).
_BOUNDARY_RECORD: dict = {"gate": None, "time": None, "probe": None, "print": None, "np": None}


def sink_effect(gate, method: str, path, payload, write=None):
    """Format 2's output-effect entry (forklift_core.mission.boundary_record.sink_effect); without a record only
    ``write``."""
    if gate is None:
        return None if write is None else write()
    from forklift_core.mission.boundary_record import sink_effect as effect_

    return effect_(gate, method, path, payload, write)


def _recording_print(*values, sep=" ", end="\n", file=None, flush=False):
    """Format 2's stdout observer (installed with the Clock proxy at the hand-over): each line printed to stdout
    while a tick is open and no recorded call runs is a ``Sink.echo`` entry -- the prints of the moved core and of
    the runner alike, wherever they stand."""
    gate = _BOUNDARY_RECORD.get("gate")
    builtin = _BOUNDARY_RECORD["print"]
    if gate is not None and (file is None or file is sys.stdout):
        # The text exactly as written, its end included (Codex P2), and the print inside the recorded call.
        text = (" " if sep is None else sep).join(str(v) for v in values) + ("\n" if end is None else end)
        return sink_effect(gate, "echo", "stdout", text,
                           lambda: builtin(*values, sep=sep, end=end, file=file, flush=flush))
    return builtin(*values, sep=sep, end=end, file=file, flush=flush)


def save_image(image, path) -> None:
    """A diagnostic image written to the run's output -- a Sink entry by its pixels while a format-2 record is open
    (PIL writes through its own file handle, past the output-path observer; Codex P2)."""
    gate = _BOUNDARY_RECORD.get("gate")
    if gate is None:
        return image.save(path)
    from replay_observers import _logical

    return sink_effect(gate, "image_save", _logical(path, _BOUNDARY_RECORD.get("output_root") or Path(str(path)).parent),
                       {"mode": image.mode, "pixels": np.asarray(image)}, lambda: image.save(path))


def install_output_observers(args) -> None:
    """Format 2 (sim/isaac/replay_observers.py): the output directory and this module's ``np`` record their file
    writes as Sink entries from here on (np undone at close; the observed directory only records while a record is
    open)."""
    from replay_observers import NumpyObserver, observed_output

    gate_fn = lambda: _BOUNDARY_RECORD.get("gate")  # noqa: E731
    args.output = observed_output(args.output, gate_fn)
    _BOUNDARY_RECORD["output_root"] = args.output
    _BOUNDARY_RECORD["np"] = globals()["np"]
    globals()["np"] = NumpyObserver(_BOUNDARY_RECORD["np"], gate_fn, args.output)


# The core containers the end-state projection covers (drive's names; a name the run never bound is left out).
PROJECTION_CONTAINERS = ("state", "near", "pocket", "obstacle", "slam", "paths", "new_obstacles")
# Result keys only the recorded run has (written by run() after the setup snapshot was taken): never compared.
PROJECTION_RECORDED_ONLY = ("setup_snapshot",)
# Objects a container holds for a service, which the service changes inside its recorded calls (the physics step
# runs the SLAM bridge's odometry, stop detector and noise): a replay never runs them, so they are compared by
# their logical id only -- their effects on the mission reach it through the watched keys (Codex P2).
PROJECTION_SERVICE_OWNED = {"slam": {"link": "SlamLink", "module": "SlamModule", "stop": "SlamStop",
                                     "odometry": "SlamOdometry", "noise": "SlamNoise", "range_model": "SlamRangeModel"},
                            # The grid preparation's own cache: written and read only inside the Planner port's
                            # grid_kwargs (the first whole replays: the only container key left different).
                            "obstacle": {"plan_cache": "PlannerGridCache"}}


_PROJECT_CLASSES: dict = {}


def _project_class(klass) -> bool:
    """A class defined in this project's code: the forklift_core package, or a module file under the checkout
    (the runner and its sibling modules, the tests)."""
    known = _PROJECT_CLASSES.get(klass)
    if known is None:
        name = getattr(klass, "__module__", "") or ""
        file_ = getattr(sys.modules.get(name), "__file__", None)
        root = Path(__file__).resolve().parents[2]
        known = name == "forklift_core" or name.startswith("forklift_core.") or bool(
            file_ and Path(file_).resolve().is_relative_to(root))
        _PROJECT_CLASSES[klass] = known
    return known


def projection_node(value, services: dict, roots=(), _stack=()):
    """One value of the end-state projection as plain JSON: services (recording/replay proxies, the SLAM link) by
    logical id, modules by name, random generators by their bit-generator state, arrays by dtype/shape/content hash,
    dataclasses and plain objects by all their attributes (CPU state is compared by value), containers by their
    items; an object with none of these (a socket, a file) raises TypeError -- the projection fails and the replay
    refuses the record (Codex P2)."""
    import dataclasses
    import hashlib
    import types as types_

    from forklift_core.mission.boundary_record import RecordingProxy
    from forklift_core.mission.replay import ReplayProxy

    if id(value) in services:
        return {"$service": services[id(value)]}
    if isinstance(value, (RecordingProxy, ReplayProxy)):
        return {"$service": object.__getattribute__(value, "_port")}
    if value is None or type(value) in (bool, int):
        return value
    if type(value) is float:
        return value if math.isfinite(value) else {"$float": repr(value)}
    if type(value) is str:
        for root in roots:
            value = value.replace(root, "<output>")
        return value
    if isinstance(value, (Path,)):
        return {"$path": projection_node(str(value), services, roots)}
    if isinstance(value, np.generic):
        return {"$np": value.dtype.str, "v": projection_node(value.item(), services, roots)}
    if isinstance(value, np.ndarray):
        digest = hashlib.sha256(value.dtype.str.encode() + repr(value.shape).encode()
                                + np.ascontiguousarray(value).tobytes()).hexdigest()
        return {"$array": digest}
    if isinstance(value, bytes):
        return {"$bytes": value.hex()}
    if isinstance(value, types_.ModuleType):
        return {"$module": value.__name__}
    if isinstance(value, np.random.Generator):
        return {"$rng": projection_node(value.bit_generator.state, services, roots, _stack)}
    if id(value) in _stack:
        return {"$cycle": type(value).__qualname__}
    stack = _stack + (id(value),)
    if isinstance(value, (list, tuple, collections.deque)):
        return {"$" + type(value).__name__: [projection_node(v, services, roots, stack) for v in value]}
    if isinstance(value, (set, frozenset)):
        items = [projection_node(v, services, roots, stack) for v in value]
        return {"$set": sorted(items, key=lambda n: json.dumps(n, sort_keys=True))}
    if isinstance(value, dict):
        items = [[projection_node(k, services, roots, stack), projection_node(v, services, roots, stack)]
                 for k, v in value.items()]
        return {"$dict": sorted(items, key=lambda kv: json.dumps(kv[0], sort_keys=True))}
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {"$object": f"{type(value).__module__}:{type(value).__qualname__}",
                "attrs": {f.name: projection_node(getattr(value, f.name), services, roots, stack)
                          for f in dataclasses.fields(value)}}
    if type(value) in (types_.SimpleNamespace, argparse.Namespace):  # plain attribute bags
        return {"$object": type(value).__qualname__,
                "attrs": {k: projection_node(v, services, roots, stack) for k, v in sorted(vars(value).items())}}
    if isinstance(value, random.Random):  # a native generator: its state is getstate(), not its attributes
        return {"$random": projection_node(value.getstate(), services, roots, _stack)}
    if hasattr(value, "__dict__") and not callable(value) and not isinstance(value, type) and all(
            klass is object or _project_class(klass) for klass in type(value).__mro__):
        # An object of this project's own classes (every class in its MRO defined in the project's code): its
        # attributes are its state. Any other object (a file, a socket, a C-backed generator) may keep state its
        # attributes do not show: refused below (Codex P2).
        return {"$object": f"{type(value).__module__}:{type(value).__qualname__}",
                "attrs": {k: projection_node(v, services, roots, stack) for k, v in sorted(vars(value).items())}}
    if isinstance(value, type) or callable(value):  # code: compared by name (its behaviour is the replay's)
        return {"$callable": f"{getattr(value, '__module__', None)}:{getattr(value, '__qualname__', type(value).__qualname__)}"}
    raise TypeError(f"no projection for a {type(value).__module__}:{type(value).__qualname__}")


def _identity_free(containers: dict) -> dict:
    """The containers with the values that are object identities (``id()``, a process's memory addresses) made
    independent of the process: the cusp heading-hold records key their tracker by ``id(tracker)`` -- replaced by
    its first-seen ordinal, so "the same tracker" still compares -- and the new-obstacle heights are keyed by
    ``id(rect)`` -- listed in the order of the pallet-frame rectangles they belong to (R5: two runs of the same
    code differed only there). Shallow copies; the run's own containers are not changed."""
    import copy

    out = dict(containers)
    state_ = containers.get("state")
    if isinstance(state_, dict) and any(k in state_ for k in ("cusp_heading_holds", "cusp_heading_hold",
                                                              "cusp_heading_watch")):
        ordinals: dict = {}

        def ordinal(tid):
            return f"<tracker {ordinals.setdefault(tid, len(ordinals))}>"

        state_ = dict(state_)
        holds = copy.deepcopy(state_.get("cusp_heading_holds"))
        for hold in holds or []:
            for field in ("key", "cancelled_by"):  # [phase, id(tracker), leg] (Codex P2: the cancelled ones too)
                if isinstance(hold, dict) and isinstance(hold.get(field), (list, tuple)) and len(hold[field]) == 3:
                    hold[field] = [hold[field][0], ordinal(hold[field][1]), hold[field][2]]
        if "cusp_heading_holds" in state_:
            state_["cusp_heading_holds"] = holds
        if state_.get("cusp_heading_hold"):
            open_ = copy.deepcopy(state_["cusp_heading_hold"])
            for field in ("key", "cancelled_by"):
                if isinstance(open_.get(field), (list, tuple)) and len(open_[field]) == 3:
                    open_[field] = [open_[field][0], ordinal(open_[field][1]), open_[field][2]]
            state_["cusp_heading_hold"] = open_
        if state_.get("cusp_heading_watch"):
            watch = dict(state_["cusp_heading_watch"])
            if "tracker" in watch:
                watch["tracker"] = ordinal(watch["tracker"])
            state_["cusp_heading_watch"] = watch
        out["state"] = state_
    new_ = containers.get("new_obstacles")
    if isinstance(new_, dict) and isinstance(new_.get("heights"), dict):
        new_ = dict(new_)
        heights = dict(new_["heights"])
        ordered = [heights.pop(id(r), None) for r in new_.get("pallet_frame") or []]
        new_["heights"] = {"by_pallet_frame_rect": ordered, "others": sorted(heights.values())}
        out["new_obstacles"] = new_
    return out


def final_projection(containers: dict, ms, output: Path) -> dict:
    """Format 2's end state (M-a1 replay plan R3 (5), Codex P2): the mission state's fields by value and, for each
    core container, every key by the digest of its projection (:func:`projection_node`) -- the run's output
    directory, as the run sees it and as its arguments record it, written as ``<output>`` (a replay writes
    elsewhere). Keys only the recorded run can have are not in it."""
    import hashlib

    services: dict = {}
    for name_, owned in PROJECTION_SERVICE_OWNED.items():
        container_ = containers.get(name_)
        if isinstance(container_, dict):
            for key_, logical in owned.items():
                if container_.get(key_) is not None:
                    services[id(container_[key_])] = logical
    containers = _identity_free(containers)
    state_ = containers.get("state") if isinstance(containers.get("state"), dict) else {}
    roots = sorted({str(output)} | {str(v) for v in [(state_.get("arguments") or {}).get("output")] if v},
                   key=len, reverse=True)
    out = {}
    for name, container in containers.items():
        if not isinstance(container, dict):
            out[name] = None if container is None else f"not_a_dict:{type(container).__name__}"
            continue
        digests = {}
        for key, value in container.items():
            if name == "state" and key in PROJECTION_RECORDED_ONLY:
                continue
            if key in PROJECTION_SERVICE_OWNED.get(name, {}):
                continue  # a port's own object: never compared (a replay may not even hold it)
            try:
                node = projection_node(value, services, roots)
            except TypeError as exc:
                raise TypeError(f"{name}[{key!r}]: {exc}") from None
            text = json.dumps(node, sort_keys=True, allow_nan=True)
            digests[str(key)] = hashlib.sha256(text.encode("utf-8")).hexdigest()
        out[name] = digests
    return {"mission": ms, "containers": out}


def close_boundary_record(state: dict, exc: BaseException | None = None, final_state=None) -> None:
    """``exc``: the exception propagating through the caller's finally (main() records it as the failure
    reason only after the run's own finally has closed the record). ``final_state`` (format 2): drive's
    end-state projection (:func:`final_projection`)."""
    gate = _BOUNDARY_RECORD.get("gate")
    if gate is None:
        return
    try:
        if gate.recorder.in_tick:  # a break, an early return or a failure inside a tick
            probe = _BOUNDARY_RECORD.get("probe")
            reason = state.get("failure_reason") or (str(exc) if exc is not None else None)
            gate.recorder.tick_end(outputs={"exit": state.get("phase"), "failure_reason": reason},
                                   branches=probe() if probe is not None else [])
        if gate.recorder.value_format == 2:  # how drive ended: a replay has to end the same way
            from forklift_core.mission.boundary_record import boundary_ending

            gate.recorder.close(final_state=final_state, ending=boundary_ending(exc))
        else:
            gate.recorder.close()
    except Exception as exc:  # noqa: BLE001 -- a recording failure never replaces the run's own result
        state["boundary_record_error"] = repr(exc)
    finally:
        _BOUNDARY_RECORD["gate"] = None
        _BOUNDARY_RECORD["probe"] = None
        if _BOUNDARY_RECORD.get("time") is not None:
            globals()["time"] = _BOUNDARY_RECORD["time"]  # the wall clock proxy (Clock) undone
            _BOUNDARY_RECORD["time"] = None
        if _BOUNDARY_RECORD.get("print") is not None:
            globals().pop("print", None)  # the stdout observer undone (the builtin again)
            _BOUNDARY_RECORD["print"] = None
        if _BOUNDARY_RECORD.get("np") is not None:
            globals()["np"] = _BOUNDARY_RECORD["np"]  # the file-writer observer undone
            _BOUNDARY_RECORD["np"] = None


SLAM_LIVE_REPLY_DEADLINE_S = 1.0  # real-time plan D9a: wall seconds a sent scan may wait for its reply
SLAM_LIVE_QUEUE_MAX = 20  # and how many scans may wait behind it (2 s of 10 Hz scans)


def run(app, args: argparse.Namespace, settings: dict, state: dict) -> None:
    """Construct and execute one immutable seeded scenario; state keeps evidence."""
    from forklift_core.mission.state import MissionState

    # M-a1 replay plan R3 (format 2): the files the setup reads, from here to the hand-over to drive().
    setup_files = None
    if args.record_boundary and args.record_boundary_format == 2:
        from replay_setup import FileAudit

        setup_files = FileAudit()

    ms = MissionState()  # M-a1 step 0: the phase and its start, shared with the mission core
    tracker_book = {"book": None}  # set once the trackers exist; drained in the finally block
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
    from forklift_core.control import cusp_stall as CUSP_STALL
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
        redraw_docking,
        straight_from_pose,
        plan_return_leg,
        plan_transport_leg,
        site_poses,
    )

    # M-a0 boundary record (docs/plans/2026-10-15-m-a05-boundary.md section 5): the planners are a port
    # (they check wall-clock deadlines and read the asynchronously replaced SLAM map), so each call's
    # request and response is recorded; the services are wrapped before the mission loop.
    boundary = None
    if args.record_boundary:
        from forklift_core.mission.boundary_record import (
            BoundaryRecorder,
            RecordingGate,
            RecordingProxy,
            recorded_function,
        )

        boundary = RecordingGate(BoundaryRecorder(args.output / "boundary", value_format=args.record_boundary_format))
        boundary.recorder.value_types.add("ArticulationAction")  # the joint commands, by value
        _BOUNDARY_RECORD["gate"] = boundary
        plan_docking_reapproach = recorded_function(plan_docking_reapproach, "Planner", "plan_docking_reapproach", boundary)
        plan_transport = recorded_function(plan_transport, "Planner", "plan_transport", boundary)
        final_straight_prefix = recorded_function(final_straight_prefix, "Planner", "final_straight_prefix", boundary)
        redraw_docking = recorded_function(redraw_docking, "Planner", "redraw_docking", boundary)
        straight_from_pose = recorded_function(straight_from_pose, "Planner", "straight_from_pose", boundary)
        plan_return_leg = recorded_function(plan_return_leg, "Planner", "plan_return_leg", boundary)
        plan_transport_leg = recorded_function(plan_transport_leg, "Planner", "plan_transport_leg", boundary)
        bicycle_rollout = recorded_function(bicycle_rollout, "Planner", "bicycle_rollout", boundary)

    if args.use_perception:
        from forklift_core.perception.pocket_detector import (
            DetectorParams,
            detect_pockets,
        )
        from forklift_core.planning import Pose2D
        from forklift_core.planning.observation_viewpoints import runtime_viewpoints
        from forklift_core.planning.pallet_mission import plan_observation_leg

        if boundary is not None:
            detect_pockets = recorded_function(detect_pockets, "Perception", "detect_pockets", boundary)
            runtime_viewpoints = recorded_function(runtime_viewpoints, "Planner", "runtime_viewpoints", boundary)
            plan_observation_leg = recorded_function(plan_observation_leg, "Planner", "plan_observation_leg", boundary)

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
    # The chassis comparison sees axles, joints, wheels and collision boxes, not masses: a scene built
    # from another URDF (e.g. before the fork-tip frames got their explicit mass) must be refused by
    # the URDF it was built from, as its manifest records it (EPAL 6 fork spacing plan, new plant).
    scene_manifest = Path(args.base_scene).resolve().parent / "base_scene_manifest.json"
    if scene_manifest.exists():
        built_from = json.loads(scene_manifest.read_text()).get("forklift_urdf_sha256")
        running = hashlib.sha256(Path(args.forklift_urdf).read_bytes()).hexdigest()
        require(built_from in (None, running), "base scene built from another forklift URDF")
        state["scene_built_from_urdf_sha256"] = built_from
    hz = float(settings["physics_hz"])  # 120 except in a --steering-probe convergence check
    world = World(stage_units_in_meters=1.0, physics_dt=1 / hz, rendering_dt=1 / hz)
    physics_context = world.get_physics_context()
    # Cause separation E (diagnostic, --steering-probe only): one scene solver setting at a time.
    if settings.get("physx_friction_correlation_distance_m") is not None:
        physics_context.set_friction_correlation_distance(float(settings["physx_friction_correlation_distance_m"]))
    if settings.get("physx_solver_pgs"):
        physics_context.set_solver_type("PGS")
    if settings.get("physx_enable_stabilization"):
        physics_context.enable_stablization(True)
    if settings.get("physx_external_forces_every_iteration"):
        physics_context._physx_scene_api.CreateEnableExternalForcesEveryIterationAttr(True)
    state["physics_scene"] = {
        "solver_type": physics_context.get_solver_type(),
        "friction_correlation_distance_m": physics_context.get_friction_correlation_distance(),
        "friction_offset_threshold_m": physics_context.get_friction_offset_threshold(),
        "stabilization": physics_context.is_stablization_enabled(),
        "external_forces_every_iteration": physics_context._physx_scene_api.GetEnableExternalForcesEveryIterationAttr().Get(),
        "dt_s": physics_context.get_physics_dt(),
    }
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
    # The observe leg the replan budget belongs to: serial 0 is the far observation leg, each near-leg
    # installation the next; "start" is its first entry in observe_replans (fix (1)).
    state["observe_replan_leg"] = {"serial": 0, "start": 0}
    state["cusp_stalls"] = []  # plan cusp position stall v7 K4, one record per stall
    ms.observe_stall_ticks = 0
    ms.slam_stall_ticks = 0
    ms.dock_stop_ticks = 0
    max_cusp_replans = 2
    # Stopped = zero command, planar speed and yaw rate under these for this
    # many consecutive ticks (0.1 s), not the forward speed alone (Codex review).
    ms.cusp_stop_ticks, cusp_stop_needed = 0, 12
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
    # The plant record (EPAL 6 fork spacing plan, new plant candidate): each tyre's material as bound.
    from scene import plant_readback, tyre_contact_readback

    state["plant_tyre_contact"] = tyre_contact_readback(stage)
    state["plant"] = plant_readback(stage)
    if args.steering_probe or args.tyre_contact_log:
        # The probe and the mission log read each tyre's floor contact directly (not the joint
        # reaction forces).
        from pxr import PhysxSchema as PS

        for wheel_ in ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"):
            report_ = PS.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"/World/Forklift/{wheel_}"))
            report_.CreateThresholdAttr(0.0)
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
        if args.pocket_check or args.record_pocket_frames or args.near_tracking:
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
    from types import SimpleNamespace as _Cells

    cells = _Cells(world=world, robot=robot, stepper=None)

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
    if getattr(args, "live", False) and not args.video:
        # Real-time plan S4 (a): the overview camera (1280x720) feeds only the video, its inset and the robot-camera
        # composite, all of which need --video; without it a live run neither renders it nor evaluates its frame
        # pipeline at every render step. FORKLIFT_LIVE_OVERVIEW=on keeps it (an A/B of the render cost).
        if os.environ.get("FORKLIFT_LIVE_OVERVIEW", "off") == "on":
            state["overview_render"] = "on (FORKLIFT_LIVE_OVERVIEW)"
        else:
            try:
                camera.pause()
                camera._render_product.hydra_texture.set_updates_enabled(False)
                state["overview_render"] = "off"
            except Exception as exc:  # noqa: BLE001 -- an SDK without the switch keeps rendering it
                state["overview_render"] = f"on ({type(exc).__name__}: {exc})"
    import carb

    # Every run records the render mode it used (G2 rerun plan, 2026-10-01).
    state["render_mode"] = {
        "before_first_capture": carb.settings.get_settings().get("/rtx/rendermode")
    }
    for extra_camera in extra_cameras.values():
        extra_camera.initialize()
    far_runtime = far_intrinsics = frr = None  # the far recognition (--recognition far)
    if args.recognition == "far":
        import far_recognition_runtime as frr
    if args.use_perception:
        perception_camera.initialize()
        verify_camera_intrinsics(perception_camera, perception_calibration, state)
        # Capture needs axial depth as well as RGBA (see determinism_probe.py).
        perception_camera.add_distance_to_image_plane_to_frame()
        if args.recognition == "far":
            # Far recognition: oracle instance masks from the semantic segmentation. The run's pallet is
            # instance 0, the factory's storage pallets the next ids (proposed too, so they can compete), and
            # the warehouse environment is "floor" -- the background label decode_semantic's readiness needs
            # (plan v2, Codex P1).
            from isaacsim.core.utils.semantics import add_labels

            label_targets = [("/World/Pallet", "inst_0"), ("/World/Environment", "floor")]
            if factory is not None:
                import factory_assets as _fa

                storage = [i_ for i_, item_ in enumerate(factory.work_items) if item_.asset.uri.endswith(_fa.PALLET)]
                label_targets += [(f"/World/Factory/Item_{i_}", f"inst_{k_}") for k_, i_ in enumerate(storage, start=1)]
            for path_, label_ in label_targets:
                prim_ = stage.GetPrimAtPath(path_)
                require(prim_.IsValid(), f"recognition label target {path_} is missing")
                add_labels(prim_, labels=[label_], instance_name="class")
            perception_camera.add_semantic_segmentation_to_frame({"colorize": False})
            state["recognition"] = {"labels": [list(t_) for t_ in label_targets]}
        if args.pocket_check or args.record_pocket_frames or args.near_tracking:
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
        # M-a1 replay plan R2: the capture's callbacks read the run's current stepper, world and robot through
        # this holder (the drive part rebinds them -- the stepper is made there, the proxies replace world and
        # robot), as they read run()'s own names before the split.
        cells.world, cells.robot = world, robot
        lift_guard = None
        if args.perception_mount != "legacy":
            lift_joint = list(robot.dof_names).index("fork_lift")

            def lift_guard():
                # Every render step of a capture, not only before and after:
                # a carriage camera's base<-optical holds at lift 0 only.
                if abs(float(cells.robot.get_joint_positions()[lift_joint])) > 0.001:
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
            step_fn=lambda: cells.stepper["fn"](True),
            physics_time_fn=lambda: cells.world.current_time,
            pose_fn=robot.get_world_pose,
            guard_fn=lift_guard,
            semantic_fn=(lambda: frr.decode_mission_semantic(
                perception_camera.get_current_frame().get("semantic_segmentation"))) if args.recognition == "far" else None,
        )
        if args.recognition == "far":
            from forklift_core.planning.observation_viewpoints import BAY_PICKUP_ZONE as _Z

            far_runtime = frr.FarRuntime(
                fork_spacing=frr.fork_spacing_m(read_fork_blades_m(args.forklift_urdf)),
                pallet_depth_m=geometry.pallet_depth_m, pallet_shape=args.pallet_geometry_loaded.geometry_version,
                # The operator's "the pallet around here": the layout's placement zone (never scenario.pickup)
                zone_centre_xy=((_Z.x_min_m + _Z.x_max_m) / 2, (_Z.y_min_m + _Z.y_max_m) / 2),
                zone_radius_m=math.hypot((_Z.x_max_m - _Z.x_min_m) / 2, (_Z.y_max_m - _Z.y_min_m) / 2),
                camera_x_m=float(perception_mount.translation_m[0]), dt=1 / 120,
                frame_every=args.recognition_frame_every, lag_steps=args.recognition_pixel_lag_steps,
                drive_frames=not args.recognition_no_drive_frames, out_dir=args.output / "recognition",
            )
            far_intrinsics = adapter.read_isaac_intrinsics(perception_camera).integer_index
            _plain_capture = perception_capture.capture

            def _paused_capture(**kw):
                # No driving frame inside a stationary capture's own steps (its frames are the stop's).
                far_runtime.paused = True
                try:
                    return _plain_capture(**kw)
                finally:
                    far_runtime.paused = False

            perception_capture.capture = _paused_capture
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
    try:
        # New plant candidate V0: what the engine holds per body, not what the stage authored.
        view_ = robot._articulation_view
        state["plant_backend"] = {
            "body_names": list(view_.body_names),
            # isaacsim.core.prims Articulation (Isaac Sim 5.1): get_body_* (get_masses does not exist).
            "masses_kg": np.asarray(view_.get_body_masses(), dtype=float).reshape(-1).tolist(),
            "coms": np.asarray(view_.get_body_coms()[0], dtype=float).reshape(-1, 3).tolist(),
            "inertias": np.asarray(view_.get_body_inertias(), dtype=float).reshape(-1, 9).tolist(),
        }
        # Cause separation E: the drives as the engine holds them (gains, effort limits) per DOF.
        kps_, kds_ = view_.get_gains()
        state["plant_backend"].update(
            dof_names=list(view_.dof_names),
            drive_stiffness=np.asarray(kps_, dtype=float).reshape(-1).tolist(),
            drive_damping=np.asarray(kds_, dtype=float).reshape(-1).tolist(),
            max_efforts=np.asarray(view_.get_max_efforts(), dtype=float).reshape(-1).tolist(),
        )
    except Exception as exc_:  # noqa: BLE001 -- the API differs between releases; record why
        state["plant_backend"] = {"error": repr(exc_)}
    if args.steering_probe:
        state["steering_probe"] = run_steering_probe(
            world, robot, wheels, steers, settings, args, ArticulationAction, ackermann_command, args.output
        )
        state["phase"] = "steering_probe_done"
        state["success"] = True
        return
    obstacle = None  # the priority-5 obstacle layer
    if args.obstacle_layer is not None:
        import obstacle_layer as obstacle_module

        from forklift_core.perception.obstacle_grid import AgeErrorTable

        layer_config = obstacle_module.load_layer_config(args.obstacle_layer)
        age_table = json.loads(Path(layer_config["odometry_age"]).read_text())
        table_problem_ = age_table_problem(age_table, args)
        require(table_problem_ is None, str(table_problem_))
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

    new_obstacles = None
    # Priority-5 D5 depth pocket check: built at the near capture, fed 10 Hz
    # carriage frames on the approach straight and the insertion.
    pocket = {"check": None, "history": [], "capture": None, "frames_read": 0}
    # Plan D8b: every near-field read and the truth of every tick, for the offline
    # delay and error calibration (tools/d8b_calibration.py).
    recorder = pocket_recorder.PocketFrameRecorder(args.output) if args.record_pocket_frames else None
    dwells = pocket_recorder.DwellSchedule(args.record_dwell_gaps) if args.record_dwell_gaps is not None else None
    safety_stops = (pocket_recorder.SafetyStopSchedule(args.measure_safety_stops_m)
                    if args.measure_safety_stops_m is not None else None)
    # Plan D8 S4a: near-field pocket tracking. The contract with the measured bounds and
    # drive terms is checked before the first tick (check_near_contract); the tracker is
    # built after the near capture's hold.
    near = None
    if args.near_tracking:
        from forklift_core.perception.near_field_bounds import NearFieldBounds

        near_bounds_data = json.loads(args.near_bounds.read_text())
        near_bounds_sha = hashlib.sha256(args.near_bounds.read_bytes()).hexdigest()
        state["near_tracking"] = {
            "bounds": str(args.near_bounds),
            "bounds_sha256": near_bounds_sha,
            "drive_terms": str(args.near_drive_terms),
            "drive_terms_sha256": hashlib.sha256(args.near_drive_terms.read_bytes()).hexdigest(),
            "stop_at": args.near_tracking_stop_at,
            # Plan D8 S4 near field, protection prediction 4판: the rule the 0.6 m correction screens with --
            # always recorded, so the judgement (xii) applies whether or not a prediction record exists.
            "protection_prediction_rules": near_tracking.PREDICT_RULES,
        }
        if args.near_inject is not None or args.near_drop_frames_from_m is not None:
            # Plan D8 S4c-1: what was injected, capture by capture (the judgement's (ix) reads it).
            state["near_tracking"]["injection"] = {
                "kind": args.near_inject,
                "lateral_m": args.near_inject_lateral_m,
                "yaw_rad": args.near_inject_yaw_rad,
                "drop_from_m": args.near_drop_frames_from_m,
                "captures": [],
            }
        near = {
            "bounds": NearFieldBounds.from_dict(near_bounds_data),
            "bounds_data": near_bounds_data,
            "terms": None,
            "tracker": None,
            "start": None,  # the near capture's observation, pose and times, until the hold
            "episode": 0,  # SLAM hold count: the tracker's frame_version
            "history": near_tracking.RearHistory(args.rear_axle_offset_m),
            "commands": near_tracking.CommandWindow(),
            "last_stamp": None,  # the last frame stamp received (tau)
            "stop_state": None,  # near_tracking.StopState once the tracker is built
            "frames": [],
            "ticks": [],
            "events": [],
            # Plan D8 S4b: the correction points and the path revisions they installed.
            "correction": None,
            "corrections": [],
            "path_revision": 0,
            "path_revisions": [],
            "first_target": None,  # the near capture's mission target (its band, B0)
            "retries": 0,  # Plan D8 S4c-2: pickup retries so far (at most one)
            "retry_stand": False,
        }
    wheel_lock = {"q": None, "gains": None, "events": []}
    # Camera-face distance at the approach goal: the stand-off gap ahead of the fork
    # tips, less the camera's distance behind them (0.431 m on the measured chassis).
    dwell_d_end_m = (
        geometry.approach_gap_m + geometry.axle_to_fork_tip_m
        - (float(perception_mount.translation_m[0]) + abs(args.rear_axle_offset_m))
    ) if args.record_pocket_frames else None








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









    def spawn_box(path_, ox, oy, oyaw, osize) -> None:
        """A new obstacle's collider prim (scenario injection): USD work the drive part asks for."""
        from pxr import Gf as NGf, UsdGeom as NUsdGeom, UsdPhysics as NUsdPhysics

        cube = NUsdGeom.Cube.Define(stage, path_)
        cube.CreateSizeAttr(1.0)
        xf = NUsdGeom.XformCommonAPI(cube)
        xf.SetTranslate(NGf.Vec3d(float(ox), float(oy), float(osize[2]) / 2))
        xf.SetRotate(NGf.Vec3f(0.0, 0.0, float(math.degrees(oyaw))))
        xf.SetScale(NGf.Vec3f(*(float(v) for v in osize)))
        NUsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    def place_chase_camera(chase_camera, chase_eye, chase_target) -> None:
        """The chase view's camera looking from its eye at its target (USD math; a video Sink)."""
        chase_look = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*chase_eye),
            Gf.Vec3d(*chase_target),
            Gf.Vec3d(0, 0, 1),
        )
        chase_quat = chase_look.GetInverse().ExtractRotationQuat()
        chase_camera.set_world_pose(
            position=chase_eye,
            orientation=np.array(
                [chase_quat.GetReal(), *chase_quat.GetImaginary()]
            ),
            camera_axes="usd",
        )

    def cast_reference_scan(*a, **k):
        """The delivery reference scan's ray cast (a USD scene query)."""
        import planar_lidar as _planar_lidar

        return _planar_lidar.cast_scan(*a, **k)

    # M-a1 replay plan R2: the Isaac work after this point goes through these adapters (a replay supplies its own).
    isaac = _Cells(
        action=lambda **kw: ArticulationAction(**kw),
        add_path_display=add_path_display,
        export_scene=lambda: stage.GetRootLayer().Export(str(args.output / "scene.usda")),
        spawn_box=spawn_box,
        remove_prim=lambda path_: stage.RemovePrim(path_),
        slam_link_client=lambda *a, **k: __import__("forklift_core.localization.slam_link",
                                                    fromlist=["SlamLinkClient"]).SlamLinkClient(*a, **k),
        cast_reference_scan=cast_reference_scan,
        place_chase_camera=place_chase_camera,
    )
    from types import SimpleNamespace as _Setup

    if setup_files is not None:  # M-a1 replay plan R3: the setup snapshot a replay builds drive's S from
        from replay_setup import write_setup_snapshot

        state["setup_snapshot"] = write_setup_snapshot(
            dict(locals()), args.output / "boundary",
            # what the setup opened, and the files the arguments name (main() read some before run())
            setup_files.stop() + [str(v_) for v_ in vars(args).values()
                                  if isinstance(v_, (str, Path)) and len(str(v_)) < 4096 and Path(v_).is_file()],
            services=({"Obstacles": obstacle["layer"], "Permission": obstacle["layer"].permission,
                       "Memory": obstacle["layer"].memory} if obstacle is not None else None),
            repo_root=Path(__file__).resolve().parents[2],
            wanted=set(__import__("replay_setup").drive_inputs(drive)))
    return drive(_Setup(**locals()), isaac)


def drive(S, isaac) -> None:
    """The run from its initial plans on (M-a1 replay plan R2): every Isaac object it touches is in ``S``
    (a service, recorded when recording) or behind ``isaac`` (adapters), so a replay can run it on CPU with
    its own. ``S`` holds run()'s names as they were bound when the setup ended; a name the setup left
    unbound stays unbound here, as it was."""
    _bound = S.__dict__
    if "far_runtime" in _bound:
        far_runtime = _bound["far_runtime"]
    if "far_intrinsics" in _bound:
        far_intrinsics = _bound["far_intrinsics"]
    if "frr" in _bound:
        frr = _bound["frr"]
    if "ARC_POSES" in _bound:
        ARC_POSES = _bound["ARC_POSES"]
    if "CUSP_STALL" in _bound:
        CUSP_STALL = _bound["CUSP_STALL"]
    if "DetectorParams" in _bound:
        DetectorParams = _bound["DetectorParams"]
    if "Footprint" in _bound:
        Footprint = _bound["Footprint"]
    if "FootprintCollisionChecker" in _bound:
        FootprintCollisionChecker = _bound["FootprintCollisionChecker"]
    if "Image" in _bound:
        Image = _bound["Image"]
    if "PALLET_BOXES" in _bound:
        PALLET_BOXES = _bound["PALLET_BOXES"]
    if "PARTS_OF" in _bound:
        PARTS_OF = _bound["PARTS_OF"]
    if "Pose2D" in _bound:
        Pose2D = _bound["Pose2D"]
    if "RearAxlePathTracker" in _bound:
        RearAxlePathTracker = _bound["RearAxlePathTracker"]
    if "RecordingProxy" in _bound:
        RecordingProxy = _bound["RecordingProxy"]
    if "Rectangle" in _bound:
        Rectangle = _bound["Rectangle"]
    if "SHAPE_MEETS" in _bound:
        SHAPE_MEETS = _bound["SHAPE_MEETS"]
    if "TrackerConfig" in _bound:
        TrackerConfig = _bound["TrackerConfig"]
    if "adapter" in _bound:
        adapter = _bound["adapter"]
    if "args" in _bound:
        args = _bound["args"]
    if "bicycle_rollout" in _bound:
        bicycle_rollout = _bound["bicycle_rollout"]
    if "boundary" in _bound:
        boundary = _bound["boundary"]
    if "camera" in _bound:
        camera = _bound["camera"]
    if "cells" in _bound:
        cells = _bound["cells"]
    if "collision_free_pose" in _bound:
        collision_free_pose = _bound["collision_free_pose"]
    if "cusp_stop_needed" in _bound:
        cusp_stop_needed = _bound["cusp_stop_needed"]
    if "cusp_stop_yaw_rate_radps" in _bound:
        cusp_stop_yaw_rate_radps = _bound["cusp_stop_yaw_rate_radps"]
    if "detect_pockets" in _bound:
        detect_pockets = _bound["detect_pockets"]
    if "directions" in _bound:
        directions = _bound["directions"]
    if "dwell_d_end_m" in _bound:
        dwell_d_end_m = _bound["dwell_d_end_m"]
    if "dwells" in _bound:
        dwells = _bound["dwells"]
    if "estop" in _bound:
        estop = _bound["estop"]
    if "extra_cameras" in _bound:
        extra_cameras = _bound["extra_cameras"]
    if "factory" in _bound:
        factory = _bound["factory"]
    if "final_straight_prefix" in _bound:
        final_straight_prefix = _bound["final_straight_prefix"]
    if "geometry" in _bound:
        geometry = _bound["geometry"]
    if "grid_planning" in _bound:
        grid_planning = _bound["grid_planning"]
    if "initial_pallet" in _bound:
        initial_pallet = _bound["initial_pallet"]
    if "insertion_geometry" in _bound:
        insertion_geometry = _bound["insertion_geometry"]
    if "inset" in _bound:
        inset = _bound["inset"]
    if "inset_font" in _bound:
        inset_font = _bound["inset_font"]
    if "kds_" in _bound:
        kds_ = _bound["kds_"]
    if "kps_" in _bound:
        kps_ = _bound["kps_"]
    if "lift_index" in _bound:
        lift_index = _bound["lift_index"]
    if "max_cusp_replans" in _bound:
        max_cusp_replans = _bound["max_cusp_replans"]
    if "mission_stages" in _bound:
        mission_stages = _bound["mission_stages"]
    if "ms" in _bound:
        ms = _bound["ms"]
    if "n" in _bound:
        n = _bound["n"]
    if "name" in _bound:
        name = _bound["name"]
    if "near" in _bound:
        near = _bound["near"]
    if "new_obstacles" in _bound:
        new_obstacles = _bound["new_obstacles"]
    if "obstacle" in _bound:
        obstacle = _bound["obstacle"]
    if "obstacle_module" in _bound:
        obstacle_module = _bound["obstacle_module"]
    if "origin" in _bound:
        origin = _bound["origin"]
    if "overlay" in _bound:
        overlay = _bound["overlay"]
    if "pallet" in _bound:
        pallet = _bound["pallet"]
    if "path" in _bound:
        path = _bound["path"]
    if "perception_calibration" in _bound:
        perception_calibration = _bound["perception_calibration"]
    if "perception_camera" in _bound:
        perception_camera = _bound["perception_camera"]
    if "perception_capture" in _bound:
        perception_capture = _bound["perception_capture"]
    if "perception_display" in _bound:
        perception_display = _bound["perception_display"]
    if "perception_mount" in _bound:
        perception_mount = _bound["perception_mount"]
    if "pickup_bounds" in _bound:
        pickup_bounds = _bound["pickup_bounds"]
    if "pickup_zone" in _bound:
        pickup_zone = _bound["pickup_zone"]
    if "plan_docking_reapproach" in _bound:
        plan_docking_reapproach = _bound["plan_docking_reapproach"]
    if "plan_observation_leg" in _bound:
        plan_observation_leg = _bound["plan_observation_leg"]
    if "plan_return_leg" in _bound:
        plan_return_leg = _bound["plan_return_leg"]
    if "plan_transport" in _bound:
        plan_transport = _bound["plan_transport"]
    if "plan_transport_leg" in _bound:
        plan_transport_leg = _bound["plan_transport_leg"]
    if "planner_config" in _bound:
        planner_config = _bound["planner_config"]
    if "pocket" in _bound:
        pocket = _bound["pocket"]
    if "pocket_camera" in _bound:
        pocket_camera = _bound["pocket_camera"]
    if "q" in _bound:
        q = _bound["q"]
    if "read_fork_blades_m" in _bound:
        read_fork_blades_m = _bound["read_fork_blades_m"]
    if "recorded_function" in _bound:
        recorded_function = _bound["recorded_function"]
    if "recorder" in _bound:
        recorder = _bound["recorder"]
    if "redraw_docking" in _bound:
        redraw_docking = _bound["redraw_docking"]
    if "return_to_pose" in _bound:
        return_to_pose = _bound["return_to_pose"]
    if "robot" in _bound:
        robot = _bound["robot"]
    if "runtime_viewpoints" in _bound:
        runtime_viewpoints = _bound["runtime_viewpoints"]
    if "safety_stops" in _bound:
        safety_stops = _bound["safety_stops"]
    if "scenario" in _bound:
        scenario = _bound["scenario"]
    if "sensor" in _bound:
        sensor = _bound["sensor"]
    if "settings" in _bound:
        settings = _bound["settings"]
    if "site_poses" in _bound:
        site_poses = _bound["site_poses"]
    if "stage" in _bound:
        stage = _bound["stage"]
    if "state" in _bound:
        state = _bound["state"]
    if "steers" in _bound:
        steers = _bound["steers"]
    if "straight_from_pose" in _bound:
        straight_from_pose = _bound["straight_from_pose"]
    if "tracker_book" in _bound:
        tracker_book = _bound["tracker_book"]
    if "travel_config" in _bound:
        travel_config = _bound["travel_config"]
    if "truth_destination" in _bound:
        truth_destination = _bound["truth_destination"]
    if "unloaded_shape" in _bound:
        unloaded_shape = _bound["unloaded_shape"]
    if "v" in _bound:
        v = _bound["v"]
    if "wheel_lock" in _bound:
        wheel_lock = _bound["wheel_lock"]
    if "wheels" in _bound:
        wheels = _bound["wheels"]
    if "world" in _bound:
        world = _bound["world"]
    if "x0" in _bound:
        x0 = _bound["x0"]
    if "x1" in _bound:
        x1 = _bound["x1"]
    if "y0" in _bound:
        y0 = _bound["y0"]
    if "y1" in _bound:
        y1 = _bound["y1"]

    def record_video_overlay(t_now: float) -> None:
        from forklift_core.perception.obstacle_grid import OCCUPIED as GRID_OCCUPIED

        current = paths.get(ms.phase)
        if current is not None and id(current) != overlay["plan_id"]:
            overlay["plan_id"] = id(current)
            why = "plan"
            if obstacle.get("backoff", {}).get("phase") == ms.phase:
                why = "backoff"
            elif obstacle.get("replans") and abs(obstacle["replans"][-1]["time_s"] - t_now) < 0.6:
                why = "replan"
            elif obstacle.get("live_plans") and abs(obstacle["live_plans"][-1]["time_s"] - t_now) < 0.6:
                why = "live"
            poses = np.asarray(current.poses, dtype=float)
            step_ = max(1, len(poses) // 400)
            overlay["plan_history"].append(
                {"time_s": float(t_now), "phase": ms.phase, "why": why,
                 "poses": np.round(poses[::step_, :2], 3).tolist() + [np.round(poses[-1, :2], 3).tolist()]}
            )
        snap_ = obstacle["layer"].snapshot
        if snap_ is not None and t_now - overlay["grid_t"] >= 0.5:
            overlay["grid_t"] = t_now
            cells_ = np.argwhere(snap_.state == GRID_OCCUPIED).astype(np.int32)
            overlay["grids"].append((float(t_now), float(snap_.origin_x_m), float(snap_.origin_y_m),
                                     float(snap_.resolution_m), cells_))

    def build_pocket_check(axis_yaw: float, rear_now, base_now, stamp_s=None, rear_at_stamp=None) -> None:
        """The D5 pocket check, at the near (stand-off) capture: the final estimate,
        the approach straight's heading, and that capture as its first frame -- at this
        tick's time and pose, or (real-time plan D8b: the check built after the plan was
        waited for) at the capture's own time and pose."""
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
        first = check.add_frame(now_s if stamp_s is None else float(stamp_s), scene.depth_m,
                                tuple(float(v) for v in (rear_now if rear_at_stamp is None else rear_at_stamp)))
        # M-a0.5 PocketCheck service: depth frames recorded by hash only.
        pocket["check"] = check if boundary is None else RecordingProxy(
            check, "PocketCheck", boundary, hash_only_requests={"add_frame"})
        state["pocket_check"] = {"built_s": now_s, "axis_yaw_rad": axis_yaw, "first_frame": first}
        require(check.valid, f"pocket_check_refused:{check.invalid_reason}")
        obstacle["layer"].exempt = check.region_control()
        obstacle["layer"].depth_support = check.depth_free_cells

    def record_pocket_read(raw) -> None:
        """Plan D8b: one near-field read for the recorder -- its time, fabric frame and the
        control pose of the loop tick it was read after, kept whether or not depth came
        back (no_depth) or normalises (bad_depth)."""
        frame = pocket_camera.get_current_frame()
        rendered = frame.get("rendering_time") if isinstance(frame, dict) else None
        try:
            stamp_r = float(rendered) - initial_time if rendered is not None else None
        except (TypeError, ValueError):
            stamp_r = None
        depth_r, error_r = None, None
        if raw is not None:
            try:
                k_r = pocket["capture"].intrinsics
                depth_r, _ = adapter.normalize_depth(np.asarray(raw).reshape(k_r.height, k_r.width))
            except (ValueError, TypeError) as exc:
                error_r = repr(exc)
        control_ = pocket.get("control_stamped")
        row = recorder.record_read(
            read_s=world.current_time - initial_time,
            stamp_s=stamp_r if stamp_r is not None and math.isfinite(stamp_r) else None,
            rendering_frame=frame.get("rendering_frame") if isinstance(frame, dict) else None,
            depth_m=depth_r, lift_m=float(robot.get_joint_positions()[lift_index[0]]),
            control_rear=None if control_ is None else control_[1],
            control_stamp_s=None if control_ is None else control_[0],
            odom_speed_mps=slam["odom_speed"] if slam is not None else None,
            odom_yaw_rate_rps=slam.get("odom_yaw_rate") if slam is not None else None,
            no_depth=raw is None, error=error_r,
        )
        if row["result"] == "bad_depth":
            state.setdefault("pocket_recording_errors", []).append(error_r)

    def check_near_contract() -> None:
        """Plan D8 S4a 적용 계약, before the first tick: the bounds file's condition (camera,
        mount with 1 mm, cadence, assets, SLAM noise seed), the drive terms tied to that
        bounds file, the approach cruise within the measured one, and the measurement
        runs' arguments, settings (drive, followers via their settings and profile,
        planners, LiDAR, chassis), detector and input files by content (Codex S4a-1
        review P1). Any difference refuses the start: near_field_contract."""
        from forklift_core.perception.pocket_detector import DetectorParams

        observed_ = {
            "video": args.video,
            "fps": args.fps,
            "mount": state.get("perception_mount"),
            "camera": state.get("pocket_camera"),
            "pallet_geometry": str(args.pallet_geometry),
            "forklift_urdf": str(args.forklift_urdf),
            "rear_axle_offset_m": args.rear_axle_offset_m,
            "slam_feedback": args.slam_feedback is not None,
            "seed": args.seed,
            "slam_noise_seed": args.slam_noise_seed,
            **{
                key_: hashlib.sha256(path_.read_bytes()).hexdigest()
                for key_, path_ in (
                    ("pallet_geometry_sha256", args.pallet_geometry),
                    ("forklift_urdf_sha256", args.forklift_urdf),
                    ("pallet_urdf_sha256", args.pallet_urdf),
                )
            },
        }
        problems_ = near_tracking.contract_problems(near["bounds_data"]["condition"], observed_)
        if args.pocket_check:
            # Plan D8 S4b: the pocket check copies the estimate, axis and certified volume when
            # it is built and would not follow a correction's paths -- until D5 v11.
            problems_.append("pocket_check: D5 with near tracking waits for D5 v11")
        if args.near_inject is not None and obstacle is not None and obstacle["layer"].known_enabled:
            # Plan D8 S4c-1: the known pallet's r_fix (10 mm) does not cover a 20-60 mm bias.
            problems_.append("near_inject: the known pallet would carry the injected bias")
        try:
            near["terms"] = near_tracking.DriveTerms.from_dict(
                json.loads(args.near_drive_terms.read_text()),
                state["near_tracking"]["bounds_sha256"],
                near["bounds_data"]["latency"],
            )
        except (ValueError, KeyError, TypeError) as exc:
            problems_.append(f"drive terms: {exc}")
        if near["terms"] is not None:
            if args.approach_straight_speed_mps > near["terms"].cruise_mps + 1e-12:
                problems_.append(
                    f"the approach straight speed {args.approach_straight_speed_mps} is above the "
                    f"measured cruise {near['terms'].cruise_mps}"
                )
            run_ = json.loads(record_json({
                "arguments": state["arguments"],
                **{name_: state.get(name_) for name_ in near_tracking.CONDITION_FIELDS},
            }))
            run_["detector_params"] = json.loads(record_json(asdict(DetectorParams.derived_for(args.pallet_prior_loaded))))
            observed_condition = near_tracking.run_condition(run_, Path.cwd())
            problems_ += near_tracking.condition_problems(
                near["terms"].condition, observed_condition, skip=("tracker_configs",)
            )
        state["near_tracking"]["contract_problems"] = problems_
        require(not problems_, "near_field_contract:" + "; ".join(problems_))

    def near_tracker_config():
        """Plan D8 S4a ① service side (M-a1 slice 6e): the D8c 3판 tracker's NearFieldConfig from the
        chassis and pallet geometry (the URDF read here) and the ⑥ detectors (front: near-field
        derived_for, roof: track_roof). The core's build_near_tracker checks and installs it."""
        from insertion_geometry import tracking_frame

        from forklift_core.perception.near_field_tracking import NearFieldConfig
        from forklift_core.perception.pocket_detector import (
            DetectorParams,
            detect_pockets,
        )
        from forklift_core.perception.roof_tracking import track_roof

        pallet_geo = args.pallet_geometry_loaded
        frame_geo = tracking_frame(args.forklift_urdf, pallet_geo)  # as ⑥ (tools/d8c_replay.py)
        prior_ = args.pallet_prior_loaded
        front_params = DetectorParams.derived_for(prior_, range_min_m=0.1)
        config = NearFieldConfig(
            handoff_start_front_x_m=frame_geo.fork_tip_x_m + 0.6 - 0.03,
            camera_xy_m=tuple(float(v) for v in np.asarray(perception_mount.translation_m)[:2]),
            rear_axle_x_m=-abs(args.rear_axle_offset_m),
            opening_width_m=pallet_geo.opening_width_m,
            pocket_spacing_m=2 * pallet_geo.opening_centre_offset_m,
        )

        def front_fn(scene_):
            return detect_pockets(scene_, prior_, front_params).observation

        def roof_fn(scene_, xy_, yaw_):
            return track_roof(scene_, pallet_geo, (xy_[0], xy_[1], pallet_geo.opening_centre_height_m),
                              yaw_).observation

        return SimpleNamespace(config=config, frame_geo=frame_geo, front_fn=front_fn, roof_fn=roof_fn)

    def near_tracker_create(service_, start_, episode_):
        """The NearFieldTracker service from the near capture (M-a1 slice 6e); under M-a0.5 recording, the
        NearTracking proxy (frames recorded by hash only)."""
        from forklift_core.perception.near_field_tracking import NearFieldTracker

        tracker_ = NearFieldTracker(
            service_.config, near["bounds"], start_["observation"], start_["pose"], start_["aligned_s"],
            front_fn=service_.front_fn, roof_fn=service_.roof_fn, frame_version=episode_,
        )
        if boundary is not None:
            tracker_ = RecordingProxy(tracker_, "NearTracking", boundary, hash_only_requests={"on_frame"})
        return tracker_

    def near_fork_geometry():
        """The fork blades and the carriage front corners, read from the URDF (M-a1 slice 6e)."""
        from insertion_geometry import read_carriage_front_corners_m

        return read_fork_blades_m(args.forklift_urdf), read_carriage_front_corners_m(args.forklift_urdf)

    def feed_near_frame(raw) -> str | None:
        """Plan D8 S4a ③: one carriage frame to the tracker -- the raw frame D5 reads,
        1 mm, placed at the control base of stamp - L in this hold, delivered at the read
        time. A frame whose depth does not normalise ("bad_depth", also withheld from
        D5), without a time, from the future, not newer than the last one, or not
        covered by the history is recorded and left out."""
        from forklift_core.perception.scene_dataset import SceneInput

        tracker_ = near["tracker"]
        now_s = world.current_time - initial_time
        row = {"read_s": now_s, "episode": near["episode"]}  # Plan D8 S4c-2: the hold it was read in
        if near.get("drop_latch") is not None:
            # Plan D8 S4c-1: an injected loss -- the frame is recorded (the recorder kept the raw
            # read) but never reaches the tracker, and tau's reference stays the last received.
            row.update(skipped="dropped", latch_s=near["drop_latch"]["time_s"])
            near["frames"].append(row)
            return "dropped"
        k_ = pocket["capture"].intrinsics
        try:
            depth_, _ = adapter.normalize_depth(np.asarray(raw).reshape(k_.height, k_.width))
        except (ValueError, TypeError) as exc:
            # Left out as a frame without a result: tau moves on and the budget or the
            # loss stops the truck; checked first, whatever its time, so the caller
            # never hands it to D5 (Codex S4a-1 reviews P2).
            row.update(skipped="bad_depth", error=repr(exc))
            near["frames"].append(row)
            return "bad_depth"
        frame = pocket_camera.get_current_frame()
        rendered = frame.get("rendering_time") if isinstance(frame, dict) else None
        try:
            stamp = float(rendered) - initial_time if rendered is not None else None
        except (TypeError, ValueError):
            stamp = None
        if stamp is None or not math.isfinite(stamp):
            row["skipped"] = "no_time"
        elif stamp > now_s + 1e-6:
            row["skipped"] = "future"
        elif near["last_fed_stamp"] is not None and stamp <= near["last_fed_stamp"]:
            row["skipped"] = "not_new"
        if "skipped" in row:
            near["frames"].append(row)
            return
        aligned = stamp - near["bounds"].align_latency_s
        pose = near["history"].base_at(aligned, near["episode"])
        row.update(stamp_s=stamp, aligned_s=aligned)
        if pose is None:
            row["skipped"] = "uncovered"
            near["frames"].append(row)
            return None
        lift_ = float(robot.get_joint_positions()[lift_index[0]])
        scene_ = SceneInput(
            rgb=np.zeros((k_.height, k_.width, 3), dtype=np.uint8),
            depth_m=near_tracking.quantize_depth_mm(depth_),
            intrinsics=k_,
            base_from_optical=replace(
                perception_mount,
                translation_m=np.asarray(perception_mount.translation_m, dtype=float) + np.array([0.0, 0.0, lift_]),
            ),
            stamp_ns=int(round(stamp * 1e9)),
            clock_domain="ros_sim",
            source_provenance="synthetic",
        )
        result_ = tracker_.on_frame(scene_, stamp, pose, now_s, near["episode"])
        near["last_fed_stamp"] = near["last_stamp"] = stamp
        latest_ = tracker_.latest
        row.update(
            pose=list(pose), pose_rows=near["history"].bracket_times(aligned, near["episode"]), lift_m=lift_,
            **asdict(result_), latest_source=latest_.source, latest_stamp_s=latest_.stamp_s,
            latest_aligned_s=latest_.aligned_s, latest_pose=list(latest_.pose),
            latest_held={k: list(v) if isinstance(v, tuple) else v for k, v in latest_.held.items()},
            latest_widths_m=[latest_.observation.left.width_m, latest_.observation.right.width_m],
            latest_bound=dict(latest_.bound), latest_frame_version=latest_.frame_version,
        )
        near["frames"].append(row)
        if result_.mismatch and not near.get("mismatch"):
            near["mismatch"] = True  # terminal in S4a (no retry before S4c)
            near["events"].append({"time_s": now_s, "event": "mismatch", "stamp_s": stamp})
        if result_.unbounded and not near.get("unbounded"):
            near["unbounded"] = True
            near["events"].append({"time_s": now_s, "event": "unbounded", "stamp_s": stamp})

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
        if recorder is not None:
            # Plan D8b: every read, labelled, before anything below can drop it; its
            # own processing, so the D5 path below keeps its order (Codex D8b impl P2).
            record_pocket_read(raw)
        if raw is None:
            return
        if near is not None and near["tracker"] is not None:
            # Plan D8 S4a: the same raw frame, before D5 reads it; a frame that does not
            # normalise is not given to D5 either -- its freshness runs out and it stops
            # the truck (Codex S4a-1 2nd review P2).
            if feed_near_frame(raw) == "bad_depth":
                return
        if check is None:
            pocket["frames_read"] += 1
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

    def grid_world(sc):
        """The scenario a grid plan sees: no ground-truth props (plan audit table)."""
        return replace(sc, props=()) if grid_planning else sc

    def docking_band(occupancy, target):
        """The perceived pallet's band a docking plan clears from the grid (plan D5): the
        estimate grown by 0.425 m along the insertion axis and 0.20 m across."""
        band_along = 0.025 + 0.40
        band_across = 0.20
        res_g = occupancy.resolution_m
        nx_g, ny_g = occupancy.occupied.shape
        gi, gj = np.meshgrid(np.arange(nx_g), np.arange(ny_g), indexing="ij")
        dx = occupancy.origin_x_m + (gi + 0.5) * res_g - target.x_m
        dy = occupancy.origin_y_m + (gj + 0.5) * res_g - target.y_m
        ct, st_ = math.cos(target.yaw_rad), math.sin(target.yaw_rad)
        return (np.abs(dx * ct + dy * st_) <= geometry.pallet_depth_m / 2 + band_along) & (
            np.abs(-dx * st_ + dy * ct) <= geometry.pallet_width_m / 2 + band_across
        )

    def grid_kwargs(kind=None, target=None, own=None, applied=None, band_pose=None, label=None) -> dict:
        """occupancy (and the pallet obstacle the plan may know) for a grid plan. band_pose (default
        the truck's rear) is where the return's delivered-pallet band is cleared around; label
        names the call in the plans record (obstacle recovery v4: the escape's grids)."""
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
            own_ = own if own is not None else rear
            # Several poses (obstacle recovery v4: an escape's end region) keep only the cells under all.
            own_now_ = (tuple(tuple(float(v) for v in p_) for p_ in own_) if np.ndim(own_) == 2
                        else tuple(float(v) for v in own_))
            loaded_now_ = bool(loaded)
        except NameError:  # the first plans, before the drive loop: nothing remembered under the truck
            own_now_, loaded_now_ = None, False
        mode_ = slam_ref["tracker"].mode if slam_ref is not None else None
        key_ = (stamp, obstacle["version"], applied_, mode_, loaded_now_,
                None if own_now_ is None else tuple(round(v, 4) for v in np.ravel(own_now_)))
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
                        # the last scan the bridge answered (an asynchronous link counts its own, real-time plan D9a)
                        replied_ = ((slam_ref["async_link"].last_reply_id if slam_ref.get("async_link") is not None
                                     else slam_ref["scan_id"] - 1) if slam_ref is not None else None)
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
                "last_replied_scan_id": ((slam_ref["async_link"].last_reply_id if slam_ref.get("async_link") is not None
                                          else slam_ref["scan_id"] - 1) if slam_ref is not None else None),
                "applied": list(applied_),
                "mode": mode_,
            }
            obstacle["plan_cache"] = (key_, occupancy, bundle)
        obstacle["plans"].append({"kind": label or kind, "stamp_s": stamp,
                                  "occupied_cells": int(occupancy.occupied.sum()), **bundle})
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
            fp_r = geometry.unloaded_footprint
            grow_r = planner_config.clearance_m + res_r
            band_poses_ = rear if band_pose is None else band_pose
            band_poses_ = [band_poses_] if np.ndim(band_poses_) == 1 else list(band_poses_)
            under_r = np.ones_like(band_r)
            for bp_ in band_poses_:  # several poses (an escape's end region): cleared only under all
                sx, sy, syaw = (float(v) for v in bp_)
                dxs = occupancy.origin_x_m + (gi_r + 0.5) * res_r - sx
                dys = occupancy.origin_y_m + (gj_r + 0.5) * res_r - sy
                cs, ss = math.cos(syaw), math.sin(syaw)
                us, vs = dxs * cs + dys * ss, -dxs * ss + dys * cs
                under_r &= ((us <= fp_r.front_m + grow_r) & (us >= -fp_r.rear_m - grow_r)
                            & (np.abs(vs) <= fp_r.half_width_m + grow_r))
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
            res_g = occupancy.resolution_m
            band = docking_band(occupancy, target)
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

    if boundary is not None:
        from forklift_core.mission.boundary_record import DictWatch as _GridWatch

        # The planning grid preparation (it reads the asynchronously replaced SLAM map file) is part of the
        # Planner port; what it installs in the core (pickup_estimate, the plan record) is recorded beside it.
        grid_kwargs = recorded_function(
            grid_kwargs, "Planner", "grid_kwargs", boundary,
            watch=_GridWatch(boundary.recorder, {"obstacle": obstacle}, keys={"obstacle": ["pickup_estimate", "plans"]},
                             append_only={"plans"}),
        )

    def correction_grid(first_target, target, rear_now) -> dict:
        """Plan D8 S4b ③: the grid a correction point's docking straights are checked on --
        the live grid, and for the docking straights that grid with only the band of the
        near capture's mission target B0 AND the new target's band cleared, so a correction
        never clears a cell the mission plan's check kept (Codex S4b design reviews)."""
        out_ = {"pickup_obstacle": Rectangle(target.x_m, target.y_m, geometry.pallet_depth_m,
                                             geometry.pallet_width_m, target.yaw_rad)}
        if not grid_planning:
            return out_
        from forklift_core.planning.grid_collision import OccupancyGrid as _OccC
        from forklift_core.planning.grid_collision import SplitOccupancy as _SplitC

        occupancy_ = grid_kwargs("correction", own=tuple(float(v) for v in rear_now))["occupancy"]
        both_ = docking_band(occupancy_, first_target) & docking_band(occupancy_, target)
        cleared_ = _OccC(occupancy_.origin_x_m, occupancy_.origin_y_m, occupancy_.resolution_m,
                         occupancy_.occupied & ~both_, version=occupancy_.version)
        obstacle["plans"][-1]["docking_band_cells"] = int((occupancy_.occupied & both_).sum())
        out_.update(occupancy=occupancy_, docking_occupancy=_SplitC(
            cleared_, cleared_, geometry.axle_to_fork_tip_m - geometry.carriage_limit_m))
        return out_

    def redraw_docking_traced(scenario_, rear_pose, target_, config_, *, start_rear, dry_configs, **kwargs):
        """redraw_docking with its dry runs built here, from start_rear at each of dry_configs; returns
        (plans or None, record, the dry runs' trajectories in call order). The callbacks run inside the
        Planner port, so what they produced is part of its response (M-a1 slice 6d: the near capture's
        stage 1 checks its correction points on them, and a replay does not run the callbacks)."""
        start_ = np.asarray(start_rear, dtype=float)
        trajectories_ = []

        def dry(config_d):
            def run_(st_):
                result_ = bicycle_rollout(st_.poses, st_.directions, st_.curvatures_inv_m, config_d, start_)
                trajectories_.append(result_.trajectory)
                return result_
            return run_

        plans_, record_ = redraw_docking(scenario_, rear_pose, target_, config_,
                                         dry_runs=tuple(dry(c_) for c_ in dry_configs), **kwargs)
        return plans_, record_, trajectories_

    if boundary is not None:
        from forklift_core.mission.core import rollout_callback_spec

        # Format 2: the dry runs this port makes are described as redraw_docking's closures describe themselves.
        redraw_docking_traced = recorded_function(
            redraw_docking_traced, "Planner", "redraw_docking_traced", boundary,
            callback_spec=lambda a_, k_: {"dry_runs": [rollout_callback_spec(c_, k_["start_rear"])
                                                       for c_ in k_["dry_configs"]]})

    def near_tracking_frame():
        """The near field's chassis/pallet frame (fork tip x, carriage limit), read from the URDF."""
        from insertion_geometry import tracking_frame

        return tracking_frame(args.forklift_urdf, args.pallet_geometry_loaded)

    # M-a1 replay plan R3 (format 2 only; format 1 keeps the M-a0 call sequence): the services are ports from the
    # hand-over on, so the initial plans, the bootstrap (SLAM link, followers, clock origin, reference scan) and
    # every tick head read them through the record.
    early_ports = boundary is not None and boundary.recorder.value_format == 2
    if early_ports:
        # The wheel lock reads and sets the drive gains on the articulation controller the robot hands back.
        robot = RecordingProxy(robot, "Vehicle", boundary, returns={"get_articulation_controller": "Controller"})
        pallet = RecordingProxy(pallet, "SimTruth", boundary)
        world = RecordingProxy(world, "Clock", boundary)
        cells.world, cells.robot = world, robot
        _BOUNDARY_RECORD["time"] = time
        globals()["time"] = RecordingProxy(time, "Clock", boundary)
        _BOUNDARY_RECORD["print"] = print
        globals()["print"] = _recording_print  # stdout lines as Sink.echo (plan 3.5)
        install_output_observers(args)  # the output files as Sink entries (plan 3.5)
        if obstacle is not None:  # the planning memory's SLAM map and revision are read by the plan bundles
            obstacle["layer"] = RecordingProxy(obstacle["layer"], "Obstacles", boundary,
                                               child_ports={"permission": "Permission", "memory": "Memory"})
        if recorder is not None:  # the pocket frames recorder: its phase, its closing statistics
            recorder = RecordingProxy(recorder, "PocketRecorder", boundary)
    state["phase"] = "planning"
    if boundary is not None:
        boundary.recorder.tick_begin(-1, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")  # the initial plans
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
        ms.next_candidate_index = 0
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
            ms.next_candidate_index = candidate_index + 1
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
        isaac.add_path_display(stage, paths["approach"], "Approach", (0.05, 0.45, 1.0))
        isaac.add_path_display(stage, paths["transport"], "Transport", (1.0, 0.65, 0.04))
        if "return_home" in paths:
            isaac.add_path_display(stage, paths["return_home"], "Return", (0.55, 0.2, 0.85))
        isaac.export_scene()
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

    if boundary is not None:
        boundary.recorder.tick_end(outputs={"setup": True, "paths": sorted(paths)}, branches=[])
    if early_ports:  # format 2: the bootstrap, from the initial plans to the first control tick
        boundary.recorder.tick_begin(-2, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
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
    cells.stepper = stepper
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
            # The four wheels' continuous joint angles (front left, front right, rear left, rear
            # right): the engine-reported rate is not their derivative under TGS (cause separation E).
            "wheel_angles_rad": [],
        }
        from forklift_core.localization.encoders import (
            AngleUnwrapper,
            TeamEncoderModel,
            encoder_model_spec,
            sensor_phases,
        )

        wheel_unwrap = [AngleUnwrapper() for _ in range(4)]
        encoder_spec = encoder_model_spec(args.encoder_model)
        encoder_model = None
        if encoder_spec is not None:
            encoder_model = TeamEncoderModel(encoder_spec, sensor_phases(args.slam_noise_seed))
            slam_log.update(wheel_counts=[], wheel_increments_rad=[], steering_measured_rad=[])
        state["encoder_model"] = {"name": "reported"} if encoder_model is None else encoder_model.record()
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

        if boundary is not None:
            dock = recorded_function(dock, "Planner", "dock", boundary)
        from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry

        # The known start (odom = map = world); the plan's v3 state machine.
        slam_start = (
            scenario.start_rear.x_m,
            scenario.start_rear.y_m,
            scenario.start_rear.yaw_rad,
        )
        slam = {
            "link": isaac.slam_link_client(
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
                    odometry_wheel_radius(args),
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
            # With a team-spec encoder model the odometry reads quantised counts and steering
            # instead of the reported rates plus Gaussian noise; the LiDAR range noise stays.
            "encoder_model": args.encoder_model,
            "wheel_steering_gaussian_noise": args.encoder_model == "reported" and args.slam_noise_seed is not None,
            "odometry_wheel_radius_m": odometry_wheel_radius(args),
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
    book_clock = {"t": None}  # the control loop's time; None before it starts
    trackers = TrackerBook(state.setdefault("leg_switches", []), state.setdefault("tracker_versions", []), book_clock, {
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
                # An observe or return cusp missed by 0.05-0.08 rad holds for the next-leg
                # judgement (cusp_heading_hold below; EPAL 6 fork spacing plan, fix A and A').
                cusp_heading_pending_rad=CUSP_HEADING_HOLD_RAD if name in CUSP_HEADING_HOLD_PHASES else None,
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
    })
    tracker_book["book"] = trackers

    from forklift_core.mission.core import apply_tracker_profile_to, record_tracker_configs_in

    apply_tracker_profile_to(trackers, args.tracker_profile)
    record_tracker_configs_in(state, trackers)
    ms.phase = "approach"
    if args.use_perception:
        ms.phase = "observe"
    ms.phase_started = 0.0
    initial_time = world.current_time
    simulation_started_wall = time.monotonic()
    ms.lift_command = 0.0
    steering_command = np.zeros(2)
    state["transitions"] = []
    state["samples"] = []
    inset_estimate = None
    inset_status = "팔레트 탐색 중"
    inset_detail = ""
    state["frames"] = 0
    state["phase"] = ms.phase

    def snapshot(label: str) -> None:
        if args.video:
            rgba = camera.get_rgba()
            if rgba is not None and rgba.shape == (720, 1280, 4):
                Image.fromarray(rgba.astype(np.uint8)).convert("RGB").save(
                    args.output / f"{label}.png"
                )

    from types import SimpleNamespace

    from forklift_core.mission.commands import Cancel as MissionCancel
    from forklift_core.mission.commands import Context as MissionContext
    from forklift_core.mission.commands import Pause as MissionPause
    from forklift_core.mission.commands import Resume as MissionResume
    from forklift_core.mission.commands import Submit as MissionSubmit
    from forklift_core.mission.targets import Pose as MissionTargetPose

    MISSION_COMMANDS = {"pause": MissionPause, "resume": MissionResume, "cancel": MissionCancel}
    from forklift_core.mission.core import MissionCore

    # M-a1 slice 1: the phase transition and the known-pallet bookkeeping it carries moved to the
    # mission core; what has not moved yet is reached through hooks.
    def save_capture(attempt_number, scene_input) -> None:
        """M-a1 slice 4b-i: the capture's diagnostic image and depth dumps, the runner's statements unchanged
        (a Sink the mission core calls where they stood)."""
        # Diagnostic dump for offline root-cause analysis; not part
        # of the perception contract itself.
        save_image(Image.fromarray(scene_input.rgb),
            args.output / f"perception_capture_{attempt_number}_rgb.png")
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
            save_image(Image.fromarray((normalized * 255).astype(np.uint8)),
                args.output
                / f"perception_capture_{attempt_number}_depth_vis.png")

    def record_capture_evaluation(target_pickup, attempt, attempt_number, capture_diagnostics, step, t) -> None:
        """M-a1 slice 4b-iv: the accepted capture's evaluation records (truth), the runner's statements unchanged,
        called by the mission core where they stood -- a plain call, so each robot/pallet read is recorded as
        before (Codex 4b design)."""
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

    def set_inset_estimate(observation_, capture_pose_, site_, scene_input_, attempt_number_) -> None:
        # The video inset's estimate (a Sink the core reaches through a hook); write_video_frame reads it.
        nonlocal inset_estimate
        if args.camera_inset or args.robot_camera:
            inset_estimate = inset.InsetEstimate(
                observation=observation_,
                capture_pose=capture_pose_,
                pallet_site=site_,
                intrinsics=scene_input_.intrinsics,
                base_from_optical=scene_input_.base_from_optical,
                attempt_number=attempt_number_,
            )

    def set_inset_status(status: str) -> None:
        nonlocal inset_status  # the video overlay's first line (a Sink)
        inset_status = status

    def set_inset(status: str, detail: str) -> None:
        # The video overlay's two lines, read by write_video_frame (a Sink the core reaches through a hook).
        nonlocal inset_status, inset_detail
        inset_status, inset_detail = status, detail

    def set_rear(value) -> None:
        # The mission core rebinds the control pose inside a call (a release); the closures it calls next
        # (grid_kwargs's own pose, dump_tracking, the docking) read run()'s rear, so it is set here at once.
        nonlocal rear
        rear = value

    def far_recognition_hook(scene_input_, base_, yaw_, reference_):
        """The far observation's decision (plan v2, 3): the stop frame with its instance ids, the control pose for
        the instruction, odom<-base for the tracker, truth only for the record's error."""
        truth_b_, truth_q_ = robot.get_world_pose()
        pallet_p_, pallet_q_ = pallet.get_world_pose()
        observation_, record_ = far_runtime.stop(
            scene_input_, perception_capture.state.instance_ids, initial_time_s=initial_time,
            control_base=(float(base_[0]), float(base_[1]), float(yaw_)), odom_base=slam_odom_base(),
            truth_base=(float(truth_b_[0]), float(truth_b_[1]), yaw_and_tilt(truth_q_)[0]),
            truth_pallet=(float(pallet_p_[0]), float(pallet_p_[1]), yaw_and_tilt(pallet_q_)[0]),
            detector_observation=reference_)
        state.setdefault("recognition", {}).update(far_runtime.summary())
        return observation_, record_

    core = MissionCore(
        ms, state=state, obstacle=obstacle, slam=slam, pocket=pocket, paths=paths, trackers=trackers,
        geometry=geometry, settings=settings, pallet_geometry=args.pallet_geometry_loaded,
        hooks=SimpleNamespace(
            echo=lambda text: print(text, flush=True), record_json=record_json, snapshot=snapshot, require=require,
            slam_odom_base=slam_odom_base,
            slam_error=slam_error, fork_blades=lambda: read_fork_blades_m(args.forklift_urdf),
            # M-a1 slice 2. Truth (the plan's SimTruth list): the loop's own samples, read when called.
            insertion_measured=lambda: insertion_geometry.insertion_measures(
                base, q, float(robot.get_joint_positions()[lift_index[0]]), ppos, pq, geometry.pallet_depth_m),
            truth_relative_yaw=lambda: yaw_and_tilt(q)[0] - yaw_and_tilt(pq)[0],
            pallet_z=lambda: ppos[2],
            lift_position=lambda: robot.get_joint_positions()[lift_index[0]],
            truth_rear=lambda: truth_rear,
            # defined further down run()
            slam_rear=lambda t_: slam_rear(t_),
            # M-a1 slice 3a: ports read when called (the recorded planners and wall clock when recording).
            wall=lambda: time.monotonic(), grid_world=lambda sc: grid_world(sc),
            grid_kwargs=lambda *a, **k: grid_kwargs(*a, **k),
            plan_transport_leg=lambda *a, **k: plan_transport_leg(*a, **k),
            plan_return_leg=lambda *a, **k: plan_return_leg(*a, **k),
            path_record=path_record,
            write_paths=lambda: (args.output / "paths.json").write_text(record_json(state["paths"], indent=2) + "\n"),
            # M-a1 slice 3b: truth (SimTruth list) and the permission's diagnostics dump.
            truth_speed=lambda: truth_speed, truth_velocity=lambda: velocity,
            save_npz=lambda name, **arrays: np.savez_compressed(args.output / name, **arrays),
            # M-a1 slice 3c: the blocked replan's releases, planners and recovery helpers (read when called).
            plan_observation_leg=lambda *a, **k: plan_observation_leg(*a, **k),
            plan_transport=lambda *a, **k: plan_transport(*a, **k),
            final_straight_prefix=lambda *a: final_straight_prefix(*a),
            same_path=same_path, path_stats=path_stats,
            # M-a1 slice 4a: the follower rules' helpers and the cusp heading holds (built further down run()).
            dump_tracking=lambda reason, tracking_: dump_tracking(reason, tracking_),
            cusp_holds=lambda: cusp_holds,
            now_s=lambda: world.current_time - initial_time,
            show_path=lambda path, label, color: isaac.add_path_display(stage, path, label, color),
            set_rear=set_rear,
            # M-a1 slice 4a-iii: the cusp helpers (built further down run()) and truth.
            cusp_monitor=lambda: cusp_monitor, cusp_heading_motion=lambda v: cusp_heading_motion(v),
            truth_yaw_rate=lambda: robot.get_angular_velocity()[2],
            # M-a1 slice 6a: the cusp rules' ports -- the loop's load state and control speed (read when
            # called), the held steering and the standing test (Vehicle reads; truth without SLAM).
            loaded=lambda: loaded, signed_speed=lambda: signed_speed, held_kappa=lambda: held_kappa(),
            cusp_standing=lambda: cusp_standing(),
            # M-a1 slice 6b: the docking's ports -- the loop's control and true pose, the world clock and stage,
            # the truth obstacles its dry run checks (SimTruth), the recorded planners and the LiDAR geometry.
            rear=lambda: rear, world=lambda: world, stage=lambda: stage,
            truth_obstacles=lambda: obstacles, dock=lambda *a, **k: dock(*a, **k),
            plan_docking_reapproach=lambda *a, **k: plan_docking_reapproach(*a, **k),
            add_path_display=lambda *a: isaac.add_path_display(*a),
            laser_mount=lambda: laser_mount, beam_angles=lambda: beam_angles,
            time_module=lambda: time,
            # M-a1 slice 4b: the video overlay's text (a Sink), the tracker config record.
            set_inset=set_inset,
            # M-a1 slice 4b-iv: the evaluation records, the planning target (truth under oracle), Sinks.
            record_capture_evaluation=record_capture_evaluation, set_inset_estimate=set_inset_estimate,
            planning_target=lambda target_: G2.planning_target(args.planning_target, target_, scenario.pickup),
            export_scene=lambda: isaac.export_scene(),
            # M-a1 slice 4b-v: the near capture's straight, its acceptance and what it installs.
            set_inset_status=set_inset_status,
            straight_from_pose=lambda *a, **k: straight_from_pose(*a, **k),
            bicycle_rollout=lambda *a, **k: bicycle_rollout(*a, **k),
            build_pocket_check=lambda *a: build_pocket_check(*a), near_tracker_config=near_tracker_config,
            near_tracker_create=near_tracker_create, near_fork_geometry=near_fork_geometry,
            # M-a1 slice 4b-i: the capture port (recorded when recording; built further down run()), the detector
            # port and the capture's dumps.
            perception_capture=lambda: perception_capture, detect_pockets=lambda *a: detect_pockets(*a),
            save_capture=save_capture,
            # The far recognition's decision (--recognition far), None otherwise.
            far_recognition=far_recognition_hook if far_runtime is not None else None,
            # M-a1 slice 5: the release at a stop, the measured steering.
            steering_position=lambda: robot.get_joint_positions()[steers],
            # M-a1 slice 6d: the near field's mission side (near_tick, the correction redraws, installs, retry).
            steering_command=lambda: steering_command, correction_grid=lambda *a: correction_grid(*a),
            redraw_docking=lambda *a, **k: redraw_docking(*a, **k),
            redraw_docking_traced=lambda *a, **k: redraw_docking_traced(*a, **k), tracking_frame=near_tracking_frame,
        ),
        near=near,
        options=SimpleNamespace(near_tracking_stop_at=args.near_tracking_stop_at, d7_docking=args.d7_docking,
                                dwells=dwells, safety_stops=safety_stops, home_rear=scenario.start_rear,
                                grid_planning=grid_planning, return_turnaround=args.return_turnaround,
                                planner_config=planner_config, travel_config=travel_config, scenario=scenario,
                                return_to_pose=return_to_pose, obstacle_act=args.obstacle_act,
                                pocket_check=args.pocket_check, mid_still_decel_mps2=MID_STILL_DECEL_MPS2,
                                pickup_bounds=pickup_bounds, mission_stages=mission_stages,
                                observation_waypoints=args.observation_waypoints, backoff_m=BACKOFF_M,
                                backoff_speed_mps=BACKOFF_SPEED_MPS, backoff_per_leg=BACKOFF_PER_LEG,
                                escape_per_leg=ESCAPE_PER_LEG, cusp_heading_hold_phases=CUSP_HEADING_HOLD_PHASES,
                                max_cusp_replans=max_cusp_replans, cusp_stop_needed=cusp_stop_needed,
                                cusp_stop_yaw_rate_radps=cusp_stop_yaw_rate_radps,
                                cusp_heading_hold_position_m=CUSP_HEADING_HOLD_POSITION_M,
                                cusp_heading_hold_rad=CUSP_HEADING_HOLD_RAD,
                                observe_replans_per_leg=OBSERVE_REPLANS_PER_LEG, dwell_d_end_m=dwell_d_end_m,
                                observe_arrival_m=OBSERVE_ARRIVAL_M, observe_arrival_yaw_rad=OBSERVE_ARRIVAL_YAW_RAD,
                                speeds=speeds, repeat_captures=args.repeat_captures,
                                repeat_at_attempt=args.repeat_at_attempt, extra_views=args.extra_views,
                                near_inject=args.near_inject, near_inject_lateral_m=args.near_inject_lateral_m,
                                near_inject_yaw_rad=args.near_inject_yaw_rad, initial_time=initial_time,
                                adapter=adapter if args.use_perception else None, near_tracking=near_tracking,
                                near_capture_box_yaw_rad=args.near_capture_box_yaw_rad,
                                approach_straight_speed_mps=args.approach_straight_speed_mps,
                                near_capture_redraw=args.near_capture_redraw, G2=G2, yaw_and_tilt=yaw_and_tilt,
                                near_drop_frames_from_m=args.near_drop_frames_from_m,
                                planning_target=args.planning_target,
                                perception_max_attempts=args.perception_max_attempts,
                                rear_axle_offset_m=args.rear_axle_offset_m,
                                pallet_prior_loaded=args.pallet_prior_loaded,
                                depth_quantize_mm=args.depth_quantize_mm, drive_geometry=drive_geometry,
                                command_accel_mps2=COMMAND_ACCEL_MPS2, dt=dt, pickup_zone=pickup_zone,
                                tracker_profile=args.tracker_profile, DOCKING_RETRY=DOCKING_RETRY,
                                ROUND2_KEEP_M=ROUND2_KEEP_M),
    )
    # M-a3 (docs/plans/2026-10-16-m-a3-mission-commands.md E1): the batch run submits its scenario's mission itself,
    # outside the loop -- the default path is a run with this one submit and no command in any tick.
    # The schedule of a --mission-commands run (plan E2); None without one, and then nothing below applies a command.
    mission_schedule = list(args.mission_schedule) if getattr(args, "mission_schedule", None) is not None else None
    mission_stop_kinds = list(getattr(args, "mission_stop_kinds", None) or [])
    # M-a3 (plan E8): the safety state for the arbiter, one JSON line per control tick, written through an open file
    # (not the observed Path writers -- no new recorded effect) and only by a run with a schedule.
    safety_file = (Path(args.output) / "mission_safety.jsonl").open("w") if mission_schedule is not None else None

    def stop_kind_at(time_s: float) -> str:
        """M-a3 (plan E7): the stop kind the schedule gives at loop time ``time_s`` (the arbiter's, later)."""
        for start_, end_, kind_ in mission_stop_kinds:
            if start_ <= time_s < end_:
                return kind_
        return "none"

    # M-a4 (plan D7): the scenario's pallet and destination are the targets list; the submit names them.
    core.targets.add_pallet("P-1", Path(args.pallet_geometry).stem, MissionTargetPose(
        float(scenario.pickup.x_m), float(scenario.pickup.y_m), float(scenario.pickup.yaw_rad)),
        depth_m=float(args.pallet_geometry_loaded.overall_depth_m),  # the console draws it (C3)
        width_m=float(args.pallet_geometry_loaded.overall_width_m))
    core.targets.add_destination("D-1", MissionTargetPose(
        float(scenario.destination.x_m), float(scenario.destination.y_m), float(scenario.destination.yaw_rad)))
    if not getattr(args, "live", False):  # M-c: live, the operator submits (a record from before M-c has no flag)
        core.commands.queue(MissionSubmit("P-1", "D-1", core.targets.revision))
        core.commands.apply(0.0)
        core.commands.start()
    transition = core.transition

    def held_kappa() -> float:
        """The curvature the wheels actually hold, from the steering joints."""
        steer_now_ = robot.get_joint_positions()[steers]
        half_track_ = args.drive_geometry.track_m / 2
        wheelbase_ = args.drive_geometry.wheelbase_m
        return float(
            np.mean(
                [
                    math.tan(steer_now_[0]) / (wheelbase_ + math.tan(steer_now_[0]) * half_track_),
                    math.tan(steer_now_[1]) / (wheelbase_ - math.tan(steer_now_[1]) * half_track_),
                ]
            )
        )

    # Plan cusp position stall v7 (docs/plans/2026-10-04-lidar-obstacle-map.md): the stall machine
    # (forklift_core.control.cusp_stall) and the checks it needs from this world.
    cusp_monitor = CUSP_STALL.CuspStallMonitor(state["cusp_stalls"])

    def cusp_standing() -> bool:
        """The stop decision the cusp rules use: SLAM's stop detector, or the planar speed and yaw rate."""
        if slam is not None:
            return bool(slam["stop_now"])
        return (
            float(np.linalg.norm(robot.get_linear_velocity()[:2])) <= trackers[ms.phase].config.stop_speed_mps
            and abs(float(robot.get_angular_velocity()[2])) <= cusp_stop_yaw_rate_radps
        )

    def cusp_heading_motion(signed_speed_now) -> dict:
        """Speed, steering and pose at this tick, for the cusp heading hold records (fix A): the
        speed and rear pose control used (under SLAM the tracker speed and the estimate) beside the
        measured speed and the true rear pose (Codex fix A implementation review 2 P3)."""
        return {"control_speed_mps": float(signed_speed_now),
                "measured_speed_mps": float(np.dot(robot.get_linear_velocity()[:2], forward)),
                "truth_rear": np.asarray(truth_rear, dtype=float).tolist(),
                "steering_command_rad": np.asarray(steering_command, dtype=float).tolist(),
                "steering_actual_rad": np.asarray(robot.get_joint_positions()[steers], dtype=float).tolist()}

    cusp_holds = CUSP_HOLD.CuspHeadingHolds(state)  # fix A's hold records

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
        ms.last_tracking = None

        def dump_tracking(reason: str, tracking) -> None:
            # The run aborts right after this, before the next sample; keep what
            # the G2b diagnosis needs (closeout plan, G2b ㉮).
            tracker = trackers[ms.phase]
            path = paths[ms.phase]
            directions = np.asarray(path.directions)
            cusps = [
                int(i)
                for i in range(1, len(directions) - 1)
                if directions[i + 1] != directions[i]
            ]
            final = int(len(path.poses) - 1)
            record = {
                "reason": reason,
                "phase": ms.phase,
                "time_s": t,
                "phase_started_s": ms.phase_started,
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
                        phase=ms.phase,
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
                    phase=ms.phase,
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
                            "phase": ms.phase,
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

        silence_ = getattr(args, "live_test_scan_silence", None)  # a recorded run's args predate it
        silence_ = tuple(float(v) for v in silence_.split(":")) if silence_ else None

        def scan_silenced(stamp: float) -> bool:
            return silence_ is not None and silence_[0] <= stamp < silence_[1]

        def obstacle_scan(stamp: float, base, q, shared_raw: dict | None = None) -> None:
            """Cast the obstacle LiDARs, feed the grid, refresh the path check.

            shared_raw: the shared scan's rays and noisy ranges (plan v10, one
            LiDAR feeding SLAM and the layer) -- no second cast, no second draw.
            """
            layer = obstacle["layer"]
            loaded_now = ms.phase in ("lift", "extract", "transport", "lower")
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
            if ms.phase in trackers:
                ahead, leg_direction = trackers[ms.phase].leg_ahead()
            else:
                ahead = np.array([current])
            check = layer.refresh(
                stamp, correction, obstacle["version"], current_pose=current, path_ahead=ahead, loaded=loaded_now,
                direction=leg_direction,
            )
            obstacle["scans"].append(
                {
                    "stamp_s": float(stamp),
                    "phase": ms.phase,
                    "verified_m": check.verified_m,
                    "blocked": check.blocked,
                    "own_beams": {n: int(np.count_nonzero(r[2])) for n, r in raw.items()},
                    "cast_wall_s": cast_s,
                    "total_wall_s": time.monotonic() - started,
                    "version": obstacle["version"],
                }
            )

        def record_truth_tick(stamp_now: float, rendered: bool, now_base, now_q) -> None:
            """Plan D8b: the physics truth of one tick. A pallet read that fails is
            recorded as the reason and raised; the camera prim pose is diagnosis only."""
            try:
                pallet_pos_, pallet_q_ = pallet.get_world_pose()
            except Exception as exc:
                recorder.tick_error = f"pallet pose at {stamp_now:.4f} s: {exc!r}"
                raise
            try:  # what the renderer may read; never the truth
                camera_pos_, camera_q_ = pocket_camera.get_world_pose(camera_axes="world")
                camera_pose_ = pocket_recorder.pose7(camera_pos_, camera_q_)
            except Exception as exc:  # noqa: BLE001
                recorder.camera_pose_failed(exc)
                camera_pose_ = None
            recorder.record_tick(
                stamp_s=stamp_now, rendered=rendered, base_pose=pocket_recorder.pose7(now_base, now_q),
                lift_m=float(robot.get_joint_positions()[lift_index[0]]),
                pallet_pose=pocket_recorder.pose7(pallet_pos_, pallet_q_), camera_prim_pose=camera_pose_,
                stop_now=slam["stop_now"] if slam is not None else None,
                odom_speed_mps=slam["odom_speed"] if slam is not None else None,
                odom_yaw_rate_rps=slam.get("odom_yaw_rate") if slam is not None else None,
                applied_speed_mps=state.get("applied_command", (None, None))[0],
                applied_steering_rad=state.get("applied_command", (None, None))[1],
            )

        def far_drive_frame(tick_: int) -> None:
            """One driving frame for the far recognition: render-metadata time, pixels and instance ids."""
            frame_ = perception_camera.get_current_frame()
            semantic_ = frr.decode_mission_semantic(frame_.get("semantic_segmentation"))
            rgba_ = np.array(perception_camera.get_rgba(), copy=True)
            raw_ = np.array(perception_camera.get_depth(), copy=True)
            if raw_.ndim == 3 and raw_.shape[-1] == 1:
                raw_ = np.squeeze(raw_, axis=-1)
            ready_ = raw_.shape == (far_intrinsics.height, far_intrinsics.width) and rgba_.ndim == 3
            depth_ = adapter.normalize_depth(raw_)[0] if ready_ else None  # the camera's first renders are empty
            applied_ = slam["tracker"].applied if slam is not None else None
            far_runtime.drive_frame(
                tick_, float(frame_["rendering_time"]) - initial_time, rgba_[:, :, :3] if ready_ else None, depth_,
                semantic_.instance_ids if semantic_.ready else None, far_intrinsics, perception_mount,
                tuple(float(v) for v in applied_[0]) if applied_ is not None else (0.0, 0.0, 0.0))

        def step_world(render: bool) -> None:
            """One physics step and everything that must see every step
            (SLAM plan v3.1): encoders, odometry, the 10 Hz scan by physics
            tick (not loop step) and the SLAM lockstep. Capture steps too."""
            tick = stepper["tick"]
            stepper["tick"] = tick + 1
            frame_due = args.video and tick % fps_divisor == 0
            if mission_stop_kinds and stop_kind_at(world.current_time - initial_time) == "preempt":
                # M-a3 (plan E7): a preempt stops the wheels at this physics step, a capture's steps included; the
                # steering stays where it is. The applied command, its slew and the stop detector's command say so.
                robot.apply_action(isaac.action(joint_velocities=np.zeros(len(wheels)), joint_indices=wheels))
                state["applied_command"] = (0.0, list((state.get("applied_command") or (0.0, []))[1]))
                state["command_speed"] = 0.0
                # Seen by the next control tick even if it ended inside a capture's steps (watched: recorded and
                # restored by a replay, which does not step).
                state["preempt_seen_s"] = float(world.current_time - initial_time)
                if slam is not None:
                    slam["last_command"] = 0.0
            # The far recognition's driving frames: every frame_every steps of a far observe leg (not the near
            # capture's leg, not inside a stationary capture), rendered here and read after the odometry below.
            far_due = far_runtime is not None and far_runtime.due(
                tick, observing=ms.phase == "observe" and state.get("near_capture", {}).get("status") != "pending")
            world.step(render=render or frame_due or far_due)
            stamp_now = world.current_time - initial_time
            if args.tyre_contact_log and tick % 12 == 0:
                # New plant candidate V1 (loaded) / V5: each tyre's floor contact every 0.1 s.
                forces_ = tyre_floor_forces(TYRE_LINKS)
                state.setdefault("tyre_contact_rows", []).append(
                    [stamp_now, *[v for row_ in forces_ for v in row_]])
            if slam_log is None:
                if frame_due:
                    write_video_frame(stamp_now)
                return
            now_base, now_q = robot.get_world_pose()
            rates_true = robot.get_joint_velocities()[wheels]
            joint_q = np.asarray(robot.get_joint_positions(), dtype=float)
            steer_true = joint_q[steers]
            wheel_angles = [unwrap_.update(joint_q[i_]) for unwrap_, i_ in zip(wheel_unwrap, wheels, strict=True)]
            slam_log["joint_stamps_s"].append(stamp_now)
            slam_log["wheel_rates_rad_s"].append(rates_true)
            slam_log["steering_rad"].append(steer_true)
            slam_log["base_pose_world"].append(np.concatenate((now_base, now_q)))
            slam_log["wheel_angles_rad"].append(wheel_angles)
            if encoder_model is not None:
                # The team's encoders, read from the joint angles this tick (row = the interval ending now).
                increments_, steering_measured_, counts_ = encoder_model.read(wheel_angles[2:4], steer_true)
                slam_log["wheel_counts"].append(counts_)
                slam_log["wheel_increments_rad"].append(increments_)
                slam_log["steering_measured_rad"].append(steering_measured_)
            if slam is not None and encoder_model is not None:
                previous_yaw = slam["odom_rear"][2]
                slam["odom_rear"] = slam["odometry"].update_increments(stamp_now, increments_, steering_measured_)
                slam["odom_speed"] = slam["odometry"].interval_speed
            elif slam is not None:
                rates = slam["noise"].wheel_rates(rates_true[2:4])
                angles = slam["noise"].steering(steer_true)
                previous_yaw = slam["odom_rear"][2]
                slam["odom_rear"] = slam["odometry"].update(stamp_now, rates, angles)
                slam["odom_speed"] = float(np.mean(rates)) * odometry_wheel_radius(args)
            if slam is not None:
                slam["odom_yaw_rate"] = (slam["odom_rear"][2] - previous_yaw) * 120.0
                if getattr(args, "live", False):
                    # M-c (plan C2): the vehicle time the odometry sample was made, before the SLAM exchange below
                    # waits (live only: a batch run's clock reads are recorded and replayed).
                    slam["odom_mono"] = time.monotonic()
                slam["stop_now"] = slam["stop"].update(
                    commanded_speed=slam.get("last_command", 0.0),
                    speed=slam["odom_speed"],
                    yaw_rate=slam["odom_yaw_rate"],
                )
                # Every tracker in this runner stops at 0.012 m/s; its arrival
                # and gear-change checks wait for the noise-aware detector.
                slam["tracker_speed"] = slam["stop"].tracker_speed(0.012)
            if far_runtime is not None and ms.phase == "observe":
                # Each step's odom and truth (the driving frames' poses and the pixel-lag fit's input).
                pallet_now_, pallet_q_now_ = pallet.get_world_pose()
                far_runtime.record_step(tick, stamp_now, slam_odom_base(),
                                        (now_base[0], now_base[1], yaw_and_tilt(now_q)[0]),
                                        (pallet_now_[0], pallet_now_[1], yaw_and_tilt(pallet_q_now_)[0]))
                if far_due:
                    far_drive_frame(tick)
            if recorder is not None:
                # Plan D8b truth, after this tick's encoders and odometry so a failure
                # here cannot drop them (Codex D8b impl P2).
                record_truth_tick(stamp_now, bool(render or frame_due), now_base, now_q)
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
                loaded_scan = ms.phase in ("lift", "extract", "transport", "lower")
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
            if obstacle is not None and slam is None and not scan_silenced(stamp_now):
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
            if slam.get("async_link") is not None:
                # Real-time plan D9a: the scan and what it was taken with, queued; its reply is applied at a
                # later tick's start (MissionCore.slam_replies). The obstacle scan runs now, on the correction
                # applied now.
                truth_yaw_, _ = yaw_and_tilt(now_q)
                try:
                    slam["async_link"].enqueue(
                        link.Scan(slam["scan_id"], float(stamp_now), slam_odom_base(), sent.astype(np.float32),
                                  scan_pattern.angle_min_rad, scan_pattern.angle_increment_rad,
                                  scan_pattern.range_min_m, scan_pattern.range_max_m),
                        {"scan_id": slam["scan_id"], "stamp_s": float(stamp_now), "phase": ms.phase,
                         "odom_base": list(slam_odom_base()),
                         "truth_base": [float(now_base[0]), float(now_base[1]), truth_yaw_],
                         "scan_correction": list(slam["tracker"].applied[0])
                         if slam["tracker"].applied is not None else None,
                         "queued_mono": time.monotonic()})
                except link.SlamLinkFailure as exc:
                    raise SessionFailure(f"slam_link_failed: {exc}") from exc
                slam["scan_id"] += 1
                if obstacle is not None and not scan_silenced(stamp_now):
                    obstacle_scan(stamp_now, now_base, now_q, shared_raw)
                return
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
                        "phase": ms.phase,
                        "status": "link_failure",
                        "error": str(exc),
                    }
                )
                if getattr(args, "live", False):
                    raise SessionFailure(f"slam_link_failed: {exc}") from exc  # the live warm-up's lockstep exchange
                require(False, f"slam_link_failed: {exc}")
            slam["tracker"].receive(
                reply.scan_id, reply.stamp_s, reply.status, reply.map_from_odom
            )
            truth_yaw, _ = yaw_and_tilt(now_q)
            slam["records"].append(
                {
                    "scan_id": slam["scan_id"],
                    "stamp_s": float(stamp_now),
                    "phase": ms.phase,
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
            if obstacle is not None and not scan_silenced(stamp_now):
                # After this scan's reply: the correction control applies now.
                obstacle_scan(stamp_now, now_base, now_q, shared_raw)

        gc_seen_: list = []  # real-time plan D9d: garbage collections timed in a live run, folded in at a tick's start
        live = None
        if getattr(args, "live", False):
            # M-c (docs/plans/2026-10-17-m-c-isaac-live-runner.md): the arbiter, its console server and the pacing.
            from forklift_core.control.path_tracking import ackermann_command as live_ackermann
            from forklift_core.safety.arbiter import ArbiterConfig as LiveArbiterConfig
            from forklift_core.safety.teleop_drive import BundlePermit as LiveBundlePermit
            from forklift_core.safety.teleop_drive import arc_permit as live_arc_permit
            from forklift_core.control.drive_permission import DrivePermission as LiveDrivePermission
            repo_root_ = str(Path(__file__).resolve().parents[2])  # console/ lives at the repository's root
            if repo_root_ not in sys.path:
                sys.path.insert(0, repo_root_)
            from live_runtime import Hooks as LiveHooks, LiveRuntime, live_camera_model  # siblings, as the rest

            def live_zero_wheels() -> None:
                # As a scheduled preempt (M-a3 E7): the wheels stop at this step, the steering stays; the applied
                # command, its slew and the stop detector's command say so.
                robot.apply_action(isaac.action(joint_velocities=np.zeros(len(wheels)), joint_indices=wheels))
                state["applied_command"] = (0.0, list((state.get("applied_command") or (0.0, []))[1]))
                state["command_speed"] = 0.0
                if slam is not None:
                    slam["last_command"] = 0.0

            def live_hold_lift(position: float) -> None:
                robot.apply_action(isaac.action(joint_positions=np.array([position]), joint_indices=lift_index))

            live_cam_ = _bound.get("perception_camera")  # unbound without --use-perception: no video

            def live_camera():
                if live_cam_ is None:
                    return None, None
                rgba_ = live_cam_.get_rgba()
                frame_ = live_cam_.get_current_frame()
                rendered_ = frame_.get("rendering_time") if isinstance(frame_, dict) else None
                if rgba_ is None or rgba_.ndim != 3 or rgba_.size == 0 or rendered_ is None:
                    return None, None
                rgb_ = np.asarray(rgba_[:, :, :3], dtype=np.uint8)
                while rgb_.shape[1] > 640:
                    rgb_ = rgb_[::2, ::2]
                return rgb_, float(rendered_)

            def live_grid():
                if obstacle is None or obstacle["layer"].snapshot is None:
                    return None
                snap_ = obstacle["layer"].snapshot  # the local window the permission reads
                cells_ = np.ascontiguousarray(snap_.state.T).astype(np.uint8)  # rows of y, as the console reads
                return {"cells": cells_.tobytes(), "width": int(snap_.state.shape[0]),
                        "height": int(snap_.state.shape[1]), "resolution_m": float(snap_.resolution_m),
                        "origin_m": (float(snap_.origin_x_m), float(snap_.origin_y_m)), "frame": "map",
                        "correction_version": int(obstacle["version"])}

            def live_scan():
                if slam_log is None or not slam_log["scan_ranges_m"]:
                    return None
                rear_ = live_state.get("rear")
                if rear_ is None:
                    return None
                ahead_ = abs(args.rear_axle_offset_m) + float(laser_mount.xyz_m[0])
                c_, s_ = math.cos(float(rear_[2])), math.sin(float(rear_[2]))
                pose_ = {"x_m": float(rear_[0]) + c_ * ahead_ - s_ * float(laser_mount.xyz_m[1]),
                         "y_m": float(rear_[1]) + s_ * ahead_ + c_ * float(laser_mount.xyz_m[1]),
                         "yaw_rad": float(rear_[2]) + float(laser_mount.yaw_rad)}
                return {"ranges": slam_log["scan_ranges_m"][-1].tolist(), "angle_min_rad": scan_pattern.angle_min_rad,
                        "angle_increment_rad": scan_pattern.angle_increment_rad,
                        "range_max_m": scan_pattern.range_max_m, "pose": pose_, "frame": "map",
                        "age_s": float(world.current_time - initial_time) - float(slam_log["scan_stamps_s"][-1])}

            freeze_ = None
            if args.live_test_camera_freeze:
                a_, b_ = (float(v) for v in args.live_test_camera_freeze.split(":"))
                freeze_ = (a_, b_)
            live_state = {"rear": None, "snapshot": None, "snapshot_key": None}
            # Real-time plan D7: at most this many steps in a row made up after a late one (0: never, the M-c rule).
            live_catchup_ = int(os.environ.get("FORKLIFT_LIVE_CATCHUP_STEPS", "0"))
            if not 0 <= live_catchup_ <= 12:
                raise ValueError("FORKLIFT_LIVE_CATCHUP_STEPS must be 0..12")
            live_perm = LiveBundlePermit(obstacle["layer"].permission.config) if obstacle is not None else None
            live = LiveRuntime(
                core, LiveHooks(zero_wheels=live_zero_wheels, hold_lift=live_hold_lift,
                                lift_position=lambda: float(robot.get_joint_positions()[lift_index[0]]),
                                camera=live_camera, grid=live_grid, scan=live_scan),
                output=Path(args.output), port_file=args.live_port_file, dt=1 / 120, factor=args.live_factor,
                # The human and network time limits scaled by the declared factor f (plan C4a): the truck in the sim
                # moves f metres of sim for a wall metre's worth, so a wall limit of L/f bounds the same sim
                # distance as L does on a real truck. The lease is administrative and only kept above the command age.
                # Real-time plan D7: a catch-up burst runs at most catchup_steps / 120 sim s ahead of f x wall, so
                # the wall limits give that much up to bound the same sim distance as before.
                config=LiveArbiterConfig(tick_s=1 / 120,
                                         command_max_age_s=(0.30 - live_catchup_ / 120) / args.live_factor,
                                         video_max_age_s=(0.50 - live_catchup_ / 120) / args.live_factor,
                                         frame_window_s=max(5.0, 2 * 0.50 / args.live_factor),
                                         lease_timeout_s=max(args.live_lease_timeout_s, 0.30 / args.live_factor,
                                                             0.50 / args.live_factor),
                                         # The truck stops in sim time: the 10 s stop deadline is 10/f wall s here
                                         # (at f 0.08 the unscaled one took the settling start for a failed stop and
                                         # latched STOPPED before TELEOP could be entered, 2026-10-17 job 3886).
                                         not_stopped_s=LiveArbiterConfig.not_stopped_s / args.live_factor,
                                         # The pixels trail the render metadata in sim steps, so the wall bound
                                         # goes as 1/f (M-c evidence); the per-frame pixel age is checked as well.
                                         camera_latency_bound_s=(args.live_camera_latency_bound_s
                                                                 or (args.live_camera_pixel_lag_s + 1 / 120)
                                                                 / args.live_factor)),
                camera_pixel_lag_s=args.live_camera_pixel_lag_s,
                host=args.live_host, web_root=args.live_web_root,
                test_step_delay_s=args.live_test_step_delay_ms / 1000.0, camera_freeze=freeze_,
                test_stop_in_capture=args.live_test_stop_in_capture,
                catchup_steps=live_catchup_,
                # The page's teleop overlay (display only, contract 3); none without the perception camera.
                camera_model=None if live_cam_ is None else live_camera_model(
                    _bound["perception_calibration"], _bound["perception_mount"],
                    rear_axle_offset_m=args.rear_axle_offset_m, sent_width=640,
                    rides_lift=args.perception_mount != "legacy"))  # legacy hangs off base_link
            from forklift_core import kernels as live_kernels

            # Real-time plan D3: a live run takes the fastest kernels (a batch run stays on the reference unless told),
            # compiled (or loaded) before the first step, not in it.
            live_kernels.prefer_fast()
            state["kernels"] = {"mode": live_kernels.mode(), "warm_up_s": live_kernels.warm_up()}
            if os.environ.get("FORKLIFT_LIVE_ASYNC_PLAN", "1") != "0":
                # Real-time plan D8: the long plans go to a worker process; the control loop stands and goes on.
                from forklift_core.planning.plan_worker import PlanWorker

                core.jobs = PlanWorker()
                state["async_plan_worker"] = {"pid": core.jobs.proc.pid}
            core.hooks.live_abort = lambda: live.stopped_in_tick  # the core stops right after a raw capture

            live_marker_ = None
            if args.live_test_marker and live_cam_ is not None:
                # The camera rides the fork carriage: a lift moving on a known wave moves the picture's horizon, so a
                # frame's pixels tell the lift height -- and so the sim time -- they were drawn at (plan C5). A plate
                # added mid-run did not show in the frames (2026-10-17). For a run that stays
                # PAUSED: the wave overrides the mission's lift target and the gate's latch at every physics step.
                def live_marker_(sim_s: float) -> None:
                    # Two sines of 2.0 and 3.1 s: no shift under 62 s maps the wave onto itself, so a lag cannot be
                    # read as a period off (Codex M-c round 16 P2). Within the joint's 0-0.28 m and 1 m/s.
                    target_ = 0.14 + 0.06 * (math.sin(2 * math.pi * sim_s / 2.0) + math.sin(2 * math.pi * sim_s / 3.1))
                    robot.apply_action(isaac.action(joint_positions=np.array([target_]), joint_indices=lift_index))

            def live_step_world(render: bool) -> None:
                sim_ = float(world.current_time - initial_time)
                live.before_step(sim_)
                if live_marker_ is not None:
                    live_marker_(sim_ + 1 / 120)  # after the gate: the wave also runs past a stopped step's lift latch
                camera_ = live.camera_due() and live_cam_ is not None
                # Real-time plan S0: the step's own wall time by kind (render, scan, plain), the pacing sleep apart.
                # (step_world renders a video frame on its own when one is due: that step is a render step too.)
                scan_ = args.record_slam and stepper["tick"] % scan_every == 0
                frame_ = args.video and stepper["tick"] % fps_divisor == 0
                kind_ = "render" if (render or camera_ or frame_) else ("scan" if scan_ else "plain")
                started_, cpu_ = time.perf_counter(), time.thread_time()
                step_world(render or camera_)
                live_timers.add(f"step.{kind_}", time.perf_counter() - started_, time.thread_time() - cpu_)
                if slam is not None:
                    speed_, yaw_rate_ = slam.get("odom_speed"), float(slam.get("odom_yaw_rate", 0.0))
                    made_ = slam.get("odom_mono")
                else:
                    made_ = live.clock()
                    rates_ = robot.get_joint_velocities()[wheels]
                    speed_, yaw_rate_ = float(np.mean(rates_[2:4])) * args.drive_geometry.wheel_radius_m, 0.0
                live.after_step(float(world.current_time - initial_time), odom_speed=speed_, yaw_rate=yaw_rate_,
                                odom_mono=made_, rendered=camera_, render_sim_time_of=lambda rt: rt - initial_time)
                if live_marker_ is not None:
                    live.log({"lift_sim_t": float(world.current_time - initial_time),
                              "lift_m": float(robot.get_joint_positions()[lift_index[0]]),
                              "step_start_mono": live.step_start[-1][1]})  # sim -> wall for the lag (round 16 P1)

            def live_permit(curvatures, direction, check):
                # The arbiter's own permission on the layer's latest grid, in the sim's time (plan C2: the world and
                # its sensors move in sim time); the snapshot is taken again only when a new scan came in.
                if live_perm is None:
                    return 0.0, "no_permission"
                layer_ = obstacle["layer"]
                now_ = float(world.current_time - initial_time)
                # The layer's own latest snapshot -- the one the mission's permission reads, with its shadow band and
                # known obstacles applied, made once per scan (a separate whole-map snapshot here cost more than a
                # live step and missed that processing).
                snap_ = layer_.snapshot
                if snap_ is None:
                    return 0.0, "no_snapshot"
                key_ = (id(snap_), float(snap_.stamp_s), obstacle["version"])
                loaded_ = ms.phase in ("lift", "extract", "transport", "lower")
                fp_, own_ = layer_.footprints(loaded_, direction)
                pose_ = tuple(float(v) for v in live_state["rear"])
                result_ = live_perm.limit(snap_, key_, now_, layer_.last_scan_s, pose=pose_, curvatures=curvatures,
                                          direction=direction, check_speed_mps=check, footprint=fp_,
                                          own_footprint=own_)
                if args.live_test_permit_compare:
                    live_state["permit_calls"] = live_state.get("permit_calls", 0) + 1
                    if live_state["permit_calls"] % 12 == 0:
                        full_ = live_arc_permit(LiveDrivePermission(obstacle["layer"].permission.config), snap_,
                                                now_, layer_.last_scan_s, pose=pose_, curvatures=list(curvatures),
                                                direction=direction, check_speed_mps=check, footprint=fp_,
                                                own_footprint=own_)
                        # The true rear-axle pose, for the test's own reading of the path the truck drove against
                        # the bundle's arcs (offline, between comparisons); nothing decides on it (Codex M-c round
                        # 24: the odometry's yaw is integrated from the steering model and cannot check it).
                        tb_, tq_ = robot.get_world_pose()
                        tyaw_, _ = yaw_and_tilt(tq_)
                        truth_ = (float(tb_[0]) - abs(args.rear_axle_offset_m) * math.cos(tyaw_),
                                  float(tb_[1]) - abs(args.rear_axle_offset_m) * math.sin(tyaw_), float(tyaw_))
                        live.log({"permit_cmp": now_, "pose": pose_, "curvatures": list(curvatures),
                                  "direction": direction, "check_mps": check, "bundle": list(result_),
                                  "full": list(full_), "truth_rear": truth_, "silenced": scan_silenced(now_)})
                return result_

            # Real-time plan S0 (docs/plans/2026-10-17-realtime-sim.md): each region's wall and CPU time, in the
            # status log once a wall second (forklift_core.perf); live only, so a batch run's calls are untouched.
            # The permission, grid, shadow and layer classes are timed (every instance -- the bundle's own
            # permissions too -- and nothing put on an instance, so a dump still pickles it), by caller where one
            # method serves several.
            import forklift_core.control.drive_permission as timed_permission
            from forklift_core.control.shadow_memory import ShadowMemory as TimedShadow
            from forklift_core.perception.obstacle_grid import ObstacleGrid as TimedGrid
            from forklift_core.perf import TIMERS as live_timers

            live_timers.enabled = True
            live_timers.wrap(world, "step", "isaac.step")
            if args.record_slam:  # the module and the scan cadence exist only with the recorded scans
                live_timers.wrap(planar_lidar, "cast_scan_flags", "lidar.cast")
            if slam is not None:
                live_timers.wrap(slam["link"], "exchange", "slam.exchange")
            if obstacle is not None:
                layer_class_ = type(obstacle["layer"])
                for name_ in ("add_scans", "refresh", "limit", "check_candidate", "planner_grid"):
                    live_timers.wrap(layer_class_, name_, f"layer.{name_}", by_caller=True)
                live_timers.wrap(TimedGrid, "snapshot", "grid.snapshot", by_caller=True)
                live_timers.wrap(TimedShadow, "apply", "shadow.apply")
                live_timers.wrap(TimedShadow, "_move", "shadow.move")  # real-time plan D9d: is the 0.27 s spike the move?
                # D9d: the move was not it (d9a_1: 114 calls, at most 14 ms); time Python's garbage collections.
                # A collection can start at any allocation -- inside the status line's walk over the timers too --
                # so the callback only appends to its own list, folded into the timers at a tick's start (Codex D9
                # impl round 2: a callback adding a timer mid-walk raised "dictionary changed size").
                import gc as live_gc_

                gc_started_ = {}

                def live_gc_timer(phase, info):
                    if phase == "start":
                        gc_started_["t"] = time.perf_counter()
                    elif "t" in gc_started_:
                        gc_seen_.append((f"gc.gen{info.get('generation')}", time.perf_counter() - gc_started_.pop("t")))

                live_gc_.callbacks.append(live_gc_timer)
                for name_ in ("update", "allowed_speed", "check_path", "check_arc"):
                    live_timers.wrap(timed_permission.DrivePermission, name_, f"permission.{name_}", by_caller=True)
            for name_, value_ in list(vars(type(core)).items()):  # every public step of the mission core
                if callable(value_) and not name_.startswith("_"):
                    live_timers.wrap(type(core), name_, f"core.{name_}")
            for name_ in ("plan_transport", "plan_transport_leg", "plan_return_leg", "plan_observation_leg",
                          "grid_world", "grid_kwargs", "final_straight_prefix"):  # the planner ports
                if callable(getattr(core.hooks, name_, None)):
                    live_timers.wrap(core.hooks, name_, f"plan.{name_}", by_caller=True)
            live_permit_untimed_ = live_permit

            def live_permit(curvatures, direction, check):
                with live_timers.region("teleop.permit"):
                    return live_permit_untimed_(curvatures, direction, check)

        stepper["fn"] = step_world if live is None else live_step_world
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
            cast_reference_ = (recorded_function(isaac.cast_reference_scan, "SimTruth", "cast_reference_scan", boundary)
                               if early_ports else isaac.cast_reference_scan)
            ray_distances, ray_hits, ray_own = cast_reference_(
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
        if near is not None:
            check_near_contract()
        if boundary is not None:
            # The services the mission logic calls (M-a0.5 ports); the physics step is one recorded call so
            # what the services do inside it (SLAM receive, scans, odometry) is not recorded as the core's.
            from forklift_core.mission.boundary_record import DictWatch

            # What the physics step writes into the core's dictionaries (services updated per tick), and the
            # scans the core reads back (docking reads last_sent_ranges, the permission diagnostics last_raw).
            # Format 2 also watches the SLAM log the step fills (its lists only grow; the run writes it at the end
            # -- a replay that does not step must still hold it, M-a1 replay R5).
            watched_log_ = early_ports and slam_log is not None
            step_watch = DictWatch(
                boundary.recorder,
                {"slam": slam, "obstacle": obstacle, **({"slam_log": slam_log} if watched_log_ else {}),
                 # format 2: the tyre contact log the step appends to the state (an evaluation record)
                 **({"state": state} if early_ports else {})},
                keys={"slam": ["stop_now", "tracker_speed", "odom_speed", "odom_rear", "odom_yaw_rate", "scan_id",
                               "last_scan_odom_base", "last_scan_estimate_rear", "last_sent_ranges", "records",
                               "last_command"],  # M-a3 (plan E9): a preempt sets it inside the step
                      "obstacle": ["version", "last_stamp", "applied", "last_raw", "scans"],
                      **({"slam_log": list(slam_log)} if watched_log_ else {}),
                      # M-a3 (plan E9): a preempt sets the applied command and its slew inside the step
                      **({"state": ["tyre_contact_rows", "applied_command", "command_speed", "preempt_seen_s"]}
                         if early_ports else {})},
                append_only={"records", "scans", *(slam_log if watched_log_ else ()),
                             *(("tyre_contact_rows",) if early_ports else ())},
            )
            if not early_ports:  # format 2 made these ports at the hand-over
                robot = RecordingProxy(robot, "Vehicle", boundary)
                pallet = RecordingProxy(pallet, "SimTruth", boundary)
                world = RecordingProxy(world, "Clock", boundary)
                cells.world, cells.robot = world, robot  # the capture's callbacks read the proxies from here on
            if slam is not None:
                slam["tracker"] = RecordingProxy(slam["tracker"], "Slam", boundary)
            if obstacle is not None and not early_ports:
                obstacle["layer"] = RecordingProxy(obstacle["layer"], "Obstacles", boundary,
                                                   child_ports={"permission": "Permission"})
            if args.use_perception:
                # A capture steps physics inside itself: the same watch covers what those steps change.
                perception_capture = RecordingProxy(perception_capture, "Capture", boundary,
                                                    watches={"capture": step_watch})
            stepper["fn"] = recorded_function(stepper["fn"], "Vehicle", "step", boundary, watch=step_watch)
            # The pocket camera's frame read and selection with what it feeds the near tracker and the
            # pocket check (services), and what that changes in the core's dictionaries.
            read_pocket_frame = recorded_function(
                read_pocket_frame, "Frames", "read_pocket_frame", boundary,
                watch=DictWatch(
                    boundary.recorder, {"pocket": pocket, "near": near},
                    keys={"pocket": ["await_still_frame", "still_frame_s", "still_frames", "frames_read",
                                     "frames_without_time", "frames_bad_time", "silenced_frames"],
                          "near": ["last_stamp", "last_fed_stamp", "mismatch", "unbounded", "frames", "events"]},
                    append_only={"frames", "events"},
                ),
            )
            # Branch coverage (plan D2): the record lists of the core's dictionaries that grew in a tick name the
            # branches it took (a transition, a replan, a retry, a stop event...); the per-tick logs are left out.
            per_tick_lists = {"state.samples", "state.tyre_contact_rows", "slam.control", "slam.records",
                              "obstacle.scans", "near.frames", "near.ticks", "pocket.history"}
            branch_counts: dict = {}

            def branch_probe() -> list:
                grown = []
                for name_, d_ in (("state", state), ("obstacle", obstacle), ("near", near), ("slam", slam),
                                  ("pocket", pocket)):
                    if not isinstance(d_, dict):
                        continue
                    for k_, v_ in list(d_.items()):
                        key_ = f"{name_}.{k_}"
                        if not isinstance(v_, list) or key_ in per_tick_lists:
                            continue
                        start_ = branch_counts.get(key_, 0)
                        for item_ in v_[start_:]:
                            # A branch id from the new record's identifying fields (a transition's from/to, a
                            # hold event, a replan kind or reason), not only the list that grew (Codex M-a1).
                            tag_ = ",".join(
                                f"{k}={item_[k]}" for k in ("from", "to", "event", "kind", "reason", "phase", "status")
                                if isinstance(item_, dict) and isinstance(item_.get(k), (str, int, float, bool))
                            ) if isinstance(item_, dict) else ""
                            grown.append(f"{key_}[{tag_}]" if tag_ else key_)
                        branch_counts[key_] = len(v_)
                return grown

            branch_probe()  # the counts the loop starts from
            _BOUNDARY_RECORD["probe"] = branch_probe  # an exit tick closes with its branches too
            # Core-side wall-clock reads (planning deadlines) through a Clock proxy of this module's `time`.
            if not early_ports:
                _BOUNDARY_RECORD["time"] = time
                globals()["time"] = RecordingProxy(time, "Clock", boundary)
        if early_ports:
            boundary.recorder.tick_end(outputs={"bootstrap": True}, branches=[])
        from forklift_core.perf import CONTEXT as perf_context

        if live is not None and slam is not None and os.environ.get("FORKLIFT_LIVE_ASYNC_SLAM", "1") != "0":
            # Real-time plan D9a: SLAM warms up before the first control tick, in lockstep (the bridge waits a second
            # for each start-up scan slam_toolbox drops), the truck standing; then the link stops waiting and the
            # tracker's age limit only backs up the reply deadline (Codex D9 1st P2-2, P2-3).
            prewarm_wall_ = time.monotonic()
            prewarm_steps_ = 0
            while slam["tracker"].mode == "warming_up":
                step_world(False)
                prewarm_steps_ += 1
                if prewarm_steps_ > 120 * 60:
                    raise SessionFailure("slam_prewarm_timeout")
            slam["async_link"] = slam["module"].AsyncSlamLink(
                slam["link"], deadline_s=SLAM_LIVE_REPLY_DEADLINE_S, queue_max=SLAM_LIVE_QUEUE_MAX)
            slam["tracker"].max_age_s = 0.25 + SLAM_LIVE_REPLY_DEADLINE_S * args.live_factor
            state["slam_async"] = {"prewarm_steps": prewarm_steps_,
                                   "prewarm_sim_s": float(world.current_time - initial_time),
                                   "prewarm_wall_s": time.monotonic() - prewarm_wall_,
                                   "reply_deadline_s": SLAM_LIVE_REPLY_DEADLINE_S, "queue_max": SLAM_LIVE_QUEUE_MAX,
                                   "tracker_max_age_s": slam["tracker"].max_age_s, "max_depth": 0}
        if live is not None:
            # Real-time plan D9d: what was built so far (the stage, the scene, the planners' tables) is frozen out of
            # the garbage collector's walks; a full collection then walks only what the run makes (d9d_2/3/sc: a 0.30-
            # 0.35 s generation-2 collection near sim 174 s was the remaining long tick).
            import gc as live_gc_freeze_

            live_gc_freeze_.collect()
            live_gc_freeze_.freeze()
            state["gc_frozen_objects"] = live_gc_freeze_.get_freeze_count()
            live.start()  # after the SLAM warm-up: the console sees the first control tick next (Codex D9 impl P3)

        for step in range(int(120 * args.max_sim_seconds)):
            if early_ports:
                # Format 2: the tick opens first, the clock is read inside it (recorded), then the head's times
                # are set from that read (a replay checks them against the recorded head).
                # (M-a3: the commands applied and the stop kind follow the clock read, so they are set with the times.)
                boundary.recorder.tick_begin(step, tick_t=None, work_t=None, commands=None, stop_kind=None)
                t = world.current_time - initial_time
            else:
                t = world.current_time - initial_time
                if boundary is not None:
                    boundary.recorder.tick_begin(step, tick_t=t, work_t=t, commands=[], stop_kind="none")
            book_clock["t"] = t
            perf_context["phase"] = ms.phase  # real-time plan S0: the dumps' strata (read only by the dumps)
            applied_commands_, stop_kind_ = [], "none"
            live_abort_ = False  # M-c: the stop gate stopped the wheels inside this tick (plan C2)
            while gc_seen_:  # D9d: the collections timed since the last tick, folded in outside any walk
                live_timers.add(*gc_seen_.pop())
            if live is not None:
                stop_kind_, applied_commands_ = live.tick_start(t, core.near_active())
                if core.commands.status == "accepted":
                    core.commands.start()  # the operator's submit: the mission runs from this tick
            elif mission_schedule is not None:
                # M-a3 (plan E2, E7): the schedule's due commands and the stop kind in force, applied at this
                # control tick's start.
                while mission_schedule and mission_schedule[0][0] <= t:
                    core.commands.queue(MISSION_COMMANDS[mission_schedule.pop(0)[1]]())
                stop_kind_ = stop_kind_at(t)
                if (state.pop("preempt_seen_s", None) is not None and stop_kind_ == "none"
                        and core.commands.status in ("running", "resuming")):
                    stop_kind_ = "preempt"  # a preempt that began and ended inside a capture still stops the mission
                applied_commands_ = core.commands.apply(t, MissionContext(
                    stop_kind=stop_kind_, near_tracker=core.near_active()))  # M-b B7: the contract applies now
            if early_ports:
                boundary.recorder.tick_time(tick_t=t, work_t=core.commands.work_elapsed(0.0, t),
                                            commands=applied_commands_, stop_kind=stop_kind_)
            rear_stamp = t  # the time of the control pose `rear` (a capture moves it on)
            dwell_hold = dwell_standing = False
            cusp_hold = False  # plan cusp position stall v7 K3: zero command, steering held
            safety_hold = False  # plan D8 S4: a safety stop -- the final wheel command steps to zero
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
            if slam is not None and slam.get("async_link") is not None:
                # Real-time plan D9a: the SLAM replies in since the last tick, applied before this tick's control
                # pose is read and before any mission work (the grid re-projected with them there).
                try:
                    replies_ = slam["async_link"].poll()
                except slam["module"].SlamLinkFailure as exc:
                    if slam["async_link"].long_poll_gaps:  # the gap that ended it stays in the record
                        state["slam_async"]["long_poll_gaps"] = [list(g) for g in slam["async_link"].long_poll_gaps]
                    raise SessionFailure(f"slam_link_failed: {exc}") from exc
                if replies_:
                    core.slam_replies(t, replies_, ms.phase in ["lift", "extract", "transport", "lower"])
                    state["slam_async"]["max_depth"] = slam["async_link"].max_depth
                if slam["async_link"].long_poll_gaps:  # polls further apart than the deadline (a stop, a stall)
                    state["slam_async"]["long_poll_gaps"] = [list(g) for g in slam["async_link"].long_poll_gaps]
            if slam is not None:
                rear = slam_rear(t)
                signed_speed = slam["tracker_speed"]
                slam["control"].append([t, *rear.tolist(), *truth_rear.tolist()])
            if obstacle is not None:
                obstacle["control_rear"] = tuple(float(v) for v in rear)  # known pallets fix against it
                if ms.phase == "withdraw" and obstacle["layer"].corridor is not None and obstacle.get("withdraw_line"):
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
            if ms.phase == "lift":
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
                        lift_command_m=ms.lift_command,
                    )
            if not (tilt < 0.1 and pallet_tilt < 0.15):
                state["tilt_abort_state"] = {
                    "phase": ms.phase,
                    "time_s": t,
                    "base_tilt_rad": tilt,
                    "pallet_tilt_rad": pallet_tilt,
                    "base_position_m": base,
                    "base_quaternion_wxyz": q,
                    "pallet_position_m": ppos,
                    "pallet_quaternion_wxyz": pq,
                    "lift_m": float(robot.get_joint_positions()[lift_index[0]]),
                    "lift_command_m": ms.lift_command,
                }
            require(
                tilt < 0.1 and pallet_tilt < 0.15, f"Excessive body tilt in {ms.phase}"
            )
            loaded = ms.phase in ["lift", "extract", "transport", "lower"]
            footprint = (
                geometry.loaded_footprint if loaded else geometry.unloaded_footprint
            )
            checked_obstacles = obstacles + (
                [pickup_obstacle] if ms.phase in ("approach", "observe") else []
            )
            if ms.phase in ("return_home", "home_settle"):
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
            truth_bounds = scenario.bounds
            if getattr(args, "live", False) and getattr(args, "live_test_permit_compare", False):
                # M-c completion 5 only (a recorded run's args predate these): the hall bounds lie 0.76 m inside the
                # wall the scan sees (x 4.7 against 5.46, 2026-10-17), so the planning-area check is widened; the
                # stop before the wall is judged from the end pose and the scanned wall, not by this check.
                b_ = scenario.bounds
                truth_bounds = type(b_)(b_.x_min_m - 3.0, b_.x_max_m + 3.0, b_.y_min_m - 3.0, b_.y_max_m + 3.0)
            require(
                collision_free_pose(truth_rear, [], footprint, truth_bounds)
                and not any(SHAPE_MEETS(o, truth_shape, tuple(float(v) for v in truth_rear)) for o in checked_obstacles),
                f"Actual truck/load footprint overlap in {ms.phase}",
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
                f"Measured pallet footprint overlap in {ms.phase}",
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
                            f"Measured pallet solid overlap in {ms.phase}: {box_.name}",
                        )
            relative = ppos[:2] - base[:2]
            if ms.phase in ["extract", "transport"]:
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
            if ms.phase in ["approach", "insert"]:
                require(
                    np.linalg.norm(ppos[:2] - initial_pallet[:2]) < 0.018,
                    "Pallet displaced before pickup",
                )
            if ms.phase in ["insert", "withdraw"]:
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
                    f"Forbidden fork/pallet contact in {ms.phase}: {contacts}",
                )
            if ms.phase in ["insert", "extract", "withdraw"]:
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
                    ms.phase, {"left": None, "right": None}
                )
                for side, gap in lateral.items():
                    if gap is not None and (minima[side] is None or gap < minima[side]):
                        minima[side] = gap
            requested_speed, curvature = 0.0, 0.0
            tracking = None
            # M-a1 slice 5: the release gate at the tick's start is the core's.
            gate_ = core.release_gate(t, rear)
            releasing, warming, rear = gate_.releasing, gate_.warming, gate_.rear
            paused_ = core.commands.stopped  # M-a3 (plan E3): paused, resuming or cancelling -- no mission work
            if live is not None and core.commands.status in ("idle", "accepted", "done", "failed", "cancelled"):
                paused_ = True  # M-c: no mission yet (or over): its work does not run
            cancel_stop_ = False
            # Real-time plan D8b: a tick waiting for a plan (asked, still computing, or just taken) stands: none of the
            # mission's own work below runs, the permission watches, the links, commands and sensors go on.
            plan_wait_ = core.jobs is not None and core.job_hold(
                t, rear, float(np.dot(velocity[:2], forward)), paused_).hold
            # Real-time plan D9c: the observe capture runs over control ticks (one render step each, at this tick's
            # step) and its observation is handled on the tick after the one that detected it; those ticks stand
            # as a plan wait does.
            if live is not None and core.jobs is not None:
                if paused_:
                    if core.capture_live is not None:
                        core.capture_abandon()
                        live.capturing = False
                    core.observation_live = None  # a pause or a cancel: the arrival captures again when running
                elif core.capture_live is not None:
                    plan_wait_ = True
                    if core.capture_live.raw is not None:
                        c_ = core.capture_live
                        core.capture_live = None
                        captured_ = core.capture_and_detect(t, rear, step, raw=(c_.attempt, c_.attempt_number, c_.raw))
                        if getattr(captured_, "aborted", False):
                            live_abort_ = True
                        else:
                            core.observation_freeze(t, captured_)
                            rear = captured_.rear  # the estimate now, after the capture's own release
                            if captured_.rear_stamp is not None:
                                rear_stamp = captured_.rear_stamp
                            if slam is not None:
                                slam["control"][-1][1:4] = rear.tolist()
                elif core.observation_live is not None and not plan_wait_:
                    plan_wait_ = True
                    captured_ = core.observation_take(t)
                    if captured_ is not None:
                        attempt, attempt_number, scene_input = captured_.attempt, captured_.attempt_number, captured_.scene_input
                        frame_diagnostics, capture_attempts = captured_.frame_diagnostics, captured_.capture_attempts
                        capture_diagnostics, base, q, yaw = captured_.capture_diagnostics, captured_.base, captured_.q, captured_.yaw
                        forward, rear = captured_.forward, captured_.rear
                        if captured_.rear_stamp is not None:
                            rear_stamp = captured_.rear_stamp
                        prior, params, detection, observation = captured_.prior, captured_.params, captured_.detection, captured_.observation
                        if observation.status != "valid":
                            if core.observe_failed(t, rear, attempt, attempt_number, observation):
                                break
                        else:
                            succeeded_ = core.observe_succeeded(t, rear, base, yaw, attempt, attempt_number, observation,
                                                                scene_input, capture_diagnostics, step)
                            if succeeded_.stop:
                                break
                            observation, near_leg = succeeded_.observation, succeeded_.near_leg
                            if not getattr(succeeded_, "pending", False):
                                core.near_leg_or_recapture(t, rear, base, yaw, observation, attempt_number, near_leg)
                        core.capture_acquired_s = None
            if ms.phase in trackers and not releasing and not warming and not paused_ and not plan_wait_:
                # Three times the path time at the tracker's own speed caps
                # (length / cruise when there are none), plus 10 s.
                # M-a1 slice 4a-i: the follower's tick and the rules that rewrite its result are the core's.
                follow_ = core.follow(t, rear, signed_speed, dt)
                tracking, cusp_hold = follow_.tracking, follow_.cusp_hold
                # M-a1 slice 4a-ii: the D7c docking stop, the SLAM goal heading miss and the stall replan.
                stall_ = core.docking_and_stall(t, rear, tracking, signed_speed, dt, loaded)
                tracking, rear, d7_docking_, d7_owned = stall_.tracking, stall_.rear, stall_.d7_docking_, stall_.d7_owned
                # M-a1 slice 4a-iii: the cusp and observe-leg recoveries are the core's.
                recover_ = core.cusp_and_observe_recovery(t, rear, tracking, signed_speed, dt, d7_docking_, d7_owned, step)
                tracking, rear = recover_.tracking, recover_.rear
                # M-a1 slice 4a-iv: the command and the stops before the arrival are the core's.
                command_ = core.command(t, rear, rear_stamp, tracking, signed_speed, dwell_hold, dwell_standing,
                                        safety_hold)
                requested_speed, curvature, tracking, rear = (command_.requested_speed, command_.curvature,
                                                              command_.tracking, command_.rear)
                dwell_hold, dwell_standing, safety_hold = command_.dwell_hold, command_.dwell_standing, command_.safety_hold
                if tracking.status == "arrived":
                    if args.use_perception and ms.phase == "observe":
                        if live is not None and core.jobs is not None:
                            # Real-time plan D9c: the capture opens here and runs over the next ticks.
                            begun_ = core.capture_begin(t, rear, step)
                            live.capturing = core.capture_live is not None
                            live_abort_ = bool(getattr(begun_, "aborted", False))
                            plan_wait_ = True
                        else:
                            # M-a1 slice 4b-i: the capture and the detection are the core's (the capture itself a port).
                            if live is not None:
                                live.capturing = True
                            try:
                                captured_ = core.capture_and_detect(t, rear, step)
                            finally:
                                if live is not None:
                                    live.capturing = False
                            # M-c (plan C2): a STOP or an expiry at a physics step inside the capture ends this tick's
                            # mission work right after the raw capture (the core returns ``aborted``) -- no SLAM release,
                            # no observation handling, no transitions; the next tick pauses the mission.
                            live_abort_ = bool(getattr(captured_, "aborted", False))
                            if not live_abort_:
                                attempt, attempt_number, scene_input = captured_.attempt, captured_.attempt_number, captured_.scene_input
                                frame_diagnostics, capture_attempts = captured_.frame_diagnostics, captured_.capture_attempts
                                capture_diagnostics, base, q, yaw = captured_.capture_diagnostics, captured_.base, captured_.q, captured_.yaw
                                forward, rear = captured_.forward, captured_.rear
                                if captured_.rear_stamp is not None:
                                    rear_stamp = captured_.rear_stamp
                                prior, params, detection, observation = captured_.prior, captured_.params, captured_.detection, captured_.observation
                            if live_abort_:
                                pass
                            elif (
                                args.repeat_captures
                                and (
                                    attempt_number == args.repeat_at_attempt
                                    if args.repeat_at_attempt is not None
                                    else state.get("near_capture", {}).get("status") == "pending"
                                )
                            ):
                                # G2r: the original capture above is G2's; hold the
                                # wheels, capture again, then stop whatever is seen.
                                robot.apply_action(
                                    isaac.action(
                                        joint_velocities=np.zeros(len(wheels)),
                                        joint_indices=wheels,
                                    )
                                )
                                first_pose = [
                                    np.asarray(v, dtype=float).tolist()
                                    for v in capture_diagnostics.accepted_pose
                                ]

                                def observed(result) -> dict:
                                    obs, diag = result.observation, result.diagnostics
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
                                        # Fork spacing plan (B): the lower-deck evidence per
                                        # capture, to see how much it moves in place.
                                        "selected_lower": diag.selected_lower,
                                        "rejected_lower": [
                                            p.lower for p in diag.rejected_patterns
                                        ],
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
                                        **observed(detection),
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
                                                )
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
                            if live_abort_:
                                pass
                            elif observation.status != "valid":
                                # M-a1 slice 4b-iii: an observation without a detection goes to the next candidate (the core's).
                                if core.observe_failed(t, rear, attempt, attempt_number, observation):
                                    break
                            else:
                                # M-a1 slice 4b-iv: a valid observation's estimate, records, plan and followers are the core's.
                                succeeded_ = core.observe_succeeded(t, rear, base, yaw, attempt, attempt_number, observation, scene_input,
                                                                    capture_diagnostics, step)
                                if succeeded_.stop:
                                    break
                                observation, near_leg = succeeded_.observation, succeeded_.near_leg
                                if getattr(succeeded_, "pending", False):
                                    plan_wait_ = True  # real-time plan D8b: the mission plan is asked; this tick stands
                                else:
                                    # M-a1 slice 4b-v: the near leg, or the near capture's re-drawn straight, is the core's.
                                    core.near_leg_or_recapture(t, rear, base, yaw, observation, attempt_number, near_leg)
                    else:
                        # M-a1 slice 2: every other arrival is the core's.
                        arrival_ = core.arrive(t, tracking)
                        if arrival_.rear is not None:
                            rear = arrival_.rear
                        if arrival_.stop:
                            break
            if core.jobs is not None and not plan_wait_ and core.job_pending():
                plan_wait_ = True  # real-time plan D9b: the delivery docking match was just asked; this tick stands
            if live_abort_:
                requested_speed, curvature = 0.0, 0.0
            if not paused_ and not live_abort_ and not plan_wait_ and core.lift_and_timers(t, dt):  # M-a1 slice 2: the lift ramp, timers
                break
            # The curvature the wheels actually hold: an emergency stop keeps it, so
            # the permission, the stop itself and the probe all use this one.
            kappa = held_kappa()
            truth_speed = float(np.dot(velocity[:2], forward))
            obstacle_hold = False
            pocket_phase = False
            # Plan cusp position stall v7: a stall handed to the obstacle side keeps this block running
            # while it stands on a zero command.
            cusp_handed_ = cusp_monitor.handed(ms.phase) if obstacle is not None else None
            permission_tick_ = not paused_ and not live_abort_ and obstacle is not None and ms.phase in trackers and (
                requested_speed != 0.0 or abs(truth_speed) > 0.05 or obstacle.get("force_replan") == ms.phase
                or cusp_handed_ is not None
            ) and not plan_wait_
            # Real-time plan D8b: a tick standing for a plan has the paused tick's watch instead (no work counters,
            # no replan), and the truth evaluation below with what it allows.
            plan_watch_ = plan_wait_ and not paused_ and obstacle is not None
            if permission_tick_ or plan_watch_:
                if plan_watch_:
                    watch_ = core.permission_watch(t, rear, kappa, loaded)
                    allowed, why, acting, pocket_phase = watch_.allowed, watch_.why, watch_.acting, False
                    obstacle_hold = watch_.obstacle_hold
                else:
                    # M-a1 slice 3b: the permission tick (re-projection, limit, pocket check, path-blocked and
                    # silence stops, speed cap, stall watch) is the core's; the replan below reads its answer.
                    perm_tick_ = core.permission(t, rear, kappa, requested_speed, loaded, cusp_handed_)
                    allowed, why, acting, pocket_phase = perm_tick_.allowed, perm_tick_.why, perm_tick_.acting, perm_tick_.pocket_phase
                    requested_speed, obstacle_hold = perm_tick_.requested_speed, perm_tick_.obstacle_hold
                    # M-a1 slice 3c: stop, replan, resume (plan D4) is the core's.
                    if obstacle["blocked_ticks"] >= 120 or obstacle.pop("force_replan", None) == ms.phase:
                        replan_tick_ = core.blocked_replan(t, rear, kappa, requested_speed)
                        rear, requested_speed = replan_tick_.rear, replan_tick_.requested_speed
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
                                    {"time_s": t, "phase": ms.phase, "speed_mps": truth_speed, "allowed_mps": allowed,
                                     "reason": why, "object": oid}
                                )
            if paused_:
                # M-a3 (plan E3, E5, E6): the paused tick -- the permission watches every tick, the lift target
                # follows the lift, the stop is watched; a cancel ends standing, a resume steps through its checks.
                paused_tick_ = core.paused_tick(t, rear, kappa, loaded)
                rear, obstacle_hold = paused_tick_.rear, paused_tick_.obstacle_hold
                requested_speed, curvature = 0.0, 0.0
                dwell_hold = True  # the steering held where it is; the wheels brake on its curvature
                cancel_stop_ = paused_tick_.stop  # the loop ends after this tick's stop output is applied
            elif plan_wait_:  # real-time plan D8b: standing for a plan, the steering held
                requested_speed, curvature = 0.0, 0.0
                dwell_hold = True
            elif core.live_leg(t, rear, loaded):  # M-a1 slice 3a: the live travel leg and the replan wait
                requested_speed = 0.0
                if core.jobs is not None and core.job_pending():
                    plan_wait_ = True  # real-time plan D8b: the leg's plan was just asked
                    curvature = 0.0
                    dwell_hold = True
            if new_obstacles is not None and ms.phase in trackers:
                leg, leg_direction = trackers[ms.phase].leg_ahead()
                driven = float(trackers[ms.phase]._distance[-1]) - trackers[ms.phase].remaining_to_goal_m()
                pallet_face = None
                if "approach" in paths and ms.phase in ("approach", "insert"):
                    # N11/N12/N14 place bars in the pallet frame: the approach
                    # face centre of the real pallet, the insertion heading of
                    # the approach's end (a scenario placement, never a planner
                    # input -- plan D6).
                    hy = float(paths["approach"].poses[-1][2])
                    half = geometry.pallet_depth_m / 2
                    pallet_face = (float(ppos[0]) - half * math.cos(hy), float(ppos[1]) - half * math.sin(hy), hy)
                if obstacle is not None and obstacle.get("backoff", {}).get("phase") == ms.phase:
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

                due_ = new_obstacles["schedule"].update(
                    t, ms.phase, driven, leg, leg_direction=int(leg_direction), pallet_face=pallet_face,
                    sweep_test=n_sweep_test,
                )
                if boundary is not None and boundary.recorder.value_format == 2:
                    # The schedule has no port: its inputs (by digest -- the leg ahead is an array every tick) and
                    # what it did with them, so a replay rerunning it from the same rules is checked (plan 6).
                    from forklift_core.mission.boundary_record import payload_digest, trace_entry

                    trace_entry(boundary, "schedule", {
                        "inputs_sha256": payload_digest({
                            "t": float(t), "phase": ms.phase, "driven": float(driven),
                            "leg": None if leg is None else np.asarray(leg, dtype=float),
                            "leg_direction": int(leg_direction), "pallet_face": pallet_face}),
                        "actions": [list(a_) for a_ in due_]})
                for action in due_:
                    if action[0] == "spawn":
                        _, oid, ox, oy, oyaw, osize = action
                        path_ = f"/World/NewObstacle_{oid}"
                        isaac.spawn_box(path_, ox, oy, oyaw, osize)
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
                        isaac.remove_prim(f"/World/NewObstacle_{oid}")
                        rect = new_obstacles["rects"].pop(oid, None)
                        if rect in obstacles:
                            obstacles.remove(rect)
            estop_holding = estop is not None and estop.update(
                t=t,
                phase=ms.phase,
                phase_elapsed_s=t - ms.phase_started,
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
            # M-a1 slice 5: the command assembly is the core's (the e-stop probe above stays the runner's, truth).
            steer_before_ = steering_command.copy() if live is not None else None
            command_ = core.assemble_command(t, dt, requested_speed, curvature, kappa, steering_command, estop_holding,
                                             obstacle_hold, dwell_hold, cusp_hold, safety_hold)
            drive, steering_command, actual_steering = command_.drive, command_.steering_command, command_.actual_steering
            requested_speed = command_.requested_speed
            if live is not None:
                # M-c (plan C2): the arbiter decides what drives. AUTO and PAUSED: the core's command as assembled.
                # TELEOP: the operator's speed and curvature through the same back end (Ackermann, steering rate);
                # a preempt: zero wheels, steering held. The applied command is written back for the core.
                live_state["rear"] = tuple(float(v) for v in rear)
                speed_now_ = live.feedback.rear_axle_speed_mps if live.feedback is not None else 0.0
                # The sim distance driven since the last tick, for the installed arcs: the odometry's own motion
                # (Codex M-c round 10 P2: the end speed times dt missed a braking interval's travel).
                odom_now_ = tuple(float(v) for v in slam["odom_rear"][:2]) if slam is not None else None
                if live_perm is not None:
                    prev_ = live_state.get("odom_prev")
                    if odom_now_ is not None and prev_ is not None:
                        live_perm.advance(math.hypot(odom_now_[0] - prev_[0], odom_now_[1] - prev_[1]))
                    elif odom_now_ is None:
                        live_perm.advance(abs(float(speed_now_)) * dt)
                live_state["odom_prev"] = odom_now_
                target_steer_ = np.asarray(live_ackermann(1.0, live.arbiter.drive.kappa, args.drive_geometry).steering_rad)
                live_safety_ = core.safety_state(t)
                out_ = live.tick_end(
                    t, applied_commands_, live_safety_, (float(drive.speed_mps), float(drive.curvature_inv_m)),
                    {"measured_mps": float(speed_now_), "measured_kappa": float(kappa),
                     "steering_error_rad": float(np.max(np.abs(target_steer_ - actual_steering))),
                     "permit": live_permit})
                if out_.source == "teleop":
                    if args.live_test_permit_compare:
                        # The bundle's inputs each teleop cycle, for the offline coverage check (teleop bundle margin
                        # plan V3): the grid bundle's last curvature is its margined edge, not the output.
                        live.log({"teleop_k": float(t), "measured_kappa": float(kappa),
                                  "output_kappa": float(live.arbiter.drive.kappa), "speed_mps": float(out_.speed_mps),
                                  "permit_asked": bool(out_.policy.drive_allowed)})  # forbidden: no arcs asked
                    drive = live_ackermann(out_.speed_mps, out_.curvature_inv_m, args.drive_geometry)
                    rate_ = settings["steering_command_rate_rad_s"] * dt
                    steering_command[:] = steer_before_ + np.clip(np.asarray(drive.steering_rad) - steer_before_,
                                                                  -rate_, rate_)
                    requested_speed = float(drive.speed_mps)
                    state["command_speed"] = float(out_.speed_mps)
                    state["applied_command"] = (float(drive.speed_mps), [float(v) for v in steering_command])
                elif out_.stop_kind == "preempt":
                    drive = live_ackermann(0.0, kappa, args.drive_geometry)
                    steering_command[:] = steer_before_
                    requested_speed = 0.0
                    state["command_speed"] = 0.0
                    state["applied_command"] = (0.0, [float(v) for v in steering_command])
                # The state carries what is applied after the choice above (contract 6, Codex M-c round 5 P1).
                live.publish(
                    t, out_, live_safety_,
                    pose={"frame": "map", "x_m": float(rear[0]), "y_m": float(rear[1]), "yaw_rad": float(rear[2]),
                          "correction_version": None if obstacle is None else int(obstacle["version"]),
                          "source": "slam" if slam is not None else "odom"},
                    freshness=None,
                    applied={"speed_mps": float(drive.speed_mps), "curvature_inv_m": float(drive.curvature_inv_m),
                             "steering_rad": [float(v) for v in steering_command]},
                    speed_mps=float(speed_now_),
                    # measured, not commanded: the joints as they stand (contract 6)
                    steering_rad=[float(v) for v in robot.get_joint_positions()[steers]],
                    lift_m=float(robot.get_joint_positions()[lift_index[0]]))
            if dwell_standing and wheel_lock["q"] is None:
                # Plan D8b: hold the wheels where they stand for the dwell (DWELL_WHEEL_KP).
                controller_ = robot.get_articulation_controller()
                kps_, kds_ = (np.asarray(g, dtype=float).copy() for g in controller_.get_gains())
                wheel_lock.update(q=np.asarray(robot.get_joint_positions()[wheels], dtype=float).copy(), gains=(kps_, kds_))
                locked_kps = kps_.copy()
                locked_kps[wheels] = DWELL_WHEEL_KP
                controller_.set_gains(kps=locked_kps, kds=kds_)
                wheel_lock["events"].append({"lock_s": float(t), "q": wheel_lock["q"].tolist()})
            elif not dwell_standing and wheel_lock["q"] is not None:
                robot.get_articulation_controller().set_gains(kps=wheel_lock["gains"][0], kds=wheel_lock["gains"][1])
                wheel_lock["events"][-1]["release_s"] = float(t)
                wheel_lock["q"] = None
            robot.apply_action(
                isaac.action(
                    joint_velocities=np.asarray(drive.wheel_rates_rad_s),
                    joint_positions=wheel_lock["q"],
                    joint_indices=wheels,
                )
            )
            robot.apply_action(
                isaac.action(
                    joint_positions=steering_command, joint_indices=steers
                )
            )
            robot.apply_action(
                isaac.action(
                    joint_positions=np.array([ms.lift_command]), joint_indices=lift_index
                )
            )
            core.refresh_targets()  # M-a4: the targets list follows the mission (no port call)
            if safety_file is not None:
                safety_file.write(json.dumps(core.safety_state(t)) + "\n")  # M-a3 (plan E8): every control tick
            if cancel_stop_:
                break  # M-a3 (plan E6): cancelled standing, its last stop output applied
            if "chase" in extra_cameras and step % fps_divisor == 0:
                chase_eye, chase_target, chase_yaw = MISSION_VIEWS.chase_pose(
                    base, yaw, chase_yaw, fps_divisor / 120
                )
                isaac.place_chase_camera(extra_cameras["chase"], chase_eye, chase_target)
            if slam is not None:
                slam["last_command"] = requested_speed
            near_done = state.get("near_capture", {}).get("status") == "done"
            if recorder is not None:
                recorder.phase = ms.phase if ms.phase in pocket_recorder.PHASES else "other"
            pocket_frames = (
                pocket["check"] is not None
                or (recorder is not None and near_done)
                or (near is not None and near["tracker"] is not None)
            ) and ms.phase in ("approach", "insert")
            if pocket_frames:
                pocket["history"].append((t, tuple(float(v) for v in rear)))
                del pocket["history"][:-120]
                # Plan D8b: the pose with its own time (D5 keeps its history as it was).
                pocket["control_stamped"] = (rear_stamp, tuple(float(v) for v in rear))
            # Leg switches the trackers made this tick, stamped with this tick's time (V5 cusp
            # arrivals: the error at the switch); a replaced tracker is drained on replacement and the
            # rest at the end (TrackerBook).
            trackers.collect()
            capturing_ = core.capture_live is not None and core.capture_live.raw is None
            stepper["fn"]((pocket_frames and step % 12 == 0) or capturing_)
            if capturing_:
                # Real-time plan D9c: right after its render step, before the next tick clears the step's STOP.
                stepped_ = core.capture_stepped(bool(live.stopped_in_tick))
                if getattr(stepped_, "aborted", False) or core.capture_live is None or core.capture_live.raw is not None:
                    live.capturing = False
            if pocket_frames and step % 12 == 0:
                read_pocket_frame()
            if step % 12 == 0:
                sample = {
                    "time_s": t,
                    "phase": ms.phase,
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
            if boundary is not None:
                boundary.recorder.tick_end(
                    outputs={
                        "candidate": {"speed_mps": requested_speed, "curvature_inv_m": curvature},
                        "holds": {"dwell": dwell_hold, "cusp": cusp_hold, "safety": safety_hold},
                        "phase": ms.phase,
                        "speed_mps": drive.speed_mps,
                        "curvature_inv_m": drive.curvature_inv_m,
                        "wheel_rates_rad_s": np.asarray(drive.wheel_rates_rad_s),
                        "steering_command": np.asarray(steering_command),
                        "lift_command": ms.lift_command,
                    },
                    branches=branch_probe(),
                )
        if early_ports and not boundary.recorder.in_tick:
            # Format 2: the loop ran out of time between ticks -- the end's own reads go in a closing tick.
            boundary.recorder.tick_begin(-3, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
        if ms.phase in ("repeat_capture_done", "repeat_target_not_reached", "near_tracking_stopped"):
            return
        if core.commands.status == "cancelled":
            state["mission_result"] = "cancelled"  # M-a3 (plan E6): an outcome of its own, not a time-out
            require(False, "mission_cancelled")
        require(ms.phase == "complete", "Mission exceeded simulation time budget")
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
        try:
            if recorder is not None:
                # Plan D8b: first in the finally block, so neither an earlier record nor the
                # encoders can stop it, and guarded, so a failed write never replaces the
                # run's own failure (Codex D8b impl P1).
                try:
                    state["pocket_recording"] = recorder.close(
                        {
                            "plan": "D8b (docs/plans/2026-10-04-lidar-obstacle-map.md)",
                            "camera": state.get("pocket_camera"),
                            "mount": state.get("perception_mount"),
                            "lift_joint": "fork_lift",
                            "forklift_urdf": str(args.forklift_urdf),
                            "pallet_geometry": str(args.pallet_geometry),
                            "rear_axle_offset_m": args.rear_axle_offset_m,
                            "approach_straight_speed_mps": args.approach_straight_speed_mps,
                            "dwell_gaps_m": None if args.record_dwell_gaps is None else list(args.record_dwell_gaps),
                            "dwell_hold_s": None if dwells is None else dwells.hold_s,
                        "dwell_wheel_kp": DWELL_WHEEL_KP if dwells is not None else None,
                        "dwell_wheel_locks": wheel_lock["events"],
                            "dwell_d_end_m": dwell_d_end_m,
                            "safety_stop_gaps_m": (None if args.measure_safety_stops_m is None
                                                   else list(args.measure_safety_stops_m)),
                            "safety_stops": [] if safety_stops is None else safety_stops.records,
                            "pocket_check": args.pocket_check,
                            "video": args.video,
                            "fps": args.fps,
                            "depth": "normalised optical-axis metres, float32, invalid NaN (adapter.normalize_depth)",
                            "truth": "base_pose/pallet_pose: physics position + quaternion wxyz per tick; "
                            "camera_prim_pose: Camera.get_world_pose(camera_axes='world'), diagnosis only",
                            "near_capture": state.get("near_capture"),
                            "approach_path": state.get("paths", {}).get("approach"),
                        },
                        [] if dwells is None else dwells.records,
                    )
                except Exception as exc:  # noqa: BLE001
                    state["pocket_recording_close_error"] = repr(exc)
                    traceback.print_exc()
                    # A recording run without its records measured nothing: fail it, never
                    # replacing a failure already raised (main() records that one).
                    state["success"] = False
                    state.setdefault("failure_reason", f"pocket_recording_close_failed: {exc!r}")
            if not early_ports:
                close_boundary_record(state, sys.exc_info()[1])
            if tracker_book["book"] is not None:
                try:
                    tracker_book["book"].close()  # the leg switches not yet drained (V5 cusp arrivals)
                except Exception as exc_:  # noqa: BLE001 -- never replaces the run's own failure
                    state.setdefault("record_errors", []).append(f"leg switches: {exc_!r}")
            if near is not None:
                # Plan D8 S4a ⑥: every frame, tick and event of the near-field tracking; the
                # summary in result.json. Guarded: a failed write never replaces the run's own
                # failure.
                try:
                    (args.output / "near_tracking.json").write_text(
                        record_json({"frames": near["frames"], "ticks": near["ticks"], "events": near["events"],
                                     "stop_events": near.get("stop_events") or [], "protect": near.get("protect"),
                                     "corrections": near["corrections"], "path_revisions": near["path_revisions"]}) + "\n"
                    )
                    fed_ = [f for f in near["frames"] if "skipped" not in f]
                    state["near_tracking"].update(
                        frames_fed=len(fed_),
                        frames_skipped={
                            k_: sum(1 for f in near["frames"] if f.get("skipped") == k_)
                            for k_ in ("no_time", "future", "not_new", "uncovered", "bad_depth")
                            + (("dropped",) if args.near_drop_frames_from_m is not None else ())
                        },
                        accepted=sum(1 for f in fed_ if f.get("accepted")),
                        rejected=sum(1 for f in fed_ if f.get("rejected")),
                        handoff_s=next((f["read_s"] for f in fed_ if f.get("mode") == "roof"), None),
                        armed_s=next((e["time_s"] for e in near["events"] if e["event"] == "armed"), None),
                        **({"drop_latch": near.get("drop_latch")} if args.near_drop_frames_from_m is not None else {}),
                        stops=[e for e in near["events"]
                               if e["event"] in ("stop", "release", "terminal_standing", "insert_end_arrival")],
                        protect=near.get("protect"),
                        stop_events=near.get("stop_events") or [],
                        mismatch=bool(near.get("mismatch")),
                        unbounded=bool(near.get("unbounded")),
                    )
                except Exception as exc:  # noqa: BLE001
                    state["near_tracking_write_error"] = repr(exc)
                    traceback.print_exc()
                    # A tracking run without its record shows nothing: fail it, never replacing
                    # a failure already raised (Codex S4a-1 review P2).
                    state["success"] = False
                    state.setdefault("failure_reason", f"near_tracking_record_failed: {exc!r}")
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
        finally:
            live = locals().get("live")  # unbound when the setup failed before the live runtime was made
            if locals().get("core") is not None and core.jobs is not None:
                try:
                    core.jobs.close()  # real-time plan D8: the planning worker ends with the run
                    state["async_plan_worker"]["restarts"] = core.jobs.restarts
                except Exception as exc_:  # noqa: BLE001 -- never replaces the run's own end
                    state["async_plan_worker_close_error"] = repr(exc_)
            if live is not None:
                try:
                    live.close()  # M-c: close frames to every client, the port file says ended (or stop_failed)
                except Exception as exc_:  # noqa: BLE001 -- never replaces the run's own end
                    state["live_close_error"] = repr(exc_)
            # M-a3: the lifecycle ends with the run, once every clean-up has run -- done only when the run
            # succeeded and nothing is propagating, failed otherwise; the loop's own time (no new clock read: the
            # replay compares every port call).
            core.commands.finish("done" if state.get("success") is True and sys.exc_info()[1] is None else "failed",
                                 book_clock["t"])
            if mission_schedule is not None:
                state["mission_commands"] = {"log": core.commands.log, "pauses": core.commands.pauses,
                                             "status": core.commands.status, "generation": core.commands.mission_gen}
            if safety_file is not None:
                # The last line carries the lifecycle's end (done, failed, cancelled); never in place of the run's
                # own end, nor in the way of the record's close below.
                try:
                    safety_file.write(json.dumps(core.safety_state(book_clock["t"])) + "\n")
                except Exception as exc_:  # noqa: BLE001
                    state["mission_safety_error"] = repr(exc_)
                try:
                    safety_file.close()  # a failed flush is recorded, never raised over the run's own end
                except Exception as exc_:  # noqa: BLE001
                    state.setdefault("mission_safety_error", repr(exc_))
            if early_ports:  # format 2: closed last, so the summaries above read the services inside the exit tick
                try:
                    locals_ = locals()
                    projection_ = final_projection(
                        {n_: locals_[n_] for n_ in PROJECTION_CONTAINERS if n_ in locals_}, ms, args.output)
                except Exception as exc_:  # noqa: BLE001 -- never replaces the run's own end; the replay sees it
                    projection_ = {"projection_error": repr(exc_)}
                close_boundary_record(state, sys.exc_info()[1], final_state=projection_)


def main() -> None:
    args = arguments()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    import yaml

    settings = yaml.safe_load(args.settings.read_text())
    require(
        settings["physics_hz"] == 120 or (args.steering_probe and settings["physics_hz"] in (120, 240)),
        "This adapter requires 120Hz physics (240 Hz only for a --steering-probe convergence check)",
    )
    require(
        all(
            isinstance(v, (int, float)) and math.isfinite(v) and v > 0
            for v in settings.values()
        ),
        "Settings must be positive finite values",
    )
    require(
        args.steering_probe or not any(k in settings for k in DIAGNOSTIC_PROBE_SETTINGS),
        f"{', '.join(k for k in DIAGNOSTIC_PROBE_SETTINGS if k in settings)}: diagnostic settings, --steering-probe only",
    )
    from chassis_contract import require_curvature_within_model
    from insertion_geometry import read_carriage_limit_m, read_drive_geometry_m

    args.drive_geometry = read_drive_geometry_m(
        args.forklift_urdf, settings["max_wheel_rate_rad_s"]
    )
    args.carriage_limit_m = read_carriage_limit_m(args.forklift_urdf)
    require_curvature_within_model(settings, args.drive_geometry)
    if args.odometry_wheel_radius_m is not None:
        # A typo guard: a rolling-radius calibration moves the radius by well under 10 %.
        geometric_ = args.drive_geometry.wheel_radius_m
        require(0.9 * geometric_ <= args.odometry_wheel_radius_m <= 1.1 * geometric_,
                f"--odometry-wheel-radius-m {args.odometry_wheel_radius_m} is not within 10 % of the geometric "
                f"radius {geometric_}")
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
            # M-a3 (plan E9): not recorded at its default, so a run without commands records what it always did
            and (k != "mission_commands" or v is not None) and k not in ("mission_schedule", "mission_stop_kinds")
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
    failure_exc = None  # what left run(), for the boundary record's ending
    try:
        from isaacsim import SimulationApp

        # Nothing here plays sound. With the default backend, a job whose user has no PulseAudio server keeps an
        # Omniverse audio thread busy retrying (seen on ws1 by another job); the null backend never connects.
        # FORKLIFT_ISAAC_AUDIO=on keeps Kit's default.
        audio_args = [] if os.environ.get("FORKLIFT_ISAAC_AUDIO") == "on" else [
            "--/audio/deviceBackend=null", "--/app/audio/enabled=false"]
        state["isaac_audio_args"] = audio_args
        app = SimulationApp(
            {
                "headless": True,
                "width": 1280,
                "height": 720,
                "renderer": "RaytracedLighting",
                "multi_gpu": False,
                "extra_args": audio_args,
            }
        )
        run(app, args, settings, state)
    except BaseException as exc:
        failure_exc = exc
        state["success"] = False
        state["failure_reason"] = str(exc)
        # Plain path: the traceback names this source's own lines, never a recorded output (M-a1 replay).
        (Path(str(args.output)) / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        # Failures before the mission loop's own finally (initial planning, the bootstrap): the record's ending
        # is that exception (Codex R3/R4 review P2).
        close_boundary_record(state, failure_exc)
        state["wall_time_s"] = time.monotonic() - started
        if state.get("tyre_contact_rows") is not None:
            # (t, then per tyre FL FR RL RR: impulse x y z, contact count, mean contact x y)
            np.save(args.output / "tyre_contacts.npy", np.asarray(state.pop("tyre_contact_rows"), dtype=float))
        if app is not None and "render_mode" in state:
            import carb

            state["render_mode"]["at_end"] = carb.settings.get_settings().get(
                "/rtx/rendermode"
            )
        (args.output / "result.json").write_text(record_json(state, indent=2) + "\n")
        from forklift_core import perf as perf_

        perf_._dual_report()  # real-time plan V2: the dual run's final counts (a no-op otherwise)
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
