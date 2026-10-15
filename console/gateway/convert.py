"""ROS 2 data to console/1 messages, without importing ROS (design C2a, sections 2 and 5).

The ROS node copies what it needs out of each message into the plain records below;
everything here is arithmetic on those records so it runs in host tests. Times are the
gateway's monotonic clock in seconds, taken when a message was received.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

from console.mock_vehicle.core import API

GRID_VALUES = {"unknown": 0, "free": 1, "occupied": 2}
OCCUPIED_ABOVE = 0.65  # map_server trinary thresholds, as slam_recorder writes them
FREE_BELOW = 0.25
GRID_YAW_TOLERANCE_RAD = 1e-3
CORRECTION_STEP_M = 1e-3
CORRECTION_STEP_RAD = 1e-3
SPEED_MAX_AGE_S = 1.0
SPEED_MAX_GAP_S = 0.5  # odom poses further apart than this give no speed
BACKENDS = ("real", "isaac")

# Display-only outline of dls08_measured (sim/models/dls08_measured/parameters.yaml),
# measured from the rear axle at base_link x -0.34 (an estimate kept fixed): fork tip
# 0.95 = rear extent -0.51 + overall length 1.46, width 0.63, wheelbase 0.66, both
# wheels at 15 deg give 0.3665 1/m. The page draws the truck and sweep with it only.
DLS08_MEASURED_CHASSIS = {
    "rear_overhang_m": 0.17,
    "nose_from_rear_m": 1.29,
    "half_width_m": 0.315,
    "wheelbase_m": 0.66,
    "max_curvature_inv_m": 0.3665,
    "model_turning_radius_m": 1 / 0.3665,
}


class GridRotated(ValueError):
    """The map origin is rotated; the console draws axis-aligned grids only."""


@dataclass(frozen=True)
class PoseSample:
    frame: str  # "map" or "odom"
    x_m: float
    y_m: float
    yaw_rad: float
    received_s: float


@dataclass(frozen=True)
class GridMessage:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float
    frame: str
    data: list[int]
    received_s: float


@dataclass(frozen=True)
class ScanMessage:
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges: list[float]
    received_s: float


@dataclass
class Snapshot:
    """Latest values the node has seen; None means never received."""

    base: PoseSample | None = None  # base_link in map (or odom)
    speed_mps: float | None = None
    speed_received_s: float | None = None
    correction_version: int | None = None
    correction_age_s: float | None = None
    grid_received_s: float | None = None
    scan_received_s: float | None = None
    notes: list[str] = field(default_factory=list)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rear_axle_pose(x: float, y: float, yaw: float, rear_axle_x_m: float):
    """base_link pose to the rear-axle pose the console uses (rear_axle_x_m in base_link)."""
    return (
        x + math.cos(yaw) * rear_axle_x_m,
        y + math.sin(yaw) * rear_axle_x_m,
        yaw,
    )


def grid_cells(data, width: int, height: int) -> bytes:
    if len(data) != width * height:
        raise ValueError(f"grid has {len(data)} cells, expected {width}x{height}")
    out = bytearray(len(data))
    for i, v in enumerate(data):
        if v < 0:
            continue  # unknown stays 0
        p = v / 100.0
        if p > OCCUPIED_ABOVE:
            out[i] = GRID_VALUES["occupied"]
        elif p < FREE_BELOW:
            out[i] = GRID_VALUES["free"]
    return bytes(out)


def grid_frame(g: GridMessage, epoch: int, correction_version: int | None):
    if abs(g.origin_yaw_rad) > GRID_YAW_TOLERANCE_RAD:
        raise GridRotated(f"map origin yaw {g.origin_yaw_rad:.4f} rad")
    head = {
        "t": "grid",
        "session_epoch": epoch,
        "frame": g.frame,
        "resolution_m": g.resolution_m,
        "origin_m": [g.origin_x_m, g.origin_y_m],
        "width": g.width,
        "height": g.height,
        "values": dict(GRID_VALUES),
        "correction_version": correction_version,
    }
    return head, grid_cells(g.data, g.width, g.height)


def scan_frame(
    s: ScanMessage,
    laser: PoseSample | None,
    epoch: int,
    pose_at: str = "scan_stamp",
    delay_s: float = 0.0,
    age_s: float = 0.0,
):
    """None without a laser pose: the page could not place the points.

    pose_at says where the laser pose came from: "scan_stamp" (the transform at the
    scan's own time) or "latest" (the newest transform, delay_s after the scan, used
    when the one at the scan time is not available -- the points may be misplaced).
    """
    if laser is None:
        return None
    vals = []
    for r in s.ranges:
        ok = math.isfinite(r) and s.range_min_m <= r <= s.range_max_m
        vals.append(float(r) if ok else math.nan)
    head = {
        "t": "scan",
        "session_epoch": epoch,
        "frame": laser.frame,
        "pose": {"x_m": laser.x_m, "y_m": laser.y_m, "yaw_rad": laser.yaw_rad},
        "angle_min_rad": s.angle_min_rad,
        "angle_increment_rad": s.angle_increment_rad,
        "range_max_m": s.range_max_m,
        "dtype": "float32le",
        "count": len(vals),
        "pose_at": pose_at,
        "pose_delay_s": delay_s,
        "age_s": age_s,  # since the gateway received it: a resent scan is not fresh
    }
    return head, struct.pack(f"<{len(vals)}f", *vals)


def withdrawn_grid(epoch: int, reason: str):
    """Tells the page to drop the grid it shows (the current map cannot be sent)."""
    return {
        "t": "grid",
        "session_epoch": epoch,
        "withdrawn": True,
        "reason": reason,
    }, b""


def camera_head(
    epoch: int,
    frame_id: int,
    stamp_ns: int,
    received_s: float,
    jpeg_bytes: int,
    domain: str,
) -> dict:
    return {
        "t": "camera",
        "session_epoch": epoch,
        "frame_id": frame_id,  # one id per capture (stamp); a re-sent capture keeps it
        "acquired_mono_ns": int(received_s * 1e9),
        "capture": {"domain": domain, "stamp_ns": stamp_ns},
        "calibration_version": None,
        "format": "jpeg",
        "width": None,
        "height": None,
        "jpeg_bytes": jpeg_bytes,
    }


def speed_from_poses(prev, cur):
    """Rear-axle speed from two odom poses (stamp_s, x, y, yaw), signed along the heading.

    The Isaac bridge and the replay publish odom poses with an empty twist, so the
    twist cannot be trusted; any point on the centreline has the same along-body speed.
    None when time did not advance or the poses are too far apart.
    """
    dt = cur[0] - prev[0]
    if not 0.0 < dt <= SPEED_MAX_GAP_S:
        return None
    yaw = prev[3] + _wrap(cur[3] - prev[3]) / 2  # midpoint the short way round
    along = (cur[1] - prev[1]) * math.cos(yaw) + (cur[2] - prev[2]) * math.sin(yaw)
    return along / dt


def _finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def is_jpeg(data: bytes) -> bool:
    return data[:2] == b"\xff\xd8"


class CorrectionTracker:
    """Counts map -> odom changes: a new count when the transform has moved more than
    1 mm or 1 mrad from the last counted one (so slow drift is counted too). A display
    value only, not evidence that SLAM is healthy (provisional version)."""

    def __init__(self):
        self.last = None
        self.version = 0
        self.changed_at = None

    def update(self, xyyaw, now: float):
        if self.last is None or (
            math.hypot(xyyaw[0] - self.last[0], xyyaw[1] - self.last[1])
            > CORRECTION_STEP_M
            or abs(_wrap(xyyaw[2] - self.last[2])) > CORRECTION_STEP_RAD
        ):
            self.last = xyyaw
            self.version += 1
            self.changed_at = now
        return self.version, now - self.changed_at

    def age(self, now: float):
        return None if self.changed_at is None else now - self.changed_at


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def welcome(
    vehicle_id: str, backend: str, epoch: int, chassis: dict | None = None
) -> dict:
    if backend not in BACKENDS:
        raise ValueError(f"backend must be one of {BACKENDS}")
    caps = {
        "control": False,  # monitor only: no lease, STOP, mode, teleop or task (design D1)
        "mission": False,
        "teleop_direct": False,
        "teleop_video": False,
        "lift": False,
        "recovery_actions": [],
        "feedback": {
            "rear_axle_speed": True,
            "steering": False,
            "lift_height": False,
            "load_state": False,
        },
        "media": {
            "camera_rgb": {"max_hz": 15},
            "scan": {"max_hz": 10},
            "grid": {"max_hz": 1},
        },
        "camera_model": None,
        "chassis": dict(chassis or DLS08_MEASURED_CHASSIS),
        "provisional_limits": True,
    }
    return {
        "t": "welcome",
        "api": API,
        "vehicle_id": vehicle_id,
        "session_epoch": epoch,
        "backend": backend,
        "capabilities": caps,
    }


def _age(now: float, at: float | None):
    return None if at is None else now - at


def state(snap: Snapshot, now: float, epoch: int, seq: int, rear_axle_x_m: float):
    pose = None
    if snap.base is not None and _finite(
        snap.base.x_m, snap.base.y_m, snap.base.yaw_rad
    ):
        b = snap.base
        x, y, yaw = rear_axle_pose(b.x_m, b.y_m, b.yaw_rad, rear_axle_x_m)
        in_map = b.frame == "map"
        pose = {
            "frame": b.frame,
            "x_m": x,
            "y_m": y,
            "yaw_rad": yaw,
            "source": "slam" if in_map else "odom",
            "correction_version": snap.correction_version if in_map else None,
            "age_s": now - b.received_s,
        }
    speed = None
    if _finite(snap.speed_mps) and snap.speed_received_s is not None:
        if now - snap.speed_received_s <= SPEED_MAX_AGE_S:
            speed = snap.speed_mps
    return {
        "t": "state",
        "state_seq": seq,
        "session_epoch": epoch,
        # judged by the arbiter, which this connection does not have (design D9)
        "mode": None,
        "stop_kind": None,
        "control_gen": None,
        "armed": None,
        "arm_gen": None,
        "lease": None,
        "pose": pose,
        "rear_axle_speed_mps": speed,
        "steering_rad": None,
        "lift_m": None,
        "load": {"state": None},
        "command": None,
        "applied": None,
        "permit": None,
        "limits": None,
        "drive_allowed": None,
        "lift_allowed": None,
        "mission": None,
        "freshness": {  # contract 6 names; scan_age_s is added by the C2a draft
            "slam_correction_age_s": snap.correction_age_s,
            "grid_free_age_s": _age(now, snap.grid_received_s),
            "near_obs_age_s": None,
            "scan_age_s": _age(now, snap.scan_received_s),
        },
        "pacing": None,
        "teleop_allowed": None,
        "teleop_forbidden_reason": None,
        "notes": list(snap.notes),
    }
