"""Messages and client for the Isaac <-> SLAM bridge lockstep.

Plan: docs/plans/2026-10-04-online-slam-closed-loop.md. The simulator sends
one ``Scan`` per LiDAR period and does not advance physics until the bridge
answers with the ``Reply`` for that same scan id: ``processed`` carries the
map<-odom slam_toolbox's /pose for that scan stamp implies, ``skipped`` the
previous correction for a scan slam_toolbox does not process (``ScanGate``
predicts which, exactly as slam_toolbox decides -- plan v3.3), ``failed`` ends
the run. Framing: a 4-byte big-endian length, then a JSON header line and,
for scans, the float32 ranges. Plain sockets and numpy only (no ROS here).
"""

from __future__ import annotations

import json
import math
import socket
import struct
from dataclasses import dataclass

import numpy as np

STATUSES = ("processed", "skipped", "failed")


class SlamLinkFailure(RuntimeError):
    """The bridge did not answer this scan correctly in time."""


class ScanGate:
    """slam_toolbox 2.8.5's SlamToolbox::shouldProcessScan, reproduced.

    Plan v3.3: the online run keeps the validated replay thresholds (0.5 m,
    0.5 rad, 0.5 s) instead of processing every scan, so the bridge must know
    which scans get a /pose. Same order and arithmetic as the C++ (source
    src/slam_toolbox_common.cpp, tag 2.8.5): first scan passes; throttle by
    count; time since the last accepted scan (integer nanoseconds, as
    rclcpp::Time); the first four scans are dropped; then squared distance of
    the odom->base poses against 0.8 x minimum_travel_distance^2 (or, in the
    precise mode, distance and heading both below their minimums). With equal
    time intervals in the node and Karto's HasMovedEnough, an accepted scan
    always passes Karto's check too (its time test returns first). The pause
    service is never used here.
    """

    def __init__(
        self,
        *,
        minimum_travel_distance: float,
        minimum_travel_heading: float,
        minimum_time_interval: float,
        throttle_scans: int = 1,
        check_min_dist_and_heading_precisely: bool = False,
    ):
        self.min_dist2 = float(minimum_travel_distance) * float(minimum_travel_distance)
        self.min_rotation = float(minimum_travel_heading)
        self.min_interval_ns = int(round(float(minimum_time_interval) * 1e9))
        self.throttle = int(throttle_scans)
        self.precise = bool(check_min_dist_and_heading_precisely)
        self.scan_ctr = 0
        self.first = True
        self.last_pose = None
        self.last_ns = 0

    @classmethod
    def from_params(cls, params: dict) -> "ScanGate":
        return cls(
            minimum_travel_distance=params["minimum_travel_distance"],
            minimum_travel_heading=params["minimum_travel_heading"],
            minimum_time_interval=params["minimum_time_interval"],
            throttle_scans=params.get("throttle_scans", 1),
            check_min_dist_and_heading_precisely=params.get(
                "check_min_dist_and_heading_precisely", False
            ),
        )

    def will_process(self, stamp_s: float, odom_from_base) -> bool:
        stamp_ns = int(round(float(stamp_s) * 1e9))
        pose = tuple(float(v) for v in odom_from_base)
        self.scan_ctr += 1
        if self.first:
            self.first, self.last_pose, self.last_ns = False, pose, stamp_ns
            return True
        if self.scan_ctr % self.throttle != 0:
            return False
        if stamp_ns - self.last_ns < self.min_interval_ns:
            return False
        if self.scan_ctr < 5:
            return False
        dx, dy = pose[0] - self.last_pose[0], pose[1] - self.last_pose[1]
        dist2 = dx * dx + dy * dy
        if self.precise:
            turn = pose[2] - self.last_pose[2]
            heading = abs(math.atan2(math.sin(turn), math.cos(turn)))
            if dist2 < self.min_dist2 and heading < self.min_rotation:
                return False
        elif dist2 < 0.8 * self.min_dist2:
            return False
        self.last_pose, self.last_ns = pose, stamp_ns
        return True


@dataclass(frozen=True)
class Scan:
    scan_id: int
    stamp_s: float
    odom_from_base: tuple
    ranges_m: np.ndarray
    angle_min_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float


@dataclass(frozen=True)
class Reply:
    scan_id: int
    stamp_s: float
    status: str
    map_from_odom: tuple
    version: int
    slam_wall_s: float

    def __post_init__(self) -> None:
        if self.status not in STATUSES:
            raise ValueError(f"unknown reply status {self.status!r}")


def _frame(header: dict, payload: bytes = b"") -> bytes:
    body = json.dumps(header, sort_keys=True).encode() + b"\n" + payload
    return struct.pack(">I", len(body)) + body


def encode_scan(scan: Scan) -> bytes:
    ranges = np.ascontiguousarray(scan.ranges_m, dtype=np.float32)
    header = {
        "type": "scan",
        "scan_id": int(scan.scan_id),
        "stamp_s": float(scan.stamp_s),
        "odom_from_base": [float(v) for v in scan.odom_from_base],
        "angle_min_rad": float(scan.angle_min_rad),
        "angle_increment_rad": float(scan.angle_increment_rad),
        "range_min_m": float(scan.range_min_m),
        "range_max_m": float(scan.range_max_m),
        "count": int(ranges.size),
    }
    return _frame(header, ranges.tobytes())


def decode_scan(body: bytes) -> Scan:
    line, _, payload = body.partition(b"\n")
    header = json.loads(line)
    if header.get("type") != "scan":
        raise ValueError("not a scan message")
    ranges = np.frombuffer(payload, dtype=np.float32)
    if ranges.size != header["count"]:
        raise ValueError("scan payload length does not match its header")
    return Scan(
        header["scan_id"],
        header["stamp_s"],
        tuple(header["odom_from_base"]),
        ranges.copy(),
        header["angle_min_rad"],
        header["angle_increment_rad"],
        header["range_min_m"],
        header["range_max_m"],
    )


def encode_reply(reply: Reply) -> bytes:
    return _frame(
        {
            "type": "reply",
            "scan_id": int(reply.scan_id),
            "stamp_s": float(reply.stamp_s),
            "status": reply.status,
            "map_from_odom": [float(v) for v in reply.map_from_odom],
            "version": int(reply.version),
            "slam_wall_s": float(reply.slam_wall_s),
        }
    )


def decode_reply(body: bytes) -> Reply:
    header = json.loads(body.partition(b"\n")[0])
    if header.get("type") != "reply":
        raise ValueError("not a reply message")
    return Reply(
        header["scan_id"],
        header["stamp_s"],
        header["status"],
        tuple(header["map_from_odom"]),
        header["version"],
        header["slam_wall_s"],
    )


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks, remaining = [], size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("peer closed the connection")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_body(sock: socket.socket) -> bytes:
    (length,) = struct.unpack(">I", _recv_exact(sock, 4))
    return _recv_exact(sock, length)


def recv_scan(sock: socket.socket) -> Scan:
    return decode_scan(recv_body(sock))


class SlamLinkClient:
    """The simulator side: one blocking exchange per scan."""

    def __init__(self, path: str, *, timeout_s: float):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout_s)
        self.sock.connect(path)

    def exchange(self, scan: Scan) -> Reply:
        try:
            self.sock.sendall(encode_scan(scan))
            reply = decode_reply(recv_body(self.sock))
        except (OSError, ConnectionError, ValueError) as exc:
            raise SlamLinkFailure(f"scan {scan.scan_id}: {exc}") from exc
        if reply.scan_id != scan.scan_id or reply.stamp_s != scan.stamp_s:
            raise SlamLinkFailure(
                f"reply for scan {reply.scan_id} @{reply.stamp_s} to scan {scan.scan_id} @{scan.stamp_s}"
            )
        if reply.status == "failed":
            raise SlamLinkFailure(f"bridge reported failure for scan {scan.scan_id}")
        return reply

    def close(self) -> None:
        self.sock.close()


__all__ = [
    "Reply",
    "STATUSES",
    "Scan",
    "SlamLinkClient",
    "ScanGate",
    "SlamLinkFailure",
    "decode_reply",
    "decode_scan",
    "encode_reply",
    "encode_scan",
    "recv_body",
    "recv_scan",
]
