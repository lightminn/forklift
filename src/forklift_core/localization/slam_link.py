"""Messages and client for the Isaac <-> SLAM bridge lockstep.

Plan: docs/plans/2026-10-04-online-slam-closed-loop.md. The simulator sends
one ``Scan`` per LiDAR period and does not advance physics until the bridge
answers with the ``Reply`` for that same scan id: ``processed`` carries the
map<-odom slam_toolbox's /pose for that scan stamp implies, ``skipped`` the
previous correction for a scan the bridge keeps to itself (not a keyframe,
``KeyframeGate``) or that slam_toolbox drops at start-up -- plan v3.4,
``failed`` ends the run. Framing: a 4-byte big-endian length, then a JSON header line and,
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


class KeyframeGate:
    """Which scans the bridge sends to slam_toolbox (plan v3.4).

    slam_toolbox runs with its processing thresholds at 0, so every scan it
    receives (after its own start-up drop of the 2nd-4th) is processed and
    answered with a /pose. Sending every 10 Hz scan ruined the map (job 754:
    replay ATE 0.146 m against 0.062 m with the 0.5 m / 0.5 s replay config),
    so the bridge sends only keyframes and waits for each one's /pose -- no
    scan is ever in flight unanswered, and nothing has to guess what
    slam_toolbox decided.

    Start-up: the first ``startup_scans`` (5) are all sent while the runner
    holds the drive; slam_toolbox processes the 1st and 5th. After that a scan
    is a keyframe when at least ``min_interval_s`` has passed since the last
    keyframe and the odom->base pose has travelled at least ``min_distance_m``
    (0.447 m = sqrt(0.8) x 0.5, the replay config's effective rule) or turned
    at least ``min_heading_rad`` (0.5; the replay config ignores heading, which
    left turns on the spot uncorrected) -- both summed scan to scan, so going
    back and forth cannot dodge a correction (Codex v3.3 P3). Time in integer
    nanoseconds.
    """

    def __init__(
        self,
        *,
        min_distance_m: float = math.sqrt(0.8) * 0.5,
        min_heading_rad: float = 0.5,
        min_interval_s: float = 0.5,
        startup_scans: int = 5,
    ):
        self.min_distance2 = float(min_distance_m) ** 2
        self.min_heading = float(min_heading_rad)
        self.min_interval_ns = int(round(float(min_interval_s) * 1e9))
        self.startup_scans = int(startup_scans)
        self.sent = 0
        self.reference = None  # stamp_ns of the last processed keyframe
        self.previous = None  # pose of the previous scan seen
        self.travel_m = 0.0  # summed since the last processed keyframe
        self.turn_rad = 0.0

    def classify(self, stamp_s: float, odom_from_base) -> str:
        """'expect_pose', 'startup_drop' (sent, no /pose expected) or 'local'."""
        pose = tuple(float(v) for v in odom_from_base)
        if self.previous is not None:
            turn = pose[2] - self.previous[2]
            self.travel_m += math.hypot(pose[0] - self.previous[0], pose[1] - self.previous[1])
            self.turn_rad += abs(math.atan2(math.sin(turn), math.cos(turn)))
        self.previous = pose
        if self.sent < self.startup_scans:
            self.sent += 1
            return "expect_pose" if self.sent in (1, self.startup_scans) else "startup_drop"
        stamp_ns = int(round(float(stamp_s) * 1e9))
        if stamp_ns - self.reference < self.min_interval_ns:
            return "local"
        if self.travel_m * self.travel_m < self.min_distance2 and self.turn_rad < self.min_heading:
            return "local"
        self.sent += 1
        return "expect_pose"

    def processed(self, stamp_s: float, odom_from_base) -> None:
        """The reference for the next keyframe: the last scan with a /pose."""
        self.reference = int(round(float(stamp_s) * 1e9))
        self.travel_m = self.turn_rad = 0.0


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
    "KeyframeGate",
    "SlamLinkFailure",
    "decode_reply",
    "decode_scan",
    "encode_reply",
    "encode_scan",
    "recv_body",
    "recv_scan",
]
