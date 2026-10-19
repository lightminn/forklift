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
import time
from collections import deque
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


class AsyncSlamLink:
    """The same bridge, without waiting (real-time plan D9a): the simulator goes on while the bridge answers.

    Scans are queued in order and sent one at a time -- the next only after the last one's reply -- so the
    bridge sees exactly the lockstep stream (its keyframe gate, its /clock) and no scan is ever unanswered there.
    ``poll`` collects the replies that have arrived (each with the scan's own record and the send/receive
    clocks) and sends the next scan. A reply not in within ``deadline_s`` (wall), a queue over ``queue_max`` or a
    ``failed`` reply raises ``SlamLinkFailure``, as a lockstep exchange would. What the deadline guarantees: a poll
    whose read finds no reply, read after the deadline, ends the link. A reply found is taken however long ago
    its scan was sent -- its arrival between two polls is unknown, so the grace is the real poll gap: one tick
    while the runner runs, the whole stop on the first poll after the runner itself stood still (a SIGSTOP, a
    long stall), when the simulation stood still too (console U20, 2026-10-19: refusing such a reply after an
    8 s SIGSTOP ended the session; this replaces the read-time check of Codex D9 impl P2). The clock is read
    before the socket, so a stop between the two never charges the stop to the bridge (Codex stall-fix round 1),
    and the deadline runs from when the scan's last byte left, not from when it was queued to go (round 2: a stop
    just before ``send`` was charged to the bridge); a scan not yet out is held to ``queue_max`` instead.
    Poll gaps over the deadline are kept in ``long_poll_gaps`` (previous poll, this poll) -- a gap, not a measured
    bridge latency or runner stall. A real-time run only; batch runs keep ``SlamLinkClient.exchange``."""

    def __init__(self, client: SlamLinkClient, *, deadline_s: float = 1.0, queue_max: int = 20, clock=time.monotonic):
        self.client, self.deadline_s, self.queue_max, self.clock = client, float(deadline_s), int(queue_max), clock
        # Never wait on the socket: the lockstep client's timeout would make an empty read wait it out (Codex D9
        # impl P1); a send that does not go whole waits in _out for the next poll.
        client.sock.setblocking(False)
        self._out = b""
        self.queue: deque = deque()
        self.in_flight = None  # (scan, meta, sent_mono): sent_mono None until the last byte is out
        self._buffer = b""
        self.max_depth = 0
        self.last_reply_id: int | None = None
        self._looked: float | None = None  # the clock just before the last poll's read
        self.long_poll_gaps: list[tuple[float, float]] = []

    def enqueue(self, scan: Scan, meta: dict) -> None:
        if len(self.queue) >= self.queue_max:
            raise SlamLinkFailure(f"scan {scan.scan_id}: {len(self.queue)} scans queued behind the bridge")
        self.queue.append((scan, meta))
        self.max_depth = max(self.max_depth, len(self.queue) + (self.in_flight is not None))
        self._pump()

    def depth(self) -> int:
        return len(self.queue) + (self.in_flight is not None)

    def _pump(self) -> None:
        if self.in_flight is None and self.queue:
            scan, meta = self.queue.popleft()
            self._out += encode_scan(scan)
            self.in_flight = (scan, meta, None)
        self._flush()

    def _flush(self) -> None:
        while self._out:
            try:
                sent = self.client.sock.send(self._out)
            except (BlockingIOError, InterruptedError):
                return
            except OSError as exc:
                raise SlamLinkFailure(f"sending a scan: {exc}") from exc
            self._out = self._out[sent:]
        if self.in_flight is not None and self.in_flight[2] is None:
            self.in_flight = (*self.in_flight[:2], self.clock())  # out whole: the deadline starts now

    def _read(self) -> list[bytes]:
        try:
            while True:
                chunk = self.client.sock.recv(65536)
                if not chunk:
                    raise SlamLinkFailure("the bridge closed the connection")
                self._buffer += chunk
        except (BlockingIOError, InterruptedError):
            pass
        except OSError as exc:
            raise SlamLinkFailure(f"reading replies: {exc}") from exc
        bodies = []
        while len(self._buffer) >= 4:
            (length,) = struct.unpack(">I", self._buffer[:4])
            if len(self._buffer) < 4 + length:
                break
            bodies.append(self._buffer[4:4 + length])
            self._buffer = self._buffer[4 + length:]
        return bodies

    def poll(self) -> list:
        """The replies in since the last call: (reply, meta, sent_mono, received_mono), in order."""
        out = []
        self._flush()
        now = self.clock()  # before the read: the read happened at or after it
        if self._looked is not None and now - self._looked > self.deadline_s:
            self.long_poll_gaps.append((self._looked, now))
        self._looked = now
        bodies = self._read()
        received = self.clock() if bodies else now  # the record's receive clock: after the read
        for body in bodies:
            if self.in_flight is None:
                raise SlamLinkFailure("a reply with no scan in flight")
            scan, meta, sent = self.in_flight
            if sent is None:
                raise SlamLinkFailure(f"a reply before scan {scan.scan_id} was sent whole")
            try:
                reply = decode_reply(body)
            except (ValueError, KeyError) as exc:
                raise SlamLinkFailure(f"scan {scan.scan_id}: {exc}") from exc
            if reply.scan_id != scan.scan_id or reply.stamp_s != scan.stamp_s:
                raise SlamLinkFailure(
                    f"reply for scan {reply.scan_id} @{reply.stamp_s} to scan {scan.scan_id} @{scan.stamp_s}")
            if reply.status == "failed":
                raise SlamLinkFailure(f"bridge reported failure for scan {scan.scan_id}")
            self.in_flight = None
            self.last_reply_id = int(reply.scan_id)
            out.append((reply, meta, sent, received))
            self._pump()
        if (self.in_flight is not None and self.in_flight[2] is not None
                and now - self.in_flight[2] > self.deadline_s):  # not there at a read after it
            raise SlamLinkFailure(
                f"scan {self.in_flight[0].scan_id}: no reply within {self.deadline_s:.2f} s")
        return out


__all__ = [
    "AsyncSlamLink",
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
