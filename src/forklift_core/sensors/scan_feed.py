"""The real LiDAR scan's host side (docs/design/2026-11-12-real-scan-obstacle-layer.md 1.2-1.4).

The ROS container's adapter sends one ``scan/1`` frame per driver message over a Unix socket; this module decodes
it, checks it, turns invalid ranges into NaN (unknown, never free), builds the beam angles per scan and converts the
driver stamp to the monotonic clock through the adapter's (system - monotonic) pair. A step in that pair (a system
clock adjustment) refuses the scan and the conversion is established again from ten scans that agree.

The driver stamp is not the acquisition time (the upstream node stamps before it fetches a scan that may have
waited in a buffer): the delay here is named stamp-to-receipt, never the observation's age (design 1.1). Nothing
here imports ROS or a serial library (K2)."""

from __future__ import annotations

import base64
import json
import math
import os
import socket
import stat
import struct
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAGIC = b"SCN1"
_LEN = struct.Struct("<I")
MAX_BEAMS = 4096
ESTABLISH_COUNT = 10  # scans that must agree before the clock pair is used
ESTABLISH_TOL_NS = 1_000_000  # ... each within 1 ms of their median (two clocks read apart: never identical)
JUMP_NS = 50_000_000  # a step past this from the last accepted pair is a system clock adjustment
NEGATIVE_LATENCY_NS = -5_000_000  # stamped this much after its receipt: refused
RECV_BYTES = 1 << 16


class FrameError(ValueError):
    """Not a scan/1 frame."""


@dataclass(frozen=True)
class RealScan:
    scan_id: int
    stamp_mono_ns: int  # the driver stamp on the monotonic clock (not the acquisition time, design 1.1)
    recv_mono_ns: int  # the adapter's receipt
    stamp_to_recv_ns: int
    angles: np.ndarray  # radians, zero ahead, positive to the left (ROS), one per beam of this scan
    ranges: np.ndarray  # metres; NaN where the driver gave no valid range (unknown)
    intensities: np.ndarray | None
    meta: dict  # frame_id, scan_time, time_increment, range_min, range_max, sys_minus_mono_ns


def encode_frame(head: dict, ranges, intensities=None) -> bytes:
    """A scan/1 frame: magic, the head's length, the head (JSON), the ranges and the intensities (f32 LE)."""
    h = dict(head)
    h["has_intensities"] = intensities is not None
    body = np.asarray(ranges, dtype="<f4").tobytes()
    if intensities is not None:
        body += np.asarray(intensities, dtype="<f4").tobytes()
    raw = json.dumps(h).encode("utf-8")
    return MAGIC + _LEN.pack(len(raw)) + raw + body


def decode_frame(data: bytes) -> tuple[dict, np.ndarray, np.ndarray | None]:
    """The head and the arrays; ``FrameError`` for anything that is not a whole scan/1 frame. The head's values
    are not checked here (``ScanFeed`` does), only the frame's shape -- except a beam count past ``MAX_BEAMS``,
    which is answered before its body is read."""
    if len(data) < len(MAGIC) + _LEN.size or data[: len(MAGIC)] != MAGIC:
        raise FrameError("bad_frame")
    (hlen,) = _LEN.unpack_from(data, len(MAGIC))
    start = len(MAGIC) + _LEN.size
    if start + hlen > len(data):
        raise FrameError("bad_frame")
    try:
        head = json.loads(data[start : start + hlen].decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        raise FrameError("bad_frame") from None
    if not isinstance(head, dict):
        raise FrameError("bad_frame")
    n = head.get("n")
    if not (isinstance(n, int) and not isinstance(n, bool) and n >= 0):
        raise FrameError("bad_meta")
    if n > MAX_BEAMS:
        raise FrameError("too_many_beams")
    arrays = 2 if head.get("has_intensities") else 1
    body = data[start + hlen :]
    if len(body) != 4 * n * arrays:
        raise FrameError("bad_frame")
    ranges = np.frombuffer(body[: 4 * n], dtype="<f4").astype(float)
    inten = np.frombuffer(body[4 * n :], dtype="<f4").astype(float) if arrays == 2 else None
    return head, ranges, inten


I64 = 1 << 63
NUM_MAX = 1e9  # no scan field (radians, seconds, metres) is anywhere near this


def _int(v, lo: int = -I64, hi: int = I64) -> bool:
    return isinstance(v, int) and not isinstance(v, bool) and lo <= v < hi


def _num(v) -> bool:
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return False
    try:
        f = float(v)
    except OverflowError:  # an int too large for a float (Codex 7a P2-7)
        return False
    return math.isfinite(f) and abs(f) < NUM_MAX


def _meta_ok(h: dict) -> bool:
    if not (_int(h.get("scan_id"), 0) and _int(h.get("ros_stamp_ns"), 0) and _int(h.get("recv_mono_ns"), 0)
            and _int(h.get("sys_minus_mono_ns"))):
        return False
    if not h.get("n", 0) >= 2:  # an empty or one-beam scan has no angle step to read (Codex 7a P2-7)
        return False
    if not all(_num(h.get(k)) for k in ("angle_min", "angle_increment", "time_increment", "scan_time",
                                        "range_min", "range_max")):
        return False
    if not (h["angle_increment"] > 0 and 0 <= h["range_min"] < h["range_max"]):
        return False
    if h["scan_time"] < 0 or h["time_increment"] < 0:
        return False
    return isinstance(h.get("frame_id"), str)


class ScanFeed:
    """``judge`` one frame -> (RealScan or None, why refused or None); ``poll`` a non-blocking socket for every frame
    waiting. ``refused`` counts every reason; ``record`` (a text file) keeps each received frame before its verdict
    with the verdict, so ``replay`` reproduces the same accept/refuse order (design 1.4, Codex 2nd-ed. P2-2)."""

    def __init__(self, *, sock: socket.socket | None = None, record=None) -> None:
        self.sock = sock
        if sock is not None:
            sock.setblocking(False)
        self.record = record
        self.refused: Counter = Counter()
        self.accepted = 0
        self._last_pair: int | None = None  # the last accepted scan's (system - monotonic)
        self._window: list[int] = []  # establishing: the latest pairs (at most ESTABLISH_COUNT)
        self.establishing = True
        self._last_id: int | None = None
        self.id_gaps = 0  # scan_ids missing between received frames: the adapter's (or the socket's) losses

    # ------------------------------------------------------------------ the clock pair
    def _establish(self, pair: int) -> bool:
        """Establishing: the latest ESTABLISH_COUNT pairs, all within ESTABLISH_TOL_NS of their median, establish
        the conversion (a rolling window -- alternating jitter inside the band is fine, Codex 7a P2-2); a pair
        JUMP_NS off the window's median starts it again from that pair."""
        if self._window and abs(pair - int(np.median(self._window))) > JUMP_NS:
            self._window = []
        self._window = (self._window + [pair])[-ESTABLISH_COUNT:]
        if len(self._window) < ESTABLISH_COUNT:
            return False
        med = int(np.median(self._window))
        if any(abs(p - med) > ESTABLISH_TOL_NS for p in self._window):
            return False
        # established with its reference: the agreeing ten's median -- set here, so a refusal of this very frame
        # (its latency) still leaves one for the next (Codex 7a re-review P2)
        self.establishing, self._window, self._last_pair = False, [], med
        return True

    # ------------------------------------------------------------------ one frame
    def judge(self, data: bytes, host_recv_ns: int) -> tuple[RealScan | None, str | None]:
        scan, why = self._judge(data)
        if why is None:
            self.accepted += 1
        else:
            self.refused[why] += 1
        if self.record is not None:
            self.record.write(json.dumps({"host_recv_ns": int(host_recv_ns),
                                          "frame": base64.b64encode(data).decode("ascii"), "verdict": why}) + "\n")
        return scan, why

    def _judge(self, data: bytes) -> tuple[RealScan | None, str | None]:
        try:
            h, ranges, inten = decode_frame(data)
        except FrameError as exc:
            return None, str(exc)
        sid = h.get("scan_id")
        if _int(sid, 0):  # every received frame, accepted or not, for the gap count (Codex 7a P2-3)
            if self._last_id is not None and sid > self._last_id + 1:
                self.id_gaps += sid - self._last_id - 1
            self._last_id = sid if self._last_id is None else max(self._last_id, sid)
        if not _meta_ok(h):
            return None, "bad_meta"
        pair = int(h["sys_minus_mono_ns"])
        stamp_mono = int(h["ros_stamp_ns"]) - pair
        latency = int(h["recv_mono_ns"]) - stamp_mono
        # the reference moves only with an accepted scan (Codex 7a P2-1): judge first, then update
        if self.establishing:
            if not self._establish(pair):
                return None, "establishing"
        elif abs(pair - self._last_pair) > JUMP_NS:
            self.establishing, self._window = True, []
            return None, "clock_jump"
        if latency < NEGATIVE_LATENCY_NS:
            return None, "negative_latency"
        self._last_pair = pair
        n = int(h["n"])
        angles = float(h["angle_min"]) + float(h["angle_increment"]) * np.arange(n)
        with np.errstate(invalid="ignore"):
            valid = np.isfinite(ranges) & (ranges >= float(h["range_min"])) & (ranges <= float(h["range_max"]))
        r = np.where(valid, ranges, np.nan)
        meta = {k: h[k] for k in ("frame_id", "scan_time", "time_increment", "range_min", "range_max",
                                  "sys_minus_mono_ns")}
        return RealScan(int(h["scan_id"]), stamp_mono, int(h["recv_mono_ns"]), latency, angles, r, inten, meta), None

    # ------------------------------------------------------------------ the socket
    def poll(self) -> list[RealScan]:
        """Every frame waiting, at once; never blocks."""
        out = []
        if self.sock is None:
            return out
        while True:
            try:
                data = self.sock.recv(RECV_BYTES)
            except (BlockingIOError, InterruptedError):
                break
            if not data:
                break
            scan, _why = self.judge(data, time.monotonic_ns())
            if scan is not None:
                out.append(scan)
        return out


def bind_socket(path: str | Path, *, rcvbuf: int = 1 << 20) -> socket.socket:
    """The receiving end the adapter sends to: a non-blocking Unix datagram socket bound at ``path`` (a stale path a
    crash left behind is replaced), group-writable so the container's user can send. A datagram is a whole frame or
    none -- a full receiver makes the adapter drop, never block (design 1.2)."""
    p = Path(path)
    if p.is_symlink() or (p.exists() and not stat.S_ISSOCK(p.stat().st_mode)):
        raise FileExistsError(f"{p} exists and is not a socket")  # never delete a file given by mistake
    if p.exists():
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            probe.connect(str(p))
        except ConnectionRefusedError:
            p.unlink()  # a socket nobody holds: a crash left it behind
        else:
            raise FileExistsError(f"{p} is held by a running receiver")  # never steal a live one's path
        finally:
            probe.close()
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(rcvbuf))
    s.bind(str(p))
    os.chmod(p, 0o660)
    s.setblocking(False)
    return s


def replay(path: str | Path) -> dict:
    """A record's frames through a new ScanFeed, in order: ``verdicts`` (now) beside ``recorded`` (then)."""
    feed = ScanFeed()
    recorded, verdicts = [], []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        recorded.append(row["verdict"])
        verdicts.append(feed.judge(base64.b64decode(row["frame"]), int(row["host_recv_ns"]))[1])
    return {"recorded": recorded, "verdicts": verdicts, "feed": feed}


__all__ = ["ESTABLISH_COUNT", "bind_socket", "FrameError", "MAX_BEAMS", "RealScan", "ScanFeed", "decode_frame", "encode_frame",
           "replay"]
