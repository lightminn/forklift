"""The real D435i depth stream's judgement (PR 7 7e, docs/plans/2026-11-26-7e-depth-pocket-check.md 1-2).

The SDK callback only queues (``on_frame``: the frame's bytes already copied, the SDK frame never kept -- plan 1); the
observe cycle (and, in 7e-2, the real loop) takes the queue in receipt order with ``take(now_ns)`` and gets back, in
that order, ``("frame", judged)`` and ``("gen", row)`` events.

Each frame's acquisition interval is ``[host receipt - bound, host receipt]`` (design 3.1, as the IMU source), the
bound the config's (``delay_bound_s``) or, unmeasured, its provisional one: such frames are diagnostic only. The device
time (``timestamp`` in ``timestamp_domain``) is never the interval; it decides the order, duplicates and restarts,
and the cumulative delay check: over every frame pair of one device clock, the receipt delays differ by at most the
bound + 1 ms + 100 ppm of their device time apart (min/max envelope, as ``imu_source``; plan 2, Codex plan round 2
P2-2 -- a neighbour check misses a delay that creeps).

Generations (``gen``, from 1): a receive gap (no frame for ``receive_gap_s``, at the last receipt + the gap) opens a
new one and the stream goes on -- it is decided when the next frame or loss is judged, or at ``finish``, never on a
quiet take, so the judgement does not depend on how the takes were batched (Codex 7e-1 round 1 P2-2); a device time going back (a restart), a time
domain change, the delay check broken, a queue overflow (raw frames lost before they were taken) or the device lost
open one and **stop** the source -- every later frame is ``stopped`` (the device is never reopened in a run). A
frame whose interval starts before the latest boundary is ``before_boundary``: its interval is kept as it is, and it
may have been taken before the boundary (as the IMU source; Codex 7e-1 round 1 P2-3). Verdicts: ``ok``, ``dup`` (the
same device time again), ``before_boundary``, ``stopped``.

The judgement depends on the frames' receipts and device times only, so a record's rows fed again in file order
(``push`` + ``take``; a loss with its own time) give the same verdicts, generations and intervals -- the replay's
verification pass (``replay_record``)."""

from __future__ import annotations

import threading
import time
from collections import deque

from forklift_core.sensors.depth_cfg import DepthConfig, calib_id

SLACK_NS = 1_000_000  # 1 ms
DRIFT_DIV = 10_000  # 100 ppm of a device time in ns


class DepthSource:
    def __init__(self, cfg: DepthConfig, *, operating: bool = False, now_ns=time.monotonic_ns) -> None:
        if operating and (not cfg.measured or cfg.delay_bound_s is None):
            raise ValueError("depth_source:delay_unmeasured")  # diagnostic until the bound is entered (plan 0)
        self.cfg, self.operating, self._now = cfg, operating, now_ns
        self.bound_ns = int(round(cfg.interval_bound_s * 1e9))
        self._gap_ns = int(round(cfg.receive_gap_s * 1e9))
        self._lock = threading.Lock()
        self._queue: deque = deque()  # ("f", host_ns, fields) frames and ("l", at_ns, why) losses, receipt order
        self._nf = 0
        self.overflow = 0  # frames dropped from a full queue (a record that lost raw frames is incomplete)
        self._overflow_pending = False
        # loop side
        self.gen = 1
        self.generation_why: str | None = None
        self.stopped: str | None = None
        self._boundary_ns: int | None = None
        self._last_rx: int | None = None
        self._gap_marked: int | None = None
        self._clk_dom: str | None = None
        self._last_dev: int | None = None
        self._env_lo: int | None = None  # min of (offset - drift)
        self._env_hi: int | None = None  # max of (offset + drift)
        self.last_take_ns: int | None = None
        self.counts = {"frames": 0, "ok": 0, "dup": 0, "before_boundary": 0, "stopped": 0, "overflow": 0, "lost": 0,
                       "generations": 0}

    # --- the SDK side: queue only ------------------------------------------------------------------------------------
    def on_frame(self, frame_number, ts_ms, domain, meta, units, intr, z16) -> None:
        self.push(None, frame_number, ts_ms, domain, meta, units, intr, z16)

    def push(self, host_ns, frame_number, ts_ms, domain, meta, units, intr, z16) -> None:
        """One frame as received at ``host_ns`` (None: now, read under the queue's lock -- the callback), or a
        record's fields. ``z16`` is the frame's own copy (or None for a record's re-judgement)."""
        with self._lock:
            host = int(self._now()) if host_ns is None else int(host_ns)
            if self._nf >= self.cfg.queue_max:
                self._drop_oldest()
            self._queue.append(("f", host, (frame_number, ts_ms, domain, dict(meta or {}), units,
                                            dict(intr or {}), z16)))
            self._nf += 1

    def _drop_oldest(self) -> None:
        for i, e in enumerate(self._queue):
            if e[0] == "f":
                del self._queue[i]
                self._nf -= 1
                self.overflow += 1
                self._overflow_pending = True
                return

    def lost(self, why: str, at_ns: int | None = None) -> None:
        """The device gone (any thread): a final boundary after the frames received so far."""
        with self._lock:
            self._queue.append(("l", int(self._now()) if at_ns is None else int(at_ns), str(why)))

    def pending(self) -> int:
        with self._lock:
            return len(self._queue)

    # --- the loop side ------------------------------------------------------------------------------------------------
    def take(self, now_ns: int, limit: int | None = None) -> list:
        """At most ``limit`` queued items judged in receipt order (with a quiet receive gap seen at ``now_ns``)."""
        with self._lock:
            n = len(self._queue) if limit is None else min(int(limit), len(self._queue))
            items = [self._queue.popleft() for _ in range(n)]
            self._nf -= sum(1 for e in items if e[0] == "f")
            overflow, self._overflow_pending = self._overflow_pending, False
        out: list = []
        if overflow:
            self.counts["overflow"] = self.overflow
            self._boundary(out, "overflow", self._last_rx if self._last_rx is not None else int(now_ns), final=True)
        for e in items:
            if e[0] == "l":
                self.counts["lost"] += 1
                self._gap_before(out, e[1])
                self._boundary(out, f"lost:{e[2]}", e[1], final=True)
                continue
            self._frame(out, e[1], e[2])
        self.last_take_ns = int(now_ns)
        return out

    def _queue_empty(self) -> bool:
        with self._lock:
            return not self._queue

    def _gap_before(self, out: list, at: int) -> None:
        """A receive gap before an item at ``at`` (or the end): one boundary at the last receipt + the gap."""
        if self._last_rx is not None and self._gap_marked != self._last_rx and at > self._last_rx + self._gap_ns:
            self._gap_marked = self._last_rx
            self._boundary(out, "receive_gap", self._last_rx + self._gap_ns, final=False)

    def _boundary(self, out: list, why: str, at_ns: int, *, final: bool) -> None:
        if self.stopped is not None:
            return  # already stopped: nothing more opens
        self.gen += 1
        self.generation_why = why
        self.counts["generations"] += 1
        self._boundary_ns = int(at_ns) if self._boundary_ns is None else max(self._boundary_ns, int(at_ns))
        if final:
            self.stopped = why
        out.append(("gen", {"t": "depth_gen", "why": why, "gen": self.gen, "host_ns": int(at_ns)}))

    def _frame(self, out: list, host: int, fields) -> None:
        frame_number, ts_ms, domain, meta, units, intr, z16 = fields
        self.counts["frames"] += 1
        self._gap_before(out, host)
        self._last_rx = host if self._last_rx is None else max(self._last_rx, host)
        verdict = self._judge(out, host, ts_ms, domain)
        lo = host - self.bound_ns
        if verdict == "ok" and self._boundary_ns is not None and lo < self._boundary_ns:
            verdict = "before_boundary"  # it may have been taken before the boundary: never given (P2-3)
        self.counts[verdict] += 1
        judged = {"host_ns": host, "frame_number": frame_number, "timestamp": ts_ms, "domain": domain, "meta": meta,
                  "units": units, "intrinsics": intr, "calib_id": calib_id(intr) if intr else None,
                  "lo": int(lo), "hi": int(host), "gen": self.gen, "verdict": verdict, "z16": z16}
        out.append(("frame", judged))

    def _judge(self, out: list, host: int, ts_ms, domain) -> str:
        if self.stopped is not None:
            return "stopped"
        dev = None if ts_ms is None else int(round(float(ts_ms) * 1e6))
        if self._clk_dom is not None and domain != self._clk_dom:
            self._boundary(out, "domain_changed", host, final=True)
            return "stopped"
        if dev is None:
            self._boundary(out, "no_device_time", host, final=True)
            return "stopped"
        if self._last_dev is not None:
            if dev == self._last_dev:
                return "dup"
            if dev < self._last_dev:
                self._boundary(out, "restart", host, final=True)
                return "stopped"
        off = host - dev
        drift = abs(dev) // DRIFT_DIV
        a, b = off - drift, off + drift
        limit = self.bound_ns + SLACK_NS
        if self._env_lo is not None and (a - self._env_lo > limit or self._env_hi - b > limit):
            self._boundary(out, "clock_mismatch", host, final=True)
            return "stopped"
        self._env_lo = a if self._env_lo is None else min(self._env_lo, a)
        self._env_hi = b if self._env_hi is None else max(self._env_hi, b)
        self._clk_dom = domain
        self._last_dev = dev
        return "ok"

    def finish(self, now_ns: int) -> list:
        """Everything still queued, then a quiet gap at ``now_ns`` (the observe run's end; a replay's verifier
        calls it with the record's ``last_take_ns``)."""
        out: list = []
        while True:
            out += self.take(now_ns)
            if self._queue_empty():
                break
        self._gap_before(out, int(now_ns))
        return out


__all__ = ["DepthSource"]
