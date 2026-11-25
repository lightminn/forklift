"""The replay run's sources (PR 7 7r, docs/plans/2026-11-26-7r-replay-run.md 2-3): an observe record's LiDAR and IMU
given to the HIL run behind the same calls as its synthetic sources.

Time: nothing is given until ``set_anchor(hil_anchor_ns, now_ns)`` -- the run's first STATE with a finite delay bound
-- fixes Δ = hil_anchor - record anchor. A row goes at its record time + Δ, never before, and never before the rows
ahead of it in its stream (file order, ``replay_record``). Rows whose time + Δ precede the anchor call's ``now_ns``
(the record's prefix, and anything already due then) build the sources' judgement state but give nothing
(``before_anchor``). Monotonic times are moved by Δ; ROS stamps, the clock pair and device counters are not.

HIL check (6a rule): every call needs ``transport.hil_plant`` and the boot it currently holds. While the check is lost
the rows that come due are still judged (the feed's generations, the IMU source's verdicts and stops), their output is
dropped and counted. A boot change sets a boundary at that instant: nothing whose acquisition starts before it is
given after.

``ReplayScan`` (``scans``, ``scan_source_events``, ``sensor``): the frames through a new ``ScanFeed`` in file order
(the record's verdict must come back, else ``fatal = "replay_mismatch"``), each accepted scan moved by Δ and made a
layer scan (``lidar_cfg.to_layer_scan``, origin ``replayed``); per call the last ``SCANS_PER_CALL`` layer scans go.
The SLAM branch is the source's own (``RunSources.slam_feed`` stays None): in file order, a producer or feed
generation change is told (``slam.generation``) before anything after it, and only a scan that goes to the layer is
offered (``slam.offer(from_feed(...))``) -- the scan the layer takes is the scan SLAM takes (design 6.1, 1; Codex plan
round 1 P1). ``scan_event`` rows reproduce the design 3.2 latch rules: ``receive_gap`` -> ``("lost", sensor)``,
``receive_resumed`` -> ``("connected", sensor)``, ``socket_error`` -> ``fatal = "scan_socket_error"``.

``ReplayImu`` (``samples``, ``generation``, ``last_read_capped``): every row into an operating ``ImuSource`` on an
injected clock set to the row's moved time (a loss at its own time), the source's output given to the loop only after
the prefix is through (at most ``limit`` rows a call) and only past the boot boundary. ``last_read_capped`` is true
while due rows are still to go in, so the loop does not ask ``generation()`` over samples not yet put in; and
``generation()`` uses the instant the last ``samples()`` put rows in up to, not the clock now (Codex plan rounds
8-10). ``drain(now_ns)``: the run's end check when the loop does not read the IMU (no watch, or bypassed) -- rows put in
and judged, their output dropped (a loss or a stop still holds afterwards, rounds 5-6).

Per call each source processes a bounded number of raw rows (LiDAR ``LIDAR_ROWS_PER_CALL``, IMU the call's
``limit``); rows left due are a backlog (``last_read_capped`` for the IMU), never a gap. A read failure surfaces at the
next call (``reader.check``). A row's turn is the later of its own time and the rows before it in the file (a
``socket_error`` stamped with its call's start may follow a later scan). The boot boundary is the first STATE of the
new boot as the transport read it (``hil_boot_since_ns``)."""

from __future__ import annotations

import base64
import collections
import math
from dataclasses import replace

from forklift_core.live.replay_record import StreamReader

SCANS_PER_CALL = 4  # real_run.SCANS_PER_CALL
LIDAR_ROWS_PER_CALL = 8  # raw LiDAR rows (scans and events) processed per call at most: 10 Hz scans, 50 Hz calls
IMU_LIMIT = 64  # the loop's IMU batch (real_loop.RealLoop.IMU_BATCH)


class _HilGate:
    """The HIL check of every call and the boot boundary (design 1.2, 8): a boot change puts the boundary at the read
    time of the new boot's first STATE (the transport's ``hil_boot_since_ns``; the call's instant only when the
    transport does not say), the same instant for both sources (Codex 7r-1 round 1 P2-5)."""

    def __init__(self, transport) -> None:
        self.transport = transport
        self.boot = getattr(transport, "hil_boot", None)
        self.boundary_ns: int | None = None
        self.counts = collections.Counter()

    def check(self, now_ns: int) -> bool:
        tr = self.transport
        if getattr(tr, "hil_plant", False) is not True:
            self.counts["hil_check_lost_calls"] += 1
            return False
        if tr.hil_boot != self.boot:
            self.boot = tr.hil_boot
            since = getattr(tr, "hil_boot_since_ns", None)
            self.boundary_ns = int(since) if isinstance(since, int) else int(now_ns)
            self.counts["boot_boundaries"] += 1
        return True

    def after_boundary(self, lo_ns: int) -> bool:
        return self.boundary_ns is None or int(lo_ns) >= self.boundary_ns


def _moved_scan(scan, delta: int):
    meta = dict(scan.meta)
    for k in ("first_beam_mono_ns", "last_beam_mono_ns"):
        if k in meta:
            meta[k] = int(meta[k]) + delta
    return replace(scan, stamp_mono_ns=int(scan.stamp_mono_ns) + delta, recv_mono_ns=int(scan.recv_mono_ns) + delta,
                   meta=meta)


class _Anchored:
    def __init__(self, record) -> None:
        self.record = record
        self.delta: int | None = None
        self.live_from_ns: int | None = None  # rows whose turn comes before it give nothing

    def set_anchor(self, hil_anchor_ns: int, now_ns: int) -> None:
        """Δ from the run's anchor STATE; delivery starts at the later of that STATE and the call (round 1 P2-5)."""
        if self.delta is None:
            self.delta = int(hil_anchor_ns) - int(self.record.anchor_ns)
            self.live_from_ns = max(int(now_ns), int(hil_anchor_ns))


class ReplayScan(_Anchored):
    def __init__(self, transport, record, lidar_cfg, start_ns: int, *, slam=None, sensor: str = "lidar",
                 reader=None) -> None:
        from forklift_core.sensors.scan_feed import ScanFeed

        super().__init__(record)
        self.gate = _HilGate(transport)
        self.cfg, self.start_ns, self.slam, self.sensor = lidar_cfg, int(start_ns), slam, sensor
        self.reader = reader if reader is not None else StreamReader(record.path, record.lidar, pin=record.stat)
        self.feed = ScanFeed()
        self.counts = collections.Counter()
        self.fatal: str | None = None
        self._events: list = []
        self._ready: list = []  # layer scans of this call, made when the events were asked
        self._advanced_at: int | None = None
        self._told_gen = None  # the (feed_gen, producer_gen) last told to SLAM
        self._producer = None
        self._turn_ns: int | None = None  # the latest turn of the rows processed (file order)

    # --- the loop's calls ---------------------------------------------------------------------------------------------
    def scan_source_events(self, now_ns: int) -> list:
        self._advance(int(now_ns))
        out, self._events = self._events, []
        return out

    def scans(self, now_ns: int) -> list:
        self._advance(int(now_ns))
        out, self._ready = self._ready, []
        return out

    def eof(self) -> bool:
        return self.reader.done and not self._ready and not self._events

    def close(self) -> None:
        self.reader.close()

    # --- the rows -----------------------------------------------------------------------------------------------------
    def _tell(self, gen) -> None:
        if self.slam is not None and gen != self._told_gen:
            self.slam.generation(gen)
        self._told_gen = gen

    def _advance(self, now: int) -> None:
        if self._advanced_at == now:
            return
        self._advanced_at = now
        self.reader.check()
        if self.delta is None or self.fatal is not None:
            return
        held = self.gate.check(now)
        t_read = self.reader.t_read
        if t_read < now - self.delta:
            self.counts["read_late"] += 1  # the reader behind the replay's now (plan 4): marked apart from gaps
        upto = min(now - self.delta, t_read)
        rows = self.reader.take(upto, LIDAR_ROWS_PER_CALL)
        nxt = self.reader.peek_ns()
        if nxt is not None and nxt <= upto:
            self.counts["backlog_calls"] += 1
        items = []  # ("gen", tuple) | ("scan", moved feed scan, layer scan or None); only what goes live
        for ns, r in rows:
            turn = ns if self._turn_ns is None else max(ns, self._turn_ns)  # file order (round 1 P2-6)
            self._turn_ns = turn
            live = held and turn + self.delta >= self.live_from_ns
            t = r.get("t")
            if t == "scan_event":
                kind = r.get("kind")
                if kind in ("receive_gap", "socket_error"):
                    self.feed.new_generation(kind)
                    if live:
                        items.append(("gen", (self.feed.generation, self._producer)))
                if not live:
                    self.counts["events_dropped"] += 1
                    continue
                if kind == "receive_gap":
                    self._events.append(("lost", self.sensor))
                elif kind == "receive_resumed":
                    self._events.append(("connected", self.sensor))
                elif kind == "socket_error":
                    self.fatal = "scan_socket_error"
                    break
                continue
            data = base64.b64decode(r["frame"])
            scan, why = self.feed.judge(data, ns)
            if why != r.get("verdict"):
                self.fatal = "replay_mismatch"
                self.counts["replay_mismatch"] += 1
                break
            if scan is None:
                continue
            self._producer = scan.producer_gen
            if not live:
                # judged (the feed's state goes on), nothing given, SLAM not told (round 1 P2-6)
                self.counts["hil_check_lost" if not held else "before_anchor"] += 1
                continue
            items.append(("gen", (scan.feed_gen, scan.producer_gen)))
            moved = _moved_scan(scan, self.delta)
            layer, lwhy = self._layer(moved)
            if layer is None:
                self.counts[f"layer_{lwhy}"] += 1
            elif not self.gate.after_boundary(int(round(layer.first_s * 1e9)) + self.start_ns):
                self.counts["before_boot_boundary"] += 1
                layer = None
            items.append(("scan", moved, layer))
        layered = [i for i, it in enumerate(items) if it[0] == "scan" and it[2] is not None]
        kept = set(layered[-SCANS_PER_CALL:])
        self.counts["cut"] += len(layered) - len(kept)
        for i, it in enumerate(items):
            if it[0] == "gen":
                self._tell(it[1])
            elif i in kept:
                self._ready.append(it[2])
                self.counts["delivered"] += 1
                if self.slam is not None:
                    self._offer(it[1])

    def _layer(self, moved):
        from forklift_core.sensors.lidar_cfg import to_layer_scan

        return to_layer_scan(moved, self.cfg, self.start_ns, sensor=self.sensor, origin="replayed")

    def _offer(self, moved) -> None:
        from forklift_core.localization.slam_real_scan import from_feed

        s, why = from_feed(moved, self.cfg)
        if s is None:
            self.counts[f"slam_{why}"] += 1
            return
        self.slam.offer(s)


class ReplayImu(_Anchored):
    def __init__(self, transport, record, imu_cfg, *, reader=None) -> None:
        from forklift_core.sensors.imu_source import ImuSource

        super().__init__(record)
        self.gate = _HilGate(transport)
        self.reader = reader if reader is not None else StreamReader(record.path, record.imu, pin=record.stat)
        self._clock = {"ns": 0}
        self.src = ImuSource(imu_cfg, operating=True, now_ns=lambda: self._clock["ns"])
        self.counts = collections.Counter()
        self.last_read_capped = False
        self.input_ns: int | None = None  # every row up to this instant is in (the gap check never looks past it)
        self.prefix_done = False  # the prefix judged through, once (round 2 P2-1)
        self._buf: collections.deque = collections.deque()  # judged output not yet handed to the loop
        self._left = False

    def _put_one(self, ns: int, r: dict, live: bool) -> None:
        """One row in and judged at once (the source's queue never holds more than it: no overflow whatever its
        queue_max -- round 2 P2-2); its output kept for the loop, or dropped before the anchor."""
        at = ns + self.delta
        self._clock["ns"] = max(self._clock["ns"], at)
        if r.get("t") == "imu":
            self.src.push(at, r.get("s", "a"), r.get("dev_us"), r.get("dom"), (r["x"], r["y"], r["z"]),
                          ts_ms=r.get("ts_ms"))
        else:
            self.src.lost(str(r.get("why"))[len("lost:"):], at_ns=at)
        out = self.src.samples(self._clock["ns"], 1 << 30)
        if live:
            self._buf.extend(out)
        else:
            self.counts["before_anchor"] += len(out)

    def _step(self, upto_ns: int, budget: int) -> None:
        """At most ``budget`` due rows in, the prefix first; ``input_ns`` moves only when every due row is in."""
        self.reader.check()
        t_read = self.reader.t_read
        if t_read == -math.inf:
            self.counts["read_late"] += 1  # nothing read yet: behind, not a gap
            self._left = True
            return
        if t_read < upto_ns - self.delta:
            self.counts["read_late"] += 1
        rec_upto = min(upto_ns - self.delta, t_read)
        live_rec = self.live_from_ns - self.delta
        rows = self.reader.take(rec_upto, budget)
        for ns, r in rows:
            self._put_one(ns, r, live=ns >= live_rec)
        nxt = self.reader.peek_ns()
        if not self.prefix_done and t_read >= live_rec and (nxt is None or nxt >= live_rec):
            self.prefix_done = True
        self._left = nxt is not None and nxt <= rec_upto
        if not self._left and self.prefix_done:
            self.input_ns = int(rec_upto + self.delta)
            self._clock["ns"] = max(self._clock["ns"], self.input_ns)

    def samples(self, upto_ns: int, limit: int) -> list:
        self.last_read_capped = False
        if self.delta is None:
            return []
        self._step(int(upto_ns), int(limit))
        held = self.gate.check(int(upto_ns))
        if not self.prefix_done:
            self.last_read_capped = True
            return []
        out = [self._buf.popleft() for _ in range(min(int(limit), len(self._buf)))]
        self.last_read_capped = bool(self._left or self._buf)
        if not held:
            self.counts["hil_check_lost"] += len(out)
            return []
        kept = [o for o in out if self.gate.after_boundary(o[1])]
        self.counts["before_boot_boundary"] += len(out) - len(kept)
        return kept

    def generation(self) -> int:
        if self.input_ns is not None:
            self._clock["ns"] = max(self._clock["ns"], self.input_ns)
        return self.src.generation()

    def drain(self, now_ns: int, limit: int = IMU_LIMIT) -> None:
        """The loop does not read the IMU: at most ``limit`` due rows put in and judged a call, output dropped and
        counted, ``last_read_capped`` kept current (round 1 P2-1)."""
        self.last_read_capped = False
        if self.delta is None:
            return
        self._step(int(now_ns), int(limit))
        self.counts["unread"] += len(self._buf)
        self._buf.clear()
        self.last_read_capped = bool(self._left or not self.prefix_done)
        if not self.last_read_capped:
            self.generation()

    def eof(self) -> bool:
        return (self.reader.done and not self._buf and self.src.pending() == 0 and not self.last_read_capped
                and self.prefix_done)

    def close(self) -> None:
        self.reader.close()


__all__ = ["IMU_LIMIT", "LIDAR_ROWS_PER_CALL", "ReplayImu", "ReplayScan", "SCANS_PER_CALL"]
