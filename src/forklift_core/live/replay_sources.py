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
and judged, their output dropped (a loss or a stop still holds afterwards, rounds 5-6)."""

from __future__ import annotations

import base64
import collections
import math
from dataclasses import replace

from forklift_core.live.replay_record import StreamReader

SCANS_PER_CALL = 4  # real_run.SCANS_PER_CALL
IMU_LIMIT = 64  # the loop's IMU batch (real_loop.RealLoop.IMU_BATCH)


class _HilGate:
    """The HIL check of every call and the boot boundary (design 1.2, 8)."""

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
            self.boundary_ns = int(now_ns)
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
        self.live_from_ns: int | None = None  # the anchor call's now: rows due before it give nothing

    def set_anchor(self, hil_anchor_ns: int, now_ns: int) -> None:
        if self.delta is None:
            self.delta = int(hil_anchor_ns) - int(self.record.anchor_ns)
            self.live_from_ns = int(now_ns)


class ReplayScan(_Anchored):
    def __init__(self, transport, record, lidar_cfg, start_ns: int, *, slam=None, sensor: str = "lidar",
                 reader=None) -> None:
        from forklift_core.sensors.scan_feed import ScanFeed

        super().__init__(record)
        self.gate = _HilGate(transport)
        self.cfg, self.start_ns, self.slam, self.sensor = lidar_cfg, int(start_ns), slam, sensor
        self.reader = reader if reader is not None else StreamReader(record.path, record.lidar)
        self.feed = ScanFeed()
        self.counts = collections.Counter()
        self.fatal: str | None = None
        self._events: list = []
        self._ready: list = []  # layer scans of this call, made when the events were asked
        self._advanced_at: int | None = None
        self._told_gen = None  # the (feed_gen, producer_gen) last told to SLAM
        self._producer = None

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
        if self.delta is None or self.fatal is not None:
            return
        held = self.gate.check(now)
        t_read = self.reader.t_read
        if t_read < now - self.delta:
            self.counts["read_late"] += 1  # the reader behind the replay's now (plan 4): marked apart from gaps
        upto = min(now - self.delta, t_read)
        rows = self.reader.take(upto)
        if not rows:
            return
        items = []  # ("gen", tuple) | ("scan", feed scan, layer scan or None)
        for ns, r in rows:
            t = r.get("t")
            if t == "scan_event":
                kind = r.get("kind")
                if kind in ("receive_gap", "socket_error"):
                    self.feed.new_generation(kind)
                    items.append(("gen", (self.feed.generation, self._producer)))
                if not held or ns + self.delta < self.live_from_ns:
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
            items.append(("gen", (scan.feed_gen, scan.producer_gen)))
            moved = _moved_scan(scan, self.delta)
            layer = None
            if ns + self.delta < self.live_from_ns:
                self.counts["before_anchor"] += 1
            elif not held:
                self.counts["hil_check_lost"] += 1
            else:
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
        self.reader = reader if reader is not None else StreamReader(record.path, record.imu)
        self._clock = {"ns": 0}
        self.src = ImuSource(imu_cfg, operating=True, now_ns=lambda: self._clock["ns"])
        self.counts = collections.Counter()
        self.last_read_capped = False
        self.input_ns: int | None = None  # the instant the last samples() put rows in up to

    def _put(self, rows) -> None:
        for ns, r in rows:
            at = ns + self.delta
            self._clock["ns"] = max(self._clock["ns"], at)
            if r.get("t") == "imu":
                self.src.push(at, r.get("s", "a"), r.get("dev_us"), r.get("dom"), (r["x"], r["y"], r["z"]),
                              ts_ms=r.get("ts_ms"))
            else:
                self.src.lost(str(r.get("why"))[len("lost:"):], at_ns=at)

    def _judge_and_drop(self, upto: int, why: str) -> None:
        self._clock["ns"] = max(self._clock["ns"], upto)
        out = self.src.samples(upto, 1 << 30)
        self.counts[why] += len(out)

    def samples(self, upto_ns: int, limit: int) -> list:
        upto_ns = int(upto_ns)
        self.last_read_capped = False
        if self.delta is None:
            return []
        t_read = self.reader.t_read
        if t_read == -math.inf:
            self.last_read_capped = True  # nothing read yet: behind, not a gap
            self.counts["read_late"] += 1
            return []
        if t_read < upto_ns - self.delta:
            self.counts["read_late"] += 1
        rec_upto = min(upto_ns - self.delta, t_read)
        held = self.gate.check(upto_ns)
        # the prefix (rows due before the anchor call), at most ``limit`` rows a call: judged, output dropped
        live_rec = self.live_from_ns - self.delta
        nxt = self.reader.peek_ns()
        if nxt is not None and nxt < live_rec:
            rows = self.reader.take(min(rec_upto, live_rec - 1), limit)
            self._put(rows)
            self._judge_and_drop(self.live_from_ns - 1, "before_anchor")
            nxt = self.reader.peek_ns()
            if (nxt is not None and nxt < live_rec) or self.src.pending():
                self.last_read_capped = True
                return []
        rows = self.reader.take(rec_upto, limit)
        self._put(rows)
        upto_in = int(min(upto_ns, rec_upto + self.delta)) if math.isfinite(rec_upto) else upto_ns
        self.input_ns = upto_in
        self._clock["ns"] = max(self._clock["ns"], upto_in)
        out = self.src.samples(upto_in, limit)
        nxt = self.reader.peek_ns()
        self.last_read_capped = bool(self.src.last_read_capped or (nxt is not None and nxt <= rec_upto))
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

    def drain(self, now_ns: int) -> None:
        """The loop does not read the IMU: due rows put in and judged, output dropped (counted)."""
        if self.delta is None:
            return
        rec_upto = min(int(now_ns) - self.delta, self.reader.t_read)
        self._put(self.reader.take(rec_upto))
        upto = int(min(now_ns, rec_upto + self.delta)) if math.isfinite(rec_upto) else int(now_ns)
        self._judge_and_drop(upto, "unread")

    def eof(self) -> bool:
        return self.reader.done and self.src.pending() == 0 and not self.last_read_capped

    def close(self) -> None:
        self.reader.close()


__all__ = ["IMU_LIMIT", "ReplayImu", "ReplayScan", "SCANS_PER_CALL"]
