"""The operating REAL run's sources (docs/plans/2026-11-26-real-operating-assembly.md 2.2, unlock U2a).

``RealScanSource`` (``scans``, ``scan_source_events``, ``sensor``, ``fatal``, ``close``): the scan container's
datagrams on the bound Unix socket, judged by a ``ScanFeed`` in receipt order -- the observe run's intake (design 3.2)
behind the loop's calls, the replay's delivery rules (``replay_sources.ReplayScan``) live:

- at most ``RAW_PER_CALL`` datagrams are read per call; more waiting is a backlog for the next call (counted), never
  a gap;
- no datagram for ``SCAN_GAP_NS`` opens a receive gap -- seen on a quiet call or on the read that ends it, before
  that receipt is taken as the newest: a new feed generation, ``("lost", sensor)`` for the layer, the generation told
  to SLAM; the next datagram tells ``("connected", sensor)``;
- a socket error is a new generation and the run's end (``fatal = "scan_socket_error"``);
- a frame carrying the synthetic mark is the run's end (``fatal = "scan_synthetic"``): an operating run never takes a
  simulator's frames;
- an evidence boundary (unlock U3a, plan 2.5; Codex U3a round 1 P1): once a scan was taken, every break of the input
  generation -- the feed's generation raised for any reason (a producer change, ids going back, the clock pair
  established again, a receive gap, a socket error) -- a ``clock_jump`` refusal (the break itself, before the clock is
  established again), and a frame that loses its qualification (no ``frame_bound`` for it: ``to_layer_scan`` refuses
  it, which opens a feed generation of its own so SLAM cuts what it holds) emit ``("boundary", sensor)`` for the loop
  (its latch, ``scan_generation_changed``, with the evidence thrown away before any install) and drop the scans of
  this call from before it; ``last_refusal`` keeps the newest refusal's reason for the console;
- a receive gap found on a read drops the scans of this call from before it (the layer sees lost, connected, then
  only later scans; SLAM is never told an older generation after the gap's);
- one call delivers one generation: scans read in the same call before the newest (feed, producer) generation are
  dropped (counted), as a gap's are -- the layer never takes a scan of a generation SLAM has left;
- every accepted scan tells SLAM its (feed, producer) generation first (7a-3), in receipt order, then becomes a layer
  scan (``lidar_cfg.to_layer_scan``: no bound for this frame keeps it diagnostic, counted by reason); per call the
  last ``SCANS_PER_CALL`` layer scans go, the rest of a backlog read on the next call; a scan reaches SLAM only through
  ``offer_for`` -- the run's SLAM branch, called after the layer accepted it (``from_feed``, its acquisition interval
  and generations kept): the scan the layer takes is the scan SLAM takes (PR 7 design 6.1, 1).

``start_real_imu(cfg, device, gap_max_s)``: the D435i motion module started on an operating ``ImuSource`` (the loop's
``samples`` / ``generation``) and judged against the watch with the profile actually opened (``fits_watch``). A
device that stops delivering is the source's own receive gap and the watch's age/gap stop; the device is stopped by
the run's closers."""

from __future__ import annotations

import collections
import math
import time

SCANS_PER_CALL = 4  # real_run.SCANS_PER_CALL
RAW_PER_CALL = 8  # datagrams read per call at most: 10 Hz scans, 50 Hz calls (plan 2.2)
SCAN_GAP_NS = 350_000_000  # the observe run's receive gap (past the layer's 0.35 s sensor timeout)
RECV_BYTES = 1 << 20


class RealScanSource:
    def __init__(self, sock, lidar_cfg, start_ns: int, *, slam=None, sensor: str = "lidar",
                 clock=time.monotonic_ns) -> None:
        from forklift_core.sensors.scan_feed import ScanFeed

        self.sock, self.cfg, self.start_ns, self.slam, self.sensor = sock, lidar_cfg, int(start_ns), slam, sensor
        self.clock = clock
        self.feed = ScanFeed()
        self.counts = collections.Counter()
        self.fatal: str | None = None
        self._events: list = []
        self._ready: list = []
        self._pending: dict = {}  # id(layer scan) -> feed scan, for the scans handed out this call (offer_for)
        self._advanced_at: int | None = None
        self._last_rx: int | None = None
        self._gap_open = False
        self._told = None
        self._producer = None  # the last accepted scan's producer generation (a gap's new feed generation keeps it)
        self.last_refusal: str | None = None  # the newest frame's refusal reason (feed or frame_bound), for the console
        self.last_boundary: str | None = None
        self._seen_gen: int | None = None  # the feed generation last looked at (a change since is a boundary)
        self._any_taken = False  # a scan was taken: before it there is no evidence a boundary could void
        self._last_key = None  # the last taken scan's (feed, producer) generation
        self._raised: Exception | None = None  # an error inside a read, raised once its events were handed over
        self._clock_break = False  # a clock jump of the current producer opened its boundary: its re-establishment
        # ("clock") is the same break, not a second boundary
        self.last_delivered_s: float | None = None

    # --- the loop's calls ---------------------------------------------------------------------------------------------
    def _run(self, now_ns: int) -> None:
        """One read per tick; an error raised inside it (SLAM told a generation and refusing, Codex U3a round 3 P2) is
        kept: the events already queued -- a boundary first of all -- reach the loop before it is raised."""
        if self._raised is None:
            try:
                self._advance(int(now_ns))
            except Exception as exc:  # noqa: BLE001 -- re-raised by the next call that has nothing to hand over
                self._raised = exc

    def scan_source_events(self, now_ns: int) -> list:
        self._run(now_ns)
        out, self._events = self._events, []
        if not out and self._raised is not None:
            raise self._raised
        return out

    def scans(self, now_ns: int) -> list:
        self._run(now_ns)
        if self._raised is not None:
            raise self._raised  # the run's end, its own reason kept
        out, self._ready = self._ready, []
        return out

    def offer_for(self, layer_scan) -> None:
        """The run's SLAM branch (``RunSources.slam_feed``), called only for a scan the layer accepted: its feed scan
        offered to SLAM (``from_feed``: its acquisition interval and generations kept). A scan the layer refused never
        reaches SLAM (Codex U2a round 1 P2-2)."""
        scan = self._pending.pop(id(layer_scan), None)
        if scan is None or self.slam is None:
            return
        from forklift_core.localization.slam_real_scan import from_feed

        s, why = from_feed(scan, self.cfg)
        if s is None:
            self.counts[f"slam_{why}"] += 1
            return
        self.slam.offer(s)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    # --- the intake ---------------------------------------------------------------------------------------------------
    def _tell(self, gen) -> None:
        if self.slam is not None and gen != self._told:
            self.slam.generation(gen)
        self._told = gen

    def _open_gap(self, items: list) -> None:
        """A receive gap: a new feed generation, ``lost``, and its evidence boundary queued **before** SLAM is told --
        a SLAM refusal raised there never swallows the boundary (Codex U3a round 4 P2)."""
        self._gap_open = True
        self.counts["receive_gaps"] += 1
        self.feed.new_generation("receive_gap")
        self._events.append(("lost", self.sensor))
        self._boundary(items, "feed_generation:receive_gap")
        self._seen_gen = self.feed.generation
        self._tell((self.feed.generation, self._producer))

    def _boundary(self, items: list, why: str) -> None:
        """An evidence boundary (U3a): the loop latches on it; the scans of this call before it never go (``items``
        emptied in place). None before any scan was taken (no evidence to void)."""
        if not self._any_taken:
            return
        self.counts["boundaries"] += 1
        self.counts["before_boundary_dropped"] += sum(1 for it in items if it[2] is not None)
        items.clear()
        self.last_boundary = why
        self._last_key = None  # the next scan starts the new generation (no second boundary for the same break)
        self._events.append(("boundary", self.sensor))

    def _frame_of_current(self, data) -> bool:
        """The refused frame is the current producer's (its nonce), not a candidate's whose refusal restored the
        current producer's state (Codex U3a round 3 P3)."""
        from forklift_core.sensors.scan_feed import FrameError, decode_frame

        try:
            gen = decode_frame(data)[0].get("gen")
        except (FrameError, ValueError, KeyError, TypeError):
            return False
        return gen == self._producer

    def _watch_generation(self, items: list) -> None:
        """A feed generation raised since the last look is a boundary (whatever raised it), except the clock's
        re-establishment after a jump whose boundary was already opened at the jump."""
        g = self.feed.generation
        if self._seen_gen is not None and g != self._seen_gen:
            why = self.feed.generation_why[-1] if self.feed.generation_why else "generation"
            if not (self._clock_break and why == "clock"):
                self._boundary(items, f"feed_generation:{why}")
            self._clock_break = False
        self._seen_gen = g

    def _end(self, why: str) -> None:
        """The run's end: the reason fixed first (a SLAM error on the way never replaces it), the layer told the
        source is lost, SLAM told the new generation."""
        self.fatal = why
        self._events.append(("lost", self.sensor))
        try:
            self._tell((self.feed.generation, self._producer))
        except Exception:  # noqa: BLE001 -- the source's own reason stands
            self.counts["tell_failed_at_end"] += 1

    def _advance(self, now: int) -> None:
        from forklift_core.sensors.lidar_cfg import to_layer_scan

        if self._advanced_at == now or self.fatal is not None:
            return
        self._advanced_at = now
        self._pending = {}
        items = []  # ("scan", feed scan, layer scan or None), receipt order, all after the last gap of this call
        for k in range(RAW_PER_CALL + 1):
            if k == RAW_PER_CALL:
                self.counts["read_capped"] += 1  # more may be waiting: the next call reads on (a backlog, not a gap)
                break
            try:
                data = self.sock.recv(RECV_BYTES)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                self.counts["socket_errors"] += 1
                self.feed.new_generation("socket_error")
                self._seen_gen = self.feed.generation
                # the boundary first (Codex U3a round 2 P2): the loop throws the evidence away before this tick's install
                self._boundary(items, "socket_error")
                self._end("scan_socket_error")
                return
            if not data:
                break
            recv = int(self.clock())
            if not self._gap_open and self._last_rx is not None and recv - self._last_rx > SCAN_GAP_NS:
                # the gap ended on this very read: opened before the receipt is the newest. The scans of this call
                # from before the gap are dropped (the layer sees lost, connected, then only scans after it; SLAM is
                # never told an older generation after the gap's -- Codex U2a round 1 P2-1)
                self.counts["before_gap_dropped"] += sum(1 for it in items if it[2] is not None)
                items.clear()
                self._open_gap(items)
            if self._gap_open:
                self._gap_open = False
                self._events.append(("connected", self.sensor))
            self._last_rx = recv
            synthetic_before = self.feed.synthetic_frames
            scan, why = self.feed.judge(data, recv)
            self._watch_generation(items)  # a producer change, ids back, the clock established again: a boundary
            if self.feed.synthetic_frames != synthetic_before or (scan is not None and scan.meta.get("synthetic")):
                self.counts["synthetic"] += 1  # whatever the verdict: an operating run takes no simulator frame
                self._end("scan_synthetic")
                return
            if scan is None:
                self.counts[f"feed_{why}"] += 1
                self.last_refusal = f"feed:{why}"
                if why == "clock_jump" and self.feed.establishing and self._frame_of_current(data):
                    # the current producer's own clock pair jumped (a candidate's refusal leaves its clock established,
                    # Codex U3a round 2 P2): the break itself -- a boundary now, and a feed generation SLAM is told, so
                    # what waits or is in flight there is cut; the re-establishment adds none
                    self._boundary(items, "clock_jump")
                    self.feed.new_generation("clock_jump")
                    self._seen_gen = self.feed.generation
                    self._clock_break = True
                    self._tell((self.feed.generation, self._producer))
                continue
            key = (scan.feed_gen, scan.producer_gen)
            if self._last_key is not None and key != self._last_key:
                self._boundary(items, "generation_changed")  # whatever the feed said: a scan of another generation
            self._last_key = key
            self._producer = scan.producer_gen
            layer, lwhy = to_layer_scan(scan, self.cfg, self.start_ns, sensor=self.sensor)
            if layer is None:
                # the frame lost its qualification: the evidence boundary even within one producer generation
                # (Codex U3 split round 2 P1) -- a new feed generation, so SLAM cuts what it holds of the earlier
                self.counts[f"layer_{lwhy}"] += 1
                self.last_refusal = f"frame:{lwhy}"
                self._boundary(items, f"qualification:{lwhy}")
                self.feed.new_generation("qualification_lost")
                self._seen_gen = self.feed.generation  # this boundary's own generation (not a second boundary)
                self._tell((self.feed.generation, self._producer))
                continue
            self._any_taken = True
            items.append(("scan", scan, layer))
        if not items and not self._gap_open and self._last_rx is not None and now - self._last_rx > SCAN_GAP_NS:
            self._open_gap(items)  # a quiet call past the gap
        if items:
            # one call delivers one generation: the scans before the newest (feed, producer) generation of this read
            # are dropped as the gap's are -- SLAM is never told an older generation after a newer one, and the layer
            # never takes a scan of a generation SLAM has left (Codex U2a round 2 P2-1)
            start = max(i for i, it in enumerate(items) if i == 0 or (it[1].feed_gen, it[1].producer_gen)
                        != (items[i - 1][1].feed_gen, items[i - 1][1].producer_gen))
            self.counts["before_generation_dropped"] += sum(1 for it in items[:start] if it[2] is not None)
            items = items[start:]
        layered = [i for i, it in enumerate(items) if it[2] is not None]
        kept = set(layered[-SCANS_PER_CALL:])
        self.counts["cut"] += len(layered) - len(kept)
        for i, (_kind, scan, layer) in enumerate(items):
            self._tell((scan.feed_gen, scan.producer_gen))  # every accepted scan's generation, in order, first
            if i in kept:
                self._ready.append(layer)
                self._pending[id(layer)] = scan
                self.counts["delivered"] += 1
                self.last_delivered_s = float(layer.last_s)


def start_real_imu(cfg, device, gap_max_s: float):
    """``(source, fits, profile)``: an operating ``ImuSource`` fed by ``device`` -- started here with the sample callback
    and the removal callback (``device.start(on_sample, on_lost=...)``: a removed device is the source's final
    boundary at once, not a wait for the watch's age -- Codex U2a round 1 P2-4) -- whether the profile actually opened
    fits the watch (``fits_watch`` with the opened accel rate; a profile that does not say its rate never fits), and
    that profile. Raises what the device raises on start (the run refuses then); the source refuses an unmeasured
    config (``imu_source:delay_unmeasured``)."""
    from forklift_core.sensors.imu_source import ImuSource, fits_watch

    source = ImuSource(cfg, operating=True)
    device.start(source.on_sample, on_lost=source.lost)
    profile = device.profile_info()
    hz = (profile.get("actual") or {}).get("accel_hz")
    if not isinstance(hz, (int, float)) or isinstance(hz, bool) or not math.isfinite(hz) or hz <= 0:
        return source, False, profile  # the rate actually opened unknown: never the requested one in its place
    fits = fits_watch(cfg, gap_max_s, bound_s=source.bound_ns / 1e9, accel_hz=hz)
    return source, bool(fits), profile


__all__ = ["RAW_PER_CALL", "RealScanSource", "SCAN_GAP_NS", "SCANS_PER_CALL", "start_real_imu"]
