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
- every accepted scan tells SLAM its (feed, producer) generation first (7a-3), then becomes a layer scan
  (``lidar_cfg.to_layer_scan``: no bound for this frame keeps it diagnostic, counted by reason); per call the last
  ``SCANS_PER_CALL`` layer scans go, and only a scan that goes to the layer is offered to SLAM (``from_feed``, its
  acquisition interval and generations kept) -- the scan the layer takes is the scan SLAM takes (PR 7 design 6.1, 1).

``start_real_imu(cfg, device, gap_max_s)``: the D435i motion module started on an operating ``ImuSource`` (the loop's
``samples`` / ``generation``) and judged against the watch with the profile actually opened (``fits_watch``). A
device that stops delivering is the source's own receive gap and the watch's age/gap stop; the device is stopped by
the run's closers."""

from __future__ import annotations

import collections
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
        self._advanced_at: int | None = None
        self._last_rx: int | None = None
        self._gap_open = False
        self._told = None
        self._producer = None  # the last accepted scan's producer generation (a gap's new feed generation keeps it)
        self.last_delivered_s: float | None = None

    # --- the loop's calls ---------------------------------------------------------------------------------------------
    def scan_source_events(self, now_ns: int) -> list:
        self._advance(int(now_ns))
        out, self._events = self._events, []
        return out

    def scans(self, now_ns: int) -> list:
        self._advance(int(now_ns))
        out, self._ready = self._ready, []
        return out

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

    def _open_gap(self) -> None:
        self._gap_open = True
        self.counts["receive_gaps"] += 1
        self.feed.new_generation("receive_gap")
        self._events.append(("lost", self.sensor))
        self._tell((self.feed.generation, self._producer))

    def _advance(self, now: int) -> None:
        if self._advanced_at == now or self.fatal is not None:
            return
        self._advanced_at = now
        items = []  # ("gen", gen) | ("scan", feed scan, layer scan)
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
                self._tell((self.feed.generation, self._producer))
                self.fatal = "scan_socket_error"
                return
            if not data:
                break
            recv = int(self.clock())
            if not self._gap_open and self._last_rx is not None and recv - self._last_rx > SCAN_GAP_NS:
                self._open_gap()  # the gap ended on this very read: opened before the receipt is the newest
            if self._gap_open:
                self._gap_open = False
                self._events.append(("connected", self.sensor))
            self._last_rx = recv
            scan, why = self.feed.judge(data, recv)
            if scan is None:
                self.counts[f"feed_{why}"] += 1
                continue
            if scan.meta.get("synthetic"):
                self.counts["synthetic"] += 1
                self.fatal = "scan_synthetic"
                return
            from forklift_core.sensors.lidar_cfg import to_layer_scan

            self._producer = scan.producer_gen
            items.append(("gen", (scan.feed_gen, scan.producer_gen)))
            layer, lwhy = to_layer_scan(scan, self.cfg, self.start_ns, sensor=self.sensor)
            if layer is None:
                self.counts[f"layer_{lwhy}"] += 1
            items.append(("scan", scan, layer))
        if not items and not self._gap_open and self._last_rx is not None and now - self._last_rx > SCAN_GAP_NS:
            self._open_gap()  # a quiet call past the gap
        layered = [i for i, it in enumerate(items) if it[0] == "scan" and it[2] is not None]
        kept = set(layered[-SCANS_PER_CALL:])
        self.counts["cut"] += len(layered) - len(kept)
        for i, it in enumerate(items):
            if it[0] == "gen":
                self._tell(it[1])
            elif i in kept:
                self._ready.append(it[2])
                self.counts["delivered"] += 1
                self.last_delivered_s = float(it[2].last_s)
                self._offer(it[1])

    def _offer(self, scan) -> None:
        if self.slam is None:
            return
        from forklift_core.localization.slam_real_scan import from_feed

        s, why = from_feed(scan, self.cfg)
        if s is None:
            self.counts[f"slam_{why}"] += 1
            return
        self.slam.offer(s)


def start_real_imu(cfg, device, gap_max_s: float):
    """``(source, fits, profile)``: an operating ``ImuSource`` fed by ``device`` (started here; its options written on
    start), whether the profile actually opened fits the watch (``imu_source.fits_watch`` with the opened accel rate),
    and that profile. Raises what the device raises on start (the run refuses then); the source refuses an unmeasured
    config (``imu_source:delay_unmeasured``)."""
    from forklift_core.sensors.imu_source import ImuSource, fits_watch

    source = ImuSource(cfg, operating=True)
    device.start(source.on_sample)
    profile = device.profile_info()
    hz = (profile.get("actual") or {}).get("accel_hz")
    fits = fits_watch(cfg, gap_max_s, bound_s=source.bound_ns / 1e9, accel_hz=hz)
    return source, bool(fits), profile


__all__ = ["RAW_PER_CALL", "RealScanSource", "SCAN_GAP_NS", "SCANS_PER_CALL", "start_real_imu"]
