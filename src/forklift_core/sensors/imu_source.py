"""The real D435i accelerometer as the IMU watch's source (docs/design/2026-11-25-real-sources-pr7.md 5, 7b;
config/real/imu.yaml).

The SDK's callback (``on_sample``, the signature of ``tools/imu_record.RealSenseSource.start``) only stamps the host
monotonic time and queues the sample: depth work or a slow loop never holds the callback up. The loop thread reads
``samples(upto_ns, limit)`` -- ``(gen, lo, hi, accel)`` in receipt order, at most ``limit``, only those received up to
``upto_ns`` -- and ``generation()``, the 5b source contract with the producer generation and the acquisition
interval added (design 5):

- the interval is ``[receipt - delay_bound, receipt]``: the sample cannot be received before it was taken, and the
  bound is the measured transport delay. A clock fit's residual is no interval (design 3.1), so the device time is
  used for continuity only. Until the bound is measured the source is diagnostic: ``operating=True`` is refused and
  the observe run records intervals under the provisional bound;
- a new generation on no accel sample for ``receive_gap_s`` (decided from one receipt history on the callback's
  side, seen by ``generation()`` with no sample as a restart must be) or a queue overflow -- the same device clock
  goes on. A receive gap, an overflow hole and a loss stay in the queue where they happened, so the samples before
  them keep the old generation, and ``generation()`` counts every boundary already queued. After a boundary only
  samples with ``lo`` not before it count (design 3.1). A wrap of the 32-bit device time is a forward step;
- a new device clock -- ``lost()`` (the device gone), the device time going back more than ``restart_back_us`` (a
  restart), its source changing -- or any two samples of the clock whose delays (receipt minus device time) differ
  by more than the bound + 1 ms + 100 ppm of their device time apart (the bound broken): a new generation, and the
  source stops (``stopped``; nothing is given from then on, the rest recorded). 7b never reopens the device;
- dropped and counted: gyro samples (the watch reads accel), a receipt not after the last one taken
  (``not_newer``), samples with no device time, the same device time again (``dup``), a small step back
  (``late``), a sample that may predate the latest boundary (``before_boundary``). The device time is the frame
  metadata's (a 32-bit counter), else ``get_timestamp()`` in the device clock domain at its full value (the
  Jetson's RSUSB build of 2026-11-17 gave no metadata);
- a read takes at most ``limit`` queue items (samples whatever their verdict, and boundaries), and
  ``last_read_capped`` says more were due (the loop treats it as its own capped read).

With ``record=True`` every accel sample taken off the queue becomes a row (``drain_rows``): its raw fields, its
verdict, generation and interval; a generation change is an ``imu_gen`` row. Feeding a record's raw fields to a new
source gives the same rows (``replay``) -- the 7r replay reads these.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import yaml

__all__ = ["IMU_YAML", "ImuSource", "ImuSourceConfig", "fits_watch", "load_imu_source_config", "replay"]

IMU_YAML = Path(__file__).resolve().parents[3] / "config" / "real" / "imu.yaml"
WRAP_US = 1 << 32
MISMATCH_SLACK_NS = 1_000_000  # the delay spread allowed between two samples: the bound + 1 ms + DRIFT x their
DRIFT = 1e-4  # device time apart (100 ppm of device clock drift against the host's)
MARKS_PER_CALL = 64  # boundaries applied by one generation() call


@dataclass(frozen=True)
class ImuSourceConfig:
    accel_hz: int
    gyro_hz: int
    delay_bound_s: float | None  # None: not measured -> diagnostic only
    measured: bool
    provisional_delay_bound_s: float
    restart_back_us: int
    receive_gap_s: float
    queue_max: int


def _pos(v, name, *, allow_none=False):
    if v is None and allow_none:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"imu source config: {name}") from None
    if isinstance(v, bool) or not math.isfinite(f) or f <= 0:
        raise ValueError(f"imu source config: {name}")
    return f


def _int(v, name):
    if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
        raise ValueError(f"imu source config: {name}")
    return v


def load_imu_source_config(path: str | Path = IMU_YAML) -> ImuSourceConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema") != "imu_source/1":
        raise ValueError("imu source config: schema")
    s, t, g = raw["stream"], raw["timing"], raw["generation"]
    cfg = ImuSourceConfig(
        accel_hz=_int(s["accel_hz"], "stream.accel_hz"), gyro_hz=_int(s["gyro_hz"], "stream.gyro_hz"),
        delay_bound_s=_pos(t.get("delay_bound_s"), "timing.delay_bound_s", allow_none=True),
        measured=t.get("measured") is True,
        provisional_delay_bound_s=_pos(t["provisional_delay_bound_s"], "timing.provisional_delay_bound_s"),
        restart_back_us=_int(g["restart_back_us"], "generation.restart_back_us"),
        receive_gap_s=_pos(g["receive_gap_s"], "generation.receive_gap_s"),
        queue_max=_int(raw["queue_max"], "queue_max"))
    if cfg.measured and cfg.delay_bound_s is None:
        raise ValueError("imu source config: measured without timing.delay_bound_s")
    return cfg


def fits_watch(cfg: ImuSourceConfig, gap_max_s: float, *, bound_s: float | None = None,
               accel_hz: float | None = None) -> bool:
    """An armed watch can hold on this source: a sample's end minus the previous one's start -- one accel period
    (``accel_hz``: the profile actually opened, else the requested one) plus the delay bound -- stays within the
    watch's gap limit (otherwise every sample is ``watch_imu_gap``). Uneven receipt widens it further: the record's
    neighbouring receipts tell by how much."""
    b = cfg.delay_bound_s if bound_s is None else bound_s
    hz = cfg.accel_hz if accel_hz is None else accel_hz
    return b is not None and hz is not None and hz > 0 and b + 1.0 / hz <= gap_max_s


def _device_us(dev_us, ts_ms, dom):
    """The sample's device time in µs and where it came from: the frame's hardware timestamp metadata (a 32-bit
    counter), else -- on a build without metadata support, as the Jetson's RSUSB librealsense of 2026-11-17 (every
    ``dev_us`` null) -- ``get_timestamp()`` when its domain is the device clock, kept at its full value (folding it
    to 32 bits would hide a jump of a whole turn, Codex 7b round 2 P1-4); else none."""
    if not isinstance(dev_us, bool) and isinstance(dev_us, int) and 0 <= dev_us < WRAP_US:
        return dev_us, "metadata"
    if dom == "hardware_clock" and not isinstance(ts_ms, bool) and isinstance(ts_ms, (int, float)) \
            and math.isfinite(ts_ms) and ts_ms >= 0:
        return int(round(ts_ms * 1000)), "ts_ms"
    return None, None


def _device_step(last: int, dev: int, src: str, folded: bool) -> tuple[int, bool]:
    """The signed step in µs from ``last`` to ``dev``, and whether the clock is now known to be folded at 32 bits. A
    32-bit counter's step (``metadata``) is taken modulo a turn; so is a ``ts_ms`` already seen folding while both
    values stay inside 32 bits (a late sample from before the fold is a small step back -- Codex 7b round 3 P2 --
    but a value outside them steps as it is: round 4 P1-3); a full ``ts_ms`` value otherwise steps as it is, the
    first fold being both values on either side of the 32-bit boundary."""
    m = (dev - last) % WRAP_US
    inside = 0 <= last < WRAP_US and 0 <= dev < WRAP_US
    if src == "metadata" or (folded and inside):
        return (m if m <= WRAP_US // 2 else m - WRAP_US), folded
    raw = dev - last
    if raw < 0 and inside and last >= WRAP_US // 2 and dev < WRAP_US // 2 and m <= WRAP_US // 2:
        return m, True
    return raw, folded


class ImuSource:
    def __init__(self, cfg: ImuSourceConfig, *, operating: bool, record: bool = False, now_ns=time.monotonic_ns):
        if operating and (not cfg.measured or cfg.delay_bound_s is None):
            raise ValueError("imu_source:delay_unmeasured")  # design 5: diagnostic until the bound is measured
        self.cfg, self.operating, self._now = cfg, operating, now_ns
        bound = cfg.delay_bound_s if cfg.delay_bound_s is not None else cfg.provisional_delay_bound_s
        self.bound_ns = int(round(bound * 1e9))
        self._gap_ns = int(round(cfg.receive_gap_s * 1e9))
        self._lock = threading.Lock()
        # in receipt order: ("s", host_ns, dev_us, ts_ms, dom, xyz) samples and ("m", host_ns, why, until_ns, n,
        # final) generation boundaries -- a receive gap, an overflow hole or a loss sits where it happened, so the
        # samples before it keep the old generation (Codex 7b round 1 P1-2). ``until_ns``: the latest instant it
        # covers; ``n``: the generations it stands for (merged boundaries keep their count, round 4 P3); ``final``:
        # the source stops there (a loss). Adjacent boundaries are merged, so the queue never holds more boundaries
        # than samples + 1 and every field stays one value (round 3-4 P2: the callback's work under the lock is
        # bounded). Receipt times and receive gaps are decided here, on the callback's side, from one monotonic
        # receipt history (round 2 P1-2, round 4 P2)
        self._queue: deque = deque()
        self._nq = 0  # samples in the queue
        self._npend = 0  # generations the queued boundaries stand for
        self._last_rx: int | None = None  # newest accel receipt (callback side)
        self._gap_marked: int | None = None  # the receipt whose following gap is already a boundary
        self._lost_open = False  # a loss reported and no sample received since: more reports are the same loss
        # loop side
        self._gen = 1
        self.generation_why: str | None = None
        self._boundary_ns: int | None = None  # the latest boundary's time: a new sample's lo must not precede it
        # why the source gives nothing more: "clock_mismatch" (the bound broken, round 1 P1-3) or a change of device
        # clock -- a loss, a restart, a new time source (round 4 P1-1: 7b never reopens the device, so whatever
        # arrives after is of unknown provenance). Recorded; the run must be started again
        self.stopped: str | None = None
        self.last_read_capped = False  # the last read stopped at its budget with more due (round 1 P2)
        self._clk_last: int | None = None  # the last device time of this clock (as ``_device_us`` gives it)
        self._clk_src: str | None = None
        self._clk_folded = False
        self._clk_ext = 0  # its unwrapped µs since the first
        # the delay envelope in integers (10 x ns, so 100 ppm of a device time in µs is the µs value itself; round 3
        # P2): min of 10 * off - ext_us and max of 10 * off + ext_us, off = host_ns - 1000 * ext_us
        self._min_b: int | None = None
        self._max_a: int | None = None
        self._last_taken: int | None = None  # the last accepted receipt (``hi`` only increases)
        self.counts = {"accepted": 0, "gyro": 0, "missing_dev": 0, "dev_from_ts": 0, "dup": 0, "late": 0,
                       "not_newer": 0, "before_boundary": 0, "bound_broken": 0, "stopped": 0, "overflow": 0,
                       "lost": 0, "generations": 0}
        self._record = record
        self._rows: list = []

    @property
    def bound_broken(self) -> bool:
        return self.stopped == "clock_mismatch"

    # --- the SDK side: queue only ----------------------------------------------------------------------------------
    def on_sample(self, stream, dev_us, ts_ms, dom, xyz) -> None:
        self.push(None, stream, dev_us, dom, xyz, ts_ms=ts_ms)

    def push(self, host_ns, stream, dev_us, dom, xyz, *, ts_ms=None) -> None:
        """One sample as received at ``host_ns`` (None: now, read under the queue's lock -- the callback), or a
        record's raw fields."""
        with self._lock:
            if stream != "a":
                self.counts["gyro"] += 1
                return
            host = int(self._now()) if host_ns is None else int(host_ns)
            self._mark_gap(host)
            if self._nq >= self.cfg.queue_max:
                self._drop_oldest()
            self._queue.append(("s", host, dev_us, ts_ms, dom, tuple(xyz)))
            self._nq += 1
            self._last_rx = host if self._last_rx is None else max(self._last_rx, host)
            self._lost_open = False

    def _mark_gap(self, now: int) -> None:
        """(under the lock) no accel sample received for ``receive_gap_s`` before ``now``: one boundary at the last
        receipt + the gap, whether a quiet tick or the next sample sees it first (round 3 P1-1)."""
        if self._last_rx is not None and self._gap_marked != self._last_rx and now > self._last_rx + self._gap_ns:
            self._gap_marked = self._last_rx
            at = self._last_rx + self._gap_ns
            self._add_mark(("m", at, "receive_gap", at, 1, False), at_head=False)

    def _add_mark(self, mark, *, at_head: bool) -> None:
        """(under the lock) a boundary at the head or the tail, merged into a boundary already there."""
        self._npend += mark[4]
        i = 0 if at_head else -1
        if self._queue and self._queue[i][0] == "m":
            _k, h, why, until, n, final = self._queue[i]
            _k2, nh, nwhy, nuntil, nn, nfinal = mark
            keep = nwhy if (nfinal and not final) else why  # one reason, the final one first
            self._queue[i] = ("m", min(h, nh), keep, max(until, nuntil), n + nn, final or nfinal)
            return
        if at_head:
            self._queue.appendleft(mark)
        else:
            self._queue.append(mark)

    def _drop_oldest(self) -> None:
        """(under the lock) the oldest sample out -- behind at most one boundary, as boundaries are merged -- and a
        boundary at the hole covering up to the dropped sample's receipt (round 3 P2)."""
        head = None
        if self._queue and self._queue[0][0] == "m":
            head = self._queue.popleft()
            self._npend -= head[4]
        if self._queue and self._queue[0][0] == "s":
            e = self._queue.popleft()
            self._nq -= 1
            self.counts["overflow"] += 1
            self._add_mark(("m", e[1], "overflow", e[1], 1, False), at_head=True)
        if head is not None:
            self._add_mark(head, at_head=True)

    def lost(self, why: str) -> None:
        """The device gone (from any thread): a final boundary after the samples received so far. Reported again
        before any sample is received, it is the same loss (counted only -- round 2 P2, round 3 P3)."""
        with self._lock:
            self.counts["lost"] += 1
            if self._lost_open:
                return
            self._lost_open = True
            now = int(self._now())
            self._add_mark(("m", now, "lost:" + str(why), now, 1, True), at_head=False)

    # --- the loop side ---------------------------------------------------------------------------------------------
    def _new_generation(self, why: str, host_ns: int | None, n: int = 1) -> None:
        self._gen += n
        self.generation_why = why
        self.counts["generations"] += n
        if self._record:
            self._rows.append({"t": "imu_gen", "gen": self._gen, "why": why, "host_ns": host_ns})

    def _stop(self, why: str, host_ns: int) -> None:
        if self.stopped is None:
            self.stopped = why
            self._new_generation(why, host_ns)

    def _apply_mark(self, mark) -> None:
        _k, host, why, until, n, final = mark
        self._new_generation(why, host, n)
        self._boundary_ns = until if self._boundary_ns is None else max(self._boundary_ns, until)
        if final and self.stopped is None:
            self.stopped = why

    @property
    def gen(self) -> int:
        """The generation of the samples given so far (no check made)."""
        return self._gen

    def generation(self) -> int:
        """The producer's current generation: the samples' own plus every generation the boundaries already queued
        stand for -- a loss behind a sample still waiting counts at once (round 2 P1-1). A receive gap seen now is
        queued first; boundaries at the head of the queue are applied (at most ``MARKS_PER_CALL``) -- each with no
        new sample (design 5, Codex round 5 P2)."""
        now = int(self._now())
        marks = []
        with self._lock:
            self._mark_gap(now)
            while self._queue and self._queue[0][0] == "m" and len(marks) < MARKS_PER_CALL:
                m = self._queue.popleft()
                self._npend -= m[4]
                marks.append(m)
            pending = self._npend
        for m in marks:
            self._apply_mark(m)
        return self._gen + pending

    def samples(self, upto_ns: int, limit: int) -> list:
        """``(gen, lo, hi, accel)`` of the accel samples received up to ``upto_ns``, oldest first. At most ``limit``
        queue items -- samples whatever their verdict, and boundaries -- are taken per call (the read cost, design
        3.3); ``last_read_capped`` says that more were due."""
        out = []
        taken = 0
        self.last_read_capped = False
        while True:
            with self._lock:
                if not self._queue or self._queue[0][1] > int(upto_ns):
                    break
                if taken >= int(limit):
                    self.last_read_capped = True
                    break
                e = self._queue.popleft()
                if e[0] == "s":
                    self._nq -= 1
                else:
                    self._npend -= e[4]
            taken += 1
            if e[0] == "m":
                self._apply_mark(e)
                continue
            item = self._take(*e[1:])
            if item is not None:
                out.append(item)
        return out

    def _take(self, host, dev_us, ts_ms, dom, xyz):
        dev, dev_src = _device_us(dev_us, ts_ms, dom)
        verdict = self._verdict(host, dev, dev_src)
        item = None
        if verdict == "ok":
            self._last_taken = host
            self.counts["accepted"] += 1
            if dev_src == "ts_ms":
                self.counts["dev_from_ts"] += 1
            item = (self._gen, host - self.bound_ns, host, xyz)
        else:
            self.counts[verdict] += 1
        if self._record:
            self._rows.append({"t": "imu", "host_ns": host, "dev_us": dev_us, "ts_ms": ts_ms, "dom": dom,
                               "x": xyz[0], "y": xyz[1], "z": xyz[2], "dev": dev, "dev_src": dev_src,
                               "verdict": verdict, "gen": self._gen,
                               "lo": None if item is None else item[1], "hi": None if item is None else item[2]})
        return item

    def _verdict(self, host, dev, src) -> str:
        if self._last_taken is not None and host <= self._last_taken:
            return "not_newer"  # a receipt going back is never given (round 1 P2), whatever the device time says
        if self.stopped is not None:
            return "bound_broken" if self.stopped == "clock_mismatch" else "stopped"
        if dev is None:
            return "missing_dev"
        if self._clk_last is not None and src != self._clk_src:
            self._stop("device_time_source", host)  # a new device clock (round 4 P1-1)
            return "stopped"
        ext = 0
        folded = self._clk_folded
        if self._clk_last is not None:
            step, folded = _device_step(self._clk_last, dev, src, self._clk_folded)
            if step == 0:
                return "dup"  # the same frame again, whether or not its first copy was counted
            if step < 0:
                if -step <= self.cfg.restart_back_us:
                    return "late"
                self._stop("device_restart", host)  # a new device clock (round 4 P1-1)
                return "stopped"
            ext = self._clk_ext + step
        # every sample of the device clock against one delay bound (round 2 P1-3) -- a sample not counted after a
        # boundary included, as the clock goes on across receive gaps and overflows (round 4 P1-2): any two samples
        # i, j must have |off_j - off_i| <= bound + 1 ms + 100 ppm * |t_j - t_i|
        off10 = 10 * (host - 1000 * ext)
        b, a = off10 - ext, off10 + ext
        tol10 = 10 * (self.bound_ns + MISMATCH_SLACK_NS)
        if self._min_b is not None and (b - self._min_b > tol10 or self._max_a - a > tol10):
            # the delays of this clock's samples spread wider than the bound allows: the bound is broken (or a
            # restart the step cannot see). Nothing is given from here on (design 3.1; round 1 P1-3)
            self._stop("clock_mismatch", host)
            return "bound_broken"
        self._clk_last, self._clk_src, self._clk_ext, self._clk_folded = dev, src, ext, folded
        self._min_b = b if self._min_b is None else min(self._min_b, b)
        self._max_a = a if self._max_a is None else max(self._max_a, a)
        if self._boundary_ns is not None and host - self.bound_ns < self._boundary_ns:
            # may have been taken before the latest generation boundary (design 3.1: after a boundary is lo >= it,
            # round 2 P1-2): not counted toward the new generation
            return "before_boundary"
        return "ok"

    def drain_rows(self) -> list:
        rows, self._rows = self._rows, []
        return rows


def replay(cfg: ImuSourceConfig, rows) -> ImuSource:
    """A recording source fed with the raw fields of a record's ``imu`` rows (an observe record's or an
    ``imu_record/1`` file's, whose ``s`` names the stream), each taken off the queue as it comes: its
    ``drain_rows()`` are the rows the record would hold. A receive gap is decided from the receipts, so it replays;
    not raw input, so not replayed: a loss and queue overflows."""
    src = ImuSource(cfg, operating=False, record=True)
    for r in rows:
        if r.get("t") == "imu":
            src.push(int(r["host_ns"]), r.get("s", "a"), r.get("dev_us"), r.get("dom"), (r["x"], r["y"], r["z"]),
                     ts_ms=r.get("ts_ms"))
            src.samples(int(r["host_ns"]), 1 << 30)
    return src
