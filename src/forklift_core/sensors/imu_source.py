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
- a new generation on a device time stepping back more than ``restart_back_us`` (a restart), no accel sample for
  ``receive_gap_s`` (seen by ``generation()`` with no sample waiting, as a restart must be), a queue overflow, or
  ``lost()`` (the device gone) -- the last two are boundaries kept in the queue where they happened, so the samples
  before them keep the old generation. A wrap of the 32-bit device time is a forward step and changes nothing;
- device and host steps apart by more than the bound + 1 ms: the bound is broken -- a new generation, and no sample
  is given from then on (``bound_broken``, recorded);
- dropped and counted: gyro samples (the watch reads accel), a receipt not after the last one taken
  (``not_newer``), samples with no device time, the same device time again (``dup``), a small step back
  (``late``). The device time is the frame metadata's, else ``get_timestamp()`` in the device clock domain (the
  Jetson's RSUSB build of 2026-11-17 gave no metadata);
- a read takes at most ``limit`` raw samples off the queue, whatever their verdict, and ``last_read_capped`` says
  more were due (the loop treats it as its own capped read).

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
MISMATCH_SLACK_NS = 1_000_000  # device and host step comparison: 1 ms of slack on top of the delay bound (steps
# are under receive_gap_s, so 100 ppm of device clock drift is under 10 µs of it)


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
    """The sample's device time in µs and where it came from: the frame's hardware timestamp metadata, else -- on a
    build without metadata support, as the Jetson's RSUSB librealsense of 2026-11-17 (every ``dev_us`` null) --
    ``get_timestamp()`` when its domain is the device clock; else none."""
    if not isinstance(dev_us, bool) and isinstance(dev_us, int) and 0 <= dev_us < WRAP_US:
        return dev_us, "metadata"
    if dom == "hardware_clock" and not isinstance(ts_ms, bool) and isinstance(ts_ms, (int, float)) \
            and math.isfinite(ts_ms) and ts_ms >= 0:
        return int(round(ts_ms * 1000)) % WRAP_US, "ts_ms"
    return None, None


class ImuSource:
    def __init__(self, cfg: ImuSourceConfig, *, operating: bool, record: bool = False, now_ns=time.monotonic_ns):
        if operating and (not cfg.measured or cfg.delay_bound_s is None):
            raise ValueError("imu_source:delay_unmeasured")  # design 5: diagnostic until the bound is measured
        self.cfg, self.operating, self._now = cfg, operating, now_ns
        bound = cfg.delay_bound_s if cfg.delay_bound_s is not None else cfg.provisional_delay_bound_s
        self.bound_ns = int(round(bound * 1e9))
        self._gap_ns = int(round(cfg.receive_gap_s * 1e9))
        self._lock = threading.Lock()
        # in receipt order: ("s", host_ns, dev_us, ts_ms, dom, xyz) samples and ("m", host_ns, why) generation
        # boundaries -- a loss or an overflow hole sits where it happened, so the samples before it keep the old
        # generation and those after it get the new one (Codex 7b round 1 P1-2)
        self._queue: deque = deque()
        self._nq = 0  # samples in the queue (the boundaries are not counted)
        self._last_rx: int | None = None  # newest accel receipt (callback side)
        # loop side
        self._gen = 1
        self.generation_why: str | None = None
        self.bound_broken = False  # a delay over the bound was seen: no sample is given any more (round 1 P1-3)
        self.last_read_capped = False  # the last read stopped at ``limit`` raw samples with more due (round 1 P2)
        self._last_dev: int | None = None  # the last accepted sample's device µs, this generation
        self._last_host: int | None = None  # its receipt
        self._last_taken: int | None = None  # the last accepted receipt, across generations (``hi`` only increases)
        self._last_seen: int | None = None  # the newest receipt taken off the queue, any verdict
        self._gap_open = False
        self.counts = {"accepted": 0, "gyro": 0, "missing_dev": 0, "dev_from_ts": 0, "dup": 0, "late": 0,
                       "not_newer": 0, "bound_broken": 0, "overflow": 0, "generations": 0}
        self._record = record
        self._rows: list = []

    # --- the SDK side: queue only ----------------------------------------------------------------------------------
    def on_sample(self, stream, dev_us, ts_ms, dom, xyz) -> None:
        self.push(self._now(), stream, dev_us, dom, xyz, ts_ms=ts_ms)

    def push(self, host_ns: int, stream, dev_us, dom, xyz, *, ts_ms=None) -> None:
        """One sample as received at ``host_ns`` (the callback, or a record's raw fields)."""
        with self._lock:
            if stream != "a":
                self.counts["gyro"] += 1
                return
            if self._nq >= self.cfg.queue_max:
                self._drop_oldest()
            self._queue.append(("s", int(host_ns), dev_us, ts_ms, dom, tuple(xyz)))
            self._nq += 1
            self._last_rx = int(host_ns)

    def _drop_oldest(self) -> None:
        """(under the lock) the oldest sample out; a boundary marks the hole, after any boundary before it."""
        kept = []
        while self._queue:
            e = self._queue.popleft()
            if e[0] == "m":
                kept.append(e)
                continue
            self._nq -= 1
            self.counts["overflow"] += 1
            if not (kept and kept[-1][2] == "overflow"):
                kept.append(("m", e[1], "overflow"))
            break
        for e in reversed(kept):
            self._queue.appendleft(e)

    def lost(self, why: str) -> None:
        """The device gone (from any thread): a boundary after the samples received so far."""
        with self._lock:
            self._queue.append(("m", int(self._now()), str(why)))

    # --- the loop side ---------------------------------------------------------------------------------------------
    def _new_generation(self, why: str, host_ns: int | None) -> None:
        self._gen += 1
        self.generation_why = why
        self.counts["generations"] += 1
        self._last_dev = self._last_host = None
        if self._record:
            self._rows.append({"t": "imu_gen", "gen": self._gen, "why": why, "host_ns": host_ns})

    @property
    def gen(self) -> int:
        """The current generation as it stands (no check made)."""
        return self._gen

    def generation(self) -> int:
        """The current generation: boundaries at the head of the queue (a loss, an overflow hole with no sample
        behind it yet) are applied, and with no sample waiting, no accel sample received for ``receive_gap_s``
        opens one -- each with no new sample (design 5, Codex round 5 P2)."""
        now = int(self._now())
        marks = []
        with self._lock:
            while self._queue and self._queue[0][0] == "m" and self._queue[0][1] <= now:
                marks.append(self._queue.popleft())
            waiting, last_rx = self._nq, self._last_rx
        for _k, host, why in marks:
            self._new_generation(why, host)
        if waiting == 0 and last_rx is not None and not self._gap_open and now - last_rx > self._gap_ns:
            self._gap_open = True
            self._new_generation("receive_gap", now)
        return self._gen

    def samples(self, upto_ns: int, limit: int) -> list:
        """``(gen, lo, hi, accel)`` of the accel samples received up to ``upto_ns``, oldest first. At most
        ``limit`` raw samples are taken off the queue per call, whatever their verdict (the read cost, design 3.3);
        ``last_read_capped`` says that more were due."""
        out = []
        taken = 0
        self.last_read_capped = False
        while True:
            with self._lock:
                if not self._queue or self._queue[0][1] > int(upto_ns):
                    break
                if self._queue[0][0] == "s" and taken >= int(limit):
                    self.last_read_capped = True
                    break
                e = self._queue.popleft()
                if e[0] == "s":
                    self._nq -= 1
            if e[0] == "m":
                self._new_generation(e[2], e[1])
                continue
            taken += 1
            item = self._take(*e[1:])
            if item is not None:
                out.append(item)
        return out

    def _take(self, host, dev_us, ts_ms, dom, xyz):
        if self._last_seen is not None and not self._gap_open and host - self._last_seen > self._gap_ns:
            self._new_generation("receive_gap", host)
        self._gap_open = False
        self._last_seen = host if self._last_seen is None else max(self._last_seen, host)
        dev, dev_src = _device_us(dev_us, ts_ms, dom)
        verdict = self._verdict(host, dev)
        item = None
        if verdict == "ok":
            self._last_dev, self._last_host, self._last_taken = dev, host, host
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

    def _verdict(self, host, dev) -> str:
        if self._last_taken is not None and host <= self._last_taken:
            return "not_newer"  # a receipt going back is never given (round 1 P2), whatever the device time says
        if dev is None:
            return "missing_dev"
        if self.bound_broken:
            return "bound_broken"
        if self._last_dev is not None:
            step = (dev - self._last_dev) % WRAP_US  # forward across a wrap
            if step == 0:
                return "dup"
            if step > WRAP_US // 2:
                if WRAP_US - step <= self.cfg.restart_back_us:
                    return "late"
                self._new_generation("device_restart", host)
            elif abs((host - self._last_host) - step * 1000) > self.bound_ns + MISMATCH_SLACK_NS:
                # the device and host steps differ by more than any delay within the bound allows: the bound is
                # broken (or a restart the unwrap cannot see). Nothing is given from here on -- an interval under a
                # broken bound is no interval (design 3.1; Codex 7b round 1 P1-3)
                self._new_generation("clock_mismatch", host)
                self.bound_broken = True
                return "bound_broken"
        return "ok"

    def drain_rows(self) -> list:
        rows, self._rows = self._rows, []
        return rows


def replay(cfg: ImuSourceConfig, rows) -> ImuSource:
    """A recording source fed with the raw fields of a record's ``imu`` rows (an observe record's or an
    ``imu_record/1`` file's, whose ``s`` names the stream), each taken off the queue as it comes: its
    ``drain_rows()`` are the rows the record would hold. Not raw input, so not replayed: generations opened with no
    sample (a receive gap seen on a quiet tick, a loss) and queue overflows."""
    src = ImuSource(cfg, operating=False, record=True)
    for r in rows:
        if r.get("t") == "imu":
            src.push(int(r["host_ns"]), r.get("s", "a"), r.get("dev_us"), r.get("dom"), (r["x"], r["y"], r["z"]),
                     ts_ms=r.get("ts_ms"))
            src.samples(int(r["host_ns"]), 1 << 30)
    return src
