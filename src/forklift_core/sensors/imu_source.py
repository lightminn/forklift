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
- a new generation on a device time stepping back more than ``restart_back_us`` (a restart), device and host steps
  apart by more than the bound (the bound broken, or a restart the unwrap cannot see), no accel sample for
  ``receive_gap_s`` (seen by ``generation()`` with no sample, as a restart must be), a queue overflow, or ``lost()``
  (the device gone). A wrap of the 32-bit device time is a forward step and changes nothing;
- dropped and counted: gyro samples (the watch reads accel), samples with no device time, the same device time
  again (``dup``), a small step back (``late``), a receipt not after the last one taken (``not_newer``).

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
MISMATCH_SLACK_NS = 1_000_000  # device and host step comparison: 1 ms of slack on top of the delay bound
DRIFT = 1e-4  # and 100 ppm of the step (device clock drift)


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


def fits_watch(cfg: ImuSourceConfig, gap_max_s: float, *, bound_s: float | None = None) -> bool:
    """An armed watch can hold on this source: a sample's end minus the previous one's start -- one accel period
    plus the delay bound -- stays within the watch's gap limit (otherwise every sample is ``watch_imu_gap``)."""
    b = cfg.delay_bound_s if bound_s is None else bound_s
    return b is not None and b + 1.0 / cfg.accel_hz <= gap_max_s


class ImuSource:
    def __init__(self, cfg: ImuSourceConfig, *, operating: bool, record: bool = False, now_ns=time.monotonic_ns):
        if operating and (not cfg.measured or cfg.delay_bound_s is None):
            raise ValueError("imu_source:delay_unmeasured")  # design 5: diagnostic until the bound is measured
        self.cfg, self.operating, self._now = cfg, operating, now_ns
        bound = cfg.delay_bound_s if cfg.delay_bound_s is not None else cfg.provisional_delay_bound_s
        self.bound_ns = int(round(bound * 1e9))
        self._gap_ns = int(round(cfg.receive_gap_s * 1e9))
        self._lock = threading.Lock()
        self._queue: deque = deque()  # [host_ns, dev_us, dom, xyz, overflow-before-it]
        self._last_rx: int | None = None  # newest accel receipt (callback side)
        self._lost: str | None = None  # a loss reported from any thread, applied on the loop side
        # loop side
        self._gen = 1
        self.generation_why: str | None = None
        self._last_dev: int | None = None  # the last accepted sample's device µs, this generation
        self._last_host: int | None = None  # its receipt
        self._last_seen: int | None = None  # the last receipt taken off the queue, any verdict
        self._gap_open = False
        self.counts = {"accepted": 0, "gyro": 0, "missing_dev": 0, "dup": 0, "late": 0, "not_newer": 0,
                       "overflow": 0, "generations": 0}
        self._record = record
        self._rows: list = []

    # --- the SDK side: queue only ----------------------------------------------------------------------------------
    def on_sample(self, stream, dev_us, ts_ms, dom, xyz) -> None:  # noqa: ARG002 -- the SDK callback's signature
        self.push(self._now(), stream, dev_us, dom, xyz)

    def push(self, host_ns: int, stream, dev_us, dom, xyz) -> None:
        """One sample as received at ``host_ns`` (the callback, or a record's raw fields)."""
        if stream != "a":
            with self._lock:
                self.counts["gyro"] += 1
            return
        with self._lock:
            overflow = False
            if len(self._queue) >= self.cfg.queue_max:
                self._queue.popleft()
                self.counts["overflow"] += 1
                overflow = True
            if overflow and self._queue:
                self._queue[0][4] = True  # the samples after the hole are a new generation
                overflow = False
            self._queue.append([int(host_ns), dev_us, dom, tuple(xyz), overflow])
            self._last_rx = int(host_ns)

    def lost(self, why: str) -> None:
        """The device gone (from any thread): the next read opens a generation."""
        with self._lock:
            self._lost = str(why)

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
        """The current generation; a loss reported, or no accel sample received for ``receive_gap_s``, opens one
        here even with no sample (design 5, Codex round 5 P2)."""
        now = int(self._now())
        with self._lock:
            lost, self._lost = self._lost, None
            last_rx = self._last_rx
        if lost is not None:
            self._new_generation(lost, now)
        if last_rx is not None and not self._gap_open and now - last_rx > self._gap_ns:
            self._gap_open = True
            self._new_generation("receive_gap", now)
        return self._gen

    def samples(self, upto_ns: int, limit: int) -> list:
        """``(gen, lo, hi, accel)`` of the accel samples received up to ``upto_ns``, oldest first, at most
        ``limit``; later ones stay queued."""
        with self._lock:
            lost, self._lost = self._lost, None
        if lost is not None:
            self._new_generation(lost, None)
        out = []
        while len(out) < int(limit):
            with self._lock:
                if not self._queue or self._queue[0][0] > int(upto_ns):
                    break
                host, dev, dom, xyz, overflow = self._queue.popleft()
            item = self._take(host, dev, dom, xyz, overflow)
            if item is not None:
                out.append(item)
        return out

    def _take(self, host, dev, dom, xyz, overflow):
        if overflow:
            self._new_generation("overflow", host)
        if self._last_seen is not None and not self._gap_open and host - self._last_seen > self._gap_ns:
            self._new_generation("receive_gap", host)
        self._gap_open = False
        self._last_seen = host
        verdict = self._verdict(host, dev)
        item = None
        if verdict == "ok":
            self._last_dev, self._last_host = dev, host
            self.counts["accepted"] += 1
            item = (self._gen, host - self.bound_ns, host, xyz)
        else:
            self.counts[verdict] += 1
        if self._record:
            self._rows.append({"t": "imu", "host_ns": host, "dev_us": dev, "dom": dom, "x": xyz[0], "y": xyz[1],
                               "z": xyz[2], "verdict": verdict, "gen": self._gen,
                               "lo": None if item is None else item[1], "hi": None if item is None else item[2]})
        return item

    def _verdict(self, host, dev) -> str:
        if isinstance(dev, bool) or not isinstance(dev, int) or not 0 <= dev < WRAP_US:
            return "missing_dev"
        if self._last_dev is not None:
            step = (dev - self._last_dev) % WRAP_US  # forward across a wrap
            if step == 0:
                return "dup"
            if step > WRAP_US // 2:
                if WRAP_US - step <= self.cfg.restart_back_us:
                    return "late"
                self._new_generation("device_restart", host)
            else:
                host_step = host - self._last_host
                if abs(host_step - step * 1000) > self.bound_ns + MISMATCH_SLACK_NS + step * 1000 * DRIFT:
                    self._new_generation("clock_mismatch", host)
        if self._last_host is not None and host <= self._last_host:
            return "not_newer"
        return "ok"

    def drain_rows(self) -> list:
        rows, self._rows = self._rows, []
        return rows


def replay(cfg: ImuSourceConfig, rows) -> ImuSource:
    """A recording source fed with the raw fields of a record's ``imu`` rows (an observe record's or an
    ``imu_record/1`` file's, whose ``s`` names the stream), all taken off the queue: its ``drain_rows()`` are the
    rows the record would hold. Generations opened by ``generation()`` with no sample (a receive gap seen on a quiet
    tick, a reported loss) are not raw input and are not replayed."""
    src = ImuSource(cfg, operating=False, record=True)
    for r in rows:
        if r.get("t") == "imu":
            src.push(int(r["host_ns"]), r.get("s", "a"), r.get("dev_us"), r.get("dom"), (r["x"], r["y"], r["z"]))
            src.samples(int(r["host_ns"]), 1 << 30)
    return src
