"""The latency-tolerance test's injections (docs/plans/2026-11-11-latency-tolerance.md). Test-only: every one is off
unless its runner option is given, and off nothing here is constructed.

- ``SlamReplyQueue`` (A, ``--latency-slam``): a batch run's lockstep SLAM replies delivered at the scan tick + n
  instead of at once (plan 2.1). The runner drains it at every physics step's start and right after a scan's exchange,
  so n = 0 delivers where the lockstep code always did. ``jitter:M`` draws 0..M per reply; a reply is never delivered
  before the one ahead of it.
- ``layer_k_fn`` (A, ``--latency-layer``): the install delay of each obstacle-layer job (plan 2.2), fixed or drawn.
- ``UseMarks``: whether each delivered reply / installed job was first used by a control tick (and when), replaced
  before any control tick read it, or never used before the end (plan 2, 3rd edition Codex P2-3).
- ``AppliedWatch`` / ``install_applied``: with a late layer install, ``obstacle["applied"]``/``version`` describe the
  installed snapshot; an install writes its job's correction only when nothing wrote ``applied`` after the job was
  prepared (a reprojection or another site did -- that newer state stands; plan 2.2, 3rd edition Codex P1-1).
- ``DelayedJobs`` (B, ``--latency-plan-ms``): a live run's planning answers held until their arrival + d (plan 2.4).

The jitter streams come from ``channel_rng(seed, channel)``: one seed, a separate stream per channel."""

from __future__ import annotations

import time
import zlib

import numpy as np

SLAM_KINDS = ("ticks", "jitter")
LAYER_KINDS = ("undeclared", "declared", "jitter")
LAYER_MAX_K = 12  # one scan period: the job is installed before the next scan's (plan 2.2)


def _spec(text: str, kinds: tuple, max_n: int | None, what: str):
    if text == "off":
        return None
    kind, sep, n = str(text).partition(":")
    if not sep or kind not in kinds:
        raise ValueError(f"{what} {text!r}: off or {'|'.join(f'{k}:N' for k in kinds)}")
    try:
        value = int(n)
    except ValueError:
        raise ValueError(f"{what} {text!r}: N must be an integer") from None
    if value < 0 or (max_n is not None and value > max_n) or (kind == "jitter" and value < 1):
        raise ValueError(f"{what} {text!r}: N out of range")
    return kind, value


def parse_slam(text: str):
    """``off`` -> None; ``ticks:N`` (N >= 0) or ``jitter:M`` (M >= 1) -> (kind, value)."""
    return _spec(text, SLAM_KINDS, None, "--latency-slam")


def parse_layer(text: str):
    """``off`` -> None; ``undeclared:k`` / ``declared:k`` (0..12) or ``jitter:M`` (1..12) -> (kind, value)."""
    return _spec(text, LAYER_KINDS, LAYER_MAX_K, "--latency-layer")


def channel_rng(seed: int, channel: str) -> np.random.Generator:
    return np.random.default_rng([int(seed), zlib.crc32(channel.encode())])


class SlamReplyQueue:
    def __init__(self, spec: tuple, seed: int):
        self.kind, self.n = spec
        self._rng = channel_rng(seed, "slam") if self.kind == "jitter" else None
        self._queue: list = []  # [(due tick, record index, item)], due ticks non-decreasing
        self.records: list[dict] = []

    @property
    def pending(self) -> int:
        return len(self._queue)

    def push(self, item, *, scan_tick: int, moving: bool, holding: bool, ident=None) -> int:
        own = int(self._rng.integers(0, self.n + 1)) if self._rng is not None else self.n
        due = int(scan_tick) + own
        if self._queue:
            due = max(due, self._queue[-1][0])  # never before the reply ahead of it
        self.records.append({**({"id": ident} if ident is not None else {}),
                             "scan_tick": int(scan_tick), "own_ticks": own, "due_tick": due,
                             "moving_at_scan": bool(moving), "holding_at_scan": bool(holding),
                             "delivered_tick": None})
        self._queue.append((due, len(self.records) - 1, item))
        return due

    def drain(self, tick: int, *, moving: bool | None = None, holding: bool | None = None) -> list:
        out = []
        while self._queue and self._queue[0][0] <= int(tick):
            _, index, item = self._queue.pop(0)
            rec = self.records[index]
            rec["delivered_tick"] = int(tick)
            if moving is not None:
                rec["moving_at_delivery"] = bool(moving)
            if holding is not None:
                rec["holding_at_delivery"] = bool(holding)
            out.append(item)
        return out

    def last_index(self) -> int:
        return len(self.records) - 1

    def truncate(self) -> int:
        """The end of the run: what is still queued is recorded as cut, never delivered."""
        for _, index, _ in self._queue:
            self.records[index]["truncated"] = True
        left = len(self._queue)
        self._queue = []
        return left


def layer_k_fn(spec: tuple, seed: int):
    kind, value = spec
    if kind != "jitter":
        return lambda: value
    rng = channel_rng(seed, "layer")
    return lambda: int(rng.integers(0, value + 1))


class UseMarks:
    def __init__(self):
        self._open: dict = {}  # channel -> id delivered since the last control tick (the latest wins)
        self._out: dict = {}

    def delivered(self, channel: str, ident, *, tick: int) -> None:
        prev = self._open.get(channel)
        if prev is not None:
            self._out.setdefault(channel, {})[prev] = {"use": "superseded"}
        self._open[channel] = ident
        self._out.setdefault(channel, {})[ident] = {"use": "unused"}

    def control_tick(self, tick: int, t: float) -> None:
        for channel, ident in self._open.items():
            self._out[channel][ident] = {"use": "used", "tick": int(tick), "t": float(t)}
        self._open = {}

    def result(self) -> dict:
        return {c: dict(v) for c, v in self._out.items()}


class AppliedWatch(dict):
    """The obstacle dict under a late layer install: counts the writes of ``applied``."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.writes = 0

    def __setitem__(self, key, value):
        if key == "applied":
            self.writes += 1
        super().__setitem__(key, value)


def install_applied(obstacle: AppliedWatch, info: dict) -> bool:
    """At a job's install: the installed snapshot's correction and version, unless ``applied`` was written after the
    job was prepared (the newer write describes what is installed after it)."""
    if obstacle.writes != info["writes"]:
        return False
    obstacle["applied"] = tuple(float(v) for v in info["correction"])
    obstacle["version"] = int(info["version"])
    return True


class DelayedJobs:
    """``PlanWorker`` with each answer held until its arrival (the I/O thread's receipt) + ``delay_s``."""

    def __init__(self, jobs, delay_s: float, *, clock=time.monotonic):
        if not delay_s >= 0:
            raise ValueError("--latency-plan-ms must be >= 0")
        self._jobs, self.delay_s, self._clock = jobs, float(delay_s), clock
        self.records: list[dict] = []

    def poll(self, job_id: int):
        arrived = self._jobs.arrival(job_id)
        if arrived is None:
            return None
        now = self._clock()
        if now < arrived + self.delay_s:
            return None
        out = self._jobs.poll(job_id)
        if out is not None:
            self.records.append({"job": int(job_id), "arrived": float(arrived), "delivered": float(now),
                                 "injected_s": float(now - arrived)})
        return out

    def __getattr__(self, name):
        return getattr(self._jobs, name)
