"""Measuring the runner's cost (real-time plan S0, docs/plans/2026-10-17-realtime-sim.md): region timers and
function-input dumps. Off unless asked for; neither changes what the measured code computes.

- ``TIMERS``: wall and thread CPU time per named region (``with TIMERS.region("name"):`` or ``TIMERS.wrap``).
  Disabled it costs one attribute read. The live runner enables it and logs ``TIMERS.take()`` once a second.
  ``wrap`` on a class times every instance (the bundle permit's own permissions too) and leaves the instances'
  state alone, so they still pickle.
- ``dumped(name)``: a decorator. With ``FORKLIFT_PERF_DUMP=<dir>`` it writes ``<dir>/<name>/<pid>_<n>.pkl.gz`` for
  every N-th call (``FORKLIFT_PERF_DUMP_EVERY``: ``50``, or per function ``check_arc=50,grid_snapshot=5``), the
  first K calls of each stratum (``FORKLIFT_PERF_DUMP_STRATA_K``, default 5) and, for a function named in
  ``FORKLIFT_PERF_DUMP_STREAM`` (``shadow_apply=300``), the first calls of each receiver one after another. A
  record holds the receiver, the arguments and the keyword arguments before the call (copied as one object
  graph, so an argument that is the receiver's own attribute stays so), the same after it, and the result, so
  ``tools/perf_bench.py`` can run the call again on another implementation and compare.
"""

from __future__ import annotations

import atexit
import copy
import dataclasses
import functools
import gzip
import json
import os
import pickle
import struct
import sys
import threading
import time
from collections import deque
from contextlib import contextmanager
from pathlib import Path

import numpy as np

_HERE = os.path.abspath(__file__)


def caller_name(skip: int = 1) -> str:
    """The name of the first frame outside this module above the caller (timers and dumps wrap each other)."""
    frame = sys._getframe(skip + 1)
    while frame is not None and os.path.abspath(frame.f_code.co_filename) == _HERE:
        frame = frame.f_back
    return frame.f_code.co_name if frame is not None else "?"


class Timers:
    def __init__(self) -> None:
        self.enabled = False
        self._total: dict[str, float] = {}
        self._count: dict[str, int] = {}
        self._max: dict[str, float] = {}
        self._cpu: dict[str, float] = {}
        # Real-time plan 5.18 G2-0: the long-call record. ``long_s`` None (default): off, nothing is kept and no
        # stack is held. A number: every region call of at least that wall time is kept as it ends, with its caller
        # path (the enclosing regions), its self time (the nested regions' time taken out, recorded or not), its
        # thread CPU time and ``where()``'s context (the live runner's trace row), up to ``long_cap`` records.
        self.long_s: float | None = None
        self.long: list[dict] = []
        self.long_cap = 200_000
        self.long_dropped = 0
        self.long_overhead_s = 0.0  # the record's own keeping time (taken out of the enclosing calls' self time)
        self.where = None
        self._long_lock = threading.Lock()
        self._local = threading.local()  # each thread's own stack of open regions: [name, start, nested wall s]

    def sched_mark(self) -> None:
        """A phase's start (the live runner marks each control tick's and each physics step's), for the long-call
        record's scheduling wait: this thread's run-queue wait (/proc/thread-self/schedstat) is read here and again
        as a long call ends, so a kept call carries the wait since its phase began -- not the call's own wait
        (that would need a read at every call's start)."""
        self._local.sched = (time.perf_counter(), _run_delay_s())

    def _stack(self) -> list:
        stack = getattr(self._local, "stack", None)
        if stack is None:
            stack = self._local.stack = []
        return stack

    def _enter(self, name: str, start: float) -> None:
        self._stack().append([name, start, 0.0])

    def _leave(self, name: str, start: float, seconds: float, cpu_s: float) -> None:
        stack = self._stack()
        frame = stack.pop() if stack else [name, start, 0.0]
        if seconds < self.long_s:
            if stack:
                stack[-1][2] += seconds
            return
        keep_start = time.perf_counter()
        delay_now = _run_delay_s()  # first, before the keeping's own work
        record = {"name": name, "path": [f[0] for f in stack], "start": start, "end": start + seconds,
                  "self_s": seconds - frame[2], "cpu_s": cpu_s}
        if threading.current_thread() is not threading.main_thread():
            record["thread"] = threading.current_thread().name
        mark = getattr(self._local, "sched", None)
        if mark is not None and mark[1] is not None and delay_now is not None:
            record["run_delay_s_since_mark"] = delay_now - mark[1]
            record["since_mark_s"] = start + seconds - mark[0]
        if self.where is not None:
            try:
                record.update(self.where())
            except Exception as exc:  # noqa: BLE001 -- a context that cannot be read is noted, never raised
                record["where_error"] = f"{type(exc).__name__}: {exc}"
        with self._long_lock:
            if len(self.long) >= self.long_cap:
                self.long_dropped += 1
            else:
                self.long.append(record)
        overhead = time.perf_counter() - keep_start
        self.long_overhead_s += overhead
        if stack:
            # The enclosing call's self time leaves out this call and the keeping of its record (Codex
            # instrumentation review P2-2: the keeping went to the parent).
            stack[-1][2] += seconds + overhead

    def add(self, name: str, seconds: float, cpu_s: float | None = None) -> None:
        self._total[name] = self._total.get(name, 0.0) + seconds
        self._count[name] = self._count.get(name, 0) + 1
        if seconds > self._max.get(name, 0.0):
            self._max[name] = seconds
        if cpu_s is not None:
            self._cpu[name] = self._cpu.get(name, 0.0) + cpu_s

    @contextmanager
    def region(self, name: str):
        if not self.enabled:
            yield
            return
        start, cpu = time.perf_counter(), time.thread_time()
        long_on = self.long_s is not None
        if long_on:
            self._enter(name, start)
        try:
            yield
        finally:
            seconds, cpu_s = time.perf_counter() - start, time.thread_time() - cpu
            self.add(name, seconds, cpu_s)
            if long_on:
                self._leave(name, start, seconds, cpu_s)

    def wrap(self, owner, attribute: str, name: str | None = None, *, by_caller: bool = False,
             long_only: bool = False, by_parent: bool = False) -> None:
        """Time every call of ``owner.attribute`` as one region: a class's method (every instance), an
        instance's method or a module's function. ``by_caller``: one region per calling function
        (``name<caller``). ``long_only``: the call enters the long-call record only (its stack and records), not
        the per-second totals ``take`` returns -- finer regions that leave the status line's timers as they were;
        it costs nothing while the long-call record is off. ``by_parent``: with the long-call record on, one region
        per enclosing region (``name|parent``, the innermost open region on this thread; ``|`` keeps it apart from
        ``by_caller``'s ``<``), so an inner step's total is split by the call it ran inside (Codex instrumentation
        review round 4 P2; 5.22 review: a parent's direct children are the keys whose part after the first ``|`` is
        the parent)."""
        raw = owner.__dict__[attribute] if isinstance(owner, type) else getattr(owner, attribute)
        static = isinstance(raw, staticmethod)
        fn = raw.__func__ if static else raw
        label = name or attribute

        @functools.wraps(fn)
        def timed(*args, **kwargs):
            if not self.enabled or (long_only and self.long_s is None):
                return fn(*args, **kwargs)
            where = f"{label}<{caller_name()}" if by_caller else label
            long_on = self.long_s is not None
            if by_parent and long_on:
                stack_ = self._stack()
                where = f"{where}|{stack_[-1][0]}" if stack_ else where
            start, cpu = time.perf_counter(), time.thread_time()
            if long_on:
                self._enter(where, start)
            try:
                return fn(*args, **kwargs)
            finally:
                seconds, cpu_s = time.perf_counter() - start, time.thread_time() - cpu
                if not long_only:
                    self.add(where, seconds, cpu_s)
                if long_on:
                    self._leave(where, start, seconds, cpu_s)

        setattr(owner, attribute, staticmethod(timed) if static else timed)

    def take(self) -> dict:
        """{region: [wall s, calls, longest call's wall s, this thread's CPU s or None]} since the last take
        (nested regions each count their own time; the CPU time tells waiting from computing)."""
        out = {k: [round(v, 6), self._count[k], round(self._max.get(k, 0.0), 6),
                   round(self._cpu[k], 6) if k in self._cpu else None] for k, v in self._total.items()}
        self._total, self._count, self._max, self._cpu = {}, {}, {}, {}
        return out


def _run_delay_s() -> float | None:
    """This thread's time waiting on the run queue (Linux schedstat), or None where it cannot be read."""
    try:
        with open("/proc/thread-self/schedstat", "rb") as f:
            return int(f.read().split()[1]) / 1e9
    except (OSError, ValueError, IndexError):
        return None


TIMERS = Timers()

_DUMP_DIR = os.environ.get("FORKLIFT_PERF_DUMP")


def _every(spec: str, default: int = 50) -> dict[str, int]:
    """``"50"`` (every function) or ``"check_arc=50,grid_snapshot=5"`` (per function; others ``default``)."""
    out = {"*": default}
    for part in filter(None, (p.strip() for p in spec.split(","))):
        name, _, n = part.rpartition("=")
        out[name or "*"] = max(0, int(n))
    return out


_DUMP_EVERY = _every(os.environ.get("FORKLIFT_PERF_DUMP_EVERY", "50"))
_STREAM = _every(os.environ.get("FORKLIFT_PERF_DUMP_STREAM", ""), default=0)
_STRATA_K = int(os.environ.get("FORKLIFT_PERF_DUMP_STRATA_K", "5"))
_calls: dict[str, int] = {}
_strata: dict[tuple, int] = {}
_streams: dict[tuple, list] = {}  # (name, id of receiver) -> [receiver (kept alive: no id reuse), calls so far]
_last: dict = {}
CONTEXT: dict = {}  # what the runner says it is doing (the mission phase), part of every stratum


def changed(key, value) -> bool:
    """Whether ``value`` differs from the one last given for ``key`` (True the first time), for strata."""
    previous = _last.get(key, _last)
    _last[key] = value
    return previous is _last or previous != value


def shape_kind(shape) -> str:
    """A stratum's name for a checked shape: one footprint by its front, or a parts union by its part count."""
    if isinstance(shape, (list, tuple)):
        return f"parts{len(shape)}"
    return f"fp{getattr(shape, 'front_m', 0.0):.2f}"


def bucket(value: float, edges=(1e-9, 0.05, 0.2)) -> int:
    """Signed size class of a value (0: zero), for strata."""
    v = abs(float(value))
    return (1 if value > 0 else -1) * sum(v >= e for e in edges)


def differences(a, b, path="", out=None, limit=5):
    """Paths where a and b differ bit for bit, at most ``limit`` (at least one is always looked for)."""
    out = [] if out is None else out
    limit = max(1, limit)
    if len(out) >= limit:
        return out
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        if not (isinstance(a, np.ndarray) and isinstance(b, np.ndarray)):
            out.append((path, type(a).__name__, type(b).__name__))
        elif a.dtype != b.dtype or a.shape != b.shape:
            out.append((path, f"{a.dtype}{a.shape}", f"{b.dtype}{b.shape}"))
        elif a.dtype == object:
            for i, (x, y) in enumerate(zip(a.ravel(), b.ravel(), strict=True)):
                differences(x, y, f"{path}[{i}]", out, limit)
        elif np.ascontiguousarray(a).tobytes() != np.ascontiguousarray(b).tobytes():
            ca, cb = np.ascontiguousarray(a), np.ascontiguousarray(b)
            bad = np.argwhere(ca.view(np.uint8).reshape(a.shape + (-1,)) != cb.view(np.uint8).reshape(b.shape + (-1,)))
            where = tuple(int(v) for v in bad[0][:-1]) if len(bad) else ()
            out.append((path + str(list(where)), repr(a[where]) if where else "?", repr(b[where]) if where else "?"))
        return out
    if isinstance(a, np.generic) or isinstance(b, np.generic):
        if type(a) is not type(b) or np.asarray(a).tobytes() != np.asarray(b).tobytes():
            out.append((path, f"{type(a).__name__}:{a!r}", f"{type(b).__name__}:{b!r}"))
        return out
    if isinstance(a, float) and isinstance(b, float):
        if struct.pack("<d", a) != struct.pack("<d", b):
            out.append((path, repr(a), repr(b)))
        return out
    if type(a) is not type(b):
        out.append((path, f"{type(a).__name__}:{a!r}"[:80], f"{type(b).__name__}:{b!r}"[:80]))
        return out
    if dataclasses.is_dataclass(a):
        for f in dataclasses.fields(a):
            differences(getattr(a, f.name), getattr(b, f.name), f"{path}.{f.name}", out, limit)
        return out
    if isinstance(a, dict):
        if list(a) != list(b):
            out.append((path + " keys", f"{len(a)} keys", f"{len(b)} keys"
                        + (" (same set, other order)" if set(a) == set(b) else "")))
            return out
        for k in a:
            differences(a[k], b[k], f"{path}[{k!r}]", out, limit)
        return out
    if isinstance(a, (set, frozenset)):
        if a != b:
            out.append((path, f"{len(a)} items", f"{len(b)} items, {len(a ^ b)} differ"))
        return out
    if isinstance(a, (list, tuple, deque)):
        if len(a) != len(b):
            out.append((path, f"len {len(a)}", f"len {len(b)}"))
            return out
        for i, (x, y) in enumerate(zip(a, b, strict=True)):
            differences(x, y, f"{path}[{i}]", out, limit)
        return out
    if hasattr(a, "__dict__") and not isinstance(a, type):
        return differences(vars(a), vars(b), path, out, limit)
    if a != b:
        out.append((path, repr(a)[:80], repr(b)[:80]))
    return out


def _write(name: str, n: int, record: dict) -> None:
    out = Path(_DUMP_DIR) / name
    out.mkdir(parents=True, exist_ok=True)
    with gzip.open(out / f"{os.getpid()}_{n:07d}.pkl.gz", "wb", compresslevel=1) as f:
        pickle.dump(record, f, protocol=pickle.HIGHEST_PROTOCOL)


def dumped(name: str, *, pure: bool = False, strata=None):
    """Dump calls of a method (see the module docstring); the method itself when off.

    ``strata(receiver, args, kwargs, result)`` names a call's stratum (the caller and ``CONTEXT``'s phase are
    added). ``pure``: the call changes neither its receiver nor its arguments, so nothing is copied before it
    and the choice can wait for the result; otherwise every call is copied first (meant for the 10 Hz ones).
    """

    def wrap(fn):
        if not _DUMP_DIR:
            return fn

        @functools.wraps(fn)
        def inner(receiver, *args, **kwargs):
            n = _calls.get(name, 0)
            _calls[name] = n + 1
            every_n = _DUMP_EVERY.get(name, _DUMP_EVERY["*"])
            due = bool(every_n) and not n % every_n
            limit = _STREAM.get(name, _STREAM["*"])
            stream = None
            if limit:
                stream = _streams.setdefault((name, id(receiver)), [receiver, 0])
                if stream[1] >= limit:
                    stream = None
            if pure:
                if not due and strata is None and stream is None:
                    return fn(receiver, *args, **kwargs)
                result = fn(receiver, *args, **kwargs)
                before = None
            else:
                before = copy.deepcopy((receiver, args, kwargs))
                result = fn(receiver, *args, **kwargs)
            stratum = None
            if strata is not None:
                stratum = (name, caller_name(), CONTEXT.get("phase"), *strata(receiver, args, kwargs, result))
            first = stratum is not None and _strata.get(stratum, 0) < _STRATA_K
            if first:
                _strata[stratum] = _strata.get(stratum, 0) + 1
            if due or first or stream is not None:
                record = {"name": name, "call": n, "stratum": stratum, "pure": pure,
                          "stream": None if stream is None else [os.getpid(), id(receiver), stream[1]],
                          "before": before, "after": copy.deepcopy((receiver, args, kwargs)),
                          "result": copy.deepcopy(result)}
                _write(name, n, record)
                if stream is not None:
                    stream[1] += 1
            return result

        return inner

    return wrap


# --- the dual run (real-time plan V2) -----------------------------------------------------------------------------
_DUAL = os.environ.get("FORKLIFT_KERNELS_DUAL")  # the candidate implementation, e.g. "numba"
_DUAL_LOG = os.environ.get("FORKLIFT_KERNELS_DUAL_LOG")  # where the counts and first mismatches go (JSON)
DUAL_STATS: dict = {}  # name -> {"calls", "same", "mismatches": [first few]}
_dual_carried: dict = {}  # (name, id of receiver) -> [receiver (kept alive), the candidate's own receiver]
_in_candidate = False


def _dual_report() -> None:
    if _DUAL and _DUAL_LOG:  # "%s" in the path becomes the process id (several runs in one job)
        report = {"asked": _DUAL, "stats": DUAL_STATS}
        Path(_DUAL_LOG.replace("%s", str(os.getpid()))).write_text(json.dumps(report, indent=1, default=str))


if _DUAL:
    atexit.register(_dual_report)  # an Isaac run may end without it: the report is also written as it goes


def dual(name: str, *, carried: bool = False):
    """With ``FORKLIFT_KERNELS_DUAL=<mode>``: every call runs as usual (the implementation the run chose leads),
    then again under ``<mode>`` on a copy of the state before the call, and the result, the receiver after the
    call and the arguments after it are compared bit for bit (``differences``). ``carried``: the candidate keeps
    its own receiver from call to call (state it carries), started from the first call's. The function itself
    when off."""

    def wrap(fn):
        if not _DUAL:
            return fn

        @functools.wraps(fn)
        def inner(receiver, *args, **kwargs):
            global _in_candidate
            if _in_candidate:
                return fn(receiver, *args, **kwargs)
            from forklift_core import kernels

            candidate = kernels._resolve(_DUAL)  # "auto" names the reference too (Codex real-time round 7 P2)
            if kernels.mode() == candidate:
                # The leading run must be another implementation, or the candidate is compared with itself (Codex
                # real-time round 6 P1): FORKLIFT_KERNELS=reference with FORKLIFT_KERNELS_DUAL=numba.
                raise ValueError(f"dual run: the leading implementation is the candidate ({candidate}) itself")
            DUAL_STATS.setdefault("_modes", {"leading": kernels.mode(), "candidate": candidate})
            if carried:
                slot = _dual_carried.setdefault((name, id(receiver)), [receiver, None])
                if slot[1] is None:
                    slot[1] = copy.deepcopy(receiver)
                cand_args, cand_kwargs = copy.deepcopy((args, kwargs))
                cand_receiver = slot[1]
            else:
                cand_receiver, cand_args, cand_kwargs = copy.deepcopy((receiver, args, kwargs))
            result = fn(receiver, *args, **kwargs)
            previous = kernels.set_mode(candidate)
            _in_candidate = True
            try:
                cand_result = fn(cand_receiver, *cand_args, **cand_kwargs)
            finally:
                _in_candidate = False
                kernels.set_mode(previous)
            diffs = differences(result, cand_result, "result", limit=3)
            differences((receiver, args, kwargs), (cand_receiver, cand_args, cand_kwargs), "after", diffs, limit=3)
            entry = DUAL_STATS.setdefault(name, {"calls": 0, "same": 0, "mismatches": []})
            entry["calls"] += 1
            entry["same"] += not diffs
            if diffs or entry["calls"] in (1, 10, 100) or entry["calls"] % 500 == 0:
                _dual_report()
            if diffs:
                if len(entry["mismatches"]) < 10:
                    entry["mismatches"].append({"call": entry["calls"] - 1, "caller": caller_name(),
                                                "phase": CONTEXT.get("phase"), "first": diffs})
                if carried:
                    # Carry on from the real state, so one mismatch is counted once and not in every later call.
                    _dual_carried[(name, id(receiver))][1] = copy.deepcopy(receiver)
            return result

        return inner

    return wrap


__all__ = ["CONTEXT", "DUAL_STATS", "TIMERS", "Timers", "bucket", "caller_name", "changed", "differences",
           "dual", "dumped", "shape_kind"]
