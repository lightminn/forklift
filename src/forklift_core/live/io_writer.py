"""The one file writer of a live session (persistent console plan D1 step 7, use case U29).

The control thread must not wait on the disk: a slow disk put the end tick's record cost at 11-37 ms against its
20 ms bound when the mission's files were written on the control thread and a thread was started per mission
(2026-10-20, Codex U29 design review). Here one long-lived thread does the session's writes -- the live log's lines
and each mission's files -- in the order they were handed over; the control thread only appends to a queue.

- ``put_line(job, nbytes)``: an unkeyed job (a log line). Lines past ``max_line_bytes`` waiting are dropped and
  counted; a line job that raises is counted (``line_errors``) and the writer goes on.
- ``put_keyed(key, job)``: a keyed job (a mission's files) returning its errors (a list of text, empty: written). At
  most ``max_keyed`` keyed jobs are pending -- the one being written counts -- so a full writer refuses the job
  (``False``, counted) rather than making the control thread wait; the caller marks it. A job that raises is the
  error ``"<Type>: <message>"``.
- ``outcomes()``: the keyed jobs finished since the last call, ``(key, errors)``, in order (the control thread notes
  them on its results).
- ``settle(timeout_s)``: waits, within one deadline, until no keyed job is pending -- queued or being written -- and
  returns the keys still pending at the deadline.
- ``dispose(value)``: lets ``value`` go on a second thread (``live-io-free``, no disk), ``dispose_after_s`` after the
  hand-over -- by then the caller's own references are gone, so the last one, and the freeing of a large record,
  is that thread's (records a full writer refused were freed on the control thread: 120,000 rows, 31 ms).
- ``close(timeout_s)``: refuses new jobs, waits for every job within the deadline and stops the thread; ``False``
  if it did not drain. A Python thread cannot be stopped from outside, so a job still on after a close keeps running
  in its daemon thread, and its outcome is not reported.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from collections import deque
from typing import Callable


class OrderedWriter:
    def __init__(self, *, max_keyed: int = 4, max_line_bytes: int = 16 << 20, dispose_after_s: float = 1.0,
                 name: str = "live-io") -> None:
        self.max_keyed, self.max_line_bytes = int(max_keyed), int(max_line_bytes)
        self._cond = threading.Condition()
        self._queue: deque = deque()  # (key or None, job, nbytes)
        self._pending_keys: list = []  # keyed jobs queued or being written, in order
        self._line_bytes = 0  # the line jobs' bytes queued
        self._outcomes: list = []
        self._busy = False
        self._closed = False
        self._abandoned = False  # a close passed its deadline: later outcomes are not reported
        self.dropped_lines = self.dropped_keyed = self.line_errors = 0
        self.dispose_after_s = float(dispose_after_s)
        self._disposals: deque = deque()  # (due monotonic, value)
        self._free_cond = threading.Condition()
        self._thread = threading.Thread(target=self._loop, name=name, daemon=True)
        self._thread.start()
        self._freer = threading.Thread(target=self._free_loop, name=f"{name}-free", daemon=True)
        self._freer.start()

    def dispose(self, value) -> None:
        with self._free_cond:
            self._disposals.append((time.monotonic() + self.dispose_after_s, value))
            self._free_cond.notify_all()
        del value

    # --- the control thread -----------------------------------------------------------------------------------------
    def put_line(self, job: Callable[[], None], nbytes: int) -> bool:
        with self._cond:
            if self._closed:
                return False
            if self._line_bytes + nbytes > self.max_line_bytes:
                self.dropped_lines += 1
                return False
            self._line_bytes += nbytes
            self._queue.append((None, job, nbytes))
            self._cond.notify_all()
            return True

    def put_keyed(self, key, job: Callable[[], list]) -> bool:
        with self._cond:
            if self._closed:
                return False
            if len(self._pending_keys) >= self.max_keyed:
                self.dropped_keyed += 1
                return False
            self._pending_keys.append(key)
            self._queue.append((key, job, 0))
            self._cond.notify_all()
            return True

    def outcomes(self) -> list:
        with self._cond:
            out, self._outcomes = self._outcomes, []
            return out

    def settle(self, timeout_s: float) -> list:
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._pending_keys:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self._cond.wait(left)
            return list(self._pending_keys)

    def close(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self._cond:
            self._closed = True
            self._cond.notify_all()
            while self._queue or self._busy:
                left = deadline - time.monotonic()
                if left <= 0:
                    self._abandoned = True
                    return False
                self._cond.wait(left)
        self._thread.join(max(0.0, deadline - time.monotonic()))
        return True

    # --- the disposer thread ----------------------------------------------------------------------------------------
    def _free_loop(self) -> None:
        while True:
            with self._free_cond:
                while not self._disposals:
                    self._free_cond.wait()
                due = self._disposals[0][0]
                left = due - time.monotonic()
                if left > 0:
                    self._free_cond.wait(left)
                    continue
                _, value = self._disposals.popleft()
            holder = [value]
            del value
            free_gradually(holder)  # the last reference: freed here, a bit at a time

    # --- the writer thread ------------------------------------------------------------------------------------------
    def _loop(self) -> None:
        while True:
            with self._cond:
                while not self._queue and not self._closed:
                    self._cond.wait()
                if not self._queue:
                    return  # closed and drained
                key, job, nbytes = self._queue.popleft()
                self._line_bytes -= nbytes
                self._busy = True
            errors = None
            try:
                result = job()
                if key is not None:
                    errors = list(result or [])
            except Exception as exc:  # noqa: BLE001 -- one job's failure is its outcome; the writer goes on
                if key is not None:
                    errors = [f"{type(exc).__name__}: {exc}"]
                else:
                    with self._cond:
                        self.line_errors += 1
            with self._cond:
                self._busy = False
                if key is not None:
                    self._pending_keys.remove(key)
                    if not self._abandoned:
                        self._outcomes.append((key, errors))
                self._cond.notify_all()


class Yielder:
    """Hands the interpreter lock to a waiting thread: after ``work_s`` of work, a real sleep of ``rest_s``. A 0 s
    sleep gives the lock up without handing it over, so the control thread waking from its pacer still waited the
    switch interval (5 ms) behind a busy writer (2026-10-20 gate: wake-ups 3.3 ms late, up to 57 ms)."""

    def __init__(self, work_s: float = 0.001, rest_s: float = 0.0002) -> None:
        self.work_s, self.rest_s = float(work_s), float(rest_s)
        self._since = time.perf_counter()

    def maybe(self) -> None:
        if time.perf_counter() - self._since >= self.work_s:
            time.sleep(self.rest_s)
            self._since = time.perf_counter()


def free_gradually(holder: list, yielder: Yielder | None = None, chunk: int = 256) -> None:
    """Free the value ``holder`` holds (popped from it, so the caller keeps no reference) a bit at a time, yielding:
    one large record freed in one go holds the interpreter lock for its whole length (120,000 rows: 31 ms). Only a
    container nothing else holds is emptied (its reference count says so); a part shared with live code is only let
    go, never changed."""
    y = yielder or Yielder()
    stack = [holder.pop()]
    while stack:
        v = stack.pop()
        if not isinstance(v, (list, dict, tuple)) or sys.getrefcount(v) > 2:  # 2: v and the call's own argument
            del v
            continue
        if isinstance(v, tuple):
            stack.extend(x for x in v if isinstance(x, (list, dict, tuple)))
        elif isinstance(v, list):
            while v:
                tail = v[-chunk:]
                del v[-chunk:]
                stack.extend(x for x in tail if isinstance(x, (list, dict, tuple)))
                del tail
                y.maybe()
        else:
            while v:
                _, item = v.popitem()
                if isinstance(item, (list, dict, tuple)):
                    stack.append(item)
                del item
                if len(v) % chunk == 0:
                    y.maybe()
        del v
        y.maybe()


def write_pieces_atomic(path, pieces) -> int:
    """The text ``pieces`` streamed to a temporary file beside ``path`` and renamed over it (a reader never sees half a
    file, and no whole-file string is made); returns the bytes written."""
    from pathlib import Path

    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    size = 0
    with open(tmp, "w", encoding="utf-8") as handle:
        for piece in pieces:
            handle.write(piece)
            size += len(piece.encode("utf-8")) if not piece.isascii() else len(piece)
    os.replace(tmp, path)
    return size


def encode_in_pieces(value, default, *, piece_items: int = 256, yielder: Yielder | None = None):
    """``json.dumps(value, default=default)`` in pieces (joined, the same text): lists and tuples go ``piece_items``
    items at a time and text-keyed dicts are entered, so the writer thread gives the interpreter lock back between
    pieces (a one-shot encode of a 2 MB record held it 11.6 ms against the control thread, 2026-10-20), and a
    ``Yielder`` hands it over. Not split (each one piece, however large): what ``default`` turns into data (an ndarray's list) and a
    dict with keys JSON would turn into text."""
    y = yielder or Yielder()

    def leaf(v) -> str:
        y.maybe()
        return json.dumps(v, default=default)

    def big(v) -> bool:
        return isinstance(v, (list, tuple)) and len(v) > piece_items or (
            isinstance(v, dict) and all(isinstance(k, str) for k in v))

    def walk(v):
        if isinstance(v, dict) and all(isinstance(k, str) for k in v):
            yield "{"
            for i, (k, item) in enumerate(v.items()):
                yield (", " if i else "") + json.dumps(k) + ": "
                yield from walk(item) if big(item) or isinstance(item, (list, tuple)) else (leaf(item),)
            yield "}"
        elif isinstance(v, (list, tuple)):
            yield "["
            first, buffer = True, []
            for item in v:
                if big(item):
                    if buffer:
                        yield ("" if first else ", ") + leaf(buffer)[1:-1]
                        first, buffer = False, []
                    yield "" if first else ", "
                    first = False
                    yield from walk(item)
                else:
                    buffer.append(item)
                    if len(buffer) >= piece_items:
                        yield ("" if first else ", ") + leaf(buffer)[1:-1]
                        first, buffer = False, []
            if buffer:
                yield ("" if first else ", ") + leaf(buffer)[1:-1]
            yield "]"
        else:
            yield leaf(v)

    return walk(value)
