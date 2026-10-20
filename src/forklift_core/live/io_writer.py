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
- ``close(timeout_s)``: refuses new jobs, waits for every job within the deadline and stops the thread; ``False``
  if it did not drain. A Python thread cannot be stopped from outside, so a job still on after a close keeps running
  in its daemon thread, and its outcome is not reported.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from typing import Callable


class OrderedWriter:
    def __init__(self, *, max_keyed: int = 4, max_line_bytes: int = 16 << 20, name: str = "live-io") -> None:
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
        self._thread = threading.Thread(target=self._loop, name=name, daemon=True)
        self._thread.start()

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


def encode_in_pieces(value, default, *, piece_items: int = 256, yield_every: int = 16):
    """``json.dumps(value, default=default)`` in pieces (joined, the same text): a long list goes ``piece_items``
    items at a time and containers are entered, so no single C call encodes a whole large record -- the writer
    thread gives the interpreter lock back between pieces (a one-shot encode of a 2 MB record held it 11.6 ms
    against the control thread, 2026-10-20). Every ``yield_every`` pieces it also sleeps 0 s, so a waiting thread
    gets the lock at once rather than after the switch interval."""
    count = 0

    def leaf(v) -> str:
        nonlocal count
        count += 1
        if count % yield_every == 0:
            time.sleep(0)
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
