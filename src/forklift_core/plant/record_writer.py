"""A JSONL record that never holds up a control loop (docs/design/2026-11-14-teleop-record-state-markers-clock.md 3).

``RecordWriter``: rows go into a bounded queue; one thread turns them into JSON lines and writes them. A full queue
drops the new row and counts it -- the loop is never blocked (no writer can promise no loss, bounded memory and a disk
that never stalls all at once). ``close`` drains the queue, writes a footer with the counts and closes the sink.
``InlineRecord`` has the same interface and writes at once (a test's ``io.StringIO``).
"""

from __future__ import annotations

import json
import queue
import threading

from forklift_core.plant.bringup import Sample
from forklift_core.plant.record_rows import state_row

MAX_ROWS = 20_000
_STOP = object()


def _line(item) -> str:
    if isinstance(item, tuple) and item and item[0] == "state":
        _, sample, read_ns = item
        row = state_row(sample)
        if read_ns is not None:
            row["read_ns"] = read_ns
        return json.dumps(row, default=str)
    return json.dumps(item, default=str)


class InlineRecord:
    def __init__(self, sink) -> None:
        self.sink = sink
        self.dropped = 0
        self.write_errors = 0

    def put_row(self, row: dict) -> None:
        self._write(row)

    def put_state(self, sample: Sample, read_ns: int | None) -> None:
        self._write(("state", sample, read_ns))

    def _write(self, item) -> None:
        try:
            self.sink.write(_line(item) + "\n")
        except (OSError, ValueError, TypeError):
            self.write_errors += 1

    def close(self, footer: dict | None = None, timeout_s: float = 5.0) -> None:
        if footer is not None:
            self._write({"t": "footer", **footer, "dropped": self.dropped, "write_errors": self.write_errors})


class RecordWriter:
    """The writer thread owns the sink: it writes the rows, then -- once ``close`` asks and the queue is empty -- the
    footer, and closes the sink. ``close`` never touches the sink itself; it waits ``timeout_s`` and reports whether
    the writer finished (``close_result``). A row offered after ``close`` began is dropped and counted."""

    def __init__(self, sink, *, max_rows: int = MAX_ROWS) -> None:
        self.sink = sink
        self.q: queue.Queue = queue.Queue(maxsize=max_rows)
        self.dropped = 0
        self.write_errors = 0
        self._lock = threading.Lock()
        self._closing = False
        self._stop = threading.Event()
        self._footer: dict | None = None
        self._pending = 0  # rows accepted and not yet written: from the put to the end of the write (one counter)
        self.close_result: dict | None = None
        self._thread = threading.Thread(target=self._run, name="record-writer", daemon=True)
        self._thread.start()

    def put_row(self, row: dict) -> None:
        self._put(row)

    def put_state(self, sample: Sample, read_ns: int | None) -> None:
        self._put(("state", sample, read_ns))

    def _put(self, item) -> None:
        with self._lock:
            if self._closing:
                self.dropped += 1
                return
            try:
                self.q.put_nowait(item)
                self._pending += 1
            except queue.Full:
                self.dropped += 1

    def _write(self, item) -> None:
        try:
            self.sink.write(_line(item) + "\n")
        except (OSError, ValueError, TypeError):
            self.write_errors += 1

    def _take_and_write(self, item) -> None:
        self._write(item)
        with self._lock:
            self._pending -= 1

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                item = self.q.get(timeout=0.05)
            except queue.Empty:
                continue
            self._take_and_write(item)
        # close set ``_closing`` under the lock before the stop: no row can be accepted any more, so draining what is
        # queued now is complete (Codex code re-review P2-1: a row accepted between an empty get and the stop)
        while True:
            try:
                item = self.q.get_nowait()
            except queue.Empty:
                break
            self._take_and_write(item)
        if self._footer is not None:
            self._write({"t": "footer", **self._footer, "dropped": self.dropped, "write_errors": self.write_errors})
        try:
            self.sink.close()
        except OSError:
            self.write_errors += 1

    def close(self, footer: dict | None = None, timeout_s: float = 5.0) -> dict:
        """Stop taking rows, let the writer drain and close. Returns (and keeps as ``close_result``) the counts as
        they stood: ``finished`` False when the writer did not end within ``timeout_s`` -- its ``unwritten`` rows and
        the footer may then never reach the file, and the sink is left to the writer."""
        with self._lock:
            self._closing = True
            self._footer = footer
        self._stop.set()
        self._thread.join(timeout_s)
        finished = not self._thread.is_alive()
        with self._lock:
            unwritten = 0 if finished else self._pending
        self.close_result = {"finished": finished, "unwritten": unwritten,
                             "dropped": self.dropped, "write_errors": self.write_errors}
        return self.close_result
