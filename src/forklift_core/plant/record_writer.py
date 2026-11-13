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
    def __init__(self, sink, *, max_rows: int = MAX_ROWS) -> None:
        self.sink = sink
        self.q: queue.Queue = queue.Queue(maxsize=max_rows)
        self.dropped = 0
        self.write_errors = 0
        self._thread = threading.Thread(target=self._run, name="record-writer", daemon=True)
        self._thread.start()

    def put_row(self, row: dict) -> None:
        self._put(row)

    def put_state(self, sample: Sample, read_ns: int | None) -> None:
        self._put(("state", sample, read_ns))

    def _put(self, item) -> None:
        try:
            self.q.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while True:
            item = self.q.get()
            if item is _STOP:
                return
            try:
                self.sink.write(_line(item) + "\n")
            except (OSError, ValueError, TypeError):
                self.write_errors += 1

    def close(self, footer: dict | None = None, timeout_s: float = 5.0) -> None:
        """Drain, write the footer (the counts as they stand), close the sink. A writer that does not finish within
        ``timeout_s`` leaves the rest unwritten and counts it as dropped."""
        while True:
            try:
                self.q.put(_STOP, timeout=timeout_s)
                break
            except queue.Full:
                self.dropped += 1
                break
        self._thread.join(timeout_s)
        if self._thread.is_alive():
            self.dropped += self.q.qsize()
        elif footer is not None:
            try:
                self.sink.write(_line({"t": "footer", **footer, "dropped": self.dropped,
                                       "write_errors": self.write_errors}) + "\n")
            except (OSError, ValueError, TypeError):
                self.write_errors += 1
        try:
            self.sink.close()
        except OSError:
            self.write_errors += 1
