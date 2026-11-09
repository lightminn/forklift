"""Real-time plan G2-1a: the capture's diagnostic files off the control thread.

The pocket detection tick wrote the capture's RGB PNG, depth array and depth picture on the control thread (about
0.044 s of a 0.16-0.22 s tick; docs/validation/2026-11-09-g2-measure-results.md). They are diagnostic -- offline
analyses read them after the run; no decision in the run does -- so a live session may write them on a thread of
their own. The writes themselves are the runner's own function, unchanged, on copies of the arrays taken now (the
capture stays the mission's).

``CaptureDumper`` (Codex G2-1a review round 1):
- its own ``OrderedWriter`` (thread ``live-capture``): the live log's writer and its byte budget are not shared, so a
  capture never pushes a log line out nor counts as a lost one (P2-2);
- every capture is a keyed job in one FIFO, so writes to the same path keep their order (the attempt numbers restart
  per mission, the file names carry no mission id; P2-4). The control thread never waits on it and never writes past
  it (round 2 P2-1/2/3): with ``max_pending`` (16, about 55 MB of copies) already waiting -- a disk stalled for many
  captures -- a new capture is dropped and recorded ``dropped_full``, its files not written;
- its records live with it for the whole session (``entries``), apart from the run's per-mission state (P1);
- ``close(timeout_s)`` ends the writer within the deadline and marks every handed-over capture not yet written
  ``unsettled`` -- a write still running in the daemon thread after that is not reported as done (P2-3).

A boundary record open (its sink observes the writes on the control thread): written in place, as before.
"""

from __future__ import annotations

import sys

import numpy as np

from forklift_core.live.io_writer import OrderedWriter


class CaptureDumper:
    def __init__(self, write, *, max_pending: int = 16, writer=None) -> None:
        self.write = write
        self.io = writer if writer is not None else OrderedWriter(max_keyed=max_pending, name="live-capture")
        self.entries: list[dict] = []
        self.closed = None

    def dump(self, attempt_number, rgb, depth_m, *, record_open: bool = False) -> str:
        """Writes or hands over attempt ``attempt_number``'s files; returns how: ``"sync"`` (a record open),
        ``"async"`` or ``"dropped_full"``."""
        if record_open:
            self.write(attempt_number, rgb, depth_m)
            return "sync"
        self.io.outcomes()  # the finished jobs' notes are not kept (the entries are the record)
        rgb_, depth_ = np.array(rgb, copy=True), np.array(depth_m, copy=True)
        entry = {"attempt": attempt_number, "mode": "async"}
        key = len(self.entries)
        self.entries.append(entry)

        def job() -> list:
            try:
                self.write(attempt_number, rgb_, depth_)
            except Exception as exc:  # noqa: BLE001 -- a diagnostic file; the mission does not depend on it
                entry["error"] = f"{type(exc).__name__}: {exc}"
                print(f"capture dump {attempt_number} failed: {entry['error']}", file=sys.stderr, flush=True)
                return [entry["error"]]
            entry["done"] = True
            return []

        if self.io.put_keyed(key, job):
            return "async"
        entry["mode"] = "dropped_full"  # never written here: no wait on the control thread, no write past the queue
        return "dropped_full"

    def close(self, timeout_s: float) -> bool:
        """The writer ended within ``timeout_s``; every handed-over capture not written by then is marked."""
        self.closed = bool(self.io.close(timeout_s))
        for e in self.entries:
            if e["mode"] == "async" and not e.get("done") and "error" not in e:
                e["unsettled"] = True
        return self.closed

    def record(self) -> dict:
        return {"mode": "async", "closed": self.closed, "entries": [dict(e) for e in self.entries],
                "dropped_full": sum(e["mode"] == "dropped_full" for e in self.entries),
                "errors": sum("error" in e for e in self.entries),
                "unsettled": sum(bool(e.get("unsettled")) for e in self.entries)}
