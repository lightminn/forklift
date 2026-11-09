"""Real-time plan G2-1a: where the capture's diagnostic files are written.

The pocket detection tick wrote the capture's RGB PNG, depth array and depth picture on the control thread (about
0.044 s of a 0.16-0.22 s tick; docs/validation/2026-11-09-g2-measure-results.md). They are diagnostic -- offline
analyses read them after the run; no decision in the run does -- so a live session may hand them to its writer thread
(``OrderedWriter.put_line``, the far recognition's files go the same way). The writes themselves are the runner's own
function, unchanged, on copies of the arrays taken now (the capture stays the mission's).

- No writer (a batch run, or --live-capture-dump sync), or a boundary record open (its sink observes the writes on the
  control thread): written here, as before.
- A writer that refuses the job (its byte budget full): written here too -- the files are never lost.
- A job that fails on the writer: its error is kept on the entry and printed (the mission goes on).
"""

from __future__ import annotations

import sys

import numpy as np


def dispatch_capture_dump(io, record_open: bool, write, attempt_number, rgb, depth_m, entries: list) -> str:
    """Writes or hands over attempt ``attempt_number``'s files; returns how: ``"sync"``, ``"async"`` or
    ``"sync_refused"``. ``write(attempt_number, rgb, depth_m)`` does the writes; ``entries`` gets one record per
    handed-over or refused dump (``{"attempt", "mode"[, "error"]}``)."""
    if io is None or record_open:
        write(attempt_number, rgb, depth_m)
        return "sync"
    rgb_, depth_ = np.array(rgb, copy=True), np.array(depth_m, copy=True)
    entry = {"attempt": attempt_number, "mode": "async"}
    entries.append(entry)

    def job() -> None:
        try:
            write(attempt_number, rgb_, depth_)
        except Exception as exc:  # noqa: BLE001 -- a diagnostic file; the mission does not depend on it
            entry["error"] = f"{type(exc).__name__}: {exc}"
            print(f"capture dump {attempt_number} failed: {entry['error']}", file=sys.stderr, flush=True)

    if io.put_line(job, rgb_.nbytes + depth_.nbytes):
        return "async"
    entry["mode"] = "sync_refused"
    write(attempt_number, rgb_, depth_)
    return "sync_refused"
