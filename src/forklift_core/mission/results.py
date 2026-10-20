"""The results book of a live session (docs/plans/2026-10-18-persistent-live-console.md D1 step 9).

A live session runs many missions; each one's outcome is kept here for the session and published as the
``missions`` message (stored like the targets list, so a page connecting later still gets it). Only the newest
results stay in memory; each mission's full record is on disk (D1 step 7).
"""

from __future__ import annotations

import time
from collections import deque

OUTCOMES = ("done", "failed", "cancelled", "recovery_required")


class ResultsBook:
    KEEP = 100

    def __init__(self, keep: int = KEEP, wall=time.time) -> None:
        self.seq = 0
        self._wall = wall  # D13 item 1: the wall time a result is made at, stamped once (not the monotonic clock)
        self._entries: deque = deque(maxlen=keep)

    def add(self, *, mission_id: str, outcome: str, reason: str | None, started_s: float | None, ended_s: float,
            pallet_id: str | None, destination_id: str | None, advice: dict | None = None) -> dict:
        if outcome not in OUTCOMES:
            raise ValueError(f"unknown outcome {outcome!r}")
        self.seq += 1
        entry = {"seq": self.seq, "mission_id": mission_id, "outcome": outcome, "reason": reason,
                 "started_s": started_s, "ended_s": ended_s, "ended_unix": float(self._wall()),
                 "pallet_id": pallet_id, "destination_id": destination_id}
        if advice is not None:
            entry["advice"] = dict(advice)  # D13 item 6: how far back before the next preparation's start passes
        self._entries.append(entry)
        return dict(entry)

    def note(self, seq: int, **fields) -> dict:
        """Add fields to the kept result ``seq`` (a record that could not be written); returns it."""
        for entry in self._entries:
            if entry["seq"] == seq:
                entry.update(fields)
                return dict(entry)
        raise KeyError(seq)

    @property
    def keep(self) -> int:
        return self._entries.maxlen

    @property
    def last(self) -> dict | None:
        return dict(self._entries[-1]) if self._entries else None

    def message(self) -> dict:
        message = {"t": "missions", "missions_seq": self.seq, "results": [dict(e) for e in self._entries]}
        if self.seq > len(self._entries):
            message["older"] = self.seq - len(self._entries)
        return message
