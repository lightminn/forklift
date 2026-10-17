"""Stored results of reliable requests (M-b, plan docs/plans/2026-10-16-m-b-arbiter.md B3; contract section 1).

A request already handled gets its original answer back and is not run again; a new one is refused ``busy`` when
the store is full, and that refusal is not stored -- dropping old results would make a resend of a forgotten
request indistinguishable from a new one. STOP never comes here (it runs every time).
"""

from __future__ import annotations

PENDING = "pending"  # a mission request waiting for the lifecycle's verdict at the next control tick


class RequestBook:
    def __init__(self, capacity: int = 4096) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self.capacity = int(capacity)
        self._results: dict = {}

    def get(self, client_id: str, request_id: str):
        return self._results.get((client_id, request_id))

    def full(self) -> bool:
        return len(self._results) >= self.capacity

    def put(self, client_id: str, request_id: str, result) -> None:
        key = (client_id, request_id)
        if key not in self._results and self.full():
            raise OverflowError("request store is full")
        self._results[key] = result

    def __len__(self) -> int:
        return len(self._results)
