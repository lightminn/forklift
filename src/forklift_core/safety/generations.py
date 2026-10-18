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


class RequestLedger:
    """The reliable requests' ledger of a long-lived session (persistent console plan D7, stage 4c).

    Every connection gets a token that is never issued again; its requests carry that token and a seq that only
    grows. A request is admitted before it runs (``admit``): its stored result when it has one, ``stale`` when the
    token is not this connection's (another client, an older connection, never issued) or its seq is at or below the
    watermark -- its result was purged, so it can be neither answered nor run again -- ``busy`` when a bound is
    reached (not stored), else run. Results are purged ``window_s`` after they are final, in seq order, raising the
    watermark; a PENDING result stops the purge and is confirmed later under its own key. A reconnect retires the
    client's older tokens (no new requests on them); the live tokens are bounded, retired ones without a pending
    result going first. ``issue`` may be called from another thread (the server's); the rest from the control
    thread.
    """

    def __init__(self, *, window_s: float = 60.0, per_token: int = 256, total: int = 4096, max_tokens: int = 64):
        if min(per_token, total, max_tokens) <= 0 or window_s < 0:
            raise ValueError("bounds must be positive")
        import threading

        self.window_s, self.per_token, self.total, self.max_tokens = window_s, per_token, total, max_tokens
        self._issue_lock = threading.Lock()
        self._next = 0
        self._tokens: dict = {}
        self._count = 0

    def issue(self) -> int:
        with self._issue_lock:
            self._next += 1
            return self._next

    def register(self, token: int, client_id: str, conn_id: str, now: float) -> bool:
        """The connection's token, its client's older tokens retired; False when no room is left (refuse it)."""
        for entry in self._tokens.values():
            if entry["client"] == client_id:
                entry["retired"] = True
        if len(self._tokens) >= self.max_tokens:
            spare = [(e["since"], t) for t, e in self._tokens.items()
                     if e["retired"] and all(r[1] != PENDING for r in e["results"].values())]
            if not spare:
                return False
            self._drop(min(spare)[1])
        self._tokens[token] = {"client": client_id, "conn": conn_id, "results": {}, "watermark": 0,
                               "retired": False, "since": float(now)}
        return True

    def retire(self, token: int) -> None:
        """The token's connection closed: no new requests on it; it goes once its results are purged."""
        entry = self._tokens.get(token)
        if entry is not None:
            entry["retired"] = True

    def token_of(self, client_id: str, conn_id: str):
        """The current (not retired) token of that connection, or None."""
        for token, entry in self._tokens.items():
            if entry["client"] == client_id and entry["conn"] == conn_id and not entry["retired"]:
                return token
        return None

    def _drop(self, token: int) -> None:
        self._count -= len(self._tokens.pop(token)["results"])

    def admit(self, token, client_id: str, conn_id: str, seq: int) -> tuple:
        entry = self._tokens.get(token)
        if entry is None or entry["client"] != client_id or entry["conn"] != conn_id:
            return ("stale", None)
        if seq in entry["results"]:
            return ("stored", entry["results"][seq][1])
        if seq <= entry["watermark"] or entry["retired"]:
            return ("stale", None)
        if len(entry["results"]) >= self.per_token or self._count >= self.total:
            return ("busy", None)
        return ("run", None)

    def put(self, token: int, seq: int, result, now: float) -> None:
        """A result (PENDING: final later, under this same key -- even when the bounds are reached)."""
        entry = self._tokens.get(token)
        if entry is None:
            return  # the token went (evicted while pending is not possible; a stale key writes nothing)
        if seq not in entry["results"]:
            self._count += 1
        entry["results"][seq] = (None if result == PENDING else float(now), result)

    def purge(self, now: float) -> None:
        for token in list(self._tokens):
            entry = self._tokens[token]
            for seq in sorted(entry["results"]):
                stamp, _ = entry["results"][seq]
                if stamp is None or now - stamp < self.window_s:
                    break
                del entry["results"][seq]
                self._count -= 1
                entry["watermark"] = seq
            if entry["retired"] and not entry["results"]:
                del self._tokens[token]

    def watermark(self, token: int) -> int:
        return self._tokens[token]["watermark"]

    def __len__(self) -> int:
        return len(self._tokens)
