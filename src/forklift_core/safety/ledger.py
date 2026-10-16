"""The reply tokens the vehicle sent and the camera frames it took (M-b, plan docs/plans/2026-10-16-m-b-arbiter.md
B10; contract docs/interfaces/2026-10-13-operator-console-contract-draft.md section 4).

Pure: the time is always an argument and the token comes from an injected source, so a test fixes both. Records are
kept for a time, not a count -- a valid reply must never fall out of the window because many ticks went out.
"""

from __future__ import annotations

from collections import OrderedDict
from typing import Callable


class TickLedger:
    """``tick`` messages: a state sequence number and an unpredictable token, with the vehicle time it was made."""

    def __init__(self, window_s: float, token_source: Callable[[], str]) -> None:
        if window_s <= 0:
            raise ValueError("window_s must be positive")
        self.window_s = float(window_s)
        self._token_source = token_source
        self.state_seq = 0
        self._sent: OrderedDict = OrderedDict()  # state_seq -> (token, sent_mono)

    def issue(self, now_mono: float) -> tuple[int, str]:
        """A new tick: the next state sequence number and a fresh token, remembered as sent at ``now_mono``."""
        self.prune(now_mono)
        self.state_seq += 1
        token = self._token_source()
        self._sent[self.state_seq] = (token, float(now_mono))
        return self.state_seq, token

    def next_seq(self) -> int:
        """A state sequence number for a ``state`` message (they share the numbering, contract section 6)."""
        self.state_seq += 1
        return self.state_seq

    def sent_mono(self, state_seq: int, token: str) -> float | None:
        """When that tick went out; None when it is unknown, forgotten or the token does not match its number."""
        entry = self._sent.get(state_seq)
        if entry is None or entry[0] != token:
            return None
        return entry[1]

    def prune(self, now_mono: float) -> None:
        """Forget only ticks older than the window."""
        while self._sent:
            seq, (_, sent) = next(iter(self._sent.items()))
            if now_mono - sent <= self.window_s:
                break
            del self._sent[seq]


class FrameLedger:
    """Camera frames by shot: the vehicle time a frame was taken from the driver or renderer. A frame sent again
    keeps its first time, so an old picture cannot be made young by resending it."""

    def __init__(self, window_s: float) -> None:
        if window_s <= 0:
            raise ValueError("window_s must be positive")
        self.window_s = float(window_s)
        self._acquired: OrderedDict = OrderedDict()
        self._max_id: int | None = None

    def register(self, frame_id: int, acquired_mono: float) -> None:
        """Frame ids only grow within a session (contract section 1): an id at or below the largest seen that is
        not held any more was dropped from the window and is not taken again -- re-registering it would make an
        old picture young (Codex M-b plan review P1)."""
        if frame_id in self._acquired or (self._max_id is not None and frame_id <= self._max_id):
            return
        self._max_id = frame_id
        self._acquired[frame_id] = float(acquired_mono)
        while self._acquired:
            fid, t = next(iter(self._acquired.items()))
            if acquired_mono - t <= self.window_s:
                break
            del self._acquired[fid]

    def acquired_mono(self, frame_id: int) -> float | None:
        return self._acquired.get(frame_id)
