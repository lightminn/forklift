"""The real truck's console channels and send boundary (docs/design/2026-11-19-real-loop.md 2.5).

``RealChannels`` counts every emergency event -- a STOP, a DISARM request, a connection added or replaced (the same
lease holder reconnecting takes the lease back), a disconnection -- under the channels' own lock
(``emergency_serial``), and hands out the emergency batch with the serial it was taken at. ``RealLink`` processes
that batch (the DISARM through the arbiter's checks) and returns the serial. ``send_boundary`` tries one first
non-blocking write under the same short lock only when no emergency came in since the batch: a STOP accepted after
the gate and before the first byte is never overtaken by that byte.
"""

from __future__ import annotations

import threading
from collections import deque

from forklift_core.live.link import Channels, Link
from forklift_core.safety.validate import problem


class RealChannels(Channels):
    def __init__(self, capacity: int = 256, notify=None) -> None:
        super().__init__(capacity, notify)
        # re-entrant: the send boundary's check runs under it and may read the channels (Codex 4c impl round 5 P1-2)
        self._lock = threading.RLock()
        self.emergency_serial = 0
        self._disarms: deque = deque()

    @property
    def lock(self):
        return self._lock

    def serial(self) -> int:
        with self._lock:
            return self.emergency_serial

    def connect(self, conn_id: str, client_id: str, control: bool = True, token: int | None = None) -> None:
        with self._lock:
            self._conns.append(("connect", conn_id, client_id, {"control": control, "token": token}))
            self.emergency_serial += 1

    def disconnect(self, conn_id: str, client_id: str) -> None:
        with self._lock:
            self._conns.append(("disconnect", conn_id, client_id, None))
            for key in [k for k in self._latest if k[0] == conn_id]:
                del self._latest[key]
            self.emergency_serial += 1

    def message(self, conn_id: str, client_id: str, msg: dict) -> dict | None:
        if problem(msg) is None and msg.get("t") in ("stop", "disarm"):
            with self._lock:
                if msg["t"] == "stop":
                    self._stops.setdefault((conn_id, msg.get("request_id")), (client_id, msg))
                else:
                    self._disarms.append((conn_id, client_id, msg))
                self.emergency_serial += 1
            return None
        return super().message(conn_id, client_id, msg)

    def take_emergency(self) -> tuple[list, list, list, int]:
        """The connection events, the STOPs and the DISARMs, with the serial they were taken at."""
        with self._lock:
            conns, self._conns = list(self._conns), deque()
            stops, self._stops = list(self._stops.items()), type(self._stops)()
            disarms, self._disarms = list(self._disarms), deque()
            serial = self.emergency_serial
        return (conns, [(c, cl, m) for (c, _), (cl, m) in stops], disarms, serial)


class RealLink(Link):
    def emergency(self, now: float) -> int:
        """The emergency batch processed (connections, STOPs, then DISARMs through the arbiter's own checks);
        returns the serial it was taken at -- the send boundary's reference."""
        conns, stops, disarms, serial = self.ch.take_emergency()
        for kind, conn_id, client_id, extra in conns:
            self._conn_event(kind, conn_id, client_id, extra, now, "emergency")
        for conn_id, client_id, msg in stops + disarms:
            self._handle(conn_id, client_id, msg, now, "emergency")
        return serial

    def begin(self, now: float, feedback) -> str:
        self.emergency(now)  # nothing of the batch waits behind the ordered requests
        return super().begin(now, feedback)


def send_boundary(channels: RealChannels, serial: int, write, check=None) -> bool:
    """One first write under the channels' lock, only when no emergency event came in since ``serial`` and ``check()``
    (read under the same lock, just before the write) passes; False: not tried (judge again before the next
    attempt)."""
    with channels.lock:
        if channels.emergency_serial != serial:
            return False
        if check is not None and not check():
            return False
        write()
        return True


__all__ = ["RealChannels", "RealLink", "send_boundary"]
