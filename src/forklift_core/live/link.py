"""The arbiter's link to a live run (M-c plan C2): the channels between the console server's thread and the runner's
control thread, and the cycle the runner calls. Isaac-free and ROS-free.

Channels (one lock): a STOP channel that never drops (one entry per connection and request id), an ordered
channel for connections, disconnections and reliable requests (bounded: a full channel answers ``busy`` at once,
not stored -- the request was not taken), and latest-value slots for ``teleop``, ``echo`` and ``input_state`` (one
per connection and kind, contract section 1). The arbiter only ever runs on the control thread.
"""

from __future__ import annotations

import threading
from collections import OrderedDict, deque
from typing import Callable

LATEST = ("teleop", "echo", "input_state")


class Channels:
    def __init__(self, capacity: int = 256, notify: Callable[[], None] | None = None) -> None:
        self.capacity = capacity
        self._lock = threading.Lock()
        self._stops: OrderedDict = OrderedDict()
        self._conns: deque = deque()  # connections and disconnections: never refused, taken before the STOPs
        self._ordered: deque = deque()
        self._latest: dict = {}
        self._out: deque = deque()
        self.notify = notify  # called (any thread) when something is put out; the server wakes its loop

    # --- the server thread ------------------------------------------------------------------------------------------
    def connect(self, conn_id: str, client_id: str, control: bool = True) -> None:
        with self._lock:
            self._conns.append(("connect", conn_id, client_id, control))

    def disconnect(self, conn_id: str, client_id: str) -> None:
        with self._lock:
            self._conns.append(("disconnect", conn_id, client_id, None))
            for key in [k for k in self._latest if k[0] == conn_id]:
                del self._latest[key]

    def message(self, conn_id: str, client_id: str, msg: dict) -> dict | None:
        """Put one message in; returns an immediate reply only when the ordered channel is full (``busy``)."""
        kind = msg.get("t")
        with self._lock:
            if kind == "stop":
                self._stops[(conn_id, msg.get("request_id"))] = (client_id, msg)
                return None
            if kind in LATEST:
                self._latest[(conn_id, kind)] = (client_id, msg)
                return None
            if len(self._ordered) >= self.capacity:
                return {"t": "reject", "of": kind, "request_id": msg.get("request_id"), "reason": "busy"}
            self._ordered.append(("request", conn_id, client_id, msg))
            return None

    def take_out(self) -> list:
        with self._lock:
            out, self._out = list(self._out), deque()
        return out

    # --- the control thread ---------------------------------------------------------------------------------------
    def take_stops(self) -> tuple[list, list]:
        """The connection events, then the STOPs -- a STOP sent right after the welcome finds its client known
        (Codex M-c round 2 P1)."""
        with self._lock:
            conns, self._conns = list(self._conns), deque()
            stops, self._stops = list(self._stops.items()), OrderedDict()
        return conns, [(conn_id, client_id, msg) for (conn_id, _), (client_id, msg) in stops]

    def take_cycle(self) -> tuple[list, list, list, list]:
        """Everything for a cycle's start: connection events, STOPs, ordered requests, latest values."""
        with self._lock:
            conns, self._conns = list(self._conns), deque()
            stops, self._stops = list(self._stops.items()), OrderedDict()
            ordered, self._ordered = list(self._ordered), deque()
            latest, self._latest = list(self._latest.items()), {}
        return (conns, [(c, cl, m) for (c, _), (cl, m) in stops], ordered,
                [(conn_id, client_id, msg) for (conn_id, _), (client_id, msg) in latest])

    def put(self, conn_id: str | None, msg: dict) -> None:
        """A message to one connection (None: every control connection)."""
        with self._lock:
            self._out.append((conn_id, msg))
        if self.notify is not None:
            self.notify()


class Link:
    """The cycle the live runner calls (M-b B3 with the physics-step gate of M-c C2)."""

    def __init__(self, arbiter, channels: Channels, *, tick_period_s: float = 0.05, state_period_s: float = 0.1):
        self.a, self.ch = arbiter, channels
        self.tick_period_s, self.state_period_s = tick_period_s, state_period_s
        self._next_tick = self._next_state = None
        self.log: list[dict] = []  # every message handled with when (mono), for live_log.jsonl
        self.conn_of: dict[str, str] = {}  # client_id -> current conn_id, for the mission replies

    def _handle(self, conn_id, client_id, msg, now, where):
        try:
            reply = self.a.handle(client_id, msg, now, conn_id=conn_id)
        except Exception as exc:  # noqa: BLE001 -- one bad message never ends the run; it is logged and refused
            self.log.append({"mono": now, "where": where, "conn": conn_id, "error": repr(exc)})
            reply = None
            if isinstance(msg, dict) and isinstance(msg.get("request_id"), str):
                reply = {"t": "reject", "of": msg.get("t"), "request_id": msg["request_id"], "reason": "invalid"}
        self.log.append({"mono": now, "where": where, "conn": conn_id, "client": client_id, "t": msg.get("t"),
                         "request_id": msg.get("request_id"),
                         "reply": None if reply is None else (reply.get("t"), reply.get("reason"))})
        if reply is not None:
            self.ch.put(conn_id, reply)

    def _conn_event(self, kind, conn_id, client_id, extra, now, where):
        if kind == "connect":
            welcome = self.a.connect(client_id, now, control=bool(extra), conn_id=conn_id)
            self.conn_of[client_id] = conn_id
            self.log.append({"mono": now, "where": where, "conn": conn_id, "t": "connect",
                             "epoch": welcome["session_epoch"]})
        else:
            self.a.disconnect(client_id, now, conn_id=conn_id)
            if self.conn_of.get(client_id) == conn_id:
                del self.conn_of[client_id]
            self.log.append({"mono": now, "where": where, "conn": conn_id, "t": "disconnect"})

    def stops(self, now: float, where: str) -> None:
        conns, stops = self.ch.take_stops()
        for kind, conn_id, client_id, extra in conns:
            self._conn_event(kind, conn_id, client_id, extra, now, where)
        for conn_id, client_id, msg in stops:
            self._handle(conn_id, client_id, msg, now, where)

    def gate(self, now: float) -> bool:
        """Before a physics step: STOPs now, then the deadlines. True: zero the wheels this step."""
        self.stops(now, "gate")
        return self.a.gate(now)

    def begin(self, now: float, feedback) -> str:
        """The cycle's start; returns the stop kind to give the mission."""
        self.a.observe(now, feedback)
        conns, stops, ordered, latest = self.ch.take_cycle()
        for kind, conn_id, client_id, extra in conns:
            self._conn_event(kind, conn_id, client_id, extra, now, "cycle")
        for conn_id, client_id, msg in stops:
            self._handle(conn_id, client_id, msg, now, "cycle")
        for _, conn_id, client_id, msg in ordered:
            self._handle(conn_id, client_id, msg, now, "cycle")
        for conn_id, client_id, msg in latest:
            self._handle(conn_id, client_id, msg, now, "cycle")
        return self.a.mission_context(now)

    def end(self, now: float, records: list, safety, candidate, drive=None):
        """After the mission's tick: the mission replies, the output, and the tick/state due."""
        for reply in self.a.resolve(now, records):
            client_id, conn_id = reply.pop("client_id", None), reply.pop("conn_id", None)
            if conn_id is not None and self.conn_of.get(client_id) == conn_id:
                self.ch.put(conn_id, reply)  # else the asking connection is gone: a resend gets the stored result
        out = self.a.step(now, safety, candidate, drive)
        if self._next_tick is None or now >= self._next_tick:
            self.ch.put(None, self.a.tick(now))
            self._next_tick = now + self.tick_period_s
        return out

    def state_due(self, now: float) -> bool:
        if self._next_state is None or now >= self._next_state:
            self._next_state = now + self.state_period_s
            return True
        return False
