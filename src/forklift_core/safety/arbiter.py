"""The arbiter (M-b, plan docs/plans/2026-10-16-m-b-arbiter.md; design docs/design/2026-10-13-operator-console.md
sections 4-5; contract docs/interfaces/2026-10-13-operator-console-contract-draft.md sections 1-6).

Pure: every entry point takes the vehicle's monotonic time ``now_mono`` and the tokens and lease ids come from an
injected ``token_source`` -- the module never reads a clock or a random source itself (contract section 9, 1).

What it owns: the control lease, the four generations, the order a reliable request is handled in, the mode and
the stop kind, arming and the dead-man switch, which source drives (mission or operator) and the source-independent
movement ban. What it does not: the mission's progress (``forklift_core.mission``) and the actuator's own expiry.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .generations import PENDING, RequestBook
from .ledger import FrameLedger, TickLedger

MODES = ("PAUSED", "AUTO", "TELEOP", "STOPPED")
LEASED = ("rearm", "mode_request", "arm", "disarm", "mission_submit", "mission_pause", "mission_resume",
          "mission_cancel", "recovery")
RELIABLE = LEASED + ("stop", "lease_request", "lease_release", "targets_request")


@dataclass(frozen=True)
class ArbiterConfig:
    """Every number is provisional until the H3 bench measurements (design 5.4); none is a constant here."""

    command_max_age_s: float = 0.30
    lease_timeout_s: float = 1.0
    grace_s: float = 10.0
    video_max_age_s: float = 0.5
    camera_latency_bound_s: float = 0.15
    request_capacity: int = 4096
    frame_window_s: float = 5.0

    def __post_init__(self):
        if self.lease_timeout_s < self.command_max_age_s:
            raise ValueError("lease_timeout_s must be at least command_max_age_s (design 5.4)")


@dataclass
class Lease:
    client_id: str
    lease_id: str
    renewed_mono: float  # the send time of the tick its last valid reply named (the grant time before any)


class Arbiter:
    def __init__(self, session_epoch: int, config: ArbiterConfig, token_source: Callable[[], str]) -> None:
        self.session_epoch = int(session_epoch)
        self.config = config
        self._token = token_source
        self.ticks = TickLedger(max(config.command_max_age_s, config.lease_timeout_s), token_source)
        self.frames = FrameLedger(config.frame_window_s)
        self.requests = RequestBook(config.request_capacity)
        self.clients: dict[str, bool] = {}  # connected client -> its connection may control (contract 10)
        self.lease: Lease | None = None
        self.reserved: tuple[str, float] | None = None  # (client_id, until) during the reconnection grace
        self.mode = "PAUSED"
        self.control_gen = 0
        self.arm_gen = 0
        self.armed = False
        self.preempt_latch: str | None = None  # why a safety preempt holds until the truck is seen standing
        self.events: list[dict] = []  # every boundary with its time and reason, for the record

    # --- connections and the lease ------------------------------------------------------------------------------
    def connect(self, client_id: str, now_mono: float, *, control: bool = True) -> dict:
        self._expire(now_mono)
        self.clients[client_id] = bool(control)
        return {"t": "welcome", "api": "console/1", "session_epoch": self.session_epoch}

    def disconnect(self, client_id: str, now_mono: float) -> None:
        self._expire(now_mono)
        self.clients.pop(client_id, None)
        if self.lease is not None and self.lease.client_id == client_id:
            self._lose_lease(now_mono, "disconnected", reserve=True)

    def _boundary(self, now_mono: float, reason: str) -> None:
        self.control_gen += 1
        self.events.append({"t": now_mono, "event": "control_gen", "control_gen": self.control_gen,
                            "reason": reason})

    def _disarm(self, now_mono: float, reason: str) -> None:
        if self.armed:
            self.armed = False
            self.events.append({"t": now_mono, "event": "disarm", "reason": reason})

    def _lose_lease(self, now_mono: float, reason: str, *, reserve: bool) -> None:
        """Expiry, a lost connection or a release: the lease goes and the control generation moves. The mode
        decides the rest -- AUTO keeps working (user decision, design 2), TELEOP stops on a safety preempt."""
        holder = self.lease.client_id
        self.lease = None
        self.reserved = (holder, now_mono + self.config.grace_s) if reserve else None
        self._boundary(now_mono, reason)
        if self.mode == "TELEOP":
            self._disarm(now_mono, reason)
            self.preempt_latch = reason
            self._set_mode("PAUSED", now_mono, reason)

    def _expire(self, now_mono: float) -> None:
        if self.lease is not None and now_mono > self.lease.renewed_mono + self.config.lease_timeout_s:
            self._lose_lease(now_mono, "lease_expired", reserve=True)
        if self.reserved is not None and now_mono > self.reserved[1]:
            self.reserved = None
        self.ticks.prune(now_mono)

    def _set_mode(self, mode: str, now_mono: float, reason: str) -> None:
        if mode not in MODES:
            raise ValueError(mode)
        if mode == self.mode:
            return
        self.events.append({"t": now_mono, "event": "mode", "from": self.mode, "to": mode, "reason": reason})
        self.mode = mode

    # --- reliable requests (contract section 1, the handling order) ---------------------------------------------
    def handle(self, client_id: str, msg: dict, now_mono: float) -> dict | None:
        """One message from a client. Reliable requests are answered (ack or reject); latest-value messages
        (teleop, echo, input_state) are not. A mission request may be answered later (``resolve``)."""
        self._expire(now_mono)
        kind = msg.get("t")
        if client_id not in self.clients:
            return None  # not connected: nothing it says counts
        if kind not in RELIABLE:
            return self._latest_value(client_id, kind, msg, now_mono)
        request_id = msg.get("request_id")
        if not isinstance(request_id, str) or not request_id:
            return self._reply(kind, request_id, "invalid")
        if kind == "stop":
            # 0. STOP before the store and every generation check, every time (never stored).
            if not self.clients[client_id]:
                return self._reply(kind, request_id, "unsupported")
            self.stop(now_mono, f"stop:{msg.get('source', 'console')}")
            return self._reply(kind, request_id)
        # 1. Handled before: its original answer, without running it again or checking generations.
        stored = self.requests.get(client_id, request_id)
        if stored is not None:
            return None if stored == PENDING else stored
        # 2. New: refused busy (not stored) when the store is full, else checked, run and stored.
        if self.requests.full():
            return self._reply(kind, request_id, "busy")
        result = self._run(client_id, kind, msg, now_mono)
        self.requests.put(client_id, request_id, result if result is not None else PENDING)
        return result

    def _reply(self, kind: str, request_id, reason: str | None = None, **fields) -> dict:
        reply = {"t": "ack" if reason is None else "reject", "of": kind, "request_id": request_id}
        if reason is not None:
            reply["reason"] = reason
        reply.update(fields)
        return reply

    def _generation_problem(self, client_id: str, msg: dict, *, lease: bool) -> str | None:
        if msg.get("session_epoch") != self.session_epoch:
            return "stale_epoch"
        if not lease:
            return None
        if self.lease is None or self.lease.client_id != client_id:
            return "no_lease"
        if msg.get("lease_id") != self.lease.lease_id:
            return "stale_lease"
        if msg.get("control_gen") != self.control_gen:
            return "stale_control"
        return None

    def _run(self, client_id: str, kind: str, msg: dict, now_mono: float) -> dict | None:
        request_id = msg["request_id"]
        if not self.clients[client_id]:
            return self._reply(kind, request_id, "unsupported")  # an observe-only connection (contract 10)
        problem = self._generation_problem(client_id, msg, lease=kind in LEASED)
        if problem is not None:
            return self._reply(kind, request_id, problem)
        handler = getattr(self, f"_on_{kind}")
        return handler(client_id, request_id, msg, now_mono)

    def _on_lease_request(self, client_id, request_id, msg, now_mono):
        if self.lease is not None:
            if self.lease.client_id == client_id:
                return self._reply("lease_request", request_id, "invalid")  # it holds one already
            return self._reply("lease_request", request_id, "no_lease")
        if self.reserved is not None and self.reserved[0] != client_id:
            return self._reply("lease_request", request_id, "lease_reserved")
        self.reserved = None
        self.lease = Lease(client_id, self._token(), now_mono)
        self.events.append({"t": now_mono, "event": "lease", "client_id": client_id})
        return self._reply("lease_request", request_id, lease_id=self.lease.lease_id,
                           lease_timeout_s=self.config.lease_timeout_s, control_gen=self.control_gen)

    def _on_lease_release(self, client_id, request_id, msg, now_mono):
        if self.lease is None or self.lease.client_id != client_id or msg.get("lease_id") != self.lease.lease_id:
            return self._reply("lease_release", request_id, "no_lease")
        self._lose_lease(now_mono, "lease_released", reserve=False)
        return self._reply("lease_release", request_id)

    def _on_targets_request(self, client_id, request_id, msg, now_mono):
        return self._reply("targets_request", request_id)  # the runner sends the list itself

    def _on_recovery(self, client_id, request_id, msg, now_mono):
        return self._reply("recovery", request_id, "unsupported")  # plan section 1: not this slice

    # --- stops ---------------------------------------------------------------------------------------------------
    def stop(self, now_mono: float, reason: str) -> None:
        """A safety preempt stop from any mode: STOPPED until a rearm, disarmed, a control boundary. A repeated
        STOP while stopped is a boundary too -- a rearm made after the first STOP must not run after the second
        (Codex M-b plan review P1)."""
        self._disarm(now_mono, reason)
        self._set_mode("STOPPED", now_mono, reason)
        self._boundary(now_mono, reason)

    # --- latest-value messages -------------------------------------------------------------------------------------
    def _latest_value(self, client_id: str, kind: str, msg: dict, now_mono: float) -> None:
        if kind == "echo" and self.clients.get(client_id):
            self._renew(client_id, msg)
        return None

    def _renew(self, client_id: str, msg: dict) -> bool:
        """A valid reply from the lease holder renews the lease from the time its tick was sent."""
        lease = self.lease
        if lease is None or lease.client_id != client_id or msg.get("session_epoch") != self.session_epoch:
            return False
        if msg.get("lease_id") != lease.lease_id:
            return False
        echo = msg.get("echo") or {}
        sent = self.ticks.sent_mono(echo.get("state_seq"), echo.get("token"))
        if sent is None:
            return False
        lease.renewed_mono = max(lease.renewed_mono, sent)
        return True

    def tick(self, now_mono: float) -> dict:
        """A ``tick`` message: a fresh token remembered as sent now (contract section 4)."""
        self._expire(now_mono)
        state_seq, token = self.ticks.issue(now_mono)
        return {"t": "tick", "session_epoch": self.session_epoch, "state_seq": state_seq, "token": token}
