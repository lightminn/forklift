"""P0-7: the host side of mcu-link/1, without I/O (docs/plans/2026-11-11-sim-to-real-bringup.md 2.2, P0-7).

``McuLinkHost`` turns received bytes into link state and events and keeps a frame outbox that an I/O
adapter (``tools/real_mcu_link.py``) drains one whole frame at a time. Its rules:

- **Operator authorization.** Wheels move only after an operator ``request_arm``: ARM -> an ARMED STATE
  of the same boot with a new ``arm_gen`` -> one zero DRIVE (wheels 0, steering at its measured angle,
  lift held) -> a STATE of that generation with ``ZERO_SEEN``. A stale link, a disconnect, a new boot,
  an unexpected ``arm_gen``, the MCU leaving ARMED or a DISARM request revokes it; a fresh STATE never
  restores it (Codex P0-7 P1: an MCU still ARMED after a gap or a host restart must not resume).
- **No automatic ARM or CLEAR_FAULTS.** Nothing here reacts to an event by sending either.
- **Deadlines are never extended.** A DRIVE carries the decision's absolute expiry: its validity is
  ``min(cap, floor(expiry - receipt of the echoed STATE))`` (contract section 2), so the MCU's deadline
  is never later than the decision's. A queued DRIVE past its expiry is dropped, not written.
- **Freshness on the host clock.** A STATE is fresh for ``state_max_age_s`` after its receipt; a STATE
  of the same boot whose MCU time does not advance, or with boot_id 0, is ignored. ``tick`` must be
  called every control cycle so silence is noticed without a STATE arriving.

Times are host ``time.monotonic_ns()``; MCU time is only echoed and ordered, never mixed with it.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

from forklift_core.mcu_link.messages import (
    Arm,
    ClearFaults,
    Disarm,
    Drive,
    LinkDecoder,
    McuState,
    Mode,
    RejectReason,
    StateFlag,
    encode,
    valid_after_echo_ms,
)
from forklift_core.mcu_link.build_policy import Client, drive_refusal
from forklift_core.plant.real_vehicle import RearDrive


@dataclass(frozen=True)
class HostConfig:
    state_max_age_s: float  # a STATE older than this (host receipt) is stale
    cap_ms: int  # upper bound on a DRIVE's validity after its echo
    arm_timeout_s: float  # an operator arm that has not reached ZERO_SEEN by then fails
    zero_valid_ms: int  # validity of the zero DRIVE that follows ARM
    outbox_max: int  # frames waiting to be written

    def __post_init__(self) -> None:
        if not (self.state_max_age_s > 0 and self.arm_timeout_s > 0):
            raise ValueError("ages must be positive")
        for name in ("cap_ms", "zero_valid_ms", "outbox_max"):
            if not isinstance(getattr(self, name), int) or getattr(self, name) < 1:
                raise ValueError(f"{name} must be a positive integer")
        if self.cap_ms > 0xFFFF or self.zero_valid_ms > 0xFFFF:
            raise ValueError("validity must fit the u16 field")


@dataclass(frozen=True)
class Blocked:
    reason: str


@dataclass(frozen=True)
class HostEvent:
    kind: str
    detail: dict = field(default_factory=dict)


@dataclass(frozen=True)
class LinkHealth:
    fresh: bool
    state_age_s: float | None
    boot_id: int | None
    mode: Mode | None
    mcu_time_us: int | None
    fault_flags: int
    reject_reason: RejectReason | None
    authorized: bool
    ready: bool
    rx_errors: int | None
    decode_errors: int


@dataclass(frozen=True)
class Outgoing:
    kind: str  # "ARM", "DISARM", "CLEAR_FAULTS", "DRIVE"
    seq: int
    data: bytes
    expires_ns: int | None  # DRIVE only: dropped, not written, once this has passed
    generation: int  # authorization generation it was made under (DRIVE dropped when it changes)
    nonzero: bool = False  # DRIVE with a non-zero wheel target


_NONE, _ARM_PENDING, _ZERO_PENDING, _READY = "none", "arm_pending", "zero_pending", "ready"


class McuLinkHost:
    def __init__(self, cfg: HostConfig, *, log: Callable[..., None] | None = None, client: Client = Client.REAL,
                 allow_locked: bool = False) -> None:
        """``client``: who drives through this host (mcu-link/2 build bits, contract 3.3): the real executor (the
        default), the HIL executor, or a bring-up tool. ``allow_locked``: a bring-up tool's run on a model or HIL
        board whose outputs are locked."""
        self.cfg = cfg
        self.client, self.allow_locked = Client(client), bool(allow_locked)
        self._log = log or (lambda *a: None)
        self._decoder = LinkDecoder()
        self._state: McuState | None = None
        self._state_rx_ns: int | None = None
        self._stale_reported = False
        self._seq = 0
        self._outbox: deque[Outgoing] = deque()
        self._auth = _NONE
        self._auth_gen = 0  # bumps whenever authorization starts or ends
        self._arm: dict | None = None  # boot, prev_gen, gen, deadline_ns, reject_at_request
        self._inflight: Outgoing | None = None  # handed to the adapter, not yet written / unsent / abandoned
        self._written: deque = deque(maxlen=256)  # every DRIVE written: seq, nonzero, expires_ns, written_ns

    # ---------------------------------------------------------------- state
    @property
    def authorized(self) -> bool:
        return self._auth in (_ZERO_PENDING, _READY)

    @property
    def ready(self) -> bool:
        return self._auth == _READY

    @property
    def auth_epoch(self) -> int:
        """Bumps whenever operator authorization starts or ends (4a' design D4): a command stamped with an
        older epoch was made under another authorization."""
        return self._auth_gen

    @property
    def state_rx_ns(self) -> int | None:
        return self._state_rx_ns

    def drive_traffic(self) -> dict:
        """What a release check needs: the non-zero DRIVEs queued or being written (their original expiries) and
        every DRIVE written (seq, non-zero, original expiry, write time), oldest first."""
        pending = [o.expires_ns for o in self._outbox if o.kind == "DRIVE" and o.nonzero]
        if self._inflight is not None and self._inflight.nonzero:
            pending.append(self._inflight.expires_ns)
        return {"pending_nonzero_expiries": pending, "written": [dict(w) for w in self._written]}

    def drop_drives(self, reason: str) -> None:
        """Drop the queued DRIVEs (a control generation change). A frame already being written is finished by
        the adapter and tracked when written: cutting it is not a cancel (4a' design D4)."""
        self._drop(lambda o: o.kind == "DRIVE", reason)

    def _fresh(self, now_ns: int) -> bool:
        return self._state_rx_ns is not None and now_ns - self._state_rx_ns <= self.cfg.state_max_age_s * 1e9

    def health(self, now_ns: int) -> LinkHealth:
        s = self._state
        return LinkHealth(
            fresh=self._fresh(now_ns),
            state_age_s=None if self._state_rx_ns is None else (now_ns - self._state_rx_ns) / 1e9,
            boot_id=None if s is None else s.boot_id,
            mode=None if s is None else s.mode,
            mcu_time_us=None if s is None else s.mcu_time_us,
            fault_flags=0 if s is None else int(s.fault_flags),
            reject_reason=None if s is None else s.reject_reason,
            authorized=self.authorized,
            ready=self.ready,
            rx_errors=None if s is None else s.rx_errors,
            decode_errors=self._decoder.errors,
        )

    @property
    def state(self) -> McuState | None:
        return self._state

    def _revoke(self, reason: str, events: list) -> None:
        if self._auth != _NONE:
            events.append(HostEvent("authorization_revoked" if self.authorized else "arm_failed", {"reason": reason}))
        self._auth, self._arm = _NONE, None
        self._auth_gen += 1
        self._drop(lambda o: o.kind in ("DRIVE", "ARM"), f"revoked: {reason}")

    # ---------------------------------------------------------------- receive
    def on_bytes(self, data: bytes, now_ns: int) -> list[HostEvent]:
        self._log("received", now_ns, bytes(data), None)
        before = self._decoder.errors
        events: list[HostEvent] = []
        for rec in self._decoder.feed(bytes(data), now_ns):
            if isinstance(rec.message, McuState):
                self._on_state(rec.message, now_ns, events)
        if self._decoder.errors != before:
            events.append(HostEvent("decode_errors", {"total": self._decoder.errors}))
        return events

    def build_refusal(self, s: McuState | None = None) -> str | None:
        """Why this host must not ARM or DRIVE the board reporting ``s`` (the latest STATE by default), from the
        build bits (mcu-link/2 contract 3.3; Codex PR review P2): the executors by ``build_policy.drive_refusal``,
        a bring-up tool never on a commissioning build (it takes no DRIVE) and on locked outputs only when allowed."""
        s = self._state if s is None else s
        if s is None:
            return None
        if self.client is Client.BRINGUP:
            if s.flags & StateFlag.COMMISSION_BUILD:
                return "commission_build"
            if s.flags & StateFlag.OUTPUTS_LOCKED and not self.allow_locked:
                return "outputs_locked"
            return None
        return drive_refusal(s.flags, self.client)

    def _on_state(self, s: McuState, now_ns: int, events: list) -> None:
        if s.boot_id == 0:
            events.append(HostEvent("state_invalid", {"why": "boot_id 0"}))
            return
        prev = self._state
        if prev is not None and s.boot_id == prev.boot_id and s.mcu_time_us <= prev.mcu_time_us:
            events.append(HostEvent("state_out_of_order", {"mcu_time_us": s.mcu_time_us}))
            return
        # The gap since the previous receipt is judged before this STATE refreshes the link: a STATE
        # that arrives after a stale gap must not hide it (Codex P0-7 round 2 P1).
        self._expire(now_ns, events)
        self._state, self._state_rx_ns = s, now_ns
        if prev is None:
            events.append(HostEvent("first_state", {"boot_id": s.boot_id, "mode": s.mode}))
        if self._stale_reported:
            self._stale_reported = False
            events.append(HostEvent("recovered"))
        if prev is not None and s.boot_id != prev.boot_id:
            events.append(HostEvent("boot_changed", {"from": prev.boot_id, "to": s.boot_id}))
            self._revoke("boot_changed", events)
        if prev is not None and s.mode != prev.mode:
            events.append(HostEvent("mode_changed", {"from": prev.mode, "to": s.mode}))
        if prev is not None and s.fault_flags != prev.fault_flags:
            events.append(HostEvent("faults_changed", {"from": int(prev.fault_flags), "to": int(s.fault_flags)}))
        if prev is not None and s.reject_reason != prev.reject_reason and s.reject_reason != RejectReason.NONE:
            events.append(HostEvent("reject", {"reason": s.reject_reason}))
        refused = self.build_refusal(s)
        if refused is not None and self._auth != _NONE:
            self._revoke(f"build_refused: {refused}", events)
            self._enqueue_disarm(now_ns)  # never leave a board armed that this host may not drive
            return
        self._advance_auth(s, now_ns, events)

    def _advance_auth(self, s: McuState, now_ns: int, events: list) -> None:
        a = self._arm
        if self._auth == _ARM_PENDING:
            if s.boot_id != a["boot"]:
                self._revoke("boot_changed", events)
            elif s.mode == Mode.ARMED and s.arm_gen not in (0, a["prev_gen"]):
                if s.steering_rad != s.steering_rad or abs(s.steering_rad) == float("inf"):
                    self._revoke("steering_invalid", events)
                    self._enqueue_disarm(now_ns)  # do not leave an ARMED MCU we will not drive
                    return
                a["gen"] = s.arm_gen
                zero = Drive(0.0, 0.0, float(s.steering_rad), 0.0,
                             valid_after_echo_ms(now_ns + self.cfg.zero_valid_ms * 1_000_000, now_ns),
                             True, s.arm_gen, s.boot_id, s.mcu_time_us)
                self._auth = _ZERO_PENDING
                self._enqueue("DRIVE", zero, now_ns, expires_ns=now_ns + self.cfg.zero_valid_ms * 1_000_000)
                events.append(HostEvent("zero_sent", {"arm_gen": s.arm_gen}))
            elif s.mode != Mode.ARMED and s.reject_reason not in (RejectReason.NONE, a["reject_at_request"]):
                self._revoke(f"rejected: {s.reject_reason.name.lower()}", events)
        elif self._auth in (_ZERO_PENDING, _READY):
            if s.boot_id != a["boot"]:
                self._revoke("boot_changed", events)
            elif s.arm_gen != a["gen"]:
                self._revoke("arm_gen_changed", events)
            elif s.mode != Mode.ARMED:
                self._revoke(f"mode_{s.mode.name.lower()}", events)
            elif self._auth == _ZERO_PENDING and s.flags & StateFlag.ZERO_SEEN:
                self._auth = _READY
                events.append(HostEvent("armed_ready", {"arm_gen": s.arm_gen}))

    def _expire(self, now_ns: int, events: list) -> None:
        """Staleness of the last receipt and the arm deadline, as of now_ns."""
        if self._state_rx_ns is not None and not self._fresh(now_ns) and not self._stale_reported:
            self._stale_reported = True
            events.append(HostEvent("stale", {"age_s": (now_ns - self._state_rx_ns) / 1e9}))
            self._revoke("stale", events)
        if self._auth in (_ARM_PENDING, _ZERO_PENDING) and now_ns > self._arm["deadline_ns"]:
            self._revoke("arm_timeout", events)

    def tick(self, now_ns: int) -> list[HostEvent]:
        events: list[HostEvent] = []
        self._expire(now_ns, events)
        return events

    def disconnected(self, now_ns: int) -> list[HostEvent]:
        """The adapter lost the device: forget the link, drop everything queued."""
        events: list[HostEvent] = []
        self._revoke("disconnected", events)
        self._drop(lambda o: True, "disconnected")
        self._state, self._state_rx_ns, self._stale_reported = None, None, False
        events.append(HostEvent("disconnected"))
        return events

    # ---------------------------------------------------------------- operator requests
    def request_arm(self, now_ns: int) -> Blocked | None:
        s = self._state
        if s is None or not self._fresh(now_ns):
            return Blocked("no_state")
        if s.mode == Mode.ARMED or s.mode == Mode.STOPPING:
            return Blocked("disarm_first")
        if s.mode in (Mode.FAULT, Mode.ESTOP):
            return Blocked("latched")
        if s.mode != Mode.DISARMED:
            return Blocked("not_disarmed")
        refused = self.build_refusal(s)
        if refused is not None:
            return Blocked(f"build_refused: {refused}")
        if self._auth != _NONE:
            return Blocked("arm_in_progress")
        if len(self._outbox) >= self.cfg.outbox_max:
            return Blocked("outbox_full")
        self._auth_gen += 1
        self._auth = _ARM_PENDING
        self._arm = {"boot": s.boot_id, "prev_gen": s.arm_gen, "gen": None, "reject_at_request": s.reject_reason,
                     "deadline_ns": now_ns + int(self.cfg.arm_timeout_s * 1e9)}
        self._enqueue("ARM", Arm(s.boot_id, s.mcu_time_us), now_ns)
        return None

    def request_disarm(self, now_ns: int) -> None:
        """Always accepted, no STATE needed: drops queued ARM/DRIVE/CLEAR_FAULTS, then queues one DISARM
        (an unsent DISARM already queued is enough; repeated requests do not grow the outbox)."""
        events: list = []
        self._revoke("disarm", events)
        self._drop(lambda o: o.kind != "DISARM", "disarm")
        self._enqueue_disarm(now_ns)

    def _enqueue_disarm(self, now_ns: int) -> None:
        if not any(o.kind == "DISARM" for o in self._outbox):
            self._enqueue("DISARM", Disarm(), now_ns)

    def request_clear_faults(self, now_ns: int) -> Blocked | None:
        s = self._state
        if s is None or not self._fresh(now_ns):
            return Blocked("no_state")
        if s.mode not in (Mode.FAULT, Mode.ESTOP):
            return Blocked("not_latched")
        if len(self._outbox) >= self.cfg.outbox_max:
            return Blocked("outbox_full")
        self._enqueue("CLEAR_FAULTS", ClearFaults(s.boot_id), now_ns)
        return None

    # ---------------------------------------------------------------- drive
    def drive(self, cmd: RearDrive, lift_m: float, lift_hold: bool, expires_ns: int, now_ns: int) -> Blocked | None:
        """Queue one DRIVE for a decision that expires at ``expires_ns`` (host monotonic)."""
        s = self._state
        if s is None:
            return Blocked("no_state")
        if not self._fresh(now_ns):
            return Blocked("stale")
        if self._auth == _ZERO_PENDING:
            return Blocked("zero_pending")
        if self._auth != _READY:
            return Blocked("not_authorized")
        refused = self.build_refusal(s)
        if refused is not None:
            return Blocked(f"build_refused: {refused}")
        if expires_ns <= now_ns:
            return Blocked("expired")
        try:
            valid = min(self.cfg.cap_ms, valid_after_echo_ms(expires_ns, self._state_rx_ns))
        except ValueError:
            return Blocked("expired")
        if len(self._outbox) >= self.cfg.outbox_max:
            return Blocked("outbox_full")
        msg = Drive(cmd.left_wheel_rad_s, cmd.right_wheel_rad_s, cmd.steering_rad, float(lift_m), valid,
                    bool(lift_hold), s.arm_gen, s.boot_id, s.mcu_time_us)
        self._enqueue("DRIVE", msg, now_ns, expires_ns=expires_ns,
                      nonzero=cmd.left_wheel_rad_s != 0.0 or cmd.right_wheel_rad_s != 0.0)
        return None

    # ---------------------------------------------------------------- outbox
    def _enqueue(self, kind: str, msg, now_ns: int, *, expires_ns: int | None = None, nonzero: bool = False) -> None:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        out = Outgoing(kind, seq, encode(msg, seq), expires_ns, self._auth_gen, nonzero)
        self._outbox.append(out)
        self._log("queued", now_ns, out.data, {"kind": kind, "seq": seq})

    def _drop(self, which, why: str) -> None:
        kept = deque()
        for o in self._outbox:
            if which(o):
                self._log("dropped", None, o.data, {"kind": o.kind, "seq": o.seq, "why": why})
            else:
                kept.append(o)
        self._outbox = kept

    def next_frame(self, now_ns: int) -> Outgoing | None:
        """The next whole frame to write; DRIVEs past their expiry or made under an ended authorization
        are dropped here. The adapter writes the frame completely before asking again."""
        while self._outbox:
            o = self._outbox.popleft()
            if o.kind == "DRIVE" and (o.expires_ns <= now_ns or o.generation != self._auth_gen):
                self._log("dropped", now_ns, o.data, {"kind": o.kind, "seq": o.seq, "why": "expired or stale"})
                continue
            self._inflight = o
            return o
        return None

    def _sent(self, out: Outgoing, now_ns: int) -> None:
        if self._inflight is out:
            self._inflight = None
        if out.kind == "DRIVE":
            self._written.append({"seq": out.seq, "nonzero": out.nonzero, "expires_ns": out.expires_ns,
                                  "written_ns": now_ns})

    def unsent(self, out: Outgoing, now_ns: int) -> None:
        """The adapter could not write the first byte of ``out`` (EAGAIN): it goes back to the head of
        the outbox, still subject to every drop rule (a DISARM or a revoke still cancels it)."""
        if self._inflight is out:
            self._inflight = None
        self._outbox.appendleft(out)
        self._log("unsent", now_ns, out.data, {"kind": out.kind, "seq": out.seq})

    def frame_written(self, out: Outgoing, now_ns: int) -> None:
        self._sent(out, now_ns)
        self._log("written", now_ns, out.data, {"kind": out.kind, "seq": out.seq})

    def frame_abandoned(self, out: Outgoing, written: int, now_ns: int) -> None:
        """A frame the adapter had started (``written`` bytes out) but could not finish (device lost): counted
        as possibly delivered."""
        self._sent(out, now_ns)
        self._log("abandoned", now_ns, out.data, {"kind": out.kind, "seq": out.seq, "written": written})


__all__ = ["Blocked", "HostConfig", "HostEvent", "LinkHealth", "McuLinkHost", "Outgoing"]
