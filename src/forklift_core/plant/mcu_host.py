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
- **Freshness on the host clock** (``gap_clock="host"``, every tool but the teleop bridge). A STATE is fresh for
  ``state_max_age_s`` after its receipt; a STATE of the same boot whose MCU time does not advance, or with boot_id
  0, is ignored. ``tick`` must be called every control cycle so silence is noticed without a STATE arriving.
- **Freshness by MCU-time gap** (``gap_clock="mcu"``, the teleop bridge; docs/design/2026-11-15-d5-link-gap-mcu-time.md
  3.3a). A host stall with STATEs queued is not a disconnect: the first STATE of a read is stale past
  ``host_gap_s`` (150 ms, the DRIVE cap) since the previous read; each STATE past ``state_max_age_s`` of MCU time
  (widened by the verified rate bound) since the previous one; the newest past ``state_max_age_s`` of age, receipt
  age plus its round-trip delay bound (``freshness.delay_s``). Forward authorization steps take the newest STATE of
  a read only. Without a delay bound, or before ``freshness.rate_verified()``, the host rule applies.

Times are host ``time.monotonic_ns()``; MCU time is echoed, ordered and, in the "mcu" rule, differenced within one
boot -- never subtracted from a host time.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

from forklift_core.mcu_link.messages import (
    Arm,
    BenchArm,
    BenchMode,
    BenchSet,
    BenchSource,
    BenchState,
    ClearFaults,
    Commission,
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
from forklift_core.mcu_link.build_policy import Client, commission_allowed, drive_refusal, estop_waived
from forklift_core.plant import bench_limits as bl
from forklift_core.plant.real_vehicle import RearDrive


@dataclass(frozen=True)
class HostConfig:
    state_max_age_s: float  # a STATE older than this (host receipt) is stale
    cap_ms: int  # upper bound on a DRIVE's validity after its echo
    arm_timeout_s: float  # an operator arm that has not reached ZERO_SEEN by then fails
    zero_valid_ms: int  # validity of the zero DRIVE that follows ARM
    outbox_max: int  # frames waiting to be written
    gap_clock: str = "host"  # "host": receipt gaps; "mcu": MCU-time gaps (D5, the teleop bridge)
    host_gap_max_s: float | None = None  # "mcu": a read this long after the previous one is stale (None: the cap)

    @property
    def host_gap_s(self) -> float:
        return self.cap_ms / 1000.0 if self.host_gap_max_s is None else float(self.host_gap_max_s)

    def __post_init__(self) -> None:
        if not (self.state_max_age_s > 0 and self.arm_timeout_s > 0):
            raise ValueError("ages must be positive")
        if self.gap_clock not in ("host", "mcu"):
            raise ValueError("gap_clock is 'host' or 'mcu'")
        if self.gap_clock == "mcu" and not (self.state_max_age_s <= self.host_gap_s <= self.cap_ms / 1000.0 + 1e-12):
            raise ValueError("host_gap_max_s must lie between state_max_age_s and the cap")
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
    delay_s: float | None = None  # "mcu": the newest STATE's delay bound
    gap_rule: str = "host"  # the rule in force now: "host" or "mcu"


@dataclass(frozen=True)
class Outgoing:
    kind: str  # "ARM", "DISARM", "CLEAR_FAULTS", "DRIVE", "COMMISSION", "BENCH_SET", "BENCH_ARM"
    seq: int
    data: bytes
    expires_ns: int | None  # DRIVE/COMMISSION only: dropped, not written, once this has passed
    generation: int  # authorization generation it was made under (DRIVE/COMMISSION dropped when it changes)
    nonzero: bool = False  # DRIVE with a non-zero wheel target, COMMISSION with a non-zero duty
    echo: tuple | None = None  # ARM and the arming zero: the (boot id, MCU time) of the STATE they echo (6a design 2)


_MOTION = ("DRIVE", "COMMISSION")  # the commands that move an axis: one outbox rule for both


_NONE, _ARM_PENDING, _ZERO_PENDING, _READY = "none", "arm_pending", "zero_pending", "ready"


class McuLinkHost:
    def __init__(self, cfg: HostConfig, *, log: Callable[..., None] | None = None, client: Client = Client.REAL,
                 allow_locked: bool = False, commission: bool = False,
                 on_state: Callable[[McuState, int], None] | None = None, initial_seq: int = 0,
                 freshness=None) -> None:
        """``client``: who drives through this host (mcu-link/2 build bits, contract 3.3): the real executor (the
        default), the HIL executor, or a bring-up tool. ``allow_locked``: a bring-up tool's run on a model or HIL
        board whose outputs are locked. ``commission``: the commissioning tool (bring-up only,
        docs/design/2026-11-12-bringup-commission-tool.md 5): COMMISSION to a commissioning build and nothing else,
        never a DRIVE. ``on_state(state, received_ns)``: called with every STATE this host accepted (never one it
        ignored as invalid or out of order) -- the teleop bridge's encoder speed reads these only. ``initial_seq``:
        the first frame's seq (the teleop bridge starts each session at a random one, so a STATE an earlier session's
        frame left in flight cannot pass for this session's acknowledgement). ``freshness``: the ``gap_clock="mcu"``
        rule's source -- ``delay_s(state, received_ns)``, ``rate_verified()``, ``rate_bound()``, ``on_disconnect()``."""
        if cfg.gap_clock == "mcu" and freshness is None:
            raise ValueError("the MCU-time gap rule needs a freshness source (delay bound and rate verdict)")
        self.cfg = cfg
        self._freshness = freshness
        self._state_delay_s: float | None = None
        self._state_delay_terms: dict | None = None  # what made _state_delay_s (diagnostics)
        self._on_state_cb = on_state
        self.client, self.allow_locked = Client(client), bool(allow_locked)
        self.commission_mode = bool(commission)
        if self.commission_mode and self.client is not Client.BRINGUP:
            raise ValueError("commission mode is for the bring-up client only")
        self._log = log or (lambda *a: None)
        self._decoder = LinkDecoder()
        self._state: McuState | None = None
        self._state_rx_ns: int | None = None
        self._stale_reported = False
        self._seq = int(initial_seq) & 0xFFFF
        self.last_queued_seq: int | None = None  # the newest queued frame's seq (the port's result names it)
        self.disconnects = 0  # how many times the adapter lost the device (the teleop bridge rechecks its session)
        # the e-stop waiver (docs/plans/2026-11-18-v1-estop-waiver.md 3): the opened tty's USB product string, read by
        # the adapter at each opening and forgotten on a disconnect; only it makes bit 7 a warning
        self.usb_product: str | None = None
        # the axis of a commissioning arming's zero COMMISSION: L, except on the bring-up tool's verified fw-single
        # path, which fixes the run's axis with it (partial-parts plan 2026-11-25 PR-B)
        self.commission_zero_axis = 0
        self.estop_waived = False
        self._outbox: deque[Outgoing] = deque()
        self._auth = _NONE
        self._auth_gen = 0  # bumps whenever authorization starts or ends
        self._arm: dict | None = None  # boot, prev_gen, gen, deadline_ns, reject_at_request
        self._inflight: Outgoing | None = None  # handed to the adapter, not yet written / unsent / abandoned
        self._written: deque = deque(maxlen=256)  # every DRIVE written: seq, nonzero, expires_ns, written_ns
        # the bench image (mcu-link contract 8): the latest BENCH_STATE whose pair_seq names the STATE decoded right
        # before it, and that STATE's seq; only a commission-mode host on a bench5 image sends bench commands
        self.bench_state: BenchState | None = None
        self.bench_state_rx_ns: int | None = None
        self._last_state_seq: int | None = None

    # ---------------------------------------------------------------- state
    @property
    def authorized(self) -> bool:
        return self._auth in (_ZERO_PENDING, _READY)

    @property
    def ready(self) -> bool:
        return self._auth == _READY

    def skip_seq(self, n: int) -> int:
        """Advance the next frame's seq by ``n`` (mod 2^16) and return it: the teleop bridge's session check writes two
        frames a random distance apart, a step no earlier session's consecutive seqs can show."""
        self._seq = (self._seq + int(n)) & 0xFFFF
        return self._seq

    @property
    def arming(self) -> bool:
        """An operator ARM asked for and not yet answered (no authorization yet, none revoked)."""
        return self._auth == _ARM_PENDING

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
        pending = [o.expires_ns for o in self._outbox if o.kind in _MOTION and o.nonzero]
        if self._inflight is not None and self._inflight.nonzero:
            pending.append(self._inflight.expires_ns)
        return {"pending_nonzero_expiries": pending, "written": [dict(w) for w in self._written]}

    def drop_drives(self, reason: str) -> None:
        """Drop the queued DRIVEs (a control generation change). A frame already being written is finished by
        the adapter and tracked when written: cutting it is not a cancel (4a' design D4)."""
        self._drop(lambda o: o.kind in _MOTION, reason)

    def _mcu_rule(self) -> bool:
        return self.cfg.gap_clock == "mcu" and bool(self._freshness.rate_verified())

    def _age_ns(self, now_ns: int) -> float:
        """The receipt age, plus the newest STATE's delay bound under the "mcu" rule."""
        age = now_ns - self._state_rx_ns
        if self._mcu_rule() and self._state_delay_s is not None:
            age += self._state_delay_s * 1e9
        return age

    def _fresh(self, now_ns: int) -> bool:
        return self._state_rx_ns is not None and self._age_ns(now_ns) <= self.cfg.state_max_age_s * 1e9

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
            delay_s=self._state_delay_s,
            gap_rule="mcu" if self._mcu_rule() else "host",
        )

    @property
    def state(self) -> McuState | None:
        return self._state

    def _revoke(self, reason: str, events: list) -> None:
        if self._auth != _NONE:
            events.append(HostEvent("authorization_revoked" if self.authorized else "arm_failed", {"reason": reason}))
        self._auth, self._arm = _NONE, None
        self._auth_gen += 1
        self._drop(lambda o: o.kind in _MOTION or o.kind in ("ARM", "BENCH_ARM"), f"revoked: {reason}")

    # ---------------------------------------------------------------- receive
    def on_bytes(self, data: bytes, now_ns: int) -> list[HostEvent]:
        self._log("received", now_ns, bytes(data), None)
        before = self._decoder.errors
        events: list[HostEvent] = []
        states = []
        for rec in self._decoder.feed(bytes(data), now_ns):
            if isinstance(rec.message, McuState):
                states.append(rec.message)
                self._last_state_seq = rec.seq
            elif isinstance(rec.message, BenchState):
                # its STATE came right before it (one write on the board, contract 8.5): any other is not a pair
                if self._last_state_seq is not None and rec.message.pair_seq == self._last_state_seq:
                    self.bench_state, self.bench_state_rx_ns = rec.message, now_ns
                self._last_state_seq = None
        if self.cfg.gap_clock == "mcu":
            self._on_read_mcu(states, now_ns, events)
        else:
            for s in states:
                self._on_state(s, now_ns, events)
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
        if self.commission_mode:
            if not commission_allowed(s.flags):
                return "not_commission_build"  # the two command kinds never cross builds (contract 3.3, 3.4)
            if s.flags & StateFlag.OUTPUTS_LOCKED and not self.allow_locked:
                return "outputs_locked"
            return None
        if self.client is Client.BRINGUP:
            if s.flags & StateFlag.COMMISSION_BUILD:
                return "commission_build"
            if s.flags & StateFlag.OUTPUTS_LOCKED and not self.allow_locked:
                return "outputs_locked"
            return None
        return drive_refusal(s.flags, self.client, estop_waived=self.estop_waived)

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
        if self._on_state_cb is not None:
            self._on_state_cb(s, now_ns)
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

    def _stale(self, why: str, value_s: float, events: list, **seen) -> None:
        """``seen``: the other clock's gap at the same moment (``host_gap_s``, ``mcu_gap_s``, ``delay_s``), so a record
        tells a host-side stall (the operating path) from an MCU-side silence (lead 2026-11-16)."""
        if not self._stale_reported:
            self._stale_reported = True
            events.append(HostEvent("stale", {"why": why, "value_s": value_s, **seen}))
            self._revoke("stale", events)

    def _on_read_mcu(self, states: list, rx: int, events: list) -> None:
        """One read under ``gap_clock="mcu"`` (design 3.3a): per STATE the gap checks and the steps that only
        revoke; the forward steps and the age check on the newest STATE of the read. A revoked authorization cannot
        come back within the read: forward steps start only from an operator's ARM (``_ARM_PENDING``)."""
        newest = None
        first = True
        rx_before, state_before, n_read = self._state_rx_ns, self._state, 0
        for s in states:
            if s.boot_id == 0:
                events.append(HostEvent("state_invalid", {"why": "boot_id 0"}))
                continue
            prev = self._state
            if prev is not None and s.boot_id == prev.boot_id and s.mcu_time_us <= prev.mcu_time_us:
                events.append(HostEvent("state_out_of_order", {"mcu_time_us": s.mcu_time_us}))
                continue
            if self._state_rx_ns is not None and rx < self._state_rx_ns:
                events.append(HostEvent("state_invalid", {"why": "host_clock"}))
                continue
            rule = self._mcu_rule()
            d = self._freshness.delay_s(s, rx) if rule else None
            terms_of = getattr(self._freshness, "delay_terms", None)  # diagnostics only (the bridge's source has it)
            d_terms = terms_of(s, rx) if rule and d is not None and terms_of is not None else None
            if self._state_rx_ns is not None and prev is not None:
                host_gap = (rx - self._state_rx_ns) / 1e9
                raw_mcu_gap = (s.mcu_time_us - prev.mcu_time_us) / 1e6 if s.boot_id == prev.boot_id else None
                if not rule:
                    if host_gap > self.cfg.state_max_age_s:
                        # the host rule's own event (unchanged)
                        if not self._stale_reported:
                            self._stale_reported = True
                            events.append(HostEvent("stale", {"age_s": host_gap}))
                            self._revoke("stale", events)
                elif first and host_gap > self.cfg.host_gap_s:
                    self._stale("host_gap", host_gap, events, mcu_gap_s=raw_mcu_gap, delay_s=d)
                elif d is not None and s.boot_id == prev.boot_id:
                    mcu_gap = (s.mcu_time_us - prev.mcu_time_us) / 1e6 * (1.0 + float(self._freshness.rate_bound()))
                    if mcu_gap > self.cfg.state_max_age_s:
                        self._stale("mcu_gap", mcu_gap, events, host_gap_s=host_gap, mcu_gap_s=raw_mcu_gap, delay_s=d)
                elif host_gap > self.cfg.state_max_age_s:
                    self._stale("host_gap", host_gap, events, mcu_gap_s=raw_mcu_gap, delay_s=d)
            if self._auth in (_ARM_PENDING, _ZERO_PENDING) and rx > self._arm["deadline_ns"]:
                self._revoke("arm_timeout", events)
            first = False
            self._state, self._state_rx_ns, self._state_delay_s = s, rx, d
            self._state_delay_terms = d_terms
            if self._on_state_cb is not None:
                self._on_state_cb(s, rx)
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
                self._enqueue_disarm(rx)
                continue
            self._advance_auth(s, rx, events, forward=False)
            newest = s
            n_read += 1
        if newest is None:
            return
        if self._mcu_rule() and self._state_delay_s is not None:
            age = self._age_ns(rx) / 1e9
            if age > self.cfg.state_max_age_s:
                # at the receipt the age is the newest STATE's delay bound alone: it was read that long after the MCU
                # made it -- with the host gap since the read before and the MCU-time gap over the read
                same = state_before is not None and state_before.boot_id == newest.boot_id
                self._stale("age", age, events, delay_s=self._state_delay_s, n_states=n_read,
                            terms=self._state_delay_terms,
                            host_gap_s=None if rx_before is None else (rx - rx_before) / 1e9,
                            mcu_gap_s=(newest.mcu_time_us - state_before.mcu_time_us) / 1e6 if same else None)
        if newest is self._state:
            self._advance_auth(newest, rx, events, forward=True)

    def _advance_auth(self, s: McuState, now_ns: int, events: list, forward: bool = True) -> None:
        a = self._arm
        if self._auth == _ARM_PENDING:
            if s.boot_id != a["boot"]:
                self._revoke("boot_changed", events)
            elif not forward:
                if s.mode != Mode.ARMED and s.reject_reason not in (RejectReason.NONE, a["reject_at_request"]):
                    self._revoke(f"rejected: {s.reject_reason.name.lower()}", events)
            elif s.mode == Mode.ARMED and s.arm_gen not in (0, a["prev_gen"]) and self.commission_mode:
                # the zero command of a commissioning build is a zero COMMISSION; axis 0 needs no steering or lift
                # sensor, so arming does not depend on them (design 3)
                a["gen"] = s.arm_gen
                zero = Commission(int(self.commission_zero_axis), 0,
                                  valid_after_echo_ms(now_ns + self.cfg.zero_valid_ms * 1_000_000, now_ns),
                                  s.arm_gen, s.boot_id, s.mcu_time_us)
                self._auth = _ZERO_PENDING
                self._enqueue("COMMISSION", zero, now_ns, expires_ns=now_ns + self.cfg.zero_valid_ms * 1_000_000)
                events.append(HostEvent("zero_sent", {"arm_gen": s.arm_gen}))
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
                self._enqueue("DRIVE", zero, now_ns, expires_ns=now_ns + self.cfg.zero_valid_ms * 1_000_000,
                              echo=(s.boot_id, s.mcu_time_us))
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
            elif forward and self._auth == _ZERO_PENDING and s.flags & StateFlag.ZERO_SEEN:
                self._auth = _READY
                events.append(HostEvent("armed_ready", {"arm_gen": s.arm_gen}))

    def _expire(self, now_ns: int, events: list) -> None:
        """Staleness of the last receipt and the arm deadline, as of now_ns."""
        if self._state_rx_ns is not None and not self._fresh(now_ns) and not self._stale_reported:
            if self._mcu_rule():
                self._stale("age", self._age_ns(now_ns) / 1e9, events, host_gap_s=(now_ns - self._state_rx_ns) / 1e9,
                            delay_s=self._state_delay_s)
            else:
                self._stale_reported = True
                events.append(HostEvent("stale", {"age_s": (now_ns - self._state_rx_ns) / 1e9}))
                self._revoke("stale", events)
        if self._auth in (_ARM_PENDING, _ZERO_PENDING) and now_ns > self._arm["deadline_ns"]:
            self._revoke("arm_timeout", events)

    def tick(self, now_ns: int) -> list[HostEvent]:
        events: list[HostEvent] = []
        self._expire(now_ns, events)
        return events

    def link_identity(self, product: str | None) -> list[HostEvent]:
        """The adapter opened the device and read its USB product string (None: none to read). A waiver image is a
        warning event, once per connection."""
        self.usb_product = product
        self.estop_waived = estop_waived(product)
        return [HostEvent("estop_waived", {"product": product})] if self.estop_waived else []

    def disconnected(self, now_ns: int) -> list[HostEvent]:
        """The adapter lost the device: forget the link, drop everything queued."""
        self.usb_product, self.estop_waived = None, False
        self.bench_state, self.bench_state_rx_ns, self._last_state_seq = None, None, None
        events: list[HostEvent] = []
        self._revoke("disconnected", events)
        self._drop(lambda o: True, "disconnected")
        self._state, self._state_rx_ns, self._stale_reported = None, None, False
        self._decoder.discard_partial()  # the next connection's bytes do not complete this one's half frame
        self._state_delay_s = None
        self._state_delay_terms = None
        if self._freshness is not None:
            self._freshness.on_disconnect()
        self.disconnects += 1
        events.append(HostEvent("disconnected"))
        return events

    # ---------------------------------------------------------------- operator requests
    def bench_refusal(self, s: McuState | None = None) -> str | None:
        """Why this host must not send a bench command (plan A1a 1, contract 8): only a commission-mode (bring-up)
        host, on the opened tty whose USB product string is exactly this tool's bench image (``lim=lr1 bench5``), whose
        STATE says a commissioning build with open outputs and no HIL plant."""
        s = self._state if s is None else s
        if not self.commission_mode:
            return "not_commission_host"
        if not bl.bench_image(self.usb_product):
            return "not_bench5_image"
        if s is None:
            return "no_state"
        if not s.flags & StateFlag.COMMISSION_BUILD:
            return "not_commission_build"
        if s.flags & (StateFlag.OUTPUTS_LOCKED | StateFlag.HIL_PLANT):
            return "outputs_locked_or_hil"
        return None

    def bench_set(self, run_id: int, ask: bl.BenchAsk, now_ns: int, count_sign: int = 1) -> Blocked | None:
        """Queue the run's BENCH_SET (duty mode, no measurement source, no distance stop -- A1a), echoing the latest
        STATE. Its seq is ``last_queued_seq``; the caller reads the board's acceptance from STATE."""
        refused = self.bench_refusal()
        if refused is not None:
            return Blocked(f"bench_refused: {refused}")
        s = self._state
        if not self._fresh(now_ns):
            return Blocked("no_state")
        if s.mode != Mode.DISARMED or self._auth != _NONE:
            return Blocked("not_disarmed")
        if len(self._outbox) >= self.cfg.outbox_max:
            return Blocked("outbox_full")
        try:
            msg = BenchSet(s.boot_id, s.mcu_time_us, int(run_id), int(BenchMode.DUTY), int(ask.axis),
                           int(BenchSource.NONE), int(count_sign), ask.bypass, ask.duty_max_e4(), ask.time_ms(), 0)
        except ValueError as exc:
            return Blocked(f"bad_value: {exc}")
        self._enqueue("BENCH_SET", msg, now_ns)
        return None

    def request_bench_arm(self, run_id: int, now_ns: int) -> Blocked | None:
        """The bench image's ARM (contract 8.2): consumes the BENCH_SET with ``run_id``; the same arming exchange as
        ``request_arm`` (ready after the zero)."""
        refused = self.bench_refusal()
        if refused is not None:
            return Blocked(f"bench_refused: {refused}")
        return self._request_arm(now_ns, lambda s: ("BENCH_ARM", BenchArm(s.boot_id, s.mcu_time_us, int(run_id))))

    def request_arm(self, now_ns: int) -> Blocked | None:
        if self.commission_mode and bl.bench_image(self.usb_product):
            return Blocked("bench_image_arms_with_bench_arm")  # the board refuses a plain ARM (wrong_build)
        return self._request_arm(now_ns, lambda s: ("ARM", Arm(s.boot_id, s.mcu_time_us)))

    def _request_arm(self, now_ns: int, build) -> Blocked | None:
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
        kind, msg = build(s)
        self._enqueue(kind, msg, now_ns, echo=(s.boot_id, s.mcu_time_us))
        return None

    def cancel_arm(self, now_ns: int, reason: str) -> list[HostEvent]:
        """Give up an arming exchange whose ARM or zero can no longer be written in time (real HIL 6a design 2): the
        authorization revoked (``arm_failed`` before the zero, ``authorization_revoked`` after) and the ARM and the
        motion frames dropped; the caller's permit latches on it."""
        events: list[HostEvent] = []
        if self._auth in (_ARM_PENDING, _ZERO_PENDING):
            self._revoke(f"arm_cancelled: {reason}", events)
        for e in events:
            self._log("event", now_ns, b"", {"event": e.kind, **e.detail})
        return events

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

    def queued_disarm_seq(self) -> int | None:
        """The seq of the DISARM waiting in the queue -- right after ``request_disarm`` the one it queued or reused
        (a DISARM being written is not reused: the request queues another; 6a-2b round 8), or None."""
        for o in self._outbox:
            if o.kind == "DISARM":
                return int(o.seq) & 0xFFFF
        return None

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
    def drive(self, cmd: RearDrive, lift_m: float, lift_hold: bool, expires_ns: int, now_ns: int, *,
              movable: bool | None = None) -> Blocked | None:
        """Queue one DRIVE for a decision that expires at ``expires_ns`` (host monotonic). ``movable``: whether the
        frame may still move an axis (the real autonomy's port counts a zero-wheel lift move and ``keep`` too, control
        tick contract 2); None: a non-zero wheel target, as before. Its seq is ``last_queued_seq`` once queued."""
        if self.commission_mode:
            return Blocked("commission_host")
        blocked, valid = self._motion_check(expires_ns, now_ns)
        if blocked is not None:
            return blocked
        s = self._state
        msg = Drive(cmd.left_wheel_rad_s, cmd.right_wheel_rad_s, cmd.steering_rad, float(lift_m), valid,
                    bool(lift_hold), s.arm_gen, s.boot_id, s.mcu_time_us)
        wheels_ = cmd.left_wheel_rad_s != 0.0 or cmd.right_wheel_rad_s != 0.0
        self._enqueue("DRIVE", msg, now_ns, expires_ns=expires_ns,
                      nonzero=wheels_ if movable is None else bool(movable) or wheels_)
        return None

    def commission(self, axis: int, duty_e4: int, expires_ns: int, now_ns: int) -> Blocked | None:
        """Queue one COMMISSION (one axis's open-loop duty, contract 3.4) expiring at ``expires_ns``; the same
        checks and validity rule as ``drive``. Only a commission-mode host sends it."""
        if not self.commission_mode:
            return Blocked("not_commission_host")
        if not (isinstance(axis, int) and isinstance(duty_e4, int) and 0 <= axis <= 3
                and -10000 <= duty_e4 <= 10000):
            return Blocked("bad_value")
        blocked, valid = self._motion_check(expires_ns, now_ns)
        if blocked is not None:
            return blocked
        s = self._state
        msg = Commission(axis, duty_e4, valid, s.arm_gen, s.boot_id, s.mcu_time_us)
        self._enqueue("COMMISSION", msg, now_ns, expires_ns=expires_ns, nonzero=duty_e4 != 0)
        return None

    def _motion_check(self, expires_ns: int, now_ns: int) -> tuple[Blocked | None, int]:
        """What a DRIVE and a COMMISSION both need, and the validity after the echo."""
        s = self._state
        if s is None:
            return Blocked("no_state"), 0
        if not self._fresh(now_ns):
            return Blocked("stale"), 0
        if self._auth == _ZERO_PENDING:
            return Blocked("zero_pending"), 0
        if self._auth != _READY:
            return Blocked("not_authorized"), 0
        refused = self.build_refusal(s)
        if refused is not None:
            return Blocked(f"build_refused: {refused}"), 0
        if expires_ns <= now_ns:
            return Blocked("expired"), 0
        try:
            valid = min(self.cfg.cap_ms, valid_after_echo_ms(expires_ns, self._state_rx_ns))
        except ValueError:
            return Blocked("expired"), 0
        if len(self._outbox) >= self.cfg.outbox_max:
            return Blocked("outbox_full"), 0
        return None, valid

    # ---------------------------------------------------------------- outbox
    def _enqueue(self, kind: str, msg, now_ns: int, *, expires_ns: int | None = None, nonzero: bool = False,
                 echo: tuple | None = None) -> None:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        self.last_queued_seq = seq
        out = Outgoing(kind, seq, encode(msg, seq), expires_ns, self._auth_gen, nonzero, echo)
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
            if o.kind in _MOTION and (o.expires_ns <= now_ns or o.generation != self._auth_gen):
                self._log("dropped", now_ns, o.data, {"kind": o.kind, "seq": o.seq, "why": "expired or stale"})
                if o.echo is not None and self._auth == _ZERO_PENDING and o.generation == self._auth_gen:
                    # the arming zero ran out unsent (real HIL 6a impl round 1 P2-2): the exchange ends here
                    self._revoke("zero_expired", [])
                continue
            self._inflight = o
            return o
        return None

    def _sent(self, out: Outgoing, now_ns: int) -> None:
        if self._inflight is out:
            self._inflight = None
        if out.kind in _MOTION:
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

    def frame_refused(self, out: Outgoing, now_ns: int, why: str) -> None:
        """The adapter refused ``out`` before its first byte (the observe run's DISARM-only boundary, PR 7 design
        1.1): never written, never counted as delivered -- dropped and logged."""
        if self._inflight is out:
            self._inflight = None
        self._log("refused", now_ns, out.data, {"kind": out.kind, "seq": out.seq, "why": why})

    def frame_abandoned(self, out: Outgoing, written: int, now_ns: int) -> None:
        """A frame the adapter had started (``written`` bytes out) but could not finish (device lost): counted
        as possibly delivered."""
        self._sent(out, now_ns)
        self._log("abandoned", now_ns, out.data, {"kind": out.kind, "seq": out.seq, "written": written})


__all__ = ["Blocked", "HostConfig", "HostEvent", "LinkHealth", "McuLinkHost", "Outgoing"]
