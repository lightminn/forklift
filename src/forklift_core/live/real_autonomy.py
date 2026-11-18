"""The real truck's autonomy executor parts (docs/design/2026-11-19-real-assembly.md, 4b): the AUTO permit state the
final DRIVE and the watches read. No device is opened here; the 50 Hz loop that drives it is 4c.

States (design 7): OFF -> ARMING -> READY -> DRIVING, the S10 control stop STOP_PENDING -> DISARMING, and LATCHED.
The AUTO request is one new mission submit; the MCU ARM is asked once, after the core adopted the plan. Every latch
runs the same order: unsent candidates dropped, DISARM queued and a transmit tried, the arbiter STOPPED, then the
mission collected -- a collection failure cannot undo the first three. A latch is left only where the arbiter really
moves STOPPED -> PAUSED on a rearm the real conditions passed.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from forklift_core.mcu_link.messages import Mode

OFF, ARMING, READY, DRIVING, STOP_PENDING, DISARMING, LATCHED = (
    "OFF", "ARMING", "READY", "DRIVING", "STOP_PENDING", "DISARMING", "LATCHED")
STOP_ACK_S = 0.100  # the control stop's frame confirmed by a STATE naming its seq within this (design 7, provisional)


@dataclass
class PermitEvent:
    t_ns: int
    event: str
    reason: str | None = None
    detail: dict = field(default_factory=dict)


class AutoPermit:
    """One AUTO permit for the real truck. ``host`` is the ``McuLinkHost``, ``port`` the ``RealPlantPort``.
    ``stop_confirmed(now_ns)`` says the truck was seen standing on new samples after the last drive settled (the
    evidence the executor supplies, design 6)."""

    def __init__(self, host, port, *, stop_confirmed, stop_ack_s: float = STOP_ACK_S) -> None:
        self.host, self.port, self.stop_confirmed = host, port, stop_confirmed
        self.stop_ack_ns = int(stop_ack_s * 1e9)
        self.state = OFF
        self.reason: str | None = None
        self.events: list[PermitEvent] = []
        self._stop_seq: int | None = None
        self._stop_since_ns: int | None = None
        self._stop_boot_gen: tuple | None = None

    # --- what the rest reads ------------------------------------------------------------------------------------
    @property
    def armed(self) -> bool:
        """The watches' input (design 7): the arbiter drops ``armed`` on entering AUTO, so they read this."""
        if self.state in (ARMING, READY, DRIVING, STOP_PENDING, DISARMING):
            return True
        s = self.host.state  # a latch's DISARM not yet seen: the MCU may still be armed
        return self.state == LATCHED and s is not None and s.mode != Mode.DISARMED

    def drive_allowed(self) -> bool:
        """A mission DRIVE may be submitted (and only then)."""
        return self.state in (READY, DRIVING) and self.host.ready

    def _note(self, now_ns: int, event: str, reason: str | None = None, **detail) -> None:
        self.events.append(PermitEvent(int(now_ns), event, reason, detail))

    # --- arming ---------------------------------------------------------------------------------------------------
    def adopted(self, now_ns: int, *, pre_arm_refusal: str | None) -> bool:
        """The core adopted the submitted mission's plan (``adopted_id == mission_id``, running, approach) and the
        lease and generation were checked again by the caller: the control generation moves, the queue goes, and
        the MCU ARM is asked once. ``pre_arm_refusal``: the pre-ARM checks' first failure (design 6: DISARMED, a
        fresh stop, the old commands settled). Returns whether the ARM was asked."""
        if self.state != OFF:
            self.latch(now_ns, f"adopted_in_{self.state.lower()}")
            return False
        if pre_arm_refusal is not None:
            self.latch(now_ns, f"pre_arm:{pre_arm_refusal}")
            return False
        self.port.new_control_generation()
        blocked = self.host.request_arm(now_ns)
        if blocked is not None:
            self.latch(now_ns, f"arm_blocked:{getattr(blocked, 'reason', blocked)}")
            return False
        self.state = ARMING
        self._note(now_ns, "arm_requested")
        return True

    def drove(self, now_ns: int) -> None:
        """The first mission DRIVE went into the queue."""
        if self.state == READY:
            self.state = DRIVING
            self._note(now_ns, "driving")

    def tick(self, now_ns: int, transmit=None, arbiter_stop=None, collect=None) -> None:
        """Each loop cycle after the host's STATEs and ``host.tick``: ARMING -> READY on the host's ready, the
        authorization's loss latches, the control stop's confirmation and its DISARM run their course."""
        host = self.host
        if self.state == ARMING:
            if host.ready:
                self.state = READY
                self._note(now_ns, "ready")
            elif not host.arming and not host.authorized:
                self.latch(now_ns, "arm_failed", transmit, arbiter_stop, collect)
        elif self.state in (READY, DRIVING):
            if not host.authorized:
                self.latch(now_ns, "authorization_revoked", transmit, arbiter_stop, collect)
        elif self.state == STOP_PENDING:
            s = host.state
            if s is not None and self._seq_echoed(s) and (s.boot_id, s.arm_gen) == self._stop_boot_gen:
                self._note(now_ns, "control_stop_confirmed", seq=self._stop_seq)
                self._disarm(now_ns, "s10_stopped", transmit)
            elif now_ns - self._stop_since_ns > self.stop_ack_ns or not host.authorized:
                self._disarm(now_ns, "control_stop_unconfirmed", transmit)
        elif self.state == DISARMING:
            s = host.state
            if s is not None and s.mode == Mode.DISARMED and host.health(now_ns).fresh:
                self.state = LATCHED
                self._note(now_ns, "disarmed")

    def _seq_echoed(self, s) -> bool:
        # the 16-bit seq the MCU last received is this frame's or a later one (mod 2^16, within half the space)
        return ((int(s.last_rx_seq) - int(self._stop_seq)) & 0xFFFF) < 0x8000

    # --- stops ------------------------------------------------------------------------------------------------------
    def latch(self, now_ns: int, reason: str, transmit=None, arbiter_stop=None, collect=None) -> None:
        """Any latch reason (design 7): (1) unsent candidates and earlier requests dropped, (2) a DISARM queued and a
        transmit tried, (3) the arbiter STOPPED (``arbiter_stop(reason)``; the caller skips a second boundary in the
        same tick), (4) the running mission collected (``collect(reason)``). A latch while latched keeps the first
        reason."""
        first = self.state != LATCHED
        if first:
            self.reason = reason
        self.port.new_control_generation()  # (1) the queue and the intent go
        self.host.request_disarm(now_ns)  # (2)
        if transmit is not None:
            transmit()
        self.state = LATCHED
        self._stop_seq = None
        self._note(now_ns, "latched", reason, first=first)
        if arbiter_stop is not None:
            arbiter_stop(reason)  # (3)
        if collect is not None:
            collect(reason)  # (4): last -- its failure cannot undo (1)-(3)

    def control_stop(self, now_ns: int, seq: int) -> None:
        """S10's end (design 7): the caller made the stop frame (wheels 0, the measured lift) on a new generation and
        queued it as ``seq``; it is confirmed by a STATE of this boot and arm generation naming the seq within
        STOP_ACK_S, then DISARM. The normal recovery lock does not drop it."""
        s = self.host.state
        self._stop_seq, self._stop_since_ns = int(seq) & 0xFFFF, int(now_ns)
        self._stop_boot_gen = None if s is None else (s.boot_id, s.arm_gen)
        self.state = STOP_PENDING
        self._note(now_ns, "control_stop", seq=self._stop_seq)

    def _disarm(self, now_ns: int, reason: str, transmit=None) -> None:
        self.port.new_control_generation()
        self.host.request_disarm(now_ns)
        if transmit is not None:
            transmit()
        self.reason = self.reason or reason
        self.state = DISARMING
        self._note(now_ns, "disarm", reason)

    # --- rearm --------------------------------------------------------------------------------------------------
    def rearm_refusal(self, now_ns: int) -> str | None:
        """The real conditions a rearm passes before its ACK (design 7; the arbiter's ``rearm_refusal``): a fresh
        DISARMED with no faults on this boot, no earlier command that may still act -- a queued or written DRIVE of
        any kind not yet expired counts, since a zero-wheel lift is not tracked yet (4c) -- and a stop seen after."""
        if self.state not in (LATCHED, OFF):
            return f"permit_{self.state.lower()}"
        host = self.host
        s = host.state
        if s is None or not host.health(now_ns).fresh:
            return "state_stale"
        if s.mode != Mode.DISARMED:
            return f"mcu_{s.mode.name.lower()}"
        if int(s.fault_flags):
            return "mcu_faults"
        if self.port.older_nonzero_may_act(now_ns):
            return "old_command_may_act"
        traffic = host.drive_traffic()
        if traffic["pending_nonzero_expiries"] or any(w["expires_ns"] > now_ns for w in traffic["written"]):
            return "outstanding_unknown"
        if not self.stop_confirmed(now_ns):
            return "stop_unconfirmed"
        return None

    def rearmed(self, now_ns: int, control_gen: int) -> None:
        """The arbiter's ``on_rearmed``: the latch goes with the arbiter's STOPPED -> PAUSED, nowhere else."""
        if self.state == LATCHED:
            self.state = OFF
            self._note(now_ns, "rearmed", control_gen=int(control_gen))


__all__ = ["ARMING", "AutoPermit", "DISARMING", "DRIVING", "LATCHED", "OFF", "READY", "STOP_ACK_S", "STOP_PENDING"]
