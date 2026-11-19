"""The real loop's output record (docs/design/2026-11-19-real-loop.md 2.6; real control tick contract 1.6).

Every frame the loop queues is recorded with its seq, boot, arm generation, expiry, rear-axle speed, curvature,
steering, lift target and whether it may move an axis. Three things are kept apart:

- the **confirmed** target: the newest frame a STATE of the same boot and arm generation named (``last_rx_seq``,
  ``COMMAND_FRESH``) after its write;
- what **may still act**: every movable frame not settled -- queued, in flight or written, not yet expired and not
  replaced by a later confirmed frame -- as the largest speed per direction;
- the **candidate** (the loop's own, never written back).

``publish`` puts the confirmed values and the per-direction residuals into the core's state (``applied_command``,
``command_speed``, ``slam["applied_speed"]``, ``outstanding_motion``), so a check or an exception path never reads a
candidate as applied.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from forklift_core.mcu_link.messages import StateFlag


@dataclass
class Frame:
    seq: int
    boot_id: int | None
    arm_gen: int | None
    expires_ns: int
    speed_mps: float
    curvature_inv_m: float
    steering: tuple
    lift_m: float
    movable: bool
    kind: str = "drive"  # drive | hold | stop
    queued_ns: int = 0
    written_ns: int | None = None
    confirmed_ns: int | None = None


@dataclass
class OutputBook:
    frames: list = field(default_factory=list)
    confirmed: Frame | None = None
    dropped: list = field(default_factory=list)

    def queued(self, frame: Frame) -> None:
        self.frames.append(frame)

    def written(self, seq: int, written_ns: int) -> None:
        for f in self.frames:
            if f.seq == (int(seq) & 0xFFFF) and f.written_ns is None:
                f.written_ns = int(written_ns)

    def discarded(self, seq: int) -> None:
        """A frame the sender never wrote (dropped before its first byte): it cannot act."""
        for f in list(self.frames):
            if f.seq == (int(seq) & 0xFFFF) and f.written_ns is None:
                self.frames.remove(f)
                self.dropped.append(f)

    def on_state(self, state, rx_ns: int) -> None:
        """A new STATE: the frame it names, written before it was received, of its boot and arm generation, with its
        command still fresh, becomes the confirmed target; earlier frames are replaced by it."""
        seq = int(state.last_rx_seq) & 0xFFFF
        fresh = bool(state.flags & StateFlag.COMMAND_FRESH)
        for i, f in enumerate(self.frames):
            if (f.seq == seq and f.written_ns is not None and rx_ns > f.written_ns and fresh
                    and (f.boot_id, f.arm_gen) == (state.boot_id, state.arm_gen)):
                f.confirmed_ns = int(rx_ns)
                self.confirmed = f
                del self.frames[: i + 1]  # it and every earlier frame: replaced at the MCU
                return

    def settle(self, now_ns: int) -> None:
        """Expired frames act no more (the MCU drops a command past its validity)."""
        self.frames = [f for f in self.frames if f.expires_ns > now_ns]
        if self.confirmed is not None and self.confirmed.expires_ns <= now_ns:
            self.confirmed = None  # its command ran out: nothing confirmed acts now

    def may_act(self, now_ns: int) -> dict:
        """The largest speed per direction a movable frame may still drive (the confirmed one included while it
        lasts)."""
        out: dict = {}
        live = [f for f in self.frames if f.movable and f.expires_ns > now_ns]
        if self.confirmed is not None and self.confirmed.movable and self.confirmed.expires_ns > now_ns:
            live.append(self.confirmed)
        for f in live:
            if f.speed_mps != 0.0:
                d = 1 if f.speed_mps > 0 else -1
                out[d] = max(out.get(d, 0.0), abs(f.speed_mps))
        return out

    def lift_may_move(self, now_ns: int) -> bool:
        return any(f.movable and f.speed_mps == 0.0 and f.expires_ns > now_ns for f in self.frames)

    def publish(self, state: dict, slam: dict | None, now_ns: int, steering_now: tuple) -> None:
        """The core's view of what is applied: the confirmed target (zero and the measured steering when none), and
        what may still act per direction."""
        c = self.confirmed
        speed = 0.0 if c is None else float(c.speed_mps)
        steering = list(steering_now) if c is None else list(c.steering)
        state["applied_command"] = (speed, [float(v) for v in steering])
        state["command_speed"] = speed
        state["outstanding_motion"] = self.may_act(now_ns)
        if slam is not None:
            slam["applied_speed"] = speed


__all__ = ["Frame", "OutputBook"]
