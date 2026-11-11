"""The plant port's command contract (sim-to-real P0-4 4a', docs/design/2026-11-11-p04a-kinematics-and-command.md D4).

``PlantCommand`` carries the decision's time and absolute expiry (host monotonic, real only), the runner's
control generation (console lease / mission command; not the MCU's arm_gen), the MCU authorization epoch it
was made under, the finalized drive, the steering command, and the lift intent (``target``: lift_m is the new
target; ``keep``: the MCU keeps its current target, firmware lift_hold). ``PortResult`` is ``queued`` (in the
link's outbox -- not applied), ``refused`` (the kinematics cannot drive it) or ``blocked`` (link, authorization,
expiry, queue or generation: not sent). A blocked result is not a zero: an older command may still act until
``older_nonzero_may_act`` is false.

``RealPlantPort`` owns the real truck's DRIVE frame: it keeps the latest intent and composes zero_wheels /
hold_lift into the same frame. The link's queue and frame writing stay with ``McuLinkHost`` / ``TtyLink``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from forklift_core.mcu_link.messages import Mode, StateFlag
from forklift_core.plant.kinematics import RealDrive, RealKinematics
from forklift_core.plant.mcu_host import Blocked, McuLinkHost
from forklift_core.plant.real_vehicle import DriveRefusal, RearDrive

LIFT_MODES = ("target", "keep")


@dataclass(frozen=True)
class PlantCommand:
    decision_ns: int
    expires_ns: int
    control_gen: int
    auth_epoch: int
    drive: object  # the finalized drive, or a DriveRefusal
    steering_command: tuple
    lift_m: float
    lift_mode: str

    def __post_init__(self) -> None:
        if self.lift_mode not in LIFT_MODES:
            raise ValueError(f"lift_mode must be one of {LIFT_MODES}")


@dataclass(frozen=True)
class PortResult:
    kind: str  # "queued" | "refused" | "blocked"
    reason: str | None = None


def _seq_at_or_after(a: int, b: int) -> bool:
    return ((a - b) & 0xFFFF) < 0x8000


class RealPlantPort:
    def __init__(self, host: McuLinkHost, kinematics: RealKinematics) -> None:
        self.host, self.kinematics = host, kinematics
        self.control_gen = 0
        self._intent: tuple | None = None  # (auth_epoch, steering q, lift_m, lift_mode, expires_ns)

    def new_control_generation(self) -> int:
        """A lease or mission command change: queued DRIVEs go, the latest intent is forgotten."""
        self.control_gen += 1
        self.host.drop_drives("control_gen")
        self._intent = None
        return self.control_gen

    def make_command(self, decision_ns: int, expires_ns: int, drive, steering_command, lift_m: float,
                     lift_mode: str) -> PlantCommand:
        return PlantCommand(int(decision_ns), int(expires_ns), self.control_gen, self.host.auth_epoch, drive,
                            tuple(float(v) for v in np.asarray(steering_command, dtype=float)), float(lift_m),
                            lift_mode)

    def _submit(self, rear: RearDrive, lift_m: float, lift_mode: str, expires_ns: int, now_ns: int) -> PortResult:
        out = self.host.drive(rear, lift_m, lift_mode == "keep", expires_ns, now_ns)
        return PortResult("blocked", out.reason) if isinstance(out, Blocked) else PortResult("queued")

    def command(self, cmd: PlantCommand, now_ns: int) -> PortResult:
        if cmd.control_gen != self.control_gen:
            return PortResult("blocked", "stale_generation")
        if cmd.auth_epoch != self.host.auth_epoch:
            return PortResult("blocked", "stale_authorization")
        if isinstance(cmd.drive, DriveRefusal):
            return PortResult("refused", cmd.drive.reason)
        if not isinstance(cmd.drive, RealDrive):
            raise TypeError("the real port takes a RealDrive")
        result = self._submit(cmd.drive.rear(), cmd.lift_m, cmd.lift_mode, cmd.expires_ns, now_ns)
        if result.kind == "queued":
            self._intent = (cmd.auth_epoch, cmd.drive.steering_rad[0], cmd.lift_m, cmd.lift_mode, cmd.expires_ns)
        return result

    def _compose(self, now_ns: int, expires_ns: int, *, lift: tuple | None = None) -> PortResult:
        if self._intent is None or self._intent[0] != self.host.auth_epoch:
            self._intent = None
            return PortResult("blocked", "no_intent")
        epoch, q, lift_m, lift_mode, _old = self._intent
        if lift is not None:
            lift_m, lift_mode = lift
        rear = RearDrive(0.0, 0.0, 0.0, 0.0, q, False)
        result = self._submit(rear, lift_m, lift_mode, expires_ns, now_ns)
        if result.kind == "queued":
            self._intent = (epoch, q, lift_m, lift_mode, expires_ns)
        return result

    def zero_wheels(self, now_ns: int, expires_ns: int) -> PortResult:
        """Both wheels to zero in the same frame as the latest steering and lift intent."""
        return self._compose(now_ns, expires_ns)

    def hold_lift(self, position_m: float, now_ns: int, expires_ns: int) -> PortResult:
        """The lift's target to ``position_m`` (lift_mode target, not the firmware hold flag), wheels zero."""
        return self._compose(now_ns, expires_ns, lift=(float(position_m), "target"))

    @property
    def outstanding_until_ns(self) -> int | None:
        last = self.host.drive_traffic()["last_nonzero"]
        return None if last is None else last["expires_ns"]

    def older_nonzero_may_act(self, now_ns: int) -> bool:
        """Release condition (a) of design D4: could an older non-zero DRIVE still move the wheels? Not a
        standstill judgement -- that is condition (b), a fresh measurement."""
        t = self.host.drive_traffic()
        if t["queued_nonzero"] or t["inflight_nonzero"]:
            return True
        last = t["last_nonzero"]
        if last is None or now_ns >= last["expires_ns"]:
            return False
        s, rx = self.host.state, self.host.state_rx_ns
        if s is None or rx is None or rx <= last["written_ns"]:
            return True  # nothing heard since the write
        if s.mode != Mode.ARMED:
            return False  # a later ARM has a new arm_gen this frame cannot match
        if t["zero_after_seq"] is not None and _seq_at_or_after(s.last_rx_seq, t["zero_after_seq"]):
            return False  # a later zero DRIVE was accepted
        if _seq_at_or_after(s.last_rx_seq, last["seq"]) and not s.flags & StateFlag.COMMAND_FRESH:
            return False  # accepted, and its deadline has passed
        return True


__all__ = ["LIFT_MODES", "PlantCommand", "PortResult", "RealPlantPort"]
