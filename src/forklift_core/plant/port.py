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

import math
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
class PlantSample:
    """One measurement of the plant (sim-to-real P0-4 4b). A getter result: reading it consumes nothing -- odometry
    and continuity are computed by their own consumers from the absolute values here.

    ``steering``: the kinematics' steering vector (Isaac: the two front joints, as drive() indexed them; real: (q,)).
    ``lift_m``: the lift position, None when invalid (never 0 in its place). Real only: ``wheel_counts`` (absolute
    32-bit modular motor counts), ``mcu_time_us`` (the MCU clock), ``received_host_ns`` (the host monotonic receipt),
    ``boot_id``. Isaac only: ``raw_joints`` (the joint array of that one read) and ``sample_time_s`` (the
    simulation time the caller passes; the plant reads no clock). ``valid`` names what can be used."""

    steering: object
    lift_m: object
    sample_time_s: float | None = None
    raw_joints: object = None
    wheel_counts: tuple | None = None
    mcu_time_us: int | None = None
    received_host_ns: int | None = None
    boot_id: int | None = None
    valid: dict | None = None


@dataclass(frozen=True)
class PlantHealth:
    """The plant link's state for the arbiter (4b: the conversion; wiring it to a stop is 4e-2)."""

    fresh: bool
    mode: object
    fault_flags: int
    authorized: bool
    ready: bool
    reasons: tuple


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
        if not math.isfinite(cmd.drive.steering_rad[0]):
            # No steering to hold (an unknown measured q): a zero frame on the last finite intent if there is
            # one, else DISARM -- outputs to their safe state (Codex 4a' impl P2).
            if self._intent is not None and self._intent[0] == self.host.auth_epoch:
                return self.zero_wheels(now_ns, cmd.expires_ns)
            self.host.request_disarm(now_ns)
            return PortResult("blocked", "steering_unknown_disarmed")
        result = self._submit(cmd.drive.rear(), cmd.lift_m, cmd.lift_mode, cmd.expires_ns, now_ns)
        if result.kind == "queued":
            self._intent = (cmd.auth_epoch, cmd.drive.steering_rad[0], cmd.lift_m, cmd.lift_mode, cmd.expires_ns)
        return result

    def _compose(self, now_ns: int, expires_ns: int, *, lift: tuple | None = None) -> PortResult:
        if self._intent is None or self._intent[0] != self.host.auth_epoch:
            self._intent = None
            return PortResult("blocked", "no_intent")
        epoch, q, lift_m, lift_mode, old_expires = self._intent
        if lift is not None:
            lift_m, lift_mode = lift
        # The intent's own lifetime bounds the composed frame: a zero or a lift target never extends the decision
        # it is composed from (Codex 4a' impl P1-3); a longer life is a new decision through command().
        expires = min(int(expires_ns), int(old_expires))
        rear = RearDrive(0.0, 0.0, 0.0, 0.0, q, False)
        result = self._submit(rear, lift_m, lift_mode, expires, now_ns)
        if result.kind == "queued":
            self._intent = (epoch, q, lift_m, lift_mode, expires)
        return result

    def zero_wheels(self, now_ns: int, expires_ns: int) -> PortResult:
        """Both wheels to zero in the same frame as the latest steering and lift intent."""
        return self._compose(now_ns, expires_ns)

    def hold_lift(self, position_m: float, now_ns: int, expires_ns: int) -> PortResult:
        """The lift's target to ``position_m`` (lift_mode target, not the firmware hold flag), wheels zero."""
        return self._compose(now_ns, expires_ns, lift=(float(position_m), "target"))

    def _unsettled(self, now_ns: int) -> list:
        """The original expiries of the non-zero DRIVEs that could still act (release condition (a))."""
        t = self.host.drive_traffic()
        out = [e for e in t["pending_nonzero_expiries"] if e > now_ns]
        s, rx = self.host.state, self.host.state_rx_ns
        for i, w in enumerate(t["written"]):
            if not w["nonzero"] or now_ns >= w["expires_ns"]:
                continue
            heard = s is not None and rx is not None and rx > w["written_ns"]
            if heard and s.mode != Mode.ARMED:
                continue  # a later ARM has a new arm_gen this frame cannot match
            if heard:
                later = [x for x in t["written"][i + 1:] if x["written_ns"] < rx]
                if any(_seq_at_or_after(s.last_rx_seq, x["seq"]) for x in later):
                    continue  # the MCU accepted a later DRIVE: this one's target and deadline are replaced
                if s.last_rx_seq == w["seq"] and not s.flags & StateFlag.COMMAND_FRESH:
                    continue  # accepted, and its deadline has passed
            out.append(w["expires_ns"])
        return out

    def outstanding_until(self, now_ns: int) -> int | None:
        """The latest original expiry among the non-zero DRIVEs that could still act at ``now_ns``."""
        left = self._unsettled(now_ns)
        return max(left) if left else None

    def sample(self) -> PlantSample | None:
        """The latest STATE as a measurement (None before the first). A steering the map cannot read and a NaN lift
        are flagged invalid, never replaced (Codex 4b P2-3). Using an invalid reading is refused where control
        starts (4e-2)."""
        s = self.host.state
        if s is None:
            return None
        lift_ok = math.isfinite(s.lift_m)
        return PlantSample(steering=(float(s.steering_rad),), lift_m=float(s.lift_m) if lift_ok else None,
                           wheel_counts=(int(s.wheel_count_left), int(s.wheel_count_right)),
                           mcu_time_us=int(s.mcu_time_us), received_host_ns=self.host.state_rx_ns,
                           boot_id=int(s.boot_id),
                           valid={"steering": self.kinematics.cfg.steering.curvature(float(s.steering_rad)) is not None,
                                  "lift": lift_ok})

    def health(self, now_ns: int) -> PlantHealth:
        """The link's health now. It reads; it does not run the host's expiry (host.tick, driven by the control
        loop in 4e-2)."""
        h = self.host.health(now_ns)
        reasons = []
        if h.mode is None:
            reasons.append("no_state")
        elif not h.fresh:
            reasons.append("stale")
        if h.mode is not None and h.mode != Mode.ARMED:
            reasons.append(f"mode_{h.mode.name.lower()}")
        if h.fault_flags:
            reasons.append("faults")
        if h.authorized and not h.ready:
            reasons.append("zero_pending")
        elif not h.authorized and h.mode == Mode.ARMED:
            reasons.append("not_authorized")
        return PlantHealth(h.fresh, h.mode, h.fault_flags, h.authorized, h.ready, tuple(reasons))

    def older_nonzero_may_act(self, now_ns: int) -> bool:
        """Release condition (a) of design D4: could an older non-zero DRIVE still move the wheels? Every one
        queued, being written or written and not yet settled counts (Codex 4a' impl P1-1). Not a standstill
        judgement -- that is condition (b), a fresh measurement."""
        return bool(self._unsettled(now_ns))


__all__ = ["LIFT_MODES", "PlantCommand", "PlantHealth", "PlantSample", "PortResult", "RealPlantPort"]
