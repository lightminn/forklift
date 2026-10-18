"""Where manual driving is allowed (M-b plan B7; design 4.5): from the mission's safety state and the feedback,
judged every control cycle, drive and lift separately. Unknown is refused."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Policy:
    drive_allowed: bool
    lift_allowed: bool
    reason: str | None  # why driving is refused (None: allowed)
    loaded: bool = False  # drive with the loaded limits and footprint

    @property
    def teleop_allowed(self) -> bool:
        return self.drive_allowed


def teleop_policy(safety: dict | None, *, feedback_ok: bool, standing: bool) -> Policy:
    def refuse(reason):
        return Policy(False, False, reason)

    if not safety:
        return refuse("state_unknown")
    load = (safety.get("load") or {}).get("loaded")
    in_pocket = (safety.get("fork_in_pocket") or {}).get("value")
    if load is None or in_pocket is None:
        return refuse("state_unknown")
    if not feedback_ok:
        return refuse("no_feedback:rear_axle_speed")
    if safety.get("recovery_required"):
        # A live mission ended where the forks may be in a pocket or the pallet on them: the phase is all the state
        # knows until D5 judges the pockets (persistent console plan D6), so no manual driving.
        return refuse("recovery_required")
    relation = (safety.get("fork_in_pocket") or {}).get("relation")  # live only (persistent console plan D5a)
    if in_pocket and relation != "carrying":
        # Inside a pocket with the pallet supported elsewhere, or not known: no manual driving (recovery is D14).
        # Carrying -- the lift verdict passed, the pallet off the floor -- drives with the load (user decision).
        return refuse("fork_in_pocket")
    near = safety.get("near") or {}
    frame = safety.get("frame") or {}
    phase = (safety.get("mission") or {}).get("phase")
    if near.get("active") or frame.get("hold") in ("approach", "insert"):
        return refuse("near_section")
    if not frame.get("slam", False) and phase == "approach":
        return refuse("near_section_unknown")
    if load:
        return Policy(True, False, None, loaded=True)
    return Policy(True, standing, None)
