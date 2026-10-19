"""Where manual driving is allowed (M-b plan B7; design 4.5): from the mission's safety state and the feedback,
judged every control cycle, drive and lift separately. Unknown is refused."""

from __future__ import annotations

from dataclasses import dataclass


LIFT_DOWN_TOL_M = 0.01  # plan D14 (main condition 6): above it the forks are lifted -- lower first (LIFT_RESUME_TOL_M)
BOTH_WAYS = frozenset({-1, 1})
NONE = frozenset()


@dataclass(frozen=True)
class Policy:
    drive_allowed: bool
    lift_allowed: bool
    reason: str | None  # why driving is refused (None: allowed)
    loaded: bool = False  # drive with the loaded limits and footprint
    lift_dirs: frozenset = NONE  # plan D14: the lift directions allowed this cycle
    drive_mode: str = "normal"  # plan D14: "normal" or "reverse_straight" (a recovery along the pocket axis)

    @property
    def teleop_allowed(self) -> bool:
        # Plan D14 (Codex design round 1 P2-5): a state that only lowers may still be entered.
        return self.drive_allowed or self.lift_allowed


def _lift(dirs) -> dict:
    dirs = frozenset(dirs)
    return {"lift_allowed": bool(dirs), "lift_dirs": dirs}


def teleop_policy(safety: dict | None, *, feedback_ok: bool, standing: bool, lift_m: float | None = 0.0,
                  steering_rad: float | None = 0.0, still: bool = False) -> Policy:
    """``lift_m`` / ``steering_rad``: the measured lift and the larger measured steering angle (None: not measured).
    ``still``: not rolling though not confirmed standing (main 2026-10-19) -- enough for a recovery's lowering."""
    def refuse(reason, dirs=NONE):
        return Policy(False, reason=reason, **_lift(dirs))

    if not safety:
        return refuse("state_unknown")
    load = (safety.get("load") or {}).get("loaded")
    in_pocket = (safety.get("fork_in_pocket") or {}).get("value")
    if load is None or in_pocket is None:
        return refuse("state_unknown")
    if not feedback_ok:
        return refuse("no_feedback:rear_axle_speed")
    near = safety.get("near") or {}
    frame = safety.get("frame") or {}
    phase = (safety.get("mission") or {}).get("phase")
    recovery = safety.get("recovery")
    if near.get("active"):
        return refuse("near_section")
    if recovery:
        return _recovery(safety, recovery, standing=standing or still, lift_m=lift_m, steering_rad=steering_rad)
    if safety.get("recovery_required"):
        # A live mission ended where the forks may be in a pocket or the pallet on them and no recovery context
        # says how to get out (persistent console plan D6): no manual driving.
        return refuse("recovery_required")
    relation = (safety.get("fork_in_pocket") or {}).get("relation")  # live only (persistent console plan D5a)
    if in_pocket and relation != "carrying":
        # Inside a pocket with the pallet supported elsewhere, or not known: no manual driving (the recovery rows
        # above open it after a live mission ended recovery_required -- plan D14). Carrying -- the lift verdict
        # passed, the pallet off the floor -- drives with the load (user decision).
        return refuse("fork_in_pocket")
    if frame.get("hold") in ("approach", "insert"):
        return refuse("near_section")  # a running near contract (an ended one's hold has its recovery context)
    if not frame.get("slam", False) and phase == "approach":
        return refuse("near_section_unknown")
    if load:
        return Policy(True, False, None, loaded=True)
    return Policy(True, reason=None, **_lift(BOTH_WAYS if standing and lift_m is not None else NONE))


def _recovery(safety, recovery: dict, *, standing: bool, lift_m, steering_rad) -> Policy:
    """Plan D14's recovery rows: lower first (standing), then a straight reverse along the pocket axis -- after the
    manual ground verdict, with the evidence valid and the truck aligned to the axis; then the manual clear."""
    def refuse(reason, dirs=NONE):
        return Policy(False, reason=reason, **_lift(dirs))

    if lift_m is None:
        return refuse("recovery:no_lift_measure")
    relation = (safety.get("fork_in_pocket") or {}).get("relation")
    stage = recovery.get("stage")
    if stage == "in_pocket":
        if not safety.get("recovery_required"):
            return refuse("fork_in_pocket")
        if lift_m > LIFT_DOWN_TOL_M:
            return refuse("recovery:lower_first", frozenset({-1}) if standing else NONE)
        if relation != "constrained":
            return refuse("recovery:unknown")  # a pallet that slipped or tilted: no pocket axis to reverse along
        if not recovery.get("grounded"):
            return refuse("recovery:ground_wait")
    elif stage != "manual_clear":
        return refuse("recovery:stage_unknown")
    elif relation != "free":
        return refuse("recovery:unknown")  # a contact in the manual clear (Codex D14 impl round 1 P1-2)
    if not recovery.get("evidence"):
        return refuse("recovery:evidence")
    if not recovery.get("aligned"):
        return refuse("recovery:misaligned")
    if steering_rad is None:
        return refuse("recovery:no_steering")
    return Policy(True, reason=None, drive_mode="reverse_straight", **_lift(NONE))
