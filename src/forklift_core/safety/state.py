"""The arbiter's part of the ``state`` message (M-b plan B12; contract section 6). What cannot be measured is null,
never 0 or an estimate; the runner passes what it owns (pose, freshness, pacing)."""

from __future__ import annotations

from forklift_core.mission.commands import FINAL


RESUME_STEPS = ("stop_confirmed", "state_verified", "frame_reexpressed", "path_rechecked")


def resume_steps(check: dict | None) -> dict | None:
    """The core's last resume check as the contract's per-step ``pending / ok / failed(reason)`` (contract 6):
    steps before the one it reached are ok, that one pending -- or failed with its reason when the check itself
    records the refusal (a STOP mid-resume leaves it paused without failing, Codex M-b review round 3 P1; a later
    unrelated command does not hide the failure, round 4 P1); a finished resume is ok throughout."""
    if not check:
        return None
    step = check.get("step")
    reached = len(RESUME_STEPS) if step == "running" else (
        RESUME_STEPS.index(step) if step in RESUME_STEPS else 0)
    out = {}
    for i, name in enumerate(RESUME_STEPS):
        if i < reached:
            out[name] = "ok"
        elif i == reached:
            reason = check.get("reason")
            out[name] = f"failed({reason})" if check.get("refused") else "pending"
        else:
            out[name] = "pending"
    return out


def state_message(arbiter, out, safety: dict | None, *, now_mono: float, rear_axle_speed_mps: float | None = None,
                  applied: dict | None = None, pose: dict | None = None, freshness: dict | None = None,
                  pacing: dict | None = None, steering_rad: list | None = None,
                  lift_m: float | None = None) -> dict:
    """``applied`` is what the back end actually ran (after the Ackermann wheel-rate and steering-rate limits:
    ``speed_mps``, ``curvature_inv_m``, ``steering_rad``); without it the field is null -- the arbiter's own output is
    not the applied command (contract 6)."""
    a = arbiter
    lim = a.config.teleop_limits
    lease = a.lease
    reserved = None if a.reserved is None else max(0.0, a.reserved[1] - now_mono)
    mission = (safety or {}).get("mission") or {}
    load = (safety or {}).get("load") or {}
    loaded = load.get("loaded")
    command = None
    if a.mode == "TELEOP" and a.teleop is not None:
        speed, curvature = a.drive.target(a.teleop, loaded=bool(out.policy.loaded))
        command = {"speed_mps": speed if a.teleop["deadman"] else 0.0, "curvature_inv_m": curvature}
    blocked = (safety or {}).get("blocked") or {}
    permit = ({"limit_mps": out.permit[0], "reason": out.permit[1]} if out.permit is not None
              else {"limit_mps": blocked.get("cap"), "reason": blocked.get("reason")})
    last = a.mission.log[-1] if a.mission is not None and a.mission.log else {}
    return {
        "t": "state", "state_seq": a.ticks.next_seq(), "session_epoch": a.session_epoch,
        "mode": out.mode, "stop_kind": out.stop_kind, "control_gen": out.control_gen, "armed": out.armed,
        "arm_gen": a.arm_gen,
        "stopped_confirmed": bool(a.stop_confirmed),  # what rearm and mode changes wait for (contract 5)
        # ... unless waived: "lower_only" -- TELEOP entry and rearm taken still but unconfirmed (plan D14, main 10-19)
        "stand_waiver": a.stand_waiver(),
        "lease": {"holder": None if lease is None else lease.client_id,
                  "observers": sum(1 for c in a.clients if lease is None or c != lease.client_id),
                  "reserved_until_s": reserved},
        "pose": pose,
        "rear_axle_speed_mps": rear_axle_speed_mps,
        "steering_rad": steering_rad, "lift_m": lift_m,  # measured by the back end, null when it cannot (C3 b)
        "load": {"state": None if loaded is None else ("loaded" if loaded else "empty"), "basis": load.get("basis"),
                 # a live state names the pallet on the forks itself, kept past the mission's end (plan D5a); a
                 # batch one judges loaded by the phase, so the pallet is the mission's (C3)
                 "pallet_id": (load["pallet_id"] if "pallet_id" in load else mission.get("pallet_id")) if loaded
                 else None},
        "command": command,
        "applied": None if applied is None else dict(applied, stop_kind=out.stop_kind),
        "permit": permit,
        "limits": {"forward_mps": lim.direct_forward_mps, "reverse_mps": lim.direct_reverse_mps,
                   "video_forward_mps": lim.video_forward_mps, "video_reverse_mps": lim.video_reverse_mps,
                   "loaded_mps": lim.loaded_mps, "curvature_inv_m": lim.curvature_inv_m, "provisional": True},
        "drive_allowed": out.policy.drive_allowed, "lift_allowed": out.policy.lift_allowed,
        "teleop_allowed": out.policy.teleop_allowed, "teleop_forbidden_reason": out.policy.reason,
        # persistent console plan D14: which lift directions, the drive mode, the lift travel and the
        # recovery stage the core reports
        "lift_dirs": sorted(out.policy.lift_dirs), "drive_mode": out.policy.drive_mode,
        "lift_limit_m": a.config.lift_limit_m, "recovery": safety.get("recovery") if safety else None,
        "mission": None if not mission.get("id") else {
            "mission_id": mission.get("id"), "status": mission.get("status"), "phase": mission.get("phase"),
            "pallet_id": mission.get("pallet_id"), "destination_id": mission.get("destination_id"),
            "reason": ("teleop" if mission.get("status") == "held"
                       # an ended mission: its end's reason (the log's last entry), not an earlier refused resume
                       else last.get("reason") if mission.get("status") in FINAL
                       else (mission.get("resume") or {}).get("reason") if (mission.get("resume") or {}).get("refused")
                       else last.get("reason")),
            "phase_elapsed_s": mission.get("phase_elapsed_s"), "remaining_m": mission.get("remaining_m"),
            "resume_steps": resume_steps(mission.get("resume"))},
        "freshness": freshness, "pacing": pacing,
        "last_teleop_reject": a.last_teleop_reject, "stop_latch": a.preempt_latch,
    }
