"""The arbiter's part of the ``state`` message (M-b plan B12; contract section 6). What cannot be measured is null,
never 0 or an estimate; the runner passes what it owns (pose, freshness, pacing)."""

from __future__ import annotations


def state_message(arbiter, out, safety: dict | None, *, now_mono: float, rear_axle_speed_mps: float | None = None,
                  pose: dict | None = None, freshness: dict | None = None, pacing: dict | None = None) -> dict:
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
        "lease": {"holder": None if lease is None else lease.client_id,
                  "observers": sum(1 for c in a.clients if lease is None or c != lease.client_id),
                  "reserved_until_s": reserved},
        "pose": pose,
        "rear_axle_speed_mps": rear_axle_speed_mps,
        "load": {"state": None if loaded is None else ("loaded" if loaded else "empty"), "basis": load.get("basis")},
        "command": command,
        "applied": {"speed_mps": out.speed_mps, "curvature_inv_m": out.curvature_inv_m, "stop_kind": out.stop_kind},
        "permit": permit,
        "limits": {"forward_mps": lim.direct_forward_mps, "reverse_mps": lim.direct_reverse_mps,
                   "video_forward_mps": lim.video_forward_mps, "video_reverse_mps": lim.video_reverse_mps,
                   "loaded_mps": lim.loaded_mps, "curvature_inv_m": lim.curvature_inv_m, "provisional": True},
        "drive_allowed": out.policy.drive_allowed, "lift_allowed": out.policy.lift_allowed,
        "teleop_allowed": out.policy.teleop_allowed, "teleop_forbidden_reason": out.policy.reason,
        "mission": None if not mission.get("id") else {
            "mission_id": mission.get("id"), "status": mission.get("status"), "phase": mission.get("phase"),
            "reason": last.get("reason"), "phase_elapsed_s": mission.get("phase_elapsed_s"),
            "resume_steps": mission.get("resume")},
        "freshness": freshness, "pacing": pacing,
        "last_teleop_reject": a.last_teleop_reject, "stop_latch": a.preempt_latch,
    }
