"""Message shape checks at the arbiter's boundary (contract section 1; the fake vehicle's rules). A malformed
message is refused ``invalid`` (reliable) or dropped (latest-value) before anything reads it -- a string where a
boolean belongs must not become ``True`` (Codex M-c review round 3 P1: ``deadman: "false"`` drove)."""

from __future__ import annotations

import math

INT_FIELDS = ("session_epoch", "control_gen", "arm_gen", "seq")
LEDGER_FIELDS = ("request_token", "request_seq")  # a reliable request's ledger key (console plan D7), when present
STR_FIELDS = ("lease_id", "pallet_id", "destination_id", "mission_id", "client_id", "source", "to", "action")


def is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def is_ratio(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and -1.0 <= v <= 1.0


def problem(msg) -> str | None:
    """None when the message's fields have the contract's types, else ``invalid``."""
    if not isinstance(msg, dict) or not isinstance(msg.get("t"), str):
        return "invalid"
    t = msg["t"]
    if "request_id" in msg and not (isinstance(msg["request_id"], str) and msg["request_id"]):
        return "invalid"
    for key in INT_FIELDS + LEDGER_FIELDS:
        if key in msg and not is_int(msg[key]):
            return "invalid"
    for key in STR_FIELDS:
        if key in msg and msg[key] is not None and not isinstance(msg[key], str):
            return "invalid"
    if "targets_revision" in msg and not is_int(msg["targets_revision"]):
        return "invalid"
    # B-P2c (start relocalization plan 1.2 step 7): the candidate set's generation, and the candidate picked from it --
    # required plain integers (a bool must not pick generation 1, Codex B-P2c design P2)
    if t in ("relocalize_pick", "relocalize_retry") and not is_int(msg.get("generation")):
        return "invalid"
    if t == "relocalize_pick" and not is_int(msg.get("candidate")):
        return "invalid"
    # F1 (plan 2026-10-31-operator-placed-start 1.2): a placed base pose of three finite numbers, and the preview id
    if t in ("relocalize_place", "relocalize_place_confirm") and not is_int(msg.get("generation")):
        return "invalid"
    if t == "relocalize_place":
        pose = msg.get("pose")
        if not (isinstance(pose, list) and len(pose) == 3 and all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in pose)):
            return "invalid"
    if t == "relocalize_place_confirm" and not is_int(msg.get("preview_id")):
        return "invalid"
    if t == "teleop":
        if not all(is_int(msg.get(k)) for k in INT_FIELDS) or not isinstance(msg.get("lease_id"), str):
            return "invalid"
        if not is_ratio(msg.get("speed_ratio")) or not is_ratio(msg.get("curvature_ratio")):
            return "invalid"
        if msg.get("lift_dir") not in (-1, 0, 1) or isinstance(msg.get("lift_dir"), bool):
            return "invalid"
        if msg.get("mode") not in ("direct", "video") or not isinstance(msg.get("deadman"), bool):
            return "invalid"
        frame = msg.get("frame_echo")
        if frame is not None and (not isinstance(frame, dict) or not is_int(frame.get("frame_id"))):
            return "invalid"
    if t in ("teleop", "echo"):
        echo = msg.get("echo")
        if not isinstance(echo, dict) or not is_int(echo.get("state_seq")) or not isinstance(echo.get("token"), str):
            return "invalid"
    return None
