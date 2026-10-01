"""SDK-free rules of the 2026-10-01 G2 rerun (docs/plans/2026-10-01-g2-rerun.md).

The planning-target oracle, the perception error against a pallet pose, the
handoff record comparison that decides whether a G2 and a G2a run correspond,
the same for a G2r repeat batch, and the repeat-capture statistics. Every
threshold here was fixed in the plan before any result was read.
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Sequence

import numpy as np

PLANNING_TARGETS = ("perception", "oracle_nominal")
STATE_TOLERANCE = 1e-6  # pose, velocity, steering, pallet pose components
TIME_TOLERANCE_S = 1e-9
DRIFT_POSITION_M = 0.001  # the capture stationarity contract's values
DRIFT_YAW_RAD = 0.001


def planning_target(mode: str, perception_target, nominal_target):
    """(target, source) for the mission plan after a valid detection.

    ``oracle_nominal`` substitutes the scenario's nominal pallet pose; the
    detection, re-observation and capture-failure handling before this point
    are unchanged, so an undetected seed stays undetected.
    """
    if mode == "perception":
        return perception_target, "camera_depth_detection"
    if mode == "oracle_nominal":
        return nominal_target, "scenario_nominal_pickup"
    raise ValueError(f"unknown planning target {mode!r}")


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def site_error(estimate: Sequence[float], truth: Sequence[float]) -> dict:
    """Planar position and wrapped yaw error of an (x, y, yaw) estimate."""
    return {
        "position_m": float(math.hypot(estimate[0] - truth[0], estimate[1] - truth[1])),
        "yaw_rad": float(wrap_angle(estimate[2] - truth[2])),
    }


def depth_sha256(depth: np.ndarray) -> str:
    """Hash of a depth array's exact bytes, NaN pattern included."""
    array = np.ascontiguousarray(np.asarray(depth))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode())
    digest.update(str(array.shape).encode())
    digest.update(array.tobytes())
    return digest.hexdigest()


def _flat(value) -> np.ndarray:
    """A (position, quaternion) pair or a plain vector, as one flat array."""
    if value is None:
        return np.zeros(0)
    if (
        isinstance(value, (list, tuple))
        and value
        and isinstance(value[0], (list, tuple, np.ndarray))
    ):
        return np.concatenate([np.ravel(np.asarray(v, dtype=float)) for v in value])
    return np.ravel(np.asarray(value, dtype=float))


def _close(a, b, tolerance: float) -> bool:
    if a is None or b is None:
        return False
    a, b = _flat(a), _flat(b)
    return a.shape == b.shape and bool(np.all(np.abs(a - b) <= tolerance))


HANDOFF_VECTORS = (
    "accepted_pose",
    "linear_velocity_mps",
    "angular_velocity_radps",
    "steering_rad",
    "pallet_position_m",
    "pallet_orientation_wxyz",
)
HANDOFF_EXACT = ("attempt_number", "candidate_index", "depth_sha256", "loop_step")
HANDOFF_TIMES = ("simulation_time_s", "t_before_capture_s")


def handoff_correspondence(a: dict | None, b: dict | None) -> dict:
    """Whether two runs' handoff records are the same state (plan criterion).

    Both missing means neither run reached a valid detection; that is
    reported separately ("no_handoff"), as are one-sided handoffs.
    """
    if a is None and b is None:
        return {"status": "no_handoff", "mismatched": []}
    if a is None or b is None:
        return {"status": "one_sided", "mismatched": ["handoff"]}
    mismatched = [k for k in HANDOFF_EXACT if a.get(k) != b.get(k)]
    mismatched += [
        k for k in HANDOFF_VECTORS if not _close(a.get(k), b.get(k), STATE_TOLERANCE)
    ]
    mismatched += [
        k for k in HANDOFF_TIMES if not _close(a.get(k), b.get(k), TIME_TOLERANCE_S)
    ]
    return {
        "status": "corresponds" if not mismatched else "does_not_correspond",
        "mismatched": mismatched,
    }


def attempts_correspond(g2_attempts: Sequence[dict], other: Sequence[dict]) -> bool:
    """Undetected seeds: every observation attempt saw the same depth."""
    return len(g2_attempts) == len(other) and all(
        x.get("depth_sha256") is not None
        and x.get("depth_sha256") == y.get("depth_sha256")
        and x.get("candidate_index") == y.get("candidate_index")
        for x, y in zip(g2_attempts, other, strict=True)
    )


def accepted_pose(attempt: dict):
    """An attempt's accepted pose: top level, or inside its capture diagnostics
    (records written before 2026-10-01 carry only the latter)."""
    if attempt.get("accepted_pose") is not None:
        return attempt["accepted_pose"]
    return (attempt.get("capture_diagnostics") or {}).get("accepted_pose")


def repeat_correspondence(g2_attempt: dict, g2r_attempt: dict) -> dict:
    """Whether a G2r batch was taken at G2's observation point."""
    mismatched = [
        k
        for k in ("candidate_index", "depth_sha256")
        if g2_attempt.get(k) != g2r_attempt.get(k)
    ]
    if not _close(
        accepted_pose(g2_attempt),
        accepted_pose(g2r_attempt),
        STATE_TOLERANCE,
    ):
        mismatched.append("accepted_pose")
    return {
        "status": "corresponds" if not mismatched else "does_not_correspond",
        "mismatched": mismatched,
    }


def depth_change(first: np.ndarray, other: np.ndarray) -> dict:
    """Numeric change on pixels valid in both, and valid/invalid flips, apart."""
    first, other = np.asarray(first, dtype=float), np.asarray(other, dtype=float)
    a, b = np.isfinite(first) & (first > 0), np.isfinite(other) & (other > 0)
    both = a & b
    delta = np.abs(other[both] - first[both])
    changed = delta > 0
    return {
        "common_valid_pixels": int(both.sum()),
        "changed_pixels": int(changed.sum()),
        "max_abs_change_m": float(delta.max()) if delta.size else 0.0,
        "mean_abs_change_m": float(delta.mean()) if delta.size else 0.0,
        "valid_to_invalid": int((a & ~b).sum()),
        "invalid_to_valid": int((~a & b).sum()),
    }


def drift(first_pose, pose) -> dict:
    """Cumulative motion of an accepted (position, wxyz) pose from the first."""
    p0, q0 = (np.asarray(v, dtype=float) for v in first_pose)
    p1, q1 = (np.asarray(v, dtype=float) for v in pose)

    def yaw(q):
        w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    position = float(np.linalg.norm(p1 - p0))
    heading = float(wrap_angle(yaw(q1) - yaw(q0)))
    return {
        "position_m": position,
        "yaw_rad": heading,
        "moved": position > DRIFT_POSITION_M or abs(heading) > DRIFT_YAW_RAD,
    }


def repeat_summary(captures: Sequence[dict]) -> dict:
    """Statistics over a repeat batch; index 0 is the original capture.

    Captures marked as moved are kept in the record but left out. The
    reference is the first valid detection among the kept captures; with at
    most one valid detection the spread statistics are null.
    """
    kept = [c for c in captures if not c.get("drift", {}).get("moved")]
    statuses = [c.get("status") for c in kept if c.get("status") is not None]
    valid = [c for c in kept if c.get("status") == "valid"]
    out = {
        "captures": len(captures),
        "kept": len(kept),
        "moved": len(captures) - len(kept),
        "capture_failures": sum(1 for c in captures if c.get("capture_failure")),
        "valid": len(valid),
        "status_changes": sum(
            1 for a, b in zip(statuses, statuses[1:], strict=False) if a != b
        ),
        "original_valid": bool(captures)
        and captures[0].get("status") == "valid"
        and not captures[0].get("drift", {}).get("moved"),
        "reference_index": None,
        "left_std_m": None,
        "right_std_m": None,
        "left_max_dev_m": None,
        "right_max_dev_m": None,
        "yaw_max_dev_rad": None,
    }
    if len(valid) < 2:
        if valid:
            out["reference_index"] = valid[0]["index"]
        return out
    reference = valid[0]
    out["reference_index"] = reference["index"]
    for side in ("left", "right"):
        centres = np.array([c[f"{side}_center_m"] for c in valid], dtype=float)
        # About the reference, so identical repeats give exactly zero.
        offsets = centres - centres[0]
        out[f"{side}_std_m"] = float(np.linalg.norm(offsets.std(axis=0)))
        out[f"{side}_max_dev_m"] = float(
            np.max(np.linalg.norm(centres - centres[0], axis=1))
        )
    out["yaw_max_dev_rad"] = float(
        max(
            abs(wrap_angle(c["insertion_yaw_rad"] - reference["insertion_yaw_rad"]))
            for c in valid
        )
    )
    return out
