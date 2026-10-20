# src/forklift_core/perception/bottom_deck_permit.py
"""The bottom-deck evidence bound to its target, and the insert permission it gives (plan 2026-10-19 task 3).

The side legs judge a target -- a map centre, a model yaw, a shape and an entry axis -- that the far range chose.
By the time the truck stands at the approach end the near capture has re-estimated the pallet it will insert
into. The evidence permits that insertion only when it reads ``present`` and was taken of the same pallet: the
same shape, the centre within ``TARGET_TOLERANCE_M`` and the same entry axis (either way round). ``present``
clears ``bottom_deck`` alone; it does not lift any other pending item or fork-fit refusal.
"""

import hashlib
import json
import math
from dataclasses import asdict, dataclass

from forklift_core._validation import _finite_scalar
from forklift_core.perception.bottom_deck_evidence import VERDICTS, BottomDeckParams

TARGET_TOLERANCE_M = 0.15
# The near capture's insertion yaw against the recorded entry axis. Far from the 45 deg that would confuse the
# axes, wide against the candidate error the evaluator draws (+-5 deg) and the near estimate's own.
AXIS_TOLERANCE_RAD = math.radians(15.0)
REFUSALS = ("no_bottom_deck", "bottom_deck_unobserved", "bottom_deck_target_mismatch")
AXES = ("x", "y")


@dataclass(frozen=True)
class BottomDeckRecord:
    verdict: str                # the two-face policy over the faces below: present | absent | unobserved
    faces: tuple                # per face: what was seen and judged (the mission's attempt records)
    centre_xy: tuple            # the judged target's pallet centre, map frame
    model_yaw_rad: float        # its model +x direction, map frame
    shape: str                  # geometry_version: epal6 | t11_06
    axis: str                   # the hypothesis' entry axis in the model frame: x | y
    support_z_m: float          # the support height the far estimate claimed
    candidate_id: str | None    # the far candidate and track the target came from
    track_id: str | None
    captured_s: tuple           # the side captures' times
    correction_s: float | None  # the map correction the target was read under (its stamp)
    params_sha256: str          # params_digest of the judge's parameters and thresholds

    def __post_init__(self) -> None:
        if self.verdict not in VERDICTS:
            raise ValueError(f"verdict must be one of {VERDICTS}")
        if self.axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}")
        if len(self.centre_xy) != 2:
            raise ValueError("centre_xy must be (x, y)")
        for i, v in enumerate(self.centre_xy):
            _finite_scalar(v, f"centre_xy[{i}]")
        _finite_scalar(self.model_yaw_rad, "model_yaw_rad")
        _finite_scalar(self.support_z_m, "support_z_m")

    def to_record(self) -> dict:
        """The JSON form the mission state keeps (the runner writes the state as JSON; Codex task 3 P2)."""
        return json.loads(json.dumps(asdict(self)))

    @classmethod
    def from_record(cls, record: dict) -> "BottomDeckRecord":
        return cls(**{**record, **{k: tuple(record[k]) for k in ("faces", "centre_xy", "captured_s")}})

    def entry_yaw_rad(self) -> float:
        """The insertion yaw the record's entry axis implies (one of the two directions along it)."""
        return self.model_yaw_rad + (0.0 if self.axis == "x" else math.pi / 2)


def params_digest(params: BottomDeckParams) -> str:
    return hashlib.sha256(json.dumps(asdict(params), sort_keys=True).encode()).hexdigest()


def _axis_error(a: float, b: float) -> float:
    """Angle between two directions taken either way round (0 .. pi/2)."""
    d = (a - b) % math.pi
    return min(d, math.pi - d)


def insert_refusal(record: BottomDeckRecord | None, target_xy, target_yaw_rad: float, shape: str) -> str | None:
    """None when the record permits inserting into this target, else the refusal (one of REFUSALS). The verdict
    is read first: an absent record is "no bottom deck" whatever it was taken of."""
    if record is None or record.verdict == "unobserved":
        return "bottom_deck_unobserved"
    if record.verdict == "absent":
        return "no_bottom_deck"
    if (record.shape != shape
            or math.hypot(target_xy[0] - record.centre_xy[0], target_xy[1] - record.centre_xy[1]) > TARGET_TOLERANCE_M
            or _axis_error(target_yaw_rad, record.entry_yaw_rad()) > AXIS_TOLERANCE_RAD):
        return "bottom_deck_target_mismatch"
    return None
