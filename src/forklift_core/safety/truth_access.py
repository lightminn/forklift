"""Safety speed and ground-truth access rules (sim-to-real P0-4 4d-1, docs/design/2026-11-11-p05-truth-audit.md).

``SafetySpeed`` is what a safety decision may read about the truck's speed: the measured signed speed, a
non-negative upper bound on |speed| (with measurement age, error, latency and slip folded in on the real truck --
the formula is 4e-2's), the directions the truck may be moving in, and whether the measurement is valid. In the
simulation it is built from the true speed so every decision keeps its present value (``exact``). A tracking
speed (``StopDetector.tracker_speed``) is never a safety speed (lead decision 2026-11-11, audit 3a).

``TruthAccessViolation`` is raised where the real truck would read simulation truth. It derives from
``BaseException`` so the runner's mission-failure handling (``except Exception``) and ``getattr`` defaults do not
absorb it: it ends the session (the real entry point's stop sequence is 4e-2's).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


class TruthAccessViolation(BaseException):
    """Simulation truth was asked for on the real truck."""


@dataclass(frozen=True)
class SafetySpeed:
    signed_mps: float  # the measured signed speed, vehicle-forward positive
    bound_mps: float  # >= |signed_mps|: the safety upper bound on |speed|
    directions: tuple  # the directions the truck may be moving in: (1,), (-1,) or (1, -1) when unsure
    valid: bool = True
    age_s: float | None = None

    def __post_init__(self) -> None:
        if not (math.isfinite(self.signed_mps) and math.isfinite(self.bound_mps)):
            raise ValueError("a safety speed is finite")
        if self.bound_mps < abs(self.signed_mps):
            raise ValueError("the bound must cover |signed|")
        if self.directions not in ((1,), (-1,), (1, -1)):
            raise ValueError("directions must be (1,), (-1,) or (1, -1)")

    @classmethod
    def exact(cls, speed_mps) -> SafetySpeed:
        """The simulation's: the true speed as both the value and the bound, and today's direction rule
        (``1 if speed > 0 else -1``: a standing truck reads -1)."""
        v = float(speed_mps)
        return cls(v, abs(v), (1,) if v > 0 else (-1,))

    def standing(self, threshold_mps: float) -> bool:
        """A stop confirmation: valid and the bound under the threshold (the simulation: |speed| < threshold)."""
        return self.valid and self.bound_mps < threshold_mps


__all__ = ["SafetySpeed", "TruthAccessViolation"]
