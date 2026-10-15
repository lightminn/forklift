"""The mission's own state, lifted out of the runner's closures (M-a1 step 0, docs/plans/2026-10-15-m-a-mission-core.md).

The runner kept the phase and its start time as ``run()`` locals that closures rebound with ``nonlocal``;
an object the core and the runner share lets the mission logic move out of ``run()`` one slice at a time.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = ["MissionState"]


@dataclass
class MissionState:
    phase: str | None = None  # None until the runner picks the first phase
    phase_started: float = 0.0  # loop time t at which the phase began
    lift_command: float = 0.0  # the lift joint's position target, ramped by the core (slice 2)
    next_candidate_index: int = 0  # the next observation candidate to try (slice 3c)
