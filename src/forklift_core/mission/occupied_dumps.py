"""When to write the diagnostic dumps of ``--occupied-dumps`` (docs/validation/2026-11-09-live-riib-v2-lifecycle4-
confirmation.md 2): a permission hold on an occupied or unknown cell is dumped at registered ages of the hold, not only
at its first ticks (the run's first-four dumps fell within 0.03 s of one hold), and a scene-replan candidate refused for
a blocked reason is dumped once per refusal (a refusal left nothing to place the blocking cells). Plain data; the
mission core writes the files."""

from __future__ import annotations

HOLD_AGES_S = (1.0, 5.0, 20.0, 60.0)
MAX_HOLD_DUMPS = 16  # a run's cap
MAX_CANDIDATE_DUMPS = 8


class HoldDumps:
    """One run's hold dumps: ``tick`` answers the age to dump at this tick (one age a tick, each once an episode), or
    None. An episode is a run of held ticks; a tick not held ends it."""

    def __init__(self, ages_s=HOLD_AGES_S, max_dumps: int = MAX_HOLD_DUMPS) -> None:
        self.ages_s, self.max_dumps = tuple(float(a) for a in ages_s), int(max_dumps)
        self.since: float | None = None
        self.done: set = set()
        self.count = 0
        self.episode = 0

    def new_mission(self) -> None:
        """A mission's start: no hold episode carries over (the run's count stays -- the cap is the run's)."""
        self.since, self.done = None, set()

    def tick(self, t: float, held: bool) -> float | None:
        if not held:
            self.since, self.done = None, set()
            return None
        if self.since is None:
            self.since, self.done = float(t), set()
            self.episode += 1
        age = float(t) - self.since
        for a in self.ages_s:
            if age >= a and a not in self.done and self.count < self.max_dumps:
                self.done.add(a)
                self.count += 1
                return a
        return None


def blocked_candidate(permit) -> bool:
    """A candidate permit refused for a blocked reason (``blocked:occupied``, ``blocked:unknown``)."""
    return bool(permit) and not permit.get("ok") and str(permit.get("why") or "").startswith("blocked:")
