"""When to write the diagnostic dumps of ``--occupied-dumps`` (docs/validation/2026-11-09-live-riib-v2-lifecycle4-
confirmation.md 2): a permission hold on an occupied or unknown cell at its start, at registered ages and on its release
(not only at the run's first ticks -- the run's first-four dumps fell within 0.03 s of one hold); a scene-replan or
escape candidate refused for a blocked reason, at once and then at most once a second per request (the final refusal of
a 3 s re-check is then within a second of a dump); the grid at a mission's start and at the transport's entry. The
budget is a mission's, under a run's cap. Plain data; the mission core writes the files."""

from __future__ import annotations

HOLD_AGES_S = (0.0, 1.0, 5.0, 20.0, 60.0)
RELEASE_MIN_S = 1.0  # a hold this long is dumped on its release too
REFUSAL_EVERY_S = 1.0
PER_MISSION = 24
PER_RUN = 96


class HoldDumps:
    """One run's hold episodes: ``tick`` answers the age to dump at this tick (one age a tick, each once an episode),
    or None; an episode is a run of held ticks, a tick not held ends it -- ``release_due`` then says whether its
    release is to be dumped (held at least RELEASE_MIN_S)."""

    def __init__(self, ages_s=HOLD_AGES_S, max_dumps: int | None = None) -> None:
        self.ages_s = tuple(float(a) for a in ages_s)
        self.max_dumps = None if max_dumps is None else int(max_dumps)
        self.since: float | None = None
        self.last_held: float | None = None
        self.done: set = set()
        self.count = 0
        self.episode = 0
        self.release_due = False
        self.released_after_s: float | None = None

    def new_mission(self) -> None:
        """A mission's start: no hold episode carries over (the run's count stays)."""
        self.since, self.last_held, self.done, self.release_due = None, None, set(), False

    def release_taken(self) -> None:
        self.release_due = False

    def tick(self, t: float, held: bool) -> float | None:
        if not held:
            if self.since is not None:
                self.released_after_s = float(self.last_held) - self.since
                self.release_due = self.released_after_s >= RELEASE_MIN_S
            self.since, self.last_held, self.done = None, None, set()
            return None
        if self.since is None:
            self.since, self.done, self.release_due = float(t), set(), False
            self.episode += 1
        self.last_held = float(t)
        age = float(t) - self.since
        for a in self.ages_s:
            if age >= a and a not in self.done and (self.max_dumps is None or self.count < self.max_dumps):
                self.done.add(a)
                self.count += 1
                return a
        return None


class DumpBudget:
    """How many dump files a mission may write, under the run's cap; ``drops`` counts the refused ones."""

    def __init__(self, per_mission: int = PER_MISSION, per_run: int = PER_RUN) -> None:
        self.per_mission, self.per_run = int(per_mission), int(per_run)
        self.mission = self.run = self.drops = 0

    def new_mission(self) -> None:
        self.mission = 0

    def take(self) -> bool:
        if self.mission >= self.per_mission or self.run >= self.per_run:
            self.drops += 1
            return False
        self.mission += 1
        self.run += 1
        return True


class RefusalPacer:
    """A request's refusals: the first at once, then at most once every ``every_s``."""

    def __init__(self, every_s: float = REFUSAL_EVERY_S) -> None:
        self.every_s = float(every_s)
        self.last: dict = {}

    def due(self, request, t: float) -> bool:
        last = self.last.get(request)
        if last is not None and float(t) - last < self.every_s:
            return False
        self.last[request] = float(t)
        return True


def blocked_candidate(permit) -> bool:
    """A candidate permit refused for a blocked reason (``blocked:occupied``, ``blocked:unknown``)."""
    return bool(permit) and not permit.get("ok") and str(permit.get("why") or "").startswith("blocked:")
