"""Mission commands and the mission lifecycle (M-a3, docs/plans/2026-10-16-m-a3-mission-commands.md E1, E2, E4).

Pure: no Isaac, no numpy. The core owns one :class:`MissionCommands`; the runner queues commands (from a schedule
file in a batch run, from the arbiter later) and the core applies the queue at the start of a control tick.

Lifecycle: ``idle -> accepted -> running <-> paused -> resuming -> running``; ``cancel`` goes through
``cancelling`` (standing still is confirmed first) to ``cancelled``; the mission's own end is ``done`` or
``failed``. A finished mission leaves the lifecycle ``idle`` for the next ``submit``.

Work time (E4): a pause stops the work clocks. The start-type comparisons ask :meth:`work_elapsed`, which is
``now - since`` minus the pauses inside that interval -- and exactly ``now - since`` when there has been no pause,
so a run without commands decides bit for bit as before. The pause record lives here, outside every dictionary
the replay compares.
"""

from __future__ import annotations

from dataclasses import dataclass, field

ACTIVE = ("accepted", "running", "paused", "resuming", "cancelling")
FINAL = ("done", "failed", "cancelled")
STOP_KINDS = ("none", "controlled", "preempt")
PRIORITY = {"cancel": 0, "pause": 1, "resume": 2, "submit": 2}


@dataclass(frozen=True)
class Submit:
    pallet_id: str
    destination_id: str
    targets_revision: int = 0
    kind: str = field(default="submit", init=False)


@dataclass(frozen=True)
class Pause:
    kind: str = field(default="pause", init=False)


@dataclass(frozen=True)
class Resume:
    kind: str = field(default="resume", init=False)


@dataclass(frozen=True)
class Cancel:
    mission_id: str | None = None  # None: the active mission
    kind: str = field(default="cancel", init=False)


@dataclass(frozen=True)
class Context:
    """What a command's acceptance depends on, read by the core at the tick's start."""

    stop_kind: str = "none"
    near_tracker: bool = False  # a near-field tracker exists (pause is refused, E3)
    clear_for_submit: bool = True  # empty forks and outside the near section (else recovery_required)


class MissionCommands:
    def __init__(self) -> None:
        self.status = "idle"
        self.mission_id: str | None = None
        self.mission: Submit | None = None
        self.mission_gen = 0
        self._issued = 0
        self._queue: list = []
        self.log: list[dict] = []  # every command with its tick, outcome and reason
        self.pauses: list[list] = []  # [start, end-or-None] of every pause (the work clock's stops)
        self.paused_since: float | None = None
        self.stop_seen_s: float | None = None  # the first standing tick of the current stop
        self.resume_check: dict | None = None  # the resume's last check (E5), for the safety state
        self.resume_retry_s = -1e9  # a resume's replan without a path tries again from here
        self.pause_pose: tuple | None = None  # the control pose at the stop's first paused tick (M-a3c)
        self.pause_hold_count: int | None = None  # the SLAM hold episode then

    # --- the queue ---------------------------------------------------------------------------------------------
    def queue(self, command) -> None:
        self._queue.append(command)

    def apply(self, t: float, context: Context = Context()) -> list[dict]:
        """Apply this tick's queue by priority (cancel > pause > resume, submit), each once; returns its records."""
        if context.stop_kind not in STOP_KINDS:
            raise ValueError(f"unknown stop kind {context.stop_kind!r}")
        if self.status in FINAL:
            self.status, self.mission_id, self.mission = "idle", None, None  # a finished mission frees the slot
        commands = sorted(self._queue, key=lambda c: PRIORITY[c.kind])  # sorted() is stable: same rank keeps order
        self._queue = []
        records = []
        for command in commands:
            reason = getattr(self, f"_{command.kind}")(t, command, context)
            record = {"t": t, "command": command.kind, "accepted": reason is None, "reason": reason,
                      "mission_id": self.mission_id, "status": self.status}
            records.append(record)
            self.log.append(record)
        return records

    def _submit(self, t, command: Submit, context: Context):
        if self.status in ACTIVE:
            return "mission_active"
        if context.stop_kind != "none":
            return "stopping"
        if not context.clear_for_submit:
            return "recovery_required"
        self._issued += 1
        self.mission_id, self.mission, self.status = f"m{self._issued}", command, "accepted"
        return None

    def _pause(self, t, command: Pause, context: Context):
        if self.status not in ("running", "resuming"):
            return "not_running"
        if context.near_tracker:
            return "near_tracking_active"
        if self.paused_since is None:
            self._stop_boundary(t)
        else:  # paused again while resuming: the same stop goes on (one open interval), a new boundary
            self.mission_gen += 1
        self.status = "paused"
        return None

    def _resume(self, t, command: Resume, context: Context):
        if self.status != "paused":
            return "not_paused"
        if context.stop_kind != "none":
            return "stopping"
        self.status = "resuming"
        return None

    def _cancel(self, t, command: Cancel, context: Context):
        if self.status not in ACTIVE or self.status == "cancelling":
            return "not_running"
        if command.mission_id is not None and command.mission_id != self.mission_id:
            return "unknown_mission"
        if self.status == "accepted":  # nothing has moved yet
            self.status = "cancelled"
            self.mission_gen += 1
            return None
        if self.paused_since is None:
            self._stop_boundary(t)
        else:  # cancelled while paused or resuming: the open stop goes on, the cancel is a boundary of its own
            self.mission_gen += 1
        self.status = "cancelling"
        return None

    def _stop_boundary(self, t: float) -> None:
        self.paused_since = t
        self.stop_seen_s = None
        self.pause_pose = None
        self.pause_hold_count = None
        self.pauses.append([t, None])
        self.mission_gen += 1

    # --- transitions the core makes ------------------------------------------------------------------------------
    def start(self) -> None:
        """The mission's first tick runs."""
        if self.status == "accepted":
            self.status = "running"

    def resumed(self, t: float) -> float:
        """The resume steps passed: running again. Returns the pause's length (deadline-type anchors move by it)."""
        if self.status != "resuming":
            raise RuntimeError(f"resumed from {self.status}")
        length = self._close_pause(t)
        self.status = "running"
        self.mission_gen += 1
        return length

    def cancelled(self, t: float) -> None:
        """Standing still confirmed after a cancel."""
        if self.status != "cancelling":
            raise RuntimeError(f"cancelled from {self.status}")
        self._close_pause(t)
        self.status = "cancelled"

    def finish(self, outcome: str, t: float | None) -> None:
        """The mission's own end (``done`` or ``failed``) at loop time ``t``: an open pause closes there, so the
        next mission's work clock does not stay stopped (``t`` None: before any tick -- the pause closes at its own
        start)."""
        if outcome not in ("done", "failed"):
            raise ValueError(outcome)
        if self.status in ACTIVE:
            self._close_pause(self.paused_since if t is None and self.paused_since is not None else t)
            self.status = outcome

    def _close_pause(self, t: float) -> float:
        if self.paused_since is None:
            return 0.0
        self.pauses[-1][1] = t
        length = t - self.paused_since
        self.paused_since = None
        return length

    # --- the work clock ------------------------------------------------------------------------------------------
    @property
    def stopped(self) -> bool:
        """Paused, resuming or cancelling: the mission's work does not run this tick."""
        return self.status in ("paused", "resuming", "cancelling")

    def work_elapsed(self, since: float, now: float) -> float:
        """``now - since`` without the pauses inside that interval; exactly ``now - since`` before any pause."""
        if not self.pauses:
            return now - since
        paused = 0.0
        for start, end in self.pauses:
            end_ = now if end is None else end
            lo, hi = max(start, since), min(end_, now)
            if hi > lo:
                paused += hi - lo
        return (now - since) - paused
