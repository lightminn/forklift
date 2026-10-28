"""Mission commands and the mission lifecycle (M-a3, docs/plans/2026-10-16-m-a3-mission-commands.md E1, E2, E4).

Pure: no Isaac, no numpy. The core owns one :class:`MissionCommands`; the runner queues commands (from a schedule
file in a batch run, from the arbiter later) and the core applies the queue at the start of a control tick.

Lifecycle: ``idle -> accepted [-> preparing, live] -> running <-> paused -> resuming -> running``; ``cancel`` goes through
``cancelling`` (standing still is confirmed first) to ``cancelled``; the mission's own end is ``done`` or
``failed``. A finished mission leaves the lifecycle ``idle`` for the next ``submit``.

Work time (E4): a pause stops the work clocks. The start-type comparisons ask :meth:`work_elapsed`, which is
``now - since`` minus the pauses inside that interval -- and exactly ``now - since`` when there has been no pause,
so a run without commands decides bit for bit as before. The pause record lives here, outside every dictionary
the replay compares. A real-time run also stops the work clocks while the core waits for a plan (real-time plan
D8c, docs/plans/2026-10-17-realtime-sim.md): :meth:`begin_wait` / :meth:`end_wait`; a batch run never waits.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# recovering: a cancel stood with the forks in a pocket and the truck backs out (persistent console plan D4)
ACTIVE = ("accepted", "preparing", "running", "paused", "resuming", "held", "cancelling", "recovering")
FINAL = ("done", "failed", "cancelled", "recovery_required")
STOP_KINDS = ("none", "controlled", "preempt")
PRIORITY = {"hold_off": -1, "cancel": 0, "hold_on": 1, "pause": 1, "resume": 2, "submit": 2}


@dataclass(frozen=True)
class Submit:
    pallet_id: str
    destination_id: str
    targets_revision: int = 0
    request_id: str | None = None  # the arbiter's reliable request this came from (M-b B3), kept in the record
    kind: str = field(default="submit", init=False)


@dataclass(frozen=True)
class Pause:
    request_id: str | None = None
    kind: str = field(default="pause", init=False)


@dataclass(frozen=True)
class Resume:
    request_id: str | None = None
    kind: str = field(default="resume", init=False)


@dataclass(frozen=True)
class Cancel:
    mission_id: str | None = None  # None: the active mission
    request_id: str | None = None
    kind: str = field(default="cancel", init=False)


@dataclass(frozen=True)
class HoldOn:
    """The arbiter's TELEOP entry (M-b B6), applied at the tick's start in the core's own time."""

    request_id: str | None = None
    kind: str = field(default="hold_on", init=False)


@dataclass(frozen=True)
class HoldOff:
    """The arbiter leaves TELEOP: held -> paused, before a resume queued in the same tick."""

    request_id: str | None = None
    kind: str = field(default="hold_off", init=False)


@dataclass(frozen=True)
class Context:
    """What a command's acceptance depends on, read by the core at the tick's start."""

    stop_kind: str = "none"
    near_tracker: bool = False  # a near-field tracker exists (pause is refused, E3)
    clear_for_submit: bool = True  # outside a SLAM hold and the near section (else recovery_required)
    # live: what the forks are in refuses a submit (persistent console plan D5a: recovery_required, cargo_held)
    submit_refusal: str | None = None
    scene_damaged: bool = False  # live: the scene's damage is latched (persistent console plan D6; else scene_damaged)
    # live: the SLAM tracker tracks or holds (plan D3; warming_up -- before its scan 4 processed -- localization_lost)
    localization_ok: bool = True
    # live: the pallet on the forks a submit may deliver as it is carried (plan D5 6c; the runner's conditions
    # met), else None -- a cargo_held refusal lets that one submit through as a loaded submit
    loaded_pallet: str | None = None
    # live: the pick's own blocking judgement for a manual re-pick of a put-down pallet (next_pick.pair_blockers on
    # this tick's snapshot): (pallet id, destination id) -> the blockers, empty when the pair is open; None (a batch
    # run) judges nothing more -- the az8 line_m queue sent P-1 back for behind P-2 (2026-10-28)
    repick_blockers: object = None


class MissionCommands:
    def __init__(self, targets=None) -> None:
        self.targets = targets  # M-a4: a submit must name the current targets list (TargetsBook.check_submit)
        self.status = "idle"
        self.recovery = False  # plan D4: the mission is a recovery (kept through a pause and its resume)
        self.mission_id: str | None = None
        self.mission: Submit | None = None
        self.mission_gen = 0
        self._issued = 0
        self._queue: list = []
        self.log: list[dict] = []  # every command with its tick, outcome and reason
        self._detail: str | None = None  # a refusal's detail, set by the handler for its record (blocked: the blockers)
        self.pauses: list[list] = []  # [start, end-or-None] of every pause (the work clock's stops)
        self.waits: list[list] = []  # [start, end-or-None] of every wait for a plan in a real-time run (plan D8c)
        self.paused_since: float | None = None
        self.stop_seen_s: float | None = None  # the first standing tick of the current stop
        self.not_stopped = False  # the stop not seen standing within its limit (E3), for the safety state
        self.resume_check: dict | None = None  # the resume's last check (E5), for the safety state
        self.resume_retry_s = -1e9  # a resume's replan without a path tries again from here
        self.stop_pose: tuple | None = None  # the control pose where the stop was first seen standing (M-a3c R11)
        self.pause_hold_count: int | None = None  # the SLAM hold episode at the stop's first paused tick
        self.pause_holding = False  # SLAM was holding then
        self.pause_release_pending = False  # a SLAM release was due then (R9, R14: it may happen while paused)
        self.end_reason: str | None = None  # why the last mission ended (live: terminate), for its result
        self.end_advice: dict | None = None  # what that end advises the operator (plan D13 item 6), for its result
        self.mission_loaded = False  # plan D5 6c: the mission delivers the pallet already on the forks
        self.end_logged_for: str | None = None  # the mission whose end is in the log (terminate, note_end)
        self.adopted_id: str | None = None  # the mission that ran (start, adopt): only it moves the targets list

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
        if context.stop_kind != "none" and self.status == "preparing":
            # Persistent console plan D2: a stop ends a preparation (nothing has moved; the operator submits again).
            self.terminate("failed", t, "prepare:stopped" if context.stop_kind == "preempt" else "prepare:paused")
            records.append(self.log[-1])
        if context.stop_kind != "none" and self.status in ("running", "resuming", "recovering"):
            # E7: the arbiter's stop is a pause of the mission (resumed only by a resume once it is lifted).
            if self.paused_since is None:
                self._stop_boundary(t)
            else:
                self.mission_gen += 1
            self.status = "paused"
            record = {"t": t, "command": f"stop_kind:{context.stop_kind}", "accepted": True, "reason": None,
                      "mission_id": self.mission_id, "status": self.status}
            records.append(record)
            self.log.append(record)
        for command in commands:
            self._detail = None
            reason = getattr(self, f"_{command.kind}")(t, command, context)
            record = {"t": t, "command": command.kind, "accepted": reason is None, "reason": reason,
                      "mission_id": self.mission_id, "status": self.status}
            if reason is not None and self._detail is not None:
                record["detail"] = self._detail
            if command.request_id is not None:
                record["request_id"] = command.request_id
            records.append(record)
            self.log.append(record)
        return records

    def _submit(self, t, command: Submit, context: Context):
        if self.status in ACTIVE or self.status in FINAL:  # FINAL: ended in this apply, its end collected first
            return "mission_active"
        if context.stop_kind != "none":
            return "stopping"
        if context.scene_damaged:
            return "scene_damaged"
        loaded = False
        if context.submit_refusal is not None:
            # plan D5 6c: the carried pallet itself, its conditions met, goes as a loaded submit
            loaded = (context.submit_refusal == "cargo_held" and context.loaded_pallet is not None
                      and command.pallet_id == context.loaded_pallet)
            if not loaded:
                return context.submit_refusal
        if not context.clear_for_submit:
            return "recovery_required"
        if not context.localization_ok:
            return "localization_lost"
        if self.targets is not None:
            refused = self.targets.check_submit(command.pallet_id, command.destination_id, command.targets_revision,
                                                loaded=loaded)
            if refused is not None:
                return refused
            pallet = self.targets.pallets.get(command.pallet_id)
            if context.repick_blockers is not None and pallet is not None and pallet.state == "delivered":
                blockers = list(context.repick_blockers(command.pallet_id, command.destination_id))
                if blockers:
                    self._detail = ",".join(blockers)
                    return "blocked"
        self._issued += 1
        self.mission_id, self.mission, self.status = f"m{self._issued}", command, "accepted"
        self.mission_loaded = loaded
        self.resume_check = None  # the last mission's resume is not this one's (Codex M-b review round 5 P1)
        return None

    def _pause(self, t, command: Pause, context: Context):
        if self.status == "held":
            return "held"
        if self.status not in ("running", "resuming", "recovering"):
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
        if self.status == "held":
            return "held"
        if self.status != "paused":
            return "not_paused"
        if context.stop_kind != "none":
            return "stopping"
        self.status = "resuming"
        return None

    def _cancel(self, t, command: Cancel, context: Context):
        if self.recovery and self.status in ACTIVE:
            return "recovering"  # a recovery is stopped by a pause or STOP, not cancelled (plan D4)
        if self.status not in ACTIVE or self.status == "cancelling":
            return "not_running"
        if command.mission_id is not None and command.mission_id != self.mission_id:
            return "unknown_mission"
        if self.status in ("accepted", "preparing"):  # nothing has moved yet
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
        self.not_stopped = False
        self.stop_pose = None
        self.pause_hold_count = None
        self.pause_holding = self.pause_release_pending = False
        self.pauses.append([t, None])
        self.mission_gen += 1

    def _hold_on(self, t, command: HoldOn, context: Context):
        return None if self.hold_on(t, record=False) else "not_running"

    def _hold_off(self, t, command: HoldOff, context: Context):
        return None if self.hold_off(t, record=False) else "not_held"

    # --- the arbiter's hold (M-b B6) ------------------------------------------------------------------------------
    def hold_on(self, t: float, *, record: bool = True) -> bool:
        """The operator takes over (TELEOP): an active mission is held -- its work clock stops (one open stop), a
        new generation, a resume's checks dropped. Held is left only by ``hold_off`` (to paused) or a cancel. A
        preparation it ends instead (persistent console plan D2): driven by hand, the mission is submitted again."""
        if self.status == "preparing":
            self.terminate("failed", t, "prepare:teleop")
            return True
        if self.status not in ("accepted", "running", "paused", "resuming"):
            return False
        if self.paused_since is None:
            self._stop_boundary(t)
        else:
            self.mission_gen += 1
        self.status = "held"
        self.resume_check = None
        if record:
            self._record(t, "hold_on")
        return True

    def hold_off(self, t: float, *, record: bool = True) -> bool:
        """The operator hands back: held becomes paused, the stop stays open and a resume is needed (its checks see
        that the truck was moved)."""
        if self.status != "held":
            return False
        self.status = "paused"
        self.mission_gen += 1
        if record:
            self._record(t, "hold_off")
        return True

    def _record(self, t: float, command: str) -> None:
        self.log.append({"t": t, "command": command, "accepted": True, "reason": None, "mission_id": self.mission_id,
                         "status": self.status})

    # --- transitions the core makes ------------------------------------------------------------------------------
    def start(self) -> None:
        """The mission's first tick runs."""
        if self.status == "accepted":
            self.status = "running"
            self.adopted_id = self.mission_id

    def begin_prepare(self) -> None:
        """A live submit's first leg is being planned (persistent console plan D2)."""
        if self.status != "accepted":
            raise RuntimeError(f"begin_prepare from {self.status}")
        self.status = "preparing"

    def adopt(self) -> None:
        """The prepared mission starts: running, a new generation (nothing asked while preparing is taken)."""
        if self.status != "preparing":
            raise RuntimeError(f"adopt from {self.status}")
        self.status = "running"
        self.adopted_id = self.mission_id
        self.mission_gen += 1

    def resumed(self, t: float) -> float:
        """The resume steps passed: running again. Returns the pause's length (deadline-type anchors move by it)."""
        if self.status != "resuming":
            raise RuntimeError(f"resumed from {self.status}")
        length = self._close_pause(t)
        self.status = "recovering" if self.recovery else "running"
        self.mission_gen += 1
        return length

    def refuse_resume(self, t: float, reason: str) -> None:
        """The resume cannot finish (E5 ``no_resume``): back to paused, the stop still open; a new resume is
        needed (or a cancel)."""
        if self.status != "resuming":
            raise RuntimeError(f"refuse_resume from {self.status}")
        self.status = "paused"
        self.log.append({"t": t, "command": "resume", "accepted": False, "reason": reason,
                         "mission_id": self.mission_id, "status": self.status})

    def begin_recovery(self, t: float) -> None:
        """A cancel stood with the forks in a pocket (plan D4): the truck backs out before the mission ends; the
        cancel's stop closes (the recovery's work clock runs)."""
        if self.status != "cancelling":
            raise RuntimeError(f"begin_recovery from {self.status}")
        self._close_pause(t)
        self.status, self.recovery = "recovering", True
        self.mission_gen += 1
        self._record(t, "recovery")

    def recovered(self, t: float) -> None:
        """The recovery backed the forks out: the mission ends cancelled."""
        if self.status not in ("recovering", "resuming"):  # resuming: already out when the resume re-judged it
            raise RuntimeError(f"recovered from {self.status}")
        self._close_pause(t)
        self.status, self.recovery = "cancelled", False

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

    # --- a live run's mission end (docs/plans/2026-10-18-persistent-live-console.md D1) --------------------------
    def close_open(self, t: float) -> None:
        """Close the open pause and the open wait at ``t`` (D1 step 5a); the status does not change."""
        self._close_pause(t)
        self.end_wait(t)

    def terminate(self, outcome: str, t: float, reason: str | None, advice: dict | None = None) -> bool:
        """End the active mission at ``t`` with ``outcome`` and ``reason`` (D1 step 9): the open stops close, a new
        generation (a plan asked before cannot be taken), one log entry. False, nothing changed: no active mission."""
        if outcome not in FINAL:
            raise ValueError(outcome)
        if self.status not in ACTIVE:
            return False
        self.close_open(t)
        self.status, self.end_reason, self.end_logged_for = outcome, reason, self.mission_id
        self.end_advice = advice
        self.recovery = False
        self.mission_gen += 1
        self._record(t, "end")
        self.log[-1]["reason"] = reason
        return True

    def note_end(self, t: float, reason: str | None) -> None:
        """Log the end of a mission the core already ended (a cancel confirmed standing: ``cancelled``)."""
        if self.status not in FINAL:
            raise RuntimeError(f"note_end while {self.status}")
        self.end_reason, self.end_logged_for = reason, self.mission_id
        self.end_advice = None
        self._record(t, "end")
        self.log[-1]["reason"] = reason

    def take_clock(self) -> dict:
        """The finished mission's pauses and waits, handed to its record; the next mission's work clock starts
        clean (D1 step 8)."""
        if self.status in ACTIVE:
            raise RuntimeError(f"take_clock while {self.status}")
        clock = {"pauses": self.pauses, "waits": self.waits}
        self.pauses, self.waits = [], []
        return clock

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
        """Paused, resuming, held or cancelling: the mission's work does not run this tick."""
        return self.status in ("paused", "resuming", "held", "cancelling")

    def begin_wait(self, t: float) -> None:
        """A real-time run starts waiting for a plan at ``t`` (sim s): the work clocks stand still from now."""
        if self.waits and self.waits[-1][1] is None:
            raise ValueError("a wait is already open")
        self.waits.append([float(t), None])

    def end_wait(self, t: float) -> None:
        """The wait ends at ``t``; nothing happens when none is open."""
        if self.waits and self.waits[-1][1] is None:
            self.waits[-1][1] = float(t)

    def work_elapsed(self, since: float, now: float) -> float:
        """``now - since`` without the pauses (and waits) inside that interval; exactly ``now - since`` before any."""
        if self.waits:
            # The union of pauses and waits (a wait may fall inside a pause), clipped to [since, now].
            spans = sorted([(start, now if end is None else end) for start, end in self.pauses + self.waits])
            stopped, reach = 0.0, since
            for start, end in spans:
                lo, hi = max(start, reach), min(end, now)
                if hi > lo:
                    stopped += hi - lo
                    reach = hi
            return (now - since) - stopped
        if not self.pauses:
            return now - since
        paused = 0.0
        for start, end in self.pauses:
            end_ = now if end is None else end
            lo, hi = max(start, since), min(end_, now)
            if hi > lo:
                paused += hi - lo
        return (now - since) - paused
