"""The scene-mode replan device (docs/plans/2026-10-23-scene-replan.md 2.1, 2.4).

In a scene layout there is no grid replan: a cusp stall outside its pose bounds ended the mission, a stop the
permission holds stood until a timeout. This device takes one request at a time -- a trigger raises it, nothing
else -- and runs it as a state machine ahead of the follower: wait for the stop, wait for the other jobs and a
pending SLAM release, release the hold, plan the leg from the stop pose, branch on the candidate tracker's status,
check its first gear leg on the permission (in a context of its own), install it, steer the wheels to its first
command standing, check again just before moving, and hand the tick back to the follower. An ``invalid_start``
gets one straight escape prefix, driven under the permission, and the same request continues from where it stops.

The world is behind ports (a namespace the core builds each tick), so this module holds the state, the limits,
the budget and the records only:

- ``stopped()``, ``busy()``, ``release_hold()``, ``snapshot_id()`` (changes with each new processed snapshot);
- ``submit(extra, clearance_m)`` / ``poll()`` -> None (waiting) | ("ok", result) | ("ok_fail", status) |
  ("dropped", why);
- ``candidate(result)`` / ``preview()`` -> {"status", "ok", "why", "steer", ...} (re-asked every tick while the
  permission does not pass it yet, up to ``permit_wait_s``; a copy's preview: a
  failure-free braking previewed one tick further, the leg actually started; ``steer`` the first command's steering
  target, opaque here);
- ``install(result)``;
- ``escape_plan(extra, clearance_m[, presteered=True])`` -> path | None, ``escape_install(path)``, ``escape_state()``
  -> "running" | "done" | "stopped", ``escape_clear()``;
- with ``escape_presteer`` on (docs/plans/2026-10-27-escape-presteer.md D2): ``escape_probe(extra, clearance_m,
  kappa)`` -> None | {"ok", "end_clearance_m", ...} (the escape evaluated as if the steering held kappa, nothing
  installed or recorded) and ``correction_version()``;
- ``steering_error(target)`` -> the largest wheel's gap to it (rad); ``steer_gap(a, b)`` -> the largest wheel's
  gap between two targets;
- ``resumable()`` (optional) -> whether the follower's own tracker still follows from the pose after the release (a
  standing-safe request ending before an install hands back only then);
- ``abandon()`` (optional) -> drops the request's plan still out and its answer, at every end.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SceneReplanConfig:
    budget_total: int = 4  # requests a mission may take, all triggers together
    budget_per_trigger: int = 3  # and per trigger ("v-g2" session, 2026-10-23)
    drops_per_request: int = 3  # dropped or refused-and-replanned answers one request may take
    request_limit_s: float = 60.0  # sim time from the request to the hand-back, plan waits included, pauses not
    presteer_tolerance_rad: float = 0.02
    presteer_timeout_s: float = 1.0
    represteers_per_request: int = 2
    escapes_per_request: int = 1
    permit_wait_s: float = 3.0  # 7판: a candidate the permission does not pass yet is re-checked standing this long
    # P6 (permission-clearance plan, Isaac p6_24f83a14): a trigger whose stop may clear on its own -- a request of it
    # that ends before anything was installed hands the tick back standing instead of ending the mission, and gives
    # its share of the mission's total back (the follower's own trigger budget is not spent on it).
    standing_safe_triggers: tuple = ("permit_stop",)
    # docs/plans/2026-10-27-escape-presteer.md D2: no escape qualifying at the held steering, the steering is set to the
    # side whose escape qualifies (a probe at each of these curvatures) and the escape is evaluated again at the
    # measured steering before it is installed
    escape_presteer: bool = False
    escape_presteer_kappas: tuple = (0.05, -0.05)


@dataclass
class StepResult:
    owned: bool  # the device owns this tick: the follower, stall and arrival rules do not run
    output: str | None = None  # "hold" (brake, steering held) | "presteer" (wheels 0, steering to steer) | "escape"
    steer: object = None
    end: str | None = None  # the mission ends with this reason


REPLAN_STATUSES = ("failed", "off_path")


class SceneReplan:
    def __init__(self, records: list, config: SceneReplanConfig = SceneReplanConfig()):
        self.records = records
        self.config = config
        self.used_total = 0
        self.used_by_trigger: dict = {}
        self.request_: dict | None = None
        self.state: str | None = None
        self._ports = None

    # --- the mission's bookkeeping -------------------------------------------------------------------------------
    def reset(self) -> None:
        """A mission's end: the budget starts again (never at a new tracker, phase or pause)."""
        self.used_total, self.used_by_trigger = 0, {}
        self.request_, self.state = None, None

    def shift_time(self, length_s: float) -> None:
        """A pause of ``length_s`` ended: the request's clock does not count it."""
        if self.request_ is not None:
            self.request_["paused_s"] += float(length_s)
            for key_ in ("presteer_since", "permit_since", "resume_since"):
                if self.request_.get(key_) is not None:
                    self.request_[key_] += float(length_s)

    def active(self) -> bool:
        return self.request_ is not None

    def abort(self, reason: str) -> None:
        """The request ends outside the machine (a worker's exception): its record closed with the reason."""
        if self.request_ is not None:
            self.request_["record"].update(outcome="failed", reason=reason)
            self.request_, self.state = None, None

    def _left(self, trigger: str) -> dict:
        return {"total": self.config.budget_total - self.used_total,
                trigger: self.config.budget_per_trigger - self.used_by_trigger.get(trigger, 0)}

    # --- a trigger -------------------------------------------------------------------------------------------
    def request(self, t: float, trigger: str, *, phase: str, tracker_serial, leg, rear, truth_rear, deviation=None,
                correction_jump=None, extra_obstacles_fn=None, clearance_m=None, detail=None) -> str | None:
        """Raise a request: None when it is taken (or noted because one runs), the end reason when the budget is
        spent. The tick that raises it hands the rest of itself to the device (the caller skips the follower)."""
        if self.request_ is not None:
            self.request_["record"]["ignored"].append(trigger)
            return None
        line = {"time_s": float(t), "trigger": trigger, "phase": phase, "tracker_serial": tracker_serial, "leg": leg,
                "rear": [float(v) for v in rear], "truth_rear": None if truth_rear is None else [float(v) for v in truth_rear],
                "deviation": deviation, "correction_jump": correction_jump, "clearance_m": clearance_m,
                "detail": detail, "outcome": None, "drops": 0, "escape": None, "presteer": None, "permit_waits": [],
                "presteer_error_rad": None, "represteers": 0, "candidate": None, "preview": None, "ignored": [],
                "states": []}
        if self.config.escape_presteer:
            line["escape_presteer"] = None  # the key only with the feature on: an off run's record is as before
        if (self.used_total >= self.config.budget_total
                or self.used_by_trigger.get(trigger, 0) >= self.config.budget_per_trigger):
            line.update(outcome="exhausted", budget_left=self._left(trigger))
            self.records.append(line)
            return f"scene_replan_exhausted:{trigger}"
        self.used_total += 1
        self.used_by_trigger[trigger] = self.used_by_trigger.get(trigger, 0) + 1
        line["budget_left"] = self._left(trigger)
        self.records.append(line)
        self.request_ = {"t0": float(t), "paused_s": 0.0, "trigger": trigger, "extra_fn": extra_obstacles_fn,
                         "clearance_m": clearance_m, "released": False, "escapes": 0, "extra": (),
                         "target": None, "presteer_since": None, "permit_since": None, "resume_since": None,
                         "result": None, "seen": None, "stretch": 0, "record": line, "changed": False}
        self._go("wait_stop", t)
        return None

    # --- the tick ------------------------------------------------------------------------------------------------
    def _go(self, state: str, t: float) -> None:
        self.state = state
        if self.request_ is not None:
            self.request_["record"]["states"].append([state, float(t)])

    def _end(self, outcome: str, reason: str | None) -> StepResult:
        r = self.request_
        r["record"]["outcome"] = outcome
        abandon_ = getattr(self._ports, "abandon", None)
        if abandon_ is not None:
            abandon_()  # a plan still out (the time limit in wait_plan) never answers a later request
        resumable_ = getattr(self._ports, "resumable", None)
        if r["trigger"] in self.config.standing_safe_triggers and not r["changed"] and (
                resumable_ is None or resumable_()):
            # Nothing installed or driven: the follower's own tracker and path stand as before the request, so the
            # truck goes back to standing under the permission (a stop that clears goes on) -- never worse than
            # without the trigger. The mission's total is given back.
            r["record"].update(mission_end=False, reason=reason)
            self.used_total = max(0, self.used_total - 1)
            self.request_, self.state = None, None
            return StepResult(owned=False)
        if r["trigger"] in self.config.standing_safe_triggers and not r["changed"]:
            # nothing installed, but the hold release moved the pose beyond what the follower's tracker can follow
            r["record"]["not_resumable"] = True
        r["record"]["mission_end"] = True
        self.request_, self.state = None, None
        return StepResult(owned=True, output="hold", end=reason)

    def _hand_back(self) -> StepResult:
        self.request_["record"]["outcome"] = "installed"
        self.request_, self.state = None, None
        return StepResult(owned=False)

    def _note_wait(self, t: float, where: str, why) -> None:
        """A standing re-check's record: one entry per stretch and reason (its first and last tick)."""
        waits_, stretch_ = self.request_["record"]["permit_waits"], self.request_["stretch"]
        if waits_ and waits_[-1]["stretch"] == stretch_ and waits_[-1]["why"] == why:
            waits_[-1]["until_s"] = float(t)  # the same stretch, not one after a replan (round 5 P3)
        else:
            waits_.append({"where": where, "why": why, "since_s": float(t), "until_s": float(t), "stretch": stretch_})

    def _drop(self, t: float, back_to: str) -> StepResult | None:
        r = self.request_
        r["record"]["drops"] += 1
        if r["record"]["drops"] > self.config.drops_per_request:
            return self._end("failed", f"scene_replan_failed:{r['trigger']}:dropped")
        self._go(back_to, t)
        return None

    def _probe(self, t: float, ports):
        """D2-2: the side to steer to before the escape -- each curvature's probe (the escape at that steering, its install
        permit included); of those that qualify, the one whose chosen escape ends with the most clearance. None: no
        side qualifies."""
        r = self.request_
        probes = []
        for kappa in self.config.escape_presteer_kappas:
            answer = ports.escape_probe(r["extra"], r["clearance_m"], float(kappa))
            probes.append({"kappa": float(kappa), "answer": answer})
        ok = [p for p in probes if p["answer"] is not None and p["answer"].get("ok")]
        best = max(ok, key=lambda p: float(p["answer"].get("end_clearance_m") or 0.0)) if ok else None
        earlier = r["record"].get("escape_presteer")
        r["record"]["escape_presteer"] = {
            "earlier": ([] if earlier is None else earlier.pop("earlier", []) + [earlier]), "probes": probes, "kappa": None if best is None else best["kappa"], "presteer": None, "error_rad": None,
            "version_changed": False, "installed": False, "since_s": float(t)}
        if best is None:
            return None
        r["target"], r["presteer_since"] = best["kappa"], float(t)
        r["probe_version"] = ports.correction_version()
        box_ = getattr(ports, "box", None)
        rec_ = r["record"]["escape_presteer"]
        rec_["rear_start"] = None if box_ is None else [float(v) for v in box_.rear]
        return best["kappa"]

    def _escape_presteer(self, t: float, ports) -> StepResult | None:
        """D2-3, D2-4: steer standing to the probe's side, then the escape evaluated again at the measured steering
        (installed only when it qualifies there); a stop lost goes back to wait_stop, a correction change back to
        plan (the trigger's rectangles read again). None: the state changed and the tick goes on."""
        r = self.request_
        rec = r["record"]["escape_presteer"]
        box_ = getattr(ports, "box", None)
        if box_ is not None and rec.get("rear_start") is not None:
            rec["rear_end"] = [float(v) for v in box_.rear]  # D2-5: the presteer's stand, also when it ends early
        if rec.get("version_changed"):
            # Codex impl round 6 P2: the grid must follow the correction before the trigger's rectangles are read again
            # (the runner re-projects it after the device's tick); held until it has, the request's limit bounding it
            # -- and standing (Codex impl round 7 P2): a plan is submitted only from a stop, so a stop lost while it
            # waits holds here as well (wait_stop would read the trigger's rectangles before the grid follows)
            if getattr(ports, "grid_current", lambda: True)() and ports.stopped():
                self._go("plan", t)
                return None
            return StepResult(owned=True, output="hold")
        if not ports.stopped():
            self._go("wait_stop", t)  # nothing installed, the escape budget unspent (agy review High 1)
            return StepResult(owned=True, output="hold")
        error = abs(float(ports.steering_error(r["target"])))
        rec["error_rad"] = error
        if error <= self.config.presteer_tolerance_rad:
            rec["presteer"] = "aligned"
        elif t - r["presteer_since"] >= self.config.presteer_timeout_s:
            rec["presteer"] = "timeout"
        else:
            return StepResult(owned=True, output="presteer", steer=r["target"])
        if ports.correction_version() != r.get("probe_version"):
            # the grid moved under the trigger's rectangles: plan again rather than mix them with the new grid
            # (Codex plan round 2 P2-2) -- once the grid has followed the correction (above)
            rec["version_changed"] = True
            return self._escape_presteer(t, ports)
        path = ports.escape_plan(r["extra"], r["clearance_m"], presteered=True)
        if path is None:
            return self._end("failed", f"scene_replan_failed:{r['trigger']}:invalid_start")
        rec["installed"] = True
        r["escapes"] += 1
        r["changed"] = True
        ports.escape_install(path)
        r["record"]["escape"] = "running"
        self._go("escape", t)
        return StepResult(owned=True, output="escape")

    def step(self, t: float, ports) -> StepResult:
        self._ports = ports  # the end of a standing-safe request asks whether the follower can resume
        r = self.request_
        if r is None:
            return StepResult(owned=False)
        trigger = r["trigger"]
        if t - r["t0"] - r["paused_s"] > self.config.request_limit_s:
            return self._end("failed", f"scene_replan_failed:{trigger}:timeout")
        hold = StepResult(owned=True, output="hold")
        for _ in range(8):  # the states a tick may pass through without acting
            state = self.state
            if state == "wait_stop":
                if not ports.stopped():
                    return hold
                self._go("wait_jobs", t)
            elif state == "wait_jobs":
                if not ports.stopped():  # every submit stands (Codex impl round 1 P2-5)
                    self._go("wait_stop", t)
                    return hold
                if ports.busy():
                    return hold
                if not r["released"]:
                    ports.release_hold()
                    r["released"] = True
                self._go("plan", t)
            elif state == "plan":
                try:
                    r["extra"] = tuple(r["extra_fn"]()) if r["extra_fn"] is not None else ()
                except Exception as exc:  # noqa: BLE001 -- the trigger's obstacles failed: no plan without them
                    r["record"]["obstacles_error"] = f"{type(exc).__name__}: {exc}"
                    return self._end("failed", f"scene_replan_failed:{trigger}:obstacles_error")
                ports.submit(r["extra"], r["clearance_m"])
                self._go("wait_plan", t)
                return hold
            elif state == "wait_plan":
                answer = ports.poll()
                if answer is None:
                    return hold
                kind, value = answer
                if kind == "dropped":
                    out = self._drop(t, "wait_stop" if value == "moving" else "wait_jobs")
                    return out or hold
                if kind == "ok_fail":
                    r["record"]["plan_status"] = value
                    if value == "invalid_start" and r["escapes"] < self.config.escapes_per_request:
                        path = ports.escape_plan(r["extra"], r["clearance_m"])
                        if path is None and self.config.escape_presteer:
                            if self._probe(t, ports) is not None:
                                self._go("escape_presteer", t)
                                continue  # its own stop check runs before any steering (Codex impl round 5 P2-1)
                        if path is None:
                            return self._end("failed", f"scene_replan_failed:{trigger}:invalid_start")
                        r["escapes"] += 1
                        r["changed"] = True
                        ports.escape_install(path)
                        r["record"]["escape"] = "running"
                        self._go("escape", t)
                        return StepResult(owned=True, output="escape")
                    return self._end("failed", f"scene_replan_failed:{trigger}:{value}")
                r["result"], r["permit_since"], r["seen"] = value, float(t), None
                r["stretch"] += 1
                self._go("candidate", t)
            elif state == "candidate":
                if not ports.stopped():  # the stop lost while it waits: nothing installed or steered (round 5 P2-2)
                    self._go("wait_stop", t)
                    return hold
                value = r["result"]
                seen_ = ports.snapshot_id()
                if r["seen"] is not None and seen_ == r["seen"]:
                    # no new snapshot since the last check (round 5 P3): only the time
                    why_ = r["record"]["candidate"].get("why")
                    self._note_wait(t, "candidate", why_)  # the wait's record runs on (Codex impl round 6 P3)
                    if t - r["permit_since"] < self.config.permit_wait_s:
                        return hold
                    return self._end("refused", f"scene_replan_refused:{trigger}:{why_}")
                r["seen"] = seen_
                candidate = ports.candidate(value)
                r["record"]["candidate"] = candidate
                status = candidate.get("status")
                if status in REPLAN_STATUSES:
                    out = self._drop(t, "wait_jobs")
                    return out or hold
                if status in ("arrived", "cusp_pending"):
                    r["changed"] = True
                    ports.install(value)  # its own rule takes it from here (arrival, the cusp heading hold)
                    return self._hand_back()
                if not candidate.get("ok"):
                    # 7판: re-checked standing on every tick (new scans fill the grid in) for permit_wait_s, then refused.
                    if t - r["permit_since"] < self.config.permit_wait_s:
                        self._note_wait(t, "candidate", candidate.get("why"))
                        return hold
                    return self._end("refused", f"scene_replan_refused:{trigger}:{candidate.get('why')}")
                r["changed"] = True
                ports.install(value)
                r["target"], r["presteer_since"] = candidate.get("steer"), float(t)
                self._go("presteer", t)
            elif state == "escape":
                escape = ports.escape_state()
                if escape == "running":
                    return StepResult(owned=True, output="escape")
                if escape == "stopped":
                    r["record"]["escape"] = "stopped"
                    return self._end("failed", f"scene_replan_failed:{trigger}:escape_stopped")
                ports.escape_clear()
                r["record"]["escape"] = "done"
                self._go("wait_stop", t)
                return hold
            elif state == "escape_presteer":
                out = self._escape_presteer(t, ports)
                if out is not None:
                    return out
            elif state == "presteer":
                error = abs(float(ports.steering_error(r["target"])))
                r["record"]["presteer_error_rad"] = error
                if error <= self.config.presteer_tolerance_rad:
                    r["record"]["presteer"] = "aligned"
                elif t - r["presteer_since"] >= self.config.presteer_timeout_s:
                    r["record"]["presteer"] = "timeout"
                else:
                    return StepResult(owned=True, output="presteer", steer=r["target"])
                self._go("resume_check", t)
            elif state == "resume_check":
                if not ports.stopped():
                    r["resume_since"] = None  # a wait ends with the state it waited in (round 5 P2-3)
                    self._go("wait_stop", t)
                    return hold
                if r["resume_since"] is not None and ports.snapshot_id() == r["seen"]:
                    self._note_wait(t, "resume", (r["record"]["preview"] or {}).get("why"))
                    if t - r["resume_since"] < self.config.permit_wait_s:
                        return hold  # no new snapshot: only the time (round 5 P3)
                    r["resume_since"] = None
                    out = self._drop(t, "wait_jobs")
                    return out or hold
                r["seen"] = ports.snapshot_id()
                preview = ports.preview()
                r["record"]["preview"] = preview
                if preview.get("status") in REPLAN_STATUSES:
                    r["resume_since"] = None
                    out = self._drop(t, "wait_jobs")
                    return out or hold
                if not preview.get("ok"):
                    # 7판 (Codex Isaac round 2): the same standing re-check as the candidate's, then planned again.
                    if r["resume_since"] is None:
                        r["resume_since"] = float(t)
                        r["stretch"] += 1
                    if t - r["resume_since"] < self.config.permit_wait_s:
                        self._note_wait(t, "resume", preview.get("why"))
                        return hold
                    r["resume_since"] = None
                    out = self._drop(t, "wait_jobs")
                    return out or hold
                r["resume_since"] = None
                steer = preview.get("steer")
                if steer is not None and ports.steer_gap(steer, r["target"]) > self.config.presteer_tolerance_rad:
                    if r["record"]["represteers"] >= self.config.represteers_per_request:
                        r["resume_since"] = None
                        out = self._drop(t, "wait_jobs")
                        return out or hold
                    r["record"]["represteers"] += 1
                    r["target"], r["presteer_since"] = steer, float(t)
                    self._go("presteer", t)
                    continue
                return self._hand_back()
            else:  # pragma: no cover -- a state this module does not set
                raise RuntimeError(f"scene replan in unknown state {state!r}")
        return hold


__all__ = ["SceneReplan", "SceneReplanConfig", "StepResult"]
