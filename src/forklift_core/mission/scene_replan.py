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

- ``stopped()``, ``busy()``, ``release()``;
- ``submit(extra, clearance_m)`` / ``poll()`` -> None (waiting) | ("ok", result) | ("ok_fail", status) |
  ("dropped", why);
- ``candidate(result)`` / ``preview()`` -> {"status", "ok", "why", "steer_rad", ...} (a copy's preview: a
  failure-free braking previewed one tick further, the leg actually started);
- ``install(result)``;
- ``escape_plan(extra, clearance_m)`` -> path | None, ``escape_install(path)``, ``escape_state()`` ->
  "running" | "done" | "stopped", ``escape_clear()``;
- ``steering_error(target_rad)`` -> measured minus target.
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


@dataclass
class StepResult:
    owned: bool  # the device owns this tick: the follower, stall and arrival rules do not run
    output: str | None = None  # "hold" (brake, steering held) | "presteer" (wheels 0, steering to steer_rad) | "escape"
    steer_rad: float | None = None
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

    # --- the mission's bookkeeping -------------------------------------------------------------------------------
    def reset(self) -> None:
        """A mission's end: the budget starts again (never at a new tracker, phase or pause)."""
        self.used_total, self.used_by_trigger = 0, {}
        self.request_, self.state = None, None

    def shift_time(self, length_s: float) -> None:
        """A pause of ``length_s`` ended: the request's clock does not count it."""
        if self.request_ is not None:
            self.request_["paused_s"] += float(length_s)
            if self.request_.get("presteer_since") is not None:
                self.request_["presteer_since"] += float(length_s)

    def active(self) -> bool:
        return self.request_ is not None

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
                "detail": detail, "outcome": None, "drops": 0, "escape": None, "presteer": None,
                "presteer_error_rad": None, "represteers": 0, "candidate": None, "preview": None, "ignored": [],
                "states": []}
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
                         "target": None, "presteer_since": None, "record": line}
        self._go("wait_stop", t)
        return None

    # --- the tick ------------------------------------------------------------------------------------------------
    def _go(self, state: str, t: float) -> None:
        self.state = state
        if self.request_ is not None:
            self.request_["record"]["states"].append([state, float(t)])

    def _end(self, outcome: str, reason: str | None) -> StepResult:
        self.request_["record"]["outcome"] = outcome
        self.request_, self.state = None, None
        return StepResult(owned=True, output="hold", end=reason)

    def _hand_back(self) -> StepResult:
        self.request_["record"]["outcome"] = "installed"
        self.request_, self.state = None, None
        return StepResult(owned=False)

    def _drop(self, t: float, back_to: str) -> StepResult | None:
        r = self.request_
        r["record"]["drops"] += 1
        if r["record"]["drops"] > self.config.drops_per_request:
            return self._end("failed", f"scene_replan_failed:{r['trigger']}:dropped")
        self._go(back_to, t)
        return None

    def step(self, t: float, ports) -> StepResult:
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
                if ports.busy():
                    return hold
                if not r["released"]:
                    ports.release()
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
                        if path is None:
                            return self._end("failed", f"scene_replan_failed:{trigger}:invalid_start")
                        r["escapes"] += 1
                        ports.escape_install(path)
                        r["record"]["escape"] = "running"
                        self._go("escape", t)
                        return StepResult(owned=True, output="escape")
                    return self._end("failed", f"scene_replan_failed:{trigger}:{value}")
                candidate = ports.candidate(value)
                r["record"]["candidate"] = candidate
                status = candidate.get("status")
                if status in REPLAN_STATUSES:
                    out = self._drop(t, "wait_jobs")
                    return out or hold
                if status in ("arrived", "cusp_pending"):
                    ports.install(value)  # its own rule takes it from here (arrival, the cusp heading hold)
                    return self._hand_back()
                if not candidate.get("ok"):
                    return self._end("refused", f"scene_replan_refused:{trigger}:{candidate.get('why')}")
                ports.install(value)
                r["target"], r["presteer_since"] = float(candidate["steer_rad"]), float(t)
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
            elif state == "presteer":
                error = float(ports.steering_error(r["target"]))
                r["record"]["presteer_error_rad"] = abs(error)
                if abs(error) <= self.config.presteer_tolerance_rad:
                    r["record"]["presteer"] = "aligned"
                elif t - r["presteer_since"] >= self.config.presteer_timeout_s:
                    r["record"]["presteer"] = "timeout"
                else:
                    return StepResult(owned=True, output="presteer", steer_rad=r["target"])
                self._go("resume_check", t)
            elif state == "resume_check":
                if not ports.stopped():
                    self._go("wait_stop", t)
                    return hold
                preview = ports.preview()
                r["record"]["preview"] = preview
                if preview.get("status") in REPLAN_STATUSES or not preview.get("ok"):
                    out = self._drop(t, "wait_jobs")
                    return out or hold
                steer = preview.get("steer_rad")
                if steer is not None and abs(float(steer) - r["target"]) > self.config.presteer_tolerance_rad:
                    if r["record"]["represteers"] >= self.config.represteers_per_request:
                        out = self._drop(t, "wait_jobs")
                        return out or hold
                    r["record"]["represteers"] += 1
                    r["target"], r["presteer_since"] = float(steer), float(t)
                    self._go("presteer", t)
                    continue
                return self._hand_back()
            else:  # pragma: no cover -- a state this module does not set
                raise RuntimeError(f"scene replan in unknown state {state!r}")
        return hold


__all__ = ["SceneReplan", "SceneReplanConfig", "StepResult"]
