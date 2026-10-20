"""The live runner's runtime (M-c plan C2, C4): the arbiter, its link and the console server inside a running
simulation, the pacing of its physics steps, and the media it publishes.

Isaac-free: the runner hands in what it reads and does as plain callables (zero the wheels, hold the lift, read the
camera, the grid, the scan), so the runtime can be tested on the CPU with fakes. The runner calls, per control
tick, ``tick_start`` (the arbiter's cycle start and the mission's commands) and ``tick_end`` (the output), and, per
physics step, ``before_step`` (pacing and the stop gate) and ``after_step`` (feedback, the step's clock, media).
"""

from __future__ import annotations

import json
import math
import os
import secrets
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable

import numpy as np

from console.live.server import LiveServer, MediaStore, camera_item, grid_item, scan_item, write_port_file
from console.vehicle_outline import DLS08_MEASURED_CHASSIS
from forklift_core.live.io_writer import OrderedWriter, encode_in_pieces, write_pieces_atomic
from forklift_core.live.link import Channels, Link
from forklift_core.live.pacing import Pacer
from forklift_core.mission.commands import FINAL, Context
from forklift_core.mission.results import ResultsBook
from forklift_core.perf import TIMERS
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback
from forklift_core.safety.state import state_message


@dataclass
class Hooks:
    """What the runner does and reads for the runtime (all called on the control thread)."""

    zero_wheels: Callable[[], None]
    hold_lift: Callable[[float], None]
    lift_position: Callable[[], float]
    camera: Callable[[], tuple]  # -> (rgb uint8 HxWx3 or None, rendering sim time or None)
    grid: Callable[[], dict | None]  # -> grid_item kwargs or None
    scan: Callable[[], dict | None]  # -> scan_item kwargs or None


@dataclass
class EndHooks:
    """What the runner does at a mission's end (docs/plans/2026-10-18-persistent-live-console.md D1), in this order:
    stop (step 1), release the wheel lock (step 5), drain the trackers' leg switches into the record (step 5a) and,
    for a completed mission, judge the delivery on the truth (step 6: ``delivery_verdict``)."""

    stop: Callable[[], None] = lambda: None
    unlock_wheels: Callable[[], None] = lambda: None
    drain_trackers: Callable[[], None] = lambda: None
    judge_delivery: Callable[[], object] | None = None


class LiveRuntime:
    def __init__(self, core, hooks: Hooks, *, output: Path, port_file: Path | None, dt: float, factor: float = 1.0,
                 camera_hz: float = 10.0, host: str = "127.0.0.1", web_root: Path | None = None,
                 config: ArbiterConfig | None = None, clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep, session_epoch: int | None = None,
                 test_step_delay_s: float = 0.0, camera_freeze: tuple | None = None,
                 test_stop_in_capture: bool = False, camera_pixel_lag_s: float = 0.0,
                 camera_model: dict | None = None, catchup_steps: int = 0, end: EndHooks | None = None,
                 port: int = 0, supervised: bool = False, log_rotate_bytes: int = 200_000_000, log_keep: int = 3,
                 trace=None):
        self.core, self.hooks, self.clock, self.sleep = core, hooks, clock, sleep
        self.trace = trace  # real-time plan 5.12 D12a: a TickTrace on this clock, or None (nothing recorded)
        self.end = end or EndHooks()
        self.results = ResultsBook()  # the session's mission outcomes (persistent console plan D1 step 9)
        self._ended_id: str | None = None  # the mission end_mission last ended (each ends once)
        # Plan D1 step 7 (U29): one writer does the session's file writes -- the log's lines, each mission's files --
        # in order; the control thread only hands them over (it never waits on the disk).
        self.io = OrderedWriter(max_keyed=MISSION_FILES_PENDING)
        self._unsettled: set = set()  # the results marked unsettled at a settle deadline (their marks stay)
        self._unlisted_errors: set = set()  # results with a record error that are out of the book (the log has them)
        self.io_budget_s = 30.0  # the session end's whole wait on the writer (settles and close share it)
        self._io_deadline: float | None = None
        self.scene = {"damaged": None}  # the runner latches the scene's damage here (plan D6); in every state
        self.slam_info: dict | None = None  # SLAM localization plan L6: set by the runner for a localization session
        self.dt = dt
        self.session_epoch = session_epoch if session_epoch is not None else secrets.randbits(48)  # a job, an epoch
        # Reliable requests are judged by the request ledger here: a session lives long (persistent console plan D7).
        self.arbiter = Arbiter(self.session_epoch, replace(config or ArbiterConfig(tick_s=dt), request_tokens=True),
                               lambda: secrets.token_hex(16), mission=core.commands)
        self.channels = Channels()
        self.link = Link(self.arbiter, self.channels)
        self.store = MediaStore()
        self.pacer = Pacer(factor, dt, catchup_steps=catchup_steps)  # real-time plan D7
        self.plan_feed = PlanFeed()  # the followed path, for the page's map (display only)
        # Frames by sim time (plan C4a): the video limit is scaled by the declared factor like the command age, so
        # a sim-rate camera keeps the same freshness in sim terms; frames by wall time made a slower run render more
        # often and run slower still (2026-10-17).
        self.camera_every = max(1, round(1.0 / (camera_hz * dt)))
        self.welcome = {"t": "welcome", "api": "console/1", "vehicle_id": "isaac-forklift",
                        "session_epoch": self.session_epoch, "backend": "isaac", "capabilities": {
                            "control": True, "mission": True, "teleop_direct": True, "teleop_video": True,
                            "lift": True, "recovery_actions": [],  # plan D14: the manual lift is driven
                            # a supervisor restarts the runner on a new scene (persistent console plan D9)
                            "sim_reset": bool(supervised),
                            "feedback": {"rear_axle_speed": True, "steering": True, "lift_height": True,
                                         "load_state": False},
                            "media": {"camera_rgb": {"max_hz": camera_hz}, "scan": {"max_hz": 10},
                                      "grid": {"max_hz": 1}},
                            "chassis": dict(DLS08_MEASURED_CHASSIS),  # display only (C3)
                            "provisional_limits": True}}
        if camera_model is not None:
            self.welcome["capabilities"]["camera_model"] = camera_model  # display only (contract 3)
        self.arbiter.sim_reset_allowed = bool(supervised)
        self.server = LiveServer(self.channels, self.store, self.welcome, host=host, port=port, web_root=web_root)
        self.output, self.port_file = Path(output), port_file
        self._log = open(self.output / "live_log.jsonl", "w")  # noqa: SIM115 -- closed in close()
        # Persistent console plan D8: the log rolls over at log_rotate_bytes into live_log.jsonl.1 (the newest) ...
        # live_log.jsonl.<log_keep>; older ones go.
        self._log_rotate_bytes, self._log_keep, self._log_bytes = int(log_rotate_bytes), int(log_keep), 0
        self._log_rotations = 0
        # Plan U29: a reader of this log expects its end summary (log_end) and refuses it without (read_live_log).
        self._write_log(json.dumps({"log_writer": "ordered"}) + "\n")
        self.feedback: Feedback | None = None
        self.step = 0
        self.step_start: deque = deque(maxlen=600)  # (sim time at the step's end, mono at its start)
        self._start_mono = None
        self.stopped_in_tick = False  # the gate stopped the wheels inside this control tick (plan C2)
        self._lift_latch: float | None = None  # the lift held since the first preempted step
        self.loaded_hold_tolerance_m: float | None = None  # loaded: the loaded sag plus a margin (the runner sets it)
        self.frame_id = 0
        self._last_render_time = None
        self._targets_revision = None
        self.test_step_delay_s = test_step_delay_s
        self.camera_freeze = camera_freeze  # (from, to) sim seconds: no new frames (the video-stall test)
        self.capturing = False  # the runner marks a capture in progress
        self._test_stop_in_capture = test_stop_in_capture  # tests only: one STOP at the first capture's steps
        self.camera_latency = LatencyWindow(maxlen=LATENCY_WINDOW)
        self.camera_pixel_lag_s = camera_pixel_lag_s  # sim s the pixels trail the render metadata (plan C5)
        self.camera_pixel_latency = LatencyWindow(maxlen=LATENCY_WINDOW)
        self.camera_pixel_unknown = 0
        self.trace_saved: dict | None = None
        self._next_status = 0.0

    # --- lifecycle -------------------------------------------------------------------------------------------------
    def start(self) -> int:
        port = self.server.start()
        if self.port_file is not None:
            write_port_file(self.port_file, {"host": self.server.host, "port": port,
                                             "session_epoch": self.session_epoch, "pid": os.getpid(),
                                             "started": time.time(), "state": "running"})
        self.log({"event": "start", "port": port, "session_epoch": self.session_epoch})
        return port

    def archive(self, mission_id: str, records: dict, seq: int | None = None, result: str | None = None) -> bool:
        """Hand a mission's files to the writer: ``missions/<id>/result.json`` (``result``, its text -- encoded by
        the caller, so a dict the caller keeps cannot change it in the queue) and
        ``records.json`` (the mission's records, handed over at its end -- its own, moved out of the core, so nothing
        changes them meanwhile). Each is written to a temporary file and renamed; the records are streamed in pieces
        and then go to the writer's disposer, freed there a bit at a time (U29: the control thread neither encodes
        nor frees them, and neither holds the interpreter lock long against it). A failure comes back as an outcome,
        noted on result ``seq`` at the next tick start; ``False``: the writer is full, nothing was written (the
        records go to the disposer; the caller marks the result)."""
        folder = self.output / "missions" / mission_id
        box = [records]
        del records

        def write() -> list:
            try:
                return write_files()
            finally:  # whatever happened, the records are freed by the disposer (Codex U29 impl round 3 P2-1)
                self.io.dispose(box.pop(), on_freed=freed)

        def freed(began: float, ended: float) -> None:  # the disposer's thread: the line goes through the writer
            self._write_log(json.dumps({"disposed": {"mission_id": mission_id, "start_mono": began,
                                                     "end_mono": ended}, "mono": self.clock()}) + "\n")

        def write_files() -> list:
            errors = []
            try:
                folder.mkdir(parents=True, exist_ok=True)
            except OSError as exc:
                return [f"missions/{mission_id}: {type(exc).__name__}: {exc}"]
            if result is not None:
                try:
                    _write_atomic(folder / "result.json", result)
                except OSError as exc:
                    errors.append(f"result.json: {type(exc).__name__}: {exc}")
            if box[0]:
                start = self.clock()
                try:
                    size = write_pieces_atomic(folder / "records.json", encode_in_pieces(box[0], _jsonable))
                except (OSError, TypeError, ValueError) as exc:
                    errors.append(f"records.json: {type(exc).__name__}: {exc}")
                else:
                    # U29: when it ran on the tick trace's clock and its size; the writer owns the log file
                    self._append_quietly({"archive": {"mission_id": mission_id, "bytes": size,
                                                      "start_mono": start, "end_mono": self.clock()}})
            if errors:
                self._append_quietly({"archive_error": errors, "mission_id": mission_id})
            return errors

        if self.io.put_keyed(seq if seq is not None else ("archive", mission_id), write):
            return True
        self.io.dispose(box.pop(), on_freed=freed)  # refused: freed on the disposer, not on this thread
        return False

    def _append_quietly(self, entry: dict) -> None:
        """The writer thread's own log line: its failure is counted, never put in place of the job's outcome."""
        try:
            self._append_log(json.dumps(dict(entry, mono=self.clock()), default=_jsonable) + "\n")
        except Exception:  # noqa: BLE001 -- counted; the mission's errors are its own
            self.io.line_errors += 1

    def _io_left(self) -> float:
        """What is left of the session end's one wait on the writer (Codex U29 implementation review P2-5)."""
        if self._io_deadline is None:
            self._io_deadline = time.monotonic() + self.io_budget_s
        return max(0.0, self._io_deadline - time.monotonic())

    def settle_records(self, timeout_s: float | None = None) -> None:
        """The mission files handed to the writer settled within one deadline -- queued or being written -- and
        their failures noted on the results (the run's results are taken after it; Codex U29 review round 2 P2-1). A
        result still pending at the deadline is marked unsettled, once; what its write does after does not change the
        mark (plan D1, as the spool's close)."""
        mark = (f"files unsettled at the session end ({self.io_budget_s:g} s for the writer)" if timeout_s is None
                else f"files unsettled after {timeout_s:g} s")
        for key in self.io.settle(self._io_left() if timeout_s is None else timeout_s):
            if isinstance(key, int) and key not in self._unsettled:
                self._unsettled.add(key)
                self._note_record_error(key, mark)
        self._note_archive_errors()

    def close(self) -> None:
        if self.trace is not None:
            # D12a: the table is written once, here, first (a failing server stop below must not lose it).
            try:
                self.trace_saved = self.trace.save(self.output / "tick_trace.npz")
            except Exception as exc:  # noqa: BLE001 -- recorded, never raised over the run's own end
                self.trace_saved = {"error": repr(exc), "rows": self.trace.rows}
        self.settle_records()
        ended = self.server.stop()
        if self.port_file is not None:
            write_port_file(self.port_file, {"host": self.server.host, "port": self.server.port,
                                             "session_epoch": self.session_epoch, "pid": os.getpid(),
                                             "state": "ended" if ended else "stop_failed"})
        for entry in self.link.log:
            self._write_log(json.dumps(entry) + "\n")
        self.link.log.clear()
        self.log({"event": "end", "server_ended": ended, "pacing": self.pacer.report(),
                  "camera_latency_s": self.camera_latency.report(),
                  "camera_pixel_latency_s": self.camera_pixel_latency.report(),
                  "camera_pixel_unknown": self.camera_pixel_unknown, "tick_trace": self.trace_saved})
        if not self.io.close(self._io_left()):
            return  # stuck on the disk: its file stays open in its thread, and the log has no end summary
        # The writer drained and stopped: the summary goes last, from here, with the counts as they ended (Codex U29
        # implementation review P2-2) -- a reader refuses a log whose summary is missing or counts a loss.
        self._append_quietly({"log_end": {"dropped_lines": self.io.dropped_lines, "line_errors": self.io.line_errors,
                                          "dropped_mission_files": self.io.dropped_keyed,
                                          "record_errors_older": self.record_errors_unlisted}})
        self._log.close()

    def log(self, entry: dict) -> None:
        entry.setdefault("mono", self.clock())
        self._write_log(json.dumps(entry, default=_jsonable) + "\n")

    def _write_log(self, line: str) -> None:
        """The line to the writer (plan U29: the control thread does not wait on the disk)."""
        self.io.put_line(lambda: self._append_log(line), len(line))

    def _append_log(self, line: str) -> None:
        """The writer thread only: the line into the log, rolling the file over at its limit."""
        self._log.write(line)
        self._log_bytes += len(line)
        if self._log_bytes < self._log_rotate_bytes:
            return
        self._log.close()
        base = self.output / "live_log.jsonl"
        for n in range(self._log_keep, 0, -1):
            older = base.with_name(f"{base.name}.{n}")
            if n == self._log_keep:
                older.unlink(missing_ok=True)
            elif older.exists():
                older.rename(base.with_name(f"{base.name}.{n + 1}"))
        base.rename(base.with_name(f"{base.name}.1"))
        self._log = open(base, "w")  # noqa: SIM115 -- closed in close()
        self._log_bytes = 0
        self._log_rotations += 1
        # How often it rolled and how many it keeps: a reader knows when the oldest part is gone (read_live_log).
        (self.output / "live_log.index.json").write_text(
            json.dumps({"rotations": self._log_rotations, "keep": self._log_keep}))

    # --- per control tick ------------------------------------------------------------------------------------------
    def tick_start(self, t: float, near_active: bool, clear_for_submit: bool = True,
                   submit_refusal: str | None = None, localization_ok: bool = True,
                   loaded_pallet: str | None = None) -> tuple[str, list]:
        """The arbiter's cycle start, then the mission's commands with the stop kind it gives (M-b B3). ``clear_for_submit``:
        outside a SLAM hold (else a submit is refused recovery_required); ``submit_refusal``: what the forks are in
        refuses it (persistent console plan D5a: recovery_required, cargo_held); ``loaded_pallet``: the carried pallet a
        submit may deliver as it is carried (plan D5 6c), else None."""
        now = self.clock()
        self.stopped_in_tick = False
        self.arbiter.pacing_ok = self.pacer.ok
        stop_kind = self.link.begin(now, self.feedback)
        self.collect_end(t)  # ended after the last apply (a preparation's job_hold failure): before this one frees it
        records = self.core.commands.apply(t, Context(stop_kind=stop_kind, near_tracker=near_active,
                                                      clear_for_submit=clear_for_submit,
                                                      submit_refusal=submit_refusal,
                                                      scene_damaged=self.scene["damaged"] is not None,
                                                      localization_ok=localization_ok,
                                                      loaded_pallet=loaded_pallet))
        self.collect_end(t)  # ended in this apply (a cancel before it ran, a stopped preparation)
        self._note_archive_errors()
        self._flush(t)
        return stop_kind, records

    def _note_archive_errors(self) -> None:
        """The mission files whose writes failed since the last tick, on their results, published (plan U29); a
        result marked unsettled keeps its mark."""
        noted = False
        for key, errors in self.io.outcomes():
            if errors and isinstance(key, int) and key not in self._unsettled:
                noted = self._note_record_error(key, "; ".join(errors), publish=False) or noted
        if noted:
            self._publish_missions()

    @property
    def record_errors_unlisted(self) -> int:
        return len(self._unlisted_errors)

    def _add_result(self, **result) -> dict:
        """The result into the book; the oldest pushed out with a record error is counted (Codex U29 implementation
        review P2-3: it left no trace in the message)."""
        kept = self.results.message()["results"]
        if len(kept) >= self.results.keep and kept[0].get("record_error"):
            self._unlisted_errors.add(kept[0]["seq"])
        return self.results.add(**result)

    def _note_record_error(self, seq: int, error: str, publish: bool = True) -> bool:
        """``error`` on result ``seq`` (joined to one it has); a result already out of the book is counted once in
        the missions message (``record_errors_older``) and its error is in the log."""
        kept = next((e for e in self.results.message()["results"] if e["seq"] == seq), None)
        if kept is None:
            self._unlisted_errors.add(seq)
        else:
            before = kept.get("record_error")
            self.results.note(seq, record_error=error if before is None else f"{before}; {error}")
        if publish:
            self._publish_missions()
        return True

    def _publish_missions(self) -> None:
        message = self.results.message()
        if self.record_errors_unlisted:
            message["record_errors_older"] = self.record_errors_unlisted
        self.store.put("missions", self.results.seq, message)  # a client connecting later gets it after the targets
        self.channels.put(None, message)

    # --- a mission's end (docs/plans/2026-10-18-persistent-live-console.md D1) -------------------------------------
    def collect_end(self, t: float) -> None:
        """A mission the commands ended by themselves (FINAL, not yet through end_mission), at its end's time."""
        cmds = self.core.commands
        if cmds.status in FINAL and cmds.mission_id is not None and cmds.mission_id != self._ended_id:
            logged = cmds.end_logged_for == cmds.mission_id
            ended = next((r["t"] for r in reversed(cmds.log) if r["command"] == "end"
                          and r["mission_id"] == cmds.mission_id), t) if logged else t
            self.end_mission(ended, cmds.status, cmds.end_reason if logged else "operator")

    def end_mission(self, t: float, outcome: str, reason: str | None) -> dict | None:
        """End the session's current mission at sim ``t``: ``outcome`` is ``complete`` (judged on the truth: done or
        failed), ``failed`` or ``cancelled``. The session goes on; the slot is free at the next tick. Returns the
        result (None: no mission, or this one already ended)."""
        cmds = self.core.commands
        mission_id, mission = cmds.mission_id, cmds.mission
        if mission_id is None or mission_id == self._ended_id:
            return None
        self._ended_id = mission_id
        began = time.perf_counter()
        self.end.stop()
        # The lift held where it is, whatever stage of the tick the end came in (Codex D6 design P2-6) -- within the
        # hold tolerance of the measurement, so a standing lift's sag does not lower its target (main 2026-10-19).
        lift = self._held_lift()
        self.hooks.hold_lift(lift)
        if getattr(self.core, "ms", None) is not None:
            self.core.ms.lift_command = lift
        self.core.abandon_jobs(t)  # step 2: the plan, the capture and the observation of this mission
        self.capturing = False
        self.end.unlock_wheels()  # step 5
        cmds.close_open(t)  # step 5a
        self.end.drain_trackers()
        extra = {}
        final = outcome
        if outcome == "complete":  # step 6
            verdict = self.end.judge_delivery()
            final, reason = ("done", None) if verdict.ok else ("failed", verdict.reason)
            extra["delivery_error_m"] = verdict.delivery_error_m
        if not cmds.terminate(final, t, reason) and cmds.end_logged_for != mission_id:
            cmds.note_end(t, reason)  # step 9: a cancel confirmed standing is already cancelled; logged once
        started = next((r["t"] for r in reversed(cmds.log) if r["command"] == "submit" and r["accepted"]
                        and r["mission_id"] == mission_id), None)
        advice = cmds.end_advice if cmds.end_logged_for == mission_id else None  # plan D13 item 6
        entry = self._add_result(mission_id=mission_id, outcome=cmds.status, reason=reason, started_s=started,
                                 ended_s=float(t), pallet_id=mission.pallet_id if mission else None,
                                 destination_id=mission.destination_id if mission else None, advice=advice,
                                 map_id=(self.slam_info or {}).get("map_id"))
        clock = cmds.take_clock()  # step 8: the next mission's work clock starts clean
        records = self.core.end_reset(t)  # steps 3 and 8: the pocket check closed, the mission's state back to the start
        if cmds.status == "done" and mission is not None:
            # plan D3: verified there only now -- the delivery verdict passed, after the end's last targets update
            self.core.targets.verify_at(mission.pallet_id, mission.destination_id)
        recording = time.perf_counter()  # plan D1 step 7 (U29): the record cost from here, bound 20 ms
        digest = getattr(self.core, "start_digest", None)
        if digest:  # use case U21 on Isaac: the mission-owned state at this mission's start goes with its records
            records = {**records, "start_digest": digest}
            self.core.start_digest = None
        # Step 7: the result (bounded: it and its clock) and the records (under this mission's own id, Codex stage 2
        # impl P2-5) to the writer; the mission's paths and logs went by chunks (plan D8).
        result = _result_text(entry, clock, extra)  # bounded (RESULT_BYTES): encoded here, not queued
        records = {**records, "clock": clock} if any(records.values()) or any(clock.values()) else {}
        if not self.archive(mission_id, records, entry["seq"], result):
            entry = self.results.note(entry["seq"], record_error=f"result.json, records.json: dropped (writer full, "
                                                                 f"{MISSION_FILES_PENDING} pending)")
        del records  # the writer or its disposer frees them (Codex U29 implementation review round 2 P2-3)
        self._publish_missions()
        self.log({"mission_end": entry, "tick_t": t, **extra})
        done = time.perf_counter()
        # U29: the record cost (the handover to the writer, the result, the publish and the end's log line) and the
        # whole end; the restore's (the mission state back to the start, with the records' top-level copies) apart
        self.log({"mission_end_cost": {"mission_id": mission_id, "record_wall_s": done - recording,
                                       "end_wall_s": done - began,
                                       "restore_wall_s": getattr(self.core, "restore_wall_s", None)}})
        return entry

    def _flush(self, t: float) -> None:
        """The link's message log and the arbiter's boundary events (mode, lease, disarm, generation reasons), and
        once a second the stop detector and the feedback it judged."""
        for entry in self.link.log:
            self._write_log(json.dumps(entry) + "\n")
        self.link.log.clear()
        for event in self.arbiter.events:
            self.log(dict(event, arbiter=True, tick_t=t))
        self.arbiter.events.clear()  # plan D8: logged, then let go (nothing else reads them)
        now = self.clock()
        if now >= self._next_status:
            self._next_status = now + 1.0
            fb = self.feedback
            self.log({"status": True, "tick_t": t, "feedback_ok": self.arbiter.feedback_ok,
                      "stop_confirmed": self.arbiter.stop_confirmed,
                      "feedback": None if fb is None else [fb.stamp_mono, fb.rear_axle_speed_mps, fb.yaw_rate_radps],
                      "lease": None if self.arbiter.lease is None else
                      [self.arbiter.lease.client_id, self.arbiter.lease.renewed_mono],
                      "pacing": self.pacer.report(),
                      # Real-time plan S0: the regions' wall time since the last status (empty unless enabled).
                      "timers": TIMERS.take() if TIMERS.enabled else None})

    def tick_end(self, t: float, records: list, safety: dict, candidate: tuple, drive: dict | None):
        """The output for this tick (the runner applies it, then calls ``publish`` with what it applied)."""
        return self.link.end(self.clock(), records, safety, candidate, drive)

    def publish(self, t: float, out, safety: dict, *, pose: dict | None, freshness: dict | None,
                applied: dict | None, speed_mps: float | None, steering_rad: list | None = None,
                lift_m: float | None = None, plan_key=None, plan_poses: Callable | None = None) -> None:
        """The state (with the command as finally applied), the targets and the log entry due this tick.
        ``plan_key`` names the path being followed (None: none); ``plan_poses()`` gives its poses and directions."""
        now = self.clock()
        if self.link.state_due(now):
            msg = state_message(self.arbiter, out, safety, now_mono=now, rear_axle_speed_mps=speed_mps,
                                applied=applied, pose=pose, freshness=freshness, pacing=self.pacer.report(),
                                steering_rad=steering_rad, lift_m=lift_m)
            plan = self.plan_feed.update(plan_key, plan_poses, now)
            if plan is not OMIT:
                msg["plan"] = plan
            msg["last_result"] = self.results.last  # the session's last mission outcome, after its slot is free
            msg["scene"] = self.scene  # a latched damage: automatic work refused until the scene is reset (D6)
            if self.slam_info is not None:  # SLAM localization plan L6: absent = online mapping (or not known)
                msg["slam"] = dict(self.slam_info)
            self.channels.put(None, msg)
        targets = self.core.targets
        if targets.revision != self._targets_revision:
            self._targets_revision = targets.revision
            message = targets.message()
            self.store.put("targets", targets.revision, message)  # a client connecting later gets it after welcome
            self.channels.put(None, message)
        self.log({"tick_t": t, "mode": out.mode, "stop_kind": out.stop_kind, "source": out.source,
                  "speed_mps": out.speed_mps, "curvature_inv_m": out.curvature_inv_m, "armed": out.armed,
                  "deadline": None if self.arbiter.teleop is None else self.arbiter.teleop["deadline"],
                  "pacing_ok": self.pacer.ok, "step": self.step})

    # --- per physics step ------------------------------------------------------------------------------------------
    def before_step(self, sim_t: float) -> bool:
        """Pace, then the gate: True when this step's wheels must be zero (they are zeroed here, the lift
        latched at the first preempted step)."""
        if self.trace is None:
            self.sleep(self.pacer.wait_s(self.clock()))
        else:
            self.trace.begin()
            asked = self.clock()
            wait = self.pacer.wait_s(asked)
            self.trace.set("q", asked)
            self.trace.set("d", wait)
            self.trace.set("live_step", self.step)
            self.sleep(wait)
        now = self.clock()
        self.pacer.started(now)
        if self.trace is not None:
            self.trace.set("S", now)
        self._start_mono = now
        if self._test_stop_in_capture and self.capturing:
            self._test_stop_in_capture = False
            self.log({"test": "stop_in_capture", "step": self.step, "sim_t": sim_t})
            self.arbiter.stop(now, "stop:test_in_capture")  # what a console STOP does at this step's gate
        zero = self.link.gate(now)
        if zero:
            self.stopped_in_tick = True
            self.hooks.zero_wheels()
            if self._lift_latch is None:
                self._lift_latch = self._held_lift()  # the plan D14 rule (main 2026-10-19), once
            self.hooks.hold_lift(self._lift_latch)
        else:
            self._lift_latch = None
        if zero or self.link.log:
            self.log({"gate": zero, "step": self.step, "sim_t": sim_t})
        return zero

    def hold_tolerance(self) -> float:
        """The event hold tolerance now (plan D14): the loaded one while the forks carry or may carry a pallet."""
        a = float(self.arbiter.config.teleop_limits.lift_hold_tolerance_m)
        cargo = getattr(self.core, "cargo", None)
        loaded = cargo is not None and (cargo.relation in ("carrying", "unknown")
                                        or cargo.load in ("acquiring", "confirmed", "releasing", "unknown"))
        if loaded and self.loaded_hold_tolerance_m is not None:
            a = max(a, float(self.loaded_hold_tolerance_m))
        return a

    def _held_lift(self) -> float:
        """Where an event stops the lift (plan D14, main 2026-10-19): the target as it is, within the hold tolerance
        of the measured lift (the loaded one when loaded) -- a target that ran ahead stops near the lift, a standing
        lift's gravity sag leaves the target alone. Without a core target: the measurement."""
        measured = float(self.hooks.lift_position())
        ms = getattr(self.core, "ms", None)
        if ms is None or getattr(ms, "lift_command", None) is None:
            return measured
        a = self.hold_tolerance()
        return float(min(measured + a, max(measured - a, float(ms.lift_command))))

    def after_step(self, sim_t: float, *, odom_speed: float | None, yaw_rate: float, rendered: bool,
                   odom_mono: float | None = None, render_sim_time_of=None, lift_m: float | None = None,
                   steering_rad: float | None = None, odom_xy=None) -> None:
        """The step ended: its clock, the feedback it made, and the media."""
        if self.test_step_delay_s:
            self.sleep(self.test_step_delay_s)  # the slow-pacing test only
        now = self.clock()
        self.pacer.after_step(now)
        self.step += 1
        self.step_start.append((sim_t, self._start_mono))
        if odom_speed is not None and math.isfinite(odom_speed):
            # Stamped when the sample was made (the runner passes it; a SLAM exchange may have waited since --
            # Codex M-c round 5 P1), else at this step's end.
            made = now if odom_mono is None else float(odom_mono)
            # plan D14: the measured lift and the larger measured steering angle go with it
            # main 2026-10-19: the odometry position too, for the stand waiver's creep across feedback gaps
            self.feedback = Feedback(made, float(odom_speed), float(yaw_rate), lift_m=lift_m, steering_rad=steering_rad,
                                     odom_xy=odom_xy)
        if rendered:
            self._camera(sim_t, render_sim_time_of)
        if self.step % 12 == 0:
            scan = self.hooks.scan()
            if scan is not None:
                self.store.put("scan", self.step, scan_item(session_epoch=self.session_epoch, **scan))
        if self.step % 120 == 0:
            grid = self.hooks.grid()
            if grid is not None:
                self.store.put("grid", self.step, grid_item(session_epoch=self.session_epoch, **grid))
        if self.trace is not None:
            self.trace.set("after_end", self.clock())

    def camera_due(self, physics_tick: int | None = None, phase: int | None = None) -> bool:
        """Whether the step about to run renders the live camera. Without a phase, by this runtime's own step count
        (whose offset to the physics counter the runner's warm-up path sets); with one (real-time plan 5.12 D12b),
        on the physics counter about to be stepped: ``physics_tick % camera_every == phase``."""
        if phase is None:
            return (self.step + 1) % self.camera_every == 0
        return physics_tick % self.camera_every == phase

    def _camera(self, sim_t: float, render_sim_time_of) -> None:
        if self.camera_freeze is not None and self.camera_freeze[0] <= sim_t < self.camera_freeze[1]:
            return
        rgb, render_time = self.hooks.camera()
        if rgb is None or render_time is None or not math.isfinite(render_time):
            return
        if self._last_render_time is not None and render_time <= self._last_render_time:
            # The same capture again, or an older one: no new id -- a seen capture keeps its first acquisition time
            # and an older one is not made young (contract 4.2, Codex M-c round 5 P1).
            return
        self._last_render_time = render_time
        acquired = self.clock()
        key = render_time if render_sim_time_of is None else render_sim_time_of(render_time)
        start = next((m for s, m in reversed(self.step_start) if abs(s - key) < 0.5 * self.dt), None)
        if start is not None:
            self.camera_latency.append(acquired - start)
        # The step whose state the pixels show started at `pixel` (plan C5: the pixels trail the render metadata,
        # measured 11 physics steps; the assumed lag carries a render of margin). The arbiter checks that age as well
        # as acquired + bound, and a frame whose step is not in the table is never fresh enough to drive by (Codex
        # M-c round 20). The frame is sent with its real arrival time (contract 4.2).
        pixel = next((m for s, m in reversed(self.step_start)
                      if abs(s - (key - self.camera_pixel_lag_s)) < 0.5 * self.dt), None)
        if pixel is not None:
            self.camera_pixel_latency.append(acquired - pixel)
        else:
            self.camera_pixel_unknown += 1
        self.frame_id += 1
        self.arbiter.register_frame(self.frame_id, acquired, -math.inf if pixel is None else pixel)
        self.store.put("camera_rgb", self.frame_id,
                       camera_item(self.frame_id, np.ascontiguousarray(rgb), acquired, key, self.session_epoch))
        self.log({"camera": self.frame_id, "render_time": render_time, "sim_t": sim_t,
                  "latency_s": None if start is None else acquired - start,
                  "pixel_latency_s": None if pixel is None else acquired - pixel})


def sent_scale(width: int, max_width: int = 640) -> int:
    """The runner halves the picture until it is at most ``max_width`` wide (live_camera in run_transport)."""
    n = 1
    while width // n > max_width:
        n *= 2
    return n


OMIT = object()  # PlanFeed.update: this state carries no plan field


class PlanFeed:
    """The display plan for the state (contract 6: ``plan: {revision, poses: [[x, y, direction], ...]}``, only when it
    changes): a new plan key is a new revision; no plan any more is said once as null; whatever is current is said
    again every ``refresh_s`` so a page that connects later gets it. At most ``max_poses`` poses (plus the cusps)."""

    def __init__(self, refresh_s: float = 2.0, max_poses: int = 200) -> None:
        self.refresh_s, self.max_poses = refresh_s, max_poses
        self.key, self.revision, self.current, self.sent_at = None, 0, None, None

    def update(self, key, poses: Callable | None, now: float):
        if key != self.key:
            self.key = key
            if key is None:
                self.current = None
            else:
                self.revision += 1
                self.current = {"revision": self.revision, "poses": self._thin(*poses())}
        elif self.sent_at is not None and now - self.sent_at < self.refresh_s:
            return OMIT
        self.sent_at = now
        return self.current

    def _thin(self, poses, directions) -> list:
        n = len(poses)
        keep = set(range(0, n, max(1, math.ceil(n / self.max_poses)))) | {n - 1}
        for i in range(1, n):
            if (directions[i] >= 0) != (directions[i - 1] >= 0):
                keep |= {i - 1, i}  # both ends of a cusp: the page draws a node there
        return [[round(float(poses[i][0]), 3), round(float(poses[i][1]), 3), 1 if directions[i] >= 0 else -1]
                for i in sorted(keep)]


def live_video_size(calibration, max_width: int = 640) -> tuple[int, int]:
    """The live video camera's resolution: the perception camera's, halved as ``sent_scale`` halves it, so the picture
    is rendered at the size it is sent and ``live_camera_model`` describes it."""
    n = sent_scale(int(calibration.width), max_width)
    return int(calibration.width) // n, int(calibration.height) // n


# camera_link (x forward, y left, z up) axes as columns, in optical (x right, y down, z forward) coordinates
_OPTICAL_FROM_LINK = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def live_camera_model(calibration, mount, *, rear_axle_offset_m: float, sent_width: int,
                      rides_lift: bool) -> dict:
    """The perception camera as the page's camera_model (contract 3, display only): intrinsics of the picture as
    sent, and the mount as camera_link placed from the rear axle on the floor -- the state's pose is the rear axle.
    ``mount`` is base_link <- optical at lift 0; ``rides_lift``: the camera is on the carriage, so the page adds the
    state's lift_m to its height (and draws no overlay while the lift height is unknown)."""
    n = sent_scale(int(calibration.width), sent_width)
    rotation = np.asarray(mount.rotation, dtype=float) @ _OPTICAL_FROM_LINK
    w = math.sqrt(max(0.0, 1.0 + np.trace(rotation))) / 2.0
    if w < 1e-6:
        raise ValueError("camera mount turned half a turn: not a forward camera")
    q = [(rotation[2, 1] - rotation[1, 2]) / (4 * w), (rotation[0, 2] - rotation[2, 0]) / (4 * w),
         (rotation[1, 0] - rotation[0, 1]) / (4 * w), w]
    t = np.asarray(mount.translation_m, dtype=float)
    return {"calibration_version": 0, "width": int(calibration.width) // n, "height": int(calibration.height) // n,
            "fx": float(calibration.fx) / n, "fy": float(calibration.fy) / n,
            "cx": float(calibration.cx) / n, "cy": float(calibration.cy) / n,
            "base_from_camera": {
                "frame": "camera_link (x forward, y left, z up) from the rear axle on the floor, lift 0",
                "xyz_m": [float(t[0]) + abs(float(rear_axle_offset_m)), float(t[1]), float(t[2])],
                "quaternion_xyzw": [float(v) for v in q]},
            "rides_lift": bool(rides_lift)}


LATENCY_WINDOW = 36_000  # the camera latencies kept (plan D8): measured frames, an hour at 10 Hz and factor 1


class LatencyWindow(deque):
    """The last ``maxlen`` latencies, with the whole session's count and maximum (plan D8: a session left up for
    hours keeps a window, not every frame's)."""

    def __init__(self, maxlen: int) -> None:
        super().__init__(maxlen=maxlen)
        self.n, self.max = 0, None

    def append(self, value: float) -> None:
        super().append(value)
        self.n += 1
        self.max = value if self.max is None else max(self.max, value)

    def report(self) -> dict | None:
        window = _quantiles(list(self))
        if window is None:
            return None
        return {"n": self.n, "max": self.max, "window_n": window["n"], "window_median": window["median"],
                "window_p99": window["p99"], "window": f"the last {self.maxlen} measured frames (a count, not a wall time)"}


def _quantiles(values: list) -> dict | None:
    if not values:
        return None
    v = np.sort(np.asarray(values, dtype=float))
    return {"n": len(v), "median": float(np.median(v)), "p99": float(v[min(len(v) - 1, int(0.99 * len(v)))]),
            "max": float(v[-1])}


# Plan U29: the missions' files the writer holds at most (queued or being written). A mission's records are bounded
# (113-195 KB measured, 2026-10-20; the near-field record by its frame bound, plan D8), and missions end seconds apart
# at the fastest (a refused preparation) while a write takes milliseconds, so four is a backlog of a stuck disk, not
# of a busy session; past it a mission's files are dropped and its result says so.
MISSION_FILES_PENDING = 4


RESULT_BYTES = 64 * 1024  # plan D1 step 7: the result file's bound (it is encoded on the control thread)


def _result_text(entry: dict, clock: dict, extra: dict) -> str:
    """The mission's result file: the result, its clock's summary (the whole clock goes with the records -- 6,000
    waits made 106 KB, Codex U29 implementation review P2-4) and the end's extras; past RESULT_BYTES, the result's
    identity and outcome only, with the size it was cut from."""
    summary = {"pauses": len(clock["pauses"]), "pause_s": float(sum(b - a for a, b in clock["pauses"] if b is not None)),
               "waits": len(clock["waits"]), "wait_s": float(sum(b - a for a, b in clock["waits"] if b is not None))}
    text = json.dumps({"result": entry, "clock_summary": summary, **extra}, default=_jsonable)
    if len(text) <= RESULT_BYTES:
        return text
    keep = {k: entry.get(k) for k in ("seq", "mission_id", "outcome", "started_s", "ended_s", "ended_unix", "map_id",
                                      "pallet_id", "destination_id")}
    return json.dumps({"result": keep, "clock_summary": summary, "cut_from_bytes": len(text)}, default=_jsonable)


def _write_atomic(path: Path, text: str) -> None:
    """``text`` to a temporary file beside ``path``, renamed over it: a reader never sees half a file."""
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def _jsonable(value):
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)
