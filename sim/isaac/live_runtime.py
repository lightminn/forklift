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
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from console.live.server import LiveServer, MediaStore, camera_item, grid_item, scan_item, write_port_file
from console.vehicle_outline import DLS08_MEASURED_CHASSIS
from forklift_core.live.link import Channels, Link
from forklift_core.live.pacing import Pacer
from forklift_core.mission.commands import Context
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
                 camera_model: dict | None = None, catchup_steps: int = 0, end: EndHooks | None = None):
        self.core, self.hooks, self.clock, self.sleep = core, hooks, clock, sleep
        self.end = end or EndHooks()
        self.results = ResultsBook()  # the session's mission outcomes (persistent console plan D1 step 9)
        self._ended_id: str | None = None  # the mission end_mission last ended (each ends once)
        self.dt = dt
        self.session_epoch = session_epoch if session_epoch is not None else secrets.randbits(48)  # a job, an epoch
        self.arbiter = Arbiter(self.session_epoch, config or ArbiterConfig(tick_s=dt),
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
                            "lift": False, "recovery_actions": [],
                            "feedback": {"rear_axle_speed": True, "steering": True, "lift_height": True,
                                         "load_state": False},
                            "media": {"camera_rgb": {"max_hz": camera_hz}, "scan": {"max_hz": 10},
                                      "grid": {"max_hz": 1}},
                            "chassis": dict(DLS08_MEASURED_CHASSIS),  # display only (C3)
                            "provisional_limits": True}}
        if camera_model is not None:
            self.welcome["capabilities"]["camera_model"] = camera_model  # display only (contract 3)
        self.server = LiveServer(self.channels, self.store, self.welcome, host=host, web_root=web_root)
        self.output, self.port_file = Path(output), port_file
        self._log = open(self.output / "live_log.jsonl", "w")  # noqa: SIM115 -- closed in close()
        self.feedback: Feedback | None = None
        self.step = 0
        self.step_start: deque = deque(maxlen=600)  # (sim time at the step's end, mono at its start)
        self._start_mono = None
        self.stopped_in_tick = False  # the gate stopped the wheels inside this control tick (plan C2)
        self._lift_latch: float | None = None  # the lift held since the first preempted step
        self.frame_id = 0
        self._last_render_time = None
        self._targets_revision = None
        self.test_step_delay_s = test_step_delay_s
        self.camera_freeze = camera_freeze  # (from, to) sim seconds: no new frames (the video-stall test)
        self.capturing = False  # the runner marks a capture in progress
        self._test_stop_in_capture = test_stop_in_capture  # tests only: one STOP at the first capture's steps
        self.camera_latency: list[float] = []
        self.camera_pixel_lag_s = camera_pixel_lag_s  # sim s the pixels trail the render metadata (plan C5)
        self.camera_pixel_latency: list[float] = []
        self.camera_pixel_unknown = 0
        self._events_logged = 0
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

    def close(self) -> None:
        ended = self.server.stop()
        if self.port_file is not None:
            write_port_file(self.port_file, {"host": self.server.host, "port": self.server.port,
                                             "session_epoch": self.session_epoch, "pid": os.getpid(),
                                             "state": "ended" if ended else "stop_failed"})
        for entry in self.link.log:
            self._log.write(json.dumps(entry) + "\n")
        self.link.log.clear()
        self.log({"event": "end", "server_ended": ended, "pacing": self.pacer.report(),
                  "camera_latency_s": _quantiles(self.camera_latency),
                  "camera_pixel_latency_s": _quantiles(self.camera_pixel_latency),
                  "camera_pixel_unknown": self.camera_pixel_unknown})
        self._log.close()

    def log(self, entry: dict) -> None:
        entry.setdefault("mono", self.clock())
        self._log.write(json.dumps(entry, default=_jsonable) + "\n")

    # --- per control tick ------------------------------------------------------------------------------------------
    def tick_start(self, t: float, near_active: bool, at_start: bool = True) -> tuple[str, list]:
        """The arbiter's cycle start, then the mission's commands with the stop kind it gives (M-b B3). ``at_start``:
        the truck stands where the mission's first leg (planned before any submit) starts."""
        now = self.clock()
        self.stopped_in_tick = False
        self.arbiter.pacing_ok = self.pacer.ok
        stop_kind = self.link.begin(now, self.feedback)
        records = self.core.commands.apply(t, Context(stop_kind=stop_kind, near_tracker=near_active,
                                                      at_start=at_start))
        if self.core.commands.status == "cancelled":
            self.end_mission(t, "cancelled", "operator")  # cancelled before it ran: nothing moved, nothing to stop
        self._flush(t)
        return stop_kind, records

    # --- a mission's end (docs/plans/2026-10-18-persistent-live-console.md D1) -------------------------------------
    def end_mission(self, t: float, outcome: str, reason: str | None) -> dict | None:
        """End the session's current mission at sim ``t``: ``outcome`` is ``complete`` (judged on the truth: done or
        failed), ``failed`` or ``cancelled``. The session goes on; the slot is free at the next tick. Returns the
        result (None: no mission, or this one already ended)."""
        cmds = self.core.commands
        mission_id, mission = cmds.mission_id, cmds.mission
        if mission_id is None or mission_id == self._ended_id:
            return None
        self._ended_id = mission_id
        self.end.stop()
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
        if not cmds.terminate(final, t, reason):  # step 9: a cancel confirmed standing is already cancelled
            cmds.end_reason = reason
        started = next((r["t"] for r in reversed(cmds.log) if r["command"] == "submit" and r["accepted"]
                        and r["mission_id"] == mission_id), None)
        entry = self.results.add(mission_id=mission_id, outcome=cmds.status, reason=reason, started_s=started,
                                 ended_s=float(t), pallet_id=mission.pallet_id if mission else None,
                                 destination_id=mission.destination_id if mission else None)
        clock = cmds.take_clock()  # step 8: the next mission's work clock starts clean
        try:  # step 7: bounded (the result, its clock); the mission's paths and logs go by chunks (plan D8)
            folder = self.output / "missions" / mission_id
            folder.mkdir(parents=True, exist_ok=True)
            (folder / "result.json").write_text(json.dumps({"result": entry, "clock": clock, **extra},
                                                           default=_jsonable))
        except OSError as exc:
            entry = self.results.note(entry["seq"], record_error=f"{type(exc).__name__}: {exc}")
        message = self.results.message()
        self.store.put("missions", self.results.seq, message)  # a client connecting later gets it after the targets
        self.channels.put(None, message)
        self.log({"mission_end": entry, "tick_t": t, **extra})
        return entry

    def _flush(self, t: float) -> None:
        """The link's message log and the arbiter's boundary events (mode, lease, disarm, generation reasons), and
        once a second the stop detector and the feedback it judged."""
        for entry in self.link.log:
            self._log.write(json.dumps(entry) + "\n")
        self.link.log.clear()
        for event in self.arbiter.events[self._events_logged:]:
            self.log(dict(event, arbiter=True, tick_t=t))
        self._events_logged = len(self.arbiter.events)
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
        self.sleep(self.pacer.wait_s(self.clock()))
        now = self.clock()
        self.pacer.started(now)
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
                self._lift_latch = float(self.hooks.lift_position())
            self.hooks.hold_lift(self._lift_latch)
        else:
            self._lift_latch = None
        if zero or self.link.log:
            self.log({"gate": zero, "step": self.step, "sim_t": sim_t})
        return zero

    def after_step(self, sim_t: float, *, odom_speed: float | None, yaw_rate: float, rendered: bool,
                   odom_mono: float | None = None, render_sim_time_of=None) -> None:
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
            self.feedback = Feedback(made, float(odom_speed), float(yaw_rate))
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

    def camera_due(self) -> bool:
        return (self.step + 1) % self.camera_every == 0

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


def _quantiles(values: list) -> dict | None:
    if not values:
        return None
    v = np.sort(np.asarray(values, dtype=float))
    return {"n": len(v), "median": float(np.median(v)), "p99": float(v[min(len(v) - 1, int(0.99 * len(v)))]),
            "max": float(v[-1])}


def _jsonable(value):
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)
