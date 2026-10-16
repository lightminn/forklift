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
from forklift_core.live.link import Channels, Link
from forklift_core.live.pacing import Pacer
from forklift_core.mission.commands import Context
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


class LiveRuntime:
    def __init__(self, core, hooks: Hooks, *, output: Path, port_file: Path | None, dt: float, factor: float = 1.0,
                 camera_hz: float = 10.0, host: str = "127.0.0.1", web_root: Path | None = None,
                 config: ArbiterConfig | None = None, clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep, session_epoch: int | None = None,
                 test_step_delay_s: float = 0.0, camera_freeze: tuple | None = None):
        self.core, self.hooks, self.clock, self.sleep = core, hooks, clock, sleep
        self.dt = dt
        self.session_epoch = session_epoch if session_epoch is not None else secrets.randbits(48)  # a job, an epoch
        self.arbiter = Arbiter(self.session_epoch, config or ArbiterConfig(tick_s=dt),
                               lambda: secrets.token_hex(16), mission=core.commands)
        self.channels = Channels()
        self.link = Link(self.arbiter, self.channels)
        self.store = MediaStore()
        self.pacer = Pacer(factor, dt)
        self.camera_every = max(1, round(1.0 / (camera_hz * dt)))
        self.welcome = {"t": "welcome", "api": "console/1", "vehicle_id": "isaac-forklift",
                        "session_epoch": self.session_epoch, "backend": "isaac", "capabilities": {
                            "control": True, "mission": True, "teleop_direct": True, "teleop_video": True,
                            "lift": False, "recovery_actions": [],
                            "feedback": {"rear_axle_speed": True, "steering": True, "lift_height": True,
                                         "load_state": False},
                            "media": {"camera_rgb": {"max_hz": camera_hz}, "scan": {"max_hz": 10},
                                      "grid": {"max_hz": 1}},
                            "provisional_limits": True}}
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
        self.camera_latency: list[float] = []
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
                  "camera_latency_s": _quantiles(self.camera_latency)})
        self._log.close()

    def log(self, entry: dict) -> None:
        entry.setdefault("mono", self.clock())
        self._log.write(json.dumps(entry, default=_jsonable) + "\n")

    # --- per control tick ------------------------------------------------------------------------------------------
    def tick_start(self, t: float, near_active: bool) -> tuple[str, list]:
        """The arbiter's cycle start, then the mission's commands with the stop kind it gives (M-b B3)."""
        now = self.clock()
        self.stopped_in_tick = False
        self.arbiter.pacing_ok = self.pacer.ok
        stop_kind = self.link.begin(now, self.feedback)
        records = self.core.commands.apply(t, Context(stop_kind=stop_kind, near_tracker=near_active))
        self._flush(t)
        return stop_kind, records

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
                      "pacing": self.pacer.report()})

    def tick_end(self, t: float, records: list, safety: dict, candidate: tuple, drive: dict | None):
        """The output for this tick (the runner applies it, then calls ``publish`` with what it applied)."""
        return self.link.end(self.clock(), records, safety, candidate, drive)

    def publish(self, t: float, out, safety: dict, *, pose: dict | None, freshness: dict | None,
                applied: dict | None, speed_mps: float | None) -> None:
        """The state (with the command as finally applied), the targets and the log entry due this tick."""
        now = self.clock()
        if self.link.state_due(now):
            msg = state_message(self.arbiter, out, safety, now_mono=now, rear_axle_speed_mps=speed_mps,
                                applied=applied, pose=pose, freshness=freshness, pacing=self.pacer.report())
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
        self.frame_id += 1
        self.arbiter.register_frame(self.frame_id, acquired)
        self.store.put("camera_rgb", self.frame_id,
                       camera_item(self.frame_id, np.ascontiguousarray(rgb), acquired, key, self.session_epoch))
        self.log({"camera": self.frame_id, "render_time": render_time, "sim_t": sim_t,
                  "latency_s": None if start is None else acquired - start})


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
