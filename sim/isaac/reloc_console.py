"""The operator console while the start is unknown (docs/plans/2026-10-29-start-relocalization.md 1.2 steps 4 and 7,
1.3; Codex B-P2c design rounds 1-3). Isaac-free.

A relocalizing runner opens the early console (``live_runtime.make_console``) before its mission core exists and
services it from its main thread -- the only owner of the arbiter -- at every physics step and wherever it waits:
``service(stage)`` takes the STOPs at every call (the gate), runs the arbiter's cycle at most once a tick with a fresh
measurement of the standing truck, interrupts the relocalization at a new STOP, leaves the phase on the operator's
session end, and publishes the state: no pose, and the relocalization in ``slam.reloc``. The survey map is the page's
grid. Gaps between calls longer than ``record_gap_s`` are recorded with the stage they ended in: inside the runner's
non-stepping setup no physics step happens (nothing can move) and the queued STOP is taken at the next call, before the
next step -- the same guarantee the live loop gives; an immediate answer is not promised there. The port file follows
the relocalization's state and never says ``running`` before the live runtime adopts the console.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np

from console.live.server import grid_item, write_port_file
from forklift_core.localization.start_reloc import base_from_laser
from forklift_core.safety.arbiter import Feedback
from forklift_core.safety.state import state_message

# the standing truck's safety picture for the arbiter's cycle: nothing loaded, no forks in a pocket, no near-field
SAFE = {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
        "frame": {"hold": "none", "slam": False}, "mission": {"phase": "idle"}}


class RelocSessionEnd(Exception):
    """The operator ended the session while the start was unknown: the runner leaves with the session's end code."""


def grid_from_bundle(grid, *, session_epoch: int, version: int = 0) -> dict:
    """The survey bundle's occupancy (-1 unknown, 0 free, 100 occupied; in between by threshold 50) as the console's
    grid item (0 unknown, 1 free, 2 occupied), unrotated origin only (the matcher's own condition)."""
    if abs(float(grid.origin[2])) > 1e-12:
        raise ValueError("a rotated grid origin cannot be drawn as the console's grid")
    data = np.asarray(grid.data)
    cells = np.zeros(data.shape, np.uint8)
    cells[(data >= 0) & (data < 50)] = 1
    cells[data >= 50] = 2
    height, width = cells.shape
    return grid_item(cells.tobytes(), width, height, float(grid.resolution_m), grid.origin[:2], session_epoch,
                     "map", version)


class RelocConsole:
    def __init__(self, parts, *, measure, clock=time.monotonic, port_file: Path | None = None, tick_s: float = 1 / 120,
                 slam_info: dict | None = None, grid=None, record_gap_s: float = 0.2):
        self.parts, self.measure, self.clock, self.port_file = parts, measure, clock, port_file
        self.tick_s, self.slam_info, self.record_gap_s = tick_s, dict(slam_info or {}), record_gap_s
        self.reloc = None
        self.notes: dict = {}
        self.gaps: list = []
        self.interrupts = 0
        self.ended_by_operator = False
        self._last_call = None
        self._last_cycle = None
        self._stops_seen = parts.arbiter.stops
        self._override = None  # "localizing" once the estimate went to the bridge
        self._port_state = None
        if grid is not None:
            parts.store.put("grid", 0, grid_from_bundle(grid, session_epoch=parts.session_epoch))

    # --- lifecycle ---------------------------------------------------------------------------------------------------
    def open(self) -> int:
        port = self.parts.server.start()
        self.parts.started_wall = time.time()
        self.open_port_file()
        return port

    def open_port_file(self) -> None:
        if self.parts.started_wall is None:
            self.parts.started_wall = time.time()
        self._write_port(self.state)

    def close(self, state: str = "ended") -> bool:
        """Before the live runtime adopts it (a failure or the session's end in the setup): the server stops."""
        ended = self.parts.server.stop()
        self._write_port(state if ended else "stop_failed", force=True)
        return ended

    def attach(self, relocalizer) -> None:
        self.reloc = relocalizer
        self.parts.arbiter.reloc = relocalizer

    def note(self, **fields) -> None:
        self.notes.update(fields)

    def localizing(self) -> None:
        self._override = "localizing"

    @property
    def state(self) -> str:
        if self._override is not None:
            return self._override
        if self.reloc is None:
            return "relocalizing"
        # the pick or the matcher's choice before the estimate goes out is the contract's localizing (Codex P3)
        return "localizing" if self.reloc.state == "chosen" else self.reloc.state

    # --- the main thread's calls -------------------------------------------------------------------------------------
    def gate(self, stage: str) -> None:
        """The STOPs only: right before a physics step (Codex B-P2c impl review P2)."""
        self.parts.link.gate(self.clock())
        self._interrupt_on_stop()

    def service(self, stage: str) -> None:
        now = self.clock()
        if self._last_call is not None and now - self._last_call > self.record_gap_s:
            self.gaps.append({"stage": stage, "gap_s": now - self._last_call})
        self._last_call = now
        link, arbiter = self.parts.link, self.parts.arbiter
        link.gate(now)  # every STOP before anything, at every call
        out = None
        if self._last_cycle is None or now - self._last_cycle >= self.tick_s - 1e-9:
            self._last_cycle = now
            speed, yaw_rate, lift_m, steering_rad = self.measure()
            link.begin(now, Feedback(now, speed, yaw_rate, lift_m=lift_m, steering_rad=steering_rad))
            out = link.end(now, [], SAFE, (0.0, 0.0))
        if arbiter.session_request == "end":
            # accepted: the session's end, before anything below can fail (Codex B-P2c impl review P2)
            self.ended_by_operator = True
        self._interrupt_on_stop()
        try:
            if out is not None and link.state_due(now):
                msg = state_message(arbiter, out, SAFE, now_mono=now, rear_axle_speed_mps=speed, pose=None,
                                    lift_m=lift_m)
                msg["slam"] = {**self.slam_info, "reloc": self._reloc_info(now)}
                self.parts.channels.put(None, msg)
            self._write_port(self.state)
        finally:
            if self.ended_by_operator:
                raise RelocSessionEnd("the operator ended the session before the start was known")

    def _interrupt_on_stop(self) -> None:
        arbiter = self.parts.arbiter
        if arbiter.stops != self._stops_seen:
            self._stops_seen = arbiter.stops
            if self.reloc is not None and self.reloc.interrupt():
                self.interrupts += 1

    # --- helpers -----------------------------------------------------------------------------------------------------
    def _reloc_info(self, now: float) -> dict:
        r = self.reloc
        deadline = None if r is None or r.deadline_at is None else max(0.0, r.deadline_at - self.clock())
        operator = self.notes.get("operator_deadline_at")
        return {"state": self.state, "generation": None if r is None else r.generation,
                "attempts": None if r is None else r.attempts, "last_reason": self.notes.get("last_reason"),
                "deadline_left_s": deadline,
                "operator_left_s": None if operator is None else max(0.0, operator - now),
                "candidates": [] if r is None else [
                    {"index": k, "base": list(base_from_laser(c.pose)), "score": float(c.search_score),
                     "endpoint_fraction": float(c.endpoint_fraction)} for k, c in enumerate(r.shown)]}

    def _write_port(self, state: str, force: bool = False) -> None:
        if self.port_file is None or self.parts.started_wall is None or (state == self._port_state and not force):
            return
        self._port_state = state
        server = self.parts.server
        write_port_file(self.port_file, {"host": server.host, "port": server.port,
                                         "session_epoch": self.parts.session_epoch, "pid": os.getpid(),
                                         "started": self.parts.started_wall, "state": state})


__all__ = ["SAFE", "RelocConsole", "RelocSessionEnd", "grid_from_bundle"]
