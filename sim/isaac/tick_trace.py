"""Per physics step timestamps of a live run (real-time plan 5.12 D12a).

One row per physics step of the live loop, in preallocated memory: nothing is written while the run goes (a
periodic write would land on the same tick phase and change what is measured), and the whole table is written once
on the run's end path. Times come from the runtime's own clock (the pacer's), so the row's pacing marks and its
work marks are on one time line.

Columns (seconds unless named otherwise):
  q, d, S        the wait request's time, the wait asked for, the step's paced start (``Pacer.wait_s`` / ``started``)
  world_start, world_end   the ``world.step`` call
  step_end       ``step_world`` returned (the scan's processing included)
  after_end      ``after_step`` returned (the camera's read included)
  ctrl_start, ctrl_end     the control tick before this step (``ctrl_end`` just before the step is asked for)
  replies_s      the time in ``core.slam_replies`` within that control tick
  tick, live_step          the physics counter about to be stepped and the runtime's step count
  flags          a bitmask of FLAGS: who asked this step to render (the live camera, a video frame, the runner's
                 render argument -- set for a capture's or a pocket frame's step, which the loop marks as
                 ``capture`` / ``pocket`` -- the far recognition's driving frame, and the scan tick) and whether
                 ``world.step`` rendered; a render with none of the request bits is unattributed

W_i, the work of step i under the pacer, is q[i+1] - S[i]: it holds the step, its media and the next control tick.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

COLUMNS = ("q", "d", "S", "world_start", "world_end", "step_end", "after_end", "ctrl_start", "ctrl_end", "replies_s",
           "tick", "live_step", "flags")
FLAGS = ("live_camera", "video", "render_arg", "scan", "rendered", "capture", "pocket", "far")
_CONTROL = ("ctrl_start", "ctrl_end", "replies_s")


class TickTrace:
    def __init__(self, capacity: int, clock) -> None:
        self.capacity = int(capacity)
        self.clock = clock
        self.data = np.full((self.capacity, len(COLUMNS)), np.nan)
        self._col = {name: i for i, name in enumerate(COLUMNS)}
        self.rows = 0  # rows begun
        self.dropped = 0  # steps past the capacity
        self._pending: dict[str, float] = {}  # control marks made before the step's row begins
        self._pending_flags = 0

    def now(self) -> float:
        return self.clock()

    # The control tick runs before its step, so its marks wait until the step's row begins.
    def control(self, name: str, value: float) -> None:
        if name not in _CONTROL:
            raise KeyError(name)
        self._pending[name] = self._pending.get(name, 0.0) + value if name == "replies_s" else value

    def control_flag(self, name: str) -> None:
        """A flag the control tick knows of the step it is about to ask for (a capture's, a pocket frame's)."""
        self._pending_flags |= 1 << FLAGS.index(name)

    def begin(self) -> None:
        """A physics step begins: a new row, holding the control marks made since the last one."""
        if self.rows >= self.capacity:
            self.dropped += 1
        else:
            row = self.data[self.rows]
            for name, value in self._pending.items():
                row[self._col[name]] = value
            if self._pending_flags:
                row[self._col["flags"]] = self._pending_flags
        self.rows += 1
        self._pending = {}
        self._pending_flags = 0

    def set(self, name: str, value: float) -> None:
        if self.rows == 0 or self.rows > self.capacity:
            return
        self.data[self.rows - 1, self._col[name]] = value

    def flag(self, name: str) -> None:
        if self.rows == 0 or self.rows > self.capacity:
            return
        row = self.data[self.rows - 1]
        bits = 0 if np.isnan(row[self._col["flags"]]) else int(row[self._col["flags"]])
        row[self._col["flags"]] = bits | (1 << FLAGS.index(name))

    def save(self, path: Path) -> dict:
        kept = min(self.rows, self.capacity)
        np.savez(path, data=self.data[:kept], columns=np.array(COLUMNS), flags=np.array(FLAGS))
        return {"rows": kept, "dropped": self.dropped, "path": str(path)}


def work(data: np.ndarray) -> np.ndarray:
    """W_i = q[i+1] - S[i] (NaN for the last row)."""
    col = {name: i for i, name in enumerate(COLUMNS)}
    out = np.full(len(data), np.nan)
    out[:-1] = data[1:, col["q"]] - data[:-1, col["S"]]
    return out
