"""Deterministic-run plan D1: a camera's frame brought to the step's time before its pixels are read.

The annotators of an Isaac camera hand back the frame of whichever render they last finished, which in a normal run
trails the step by a varying number of renders -- the reason two same-seed runs read different RGB (and, at times,
depth) on the same tick. In a deterministic run the caller renders without physics (``world.render()``) until the
camera frame's ``rendering_time`` is the step's simulation time; the 2026-10-24 probes found the pixels identical
across processes that way even when the number of extra renders differed. The count is a diagnostic (left out of the
deterministic comparison); past ``cap`` renders the run ends. Off, nothing is rendered."""

from __future__ import annotations

import math

from forklift_core import determinism

TOLERANCE_S = 1e-6


class FrameSync:
    def __init__(self, cap: int = 20):
        self.cap = int(cap)
        self.calls = 0
        self.renders = 0
        self.max_renders = 0

    def sync(self, camera, render, sim_time: float) -> int:
        """Renders (``render()``) until ``camera``'s frame is at ``sim_time``; the number of renders it took."""
        if not determinism.enabled():
            return 0
        self.calls += 1
        done = 0
        while True:
            frame = camera.get_current_frame()
            rendered = frame.get("rendering_time") if isinstance(frame, dict) else None
            if rendered is not None and math.isfinite(float(rendered)) and abs(float(rendered) - sim_time) <= TOLERANCE_S:
                break
            if done >= self.cap:
                determinism.mark(f"frame_sync:{self.cap}")
                raise determinism.DeterministicBroken(f"frame_sync: no frame at {sim_time:.6f} s after {done} renders")
            render()
            done += 1
        self.renders += done
        self.max_renders = max(self.max_renders, done)
        return done

    def summary(self) -> dict:
        return {"calls": self.calls, "renders": self.renders, "max_renders": self.max_renders, "cap": self.cap}
