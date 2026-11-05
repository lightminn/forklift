"""Deterministic-run plan D1: a camera's frame brought to the step's time before its pixels are read.

The annotators of an Isaac camera hand back the frame of whichever render they last finished, which in a normal run
trails the step by a varying number of renders -- the reason two same-seed runs read different depth on the same
tick at times. In a deterministic run the caller renders without physics (``world.render()``) until the camera
frame's ``rendering_time`` is the step's simulation time; the 2026-10-25 probes (ws1 job 11154) found depth identical
across processes that way, even when the number of extra renders differed. RTX RGB is not: it differed between
processes on every frame with the same simulation and rendering times (plan D1b). The count is a diagnostic (left out
of the deterministic comparison); past ``cap`` renders the run ends. Off, nothing is rendered."""

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

    def sync(self, camera, render, sim_time: float, rgb: bool = False) -> int:
        """Renders (``render()``) until ``camera``'s frame is at ``sim_time``; the number of renders it took.

        ``rgb``: the caller reads the frame's RGB (``rgba_of``), so the frame must also hold it. A camera's first
        renders carry the step's rendering_time with an empty RGB array (Isaac 5.1 stores each annotator's get_data()
        at every render) -- with video off the perception camera is first rendered at the first far frame, which then
        held no RGB (2026-11-05 mass-probe batch). The extra renders do not step physics. A frame that already holds
        RGB adds none, so a deterministic run that read RGB here before renders as it did."""
        if not determinism.enabled():
            return 0
        self.calls += 1
        done = 0
        while True:
            frame = camera.get_current_frame()
            rendered = frame.get("rendering_time") if isinstance(frame, dict) else None
            at_time = (rendered is not None and math.isfinite(float(rendered))
                       and abs(float(rendered) - sim_time) <= TOLERANCE_S)
            if at_time and (not rgb or _holds_rgb(frame)):
                break
            if done >= self.cap:
                if at_time:
                    determinism.mark("frame_sync:no_rgba")
                    raise determinism.DeterministicBroken(
                        f"frame_sync: no rgb in the frame at {sim_time:.6f} s after {done} renders")
                determinism.mark(f"frame_sync:{self.cap}")
                raise determinism.DeterministicBroken(f"frame_sync: no frame at {sim_time:.6f} s after {done} renders")
            render()
            done += 1
        self.renders += done
        self.max_renders = max(self.max_renders, done)
        return done

    def summary(self) -> dict:
        return {"calls": self.calls, "renders": self.renders, "max_renders": self.max_renders, "cap": self.cap}


def _holds_rgb(frame) -> bool:
    """The frame's RGB as ``rgba_of`` accepts it: present and not empty."""
    rgba = frame.get("rgb") if isinstance(frame, dict) else None
    return rgba is not None and hasattr(rgba, "__len__") and len(rgba) > 0


def rgba_of(camera):
    """A camera's RGB(A) as the run reads it: the direct getter (``get_rgba``, as before) -- or, in a deterministic
    run, the current frame's own RGB (key "rgb"), the annotator's data taken at that render's completion with its
    rendering_time. The direct getter reads the annotator again, whenever it is called; the 2026-10-25 probe found its
    RGB different between processes even after the sync (0/40 frames)."""
    if not determinism.enabled():
        return camera.get_rgba()
    frame = camera.get_current_frame()
    # the rgb annotator's data sits under its own name, "rgb" (Isaac 5.1 Camera.attach_annotator)
    rgba = frame.get("rgb") if isinstance(frame, dict) else None
    if rgba is None or not hasattr(rgba, "__len__") or len(rgba) == 0:
        determinism.mark("frame_sync:no_rgba")
        raise determinism.DeterministicBroken("frame_sync: the current frame holds no rgb")
    return rgba
