"""Deterministic-run plan D1: before a camera's pixels are read in a deterministic run, render (no physics) until the
frame's rendering_time is the step's simulation time -- the annotators otherwise hand back a frame of an earlier
render, which differs between runs. The render count is a diagnostic, the cap ends the run."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from frame_sync import FrameSync, rgba_of  # noqa: E402

from forklift_core import determinism  # noqa: E402


class FakeCamera:
    """rendering_time trails by ``lag`` renders: each render moves it one render period on, up to the current time."""

    def __init__(self, sim_time, lag, dt=1 / 120):
        self.sim_time, self.dt = sim_time, dt
        self.time = sim_time - lag * dt

    def get_current_frame(self):
        return {"rendering_time": self.time}

    def render(self):
        self.time = min(self.sim_time, self.time + self.dt)


@pytest.fixture(autouse=True)
def det_on():
    determinism.configure(True)
    yield
    determinism.configure(False)


def test_renders_until_the_frame_is_the_steps_time_and_counts():
    cam = FakeCamera(5.0, lag=3)
    sync = FrameSync(cap=20)
    assert sync.sync(cam, cam.render, 5.0) == 3
    assert cam.get_current_frame()["rendering_time"] == pytest.approx(5.0, abs=1e-9)
    assert sync.sync(cam, cam.render, 5.0) == 0  # already there
    assert sync.renders == 3 and sync.calls == 2 and sync.max_renders == 3


def test_a_frame_that_never_arrives_ends_the_run():
    cam = FakeCamera(5.0, lag=3)
    stuck = lambda: None  # noqa: E731
    sync = FrameSync(cap=5)
    with pytest.raises(determinism.DeterministicBroken, match="frame_sync"):
        sync.sync(cam, stuck, 5.0)
    assert determinism.broken() == ["frame_sync:5"]


def test_a_frame_without_a_time_ends_the_run():
    class NoTime:
        def get_current_frame(self):
            return {}

    with pytest.raises(determinism.DeterministicBroken):
        FrameSync(cap=3).sync(NoTime(), lambda: None, 1.0)


def test_off_it_does_nothing():
    determinism.configure(False)
    cam = FakeCamera(5.0, lag=3)
    sync = FrameSync(cap=20)
    assert sync.sync(cam, cam.render, 5.0) == 0
    assert cam.get_current_frame()["rendering_time"] < 5.0


class RgbCamera:
    """The current frame's rgba (one snapshot with its rendering_time) and a direct getter that reads the annotator
    again -- which, the 2026-10-25 probe found, differs between processes even after the sync."""

    def get_current_frame(self):
        return {"rendering_time": 1.0, "rgba": "frame-rgba"}

    def get_rgba(self):
        return "direct-rgba"


def test_a_deterministic_run_reads_rgb_from_the_frame_snapshot():
    assert rgba_of(RgbCamera()) == "frame-rgba"


def test_off_it_reads_the_direct_getter_as_before():
    determinism.configure(False)
    assert rgba_of(RgbCamera()) == "direct-rgba"


def test_a_frame_without_rgba_ends_the_run():
    class NoRgba(RgbCamera):
        def get_current_frame(self):
            return {"rendering_time": 1.0}

    with pytest.raises(determinism.DeterministicBroken, match="rgba"):
        rgba_of(NoRgba())
