"""Deterministic-run plan D1: before a camera's pixels are read in a deterministic run, render (no physics) until the
frame's rendering_time is the step's simulation time -- the annotators otherwise hand back a frame of an earlier
render, which differs between runs. The render count is a diagnostic, the cap ends the run."""

import sys
from pathlib import Path

import numpy as np
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
        return {"rendering_time": 1.0, "rgb": "frame-rgba"}  # Isaac 5.1: the rgb annotator's own name

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
            return {"rendering_time": 1.0, "rgba": "not the key Isaac uses"}

    with pytest.raises(determinism.DeterministicBroken, match="holds no rgb"):
        rgba_of(NoRgba())


class ColdCamera:
    """A camera rendered for the first time (a deterministic run with video off): each render moves rendering_time one
    period on up to the step's time, and the RGB annotator hands back an empty array for its first ``warm`` renders
    (Isaac 5.1 stores each annotator's get_data() at every render; its own get_rgb treats an empty array as no
    data) -- the 2026-11-05 mass-probe batch's frame: rendering_time at the step, rgb empty."""

    def __init__(self, sim_time, lag, warm, empty=lambda: np.zeros((0,), dtype=np.uint8), dt=1 / 120):
        self.sim_time, self.dt, self.warm, self.empty = sim_time, dt, warm, empty
        self.time = sim_time - lag * dt
        self.renders = 0

    def get_current_frame(self):
        frame = {"rendering_time": self.time}
        rgb = np.zeros((4, 6, 4), dtype=np.uint8) if self.renders >= self.warm else self.empty()
        if rgb is not KEY_MISSING:
            frame["rgb"] = rgb
        return frame

    def render(self):
        self.renders += 1
        self.time = min(self.sim_time, self.time + self.dt)


KEY_MISSING = object()


def test_the_first_frame_of_a_cold_camera_holds_no_rgb_without_the_rgb_wait():
    # the failure as it was: the frame reaches the step's time on the first render, its rgb is still empty
    cam = ColdCamera(5.0, lag=1, warm=3)
    assert FrameSync(cap=20).sync(cam, cam.render, 5.0) == 1
    with pytest.raises(determinism.DeterministicBroken, match="holds no rgb"):
        rgba_of(cam)


def test_a_reader_of_rgb_renders_until_the_frame_holds_rgb_at_the_steps_time():
    cam = ColdCamera(5.0, lag=1, warm=3)
    sync = FrameSync(cap=20)
    assert sync.sync(cam, cam.render, 5.0, rgb=True) == 3
    assert cam.get_current_frame()["rendering_time"] == pytest.approx(5.0, abs=1e-9)
    assert rgba_of(cam).shape == (4, 6, 4)
    assert sync.renders == 3 and sync.max_renders == 3 and sync.summary() == {
        "calls": 1, "renders": 3, "max_renders": 3, "cap": 20}  # no new keys: a recorded run's result stays the same


def test_the_rgb_wait_adds_nothing_when_the_frame_already_holds_rgb():
    # a deterministic run that already read rgb at this point renders exactly as before
    warm = ColdCamera(5.0, lag=3, warm=0)
    assert FrameSync(cap=20).sync(warm, warm.render, 5.0, rgb=True) == 3  # only the time, as before
    assert FrameSync(cap=20).sync(warm, warm.render, 5.0, rgb=True) == 0


@pytest.mark.parametrize("empty", [lambda: KEY_MISSING, lambda: None, lambda: np.zeros((0,), dtype=np.uint8)],
                         ids=["missing", "none", "empty"])
def test_missing_none_and_empty_rgb_are_not_ready(empty):
    cam = ColdCamera(5.0, lag=0, warm=2, empty=empty)
    assert FrameSync(cap=20).sync(cam, cam.render, 5.0, rgb=True) == 2


def test_a_black_image_is_rgb():
    class Black(ColdCamera):
        def get_current_frame(self):
            return {"rendering_time": self.time, "rgb": np.zeros((4, 6, 4), dtype=np.uint8)}

    cam = Black(5.0, lag=0, warm=0)
    assert FrameSync(cap=20).sync(cam, cam.render, 5.0, rgb=True) == 0


def test_the_cap_counts_every_render_and_rgb_on_the_last_one_is_accepted():
    cam = ColdCamera(5.0, lag=1, warm=20)
    assert FrameSync(cap=20).sync(cam, cam.render, 5.0, rgb=True) == 20


def test_rgb_that_never_arrives_ends_the_run_as_no_rgba_with_the_render_count():
    cam = ColdCamera(5.0, lag=1, warm=10**6)
    sync = FrameSync(cap=20)
    with pytest.raises(determinism.DeterministicBroken, match=r"no rgb .* after 20 renders"):
        sync.sync(cam, cam.render, 5.0, rgb=True)
    assert cam.renders == 20
    assert determinism.broken() == ["frame_sync:no_rgba"]


def test_a_frame_whose_time_never_arrives_still_ends_the_run_by_the_cap():
    cam = ColdCamera(5.0, lag=3, warm=0)
    with pytest.raises(determinism.DeterministicBroken, match="no frame at"):
        FrameSync(cap=5).sync(cam, lambda: None, 5.0, rgb=True)
    assert determinism.broken() == ["frame_sync:5"]


def test_off_the_rgb_wait_does_nothing():
    determinism.configure(False)
    cam = ColdCamera(5.0, lag=1, warm=3)
    assert FrameSync(cap=20).sync(cam, cam.render, 5.0, rgb=True) == 0
    assert cam.renders == 0


def test_the_rgb_wait_renders_without_stepping_physics_or_drawing_random_numbers():
    import random

    class World:
        def __init__(self, cam):
            self.cam, self.steps = cam, 0

        def step(self, render=False):
            self.steps += 1

        def render(self):
            self.cam.render()

    cam = ColdCamera(5.0, lag=1, warm=4)
    world = World(cam)
    np_state, py_state = np.random.get_state(), random.getstate()
    assert FrameSync(cap=20).sync(cam, world.render, 5.0, rgb=True) == 4
    assert world.steps == 0
    after = np.random.get_state()
    assert all(np.array_equal(a, b) if isinstance(a, np.ndarray) else a == b for a, b in zip(np_state, after))
    assert random.getstate() == py_state
