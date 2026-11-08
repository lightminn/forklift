"""The vehicle-side F1 block in the live runtime (docs/plans/2026-11-08-f1-vehicle-block.md 2.2, 2.4): the latch is set
in start(), before the relocalization is let go, from what the runner decided; the state and the mission commands carry
it."""

import inspect

import pytest

pytest.importorskip("websockets")

from test_live_runtime import (  # noqa: E402, F401
    _MADE,
    DT,
    Clock,
    _close_runtimes,
    runtime,
)

from forklift_core.safety.arbiter import ArbiterConfig  # noqa: E402
from sim.isaac import live_runtime as L  # noqa: E402


def _parts():
    return L.make_console(config=ArbiterConfig(tick_s=DT), session_epoch=9, port=0)


def test_without_a_block_the_runtime_starts_as_before(tmp_path):
    rt, _, _, _ = runtime(tmp_path)
    rt.start()
    assert rt.arbiter.start_block is None


def test_a_block_is_latched_when_the_server_opens(tmp_path):
    rt, _, _, clock = runtime(tmp_path, start_block=("operator_placed_start", "operator_placed"))
    assert rt.arbiter.start_block is None  # not before start()
    rt.start()
    assert rt.arbiter.start_block == {"reason": "operator_placed_start", "source": "operator_placed",
                                      "since_mono": clock()}


def test_the_latch_comes_before_the_relocalization_is_let_go(tmp_path):
    parts = _parts()
    seen = []

    class Reloc:  # what the arbiter holds while relocalizing; records the latch the moment it is dropped
        pass

    parts.arbiter.reloc = Reloc()
    rt, _, _, _ = runtime(tmp_path, parts=parts, config=ArbiterConfig(tick_s=DT),
                          start_block=("operator_placed_start", "operator_placed"))
    original = type(rt.arbiter).__setattr__

    def watch(self, name, value):
        if name == "reloc" and value is None:
            seen.append(self.start_block)
        original(self, name, value)

    type(rt.arbiter).__setattr__ = watch
    try:
        rt.start()
    finally:
        type(rt.arbiter).__setattr__ = original
    assert seen and seen[0] is not None and seen[0]["reason"] == "operator_placed_start"
    assert rt.arbiter.reloc is None


def test_a_bad_block_is_refused(tmp_path):
    with pytest.raises(ValueError):
        runtime(tmp_path, start_block=("something_else", "operator_placed"))


def test_the_state_and_the_commands_carry_the_latch():
    src = inspect.getsource(L.LiveRuntime)
    assert 'msg["start_block"] = dict(self.arbiter.start_block)' in src
    assert "start_blocked=self.arbiter.start_block is not None" in src
