"""The deterministic-run switch's core side (docs/plans/2026-10-25-deterministic-sim-mode.md D6): off, every wall
budget is what the caller asked for and a timeout is the planner's ordinary result; on, a budget is the wall cap and
reaching it ends the run (DeterministicBroken) instead of switching to another path; the broken reasons are kept."""

import pytest

from forklift_core import determinism


@pytest.fixture(autouse=True)
def off_after():
    yield
    determinism.configure(False)


def test_off_by_default_budgets_are_the_callers():
    assert determinism.enabled() is False
    assert determinism.budget(20.0) == 20.0
    assert determinism.deadline(20.0, now=100.0) == 120.0
    determinism.timed_out("hybrid_astar")  # off: nothing happens, the caller's timeout stands
    assert determinism.broken() == []


def test_on_budgets_are_the_wall_cap_and_a_timeout_ends_the_run():
    determinism.configure(True)
    assert determinism.enabled() is True
    assert determinism.budget(20.0) == determinism.WALL_CAP_S == 300.0
    assert determinism.budget(18.0) == 300.0
    assert determinism.deadline(20.0, now=100.0) == 400.0
    with pytest.raises(determinism.DeterministicBroken, match="wall_cap:hybrid_astar"):
        determinism.timed_out("hybrid_astar")
    assert determinism.broken() == ["wall_cap:hybrid_astar"]


def test_marks_are_kept_in_order_and_cleared_by_configure():
    determinism.configure(True)
    determinism.mark("learned_timeout")
    determinism.mark("event_during_wait:lease_expired")
    assert determinism.broken() == ["learned_timeout", "event_during_wait:lease_expired"]
    determinism.configure(True)
    assert determinism.broken() == []


def test_mark_when_off_is_ignored():
    determinism.mark("learned_timeout")
    assert determinism.broken() == []


def test_the_worker_reads_the_switch_from_its_environment(monkeypatch):
    monkeypatch.setenv(determinism.ENV, "1")
    determinism.configure_from_env()
    assert determinism.enabled() is True
    monkeypatch.setenv(determinism.ENV, "0")
    determinism.configure_from_env()
    assert determinism.enabled() is False
