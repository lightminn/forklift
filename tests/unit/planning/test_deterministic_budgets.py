"""Deterministic-run plan D6: in a deterministic run every planning wall budget is the cap (300 s) and reaching it ends
the run (DeterministicBroken) rather than switching path; off, the budgets and timeouts are as before."""

import math
import re
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core import determinism
from forklift_core.planning import Pose2D, pallet_mission
from forklift_core.planning import plan_worker as pw
from forklift_core.planning.pallet_mission import backoff_advice

ROOT = Path(__file__).resolve().parents[3]

from test_return_turnaround import _Clock, plan  # noqa: E402


@pytest.fixture(autouse=True)
def det_off_after():
    yield
    determinism.configure(False)


def _slow_direct(monkeypatch, seconds):
    from forklift_core.planning import hybrid_astar

    clock = _Clock(0.0)
    real = pallet_mission.plan_hybrid_astar

    def slow(*args, **kwargs):
        result = real(*args, **kwargs)
        clock.now += seconds
        return result

    monkeypatch.setattr(pallet_mission, "time", clock)
    monkeypatch.setattr(hybrid_astar, "time", clock)
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", slow)
    return clock


def test_off_the_turnaround_keeps_its_18_s_limit(monkeypatch):
    _slow_direct(monkeypatch, 19.0)
    turned = plan(forward_turnaround=True, deadline=pallet_mission.SearchBudget(determinism.budget(20.0)))
    assert turned.turnaround["timed_out"]


def test_on_the_turnaround_limit_is_the_cap(monkeypatch):
    determinism.configure(True)
    _slow_direct(monkeypatch, 19.0)
    turned = plan(forward_turnaround=True, deadline=pallet_mission.SearchBudget(determinism.budget(20.0)))
    assert not turned.turnaround["timed_out"]
    assert determinism.broken() == []


def test_on_a_turnaround_past_the_cap_ends_the_run(monkeypatch):
    determinism.configure(True)
    _slow_direct(monkeypatch, 301.0)
    with pytest.raises(determinism.DeterministicBroken, match="wall_cap"):  # the direct plan or the turnaround
        plan(forward_turnaround=True, deadline=pallet_mission.SearchBudget(determinism.budget(20.0)))


def test_a_search_past_its_deadline_times_out_off_and_ends_the_run_on():
    late = time.monotonic() - 1.0
    assert plan(deadline=late).status == "timeout"
    determinism.configure(True)
    with pytest.raises(determinism.DeterministicBroken, match="wall_cap"):
        plan(deadline=late)


def test_the_backoff_advice_past_its_deadline():
    free = lambda pose, clearance: True  # noqa: E731
    start = Pose2D(0.0, 0.0, 0.0)
    assert backoff_advice(free, 0.1, start, deadline=0.0, clock=lambda: 1.0) == {"skipped": "deadline"}
    determinism.configure(True)
    with pytest.raises(determinism.DeterministicBroken, match="backoff_advice"):
        backoff_advice(free, 0.1, start, deadline=0.0, clock=lambda: 1.0)


def test_the_observe_ladder_past_its_deadline(monkeypatch):
    monkeypatch.setattr("forklift_core.planning.pallet_mission.plan_observation_leg",
                        lambda *a, **k: SimpleNamespace(success=False, status="no_path", analytic_expansion_interval=4,
                                                        search_attempts=[]))
    payload = {"world": SimpleNamespace(start_rear=SimpleNamespace(x_m=0.0, y_m=0.0, yaw_rad=0.0)),
               "waypoints": [(1, 0, 0)], "config": "CFG", "geometry": "GEO", "pickup_bounds": None,
               "grid_kwargs": {}, "deadline": time.monotonic() - 1.0}
    assert pw._run("observe_ladder", dict(payload))["timed_out"]
    determinism.configure(True)
    with pytest.raises(determinism.DeterministicBroken, match="observe_ladder"):
        pw._run("observe_ladder", dict(payload))


def test_no_planning_wall_budget_is_a_bare_literal_any_more():
    """Every planning budget goes through determinism.budget / deadline (core and runner)."""
    for rel in ("src/forklift_core/mission/core.py", "sim/isaac/run_transport.py"):
        text = (ROOT / rel).read_text()
        assert not re.search(r"(time\.monotonic\(\)|hooks\.wall\(\))\s*\+\s*(?=\S)(?!determinism)[^\n]*20\.0", text), rel
        # the obstacle replan's shared budget: the core's wall() alias and its replan_start (Codex PR 1 review P2)
        assert not re.search(r"(monotonic\(\)|wall\(\)|replan_start)\s*\+\s*[0-9]", text), rel
        assert "SearchBudget(20.0)" not in text, rel
    assert "time.monotonic() + RETURN_TURNAROUND_MAX_S" not in (ROOT / "src/forklift_core/planning/pallet_mission.py").read_text()
    assert math.isclose(determinism.budget(20.0), 20.0)
