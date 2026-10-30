"""The search ladder's tail attempt (docs/validation/2026-10-31-invalid-start-diagnosis.md 2(b), lead 2026-10-31): after
every attempt of the ladder ran out, once more on the caller's position lattice with a 5 deg heading lattice and four
times the budget, under its own wall cap. The same footprint, clearance and collision checks; a search the earlier
ladder solved never reaches it, and a failed tail leaves the ladder's own failure as the result."""

import math
import time
from dataclasses import replace

import numpy as np
import pytest

from forklift_core import determinism
from forklift_core.planning import pallet_mission


def config():
    return pallet_mission.make_transport_planner_config(curvature_limit_inv_m=0.36, clearance_m=0.10,
                                                        max_expansions=30000)


def result(ok, status="expansion_limit"):
    return pallet_mission.PlanResult(ok, "success" if ok else status, np.zeros((2 if ok else 0, 3)),
                                     np.zeros(2 if ok else 0, np.int8), np.zeros(2 if ok else 0), 0.0, 100)


def stub(received, answer):
    def plan(*args, **kwargs):
        received.append((args[-1], kwargs.get("deadline")))
        return answer(args[-1], len(received))
    return plan


def test_the_tail_runs_last_with_only_the_heading_lattice_and_the_budget_changed(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", stub(received, lambda c, n: result(False, "no_path")))
    base = config()
    out = pallet_mission._search(None, None, [], None, None, base)
    tail, deadline = received[-1]
    assert len(received) == 7  # the ladder (its closed fine-lattice budget step skipped after no_path), then the tail
    assert tail.yaw_resolution_rad == pytest.approx(math.radians(5)) and tail.xy_resolution_m == base.xy_resolution_m
    assert tail.max_expansions == pallet_mission.FALLBACK_TAIL_BUDGET_FACTOR * base.max_expansions
    # the same judgement: footprint, clearance, collision step, primitive, curvature, penalties all the caller's
    assert replace(tail, yaw_resolution_rad=base.yaw_resolution_rad, max_expansions=base.max_expansions) == base
    assert deadline is not None and deadline <= time.monotonic() + pallet_mission.FALLBACK_TAIL_WALL_S
    # a failed tail: the result is the ladder's own failure; the tail is in the attempts
    assert out.status == "no_path" and len(out.search_attempts) == 7
    assert out.search_attempts[-1][1] == "no_path" and out.search_attempts[-1][4] == pytest.approx(math.radians(5))


def test_a_search_the_ladder_solved_never_reaches_the_tail(monkeypatch):
    for solved_at in range(1, 8):
        received = []
        monkeypatch.setattr(pallet_mission, "plan_hybrid_astar",
                            stub(received, lambda c, n, k=solved_at: result(n >= k)))
        out = pallet_mission._search(None, None, [], None, None, config())
        assert out.success and len(received) == solved_at and all(d is None for _, d in received)


def test_a_tail_success_is_taken(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", stub(received, lambda c, n: result(n == 8)))
    out = pallet_mission._search(None, None, [], None, None, config())
    assert out.success and len(received) == 8 and len(out.search_attempts) == 8


def test_the_tail_is_left_out_without_the_extended_ladder_or_another_status(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", stub(received, lambda c, n: result(False)))
    pallet_mission._search(None, None, [], None, None, config(), extended=False)
    assert len(received) == 5
    received.clear()
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar",
                        stub(received, lambda c, n: result(False, "invalid_start")))
    pallet_mission._search(None, None, [], None, None, config())
    assert len(received) == 1  # an invalid start is not a search that ran out


def test_a_budgeted_ladder_has_no_tail(monkeypatch):
    """Codex review P2 2-3: a caller that gives a deadline (a SearchBudget, or one shared by several candidates, as the
    observe ladder does) keeps the ladder it had -- a tail there would spend the time its next candidate needs and make
    a deterministic run's result depend on the clock. The tail is for the searches without one (the release replans)."""
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", stub(received, lambda c, n: result(False)))
    out = pallet_mission._search(None, None, [], None, None, config(), deadline=pallet_mission.SearchBudget(1000.0))
    assert len(received) == 7 and out.status == "expansion_limit"
    received.clear()
    out = pallet_mission._search(None, None, [], None, None, config(), deadline=time.monotonic() + 1000.0)
    assert len(received) == 7 and out.status == "expansion_limit"


def test_a_tail_success_returned_after_its_cap_is_not_taken(monkeypatch):
    """Codex review P2 1: the search checks the clock every 256 expansions, so a goal connection can come back past the
    tail's cap -- off, that result is a failed tail (the ladder's failure stands); deterministic, the run ends (D6)."""
    clock = {"t": 0.0}
    monkeypatch.setattr(pallet_mission.time, "monotonic", lambda: clock["t"])

    def late(c, n):
        if n == 8:
            clock["t"] += pallet_mission.FALLBACK_TAIL_WALL_S + 1.0  # the tail comes back a second past its cap
            return result(True)
        return result(False)

    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", stub(received, late))
    out = pallet_mission._search(None, None, [], None, None, config())
    assert len(received) == 8 and not out.success and out.status == "expansion_limit"
    clock["t"], received[:] = 0.0, []
    determinism.configure(True)
    try:
        monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", stub(received, lambda c, n: (
            clock.__setitem__("t", clock["t"] + determinism.WALL_CAP_S + 1.0) or result(True)) if n == 8 else result(False)))
        with pytest.raises(determinism.DeterministicBroken):
            pallet_mission._search(None, None, [], None, None, config())
    finally:
        determinism.configure(False)


def test_a_deterministic_run_caps_the_tail_with_the_deterministic_budget(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", stub(received, lambda c, n: result(False)))
    determinism.configure(True)
    try:
        cap = determinism.budget(pallet_mission.FALLBACK_TAIL_WALL_S)
        started = time.monotonic()
        pallet_mission._search(None, None, [], None, None, config())
    finally:
        determinism.configure(False)
    assert cap == determinism.WALL_CAP_S and started + cap <= received[-1][1] <= time.monotonic() + cap
