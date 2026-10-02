"""Observation-candidate design rules (G4 plan, docs/plans/2026-10-02-g4-observation-candidates.md)."""

import pytest

from tools import observation_candidate_design as design


class FakeEvaluator:
    """Legs and views from tables; the real evaluator's sequence logic on top."""

    def __init__(self, legs, views, approaches=None):
        self.legs, self.views = legs, views
        self.approaches = approaches or {}
        self.calls = []

    def leg(self, seed, start, candidate):
        key = (None if start is None else (start.x_m, start.y_m), candidate)
        self.calls.append(key)
        ok = self.legs.get(key, self.legs.get(candidate, False))
        return ok, "success" if ok else "invalid_goal", 1.0

    def view(self, seed, candidate):
        return self.views.get(candidate, 0)

    def approach(self, seed, candidate):
        return self.approaches.get(candidate, "success")

    sequence = design.Evaluator.sequence


A, B, C = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)


def test_the_sequence_stops_at_the_first_good_view():
    ev = FakeEvaluator({A: True, B: True}, {A: 2000, B: 2000})
    result = ev.sequence(0, [A, B], 1500)
    assert result["served"] and result["by"] == A
    assert len(result["trace"]) == 1


def test_a_poor_view_moves_on_from_where_the_truck_stands():
    ev = FakeEvaluator({A: True, B: True}, {A: 100, B: 2000})
    result = ev.sequence(0, [A, B], 1500)
    assert result["by"] == B
    # B is planned from A's pose, as the runner does after a failed detection.
    assert ev.calls[-1] == ((A[0], A[1]), B)
    assert result["travelled_m"] == 2.0


def test_unplannable_candidates_are_skipped_and_exhaustion_is_unserved():
    ev = FakeEvaluator({A: False, B: True}, {B: 100})
    result = ev.sequence(0, [A, B], 1500)
    assert not result["served"]
    assert [t["plan"] for t in result["trace"]] == ["invalid_goal", "success"]


def test_a_view_the_mission_cannot_leave_from_does_not_count():
    ev = FakeEvaluator({A: True}, {A: 5000}, {A: "approach:expansion_limit"})
    result = ev.sequence(0, [A], 1500)
    assert not result["served"]
    assert result["trace"][0]["approach"] == "approach:expansion_limit"


def test_g5_seeds_are_refused():
    with pytest.raises(ValueError, match="not for design"):
        design.Evaluator(config=None).scenario(1005)
    with pytest.raises(ValueError, match="not for design"):
        design.Evaluator(config=None).scenario(110)
    with pytest.raises(SystemExit):
        design.main(["--record", "unused.json", "--seeds", "990:1001"])


def test_greedy_choice_follows_the_fixed_rules(monkeypatch):
    """Most newly served first; ties to smaller x, then y, then yaw 0; stop at zero."""
    serves = {
        # seed -> grid candidates that serve it once appended
        1: {(0.0, 1.2, 0.0), (-0.6, 1.2, 0.0)},
        2: {(0.0, 1.2, 0.0), (-0.6, 1.2, 0.0)},
        3: {(0.4, 2.4, -0.25)},
        4: set(),
    }

    def fake_trace(job):
        seed, _, _ = job
        return seed, {"served": seed not in serves}

    def fake_serving(job):
        seed, prefix, options, _ = job
        return seed, [c for c in options if c in serves[seed]]

    monkeypatch.setattr(design, "_trace", fake_trace)
    monkeypatch.setattr(design, "_serving", fake_serving)
    chosen, rounds = design.choose([1, 2, 3, 4, 5], 1500, limit=3)
    # (-0.6, 1.2, 0) and (0.0, 1.2, 0) tie at two seeds; smaller x wins.
    assert chosen == [(-0.6, 1.2, 0.0), (0.4, 2.4, -0.25)]
    assert rounds[-1]["unserved"] == [4]


RECORDS = design.ROOT / "artifacts/20261001_g2_rerun/perception"


@pytest.mark.skipif(not RECORDS.is_dir(), reason="G2 rerun records are local artifacts")
def test_the_runner_configuration_and_scenarios_are_rebuilt_exactly():
    import dataclasses
    import json

    # Seed 0's record holds all three catalogue assets; the design run uses it.
    config = design.RunConfig.from_record(
        json.loads((RECORDS / "seed_0/result.json").read_text())
    )
    evaluator = design.Evaluator(config)
    for seed in range(9):
        record = json.loads((RECORDS / f"seed_{seed}/result.json").read_text())
        assert dataclasses.asdict(config.planner) == record["planner_config"]
        assert dataclasses.asdict(config.geometry) == record["geometry"]
        scenario = evaluator.scenario(seed)
        assert json.dumps(dataclasses.asdict(scenario), sort_keys=True) == json.dumps(
            record["scenario"], sort_keys=True
        )


def test_the_runner_appends_the_chosen_candidates_after_the_original_five():
    """Appending keeps every seed served by the first five on its old path."""
    import ast

    source = (design.ROOT / "sim/isaac/run_transport.py").read_text()
    lists = [
        node.value
        for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Assign)
        and any(ast.unparse(t) == "args.observation_waypoints" for t in node.targets)
        and isinstance(node.value, ast.List)
    ]
    assert len(lists) == 1
    waypoints = [tuple(ast.literal_eval(e)) for e in lists[0].elts]
    assert tuple(waypoints[:5]) == design.DEFAULT_CANDIDATES
    # docs/validation/2026-10-02-g4-observation-candidates.md, design seeds 200-399.
    assert waypoints[5:] == [(0.0, 2.1, -0.25), (0.4, 1.2, 0.0), (-0.6, 1.8, -0.25)]
