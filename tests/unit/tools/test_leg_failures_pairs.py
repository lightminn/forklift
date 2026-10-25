"""The V4a seed-pair test (docs/plans/2026-10-25-leg-failures.md V4a): the one-sided signed-rank p by enumerating
every sign combination over average ranks, zero differences dropped."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

from leg_failures_pairs import judge, pairs_from_rows, signed_rank_p  # noqa: E402


def test_ties_take_average_ranks_over_every_sign_combination():
    # Codex plan round 4 P2: scipy's method="exact" gives 0.046875 here; the average-rank enumeration 0.0625
    assert signed_rank_p([0.5, 0.5, -0.5, 1, 1, 1]) == pytest.approx(4 / 64)


def test_without_ties_it_is_the_exact_distribution_and_zeros_are_dropped():
    assert signed_rank_p([1, 2, 3, 4, 5, 6]) == pytest.approx(1 / 64)
    assert signed_rank_p([0, 0, 1, 2, 3, 4, 5, 6]) == pytest.approx(1 / 64)
    assert signed_rank_p([-1, 2, 3, 4, 5, 6]) == pytest.approx(2 / 64)
    assert signed_rank_p([]) == 1.0


def test_the_judgement_rules():
    # pair: (main rate or None when no attempt, fix rate or None, main refusals, fix refusals)
    six_better = [(0.0, 1.0, 0, 0)] * 6
    assert judge(six_better)["verdict"] == "pass"
    # fewer than six usable pairs (a pair with no attempt on one side is not used): undecidable
    assert judge([(0.0, 1.0, 0, 0)] * 5 + [(None, 1.0, 0, 0)])["verdict"] == "undecidable"
    # more than two pairs worse
    worse = [(0.0, 1.0, 0, 0)] * 7 + [(1.0, 0.5, 0, 0)] * 3
    assert judge(worse)["verdict"] == "fail" and judge(worse)["worse_pairs"] == 3
    # refusals must not increase
    assert judge([(0.0, 1.0, 0, 1)] * 6)["verdict"] == "fail"
    # p >= 0.05: not adopted (zero differences count as usable pairs but leave the test)
    assert judge([(0.5, 1.0, 0, 0)] * 2 + [(0.5, 0.5, 0, 0)] * 6)["verdict"] == "fail"
    out = judge([(0.0, 1.0, 0, 0)] * 3 + [(1.0, 0.5, 0, 0)] * 2 + [(0.5, 0.5, 0, 0)] * 3)
    assert out["verdict"] == "fail" and out["usable_pairs"] == 8 and out["p"] >= 0.05


def test_a_session_counts_its_counted_reapproaches_by_the_docking_round_in_each_window(tmp_path):
    import json

    from leg_failures_pairs import session_counts

    md = tmp_path / "run" / "live/epoch_1/run/missions/m1"
    md.mkdir(parents=True)
    retries = [{"trigger": "a", "time_s": 90.0, "counted": False},
               {"trigger": "b", "time_s": 100.0, "counted": True, "plan": {"length_m": 2.0}},  # stood: no round
               {"trigger": "obstacle_replan", "time_s": 105.0, "counted": False, "plan": {"length_m": 2.0}},
               {"trigger": "d", "time_s": 110.0, "counted": True, "plan": {"length_m": 2.0}},  # arrived: round 120
               {"trigger": "b", "time_s": 130.0, "counted": True, "plan": {"refused": "dry_run"}}]
    (md / "records.json").write_text(json.dumps({"state": {"docking_retries": retries, "delivery_docking": {
        "rounds": [{"time_s": 95.0}, {"time_s": 120.0}]}}}))
    (md / "result.json").write_text(json.dumps({"result": {"ended_s": 140.0}}))
    assert session_counts(str(tmp_path / "run")) == [1, 3, 1]
    assert session_counts(str(tmp_path / "missing")) == [0, 0, 0]


def test_mission_outcomes_and_arm_names_for_the_reapproach_hold_judgement(tmp_path):
    """Re-approach hold plan V4a guard 2: each arm's done missions, its transport_recovery_after_docking endings and
    its hold exits, read from run folders named by the arms given (off / on)."""
    import json

    from leg_failures_pairs import collect, mission_totals

    def mission(run, m, outcome, reason=None, retries=()):
        md = tmp_path / run / "live/epoch_1/run/missions" / m
        md.mkdir(parents=True)
        (md / "records.json").write_text(json.dumps({"state": {"docking_retries": list(retries)}}))
        (md / "result.json").write_text(json.dumps({"result": {"outcome": outcome, "reason": reason, "ended_s": 9}}))

    mission("p_off_random_m_s0", "m1", "done")
    mission("p_off_random_m_s0", "m2", "failed", "transport_recovery_after_docking")
    mission("p_on_random_m_s0", "m1", "done", retries=[{"trigger": "h", "time_s": 3.0, "counted": False}])
    mission("p_on_random_m_s0", "m2", "done")
    rows = collect(str(tmp_path), "p", "random_m", range(1), arms=("off", "on"))
    tot = mission_totals(rows)
    assert tot["main"] == {"missions": 2, "done": 1, "recovery_after_docking": 1, "hold_exits": 0}
    assert tot["fix"] == {"missions": 2, "done": 2, "recovery_after_docking": 0, "hold_exits": 1}
    assert rows[0]["fix"] == [0, 0, 0]  # an uncounted h is no attempt


def test_rates_from_counts_stay_exact_so_ties_hold():
    """Codex plan round 5 P2: 3/3 - 2/3 and 0/3 - 1/3 are the same size; in floats they were not (p 0.015625, pass)."""
    rows = [{"seed": k, "main": [2, 3, 0], "fix": [3, 3, 0]} for k in range(6)]
    rows.append({"seed": 6, "main": [1, 3, 0], "fix": [0, 3, 0]})
    out = judge(pairs_from_rows(rows))
    assert out["p"] == pytest.approx(8 / 128) and out["verdict"] == "fail"
    assert pairs_from_rows([{"seed": 0, "main": [0, 0, 1], "fix": [1, 2, 0]}])[0][:2] == (None, 0.5)
