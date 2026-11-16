"""The observation move's paired comparison (tools/reloc_move_compare.py, plan 2026-11-16-reloc-observe-move 2)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import reloc_move_compare as M  # noqa: E402


def row(i, outcome, pose_class=None, cls="valid"):
    return {"id": i, "class": cls, "outcome": outcome, "pose_class": pose_class}


def moved(static="needs_operator", phases=("move+",), blocked=(), wall=4.0, late=False):
    return {"static_outcome": static,
            "moves": [{"dir": 1, "blocked": 1 in blocked, "wall_s": wall, "stopped": False},
                      {"dir": -1, "blocked": -1 in blocked, "wall_s": 0.0 if -1 in blocked else wall,
                       "stopped": False}],
            "attempts": [{"phase": "static", "wall_s": 9.0}] * 3
            + [{"phase": p, "wall_s": 5.0, "reason": "deadline" if late else "confirmed"} for p in phases]}


def test_a_static_success_is_both_arms_and_a_rescue_is_counted_once():
    rows = [row("s1", "success", "success"), row("s2", "success", "success"), row("s3", "needs_operator"),
            row("s4", "success", "success"), row("x", "not_run", cls="not_run"), row("v", "invalid", cls="invalid")]
    recs = {"s1": {"attempts": [], "moves": []}, "s2": moved(), "s3": moved(phases=("move+", "move+", "move-", "move-")),
            "s4": moved(static="failed", blocked=(1,), phases=("move-",))}
    out = M.compare(rows, recs)
    assert out["valid"] == 4 and out["static_failed"] == 3
    assert (out["a_success"], out["b_success"], out["rescued"], out["worse"]) == (1, 3, 2, 0)
    assert out["mcnemar_p_one_sided"] == pytest.approx(0.25)
    assert out["rescue_rate"] == pytest.approx(2 / 3) and out["run_0_6"] is False  # b = 2 < 3
    p3 = next(p for p in out["pairs"] if p["id"] == "s3")
    assert p3["times"]["places"]["move-"]["scan_sets"] == 2 and p3["times"]["added_s"] == pytest.approx(28.0)
    p4 = next(p for p in out["pairs"] if p["id"] == "s4")
    assert p4["times"]["blocked"] == [1]


def test_a_wrong_confirmation_rejects_and_three_rescues_open_0_6():
    rows = [row(f"r{i}", "success", "success") for i in range(3)] + [row("w", "wrong", "wrong")]
    recs = {f"r{i}": moved() for i in range(3)} | {"w": moved()}
    out = M.compare(rows, recs)
    assert out["rescued"] == 3 and out["b_wrong"] == 1 and out["rejected_for_safety"]
    assert out["run_0_6"] is False
    out2 = M.compare(rows[:3], recs)
    assert out2["run_0_6"] and out2["mcnemar_p_one_sided"] == pytest.approx(1 / 8)


def test_deadlines_and_both_blocked_are_reported():
    out = M.compare([row("a", "needs_operator")], {"a": moved(blocked=(1, -1), phases=())})
    assert out["both_blocked"] == 1 and out["rescued"] == 0 and out["mcnemar_p_one_sided"] == 1.0
    out = M.compare([row("a", "needs_operator")], {"a": moved(late=True)})
    assert out["pairs"][0]["times"]["places"]["move+"]["deadline_hit"]


def test_the_two_sided_interval():
    lo, hi = M.cp_two_sided(2, 10)
    assert lo == pytest.approx(0.0252, abs=5e-4) and hi == pytest.approx(0.5561, abs=5e-4)
    assert M.cp_two_sided(0, 5)[0] == 0.0 and M.cp_two_sided(5, 5)[1] == 1.0 and M.cp_two_sided(0, 0) is None


def test_a_valid_session_without_a_move_on_record_stops_the_comparison():
    with pytest.raises(ValueError, match="without a move-on"):
        M.compare([row("a", "success", "success")], {"a": {"attempts": []}})
    out = M.compare([row("a", "not_started")], {})  # no start_reloc at all: both arms fail it, listed
    assert out["no_reloc"] == ["a"] and out["rescued"] == 0 and out["a_success"] == out["b_success"] == 0
    with pytest.raises(ValueError, match="disagree"):
        M.compare([row("a", "success", "success")], {"a": {"moves": [], "static_outcome": "failed"}})
