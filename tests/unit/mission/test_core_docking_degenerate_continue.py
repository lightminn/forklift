"""docs/plans/2026-11-04-line2-delivery-robustness.md V1: a first non-retry round-1 match refused by degenerate at
every start goes on to the round-2 stop on the SLAM goal (--docking-degenerate-continue on), every other refusal and
every check failure keeps its earlier branch, and off changes nothing (records included)."""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.localization.scan_docking import DockingResult

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_core_docking_helpers import docking_world  # noqa: E402


def degenerate_dock(w, *, reasons=("degenerate",) * 7, accepted=False, goal=(9.05, 0.02, 0.01)):
    """The matcher as a stand-in: refused with these per-start reasons, or accepted."""
    def dock(reference, live, **k):
        w.log("dock", len(live), **k)
        if accepted:
            return SimpleNamespace(accepted=True, reason="ok", correction=[0.0], starts=[(True, "ok", (0.0, 0.0, 0.0))],
                                   start_details=[{"iterations": 3}], goal_estimate=goal, relative_rear=(-1.0, 0.0, 0.0))
        return SimpleNamespace(accepted=False, reason=f"start_rejected:{reasons[0]}", correction=None,
                               starts=[(False, r, (0.0, 0.0, 0.0)) for r in reasons],
                               start_details=[{"min_information": 49.6}] * len(reasons), goal_estimate=None,
                               relative_rear=None)
    w.hooks.dock = dock


def straight_goal(w):
    return [c for c in w.calls if c[0] == "straight"][0][1][1]


def world(tmp_path, monkeypatch, *, on=True, inject=False, **kw):
    w = docking_world(tmp_path, monkeypatch, **kw)
    if on:
        w.core.options.docking_degenerate_continue = "on"
    if inject:
        w.core.options.inject_round1_degenerate = True
    w.core.hooks.unaligned_delivery_refusal = lambda: w.log("anchor_check") or None
    return w


def test_off_keeps_the_fallback_and_its_record(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch, on=False)
    degenerate_dock(w)
    w.core.dock_at_delivery_straight(12.0)
    d = w.slam["docking"]
    assert d["status"] == "fallback" and "stop_check" not in w.names() and "prefix" not in w.names()
    assert "degenerate_continued" not in d and set(d["rounds"][0]) .isdisjoint(
        {"start_verdicts", "degenerate_continue", "chosen", "dry_run"})
    assert w.core.paths["transport"].length_m == pytest.approx(1.0)  # the whole straight to the SLAM goal


def test_all_degenerate_goes_on_to_the_round_two_stop_on_the_slam_goal(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch)
    degenerate_dock(w)
    w.core.dock_at_delivery_straight(12.0)
    d = w.slam["docking"]
    assert w.names() == ["dock", "save_npz", "anchor_check", "straight", "rollout", "stop_check", "write_paths",
                         "show", "prefix"]
    assert (d["status"], d["round"], d["keep_m"]) == ("armed", 2, 0.6)
    assert not d.get("accepted_any") and d["degenerate_continued"] is True and d["previous_goal"] == [9.0, 0.0, 0.0]
    r = d["rounds"][0]
    assert r["start_verdicts"] == {"degenerate": 7} and r["chosen"] == "round2_stop"
    assert r["degenerate_continue"] == {"eligible": True, "stop_check": True}
    assert r["prefix_dry_run"] == {"status": "arrived"} and r["dry_run"]["status"] == "arrived"
    assert r["straight_refused"] is False and r["offsets_to_new_line"] == {"lateral_m": 0.02}
    assert straight_goal(w) == (9.0, 0.0, 0.0)  # the straight to the SLAM goal
    assert w.core.trackers["transport"].config.position_tolerance_m == 0.03  # the round-2 prefix's stop


def test_a_mixed_refusal_keeps_the_fallback(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch)
    degenerate_dock(w, reasons=("degenerate",) * 6 + ("large_residual",))
    w.core.dock_at_delivery_straight(12.0)
    d = w.slam["docking"]
    assert d["status"] == "fallback" and "stop_check" not in w.names() and "degenerate_continued" not in d
    assert d["rounds"][0]["degenerate_continue"] == {"eligible": False} and d["rounds"][0]["chosen"] == "straight"
    assert d["rounds"][0]["start_verdicts"] == {"degenerate": 6, "large_residual": 1}


@pytest.mark.parametrize("refusal", ["straight", "dry_run", "swept"])
def test_a_refused_whole_straight_retries_as_before(tmp_path, monkeypatch, refusal):
    w = world(tmp_path, monkeypatch, straight=refusal != "straight",
              dry_status="failed" if refusal == "dry_run" else "arrived", swept=refusal != "swept")
    degenerate_dock(w)
    w.core.dock_at_delivery_straight(12.0)
    d = w.slam["docking"]
    assert w.names()[-1] == "retry" and w.calls[-1][1] == ("b", "docking_unaligned") and "stop_check" not in w.names()
    assert d["rounds"][0]["chosen"] == "retry_b" and "degenerate_continued" not in d


def test_a_refused_stop_drives_the_checked_whole_straight(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch, stop_ok=False)
    degenerate_dock(w)
    w.core.dock_at_delivery_straight(12.0)
    d = w.slam["docking"]
    assert d["status"] == "fallback" and "prefix" not in w.names() and "retry" not in w.names()
    assert d["rounds"][0]["degenerate_continue"]["stop_check"] is False and d["rounds"][0]["chosen"] == "straight"
    assert w.core.paths["transport"].length_m == pytest.approx(1.0) and "degenerate_continued" not in d


def test_the_first_match_after_a_retry_still_must_be_accepted(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch, docking={"await_retry_match": True, "retry_seed_goal": [9.02, 0.0, 0.0]})
    degenerate_dock(w)
    with pytest.raises(RuntimeError, match="docking_retry_match_refused:start_rejected:degenerate"):
        w.core.dock_at_delivery_straight(30.0)
    assert w.slam["docking"]["rounds"][-1]["degenerate_continue"] == {"eligible": False}


def round_two(tmp_path, monkeypatch, **kw):
    return world(tmp_path, monkeypatch, docking={"round": 2, "keep_m": 0.6, "previous_goal": [9.0, 0.0, 0.0],
                                                 "degenerate_continued": True}, **kw)


def test_round_two_after_the_continue_corrects_when_accepted(tmp_path, monkeypatch):
    w = round_two(tmp_path, monkeypatch)
    degenerate_dock(w, accepted=True)
    w.core.dock_at_delivery_straight(20.0)
    d = w.slam["docking"]
    assert d["status"] == "done" and d["accepted_any"] is True and d["previous_goal"] == [9.05, 0.02, 0.01]
    assert straight_goal(w) == pytest.approx((9.05, 0.02, 0.01)) and "stop_check" not in w.names()
    assert d["rounds"][-1]["degenerate_continue"] == {"eligible": False} and d["rounds"][-1]["chosen"] == "straight"


def test_round_two_refused_after_the_continue_checks_the_rest_on_the_slam_goal(tmp_path, monkeypatch):
    w = round_two(tmp_path, monkeypatch)
    degenerate_dock(w)
    w.core.dock_at_delivery_straight(20.0)
    d = w.slam["docking"]
    assert d["status"] == "fallback" and not d.get("accepted_any") and "anchor_check" in w.names()
    assert straight_goal(w) == (9.0, 0.0, 0.0) and "prefix" not in w.names()  # no third round
    w = round_two(tmp_path, monkeypatch, straight=False)
    degenerate_dock(w)
    w.core.dock_at_delivery_straight(20.0)
    assert w.names()[-1] == "retry" and w.slam["docking"]["rounds"][-1]["chosen"] == "retry_b"


def real_result(accepted=True):
    return DockingResult(accepted, "ok" if accepted else "not_converged", (9.05, 0.02, 0.01) if accepted else None,
                         (-1.0, 0.0, 0.0) if accepted else None, (0.05, 0.02, 0.01) if accepted else None,
                         tuple((accepted, "ok", (0.0, 0.0, 0.0)) for _ in range(7)), ({"min_information": 55.0},) * 7)


def test_the_injection_refuses_the_first_round_once_and_keeps_what_the_matcher_said(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch, inject=True)
    w.hooks.dock = lambda reference, live, **k: w.log("dock") or real_result()
    w.core.dock_at_delivery_straight(12.0)
    d = w.slam["docking"]
    assert d["round1_injected"] == {"accepted": True, "reason": "ok", "start_reasons": ["ok"] * 7}
    assert d["rounds"][0]["accepted"] is False and d["rounds"][0]["reason"] == "start_rejected:degenerate"
    assert d["rounds"][0]["start_verdicts"] == {"degenerate": 7} and d["status"] == "armed" and d["round"] == 2
    assert d["goal_error_vs_truth"] is None
    w.core.dock_at_delivery_straight(20.0)  # round 2: the real result, accepted
    assert w.slam["docking"]["rounds"][-1]["accepted"] is True and w.slam["docking"]["accepted_any"] is True


def test_the_injection_skips_a_retry_match_and_off_injects_into_the_fallback(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch, inject=True, docking={"await_retry_match": True,
                                                           "retry_seed_goal": [9.02, 0.0, 0.0]})
    w.hooks.dock = lambda reference, live, **k: w.log("dock") or real_result()
    w.core.dock_at_delivery_straight(30.0)
    assert "round1_injected" not in w.slam["docking"] and w.slam["docking"]["rounds"][0]["accepted"] is True
    w = world(tmp_path, monkeypatch, on=False, inject=True)  # the off arm of the same injection
    w.hooks.dock = lambda reference, live, **k: w.log("dock") or real_result()
    w.core.dock_at_delivery_straight(12.0)
    d = w.slam["docking"]
    assert d["status"] == "fallback" and "round1_injected" in d and "stop_check" not in w.names()


def test_off_and_no_injection_leave_the_record_as_before(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch, on=False)
    w.core.dock_at_delivery_straight(12.0)  # an accepted first round, as always
    d = w.slam["docking"]
    assert "round1_injected" not in d and "degenerate_continued" not in d
    assert set(d["rounds"][0]).isdisjoint({"start_verdicts", "degenerate_continue", "chosen", "dry_run"})


def test_round_two_accepted_with_a_refused_straight_retries_from_the_committed_goal(tmp_path, monkeypatch):
    w = round_two(tmp_path, monkeypatch, straight=False)
    w.slam["docking"]["dry_run"] = {"status": "arrived", "swept_clear": True}  # the first round's, left at the top
    degenerate_dock(w, accepted=True)
    w.core.dock_at_delivery_straight(20.0)
    d = w.slam["docking"]
    assert w.names()[-1] == "retry" and w.calls[-1][1] == ("b", "docking_unaligned")
    assert d["previous_goal"] == [9.05, 0.02, 0.01] and d["goal_frame"] == "held" and d["accepted_any"] is True
    r = d["rounds"][-1]
    assert r["chosen"] == "retry_b" and r["straight_refused"] is True
    assert r["dry_run"] is None and r["prefix_dry_run"] is None  # not the first round's carried-over check


def test_an_accepted_first_round_keeps_its_own_stop_check_when_on(tmp_path, monkeypatch):
    w = world(tmp_path, monkeypatch)  # on, an accepted first match
    w.core.dock_at_delivery_straight(12.0)
    r = w.slam["docking"]["rounds"][0]
    assert r["chosen"] == "round2_stop" and r["prefix_dry_run"] == {"status": "arrived"}
    assert r["degenerate_continue"] == {"eligible": False}
