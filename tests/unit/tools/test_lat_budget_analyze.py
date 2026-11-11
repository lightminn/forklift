"""The layer latency budget check's judgement (docs/plans/2026-11-11-layer-latency-budget.md 3): the budget each arm
must carry, the watchdog record, Uo9's all-must-hold verdict (the end on the first watchdog tick past the limit with a
safe stop, unexposed when the limit was never reached, unreadable fails) and the conclusion rules. Synthetic records,
no Isaac."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "lat_budget_analyze", ROOT / "docs/validation/2026-11-11-latency/lat_budget_analyze.py")
G = importlib.util.module_from_spec(spec)
spec.loader.exec_module(G)


def budgeted(b=0.10, events=1, longest=0.3, end=1.0, inv=0, reason=None, **k):
    r = {"success": reason is None, "failure_reason": reason,
         "layer_budget": {"budget_s": b, "overdue_end_s": end, "source": "x", "invariant_violations": inv,
                          "watchdog": {"events": events, "ticks": 10, "longest_s": longest}, "d_max_s": 0.2,
                          "scans_used": 100}}
    r.update(k)
    return r


def test_each_arm_must_carry_its_budget_and_its_records(tmp_path):
    (tmp_path / "layer_latency.json").write_text("{}")
    (tmp_path / "layer_worker.json").write_text("[]")
    assert G.budget_problems("B0", {"success": True}, tmp_path) == []
    assert G.budget_problems("B0", budgeted(), tmp_path) == ["layer budget 0.1, not None"]
    assert G.budget_problems("Bb10", budgeted(), tmp_path) == []
    assert G.budget_problems("Bb10", {"success": True}, tmp_path)[0] == "layer budget None, not 0.1"
    assert G.budget_problems("Uo9", budgeted(), tmp_path) == ["layer budget 0.1, not 0.05", "no latency_injection record"]
    assert G.budget_problems("Uo9", budgeted(0.05, latency_injection={}), tmp_path) == []
    (tmp_path / "layer_latency.json").unlink()
    assert "no layer_latency.json" in G.budget_problems("Ub6", budgeted(latency_injection={}), tmp_path)


def ticks(*reasons):
    return {"overdue_ticks": [[i / 120, 0.0, None, None, "deadline", r, i / 120] for i, r in enumerate(reasons)]}


def uo9_row(result, log, violations=0, stop="safe_failure"):
    return {"safety": {"violations": violations}, "stop": stop, "watchdog": G.watchdog_of(result, log)}


def test_uo9_passes_on_the_end_at_the_first_tick_past_the_limit_with_a_safe_stop():
    res = budgeted(0.05, longest=1.004, reason=G.END_REASON)
    log = ticks("layer_overdue", "layer_overdue", G.END_REASON)
    v = G.uo9_verdict(uo9_row(res, log))
    assert v["pass"] and v["end_function"] == "exposed"
    assert not G.uo9_verdict(uo9_row(res, log, stop="observed_end"))["pass"]  # the end must be a safe stop
    assert not G.uo9_verdict(uo9_row(res, ticks("layer_overdue", G.END_REASON, G.END_REASON)))["pass"]  # drove on
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=1.004, reason="tracking"), log))["pass"]  # other end
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=1.004, reason=G.END_REASON, inv=1), log))["pass"]
    assert not G.uo9_verdict(uo9_row(res, log, violations=1))["pass"]


def test_uo9_without_reaching_the_limit_is_unexposed_and_without_events_fails():
    v = G.uo9_verdict(uo9_row(budgeted(0.05, longest=0.4), ticks("layer_overdue")))
    assert v["pass"] and v["end_function"] == "unexposed"
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, events=0, longest=0.0), ticks()))["pass"]
    unread = budgeted(0.05)
    del unread["layer_budget"]["watchdog"]
    v = G.uo9_verdict(uo9_row(unread, None))
    assert not v["pass"] and v["end_function"] == "unreadable"


def crow(arm, cls, scene="line_2", violations=0, **k):
    r = {"arm": arm, "scene": scene, "seed": 0, "effective": True, "invalid": [], "repeat": False,
         "safety": {"violations": violations}, "cell": {"class": cls}}
    r.update(k)
    return r


def test_the_conclusion_rules():
    rows = {"a": crow("Ub3", "tolerated"), "b": crow("Ub12", "safe_failure"), "c": crow("Bb10", "operating_degraded"),
            "d": crow("Uo9", "safe_failure", uo9={"pass": True, "checks": {}}),
            "e": crow("Uo9", "safe_failure", scene="random_1", uo9={"pass": False, "checks": {"invariant_0": False}}),
            "f": crow("B0", "tolerated", violations=1), "g": crow("Ub6", "observed_end", scene="aisle_rack"),
            "h": crow("Ub6", "safe_failure", scene="aisle_rack", cell={"class": "safe_failure",
                                                                         "attribution": "baseline_failed"})}
    c = G.conclusions(rows, {"line_2": True, "random_1": True})
    assert [(d["run"], d["priority"]) for d in c["defects"]] == [("f", 1), ("e", 2), ("g", 2), ("b", 3)]
    assert c["bb10_to_lead"] == [{"run": "c", "scene": "line_2", "seed": 0, "class": "operating_degraded"}]
    assert set(c["uo9"]) == {"d", "e"}
    c = G.conclusions(rows, {"line_2": False, "random_1": True})  # line_2 unread, safety still listed
    assert [d["run"] for d in c["defects"]] == ["f", "e", "g"] and ["line_2", "Ub12"] in c["unread"]
