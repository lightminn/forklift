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


def ticks(*reasons, held=None):
    held = held or [i / 120 for i in range(len(reasons))]
    return {"overdue_ticks": [[i / 120, 0.0, None, None, "deadline", r, h] for i, (r, h) in enumerate(zip(reasons, held))]}


def uo9_row(result, log, violations=0, stop="safe_failure"):
    return {"safety": {"violations": violations}, "stop": stop, "watchdog": G.watchdog_of(result, log)}


def test_uo9_passes_on_the_end_at_the_first_tick_past_the_limit_with_a_safe_stop():
    res = budgeted(0.05, longest=1.004, reason=G.END_REASON)
    log = ticks("layer_overdue", "layer_overdue", G.END_REASON, held=[0.99, 1.0, 1.004])
    v = G.uo9_verdict(uo9_row(res, log))
    assert v["pass"] and v["end_function"] == "exposed"
    assert not G.uo9_verdict(uo9_row(res, log, stop="observed_end"))["pass"]  # the end must be a safe stop
    assert not G.uo9_verdict(uo9_row(res, ticks("layer_overdue", G.END_REASON, G.END_REASON,
                                                held=[0.99, 1.004, 1.012])))["pass"]  # drove on
    # ended one tick late: the first row past the limit was still an overdue (Codex round 2 P2)
    late = ticks("layer_overdue", "layer_overdue", G.END_REASON, held=[1.0, 1.0083, 1.0167])
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=1.0167, reason=G.END_REASON), late))["pass"]
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=1.004, reason="tracking"), log))["pass"]  # other end
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=1.004, reason=G.END_REASON, inv=1), log))["pass"]
    assert not G.uo9_verdict(uo9_row(res, log, violations=1))["pass"]


def test_uo9_without_reaching_the_limit_is_unexposed_and_without_events_fails():
    v = G.uo9_verdict(uo9_row(budgeted(0.05, longest=0.4), ticks("layer_overdue", held=[0.4])))
    assert v["pass"] and v["end_function"] == "unexposed"
    # an unparsed log never passes, whatever the summary says (Codex round 2 P2)
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=0.4), None))["pass"]
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=0.4), ticks("layer_overdue", held=[1.2])))["pass"]
    nan = G.uo9_verdict(uo9_row(budgeted(0.05, longest=0.4), ticks("layer_overdue", held=[float("nan")])))
    assert not nan["pass"] and nan["end_function"] == "unreadable"  # Codex round 3 P2
    assert not G.uo9_verdict(uo9_row(budgeted(0.05, longest=float("nan")), ticks("layer_overdue", held=[0.4])))["pass"]
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


def test_exception_2_strips_the_slot_cpu_set_only_when_the_worker_does_not_pin_itself():
    from types import SimpleNamespace

    def doc(mode, cpus=None, affinity=None):
        return {"layer_worker": {"mode": mode, "cpus": cpus, "affinity": affinity, "k": 6,
                                 "process_affinity": [4, 5], "process_cores": [[[4], [5]]]}}

    docs = [doc("delay"), doc("thread", cpus=[1], affinity=[1]), doc("delay", cpus=[1])]
    dc = SimpleNamespace(_items=lambda run: {"result.json": ("json", [dict(d, layer_worker=dict(d["layer_worker"]))
                                                                      for d in docs])})
    G.strip_affinity(dc)
    got = [d["layer_worker"] for d in dc._items(None)["result.json"][1]]
    assert got[0]["process_affinity"] == {"count": 2} and got[0]["process_cores"] == {"count": 1} and got[0]["k"] == 6
    assert got[1]["process_affinity"] == [4, 5] and got[2]["process_cores"] == [[[4], [5]]]  # pinned: kept
    assert not G.affinity_exempt(None) and not G.affinity_exempt({"mode": "thread"})


def test_exception_2_keeps_the_cpu_count_so_different_sized_sets_still_differ():
    from types import SimpleNamespace

    def run(cpus):
        return {"result.json": ("json", [{"layer_worker": {"mode": "delay", "cpus": None, "affinity": None,
                                                           "process_affinity": cpus,
                                                           "process_cores": [[[c], [c]] for c in cpus]}}])}

    sets = {"a": [4, 5, 6, 16, 17, 18], "b": [0, 1, 2, 12, 13, 14], "c": [0, 1, 2, 12]}
    dc = SimpleNamespace(_items=lambda r: run(list(sets[r])))
    G.strip_affinity(dc)
    a, b, c = (dc._items(k)["result.json"][1][0]["layer_worker"] for k in "abc")
    assert a == b  # same size: equal once the numbers are gone
    assert a["process_affinity"] != c["process_affinity"]  # a different size still differs


def test_a_bb10_mission_failure_is_a_defect_and_its_degradation_goes_to_the_lead():
    rows = {"a": crow("Bb10", "observed_end"), "b": crow("Bb10", "operating_degraded"), "c": crow("Bb10", "safe_failure")}
    c = G.conclusions(rows, {"line_2": True})
    assert [(d["run"], d["priority"]) for d in c["defects"]] == [("a", 2), ("c", 3)]
    assert [d["run"] for d in c["bb10_to_lead"]] == ["a", "b", "c"]
