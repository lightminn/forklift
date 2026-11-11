"""The latency test B's judgement (docs/plans/2026-11-11-latency-tolerance.md 4.6): exposure joined to the mission's
plan sites, the stop after a failed mission, the pass rule over the 12 judged sessions. Synthetic records."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("lat_b_analyze", ROOT / "docs/validation/2026-11-11-latency/lat_b_analyze.py")
B = importlib.util.module_from_spec(spec)
spec.loader.exec_module(B)


def test_exposure_needs_three_delayed_answers_with_prepare_and_dock():
    res = {"latency_injection": {"plan": {"records": [
        {"job": 1, "delivered": 1.0, "injected_s": 0.05}, {"job": 2, "delivered": 2.0, "injected_s": 0.06},
        {"job": 3, "delivered": 3.0, "injected_s": 0.05}, {"job": 4, "delivered": None, "held_s": 0.01}]}}}
    state = {"async_plans": [{"job": 1, "site": "prepare"}, {"job": 2, "site": "observe"}, {"job": 3, "site": "dock"}]}
    e = B.exposure_of(res, state)
    assert e["ok"] and e["delivered"] == 3 and e["held_dropped_or_cut"] == 1 and e["sites"] == ["dock", "observe", "prepare"]
    state["async_plans"][2]["site"] = "release"
    assert not B.exposure_of(res, state)["ok"]


def rows_(n):  # [t, rear(3), truth(3)] at 120 Hz, standing
    return [[k / 120, 0, 0, 0, 1.0, 2.0, 0.0] for k in range(n)]


def test_the_stop_after_a_failed_mission():
    assert B.stop_after(rows_(120), 0.0) == "safe_failure"
    assert B.stop_after(rows_(50), 0.0) == "observed_end"  # 0.41 s
    moving = [[k / 120, 0, 0, 0, 0.002 * k, 0, 0] for k in range(120)]  # 0.24 m/s
    assert B.stop_after(moving, 0.0) == "observed_end"
    assert B.stop_after(rows_(120), None) == "observed_end"


def srow(d, done=True, viol=0, ok=True, invalid=(), eff=True):
    return {"delay_ms": d, "mission_done": done, "safety": {"violations": viol}, "exposure": {"ok": ok},
            "invalid": list(invalid), "effective": eff, "stop": None if done else "safe_failure"}


def test_b_passes_only_with_all_twelve_safe_done_and_exposed():
    rows = {f"s{d}_{i}": srow(d) for d in (0, 25, 50, 100, 500) for i in range(3)}
    assert B.judge(rows)["decision"] == "B passes"
    rows["s500_0"] = srow(500, done=False)  # 500 ms is reported only
    assert B.judge(rows)["decision"] == "B passes"
    rows["s50_1"] = srow(50, done=False)
    r = B.judge(rows)
    assert r["decision"] == "B does not pass" and "s50_1" in r["defects"]
    rows["s50_1"] = srow(50)
    rows["s500_1"] = srow(500, viol=1)  # a safety event anywhere is listed
    assert B.judge(rows)["dangerous"] == ["s500_1"]
    rows["s500_1"] = srow(500)
    rows["s25_2"] = srow(25, eff=False, invalid=["x"])  # replaced by no rerun: only two judged at 25
    assert B.judge(rows)["decision"] == "to the lead"
