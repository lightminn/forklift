"""The latency test B's judgement (docs/plans/2026-11-11-latency-tolerance.md 4.1, 4.2, 4.6, 4.7): the environment
split (GPU notes, probe exits as the mission's), exposure joined to this mission's plans (d 0 from its own), the stop
after a failure, absolute safety with confirmed counts kept, the pass rule, d 500 and first runs, the submit parsing.
Synthetic records."""

import importlib.util
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("lat_b_analyze", ROOT / "docs/validation/2026-11-11-latency/lat_b_analyze.py")
B = importlib.util.module_from_spec(spec)
spec.loader.exec_module(B)


def test_the_environment_split():
    k = B.split_invalid(["a GPU process not known to be this job's: 5", "probe exit 1", "other users' CPU over 1 core",
                         "another job on the node: x"])
    assert k["environment"] == ["other users' CPU over 1 core"] and k["mission"] == ["probe exit 1"]
    assert len(k["notes"]) == 2
    assert B.split_invalid(["probe exit None", "session not started (no trace of running)"], ran=False)["mission"] == []


def test_exposure_counts_only_this_missions_answers():
    res = {"latency_injection": {"plan": {"records": [
        {"job": 1, "delivered": 1.0, "injected_s": 0.05}, {"job": 2, "delivered": 2.0, "injected_s": 0.06},
        {"job": 9, "delivered": 3.0, "injected_s": 0.05}, {"job": 3, "delivered": None, "held_s": 0.01}]}}}
    state = {"async_plans": [{"job": 1, "site": "prepare"}, {"job": 2, "site": "dock"}, {"job": 3, "site": "observe"}]}
    e = B.exposure_of(res, state, 50)
    assert not e["ok"] and e["delivered"] == 2 and e["held_dropped_or_cut"] == 1  # job 9 is not this mission's
    state["async_plans"].append({"job": 4, "site": "release"})
    res["latency_injection"]["plan"]["records"].append({"job": 4, "delivered": 4.0, "injected_s": 0.05})
    assert B.exposure_of(res, state, 50)["ok"]
    d0 = {"async_plans": [{"job": 1, "site": "prepare", "finished_s": 1.0}, {"job": 2, "site": "dock", "finished_s": 2.0},
                          {"job": 3, "site": "observe", "finished_s": 3.0}, {"job": 4, "site": "dock", "dropped": "drift"}]}
    assert B.exposure_of({}, d0, 0)["ok"]  # d 0: the mission's own taken plans


def rows_(n, dt=1 / 120):
    return [[k * dt, 0, 0, 0, 1.0, 2.0, 0.0] for k in range(n)]


def test_the_stop_after_a_failed_mission():
    assert B.stop_after(rows_(120), 0.0, [(0.0, "STOPPED")]) == "safe_failure"
    assert B.stop_after(rows_(50), 0.0, [(0.0, "STOPPED")]) == "observed_end"  # 0.41 s
    assert B.stop_after(rows_(120), 0.0, []) == "observed_end"  # no mode known
    assert B.stop_after(rows_(3, dt=1.0), 0.0, []) == "observed_end"  # standing truth but no control in between
    moving = [[k / 120, 0, 0, 0, 0.002 * k, 0, 0] for k in range(120)]
    assert B.stop_after(moving, 0.0, []) == "observed_end"
    assert B.stop_after(rows_(120), 0.0, [(0.3, "AUTO")]) == "observed_end"  # driving again after the end
    assert B.stop_after(rows_(120), None, []) == "observed_end"
    # AUTO up to 0.8 s after the end, the record ends at 1.0 s: only 0.2 s confirmable
    assert B.stop_after(rows_(121), 0.0, [(-1.0, "AUTO"), (0.8, "STOPPED")]) == "observed_end"
    assert B.stop_after(rows_(240), 0.0, [(-1.0, "AUTO"), (0.8, "STOPPED")]) == "safe_failure"  # 0.8 -> 2.0 s
    assert B.stop_after(rows_(121), 0.0, [(0.0, "STOPPED"), (1.0, "AUTO")]) == "observed_end"  # AUTO at the last row


LOW_OK = {"rows": {"truck_physical": {"min_m": 0.2}}, "physical_arc_violations": []}


def test_safety_keeps_confirmed_counts_and_reads_the_low_objects():
    s = {"obstacle_layer": {"unpermitted_entries": 0, "unpermitted_dir_entries": 0}}
    assert B.safety_of(s, {"m1": {}}, LOW_OK)["violations"] == 0
    assert B.safety_of(s, {"m1": None}, LOW_OK)["violations"] is None  # a mission's records missing
    assert B.safety_of(s, {"m1": {"forbidden_pocket_contacts": [{}]}}, {"error": "x"})["violations"] == 1
    low_bad = {"rows": {"truck_physical": {"min_m": -0.01}}, "physical_arc_violations": [{}]}
    assert B.safety_of(s, {"m1": {}}, low_bad)["violations"] == 2
    assert B.safety_of({**s, "scene_damaged": {"x": 1}}, {"m1": {}}, LOW_OK)["violations"] == 1


def srow(d, done=True, viol=0, ok=True, env=(), eff=True, stop=None):
    return {"delay_ms": d, "mission_done": done, "ran": True, "safety": {"violations": viol}, "exposure": {"ok": ok},
            "kinds": {"environment": list(env), "notes": [], "mission": []}, "effective": eff, "arm_problems": [],
            "stop": stop if not done else None, "launcher": {"RETURN_START": "boxed"}}


def test_b_passes_only_with_all_twelve_and_d500_is_reported():
    rows = {f"s{d}_{i}": srow(d) for d in (0, 25, 50, 100, 500) for i in range(3)}
    assert B.judge(rows)["decision"] == "B passes"
    rows["s500_0"] = srow(500, done=False, stop="safe_failure")
    r = B.judge(rows)
    assert r["decision"] == "B passes" and r["defects"][-1]["priority"] == 4
    rows["s500_1"] = srow(500, viol=1)  # a P1 defect, not a judged session's failure
    r = B.judge(rows)
    assert r["decision"] == "B passes" and r["defects"][0] == {"run": "s500_1", "priority": 1, "class": "dangerous",
                                                               "delay_ms": 500}
    rows["s50_1"] = srow(50, done=False, stop="observed_end")
    r = B.judge(rows)
    assert r["decision"] == "B does not pass" and any(d["run"] == "s50_1" and d["priority"] == 2 for d in r["defects"])


def test_a_first_runs_failure_stays_a_defect_beside_its_rerun():
    rows = {f"s{d}_{i}": srow(d) for d in (0, 25, 50, 100, 500) for i in range(3)}
    rows["s25_2"] = srow(25, done=False, stop="safe_failure", eff=False, env=["other users' CPU over 1 core"])
    rows["r_s25_2"] = {**srow(25), "rerun_of": "s25_2"}
    assert any(d["run"] == "s25_2" for d in B.judge(rows)["defects"])


def test_the_submit_script_parses_an_optional_dependency_before_the_rerun_names(tmp_path):
    text = (ROOT / "docs/validation/2026-11-11-latency/lat_b_submit.sh").read_text()
    head = text.split('PY=${G2M_PY')[0] + 'echo "DEP=[$DEP] REST=[$*]"\n'
    f = tmp_path / "h.sh"
    f.write_text(head)
    out = subprocess.run(["bash", str(f), "/s", "/o", "--", "lat_b03_d50_r1"], capture_output=True, text=True).stdout
    assert out.strip() == "DEP=[] REST=[lat_b03_d50_r1]"
    out = subprocess.run(["bash", str(f), "/s", "/o", "17900"], capture_output=True, text=True).stdout
    assert out.strip() == "DEP=[17900] REST=[]"
