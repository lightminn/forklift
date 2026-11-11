"""The latency test A's judgement (docs/plans/2026-11-11-latency-tolerance.md 4): stop vs end, absolute safety with
the B0-repeat mark, the operating widths at their edges, the exposure conditions, the ordered cell classes, aisle_rack's
baseline exceptions, the environment problems and the conclusion rules. Synthetic records, no Isaac."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("lat_analyze", ROOT / "docs/validation/2026-11-11-latency/lat_analyze.py")
A = importlib.util.module_from_spec(spec)
spec.loader.exec_module(A)

LOW_OK = {"rows": {"truck_physical": {"min_m": 0.2, "phase": "transport", "object": 3},
                   "pallet_physical": {"min_m": 0.4, "phase": "transport", "object": 3}},
          "physical_arc_violations": []}


def result(**k):
    base = {"success": True, "forbidden_pocket_contacts": [], "delivery_error_m": 0.01,
            "transitions": [{"time_s": 1.0}, {"time_s": 101.0}], "delivery_docking": {"rounds": [{}]},
            "obstacle_layer": {"unpermitted_entries": 0, "unpermitted_dir_entries": 0, "slowed_ticks": {"a": 3}},
            "deterministic": {"broken": []}}
    base.update(k)
    return base


def test_a_failure_is_a_safe_failure_only_with_the_last_half_second_standing():
    assert A.stop_or_end(result()) is None
    def stand(s, gap=1 / 120, tail=0.0, n=60):
        return {"latency_injection": {"final_stand": {"length_s": s, "control_ticks": n, "control_gap_max_s": gap,
                                                      "control_tail_s": tail}}}
    assert A.stop_or_end(result(success=False, **stand(0.5))) == "safe_failure"
    assert A.stop_or_end(result(success=False, **stand(0.49))) == "observed_end"
    assert A.stop_or_end(result(success=False)) == "observed_end"  # no stand record: not confirmed
    assert A.stop_or_end(result(success=False, **stand(2.0, gap=0.3))) == "observed_end"  # a capture-long gap
    assert A.stop_or_end(result(success=False, **stand(2.0, tail=0.1))) == "observed_end"  # no control at the end
    assert A.stop_or_end(result(success=False, **stand(2.0, n=0))) == "observed_end"


def test_safety_is_absolute_and_unreadable_is_none():
    s = A.safety_of(result(), LOW_OK)
    assert s["violations"] == 0 and s["physical_min_m"] == 0.2
    s = A.safety_of(result(forbidden_pocket_contacts=[{"t": 1}]), LOW_OK)
    assert s["violations"] == 1
    low = {"rows": {"truck_physical": {"min_m": -0.01, "phase": "return_home", "object": 7}},
           "physical_arc_violations": [{"objects": [7], "phase": "return_home"}]}
    s = A.safety_of(result(), low)
    assert s["violations"] == 2  # an overlap and an arc
    assert A.safety_of(result(), {"error": "x"})["violations"] is None
    assert A.safety_of(result(obstacle_layer={}), LOW_OK)["violations"] is None
    assert A.safety_of(result(), {"unevaluated": ["no samples"], "rows": {}})["violations"] is None  # not evaluated
    # a confirmed violation stands even when the low-object report could not be read
    assert A.safety_of(result(forbidden_pocket_contacts=[{}]), {"error": "x"})["violations"] == 1


def test_the_b0_repeat_mark_needs_kind_object_phase_and_not_more_than_a_centimetre_worse():
    base = A.safety_of(result(), {"rows": {"truck_physical": {"min_m": -0.005, "phase": "p", "object": 7}},
                                  "physical_arc_violations": []})
    worse = A.safety_of(result(), {"rows": {"truck_physical": {"min_m": -0.02, "phase": "p", "object": 7}},
                                   "physical_arc_violations": []})
    near = A.safety_of(result(), {"rows": {"truck_physical": {"min_m": -0.012, "phase": "p", "object": 7}},
                                  "physical_arc_violations": []})
    assert A.b0_repeat(worse, base)[0]["b0_repeat"] is False
    assert A.b0_repeat(near, base)[0]["b0_repeat"] is True


def test_the_operating_widths_at_their_edges():
    base = A.mission_of(result())  # 100 s, 1 cm, 1 round
    edge = A.mission_of(result(transitions=[{"time_s": 0.0}, {"time_s": 130.0}], delivery_error_m=0.02,
                               delivery_docking={"rounds": [{}, {}]}))
    assert all(c["within"] for c in A.operating_within(edge, base).values())  # max(115, 130) = 130
    over = A.mission_of(result(transitions=[{"time_s": 0.0}, {"time_s": 130.1}]))
    assert A.operating_within(over, base)["mission_sim_s"]["within"] is False
    long_base = A.mission_of(result(transitions=[{"time_s": 0.0}, {"time_s": 300.0}]))
    assert A.operating_within(A.mission_of(result(transitions=[{"time_s": 0.0}, {"time_s": 345.0}])),
                              long_base)["mission_sim_s"]["within"]  # x 1.15 is the larger: 345


def slam_recs(n, moving=True, holding=False):
    return [{"scan_tick": 12 * i, "delivered_tick": 12 * i + 6, "moving_at_delivery": moving,
             "holding_at_delivery": holding} for i in range(n)]


def test_the_exposure_conditions():
    r = result(latency_injection={"slam": {"records": slam_recs(200) + slam_recs(50, holding=True)}})
    e = A.exposure_of("S6", r, None)
    assert e["ok"] and e["slam"]["direct"] == 200 and e["slam"]["holding"] == 50
    assert not A.exposure_of("S6", result(latency_injection={"slam": {"records": slam_recs(199)}}), None)["ok"]
    jobs = [{"moving": True, "reason": "scheduled", "scan_tick": 12 * i, "install_tick": 12 * i + 6}
            for i in range(160)] + [{"moving": True, "reason": "add_scans", "scan_tick": 0, "install_tick": 1}
                                    for _ in range(40)]
    assert A.exposure_of("U6", result(), jobs)["ok"]  # 200 moving, 0.8 on schedule
    assert not A.exposure_of("U6", result(), jobs + [{"moving": True, "reason": "refresh", "scan_tick": 0,
                                                     "install_tick": 1}])["ok"]  # 160/201 < 0.8


def row(arm, done=True, violations=0, stop=None, ok=True, sim=(0.0, 100.0), scene="line_2"):
    res = result(success=done, transitions=[{"time_s": sim[0]}, {"time_s": sim[1]}])
    return {"arm": arm, "scene": scene, "seed": 0, "safety": {"violations": violations, "events": []},
            "mission": A.mission_of(res), "stop": stop, "exposure": {"ok": ok}, "invalid": []}


def test_the_cell_classes_in_order():
    base = row("B0")
    assert A.classify(row("S6", violations=1, done=False, ok=False), base, aisle=False)["class"] == "dangerous"
    assert A.classify(row("S6", done=False, stop="observed_end", ok=False), base, aisle=False)["class"] == "observed_end"
    assert A.classify(row("S6", done=False, stop="safe_failure", ok=False), base, aisle=False)["class"] == "safe_failure"
    assert A.classify(row("S6", ok=False), base, aisle=False)["class"] == "underexposed"
    assert A.classify(row("S6", sim=(0, 131)), base, aisle=False)["class"] == "operating_degraded"
    assert A.classify(row("S6"), base, aisle=False)["class"] == "tolerated"


def test_aisle_rack_baseline_failure_holds_the_attribution_before_the_conclusions():
    base = row("S0", done=False, stop="observed_end", scene="aisle_rack")
    r = {**row("S12", done=False, stop="observed_end", scene="aisle_rack"), "effective": True}
    r["cell"] = A.classify(r, base, aisle=True)
    out = A.conclusions({"lat_S12_aisle_rack_s0": r}, {"aisle_rack": {"S0": True}}, {})
    assert out["defects"] == []  # the baseline failed: not a latency defect


def test_aisle_rack_baseline_failure_holds_the_attribution():
    base = row("S0", done=False, stop="observed_end", scene="aisle_rack")
    c = A.classify(row("S12", done=False, stop="observed_end", scene="aisle_rack"), base, aisle=True)
    assert c["class"] == "observed_end" and "baseline_failed" in c["attribution"]
    c = A.classify(row("S12", violations=1, scene="aisle_rack"), base, aisle=True)
    assert c["class"] == "dangerous"  # safety stays absolute


def test_the_environment_problems(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    ended = [{"attempt": 1, "status": "ended", "retried": False}]
    assert A.environment_problems(ended, run, "B0", None) == ["no result.json"]
    (run / "slam_control.npy").write_bytes(b"x")
    assert A.environment_problems(ended, run, "B0", result()) == []
    p = A.environment_problems(ended, run, "U6", result(deterministic={"broken": ["wall_cap:x"]}))
    assert any("broken" in x for x in p) and "no layer_worker.json" in p and "no latency_injection record" in p
    assert A.environment_problems([{"attempt": 1, "status": "infra_failed"}], run, "B0", result())


def crow(arm, cls, violations=0, **k):
    return {"arm": arm, "scene": "line_2", "seed": 0, "invalid": [], "effective": True, "cell": {"class": cls},
            "safety": {"violations": violations}, **k}


def test_the_conclusion_rules_order_and_unread_arms():
    rows = {f"lat_{arm}_line_2_s0": crow(arm, cls, int(cls == "dangerous")) for arm, cls in (
        ("S6", "dangerous"), ("U6", "observed_end"), ("U3", "safe_failure"), ("S12", "safe_failure"),
        ("C6", "operating_degraded"), ("D12", "tolerated"))}
    ok_eq, ok_det = {"line_2": {"S0": True, "L0": True}}, {"line_2": True}
    out = A.conclusions(rows, ok_eq, ok_det)
    assert [d["arm"] for d in out["defects"]] == ["S6", "U6", "U3", "C6", "S12"]
    out = A.conclusions(rows, {"line_2": {"S0": False, "L0": True}}, ok_det)
    assert [d["arm"] for d in out["defects"]] == ["S6", "U6", "U3"]  # SLAM arms unread, the dangerous one kept
    out = A.conclusions(rows, {"line_2": {"S0": None, "L0": True}}, ok_det)  # not confirmed: unread too
    assert [d["arm"] for d in out["defects"]] == ["S6", "U6", "U3"]
    out = A.conclusions(rows, ok_eq, {"line_2": False})
    assert [d["arm"] for d in out["defects"]] == ["S6"] and out["determinism_not_confirmed"] == ["line_2", "random_1"]
    rows["lat_U12_line_2_s0"] = crow("U12", "safe_failure")
    assert A.conclusions(rows, ok_eq, ok_det)["d12_u12"]["line_2_s0"] == "knows it: holds; unaware: stops"
    assert A.conclusions(rows, {"line_2": {"S0": True, "L0": False}}, ok_det)["d12_u12"] == {}
    assert A.conclusions(rows, ok_eq, {"line_2": None})["d12_u12"] == {}


def test_a_safety_event_survives_an_invalid_run_a_rerun_and_a_baseline():
    rows = {"lat_U6_line_2_s1": crow("U6", "dangerous", 1, invalid=["deterministic broken ['wall_cap:x']"],
                                     effective=False),
            "lat_r_lat_U6_line_2_s1": crow("U6", "tolerated", rerun_of="lat_U6_line_2_s1"),
            "lat_B0_line_2_s1": crow("B0", "dangerous", 2)}
    out = A.conclusions(rows, {"line_2": {"S0": True, "L0": True}}, {"line_2": True})
    runs = {d["run"]: d for d in out["defects"]}
    assert runs["lat_U6_line_2_s1"]["class"] == "dangerous" and runs["lat_U6_line_2_s1"]["invalid"]
    assert runs["lat_B0_line_2_s1"]["baseline_violation"] is True and "lat_r_lat_U6_line_2_s1" not in runs
