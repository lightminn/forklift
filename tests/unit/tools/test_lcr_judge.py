"""The loaded curvature rule comparison judgement (docs/validation/2026-11-14-loaded-curvature-comparison/lcr_judge.py)
on synthetic results."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "lcr_judge", ROOT / "docs/validation/2026-11-14-loaded-curvature-comparison/lcr_judge.py")
lj = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lj)

RULE = {"k": 0.9, "saturated_curvature_inv_m": 0.3, "planned_curvature_inv_m": 0.27}


def result(arm="B0", state="complete", transport=(65.0, 135.0), cycle=200.0, failure=None, phase=None):
    r = {"success": state == "complete", "deterministic": {"enabled": True, "broken": []},
         "transitions": [{"to": "transport", "time_s": transport[0]}], "simulated_time_s": 300.0,
         "loaded_curvature_rule": RULE if arm == "R" else None,
         "paths": {"transport": {"length_m": 15.0, "directions": [1, 1, -1, -1, 1], "curvatures_inv_m": [0.0, 0.27]}}}
    if state == "complete":
        r["transitions"] += [{"to": "lower", "time_s": transport[1]}, {"to": "home_settle", "time_s": cycle}]
        r["phase"] = "complete"
    elif state == "transport_cross_track":
        r.update(phase="transport", failure_reason="Tracking failed in transport: pos=2.9,yaw=1.0",
                 tracking_failure={"phase": "transport", "tracking": {"failure": "cross_track"}})
    elif state == "transport_planning":
        r.update(phase=phase or "observe", failure_reason=failure or "transport:no_path")
    else:
        r.update(phase=phase or "observe", failure_reason=failure or "near_capture_failed:x")
    return r


def rows(override=None):
    override = override or {}
    out = []
    for scene, seed in lj.cells():
        for arm in lj.ARMS:
            r = override.get((arm, scene, seed)) or result(arm)
            out.append(lj.summarize(f"lcr_{arm}_{scene}_s{seed}", r))
    return out


def ct(arm, n):
    return {(arm, "random_1", s): result(arm, "transport_cross_track") for s in range(n)}


def test_state_classes():
    assert lj.state_of(result()) == "complete"
    assert lj.state_of(result(state="transport_cross_track")) == "transport_cross_track"
    assert lj.state_of(result(state="transport_planning")) == "transport_planning"
    assert lj.state_of(result(state="transport_planning", failure="Mission planning failed: transport:no_path")) \
        == "transport_planning"
    assert lj.state_of(result(state="transport_planning", failure="stall_replan_failed: no_path",
                              phase="transport")) == "transport_planning"
    assert lj.state_of(result(state="other", failure="stall_replan_failed: no_path", phase="return_home")) == "other"
    assert lj.state_of(result(state="other")) == "other"
    for f in ("obstacle_replan_failed in transport: no_path", "obstacle_live_plan_failed in transport: no_path",
              "docking_retry_no_plan:no_path:no_path"):
        assert lj.state_of(result(state="transport_planning", failure=f)) == "transport_planning", f
    assert lj.state_of(result(state="other", failure="obstacle_replan_failed in return_home: no_path")) == "other"


def test_the_ended_phase_is_the_last_transition_without_a_tracking_failure():
    r = result(state="transport_planning", failure="stall_replan_failed: no_path", phase="complete")
    r["transitions"] = [{"to": "transport", "time_s": 65.0}]
    assert lj.ended_phase(r) == "transport" and lj.state_of(r) == "transport_planning"


def test_the_cross_track_criterion_counts_random_1_runs():
    out = lj.judge(rows({**ct("B0", 8), **ct("R", 4)}))
    assert out["criteria"]["2_cross_track"] == {"B0": 8, "R": 4, "underexposed": False, "pass": True}
    out = lj.judge(rows({**ct("B0", 8), **ct("R", 5)}))
    assert not out["criteria"]["2_cross_track"]["pass"] and out["verdict"] == "no_difference_confirmed"


def test_better_when_every_criterion_holds():
    b0 = ct("B0", 8)
    r = ct("R", 3)
    out = lj.judge(rows({**b0, **r}))
    c = out["criteria"]
    assert c["3_completion"]["pass"] and c["1_safety"]["pass"] and c["6_non_regression"]["pass"]
    assert out["verdict"] == "r_better", c


def test_underexposed_holds_promotion():
    out = lj.judge(rows({**ct("B0", 3), **ct("R", 3)}))
    assert out["criteria"]["2_cross_track"]["underexposed"]
    assert out["verdict"] == "effect_unconfirmed_promotion_held"


def test_new_planning_failure_in_aisle_rack_fails_criterion_4():
    o = {**ct("B0", 8), **ct("R", 3), ("R", "aisle_rack", 1): result("R", "transport_planning")}
    out = lj.judge(rows(o))
    assert out["criteria"]["4_planning"]["new_in_line2_aisle_rack"] == ["lcr_R_aisle_rack_s1"]
    assert out["verdict"] == "worse"  # also a non-regression difference


def test_slower_transport_fails_the_time_cost():
    o = {**ct("B0", 8), **ct("R", 3)}
    for scene, seed in lj.cells():
        if ("R", scene, seed) not in o:
            o[("R", scene, seed)] = result("R", transport=(65.0, 150.0), cycle=210.0)
    out = lj.judge(rows(o))
    assert out["criteria"]["5_time"]["transport_time_ratio"] > 1.15
    assert out["verdict"] == "no_difference_confirmed"


def test_a_mislabelled_arm_stops_the_judgement():
    rs = rows()
    rs[1]["rule"] = None  # an R run without the rule's record
    assert lj.judge(rs)["verdict"].startswith("not_judged")


def test_a_missing_run_stops_the_judgement():
    assert lj.judge(rows()[:-1])["verdict"].startswith("not_judged")


def test_output_is_strict_json():
    json.dumps(lj.judge(rows({**ct("B0", 8), **ct("R", 3)})), allow_nan=False)
