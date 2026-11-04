"""The F1 v2 operating regression verdict (plan section 4): v1's rules per scene, the two mechanism failures counted
apart, the physical settle runs read both ways, the reach table and the round-1 stop report."""

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1_tip_mass_judge as V1  # noqa: E402
import f1v2_regression_judge as J  # noqa: E402


def _result(arm, key, *, complete=True, reason=None, round1=True, cusp_replans=0, phase=None):
    sha = V1.URDF_SHA[(arm, "epal6")] + "0" * 56
    samples = [{"time_s": 0.1 * i, "phase": "transport", "steering_command_rad": [0.0, 0.0],
                "steering_actual_rad": [0.0, 0.0], "signed_speed_mps": 0.0, "lift_m": 0.0,
                "base_position_m": [0, 0, 0], "pallet_position_m": [1, 0, 0], "base_tilt_rad": 0.001} for i in range(3)]
    return {"success": complete, "phase": phase or ("complete" if complete else "insert"), "failure_reason": reason,
            "forbidden_pocket_contacts": [], "lift_tilt_peak": {"pallet_tilt_rad": 1e-4, "pallet_position_m": [1, 0, 0]},
            "initial_pallet_m": [1, 0, 0], "insertion_measured": {}, "slip_watch": {},
            "lateral_clearance_min_m": {"insert": {"left": 0.06}},
            "plant_backend": {"body_names": ["base_link", "left_fork_tip", "right_fork_tip"],
                              "masses_kg": [20.0] + [V1.TIP_MASS[arm]] * 2},
            "forklift_urdf_sha256": sha, "scene_built_from_urdf_sha256": sha,
            "scene_sha256": {"scene.usda": f"{arm}-epal6"}, "pallet_mass": {"kg": V1.PALLET_KG["epal6"]},
            "delivery_docking": {"rounds": [{"accepted": round1, "reason": "accepted" if round1 else
                                             "start_rejected:degenerate",
                                             "start_details": [{"min_information": 52.0 if round1 else 49.0}]}]},
            "cusp_replans": [{}] * cusp_replans, "arguments": {"transport_cusp_replans": 2}, "samples": samples}


def _batch(tmp: Path, override=None, skip=()):
    (tmp / "b_x").mkdir(exist_ok=True)
    for key in J.EXPECTED_KEYS:
        for arm in J.ARMS:
            name = f"f1_{arm}_{key}"
            if name in skip:
                continue
            r = (override and override(arm, key)) or _result(arm, key)
            (tmp / name / "run").mkdir(parents=True)
            (tmp / name / "run" / "result.json").write_text(json.dumps(r))
            with open(tmp / "b_x" / "results.jsonl", "a") as f:
                f.write(json.dumps({"name": name, "status": "ended"}) + "\n")
    return tmp


def test_all_complete_passes(tmp_path):
    out = J.judge(_batch(tmp_path))
    assert out["overall"] == "pass"
    for s in J.SCENES:
        assert out["scenes"][s]["verdict"] == "pass" and out["scenes"][s]["exclude"]["valid_pairs"] == 15
        assert out["scenes"][s]["reach"]["complete"]["both"] == 15


def test_mechanism_failures_counted_apart(tmp_path):
    rej = {"line_2_s3", "line_2_s4"}

    def ov(arm, key):
        if arm == "F1" and key in rej:  # rejected but still completed: the completion rule passes, the count does not
            return _result(arm, key, round1=False)
        return None
    out = J.judge(_batch(tmp_path, ov))
    line = out["scenes"]["line_2"]
    assert line["exclude"]["mechanism"] == {"field": "round1_rejected", "O": 0, "F1": 2}
    assert line["verdict"] == "fail" and out["scenes"]["aisle_rack"]["verdict"] == "pass"


def test_one_more_mechanism_failure_is_allowed_and_cusp_budget(tmp_path):
    def ov(arm, key):
        if key == "aisle_rack_s5" and arm == "F1":
            return _result(arm, key, complete=False, phase="transport", cusp_replans=2,
                           reason="Tracking failed in transport: pos=0.0164,yaw=-0.0512")
        if key == "aisle_rack_s6" and arm == "O":
            return _result(arm, key, complete=False, phase="transport", cusp_replans=2,
                           reason="Tracking failed in transport: pos=0.0164,yaw=-0.0512")  # v1: whole strings
        return None
    out = J.judge(_batch(tmp_path, ov))
    a = out["scenes"]["aisle_rack"]
    assert a["exclude"]["mechanism"]["F1"] == 1 and a["exclude"]["mechanism"]["O"] == 1
    assert a["verdict"] == "pass"  # the same end in both arms: no new reason; completion 14 vs 14


def test_new_reason_needs_cause_and_completion_rule(tmp_path):
    def ov(arm, key):
        if arm == "F1" and key in ("line_2_s3", "line_2_s4"):
            return _result(arm, key, complete=False, reason="Pallet missed the green destination center")
        return None
    out = J.judge(_batch(tmp_path, ov))
    assert out["scenes"]["line_2"]["verdict"] == "fail"  # 13 < 15 - 1

    def ov1(arm, key):
        if arm == "F1" and key == "line_2_s3":
            return _result(arm, key, complete=False, reason="Pallet missed the green destination center")
        return None
    (tmp_path / "b").mkdir()
    out = J.judge(_batch(tmp_path / "b", ov1))
    assert out["scenes"]["line_2"]["verdict"] == "needs_cause"
    assert out["scenes"]["line_2"]["exclude"]["new_reasons_F1"] == ["Pallet missed the green destination center"]


def test_settle_runs_read_both_ways(tmp_path):
    # one F1 settle failure: excluded -> 14 valid pairs, pass; counted -> F1 14 complete vs O 15, a new reason ->
    # needs_cause; the two readings differ -> unjudgeable
    def ov(arm, key):
        if arm == "F1" and key == "line_2_s3":
            return _result(arm, key, complete=False, reason="Truck spawn displaced")
        return None
    out = J.judge(_batch(tmp_path, ov))
    s = out["scenes"]["line_2"]
    assert s["exclude"]["verdict"] == "pass" and s["fail"]["verdict"] == "needs_cause" and s["verdict"] == "unjudgeable"
    assert out["overall"] == "unjudgeable"


def test_settle_runs_two_in_one_arm_hold(tmp_path):
    def ov(arm, key):
        if key in ("line_2_s3", "line_2_s4"):
            if arm == "F1":
                return _result(arm, key, complete=False, reason="Invalid pallet spawn after settling")
            return _result(arm, key, complete=False, reason="Invalid pallet spawn after settling") if key == "x" else None
        return None
    out = J.judge(_batch(tmp_path, ov))
    assert out["scenes"]["line_2"]["settle_hold"] and out["scenes"]["line_2"]["verdict"] == "unjudgeable"


def test_too_few_valid_pairs(tmp_path):
    skip = {f"f1_F1_line_2_s{s}" for s in (3, 4, 5, 6)}
    out = J.judge(_batch(tmp_path, skip=skip))
    assert out["scenes"]["line_2"]["exclude"]["valid_pairs"] == 11 and out["scenes"]["line_2"]["verdict"] == "inconclusive"


def test_round1_stop_from_the_match_file(tmp_path):
    b = _batch(tmp_path)
    run = b / "f1_O_line_2_s3" / "run"
    np.savez(run / "delivery_match_1.npz", reference_rear=np.array([0.678, -8.946, 0.0]),
             truth_rear=np.array([0.678 - 1.46, -8.95, 0.01]), estimate=np.array([0.678 - 1.40, -8.94, 0.01]))
    out = J.judge(b)
    rep = out["scenes"]["line_2"]["round1_stop"]["O"]
    assert math.isclose(rep["stop_distance_m"]["mean"], 1.46, abs_tol=1e-9)
    assert math.isclose(rep["slam_along_error_m"]["mean"], 0.06, abs_tol=1e-9)
    assert out["scenes"]["line_2"]["round1_stop"]["F1"]["stop_distance_m"] is None
