import json

import pytest

from tools import t2_drift_judge as j


def result(*, success=True, phase="complete", gap_mm=60.0, reason=None, home=200.0, contact=False, second=None):
    r = {"success": success, "phase": phase, "transitions": [{"to": "insert", "time_s": 10.0},
                                                             {"to": "home_settle", "time_s": home}]}
    if gap_mm is not None:
        r["lateral_clearance_min_m"] = {"insert": {"left": gap_mm / 1000, "right": (gap_mm + 5) / 1000}}
    if reason:
        r["failure_reason"] = reason
    if contact:
        r["forbidden_pocket_contacts"] = [{"x": 1}]
    if second:
        r["near_capture"] = {"second": {"status": second}}
    return r


def write(tmp_path, arm, scene, seed, r):
    d = tmp_path / f"t2d_{arm}_{scene}_s{seed}" / "run"
    d.mkdir(parents=True)
    (d / "result.json").write_text(json.dumps(r))


def test_outcome_normalisation():
    assert j.outcome(None) == "infrastructure"
    assert j.outcome(result()) == "complete"
    assert j.outcome(result(success=False, phase="lift", reason="plan_worker_dead")) == "infrastructure"
    assert j.outcome(result(success=False, phase="lift", reason="Mission exceeded simulation time budget")) == "mission_failure"
    assert j.outcome(result(success=False, phase="insert", reason="sensor_silence_stopped")) == "mission_failure"
    assert j.outcome(result(success=False, phase="observe", reason="not_pickable:fork_spacing_adjust")) == "expected_refusal"


def test_loss_is_nominal_minus_the_smaller_gap():
    assert j.loss_mm(result(gap_mm=60.0)) == pytest.approx(26.25)
    assert j.loss_mm(result(gap_mm=None)) is None


def test_sign_test_twelve_of_fifteen_is_significant():
    assert j.sign_test_p(12, 3) == pytest.approx(0.03516, abs=1e-4)
    assert j.sign_test_p(11, 4) > 0.05


def test_p95_higher_is_the_max_for_small_samples():
    assert j.p95_higher(list(range(15))) == 14


def test_a_clearly_better_arm(tmp_path):
    for seed in range(15):
        write(tmp_path, "b0", "s", seed, result(gap_mm=40.0 + seed))
        write(tmp_path, "c3", "s", seed, result(gap_mm=70.0 + seed * 0.5, second="adopted"))
        write(tmp_path, "c1", "s", seed, result(gap_mm=40.0 + seed))
    runs = j.load(tmp_path)
    out = j.judge_arm(runs, "c3")
    assert out["pairs"]["better"] == 15 and out["verdict"] == "나음"
    assert j.judge_arm(runs, "c1")["verdict"] == "차이 확인 안 됨"


def test_an_arm_that_does_not_insert_counts_as_worse(tmp_path):
    for seed in range(14):
        write(tmp_path, "b0", "s", seed, result(gap_mm=40.0))
        write(tmp_path, "c3", "s", seed, result(gap_mm=70.0))
    write(tmp_path, "b0", "s", 14, result(gap_mm=40.0))
    write(tmp_path, "c3", "s", 14, result(success=False, phase="approach", gap_mm=None, reason="docking_x"))
    out = j.judge_arm(j.load(tmp_path), "c3")
    assert out["pairs"]["better"] == 14 and out["pairs"]["worse"] == 1


def test_a_contact_makes_it_worse(tmp_path):
    for seed in range(15):
        write(tmp_path, "b0", "s", seed, result(gap_mm=40.0))
        write(tmp_path, "c3", "s", seed, result(gap_mm=70.0, contact=seed == 0))
    assert j.judge_arm(j.load(tmp_path), "c3")["verdict"] == "더 나쁨"


def test_infrastructure_pairs_are_left_out_and_too_few_valid_pairs_is_undecided(tmp_path):
    for seed in range(15):
        write(tmp_path, "b0", "s", seed, result(gap_mm=40.0))
        write(tmp_path, "c3", "s", seed, result(gap_mm=70.0) if seed < 11 else
              result(success=False, phase="observe", gap_mm=None, reason="plan_worker_dead"))
    out = j.judge_arm(j.load(tmp_path), "c3")
    assert out["pairs"]["valid"] == 11 and out["verdict"].startswith("판정 불가")
