"""The F1 fork-tip-mass registration verdict (docs/plans/2026-11-04-f1-fork-tip-mass.md section 3): the registered
population and the rerun pair, validity and stage reach, the operating-regression inequalities, the live STOP rule,
the steering residual cells (sample filter, missing records, sufficiency, the pooled estimator and its bootstrap
interval, the non-inferiority reading), the arm identity and the odometry age comparison."""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1_tip_mass_judge as J  # noqa: E402

RIIB = [k for k in J.EXPECTED_KEYS if not k.startswith(("cy_", "live_"))]


def _samples(shortfall_l: float, n: int = 40, *, cmd: float = 0.3, speed: float = 0.0, lift: float = 0.0,
             phase: str = "transport", t0: float = 0.0):
    return [{"time_s": t0 + 0.1 * i, "steering_command_rad": [cmd, cmd],
             "steering_actual_rad": [cmd - shortfall_l * (1 if cmd > 0 else -1), cmd],
             "signed_speed_mps": speed, "lift_m": lift, "phase": phase,
             "base_position_m": [0.0, 0.0, 0.0], "pallet_position_m": [1.0, 0.0, 0.0], "base_tilt_rad": 0.001}
            for i in range(n)]


def _default_samples(shortfall=0.03):
    return _samples(shortfall, n=30) + _samples(shortfall, n=30, speed=0.2, t0=10.0)


def _result(arm: str, key: str, *, complete=True, reason=None, lifted=True, samples=None):
    shape = J.shape_of(key)
    sha = J.URDF_SHA[(arm, shape)] + "0" * 56
    return {"success": complete, "phase": "complete" if complete else "insert", "failure_reason": reason,
            "forbidden_pocket_contacts": [], "lift_tilt_peak": {"pallet_tilt_rad": 1e-4, "pallet_position_m": [1, 0, 0]}
            if lifted else None, "initial_pallet_m": [1, 0, 0], "insertion_measured": {}, "slip_watch": {},
            "lateral_clearance_min_m": {"insert": {"left": 0.06, "right": 0.06}},
            "plant_backend": {"body_names": ["base_link", "left_fork_tip", "right_fork_tip"],
                              "masses_kg": [20.0] + [J.TIP_MASS[arm]] * 2},
            "forklift_urdf_sha256": sha, "scene_built_from_urdf_sha256": sha,
            "scene_sha256": {"scene.usda": f"{arm}-{shape}"}, "pallet_mass": {"kg": J.PALLET_KG[shape]},
            "samples": samples if samples is not None else _default_samples()}


def _write(batch: Path, name: str, result: dict | None, *, status="ended", live_probe=None):
    root = batch / name
    if live_probe is not None:
        (root / "live" / "epoch_1" / "run").mkdir(parents=True)
        (root / "probe.json").write_text(json.dumps(live_probe))
        (root / "live" / "epoch_1" / "run" / "result.json").write_text(json.dumps(result))
    else:
        (root / "run").mkdir(parents=True)
        if result is not None:
            (root / "run" / "result.json").write_text(json.dumps(result))
    (batch / "b_x").mkdir(exist_ok=True)
    with open(batch / "b_x" / "results.jsonl", "a") as f:
        f.write(json.dumps({"name": name, "status": status}) + "\n")


def _probe(outcome="cancelled", span=30.0, mission_id="m1"):
    return {"missions_run": [{"stop_wait": {"phase_reached": True, "stop_reply": {"t": "ack"}, "end_reason": "sim_time",
                                            "start_sim_s": 10.0, "end_sim_s": 10.0 + span, "mode_at_end": "STOPPED",
                                            "rearm": {"t": "ack"}, "mission_id": mission_id},
                              "result": {"mission_id": mission_id, "outcome": outcome}}],
            "events": [{"event": "reached", "label": "paused_after_rearm"}]}


def _batch(tmp: Path, *, skip=(), result=None, probe=None):
    """The registered batch; result(arm, key) / probe(arm, key) override a run (None -> default)."""
    for key in J.EXPECTED_KEYS:
        for arm in J.ARMS:
            name = f"f1_{arm}_{key}"
            if name in skip:
                continue
            r = (result and result(arm, key)) or _result(arm, key)
            if key.startswith("live_"):
                _write(tmp, name, r, live_probe=(probe and probe(arm, key)) or _probe())
            else:
                _write(tmp, name, r)


def test_residual_filter_reads_steady_commands_per_wheel_and_turn():
    s = _samples(0.03, n=5, speed=0.1)
    s[2]["steering_command_rad"] = [0.31, 0.31]  # a moving command: rows 2 and 3 are not steady
    cells = J.residual_samples(s)
    assert len(cells[("forward", "empty", "left", "L")]) == 2
    assert cells[("forward", "empty", "left", "L")][0] == pytest.approx(0.03)
    assert cells[("forward", "empty", "left", "R")] == [0.0, 0.0]
    right = J.residual_samples(_samples(0.02, n=3, cmd=-0.3, lift=0.05))
    assert right[("still", "loaded", "right", "L")] == pytest.approx([0.02, 0.02])
    assert J.residual_samples(_samples(0.02, n=3, cmd=0.1)) == {}  # |command| < 0.20
    gap = _samples(0.02, n=2)
    gap[1]["time_s"] = 0.5  # the previous sample is not 0.1 s back
    assert J.residual_samples(gap) == {}
    missing = _samples(0.02, n=3)
    for x in missing:
        x["lift_m"] = None  # missing, not empty
    assert J.residual_samples(missing) == {}
    missing = _samples(0.02, n=3)
    for x in missing:
        x["signed_speed_mps"] = None  # missing, not standstill
    assert J.residual_samples(missing) == {}


def test_cell_sufficiency_and_reading():
    assert J.cell_estimate([([0.03] * 300, [0.03] * 300)] * 7)["status"] == "insufficient"  # 7 pairs
    assert J.cell_estimate([([0.03] * 19, [0.03] * 19)] * 10)["status"] == "insufficient"  # 190 samples
    same = [([0.030 + 0.0001 * i] * 25, [0.030 + 0.0001 * i] * 25) for i in range(10)]
    est = J.cell_estimate(same)
    assert est["status"] == "non_inferior" and est["d"] == pytest.approx(0.0) and not est["decrease"]
    up = [(o, [x + 0.005 for x in f]) for o, f in same]
    assert J.cell_estimate(up)["status"] == "increase"
    down = [(o, [x - 0.005 for x in f]) for o, f in same]
    est = J.cell_estimate(down)
    assert est["status"] == "non_inferior" and est["decrease"]


def test_cell_pools_every_valid_pair_not_only_the_common_ones():
    # 8 common pairs agree; 12 more valid pairs have the cell in F1 only, 10 mrad higher (Codex P2-2's case)
    pairs = [([0.03] * 30, [0.03] * 30)] * 8 + [([], [0.04] * 30)] * 12
    est = J.cell_estimate(pairs)
    assert est["pairs_with_both"] == 8 and est["d"] == pytest.approx(0.01)
    assert est["status"] != "non_inferior"  # the common-pairs-only reading would have passed (d 0, CI [0, 0])


def test_residual_needs_a_sufficient_still_and_forward_cell(tmp_path):
    _batch(tmp_path, result=lambda arm, key: _result(arm, key, samples=_samples(0.03, n=30)))
    out = J.judge(tmp_path)
    assert out["residual"]["verdict"] == "inconclusive"  # standstill only
    assert out["residual"]["sufficient_primary_cells"] == ["still/empty/left/L"]


def test_full_batch_pass(tmp_path):
    _batch(tmp_path)
    out = J.judge(tmp_path)
    assert {g: v["verdict"] for g, v in out["groups"].items()} == {
        "riib": "pass", "cycle_epal6": "pass", "cycle_t11_06_d600": "pass", "live": "pass"}
    assert out["residual"]["verdict"] == "pass" and out["residual"]["pairs_read"] == 20
    assert out["overall"] == "pass" and not out["identity_mismatch"] and not out["scene_problems"]


@pytest.mark.parametrize("field, value", [
    ("tips", 1.0), ("forklift_urdf_sha256", "517e8260" + "0" * 56), ("scene_built_from_urdf_sha256", "x"),
    ("pallet_mass", {"kg": 10.0}), ("scene_sha256", {"scene.usda": "other"}), ("scene_sha256", None),
    ("scene_sha256", {}), ("scene_sha256", 123), ("scene_sha256", True), ("scene_sha256", ["not-a-hash"])])
def test_a_run_that_does_not_show_its_arm_stops_the_reading(tmp_path, field, value):
    def result(arm, key):
        if (arm, key) != ("F1", "line_2_s0"):
            return None
        r = _result(arm, key)
        if field == "tips":
            r["plant_backend"]["masses_kg"] = [20.0, value, value]
        else:
            r[field] = value
        return r

    _batch(tmp_path, result=result)
    out = J.judge(tmp_path)
    assert out["overall"] == "inconclusive"
    assert out["identity_mismatch"] or out["scene_problems"]


def test_missing_groups_and_unregistered_folders(tmp_path):
    _batch(tmp_path, skip={f"f1_{a}_{k}" for a in J.ARMS for k in J.EXPECTED_KEYS if k.startswith(("cy_", "live_"))})
    for arm in J.ARMS:
        _write(tmp_path, f"f1_{arm}_line_2_s7", _result(arm, "line_2_s0"))  # outside the registered list
    out = J.judge(tmp_path)
    assert out["groups"]["cycle_epal6"]["verdict"] == "inconclusive"
    assert out["groups"]["live"]["verdict"] == "inconclusive"
    assert out["overall"] == "inconclusive"
    assert out["unregistered_folders"] == ["f1_F1_line_2_s7", "f1_O_line_2_s7"]


def test_operating_regression_rules(tmp_path):
    bad = {"line_2_s0", "line_2_s1"}  # F1 completes two fewer
    _batch(tmp_path, result=lambda arm, key: _result(arm, key, complete=False, reason="Tracking timeout in observe")
           if arm == "F1" and key in bad else None)
    g = J.judge(tmp_path)["groups"]["riib"]
    assert g["complete"] == {"O": 12, "F1": 10} and g["verdict"] == "fail"


def test_new_reason_needs_a_cause_and_the_rerun_pair_stands(tmp_path):
    def result(arm, key):
        if (arm, key) == ("F1", "line_2_s0"):
            return _result(arm, key, complete=False, reason="Some new end")
        if (arm, key) == ("O", "line_2_s1"):
            return _result(arm, key, complete=False, reason="slam_link_failed: x")
        return None

    _batch(tmp_path, result=result)
    for arm in J.ARMS:
        _write(tmp_path, f"f1_{arm}_line_2_s1_r2", _result(arm, "line_2_s1"))
    g = J.judge(tmp_path)["groups"]["riib"]
    assert g["valid_pairs"] == 12 and g["infra"] == {"O": 1, "F1": 0}  # the replaced run still counted
    assert g["new_reasons_F1"] == ["Some new end"] and g["verdict"] == "needs_cause"


def test_a_rerun_pair_without_its_first_pair_is_read(tmp_path):
    _batch(tmp_path, skip={"f1_O_line_2_s0", "f1_F1_line_2_s0"})
    for arm in J.ARMS:
        _write(tmp_path, f"f1_{arm}_line_2_s0_r2",
               _result(arm, "line_2_s0", complete=arm == "O", reason=None if arm == "O" else "Some new end"))
    g = J.judge(tmp_path)["groups"]["riib"]
    assert g["valid_pairs"] == 12 and g["complete"]["F1"] == 11


def test_live_rule(tmp_path):
    _batch(tmp_path, probe=lambda arm, key: _probe(outcome="completed") if (arm, key) == ("F1", "live_s1") else None)
    g = J.judge(tmp_path)["groups"]["live"]
    assert g["pass"]["f1_F1_live_s1"] is False and g["verdict"] == "fail"


def test_live_missing_mission_ids_do_not_match(tmp_path):
    _batch(tmp_path, probe=lambda arm, key: _probe(mission_id=None) if (arm, key) == ("F1", "live_s0") else None)
    assert J.judge(tmp_path)["groups"]["live"]["verdict"] == "fail"


def test_live_infrastructure_end_is_invalid(tmp_path):
    _batch(tmp_path, result=lambda arm, key: _result(arm, key, complete=False, reason="slam_link_failed: gone")
           if key == "live_s0" else None)
    g = J.judge(tmp_path)["groups"]["live"]
    assert g["valid_pairs"] == 1 and g["verdict"] == "inconclusive"


def test_live_different_ends_need_a_cause(tmp_path):
    _batch(tmp_path, probe=lambda arm, key: _probe(outcome="recovery_required") if (arm, key) == ("F1", "live_s0")
           else None)
    g = J.judge(tmp_path)["groups"]["live"]
    assert g["outcome_differs"] == ["live_s0"] and g["verdict"] == "needs_cause"


def test_missing_records_do_not_stop_the_reading(tmp_path):
    def result(arm, key):
        if (arm, key) != ("O", "line_2_s0"):
            return None
        r = _result(arm, key)
        r["lateral_clearance_min_m"] = {"insert": {"left": None, "right": None}}
        return r

    _batch(tmp_path, result=result)
    (tmp_path / "f1_O_live_s1" / "probe.json").unlink()
    out = J.judge(tmp_path)
    assert out["groups"]["live"]["valid_pairs"] == 1 and out["groups"]["riib"]["verdict"] == "pass"


def test_a_live_run_without_its_probe_still_counts_its_safety_failure(tmp_path):
    def result(arm, key):
        if (arm, key) != ("F1", "live_s0"):
            return None
        r = _result(arm, key, complete=False, reason="Forbidden fork/pallet contact in transport: x")
        r["forbidden_pocket_contacts"] = ["x"]
        return r

    _batch(tmp_path, result=result)
    (tmp_path / "f1_F1_live_s0" / "probe.json").unlink()
    for arm in J.ARMS:
        _write(tmp_path, f"f1_{arm}_live_s0_r2", _result(arm, "live_s0"), live_probe=_probe())
    g = J.judge(tmp_path)["groups"]["live"]
    assert g["safety_failures"] == {"O": 0, "F1": 1} and g["verdict"] == "fail"


def test_upper_gap_and_insert_stops():
    s = _samples(0.0, n=10, speed=0.05, phase="insert", lift=0.01)
    for x in s[3:9]:
        x["signed_speed_mps"] = 0.0
    s[-1]["signed_speed_mps"] = 0.05
    stops = J.insert_stops(s)
    assert len(stops) == 1 and stops[0]["duration_s"] == pytest.approx(0.6)
    # T11 opening top 0.060; blade top 0.0525 + lift 0.01 over a base at 0 -> -0.0025 (into the deck)
    assert J.upper_gap(s, "t11_06_d600") == pytest.approx(0.060 - 0.0525 - 0.01)
    assert J.upper_gap(s, "epal6") == pytest.approx(0.100 - 0.0525 - 0.01)


def test_age_comparison():
    table = {"ages_s": [0.1, 1.0], "position_max_m": [0.02, 0.03], "yaw_max_rad": [0.01, 0.02]}
    out = J.age_comparison(table, {"ages_s": [0.1, 1.0], "position_max_m": [0.019, 0.031], "yaw_max_rad": [0.0, 0.0]})
    assert out["ages_over"] == [1.0]
    assert "error" in J.age_comparison(table, {"ages_s": [0.1], "position_max_m": [0], "yaw_max_rad": [0]})


def _overscan(tmp_path: Path, name: str, arm: str, key: str, *, samples=(), moved=0.0, trace=True):
    """A run ended on the omni.replicator overscan TypeError (registration amendment 2)."""
    import numpy as np

    r = _result(arm, key, complete=False, reason=J.OVERSCAN_REASON, lifted=False, samples=list(samples))
    r.update(transitions=[], applied_command=[0.0, [0.0, 0.0]])
    _write(tmp_path, name, r)
    run = tmp_path / name / "run"
    (run / "failure.txt").write_text("File camera.py, in get_rgba\n  in _resize_data_for_overscan\nTypeError"
                                     if trace else "TypeError")
    np.save(run / "slam_control.npy", np.array([[0, -2.34, 0, 0, -2.34, 0, 0], [0.1, -2.34, 0, 0, -2.34 + moved, 0, 0]]))


def test_amendment_2_overscan_end_is_infrastructure_and_its_pair_reruns(tmp_path):
    _batch(tmp_path, skip={"f1_O_cy_epal6_s0"})
    _overscan(tmp_path, "f1_O_cy_epal6_s0", "O", "cy_epal6_s0")
    for arm in J.ARMS:
        _write(tmp_path, f"f1_{arm}_cy_epal6_s0_r2", _result(arm, "cy_epal6_s0"))
    out = J.judge(tmp_path)
    g = out["groups"]["cycle_epal6"]
    assert g["valid_pairs"] == 3 and g["infra"] == {"O": 1, "F1": 0} and g["verdict"] == "pass"
    assert out["overscan"]["O"]["overscan"] == ["f1_O_cy_epal6_s0"]


@pytest.mark.parametrize("kw", [{"samples": _samples(0.0, n=2)}, {"moved": 0.05}, {"trace": False}])
def test_amendment_2_does_not_cover_a_run_that_started_or_moved(tmp_path, kw):
    _batch(tmp_path, skip={"f1_O_cy_epal6_s0"})
    _overscan(tmp_path, "f1_O_cy_epal6_s0", "O", "cy_epal6_s0", **kw)
    run = [r for r in J.judge(tmp_path)["runs"] if r["name"] == "f1_O_cy_epal6_s0"][0]
    assert not run["infra"] and not run["overscan_end"] and run["overscan_reason"]


def test_amendment_2_rerun_broken_again_leaves_the_pair_out(tmp_path):
    _batch(tmp_path, skip={"f1_O_cy_epal6_s0"})
    _overscan(tmp_path, "f1_O_cy_epal6_s0", "O", "cy_epal6_s0")
    _overscan(tmp_path, "f1_O_cy_epal6_s0_r2", "O", "cy_epal6_s0")
    _write(tmp_path, "f1_F1_cy_epal6_s0_r2", _result("F1", "cy_epal6_s0"))
    out = J.judge(tmp_path)
    assert out["groups"]["cycle_epal6"]["valid_pairs"] == 2
    assert out["groups"]["cycle_epal6"]["verdict"] == "inconclusive"  # 3 valid pairs needed
    assert out["overscan"]["O"]["overscan"] == ["f1_O_cy_epal6_s0", "f1_O_cy_epal6_s0_r2"]
    assert out["overscan"]["O"]["over_hold_share"] is False  # 2 of 21


def test_amendment_2_hold_share():
    runs = [{"name": f"r{i}", "arm": "O", "identity": {"x": 1}, "overscan_reason": i < 2} for i in range(5)]
    assert J.overscan_share(runs)["O"]["over_hold_share"] is True  # 2 / 5 > 25 %
    assert J.overscan_share(runs[1:])["O"]["over_hold_share"] is False  # 1 / 4
    # a hung session without a result counts in the denominator
    hung = [{"name": "e", "arm": "O", "identity": {}, "overscan_reason": True}] + [
        {"name": f"h{i}", "arm": "O", "identity": {}} for i in range(3)]
    assert J.overscan_share(hung)["O"]["over_hold_share"] is False  # 1 / 4


@pytest.mark.parametrize("drop", ["applied_command", "samples", "transitions", "forbidden_pocket_contacts"])
def test_amendment_2_missing_record_is_not_infrastructure(tmp_path, drop):
    _batch(tmp_path, skip={"f1_O_cy_epal6_s0"})
    _overscan(tmp_path, "f1_O_cy_epal6_s0", "O", "cy_epal6_s0")
    p = tmp_path / "f1_O_cy_epal6_s0" / "run" / "result.json"
    d = json.loads(p.read_text())
    d.pop(drop)
    p.write_text(json.dumps(d))
    run = [r for r in J.judge(tmp_path)["runs"] if r["name"] == "f1_O_cy_epal6_s0"][0]
    assert not run["overscan_end"] and not run["infra"]
