"""Real-time plan 5.26 G3-1b stage check: the registered pass / extend / fail lines on synthetic pair readings."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g3_1b_loop_analyze", ROOT / "docs" / "validation" / "2026-11-10-g3-1b-loop" / "g3_1b_loop_analyze.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def r(pair, arm, share, rate, done=True, invalid=()):
    return {"name": f"p{pair}{arm}", "pair": pair, "arm": arm, "invalid": list(invalid), "mission_done": done,
            "scan_step_rest_s_per_sim_s": share, "auto": {"rate": rate}, "kernels_mode": "numba",
            "safety": {"forbidden_contacts": 0, "unpermitted_entries": 0, "unpermitted_dir_entries": 0}}


def three(falls, rises, overrides=None):
    rows = []
    for p, (f, g) in enumerate(zip(falls, rises), start=1):
        rows += [r(p, "off", 0.20, 0.90), r(p, "on", 0.20 - f, 0.90 + g, **(overrides or {}).get(p, {}))]
    return rows


def test_pass_extend_and_fail():
    assert tool.verdict(three([0.012, 0.010, 0.009], [0.01, 0.02, 0.01]))["result"] == "pass"
    assert tool.verdict(three([0.012, 0.010, 0.005], [0.01, 0.02, 0.01]))["result"] == "fail"  # one short of half
    assert tool.verdict(three([0.012, -0.002, 0.010], [0.01, -0.01, 0.02]))["result"] == "extend to 6 pairs"
    assert tool.verdict(three([-0.001, -0.002, -0.003], [0.0, -0.01, -0.02]))["result"] == "fail"
    assert tool.verdict(three([0.012, 0.010, 0.009], [0.01, -0.001, 0.01]))["result"] == "extend to 6 pairs"


def test_a_mission_lost_in_the_on_arm_fails_and_its_timing_is_not_counted():
    v = tool.verdict(three([0.012, 0.010, 0.009], [0.01, 0.02, 0.01], {2: {"done": False}}))
    assert v["result"] == "fail" and v["mission_degraded_pairs"] == [2]
    assert [x["pair"] for x in v["pairs"]] == [1, 3]


def test_an_invalid_session_leaves_too_few_pairs():
    v = tool.verdict(three([0.012, 0.010, 0.009], [0.01, 0.02, 0.01], {3: {"invalid": ["probe exit 1"]}}))
    assert v["result"] == "not enough comparable pairs" and v["left_out"][0]["pair"] == 3


def test_six_pairs_decide_on_the_means():
    rows = three([0.012, -0.002, 0.010], [0.01, -0.01, 0.02])
    rows += [x | {"pair": x["pair"] + 3, "name": x["name"] + "b"} for x in three([0.010, 0.011, 0.002], [0.01, 0.01, 0.0])]
    v = tool.verdict(rows, registered_pairs=6)
    assert v["result"] == "pass" and abs(v["mean_fall_s_per_sim_s"] - 0.007166666666666667) < 1e-12
    rows = [x if x["arm"] == "off" else x | {"scan_step_rest_s_per_sim_s": 0.199} for x in rows]
    assert tool.verdict(rows, registered_pairs=6)["result"] == "fail"


def test_safety_degradation_fails_and_a_missing_safety_record_does_not_pass():
    rows = three([0.012, 0.010, 0.009], [0.01, 0.02, 0.01])
    rows[3]["safety"]["unpermitted_entries"] = 1  # pair 2's on arm
    v = tool.verdict(rows)
    assert v["result"] == "fail" and v["mission_degraded_pairs"] == [2]
    rows = three([0.012, 0.010, 0.009], [0.01, 0.02, 0.01])
    rows[0]["safety"]["forbidden_contacts"] = None
    v = tool.verdict(rows)
    assert v["result"] == "fail" and v["safety_undecided_pairs"] == [1]


def test_the_list_must_hold_the_registered_pairs():
    def e(p, arm):
        return {"env": {"PAIR": str(p), "FORKLIFT_GRID_CLEAR_KERNEL": "1" if arm == "on" else "0"}}
    ok = [e(p, a) for p in (1, 2, 3) for a in ("off", "on")]
    assert tool.list_problems(ok) == []
    assert tool.list_problems(ok[:2])  # one pair only
    assert tool.list_problems([e(p, a) for p in (4, 5, 6) for a in ("off", "on")])  # the extension alone
    assert tool.list_problems(ok[:-1] + [e(3, "off")])  # a pair with two off sessions


def test_the_stage_target_is_reported_not_judged():
    v = tool.verdict(three([0.012, 0.010, 0.009], [0.01, 0.02, 0.01]))
    assert v["result"] == "pass" and v["reported_reaches_stage_target_low"] is False


def test_a_missing_kernel_mode_and_mission_contact_records(tmp_path):
    rows = three([0.012, 0.010, 0.009], [0.01, 0.02, 0.01])
    for x in rows[:2]:
        x["kernels_mode"] = None  # both arms of pair 1 without a recorded mode
    v = tool.verdict(rows)
    assert v["result"] == "not enough comparable pairs" and v["left_out"][0]["pair"] == 1
    out, run = tmp_path / "s", tmp_path / "s" / "run"
    (run / "missions" / "m1").mkdir(parents=True)
    (out / "probe.json").write_text('{"confirm": {"mission_result": {"mission_id": "m1"}}}')
    (run / "missions" / "m1" / "records.json").write_text('{"state": {"forbidden_pocket_contacts": [1, 2]}}')
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) == 2
    assert tool.forbidden_contacts(out, run, {}) is None
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": None}) is None  # null: unknown
    (run / "missions" / "m1" / "records.json").unlink()  # a reported mission without its record: unknown
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) is None


def test_a_reported_mission_without_an_id_makes_the_contacts_unknown(tmp_path):
    out, run = tmp_path / "s", tmp_path / "s" / "run"
    run.mkdir(parents=True)
    (out / "probe.json").write_text('{"confirm": {"mission_result": {"outcome": "done"}}}')
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) is None
    (out / "probe.json").write_text('{}')  # no mission reported: the session's own list only
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) == 0
