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
            "scan_step_rest_s_per_sim_s": share, "auto": {"rate": rate}}


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
