"""Real-time plan 5.26: the 1d stage check and the 1e judgement run -- the lists (arms, switches, balanced order, the
1e list by the 1d verdict) and the registered verdicts (1d: half of -11.5 per pair; 1e: the 0.100 budget per pair,
the 6-pair mean, supplement and hold)."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


lists = load("g3_1d_1e_lists", "docs/validation/2026-11-10-g3-1d-1e/g3_1d_1e_lists.py")
tool = load("g3_1d_1e_analyze", "docs/validation/2026-11-10-g3-1d-1e/g3_1d_1e_analyze.py")


def test_the_1d_list_runs_loop_and_fast_on_1b_and_1c():
    e = lists.entries("1d", "/s", "/o")
    assert len(e) == 6 and {x["env"]["LIDAR_CAST"] for x in e} == {"loop", "fast"}
    assert all(x["env"]["FORKLIFT_GRID_CLEAR_KERNEL"] == "1" and x["env"]["FORKLIFT_SHADOW_KERNEL"] == "1" for x in e)
    assert tool.list_problems("1d", e) == []
    firsts = [x["env"]["ARM_NAME"] for x in e[::2]]
    assert set(firsts[:2]) == {"loop", "fast"}


@pytest.mark.parametrize("lidar", ["fast", "loop"])
def test_the_1e_lists_compare_all_off_with_the_candidate(lidar):
    e = lists.entries("1e", "/s", "/o", lidar)
    for x in e:
        env = x["env"]
        if env["ARM_NAME"] == "base":
            assert (env["FORKLIFT_GRID_CLEAR_KERNEL"], env["FORKLIFT_SHADOW_KERNEL"], env["LIDAR_CAST"]) == ("0", "0", "loop")
        else:
            assert (env["FORKLIFT_GRID_CLEAR_KERNEL"], env["FORKLIFT_SHADOW_KERNEL"], env["LIDAR_CAST"]) == ("1", "1", lidar)
    assert tool.list_problems("1e", e) == []
    with pytest.raises(ValueError):
        lists.entries("1e", "/s", "/o", None)
    more = lists.entries("1e", "/s", "/o", lidar, pairs=6, first=4)
    assert [x["name"] for x in lists.entries("1e", "/s", "/o", lidar, pairs=6)[:6]] == [x["name"] for x in e]
    assert len(more) == 6 and {x["env"]["PAIR"] for x in more} == {"4", "5", "6"}


def r(pair, arm, share, rate=0.9, done=True, invalid=()):
    return {"name": f"p{pair}{arm}", "pair": pair, "arm": arm, "invalid": list(invalid), "mission_done": done,
            "scan_step_rest_s_per_sim_s": share, "auto": {"rate": rate, "ok": 0.5, "sim_s": 150.0},
            "scan_step_W_ms": {"p99": 70.0, "max": 130.0}, "kernels_mode": "numba",
            "safety": {"forbidden_contacts": 0, "unpermitted_entries": 0, "unpermitted_dir_entries": 0}}


def pairs(savings, overrides=None):
    rows = []
    for p, s in enumerate(savings, start=1):
        rows += [r(p, "off", 0.21), r(p, "on", 0.21 - s, 0.95, **(overrides or {}).get(p, {}))]
    return rows


def test_1d_uses_half_of_minus_11_5():
    assert tool.verdict_1d(pairs([0.007, 0.006, 0.0058]), 3)["result"] == "pass"
    assert tool.verdict_1d(pairs([0.007, 0.006, 0.005]), 3)["result"] == "fail"
    assert tool.base.HALF_PREDICTION == 0.007  # the 1b module's value is put back


def test_1e_branches():
    assert tool.verdict_1e(pairs([0.11, 0.12, 0.105]), 3)["result"] == "no 1e"
    v = tool.verdict_1e(pairs([0.06, 0.07, 0.08]), 3)
    assert v["result"] == "plan 1e" and abs(v["1e_target_s_per_sim_s"] - 0.03) < 1e-12
    assert tool.verdict_1e(pairs([0.09, 0.11, 0.10]), 3)["result"] == "extend to 6 pairs"
    v = tool.verdict_1e(pairs([0.09, 0.11, 0.10, 0.095, 0.098, 0.099]), 6)
    assert v["result"] == "plan 1e" and v["straddle"] and abs(v["1e_target_s_per_sim_s"] - (0.1 - 0.0986666666)) < 1e-9
    assert tool.verdict_1e(pairs([0.11, 0.12, 0.105], {2: {"invalid": ["probe exit 1"]}}), 3)["result"] == "supplement"
    assert tool.verdict_1e(pairs([0.11, 0.12, 0.105], {3: {"done": False}}), 3)["result"] == "hold: to the lead"
