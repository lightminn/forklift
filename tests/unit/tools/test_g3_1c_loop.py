"""Real-time plan 5.27 G3-1c stage check: the two lists (grid switch by the 1b verdict) and the analyzer's reading --
the arm on the shadow switch, one grid value across the list, half of 1c's -30 ms/sim s per pair."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


lists = load("g3_1c_loop_lists", "docs/validation/2026-11-10-g3-1c-loop/g3_1c_loop_lists.py")
tool = load("g3_1c_loop_analyze", "docs/validation/2026-11-10-g3-1c-loop/g3_1c_loop_analyze.py")


@pytest.mark.parametrize("grid", ["0", "1"])
def test_each_list_holds_three_balanced_pairs_with_one_grid_value(grid):
    e = lists.entries("/snap", "/out", grid)
    assert len(e) == 6 and {x["env"]["FORKLIFT_GRID_CLEAR_KERNEL"] for x in e} == {grid}
    for a, b in zip(e[::2], e[1::2]):
        assert a["env"]["PAIR"] == b["env"]["PAIR"]
        assert {a["env"]["FORKLIFT_SHADOW_KERNEL"], b["env"]["FORKLIFT_SHADOW_KERNEL"]} == {"0", "1"}
    firsts = [x["name"].rsplit("_", 1)[1] for x in e[::2]]
    assert set(firsts[:2]) == {"off", "on"}
    assert tool.list_problems(e) == []
    with pytest.raises(ValueError):
        lists.entries("/snap", "/out", "2")


def test_a_list_mixing_grid_values_or_with_an_odd_shadow_value_is_refused():
    e = lists.entries("/snap", "/out", "1")
    e[0]["env"]["FORKLIFT_GRID_CLEAR_KERNEL"] = "0"
    assert any("grid switch values" in p for p in tool.list_problems(e))
    e = lists.entries("/snap", "/out", "1")
    e[0]["env"]["FORKLIFT_SHADOW_KERNEL"] = "2"
    assert tool.list_problems(e)
    e = lists.entries("/snap", "/out", "1")  # a fresh list: only an off session's key removed
    off = next(x for x in e if x["env"]["FORKLIFT_SHADOW_KERNEL"] == "0")
    del off["env"]["FORKLIFT_SHADOW_KERNEL"]
    assert tool.list_problems(e)


def r(pair, arm, share, rate):
    return {"name": f"p{pair}{arm}", "pair": pair, "arm": arm, "invalid": [], "mission_done": True,
            "scan_step_rest_s_per_sim_s": share, "auto": {"rate": rate}, "kernels_mode": "numba",
            "safety": {"forbidden_contacts": 0, "unpermitted_entries": 0, "unpermitted_dir_entries": 0}}


def test_the_threshold_is_half_of_minus_30():
    assert tool.HALF_PREDICTION == 0.015 and tool.base.HALF_PREDICTION == 0.015
    rows = []
    for p, f in enumerate([0.020, 0.018, 0.016], start=1):
        rows += [r(p, "off", 0.20, 0.90), r(p, "on", 0.20 - f, 0.92)]
    assert tool.base.verdict(rows)["result"] == "pass"
    rows[-1]["scan_step_rest_s_per_sim_s"] = 0.20 - 0.010  # a pair at 10 ms: under half of 1c's prediction
    assert tool.base.verdict(rows)["result"] == "fail"
