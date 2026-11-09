"""Real-time plan 5.26: the 1b stage check's list -- 3 pairs of line_m far K f 1.0, each pair one off and one on, the
order fixed by the seed, the switches explicit; a 6-pair extension keeps the first three pairs and their order."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g3_1b_loop_lists", ROOT / "docs" / "validation" / "2026-11-10-g3-1b-loop" / "g3_1b_loop_lists.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_three_pairs_each_off_and_on_with_explicit_switches():
    e = tool.entries("/snap", "/out")
    assert len(e) == 6
    for a, b in zip(e[::2], e[1::2]):
        assert a["env"]["PAIR"] == b["env"]["PAIR"]
        assert {a["env"]["FORKLIFT_GRID_CLEAR_KERNEL"], b["env"]["FORKLIFT_GRID_CLEAR_KERNEL"]} == {"0", "1"}
    for x in e:
        arm = x["name"].rsplit("_", 1)[1]
        assert x["env"]["FORKLIFT_GRID_CLEAR_KERNEL"] == ("1" if arm == "on" else "0")
        assert x["env"]["FORKLIFT_SHADOW_KERNEL"] == "0" and x["env"]["FACTOR"] == "1.0" and x["env"]["ARM"] == "K"
    assert tool.entries("/snap", "/out") == e  # the seed fixes the order
    firsts = [x["name"].rsplit("_", 1)[1] for x in e[::2]]
    assert set(firsts[:2]) == {"off", "on"}  # balanced: the arm is not the order in every pair
    more = tool.entries("/snap", "/out", pairs=6, first=4)
    assert [x["name"][:7] for x in more] == ["g3bl_07", "g3bl_08", "g3bl_09", "g3bl_10", "g3bl_11", "g3bl_12"]
    assert [x["name"] for x in tool.entries("/snap", "/out", pairs=6)[:6]] == [x["name"] for x in e]


def test_the_wrapper_accepts_the_f_1_0_combination():
    assert "far:K:1.0) ;;" in (ROOT / tool.WRAPPER).read_text()
