"""Real-time plan 5.24: the G5-0 arms' registered reading -- each stall on its own, a split arm calls for a third
session, a moved stall is not a gone one, short prep renders leave the render reading undecided, and the oracle arm
counts only with P0's semantic labels."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("g5_0_analyze",
                                              ROOT / "docs" / "validation" / "2026-11-09-g5-0-arms" / "g5_0_analyze.py")
g5 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(g5)
LABELS = [["/World/P_1", "inst_0"], ["/World/Environment", "floor"]]


def row(arm, s1, s2, prep=None, labels=LABELS, later=()):
    r = {"name": f"x_{arm}", "arm": arm, "invalid": [], "semantic_labels": labels,
         "render_steps": [{"world_s": s1}, {"world_s": s2}], "later_render_over_0_25": list(later)}
    if prep is not None:
        r["prep"] = {"renders_s": prep}
    return r


def base():
    return [row("P0", 2.3, 1.0), row("P0", 2.3, 0.97)]


def test_prep_renders_that_take_the_stalls_read_as_the_render_graph():
    rows = base() + [row("P1", 0.13, 0.9, prep=[2.3]), row("P1", 0.12, 0.95, prep=[2.2]),
                     row("P2", 0.12, 0.03, prep=[2.3, 1.0]), row("P2", 0.13, 0.02, prep=[2.4, 0.9])]
    out = g5.reading(g5.arm_summary(rows))
    assert any(o.startswith("stall 1: first render-graph") for o in out)
    assert any(o.startswith("stall 2: first render-graph") for o in out)
    assert not any("undecided" in o for o in out)


def test_short_prep_renders_with_the_stalls_still_there_are_undecided_not_rejected():
    rows = base() + [row("P2", 2.3, 1.0, prep=[0.05, 0.04]), row("P2", 2.2, 0.99, prep=[0.05, 0.04])]
    out = g5.reading(g5.arm_summary(rows))
    assert any("P2: its prep renders each took at most 0.25 s" in o for o in out)


def test_each_stall_on_its_own_a_split_arm_and_a_moved_stall():
    rows = base() + [row("W", 0.1, 1.0), row("W", 0.4, 0.98), row("P2", 0.1, 0.03, prep=[2.3, 1.0], later=[{"row": 40}]),
                     row("P2", 0.1, 0.02, prep=[2.3, 1.0])]
    out = g5.reading(g5.arm_summary(rows))
    assert "W stall 1 split between its sessions: a third W session is due" in out
    assert any(o.startswith("P2: a later render step") for o in out)
    s = g5.arm_summary(rows)
    assert s["W"]["stall1"]["reduced"] and not s["W"]["stall2"]["reduced"]


def test_the_oracle_arm_counts_only_with_p0s_labels():
    good = base() + [row("O", 0.1, 0.02), row("O", 0.12, 0.03)]
    assert any("learned worker" in o for o in g5.reading(g5.arm_summary(good)))
    other = base() + [row("O", 0.1, 0.02, labels=[["/World/P_1", "inst_0"]]), row("O", 0.1, 0.02, labels=[])]
    assert any("labels differ" in o for o in g5.reading(g5.arm_summary(other)))
