"""Real-time plan 5.24: the G5-0 arms' registered reading -- each stall on its own, decided only with enough valid
sessions and a P0 that shows it, a split arm waits for its third session, a moved stall is never credited, short
prep renders leave the render reading undecided, and the oracle arm counts only alone and with P0's labels (Codex 5.24
review: the counterexamples below)."""

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


def read(rows):
    return g5.reading(g5.arm_summary(rows))


def test_prep_renders_that_take_the_stalls_read_as_the_render_graph():
    out = read(base() + [row("P1", 0.13, 0.9, prep=[2.3]), row("P1", 0.12, 0.95, prep=[2.2]),
                         row("P2", 0.12, 0.03, prep=[2.3, 1.0]), row("P2", 0.13, 0.02, prep=[2.4, 0.9])])
    assert any(o.startswith("stall 1: first render-graph") for o in out)
    assert any(o.startswith("stall 2: first render-graph") for o in out)
    assert not any("undecided" in o for o in out)


def test_short_prep_renders_with_the_stalls_still_there_are_undecided_not_rejected():
    out = read(base() + [row("P2", 2.3, 1.0, prep=[0.05, 0.04]), row("P2", 2.2, 0.99, prep=[0.05, 0.04])])
    assert any("P2: its prep renders each took at most 0.25 s" in o for o in out)


def test_a_moved_stall_is_never_credited():
    out = read(base() + [row("P2", 0.1, 0.03, prep=[0.1, 0.1], later=[{"row": 40}]),
                         row("P2", 0.1, 0.02, prep=[0.1, 0.1])])
    assert any(o.startswith("P2: a later render step") for o in out)
    assert not any(o.startswith("stall 1: first render-graph") for o in out)


def test_too_few_sessions_or_no_p0_or_a_p0_without_the_stall_decide_nothing():
    out = read(base() + [row("P1", 0.1, 0.9, prep=[2.3])])  # one P1 session left valid
    assert "P1 stall 1: fewer than 2 valid sessions -- undecided" in out
    assert not any(o.startswith("stall 1: first render-graph") for o in out)
    out = read([row("P2", 0.1, 0.02, prep=[2.3, 1.0]), row("P2", 0.1, 0.02, prep=[2.3, 1.0])])  # no P0
    assert any(o.startswith("stall 1: P0 does not reproduce it") for o in out)
    out = read([row("P0", 0.1, 0.02), row("P0", 0.1, 0.02), row("P2", 0.1, 0.02, prep=[0.2, 0.1]),
                row("P2", 0.1, 0.02, prep=[0.2, 0.1])])  # P0 without the stall
    assert any(o.startswith("stall 1: P0 does not reproduce it") for o in out)


def test_a_split_arm_waits_for_its_third_session_then_its_median_decides():
    rows = base() + [row("W", 0.1, 1.0), row("W", 0.4, 0.98)]
    out = read(rows)
    assert "W stall 1 split between its sessions: a third W session is due" in out
    assert "stall 1: an arm is split -- undecided until its third session" in out
    out = read(rows + [row("W", 0.2, 0.99)])  # the third: median 0.2 s
    assert not any("split" in o for o in out)
    assert any(o.startswith("stall 1: time-based") for o in out)


def test_the_oracle_arm_counts_only_alone_and_with_p0s_labels():
    out = read(base() + [row("O", 0.1, 0.02), row("O", 0.12, 0.03)])
    assert any("learned worker" in o for o in out)
    # P1 removes stall 1 too: then it is not the oracle alone
    out = read(base() + [row("O", 0.1, 0.02), row("O", 0.12, 0.03), row("P1", 0.1, 0.9, prep=[2.3]),
                         row("P1", 0.1, 0.95, prep=[2.3])])
    assert not any(o.startswith("stall 1: the learned worker") for o in out)
    # labels missing on both sides: undecided, not equal
    out = read([row("P0", 2.3, 1.0, labels=None), row("P0", 2.3, 1.0, labels=None),
                row("O", 0.1, 0.02, labels=None), row("O", 0.1, 0.02, labels=None)])
    assert any("labels are not P0's (or missing)" in o for o in out)
