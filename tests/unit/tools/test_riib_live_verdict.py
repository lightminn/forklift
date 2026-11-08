"""The live riib-defaults verdict's rules (registration 3.3, 3.4): the substitute's overlap check, the uncertain band,
the undecidable span, C2 steps apart, and the false-positive check that has no cancel source."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import riib_live_verdict as V  # noqa: E402


def steps_of(monos, ok=True):
    return {k: (m, k / 120.0, ok) for k, m in monos.items()}


def test_no_interpolation_across_a_missing_step():
    assert V.to_sim(steps_of({0: 0.0, 120: 10.0}), 5.0) is None  # Codex: steps 0 and 120 only
    assert V.to_sim(steps_of({0: 0.0, 1: 0.1}), 0.05) == pytest.approx(0.5 / 120.0)
    assert V.to_sim(steps_of({0: 0.0, 1: 0.1}), 0.2) is None  # outside the records


def test_the_overlap_check_needs_the_same_classification_and_a_small_difference():
    tg = {1: 0.10, 2: 0.26, 3: 0.03}
    assert V.overlap_check(tg, {1: 0.12, 2: 0.27, 3: 0.04})["passed"]
    assert not V.overlap_check(tg, {1: 0.12, 2: 0.24, 3: 0.04})["passed"]  # 0.26 vs 0.24: the 0.25 s class differs
    assert not V.overlap_check(tg, {1: 0.20, 2: 0.27, 3: 0.04})["passed"]  # a 0.10 s difference
    assert not V.overlap_check({}, {1: 0.1})["passed"]


def test_the_substitute_span_counts_its_band_uncertain_and_the_trace_span_never():
    monos = {k: k * 0.03 for k in range(10)}
    steps = steps_of(monos)
    tg = {1: 0.27, 2: 0.03}  # the trace: one confirmed stall at step 1 (0.27 is inside the band, but from the trace)
    lg = {3: 0.27, 4: 0.40, 5: 0.03, 6: 0.22}  # the substitute: 0.27 and 0.22 uncertain, 0.40 confirmed
    out = V.pace(steps, tg, lg, True, 0.0, 1.0, set(), trace_last=2)
    full, trace_only = out["full"], out["trace_only"]
    assert (full["stalls_confirmed"], full["stalls_uncertain"], full["gaps_from_log"]) == (2, 2, 4)
    assert (trace_only["stalls_confirmed"], trace_only["stalls_uncertain"]) == (1, 0)
    failed = V.pace(steps, tg, lg, False, 0.0, 1.0, set(), trace_last=2)["full"]
    assert failed["undecidable_steps"] == 4 and failed["stalls_confirmed"] == 1  # the failed overlap: no substitute


def test_c2_steps_are_apart_and_never_take_the_substitute():
    steps = steps_of({k: k * 0.03 for k in range(6)})
    out = V.pace(steps, {1: 0.03, 2: 0.5}, {4: 1.9}, True, 0.0, 1.0, {2, 4}, trace_last=2)["full"]
    assert out["stalls_confirmed"] == 0
    c2 = {c["step"]: c for c in out["c2_steps"]}
    assert c2[2]["gap_s"] == 0.5 and c2[2]["source"] == "trace"
    assert c2[4]["gap_s"] is None and c2[4]["source"] == "none" and c2[4]["log_gap_s"] == 1.9


def test_the_longest_pacing_false_run_breaks_at_a_missing_step():
    steps = {0: (0.0, 0.0, False), 1: (1.0, 0.1, False), 2: (2.0, 0.2, False), 4: (4.0, 0.4, False),
             5: (9.0, 0.5, False)}
    assert V.pace(steps, {}, {}, True, 0.0, 10.0, set(), None)["full"]["longest_pacing_false_s"] == 5.0


def test_a_false_positive_check_is_always_undecidable_for_want_of_a_cancel_source():
    steps = steps_of({0: 0.0, 1: 0.1, 2: 0.2})
    entry = {"stall_recovery": {"stall_span_mono": {"since": 0.05, "decided": 0.15}}}
    guard = {"holds": [{"since_s": 0.005, "last_s": 0.006}], "holds_saturated": False}  # inside the span (sim 0.0042-0.0125 s)
    events = [{"event": "reply", "kind": "mission_cancel", "request_id": "r4", "mono": 0.19, "rtt_s": 0.02}]
    entries = [{"t": "mission_cancel", "request_id": "r4", "mono": 0.18}]
    out = V.false_positive(entry, steps, guard, entries, events, True)
    assert out["verdict"] == "undecidable" and "no source" in out["why"]
    assert out["overlap_if_mapped"] == "guard_stop" and out["clock_check"]["ordered"]
    assert V.false_positive(entry, steps, guard, entries, events, False)["why"] == "the live log is incomplete"
