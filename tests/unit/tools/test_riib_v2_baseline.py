"""v2 baseline rules (docs/plans/2026-11-08-live-riib-defaults-v2.md 2, 2.1): F, R and the missing rules on an
interval, the held rule against the frozen 1-concurrent values, the catch-up setting check, and the mechanical
stage-2 targets."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import riib_v2_baseline as B  # noqa: E402

DT = 1 / 120


def steps(n, wall_per_step, ok=lambda k: True, missing=()):
    return {k: (k * wall_per_step, k * DT, ok(k)) for k in range(n) if k not in missing}


def test_factor_and_false_share_on_an_interval():
    s = steps(1201, DT / 0.28, ok=lambda k: not (100 <= k < 200))  # 100 of 1200 intervals false
    st = B.interval_stats(s, 0.0, 1e9)
    assert st["F"] == pytest.approx(0.28)
    assert st["R"] == pytest.approx(100 / 1200)
    assert st["missing_steps"] == 0 and st["decidable"]


def test_missing_steps_drop_out_and_their_share_can_make_a_session_undecidable():
    s = steps(1001, DT / 0.3, missing=range(500, 520))  # 20 missing of 1001 expected: 2 %
    st = B.interval_stats(s, 0.0, 1e9)
    assert st["missing_steps"] == 20 and not st["decidable"] and "missing steps" in st["why"]
    gap = steps(1001, DT / 0.3, missing=range(500, 505))  # 0.5 % of steps, but a long silence there
    gap = {k: (m + (8.0 if k >= 505 else 0.0), t, o) for k, (m, t, o) in gap.items()}
    st2 = B.interval_stats(gap, 0.0, 1e9)
    assert not st2["decidable"] and "gap" in st2["why"]


def test_held_needs_every_session_within_the_frozen_budget():
    f1, r1 = 0.290, 0.10
    assert B.held([{"F": 0.282, "R": 0.12, "eligible": True}], f1, r1) is True  # 0.282 >= 0.2813, 0.12 <= 0.13
    assert B.held([{"F": 0.281, "R": 0.12, "eligible": True}], f1, r1) is False
    assert B.held([{"F": 0.29, "R": 0.131, "eligible": True}], f1, r1) is False
    assert B.held([{"F": 0.29, "R": 0.1, "eligible": False}], f1, r1) is None  # undecidable, not 'not held'


def test_the_catchup_setting_check_reads_short_gaps():
    spacing = DT / 0.3
    no_short = [spacing, spacing + 0.002, 0.3]
    with_short = [0.05, spacing - 0.004, spacing]
    assert B.setting_ok(0, no_short, spacing) and not B.setting_ok(6, no_short, spacing)
    assert B.setting_ok(6, with_short, spacing) and not B.setting_ok(0, with_short, spacing)


def test_stage_two_targets_come_from_the_catchup_6_arm_and_collapse_when_equal():
    held6 = {1: True, 2: True, 3: True}
    assert B.stage2_targets(held6) == [3, 2]
    assert B.stage2_targets({1: True, 2: False, 3: True}) == [3, 1]
    assert B.stage2_targets({1: True, 2: True, 3: False}) == [2]  # the largest held N and the A/B N coincide
    assert B.stage2_targets({1: False, 2: False, 3: False}) is None


def test_no_stage_two_when_an_arm_has_no_held_n_a_low_f1_or_an_undecidable_concurrency():
    ok = {"held": {1: True, 2: True, 3: True}, "F1": 0.29}
    assert B.stage2_blockers({0: ok, 6: ok}) == []
    assert B.stage2_blockers({0: {"held": {1: False, 2: False, 3: False}, "F1": 0.29}, 6: ok})
    assert B.stage2_blockers({0: ok, 6: {"held": ok["held"], "F1": 0.268}})
    assert B.stage2_blockers({0: {"held": {1: True, 2: None, 3: True}, "F1": 0.29}, 6: ok})


def test_a_log_refused_even_when_incomplete_is_allowed_leaves_the_session_undecidable(monkeypatch):
    def refuse(run, allow_incomplete=False):
        raise ValueError("live log parts missing")

    monkeypatch.setattr(B, "read_live_log", refuse)
    entries, complete, error = B.read_log("run")
    assert entries == [] and complete is False and "missing" in error

    calls = []

    def strict_only(run, allow_incomplete=False):
        calls.append(allow_incomplete)
        if not allow_incomplete:
            raise ValueError("no end summary")
        return [{"step": 1}]

    monkeypatch.setattr(B, "read_live_log", strict_only)
    assert B.read_log("run") == ([{"step": 1}], False, "no end summary") and calls == [False, True]
