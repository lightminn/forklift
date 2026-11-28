"""The commissioning tool's MT asks (bench console plan 4.5 A1c decision 11): the source by axis, the explicit sign, the
distance options and their exclusions, the units from vehicle.yaml, the encoder bypass with a feedback run, the hold's
length on a wheel."""
import argparse
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import bringup  # noqa: E402


def args(**kw):
    base = dict(cmd="commission", axis="L", duty=0.1, seconds=0.6, window=None, observed=None, yes=True,
                power_switch_in_hand=True, count_sign=None)
    return argparse.Namespace(**{**base, **kw})


def refused(**kw) -> str:
    with pytest.raises(bringup.Refused) as e:
        bringup._check_commission_args(args(**kw))
    return str(e.value)


def test_an_mt_distance_by_counts_and_by_motor_turns():
    a, _p, _n = bringup._check_commission_args(args(stop_counts=2000, count_sign=1))
    ask = bringup._bench_ask(args(stop_counts=2000, count_sign=1))
    assert (ask.source, ask.stop_counts, ask.axis) == (2, 2000, 0)
    assert ask.units["motor_cpr"] == 4096 and ask.units["measured"] is False  # fixed with the record
    ask = bringup._bench_ask(args(axis="R", duty=-0.1, stop_turns=0.5, count_sign=1))
    assert (ask.source, ask.stop_counts) == (2, -2048)  # half a MOTOR shaft turn, signed by the duty
    ask = bringup._bench_ask(args(measure=True, count_sign=-1))
    assert (ask.source, ask.stop_counts, ask.count_sign) == (2, 0, -1)


def test_kth_options_stay_kth():
    assert "KTH count is M's or H's" in refused(stop_kth_turns=1.0)  # never re-read as an MT run (plan round 1 P1-5)
    assert "KTH's" in refused(axis="M", window=(-0.3, 0.3), stop_kth_turns=1.0, source="mt")
    ask = bringup._bench_ask(args(axis="M", stop_turns=0.25))
    assert (ask.source, ask.stop_counts, ask.count_sign) == (1, 1024, 1)  # KTH keeps the +1 default


def test_an_mt_run_needs_its_sign():
    assert "count-sign: required for an MT run" in refused(stop_counts=2000)
    assert "count-sign: required for an MT run" in refused(measure=True)
    assert "count-sign: required for an MT run" in refused(hold="here", duty=0.15, seconds=2.0)


@pytest.mark.parametrize(("kw", "words"), [
    (dict(stop_counts=100, stop_turns=1.0), "one of"),
    (dict(stop_kth_turns=1.0, stop_counts=100), "one of"),
    (dict(stop_counts=100, measure=True), "not a measuring run"),
    (dict(measure=True, kth_measure=True), "not both"),
    (dict(source="mt"), "only with"),
    (dict(hold="here", duty=0.15, seconds=2.0, stop_counts=100), "no distance or measuring"),
    (dict(stop_counts=0), "1 .. 2^31-1"),
    (dict(stop_counts=2000, duty=0.00004), "rounds to 0 on the wire"),
    (dict(axis="M", window=(-0.3, 0.3), source="mt", measure=True), "MT count is L's or R's"),
])
def test_the_options_exclude_each_other(kw, words):
    assert words in refused(count_sign=1, **kw)


def test_the_encoder_bypass_with_a_feedback_run():
    assert "needs its encoder checks" in refused(stop_counts=2000, count_sign=1, bypass="enc_stuck")
    assert "needs its encoder checks" in refused(hold="here", duty=0.15, seconds=2.0, count_sign=1, bypass="enc_polarity")
    # a measuring run may bypass them, as a duty run does (plan 2: only the feedback runs need the detection)
    bringup._check_commission_args(args(measure=True, count_sign=1, bypass="enc_stuck"))


def test_an_mt_hold_on_a_wheel(monkeypatch):
    ask = bringup._bench_ask(args(hold="rel:200", duty=0.15, seconds=2.0, count_sign=1))
    assert ask.hold_run and ask.source == 2 and (ask.hold.source, ask.hold.cpr) == (2, 4096)
    assert "relative" in refused(hold="zero", duty=0.15, seconds=2.0, count_sign=1)
    assert "a hold runs 1.2" in refused(hold="here", duty=0.15, seconds=0.6, count_sign=1)
    assert "a hold runs 1.2" in refused(hold="here", duty=0.15, seconds=61.0, count_sign=1, wheels_lifted=True)
    # the hold's length is not its continuous output: a run time limit below it is no refusal (Codex A1c-2 r1 P2-2)
    bringup._check_commission_args(args(hold="here", duty=0.15, seconds=1.4, count_sign=1, bench_time_max=1.0))
    bringup._check_commission_args(args(hold="here", duty=0.15, seconds=30.0, count_sign=1, wheels_lifted=True))  # a hold's length, not hold_max (30 s is a large wheel run)


def test_no_motor_cpr_refuses_turns_and_hold_but_not_counts(tmp_path, monkeypatch):
    y = tmp_path / "vehicle.yaml"
    y.write_text("wheel_counts:\n  left: {counts_per_wheel_rev: 327680}\n"
                 "  right: {motor_counts_per_rev: 8192, gear_ratio: 40.0}\n")
    monkeypatch.setattr(bringup, "VEHICLE_YAML", y)
    assert "no_motor_cpr" in refused(stop_turns=1.0, count_sign=1)
    assert "no_motor_cpr" in refused(hold="here", duty=0.15, seconds=2.0, count_sign=1)
    assert bringup._bench_ask(args(stop_counts=500, count_sign=1)).stop_counts == 500
    ask = bringup._bench_ask(args(axis="R", stop_turns=1.0, count_sign=1))
    assert ask.stop_counts == 8192 and ask.units["motor_cpr"] == 8192  # R's own CPR, not L's


def test_the_new_options_are_bench_arguments():
    for kw in (dict(stop_turns=1.0), dict(stop_counts=10), dict(measure=True), dict(source="mt")):
        assert bringup._bench_args_given(args(**kw)), kw  # never read as a plain timed duty run (indep r1 P2-7)
