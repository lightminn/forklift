"""The commissioning tool's MT runs on the bench8 model (bench console plan 4.5 A1c-2): a wheel's distance stop, the
wrong way and a frozen encoder stopped by the board's watch, a wheel lost from boot refusing its MT arming, an MT hold
on its motor shaft CPR. The model's plant is a first-order wheel: these check the count handling and the state
transitions, not a real MT6701 (plan 4.5 decision 14)."""
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))

import bringup  # noqa: E402
from test_bringup_commission_tool import args, kinds_written, model  # noqa: E402,F401
from test_bringup_tool import mcu_model  # noqa: E402,F401


def rows(b):
    return [json.loads(x) for x in b.out.getvalue().splitlines()]


def test_an_mt_distance_run_on_l_stops_at_its_counts(model):
    b = model("--bench", "1")
    res = bringup.execute(args(axis="L", duty=0.1, seconds=1.0, stop_counts=2000, count_sign=1, observed="none", wheels_lifted=True), b)
    run = res["bench_run"]
    assert run["confirmed"] and run["stop_reason"] == "distance_reached" and run["source"] == 2, res
    assert res["distance"]["result"] == "reached", res
    advisories = [r["advisory"] for r in rows(b) if r["t"] == "bench_advisory"]
    assert "mt_signal_unverified" in advisories  # a count crossing, not a verified wheel motion
    (cond,) = [r for r in rows(b) if r["t"] == "run_conditions"]
    assert cond["bench"]["source"] == 2 and cond["bench"]["units"]["motor_cpr"] == 4096  # the units fixed
    rest = [r for r in rows(b) if r["t"] == "bench_rest_position"]
    assert rest and rest[-1]["result"] in ("unchanged_at_100hz", "unconfirmed")  # never "settled" for MT
    pairs = [r for r in rows(b) if r["t"] == "bench_pair"]
    assert pairs and "result_source" in pairs[-1] and "result_axis" in pairs[-1]


def test_the_wrong_way_and_a_frozen_encoder_are_stopped_by_the_board(model):
    for extra in (("--encoder-sign-flip", "0"), ("--encoder-frozen", "0")):
        b = model("--bench", "1", *extra)
        res = bringup.execute(args(axis="L", duty=0.1, seconds=1.0, stop_counts=50000, count_sign=1,
                                   observed="none", wheels_lifted=True), b)
        run = res["bench_run"]
        assert run["confirmed"] and run["stop_reason"] == "no_progress", (extra, res)
        assert res["distance"]["result"] == "stopped:no_progress", (extra, res)


def test_a_wheel_lost_from_boot_refuses_its_mt_arming(model):
    """The model's L reads fail from the start (the first read is taken): past 20 ms the wheel is lost, reported
    only, and its MT arming is refused (plan 4.5 decision 4); nothing drives."""
    b = model("--bench", "1", "--encoder-read-fail", "0:0:1000000")
    res = bringup.execute(args(axis="L", duty=0.1, seconds=1.0, measure=True, count_sign=1, observed="none", wheels_lifted=True), b)
    assert res["result"] != "ok", res
    assert "BENCH_ARM" in kinds_written(b)
    assert not any(k == "COMMISSION" and m.duty_e4 for k, _t, m in b.written.values())  # no non-zero sent
    assert not any(s.state.duty_e4[0] for s in b.samples)  # L never driven


def test_an_mt_hold_sends_its_source_and_motor_cpr(model):
    b = model("--bench", "1")
    res = bringup.execute(args(axis="L", duty=0.15, seconds=1.3, hold="here", count_sign=1, observed="none", wheels_lifted=True), b)
    holds = [m for k, _t, m in b.written.values() if k == "BENCH_HOLD"]
    assert holds and all(m.source == 2 and m.cpr == 4096 and m.axis == 0 for m in holds), res
    v2 = res["hold"]["verdict"]
    # a still wheel under here: no qualified progress, so no MT evidence -- held, with what was seen (decision 8)
    assert v2["result"] == "held" and v2["why"] == ["mt_evidence_age"], v2
