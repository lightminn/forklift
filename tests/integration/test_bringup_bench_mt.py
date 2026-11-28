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
from test_bringup_commission_tool import _typed, args, kinds_written, model  # noqa: E402,F401
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
    assert rest and rest[-1]["result"] == "unchanged_at_100hz", rest  # 1.0 s window; never "settled" for MT
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


def test_an_mt_distance_run_on_r_backwards(model):
    b = model("--bench", "1")
    res = bringup.execute(args(axis="R", duty=-0.1, seconds=1.0, stop_counts=1500, count_sign=1, observed="none",
                               wheels_lifted=True), b)
    assert res["bench_run"]["stop_reason"] == "distance_reached" and res["distance"]["result"] == "reached", res
    assert res["bench_run"]["axis"] == "R" and res["bench_run"]["distance_ref"] > res["bench_run"]["cut_position"]


def test_a_read_failure_while_driving_is_a_fault_and_withheld(model):
    """L's reads fail 600 ms after the arming (after the 0.3 s readiness zero, while driving) for 40 ticks: past
    20 ms the measured wheel's loss faults the run (plan 4.5 decision 4); the measuring verdict is withheld."""
    b = model("--bench", "1", "--encoder-read-fail-armed", "0:600:40")
    res = bringup.execute(args(axis="L", duty=0.1, seconds=1.0, measure=True, count_sign=1, observed="none",
                               wheels_lifted=True), b)
    assert res["bench_result"]["stop_reason"] == "fault", res
    assert res["measure"]["status"] == "withheld:continuity", res


def test_a_moving_mt_hold_that_settles_is_held_after_1_s(model):
    """rel:600 on L: the wheel moves (qualified progress, evidence), then stands still in the band for more than 1 s
    before the end: held by the MT evidence age, with the time it was seen inside (plan 4.5 decision 8)."""
    b = model("--bench", "1")
    res = bringup.execute(args(axis="L", duty=0.15, seconds=1.45, hold="rel:600", count_sign=1, observed="none",
                               wheels_lifted=True), b)
    v2, br = res["hold"]["verdict"], res["bench_result"]
    # the model's settling time varies with the host's timing: held when the last qualified progress is more than 1 s
    # before the end (the usual case), met when it lands within it -- each checked by its own evidence age
    if v2["result"] == "held":
        # moved, then still: evidence older than 1 s but present (0xFFFF would be no progress at all -- r3 P2-2)
        assert v2["why"] == ["mt_evidence_age"] and 1000 < br["end_evidence_age_ms"] < 0xFFFF, (v2, br)
        assert v2["inside_s"] > 0.5, v2
    else:
        assert v2["result"] == "met" and br["end_evidence_age_ms"] <= 1000, (v2, br)


def test_another_vehicle_cpr_reaches_the_wire(model, tmp_path, monkeypatch):
    y = tmp_path / "vehicle.yaml"
    y.write_text("wheel_counts:\n  left: {motor_counts_per_rev: 8192, gear_ratio: 40.0}\n"
                 "  right: {motor_counts_per_rev: 4096, gear_ratio: 80.0}\n")
    monkeypatch.setattr(bringup, "VEHICLE_YAML", y)
    b = model("--bench", "1")
    res = bringup.execute(args(axis="L", duty=0.15, seconds=1.3, hold="here", count_sign=1, observed="none",
                               wheels_lifted=True), b)
    holds = [m for k, _t, m in b.written.values() if k == "BENCH_HOLD"]
    assert holds and all(m.cpr == 8192 for m in holds), res
    (cond,) = [r for r in rows(b) if r["t"] == "run_conditions"]
    assert cond["bench"]["units"]["motor_cpr"] == 8192 and cond["bench"]["units"]["gear_ratio"] == 40.0


def test_a_read_failure_while_a_distance_drives_fails_its_continuity(model):
    b = model("--bench", "1", "--encoder-read-fail-armed", "0:600:40")
    res = bringup.execute(args(axis="L", duty=0.1, seconds=1.0, stop_counts=60000, count_sign=1, observed="none",
                               wheels_lifted=True), b)
    assert res["bench_result"]["stop_reason"] == "fault", res
    assert res["distance"]["result"] == "failed_continuity", res


def test_a_wheel_that_keeps_creeping_is_never_unchanged(model):
    """The model turns L slowly by hand from its start (1 wheel turn over 10 s, about 33 counts/ms): after the cut the
    count keeps changing, so the rest is unconfirmed, not unchanged (Codex A1c-2 r3 P2-3)."""
    b = model("--bench", "1", "--hand-turn-wheel", "0", "--hand-turn-revs", "1", "--hand-turn-ms", "0",
              "--hand-turn-duration-ms", "10000")
    bringup.execute(args(axis="L", duty=0.1, seconds=1.0, stop_counts=2000, count_sign=1, observed="none",
                         wheels_lifted=True), b)
    rest = [r for r in rows(b) if r["t"] == "bench_rest_position"]
    assert rest and rest[-1]["result"] == "unconfirmed" and rest[-1]["why"] == "not_settled", rest


def test_an_mt_hold_disturbed_and_returned_is_met(model, monkeypatch):
    """The met vector (plan 4.5 decision 8; the parameters Codex A1c-2 r4 found): the hold settles, then from 15 s a
    hand turn pushes L for 5 s; the hold's returns are qualified progress, so the end evidence is fresh (< 1 s) and the
    settling judgement says met. A large wheel run: the operator's typed line is the test's."""
    _typed(monkeypatch)
    b = model("--bench", "1", "--hand-turn-wheel", "0", "--hand-turn-revs", "1", "--hand-turn-ms", "15000",
              "--hand-turn-duration-ms", "5000")
    res = bringup.execute(args(axis="L", duty=0.25, seconds=20.2, hold="here", count_sign=1, hold_min_duty=0.18,
                               hold_kp=0.45, hold_deadband=256, observed="none", wheels_lifted=True), b)
    v2, br = res["hold"]["verdict"], res["bench_result"]
    assert br["end_evidence_age_ms"] <= 1000, (v2, br)
    assert v2["result"] == "met", (v2, br)
