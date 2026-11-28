"""bench8 MT source rules shared by the CLI and the console (bench console plan 4.5 A1c decisions 5-11): the units from
vehicle.yaml, the distance stops, the source pairs, the MT hold's checks, the MT rest position and the MT hold verdict."""
import math
from types import SimpleNamespace

import pytest

from forklift_core.mcu_link.messages import (
    BenchCountState,
    BenchDriveState,
    BenchRunFlag,
    Mode,
)
from forklift_core.plant import bench_limits as bl
from forklift_core.plant import bench_session as bs


def units_of(left: dict, right: dict):
    return bl.mt_units_from_mapping({"wheel_counts": {"left": left, "right": right}})


def test_the_units_keep_the_motor_shaft_cpr():
    left, right = units_of({"motor_counts_per_rev": 4096, "gear_ratio": 80.0, "provenance": {"measured": False}},
                    {"motor_counts_per_rev": 8192, "gear_ratio": 40.0, "provenance": {"measured": True}})
    # the same counts per wheel turn, two different motor shaft CPRs (plan 4.5 decision 11)
    assert left.counts_per_wheel_rev == right.counts_per_wheel_rev == 327680
    assert (left.motor_cpr, right.motor_cpr) == (4096, 8192) and not left.measured and right.measured
    assert left.record()["motor_cpr"] == 4096 and left.record()["no_motor_cpr"] is None


@pytest.mark.parametrize(("entry", "why"), [
    ({"counts_per_wheel_rev": 327680}, "counts per wheel turn only"),
    ({"motor_counts_per_rev": 4096.5, "gear_ratio": 80.0}, "not whole"),
    ({"motor_counts_per_rev": 70000, "gear_ratio": 80.0}, "outside"),
    ({"motor_counts_per_rev": 0, "gear_ratio": 80.0}, "finite positive"),
])
def test_a_side_without_a_usable_motor_cpr(entry, why):
    left, right = units_of(entry, {"motor_counts_per_rev": 4096, "gear_ratio": 80.0})
    assert left.motor_cpr is None and why in left.why
    assert right.motor_cpr == 4096  # the other side keeps its own


def test_the_real_vehicle_yaml_reads():
    from pathlib import Path

    left, right = bl.mt_units(Path(__file__).resolve().parents[3] / "config" / "real" / "vehicle.yaml")
    assert left.motor_cpr == right.motor_cpr == 4096 and left.gear_ratio == 80.0 and not left.measured


def test_distance_stops_in_turns_and_counts():
    assert bl.turns_stop_counts(1.0, 4096, 0.2, 1) == 4096  # one motor shaft turn = 1/80 of a wheel turn
    assert bl.turns_stop_counts(0.5, 8192, -0.2, 1) == -4096
    assert bl.turns_stop_counts(1.0, 4096, 0.2, -1) == -4096
    assert bl.counts_stop(40, 0.2, 1) == 40 and bl.counts_stop(40, -0.2, 1) == -40 and bl.counts_stop(40, 0.2, -1) == -40
    for bad in (0, -5, 2 ** 31, True, 3.0, "40"):
        with pytest.raises(bl.LimitRefused):
            bl.counts_stop(bad, 0.2, 1)  # 0 means no stop on the wire; a bool or a float is not a count
    with pytest.raises(bl.LimitRefused):
        bl.counts_stop(40, 0.0, 1)
    with pytest.raises(bl.LimitRefused):
        bl.turns_stop_counts(1e-6, 4096, 0.2, 1)  # rounds to 0 counts


def test_the_source_pairs():
    ok = [bl.BenchAsk(0, (), None, None, source=2, stop_counts=40), bl.BenchAsk(1, (), None, None, source=2),
          bl.BenchAsk(2, (), None, None, source=1, stop_counts=40)]
    assert all(bl.bench_refusal(a) is None for a in ok)
    assert "MT count is L's or R's" in bl.bench_refusal(bl.BenchAsk(2, (), None, None, source=2))
    assert "KTH count is M's or H's" in bl.bench_refusal(bl.BenchAsk(0, (), None, None, source=1))
    assert "measured source" in bl.bench_refusal(bl.BenchAsk(0, (), None, None, source=0, stop_counts=40))


def test_the_mt_hold_checks():
    mt = dict(source=2, cpr=4096)
    assert bl.hold_refusal(bl.HoldAsk(2, 0, **mt), 1500) is None  # here
    assert bl.hold_refusal(bl.HoldAsk(3, 200, **mt), 1500) is None  # rel
    assert "relative" in bl.hold_refusal(bl.HoldAsk(0, 0, **mt), 1500)  # zero
    assert "relative" in bl.hold_refusal(bl.HoldAsk(1, 100, **mt), 1500)  # abs
    assert "CPR" in bl.hold_refusal(bl.HoldAsk(2, 0, source=2, cpr=0), 1500)
    assert "KTH CPR" in bl.hold_refusal(bl.HoldAsk(2, 0, source=1, cpr=8192), 1500)
    ask = bl.BenchAsk(0, (), None, None, source=2, mode=2, hold=bl.HoldAsk(2, 0, source=1))
    assert "its source" in bl.bench_refusal(ask)
    assert bl.bench_refusal(bl.BenchAsk(1, (), None, None, source=2, mode=2, hold=bl.HoldAsk(2, 0, **mt))) is None


def pair(t_us, cum, *, age=0, run=7, flags=0, count_state=int(BenchCountState.VALID), source=2, axis=0, gen=3):
    return (t_us, SimpleNamespace(run_id=run, source=source, axis=axis, count_state=count_state, run_flags=flags,
                                  raw_angle=age, cumulative=cum, evidence_age_ms=0xFFFF, conn_gen=gen))


def test_the_mt_rest_position():
    still = [pair(10_000 * k, 500) for k in range(60)]
    out = bs.rest_position(still, source=2)
    assert out["result"] == "unchanged_at_100hz" and out["cumulative"] == 500  # never "settled" for MT
    # inertia first, then still for 0.8 s: the backward segment is enough (Codex plan round 2 P2-3)
    moving = [pair(10_000 * k, 400 + 5 * k) for k in range(20)] + [pair(200_000 + 10_000 * k, 495) for k in range(80)]
    assert bs.rest_position(moving, source=2)["result"] == "unchanged_at_100hz"
    stale = still[:-1] + [pair(600_000, 500, age=3)]  # the last pair's read failed: an older sample's count
    assert bs.rest_position(stale, source=2) == {"result": "unconfirmed", "cumulative": 500, "why": "stale_read"}
    lost = still[:-1] + [pair(600_000, 500, flags=int(BenchRunFlag.UNCLEAN))]
    assert bs.rest_position(lost, source=2)["why"] == "continuity"
    short = [pair(10_000 * k, 500) for k in range(30)]
    assert bs.rest_position(short, source=2)["why"] == "too_short"
    # two short stretches across a board connection change never join (Codex A1c-2 r1 P2-1)
    joined = [pair(10_000 * k, 500, gen=3) for k in range(30)] + [pair(300_000 + 10_000 * k, 500, gen=4)
                                                                   for k in range(30)]
    assert bs.rest_position(joined, source=2) == {"result": "unconfirmed", "cumulative": 500, "why": "continuity"}
    # the KTH rule is unchanged: the same MT-shaped samples are not re-anchored there
    assert bs.rest_position(still, source=1)["result"] == "unconfirmed"


def hold_pair(t_us, cum, *, age=0, drive=BenchDriveState.DRIVING, run=9, count_state=int(BenchCountState.VALID)):
    b = SimpleNamespace(run_id=run, mode=2, drive_state=drive, target=1000, cumulative=cum, raw_angle=age,
                        result_run_id=run, count_state=count_state)
    return (t_us, Mode.ARMED, b)


def test_a_settled_mt_hold_is_held_with_what_it_saw():
    pairs = [hold_pair(10_000 * k, 1000) for k in range(200)]
    result = {"stop_reason": "end_zero", "run_flags": int(BenchRunFlag.RES_HOLD), "end_evidence_age_ms": 1800,
              "target": 1000, "cut_position": 1000}
    v = bs.hold_verdict(pairs, result, deadband=12, run_id=9, source=2)
    assert v["result"] == "held" and v["why"] == ["mt_evidence_age"]
    assert v["inside_s"] == pytest.approx(1.99) and v["inside_share"] == 1.0 and "확인할 수 없음" in v["note"]
    # within 1 s the MT evidence is enough (KTH needs 0.1 s): the settling judgement runs
    result["end_evidence_age_ms"] = 600
    pairs.append((2_000_000, Mode.DISARMED, SimpleNamespace(run_id=9, mode=2, drive_state=BenchDriveState.ENDED,
                                                            target=1000, cumulative=1000, raw_angle=0,
                                                            result_run_id=9)))
    assert bs.hold_verdict(pairs, result, deadband=12, run_id=9, source=2)["result"] == "met"
    assert bs.hold_verdict(pairs, result, deadband=12, run_id=9, source=1)["result"] == "held"  # KTH: 0.1 s


def test_the_seen_time_counts_only_contiguous_inside_pairs():
    """Two inside pairs 2 s apart, or pairs outside between, are not 2 s seen inside (Codex A1c-2 r1 P2-3)."""
    result = {"stop_reason": "end_zero", "run_flags": int(BenchRunFlag.RES_HOLD), "end_evidence_age_ms": 3000,
              "target": 1000, "cut_position": 1000}
    two = [hold_pair(0, 1000), hold_pair(2_000_000, 1000)]
    assert bs.hold_verdict(two, result, deadband=12, run_id=9, source=2)["inside_s"] == 0.0
    mixed = [hold_pair(10_000 * k, 1000 if k % 2 == 0 else 1100) for k in range(100)]
    v = bs.hold_verdict(mixed, result, deadband=12, run_id=9, source=2)
    assert v["inside_s"] == 0.0 and v["inside_share"] == 0.5
    missing = [hold_pair(10_000 * k, 1000) for k in range(10)]
    for _t, _m, b in missing:
        del b.count_state  # a row without the field never counts as inside (Codex A1c-2 r2 P3)
    assert not any(bs._inside(b, 12, source=2) for _t, _m, b in missing)
    lost = [hold_pair(10_000 * k, 1000, count_state=int(BenchCountState.LOST)) for k in range(100)]
    assert bs.hold_verdict(lost, result, deadband=12, run_id=9, source=2)["inside_s"] == 0.0  # VALID only


def test_an_mt_inside_pair_is_one_read_at_its_tick():
    b = SimpleNamespace(target=1000, cumulative=1005, raw_angle=0, count_state=int(BenchCountState.VALID))
    assert bs._inside(b, 12, source=2)
    b.raw_angle = 1  # the pair's read failed: its count is an older sample's
    assert not bs._inside(b, 12, source=2)
    assert bs._inside(b, 12, source=1)  # KTH: raw angle 1 is a real angle


def test_the_hold_carries_its_source_and_cpr():
    h = bl.HoldAsk(3, 200, source=2, cpr=8192)
    assert (h.source, h.cpr) == (2, 8192)
    assert (bl.HoldAsk(2, 0).source, bl.HoldAsk(2, 0).cpr) == (1, 4096)  # the KTH default unchanged
    assert not math.isnan(units_of({"motor_counts_per_rev": 4096, "gear_ratio": 80.0},
                                   {"counts_per_wheel_rev": 1000})[1].counts_per_wheel_rev)


def test_a_cpr_that_is_no_number_leaves_no_nan_in_the_record():
    import json

    left, _right = units_of({"motor_counts_per_rev": "4096", "gear_ratio": 80.0}, {"motor_counts_per_rev": 4096,
                                                                           "gear_ratio": 80.0})
    assert left.motor_cpr is None and left.counts_per_wheel_rev is None  # never NaN (Codex A1c-2 r1 P2-5)
    json.dumps(left.record(), allow_nan=False)


@pytest.mark.parametrize("entry", [
    {"motor_counts_per_rev": 4096, "gear_ratio": float("nan")},
    {"motor_counts_per_rev": 4096, "gear_ratio": float("inf")},
    {"motor_counts_per_rev": 4096, "gear_ratio": 1e308},
    {"counts_per_wheel_rev": float("inf")},
])
def test_no_infinity_or_nan_in_the_units_record(entry):
    import json

    left, _right = units_of(entry, {"motor_counts_per_rev": 4096, "gear_ratio": 80.0})
    json.dumps(left.record(), allow_nan=False)  # Codex A1c-2 r2 P2-2
    assert left.motor_cpr is None


def test_a_board_only_recovery_reads_the_results_own_axis_and_source():
    """A pair whose current SET is M/KTH while its result is R/MT: the recovery reads the result's fields (plan 4.5
    decision 11, Codex plan round 8 P2-4) -- in counts only."""
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
    import bringup

    st = SimpleNamespace(stop_reason=5, run_flags=int(BenchRunFlag.RES_DISTANCE) | int(BenchRunFlag.RES_UNCLEAN),
                         result_run_id=11, result_axis=1, result_source=2, axis=2, source=1, blocked=0,
                         distance_confirm=0, distance_ref=100, cut_position=900, end_evidence_age_ms=4,
                         result_bypass=0, target=0, deadband_entries=0)
    out = bringup.result_fields(st)
    assert out["axis"] == "R" and out["source"] == 2 and out["kind"] == "distance"
    assert out["distance"]["result"] == "failed_continuity" and out["measure"]["status"] == "withheld:continuity"
    assert "turns" not in json_keys(out)


def json_keys(d) -> set:
    keys = set()
    for k, v in d.items():
        keys.add(k)
        if isinstance(v, dict):
            keys |= json_keys(v)
    return keys
