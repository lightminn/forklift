"""The bench console's B2 requests at the web boundary (plan 4.4 decision 2): hold, distance and measuring runs, the
web schema before the CLI's checks, the explicit sign, the exclusions, the number bounds, and the checked-range mark."""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import bench_console as bc  # noqa: E402

BASE = {"axis": "M", "seconds": 1.0, "power_switch_in_hand": True, "bypass": ["steer_range"], "bench_duty_max": 0.25}


def ask(**kw):
    return bc.parse_run({**BASE, **kw})


def refused(**kw) -> str:
    with pytest.raises(bc.Refused) as exc:
        ask(**kw)
    return str(exc.value)


def test_a_b1_duty_request_keeps_its_meaning():
    a = ask(duty=0.2, source="kth", mode="duty")
    assert a.kind == "duty" and a.source_wire == "none" and a.bench.source == 0 and a.range_mark is None


def test_a_hold_request_with_its_defaults():
    a = ask(mode="hold", seconds=2.0, count_sign=1, hold={"target": "here"})
    assert a.kind == "hold" and a.source_wire == "kth" and a.bench.mode == 2
    h = a.bench.hold
    assert (h.kp, h.ki, h.kd, h.deadband, h.min_duty, h.cap) == (0.5, 0.0, 0.0, 12, 0.12, 0.15)
    assert a.range_mark == "not_applicable"


def test_a_hold_with_its_own_kd_and_cap():
    a = ask(mode="hold", seconds=2.0, duty=0.13, count_sign=-1, hold={"target": "rel:512", "kd": 0.02})
    assert a.bench.hold.kd == 0.02 and a.bench.hold.cap == 0.13 and a.bench.count_sign == -1


def test_a_distance_and_a_measuring_request():
    d = ask(duty=0.2, stop={"turns": 0.5}, count_sign=1)
    assert d.kind == "distance" and d.bench.source == 1 and d.bench.stop_counts == 2048
    assert d.range_mark == "unchecked"
    m = ask(duty=0.2, measure=True, count_sign=1, source="kth")
    assert m.kind == "measure" and m.bench.source == 1 and m.bench.stop_counts == 0


def test_a_distance_request_without_measure_is_never_a_b1_run():
    d = ask(duty=0.2, stop={"turns": 0.5}, count_sign=1, source="kth")
    assert d.kind == "distance" and d.bench.stop_counts == 2048


@pytest.mark.parametrize("kw, why", [
    (dict(duty=0.2, stop={"turns": 0.5}), "count_sign"),
    (dict(mode="hold", seconds=2.0, hold={"target": "here"}), "count_sign"),
    (dict(duty=0.2, measure=True, source="kth"), "count_sign"),
    (dict(duty=0.2, stop={"turns": 0.5}, count_sign=0), "count_sign"),
    (dict(duty=0.2, stop={"turns": 0.5}, count_sign=True), "count_sign"),
    (dict(duty=0.2, stop={"turns": True}, count_sign=1), "stop.turns"),
    (dict(duty=0.2, stop={"counts": 100}, count_sign=1), "not_yet_a1c"),
    (dict(duty=0.2, stop={"turns": 0.5, "x": 1}, count_sign=1), "stop:"),
    (dict(duty=0.2, mode="duty", hold={"target": "here"}), "hold: only with mode hold"),
    (dict(mode="hold", seconds=2.0, count_sign=1, hold={"target": "here"}, stop={"turns": 0.5}), "no stop"),
    (dict(duty=0.2, measure=True, count_sign=1, source="none"), "measure"),
    (dict(duty=0.2, measure=True, count_sign=1, stop={"turns": 0.5}), "measure"),
    (dict(duty=0.2, measure="yes", count_sign=1), "measure"),
    (dict(duty=0.2, stop={"turns": 0.5}, count_sign=1, source="none"), "source kth"),
    (dict(mode="hold", seconds=2.0, count_sign=1, hold={"target": "here", "deadband": 12.9}), "deadband"),
    (dict(mode="hold", seconds=2.0, count_sign=1, hold={"target": "here", "kp": 1e-50}), "float32"),
    (dict(mode="hold", seconds=2.0, count_sign=1, hold={"target": "here", "kp": True}), "hold.kp"),
    (dict(mode="hold", seconds=2.0, count_sign=1, hold={"target": 5}), "hold.target"),
    (dict(mode="hold", seconds=2.0, count_sign=1, hold={"target": "here", "gain": 1}), "unknown"),
    (dict(mode="hold", seconds=2.0, count_sign=1), "hold: an object"),
    (dict(duty=10**400, stop={"turns": 0.5}, count_sign=1), "duty"),
    (dict(duty="NaN", stop={"turns": 0.5}, count_sign=1), "duty"),
    (dict(duty=0.00004, stop={"turns": 0.5}, count_sign=1), "non-zero duty"),
    (dict(mode="hold", seconds=0.5, count_sign=1, hold={"target": "here"}), "1.2"),
])
def test_refusals(kw, why):
    assert why in refused(**kw)


def test_a_high_duty_distance_is_marked_never_refused():
    d = ask(duty=0.45, stop={"turns": 0.5}, count_sign=1, bench_duty_max=0.5, vcc_watched=True)
    assert d.advisories == ("high_duty_distance_unverified",)


def test_the_checked_range_mark(monkeypatch):
    monkeypatch.setattr(bc, "CHECKED_RANGE", {"M": {"S": 1, "duty_max": {1: 0.2}}})
    assert ask(duty=0.2, stop={"turns": 0.5}, count_sign=1, same_setup_as_bundle=True).range_mark == "inside"
    assert ask(duty=0.2, stop={"turns": 0.5}, count_sign=1).range_mark == "outside"  # setup not confirmed
    assert ask(duty=-0.2, stop={"turns": 0.5}, count_sign=1, same_setup_as_bundle=True).range_mark == "unchecked"
    assert ask(duty=0.2, stop={"turns": 0.5}, count_sign=-1, same_setup_as_bundle=True).range_mark == "outside"
    monkeypatch.setattr(bc, "CHECKED_RANGE", {"M": {"S": 1, "duty_max": {1: 0.2}, "excluded": True}})
    assert ask(duty=0.2, stop={"turns": 0.5}, count_sign=1, same_setup_as_bundle=True).range_mark == "outside"
