"""The commissioning tool's B2a-1 changes on the bench image (bench console plan 4.4 B2): verdict ① on every bench7
run (the bench's expected status bits left out), the first frame only after the last readiness zero was accepted,
the shared first-answer rule, the run's record rows (pairs with their receipt and connection, the host's
acceptances), the late board record of a short distance run kept as a label, and the final row's own fate
(``record_complete``) apart from the verdict. MCU model with --bench 1: a synthetic plant."""

import io
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import bringup  # noqa: E402
from test_bringup_commission_tool import args, model  # noqa: E402,F401
from test_bringup_tool import mcu_model  # noqa: E402,F401

from forklift_core.mcu_link.messages import RejectReason  # noqa: E402
from forklift_core.plant import bench_session  # noqa: E402

PRODUCT = "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1 bench7"
HOLD = dict(hold_kp=None, hold_ki=None, hold_kd=None, hold_deadband=None, hold_min_duty=None, count_sign=1)


def record(b):
    return [json.loads(x) for x in b.out.getvalue().splitlines()]


def bench(model, *extra):  # noqa: F811
    b = model("--bench", "1", "--kth-start-raw", "1000,1000", *extra)
    b.host.usb_product = PRODUCT
    return b


def dist_args(**kw):
    base = dict(axis="M", duty=0.2, seconds=1.0, bench_bypass="steer_range", bench_duty_max=0.25, bench_time_max=5.0,
                observed="none")
    return args(**{**base, **kw})


def test_a_measuring_run_carries_verdict1_and_its_record_rows(model):  # noqa: F811
    b = bench(model)
    res = bringup.execute(dist_args(seconds=0.8, kth_measure=True), b)
    assert res["result"] == "ok", res
    v = res["verdict1"]
    assert v["ok"] and v["labels"] == [], v
    assert res["record_complete"] is True
    rows = record(b)
    pairs = [r for r in rows if r["t"] == "bench_pair"]
    assert pairs and all({"rx_ns", "conn", "conn_gen", "last_rx_seq", "distance_ref"} <= r.keys() for r in pairs)
    acc = [r for r in rows if r["t"] == "state_accepted"]
    assert acc and all(r["conn"] == b.conn for r in acc)
    assert rows[-1]["t"] == "result" and "record_complete" not in rows[-1]


def test_a_distance_run_waits_for_its_last_readiness_zero_then_writes_the_first_non_zero(model):  # noqa: F811
    b = bench(model)
    res = bringup.execute(dist_args(stop_kth_turns=0.02), b)
    assert res["verdict1"]["ok"], res["verdict1"]
    assert res["start"] == "accepted"
    (last,) = [r for r in record(b) if r["t"] == "prep_zero_last"]
    assert last["accepted"] is True
    zero = next(w for w in b.writes if w.kind == "COMMISSION" and (w.seq & 0xFFFF) == last["seq"])
    first = next(w for w in b.writes if w.kind == "COMMISSION" and w.duty_e4)
    # the last zero was accepted (a STATE naming it, received after its last byte) before the first non-zero went out
    taken = next(s for s in b.samples if s.host_ns >= zero.done_ns and (s.state.last_rx_seq & 0xFFFF) == last["seq"])
    assert taken.host_ns <= first.start_ns
    assert not [w for w in b.writes if w.kind == "COMMISSION" and not w.duty_e4 and zero.order < w.order < first.order]


def test_a_one_count_run_passes_verdict1_with_or_without_a_driving_sample(model):  # noqa: F811
    b = bench(model)
    res = bringup.execute(dist_args(seconds=1.0, stop_kth_turns=1 / 4096), b)
    assert res["distance"]["result"] == "reached", res
    assert res["result"] in ("ok", "board_record_only"), res
    assert res["verdict1"]["ok"], res["verdict1"]


def test_a_late_board_record_turns_an_unconfirmed_start_into_a_label(model, monkeypatch):  # noqa: F811
    """The first answer judged unconfirmed although the board took it: the one-count run reached its distance and
    DISARMED in this arming, so the doubt is a label and verdict ① passes (plan 4.4, consistency round 1 P2-1)."""
    real = bench_session.first_answer

    def hide(*a, **kw):
        v = real(*a, **kw)
        return None if v is None else "unconfirmed"

    monkeypatch.setattr(bench_session, "first_answer", hide)
    b = bench(model)
    res = bringup.execute(dist_args(seconds=1.0, stop_kth_turns=1 / 4096), b)
    br = res["bench_result"]
    if br.get("stop_reason") == "distance_reached":
        assert res["aborted"] is None, res
        assert res["verdict1"]["ok"] and res["verdict1"]["labels"] == ["첫 응답 미확인 — 보드 기록으로 끝 확인"]
        assert res["start"] == "unconfirmed_board_record"
    else:  # the closing zero landed before the count moved: the doubt stays an abort
        assert res["aborted"] == "bench_start_unconfirmed" and not res["verdict1"]["ok"], res


def test_a_hold_carries_verdict1_at_the_top_too(model):  # noqa: F811
    b = bench(model)
    res = bringup.execute(args(axis="M", duty=0.15, bench_bypass="steer_range", observed="none", hold="here",
                               seconds=1.2, **HOLD), b)
    assert res["verdict1"]["ok"] and res["hold"]["verdict1"]["ok"], res
    assert [r for r in record(b) if r["t"] == "prep_zero_last" and r["accepted"]]


def test_a_refused_readiness_zero_sends_no_first_frame(model, monkeypatch):  # noqa: F811
    """The last readiness zero never shows accepted: no first frame goes out (the refusal after it could not be told
    from the zero's)."""

    class Refusing:
        def __init__(self, st):
            self._st = st

        def __getattr__(self, k):
            return RejectReason.STALE_ECHO if k == "reject_reason" else getattr(self._st, k)

    b = bench(model)
    orig = bringup.CommissionSession.confirm_last_prep_zero

    def refusing(self):
        host = self.b.host
        cls = type(host)
        prop = cls.state
        monkeypatch.setattr(cls, "state", property(lambda h: None if prop.fget(h) is None else Refusing(prop.fget(h))))
        try:
            return orig(self)
        finally:
            monkeypatch.setattr(cls, "state", prop)

    monkeypatch.setattr(bringup.CommissionSession, "confirm_last_prep_zero", refusing)
    res = bringup.execute(dist_args(stop_kth_turns=0.02), b)
    assert res["aborted"] == "prep_zero_unconfirmed", res
    assert not [w for w in b.writes if w.kind == "COMMISSION" and w.duty_e4]
    assert not res["verdict1"]["ok"]


def test_the_final_row_is_written_whole_or_cut_back(tmp_path, monkeypatch):
    path = tmp_path / "run.jsonl"
    with open(path, "a", encoding="utf-8") as out:
        b = bringup.Bench(None, out, commission=True)
        b.emit({"t": "a"})
        assert b.final_row({"t": "result", "n": 1}) is True
    assert [json.loads(x)["t"] for x in path.read_text().splitlines()] == ["a", "result"]

    path2 = tmp_path / "run2.jsonl"

    def fsync_fails(fd):
        raise OSError("disk")

    with open(path2, "a", encoding="utf-8") as out:
        b = bringup.Bench(None, out, commission=True)
        b.emit({"t": "a"})
        monkeypatch.setattr(bringup.os, "fsync", fsync_fails)
        assert b.final_row({"t": "result", "n": 1}) is False
    # the row was cut back: the file keeps the earlier rows only, and the close wrote nothing more
    assert [json.loads(x)["t"] for x in path2.read_text().splitlines()] == ["a"]


def test_an_in_memory_record_takes_the_final_row_as_any_row():
    b = bringup.Bench(None, io.StringIO(), commission=True)
    assert b.final_row({"t": "result"}) is True
    assert json.loads(b.out.getvalue().splitlines()[-1]) == {"t": "result"}
