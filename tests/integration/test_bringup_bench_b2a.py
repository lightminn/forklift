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

PRODUCT = "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1 bench8"
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
    driving = [r for r in record(b) if r["t"] == "bench_pair" and r["mode"] == "ARMED" and (r["drive"] & 3) == 2
               and r["run_id"] == res["bench_set"]["run_id"]]
    if res["result"] == "board_record_only":
        assert not driving and "보드 기록만(구동 표본 없음)" in res["verdict1"]["labels"], res["verdict1"]
        assert res["metrics"]["measured_sign"] == "invalid" and res["verdict"]["sensor_sign_ok"] is None, res
    else:
        assert driving and res["verdict1"]["labels"] == []


def test_the_first_answer_is_recomputed_from_the_record_alone(model):  # noqa: F811
    """The JSONL alone gives back the first answer the run took (plan 4.4 B2: the host's accepted STATEs and pairs,
    the first frame's write and its two deadlines)."""
    from types import SimpleNamespace as NS

    b = bench(model)
    res = bringup.execute(dist_args(stop_kth_turns=0.02), b)
    rows = record(b)
    (ff,) = [r for r in rows if r["t"] == "first_frame"]
    (start,) = [r for r in rows if r["t"] == "bench_start"]
    w = next(r for r in rows if r["t"] == "written" and r["kind"] == ff["kind"] and r["seq"] & 0xFFFF == ff["seq"])
    ev = []
    for r in rows:
        if r["t"] == "state_accepted":
            st = NS(mcu_time_us=r["mcu_time_us"], boot_id=r["boot_id"], arm_gen=r["arm_gen"],
                    last_rx_seq=r["last_rx_seq"], reject_reason=RejectReason[r["reject_reason"]])
            ev.append(("state", st, None, r["rx_ns"], r["conn"]))
        elif r["t"] == "bench_pair":
            st = NS(mcu_time_us=r["mcu_time_us"], boot_id=r["boot_id"], arm_gen=r["arm_gen"],
                    last_rx_seq=r["last_rx_seq"], reject_reason=RejectReason.NONE)
            ev.append(("pair", st, NS(result_run_id=r["result_run_id"]), r["rx_ns"], r["conn"]))
    v = bench_session.first_answer(ev, seq16=ff["seq"], run_id=res["bench_set"]["run_id"], write=NS(**w),
                                   lim_mcu_us=start["lim_mcu_us"], lim_host_ns=ff["host_deadline_ns"],
                                   now_ns=ff["host_deadline_ns"] + 1)
    assert {"confirmed_by_result": "confirmed_by_result", "accepted": "accepted"}.get(v) == res["start"], (v, res)


def test_rows_lost_before_the_verdict_fail_it(model):  # noqa: F811
    class FlushFails(io.StringIO):
        def flush(self):
            raise OSError("disk")

    b = bench(model)
    b.out = FlushFails()
    res = bringup.execute(dist_args(seconds=0.8, kth_measure=True), b)
    assert not res["verdict1"]["ok"] and any(w.startswith("record_errors") for w in res["verdict1"]["why"]), res


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
        assert res["verdict1"]["ok"] and "첫 응답 미확인 — 보드 기록으로 끝 확인" in res["verdict1"]["labels"]
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
    """The last readiness zero never shows accepted (here: no STATE counts as naming it without a reject reason): no
    first frame goes out -- a refusal after it could not be told from the zero's."""
    from types import SimpleNamespace as NS

    b = bench(model)
    orig = bringup.CommissionSession.confirm_last_prep_zero

    def refusing(self):
        monkeypatch.setattr(bringup, "RejectReason", NS(NONE=object()))
        try:
            return orig(self)
        finally:
            monkeypatch.setattr(bringup, "RejectReason", RejectReason)

    monkeypatch.setattr(bringup.CommissionSession, "confirm_last_prep_zero", refusing)
    res = bringup.execute(dist_args(stop_kth_turns=0.02), b)
    assert res["aborted"] == "prep_zero_unconfirmed", res
    assert not [w for w in b.writes if w.kind == "COMMISSION" and w.duty_e4]
    assert not res["verdict1"]["ok"]
    (last,) = [r for r in record(b) if r["t"] == "prep_zero_last"]
    assert last["accepted"] is False and last["seq"] is not None


def test_the_final_row_is_written_whole_and_never_cut_back(tmp_path, monkeypatch):
    path = tmp_path / "run.jsonl"
    with open(path, "a", encoding="utf-8") as out:
        b = bringup.Bench(None, out, commission=True)
        b.emit({"t": "a"})
        assert b.final_row({"t": "result", "n": 1}) is True
    assert [json.loads(x)["t"] for x in path.read_text().splitlines()] == ["a", "result"]

    path2 = tmp_path / "run2.jsonl"

    def other_writer_then_fail(fd):
        with open(path2, "a", encoding="utf-8") as other:
            other.write(json.dumps({"t": "other"}) + "\n")
        raise OSError("disk")

    with open(path2, "a", encoding="utf-8") as out:
        b = bringup.Bench(None, out, commission=True)
        b.emit({"t": "a"})
        monkeypatch.setattr(bringup.os, "fsync", other_writer_then_fail)
        assert b.final_row({"t": "result", "n": 1}) is False
    # not on disk for sure, so not complete -- but nothing is cut: the other writer's row stays
    assert [json.loads(x)["t"] for x in path2.read_text().splitlines()] == ["a", "result", "other"]


def test_verdict1_is_made_again_where_execute_ends(model, monkeypatch):  # noqa: F811
    """Verdict 1 is made again at execute's end -- after its cleanup retry and every row lost since -- and the result
    follows it; the run's recorders stay on until then (Codex B2a-1 impl round 2 P2-2, P2-4)."""
    made, hooked = [], []
    real = bringup.bench_verdict1

    def counting(*a, **kw):
        v = real(*a, **kw)
        made.append(v)
        hooked.append(a[0].host.on_pair is not None and a[0].host.on_accepted is not None)
        return v

    monkeypatch.setattr(bringup, "bench_verdict1", counting)
    b = bench(model)
    res = bringup.execute(args(axis="M", duty=0.15, bench_bypass="steer_range", observed="none", hold="here",
                               seconds=1.4, **HOLD), b)
    assert len(made) == 2 and res["verdict1"] is made[-1] and res["hold"]["verdict1"] is made[-1]
    assert res["verdict1"]["ok"] == (res["result"] == "ok"), res
    # the recorders were still on when verdict 1 was made again (a cleanup retry's pairs are recorded), off after
    assert hooked == [True, True] and b.host.on_pair is None and b.host.on_accepted is None


def test_a_verdict1_failed_at_the_end_never_leaves_the_result_ok(model, monkeypatch):  # noqa: F811
    real = bringup.bench_verdict1
    calls = {"n": 0}

    def second_fails(*a, **kw):
        v = real(*a, **kw)
        calls["n"] += 1
        if calls["n"] == 2:
            v = {**v, "ok": False, "why": [*v["why"], "record_errors:1"]}
        return v

    monkeypatch.setattr(bringup, "bench_verdict1", second_fails)
    b = bench(model)
    res = bringup.execute(args(axis="M", duty=0.15, bench_bypass="steer_range", observed="none", hold="here",
                               seconds=1.4, **HOLD), b)
    assert not res["verdict1"]["ok"] and res["result"] == "aborted", res


def test_a_torn_final_row_never_swallows_the_next_runs_rows(tmp_path, monkeypatch):
    path = tmp_path / "run.jsonl"
    real_write = bringup.os.write

    def short(fd, data):
        return real_write(fd, data[:10]) if data.startswith(b"{") else real_write(fd, data)

    with open(path, "a", encoding="utf-8") as out:
        b = bringup.Bench(None, out, commission=True)
        b.emit({"t": "a"})
        monkeypatch.setattr(bringup.os, "write", short)
        assert b.final_row({"t": "result", "n": 1}) is False
    monkeypatch.setattr(bringup.os, "write", real_write)
    bringup._end_torn_line(str(path))  # a no-op here: the torn row was already closed with a line end
    with open(path, "a", encoding="utf-8") as out:
        out.write(json.dumps({"t": "header"}) + "\n")
    lines = path.read_text().splitlines()
    assert json.loads(lines[0]) == {"t": "a"} and json.loads(lines[-1]) == {"t": "header"}
    assert len(lines) == 3  # the torn row stands alone on its line


def test_a_record_ending_in_a_torn_line_gets_a_line_end_first(tmp_path):
    path = tmp_path / "old.jsonl"
    path.write_text('{"t": "a"}\n{"t": "res')
    bringup._end_torn_line(str(path))
    assert path.read_text().endswith('res\n')
    bringup._end_torn_line(str(path))
    assert path.read_text().endswith('res\n')  # once only


def test_an_in_memory_record_takes_the_final_row_as_any_row():
    b = bringup.Bench(None, io.StringIO(), commission=True)
    assert b.final_row({"t": "result"}) is True
    assert json.loads(b.out.getvalue().splitlines()[-1]) == {"t": "result"}
