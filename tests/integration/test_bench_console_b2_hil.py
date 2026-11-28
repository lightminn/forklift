"""The bench console's B2 runs (plan 4.4) against the firmware core as the bench image on the fake board
(``mcu_model --bench 1``, pty): hold, distance and measuring runs, the one-shot first frame, the board's own end, the
rest position, the verdicts and the record. The model's plant is synthetic: no real currents or speeds."""

import json
import os
import signal
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import bench_console  # noqa: E402,F401 -- the console under test (through the rig)
from test_bench_console_hil import BENCH5, Browser, Model, PtyDevice, record_rows, rig, written_kinds  # noqa: E402,F401
from test_bringup_tool import mcu_model  # noqa: E402,F401


M = dict(axis="M", power_switch_in_hand=True, bypass=["steer_range"], bench_duty_max=0.25, bench_time_max=5.0)
KTH = ("--kth-start-raw", "1000,1000")


def hold(**kw):
    return {**M, "mode": "hold", "seconds": 1.4, "count_sign": 1, "hold": {"target": "here"}, **kw}


def test_a_here_hold_settles_and_ends_by_its_zero(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold())
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    v1 = res["verdict"]["run_cleanup"]
    assert v1["ok"] and res["closed"] == "ended" and res["stop_reason"] == "end_zero", res
    assert res["kind"] == "hold" and res["start"] == "accepted" and res["cleanup"] == "confirmed"
    assert res["hold"]["verdict"]["result"] == "met", res["hold"]
    assert res["hold"]["settings"]["target"] == "here" and res["rest"] is not None
    assert res["record_complete"] is True
    kinds = written_kinds(c)
    assert "BENCH_HOLD" in kinds
    zeros = [w for w in c.write_info.values() if w.kind == "COMMISSION"]
    assert zeros  # the readiness and end zeros
    rows = record_rows(res)
    assert [r for r in rows if r["t"] == "hold_settings"] and [r for r in rows if r["t"] == "first_frame"]
    assert [r for r in rows if r["t"] == "bench_pair" and {"rx_ns", "conn", "conn_gen"} <= r.keys()]
    assert rows[-1]["t"] == "result" and rows[-1]["verdict"]["run_cleanup"]["ok"]
    assert c.host.commission_zero_axis == 0  # back to L after the run


def test_a_duty_run_right_after_a_hold_arms_with_the_l_zero(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold())
    assert run_until(lambda: b.results, 8.0)
    b.send("bench_run", axis="L", duty=0.1, seconds=0.6, power_switch_in_hand=True)
    assert run_until(lambda: len(b.results) == 2, 6.0), b.replies
    res = b.results[-1]
    assert res["verdict"]["run_cleanup"]["ok"], res


def test_a_distance_run_ends_at_its_distance_with_no_end_zero(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=1.0, stop={"turns": 0.02}, count_sign=1)
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    assert res["kind"] == "distance" and res["stop_reason"] == "distance_reached", res
    assert res["distance"]["result"] == "reached" and res["distance"]["stop_counts"] == 82
    assert res["board_cause"] == "distance_reached" and res["verdict"]["run_cleanup"]["ok"], res
    rows = record_rows(res)
    end = next(r for r in rows if r["t"] == "board_end")
    late = [r for r in rows if r["t"] == "written" and r.get("kind") in ("COMMISSION", "DISARM")
            and r["host_ns"] > end["host_ns"]]
    assert not late, late  # the board ended it: no zero or DISARM after its end was seen


def test_a_measuring_run_reports_its_displacement(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=0.8, measure=True, count_sign=1, source="kth")
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    assert res["kind"] == "measure" and res["verdict"]["run_cleanup"]["ok"], res
    assert res["measure"]["status"] == "confirmed" and res["measure"]["moved"] > 0, res["measure"]
    assert res["range_mark"] == "unchecked"


def test_a_b1_style_kth_request_is_still_a_plain_duty_run(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=0.8, source="kth", mode="duty")
    assert run_until(lambda: b.results, 6.0), b.replies
    res = b.results[-1]
    assert res["kind"] == "duty" and res["bench_set"]["source"] == 0 and "rest" not in res, res
    assert res["verdict"]["run_cleanup"]["ok"]


def test_a_stop_mid_hold_sends_no_hold_after_it(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    assert run_until(lambda: sum(1 for w in c.write_info.values() if w.kind == "BENCH_HOLD") >= 5, 2.0)
    b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S1", "source": "console"})
    assert run_until(lambda: b.results, 6.0)
    res = b.results[-1]
    assert res["aborted"] == "stop" and not res["verdict"]["run_cleanup"]["ok"], res
    rows = record_rows(res)
    stop = next(r for r in rows if r["t"] == "stop")
    assert not [r for r in rows if r["t"] == "written" and r.get("kind") == "BENCH_HOLD"
                and r["host_ns"] > stop["host_ns"]]


def test_a_hold_on_a_stuck_count_ends_by_the_board_and_fails(rig):  # noqa: F811
    """here at raw 16, then the count reads 0x0000 only: the board stops the run itself (kth_zero_suspect); no zero
    or DISARM after its end; verdict 2 failed and verdict 1 fails on the board's cause."""
    c, b, run_until = rig("--kth-start-raw", "16,16", "--kth-zero-at-ms", "M:1500")
    b.send("bench_run", **hold(seconds=5.0))
    assert run_until(lambda: b.results, 10.0), b.replies
    res = b.results[-1]
    assert res["stop_reason"] == "kth_zero_suspect" and res["board_cause"] == "kth_zero_suspect", res
    assert res["hold"]["verdict"]["result"] == "failed" and not res["verdict"]["run_cleanup"]["ok"], res
    rows = record_rows(res)
    end = next(r for r in rows if r["t"] == "board_end")
    assert not [r for r in rows if r["t"] == "written" and r.get("kind") in ("COMMISSION", "DISARM", "BENCH_HOLD")
                and r["host_ns"] > end["host_ns"]]


def test_a_request_without_its_sign_arms_nothing(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    rid = b.send("bench_run", **M, duty=0.2, seconds=1.0, stop={"turns": 0.5})
    assert run_until(lambda: rid in b.replies, 2.0)
    assert b.replies[rid]["t"] == "reject" and "count_sign" in json.dumps(b.replies[rid]), b.replies[rid]
    assert "BENCH_ARM" not in written_kinds(c)


def test_a_request_during_the_rest_position_is_busy(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=0.8, measure=True, count_sign=1, source="kth")
    assert run_until(lambda: c.run is not None and c.run.phase == "rest", 8.0)
    rid = b.send("bench_run", axis="L", duty=0.1, seconds=0.6, power_switch_in_hand=True)
    assert run_until(lambda: rid in b.replies, 1.0)
    assert b.replies[rid]["t"] == "reject" and "busy_running" in json.dumps(b.replies[rid])
    b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S2", "source": "console"})
    assert run_until(lambda: b.results, 3.0)
    res = b.results[-1]
    assert res.get("rest_note") == "stop" and res["verdict"]["run_cleanup"]["ok"], res


def test_the_stream_shows_the_runs_own_values_only(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=2.0))
    seen = []

    def look():
        m = c.state_message(c.clock())
        run = m.get("run")
        if run:
            seen.append((run["phase"], run["error_counts"], run["pair_valid"]))
        return bool(b.results)

    assert run_until(look, 8.0)
    before = [e for ph, e, _v in seen if ph in ("recover", "set", "arm", "prep", "first")]
    assert all(e is None for e in before), seen[:20]
    assert any(e is not None and v for ph, e, v in seen if ph == "drive"), seen


# ---- the gaps of plan 4.4 decision 7 on the console path (Codex B2a-2 impl round 1) ---------------------------------

@pytest.fixture
def mrig(mcu_model, tmp_path):  # noqa: F811
    """The rig with the model process at hand (SIGUSR1 pushes the axis)."""
    made = []

    def make(*extra):
        m = Model(mcu_model, *extra)
        made.append(m)
        dev = PtyDevice(m.master, m.product)
        c = bench_console.BenchConsole(dev, runs_root=tmp_path / "runs")
        dev.attach(c)
        b = Browser(c)

        def run_until(pred, timeout=5.0):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                c.cycle()
                b.pump()
                if pred():
                    return True
                time.sleep(bench_console.CYCLE_S)
            return False

        assert run_until(lambda: c.not_ready() is None, 5.0), c.not_ready()
        b.send("lease_request", leased=False, seq=None)
        assert run_until(lambda: b.lease_id is not None, 2.0)
        return c, b, run_until, m

    yield make
    for m in made:
        m.close()


def holds_written(c):
    return [w for w in c.write_info.values() if w.kind == "BENCH_HOLD"]


def test_a_rel_hold_moves_to_its_target(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=2.5, hold={"target": "rel:60"}))
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    assert res["hold"]["settings"]["target"] == "rel:60" and res["verdict"]["run_cleanup"]["ok"], res
    assert res["hold"]["verdict"]["result"] == "met", res["hold"]


def test_a_pushed_hold_comes_back(mrig):
    c, b, run_until, m = mrig("--kth-start-raw", "1000,1000", "--axis-push-steer-rad", "0.3")
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    assert run_until(lambda: len(holds_written(c)) >= 20, 2.0)
    m.proc.send_signal(signal.SIGUSR1)
    assert run_until(lambda: b.results, 8.0)
    res = b.results[-1]
    assert res["kind"] == "hold" and res["stop_reason"] == "end_zero" and res["verdict"]["run_cleanup"]["ok"], res
    errs = [r["target"] - r["cumulative"] for r in record_rows(res) if r["t"] == "bench_pair" and r["mode"] == "ARMED"
            and r["run_id"] == res["run_id"] and (r["drive"] & 3) == 2]
    out = [i for i, e in enumerate(errs) if abs(e) > 12]
    assert out, "the push never moved the axis out of the deadband"
    assert abs(errs[-1]) <= 12 and out[-1] < len(errs) - 1, errs[-20:]  # it came back


def test_a_refused_first_hold_is_written_once_and_closed_with_a_zero(mrig):
    """The axis pushed to raw 0 before the first HOLD: the board refuses it (sensor_invalid); exactly one HOLD is
    written, then the stop rule's zero, never another HOLD."""
    c, b, run_until, m = mrig("--kth-start-raw", "256,256", "--axis-push-steer-rad", "-0.3926990817")
    b.send("bench_run", **hold())
    assert run_until(lambda: c.run is not None and c.run.phase in ("arm", "prep"), 6.0)
    m.proc.send_signal(signal.SIGUSR1)
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    assert res["start"] == "refused:sensor_invalid" and not res["verdict"]["run_cleanup"]["ok"], res
    assert len(holds_written(c)) == 1


def test_a_reversed_sign_distance_stops_without_progress(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=1.0, stop={"turns": 0.5}, count_sign=-1)
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    assert res["stop_reason"] == "no_progress" and res["distance"]["result"] == "stopped:no_progress", res
    assert not res["verdict"]["run_cleanup"]["ok"]


def test_a_lease_lost_mid_hold_stops_it_with_no_hold_after(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=4.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    b.echo = False  # the page stops answering: the lease runs out
    assert run_until(lambda: b.results, 8.0)
    res = b.results[-1]
    assert res["aborted"] in ("lease_expired", "lease_lost"), res
    rows = record_rows(res)
    stop = next(r for r in rows if r["t"] == "stop")
    assert not [r for r in rows if r["t"] == "written" and r.get("kind") == "BENCH_HOLD"
                and r["host_ns"] > stop["host_ns"]]


def test_a_clear_during_the_rest_position_is_busy(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=0.8, measure=True, count_sign=1, source="kth")
    assert run_until(lambda: c.run is not None and c.run.phase == "rest", 8.0)
    rid = b.send("clear_faults", cause="test")
    assert run_until(lambda: rid in b.replies, 1.0)
    assert b.replies[rid]["t"] == "reject" and "busy_running" in json.dumps(b.replies[rid])
    assert run_until(lambda: b.results, 3.0)  # the run (and its record) closed before the test ends


def test_a_hold_whose_settings_cannot_be_written_sends_no_hold(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold())
    assert run_until(lambda: c.run is not None and c.run.phase == "prep", 6.0)
    real = c.run.rec._f.write

    def broken(row):
        if '"hold_settings"' in row:
            raise OSError("disk full")
        return real(row)

    c.run.rec._f.write = broken
    assert run_until(lambda: b.results, 8.0)
    res = b.results[-1]
    assert not holds_written(c) and not res["verdict"]["run_cleanup"]["ok"], res


def test_a_pair_row_that_cannot_be_written_cancels_the_hold(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    real = c.run.rec._f.write

    def broken(row):
        if '"bench_pair"' in row:
            raise OSError("disk full")
        return real(row)

    c.run.rec._f.write = broken
    assert run_until(lambda: b.results, 6.0)
    res = b.results[-1]
    assert (res["aborted"] or "").startswith("record_error") and not res["verdict"]["run_cleanup"]["ok"], res


def test_a_one_count_distance_passes_with_or_without_a_driving_sample(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=1.0, stop={"turns": 1 / 4096}, count_sign=1)
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    assert res["distance"]["result"] == "reached" and res["verdict"]["run_cleanup"]["ok"], res
    if res.get("drive_samples") == 0:
        assert "보드 기록만(구동 표본 없음)" in res["verdict"]["run_cleanup"]["labels"] and res["peak_current_ma"] is None


def test_a_copy_that_cannot_be_saved_leaves_the_verdict(rig, monkeypatch):  # noqa: F811
    c, b, run_until = rig(*KTH)

    def no_copy(self, obj):
        raise OSError("copy failed")

    monkeypatch.setattr(bench_console.RunRecord, "result", no_copy)
    b.send("bench_run", **M, duty=0.2, seconds=0.8, measure=True, count_sign=1, source="kth")
    assert run_until(lambda: b.results, 8.0)
    res = b.results[-1]
    assert res["verdict"]["run_cleanup"]["ok"] and res["record_complete"] and res["copy_error"] == "copy failed"
    rows = record_rows(res)
    assert rows[-1]["t"] == "result" and rows[-1]["verdict"]["run_cleanup"]["ok"]


def test_a_short_written_hold_then_stop_is_abandoned(rig):  # noqa: F811
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    state = {"seq": None, "calls": 0}
    real_write = os.write

    def writer(fd, data):
        cur = c.dev.link._cur
        if state["seq"] is None and cur is not None and cur.kind == "BENCH_HOLD" and bytes(data) == bytes(cur.data):
            state["seq"] = int(cur.seq) & 0xFFFF
            b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S1", "source": "console"})
            return real_write(fd, data[:-1])  # everything but the delimiter
        if state["seq"] is not None and state["calls"] < 2 and cur is not None:
            state["calls"] += 1
            raise BlockingIOError
        return real_write(fd, data)

    c.dev.link._write = writer
    assert run_until(lambda: b.results, 6.0)
    assert state["seq"] is not None and c.dev.link.abandoned >= 1
    res = b.results[-1]
    assert res["aborted"] == "stop", res
    rows = record_rows(res)
    assert not any(r.get("t") == "state" and r["last_rx_seq"] == state["seq"] for r in rows)


def test_the_first_answer_is_recomputed_from_the_record_alone(rig):  # noqa: F811
    from types import SimpleNamespace as NS

    from forklift_core.mcu_link.messages import RejectReason
    from forklift_core.plant import bench_session

    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold())
    assert run_until(lambda: b.results, 8.0)
    res = b.results[-1]
    rows = record_rows(res)
    (ff,) = [r for r in rows if r["t"] == "first_frame"]
    (start,) = [r for r in rows if r["t"] == "bench_start"]
    w = next(r for r in rows if r["t"] == "written" and r["kind"] == ff["kind"] and r["seq"] & 0xFFFF == ff["seq"])
    ev = []
    for r in rows:
        if r["t"] == "state":
            st = NS(mcu_time_us=r["mcu_time_us"], boot_id=r["boot_id"], arm_gen=r["arm_gen"],
                    last_rx_seq=r["last_rx_seq"], reject_reason=RejectReason[r["reject_reason"]])
            ev.append(("state", st, None, r["host_ns"], r["conn"]))
        elif r["t"] == "bench_pair":
            st = NS(mcu_time_us=r["mcu_time_us"], boot_id=r["boot_id"], arm_gen=r["arm_gen"],
                    last_rx_seq=r["last_rx_seq"], reject_reason=RejectReason.NONE)
            ev.append(("pair", st, NS(result_run_id=r["result_run_id"]), r["rx_ns"], r["conn"]))
    wr = NS(done_ns=w["done_ns"], conn=w["conn"], boot_id=w["boot_id"], arm_gen=w["arm_gen"],
            echo_mcu_time_us=w["echo_mcu_time_us"])
    v = bench_session.first_answer(ev, seq16=ff["seq"], run_id=res["run_id"], write=wr, lim_mcu_us=start["lim_mcu_us"],
                                   lim_host_ns=ff["host_deadline_ns"], now_ns=start["host_ns"],
                                   stop_zero_done_ns=start["stop_zero_done_ns"])
    assert v == res["start"], (v, res["start"])


def test_a_first_answer_at_the_renewal_end_stops_in_the_same_cycle(rig, monkeypatch):  # noqa: F811
    """The first HOLD's acceptance seen only at T - HANDOVER: recorded, and the stop entered in that same cycle -- no
    renewal, the end zero not held to the next cycle (Codex B2a-2 impl round 2 P1)."""
    c, b, run_until = rig(*KTH)
    real = bench_console.bs.first_answer

    def late(*a, **kw):
        v = real(*a, **kw)
        r = c.run
        if v is None or r is None or r.send_until_ns is None or kw["now_ns"] < r.send_until_ns:
            return None
        return v

    monkeypatch.setattr(bench_console.bs, "first_answer", late)
    b.send("bench_run", **hold(seconds=1.2))
    assert run_until(lambda: b.results, 8.0), b.replies
    res = b.results[-1]
    rows = record_rows(res)
    phases = [r for r in rows if r["t"] == "phase"]
    drive = next(r for r in phases if r["phase"] == "drive")
    stop = next(r for r in phases if r["phase"] == "stop")
    assert drive["host_ns"] == stop["host_ns"], (drive, stop)
    assert len(holds_written(c)) == 1  # the first HOLD only


def test_a_reopening_during_the_rest_position_keeps_the_boards_result(rig):  # noqa: F811
    from forklift_core.plant.mcu_host import HostEvent

    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=1.0, stop={"turns": 0.02}, count_sign=1)
    assert run_until(lambda: c.run is not None and c.run.phase == "rest", 8.0)
    c.dev.extra_events.append(HostEvent("connected", {"path": "pty", "product": BENCH5}))
    assert run_until(lambda: b.results, 3.0)
    res = b.results[-1]
    assert res["closed"] == "ended" and res["stop_reason"] == "distance_reached", res
    assert res["distance"]["result"] == "reached" and res["bench_result"]["confirmed"], res
    assert res["rest"] is None and "reopened" in res["rest_note"]
    assert res["final"] is not None and res["verdict"]["run_cleanup"]["ok"], res  # the cleanup seen before it holds


def test_a_stopping_state_closes_the_gate_on_our_stop_frames(rig):  # noqa: F811
    """Our own stop zero is let through the gate while the board is armed; once the latest STATE of the run's boot is
    STOPPING (the board stopping itself), the same frame -- its first byte or the rest of it -- is refused (plan 4.4,
    Codex plan round 2 P1-1; impl round 3 P2: the gate itself, not the 20 ms continuation limit)."""
    import dataclasses
    from types import SimpleNamespace

    from forklift_core.mcu_link.messages import Mode

    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    r = c.run
    r.cancel = "stop"
    c._enter_stop(r, "stop", c.clock())
    seq = 0xABCD
    c.cleanup_seqs.add(seq)
    f = SimpleNamespace(kind="COMMISSION", nonzero=False, seq=seq, expires_ns=None, echo=None,
                        generation=c.host.auth_epoch)
    assert c.frame_allowed(f)
    host_cls = type(c.host)
    prop = host_cls.state
    st = prop.fget(c.host)
    try:
        host_cls.state = property(lambda h: dataclasses.replace(st, mode=Mode.STOPPING,
                                                                mcu_time_us=st.mcu_time_us + 1))
        c._stop(r, c.clock())
        assert not c.frame_allowed(f)
    finally:
        host_cls.state = prop
    assert run_until(lambda: b.results, 6.0)  # the run (and its record) closed before the test ends


def test_a_stop_zero_torn_while_the_board_stops_itself_is_abandoned(rig):  # noqa: F811
    """STOP mid-hold, its zero written up to the delimiter and then held back: the HOLDs run out, the board stops by
    itself (STOPPING), and the rest of the zero is never written -- abandoned with the poison (plan 4.4, r2 P1-1)."""
    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    assert run_until(lambda: len(holds_written(c)) >= 5, 2.0)
    state = {"seq": None}
    real_write = os.write

    def writer(fd, data):
        cur = c.dev.link._cur
        if cur is not None and cur.kind == "COMMISSION" and not cur.nonzero and c.run is not None \
                and c.run.phase == "stop":
            seq = int(cur.seq) & 0xFFFF
            if state["seq"] is None and bytes(data) == bytes(cur.data):
                state["seq"] = seq
                return real_write(fd, data[:-1])  # all but the delimiter
            if state["seq"] == seq:
                raise BlockingIOError  # held back until the gate refuses it
        return real_write(fd, data)

    c.dev.link._write = writer
    b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S1", "source": "console"})
    assert run_until(lambda: b.results, 8.0)
    res = b.results[-1]
    assert state["seq"] is not None and c.dev.link.abandoned >= 1, res
    rows = record_rows(res)
    assert not any(r.get("t") == "state" and r["last_rx_seq"] == state["seq"] for r in rows)
    assert res["aborted"] == "stop" and not res["verdict"]["run_cleanup"]["ok"]


def test_a_fault_state_seen_before_a_loss_in_the_rest_position_fails_verdict1(rig):  # noqa: F811
    """A FAULT STATE of this arming reaches the console's receipt callback during the rest position, its pair never
    in, then the device is lost (the host's STATE cleared): the verdict reads that STATE, not the earlier DISARMED
    pair (impl rounds 4-5 P2). No pty read follows, so nothing newer can cover it."""
    import dataclasses

    from forklift_core.mcu_link.messages import FaultFlag, Mode
    from forklift_core.plant.mcu_host import HostEvent

    c, b, run_until = rig(*KTH)
    b.send("bench_run", **M, duty=0.2, seconds=0.8, measure=True, count_sign=1, source="kth")
    assert run_until(lambda: c.run is not None and c.run.phase == "rest", 8.0)
    r = c.run
    st = c.host.state
    now = c.clock()
    c._on_state(dataclasses.replace(st, mode=Mode.FAULT, fault_flags=FaultFlag.DRIVER_FAULT_M,
                                    mcu_time_us=st.mcu_time_us + 5_000), now)  # the host's accepted-STATE hook
    for e in c.host.disconnected(now):
        c._device_event(e, now)
    c._device_event(HostEvent("disconnected", {}), now)
    assert c.host.state is None or c.host.state.mode != Mode.FAULT  # the host kept nothing of it
    c._step(r, now + 1)  # the rest position ends on the loss, without another read
    res = b.results[-1] if b.results else c.last_result
    v1 = res["verdict"]["run_cleanup"]
    assert not v1["ok"] and res["final"]["mode"] == "FAULT", res


def test_a_window_sample_before_the_boards_end_in_one_read_is_kept(rig, monkeypatch):  # noqa: F811
    """A driving sample out of the window and the board's own end seen in the same cycle: the window stop is kept
    (verdict 1 fails), the board's end still decides what is sent (impl round 5 P2)."""
    import dataclasses

    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    r = c.run
    r.ask = dataclasses.replace(r.ask, window=(-0.01, 0.01), range_off=False)
    st = c.host.state
    out = dataclasses.replace(st, steering_rad=0.06, mcu_time_us=st.mcu_time_us + 1)
    monkeypatch.setattr(c, "_ended_by_board", lambda run: True)
    c._drive(r, c.clock(), [(out, c.clock())])
    assert r.phase == "stop" and r.window_stop == "window:0.06" and "window:0.06" in r.late_failures
    monkeypatch.undo()
    assert run_until(lambda: b.results, 6.0)
    assert not b.results[-1]["verdict"]["run_cleanup"]["ok"]


def test_the_coast_after_the_boards_end_in_the_same_read_is_no_window_stop(rig, monkeypatch):  # noqa: F811
    import dataclasses

    from forklift_core.mcu_link.messages import Mode

    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    r = c.run
    r.ask = dataclasses.replace(r.ask, window=(-0.05, 0.05), range_off=False)
    st = c.host.state
    inside = dataclasses.replace(st, steering_rad=0.04, mcu_time_us=st.mcu_time_us + 1)
    coast = dataclasses.replace(st, mode=Mode.STOPPING, steering_rad=0.06, mcu_time_us=st.mcu_time_us + 2)
    monkeypatch.setattr(c, "_ended_by_board", lambda run: True)
    c._drive(r, c.clock(), [(inside, c.clock()), (coast, c.clock())])
    assert r.phase == "stop" and r.window_stop is None and not r.late_failures
    monkeypatch.undo()
    assert run_until(lambda: b.results, 6.0)


def test_a_coast_state_before_its_end_pair_is_no_window_stop_either(rig):  # noqa: F811
    """The coast STATE (STOPPING, out of the window) read before the board's end pair arrives: still no window stop --
    the window reads driving samples only, whichever read the end pair comes in (impl round 7 P2)."""
    import dataclasses

    from forklift_core.mcu_link.messages import Mode

    c, b, run_until = rig(*KTH)
    b.send("bench_run", **hold(seconds=3.0))
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 6.0)
    r = c.run
    r.ask = dataclasses.replace(r.ask, window=(-0.05, 0.05), range_off=False)
    st = c.host.state
    coast = dataclasses.replace(st, mode=Mode.STOPPING, steering_rad=0.06, mcu_time_us=st.mcu_time_us + 2)
    c._drive(r, c.clock(), [(coast, c.clock())])
    assert r.window_stop is None and not r.late_failures
    out = dataclasses.replace(st, steering_rad=0.06, mcu_time_us=st.mcu_time_us + 3)
    c._drive(r, c.clock(), [(out, c.clock())])  # a driving sample out of the window still stops it
    assert r.window_stop == "window:0.06"
    assert run_until(lambda: b.results, 6.0)
