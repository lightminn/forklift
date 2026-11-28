"""The bench console's B2 runs (plan 4.4) against the firmware core as the bench image on the fake board
(``mcu_model --bench 1``, pty): hold, distance and measuring runs, the one-shot first frame, the board's own end, the
rest position, the verdicts and the record. The model's plant is synthetic: no real currents or speeds."""

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import bench_console  # noqa: E402,F401 -- the console under test (through the rig)
from test_bench_console_hil import record_rows, rig, written_kinds  # noqa: E402,F401
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
