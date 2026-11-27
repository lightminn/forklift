"""The bench console's back end (plan 2026-11-27 3, 3.1 B1) against the firmware core built as the bench image on the
fake board (``mcu_model --bench 1``, pty). A browser stand-in speaks the console protocol on the channels: the lease,
the tick echo, the requests through the ledger. It proves the run's order (BENCH_SET, BENCH_ARM, the axis's zero, the
duty, one zero, the board's own stop), the three cancellations (STOP, the lease, the connection), the refusals, the
record and the clear -- not currents or speeds of a real motor (the model's plant is synthetic). The model's command
validity is the board's 150 ms; the pty's timing is the host's scheduler, so no settling timing is judged here."""

import dataclasses
import json
import os
import signal
import subprocess
import sys
import time
import tty
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import FaultFlag, Mode, StateFlag
from forklift_core.plant.mcu_host import HostEvent

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import bench_console  # noqa: E402
from real_mcu_link import BenchTtyLink  # noqa: E402
from test_bringup_tool import mcu_model  # noqa: E402,F401 -- the module-scoped model build

BENCH5 = "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1 bench5"


class PtyDevice:
    """The console's device on a pty master: one opening (the product told once), then reads and gated writes."""

    def __init__(self, fd: int, product: str) -> None:
        self.fd, self.product = fd, product
        self.link = None
        self.opened = False
        self.extra_events: list = []

    def attach(self, console) -> None:
        self.link = BenchTtyLink(self.fd, gate=console.gate)

    def receive(self, host, now):
        events, self.extra_events = self.extra_events, []
        if not self.opened:
            self.opened = True
            events.append(HostEvent("connected", {"path": "pty", "product": self.product}))
            events += host.link_identity(self.product)
        data = self.link.read_available()
        if data:
            events += host.on_bytes(data, now)
        events += host.tick(now)
        return events

    def transmit(self, host, now):
        self.link.pump_writes(host, now)
        return []


class Model:
    def __init__(self, exe, *extra):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        argv = [str(exe), "--link", os.ttyname(self.slave), "--commission-build", "1", "--bench", "1"]
        self.proc = subprocess.Popen(argv + list(extra))

    def close(self):
        self.proc.send_signal(signal.SIGTERM)
        self.proc.wait(timeout=5)
        os.close(self.master)
        os.close(self.slave)


class Browser:
    """The console page's protocol on the channels (no server): lease, echo, requests with the connection's token."""

    def __init__(self, console, client="laptop", conn="c1"):
        self.c, self.client, self.conn = console, client, conn
        self.token = console.channels.issue_token()
        console.channels.connect(conn, client, True, self.token)
        self.seq = 0
        self.lease_id = None
        self.replies: dict = {}
        self.results: list = []
        self.clears: list = []
        self.echo = True
        self.peers: list = []  # other tabs on the same channels: the messages for them are handed on

    def send(self, kind, *, leased=True, seq=None, request_id=None, **fields):
        if seq is None:
            self.seq += 1
            seq = self.seq
        rid = request_id or f"{kind}-{seq}"
        msg = {"t": kind, "request_id": rid, "session_epoch": self.c.epoch, "request_token": self.token,
               "request_seq": seq, **fields}
        if leased:
            msg.setdefault("lease_id", self.lease_id)
            msg.setdefault("control_gen", self.c.arbiter.control_gen)
        self.c.channels.message(self.conn, self.client, msg)
        return rid

    def pump(self):
        for conn, msg in self.c.channels.take_out():
            for tab in [self] + self.peers:
                if conn in (None, tab.conn):
                    tab._take(msg)

    def _take(self, msg):
        if True:
            t = msg.get("t")
            if t in ("ack", "reject"):
                self.replies[msg.get("request_id")] = msg
                if t == "ack" and msg.get("of") == "lease_request":
                    self.lease_id = msg["lease_id"]
            elif t == "tick" and self.echo and self.lease_id is not None:
                self.c.channels.message(self.conn, self.client, {
                    "t": "echo", "session_epoch": self.c.epoch, "lease_id": self.lease_id,
                    "echo": {"state_seq": msg["state_seq"], "token": msg["token"]}})
            elif t == "bench_result":
                self.results.append(msg)
            elif t == "clear_result":
                self.clears.append(msg)


@pytest.fixture
def rig(mcu_model, tmp_path):  # noqa: F811
    made = []

    def make(*extra, product=BENCH5):
        m = Model(mcu_model, *extra)
        made.append(m)
        dev = PtyDevice(m.master, product)
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
        return c, b, run_until

    yield make
    for m in made:
        m.close()


def written_kinds(c):
    return [k for _seq, (k, _ns) in sorted(c.written.items(), key=lambda kv: kv[1][1])]


def record_rows(res):
    return [json.loads(x) for x in (Path(res["record"]) / "run.jsonl").read_text().splitlines()]


def assert_stopped_in_time(res, axis=0, bound_s=0.2):
    """Plan 3 HIL acceptance: after the cancellation no non-zero frame is written, and the axis's duty in STATE is
    zero within the command validity (150 ms) plus a margin after the last non-zero command written."""
    rows = record_rows(res)
    stop = next(r for r in rows if r.get("t") == "stop")
    nonzero = [r for r in rows if r.get("t") == "written" and r.get("kind") == "COMMISSION" and r.get("duty_e4")]
    assert not [r for r in nonzero if r["host_ns"] > stop["host_ns"]], "a non-zero COMMISSION after the cancel"
    assert nonzero, "the run never drove: nothing to bound"
    assert any(r.get("t") == "state" and r["duty_e4"][axis] != 0 for r in rows), "the board never reported the duty"
    last = nonzero[-1]["host_ns"]
    zero = next(r for r in rows if r.get("t") == "state" and r["host_ns"] > last and r["duty_e4"][axis] == 0)
    assert zero["host_ns"] - last <= bound_s * 1e9, (zero["host_ns"] - last) / 1e9


RUN = dict(axis="L", duty=0.1, seconds=0.6, power_switch_in_hand=True)


def test_a_wheel_run_drives_ends_with_one_zero_and_the_boards_own_stop(rig):
    c, b, run_until = rig()
    rid = b.send("bench_run", **RUN)
    assert run_until(lambda: b.results, 6.0), b.replies
    res = b.results[-1]
    assert b.replies[rid]["t"] == "ack"
    assert res["closed"] == "ended" and res["stop_reason"] == "end_zero" and res["cleanup_confirmed"], res
    assert res["verdict"]["run_cleanup"]["ok"], res
    kinds = written_kinds(c)
    assert "ARM" not in kinds and kinds.index("BENCH_SET") < kinds.index("BENCH_ARM")
    assert "DISARM" not in kinds  # the board stopped itself
    assert c.host.state.mode == Mode.DISARMED
    d = Path(res["record"])
    rows = [json.loads(x) for x in (d / "run.jsonl").read_text().splitlines()]
    assert rows[0]["t"] == "request" and any(r["t"] == "state" for r in rows) and rows[-1]["t"] == "result"
    assert json.loads((d / "result.json").read_text())["run_id"] == res["run_id"]


def test_a_steering_run_with_its_range_off(rig):
    c, b, run_until = rig()
    b.send("bench_run", axis="M", duty=0.2, seconds=1.0, bypass=["steer_range"], bench_duty_max=0.25,
           bench_time_max=5.0, power_switch_in_hand=True)
    assert run_until(lambda: b.results, 6.0), b.replies
    res = b.results[-1]
    assert res["closed"] == "ended" and res["verdict"]["run_cleanup"]["ok"], res
    assert res["bypass_mcu"] == ["steer_range"] and res["bench_set"]["time_ms"][2] == 5000


def test_a_shared_stop_mid_drive_ends_with_a_zero_and_no_disarm(rig):
    c, b, run_until = rig()
    b.send("bench_run", axis="L", duty=0.1, seconds=1.4, power_switch_in_hand=True)
    assert run_until(lambda: c.run is not None and c.run.nonzero_written and c.host.state.duty_e4[0] != 0, 4.0)
    n_before = len(c.written)
    b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S1", "source": "console"})
    assert run_until(lambda: b.results, 4.0)
    res = b.results[-1]
    assert res["aborted"] == "stop" and res["stop_reason"] == "end_zero" and res["cleanup_confirmed"], res
    assert_stopped_in_time(res)
    after = [k for _s, (k, _n) in sorted(list(c.written.items())[n_before:], key=lambda kv: kv[1][1])]
    assert "DISARM" not in after  # driving: the zero ends it, the board brakes and disarms itself
    assert c.arbiter.mode == "PAUSED"  # the bench released the STOPPED lock once the stop was confirmed


def test_a_lost_lease_cancels_the_run(rig):
    c, b, run_until = rig()
    b.send("bench_run", axis="L", duty=0.1, seconds=1.4, power_switch_in_hand=True)
    assert run_until(lambda: c.run is not None and c.run.nonzero_written and c.host.state.duty_e4[0] != 0, 4.0)
    b.echo = False  # the heartbeat stops
    assert run_until(lambda: b.results, 4.0)
    res = b.results[-1]
    assert res["aborted"] in ("lease_expired", "lease_lost") and res["cleanup_confirmed"], res
    assert_stopped_in_time(res)


def test_a_lost_connection_cancels_the_run(rig):
    c, b, run_until = rig()
    b.send("bench_run", axis="L", duty=0.1, seconds=1.4, power_switch_in_hand=True)
    assert run_until(lambda: c.run is not None and c.run.nonzero_written and c.host.state.duty_e4[0] != 0, 4.0)
    c.channels.disconnect(b.conn, b.client)
    assert run_until(lambda: c.last_result is not None, 4.0)
    res = c.last_result
    assert res["aborted"] in ("lease_lost", "disconnected") and res["cleanup_confirmed"], res
    assert_stopped_in_time(res)


def test_a_bench_stop_before_the_arm_cancels_with_nothing_armed(rig):
    c, b, run_until = rig()
    b.send("bench_run", **RUN)
    b.send("bench_stop")
    assert run_until(lambda: b.results, 4.0)
    res = b.results[-1]
    assert res["closed"] == "no_drive" and not res["verdict"]["run_cleanup"]["ok"]
    assert "BENCH_ARM" not in written_kinds(c)


@pytest.mark.parametrize("fields,why", [
    (dict(bypass=["kth_invalid"]), "not_yet_a2"),
    (dict(mode="hold"), "not_yet_a1b"),
    (dict(axis="L", source="mt"), "not_yet_a1c"),
    (dict(stop={"turns": 1}), "not_yet_a1b"),
    (dict(power_switch_in_hand="true"), "power_switch_in_hand: true or false"),
    (dict(power_switch_in_hand=False), "power switch"),
    (dict(absent=["L"]), "absent"),
    (dict(duty=0.5, seconds=0.6), "large run"),
    (dict(axis="M", duty=0.12, seconds=1.0), "--window"),
])
def test_refusals_before_anything_is_sent(rig, fields, why):
    c, b, run_until = rig()
    rid = b.send("bench_run", **{**RUN, **fields})
    assert run_until(lambda: rid in b.replies, 2.0)
    rep = b.replies[rid]
    assert rep["t"] == "reject" and why in rep.get("detail", ""), rep
    assert not {"BENCH_SET", "BENCH_ARM"} & set(written_kinds(c))


def test_a_second_run_while_one_runs_is_busy_and_a_resend_is_not_a_second_run(rig):
    c, b, run_until = rig()
    rid = b.send("bench_run", **RUN)
    assert run_until(lambda: rid in b.replies, 2.0)
    b.send("bench_run", seq=b.seq, request_id=rid, **RUN)  # the page's resend of the same request
    other = b.send("bench_run", **RUN)
    assert run_until(lambda: other in b.replies, 2.0)
    assert b.replies[other]["reason"] == "busy_running"
    assert run_until(lambda: b.results, 6.0)
    run_until(lambda: False, 0.3)
    assert len(b.results) == 1


def test_clear_faults_needs_a_latched_board_and_a_cause(rig):
    c, b, run_until = rig()
    rid = b.send("clear_faults", cause="")
    assert run_until(lambda: rid in b.replies, 2.0) and b.replies[rid]["detail"] == "cause: required"
    rid = b.send("clear_faults", cause="test")
    assert run_until(lambda: rid in b.replies, 2.0) and b.replies[rid]["reason"] == "not_latched"


def test_a_fault_is_cleared_through_a_disarm_then_the_clear(rig):
    c, b, run_until = rig("--fault-at-ms", "300")
    assert run_until(lambda: c.host.state is not None and c.host.state.mode == Mode.FAULT, 3.0)
    rid = b.send("clear_faults", cause="model fault")
    assert run_until(lambda: b.clears, 6.0), b.replies
    out = b.clears[-1]
    assert b.replies[rid]["t"] == "ack" and out["before"]["mode"] == "FAULT"
    assert out["result"] in ("cleared", "unconfirmed", "not_ready"), out  # the model's fault may persist
    kinds = written_kinds(c)
    assert kinds.index("DISARM") < kinds.index("CLEAR_FAULTS")


def test_the_stream_carries_the_generation_and_the_lease(rig):
    c, b, run_until = rig()
    run_until(lambda: False, 0.1)
    st = [m for m in c.messages if m.get("t") == "bench_state"][-1]
    assert st["control_gen"] == c.arbiter.control_gen and st["lease"]["client_id"] == "laptop"
    assert st["product"] == BENCH5 and st["ready"] and st["board"]["mode"] == "DISARMED"
    json.dumps(st, allow_nan=False)  # no NaN reaches the shared encoder


def test_another_bench_generation_is_never_ready(mcu_model, tmp_path):  # noqa: F811
    m = Model(mcu_model)
    try:
        dev = PtyDevice(m.master, "forklift-mcu commission OPEN v0-test rec=0 lim=lr1 bench4")
        c = bench_console.BenchConsole(dev, runs_root=tmp_path / "runs")
        dev.attach(c)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            c.cycle()
            time.sleep(0.002)
        assert c.not_ready() == "bench_refused:not_bench5_image"
    finally:
        m.close()


def test_a_cut_transmit_stops_the_board_by_its_command_expiry(rig):
    """HIL acceptance (3): the Jetson's writes stop mid-drive; the board's 150 ms expiry ends the run."""
    c, b, run_until = rig()
    b.send("bench_run", **{**RUN, "seconds": 1.4})
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 4.0)
    run_until(lambda: False, 0.1)
    c.dev.transmit = lambda host, now: []  # nothing reaches the board from here
    assert run_until(lambda: b.results, 5.0)
    res = b.results[-1]
    assert res["stop_reason"] == "command_timeout" and not res["verdict"]["run_cleanup"]["ok"], res
    assert c.host.state.mode in (Mode.DISARMED, Mode.FAULT)
    rows = record_rows(res)
    last = [r for r in rows if r.get("t") == "written" and r.get("kind") == "COMMISSION" and r.get("duty_e4")][-1]
    zero = next(r for r in rows if r.get("t") == "state" and r["host_ns"] > last["host_ns"] and r["duty_e4"][0] == 0)
    assert zero["host_ns"] - last["host_ns"] <= 0.25e9  # the board's own 150 ms expiry, with the pty's margin


def test_a_short_written_command_then_stop_is_never_taken(rig):
    """A non-zero COMMISSION written up to just before its delimiter, then STOP, then EAGAIN: abandoned, the poison
    follows, the board never names its seq, and the run closes with the board stopped (plan 3 partial writes)."""
    c, b, run_until = rig()
    b.send("bench_run", **{**RUN, "seconds": 1.4})
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 4.0)
    state = {"seq": None, "calls": 0}
    real_write = os.write

    def writer(fd, data):
        cur = c.dev.link._cur
        if state["seq"] is None and cur is not None and cur.kind == "COMMISSION" and cur.nonzero \
                and bytes(data) == bytes(cur.data):
            state["seq"] = int(cur.seq) & 0xFFFF
            b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S1", "source": "console"})
            return real_write(fd, data[:-1])  # everything but the delimiter
        if state["seq"] is not None and state["calls"] < 2 and cur is not None:
            state["calls"] += 1
            raise BlockingIOError
        return real_write(fd, data)

    c.dev.link._write = writer
    assert run_until(lambda: b.results, 5.0)
    assert state["seq"] is not None
    res = b.results[-1]
    assert res["aborted"] == "stop" and c.host.state.mode == Mode.DISARMED, res
    assert c.dev.link.abandoned >= 1
    rows = [json.loads(x) for x in (Path(res["record"]) / "run.jsonl").read_text().splitlines()]
    assert not any(r.get("t") == "state" and r["last_rx_seq"] == state["seq"] for r in rows)


def test_a_reopening_mid_drive_sends_nothing_until_a_pair_after_the_boundary(rig):
    """The board ends its arming at a connection boundary itself: the console sends no DISARM or zero after it. The
    model keeps its connection generation (no reopening of its own), so no pair after the boundary ever comes and the
    run closes unconfirmed -- with not one frame written after the boundary."""
    c, b, run_until = rig()
    b.send("bench_run", **{**RUN, "seconds": 1.4})
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 4.0)
    c.dev.extra_events.append(HostEvent("connected", {"path": "pty", "product": BENCH5}))
    assert run_until(lambda: c.run is None or c.run.link_reopened, 1.0)
    n = len(c.written)
    assert run_until(lambda: b.results, 8.0)
    assert len(c.written) == n
    assert not b.results[-1]["verdict"]["run_cleanup"]["ok"]


def test_a_steering_run_leaving_its_window_stops(rig):
    c, b, run_until = rig()
    v = c.host.state.steering_rad
    b.send("bench_run", axis="M", duty=0.15, seconds=1.4, window=[v - 0.02, v + 0.02], bench_duty_max=0.15,
           bench_time_max=2.0, power_switch_in_hand=True)
    assert run_until(lambda: b.results, 6.0), b.replies
    res = b.results[-1]
    assert (res["aborted"] or "").startswith("window"), res
    assert c.host.state.mode == Mode.DISARMED


def test_a_second_tab_without_the_lease_is_refused(rig):
    c, b, run_until = rig()
    other = Browser(c, client="tablet", conn="c2")
    b.peers.append(other)
    run_until(lambda: False, 0.05)
    rid = other.send("bench_run", leased=False, lease_id="none", control_gen=c.arbiter.control_gen, **RUN)
    assert run_until(lambda: rid in other.replies, 2.0)
    assert other.replies[rid]["t"] == "reject" and other.replies[rid]["reason"] in ("no_lease", "stale_lease")


def test_no_record_no_run(rig, tmp_path):
    c, b, run_until = rig()
    c.runs_root = tmp_path / "a_file"
    c.runs_root.write_text("not a directory")
    rid = b.send("bench_run", **RUN)
    assert run_until(lambda: rid in b.replies, 2.0)
    assert b.replies[rid]["reason"] == "record_unavailable" and "BENCH_SET" not in written_kinds(c)


def test_a_record_write_failure_mid_drive_cancels_and_stops(rig):
    c, b, run_until = rig()
    b.send("bench_run", **{**RUN, "seconds": 1.4})
    assert run_until(lambda: c.run is not None and c.run.phase == "drive", 4.0)

    def broken(_s):
        raise OSError("disk full")

    c.run.rec._f.write = broken
    assert run_until(lambda: b.results, 5.0)
    res = b.results[-1]
    assert (res["aborted"] or "").startswith("record_error") and c.host.state.mode == Mode.DISARMED, res


def test_a_stop_right_after_the_arm_went_out_lets_the_arming_zero_land_and_never_drives(rig):
    """ARM written -> STOP before the ARMED STATE -> the host's arming zero must still go out (else the arming stays
    open), then the stop zero / DISARM: no non-zero COMMISSION, the board ends DISARMED (round 3 P1-2)."""
    c, b, run_until = rig()
    b.send("bench_run", **RUN)
    assert run_until(lambda: c.run is not None and c.run.arm_written_ns is not None, 4.0)
    b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S1", "source": "console"})
    assert run_until(lambda: b.results, 5.0)
    res = b.results[-1]
    assert res["closed"] == "no_drive" and res["aborted"] == "stop", res
    assert c.host.state.mode == Mode.DISARMED
    rows = record_rows(res)
    commissions = [r for r in rows if r.get("t") == "written" and r.get("kind") == "COMMISSION"]
    assert any(r["axis"] == 0 and r["duty_e4"] == 0 for r in commissions), commissions  # the arming zero went out
    assert not any(r["duty_e4"] != 0 for r in commissions), commissions
    # the stop's own zero was taken while preparing, so the run's DISARM ended it, its answer confirmed
    assert any(r.get("t") == "written" and r.get("kind") == "DISARM" for r in rows)
    assert res["cleanup_confirmed"], res


def test_a_reopening_whose_new_generation_shows_up_late_closes_and_the_next_run_goes(rig):
    """The pair after the boundary arrives late (injected: the model keeps its generation, so the boundary is taken
    as coming from another one): nothing is sent before it, the run closes once the board shows it stopped, and the
    next request runs normally."""
    c, b, run_until = rig()
    b.send("bench_run", **{**RUN, "seconds": 1.4})
    assert run_until(lambda: c.run is not None and c.run.nonzero_written, 4.0)  # the board drives now
    c.dev.extra_events.append(HostEvent("connected", {"path": "pty", "product": BENCH5}))
    assert run_until(lambda: c.run is None or c.run.link_reopened, 1.0)
    n = len([1 for v in c.fate.values() if v == "written"])
    run_until(lambda: False, 0.2)  # no pair after the boundary yet: nothing is written
    assert len([1 for v in c.fate.values() if v == "written"]) == n
    boot, gen = c.last_seen
    c.boundary_from = (boot, (gen + 1) & 0xFF)  # the board's new generation, as a real reopening would show
    assert run_until(lambda: b.results, 6.0)
    res = b.results[-1]
    assert res["aborted"] == "link_reopened" and res["stop_reason"] == "command_timeout", res
    assert res["cleanup_confirmed"], res
    rid = b.send("bench_run", **RUN)
    assert run_until(lambda: len(b.results) == 2, 6.0), b.replies.get(rid)
    assert b.results[-1]["verdict"]["run_cleanup"]["ok"], b.results[-1]


def test_an_idle_stop_is_released_once_the_board_shows_it_stopped(rig):
    c, b, run_until = rig()
    b.c.channels.message(b.conn, b.client, {"t": "stop", "request_id": "S9", "source": "console"})
    gen = c.arbiter.control_gen
    rid = b.send("bench_run", control_gen=gen + 1, **RUN)  # a start sent with the STOP, before its release
    assert run_until(lambda: rid in b.replies, 1.0)
    assert b.replies[rid]["reason"] in ("stopped", "stale_control"), b.replies[rid]
    assert run_until(lambda: c.arbiter.mode == "PAUSED", 1.0)
    rid2 = b.send("bench_run", **RUN)  # asked again after the release: it runs
    assert run_until(lambda: b.results, 6.0), b.replies.get(rid2)
    assert b.results[-1]["verdict"]["run_cleanup"]["ok"], b.results[-1]


def test_the_result_carries_the_peak_current_from_the_runs_states(rig):
    c, b, run_until = rig()
    b.send("bench_run", **RUN)
    assert run_until(lambda: b.results, 6.0)
    res = b.results[-1]
    rows = record_rows(res)
    peak = [max(abs(r["current_ma"][k]) for r in rows if r.get("t") == "state") for k in range(4)]
    assert res["peak_current_ma"] == peak


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_ctrl_c_or_sigterm_ends_the_model_console_cleanly(mcu_model, tmp_path, sig):  # noqa: F811
    """Both signals only ask (Codex C round 4 P1): the loop ends at a cycle boundary, exit 0, the model child gone."""
    import select

    log = tmp_path / "err.txt"
    with open(log, "w") as err:
        proc = subprocess.Popen([sys.executable, str(REPO / "tools" / "bench_console.py"), "--model", str(mcu_model),
                                 "--port", "0", "--runs-root", str(tmp_path / "runs")],
                                stdout=subprocess.PIPE, stderr=err, text=True,
                                env={**os.environ, "PYTHONPATH": f"{REPO / 'src'}:{REPO}"})
        try:
            line, end, fd = b"", time.monotonic() + 10.0, proc.stdout.fileno()  # the whole start line, bounded
            while b"\n" not in line and time.monotonic() < end:
                if select.select([fd], [], [], max(0.0, end - time.monotonic()))[0]:
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        break
                    line += chunk
            line = line.decode(errors="replace")
            assert '"t": "started"' in line, (line, log.read_text())
            time.sleep(0.5)
            proc.send_signal(sig)
            assert proc.wait(timeout=10) == 0
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=5)
            proc.stdout.close()
    out = subprocess.run(["pgrep", "-f", f"{mcu_model} --link"], capture_output=True, text=True)
    assert out.stdout.strip() == ""


def test_a_model_ending_after_its_first_state_is_one_lost_device(mcu_model, tmp_path):  # noqa: F811
    from forklift_core.plant.mcu_host import HostEvent  # noqa: F401

    holder = {}
    dev = bench_console.ModelDevice(str(mcu_model), gate=lambda f, w: holder["c"].gate(f, w))
    c = bench_console.BenchConsole(dev, runs_root=tmp_path / "runs")
    holder["c"] = c
    events = []
    real = c._device_event
    c._device_event = lambda e, now: (events.append(e.kind), real(e, now))
    try:
        end = time.monotonic() + 5.0
        while c.not_ready() is not None and time.monotonic() < end:
            c.cycle()
            time.sleep(0.002)
        assert c.not_ready() is None
        dev.proc.terminate()
        dev.proc.wait(timeout=5)
        for _ in range(5):
            c.cycle()
        assert events.count("disconnected") == 1 and c.not_ready() is not None
    finally:
        dev.close()


def test_an_exit_while_driving_with_the_board_silent_still_records_the_run(rig):
    """The tool stopped mid-drive and the board went silent (a terminal Ctrl-C can end both): the run is closed at the
    exit deadline with a record, never left without one (Codex C round 5 P2-1)."""
    c, b, run_until = rig()
    b.send("bench_run", **{**RUN, "seconds": 1.4})
    assert run_until(lambda: c.run is not None and c.run.nonzero_written, 4.0)
    record = c.run.rec.dir
    c.dev.receive = lambda host, now: host.tick(now)  # nothing more comes from the board
    bench_console.shutdown(c, wait_s=0.5)
    assert c.run is None
    res = json.loads((record / "result.json").read_text())
    assert res["aborted"] == "tool_exit", res  # the first cause; the verdict says the cleanup went unconfirmed
    assert not res["verdict"]["run_cleanup"]["ok"] and "cleanup_unconfirmed" in res["verdict"]["run_cleanup"]["why"], res


def test_a_clear_with_no_state_after_its_clear_is_unconfirmed_not_refused(rig):
    """CLEAR sent, then not one STATE: the clear closes 'unconfirmed', its 'after' empty -- never 'not_cleared' from a
    sample made before the CLEAR (Codex C round 5 P2-2)."""
    c, b, run_until = rig("--fault-at-ms", "300")
    assert run_until(lambda: c.host.state is not None and c.host.state.mode == Mode.FAULT, 3.0)
    b.send("clear_faults", cause="model fault")
    assert run_until(lambda: c.clear is not None and c.clear.clear_seq is not None
                     and c.clear.clear_seq in c.written, 4.0)
    c.dev.receive = lambda host, now: host.tick(now)  # the board silent from the CLEAR on
    assert run_until(lambda: b.clears, 4.0)
    out = b.clears[-1]
    assert out["result"] == "unconfirmed" and out["after"] is None, out


def _after_the_clear_went_out(c, b, run_until):
    assert run_until(lambda: c.host.state is not None and c.host.state.mode == Mode.FAULT, 3.0)
    b.send("clear_faults", cause="model fault")
    assert run_until(lambda: c.clear is not None and c.clear.clear_seq is not None
                     and c.clear.clear_seq in c.written, 4.0)
    c.dev.receive = lambda host, now: host.tick(now)  # the board silent from here: only the injected STATE counts
    return c.host.state


def test_a_state_made_before_the_clear_but_received_after_it_does_not_decide(rig):
    """A STATE the board made before it read the CLEAR, arriving after the CLEAR went out, says nothing of the CLEAR:
    the clear is 'unconfirmed' with no 'after' (Codex C round 6 P2)."""
    c, b, run_until = rig("--fault-at-ms", "300")
    st = _after_the_clear_went_out(c, b, run_until)
    late = dataclasses.replace(st, mcu_time_us=st.mcu_time_us + 1)  # still FAULT, last_rx_seq the DISARM's
    c.host._state, c.host._state_rx_ns = late, int(c.clock())
    assert run_until(lambda: b.clears, 4.0)
    out = b.clears[-1]
    assert out["result"] == "unconfirmed" and out["after"] is None, out


def test_a_taken_clear_whose_echo_a_later_frame_replaced_is_still_seen(rig):
    """The CLEAR taken, then a later frame's seq in last_rx_seq: the board out of FAULT in the same boot is the CLEAR's
    reply, so the clear goes on to its ready check (Codex C round 6 P2)."""
    c, b, run_until = rig("--fault-at-ms", "300")
    st = _after_the_clear_went_out(c, b, run_until)
    after = dataclasses.replace(st, mcu_time_us=st.mcu_time_us + 1000, mode=Mode.DISARMED,
                                last_rx_seq=(c.clear.clear_seq + 1) & 0xFFFF,
                                flags=st.flags | StateFlag.DRIVERS_READY, fault_flags=FaultFlag(0))
    c.host._state, c.host._state_rx_ns = after, int(c.clock())
    assert run_until(lambda: b.clears, 4.0)
    out = b.clears[-1]
    assert out["result"] == "cleared" and out["after"]["mode"] == "DISARMED", out
