"""The S6 teleop bridge end to end (docs/design/2026-11-12-teleop-bridge.md 9): the CLI's Bridge (arbiter, encoder
speed, arm state machine, port) against the firmware core built as the MCU model over a pty, a console speaking
console/1 through the Channels with request tokens. What it proves: the protocol, arming, the stops and the
recording on a real core -- not a real motor's currents, directions or stopping (the model's plant is synthetic)."""

import io
import itertools
import json
import math
import os
import signal
import subprocess
import sys
import time
import tty
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import Drive, LinkDecoder, Mode

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import teleop_bridge as tb  # noqa: E402
from real_mcu_link import TtyLink  # noqa: E402
from test_bringup_tool import mcu_model  # noqa: E402,F401 -- the module-scoped model build

ALL = ("empty_forks", "forks_clear", "lift_low", "no_open_recovery")


class PtyDevice:
    """The bridge's device on a pty master: receive only, then transmit only (design 5)."""

    def __init__(self, fd):
        self.tty, self.tx_enabled = TtyLink(fd), True
        self.dec = LinkDecoder()
        self.sent = []

    def watch(self, host):
        """Every frame the host reports written, decoded (the host's log, chained)."""
        orig = host._log

        def log(what, now_ns, data, info):
            if what == "written":
                for r in self.dec.feed(bytes(data), now_ns):
                    self.sent.append((now_ns, r.message))
            orig(what, now_ns, data, info)
        host._log = log

    def receive(self, host, now_ns):
        events = []
        data = self.tty.read_available()
        if data:
            events += host.on_bytes(data, now_ns)
        return events + host.tick(now_ns)

    def transmit(self, host, now_ns):
        if self.tx_enabled:
            self.tty.pump_writes(host, now_ns)
        return []


class Model:
    def __init__(self, exe, *extra):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.proc = subprocess.Popen([str(exe), "--link", os.ttyname(self.slave), *extra])

    def close(self):
        self.proc.send_signal(signal.SIGTERM)
        self.proc.wait(timeout=5)
        os.close(self.master)
        os.close(self.slave)


class Console:
    """A console/1 client on the Channels: a token, a lease, echoes of the newest tick, teleop messages."""

    def __init__(self, bridge):
        self.b, self.ch = bridge, bridge.channels
        self.token = self.ch.issue_token()
        self.ch.connect("c1", "op", control=True, token=self.token)
        self.seq, self.rseq = itertools.count(1), itertools.count(1)
        self.tick, self.lease, self.replies = None, None, []
        self.events = []  # the vehicle's event messages (the console logs them)

    def pump(self):
        for _conn, msg in self.ch.take_out():
            if msg.get("t") == "tick":
                self.tick = msg
            elif msg.get("t") == "event":
                self.events.append(msg)
            elif "request_id" in msg:
                self.replies.append(msg)
                if msg.get("lease_id"):
                    self.lease = msg["lease_id"]

    def request(self, kind, **fields):
        msg = {"t": kind, "request_id": f"r{next(self.seq)}", "session_epoch": self.b.epoch,
               "request_token": self.token, "request_seq": next(self.rseq),
               "control_gen": self.b.arbiter.control_gen, **fields}
        if self.lease is not None:
            msg.setdefault("lease_id", self.lease)
        self.ch.message("c1", "op", msg)

    def stop(self):
        self.ch.message("c1", "op", {"t": "stop", "session_epoch": self.b.epoch, "request_id": f"s{next(self.seq)}"})

    def echo(self):
        if self.tick is not None and self.lease is not None:
            self.ch.message("c1", "op", {"t": "echo", "session_epoch": self.b.epoch, "lease_id": self.lease,
                                         "echo": {"state_seq": self.tick["state_seq"], "token": self.tick["token"]}})

    def teleop(self, speed_ratio=1.0, curvature_ratio=0.0, deadman=True):
        a = self.b.arbiter
        self.ch.message("c1", "op", {
            "t": "teleop", "session_epoch": self.b.epoch, "lease_id": self.lease, "control_gen": a.control_gen,
            "arm_gen": a.arm_gen, "seq": next(self.seq), "mode": "direct", "lift_dir": 0, "deadman": deadman,
            "speed_ratio": speed_ratio, "curvature_ratio": curvature_ratio,
            "echo": {"state_seq": self.tick["state_seq"], "token": self.tick["token"]}})


class Run:
    def __init__(self, exe, *extra, floor=lambda c, d, v: (1.0, "ok"), confirmed=ALL):
        self.model = Model(exe, *extra)
        self.dev = PtyDevice(self.model.master)
        self.b = tb.Bridge(self.dev, confirmed=set(confirmed), floor_permit=floor)
        self.dev.watch(self.b.host)
        self.c = Console(self.b)
        self.recs = []

    def cycles(self, n, *, teleop=None, until=None):
        for _ in range(n):
            self.c.pump()
            self.c.echo()
            if teleop is not None and self.b.arbiter.mode == "TELEOP" and self.b.arbiter.armed and self.c.tick:
                self.c.teleop(**teleop)
            rec = self.b.cycle(time.monotonic_ns())
            self.recs.append(rec)
            if until is not None and until(rec):
                return rec
            time.sleep(tb.CYCLE_S)
        return self.recs[-1] if self.recs else None

    def to_ready(self):
        self.cycles(25)
        self.c.request("lease_request")
        self.cycles(10)
        assert self.c.lease, self.c.replies
        self.cycles(40)  # a confirmed stand
        self.c.request("mode_request", to="TELEOP")
        self.cycles(5)
        assert self.b.arbiter.mode == "TELEOP", self.c.replies[-3:]
        self.c.request("arm")
        self.cycles(60, until=lambda r: r["phase"] == "ready")
        assert self.b.core.phase == "ready", self.recs[-1]

    def nonzero(self, since=0):
        return [m for _t, m in self.dev.sent[since:] if isinstance(m, Drive)
                and (m.left_wheel_rad_s != 0.0 or m.right_wheel_rad_s != 0.0)]

    def close(self):
        self.model.close()


@pytest.fixture
def run(mcu_model):  # noqa: F811
    made = []

    def make(*extra, **kw):
        r = Run(mcu_model, *extra, **kw)
        made.append(r)
        return r
    yield make
    for r in made:
        r.close()


def test_an_operator_teleop_drives_the_model_under_the_bound_and_a_stop_disarms_it(run):
    r = run()
    r.to_ready()
    assert r.b.host.state.mode == Mode.ARMED
    r.cycles(60, teleop={"speed_ratio": 1.0})
    assert r.nonzero(), r.recs[-1]
    moving = [x for x in r.recs if x.get("speed") and x["speed"]["signed"] > 0.05]
    assert moving and max(x["speed"]["bound"] for x in moving) <= r.b.vehicle.teleop_speed_limit_mps
    n = len(r.dev.sent)
    r.c.stop()
    r.cycles(30)
    assert r.b.arbiter.mode == "STOPPED" and r.nonzero(n) == []
    assert r.b.host.state.mode == Mode.DISARMED


def test_a_wall_escape_against_the_model(run):
    """Standing-direction-rule design 5 (end to end): a wall behind holds the forward request at a stand; the bridge
    offers the escape on the state message, the operator approves it, the model's wheels drive forward straight and
    slow, and the escape ends on its measured formula within the budget."""
    r = run(floor=lambda c, d, v: (1.0, "ok") if d > 0 else (0.0, "blocked_behind"))
    r.b.record = io.StringIO()
    r.to_ready()
    r.cycles(60, teleop={"speed_ratio": 1.0}, until=lambda rec: r.b.core.offer is not None)
    offer = r.b.core.offer
    assert offer is not None and offer["direction"] == 1, r.recs[-1]
    n = len(r.dev.sent)
    assert r.nonzero(n) == []
    st0 = r.b.host.state
    c0 = (st0.wheel_count_left, st0.wheel_count_right)
    r.c.request("escape_approve", arm_gen=r.b.arbiter.arm_gen, offer_id=offer["offer_id"])
    r.cycles(400, teleop={"speed_ratio": 1.0},
             until=lambda rec: rec.get("escape_end") is not None or r.b.arbiter.mode == "STOPPED")
    assert any(m.get("of") == "escape_approve" and m["t"] == "ack" for m in r.c.replies), r.c.replies[-3:]
    end = r.b.core.escape_log[-1]
    assert end["event"] == "end" and end["reason"] == "distance", r.b.core.escape_log
    drives = r.nonzero(n)
    assert drives and all(m.left_wheel_rad_s > 0 and m.right_wheel_rad_s > 0 for m in drives)
    assert max(max(m.left_wheel_rad_s, m.right_wheel_rad_s) for m in drives) * r.b.vehicle.wheel_radius_m[0] \
        <= 0.05 + 1e-9
    # the model's wheel PI creeps the wheels back after the zero (its negative integral, design 3): the record
    # follows the travel to the stand or to its 5 s limit
    r.cycles(320, teleop={"speed_ratio": 1.0, "deadman": False},
             until=lambda rec: any(e["event"] in ("settled", "settle_timeout")
                                   for e in rec.get("escape_events", ())))
    st1 = r.b.host.state
    m_per = 2 * math.pi * r.b.vehicle.wheel_radius_m[0] / r.b.vehicle.counts_per_wheel_rev[0]
    travel = max(abs(st1.wheel_count_left - c0[0]), abs(st1.wheel_count_right - c0[1])) * m_per
    assert 0.03 < travel <= 0.20, travel
    # Codex impl review P2-2: every step reaches the JSONL record and the console's event log
    r.c.pump()  # the last cycle's messages
    kinds = [m["kind"] for m in r.c.events]
    assert kinds[:3] == ["escape_offer", "escape_approved", "escape_start"] and "escape_end" in kinds, kinds
    assert {"escape_settled", "escape_settle_timeout"} & set(kinds), kinds
    tail = r.b.core.escape_log[-1]
    # from the start to the tail's end, every count both ways (the creep back adds): at least the net, within budget
    assert travel - 1e-4 <= max(tail["travel_m"]) <= 0.20
    rows = [json.loads(x) for x in r.b.record.getvalue().splitlines()]
    logged = [e["event"] for row in rows for e in row.get("escape_events", [])]
    assert logged == [e["event"] for e in r.b.core.escape_log]


def test_the_cli_floor_without_a_scan_moves_nothing(run):
    r = run(floor=tb.no_scan)
    r.to_ready()
    r.cycles(40, teleop={"speed_ratio": 1.0})
    assert r.nonzero() == [] and r.b.host.state.mode == Mode.ARMED


def test_a_tx_cut_lets_the_mcu_expiry_run_and_stops_the_arbiter(run):
    r = run()
    r.to_ready()
    r.cycles(20, teleop={"speed_ratio": 1.0})
    r.dev.tx_enabled = False
    rec = r.cycles(60, teleop={"speed_ratio": 1.0}, until=lambda x: x.get("stop"))
    assert str(rec.get("stop", "")).startswith("mcu_"), rec
    assert r.b.arbiter.mode == "STOPPED"
    r.dev.tx_enabled = True
    r.cycles(40)
    assert r.b.host.state.mode == Mode.DISARMED and r.b.core.phase in ("idle", "held")


def test_a_locked_board_is_never_armed(run):
    r = run("--outputs-locked", "1")
    r.cycles(25)
    r.c.request("lease_request")
    r.cycles(50)
    r.c.request("mode_request", to="TELEOP")
    r.cycles(5)
    r.c.request("arm")
    r.cycles(30)
    assert r.b.host.state.mode == Mode.DISARMED and r.b.arbiter.mode == "STOPPED"
    assert any(str(x.get("stop", "")).startswith("mcu_arm_refused") for x in r.recs)


def test_without_the_four_confirmations_teleop_stays_closed(run):
    r = run(confirmed=ALL[:3])
    r.cycles(25)
    r.c.request("lease_request")
    r.cycles(50)
    r.c.request("mode_request", to="TELEOP")
    r.c.request("arm")
    r.cycles(30)
    assert r.b.arbiter.mode != "TELEOP" and r.b.host.state.mode == Mode.DISARMED and r.nonzero() == []


def test_the_loop_never_catches_up():
    """Codex impl P2-6: a 35 ms first cycle -- the next starts 20 ms after the late one, not 5 ms."""
    t = {"now": 0}
    took = iter([35_000_000] + [1_000_000] * 10)

    class Fake:
        overruns = 0

        def cycle(self, now):
            t["now"] += next(took)

    def sleep(s):
        t["now"] += int(s * 1e9)

    starts = tb.run(Fake(), duration_s=0.12, clock=lambda: t["now"], sleep=sleep)
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    assert min(gaps) >= 20_000_000 and gaps[0] == 35_000_000


def test_the_cli_exits_armed_and_leaves_the_mcu_disarmed(mcu_model, tmp_path, monkeypatch, capsys):  # noqa: F811
    """Codex re-review P3: armed (READY) when the loop ends -- the closing DISARM, written after the ARM, is the
    frame the MCU took last, and it is DISARMED."""
    m = Model(mcu_model)
    seen = {}

    def run_armed(bridge, *, duration_s, **_kw):
        c = Console(bridge)
        recs = []

        def cycles(n, until=None):
            for _ in range(n):
                c.pump()
                c.echo()
                rec = bridge.cycle(time.monotonic_ns())
                recs.append(rec)
                if until is not None and until(rec):
                    return
                time.sleep(tb.CYCLE_S)

        cycles(25)
        c.request("lease_request")
        cycles(50)
        c.request("mode_request", to="TELEOP")
        cycles(5)
        c.request("arm")
        cycles(60, until=lambda r: r["phase"] == "ready")
        seen["ready"] = bridge.core.phase == "ready" and bridge.host.state.mode == Mode.ARMED
        seen["seq_at_ready"] = bridge.host._seq
        seen["bridge"] = bridge

    try:
        dev = PtyDevice(m.master)
        monkeypatch.setattr(tb, "ReconnectingLink", lambda path, baud=None: dev)
        monkeypatch.setattr(tb, "run", run_armed)

        class NoServer:  # the stand-in console reads the channels itself (a real server would drain them)
            port = 0

            def __init__(self, *a, **k):
                pass

            def start(self):
                pass

            def stop(self):
                pass

        monkeypatch.setattr(tb, "LiveServer", NoServer)
        code = tb.main(["--device", "pty", "--port", "0", "--record", str(tmp_path / "r.jsonl"),
                        *[f"--{n.replace('_', '-')}" for n in ALL]])
        assert code == 0 and seen["ready"]
        b = seen["bridge"]
        assert b.host.state.mode == Mode.DISARMED
        assert b.host.state.last_rx_seq >= seen["seq_at_ready"]  # a frame queued after READY: the closing DISARM
    finally:
        m.close()


def test_the_cli_runs_against_the_model_records_and_disarms_at_the_end(mcu_model, tmp_path, monkeypatch,  # noqa: F811
                                                                       capsys):
    m = Model(mcu_model)
    made = []
    orig = tb.Bridge.__init__

    def init(self, *a, **k):
        orig(self, *a, **k)
        made.append(self)

    try:
        dev = PtyDevice(m.master)
        monkeypatch.setattr(tb, "ReconnectingLink", lambda path, baud=None: dev)
        monkeypatch.setattr(tb.Bridge, "__init__", init)
        rec = tmp_path / "run.jsonl"
        code = tb.main(["--device", "pty", "--port", "0", "--record", str(rec), "--duration-s", "1.0",
                        *[f"--{n.replace('_', '-')}" for n in ALL]])
        out = capsys.readouterr().out
        assert code == 0 and '"command_cap_mps"' in out and "no_scan" in out
        assert rec.read_text().count("\n") >= 20
        b = made[0]
        assert b.host.state is not None and b.host.state.mode == Mode.DISARMED
        assert b.host.state.last_rx_seq == b.host._seq - 1  # the closing DISARM was the last frame the MCU took
    finally:
        m.close()
