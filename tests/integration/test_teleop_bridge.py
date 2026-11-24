"""The S6 teleop bridge end to end (docs/design/2026-11-12-teleop-bridge.md 9): the CLI's Bridge (arbiter, encoder
speed, arm state machine, port) against the firmware core built as the MCU model over a pty, a console speaking
console/1 through the Channels with request tokens. What it proves: the protocol, arming, the stops and the
recording on a real core -- not a real motor's currents, directions or stopping (the model's plant is synthetic)."""

import io
import itertools
import hashlib
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
from forklift_core.live import real_teleop as rt  # noqa: E402
from real_mcu_link import TtyLink  # noqa: E402
from test_bringup_tool import mcu_model  # noqa: E402,F401 -- the module-scoped model build
import importlib.util  # noqa: E402

# 7b: the synthetic scan world of the scan permit's unit tests (boxes, the cast, the stand-in mount)
_bscan_spec = importlib.util.spec_from_file_location(
    "test_bridge_scan_helpers", REPO / "tests" / "unit" / "tools" / "test_bridge_scan.py")
bscan_t = importlib.util.module_from_spec(_bscan_spec)
_bscan_spec.loader.exec_module(bscan_t)

ALL = ("empty_forks", "forks_clear", "lift_low", "no_open_recovery")
# The end-to-end harness does not test loop_slow (D5 3.12; its unit tests do): its own loop is not real time, so a
# loaded CI node could stop an armed truck through it. Out of reach rather than a magic number.
HARNESS_LOOP_SLOW_OFF = 10**6


class PtyDevice:
    """The bridge's device on a pty master: receive only, then transmit only (design 5)."""

    def __init__(self, fd):
        self.tty, self.tx_enabled = TtyLink(fd), True
        self.last_read_ns = None
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

    def open_first(self, host, now_ns):
        """ReconnectingLink's early first opening (design 8.1): the pty is already open."""

    def receive(self, host, now_ns):
        events = []
        data = self.tty.read_available()
        if data:
            self.last_read_ns = time.monotonic_ns()  # as ReconnectingLink: the read time the record keeps
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
    def __init__(self, exe, *extra, floor=lambda c, d, v: (1.0, "ok"), confirmed=ALL, profile=None, scan_permit=None):
        self.model = Model(exe, *extra)
        self.dev = PtyDevice(self.model.master)
        # the operating rate guard (D5 3.11, rho 0.001): the model MCU runs on the host's monotonic clock, so its rate
        # verifies in ~20 s. A widened rho (0.05 until 2026-11-16) verified early with a range deviation of up to 5 %,
        # and that deviation is the delay bound's drift coefficient: ws1 showed age stales of 51-53 ms made of anchor
        # 20.8 + offset 6.7 + drift 23.8 ms at k 0.046 (job 19326) -- a harness artifact, not the operating bound
        self.b = tb.Bridge(self.dev, confirmed=set(confirmed), floor_permit=floor, profile=profile,
                           rate_rho=rt.RATE_RHO_MAX, scan_permit=scan_permit)
        # this harness is not real time: on a loaded CI node its own cycles can run late three times in a second and
        # stop the armed truck as loop_slow (D5 3.12) -- a test artefact; loop_slow has its own unit tests
        self.b.core.loop_slow_count = HARNESS_LOOP_SLOW_OFF
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
        rec0, ev0 = len(self.recs), len(self.b.arbiter.events)
        self.cycles(25)
        self.c.request("lease_request")
        self.cycles(10)
        assert self.c.lease, self.c.replies
        self.cycles(40)  # a confirmed stand
        self.c.request("mode_request", to="TELEOP")
        self.cycles(5)
        assert self.b.arbiter.mode == "TELEOP", self.c.replies[-3:]
        # the arrival procedure's order (D5 3.11): the console shows the clock verified, then the operator arms --
        # and, like the operator, only with no safety latch showing: a host stall over the cap while waiting marks the
        # link stale, the arbiter disarms and latches no_feedback until it sees the truck standing again, and an ARM
        # meanwhile is refused "invalid" (ws1 2026-11-16, the loop evidence of the arm attempt itself was clean)
        a = self.b.arbiter
        # the operating rate check takes ~20 s on the model (laptop 20.7 s); 2,500 cycles leave room under ws1 load
        self.cycles(2500, until=lambda r: r.get("rate") == "verified" and a.preempt_latch is None)
        assert self.b.core.rate.verdict == "verified", self.b.core.rate.progress_s()
        arms = 0
        while True:
            replies_before = len(self.c.replies)
            self.c.request("arm")
            arms += 1
            self.cycles(60, until=lambda r: r["phase"] == "ready" or a.preempt_latch is not None)
            refused = [x for x in self.c.replies[replies_before:] if x.get("t") == "reject" and x.get("of") == "arm"]
            if self.b.core.phase == "ready" or arms >= 3 or not (refused or a.preempt_latch is not None):
                break
            self.cycles(400, until=lambda r: a.preempt_latch is None)  # the latch released: the operator arms again
        if self.b.core.phase != "ready":
            self.cycles(60, until=lambda r: r["phase"] == "ready")
        ok = self.b.core.phase == "ready"
        # the evidence over the whole of to_ready, written for every call (passed or not) to the file the root
        # tests/conftest.py prints in the run's summary: cycle gaps (would loop_slow, off in this harness, have fired:
        # over 40 ms three times within 1 s), the arbiter's latches and disarms, ARM attempts, and on a failure every
        # change of the bridge record and the replies -- whole, not cut by pytest's assertion repr
        recs = self.recs[rec0:]
        gaps = [(x["now_ns"], x["cycle_gap_ms"]) for x in recs if x.get("cycle_gap_ms") is not None]
        slow = [t for t, g in gaps if g > rt.LOOP_SLOW_GAP_S * 1e3]
        would = [t for t in slow if sum(1 for u in slow if 0 <= t - u <= rt.LOOP_SLOW_WINDOW_S * 1e9)
                 >= rt.LOOP_SLOW_COUNT]
        events = [{k: v for k, v in e.items() if k != "t"} for e in a.events[ev0:]
                  if e.get("event") in ("disarm", "latch_released", "arm", "stand_unconfirmed")]
        # which side went stale (lead 2026-11-16): host_gap = the bridge stalled past the 150 ms cap (the operating
        # path); mcu_gap / age = the MCU model's STATEs stopped or came late (this test's own process) -- with the
        # other clock's gap at that moment and the bridge's cycle gap of that record
        # the stale's own "terms" (taken with its bound on receipt) stay; the cycle record's terms -- taken after the
        # STATE's own receipt may have added an anchor -- go under their own key (ws1 job 19326 lost one to a clash)
        stale = [{**d, "cycle_gap_ms": x.get("cycle_gap_ms"), "cycle_terms": x.get("delay_terms"), "rate": x.get("rate"),
                  "rate_range_at_end": list(self.b.core.rate.range) if self.b.core.rate.range else None}
                 for x in recs for d in x.get("stale_detail", ())]
        # the DelayBound's own size (lead 2026-11-16: the to_ready latches came from a loose bound, the operating path;
        # the harness's rearm only gathers evidence until that is fixed)
        delays = [x["delay_s"] for x in recs if x.get("delay_s") is not None]
        evidence = {"ready": ok, "arms": arms, "latches": sum(1 for e in events if e.get("event") == "latch_released"),
                    "age_stale": sum(1 for d in stale if d.get("why") == "age"),
                    "delay_max_ms": round(max(delays) * 1e3, 3) if delays else None, "stale": stale, "cycles": len(gaps), "over_40ms": len(slow),
                    "max_gap_ms": max((g for _t, g in gaps), default=None), "would_loop_slow": bool(would),
                    "latch_events": events}
        if not ok:
            changes, last = [], None
            for x in recs:
                key = (x.get("phase"), x.get("stop"), x.get("disarm"), x.get("arm_wait"), x.get("rate"),
                       tuple(x.get("link_events") or ()), x.get("rate_changed"))
                if key != last:
                    changes.append(key)
                    last = key
            evidence.update(latch=a.preempt_latch, mode=a.mode, changes=changes[-20:],
                            replies=self.c.replies[-6:], last=recs[-1] if recs else None)
        path = os.environ.get("FORKLIFT_LOOP_EVIDENCE")
        if path:
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(evidence, default=str) + "\n")
        assert ok, f"not ready (full evidence in the LOOP_EVIDENCE summary): latch={a.preempt_latch} " \
                   f"arms={arms} events={events[-6:]}"

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
    rows = [json.loads(x) for x in r.b.record.sink.getvalue().splitlines()]
    logged = [e["event"] for row in rows for e in row.get("escape_events", [])]
    assert logged == [e["event"] for e in r.b.core.escape_log]


def test_the_cli_floor_without_a_scan_moves_nothing(run):
    r = run(floor=tb.no_scan)
    r.to_ready()
    r.cycles(40, teleop={"speed_ratio": 1.0})
    assert r.nonzero() == [] and r.b.host.state.mode == Mode.ARMED


class CastFeed:
    """7b without the A2M12: a scan every 100 ms cast from the permit's own odometry pose against boxes (the
    synthetic world of tests/unit/tools/test_bridge_scan.py)."""

    def __init__(self, boxes):
        self.boxes, self.permit, self.last, self.k = boxes, None, None, 0

    def poll(self):
        now = time.monotonic_ns()
        if self.permit is None or not self.permit.hist or (self.last is not None and now - self.last < 100_000_000):
            return []
        self.last = now
        angles, ranges = bscan_t.cast(self.permit.pose, bscan_t.MOUNT, bscan_t.WALLS + self.boxes)
        self.k += 1
        return [bscan_t.scan(self.k, now, ranges, angles)]


def _scan_run(run, boxes):
    feed = CastFeed(boxes)
    permit = bscan_t.bs.ScanPermit(feed, mount=bscan_t.MOUNT, config_path=bscan_t.CONFIG, vehicle=bscan_t.VEH,
                                   v_max_mps=bscan_t.VEH.teleop_speed_limit_mps)
    feed.permit = permit
    return run(scan_permit=permit), permit


def test_the_scan_permit_opens_on_an_empty_floor_and_drives(run):
    r, permit = _scan_run(run, [])
    r.to_ready()
    r.cycles(60, teleop={"speed_ratio": 1.0})
    assert r.nonzero(), (r.recs[-1].get("scan"), permit.status())
    assert all("scan" in x for x in r.recs[-10:]) and permit.status()["scans_used"] > 10


def test_the_scan_permit_holds_the_truck_before_a_box(run):
    r, permit = _scan_run(run, [bscan_t.BOX_AHEAD])
    r.to_ready()
    r.cycles(40, teleop={"speed_ratio": 1.0})
    assert r.nonzero() == [] and permit.status()["scans_used"] > 10


def test_the_cli_refuses_half_a_scan_setup(tmp_path, capsys):
    flags = [f"--{n.replace('_', '-')}" for n in ALL]
    # PR 7 design 4: the mount's one source is config/real/lidar.yaml once measured; unmeasured, it is given here
    assert tb.main(["--device", "pty", "--scan-socket", str(tmp_path / "s.sock"), *flags]) == 2
    assert "lidar.yaml" in capsys.readouterr().err
    assert tb.main(["--device", "pty", "--lidar-mount", "0.9,0,1.05,0", *flags]) == 2
    assert "needs --scan-socket" in capsys.readouterr().err
    assert tb.main(["--device", "pty", "--scan-socket", str(tmp_path / "s.sock"), "--lidar-mount", "0.9,0,1.05",
                    *flags]) == 2
    assert "four finite" in capsys.readouterr().err
    assert tb.main(["--device", "pty", "--scan-socket", str(tmp_path / "s.sock"), "--lidar-mount", "0.9,0,1.05,0",
                    "--synthetic-permit", *flags]) == 2
    assert "not combined" in capsys.readouterr().err


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
    # the operating rate check first (D5 3.11): before it the bridge holds the ARM itself and the board is never asked
    r.cycles(2500, until=lambda x: x.get("rate") == "verified")
    assert r.b.core.rate.verdict == "verified", r.b.core.rate.progress_s()
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

        bridge.core.rate.rho_max = 0.05  # D5 3.11: verify within a second of anchors (the operating 0.001 needs tens)
        cycles(25)
        c.request("lease_request")
        cycles(50)
        c.request("mode_request", to="TELEOP")
        cycles(5)
        cycles(200, until=lambda r: r.get("rate") == "verified")  # the arrival procedure's order (12.3a)
        c.request("arm")
        cycles(60, until=lambda r: r["phase"] == "ready")
        seen["ready"] = bridge.core.phase == "ready" and bridge.host.state.mode == Mode.ARMED
        seen["seq_at_ready"] = bridge.host._seq
        seen["bridge"] = bridge

    try:
        dev = PtyDevice(m.master)
        monkeypatch.setattr(tb, "ReconnectingLink", lambda path, baud=None, **kw: dev)
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
        monkeypatch.setattr(tb, "ReconnectingLink", lambda path, baud=None, **kw: dev)
        monkeypatch.setattr(tb.Bridge, "__init__", init)
        rec = tmp_path / "run.jsonl"
        code = tb.main(["--device", "pty", "--port", "0", "--record", str(rec), "--duration-s", "1.0",
                        *[f"--{n.replace('_', '-')}" for n in ALL]])
        out = capsys.readouterr().out
        assert code == 0 and '"command_cap_mps"' in out and "no_scan" in out
        assert rec.read_text().count("\n") >= 20
        rows = [json.loads(x) for x in rec.read_text().splitlines()]
        # record design 3: a header first, a footer last with the counts, STATE rows with their read time, and the
        # STATEs taken while closing (after the last cycle row) are in it
        assert rows[0]["t"] == "header" and rows[0]["schema"] == tb.RECORD_SCHEMA
        # D4 (lead 2026-11-15): the record names the configuration it was made with, by content hash
        cfg = rows[0]["config"]
        vehicle = tb.load_real_vehicle(tb.VEHICLE_YAML)
        assert cfg["vehicle"]["calibration_id"] == vehicle.calibration_id
        assert cfg["vehicle"]["sha256"] == hashlib.sha256(tb.VEHICLE_YAML.read_bytes()).hexdigest()
        assert cfg["vehicle"]["path"] == "config/real/vehicle.yaml"
        chassis = tb.chassis_model_path(tb.VEHICLE_YAML)
        assert cfg["vehicle"]["chassis_model"]["sha256"] == hashlib.sha256(chassis.read_bytes()).hexdigest()
        assert cfg["bridge_params"]["sha256"] == hashlib.sha256(tb.BRIDGE_PARAMS_YAML.read_bytes()).hexdigest()
        assert cfg["profile"] is None
        assert rows[-1]["t"] == "footer" and rows[-1]["dropped"] == 0 and rows[-1]["write_errors"] == 0
        states = [i for i, r in enumerate(rows) if r.get("t") == "state"]
        cycles = [i for i, r in enumerate(rows) if "t" not in r]
        assert states and all("read_ns" in rows[i] and "mcu_time_us" in rows[i] for i in states)
        assert states[-1] > cycles[-1]
        b = made[0]
        assert b.host.state is not None and b.host.state.mode == Mode.DISARMED
        assert b.host.state.last_rx_seq == b.host._seq - 1  # the closing DISARM was the last frame the MCU took
    finally:
        m.close()


def test_the_record_keeps_every_state_the_host_took_and_the_markers(run):
    """Record design 3/4: one state row per STATE the host accepted (not the core's trimmed samples), each with the
    host time it was given and the read time, and an operator marker as a row of its own."""
    r = run()
    r.b.record = io.StringIO()
    taken = []
    orig = r.b.core.add_sample
    r.b.core.add_sample = lambda s: (taken.append(s), orig(s))[1]
    r.cycles(30)
    r.b.mark("m1 fwd #1 start", 123)
    r.cycles(5)
    rows = [json.loads(x) for x in r.b.record.sink.getvalue().splitlines()]
    states = [x for x in rows if x.get("t") == "state"]
    assert len(states) == len(taken) > 10
    assert [x["host_ns"] for x in states] == [s.host_ns for s in taken]
    assert all(x["read_ns"] >= x["host_ns"] for x in states)  # read after the cycle started
    marks = [x for x in rows if x.get("t") == "marker"]
    assert marks == [{"t": "marker", "host_ns": 123, "index": 0, "label": "m1 fwd #1 start"}]


def test_the_cli_refuses_an_existing_record_and_markers_without_a_record(tmp_path, capsys):
    rec = tmp_path / "old.jsonl"
    rec.write_text("x\n")
    flags = [f"--{n.replace('_', '-')}" for n in ALL]
    assert tb.main(["--device", "pty", "--record", str(rec), *flags]) == 2
    assert "exists" in capsys.readouterr().err and rec.read_text() == "x\n"
    assert tb.main(["--device", "pty", "--markers", *flags]) == 2
    assert "--markers needs --record" in capsys.readouterr().err


def test_a_failed_start_or_ctrl_c_still_closes_the_record_with_its_footer(mcu_model, tmp_path, monkeypatch):  # noqa: F811
    """Codex code review P2-3: from the record's creation on every exit path closes it -- a console that fails to
    start, and Ctrl-C in the loop."""
    flags = [f"--{n.replace('_', '-')}" for n in ALL]
    m = Model(mcu_model)
    try:
        dev = PtyDevice(m.master)
        monkeypatch.setattr(tb, "ReconnectingLink", lambda path, baud=None, **kw: dev)

        class BadServer:
            def __init__(self, *a, **k):
                pass

            def start(self):
                raise OSError("port in use")

        monkeypatch.setattr(tb, "LiveServer", BadServer)
        rec = tmp_path / "fail.jsonl"
        with pytest.raises(OSError):
            tb.main(["--device", "pty", "--record", str(rec), *flags])
        rows = [json.loads(x) for x in rec.read_text().splitlines()]
        assert rows[0]["t"] == "header" and rows[-1]["t"] == "footer"

        class NoServer:
            port = 0

            def __init__(self, *a, **k):
                pass

            def start(self):
                pass

            def stop(self):
                pass

        monkeypatch.setattr(tb, "LiveServer", NoServer)
        real_run = tb.run

        def run_then_interrupt(bridge, *, duration_s):
            real_run(bridge, duration_s=0.4)
            raise KeyboardInterrupt

        monkeypatch.setattr(tb, "run", run_then_interrupt)
        rec2 = tmp_path / "ctrlc.jsonl"
        assert tb.main(["--device", "pty", "--record", str(rec2), *flags]) == 0
        rows = [json.loads(x) for x in rec2.read_text().splitlines()]
        assert rows[-1]["t"] == "footer" and any(r.get("t") == "state" for r in rows)
    finally:
        m.close()


# --- D5: link freshness by MCU-time gap end to end (docs/design/2026-11-15-d5-link-gap-mcu-time.md 5.2, lead 10) ---

def _stall_then(r, gap_ms, n=10, teleop=None):
    """One cycle gap of ``gap_ms`` (the loop's own 20 ms sleep included); the model keeps sending meanwhile."""
    time.sleep(max(0.0, gap_ms / 1000.0 - tb.CYCLE_S))
    i = len(r.recs)
    r.cycles(n, teleop=teleop)
    gaps = [(b["now_ns"] - a["now_ns"]) / 1e6 for a, b in zip(r.recs[i - 1:], r.recs[i:])]
    return r.recs[i:], gaps[0]


def _events(recs):
    return [e for x in recs for e in x.get("link_events", [])]


@pytest.mark.parametrize("gap_ms", [30, 50, 70, 80])
def test_a_bridge_stall_with_states_queued_keeps_the_arm(run, gap_ms):
    r = run()
    r.to_ready()
    stand = {"speed_ratio": 0.0}
    r.cycles(10, teleop=stand)
    for _attempt in range(3):  # a scheduler overshoot past the band is retried, never judged
        recs, measured = _stall_then(r, gap_ms, teleop=stand)
        if measured <= gap_ms + 12:
            break
    assert measured <= gap_ms + 12, measured
    assert "stale" not in _events(recs) and "authorization_revoked" not in _events(recs), (measured, recs[0])
    assert r.b.core.phase == "ready" and r.b.host.ready


@pytest.mark.parametrize("gap_ms", [166, 328])
def test_a_bridge_stall_over_the_cap_is_a_disconnect(run, gap_ms):
    r = run()
    r.to_ready()
    r.cycles(10, teleop={"speed_ratio": 0.0})
    recs, measured = _stall_then(r, gap_ms, n=15)
    assert measured > 150
    details = [d for x in recs for d in x.get("stale_detail", [])]
    assert "stale" in _events(recs) and any(d.get("why") == "host_gap" for d in details), (measured, details)
    assert not r.b.host.authorized and r.b.core.phase != "ready"


def test_an_mcu_silence_of_60ms_is_a_disconnect(run):
    r = run()
    r.to_ready()
    r.cycles(10, teleop={"speed_ratio": 0.0})
    i = len(r.recs)
    r.model.proc.send_signal(signal.SIGSTOP)
    try:
        t_end = time.monotonic() + 0.060
        while time.monotonic() < t_end:
            r.cycles(1, teleop={"speed_ratio": 0.0})
    finally:
        r.model.proc.send_signal(signal.SIGCONT)
    r.cycles(15, teleop={"speed_ratio": 0.0})
    details = [d for x in r.recs[i:] for d in x.get("stale_detail", [])]
    assert "stale" in _events(r.recs[i:]) and details, r.recs[i:i + 5]
    assert details[0].get("why") in ("age", "mcu_gap") and not r.b.host.authorized


def test_the_config_identity_names_a_profile_by_hash_and_name(tmp_path):
    prof = REPO / "config" / "profiles" / "m1_m3" / "m1_fwd_v010.yaml"
    cfg = tb.config_identity(str(prof))
    assert cfg["profile"] == {"path": "config/profiles/m1_m3/m1_fwd_v010.yaml",
                              "sha256": hashlib.sha256(prof.read_bytes()).hexdigest(), "name": "m1_fwd_v010"}
    other = tmp_path / "p.yaml"
    other.write_bytes(prof.read_bytes() + b"# edited\n")
    assert tb.config_identity(str(other))["profile"]["sha256"] != cfg["profile"]["sha256"]


def test_an_unreadable_profile_stops_before_the_record_exists(tmp_path, capsys):
    """Codex #530 P2: the configuration is identified before the record file is made."""
    rec = tmp_path / "run.jsonl"
    code = tb.main(["--device", "pty", "--port", "0", "--record", str(rec), "--profile", str(tmp_path / "none.yaml"),
                    *[f"--{n.replace('_', '-')}" for n in ALL]])
    assert code == 2 and not rec.exists() and "cannot be read" in capsys.readouterr().err


def test_a_configuration_changed_while_starting_closes_the_record_and_refuses(mcu_model, tmp_path, monkeypatch,  # noqa: F811
                                                                           capsys):
    m = Model(mcu_model)
    try:
        dev = PtyDevice(m.master)
        monkeypatch.setattr(tb, "ReconnectingLink", lambda path, baud=None, **kw: dev)
        real = tb.config_identity

        def stale(*a, **kw):
            out = real(*a, **kw)
            # the header saw another chassis file than the bridge then loads (a file hash, not only the id)
            out["vehicle"] = {**out["vehicle"], "chassis_model": {**out["vehicle"]["chassis_model"], "sha256": "0" * 64}}
            return out

        monkeypatch.setattr(tb, "config_identity", stale)
        rec = tmp_path / "run.jsonl"
        code = tb.main(["--device", "pty", "--port", "0", "--record", str(rec), "--duration-s", "1.0",
                        *[f"--{n.replace('_', '-')}" for n in ALL]])
        assert code == 2 and "configuration changed" in capsys.readouterr().err
        rows = [json.loads(x) for x in rec.read_text().splitlines()]
        assert rows[0]["t"] == "header" and any(r.get("t") == "refused" for r in rows) and rows[-1]["t"] == "footer"
    finally:
        m.close()

