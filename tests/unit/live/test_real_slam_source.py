"""PR 7c-1b-2 (design 6.1, 1-4): the executor's real SLAM source against a fake bridge server on the same socket
protocol -- the session start, the scan slots, the odometry rule, seeds, cancels and voids, the executor's
deadlines, late replies, forced requests and the transport faults."""

import math
import socket

import numpy as np
import pytest

from forklift_core.live import real_slam_source as rss
from forklift_core.localization import slam_link
from forklift_core.localization import slam_real_link as rl
from forklift_core.localization import slam_real_scan as srs

MS = 1_000_000
START_NS = 1_000 * MS
MAP = "m" * 64
_OPEN = []


@pytest.fixture(autouse=True)
def _close_sockets():
    yield
    while _OPEN:
        _OPEN.pop().close()


class Bridge:
    """The bridge end: reads the executor's frames, answers states and replies."""

    def __init__(self, sock):
        self.sock = sock
        sock.setblocking(False)
        self.buf = b""
        self.got = []

    def frames(self):
        try:
            while True:
                chunk = self.sock.recv(1 << 16)
                if not chunk:
                    break
                self.buf += chunk
        except BlockingIOError:
            pass
        out = []
        while len(self.buf) >= 4:
            n = int.from_bytes(self.buf[:4], "big")
            if len(self.buf) < 4 + n:
                break
            out.append(rl.decode(self.buf[4:4 + n]))
            self.buf = self.buf[4 + n:]
        self.got += out
        return out

    def state(self, ident, kind, reason=None):
        self.sock.sendall(rl.encode_state(rl.State(ident, kind, reason)))

    def reply(self, real_scan, status="processed", corr=(0.0, 0.0, 0.0)):
        s = real_scan.scan
        self.sock.sendall(slam_link.encode_reply(slam_link.Reply(s.scan_id, s.stamp_s, status, corr, 1, 0.0)))


class Rig:
    def __init__(self, mode="operating"):
        a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        _OPEN.extend((a, b))
        self.bridge = Bridge(b)
        self.now = START_NS + 2_000 * MS
        self.history = []
        self.odom = (0.0, 0.0, 0.0)
        self.boot_id = 1
        self.odom_gen = 0
        self.gen = (0, None)
        self.src = rss.RealSlamSource(
            rss.RealSlamLink(a, clock=lambda: self.now), mode=mode, map_id=MAP, start_ns=START_NS, start_pose=(1.0, 2.0, 0.3),
            odom_history=lambda: self.history, odom_base=lambda: self.odom, odom_gen=lambda: self.odom_gen,
            boot=lambda: self.boot_id, rear_offset_m=0.5, clock=lambda: self.now)
        self.hist_to(self.now)

    def s(self, ns):
        return (ns - START_NS) / 1e9

    def hist_to(self, ns):
        """STATE entries every 10 ms up to ns (core seconds), at the current odometry (rear axle)."""
        from forklift_core.localization.start_reloc import rear_from_base

        last = self.history[-1][1] if self.history else self.s(ns) - 0.5
        t = last + 0.01
        while t <= self.s(ns) + 1e-9:
            self.history.append((t - 0.005, t, rear_from_base(self.odom, 0.5), True, True, True, True))
            t += 0.01

    def step(self, ms=20):
        self.now += ms * MS
        self.hist_to(self.now)
        return self.src.replies(self.now)

    def scan(self, lag_ms=30, measured=True, width_ms=1):
        t = self.now - lag_ms * MS
        n = 8
        return srs.SlamScanIn(key=(*self.gen, t), gen=self.gen, stamp_ns=t, flo_ns=t - width_ms * MS, fhi_ns=t + width_ms * MS,
                              measured=measured, angles=np.linspace(-math.pi, math.pi, n, endpoint=False),
                              ranges=np.full(n, 2.0), self_hit=np.zeros(n, bool), range_min=0.1, range_max=10.0)

    def started(self, ident=1):
        """hello -> seeding -> ready_for_scans -> seven start scans -> ready."""
        self.step()
        hello = [f for f in self.bridge.frames() if isinstance(f, rl.Hello)]
        assert hello and hello[0].start_id == ident
        self.bridge.state(ident, "seeding")
        self.bridge.state(ident, "ready_for_scans")
        self.step()
        for _ in range(7):
            self.src.offer(self.scan())
            sent = [f for f in self.bridge.frames() if isinstance(f, rl.RealScan)]
            assert len(sent) == 1
            self.bridge.reply(sent[0])
            assert self.step() == []  # start-check replies never reach the loop
        self.bridge.state(ident, "ready")
        self.step()
        assert self.src.can_confirm()

    def one(self, **kw):
        """A scan through a localized instance: its loop item."""
        self.src.offer(self.scan(**kw))
        sent = [f for f in self.bridge.frames() if isinstance(f, rl.RealScan)]
        assert len(sent) == 1
        self.bridge.reply(sent[0])
        return self.step(1), sent[0]


def test_the_session_starts_with_hello_and_reaches_ready_without_a_seed():
    r = Rig()
    r.step()
    [hello] = r.bridge.frames()
    assert isinstance(hello, rl.Hello) and hello.map_id == MAP and hello.start_pose == (1.0, 2.0, 0.3)
    assert hello.deadline_ns == r.now - 20 * MS + int(rss.SEED_WAIT_S * 1e9) or hello.deadline_ns > r.now
    r.src.offer(r.scan())
    r.step()
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())  # nothing before ready_for_scans
    # the start check running: no confirmation, and no seed (the bridge takes none before the start's end state)
    assert not r.src.can_confirm() and not r.src.can_seed()


def test_a_localized_reply_carries_the_three_times_and_the_epoch():
    r = Rig()
    r.started()
    out, sent = r.one(lag_ms=40, width_ms=2)
    [(reply, meta, sent_s, recv_s)] = out
    stamp_ns = sent.stamp_ns
    assert reply.stamp_s == pytest.approx(r.s(stamp_ns)) and reply.status == "processed"
    assert reply.evidence_lo_s == pytest.approx(r.s(stamp_ns - 2 * MS))
    assert reply.evidence_hi_s == pytest.approx(r.s(stamp_ns + 2 * MS))
    assert reply.epoch == 0 and reply.attempt is None and meta == {"source": "real_slam"}


def test_a_diagnostic_session_sends_every_scan_and_hands_none_over():
    """7a-3 plan 3: a diagnostic session's replies are recorded, never evidence -- even of a scan with an interval."""
    r = Rig(mode="diagnostic")
    r.started()
    for measured in (False, True):
        out, _ = r.one(measured=measured)
        assert out == []
    assert [e["why"] for e in r.src.events if e.get("t") == "reply" and e.get("phase") == "ready"] == \
        ["diagnostic", "diagnostic"]


def test_an_operating_session_sends_only_scans_with_an_interval():
    """7a-3 plan 3 (Codex round 2 P2-3): a scan without a first-beam interval never reaches an operating bridge --
    not in the start check either."""
    r = Rig()
    r.step()
    [hello] = r.bridge.frames()
    r.bridge.state(1, "seeding")
    r.bridge.state(1, "ready_for_scans")
    r.step()
    r.src.offer(r.scan(measured=False))
    r.step()
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())
    assert r.src.counts["slam_not_qualified"] == 1 and r.src.target["start_scans"] == 0


def test_a_mode_is_always_named():
    with pytest.raises(TypeError):
        rss.RealSlamSource(None, map_id=MAP, start_ns=0, start_pose=(0, 0, 0), odom_history=None, odom_base=None,
                           odom_gen=None, boot=None, rear_offset_m=0.5)
    with pytest.raises(ValueError):
        rss.RealSlamSource(None, mode="evidence", map_id=MAP, start_ns=0, start_pose=(0, 0, 0), odom_history=None,
                           odom_base=None, odom_gen=None, boot=None, rear_offset_m=0.5)


def test_the_odometry_is_tried_once_more_a_tick_later_then_dropped():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=-5))  # its interval ends after the newest STATE
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())
    r.step()  # STATEs past it arrive
    assert len([f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]) == 1
    r2 = Rig()
    r2.started()
    r2.src.offer(r2.scan(lag_ms=-200))
    r2.step(20)
    r2.step(20)
    assert r2.src.counts["slam_no_odom"] == 1


def test_one_scan_in_flight_and_only_the_newest_waits():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=60))
    r.step(1)
    r.src.offer(r.scan(lag_ms=40))
    r.step(1)
    r.src.offer(r.scan(lag_ms=20))
    [first] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    assert r.src.counts["slam_superseded"] == 1
    r.bridge.reply(first)
    r.step(1)
    [nxt] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    assert nxt.stamp_ns > first.stamp_ns


def test_stamps_strictly_increase():
    r = Rig()
    r.started()
    s = r.scan(lag_ms=30)
    r.one()
    r.src.offer(s)  # older than what went
    assert r.src.counts["slam_stamp_order"] == 1


def test_a_late_reply_is_dropped_and_its_request_goes_to_the_next_scan():
    r = Rig()
    r.started()
    r.src.request_processed(0, 5, r.s(r.now) - 1.0)
    r.src.offer(r.scan(lag_ms=30))
    [sent] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.now += 300 * MS  # the reply read 0.33 s after the first beam
    r.bridge.reply(sent)
    assert r.step(1) == [] and r.src.counts["slam_late"] == 1
    out, _ = r.one()
    assert out[0][0].attempt == 5


def test_a_forced_request_tags_the_first_scan_after_its_time():
    r = Rig()
    r.started()
    r.src.request_processed(0, 3, r.s(r.now) + 0.05)
    out, _ = r.one(lag_ms=30)
    assert out[0][0].attempt is None
    r.step(80)
    out, _ = r.one(lag_ms=10)
    assert out[0][0].attempt == 3


def seed_to_ready(r, epoch=1, pose=(3.0, 1.0, 0.0)):
    sid = r.src.seed(epoch, pose, r.s(r.now))
    assert not r.src.can_seed() and not r.src.can_confirm() and r.src.seed_state(epoch) == ("seeding", None)
    r.step()
    [seed] = [f for f in r.bridge.frames() if isinstance(f, rl.Seed)]
    assert seed.seed_id == sid and seed.epoch == epoch and seed.start_pose == pose and seed.boot == r.boot_id
    r.bridge.state(sid, "seeding")
    r.bridge.state(sid, "ready_for_scans")
    r.step()
    return sid


def test_a_seed_runs_to_ready_and_its_replies_carry_its_epoch():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for _ in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        assert r.step() == []
    r.bridge.state(sid, "ready")
    r.step()
    assert r.src.seed_state(1) == ("ready", None) and r.src.can_confirm() and r.src.can_seed()
    out, _ = r.one(lag_ms=10)
    assert out[0][0].epoch == 1


def test_a_seed_waits_for_the_scan_in_flight_and_drops_the_waiting_scan():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=30))
    [inflight] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.step(1)
    r.src.offer(r.scan(lag_ms=10))  # waits behind the one in flight
    r.src.seed(1, (0.0, 0.0, 0.0), r.s(r.now))
    assert r.src.counts["slam_epoch_cut"] == 1
    r.step(1)
    assert not any(isinstance(f, rl.Seed) for f in r.bridge.frames())  # not over a scan in flight
    r.bridge.reply(inflight)
    assert r.step(1) == [] and r.src.counts["slam_old_instance_reply"] == 1  # the stopped instance's: no evidence
    assert any(isinstance(f, rl.Seed) for f in r.bridge.frames())


def test_scans_from_before_the_seed_never_reach_its_instance():
    r = Rig()
    r.started()
    early = r.scan(lag_ms=50)
    seed_to_ready(r)
    r.src.offer(early)
    r.step()
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())
    assert r.src.counts["slam_before_seed"] == 1


def test_a_second_seed_while_seeding_is_a_fault():
    r = Rig()
    r.started()
    r.src.seed(1, (0.0, 0.0, 0.0), r.s(r.now))
    with pytest.raises(rss.SlamSourceFatal):
        r.src.seed(2, (0.0, 0.0, 0.0), r.s(r.now))


def test_a_move_cancels_the_seed_and_an_end_state_before_the_void_opens_nothing():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    r.odom = (0.05, 0.0, 0.0)  # pushed
    r.step()
    [c] = [f for f in r.bridge.frames() if isinstance(f, rl.Cancel)]
    assert c.seed_id == sid and c.reason == "seed_moved"
    r.bridge.state(sid, "ready")  # sent before the bridge read the cancel
    r.step()
    assert r.src.seed_state(1) == ("seeding", None) and not r.src.can_confirm() and not r.src.can_seed()
    assert r.src.counts["slam_end_after_cancel"] == 1
    r.bridge.state(sid, "void", "seed_moved")
    r.step()
    assert r.src.seed_state(1) == ("void", "seed_moved") and r.src.can_seed() and not r.src.can_confirm()
    r.odom = (0.05, 0.0, 0.0)
    seed_to_ready(r, epoch=2)  # the next seed goes


def test_a_cancel_after_scan_five_with_its_pose_pending_lets_the_next_seed_run():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for k in range(5):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        if k < 4:
            r.bridge.reply(s)
        r.step()
    r.boot_id = 2  # the scan five's /pose never comes: the MCU rebooted
    r.step()
    [c] = [f for f in r.bridge.frames() if isinstance(f, rl.Cancel)]
    assert c.reason == "boot_changed"
    r.bridge.state(sid, "void", "boot_changed")
    r.step()
    r.bridge.reply(s)  # the scan five's late reply after the void: counted, nothing reopened (round 1 P2-4)
    r.bridge.state(sid, "ready")
    r.step()
    assert r.src.counts["slam_discarded_reply"] == 1 and r.src.counts["slam_state_after_void"] == 1
    assert r.src.seed_state(1) == ("void", "boot_changed")
    seed_to_ready(r, epoch=2)


def test_a_seed_not_yet_sent_is_cancelled_here_without_a_frame():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=30))
    [inflight] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.src.seed(1, (0.0, 0.0, 0.0), r.s(r.now))
    r.boot_id = 2  # the MCU rebooted before the seed went
    r.step()
    assert r.src.seed_state(1) == ("void", "boot_changed")
    r.bridge.reply(inflight)
    r.step()
    assert not any(isinstance(f, (rl.Seed, rl.Cancel)) for f in r.bridge.frames())
    r.src.seed(2, (0.0, 0.0, 0.0), r.s(r.now))  # the new boot's seed is taken
    r.step()
    assert any(isinstance(f, rl.Seed) and f.epoch == 2 for f in r.bridge.frames())


def test_a_cancel_not_sent_within_a_second_is_a_transport_fault(monkeypatch):
    r = Rig()
    r.started()
    seed_to_ready(r)
    monkeypatch.setattr(r.src.link, "flush", lambda: False)  # the socket stays full
    monkeypatch.setattr(r.src.link, "_out", b"x")
    r.odom = (0.05, 0.0, 0.0)
    r.step()
    with pytest.raises(rss.SlamSourceFatal, match="not sent"):
        for _ in range(60):
            r.step()


def test_a_late_tidy_up_is_no_fault_and_a_missing_void_is():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    r.odom = (0.05, 0.0, 0.0)
    r.step()
    r.now += int(4.9e9)
    r.bridge.state(sid, "void", "seed_moved")
    r.step(1)
    assert r.src.seed_state(1)[0] == "void"
    r2 = Rig()
    r2.started()
    seed_to_ready(r2)
    r2.odom = (0.05, 0.0, 0.0)
    r2.step()
    r2.now += int(5.1e9)
    with pytest.raises(rss.SlamSourceFatal, match="no void"):
        r2.step(1)


def test_a_cancel_decided_at_19_9_s_sent_after_20_s_still_voids_normally():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    r.now += int(19.88e9)
    r.hist_to(r.now)
    r.odom = (0.05, 0.0, 0.0)
    plain = r.src.link.flush
    r.src.link.flush = lambda: False  # the decision at 19.9 s, the frame stuck for a while
    r.src.link._out = b""
    r.step()
    r.src.link.flush = plain
    r.now += 500 * MS
    r.step(1)  # sent at ~20.4 s
    assert any(isinstance(f, rl.Cancel) for f in r.bridge.frames())
    r.bridge.state(sid, "void", "seed_moved")
    r.step(1)
    assert r.src.seed_state(1)[0] == "void"
    r.odom = (0.05, 0.0, 0.0)
    seed_to_ready(r, epoch=2)


def test_no_end_state_within_the_deadline_is_the_executors_fault():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.now += int(rss.SEED_WAIT_S * 1e9)
    r.hist_to(r.now)
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.step()


def test_no_reply_from_a_localized_instance_within_a_second_is_a_fault():
    r = Rig()
    r.started()
    r.src.offer(r.scan())
    r.now += int(1.1e9)
    with pytest.raises(rss.SlamSourceFatal, match="no reply"):
        r.step(1)


def test_the_seventh_reply_and_the_end_state_in_one_read_and_in_two():
    for split in (False, True):
        r = Rig()
        r.started()
        sid = seed_to_ready(r)
        for k in range(7):
            r.src.offer(r.scan(lag_ms=10))
            [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
            r.bridge.reply(s)
            if k < 6:
                r.step()
        if split:
            r.step()
            r.src.offer(r.scan(lag_ms=10))  # between the seventh reply and the end state: no eighth scan (P1-3)
            r.step()
            assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())
        r.bridge.state(sid, "ready")
        r.step()
        assert r.src.seed_state(1) == ("ready", None)


def test_a_reply_and_a_state_in_one_poll():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=20))
    [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    sid = r.src.seed(1, (0.0, 0.0, 0.0), r.s(r.now))
    r.bridge.reply(s)  # the old scan's reply ...
    out = r.step(1)
    [seed] = [f for f in r.bridge.frames() if isinstance(f, rl.Seed)]
    r.bridge.state(sid, "seeding")  # ... the seed's two states together in one read
    r.bridge.state(sid, "ready_for_scans")
    r.step(1)
    assert out == [] and seed.seed_id == sid and r.src.target["state"] == "scans"


def test_a_not_localized_seed_ends_and_hands_nothing_over():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for _ in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.bridge.state(sid, "not_localized", "NOT_LOCALIZED: fractions")
    r.step()
    assert r.src.seed_state(1) == ("not_localized", "NOT_LOCALIZED: fractions")
    assert not r.src.can_confirm() and r.src.can_seed()
    r.src.offer(r.scan(lag_ms=10))
    r.step()
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())  # the bridge takes no scan now


@pytest.mark.parametrize("what", ["eof", "garbage", "fault", "stray_state", "void_without_cancel"])
def test_transport_and_protocol_faults(what):
    r = Rig()
    r.started()
    if what == "eof":
        r.bridge.sock.close()
    elif what == "garbage":
        r.bridge.sock.sendall((5).to_bytes(4, "big") + b"nope\n")
    elif what == "fault":
        r.bridge.state(1, "fault", "NOT_LOADED")
    elif what == "stray_state":
        r.bridge.state(9, "ready")
    else:
        r.bridge.state(1, "void", "x")
    with pytest.raises(rss.SlamSourceFatal):
        r.step()


def test_a_relocalize_labels_only_scans_acquired_after_it():
    r = Rig()
    r.started()
    early = r.scan(lag_ms=50)
    r.src.relocalize(1)
    r.src.offer(early)
    [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.bridge.reply(s)
    out = r.step(1)
    assert out[0][0].epoch == 0
    r.step(60)
    out, _ = r.one(lag_ms=10)
    assert out[0][0].epoch == 1


# --- Codex 7c-1b-2a round 1 ----------------------------------------------------------------------------------------
def test_a_broken_input_and_the_end_state_in_one_poll_the_cancel_wins():
    """P1-1: the boot changed and ready arrived in the same poll -- the seed is cancelled, never ready."""
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for _ in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.boot_id = 2
    r.bridge.state(sid, "ready")
    r.step()
    assert r.src.seed_state(1) == ("seeding", None) and not r.src.can_confirm()
    assert any(isinstance(f, rl.Cancel) and f.reason == "boot_changed" for f in r.bridge.frames())


def test_a_same_boot_break_cancels_even_with_the_history_cleared():
    """P1-2 / round 2 P1: the loop's continuity generation, not the (cleared) history nor a flag seen at one poll."""
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.odom_gen += 1
    r.history.clear()
    r.step()
    assert any(isinstance(f, rl.Cancel) and f.reason == "odom_break" for f in r.bridge.frames())


def test_a_move_and_back_within_one_batch_is_still_a_move():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.odom = (0.05, 0.0, 0.0)
    r.hist_to(r.now + 10 * MS)
    r.odom = (0.0, 0.0, 0.0)  # back where it was by the poll
    r.step()
    assert any(isinstance(f, rl.Cancel) and f.reason == "seed_moved" for f in r.bridge.frames())


def test_an_end_state_delivered_past_the_deadline_and_a_late_reply_are_faults():
    """Round 2 P2-3: the deadline runs to the end state's delivery (design 6.1, 2)."""
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for _ in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.bridge.state(sid, "ready")
    r.now += int(rss.SEED_WAIT_S * 1e9)
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.step(1)
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=10))
    [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.now += int(1.5e9)
    r.bridge.reply(s)
    seen = []
    r.src.reply_hook = lambda scan, rec: seen.append(scan.scan_id) or {"q": 1}
    with pytest.raises(rss.SlamSourceFatal, match="no reply"):
        r.step(1)
    rows = [e for e in r.src.events if e.get("t") == "reply" and e.get("why") == "past_reply_bound"]
    assert rows and rows[0]["q"] == 1 and "epoch" in rows[0] and seen == [s.scan.scan_id]  # recorded before the fault


def test_a_void_read_past_its_bound_is_a_fault():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    r.odom = (0.05, 0.0, 0.0)
    r.step()
    r.bridge.state(sid, "void", "seed_moved")
    r.now += int(5.5e9)
    with pytest.raises(rss.SlamSourceFatal, match="no void"):
        r.step(1)


def test_a_broken_input_first_seen_past_the_deadline_is_the_deadline():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.now += int(rss.SEED_WAIT_S * 1e9) + 1
    r.boot_id = 2
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.step(1)


class Chunky:
    """A socket whose sends go ``limits`` bytes at a time (None: EAGAIN)."""

    def __init__(self, sock, limits):
        self.sock, self.limits = sock, list(limits)

    def send(self, data):
        lim = self.limits.pop(0) if self.limits else len(data)
        if lim is None:
            raise BlockingIOError
        return self.sock.send(data[:lim])

    def __getattr__(self, name):
        return getattr(self.sock, name)


def test_a_cancel_written_in_parts_goes_once():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    r.src.link.sock = Chunky(r.src.link.sock, [5, None])
    r.odom = (0.05, 0.0, 0.0)
    r.step()
    r.step()
    r.step()
    cancels = [f for f in r.bridge.frames() if isinstance(f, rl.Cancel)]
    assert len(cancels) == 1 and cancels[0].seed_id == sid


def test_a_seed_not_one_byte_of_which_went_is_cancelled_here():
    r = Rig()
    r.started()
    r.src.link.sock = Chunky(r.src.link.sock, [None])  # only the seed's first write: the next poll's could go
    r.src.seed(1, (0.0, 0.0, 0.0), r.s(r.now))
    r.boot_id = 2
    r.step()
    assert r.src.seed_state(1) == ("void", "boot_changed")
    r.step()
    assert not any(isinstance(f, (rl.Seed, rl.Cancel)) for f in r.bridge.frames())


def test_a_scan_stuck_in_the_socket_is_a_fault_after_a_second():
    r = Rig()
    r.started()
    r.src.link.sock = Chunky(r.src.link.sock, [None] * 1000)
    r.src.offer(r.scan())
    with pytest.raises(rss.SlamSourceFatal, match="not written whole"):
        for _ in range(60):
            r.step()


def _start_scans(r, k, ident=1):
    """hello -> seeding -> ready_for_scans -> k start scans, each replied."""
    r.step()
    r.bridge.frames()
    r.bridge.state(ident, "seeding")
    r.bridge.state(ident, "ready_for_scans")
    r.step()
    for _ in range(k):
        r.src.offer(r.scan())
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()


def test_an_old_generations_ready_never_opens_the_confirmation():
    """7a-3 plan 3 (Codex plan round 4 P2-3, a gap since 7c-1): seven start scans, then a generation change, then the
    earlier generation's ready -- the start check is void: the session ends (no second hello on the connection,
    round 5) and nothing confirms."""
    r = Rig()
    _start_scans(r, 7)
    r.gen = (1, 77)
    with pytest.raises(rss.SlamSourceFatal, match="start_check_generation_changed"):
        r.src.generation(r.gen)  # told before the scan is judged, whatever its verdict
    r.bridge.state(1, "ready")
    with pytest.raises(rss.SlamSourceFatal):
        r.step()
    assert not r.src.can_confirm()


def test_a_generation_change_before_any_start_scan_keeps_the_start_check():
    r = Rig()
    _start_scans(r, 0)
    r.src.generation(r.gen)  # the earlier generation is known (a scan of it seen, none sent)
    r.gen = (1, 77)
    r.src.generation(r.gen)
    for _ in range(7):
        r.src.offer(r.scan())
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.bridge.state(1, "ready")
    r.step()
    assert r.src.can_confirm()


def test_a_start_scan_not_one_byte_of_which_went_is_taken_back_on_a_generation_change():
    """Codex 7a-3a round 1 P3-1: the first start scan's write stalled before any byte -- the bridge has seen no scan
    of the earlier generation, so the start check goes on with the new one."""
    r = Rig()
    _start_scans(r, 0)
    sock = r.src.link.sock
    r.src.link.sock = Chunky(sock, [None])
    r.src.offer(r.scan())
    assert r.src.target["start_scans"] == 1 and r.src.link.busy
    r.gen = (1, 77)
    r.src.generation(r.gen)
    assert r.src.target["start_scans"] == 0 and not r.src.link.busy
    assert r.src.counts["slam_generation_cut"] == 1
    r.src.link.sock = sock
    # the scan taken back is no stamp sent: the new generation's scan at the same instant goes (round 2 P3)
    for _ in range(7):
        r.src.offer(r.scan())
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.bridge.state(1, "ready")
    r.step()
    assert r.src.can_confirm()


def test_an_unqualified_scan_still_changes_the_generation():
    """Codex plan round 3 P2-3: the generation is told before the qualification -- an operating session's start
    check with old-generation scans ends even when the new generation's first scan has no interval."""
    r = Rig()
    _start_scans(r, 3)
    r.gen = (1, 77)
    with pytest.raises(rss.SlamSourceFatal, match="start_check_generation_changed"):
        r.src.offer(r.scan(measured=False))


def test_a_generation_change_cancels_a_seed_and_only_its_successor_confirms():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for _ in range(3):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.gen = (1, 77)
    r.src.generation(r.gen)
    r.step()
    [c] = [f for f in r.bridge.frames() if isinstance(f, rl.Cancel)]
    assert c.seed_id == sid and c.reason == "generation_changed"
    r.bridge.state(sid, "ready")  # the old seed's end state, sent before the bridge read the cancel
    r.step()
    assert not r.src.can_confirm() and r.src.counts["slam_end_after_cancel"] == 1
    r.bridge.state(sid, "void", "generation_changed")
    r.step()
    assert r.src.seed_state(1) == ("void", "generation_changed") and not r.src.can_confirm()
    sid2 = seed_to_ready(r, epoch=2)
    for _ in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.bridge.state(sid2, "ready")
    r.step()
    assert r.src.seed_state(2) == ("ready", None) and r.src.can_confirm()


def test_an_earlier_generations_reply_frees_the_slot_and_its_request_goes_on():
    r = Rig()
    r.started()
    r.src.request_processed(0, 4, r.s(r.now) - 1.0)
    r.src.offer(r.scan(lag_ms=30))
    [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.gen = (1, 77)  # the adapter restarted
    r.src.offer(r.scan(lag_ms=10))
    r.bridge.reply(s)
    assert r.step(1) == [] and r.src.counts["slam_old_generation_reply"] == 1
    r.step(30)
    out, _ = r.one(lag_ms=10)
    assert out[0][0].attempt == 4


def test_a_non_finite_correction_or_a_float_id_is_a_fault():
    for corr, sid_f in (((0.0, float("nan"), 0.0), False), ((0.0, 0.0, 0.0), True)):
        r = Rig()
        r.started()
        r.src.offer(r.scan())
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.sock.sendall(slam_link._frame({"type": "reply", "scan_id": s.scan.scan_id + (0.0 if sid_f else 0),  # noqa: SLF001
                                                "stamp_s": s.scan.stamp_s, "status": "processed",
                                                "map_from_odom": list(corr), "version": 1, "slam_wall_s": 0.0}))
        with pytest.raises(rss.SlamSourceFatal, match="malformed"):
            r.step(1)


def test_a_scan_without_an_interval_does_not_take_a_forced_request():
    r = Rig()
    r.started()
    r.src.request_processed(0, 6, r.s(r.now) - 1.0)
    r.src.offer(r.scan(measured=False))  # not sent at all
    r.step(1)
    out, _ = r.one()
    assert out[0][0].attempt == 6


def test_a_diagnostic_session_takes_no_forced_request():
    r = Rig(mode="diagnostic")
    r.started()
    r.src.request_processed(0, 6, r.s(r.now) - 1.0)
    r.one()
    assert [q[1] for q in r.src._requests] == [6]  # noqa: SLF001 -- still waiting for evidence


def test_an_oversized_frame_is_a_fault():
    r = Rig()
    r.started()
    r.bridge.sock.sendall((rss.MAX_FRAME + 1).to_bytes(4, "big"))
    with pytest.raises(rss.SlamSourceFatal, match="bytes"):
        r.step(1)


def test_no_seed_before_the_sessions_start_ends():
    r = Rig()
    assert not r.src.can_seed()  # before the first poll too (round 1 P3)
    with pytest.raises(rss.SlamSourceFatal):
        r.src.seed(1, (0.0, 0.0, 0.0), 0.0)


def test_the_seed_lower_bound_holds_while_its_instance_is_ready():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for _ in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.bridge.state(sid, "ready")
    r.step()
    accepted = r.src.target["accepted_ns"]
    wide = r.scan(lag_ms=5)
    wide = srs.SlamScanIn(**{**wide.__dict__, "flo_ns": accepted - MS, "fhi_ns": wide.stamp_ns})
    r.src.offer(wide)
    r.step()
    assert r.src.counts["slam_before_seed"] == 1


# --- Codex 7c-1b-2a round 2 ----------------------------------------------------------------------------------------
def test_a_frame_whose_last_byte_goes_late_is_a_fault():
    r = Rig()
    r.started()
    r.src.link.sock = Chunky(r.src.link.sock, [3, None])
    r.src.offer(r.scan())
    r.now += int(1.01e9)
    with pytest.raises(rss.SlamSourceFatal, match="not written whole"):
        r.step(1)


def test_a_cancel_whose_last_byte_goes_past_its_bound_is_a_fault():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.src.link.sock = Chunky(r.src.link.sock, [5] + [None] * 50)
    r.odom = (0.05, 0.0, 0.0)
    r.step()
    r.now += int(0.95e9)
    r.step(1)
    r.now += int(0.15e9)
    with pytest.raises(rss.SlamSourceFatal, match="not"):
        r.step(1)


def test_an_offer_past_the_deadline_never_turns_it_into_a_cancel():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.now += int(21e9)
    r.hist_to(r.now)
    r.boot_id = 2
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.src.offer(r.scan())


def test_an_end_state_before_the_seventh_reply_is_a_fault():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for k in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        if k < 6:
            r.bridge.reply(s)
            r.step()
    r.bridge.state(sid, "ready")
    with pytest.raises(rss.SlamSourceFatal, match="start scans"):
        r.step()


def test_a_skipped_from_a_localized_instance_is_a_fault():
    r = Rig()
    r.started()
    r.src.offer(r.scan())
    [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.bridge.reply(s, status="skipped")
    with pytest.raises(rss.SlamSourceFatal, match="skipped"):
        r.step(1)


def test_an_older_scan_never_replaces_a_fresher_waiting_one():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=30))
    [first] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.step(1)
    fresh = r.scan(lag_ms=10)
    r.src.offer(fresh)
    r.src.offer(r.scan(lag_ms=25))
    assert r.src.counts["slam_stamp_order"] == 1
    r.bridge.reply(first)
    r.step(1)
    [nxt] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    assert nxt.stamp_ns == fresh.stamp_ns


def test_buffered_frames_come_before_new_reads_and_the_backlog_is_bounded():
    r = Rig()
    r.started()
    link = r.src.link
    link._buffer = b"".join(rl.encode_state(rl.State(1, "seeding")) for _ in range(rss.READ_FRAMES + 5))
    assert len(link.read()) == rss.READ_FRAMES and len(link.read()) == 5
    one = rl.encode_state(rl.State(1, "seeding"))
    link._buffer = one * ((rss.MAX_FRAME + rss.READ_BYTES) // len(one) + 4 * rss.READ_FRAMES)  # past a poll
    with pytest.raises(rss.SlamSourceFatal, match="behind"):
        link.read()


# --- Codex 7c-1b-2a round 3 ----------------------------------------------------------------------------------------
def test_a_move_seen_only_on_a_pose_that_left_the_history_still_cancels():
    r = Rig()
    r.started()
    seed_to_ready(r)
    from forklift_core.localization.start_reloc import rear_from_base

    t = r.s(r.now)
    r.src.odom_step((t, t, rear_from_base((0.02, 0.0, 0.0), 0.5), True, True, True, True))  # 2 cm and back
    r.src.odom_step((t, t, rear_from_base((0.0, 0.0, 0.0), 0.5), True, True, True, True))
    r.history.clear()
    r.hist_to(r.now)
    r.step()
    assert any(isinstance(f, rl.Cancel) and f.reason == "seed_moved" for f in r.bridge.frames())


def test_an_end_state_read_across_the_deadline_within_one_poll_is_a_fault():
    r = Rig()
    r.started()
    sid = seed_to_ready(r)
    for _ in range(7):
        r.src.offer(r.scan(lag_ms=10))
        [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
        r.bridge.reply(s)
        r.step()
    r.bridge.state(sid, "ready")
    r.now = r.src.target["deadline_ns"] - MS
    plain = r.src.link.read

    def slow_read():
        r.now += 2 * MS  # the read itself crosses the deadline
        return plain()

    r.src.link.read = slow_read
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.src.replies(r.now)


def test_a_cancel_goes_before_a_seed_scan_not_one_byte_of_which_went():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.src.link.sock = Chunky(r.src.link.sock, [None])
    r.src.offer(r.scan(lag_ms=10))
    r.boot_id = 2
    r.step()
    got = [f for f in r.bridge.frames() if isinstance(f, (rl.RealScan, rl.Cancel))]
    assert [type(f).__name__ for f in got] == ["Cancel"]


def test_a_fresher_scan_waiting_for_its_odometry_is_not_pushed_out_by_an_older_one():
    r = Rig()
    r.started()
    fresh = r.scan(lag_ms=-5)  # its odometry not in yet
    r.src.offer(fresh)
    r.src.offer(r.scan(lag_ms=30))
    assert r.src.counts["slam_stamp_order"] == 1
    r.step()
    [sent] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    assert sent.stamp_ns == fresh.stamp_ns


# --- Codex 7c-1b-2a round 4 ----------------------------------------------------------------------------------------
def test_a_broken_entry_that_left_the_history_still_cancels_and_an_excluded_one_does_not():
    r = Rig()
    r.started()
    seed_to_ready(r)
    t = r.s(r.now)
    r.src.odom_step((t - 0.05, t, r.history[-1][2], True, False, True, True))  # over the STATE delay bound only
    r.step()
    assert not any(isinstance(f, rl.Cancel) for f in r.bridge.frames())  # still a continuous pose (7c-1b-2b)
    r.src.odom_step((t, t + 0.01, r.history[-1][2], False, True, True, True))  # a continuity break
    r.history.clear()
    r.hist_to(r.now)
    r.step()
    assert any(isinstance(f, rl.Cancel) and f.reason == "odom_break" for f in r.bridge.frames())


def test_an_offer_never_lifts_an_expired_write_bound_or_seed_deadline():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.src.link.sock = Chunky(r.src.link.sock, [None] * 10)
    r.src.offer(r.scan(lag_ms=10))
    r.now += int(1.001e9)
    r.boot_id = 2
    with pytest.raises(rss.SlamSourceFatal, match="not written whole"):
        r.src.offer(r.scan(lag_ms=5))
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.now = r.src.target["deadline_ns"] + MS
    r.hist_to(r.now)
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.src.offer(r.scan(lag_ms=10))


def test_a_scan_taken_back_is_not_recorded_as_sent():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.src.link.sock = Chunky(r.src.link.sock, [None])
    r.src.offer(r.scan(lag_ms=10))
    r.boot_id = 2
    r.step()
    key = [e for e in r.src.events if e.get("t") == "slam_drop" and e["why"] == "slam_cancel_cut"][0]["key"]
    assert not any(e.get("t") == "scan_sent" and e["key"] == key for e in r.src.events)


# --- Codex 7c-1b-2a round 5 ----------------------------------------------------------------------------------------
def test_offers_alone_still_judge_the_void_and_reply_bounds():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.odom = (0.05, 0.0, 0.0)
    r.step()  # the cancel goes; no void ever comes and no poll runs after
    r.now += int(5.1e9)
    r.hist_to(r.now)
    with pytest.raises(rss.SlamSourceFatal, match="no void"):
        r.src.offer(r.scan(lag_ms=10))


def test_a_start_scan_never_goes_once_the_deadline_passes_during_the_offer():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.now = r.src.target["deadline_ns"] - MS
    r.hist_to(r.now)
    plain = r.src.odom_history

    def slow_history():
        r.now += 2 * MS  # the odometry lookup itself crosses the deadline
        return plain()

    r.src.odom_history = slow_history
    scan = r.scan(lag_ms=40)  # its odometry in: it would go at once
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.src.offer(scan)
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())


# --- Codex 7c-1b-2a round 6 ----------------------------------------------------------------------------------------
def test_a_start_scan_never_goes_once_the_deadline_passes_while_it_is_encoded_or_retried():
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.now = r.src.target["deadline_ns"] - MS
    r.hist_to(r.now)
    plain = rl.encode_scan

    def slow_encode(*a):
        r.now += 2 * MS
        return plain(*a)

    rss.rl.encode_scan = slow_encode
    try:
        with pytest.raises(rss.SlamSourceFatal, match="no end state"):
            r.src.offer(r.scan(lag_ms=40))
    finally:
        rss.rl.encode_scan = plain
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())
    r = Rig()
    r.started()
    seed_to_ready(r)
    r.src.link.sock = Chunky(r.src.link.sock, [None])
    r.now = r.src.target["deadline_ns"] - 5 * MS
    r.hist_to(r.now)
    r.src.offer(r.scan(lag_ms=40))  # its first write: EAGAIN, no byte out
    r.now += 10 * MS
    with pytest.raises(rss.SlamSourceFatal, match="no end state"):
        r.src.link.flush()  # a retry past the deadline: the gate refuses the first byte
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())


def test_offers_alone_judge_the_reply_bound():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=10))  # to the localized instance; no reply comes and no poll runs
    r.now += int(1.1e9)
    r.hist_to(r.now)
    with pytest.raises(rss.SlamSourceFatal, match="no reply"):
        r.src.offer(r.scan(lag_ms=10))


# --- 7c-1c (the observe run's input breaks) --------------------------------------------------------------------------
def test_an_input_break_takes_back_a_start_scan_not_one_byte_of_which_went():
    r = Rig()
    r.step()
    [hello] = r.bridge.frames()
    r.bridge.state(1, "seeding")
    r.bridge.state(1, "ready_for_scans")
    r.step()
    r.src.link.sock = Chunky(r.src.link.sock, [None, None])
    r.src.offer(r.scan())
    assert r.src.target["start_scans"] == 1 and r.src._inflight is not None  # noqa: SLF001
    r.src.input_break("disconnected")
    assert r.src._inflight is None and r.src.target["start_scans"] == 0  # noqa: SLF001
    assert r.src.counts["slam_input_break"] == 1
    r.step()
    assert not any(isinstance(f, rl.RealScan) for f in r.bridge.frames())



def test_a_late_read_records_only_the_reply_with_the_scans_own_stamp():
    """7c-1c Codex round 4 P2: a reply of the same id but another stamp is not that scan's."""
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=10))
    [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    r.now += int(1.5e9)
    r.bridge.sock.sendall(slam_link.encode_reply(slam_link.Reply(s.scan.scan_id, s.scan.stamp_s + 1.0, "processed",
                                                                 (9.0, 9.0, 0.0), 1, 0.0)))
    r.bridge.reply(s, corr=(0.1, 0.0, 0.0))
    with pytest.raises(rss.SlamSourceFatal, match="no reply"):
        r.step(1)
    rows = [e for e in r.src.events if e.get("t") == "reply" and e.get("why") == "past_reply_bound"]
    assert len(rows) == 1 and rows[0]["map_from_odom"] == [0.1, 0.0, 0.0]


def test_a_feed_generation_told_before_the_replies_keeps_a_late_ready_from_confirming():
    """Codex 7a-3a round 1 P2-1: the observe run's receive gap opens a feed generation with no scan; the bridge's late
    ready of the start check (its seventh reply already in) arrives after -- told first, it never confirms."""
    from forklift_core.live.observe_slam import ObserveSlam

    r = Rig(mode="diagnostic")
    obs = ObserveSlam.__new__(ObserveSlam)
    obs.src, obs.fatal, obs._producer, obs._events = r.src, None, None, 0  # noqa: SLF001
    obs.put = lambda row: None
    _start_scans(r, 7)
    obs.on_feed_generation(1)  # what the observe loop does at the gap, before slam.cycle reads the bridge
    assert obs.fatal == "start_check_generation_changed"
    r.bridge.state(1, "ready")
    with pytest.raises(rss.SlamSourceFatal):
        r.step()
    assert not r.src.can_confirm()
