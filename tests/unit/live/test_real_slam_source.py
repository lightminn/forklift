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
    def __init__(self):
        a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        self.bridge = Bridge(b)
        self.now = START_NS + 2_000 * MS
        self.history = []
        self.odom = (0.0, 0.0, 0.0)
        self.boot_id = 1
        self.src = rss.RealSlamSource(
            rss.RealSlamLink(a), map_id=MAP, start_ns=START_NS, start_pose=(1.0, 2.0, 0.3),
            odom_history=lambda: self.history, odom_base=lambda: self.odom, boot=lambda: self.boot_id,
            rear_offset_m=0.5, clock=lambda: self.now)
        self.hist_to(self.now)

    def s(self, ns):
        return (ns - START_NS) / 1e9

    def hist_to(self, ns):
        """STATE entries every 10 ms up to ns (core seconds), at the current odometry."""
        last = self.history[-1][1] if self.history else self.s(ns) - 0.5
        t = last + 0.01
        while t <= self.s(ns) + 1e-9:
            self.history.append((t - 0.005, t, self.odom, True, True, True, True))
            t += 0.01

    def step(self, ms=20):
        self.now += ms * MS
        self.hist_to(self.now)
        return self.src.replies(self.now)

    def scan(self, lag_ms=30, measured=True, width_ms=1):
        t = self.now - lag_ms * MS
        n = 8
        return srs.SlamScanIn(key=(0, None, t), stamp_ns=t, flo_ns=t - width_ms * MS, fhi_ns=t + width_ms * MS,
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


def test_a_diagnostic_scan_is_sent_and_never_handed_over():
    r = Rig()
    r.started()
    out, _ = r.one(measured=False)
    assert out == [] and any(e.get("why") == "diagnostic" for e in r.src.events)


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
    r.bridge.reply(s)  # its late reply after the void: protocol (nothing in flight)
    with pytest.raises(rss.SlamSourceFatal):
        r.step()


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


def test_a_reply_and_a_state_in_one_poll():
    r = Rig()
    r.started()
    r.src.offer(r.scan(lag_ms=20))
    [s] = [f for f in r.bridge.frames() if isinstance(f, rl.RealScan)]
    sid = r.src.seed(1, (0.0, 0.0, 0.0), r.s(r.now))
    r.bridge.reply(s)  # the old scan's reply ...
    out = r.step(1)
    [seed] = [f for f in r.bridge.frames() if isinstance(f, rl.Seed)]
    r.bridge.state(sid, "seeding")  # ... and the seed's states in the same read
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
