"""Isaac <-> SLAM bridge messages (docs/plans/2026-10-04-online-slam-closed-loop.md)."""

import socket
import threading

import numpy as np
import pytest

from forklift_core.localization import slam_link


def _scan(scan_id=7):
    return slam_link.Scan(
        scan_id=scan_id,
        stamp_s=12.5,
        odom_from_base=(1.0, -2.0, 0.25),
        ranges_m=np.linspace(0.3, 11.0, 1600).astype(np.float32),
        angle_min_rad=-np.pi,
        angle_increment_rad=2 * np.pi / 1600,
        range_min_m=0.2,
        range_max_m=12.0,
    )


def test_a_scan_round_trips_bit_for_bit():
    scan = _scan()
    back = slam_link.decode_scan(slam_link.encode_scan(scan)[4:])
    assert back.scan_id == 7 and back.stamp_s == 12.5
    assert back.odom_from_base == (1.0, -2.0, 0.25)
    np.testing.assert_array_equal(back.ranges_m, scan.ranges_m)
    assert back.ranges_m.dtype == np.float32


def test_a_reply_round_trips():
    reply = slam_link.Reply(7, 12.5, "processed", (0.1, 0.2, 0.03), 4, 0.014)
    assert slam_link.decode_reply(slam_link.encode_reply(reply)[4:]) == reply


def test_unknown_reply_status_is_rejected():
    with pytest.raises(ValueError):
        slam_link.Reply(7, 12.5, "stale", (0, 0, 0), 0, 0.0)


def _started():
    gate = slam_link.KeyframeGate()
    kinds = []
    for k in range(5):
        kinds.append(gate.classify(0.1 * k, (0.0, 0.0, 0.0)))
        if kinds[-1] == "expect_pose":
            gate.processed(0.1 * k, (0.0, 0.0, 0.0))
    return gate, kinds


def test_start_up_sends_five_scans_and_expects_poses_for_the_first_and_fifth():
    """slam_toolbox drops its 2nd-4th received scans (job 747)."""
    _, kinds = _started()
    assert kinds == ["expect_pose", "startup_drop", "startup_drop", "startup_drop", "expect_pose"]


def test_keyframes_need_half_a_second_and_0p447_m_or_0p5_rad():
    gate, _ = _started()  # reference: t=0.4, pose 0
    assert gate.classify(0.8, (0.30, 0.0, 0.0)) == "local"  # 0.4 s: too soon
    assert gate.classify(0.9, (0.44, 0.0, 0.0)) == "local"  # 0.44 m travelled, no turn
    assert gate.classify(1.0, (0.45, 0.0, 0.0)) == "expect_pose"
    gate.processed(1.0, (0.45, 0.0, 0.0))
    assert gate.classify(1.5, (0.45, 0.0, 0.49)) == "local"
    assert gate.classify(1.6, (0.45, 0.0, -0.02)) == "expect_pose"  # 0.49 + 0.51 rad turned
    # The reference moves only with processed(): an unanswered keyframe does not.
    assert gate.classify(2.2, (0.45, 0.0, -0.02)) == "expect_pose"


def test_going_back_and_forth_still_makes_keyframes():
    gate, _ = _started()
    kinds = [gate.classify(0.5 + 0.1 * k, (0.1 * (k % 4), 0.0, 0.0)) for k in range(12)]
    assert "expect_pose" in kinds  # displacement never exceeds 0.3 m


def test_keyframe_heading_wraps_at_pi():
    gate, _ = _started()
    gate.classify(1.0, (0.0, 0.0, 3.1))
    gate.processed(1.0, (0.0, 0.0, 3.1))
    assert gate.classify(1.6, (0.0, 0.0, -3.1)) == "local"  # 0.083 rad across the wrap


def _serve(server_sock, handler):
    conn, _ = server_sock.accept()
    with conn:
        while True:
            try:
                scan = slam_link.recv_scan(conn)
            except ConnectionError:
                return
            conn.sendall(slam_link.encode_reply(handler(scan)))


def test_the_client_waits_for_the_reply_to_this_scan(tmp_path):
    path = str(tmp_path / "slam.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(path)
    server.listen(1)
    thread = threading.Thread(
        target=_serve,
        args=(server, lambda s: slam_link.Reply(s.scan_id, s.stamp_s, "processed", (0, 0, 0), 1, 0.01)),
        daemon=True,
    )
    thread.start()
    client = slam_link.SlamLinkClient(path, timeout_s=5.0)
    reply = client.exchange(_scan(11))
    assert reply.scan_id == 11 and reply.status == "processed"
    client.close()
    server.close()


def test_a_reply_for_another_scan_is_a_failure(tmp_path):
    path = str(tmp_path / "slam.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(path)
    server.listen(1)
    threading.Thread(
        target=_serve,
        args=(server, lambda s: slam_link.Reply(s.scan_id + 1, s.stamp_s, "processed", (0, 0, 0), 1, 0.0)),
        daemon=True,
    ).start()
    client = slam_link.SlamLinkClient(path, timeout_s=5.0)
    with pytest.raises(slam_link.SlamLinkFailure):
        client.exchange(_scan(3))
    client.close()
    server.close()


def test_a_silent_bridge_times_out(tmp_path):
    path = str(tmp_path / "slam.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(path)
    server.listen(1)
    client = slam_link.SlamLinkClient(path, timeout_s=0.2)
    with pytest.raises(slam_link.SlamLinkFailure):
        client.exchange(_scan())
    client.close()
    server.close()


# --- real-time plan D9a: the link without waiting ---------------------------------------------------------------
import socket as _socket
from types import SimpleNamespace

from forklift_core.localization import slam_link as _sl


_OPEN: list = []


@pytest.fixture(autouse=True)
def _close_pairs():
    yield
    while _OPEN:
        _OPEN.pop().close()


def async_pair(**kw):
    a, b = _socket.socketpair(_socket.AF_UNIX, _socket.SOCK_STREAM)
    _OPEN.extend((a, b))
    now = [0.0]
    link = _sl.AsyncSlamLink(SimpleNamespace(sock=a), clock=lambda: now[0], **kw)
    return link, b, now


def scan_of(i):
    return _sl.Scan(i, 0.1 * i, (0.0, 0.0, 0.0), np.ones(4, dtype=np.float32), -1.0, 0.5, 0.1, 10.0)


def answer(bridge, status="skipped", pose=(0.0, 0.0, 0.0)):
    scan = _sl.recv_scan(bridge)
    bridge.sendall(_sl.encode_reply(_sl.Reply(scan.scan_id, scan.stamp_s, status, pose, 1, 0.01)))
    return scan


def test_scans_go_one_at_a_time_in_order_and_replies_come_back_with_their_records():
    link, bridge, now = async_pair()
    for i in range(3):
        link.enqueue(scan_of(i), {"i": i})
    assert link.depth() == 3 and link.max_depth == 3
    bridge.settimeout(0.5)
    assert answer(bridge).scan_id == 0
    with pytest.raises(_socket.timeout):
        _sl.recv_scan(bridge)  # the next one is not sent before the reply is read
    now[0] = 0.2
    (got,) = link.poll()
    assert got[0].scan_id == 0 and got[1] == {"i": 0} and got[2:] == (0.0, 0.2) and link.last_reply_id == 0
    assert answer(bridge).scan_id == 1  # sent on the poll that read the reply
    (got,) = link.poll()
    assert got[0].scan_id == 1  # and scan 2 sent now
    assert answer(bridge, "processed", (1.0, 0.0, 0.0)).scan_id == 2
    (got,) = link.poll()
    assert got[0].scan_id == 2 and got[0].status == "processed" and got[0].map_from_odom == (1.0, 0.0, 0.0)
    assert link.depth() == 0 and link.poll() == []


def test_a_failed_reply_a_missed_deadline_a_full_queue_or_a_closed_bridge_fail_the_link():
    link, bridge, now = async_pair()
    link.enqueue(scan_of(0), {})
    answer(bridge, "failed")
    with pytest.raises(_sl.SlamLinkFailure, match="failure"):
        link.poll()
    link, bridge, now = async_pair(deadline_s=1.0)
    link.enqueue(scan_of(0), {})
    now[0] = 0.9
    assert link.poll() == []
    now[0] = 1.01
    with pytest.raises(_sl.SlamLinkFailure, match="no reply within"):
        link.poll()
    link, bridge, now = async_pair(queue_max=2)
    for i in range(3):
        link.enqueue(scan_of(i), {})  # one in flight, two queued
    with pytest.raises(_sl.SlamLinkFailure, match="queued"):
        link.enqueue(scan_of(3), {})
    link, bridge, now = async_pair()
    link.enqueue(scan_of(0), {})
    bridge.close()
    with pytest.raises(_sl.SlamLinkFailure):
        link.poll()


def test_a_reply_to_another_scan_fails_the_link():
    link, bridge, now = async_pair()
    link.enqueue(scan_of(0), {})
    _sl.recv_scan(bridge)
    bridge.sendall(_sl.encode_reply(_sl.Reply(7, 0.7, "skipped", (0.0, 0.0, 0.0), 1, 0.0)))
    with pytest.raises(_sl.SlamLinkFailure, match="reply for scan 7"):
        link.poll()


def test_an_empty_poll_never_waits_even_on_the_lockstep_client_s_timeout_socket():
    """Codex D9 impl P1: the lockstep client's socket has a timeout; the link must not wait it out."""
    import time as _time

    link, bridge, now = async_pair()
    link.client.sock.settimeout(5.0)
    link = _sl.AsyncSlamLink(link.client, clock=lambda: now[0])  # takes the socket over as the runner does
    link.enqueue(scan_of(0), {})
    started = _time.perf_counter()
    assert link.poll() == []
    assert _time.perf_counter() - started < 0.05


def test_a_reply_missing_at_a_poll_after_the_deadline_fails_the_link_before_it_comes():
    """Codex D9 impl P2: a late reply is no reply -- the poll that finds none after the deadline ends the link."""
    link, bridge, now = async_pair(deadline_s=1.0)
    link.enqueue(scan_of(0), {})
    now[0] = 0.99
    assert link.poll() == []
    now[0] = 1.005
    with pytest.raises(_sl.SlamLinkFailure, match="no reply within"):
        link.poll()


def test_a_reply_found_on_the_first_poll_after_the_runner_stood_still_is_taken():
    """Console U20 (2026-10-19): the runner SIGSTOPped 8 s with a scan in flight and the bridge had answered;
    the first poll after it takes the reply (the simulation stood still too) and the link goes on."""
    link, bridge, now = async_pair(deadline_s=1.0)
    link.enqueue(scan_of(0), {})
    now[0] = 0.02
    assert link.poll() == []
    answer(bridge)
    now[0] = 8.02  # the runner was stopped
    (got,) = link.poll()
    assert got[0].scan_id == 0 and got[2:] == (0.0, 8.02)
    link.enqueue(scan_of(1), {})
    answer(bridge, "processed")
    now[0] = 8.05
    (got,) = link.poll()
    assert got[0].scan_id == 1 and got[0].status == "processed"


def test_a_bridge_that_did_not_answer_while_the_runner_stood_still_fails_the_link():
    """The bridge ran on while the runner stood still: no reply after that is a missed deadline."""
    link, bridge, now = async_pair(deadline_s=1.0)
    link.enqueue(scan_of(0), {})
    now[0] = 0.02
    assert link.poll() == []
    now[0] = 8.02
    with pytest.raises(_sl.SlamLinkFailure, match="no reply within"):
        link.poll()


def test_a_stop_between_an_empty_read_and_the_deadline_check_is_not_charged_to_the_bridge():
    """Codex stall-fix round 1: empty read at 0.02 s, SIGSTOP, the reply arrives, resume at 8.02 s -- the poll
    judges the deadline at its read, so it does not fail; the next poll takes the reply and records the gap."""
    a, b = _socket.socketpair(_socket.AF_UNIX, _socket.SOCK_STREAM)
    _OPEN.extend((a, b))
    ticks = iter([0.0, 0.02, 8.02, 8.03, 8.03, 8.03])  # pump; poll 1 before its read; poll 2 before/after, pump
    link = _sl.AsyncSlamLink(SimpleNamespace(sock=a), clock=lambda: next(ticks))
    link.enqueue(scan_of(0), {})
    assert link.poll() == []  # read empty at 0.02; the stop comes after it
    answer(b)
    (got,) = link.poll()
    assert got[0].scan_id == 0 and got[2:] == (0.0, 8.03)
    assert link.long_poll_gaps == [(0.02, 8.02)]


class _StoppingSocket:
    """A socket whose next send first lets the clock jump (the runner stopped just before it)."""

    def __init__(self, sock, now, jump_s):
        self.sock, self.now, self.jump_s = sock, now, jump_s

    def setblocking(self, flag):
        self.sock.setblocking(flag)

    def recv(self, n):
        return self.sock.recv(n)

    def send(self, data):
        if self.jump_s:
            self.now[0] += self.jump_s
            self.jump_s = 0.0
        return self.sock.send(data)


def test_the_deadline_runs_from_the_last_byte_out_not_from_the_queueing():
    """Codex stall-fix round 2 P2: queued at 0.20 s, stopped before send, sent at 8.20 s -- a poll 10 ms later
    is not past the deadline; the reply's send clock is the real one."""
    a, b = _socket.socketpair(_socket.AF_UNIX, _socket.SOCK_STREAM)
    _OPEN.extend((a, b))
    now = [0.2]
    link = _sl.AsyncSlamLink(SimpleNamespace(sock=_StoppingSocket(a, now, 8.0)), clock=lambda: now[0])
    link.enqueue(scan_of(0), {})
    assert link.in_flight[2] == 8.2
    now[0] = 8.21
    assert link.poll() == []
    answer(b)
    now[0] = 8.25
    (got,) = link.poll()
    assert got[2:] == (8.2, 8.25)


def test_a_scan_not_yet_out_has_no_deadline_and_a_reply_before_it_is_out_fails():
    a, b = _socket.socketpair(_socket.AF_UNIX, _socket.SOCK_STREAM)
    _OPEN.extend((a, b))
    now = [0.0]

    class Full(_StoppingSocket):
        def send(self, data):
            raise BlockingIOError

    link = _sl.AsyncSlamLink(SimpleNamespace(sock=Full(a, now, 0.0)), clock=lambda: now[0])
    link.enqueue(scan_of(0), {})
    now[0] = 5.0
    assert link.poll() == [] and link.in_flight[2] is None  # held by queue_max, not the deadline
    b.sendall(_sl.encode_reply(_sl.Reply(0, 0.0, "skipped", (0.0, 0.0, 0.0), 1, 0.0)))
    with pytest.raises(_sl.SlamLinkFailure, match="sent whole"):
        link.poll()
