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


def test_the_warmup_scans_are_the_three_after_the_first():
    """Measured on ws1 job 747: slam_toolbox skips scans 1-3 (0-based)."""
    assert [slam_link.is_warmup(k) for k in range(6)] == [False, True, True, True, False, False]


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
