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


def _gate(**overrides):
    params = {
        "minimum_travel_distance": 0.5,
        "minimum_travel_heading": 0.5,
        "minimum_time_interval": 0.5,
        "throttle_scans": 1,
        "check_min_dist_and_heading_precisely": False,
    }
    params.update(overrides)
    return slam_link.ScanGate.from_params(params)


def test_the_gate_processes_the_first_scan_and_then_needs_time_and_distance():
    """slam_toolbox 2.8.5 shouldProcessScan with the replay config (plan v3.3)."""
    gate = _gate()
    assert gate.will_process(0.0, (0.0, 0.0, 0.0))
    # Scans 2-4 are dropped for start-up whatever the motion.
    assert not gate.will_process(0.6, (5.0, 0.0, 0.0))
    assert not gate.will_process(0.7, (5.0, 0.0, 0.0))
    assert not gate.will_process(0.8, (5.0, 0.0, 0.0))
    # Time is checked first: 0.4 s after the last processed is too soon.
    gate = _gate()
    gate.will_process(0.0, (0, 0, 0))
    for k in range(1, 4):
        gate.will_process(0.1 * k, (0, 0, 0))
    assert not gate.will_process(0.4, (1.0, 0, 0))
    # Distance squared against 0.8 x 0.5^2 = 0.2, i.e. 0.4472 m; heading ignored.
    assert not gate.will_process(0.5, (0.44, 0, 3.0))
    assert gate.will_process(0.6, (0.45, 0, 0))
    # The reference moves to the processed scan.
    assert not gate.will_process(1.2, (0.80, 0, 0))
    assert gate.will_process(1.3, (0.90, 0, 0))


def test_the_precise_mode_needs_distance_or_heading():
    gate = _gate(check_min_dist_and_heading_precisely=True)
    gate.will_process(0.0, (0, 0, 0))
    for k in range(1, 5):
        gate.will_process(0.1 * k, (0, 0, 0))
    assert gate.will_process(1.0, (0.0, 0.0, 0.6))  # heading alone
    assert not gate.will_process(1.6, (0.3, 0.0, 0.6))


def test_the_gate_throttles_by_scan_count():
    gate = _gate(throttle_scans=2, minimum_time_interval=0.0, minimum_travel_distance=0.0)
    results = [gate.will_process(0.1 * k, (0, 0, 0)) for k in range(8)]
    # First always; then only even counts (scan_ctr 6, 8 -> ids 5, 7) once past start-up.
    assert results == [True, False, False, False, False, True, False, True]


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
