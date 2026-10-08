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


def _rig_frame(with_scan=True, cameras=("front", "rear")):
    rng = np.random.default_rng(0)
    images = tuple(
        slam_link.RigImage(
            name,
            rng.integers(0, 256, (6, 8, 3), dtype=np.uint8),
            rng.integers(0, 9000, (6, 8), dtype=np.uint16),
        )
        for name in cameras
    )
    return slam_link.Frame(
        scan_id=3,
        stamp_s=0.3,
        odom_from_base=(0.5, 0.0, 0.1),
        ranges_m=_scan().ranges_m if with_scan else None,
        angle_min_rad=-np.pi,
        angle_increment_rad=2 * np.pi / 1600,
        range_min_m=0.2,
        range_max_m=12.0,
        images=images,
    )


def test_a_rig_frame_round_trips_bit_for_bit():
    frame = _rig_frame()
    back = slam_link.decode_frame(slam_link.encode_frame(frame)[4:])
    assert (back.scan_id, back.stamp_s, back.odom_from_base) == (3, 0.3, (0.5, 0.0, 0.1))
    np.testing.assert_array_equal(back.ranges_m, frame.ranges_m)
    assert [image.camera for image in back.images] == ["front", "rear"]
    for sent, got in zip(frame.images, back.images, strict=True):
        np.testing.assert_array_equal(got.rgb, sent.rgb)
        np.testing.assert_array_equal(got.depth_mm, sent.depth_mm)
        assert got.depth_mm.dtype == np.uint16


def test_a_frame_without_scan_or_images_says_so():
    back = slam_link.decode_frame(
        slam_link.encode_frame(_rig_frame(with_scan=False, cameras=()))[4:]
    )
    assert back.ranges_m is None and back.images == ()


def test_a_truncated_frame_is_refused():
    body = slam_link.encode_frame(_rig_frame())[4:]
    with pytest.raises(ValueError, match="shorter"):
        slam_link.decode_frame(body[:-1])


def test_rig_image_shapes_are_checked():
    with pytest.raises(ValueError, match="uint16"):
        slam_link.RigImage("front", np.zeros((6, 8, 3), np.uint8), np.zeros((6, 8), np.float32))
