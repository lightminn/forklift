"""PR 7 7c-1c (docs/design/2026-11-25-real-sources-pr7.md 6.1, 8): the observe run's ``--slam`` on the MCU model behind
a pty, a protocol-following fake bridge and the real scan adapter's frames -- the odometry history built from the
observe run's own STATEs with the loop's functions, the session start to ``ready``, every scan sent and every reply
recorded with its match quality, all diagnostic (no measured first-beam bound before 7a-3); the board still sees
DISARMs only."""

import socket
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.live.observe_slam import ObserveSlam
from forklift_core.live.real_run import RUN_STATE_DELAY_S, chassis_reference
from forklift_core.live.real_slam_source import RealSlamLink, RealSlamSource
from forklift_core.live.real_teleop import EncoderSpeedParams
from forklift_core.localization.slam_map import Grid
from forklift_core.localization.slam_real_scan import base_from_laser
from forklift_core.plant.real_vehicle import RealOdometry, chassis_model_path, load_real_vehicle
from forklift_core.sensors import lidar_cfg
from forklift_core.sensors.scan_feed import bind_socket

from fake_slam_bridge import FakeBridge
from test_real_observe import REPO, _rows, model, ro  # noqa: F401 -- the model fixture

VEHICLE = REPO / "config" / "real" / "vehicle.yaml"


def _grid():
    """A survey with a wall all round a 4 m square at 0.05 m (the scans below are 2 m rays from the middle)."""
    data = np.zeros((100, 100), np.int8)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    return Grid(data, (-2.5, -2.5, 0.0), 0.05)


def _slam(tmp_path, lid, start_ns):
    bridge = FakeBridge(tmp_path / "slam.sock")
    bridge.start()
    vehicle = load_real_vehicle(VEHICLE)
    _tip, rear, _c = chassis_reference(chassis_model_path(VEHICLE).parent / "forklift.urdf")
    kappa = max(abs(k) for k in vehicle.steering.curvatures_inv_m)
    params = EncoderSpeedParams(radius_m=vehicle.wheel_radius_m, counts_per_rev=vehicle.counts_per_wheel_rev,
                                track_m=vehicle.track_m, kappa_max=kappa, axle_accel_mps2=0.5, max_age_s=0.1,
                                dt_min_s=0.02, dt_max_s=0.2)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(bridge.path)
    src = RealSlamSource(RealSlamLink(sock), map_id="m" * 64, start_ns=start_ns, start_pose=(0.0, 0.0, 0.0),
                         odom_history=None, odom_base=None, odom_gen=None, boot=None, rear_offset_m=rear)
    rows = []
    slam = ObserveSlam(src, rows.append, odometry=RealOdometry(vehicle, max_gap_s=0.2), params=params,
                       rear_offset_m=rear, start_ns=start_ns, grid=_grid(), base_laser=base_from_laser(lid.mount, rear),
                       lidar=lid, state_delay_s=RUN_STATE_DELAY_S, v_scan_mps=0.225, kappa_max=kappa)
    return bridge, sock, slam, rows


def _sender(tmp_path):
    sys.path.insert(0, str(REPO / "ros2" / "src" / "forklift_ros" / "forklift_ros"))
    import real_scan_adapter as ra

    path = tmp_path / "scan.sock"
    return ra, bind_socket(path), ra.DatagramSender(str(path))


def _scan_msg(ra, recv, pair, n=16):
    return SimpleNamespace(ranges=list(np.full(n, 2.0)), intensities=[], angle_min=-np.pi,
                           angle_increment=2 * np.pi / n, time_increment=0.0001, scan_time=0.1, range_min=0.1,
                           range_max=10.0,
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=(recv + pair) // 10**9,
                                                                        nanosec=(recv + pair) % 10**9),
                                                  frame_id="laser"))


def _observe(model, tmp_path, seconds, *, lose_bridge_at=None):
    """The observe run with --slam for ``seconds`` (the bridge stopped at ``lose_bridge_at`` s): its rows, the bridge,
    the SLAM path and whether the end DISARM was confirmed."""
    lid = lidar_cfg.load_lidar_config()
    ra, scan_sock, sender = _sender(tmp_path)
    openings = ro.Openings()
    from real_mcu_link import ReconnectingLink
    import os

    link = ReconnectingLink("/dev/model", opener=lambda path, baud, **kw: os.dup(model), exclusive=False,
                            link_factory=openings, product_reader=lambda fd: None)
    import io

    rec = ro.Record(io.StringIO())
    start_ns = time.monotonic_ns()
    bridge, sock, slam, slam_rows = _slam(tmp_path, lid, start_ns)
    obs = ro.Observer(link, rec, openings=openings, scan_sock=scan_sock, lidar=lid, slam=slam)
    try:
        link.open_first(obs.host, time.monotonic_ns())
        t0 = time.monotonic()
        end = t0 + seconds
        k = 0
        next_scan = time.monotonic()
        lost = False
        while time.monotonic() < end:
            if lose_bridge_at is not None and not lost and time.monotonic() - t0 > lose_bridge_at:
                lost = True
                bridge.closing()
                bridge.stop()
            if time.monotonic() >= next_scan:
                k += 1
                recv, pair = ra.clock_pair()
                sender.send(ra.make_frame(_scan_msg(ra, recv, pair), scan_id=k, recv_mono_ns=recv,
                                          sys_minus_mono_ns=pair, gen=5, first_beam_mono_ns=recv - 30_000_000))
                next_scan += 0.1
            obs.cycle()
            time.sleep(0.01)
        stopped = obs.finish()
    finally:
        bridge.closing()
        sender.close()
        scan_sock.close()
        sock.close()
        bridge.stop()
    return _rows(rec), bridge, slam, stopped, lid


def test_the_observe_run_records_the_real_slam_source_diagnostic(model, tmp_path):
    rows, bridge, slam, stopped, lid = _observe(model, tmp_path, 6.0)
    assert stopped
    assert not lid.timing_measured  # before 7a-3: diagnostic
    assert bridge.error is None and slam.fatal is None, (bridge.error, slam.fatal)
    kinds = [g[1] for g in bridge.got]
    assert kinds[0] == "Hello" and kinds.count("RealScan") > 7
    assert ("ready", 1) in [(kk, i) for _, kk, i in bridge.sent]
    ev = [r for r in rows if r["t"] == "slam_event"]
    sent = [e for e in ev if e["event"] == "scan_sent"]
    replies = [e for e in ev if e["event"] == "reply"]
    assert sent and all(e["measured"] is False for e in sent)  # diagnostic: no first-beam bound yet
    assert all("odom_from_base" in e for e in sent)
    assert replies and all(not e["passed"] for e in replies)
    assert {e["why"] for e in replies} <= {"start_check", "diagnostic"} and "diagnostic" in {e["why"] for e in replies}
    assert all("epoch" in e and "attempt" in e for e in replies)  # on every reply row (round 1 P2-4)
    # each reply's match quality: the still truck's laser at the start pose through the bridge's correction -- the
    # same fraction computed here independently (a non-zero mount offset: lidar.yaml's rear-axle mount and the
    # chassis model's rear axle offset)
    from forklift_core.localization import slam_map
    from forklift_core.localization.slam_pose import compose

    bx, by, _bz, byaw = slam.base_laser
    assert abs(bx) > 0.0  # the mount is not at base_link: the transform is exercised
    n = 16
    expect = slam_map.endpoint_fraction(_grid(), compose((0.0, 0.0, 0.0), (bx, by, byaw)), np.full(n, 2.0),
                                        -np.pi, 2 * np.pi / n, 0.1, 10.0)
    fr = [e["endpoint_fraction"] for e in replies]
    assert fr and all(f is not None and abs(f - expect) < 1e-6 for f in fr), (fr[:3], expect)
    assert any(e["event"] == "state" and e["kind"] == "ready" and e["id"] == 1 for e in ev)
    assert slam.gen == 0 and len(slam.history) > 100  # the odometry history from the observe run's STATEs
    written = [r for r in rows if r["t"] == "written"]
    assert written and all(r["kind"] == "DISARM" for r in written)  # still nothing that moves the truck


def test_a_scan_without_the_sdk_first_sample_is_recorded_not_sent(tmp_path):
    lid = lidar_cfg.load_lidar_config()
    start_ns = time.monotonic_ns()
    bridge, sock, slam, rows = _slam(tmp_path, lid, start_ns)
    try:
        scan = SimpleNamespace(meta={"range_min": 0.1, "range_max": 10.0}, feed_gen=0, producer_gen=5, scan_id=3,
                               angles=np.linspace(-np.pi, np.pi, 8, endpoint=False), ranges=np.full(8, 2.0))
        slam.on_scan(scan)
        assert rows[-1] == {"t": "slam_scan", "scan_id": 3, "feed_gen": 0, "why": "no_acquisition_time"}
    finally:
        bridge.closing()
        sock.close()
        bridge.stop()


def test_the_cli_refuses_slam_without_a_bundle_or_a_scan_socket(tmp_path, capsys):
    base = ["--device", "/dev/null-forklift", "--seconds", "1", "--slam", str(tmp_path / "none.sock")]
    code = ro.main(base + ["--out", str(tmp_path / "a.jsonl")])
    assert code == 2 and "--slam needs --slam-bundle" in capsys.readouterr().out
    code = ro.main(base + ["--out", str(tmp_path / "b.jsonl"), "--slam-bundle", str(tmp_path)])
    assert code == 2 and "--slam needs --scan-socket" in capsys.readouterr().out
    assert not (tmp_path / "a.jsonl").exists() and not (tmp_path / "b.jsonl").exists()


def test_a_bridge_that_refuses_the_connection_refuses_the_start(tmp_path):
    parts = {"bundle": SimpleNamespace(map_id="m" * 64, grid=_grid()), "start_pose": (0.0, 0.0, 0.0),
             "rear_offset_m": 0.3}
    args = SimpleNamespace(slam=str(tmp_path / "none.sock"), slam_bundle=tmp_path)
    with pytest.raises(OSError):
        ro._slam_start(args, parts, None, lidar_cfg.load_lidar_config())


@pytest.mark.parametrize("why", ["eof"])
def test_a_bridge_ending_records_the_fault_and_the_run_goes_on(tmp_path, why):
    lid = lidar_cfg.load_lidar_config()
    bridge, sock, slam, rows = _slam(tmp_path, lid, time.monotonic_ns())
    try:
        slam.cycle(time.monotonic_ns())
        time.sleep(0.1)
        bridge.closing()
        bridge.stop()  # the bridge goes away
        for _ in range(20):
            slam.cycle(time.monotonic_ns())
            if slam.fatal:
                break
            time.sleep(0.02)
        assert slam.fatal and any(r["t"] == "slam_fatal" for r in rows)
        slam.cycle(time.monotonic_ns())  # no more calls into the source after its end
    finally:
        sock.close()



def test_a_bridge_lost_mid_run_ends_the_slam_path_and_the_board_still_stops(model, tmp_path):
    """Round 1 P3-2: the observe run goes on recording the board, its DISARM refresh and its end DISARM confirmed."""
    rows, bridge, slam, stopped, _lid = _observe(model, tmp_path, 5.0, lose_bridge_at=2.5)
    assert stopped and slam.fatal is not None and any(r["t"] == "slam_fatal" for r in rows)
    fatal_at = next(i for i, r in enumerate(rows) if r["t"] == "slam_fatal")
    later = [r for r in rows[fatal_at:] if r["t"] == "written" and r["kind"] == "DISARM"]
    assert later  # the 1 s refresh and the end DISARM after the SLAM path ended


def test_an_input_break_cuts_what_waits_and_drops_the_reply_in_flight(tmp_path):
    lid = lidar_cfg.load_lidar_config()
    bridge, sock, slam, rows = _slam(tmp_path, lid, time.monotonic_ns())
    try:
        slam.src._waiting = {"key": ("w",), "stamp_ns": 1}  # noqa: SLF001 -- a scan waiting
        slam.src._inflight = {"old_gen": False}  # noqa: SLF001
        slam._break("new_connection")  # noqa: SLF001
        assert slam.src._waiting is None and slam.src._inflight["old_gen"] is True  # noqa: SLF001
        assert slam.gen == 1 and not slam.history and slam.src.counts["slam_input_break"] == 1
    finally:
        bridge.closing()
        sock.close()
        bridge.stop()
