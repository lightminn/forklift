"""The operating unlock U2a (docs/plans/2026-11-26-real-operating-assembly.md 2.2-2.3): the real scan source on a
datagram socket -- the adapter's frames judged in receipt order, a receive gap and its end, a socket error, a synthetic
frame, the per-call caps, SLAM told each generation before an offer and offered only what the layer took -- the IMU's
operating start judged against the watch, and the operating layer file from lidar.yaml."""

import base64
import json
import socket
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.live import real_sources as rs

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tests" / "fixtures" / "replay"))
import make_record as mr  # noqa: E402

MS = 1_000_000


@pytest.fixture(scope="module")
def frames(tmp_path_factory):
    """(receipt ns, frame bytes) of a recorded adapter stream (v2 frames, SDK times, facts the test bound applies to)."""
    path = tmp_path_factory.mktemp("rec") / "obs.jsonl"
    mr.write_record(path, seconds=4.0, imu=False)
    out = []
    for line in path.read_text().splitlines():
        row = json.loads(line)
        if row.get("t") == "scan":
            out.append((int(row["host_recv_ns"]), base64.b64decode(row["frame"])))
    assert len(out) > 20
    return out


class Slam:
    def __init__(self):
        self.log = []

    def generation(self, gen):
        self.log.append(("gen", gen))

    def offer(self, s):
        self.log.append(("offer", s.gen, s.key))


class Clock:
    def __init__(self, ns=0):
        self.ns = ns

    def __call__(self):
        return self.ns


def pair():
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    b.setblocking(False)
    return a, b


def source(sock, clock, slam=None):
    return rs.RealScanSource(sock, mr.lidar_config(), 0, slam=slam, clock=clock)


def test_the_recorded_stream_is_delivered_and_slam_takes_what_the_layer_takes(frames):
    tx, rx = pair()
    clock, slam = Clock(), Slam()
    src = source(rx, clock, slam)
    delivered = 0
    for recv, data in frames:
        tx.send(data)
        clock.ns = recv
        assert src.scan_source_events(recv) == []
        delivered += len(src.scans(recv))
    assert delivered > 0 and src.fatal is None
    offers = [e for e in slam.log if e[0] == "offer"]
    assert len(offers) == delivered  # only what went to the layer, each once
    # every offer's generation was told before it
    told = None
    for e in slam.log:
        if e[0] == "gen":
            told = e[1]
        else:
            assert e[1] == told


def test_a_quiet_gap_is_lost_then_connected(frames):
    tx, rx = pair()
    clock = Clock()
    src = source(rx, clock, Slam())
    recv, data = frames[0]
    tx.send(data)
    clock.ns = recv
    src.scans(recv)
    gen0 = src.feed.generation
    quiet = recv + rs.SCAN_GAP_NS + MS
    clock.ns = quiet
    assert src.scan_source_events(quiet) == [("lost", "lidar")] and src.feed.generation == gen0 + 1
    tx.send(frames[1][1])
    clock.ns = quiet + MS
    assert src.scan_source_events(quiet + MS) == [("connected", "lidar")]


def test_a_gap_ended_on_the_read_that_ends_it_is_still_opened(frames):
    tx, rx = pair()
    clock = Clock()
    src = source(rx, clock)
    tx.send(frames[0][1])
    clock.ns = frames[0][0]
    src.scans(clock.ns)
    tx.send(frames[1][1])
    clock.ns = frames[0][0] + rs.SCAN_GAP_NS + 5 * MS  # no quiet call saw the gap
    assert src.scan_source_events(clock.ns) == [("lost", "lidar"), ("connected", "lidar")]
    assert src.counts["receive_gaps"] == 1


def test_a_socket_error_ends_the_run():
    clock = Clock(10)

    class Broken:
        def recv(self, n):
            raise ConnectionResetError("gone")

        def close(self):
            pass

    slam = Slam()
    src = source(Broken(), clock, slam)
    src.scans(10)
    assert src.fatal == "scan_socket_error" and slam.log and slam.log[-1][0] == "gen"


def test_a_synthetic_frame_ends_the_run(frames, monkeypatch):
    tx, rx = pair()
    clock = Clock()
    src = source(rx, clock)
    real = src.feed.judge

    def judge(data, recv):
        scan, why = real(data, recv)
        if scan is not None:
            scan.meta["synthetic"] = True
        return scan, why

    monkeypatch.setattr(src.feed, "judge", judge)
    for recv, data in frames[:10]:
        tx.send(data)
        clock.ns = recv
        src.scans(recv)
        if src.fatal:
            break
    assert src.fatal == "scan_synthetic" and src.counts["synthetic"] == 1


def test_reads_per_call_are_capped_and_the_rest_is_a_backlog(frames):
    tx, rx = pair()
    clock = Clock(frames[11][0])
    src = source(rx, clock)
    for _recv, data in frames[:12]:
        tx.send(data)
    first = src.scans(clock.ns)
    assert src.counts["read_capped"] == 1 and len(first) <= rs.SCANS_PER_CALL
    clock.ns += MS
    src.scans(clock.ns)
    assert src.counts["receive_gaps"] == 0  # a backlog is never a gap


def test_the_imu_start_is_judged_against_the_watch_with_the_opened_profile():
    class Device:
        def __init__(self, hz):
            self.hz, self.cb = hz, None

        def start(self, cb):
            self.cb = cb

        def profile_info(self):
            return {"actual": {"accel_hz": self.hz}}

    cfg = mr.imu_config()
    src, fits, prof = rs.start_real_imu(cfg, Device(200), gap_max_s=0.05)
    assert fits is True and prof["actual"]["accel_hz"] == 200
    _src, fits, _ = rs.start_real_imu(cfg, Device(5), gap_max_s=0.05)  # a fallback rate the watch cannot hold
    assert fits is False
    with pytest.raises(ValueError):
        rs.start_real_imu(mr.imu_config(measured=False, delay_bound_s=None), Device(200), gap_max_s=0.05)


def test_the_operating_layer_file_takes_the_lidar_mount_period_and_measured_flag(tmp_path):
    import yaml

    from forklift_core.live import real_run as rr

    layer = yaml.safe_load((REPO / "config/real/obstacle_layer_real.yaml").read_text())
    lidar = yaml.safe_load((REPO / "config/real/lidar.yaml").read_text())
    lidar["mount"].update(xyz=[0.3, 0.05, 0.25], yaw_deg=10.0, measured=True)
    lidar["timing"]["scan_period_s"] = 0.12
    p = tmp_path / "lidar.yaml"
    p.write_text(yaml.safe_dump(lidar))
    out = rr.real_layer_cfg(layer, p)
    assert out["bridge"]["scan_period_s"] == pytest.approx(0.12)
    (xy_yaw, _primary, info), = rr._layer_sensors(out).values()  # noqa: SLF001
    assert xy_yaw[0] == pytest.approx(0.3) and xy_yaw[1] == pytest.approx(0.05) and info == {"measured": True}
    # the HIL and replay layers stay unmeasured
    assert rr._layer_sensors(layer)["lidar"][2] == {"measured": False}  # noqa: SLF001


def test_real_sources_are_refused_on_a_hil_run():
    from forklift_core.live import real_run as rr
    from forklift_core.live.real_assembly import AssemblyRefused

    prep = SimpleNamespace(paths=SimpleNamespace(hil=True), replay=None)
    with pytest.raises(AssemblyRefused) as err:
        rr.make_real_sources(prep, None, None, scan_sock=None, slam_socket="/nonexistent")
    assert err.value.reasons == ["real_sources_on_hil"]


def _prep(tmp_path, imu_watch=None):
    from forklift_core.live import real_run as rr

    return SimpleNamespace(paths=rr.RunPaths(s10a=REPO / "config/real/s10a.yaml", hil=False), replay=None,
                           map_id="m" * 64, start_ns=0, cfg=SimpleNamespace(start_rear=(0.0, 0.0, 0.0)),
                           rear_offset_m=0.34, epoch=5, watches=SimpleNamespace(imu=imu_watch))


def test_make_real_sources_on_a_listening_bridge(tmp_path):
    """The REAL client throughout, the SLAM's scans the scan source's own branch (no layer branch), no depth, the
    IMU not built without a device, every part on the closers."""
    from forklift_core.live import real_run as rr
    from forklift_core.mcu_link.build_policy import Client

    path = tmp_path / "slam.sock"
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(str(path))
    srv.listen(1)
    tx, rx = pair()
    try:
        host = SimpleNamespace(state=None)
        src = rr.make_real_sources(_prep(tmp_path), host, SimpleNamespace(), scan_sock=rx, slam_socket=str(path))
        assert src.client is Client.REAL and src.slam.client is Client.REAL
        assert isinstance(src.scans, rs.RealScanSource) and src.scans.slam is src.slam and src.slam_feed is None
        assert src.imu is None and src.imu_qualified is False and src.depth is None
        assert src.slam.hil_gate is None
        for fn in src.closers:
            fn()
        with pytest.raises(OSError):
            rx.recv(1)  # the scan socket closed by the closers
    finally:
        srv.close()
        tx.close()


def test_an_unmeasured_imu_is_not_started_and_not_qualified(tmp_path):
    from forklift_core.live import real_run as rr

    path = tmp_path / "slam.sock"
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(str(path))
    srv.listen(1)
    started = []
    device = SimpleNamespace(start=lambda cb: started.append(cb), stop=lambda: None, profile_info=lambda: {})
    watch = SimpleNamespace(cfg=SimpleNamespace(gap_max_s=0.05))
    tx, rx = pair()
    try:
        src = rr.make_real_sources(_prep(tmp_path, watch), SimpleNamespace(state=None), SimpleNamespace(),
                                   scan_sock=rx, slam_socket=str(path), imu_device=device)
        assert src.imu is None and src.imu_qualified is False and started == []  # config/real/imu.yaml unmeasured
        for fn in src.closers:
            fn()
    finally:
        srv.close()
        tx.close()


def test_a_refused_bridge_refuses_the_sources(tmp_path):
    from forklift_core.live import real_run as rr
    from forklift_core.live.real_assembly import AssemblyRefused

    tx, rx = pair()
    try:
        with pytest.raises(AssemblyRefused) as err:
            rr.make_real_sources(_prep(tmp_path), SimpleNamespace(state=None), SimpleNamespace(), scan_sock=rx,
                                 slam_socket=str(tmp_path / "none.sock"))
        assert err.value.reasons[0].startswith("slam_bridge:")
    finally:
        tx.close()
        rx.close()
