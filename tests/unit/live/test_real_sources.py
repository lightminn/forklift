"""The operating unlock U2a (docs/plans/2026-11-26-real-operating-assembly.md 2.2-2.3): the real scan source on a
datagram socket -- the adapter's frames judged in receipt order, a receive gap and its end, a socket error, a synthetic
frame, the per-call caps, SLAM told each generation before an offer and offered only what the layer took -- the IMU's
operating start judged against the watch, and the operating layer file from lidar.yaml."""

import base64
import dataclasses
import json
import socket
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.live import real_sources as rs
from forklift_core.sensors.scan_feed import decode_frame

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


_OPEN: list = []


def pair():
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    b.setblocking(False)
    _OPEN.extend((a, b))
    return a, b


@pytest.fixture(autouse=True)
def _close_sockets():
    yield
    while _OPEN:
        _OPEN.pop().close()  # every test's pair closed (ws1 runs with -W error: no ResourceWarning)


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
        out = src.scans(recv)
        delivered += len(out)
        for layer in out:
            src.offer_for(layer)  # the run's branch after the layer accepted it
            src.offer_for(layer)  # a second call offers nothing (each scan once)
    assert delivered > 0 and src.fatal is None
    offers = [e for e in slam.log if e[0] == "offer"]
    assert len(offers) == delivered  # only what the layer took, each once
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


def test_a_scan_the_layer_refused_never_reaches_slam(frames):
    tx, rx = pair()
    clock, slam = Clock(), Slam()
    src = source(rx, clock, slam)
    out, i = [], 0
    while not out:  # the feed's first scans establish its reference before any is delivered
        recv, data = frames[i]
        tx.send(data)
        clock.ns = recv
        out = src.scans(recv)
        i += 1
    # the layer refused it: offer_for is never called; the next call forgets it
    tx.send(frames[i][1])
    clock.ns = frames[i][0]
    src.scans(clock.ns)
    src.offer_for(out[0])  # too late: no longer pending
    assert [e for e in slam.log if e[0] == "offer"] == []


def test_a_gap_inside_one_read_drops_the_scans_before_it(frames):
    """Codex U2a round 1 P2-1: one call reads a scan, then a scan past a gap -- the layer sees lost, connected and the
    later scan only; SLAM is never told the older generation after the gap's."""
    class Seq(Clock):
        queue: list = []

        def __call__(self):
            return self.queue.pop(0) if self.queue else self.ns

    tx, rx = pair()
    slam, clock = Slam(), Seq()
    src = source(rx, clock, slam)
    i = 0
    while src.counts["delivered"] == 0:  # warm the feed one frame per call
        tx.send(frames[i][1])
        clock.ns = frames[i][0]
        src.scans(clock.ns)
        i += 1
    k = next(j for j in range(i + 1, len(frames)) if frames[j][0] - frames[i][0] > rs.SCAN_GAP_NS)
    tx.send(frames[i][1])
    tx.send(frames[k][1])
    clock.queue = [frames[i][0], frames[k][0]]
    now = clock.ns = frames[k][0]
    told = len(slam.log)
    assert src.scan_source_events(now) == [("lost", "lidar"), ("connected", "lidar")]
    out = src.scans(now)
    assert src.counts["before_gap_dropped"] == 1 and src.counts["receive_gaps"] == 1 and len(out) <= 1
    gens = [e[1] for e in slam.log[told:] if e[0] == "gen"]
    # the dropped scan's (older) generation is never told: only the gap's and the later scan's
    assert gens and all(g[0] == src.feed.generation for g in gens)


def test_the_last_scans_of_a_call_reach_the_layer_and_slam(frames):
    tx, rx = pair()
    clock, slam = Clock(), Slam()
    src = source(rx, clock, slam)
    i = 0
    while src.counts["delivered"] == 0:  # warm the feed one frame per call
        tx.send(frames[i][1])
        clock.ns = frames[i][0]
        for layer in src.scans(clock.ns):
            src.offer_for(layer)
        i += 1
    before = len([e for e in slam.log if e[0] == "offer"])
    burst = frames[i:i + 6]
    for _recv, data in burst:
        tx.send(data)
    clock.ns = burst[-1][0]
    out = src.scans(clock.ns)
    assert len(out) == rs.SCANS_PER_CALL and src.counts["cut"] == len(burst) - rs.SCANS_PER_CALL
    want = [decode_frame(data)[0]["scan_id"] for _recv, data in burst][-rs.SCANS_PER_CALL:]
    assert [src._pending[id(layer)].scan_id for layer in out] == want  # noqa: SLF001 -- the layer's four
    for layer in out:
        src.offer_for(layer)
    offers = [e for e in slam.log if e[0] == "offer"][before:]
    assert [e[2][2] for e in offers] == want  # SLAM's four: the same, the newest, in order


def test_one_call_delivers_one_generation(frames, monkeypatch):
    """Codex U2a round 2 P2-1: a read holding scans of a producer generation and then of a newer one delivers the newer
    one's only -- SLAM is never told the older generation after the newer, the layer never takes the older."""
    tx, rx = pair()
    clock, slam = Clock(), Slam()
    src = source(rx, clock, slam)
    i = 0
    while src.counts["delivered"] == 0:
        tx.send(frames[i][1])
        clock.ns = frames[i][0]
        for layer in src.scans(clock.ns):
            src.offer_for(layer)
        i += 1
    real = src.feed.judge
    switch_at = frames[i + 2][0]
    switched: set = set()

    def judge(data, recv):
        scan, why = real(data, recv)
        if scan is not None and recv >= switch_at:
            scan = dataclasses.replace(scan, producer_gen=(scan.producer_gen or 0) + 1)  # a new producer
            switched.add(scan.producer_gen)
        return scan, why

    monkeypatch.setattr(src.feed, "judge", judge)
    burst = frames[i:i + 4]
    stamps = iter(r for r, _ in burst)
    clock_ns = clock.ns
    monkeypatch.setattr(src, "clock", lambda: next(stamps, clock_ns))
    for _recv, data in burst:
        tx.send(data)
    told = len(slam.log)
    out = src.scans(burst[-1][0])
    for layer in out:
        src.offer_for(layer)
    gens = [e[1] for e in slam.log[told:] if e[0] == "gen"]
    offers = [e for e in slam.log[told:] if e[0] == "offer"]
    ids = [decode_frame(data)[0]["scan_id"] for _r, data in burst]
    # U3a: the producer change is an evidence boundary -- the older scans of the read are dropped there
    assert src.counts["before_boundary_dropped"] == 2 and len(out) == 2
    assert src.scan_source_events(burst[-1][0]) == [("boundary", "lidar")] and src.counts["boundaries"] == 1
    assert src.last_boundary == "producer_changed"
    assert [e[2][2] for e in offers] == ids[2:]  # exactly the newer generation's two scans
    assert gens and all(g[1] in switched for g in gens) and all(e[1][1] in switched for e in offers)


def test_a_newer_generation_the_layer_cannot_take_still_drops_the_older(frames, monkeypatch):
    """Codex U2a round 3 P3: the newer generation's scan has no layer scan (no bound): nothing of the older goes."""
    from forklift_core.sensors import lidar_cfg

    tx, rx = pair()
    clock, slam = Clock(), Slam()
    src = source(rx, clock, slam)
    i = 0
    while src.counts["delivered"] == 0:
        tx.send(frames[i][1])
        clock.ns = frames[i][0]
        for layer in src.scans(clock.ns):
            src.offer_for(layer)
        i += 1
    real, real_layer = src.feed.judge, lidar_cfg.to_layer_scan
    switch_at = frames[i + 2][0]
    switched: set = set()

    def judge(data, recv):
        scan, why = real(data, recv)
        if scan is not None and recv >= switch_at:
            scan = dataclasses.replace(scan, producer_gen=(scan.producer_gen or 0) + 1)
            switched.add(scan.producer_gen)
        return scan, why

    def to_layer(scan, *a, **kw):
        return (None, "no_bound") if scan.producer_gen in switched else real_layer(scan, *a, **kw)

    monkeypatch.setattr(src.feed, "judge", judge)
    monkeypatch.setattr(lidar_cfg, "to_layer_scan", to_layer)
    burst = frames[i:i + 4]
    stamps = iter(r for r, _ in burst)
    monkeypatch.setattr(src, "clock", lambda: next(stamps, burst[-1][0]))
    for _recv, data in burst:
        tx.send(data)
    told = len(slam.log)
    out = src.scans(burst[-1][0])
    assert out == [] and src.counts["before_boundary_dropped"] == 2 and src.counts["boundaries"] == 3
    assert src.last_boundary.startswith("qualification:") and src.last_refusal == "frame:no_bound"
    assert [e for e in slam.log[told:] if e[0] == "offer"] == []
    # SLAM is still told the newer generation (the older one's pending replies are void -- Codex U2a round 4 P3)
    gens = [e[1] for e in slam.log[told:] if e[0] == "gen"]
    assert gens and gens[-1][1] in switched and all(g[1] in switched for g in gens)
    assert gens[-1][0] == src.feed.generation  # each lost qualification opened a feed generation SLAM was told


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


def test_a_synthetic_frame_the_feed_refuses_still_ends_the_run(frames, monkeypatch):
    """Codex U2a round 1 P3: the feed's own count of synthetic heads, whatever its verdict."""
    tx, rx = pair()
    clock = Clock()
    src = source(rx, clock)

    def judge(data, recv):
        src.feed.synthetic_frames += 1
        return None, "bad_meta"

    monkeypatch.setattr(src.feed, "judge", judge)
    tx.send(frames[0][1])
    clock.ns = frames[0][0]
    assert src.scan_source_events(clock.ns) == [("lost", "lidar")]
    assert src.fatal == "scan_synthetic"


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

        def start(self, cb, on_lost=None):
            self.cb, self.on_lost = cb, on_lost

        def profile_info(self):
            return {"actual": {"accel_hz": self.hz}}

    cfg = mr.imu_config()
    dev = Device(200)
    src, fits, prof = rs.start_real_imu(cfg, dev, gap_max_s=0.05)
    assert fits is True and prof["actual"]["accel_hz"] == 200
    assert dev.on_lost == src.lost  # a removed device is the source's boundary at once
    _src, fits, _ = rs.start_real_imu(cfg, Device(5), gap_max_s=0.05)  # a fallback rate the watch cannot hold
    assert fits is False
    with pytest.raises(ValueError):
        rs.start_real_imu(mr.imu_config(measured=False, delay_bound_s=None), Device(200), gap_max_s=0.05)


@pytest.mark.parametrize("profile", [{}, {"actual": {}}, {"actual": {"accel_hz": None}}, {"actual": {"accel_hz": 0}},
                                     {"actual": {"accel_hz": True}}, {"requested": {"accel_hz": 200}},
                                     {"actual": {"accel_hz": float("inf")}}, {"actual": {"accel_hz": float("nan")}}])
def test_a_profile_that_does_not_say_its_opened_rate_never_fits(profile):
    dev = SimpleNamespace(start=lambda cb, on_lost=None: None, profile_info=lambda: profile)
    _src, fits, _ = rs.start_real_imu(mr.imu_config(), dev, gap_max_s=0.05)
    assert fits is False


def test_the_operating_layer_file_takes_the_lidar_mount_period_and_measured_flag(tmp_path):
    import yaml

    from forklift_core.live import real_run as rr

    layer = yaml.safe_load((REPO / "config/real/obstacle_layer_real.yaml").read_text())
    lidar = yaml.safe_load((REPO / "config/real/lidar.yaml").read_text())
    lidar["mount"].update(xyz=[0.3, 0.05, 0.25], yaw_deg=10.0, measured=True)
    lidar["timing"]["scan_period_s"] = 0.12
    p = tmp_path / "lidar.yaml"
    p.write_text(yaml.safe_dump(lidar))
    from forklift_core.sensors.lidar_cfg import load_lidar_config

    out = rr.real_layer_cfg(layer, load_lidar_config(p))
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


_PREP_LIDAR: list = []


def _prep(tmp_path, imu_watch=None):
    from forklift_core.live import real_run as rr

    return SimpleNamespace(paths=rr.RunPaths(s10a=REPO / "config/real/s10a.yaml", hil=False), replay=None,
                           lidar=mr.lidar_config(),
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
        prep = _prep(tmp_path)
        _PREP_LIDAR[:] = [prep.lidar]
        src = rr.make_real_sources(prep, host, SimpleNamespace(), scan_sock=rx, slam_socket=str(path))
        assert src.client is Client.REAL and src.slam.client is Client.REAL
        assert isinstance(src.scans, rs.RealScanSource) and src.scans.slam is src.slam
        assert src.slam_feed == src.scans.offer_for and src.scans.cfg is _PREP_LIDAR[0]
        assert src.imu is None and src.imu_qualified is False and src.depth is None
        assert src.slam.hil_gate is None
        # the hello carries base_link <- laser from the run's one LiDAR calibration (plan 2.3, U2b)
        assert src.slam.base_from_laser == rr.bridge_laser_xyz_yaw(prep.lidar, prep.rear_offset_m)
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
    device = SimpleNamespace(start=lambda cb, on_lost=None: started.append(cb), stop=lambda: None,
                             profile_info=lambda: {})
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


def test_a_refused_bridge_refuses_the_sources_and_closes_the_scan_socket(tmp_path):
    from forklift_core.live import real_run as rr
    from forklift_core.live.real_assembly import AssemblyRefused

    tx, rx = pair()
    with pytest.raises(AssemblyRefused) as err:
        rr.make_real_sources(_prep(tmp_path), SimpleNamespace(state=None), SimpleNamespace(), scan_sock=rx,
                             slam_socket=str(tmp_path / "none.sock"))
    assert err.value.reasons[0].startswith("slam_bridge:")
    with pytest.raises(OSError):
        rx.recv(1)  # the call owns the scan socket: closed on its own refusal


def _measured_imu(tmp_path, monkeypatch, *, level_measured):
    """config/real/imu.yaml read as measured; the watch file with its level reference measured or not."""
    import yaml

    from forklift_core.live import real_run as rr
    from forklift_core.sensors import imu_source

    cfg = mr.imu_config()
    monkeypatch.setattr(imu_source, "imu_source_config_from", lambda raw: cfg)
    watch = yaml.safe_load((REPO / "config/real/safety_watch.yaml").read_text())
    watch["imu"]["provenance"]["measured"] = level_measured
    wp = tmp_path / "watch.yaml"
    wp.write_text(yaml.safe_dump(watch))
    from forklift_core.safety.real_watch import load_imu_config

    prep = _prep(tmp_path, SimpleNamespace(cfg=load_imu_config(wp)))
    prep.paths = rr.RunPaths(s10a=REPO / "config/real/s10a.yaml", hil=False, watch=wp)
    prep.hashes = {"watch": rr._sha(wp), "imu": rr._sha(prep.paths.imu_config)}  # noqa: SLF001 -- as prep hashed them
    return prep


def test_a_running_watch_other_than_the_judged_one_does_not_qualify(tmp_path, monkeypatch):
    """Codex U2a round 3 P2: the watch built from another read (A -> B -> A) is not the judged one."""
    import dataclasses as dc

    from forklift_core.live import real_run as rr

    prep = _measured_imu(tmp_path, monkeypatch, level_measured=True)
    prep.watches.imu = SimpleNamespace(cfg=dc.replace(prep.watches.imu.cfg, gap_max_s=0.09))
    started = []
    device = SimpleNamespace(start=lambda cb, on_lost=None: started.append(cb), stop=lambda: None,
                             profile_info=lambda: {"actual": {"accel_hz": 200}})
    _tx, rx = pair()
    src = rr.make_real_sources(prep, SimpleNamespace(state=None), SimpleNamespace(), scan_sock=rx,
                               slam_socket=_bridge(tmp_path), imu_device=device)
    assert src.imu is None and src.imu_qualified is False and started == []
    for fn in src.closers:
        fn()


def test_an_imu_judged_on_other_bytes_than_prep_hashed_is_not_started(tmp_path, monkeypatch):
    from forklift_core.live import real_run as rr

    for name in ("watch", "imu"):
        prep = _measured_imu(tmp_path, monkeypatch, level_measured=True)
        prep.hashes[name] = "0" * 64  # the file changed since prep read it
        started = []
        device = SimpleNamespace(start=lambda cb, on_lost=None: started.append(cb), stop=lambda: None,
                                 profile_info=lambda: {"actual": {"accel_hz": 200}})
        _tx, rx = pair()
        src = rr.make_real_sources(prep, SimpleNamespace(state=None), SimpleNamespace(), scan_sock=rx,
                                   slam_socket=_bridge(tmp_path / name), imu_device=device)
        assert src.imu is None and src.imu_qualified is False and started == [], name
        for fn in src.closers:
            fn()


def _bridge(tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    path = tmp_path / "slam.sock"
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(str(path))
    srv.listen(1)
    _OPEN.append(srv)
    return str(path)


def test_an_imu_with_its_level_unmeasured_is_not_started(tmp_path, monkeypatch):
    from forklift_core.live import real_run as rr

    prep = _measured_imu(tmp_path, monkeypatch, level_measured=False)
    started = []
    device = SimpleNamespace(start=lambda cb, on_lost=None: started.append(cb), stop=lambda: None,
                             profile_info=lambda: {"actual": {"accel_hz": 200}})
    _tx, rx = pair()
    src = rr.make_real_sources(prep, SimpleNamespace(state=None), SimpleNamespace(), scan_sock=rx,
                               slam_socket=_bridge(tmp_path), imu_device=device)
    assert src.imu is None and src.imu_qualified is False and started == []
    for fn in src.closers:
        fn()


def test_a_qualified_imu_is_started_and_stopped_by_the_closers(tmp_path, monkeypatch):
    from forklift_core.live import real_run as rr

    prep = _measured_imu(tmp_path, monkeypatch, level_measured=True)
    log = []
    device = SimpleNamespace(start=lambda cb, on_lost=None: log.append("start"), stop=lambda: log.append("stop"),
                             profile_info=lambda: {"actual": {"accel_hz": 200}})
    _tx, rx = pair()
    src = rr.make_real_sources(prep, SimpleNamespace(state=None), SimpleNamespace(), scan_sock=rx,
                               slam_socket=_bridge(tmp_path), imu_device=device)
    assert src.imu is not None and src.imu_qualified is True and log == ["start"]
    for fn in reversed(src.closers):
        fn()
    assert log == ["start", "stop"]


def test_an_unfit_imu_whose_stop_fails_keeps_its_stop_on_the_closers(tmp_path, monkeypatch):
    from forklift_core.live import real_run as rr

    prep = _measured_imu(tmp_path, monkeypatch, level_measured=True)
    log, fail = [], [True]

    def stop():
        log.append("stop")
        if fail[0]:
            raise RuntimeError("busy")

    device = SimpleNamespace(start=lambda cb, on_lost=None: log.append("start"), stop=stop,
                             profile_info=lambda: {"actual": {"accel_hz": 5}})
    _tx, rx = pair()
    src = rr.make_real_sources(prep, SimpleNamespace(state=None), SimpleNamespace(), scan_sock=rx,
                               slam_socket=_bridge(tmp_path), imu_device=device)
    assert src.imu is None and src.imu_qualified is False and log == ["start", "stop"]
    fail[0] = False
    for fn in reversed(src.closers):
        fn()
    assert log == ["start", "stop", "stop"]  # tried again by the run's closers, once it can
    for fn in reversed(src.closers):
        fn()
    assert log.count("stop") == 2  # and not again once done


# --- U3a: a lost qualification within one producer generation --------------------------------------------------------
def test_a_lost_qualification_in_one_producer_generation_is_a_boundary(frames, monkeypatch):
    """Codex U3 split round 2 P1: good A, unqualified B, good C of the same producer -- B is a boundary: A of the same
    read is dropped, the feed opens a generation SLAM is told before C, and the loop gets ("boundary", sensor)."""
    from forklift_core.sensors import lidar_cfg

    tx, rx = pair()
    clock, slam = Clock(), Slam()
    src = source(rx, clock, slam)
    i = 0
    while src.counts["delivered"] == 0:
        tx.send(frames[i][1])
        clock.ns = frames[i][0]
        for layer in src.scans(clock.ns):
            src.offer_for(layer)
        i += 1
    bad_id = decode_frame(frames[i + 1][1])[0]["scan_id"]
    real_layer = lidar_cfg.to_layer_scan

    def to_layer(scan, *a, **kw):
        return (None, "no_bound") if scan.scan_id == bad_id else real_layer(scan, *a, **kw)

    monkeypatch.setattr(lidar_cfg, "to_layer_scan", to_layer)
    burst = frames[i:i + 3]
    stamps = iter(r for r, _ in burst)
    monkeypatch.setattr(src, "clock", lambda: next(stamps, burst[-1][0]))
    for _recv, data in burst:
        tx.send(data)
    gen_before = src.feed.generation
    told = len(slam.log)
    assert src.scan_source_events(burst[-1][0]) == [("boundary", "lidar")]
    out = src.scans(burst[-1][0])
    for layer in out:
        src.offer_for(layer)
    assert src.counts["before_boundary_dropped"] == 1 and len(out) == 1  # A dropped, C delivered
    assert src.feed.generation == gen_before + 1 and src.last_refusal == "frame:no_bound"
    offers = [e for e in slam.log[told:] if e[0] == "offer"]
    gens = [e[1] for e in slam.log[told:] if e[0] == "gen"]
    assert [o[1][0] for o in offers] == [src.feed.generation] and gens[0][0] == src.feed.generation
