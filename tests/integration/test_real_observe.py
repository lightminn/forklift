"""PR 7 design 1.1 (7a-1): the observe run on the MCU model behind a pty -- DISARMs only (a session pair, the 1 s
refresh, a confirmed end DISARM), anything else queued refused before its first byte, the model never armed, the
record complete; a held device refused with nothing written; scans recorded and kept diagnostic without the SDK
first-sample time."""

import io
import json
import os
import signal
import subprocess
import sys
import time
import tty
from pathlib import Path

import numpy as np
import pytest

from forklift_core.live.executor_lock import lock_fd
from forklift_core.mcu_link.messages import Arm, Mode
from forklift_core.sensors import lidar_cfg

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import real_observe as ro  # noqa: E402
from real_mcu_link import ReconnectingLink  # noqa: E402
from test_bringup_tool import mcu_model  # noqa: E402,F401 -- the module-scoped model build


@pytest.fixture
def model(mcu_model):  # noqa: F811
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    proc = subprocess.Popen([str(mcu_model), "--link", os.ttyname(slave)])
    yield master
    proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=5)
    os.close(master)
    os.close(slave)


def _observer(master, *, scan_sock=None):
    openings = ro.Openings()
    link = ReconnectingLink("/dev/model", opener=lambda path, baud, **kw: os.dup(master), exclusive=False,
                            link_factory=openings, product_reader=lambda fd: None)
    rec = ro.Record(io.StringIO())
    lid = lidar_cfg.load_lidar_config()
    return ro.Observer(link, rec, openings=openings, scan_sock=scan_sock, lidar=lid), rec


def _rows(rec):
    return [json.loads(x) for x in rec.f.getvalue().splitlines()]


def _run(obs, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        obs.cycle()
        time.sleep(ro.CYCLE_S)


def test_the_observe_run_writes_only_disarms_and_confirms_its_end(model):
    obs, rec = _observer(model)
    _run(obs, 1.4)
    obs.host._enqueue("ARM", Arm(obs.host.state.boot_id, obs.host.state.mcu_time_us), time.monotonic_ns())  # noqa: SLF001
    _run(obs, 0.2)
    assert obs.finish()
    rows = _rows(rec)
    written = [r for r in rows if r["t"] == "written"]
    assert written and all(r["kind"] == "DISARM" for r in written)
    assert len(written) >= 3  # the session pair and at least one 1 s refresh
    assert [r["kind"] for r in rows if r["t"] == "refused"] == ["ARM"]
    states = [r for r in rows if r["t"] == "state"]
    assert states and all(r["mode"] != Mode.ARMED.name for r in states)
    assert all(r["conn"] == 1 for r in states)


def test_the_cli_refuses_a_held_device_and_writes_nothing(tmp_path, monkeypatch, capsys):
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    other = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    lock_fd(other, path)
    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)
    try:
        out = tmp_path / "obs.jsonl"
        assert ro.main(["--device", path, "--out", str(out), "--seconds", "0.3"]) == 2
        assert json.loads(capsys.readouterr().out)["result"] == "refused"
        try:
            assert os.read(master, 4096) == b""
        except BlockingIOError:
            pass
        assert json.loads(out.read_text().splitlines()[-1])["result"] == "refused"
    finally:
        os.close(other)
        os.close(slave)
        os.close(master)


def test_the_cli_refuses_an_existing_record(tmp_path, capsys, monkeypatch):
    out = tmp_path / "obs.jsonl"
    out.write_text("x\n")
    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)
    assert ro.main(["--device", "/dev/null", "--out", str(out), "--seconds", "1"]) == 2
    assert "exists" in json.loads(capsys.readouterr().out)["why"]


def test_the_cli_records_a_complete_run_on_the_model(model, tmp_path, monkeypatch, capsys):
    path = None
    # the CLI opens a path: give it the model's master through an opener (the bench check is the rehearsal tool's)
    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)
    monkeypatch.setattr(ro, "open_raw", lambda p, baud=None, **kw: os.dup(model))
    out = tmp_path / "obs.jsonl"
    code = ro.main(["--device", "/dev/model", "--out", str(out), "--seconds", "1.2"])
    res = json.loads(capsys.readouterr().out)
    assert code == 0 and res["complete"] and res["stopped_confirmed"], res
    rows = [json.loads(x) for x in out.read_text().splitlines()]
    assert rows[0]["t"] == "header" and rows[0]["schema"] == ro.SCHEMA and rows[-1]["t"] == "end"
    assert rows[-1]["complete"] and rows[-1]["record_errors"] == 0
    assert all(r["kind"] == "DISARM" for r in rows if r["t"] == "written")
    assert path is None


def test_scans_are_recorded_and_stay_diagnostic_without_the_first_sample_time(model, tmp_path):
    sys.path.insert(0, str(REPO / "ros2" / "src" / "forklift_ros" / "forklift_ros"))
    import real_scan_adapter as ra
    from types import SimpleNamespace

    from forklift_core.sensors.scan_feed import bind_socket

    sock_path = tmp_path / "scan.sock"
    sock = bind_socket(sock_path)
    sender = ra.DatagramSender(str(sock_path))
    obs, rec = _observer(model, scan_sock=sock)
    try:
        for k in range(1, 15):
            recv, pair = ra.clock_pair()
            msg = SimpleNamespace(ranges=list(np.full(8, 2.0)), intensities=[], angle_min=-3.14,
                                  angle_increment=0.785, time_increment=0.0125, scan_time=0.1, range_min=0.1,
                                  range_max=10.0,
                                  header=SimpleNamespace(stamp=SimpleNamespace(sec=(recv + pair) // 10**9,
                                                                               nanosec=(recv + pair) % 10**9),
                                                         frame_id="laser"))
            sender.send(ra.make_frame(msg, scan_id=k, recv_mono_ns=recv, sys_minus_mono_ns=pair, gen=5))
            obs.cycle()
            time.sleep(0.01)
        scans = [r for r in _rows(rec) if r["t"] == "scan"]
        assert len(scans) == 14
        accepted = [r for r in scans if r["verdict"] is None]
        assert accepted and all(r["layer"] == "no_acquisition_time" and r["producer_gen"] == 5 for r in accepted)
        assert obs.counts["scans_layer_ok"] == 0
    finally:
        sender.close()
        sock.close()


def test_the_session_pair_verifies_the_delay_bound(model):
    """Codex 7a-1 P2-2: the second DISARM goes a random 1024-64512 seq step after the first is acknowledged, so the
    STATE delay bound is built (DelayBound verified, STATE rows carry a bound)."""
    obs, rec = _observer(model)
    _run(obs, 1.0)
    rows = _rows(rec)
    ready = [r for r in rows if r["t"] == "session_ready"]
    assert ready and ready[0]["verified"] is True
    seqs = [r["seq"] for r in rows if r["t"] == "written"]
    assert 1024 <= (seqs[1] - seqs[0]) % 65536 <= 64512
    assert any(r.get("delay_bound_s") is not None for r in rows if r["t"] == "state")


def test_an_acknowledgement_needs_a_step_to_the_seq_after_it_was_written():
    """Codex 7a-1 P2-1: a STATE already naming the seq before it was written (an earlier session's, or the initial
    value) is not its acknowledgement."""
    from forklift_core.mcu_link.messages import FaultFlag, McuState, RejectReason, StateFlag

    class _Link:
        link = None
        ever_connected = True

    openings = ro.Openings()
    openings.count = 1
    obs = ro.Observer(_Link(), ro.Record(io.StringIO()), openings=openings, lidar=lidar_cfg.load_lidar_config())

    def st(t, seq):
        return McuState(mcu_time_us=t, boot_id=3, arm_gen=0, last_rx_seq=seq, mode=Mode.DISARMED,
                        reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0,
                        wheel_count_right=0, wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.0,
                        lift_m=0.0, current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0), driver_status=(0, 0, 0, 0),
                        rx_errors=0, cycle_overruns=0, flags=StateFlag(0))

    from types import SimpleNamespace

    obs._on_state(st(1, 77), 100)  # noqa: SLF001 -- the board already names 77
    obs._log("written", 200, b"", {"kind": "DISARM", "seq": 77})  # noqa: SLF001
    obs._done(77, 210)  # noqa: SLF001 -- its last byte
    obs._on_state(st(2, 77), 300)  # noqa: SLF001 -- still 77: no step
    obs.host = SimpleNamespace(state=st(2, 77))  # acknowledged() reads only the host's latest STATE
    assert not obs.acknowledged(77)
    obs._on_state(st(3, 78), 400)  # noqa: SLF001
    obs._log("written", 500, b"", {"kind": "DISARM", "seq": 79})  # noqa: SLF001
    obs._on_state(st(4, 79), 505)  # noqa: SLF001 -- a step to 79 after its first byte but before its last
    obs._done(79, 510)  # noqa: SLF001
    obs.host = SimpleNamespace(state=st(4, 79))
    assert not obs.acknowledged(79)  # Codex 7a-1 round 2 P2-1: not before the last byte
    obs._on_state(st(5, 80), 550)  # noqa: SLF001
    obs._log("written", 560, b"", {"kind": "DISARM", "seq": 81})  # noqa: SLF001
    obs._done(81, 570)  # noqa: SLF001
    obs._on_state(st(6, 81), 600)  # noqa: SLF001 -- a step to 81 after its last byte
    obs.host = SimpleNamespace(state=st(6, 81))
    assert obs.acknowledged(81)


class _QuietLink:
    """A link that is connected and receives nothing (only the observer's own bookkeeping runs)."""

    link = object()
    ever_connected = True

    def service(self, host, now_ns):
        return []


def test_an_unanswered_session_stage_starts_the_pair_again():
    """Codex 7a-1 round 2 P2-2: a stage not answered within SESSION_STAGE_S restarts with fresh seqs."""
    obs = ro.Observer(_QuietLink(), ro.Record(io.StringIO()), lidar=lidar_cfg.load_lidar_config())
    t0 = 10**12
    obs._disarms(t0)  # noqa: SLF001
    obs._disarms(t0 + int(0.6e9))  # noqa: SLF001
    rows = [json.loads(x) for x in obs.rec.f.getvalue().splitlines()]
    assert [r["stage"] for r in rows if r["t"] == "session_retry"] == ["first"]
    assert obs.counts["sessions"] == 2


def test_a_scan_gap_is_recorded_on_the_tick_it_happens():
    """Codex 7a-1 round 2 P2-4: no scan for longer than the timeout is seen without waiting for the next scan."""
    class _NoScans:
        def recv(self, n):
            raise BlockingIOError

    clock = {"t": 10**12}
    obs = ro.Observer(_QuietLink(), ro.Record(io.StringIO()), scan_sock=_NoScans(),
                      lidar=lidar_cfg.load_lidar_config(), clock=lambda: clock["t"])
    obs._last_scan_ns = clock["t"]  # noqa: SLF001 -- a scan just now
    clock["t"] += int(0.5e9)
    obs.cycle(refresh=False)
    obs.cycle(refresh=False)
    rows = [json.loads(x) for x in obs.rec.f.getvalue().splitlines()]
    gaps = [r for r in rows if r["t"] == "scan_event" and r["kind"] == "receive_gap"]
    assert len(gaps) == 1 and obs.feed.generation == 1


def test_a_complete_record_has_its_ok_marker_and_a_changed_one_does_not(model, tmp_path, monkeypatch, capsys):
    """Codex 7a-1 round 2 P2-5: a replay takes a record only with a .ok naming its sha256."""
    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)
    monkeypatch.setattr(ro, "open_raw", lambda p, baud=None, **kw: os.dup(model))
    out = tmp_path / "obs.jsonl"
    assert ro.main(["--device", "/dev/model", "--out", str(out), "--seconds", "1.0"]) == 0
    assert (tmp_path / "obs.jsonl.ok").exists() and ro.record_complete(out)
    with open(out, "a", encoding="utf-8") as f:
        f.write('{"t": "state"}\n')
    assert not ro.record_complete(out)


def test_a_gap_that_ends_on_the_read_itself_still_opens_a_generation():
    """Codex 7a-1 round 3 P2-1: no quiet tick saw the gap; the returning scan's read opens it before its receipt
    counts as the newest."""
    class _OneFrame:
        def __init__(self):
            self.sent = False

        def recv(self, n):
            if self.sent:
                raise BlockingIOError
            self.sent = True
            return b"not a frame"

    clock = {"t": 10**12}
    obs = ro.Observer(_QuietLink(), ro.Record(io.StringIO()), scan_sock=_OneFrame(),
                      lidar=lidar_cfg.load_lidar_config(), clock=lambda: clock["t"])
    obs._last_scan_ns = clock["t"]  # noqa: SLF001
    clock["t"] += int(0.36e9)
    obs.cycle(refresh=False)
    rows = [json.loads(x) for x in obs.rec.f.getvalue().splitlines()]
    kinds = [r["kind"] for r in rows if r["t"] == "scan_event"]
    assert kinds == ["receive_gap", "receive_resumed"] and obs.feed.generation == 1


def test_the_ok_marker_never_writes_through_an_existing_tmp_path(tmp_path):
    """Codex 7a-1 round 3 P2-3: a fresh file of its own, never an existing path or a link's target."""
    rec = tmp_path / "obs.jsonl"
    rec.write_text('{"t": "end", "complete": true}\n')
    victim = tmp_path / "victim"
    victim.write_text("keep\n")
    (tmp_path / "obs.jsonl.ok.tmp").symlink_to(victim)
    ro.write_ok_marker(rec)
    assert victim.read_text() == "keep\n" and ro.record_complete(rec)


def test_the_last_line_reader_streams():
    import tempfile as tf

    with tf.NamedTemporaryFile("wb", delete=False) as f:
        f.write(b'{"t": "x"}\n' * 20000 + b'{"t": "end", "complete": true}\n')
        name = f.name
    try:
        assert ro._last_line(Path(name)) == b'{"t": "end", "complete": true}'  # noqa: SLF001
    finally:
        os.unlink(name)


def test_a_short_ok_write_never_publishes_a_marker(tmp_path, monkeypatch):
    """Codex 7a-1 round 4 P2-2."""
    rec = tmp_path / "obs.jsonl"
    rec.write_text('{"t": "end", "complete": true}\n')
    real_write = os.write
    monkeypatch.setattr(ro.os, "write", lambda fd, data: real_write(fd, data[:9]) if len(data) > 9 else 0)
    with pytest.raises(OSError):
        ro.write_ok_marker(rec)
    assert not (tmp_path / "obs.jsonl.ok").exists()
    assert [p.name for p in tmp_path.iterdir()] == ["obs.jsonl"]  # no temporary left behind


# --- 7b: the D435i accelerometer (PR 7 design 5) -------------------------------------------------------------------
class FakeImuDevice:
    """The motion module's shape (``tools/imu_record.RealSenseSource``): ``start(on_sample)`` calls back from its own
    thread -- gyro and accel, 200 Hz, a device time that follows the host clock -- until ``stop``."""

    def __init__(self, *, fail_start=False, silent=False, accel_hz=200, fail_profile=False, fail_stop=False,
                 burst_on_stop=0):
        self.fail_start, self.silent = fail_start, silent
        self.accel_hz, self.fail_profile, self.fail_stop = accel_hz, fail_profile, fail_stop
        self.burst_on_stop = burst_on_stop
        self.on_sample = None
        self.accel_sent = 0
        self.started = self.stopped = False
        self._halt = None
        self._thread = None

    def start(self, on_sample):
        import threading

        self.started = True
        self.on_sample = on_sample
        if self.fail_start:
            raise RuntimeError("motion module busy")
        self._halt = threading.Event()

        def run():
            while not self._halt.is_set():
                if not self.silent:
                    dev = (time.monotonic_ns() // 1000) % (1 << 32)
                    on_sample("g", dev, 0.0, "hardware_clock", (0.0, 0.0, 0.0))
                    on_sample("a", dev, 0.0, "hardware_clock", (0.0, -9.81, 0.0))
                    self.accel_sent += 1
                time.sleep(0.005)

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()

    def profile_info(self):
        if self.fail_profile:
            raise OSError("usb gone while reading options")
        return {"actual": {"gyro_hz": 200, "accel_hz": self.accel_hz}, "rate_fallback": None}

    def stop(self):
        if self._halt is not None:
            self._halt.set()
            self._thread.join()
        for _ in range(self.burst_on_stop):  # samples the SDK hands over while stopping
            time.sleep(0.0001)
            self.on_sample("a", (time.monotonic_ns() // 1000) % (1 << 32), 0.0, "hardware_clock", (0.0, -9.81, 0.0))
            self.accel_sent += 1
        self.stopped = True
        if self.fail_stop:
            raise RuntimeError("stop timed out")


def _imu_cli(model, tmp_path, monkeypatch, device, *, extra=()):
    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)
    monkeypatch.setattr(ro, "open_raw", lambda p, baud=None, **kw: os.dup(model))
    out = tmp_path / "obs.jsonl"
    code = ro.main(["--device", "/dev/model", "--out", str(out), "--seconds", "1.0", "--imu", *extra],
                   imu_factory=lambda cfg: device)
    return code, out


def test_the_imu_is_recorded_with_its_verdicts_generations_and_intervals(model, tmp_path, monkeypatch, capsys):
    dev = FakeImuDevice()
    code, out = _imu_cli(model, tmp_path, monkeypatch, dev)
    res = json.loads(capsys.readouterr().out)
    assert code == 0 and res["result"] == "ok" and res["complete"] and res["stopped_confirmed"], res
    assert dev.started and dev.stopped
    rows = [json.loads(x) for x in out.read_text().splitlines()]
    head = rows[0]["imu"]
    assert head["measured"] is False and head["operating"] is False and head["interval_bound_s"] == 0.03
    assert [r for r in rows if r["t"] == "imu_profile"][0]["actual"]["accel_hz"] == 200
    imu = [r for r in rows if r["t"] == "imu"]
    ok = [r for r in imu if r["verdict"] == "ok"]
    assert len(ok) > 50
    assert all((r["lo"], r["hi"]) == (r["host_ns"] - 30_000_000, r["host_ns"]) for r in ok)
    end = rows[-1]["imu"]
    assert end["counts"]["accepted"] == len(ok) and end["counts"]["gyro"] >= len(ok)
    assert all(r["kind"] == "DISARM" for r in rows if r["t"] == "written")


def test_no_imu_device_is_refused_before_anything_is_written(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)
    out = tmp_path / "obs.jsonl"

    def none(cfg):
        raise RuntimeError("no RealSense device found")

    assert ro.main(["--device", "/dev/null", "--out", str(out), "--seconds", "1", "--imu"], imu_factory=none) == 2
    assert "imu" in json.loads(capsys.readouterr().out)["why"] and not out.exists()


def test_an_imu_that_fails_to_start_still_ends_with_a_confirmed_disarm(model, tmp_path, monkeypatch, capsys):
    dev = FakeImuDevice(fail_start=True)
    code, out = _imu_cli(model, tmp_path, monkeypatch, dev)
    res = json.loads(capsys.readouterr().out)
    assert code == 1 and res["result"] == "imu_start_failed" and res["stopped_confirmed"], res
    assert dev.stopped  # a failed start is stopped too
    rows = [json.loads(x) for x in out.read_text().splitlines()]
    assert [(r["stage"], r["error"]) for r in rows if r["t"] == "imu_error"] == [
        ("start", "RuntimeError: motion module busy")]


def test_an_imu_that_sends_nothing_is_no_observed_source(model, tmp_path, monkeypatch, capsys):
    code, _ = _imu_cli(model, tmp_path, monkeypatch, FakeImuDevice(silent=True))
    res = json.loads(capsys.readouterr().out)
    assert code == 1 and res["result"] == "no_imu", res


# --- Codex 7b round 1 ------------------------------------------------------------------------------------------------
def test_a_profile_read_failing_after_the_start_is_recorded_and_stopped(model, tmp_path, monkeypatch, capsys):
    dev = FakeImuDevice(fail_profile=True)
    code, out = _imu_cli(model, tmp_path, monkeypatch, dev)
    res = json.loads(capsys.readouterr().out)
    assert code == 1 and res["result"] == "imu_profile_failed" and res["stopped_confirmed"], res
    assert dev.stopped


def test_the_profile_row_says_whether_the_opened_rate_fits_the_watch(model, tmp_path, monkeypatch, capsys):
    """40 Hz opened: 25 ms period + 30 ms bound > the watch's 50 ms gap limit."""
    code, out = _imu_cli(model, tmp_path, monkeypatch, FakeImuDevice(accel_hz=40))
    assert code == 0, capsys.readouterr().out
    prof = [json.loads(x) for x in out.read_text().splitlines() if '"imu_profile"' in x][0]
    assert prof["fits_watch"] == {"gap_max_s": 0.05, "accel_hz": 40, "interval_bound_s": 0.03, "ok": False}


def test_every_sample_handed_over_while_stopping_is_recorded(model, tmp_path, monkeypatch, capsys):
    """P2: 150 samples during stop (more than one 64-sample read) all reach the record before its end row -- as
    many rows as the device sent (round 2: counted on the producer side)."""
    dev = FakeImuDevice(burst_on_stop=150)
    code, out = _imu_cli(model, tmp_path, monkeypatch, dev)
    assert code == 0, capsys.readouterr().out
    rows = [json.loads(x) for x in out.read_text().splitlines()]
    assert sum(1 for r in rows if r["t"] == "imu") == dev.accel_sent
    end = rows[-1]
    counts = end["imu"]["counts"]
    assert end["t"] == "end"
    assert counts["accepted"] == sum(1 for r in rows if r["t"] == "imu" and r["verdict"] == "ok")
    dropped = sum(counts[k] for k in ("missing_dev", "dup", "late", "not_newer", "bound_broken"))
    assert sum(1 for r in rows if r["t"] == "imu") == counts["accepted"] + dropped


def test_a_failed_imu_stop_leaves_the_record_incomplete(model, tmp_path, monkeypatch, capsys):
    """P2: callbacks may still run after a failed stop -- the record is not complete and gets no .ok."""
    code, out = _imu_cli(model, tmp_path, monkeypatch, FakeImuDevice(fail_stop=True))
    res = json.loads(capsys.readouterr().out)
    assert code == 1 and res["result"] == "imu_stop_failed" and res["complete"] is False, res
    assert not Path(str(out) + ".ok").exists() and not ro.record_complete(out)


def _st(t, seq):
    from forklift_core.mcu_link.messages import FaultFlag, McuState, RejectReason, StateFlag

    return McuState(mcu_time_us=t, boot_id=3, arm_gen=0, last_rx_seq=seq, mode=Mode.DISARMED,
                    reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0,
                    wheel_count_right=0, wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.0,
                    lift_m=0.0, current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0), driver_status=(0, 0, 0, 0),
                    rx_errors=0, cycle_overruns=0, flags=StateFlag(0))


class _ScriptedLink:
    """Writes every queued DISARM (recorded as written, last byte done) -- from service call ``write_from`` on, as a
    port that answered EAGAIN before -- and delivers STATEs from a script: ``script(calls, last_written_seq,
    queued_seq) -> [seq, ...]`` named by the STATEs of this service call."""

    ever_connected = True

    def __init__(self, script, write_from=1):
        self.link, self.script, self.calls, self.obs, self.t = object(), script, 0, None, 0
        self.written = []
        self.write_from = write_from

    def service(self, host, now_ns):
        self.calls += 1
        for o in list(host._outbox) if self.calls >= self.write_from else []:  # noqa: SLF001
            if o.kind == "DISARM":
                host._outbox.remove(o)  # noqa: SLF001
                seq = int(o.seq) & 0xFFFF
                self.written.append(seq)
                self.obs._log("written", now_ns, b"", {"kind": "DISARM", "seq": seq})  # noqa: SLF001
                self.obs._done(seq, now_ns)  # noqa: SLF001
        for seq in self.script(self.calls, self.written[-1] if self.written else None, host.queued_disarm_seq()):
            self.t += 1
            st = _st(self.t, seq)
            host._state = st  # noqa: SLF001
            self.obs._on_state(st, now_ns + self.t)  # noqa: SLF001
        return []


def _finishing(script, write_from=1):
    link = _ScriptedLink(script, write_from)
    openings = ro.Openings()
    openings.count = 1
    obs = ro.Observer(link, ro.Record(io.StringIO()), openings=openings, lidar=lidar_cfg.load_lidar_config())
    link.obs = obs
    return obs, link


def test_the_end_disarm_goes_out_at_once_and_two_late_states_confirm_it():
    """P1-4: the first two STATEs of the connection arrive together late in the confirm window -- the earlier seq,
    then the end DISARM's: confirmed, with one DISARM written."""
    obs, link = _finishing(lambda k, w, q: [(w - 1) & 0xFFFF, w] if k == 60 else [])
    assert obs.finish() and len(link.written) == 1


def test_a_first_state_already_naming_the_end_disarm_gets_one_more():
    """P1-4: the connection's first STATE already names the end DISARM (no step can show it): asked once more with
    a new seq, whose step confirms it."""
    obs, link = _finishing(lambda k, w, q: [w] if k >= 3 else [])
    assert obs.finish() and len(link.written) == 2 and link.written[0] != link.written[1]


# --- Codex 7b round 2 ------------------------------------------------------------------------------------------------
def test_a_reask_waits_for_the_first_end_disarm_to_be_written():
    """P2: the port answers EAGAIN at first and the first STATE already names the still queued seq -- the re-ask
    comes only once that DISARM is out, with a new seq, and confirms."""
    obs, link = _finishing(lambda k, w, q: [w if w is not None else q] if k >= 2 else [], write_from=4)
    assert obs.finish() and len(link.written) == 2 and link.written[0] != link.written[1]


def test_the_imu_waits_for_the_board_link_to_open(tmp_path, monkeypatch, capsys):
    """P2: the board is not there (open_first only schedules a retry): the D435i is never started."""
    import errno

    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)

    def missing(p, baud=None, **kw):
        raise FileNotFoundError(errno.ENOENT, "no such device", p)

    monkeypatch.setattr(ro, "open_raw", missing)
    dev = FakeImuDevice()
    out = tmp_path / "obs.jsonl"
    code = ro.main(["--device", "/dev/absent", "--out", str(out), "--seconds", "0.3", "--imu"],
                   imu_factory=lambda cfg: dev)
    res = json.loads(capsys.readouterr().out)
    assert code == 1 and not dev.started, res


def test_an_imu_queue_overflow_leaves_the_record_incomplete(model, tmp_path, monkeypatch, capsys):
    """P2: raw samples lost before they were recorded -- result imu_overflow, no .ok."""
    cfg = tmp_path / "imu.yaml"
    cfg.write_text((REPO / "config/real/imu.yaml").read_text().replace("queue_max: 400", "queue_max: 10"))
    code, out = _imu_cli(model, tmp_path, monkeypatch, FakeImuDevice(burst_on_stop=50),
                         extra=("--imu-config", str(cfg)))
    res = json.loads(capsys.readouterr().out)
    assert code == 1 and res["result"] == "imu_overflow" and res["complete"] is False, res
    assert not Path(str(out) + ".ok").exists()


# --- Codex 7b round 3 ------------------------------------------------------------------------------------------------
def test_a_device_held_at_a_retried_first_opening_is_refused(tmp_path, monkeypatch, capsys):
    """P2: the first opening finds no device, the retry finds it held: refused, exit 2, the record incomplete."""
    import errno

    from forklift_core.live.executor_lock import DeviceBusy

    monkeypatch.setattr(ro, "writer_marker", lambda device, bench: None)
    calls = []

    def opener(p, baud=None, **kw):
        calls.append(p)
        if len(calls) == 1:
            raise FileNotFoundError(errno.ENOENT, "no such device", p)
        raise DeviceBusy(p, {"pid": 4242, "tool": "teleop_bridge"})

    monkeypatch.setattr(ro, "open_raw", opener)
    monkeypatch.setattr(ro.ReconnectingLink, "__init__", _fast_retry(ro.ReconnectingLink.__init__))
    out = tmp_path / "obs.jsonl"
    code = ro.main(["--device", "/dev/later", "--out", str(out), "--seconds", "0.5"])
    res = json.loads(capsys.readouterr().out)
    assert code == 2 and res["result"] == "refused" and res["complete"] is False, res
    end = json.loads(out.read_text().splitlines()[-1])
    assert end["t"] == "end" and end["complete"] is False


def _fast_retry(init):
    def wrapped(self, *a, **kw):
        kw["retry_s"] = 0.05
        init(self, *a, **kw)
    return wrapped


def test_an_overflow_with_nothing_accepted_is_reported_as_the_overflow(model, tmp_path, monkeypatch, capsys):
    """P2: a quiet device sending a burst on stop: the overflow, not no_imu."""
    cfg = tmp_path / "imu.yaml"
    cfg.write_text((REPO / "config/real/imu.yaml").read_text().replace("queue_max: 400", "queue_max: 10"))
    code, out = _imu_cli(model, tmp_path, monkeypatch, FakeImuDevice(silent=True, burst_on_stop=50),
                         extra=("--imu-config", str(cfg)))
    res = json.loads(capsys.readouterr().out)
    assert code == 1 and res["result"] == "imu_overflow", res
