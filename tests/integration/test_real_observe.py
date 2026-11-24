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
    obs._on_state(st(2, 77), 300)  # noqa: SLF001 -- still 77: no step
    obs.host = SimpleNamespace(state=st(2, 77))  # acknowledged() reads only the host's latest STATE
    assert not obs.acknowledged(77)
    obs._on_state(st(3, 78), 400)  # noqa: SLF001
    obs._log("written", 500, b"", {"kind": "DISARM", "seq": 79})  # noqa: SLF001
    obs._on_state(st(4, 79), 600)  # noqa: SLF001 -- a step to 79 after its write
    obs.host = SimpleNamespace(state=st(4, 79))
    assert obs.acknowledged(79)
