"""tools/h743_link_check.py (plan docs/plans/2026-11-17-h743-board-check.md): the verdict arithmetic on synthetic
STATE rows, and one short run against the firmware core on the host model over a pty. Skipped without cmake or a
C compiler (the model part only)."""

import os
import shutil
import subprocess
import sys
import tty
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import h743_link_check as chk  # noqa: E402


def _rows(n, *, period_us=10_000, gap_at=None, drop_at=None, boot_change_at=None):
    rows, seq, t = [], 100, 0
    for i in range(n):
        if drop_at is not None and i == drop_at:
            t += period_us  # the MCU skipped a STATE: time jumps, seq does not
        if gap_at is not None and i == gap_at:
            seq += 1  # a STATE lost on the way
        boot = 2 if boot_change_at is not None and i >= boot_change_at else 1
        rows.append({"seq": seq % 0x10000, "mcu_time_us": t, "host_ns": t * 1000, "boot_id": boot,
                     "rx_errors": 0, "cycle_overruns": 0, "reset_cause": 0, "flags": 0, "mode": 1, "fault_flags": 0})
        seq += 1
        t += period_us
    return rows


def _summary(rows, rtts=(12.0,), lost=0, sent=1, errors=0, seconds=None):
    seconds = seconds if seconds is not None else len(rows) / 100.0
    return chk.summarize(rows, list(rtts), lost, sent, errors, seconds, seconds, None)


def test_a_clean_run_passes():
    s = _summary(_rows(1000))
    assert s["result"] == "PASS", s["checks"]
    assert s["seq_gaps"] == 0 and s["mcu_period_ms"]["mean"] == 10.0


def test_each_failure_is_caught():
    assert not _summary(_rows(1000, gap_at=500))["checks"]["no_seq_gaps"]
    assert not _summary(_rows(1000, drop_at=500))["checks"]["mcu_period"]  # 20 ms step
    assert not _summary(_rows(1000, boot_change_at=500))["checks"]["no_reset"]
    assert not _summary(_rows(1000), errors=1)["checks"]["no_frame_errors"]
    assert not _summary(_rows(1000), lost=1)["checks"]["round_trip"]
    assert not _summary(_rows(1000), rtts=(12.0, 60.0))["checks"]["round_trip"]
    assert not _summary(_rows(500), seconds=10.0)["checks"]["states_flowing"]
    slow = _rows(1000)
    slow[600]["host_ns"] += 150_000_000  # a 150 ms host-side stall
    for r in slow[601:]:
        r["host_ns"] += 150_000_000
    assert not _summary(slow)["checks"]["host_gap"]


def test_seq_wraps_without_a_gap():
    rows = _rows(1000)
    for i, r in enumerate(rows):
        r["seq"] = (0xFFF0 + i) % 0x10000
    assert _summary(rows)["seq_gaps"] == 0


@pytest.fixture(scope="module")
def mcu_model(tmp_path_factory) -> Path:
    if shutil.which("cmake") is None:
        pytest.skip("cmake not installed")
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("no C compiler")
    build = tmp_path_factory.mktemp("mcu_model_build")
    env = {**os.environ, "PYTHONPATH": str(REPO / "src")}
    r = subprocess.run(["cmake", "-S", str(REPO / "firmware"), "-B", str(build), "-DFW_MCU_MODEL=ON",
                        f"-DFW_PYTHON={sys.executable}"], capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stdout + r.stderr
    r = subprocess.run(["cmake", "--build", str(build), "--target", "mcu_model", "-j2"], capture_output=True,
                       text=True, env=env)
    assert r.returncode == 0, r.stdout + r.stderr
    return build / "mcu_model"


def test_a_short_run_against_the_host_model(mcu_model, tmp_path):
    """The tool's I/O end to end: STATEs decoded, DISARMs reflected in last_rx_seq, the record and summary written.
    Host-timing checks are not asserted here (a loaded test host is not the board)."""
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    proc = subprocess.Popen([str(mcu_model), "--link", os.ttyname(slave)])
    try:
        out = tmp_path / "link.jsonl"
        s = chk.run("pty", out, 3.0, 0.5, None, fd=master)  # run() closes the master
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        os.close(slave)
    print(chk.one_line(s))
    assert s["states"] > 150 and s["decoder_errors"] == 0, s
    assert s["checks"]["no_seq_gaps"] and s["checks"]["no_reset"], s
    assert s["disarm_sent"] >= 4 and s["disarm_lost"] == 0 and s["round_trip_ms"] is not None, s
    lines = out.read_text().splitlines()
    assert '"t": "summary"' in lines[-1] and any('"t": "rtt"' in x for x in lines)


def test_silence_before_the_end_and_late_rx_errors_fail():
    rows = _rows(1000)
    end = rows[-1]["host_ns"]
    assert _summary(rows)["checks"]["host_gap"]
    s = chk.summarize(rows, [12.0], 0, 1, 0, 10.0, 10.0, None, end_ns=end + 600_000_000)
    assert not s["checks"]["host_gap"]  # the link went quiet 600 ms before the end
    rows[0]["rx_errors"] = 0
    for r in rows[1:]:
        r["rx_errors"] = 1  # the opening delimiter closed a half frame once, early
    early = chk.summarize(rows, [12.0], 0, 1, 0, 10.0, 10.0, None, rx_baseline_ns=rows[20]["host_ns"])
    assert early["checks"]["mcu_rx_errors_zero"]
    rows[-1]["rx_errors"] = 2  # a later one is a real receive error
    late = chk.summarize(rows, [12.0], 0, 1, 0, 10.0, 10.0, None, rx_baseline_ns=rows[20]["host_ns"])
    assert not late["checks"]["mcu_rx_errors_zero"]


def test_a_device_lost_while_writing_still_leaves_a_failing_summary(tmp_path, monkeypatch):
    import errno

    master, slave = os.openpty()
    os.set_blocking(master, False)
    real_write = chk.os.write
    calls = {"n": 0}

    def write(fd, data):
        calls["n"] += 1
        if calls["n"] > 1:  # the delimiter goes out, the first DISARM meets a vanished device
            raise OSError(errno.EIO, "Input/output error")
        return real_write(fd, data)

    monkeypatch.setattr(chk.os, "write", write)
    out = tmp_path / "lost.jsonl"
    s = chk.run("pty", out, 2.0, 0.2, None, fd=master)
    os.close(slave)
    assert s["result"] == "FAIL" and s["link_lost"] and not s["checks"]["link_kept"]
    assert '"t": "summary"' in out.read_text().splitlines()[-1]
