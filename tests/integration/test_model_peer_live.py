"""model_peer against the live /proc (docs/design/2026-11-15-arrival-day-rehearsal.md 2.2): the built mcu_model given
the pty master (--link-fd) passes; the same pty with the master held by another process does not."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_bringup_tool import mcu_model  # noqa: E402,F401,I001 -- the module-scoped model build

from forklift_core.live import model_peer as mp  # noqa: E402


def bench_for(proc_pid: int, slave_path: str, exe: str) -> dict:
    sysproc = mp.SystemProc()
    return {"schema": mp.BENCH_SCHEMA, "pts": slave_path, "pts_rdev": os.stat(slave_path).st_rdev,
            "model_pid": proc_pid, "model_starttime": sysproc.starttime(proc_pid), "model_exe": exe,
            "synthetic": True}


@pytest.fixture
def model_on_master(mcu_model):  # noqa: F811
    master, slave = os.openpty()
    path = os.ttyname(slave)
    os.close(slave)
    p = subprocess.Popen([str(mcu_model), "--link-fd", str(master), "--duration-ms", "20000"], pass_fds=(master,))
    os.close(master)  # the model alone holds it
    time.sleep(0.3)
    yield p, path, os.path.realpath(str(mcu_model))
    p.terminate()
    p.wait(timeout=5)


@pytest.mark.skipif(os.geteuid() == 0, reason="the check refuses root")
def test_the_model_holding_the_master_passes_and_a_dead_model_does_not(model_on_master):
    p, path, exe = model_on_master
    bench = bench_for(p.pid, path, exe)
    assert mp.model_peer_refusal(path, bench) is None
    # the host tools open the slave by path and the model answers: a STATE frame arrives
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        deadline, got = time.monotonic() + 3.0, b""
        while time.monotonic() < deadline and not got:
            try:
                got = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.05)
        assert got, "no bytes from the model on the slave"
    finally:
        os.close(fd)
    p.terminate()
    p.wait(timeout=5)
    assert mp.model_peer_refusal(path, bench) in ("model_gone", "no_device", "not_the_bench_pty")


@pytest.mark.skipif(os.geteuid() == 0, reason="the check refuses root")
def test_a_master_held_by_a_relay_is_refused(mcu_model):  # noqa: F811
    master, slave = os.openpty()
    path = os.ttyname(slave)
    os.close(slave)
    # the model holds the master and so does a second process (a relay would): refused as shared
    relay = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(20)"], pass_fds=(master,))
    p = subprocess.Popen([str(mcu_model), "--link-fd", str(master), "--duration-ms", "20000"], pass_fds=(master,))
    os.close(master)
    try:
        time.sleep(0.3)
        assert mp.model_peer_refusal(path, bench_for(p.pid, path, os.path.realpath(str(mcu_model)))) == "master_shared"
        # and with only the relay holding it, there is no model at all
        assert mp.model_peer_refusal(path, bench_for(relay.pid, path, sys.executable)) == "not_mcu_model"
    finally:
        for q in (p, relay):
            q.terminate()
            q.wait(timeout=5)


@pytest.mark.skipif(os.geteuid() == 0, reason="the check refuses root")
def test_the_check_passes_while_short_lived_processes_churn(model_on_master):
    """ws1's parallel test workers start and end processes faster than two scans can miss them (2026-11-15: the
    bench refused its own pty, peer_scan_incomplete): a process that exits mid-scan holds nothing and is skipped."""
    p, path, exe = model_on_master
    bench = bench_for(p.pid, path, exe)
    churn = subprocess.Popen(["/bin/sh", "-c", "while :; do /bin/true & /bin/true & /bin/true; done"])
    try:
        time.sleep(0.2)
        results = []
        for _ in range(20):
            detail = []
            results.append((mp.model_peer_refusal(path, bench, detail=detail), detail))
    finally:
        churn.kill()
        churn.wait(timeout=5)
    assert all(why is None for why, _ in results), [r for r in results if r[0] is not None][:3]
