"""The arrival-day rehearsal's model-peer check (docs/design/2026-11-15-arrival-day-rehearsal.md 2.2): a synthetic
stand-in opens only when the device is a pty whose master the bench's mcu_model alone holds."""

from __future__ import annotations

import errno
import os

import pytest

from forklift_core.live import model_peer as mp

PTS_MAJOR, ACM_MAJOR, USB_MAJOR, TTYS_MAJOR = 136, 166, 188, 4
ME = 1000


class FakeProc:
    """A /proc and stat view: the bench's model (pid 50) holds the master of /dev/pts/7."""

    def __init__(self):
        self.devices = {"/dev/pts/7": (True, os.makedev(PTS_MAJOR, 7))}
        self.procs = {50: {"uid": ME, "starttime": 12345, "exe": "/build/mcu_model",
                           "fds": {3: ("/dev/ptmx", 7), 4: ("/dev/null", None)}},
                      60: {"uid": ME, "starttime": 999, "exe": "/usr/bin/zsh", "fds": {0: ("/dev/pts/2", None)}},
                      70: {"uid": 0, "starttime": 1, "exe": None, "fds": None}}  # root's: not readable
        self.errors = {}  # (pid, what) -> errno, raised once per scan
        self.euid = ME

    # the interface model_peer uses
    def device(self, path):
        if path not in self.devices:
            raise FileNotFoundError(path)
        return self.devices[path]

    def pids(self):
        return sorted(self.procs)

    def uid(self, pid):
        return self._get(pid, "uid")

    def starttime(self, pid):
        return self._get(pid, "starttime")

    def exe(self, pid):
        return self._get(pid, "exe")

    def fds(self, pid):
        fds = self._get(pid, "fds")
        if fds is None:
            raise PermissionError(errno.EACCES, "fd")
        return {fd: target for fd, (target, _idx) in fds.items()}

    def tty_index(self, pid, fd):
        return self._get(pid, "fds")[fd][1]

    def _get(self, pid, what):
        if (pid, what) in self.errors:
            raise OSError(self.errors[(pid, what)], what)
        if pid not in self.procs:
            raise ProcessLookupError(pid)
        return self.procs[pid][what]


BENCH = {"schema": "rehearsal_bench/1", "pts": "/dev/pts/7", "pts_rdev": os.makedev(PTS_MAJOR, 7), "model_pid": 50,
         "model_starttime": 12345, "model_exe": "/build/mcu_model", "synthetic": True}


def refusal(proc, device="/dev/pts/7", bench=None):
    return mp.model_peer_refusal(device, BENCH if bench is None else bench, proc=proc)


def test_the_benchs_model_holding_the_master_alone_passes():
    assert refusal(FakeProc()) is None


@pytest.mark.parametrize("path, major", [("/dev/ttyACM0", ACM_MAJOR), ("/dev/ttyUSB0", USB_MAJOR),
                                         ("/dev/ttyS0", TTYS_MAJOR), ("/dev/forklift-mcu", ACM_MAJOR)])
def test_a_usb_cdc_or_uart_device_is_never_a_model(path, major):
    proc = FakeProc()
    proc.devices[path] = (True, os.makedev(major, 0))
    assert refusal(proc, device=path) == "not_a_pty"


def test_a_device_that_is_not_the_benchs_pty_is_refused():
    proc = FakeProc()
    proc.devices["/dev/pts/9"] = (True, os.makedev(PTS_MAJOR, 9))
    assert refusal(proc, device="/dev/pts/9") == "not_the_bench_pty"
    proc.devices["/tmp/file"] = (False, 0)
    assert refusal(proc, device="/tmp/file") == "not_a_pty"
    assert refusal(proc, device="/dev/pts/404") == "no_device"


@pytest.mark.parametrize("change, why", [
    (lambda p: p.procs.pop(50), "model_gone"),
    (lambda p: p.procs[50].update(starttime=1), "model_restarted"),  # the pid reused by another process
    (lambda p: p.procs[50].update(exe="/usr/bin/socat"), "not_mcu_model"),
    (lambda p: p.procs[50].update(fds={4: ("/dev/null", None)}), "model_holds_no_master"),
    (lambda p: p.procs[50].update(fds={3: ("/dev/ptmx", 8)}), "model_holds_no_master"),
])
def test_the_model_must_be_the_live_mcu_model_holding_that_master(change, why):
    proc = FakeProc()
    change(proc)
    assert refusal(proc) == why


def test_another_process_holding_the_same_master_is_a_relay():
    proc = FakeProc()
    proc.procs[61] = {"uid": ME, "starttime": 5, "exe": "/usr/bin/socat", "fds": {5: ("/dev/ptmx", 7)}}
    assert refusal(proc) == "master_shared"


def test_an_unreadable_process_is_tolerated_only_if_it_started_before_the_model():
    proc = FakeProc()
    proc.procs[60]["fds"] = None  # a non-dumpable same-uid session component (fds unreadable)
    proc.procs[60]["starttime"] = 100  # started at login, before the model (12345)
    assert refusal(proc) is None
    proc.procs[60]["starttime"] = 20000  # started after the model: it could be a relay
    assert refusal(proc) == "peer_scan_incomplete"


def test_any_other_read_error_closes_the_check():
    proc = FakeProc()
    proc.errors[(60, "uid")] = errno.EIO
    with pytest.raises(OSError):
        refusal(proc)
    proc = FakeProc()
    proc.errors[(60, "fds")] = errno.EPERM  # a permission error on a process started after the model
    proc.procs[60]["starttime"] = 20000
    assert refusal(proc) == "peer_scan_incomplete"


def test_a_process_that_vanishes_during_the_scan_is_rescanned_once():
    proc = FakeProc()

    class Vanish(FakeProc):
        calls = 0

        def fds(self, pid):
            if pid == 60:
                Vanish.calls += 1
                if Vanish.calls == 1:
                    raise FileNotFoundError(pid)  # exited between listing and reading
            return super().fds(pid)

    v = Vanish()
    v.__dict__.update(proc.__dict__)
    assert refusal(v) is None and Vanish.calls == 2

    class AlwaysVanish(FakeProc):
        def fds(self, pid):
            if pid == 60:
                raise FileNotFoundError(pid)
            return super().fds(pid)

    a = AlwaysVanish()
    a.__dict__.update(proc.__dict__)
    assert refusal(a) == "peer_scan_incomplete"


def test_root_is_refused():
    proc = FakeProc()
    proc.euid = 0
    assert refusal(proc) == "running_as_root"


def test_a_bench_record_that_is_not_one_is_refused():
    assert refusal(FakeProc(), bench={"schema": "something/1"}) == "bad_bench"
    assert refusal(FakeProc(), bench={**BENCH, "synthetic": False}) == "bad_bench"


def test_the_cheap_recheck_never_scans_other_processes():
    proc = FakeProc()

    class NoScan(FakeProc):
        def pids(self):
            raise AssertionError("the loop recheck must not enumerate processes")

    n = NoScan()
    n.__dict__.update(proc.__dict__)
    assert mp.model_alive_refusal("/dev/pts/7", BENCH, proc=n) is None
    n.procs[50]["starttime"] = 1
    assert mp.model_alive_refusal("/dev/pts/7", BENCH, proc=n) == "model_restarted"
    n.procs.pop(50)
    assert mp.model_alive_refusal("/dev/pts/7", BENCH, proc=n) == "model_gone"
