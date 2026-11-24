"""docs/design/2026-11-19-real-safety-wiring.md 8: one executor per MCU device -- an exclusive flock on the very fd the
link opens (every opening, reconnects included); a refusal names the holder (pid, tool) from /proc/locks; the kernel
drops it when its holder dies, and no file is left to block anyone (lead conditions a-c, 2026-11-19)."""

import os
import signal
import subprocess
import sys
import time
import tty

import pytest

from forklift_core.live.executor_lock import DeviceBusy, holder, lock_fd

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="flock and /proc/locks are Linux's")


@pytest.fixture
def pty_path():
    master, slave = os.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    yield path
    os.close(slave)
    os.close(master)


def open_dev(path):
    return os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)


def probe(path):
    """Whether the device is free now (test only: open, lock, close)."""
    fd = open_dev(path)
    try:
        lock_fd(fd, path)
    finally:
        os.close(fd)


def holder_of(path):
    fd = open_dev(path)
    try:
        return holder(fd)
    finally:
        os.close(fd)


def test_a_second_opening_of_a_locked_device_is_refused_with_its_holder(pty_path):
    a = open_dev(pty_path)
    b = open_dev(pty_path)
    try:
        lock_fd(a, pty_path)
        with pytest.raises(DeviceBusy) as err:
            lock_fd(b, pty_path)
        assert err.value.holder is not None and err.value.holder["pid"] == os.getpid()
        assert str(os.getpid()) in str(err.value)
    finally:
        os.close(a)
        os.close(b)


def test_an_alias_of_the_device_is_the_same_lock(pty_path, tmp_path):
    alias = tmp_path / "forklift-mcu"
    alias.symlink_to(pty_path)
    a = open_dev(pty_path)
    b = open_dev(str(alias))
    try:
        lock_fd(a, pty_path)
        with pytest.raises(DeviceBusy):
            lock_fd(b, str(alias))
    finally:
        os.close(a)
        os.close(b)


def test_a_holder_killed_hard_leaves_nothing_behind_and_the_next_opening_works(pty_path):
    code = ("import os, sys, time\n"
            "sys.path.insert(0, sys.argv[2])\n"
            "from forklift_core.live.executor_lock import lock_fd\n"
            "fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)\n"
            "lock_fd(fd, sys.argv[1])\n"
            "print('held', flush=True)\n"
            "time.sleep(60)\n")
    src = os.path.join(os.path.dirname(__file__), "..", "..", "..", "src")
    child = subprocess.Popen([sys.executable, "-c", code, pty_path, os.path.abspath(src)], stdout=subprocess.PIPE,
                             text=True)
    try:
        assert child.stdout.readline().strip() == "held"
        with pytest.raises(DeviceBusy) as err:
            probe(pty_path)
        assert err.value.holder["pid"] == child.pid and "python" in err.value.holder["tool"]
        child.send_signal(signal.SIGKILL)
        child.wait(timeout=5)
        deadline = time.monotonic() + 5
        while holder_of(pty_path) is not None and time.monotonic() < deadline:
            time.sleep(0.01)
        probe(pty_path)  # the kernel dropped it: free again
        fd = open_dev(pty_path)
        try:
            lock_fd(fd, pty_path)
        finally:
            os.close(fd)
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=5)
        child.stdout.close()


def test_no_holder_on_a_free_device(pty_path):
    assert holder_of(pty_path) is None


def _link_module():
    from pathlib import Path

    tools = Path(__file__).resolve().parents[3] / "tools"
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    import real_mcu_link

    return real_mcu_link


def _host():
    from forklift_core.plant.mcu_host import HostConfig, McuLinkHost

    return McuLinkHost(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                  outbox_max=16))


def test_held_at_the_first_opening_the_link_ends_refused(pty_path):
    """Design 8.1: raised once, then never opened nor written again -- even after the holder lets go."""
    rml = _link_module()
    other = open_dev(pty_path)
    lock_fd(other, pty_path)
    link = rml.ReconnectingLink(pty_path, retry_s=0.0, exclusive=True, product_reader=lambda fd: None)
    host = _host()
    with pytest.raises(DeviceBusy) as err:
        link.receive(host, 1)
    assert err.value.holder["pid"] == os.getpid() and link.refused is not None
    os.close(other)  # the holder goes
    host.request_disarm(2)
    assert link.receive(host, 3) == [] and link.transmit(host, 4) == [] and link.service(host, 5) == []
    assert link.link is None
    probe(pty_path)  # nothing of the refused link holds it


def test_a_reconnect_to_a_held_device_is_shown_and_retried(pty_path):
    """After a connection, a held device at a reopening is a device_busy event and a retry (design 8.1)."""
    rml = _link_module()
    link = rml.ReconnectingLink(pty_path, retry_s=0.0, exclusive=True, product_reader=lambda fd: None)
    host = _host()
    link.receive(host, 1)
    assert link.link is not None and link.ever_connected
    with pytest.raises(DeviceBusy):  # the link's own fd holds it
        probe(pty_path)
    link.link.close()
    link.link = None  # as after a lost device
    other = open_dev(pty_path)
    lock_fd(other, pty_path)
    try:
        busy = [e for e in link.receive(host, 2) if e.kind == "device_busy"]
        assert link.link is None and busy and busy[0].detail["holder"]["pid"] == os.getpid()
        assert not [e for e in link.receive(host, 3) if e.kind == "device_busy"]  # shown once, not every retry
    finally:
        os.close(other)
    events = link.receive(host, 4)
    try:
        assert link.link is not None and any(e.kind == "connected" for e in events)
    finally:
        link.link.close()


def test_a_link_without_exclusive_is_as_before(pty_path):
    rml = _link_module()
    other = open_dev(pty_path)
    lock_fd(other, pty_path)
    link = rml.ReconnectingLink(pty_path, retry_s=0.0, product_reader=lambda fd: None)
    try:
        link.receive(_host(), 1)
        assert link.link is not None  # opens whatever holds the lock (the listening tools are unchanged)
    finally:
        os.close(other)
        if link.link is not None:
            link.link.close()


def test_the_teleop_bridge_refuses_to_start_on_a_held_device_naming_the_holder(pty_path, capsys, monkeypatch):
    from pathlib import Path

    tools = Path(__file__).resolve().parents[3] / "tools"
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    import teleop_bridge

    monkeypatch.setattr(teleop_bridge, "writer_marker", lambda device, bench: None)  # past the bench check

    def no_console(self):
        raise RuntimeError("the console port is taken too")  # Codex impl round 2: the device is refused first

    monkeypatch.setattr(teleop_bridge.LiveServer, "start", no_console)
    other = open_dev(pty_path)
    lock_fd(other, pty_path)
    try:
        code = teleop_bridge.main(["--device", pty_path])
    finally:
        os.close(other)
    err = capsys.readouterr().err
    assert code == 2 and f"pid {os.getpid()}" in err and "held by another executor" in err


def test_a_refused_opening_sets_nothing_on_the_held_port(pty_path, monkeypatch):
    """Codex PR 5 design round 3 P2-6: the lock is taken before the port is configured -- refused, no termios and no
    modem-bit call reached the device another executor holds."""
    rml = _link_module()
    calls = []
    monkeypatch.setattr(rml.termios, "tcsetattr", lambda *a: calls.append("tcsetattr"))
    monkeypatch.setattr(rml.fcntl, "ioctl", lambda *a: calls.append("ioctl"))
    other = open_dev(pty_path)
    lock_fd(other, pty_path)
    link = rml.ReconnectingLink(pty_path, retry_s=0.0, exclusive=True, product_reader=lambda fd: None)
    try:
        with pytest.raises(DeviceBusy):
            link.receive(_host(), 1)
        assert link.link is None and calls == []
    finally:
        os.close(other)
    link2 = rml.ReconnectingLink(pty_path, retry_s=0.0, exclusive=True, product_reader=lambda fd: None)
    link2.receive(_host(), 2)
    try:
        assert link2.link is not None and "tcsetattr" in calls  # configured once it holds the lock
    finally:
        link2.link.close()
