"""PR 7 design (docs/design/2026-11-25-real-sources-pr7.md) 1.1: the observe run's write boundary. Every opening's
link is an ObserveTtyLink, which writes only the poison and canonically encoded DISARMs; anything else is refused
before its first byte. The expectation is on the raw bytes the far end receives, and the same scenarios written
through a plain TtyLink do write the refused frames -- so these tests fail if the boundary is taken out."""

import os
import sys
import tty
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import Arm, ClearFaults, Disarm, encode
from forklift_core.plant.mcu_host import HostConfig, McuLinkHost

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import real_mcu_link as rml  # noqa: E402

CFG = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)


def _pty():
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    os.set_blocking(slave, False)
    return master, slave


def _drain(fd) -> bytes:
    out = b""
    while True:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            return out
        if not chunk:
            return out
        out += chunk


def _queue(host, kinds):
    """Frames straight into the host's outbox, as any code holding the host could (the boundary must not rely on
    the host's own rules)."""
    seqs = []
    for k in kinds:
        msg = {"DISARM": Disarm(), "ARM": Arm(7, 1000), "CLEAR_FAULTS": ClearFaults(7)}.get(k)
        if msg is None:  # a DRIVE or COMMISSION: any non-DISARM bytes with that kind
            host._enqueue(k, Disarm(), 1, expires_ns=10**18, nonzero=True)  # noqa: SLF001
        else:
            host._enqueue(k, msg, 1)  # noqa: SLF001
        seqs.append(host.last_queued_seq)
    return seqs


def _expected(seqs_written):
    return rml.POISON + b"".join(encode(Disarm(), s) for s in seqs_written)


@pytest.mark.parametrize("kinds,allowed", [
    (["ARM"], []),
    (["DRIVE"], []),
    (["COMMISSION"], []),
    (["CLEAR_FAULTS"], []),
    (["ARM", "DRIVE", "COMMISSION", "CLEAR_FAULTS"], []),
    (["DISARM"], [0]),
    (["ARM", "DISARM", "DRIVE", "DISARM", "CLEAR_FAULTS"], [1, 3]),
])
def test_only_the_poison_and_canonical_disarms_reach_the_device(kinds, allowed):
    master, slave = _pty()
    try:
        link = rml.ObserveTtyLink(master)
        host = McuLinkHost(CFG)
        seqs = _queue(host, kinds)
        for k in range(len(kinds) + 2):
            link.pump_writes(host, 10 + k)
        got = _drain(slave)
        assert got == (_expected([seqs[i] for i in allowed]) if allowed else b"")
        assert link.refused == len(kinds) - len(allowed)
    finally:
        os.close(master)
        os.close(slave)


def test_without_the_boundary_the_same_queue_reaches_the_device():
    """The control: a plain TtyLink writes the refused frames -- the raw-byte expectation above sees the difference."""
    master, slave = _pty()
    try:
        link = rml.TtyLink(master)
        host = McuLinkHost(CFG)
        _queue(host, ["ARM", "CLEAR_FAULTS"])
        for k in range(4):
            link.pump_writes(host, 10 + k)
        assert _drain(slave) != b""
    finally:
        os.close(master)
        os.close(slave)


def test_a_disarm_whose_bytes_are_not_its_canonical_encoding_is_refused():
    master, slave = _pty()
    try:
        link = rml.ObserveTtyLink(master)
        host = McuLinkHost(CFG)
        host._enqueue("DISARM", Arm(7, 1000), 1)  # noqa: SLF001 -- labelled DISARM, ARM bytes
        link.pump_writes(host, 10)
        assert _drain(slave) == b"" and link.refused == 1
    finally:
        os.close(master)
        os.close(slave)


def test_partial_writes_resume_and_never_leak_a_refused_frame(monkeypatch):
    """EAGAIN in the poison and in a DISARM: the rest follows; a refused frame between never gets a byte."""
    rfd, wfd = os.pipe()
    os.set_blocking(rfd, False)
    real_write = os.write
    calls = {"n": 0}

    def choppy(fd, data):
        calls["n"] += 1
        if calls["n"] % 3 == 0:
            raise BlockingIOError
        return real_write(fd, bytes(data[:7]))

    monkeypatch.setattr(rml.os, "write", choppy)
    link = rml.ObserveTtyLink(wfd)
    host = McuLinkHost(CFG)
    seqs = _queue(host, ["DISARM", "ARM", "DISARM"])
    for k in range(200):
        link.pump_writes(host, 10 + k)
    got = _drain(rfd)
    os.close(rfd)
    os.close(wfd)
    assert got == _expected([seqs[0], seqs[2]])


def test_every_reopening_gets_the_observe_link_and_its_own_poison():
    master, slave = _pty()
    try:
        opened = []

        def opener(path, baud, **kw):
            fd = os.dup(master)
            opened.append(fd)
            return fd

        link = rml.ReconnectingLink("/dev/fake", opener=opener, link_factory=rml.ObserveTtyLink,
                                    product_reader=lambda fd: None)
        host = McuLinkHost(CFG)
        expect = b""
        for k in range(2):
            link.receive(host, 10 + k)
            assert isinstance(link.link, rml.ObserveTtyLink)
            seqs = _queue(host, ["ARM", "DISARM"])
            link.transmit(host, 20 + k)
            link.service(host, 30 + k)  # the other write path
            expect += _expected([seqs[1]])
            link.link.close()
            link.link = None  # as after a lost device
        assert _drain(slave) == expect
    finally:
        os.close(master)
        os.close(slave)


def test_the_teleop_bridge_takes_the_mount_from_the_lidar_file_only_once_measured(capsys, monkeypatch):
    """PR 7 design 4: config/real/lidar.yaml is the mount's one source -- unmeasured, the bridge still asks for
    --lidar-mount."""
    import teleop_bridge

    code = teleop_bridge.main(["--device", "/dev/null", "--scan-socket", "/tmp/nowhere.sock"])
    assert code == 2 and "lidar.yaml" in capsys.readouterr().err


def test_the_observe_tool_wires_the_observe_link_on_every_opening():
    """The boundary is only as good as its wiring: real_observe's link is made with the Openings factory, and that
    factory makes ObserveTtyLinks."""
    import ast

    import real_observe

    tree = ast.parse((REPO / "tools" / "real_observe.py").read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "ReconnectingLink"]
    assert len(calls) == 1
    kw = {k.arg: k.value for k in calls[0].keywords}
    assert isinstance(kw.get("link_factory"), ast.Name) and kw["link_factory"].id == "openings"
    r, w = os.pipe()
    try:
        made = real_observe.Openings()(w)
        assert isinstance(made, rml.ObserveTtyLink)
    finally:
        os.close(r)
        os.close(w)


def test_a_link_lost_mid_disarm_leaves_a_prefix_and_the_new_fd_starts_with_its_poison(monkeypatch):
    """Codex 7a-1 P3: the loss happens in ``service()`` itself, with a refused ARM queued first; the lost fd shows a
    prefix of what it was allowed, the new fd the poison and whole DISARMs."""
    import errno

    master, slave = _pty()
    real_write = os.write
    state = {"fail": True, "n": 0}

    def write(fd, data):
        state["n"] += 1
        if state["fail"] and state["n"] == 2:
            state["fail"] = False
            raise OSError(errno.EIO, "gone")
        return real_write(fd, bytes(data[:40]))

    try:
        link = rml.ReconnectingLink("/dev/fake", opener=lambda p, b, **kw: os.dup(master),
                                    link_factory=rml.ObserveTtyLink, product_reader=lambda fd: None, retry_s=0.0)
        host = McuLinkHost(CFG)
        monkeypatch.setattr(rml.os, "write", write)
        link.service(host, 1)  # opens
        seqs = _queue(host, ["ARM", "DISARM"])
        link.service(host, 2)  # 40 bytes of the poison (a partial write: the rest next time)
        link.service(host, 3)  # the rest fails with EIO: lost
        assert link.link is None
        first_fd = _drain(slave)
        assert first_fd == rml.POISON[:40]
        seqs2 = _queue(host, ["ARM", "DISARM"])
        for k in range(10):
            link.service(host, 4 + k)  # reopens, refuses the ARM, writes the poison and the DISARM
        assert _drain(slave) == rml.POISON + encode(Disarm(), seqs2[1])
        assert seqs  # the first DISARM went with the lost fd (the host dropped it on the disconnect)
    finally:
        os.close(master)
        os.close(slave)
