"""The bench console's link (plan 3, partial writes; 3.1 B1): ``BenchTtyLink`` passes each frame's first byte and
every continuation through the gate (with the frame: its kind decides the conditions); a refused or overdue continuation (20 ms) is abandoned and the poison follows at
once, not waiting for another frame; a poison not out within 50 ms loses the link (the port closes). A frame refused
before its first byte goes back to the host untouched."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from real_mcu_link import POISON, BenchTtyLink, LinkLost  # noqa: E402

from forklift_core.plant.mcu_host import Outgoing  # noqa: E402


class Clock:
    def __init__(self):
        self.ns = 1_000_000_000

    def __call__(self):
        return self.ns

    def advance_ms(self, ms):
        self.ns += int(ms * 1e6)


class Host:
    def __init__(self, frames):
        self.queue = list(frames)
        self.log = []

    def next_frame(self, now_ns):
        return self.queue.pop(0) if self.queue else None

    def unsent(self, f, now_ns):
        self.log.append(("unsent", f.kind))
        self.queue.insert(0, f)

    def frame_written(self, f, t):
        self.log.append(("written", f.kind))

    def frame_abandoned(self, f, n, now_ns):
        self.log.append(("abandoned", f.kind, n))


class Writer:
    """os.write stand-in: ``plan`` is a list of per-call byte limits (0 = EAGAIN); afterwards everything goes."""

    def __init__(self, plan=()):
        self.plan = list(plan)
        self.out = bytearray()

    def __call__(self, fd, data):
        if self.plan:
            n = self.plan.pop(0)
            if n == 0:
                raise BlockingIOError
            data = data[:n]
        self.out += data
        return len(data)


def frame(kind="COMMISSION", seq=1, size=20):
    return Outgoing(kind, seq, bytes([0x11]) * (size - 1) + b"\x00", None, 0, nonzero=True)


def allow(f, write):
    write()
    return True


def link(writer, gate=allow, clock=None):
    return BenchTtyLink(-1, gate=gate, clock=clock or Clock(), writer=writer)


def test_an_opening_writes_the_poison_before_its_first_frame():
    w = Writer()
    host = Host([frame()])
    link(w).pump_writes(host, 0)
    assert bytes(w.out[:len(POISON)]) == POISON and host.log == [("written", "COMMISSION")]


def test_a_first_byte_the_gate_refuses_goes_back_unsent():
    w = Writer()
    host = Host([frame()])
    lk = link(w, gate=lambda f, write: False)
    lk._poison_at = None  # the opening's poison already out
    lk.pump_writes(host, 0)
    assert w.out == b"" and host.log == [("unsent", "COMMISSION")] and len(host.queue) == 1


def test_a_continuation_the_gate_refuses_is_abandoned_and_the_poison_follows_at_once():
    w = Writer(plan=[5, 0])  # 5 bytes of the frame, then EAGAIN; then the gate turns
    host = Host([frame(size=20)])
    allow = {"on": True}

    def gate(f, write):
        if not allow["on"]:
            return False
        write()
        return True

    lk = link(w, gate=gate)
    lk._poison_at = None
    lk.pump_writes(host, 0)
    assert bytes(w.out) == bytes([0x11]) * 5
    allow["on"] = False  # a STOP came in
    lk.pump_writes(host, 0)
    assert ("abandoned", "COMMISSION", 5) in host.log
    assert bytes(w.out[5:]) == POISON  # the poison right after, with no frame waiting
    assert lk.abandoned == 1


def test_an_overdue_continuation_is_abandoned_after_20_ms():
    clock = Clock()
    w = Writer(plan=[5, 0, 0])
    host = Host([frame(size=20)])
    lk = link(w, clock=clock)
    lk._poison_at = None
    lk.pump_writes(host, 0)  # 5 bytes, then EAGAIN
    clock.advance_ms(10)
    lk.pump_writes(host, 0)  # EAGAIN again, within 20 ms: kept
    assert not any(e[0] == "abandoned" for e in host.log)
    clock.advance_ms(11)
    lk.pump_writes(host, 0)
    assert ("abandoned", "COMMISSION", 5) in host.log and bytes(w.out[5:]) == POISON


def test_a_short_eagain_within_20_ms_lets_the_frame_finish():
    clock = Clock()
    w = Writer(plan=[5, 0])
    host = Host([frame(size=20)])
    lk = link(w, clock=clock)
    lk._poison_at = None
    lk.pump_writes(host, 0)
    clock.advance_ms(5)
    lk.pump_writes(host, 0)
    assert host.log == [("written", "COMMISSION")] and len(w.out) == 20


def test_a_poison_not_out_within_50_ms_loses_the_link():
    clock = Clock()
    w = Writer(plan=[5] + [0] * 50)
    host = Host([frame(size=20)])
    allow = {"on": True}

    def gate(f, write):
        if allow["on"]:
            write()
        return allow["on"]

    lk = link(w, gate=gate, clock=clock)
    lk._poison_at = None
    lk.pump_writes(host, 0)
    allow["on"] = False
    lk.pump_writes(host, 0)  # abandoned; the poison gets EAGAIN
    clock.advance_ms(40)
    lk.pump_writes(host, 0)  # still within 50 ms
    clock.advance_ms(11)
    with pytest.raises(LinkLost):
        lk.pump_writes(host, 0)


def test_no_new_frame_goes_out_while_an_abandonment_poison_is_pending():
    clock = Clock()
    w = Writer(plan=[5, 0])
    second = frame("DISARM", seq=2, size=10)
    host = Host([frame(size=20), second])
    allow = {"on": True}

    def gate(f, write):
        if allow["on"]:
            write()
        return allow["on"]

    lk = link(w, gate=gate, clock=clock)
    lk._poison_at = None
    lk.pump_writes(host, 0)
    allow["on"] = False
    lk.pump_writes(host, 0)  # abandoned, poison EAGAIN
    allow["on"] = True
    assert host.queue == [second]  # untouched until the poison is out
    lk.pump_writes(host, 0)
    assert bytes(w.out[5:5 + len(POISON)]) == POISON and ("written", "DISARM") in host.log


def test_the_gate_is_given_the_frame_each_time():
    seen = []
    w = Writer(plan=[5, 0])

    def gate(f, write):
        seen.append(f.kind)
        write()
        return True

    lk = link(w, gate=gate)
    lk._poison_at = None
    host = Host([frame("BENCH_ARM", size=20)])
    lk.pump_writes(host, 0)
    lk.pump_writes(host, 0)
    assert seen == ["BENCH_ARM", "BENCH_ARM", "BENCH_ARM"] and ("written", "BENCH_ARM") in host.log


def test_the_gate_is_required():
    with pytest.raises(TypeError):
        BenchTtyLink(-1, gate=None)


def test_a_continuation_that_turns_overdue_inside_the_gate_is_not_written():
    """The 20 ms are judged under the boundary, right before the write (round 2 P2-1)."""
    clock = Clock()
    w = Writer(plan=[5, 0])
    host = Host([frame(size=20)])
    calls = {"n": 0}

    def gate(f, write):
        calls["n"] += 1
        if calls["n"] == 3:
            clock.advance_ms(21)  # the lock took long: overdue by the time the write would happen
        write()
        return True

    lk = link(w, gate=gate, clock=clock)
    lk._poison_at = None
    lk.pump_writes(host, 0)  # first byte, then a continuation that hits EAGAIN
    lk.pump_writes(host, 0)
    assert ("abandoned", "COMMISSION", 5) in host.log and ("written", "COMMISSION") not in host.log


def test_a_poison_that_unblocks_after_50_ms_is_not_finished_the_link_is_lost():
    clock = Clock()
    w = Writer(plan=[5, 0, 0])
    host = Host([frame(size=20)])
    on = {"v": True}

    def gate(f, write):
        if on["v"]:
            write()
        return on["v"]

    lk = link(w, gate=gate, clock=clock)
    lk._poison_at = None
    lk.pump_writes(host, 0)
    on["v"] = False
    lk.pump_writes(host, 0)  # abandoned; the poison gets EAGAIN
    clock.advance_ms(51)  # the fd is writable again, but too late
    with pytest.raises(LinkLost):
        lk.pump_writes(host, 0)
    assert bytes(w.out[5:]) == b""  # not one poison byte after the deadline
