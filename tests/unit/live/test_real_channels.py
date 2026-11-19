"""docs/design/2026-11-19-real-loop.md 2.5 (the real send boundary): RealChannels counts every emergency event (STOP,
DISARM, a connection added or replaced, a disconnection) under its lock and hands the emergency batch out with the
serial it was taken at; RealLink processes it (DISARM through the arbiter's checks); a write is tried only when no
emergency came in since."""

import itertools

from forklift_core.live.real_channels import RealChannels, RealLink, send_boundary
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig

EPOCH = 7


def rig():
    tokens = itertools.count(1)
    a = Arbiter(EPOCH, ArbiterConfig(disarm_ignores_control_gen=True), lambda: f"t{next(tokens)}")
    ch = RealChannels()
    link = RealLink(a, ch)
    disarms = []
    a.on_disarm = lambda now: disarms.append(now)
    ch.connect("c1", "op", control=True)
    link.emergency(1.0)
    reply = a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH}, 1.0)
    a.lease_id_for_test = reply["lease_id"]
    return a, ch, link, disarms


def msg(a, kind, **f):
    return {"t": kind, "request_id": f"r{kind}{len(f)}", "session_epoch": EPOCH, "control_gen": a.control_gen,
            "lease_id": a.lease_id_for_test, **f}


def test_every_emergency_event_raises_the_serial_and_the_batch_carries_the_serial_it_was_taken_at():
    a, ch, link, disarms = rig()
    s0 = link.emergency(2.0)
    ch.message("c1", "op", msg(a, "stop"))
    ch.message("c1", "op", msg(a, "disarm"))
    ch.connect("c2", "op", control=True)  # the same client again: the lease is taken back
    ch.disconnect("c1", "op")
    assert ch.serial() == s0 + 4
    s1 = link.emergency(3.0)
    assert s1 == s0 + 4 and a.mode == "STOPPED"  # the STOP taken (the DISARM after the reconnect: lease taken back)


def test_a_write_is_tried_only_when_nothing_came_in_since_the_batch():
    a, ch, link, disarms = rig()
    serial = link.emergency(2.0)
    wrote = []
    assert send_boundary(ch, serial, lambda: wrote.append(1)) is True and wrote == [1]
    ch.message("c1", "op", msg(a, "stop"))  # after the gate, before the first byte
    assert send_boundary(ch, serial, lambda: wrote.append(2)) is False and wrote == [1]


def test_a_disarm_is_not_left_for_the_ordered_queue():
    a, ch, link, disarms = rig()
    ch.message("c1", "op", msg(a, "disarm"))
    conns, stops, ordered, latest = ch.take_cycle()
    assert not ordered  # it waits in the emergency batch only
    link.emergency(2.0)
    assert disarms


def test_the_cycles_begin_takes_the_emergency_batch_first():
    a, ch, link, disarms = rig()
    ch.message("c1", "op", msg(a, "disarm"))
    link.begin(2.0, None)
    assert disarms


def test_a_disarm_is_taken_even_when_the_ordered_queue_is_full():
    a, ch, link, disarms = rig()
    for k in range(ch.capacity):
        ch.message("c1", "op", {**msg(a, "echo"), "t": "targets_request", "request_id": f"q{k}"})
    assert ch.message("c1", "op", msg(a, "disarm")) is None  # not refused busy
    link.emergency(2.0)
    assert disarms
