"""M-c2: the channels between the console server and the runner, and the link's cycle (plan C2)."""

import itertools
import threading

from forklift_core.live.link import Channels, Link
from forklift_core.mission.commands import Context, MissionCommands
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback

EPOCH = 5
SAFE = {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
        "frame": {"hold": "none", "slam": True}, "mission": {"phase": "observe"}}


def rig(capacity=256):
    counter = itertools.count(1)
    m = MissionCommands()
    a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(counter)}", mission=m)
    ch = Channels(capacity)
    return m, a, ch, Link(a, ch)


def cycle(m, link, t):
    stop_kind = link.begin(t, Feedback(t, 0.0))
    records = m.apply(t, Context(stop_kind=stop_kind))
    if m.status == "accepted":
        m.start()
    return link.end(t, records, SAFE, (0.0, 0.0))


def out_for(ch, conn):
    return [msg for c, msg in ch.take_out() if c == conn]


def test_a_stop_is_never_dropped_and_the_gate_takes_it_between_cycles():
    m, a, ch, link = rig(capacity=1)
    ch.connect("A", "op")
    cycle(m, link, 0.01)
    ch.message("A", "op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH})
    assert ch.message("A", "op", {"t": "lease_request", "request_id": "L2", "session_epoch": EPOCH})["reason"] == "busy"
    ch.message("A", "op", {"t": "stop", "request_id": "S"})  # a full channel does not refuse a STOP
    assert link.gate(0.02) is True and a.mode == "STOPPED"
    assert ("ack", "stop") in [(r["t"], r["of"]) for r in out_for(ch, "A")]


def test_latest_values_keep_only_the_last_per_connection():
    m, a, ch, link = rig()
    ch.connect("A", "op")
    for k in range(5):
        ch.message("A", "op", {"t": "echo", "k": k})
    _, _, _, latest = ch.take_cycle()
    assert [msg["k"] for _, _, msg in latest] == [4]


def test_mission_replies_go_to_the_current_connection_and_an_old_close_is_ignored():
    m, a, ch, link = rig()
    ch.connect("A", "op")
    for k in range(40):
        cycle(m, link, 0.01 * (k + 1))
    ch.message("A", "op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH})
    cycle(m, link, 0.5)
    assert next(r for r in out_for(ch, "A") if r.get("of") == "lease_request")["t"] == "ack"
    ch.connect("B", "op")  # the same client reconnects: the old lease goes with the old connection
    ch.disconnect("A", "op")  # the old socket closes late
    ch.message("B", "op", {"t": "lease_request", "request_id": "L2", "session_epoch": EPOCH})
    cycle(m, link, 0.505)
    lease_id = next(r for r in out_for(ch, "B") if r.get("of") == "lease_request")["lease_id"]
    assert a.conns["op"] == "B"
    ch.message("B", "op", {"t": "mission_submit", "request_id": "M", "session_epoch": EPOCH, "lease_id": lease_id,
                           "control_gen": a.control_gen, "pallet_id": "P", "destination_id": "D",
                           "targets_revision": 0})
    cycle(m, link, 0.51)
    replies = out_for(ch, "B")
    assert any(r.get("of") == "mission_submit" and r["t"] == "ack" for r in replies) and a.mode == "AUTO"


def test_the_channels_take_puts_from_another_thread():
    ch = Channels()
    woke = threading.Event()
    ch.notify = woke.set
    threading.Thread(target=lambda: ch.put("A", {"t": "x"})).start()
    assert woke.wait(1.0) and ch.take_out() == [("A", {"t": "x"})]


def test_ticks_are_broadcast_at_their_period():
    m, a, ch, link = rig()
    ch.connect("A", "op")
    for k in range(14):
        cycle(m, link, 0.01 * (k + 1))
    ticks = [msg for c, msg in ch.take_out() if c is None and msg["t"] == "tick"]
    assert len(ticks) == 3  # about 0.01, 0.06, 0.11 (or the cycle just after)



def test_a_stop_right_after_the_welcome_finds_its_client():
    """Codex M-c round 2 P1: the connection is registered before the STOP channel is taken."""
    m, a, ch, link = rig()
    ch.connect("A", "op")
    ch.message("A", "op", {"t": "stop", "request_id": "S"})
    assert link.gate(0.01) is True
    assert [r["t"] for c, r in ch.take_out() if c == "A"] == ["ack"]


def test_a_full_ordered_channel_still_takes_connections():
    m, a, ch, link = rig(capacity=1)
    ch.connect("A", "op")
    ch.message("A", "op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH})
    ch.connect("B", "pad")
    cycle(m, link, 0.01)
    assert set(a.clients) == {"op", "pad"}
