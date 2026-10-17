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
        ch.message("A", "op", {"t": "echo", "session_epoch": EPOCH, "lease_id": "x",
                               "echo": {"state_seq": k, "token": "t"}})
    _, _, _, latest = ch.take_cycle()
    assert [msg["echo"]["state_seq"] for _, _, msg in latest] == [4]


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


def test_a_mission_reply_goes_only_to_the_connection_that_asked():
    """Codex M-c review round 3 P2: A asked, B replaced A before the verdict -- B does not get A's answer."""
    m, a, ch, link = rig()
    ch.connect("A", "op")
    for k in range(40):
        cycle(m, link, 0.01 * (k + 1))
    ch.message("A", "op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH})
    cycle(m, link, 0.5)
    lease_id = next(r for r in out_for(ch, "A") if r.get("of") == "lease_request")["lease_id"]
    ch.message("A", "op", {"t": "mission_submit", "request_id": "M", "session_epoch": EPOCH, "lease_id": lease_id,
                           "control_gen": a.control_gen, "pallet_id": "P", "destination_id": "D",
                           "targets_revision": 0})
    stop_kind = link.begin(0.51, Feedback(0.51, 0.0))  # A's submit queued for this tick
    records = m.apply(0.51, Context(stop_kind=stop_kind))
    ch.connect("B", "op")  # B arrives during the mission's tick
    link.stops(0.511, "gate")
    link.end(0.512, records, SAFE, (0.0, 0.0))
    assert not any(r.get("of") == "mission_submit" for c, r in ch.take_out() if c in ("A", "B"))
    assert a.requests.get("op", "M")["t"] == "ack"  # stored: A's resend gets it


def test_a_bad_message_is_refused_and_logged_not_raised():
    m, a, ch, link = rig()
    ch.connect("A", "op")
    assert ch.message("A", "op", {"t": "echo", "echo": "bad"}) is None  # a latest value: dropped
    reply = ch.message("A", "op", {"t": "lease_request", "request_id": "L", "session_epoch": "41"})
    assert (reply["t"], reply["reason"]) == ("reject", "invalid")  # refused at the channel's door
    cycle(m, link, 0.01)
    assert not [r for c, r in ch.take_out() if c == "A" and r.get("t") == "reject"]


def test_a_malformed_duplicate_does_not_replace_a_valid_stop():
    """Codex M-c review round 4 P1."""
    m, a, ch, link = rig()
    ch.connect("A", "op")
    ch.message("A", "op", {"t": "stop", "request_id": "S", "source": "key"})
    assert ch.message("A", "op", {"t": "stop", "request_id": "S", "source": 0})["reason"] == "invalid"
    assert ch.message("A", "op", {"t": "stop", "request_id": []}) is None  # unhashable: refused, not raised
    assert link.gate(0.01) is True and a.mode == "STOPPED"


def test_a_targets_request_gets_the_current_list():
    from forklift_core.mission.targets import Pose, TargetsBook

    m, a, ch, link = rig()
    m.targets = TargetsBook()
    m.targets.add_pallet("P-1", "epal6", Pose(1.0, 0.0, 0.0))
    ch.connect("A", "op")
    ch.message("A", "op", {"t": "targets_request", "request_id": "T", "session_epoch": EPOCH})
    cycle(m, link, 0.01)
    kinds = [r["t"] for c, r in ch.take_out() if c == "A"]
    assert kinds == ["ack", "targets"]
