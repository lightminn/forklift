"""CT-2 console plan C7: the fake vehicle answers pick_request (contract D3) for the screen's tests -- an ack, then a
separate pick message -- with a toy rule (the first available pallet, the first destination no pallet is verified at),
not the vehicle's next_pick rule. Injections change the next pick's reason."""

import itertools

import pytest

from console.mock_vehicle.core import Config, MockVehicle


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def v():
    tokens = (f"tok{i}" for i in itertools.count())
    return MockVehicle(Clock(), lambda: next(tokens), Config())


def ask(v, rid="p1", conn="c1"):
    v.connect("a", conn)
    reply = v.handle("a", {"t": "pick_request", "request_id": rid, "session_epoch": v.session_epoch}, conn_id=conn)
    return reply, v.take_picks(conn)


def test_welcome_announces_the_feature_and_its_deadline(v):
    caps = v.connect("a", "c1")["capabilities"]
    assert caps["next_pick"] is True and caps["next_pick_deadline_ms"] == 5000


def test_an_ack_then_a_pick_with_the_toy_rule(v):
    reply, picks = ask(v)
    assert reply["t"] == "ack" and reply["request_id"] == "p1"
    assert len(picks) == 1
    p = picks[0]
    assert p["t"] == "pick" and p["request_id"] == "p1" and p["reason"] == "ok"
    assert p["targets_revision"] == v.targets_revision
    first_pallet = sorted(q.pallet_id for q in v.world.pallets if q.state == "available")[0]
    first_dest = sorted(v.world.destinations)[0]
    assert p["pick"] == {"pallet_id": first_pallet, "destination_id": first_dest}
    assert p["plan"][0] == [first_pallet, first_dest] and p["missions"] >= 1
    assert v.take_picks("c1") == [], "taken once"


def test_none_remaining_and_no_free_destination(v):
    for q in v.world.pallets:
        q.state = "delivered"
    _, picks = ask(v, "p1")
    assert picks[0]["reason"] == "none_remaining" and picks[0]["pick"] is None
    v.world.pallets[0].state = "available"
    for q, d in zip(v.world.pallets[1:], sorted(v.world.destinations)):
        v.at[q.pallet_id] = d
    if len(v.world.pallets) - 1 < len(v.world.destinations):
        for d in sorted(v.world.destinations)[len(v.world.pallets) - 1:]:
            v.at[f"X-{d}"] = d
    _, picks = ask(v, "p2", "c2")
    assert picks[0]["reason"] == "no_free_destination"


def test_injections_change_the_next_pick_once(v):
    v.inject("pick_all_blocked")
    _, picks = ask(v, "p1")
    assert picks[0]["reason"] == "all_blocked" and picks[0]["pick"] is None and picks[0]["blocked"]
    _, picks = ask(v, "p2", "c2")
    assert picks[0]["reason"] == "ok", "one pick only"
    v.inject("pick_error")
    assert ask(v, "p3", "c3")[1][0]["reason"].startswith("error:")
    v.inject("pick_silent")
    reply, picks = ask(v, "p4", "c4")
    assert reply["t"] == "ack" and picks == [], "acked, no pick (the console's deadline)"
    v.inject("pick_stale_revision")
    assert ask(v, "p5", "c5")[1][0]["targets_revision"] == v.targets_revision - 1


def test_a_resend_that_gets_the_stored_ack_gets_a_newly_computed_pick(v):
    """Contract 5.3 (Codex CT-2 r1 P2): the stored ack goes back and the pick is computed again."""
    v.connect("a", "c1")
    msg = {"t": "pick_request", "request_id": "p1", "session_epoch": v.session_epoch}
    first = v.handle("a", dict(msg), conn_id="c1")
    assert len(v.take_picks("c1")) == 1
    v.targets_revision += 1
    again = v.handle("a", dict(msg), conn_id="c1")
    assert again == first
    picks = v.take_picks("c1")
    assert len(picks) == 1 and picks[0]["targets_revision"] == v.targets_revision


def test_a_resend_through_the_request_ledger_gets_a_pick_too():
    tokens = (f"tok{i}" for i in itertools.count())
    v = MockVehicle(Clock(), lambda: next(tokens), Config(request_tokens=True))
    w = v.connect("a", "c1")
    msg = {"t": "pick_request", "request_id": "p1", "session_epoch": v.session_epoch,
           "request_token": w["request_token"], "request_seq": 1}
    assert v.handle("a", dict(msg), conn_id="c1")["t"] == "ack"
    assert len(v.take_picks("c1")) == 1
    assert v.handle("a", dict(msg), conn_id="c1")["t"] == "ack"
    assert len(v.take_picks("c1")) == 1


def test_refused_while_a_mission_runs(v):
    v.connect("a", "c1")
    v.mission = {"mission_id": "M-0001", "status": "running"}
    reply = v.handle("a", {"t": "pick_request", "request_id": "p1", "session_epoch": v.session_epoch}, conn_id="c1")
    assert reply["t"] == "reject" and reply["reason"] == "mission_active"
    assert v.take_picks("c1") == []
