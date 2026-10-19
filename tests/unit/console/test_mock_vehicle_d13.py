"""Persistent console plan D13 item 8: the fake vehicle behaves as the live runner for the screen's D13 work -- the
shared request ledger and tokens, the results book, the scene, the session end and reset, and injected states."""

import itertools

import pytest

from console.mock_vehicle.core import Config, MockVehicle


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def env():
    clock = Clock()
    tokens = (f"tok{i}" for i in itertools.count())
    return MockVehicle(clock, lambda: next(tokens), Config()), clock


class Client:
    """One connection with its request token and seq, as the page sends (contract 1, the request ledger)."""

    def __init__(self, v, cid="a", conn="c1"):
        self.v, self.cid, self.conn = v, cid, conn
        self.welcome = v.connect(cid, conn)
        self.token, self.seq, self.n = self.welcome.get("request_token"), 0, 0
        self.lease_id = None

    def send(self, t, *, seq=None, **fields):
        self.n += 1
        if seq is None:
            self.seq += 1
            seq = self.seq
        msg = {"t": t, "request_id": f"{self.cid}{self.n}", "session_epoch": self.v.session_epoch,
               "request_token": self.token, "request_seq": seq, **fields}
        if self.lease_id is not None:
            msg.update(lease_id=self.lease_id, control_gen=self.v.control_gen)
        return self.v.handle(self.cid, msg, conn_id=self.conn)

    def take_lease(self):
        ack = self.send("lease_request")
        assert ack["t"] == "ack", ack
        self.lease_id = ack["lease_id"]


def stand(v, clock, seconds=1.0, client=None):
    """Time passing; the lease holder echoes every tick as the page does (else the lease expires)."""
    for _ in range(int(seconds / 0.05)):
        clock.t += 0.05
        tick = v.make_tick()
        if v.lease is not None:
            holder = v.lease["client_id"]
            v.handle(holder, {"t": "echo", "session_epoch": v.session_epoch, "lease_id": v.lease["lease_id"],
                              "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}},
                     conn_id=v.lease["conn_id"])
        v.step(0.05)


def test_a_connection_gets_a_request_token_and_its_requests_go_through_the_ledger(env):
    v, clock = env
    c = Client(v)
    assert isinstance(c.token, int)
    c.take_lease()
    first = c.send("mode_request", to="PAUSED")
    again = c.send("mode_request", seq=c.seq, to="PAUSED")  # a resend: the stored answer
    assert first["t"] == "ack" and again == first
    other = Client(v, "a", "c2")  # a reconnect retires the old token
    assert c.send("disarm")["reason"] == "stale"
    assert isinstance(other.token, int) and other.token != c.token


def test_a_stale_epoch_request_is_refused_before_the_ledger(env):
    v, _ = env
    c = Client(v)
    msg = {"t": "lease_request", "request_id": "x", "session_epoch": v.session_epoch + 1,
           "request_token": c.token, "request_seq": 1}
    assert v.handle("a", msg, conn_id="c1")["reason"] == "stale_epoch"
    assert c.send("lease_request", seq=1)["t"] == "ack"  # the same seq was not taken


def submit(c, v, pallet="P-01", dest="D-A"):
    r = c.send("mission_submit", pallet_id=pallet, destination_id=dest, targets_revision=v.targets_revision)
    assert r["t"] == "ack", r
    return r["mission_id"]


def test_a_mission_end_is_booked_published_and_in_the_state(env):
    v, clock = env
    c = Client(v)
    c.take_lease()
    mid = submit(c, v)
    assert c.send("mission_cancel", mission_id=mid)["t"] == "ack"
    stand(v, clock, 0.2)
    books = [e for e in v.events if e.get("t") == "missions"]
    assert books and books[-1]["missions_seq"] == 1 and books[-1]["results"][0]["mission_id"] == mid
    r = books[-1]["results"][0]
    assert r["outcome"] == "cancelled" and isinstance(r["ended_unix"], float)
    st = v.make_state()
    assert st["last_result"]["mission_id"] == mid and st["scene"] == {"damaged": None}
    assert st["feedback_ok"] is True
    assert v.make_missions()["missions_seq"] == 1  # what a page connecting later gets


def test_the_session_requests_follow_the_arbiters_order(env):
    v, clock = env
    c = Client(v)
    c.take_lease()
    stand(v, clock)
    mid = submit(c, v)
    assert c.send("session_end")["reason"] == "mission_active"
    c.send("mission_cancel", mission_id=mid)
    stand(v, clock)
    assert c.send("session_end")["t"] == "ack" and v.session_request == "end"


def test_a_scene_reset_starts_a_new_session(env):
    v, clock = env
    c = Client(v)
    c.take_lease()
    stand(v, clock)
    epoch = v.session_epoch
    v.inject("scene_damage")
    assert v.make_state()["scene"]["damaged"]["reason"]
    assert c.send("sim_reset")["t"] == "ack" and v.session_request == "reset"
    v.reset_session()
    assert v.session_epoch != epoch and v.session_request is None
    assert v.make_state()["scene"] == {"damaged": None} and v.make_missions()["missions_seq"] == 0


def test_the_injections_give_the_screens_cases(env):
    v, clock = env
    c = Client(v)
    c.take_lease()
    stand(v, clock)
    v.inject("recovery_required")
    st = v.make_state()
    assert st["recovery"]["stage"] == "in_pocket" and st["last_result"]["outcome"] == "recovery_required"
    v.inject("steering_wait")
    assert v.make_state()["permit"]["reason"] == "recovery:steering"
    v.inject("invalid_start")
    last = v.make_state()["last_result"]
    assert last["outcome"] == "failed" and last["advice"]["backoff_m"] == pytest.approx(0.15)
    with pytest.raises(ValueError):
        v.inject("nonsense")
