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
    return MockVehicle(clock, lambda: next(tokens), Config(request_tokens=True)), clock


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


def test_without_request_tokens_configured_the_old_store_is_kept():
    clock = Clock()
    v = MockVehicle(clock, lambda: "t", Config())
    assert "request_token" not in v.connect("a", "c1")


def test_a_connection_with_a_token_must_send_it(env):
    """Codex D13 impl round 2 P3: the live runner refuses a tokenless request (request_tokens=True)."""
    v, _ = env
    Client(v)
    msg = {"t": "lease_request", "request_id": "x", "session_epoch": v.session_epoch}
    assert v.handle("a", msg, conn_id="c1")["reason"] == "invalid"


def test_a_session_request_is_taken_while_stopped_and_closes_everything_else(env):
    """Codex D13 impl round 2 P2: STOPPED does not lock it out (the arbiter allows it); after the ack every other
    request is session_ending."""
    v, clock = env
    c = Client(v)
    c.take_lease()
    v._stop("console")
    stand(v, clock)
    assert v.mode == "STOPPED"
    assert c.send("session_end")["t"] == "ack"
    r = c.send("mission_submit", pallet_id="P-01", destination_id="D-A", targets_revision=v.targets_revision)
    assert r["reason"] == "session_ending"


def test_a_cancelled_mission_is_booked_before_the_next_one_replaces_it(env):
    """Codex D13 impl round 2 P2: submit M1, cancel, submit M2 at once -- M1's result was lost."""
    v, clock = env
    c = Client(v)
    c.take_lease()
    m1 = submit(c, v)
    c.send("mission_cancel", mission_id=m1)
    stand(v, clock, 0.6)
    submit(c, v)
    stand(v, clock, 0.1)
    assert [r["mission_id"] for r in v.make_missions()["results"]][:1] == [m1]


def test_a_damaged_scene_refuses_automatic_work(env):
    """Codex D13 impl round 2 P2: the screen says automatic work is refused; the vehicle must agree."""
    v, clock = env
    c = Client(v)
    c.take_lease()
    v.inject("scene_damage")
    r = c.send("mission_submit", pallet_id="P-01", destination_id="D-A", targets_revision=v.targets_revision)
    assert r["reason"] == "scene_damaged"


def test_the_recovery_injections_carry_the_policy_the_guidance_reads(env):
    """Codex D13 impl round 2 P3: lift_dirs, drive_mode and the stand waiver, as the live state sends them."""
    v, clock = env
    v.inject("recovery_required")
    st = v.make_state()
    assert st["lift_dirs"] == [-1] and st["drive_mode"] == "normal" and st["stand_waiver"] == "lower_only"
    assert st["teleop_forbidden_reason"] == "recovery:lower_first"
    v.inject("steering_wait")
    st = v.make_state()
    assert st["drive_mode"] == "reverse_straight" and st["lift_dirs"] == [] and st["stand_waiver"] is None


def test_a_delivered_pallet_can_go_elsewhere_but_not_where_it_already_is(env):
    """Codex D3 round 1 P2-4: the fake vehicle refused delivered pallets invalid; plan D3: already_there."""
    v, clock = env
    c = Client(v)
    c.take_lease()
    p = v.world.pallets[0]
    p.state, p.pose = "delivered", v.world.destinations["D-A"]
    v.at[p.pallet_id] = "D-A"
    v.targets_revision += 1
    r = c.send("mission_submit", pallet_id=p.pallet_id, destination_id="D-A", targets_revision=v.targets_revision)
    assert r["reason"] == "already_there"
    other = next(d for d in v.world.destinations if d != "D-A")
    r = c.send("mission_submit", pallet_id=p.pallet_id, destination_id=other, targets_revision=v.targets_revision)
    assert r["t"] == "ack", r


def test_a_destination_another_pallet_stands_on_is_occupied(env):
    v, clock = env
    c = Client(v)
    c.take_lease()
    p1, p2 = v.world.pallets[0], v.world.pallets[1]
    p2.pose = v.world.destinations["D-A"]
    v.targets_revision += 1
    r = c.send("mission_submit", pallet_id=p1.pallet_id, destination_id="D-A", targets_revision=v.targets_revision)
    assert r["reason"] == "destination_occupied"


def test_the_fake_vehicle_verifies_the_place_only_at_done(env):
    """Codex D3 impl round 1 P3: it set the place when the lowering ended; cancelled after that, the same
    destination was refused already_there while the live runner takes it."""
    v, clock = env
    c = Client(v)
    c.take_lease()
    pallet = v.world.pallets[0]
    mid = submit(c, v, pallet=pallet.pallet_id, dest="D-A")
    for _ in range(4000):  # until the lowering has ended, before done
        clock.t += 0.05
        v.step(0.05)
        if pallet.state == "delivered" or v.mission["status"] == "done":
            break
    assert pallet.state == "delivered" and v.mission["status"] != "done"
    assert pallet.pallet_id not in v.at


def test_the_verified_place_at_done_is_a_new_targets_revision(env):
    v, clock = env
    c = Client(v)
    c.take_lease()
    pallet = v.world.pallets[0]
    submit(c, v, pallet=pallet.pallet_id, dest="D-A")
    before = None
    for _ in range(8000):
        clock.t += 0.05
        v.step(0.05)
        if pallet.state == "delivered" and before is None:
            before = v.targets_revision
        if v.mission["status"] == "done":
            break
    assert v.mission["status"] == "done" and v.targets_revision > before


def test_fail_running_ends_the_running_mission_as_failed(env):
    """Mission queue plan Task 3: the screen's queue test needs a mission that fails while it runs."""
    v, clock = env
    c = Client(v)
    c.take_lease()
    stand(v, clock)
    with pytest.raises(ValueError):
        v.inject("fail_running")  # nothing runs yet
    mid = submit(c, v)
    stand(v, clock, 0.2)
    v.inject("fail_running")
    stand(v, clock, 0.2)
    st = v.make_state()
    assert st["mode"] == "PAUSED" and st["mission"]["status"] == "failed"
    assert st["last_result"]["mission_id"] == mid and st["last_result"]["outcome"] == "failed"
    assert st["last_result"]["reason"] == "test:fail_running"
    assert v.make_missions()["results"][0]["mission_id"] == mid
