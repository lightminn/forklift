"""M-b1: connections, the lease and the order a reliable request is handled in (plan B2-B4; contract 1, 2)."""

import itertools

from forklift_core.safety.arbiter import Arbiter, ArbiterConfig

EPOCH = 41


def arbiter(**config):
    counter = itertools.count(1)
    return Arbiter(EPOCH, ArbiterConfig(**config), lambda: f"tok{next(counter)}")


def lease(a, client="laptop", now=0.0, request_id="L1"):
    a.connect(client, now)
    reply = a.handle(client, {"t": "lease_request", "request_id": request_id, "session_epoch": EPOCH}, now)
    assert reply["t"] == "ack", reply
    return reply["lease_id"]


def leased(a, client, lease_id, kind, request_id, now, **fields):
    msg = {"t": kind, "request_id": request_id, "session_epoch": EPOCH, "lease_id": lease_id,
           "control_gen": a.control_gen, **fields}
    return a.handle(client, msg, now)


def test_one_holder_and_the_others_observe():
    a = arbiter()
    lease(a, "laptop")
    a.connect("phone", 0.0)
    reply = a.handle("phone", {"t": "lease_request", "request_id": "P1", "session_epoch": EPOCH}, 0.1)
    assert (reply["t"], reply["reason"]) == ("reject", "no_lease")


def test_a_wrong_epoch_is_refused_and_the_refusal_is_kept():
    a = arbiter()
    a.connect("laptop", 0.0)
    msg = {"t": "lease_request", "request_id": "L1", "session_epoch": EPOCH - 1}
    assert a.handle("laptop", msg, 0.0)["reason"] == "stale_epoch"
    msg["session_epoch"] = EPOCH  # the same request id is answered from the store, not run again
    assert a.handle("laptop", msg, 0.1)["reason"] == "stale_epoch" and a.lease is None


def test_a_resend_gets_the_original_answer_without_running_again():
    a = arbiter()
    lease_id = lease(a)
    first = leased(a, "laptop", lease_id, "lease_release", "R1", 0.2)
    assert first["t"] == "ack" and a.lease is None
    gen = a.control_gen
    assert leased(a, "laptop", lease_id, "lease_release", "R1", 0.3) == first and a.control_gen == gen


def test_stop_runs_every_time_from_any_connected_client_and_is_never_stored():
    a = arbiter(request_capacity=1)
    lease(a, "laptop")  # fills the store
    a.connect("phone", 0.0)
    stop = {"t": "stop", "request_id": "S1", "source": "gamepad"}
    assert a.handle("phone", stop, 0.1)["t"] == "ack" and a.mode == "STOPPED"
    gen = a.control_gen
    assert a.handle("phone", stop, 0.2)["t"] == "ack" and a.control_gen == gen + 1  # a boundary again
    assert a.requests.get("phone", "S1") is None


def test_an_observe_only_connection_cannot_stop_or_ask():
    a = arbiter()
    a.connect("monitor", 0.0, control=False)
    assert a.handle("monitor", {"t": "stop", "request_id": "S1"}, 0.1)["reason"] == "unsupported"
    assert a.mode == "PAUSED"


def test_a_full_store_refuses_new_requests_busy_without_storing_them():
    a = arbiter(request_capacity=1)
    lease_id = lease(a)
    reply = leased(a, "laptop", lease_id, "lease_release", "R1", 0.1)
    assert reply["reason"] == "busy" and a.lease is not None and a.requests.get("laptop", "R1") is None


def test_the_lease_expires_from_the_send_time_of_the_tick_its_last_reply_named():
    a = arbiter(lease_timeout_s=1.0)
    lease_id = lease(a)
    tick = a.tick(0.5)
    a.handle("laptop", {"t": "echo", "session_epoch": EPOCH, "lease_id": lease_id,
                        "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}}, 0.6)
    a.tick(1.5)
    assert a.lease is not None  # 1.5 <= 0.5 + 1.0
    gen = a.control_gen
    a.tick(1.5001)
    assert a.lease is None and a.control_gen == gen + 1 and a.reserved[0] == "laptop"


def test_a_reply_naming_an_unknown_token_does_not_renew():
    a = arbiter(lease_timeout_s=1.0)
    lease_id = lease(a)
    tick = a.tick(0.5)
    a.handle("laptop", {"t": "echo", "session_epoch": EPOCH, "lease_id": lease_id,
                        "echo": {"state_seq": tick["state_seq"], "token": "forged"}}, 0.6)
    a.tick(1.01)
    assert a.lease is None


def test_the_grace_keeps_the_lease_for_its_holder_with_a_new_lease_id():
    a = arbiter(grace_s=10.0)
    old = lease(a)
    a.disconnect("laptop", 1.0)
    a.connect("phone", 2.0)
    assert a.handle("phone", {"t": "lease_request", "request_id": "P1", "session_epoch": EPOCH},
                    2.0)["reason"] == "lease_reserved"
    new = lease(a, "laptop", 3.0, request_id="L2")
    assert new != old
    assert leased(a, "laptop", old, "lease_release", "R1", 3.1)["reason"] == "no_lease"


def test_after_the_grace_another_client_may_take_the_lease():
    a = arbiter(grace_s=10.0)
    lease(a)
    a.disconnect("laptop", 1.0)
    assert lease(a, "phone", 11.01, request_id="P1")


def test_a_request_sent_before_a_boundary_is_refused_after_it():
    a = arbiter()
    lease_id = lease(a)
    gen = a.control_gen
    a.handle("laptop", {"t": "stop", "request_id": "S1"}, 0.1)
    msg = {"t": "mode_request", "request_id": "M1", "session_epoch": EPOCH, "lease_id": lease_id,
           "control_gen": gen, "to": "TELEOP"}
    assert a.handle("laptop", msg, 0.2)["reason"] == "stale_control"


def test_recovery_is_not_offered_yet():
    a = arbiter()
    lease_id = lease(a)
    assert leased(a, "laptop", lease_id, "recovery", "X1", 0.1, action="withdraw_straight")["reason"] == "unsupported"
