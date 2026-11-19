"""docs/design/2026-11-19-real-safety-wiring.md 5 (stage 5c-1): the real console requests through the arbiter. With
``real_requests`` off (the default, the teleop bridge and Isaac) nothing changes: the new names are not reliable
requests (no answer) and ``recovery`` stays unsupported. On, they are leased reliable requests answered by the loop's
``real_request`` callback after the usual session, lease and generation checks, stored like any other."""

import pytest

from forklift_core.safety.arbiter import REAL_LEASED

from test_arbiter_lease import EPOCH, arbiter, lease


TOKEN = 7001
_SEQ = iter(range(1, 10_000))


def leased(a, client, lease_id, request_kind, request_id, now, *, token=TOKEN, seq=None, **fields):
    """test_arbiter_lease.leased with the request's own name apart (operator_answer has a ``kind`` field) and, for
    the real requests, the connection's request token (5c-1: through the ledger only)."""
    msg = {"t": request_kind, "request_id": request_id, "session_epoch": EPOCH, "lease_id": lease_id,
           "control_gen": a.control_gen, **fields}
    if token is not None and (request_kind in REAL_LEASED or request_kind == "recovery"):
        msg.update(request_token=token, request_seq=next(_SEQ) if seq is None else seq)
    return a.handle(client, msg, now)


def tokened_lease(a, client="laptop"):
    a.connect(client, 0.0, conn_id="c1", token=TOKEN)
    reply = a.handle(client, {"t": "lease_request", "request_id": "L1", "session_epoch": EPOCH}, 0.0, conn_id="c1")
    assert reply["t"] == "ack", reply
    return reply["lease_id"]

PAYLOADS = {
    "pallet_register": {"pallet_id": "P1", "pose": [3.0, 0.0, 0.0]},
    "operator_start": {},
    "operator_answer": {"kind": "lifted", "serial": 1, "verdict": "confirm"},
    "check_bypass": {"check": "imu", "on": True},
    "relocalize_confirm": {"generation": 1},  # 5c-2 (design 6)
    "relocalize_declare": {"generation": 1},
    "recovery": {"recovery_id": 1, "action": "done"},
}


def real(**config):
    a = arbiter(real_requests=True, **config)
    calls = []

    def answer(kind, msg, now):
        calls.append((kind, dict(msg)))
        return None, {"echo": kind}

    a.real_request = answer
    return a, calls


@pytest.mark.parametrize("kind", REAL_LEASED)
def test_off_the_new_names_get_no_answer(kind):
    a = arbiter()
    lease_id = lease(a)
    assert leased(a, "laptop", lease_id, kind, "X1", 0.1, token=None, **PAYLOADS[kind]) is None


def test_off_recovery_stays_unsupported():
    a = arbiter()
    lease_id = lease(a)
    assert leased(a, "laptop", lease_id, "recovery", "X1", 0.1, token=None,
                  **PAYLOADS["recovery"])["reason"] == "unsupported"


@pytest.mark.parametrize("kind", REAL_LEASED + ("recovery",))
def test_on_the_loop_answers_after_the_checks(kind):
    a, calls = real()
    lease_id = tokened_lease(a)
    reply = leased(a, "laptop", lease_id, kind, "X1", 0.1, seq=1, **PAYLOADS[kind])
    assert reply["t"] == "ack" and reply["echo"] == kind and calls and calls[0][0] == kind
    # stored under the token's sequence: the same request answers again without running the callback
    assert leased(a, "laptop", lease_id, kind, "X1", 0.2, seq=1, **PAYLOADS[kind]) == reply and len(calls) == 1


@pytest.mark.parametrize("kind", REAL_LEASED + ("recovery",))
def test_on_a_real_request_without_a_token_is_refused_unstored(kind):
    a, calls = real()
    lease_id = tokened_lease(a)
    reply = leased(a, "laptop", lease_id, kind, "X1", 0.1, token=None, **PAYLOADS[kind])
    assert reply["reason"] == "token_required" and not calls
    assert leased(a, "laptop", lease_id, kind, "X1", 0.2, seq=5, **PAYLOADS[kind])["t"] == "ack"  # not stored


def test_on_another_connections_token_is_stale():
    a, calls = real()
    lease_id = tokened_lease(a)
    reply = leased(a, "laptop", lease_id, "check_bypass", "X1", 0.1, token=TOKEN + 1, **PAYLOADS["check_bypass"])
    assert reply["reason"] == "stale" and not calls


@pytest.mark.parametrize("kind", REAL_LEASED + ("recovery",))
def test_on_a_request_without_the_lease_or_on_an_old_generation_never_reaches_the_loop(kind):
    a, calls = real()
    lease_id = tokened_lease(a)
    a.connect("phone", 0.0, conn_id="c2", token=TOKEN + 5)
    msg = {"t": kind, "request_id": "P1", "session_epoch": EPOCH, "lease_id": lease_id, "control_gen": a.control_gen,
           "request_token": TOKEN + 5, "request_seq": 1, **PAYLOADS[kind]}
    assert a.handle("phone", msg, 0.1, conn_id="c2")["reason"] == "no_lease"
    stale = dict(msg, request_id="X2", control_gen=a.control_gen - 1, request_token=TOKEN, request_seq=2)
    assert a.handle("laptop", stale, 0.1, conn_id="c1")["reason"] == "stale_control"
    assert not calls


def test_on_a_refusal_from_the_loop_is_a_reject_with_its_reason():
    a = arbiter(real_requests=True)
    a.real_request = lambda kind, msg, now: ("mission_active", {})
    lease_id = tokened_lease(a)
    reply = leased(a, "laptop", lease_id, "pallet_register", "X1", 0.1, **PAYLOADS["pallet_register"])
    assert (reply["t"], reply["reason"]) == ("reject", "mission_active")


def test_on_without_a_callback_is_unsupported():
    a = arbiter(real_requests=True)
    lease_id = tokened_lease(a)
    reply = leased(a, "laptop", lease_id, "operator_start", "X1", 0.1)
    assert reply["reason"] == "unsupported"


@pytest.mark.parametrize("kind,bad", [
    ("pallet_register", {"pallet_id": "", "pose": [3.0, 0.0, 0.0]}),
    ("pallet_register", {"pallet_id": "P1", "pose": [3.0, 0.0]}),
    ("pallet_register", {"pallet_id": "P1", "pose": [3.0, 0.0, float("nan")]}),
    ("pallet_register", {"pallet_id": "P1", "pose": [3.0, 0.0, 10 ** 400]}),
    ("operator_answer", {"kind": "lifted", "serial": True, "verdict": "confirm"}),
    ("operator_answer", {"kind": "lifted", "serial": 1, "verdict": "maybe"}),
    ("check_bypass", {"check": "imu", "on": 1}),
    ("check_bypass", {"check": 3, "on": True}),
    ("recovery", {"recovery_id": 1, "action": "withdraw_straight"}),
    ("recovery", {"recovery_id": "1", "action": "done"}),
    ("relocalize_confirm", {"generation": True}),
    ("relocalize_confirm", {"generation": "1"}),
    ("relocalize_declare", {}),
])
def test_on_a_malformed_request_is_invalid_and_never_reaches_the_loop(kind, bad):
    a, calls = real()
    lease_id = tokened_lease(a)
    assert leased(a, "laptop", lease_id, kind, "X1", 0.1, **bad)["reason"] == "invalid"
    assert not calls


def test_on_the_ledger_is_purged_without_request_tokens_on():
    """5c-1 r2 P2-4: the real requests use the ledger, so it is purged each cycle with real_requests alone."""
    a, calls = real()
    lease_id = tokened_lease(a)
    seen = []
    a.ledger.purge = lambda now: seen.append(now)
    a.mission_context(1.0)  # the cycle's start
    assert seen == [1.0]
