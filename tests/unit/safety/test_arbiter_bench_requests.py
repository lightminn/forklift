"""Bench console plan 3.1 (B1): the bench session's four requests through the arbiter. Off (the default: Isaac, REAL,
HIL, the teleop bridge) they are not reliable requests -- no answer, nothing reaches a callback. On, they are leased
reliable requests through the ledger only (the connection's token and seq: a resend gets the stored result, never a
second run), answered by ``bench_request``."""

import pytest

from forklift_core.safety.arbiter import BENCH_LEASED

from test_arbiter_lease import EPOCH, arbiter, lease

TOKEN = 7001


def bench(**config):
    a = arbiter(bench_requests=True, **config)
    calls = []

    def answer(kind, msg, now):
        calls.append((kind, dict(msg)))
        return None, {"run": len(calls)}

    a.bench_request = answer
    return a, calls


def tokened_lease(a, client="laptop", conn="c1"):
    a.connect(client, 0.0, conn_id=conn, token=TOKEN)
    reply = a.handle(client, {"t": "lease_request", "request_id": "L1", "session_epoch": EPOCH}, 0.0, conn_id=conn)
    assert reply["t"] == "ack", reply
    return reply["lease_id"]


def req(a, kind, lease_id, rid, now, *, seq, token=TOKEN, client="laptop", conn="c1", **fields):
    msg = {"t": kind, "request_id": rid, "session_epoch": EPOCH, "lease_id": lease_id, "control_gen": a.control_gen,
           **fields}
    if token is not None:
        msg.update(request_token=token, request_seq=seq)
    return a.handle(client, msg, now, conn_id=conn)


def test_the_four_names():
    assert BENCH_LEASED == ("bench_run", "bench_stop", "bench_disarm", "clear_faults")


@pytest.mark.parametrize("kind", BENCH_LEASED)
def test_off_no_answer_and_no_callback(kind):
    a = arbiter()
    called = []
    a.bench_request = lambda *x: called.append(x) or (None, {})
    lease_id = lease(a)
    assert req(a, kind, lease_id, "B1", 0.1, seq=1, token=None) is None
    assert called == []


@pytest.mark.parametrize("kind", BENCH_LEASED)
def test_on_answered_through_the_callback(kind):
    a, calls = bench()
    lease_id = tokened_lease(a)
    reply = req(a, kind, lease_id, "B1", 0.1, seq=1, axis="L")
    assert reply["t"] == "ack" and reply["of"] == kind and reply["run"] == 1
    assert calls[0][0] == kind and calls[0][1]["axis"] == "L"


def test_on_a_resend_gets_the_stored_result_never_a_second_run():
    a, calls = bench()
    lease_id = tokened_lease(a)
    first = req(a, "bench_run", lease_id, "B1", 0.1, seq=1)
    again = req(a, "bench_run", lease_id, "B1", 0.2, seq=1)
    assert again == first and len(calls) == 1


def test_on_without_a_token_refused():
    a, calls = bench()
    lease_id = tokened_lease(a)
    assert req(a, "bench_run", lease_id, "B1", 0.1, seq=1, token=None)["reason"] == "token_required"
    assert calls == []


@pytest.mark.parametrize("why,kw", [("no_lease", dict(lease_id="nope")), ("stale_epoch", dict(session_epoch=1))])
def test_on_the_usual_lease_and_session_checks(why, kw):
    a, calls = bench()
    lease_id = tokened_lease(a)
    msg = {"t": "bench_run", "request_id": "B1", "session_epoch": EPOCH, "lease_id": lease_id,
           "control_gen": a.control_gen, "request_token": TOKEN, "request_seq": 1, **kw}
    reply = a.handle("laptop", msg, 0.1, conn_id="c1")
    assert reply["t"] == "reject" and reply["reason"] in (why, "stale_lease"), reply
    assert calls == []


def test_on_no_callback_unsupported():
    a = arbiter(bench_requests=True)
    lease_id = tokened_lease(a)
    assert req(a, "bench_run", lease_id, "B1", 0.1, seq=1)["reason"] == "unsupported"


def test_on_a_callback_refusal_is_the_reply_reason():
    a, _ = bench()
    a.bench_request = lambda kind, msg, now: ("busy_running", {"detail": "x"})
    lease_id = tokened_lease(a)
    reply = req(a, "bench_run", lease_id, "B1", 0.1, seq=1)
    assert reply["t"] == "reject" and reply["reason"] == "busy_running" and reply["detail"] == "x"


def test_on_the_stop_still_comes_first_and_is_counted():
    a, _ = bench()
    tokened_lease(a)
    before = a.stops
    a.handle("laptop", {"t": "stop", "request_id": "S1", "source": "console"}, 0.1, conn_id="c1")
    assert a.stops == before + 1


def test_release_stop_only_for_the_bench_and_only_from_stopped():
    a, _ = bench()
    tokened_lease(a)
    assert not a.release_stop(0.1, "bench_stop_confirmed")  # PAUSED: nothing to release
    a.stop(0.2, "stop:console")
    gen = a.control_gen
    assert a.mode == "STOPPED" and a.release_stop(0.3, "bench_stop_confirmed")
    assert a.mode == "PAUSED" and a.control_gen == gen + 1  # one more boundary
    off = arbiter()
    off.stop(0.1, "stop:console")
    assert not off.release_stop(0.2, "x") and off.mode == "STOPPED"


def test_a_request_made_before_a_stop_is_stale_after_it():
    """The next run opens only on a request of the current control generation (plan 3.1)."""
    a, calls = bench()
    lease_id = tokened_lease(a)
    old_gen = a.control_gen
    a.stop(0.2, "stop:console")
    msg = {"t": "bench_run", "request_id": "B9", "session_epoch": EPOCH, "lease_id": lease_id,
           "control_gen": old_gen, "request_token": TOKEN, "request_seq": 9}
    assert a.handle("laptop", msg, 0.3, conn_id="c1")["reason"] == "stale_control" and calls == []


def test_a_start_taken_before_a_shared_disarm_but_handled_after_it_is_stale():
    """bench_run(seq 2) then disarm(seq 3) in one cycle: the DISARM is handled first (emergency batch) and moves the
    generation, so the start is refused (Codex B1 design round 2 P1-1)."""
    a, calls = bench()
    lease_id = tokened_lease(a)
    gen = a.control_gen
    a.handle("laptop", {"t": "disarm", "request_id": "D1", "session_epoch": EPOCH, "lease_id": lease_id,
                        "control_gen": gen}, 0.1, conn_id="c1")
    assert a.control_gen == gen + 1
    msg = {"t": "bench_run", "request_id": "B2", "session_epoch": EPOCH, "lease_id": lease_id, "control_gen": gen,
           "request_token": TOKEN, "request_seq": 2}
    assert a.handle("laptop", msg, 0.1, conn_id="c1")["reason"] == "stale_control" and calls == []


def test_off_a_disarm_keeps_the_generation():
    a = arbiter()
    lease_id = lease(a)
    gen = a.control_gen
    a.handle("laptop", {"t": "disarm", "request_id": "D1", "session_epoch": EPOCH, "lease_id": lease_id,
                        "control_gen": gen}, 0.1)
    assert a.control_gen == gen
