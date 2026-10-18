"""The request ledger (persistent console plan D7, stage 4c): a never-reused token per connection, a per-token seq,
results purged after the resend window with a watermark -- a forgotten request is refused stale, never run again."""

import pytest

from forklift_core.safety.generations import PENDING, RequestLedger


def ledger(**k):
    book = RequestLedger(**k)
    a = book.issue()
    book.register(a, "A", "c1", now=0.0)
    return book, a


def test_a_new_request_is_run_and_its_resend_gets_the_stored_result():
    book, a = ledger()
    assert book.admit(a, "A", "c1", 1) == ("run", None)
    book.put(a, 1, {"t": "ack"}, now=1.0)
    assert book.admit(a, "A", "c1", 1) == ("stored", {"t": "ack"})


def test_a_purged_request_is_stale_and_never_run_again():
    book, a = ledger(window_s=60.0)
    for seq in (1, 2, 3):
        assert book.admit(a, "A", "c1", seq)[0] == "run"
        book.put(a, seq, {"seq": seq}, now=float(seq))
    book.purge(now=62.5)  # seq 1 and 2 are older than the window
    assert book.watermark(a) == 2
    assert book.admit(a, "A", "c1", 1) == ("stale", None) and book.admit(a, "A", "c1", 2) == ("stale", None)
    assert book.admit(a, "A", "c1", 3) == ("stored", {"seq": 3})


def test_a_pending_request_stops_the_purge_and_is_confirmed_under_its_own_key():
    book, a = ledger(window_s=60.0)
    book.admit(a, "A", "c1", 1)
    book.put(a, 1, PENDING, now=1.0)
    book.admit(a, "A", "c1", 2)
    book.put(a, 2, {"seq": 2}, now=2.0)
    book.purge(now=500.0)
    assert book.watermark(a) == 0 and book.admit(a, "A", "c1", 1) == ("stored", PENDING)
    book.put(a, 1, {"seq": 1}, now=501.0)  # confirmed later; the window counts from here
    book.purge(now=502.0)
    assert book.admit(a, "A", "c1", 1) == ("stored", {"seq": 1})
    book.purge(now=562.0)
    assert book.watermark(a) == 2


@pytest.mark.parametrize("client, conn", [("B", "c2"), ("A", "c-old")])
def test_another_connections_token_is_stale(client, conn):
    book, a = ledger()
    assert book.admit(a, client, conn, 1) == ("stale", None)
    assert book.admit(999, "A", "c1", 1) == ("stale", None)  # never issued


def test_a_reconnect_retires_the_old_token():
    book, a = ledger()
    book.admit(a, "A", "c1", 1)
    book.put(a, 1, {"seq": 1}, now=1.0)
    b = book.issue()
    book.register(b, "A", "c2", now=2.0)
    assert b != a and book.admit(a, "A", "c1", 2) == ("stale", None)
    assert book.admit(b, "A", "c2", 1) == ("run", None)


def test_the_ledger_is_bounded():
    book, a = ledger(per_token=2, total=3)
    for seq in (1, 2):
        book.admit(a, "A", "c1", seq)
        book.put(a, seq, {"seq": seq}, now=0.0)
    assert book.admit(a, "A", "c1", 3) == ("busy", None)
    b = book.issue()
    book.register(b, "B", "c2", now=0.0)
    book.admit(b, "B", "c2", 1)
    book.put(b, 1, {"x": 1}, now=0.0)
    assert book.admit(b, "B", "c2", 2) == ("busy", None)  # the total


def test_live_tokens_are_bounded_and_retired_ones_go_first():
    book = RequestLedger(max_tokens=2)
    a = book.issue()
    book.register(a, "A", "c1", now=0.0)
    b = book.issue()
    book.register(b, "A", "c2", now=1.0)  # A reconnects: a retired
    c = book.issue()
    assert book.register(c, "B", "c3", now=2.0) is True  # a (retired, nothing pending) makes room
    d = book.issue()
    assert book.register(d, "C", "c4", now=3.0) is False  # b and c are current: refused
    assert len(book) <= 2


def test_tokens_are_never_reused():
    book = RequestLedger()
    issued = [book.issue() for _ in range(5)]
    assert issued == sorted(set(issued))


# --- the arbiter on the ledger (tokens required, as a live runtime runs it) ------------------------------------------
import itertools  # noqa: E402

from forklift_core.mission.commands import MissionCommands  # noqa: E402
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig  # noqa: E402

EPOCH = 5


def arbiter():
    counter = itertools.count(1)
    a = Arbiter(EPOCH, ArbiterConfig(request_tokens=True), lambda: f"tok{next(counter)}", mission=MissionCommands())
    welcome = a.connect("A", 0.0, conn_id="c1", token=a.ledger.issue())
    return a, welcome


def msg(kind, token, seq, **fields):
    return {"t": kind, "request_id": f"r{seq}", "session_epoch": EPOCH, "request_token": token, "request_seq": seq,
            **fields}


def test_the_welcome_carries_the_connections_token_and_requests_need_it():
    a, welcome = arbiter()
    token = welcome["request_token"]
    assert isinstance(token, int)
    no_token = {"t": "lease_request", "request_id": "x", "session_epoch": EPOCH}
    assert a.handle("A", no_token, 0.1, conn_id="c1")["reason"] == "invalid"
    assert a.handle("A", msg("lease_request", token, 1), 0.1, conn_id="c1")["t"] == "ack"


def test_a_lease_request_resent_after_its_result_was_purged_is_stale_not_a_new_lease():
    a, welcome = arbiter()
    token = welcome["request_token"]
    first = a.handle("A", msg("lease_request", token, 1), 0.1, conn_id="c1")
    assert first["t"] == "ack"
    assert a.handle("A", msg("lease_request", token, 1), 0.2, conn_id="c1") == first  # resent: the same answer
    a.handle("A", msg("lease_release", token, 2, lease_id=first["lease_id"]), 0.3, conn_id="c1")
    a.ledger.purge(now=100.0)
    late = a.handle("A", msg("lease_request", token, 1), 100.1, conn_id="c1")
    assert late["t"] == "reject" and late["reason"] == "stale" and a.lease is None


def test_a_forged_or_old_token_is_stale_and_a_stop_always_runs():
    a, welcome = arbiter()
    token = welcome["request_token"]
    assert a.handle("A", msg("lease_request", token + 7, 1), 0.1, conn_id="c1")["reason"] == "stale"
    a.connect("A", 0.2, conn_id="c2", token=a.ledger.issue())  # a reconnect: the old token retires
    assert a.handle("A", msg("lease_request", token, 1), 0.3, conn_id="c2")["reason"] == "stale"
    stop = {"t": "stop", "request_id": "s", "session_epoch": EPOCH, "source": "key"}  # no token at all
    assert a.handle("A", stop, 0.4, conn_id="c2")["t"] == "ack" and a.mode == "STOPPED"


def test_a_closed_connections_token_is_retired_and_goes_when_its_results_do():
    """Codex 4c review P2-1: 64 different clients coming and going filled the ledger for good."""
    book = RequestLedger(max_tokens=2, window_s=10.0)
    for n in range(10):
        token = book.issue()
        assert book.register(token, f"C{n}", f"c{n}", now=float(n))
        book.admit(token, f"C{n}", f"c{n}", 1)
        book.put(token, 1, {"n": n}, now=float(n))
        book.retire(token)  # the connection closed
        book.purge(now=float(n) + 20.0)
    assert len(book) <= 2


def test_an_old_epochs_request_is_refused_before_the_ledger_and_not_stored():
    """Codex 4c review P2-2: after a restart the tokens began at 1 again and an old epoch's request took the key."""
    a, welcome = arbiter()
    token = welcome["request_token"]
    old = dict(msg("lease_request", token, 1), session_epoch=EPOCH - 1, request_id="old")
    assert a.handle("A", old, 0.1, conn_id="c1")["reason"] == "stale_epoch"
    new = a.handle("A", msg("lease_request", token, 1), 0.2, conn_id="c1")
    assert new["t"] == "ack" and new["request_id"] == "r1"


def test_tokens_do_not_start_at_the_same_number_in_two_runs():
    from forklift_core.live.link import Channels

    assert Channels().issue_token() != Channels().issue_token()


def test_a_disconnect_retires_the_connections_token():
    a, welcome = arbiter()
    token = welcome["request_token"]
    a.disconnect("A", 1.0, conn_id="c1")
    a.connect("A", 2.0, conn_id="c1")  # the same conn id reused without a token must not revive it
    assert a.handle("A", msg("lease_request", token, 1), 2.1, conn_id="c1")["reason"] == "stale"


def test_a_connection_with_no_room_in_the_ledger_is_told_busy():
    """Codex 4c review P3-3: its welcome carried a token the ledger had refused."""
    from forklift_core.live.link import Channels, Link
    from forklift_core.safety.arbiter import Feedback

    a, _ = arbiter()
    a.ledger = RequestLedger(max_tokens=1)
    ch = Channels()
    link = Link(a, ch)
    ch.connect("k1", "one", True, ch.issue_token())
    ch.connect("k2", "two", True, ch.issue_token())
    link.begin(1.0, Feedback(1.0, 0.0))
    out = ch.take_out()
    assert ("k2", {"t": "reject", "of": "hello", "reason": "busy"}) in out
    assert not any(c == "k1" and m.get("of") == "hello" for c, m in out)
