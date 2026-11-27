"""The bench console's channels and cycle (plan 2026-11-27 3.1): the stop-like requests raise the emergency serial
the moment they arrive, under the channels' lock (so a write judged before them is refused); the cycle handles the
emergency batch, the ledger's purge and the requests, and never the vehicle's observe/mission context (no
``not_stopped`` stop without feedback)."""

import itertools

import pytest

from forklift_core.live.bench_link import BenchChannels, BenchLink
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig

EPOCH = 4242


def rig():
    counter = itertools.count(1)
    a = Arbiter(EPOCH, ArbiterConfig(bench_requests=True, request_tokens=True), lambda: f"tok{next(counter)}")
    calls = []
    a.bench_request = lambda kind, msg, now: (calls.append(kind), (None, {}))[1]
    ch = BenchChannels()
    return a, ch, BenchLink(a, ch), calls


def connect_and_lease(a, ch, link, now=0.0):
    token = ch.issue_token()
    ch.connect("c1", "laptop", True, token)
    ch.message("c1", "laptop", {"t": "lease_request", "request_id": "L1", "session_epoch": EPOCH,
                                "request_token": token, "request_seq": 1})
    link.begin(now)
    ack = [m for c, m in ch.take_out() if m.get("t") == "ack" and m.get("of") == "lease_request"]
    assert ack, ch
    return token, ack[0]["lease_id"]


@pytest.mark.parametrize("kind", ["bench_stop", "bench_disarm", "lease_release", "stop", "disarm"])
def test_stop_like_requests_raise_the_serial_on_arrival(kind):
    _, ch, _, _ = rig()
    s0 = ch.serial()
    ch.message("c1", "laptop", {"t": kind, "request_id": "X1", "session_epoch": EPOCH})
    assert ch.serial() == s0 + 1


@pytest.mark.parametrize("kind", ["bench_run", "clear_faults", "echo"])
def test_other_requests_do_not(kind):
    _, ch, _, _ = rig()
    s0 = ch.serial()
    msg = {"t": kind, "request_id": "X1", "session_epoch": EPOCH}
    if kind == "echo":
        msg = {"t": "echo", "session_epoch": EPOCH, "lease_id": "x", "echo": {"state_seq": 1, "token": "t"}}
    ch.message("c1", "laptop", msg)
    assert ch.serial() == s0


def test_a_bench_stop_is_still_answered_through_the_ledger():
    a, ch, link, calls = rig()
    token, lease_id = connect_and_lease(a, ch, link)
    ch.message("c1", "laptop", {"t": "bench_stop", "request_id": "S1", "session_epoch": EPOCH, "lease_id": lease_id,
                                "control_gen": a.control_gen, "request_token": token, "request_seq": 2})
    link.begin(0.1)
    assert calls == ["bench_stop"]
    assert any(m.get("of") == "bench_stop" and m["t"] == "ack" for _, m in ch.take_out())


def test_the_cycle_never_stops_for_missing_feedback():
    """The vehicle's not_stopped fault (10 s without a confirmed stand) is not the bench's."""
    a, ch, link, _ = rig()
    connect_and_lease(a, ch, link)
    for k in range(0, 300):
        link.begin(k * 0.1)
    assert a.mode == "PAUSED" and a.stops == 0


def test_ticks_go_out_every_period_and_an_echo_renews_the_lease():
    a, ch, link, _ = rig()
    token, lease_id = connect_and_lease(a, ch, link)
    link.tick(0.0)
    ticks = [m for c, m in ch.take_out() if m.get("t") == "tick"]
    assert len(ticks) == 1
    link.tick(0.01)
    assert not [m for c, m in ch.take_out() if m.get("t") == "tick"]  # not due yet
    tk = ticks[0]
    ch.message("c1", "laptop", {"t": "echo", "session_epoch": EPOCH, "lease_id": lease_id,
                                "echo": {"state_seq": tk["state_seq"], "token": tk["token"]}})
    link.begin(a.config.lease_timeout_s - 0.01)
    assert a.lease is not None and a.lease.renewed_mono == 0.0
    link.begin(a.config.lease_timeout_s + 0.5)
    assert a.lease is None  # no newer echo: it expired
