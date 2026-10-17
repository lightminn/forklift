"""M-b1: reply tokens, camera frames and stored request results (plan B3, B10)."""

import itertools

import pytest

from forklift_core.safety.generations import RequestBook
from forklift_core.safety.ledger import FrameLedger, TickLedger


def tokens():
    counter = itertools.count(1)
    return lambda: f"tok{next(counter)}"


def test_a_tick_is_known_by_its_number_and_token_only():
    ledger = TickLedger(1.0, tokens())
    assert ledger.issue(10.0) == (1, "tok1")
    assert ledger.sent_mono(1, "tok1") == 10.0
    assert ledger.sent_mono(1, "tok2") is None and ledger.sent_mono(2, "tok1") is None


def test_ticks_are_kept_for_the_window_however_many_went_out():
    ledger = TickLedger(1.0, tokens())
    ledger.issue(0.0)
    for k in range(1, 2001):  # 2000 ticks inside the window
        ledger.issue(k * 0.0005)
    assert ledger.sent_mono(1, "tok1") == 0.0  # 1.0 s old: still inside
    ledger.issue(1.0 + 1e-6)
    assert ledger.sent_mono(1, "tok1") is None


def test_state_messages_share_the_numbering():
    ledger = TickLedger(1.0, tokens())
    ledger.issue(0.0)
    assert ledger.next_seq() == 2 and ledger.issue(0.1)[0] == 3


def test_a_resent_frame_keeps_its_first_time():
    frames = FrameLedger(5.0)
    frames.register(7, 1.0)
    frames.register(7, 3.0)
    assert frames.acquired_mono(7) == 1.0 and frames.acquired_mono(8) is None
    frames.register(9, 6.5)
    assert frames.acquired_mono(7) is None and frames.acquired_mono(9) == 6.5
    frames.register(7, 7.0)  # dropped from the window: not taken again with a young time
    assert frames.acquired_mono(7) is None
    frames.register(8, 7.0)  # below the largest seen and never held: refused too
    assert frames.acquired_mono(8) is None


def test_the_request_store_refuses_new_entries_when_full_but_keeps_answering_old_ones():
    book = RequestBook(capacity=2)
    book.put("c", "r1", "ack")
    book.put("c", "r2", "reject")
    assert book.full() and book.get("c", "r1") == "ack"
    with pytest.raises(OverflowError):
        book.put("c", "r3", "ack")
    book.put("c", "r1", "ack2")  # an existing key may be settled (a pending result)
    assert book.get("c", "r1") == "ack2" and len(book) == 2
