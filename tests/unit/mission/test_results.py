"""The persistent console's results book (docs/plans/2026-10-18-persistent-live-console.md D1 step 9): each mission's
outcome is kept for the session and published, so a page connecting later still sees it."""

import pytest

from forklift_core.mission.results import ResultsBook


def entry(book, mission_id="m1", outcome="done", reason=None, **k):
    return book.add(mission_id=mission_id, outcome=outcome, reason=reason, started_s=k.get("started_s", 1.0),
                    ended_s=k.get("ended_s", 9.0), pallet_id="P-1", destination_id="D-1")


def test_results_are_numbered_in_order_and_the_last_one_is_kept():
    book = ResultsBook()
    assert book.last is None and book.message() == {"t": "missions", "missions_seq": 0, "results": []}
    first = entry(book)
    second = entry(book, "m2", "failed", "Pallet was not lifted")
    assert first == {"seq": 1, "mission_id": "m1", "outcome": "done", "reason": None, "started_s": 1.0,
                     "ended_s": 9.0, "pallet_id": "P-1", "destination_id": "D-1"}
    assert second["seq"] == 2 and book.last == second
    assert book.message() == {"t": "missions", "missions_seq": 2, "results": [first, second]}


def test_the_message_keeps_the_newest_results_and_counts_the_rest():
    book = ResultsBook(keep=2)
    for i in range(5):
        entry(book, f"m{i}")
    message = book.message()
    assert [r["mission_id"] for r in message["results"]] == ["m3", "m4"]
    assert message["missions_seq"] == 5 and message["older"] == 3


@pytest.mark.parametrize("outcome", ["finished", "idle", None])
def test_an_unknown_outcome_is_refused(outcome):
    with pytest.raises(ValueError):
        entry(ResultsBook(), outcome=outcome)


def test_a_returned_entry_cannot_change_the_book():
    book = ResultsBook()
    e = entry(book)
    e["outcome"] = "failed"
    assert book.last["outcome"] == "done" and book.message()["results"][0]["outcome"] == "done"


def test_a_note_adds_fields_to_a_kept_result():
    book = ResultsBook()
    e = entry(book)
    assert book.note(e["seq"], record_error="OSError: full")["record_error"] == "OSError: full"
    assert book.last["record_error"] == "OSError: full"
    with pytest.raises(KeyError):
        book.note(9, x=1)
