"""Validation v2 H3 (docs/plans/2026-11-08-live-riib-defaults-v2.md): the live runtime logs each tick's applied mission
commands with the core's time before their replies are queued -- the applied cancel's sim time the false-positive
check needs."""

import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim" / "isaac"))

import live_runtime  # noqa: E402


def make(order):
    def log(entry):
        order.append(("log", entry))

    def end(now, records, safety, candidate, drive):
        order.append(("replies", list(records)))
        return "out"

    return SimpleNamespace(clock=lambda: 55.5, log=log, link=SimpleNamespace(end=end))


def test_applied_commands_are_logged_with_their_core_time_before_the_replies():
    order = []
    records = [{"t": 52.71, "command": "cancel", "accepted": True, "reason": None, "mission_id": "m1",
                "status": "cancelling", "request_id": "r4", "detail": "kept out"},
               {"t": 52.71, "command": "stop_kind:controlled", "accepted": True, "reason": None, "mission_id": "m1",
                "status": "paused"}]
    out = live_runtime.LiveRuntime.tick_end(make(order), 52.71, records, {}, (0.0, 0.0), None)
    assert out == "out"
    (kind0, entry), (kind1, _) = order
    assert (kind0, kind1) == ("log", "replies")
    assert entry["tick_t"] == 52.71
    assert entry["mission_commands"][0] == {"t": 52.71, "command": "cancel", "accepted": True, "reason": None,
                                            "mission_id": "m1", "status": "cancelling", "request_id": "r4"}
    assert entry["mission_commands"][1]["request_id"] is None  # a command the core made itself


def test_a_tick_without_commands_logs_nothing():
    order = []
    live_runtime.LiveRuntime.tick_end(make(order), 1.0, [], {}, (0.0, 0.0), None)
    assert [k for k, _ in order] == ["replies"]
