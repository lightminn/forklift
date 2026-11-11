"""The wall escape's approval in the arbiter (docs/design/2026-11-12-standing-direction-rule.md 4.2): a leased,
reliable ``escape_approve`` that only a back end with an injected handler takes (the S6 bridge); every other back end
-- Isaac, the mock -- answers ``unsupported``. The arbiter checks TELEOP, armed, no latch and the arm generation, and
hands the rest to the handler."""

import pytest
from test_arbiter_modes import Rig

from forklift_core.safety.arbiter import LEASED, RELIABLE
from forklift_core.safety.validate import problem


def _armed(handler=None):
    rig = Rig()
    rig.stand()
    if handler is not None:
        rig.a.escape = handler
    [reply] = rig.cycle(rig.msg("mode_request", to="TELEOP", source="direct"))
    assert reply["t"] == "ack" and rig.a.mode == "TELEOP"
    [arm] = rig.cycle(rig.msg("arm"))
    assert arm["t"] == "ack" and rig.a.armed
    return rig


def _approve(rig, **fields):
    return rig.msg("escape_approve", arm_gen=rig.a.arm_gen, offer_id=7, **fields)


def test_escape_approve_is_leased_and_reliable():
    assert "escape_approve" in LEASED and set(LEASED) <= set(RELIABLE)


def test_without_a_handler_it_is_unsupported():
    rig = _armed()
    [reply] = rig.cycle(_approve(rig))
    assert reply["t"] == "reject" and reply["reason"] == "unsupported"


def test_the_handler_gets_the_request_and_its_verdict_is_the_reply():
    seen = []

    def handler(msg, now):
        seen.append((msg["offer_id"], msg["arm_gen"], msg["control_gen"]))
        return None if msg["offer_id"] == 7 else "escape_no_offer"

    rig = _armed(handler)
    [reply] = rig.cycle(_approve(rig))
    assert reply["t"] == "ack" and seen == [(7, rig.a.arm_gen, rig.a.control_gen)]
    [reply] = rig.cycle(rig.msg("escape_approve", arm_gen=rig.a.arm_gen, offer_id=8))
    assert reply["t"] == "reject" and reply["reason"] == "escape_no_offer"


@pytest.mark.parametrize("fields", [
    {"offer_id": None}, {"offer_id": "7"}, {"offer_id": True}, {"offer_id": 7.0},
    {"arm_gen": None}, {"arm_gen": True},
])
def test_a_malformed_approval_is_invalid(fields):
    rig = _armed(lambda msg, now: None)
    msg = _approve(rig)
    for k, v in fields.items():
        if v is None:
            del msg[k]
        else:
            msg[k] = v
    assert problem(msg) == "invalid"
    [reply] = rig.cycle(msg)
    assert reply["t"] == "reject" and reply["reason"] == "invalid"


def test_it_needs_the_lease_and_the_control_generation():
    calls = []
    rig = _armed(lambda msg, now: calls.append(msg) or None)
    msg = _approve(rig)
    msg["lease_id"] = "someone-else"
    [reply] = rig.cycle(msg)
    assert reply["t"] == "reject" and reply["reason"] in ("no_lease", "stale_lease")
    msg = _approve(rig)
    msg["control_gen"] = rig.a.control_gen - 1
    [reply] = rig.cycle(msg)
    assert reply["t"] == "reject" and reply["reason"] == "stale_control" and calls == []


def test_an_older_arm_generation_is_refused_before_the_handler():
    calls = []
    rig = _armed(lambda msg, now: calls.append(msg) or None)
    [reply] = rig.cycle(rig.msg("escape_approve", arm_gen=rig.a.arm_gen - 1, offer_id=7))
    assert reply["t"] == "reject" and reply["reason"] == "stale_arm" and calls == []


def test_it_is_refused_outside_an_armed_teleop():
    calls = []
    rig = Rig()
    rig.stand()
    rig.a.escape = lambda msg, now: calls.append(msg) or None
    [reply] = rig.cycle(rig.msg("escape_approve", arm_gen=0, offer_id=7))
    assert reply["t"] == "reject" and reply["reason"] == "not_teleop"
    [reply] = rig.cycle(rig.msg("mode_request", to="TELEOP", source="direct"))
    [reply] = rig.cycle(rig.msg("escape_approve", arm_gen=rig.a.arm_gen, offer_id=7))
    assert reply["t"] == "reject" and reply["reason"] == "not_armed" and calls == []


def test_it_is_refused_while_a_preempt_holds():
    calls = []
    rig = _armed(lambda msg, now: calls.append(msg) or None)
    rig.a.preempt_latch = "overspeed"  # set right before the request (a standing cycle would lift it)
    reply = rig.a.handle("op", _approve(rig), rig.t)
    assert reply["t"] == "reject" and reply["reason"] == "latched" and calls == []


def test_a_stored_answer_is_not_run_again():
    calls = []
    rig = _armed(lambda msg, now: calls.append(msg) or None)
    msg = _approve(rig)
    [first] = rig.cycle(msg)
    [again] = rig.cycle(dict(msg))
    assert first["t"] == again["t"] == "ack" and len(calls) == 1
