"""The vehicle-side F1 block (docs/plans/2026-11-08-f1-vehicle-block.md 2): an operator-placed start latches a refusal
of what drives the truck by the map -- missions, resumes, picks, recoveries, AUTO -- with operator_placed_start, in the
arbiter and in the mission commands; teleop, arming, STOP, the session's end and a scene reset stay open; nothing in
the epoch lifts it."""

import pytest
from test_arbiter_modes import EPOCH, Rig, auto

from forklift_core.mission.commands import Context, MissionCommands, Resume, Submit
from forklift_core.safety.arbiter import START_BLOCK_REASON, START_REFUSED

REFUSED = [
    ("mission_submit", {"pallet_id": "P", "destination_id": "D", "targets_revision": 0}),
    ("mission_resume", {}),
    ("mode_request", {"to": "AUTO"}),
    ("recovery", {}),
    ("pick_request", {}),
]


def _rig(block=True):
    rig = Rig()
    rig.a.sim_reset_allowed = rig.a.next_pick_allowed = True
    rig.stand()
    if block:
        rig.a.block_start("operator_placed", rig.t)
    return rig


def test_the_reason_and_the_list():
    assert START_BLOCK_REASON == "operator_placed_start"
    assert set(START_REFUSED) == {"mission_submit", "mission_resume", "pick_request", "recovery"}


@pytest.mark.parametrize("kind, fields", REFUSED)
def test_what_drives_by_the_map_is_refused_and_changes_nothing(kind, fields):
    rig = _rig()
    [reply] = rig.cycle(rig.msg(kind, **fields))
    assert reply["t"] == "reject" and reply["reason"] == "operator_placed_start"
    assert rig.a.mode == "PAUSED" and rig.m.status == "idle" and rig.a.session_request is None


def test_without_the_latch_the_mission_goes_as_before():
    rig = _rig(block=False)
    msg = rig.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0)
    rig.cycle(msg)
    assert rig.reply(msg["request_id"])["t"] == "ack" and rig.a.mode == "AUTO"


def test_teleop_and_arming_stay_open():
    rig = _rig()
    [reply] = rig.cycle(rig.msg("mode_request", to="TELEOP", source="direct"))
    assert reply["t"] == "ack" and rig.a.mode == "TELEOP"
    [arm] = rig.cycle(rig.msg("arm"))
    assert arm["t"] == "ack" and rig.out.policy.drive_allowed
    [paused] = rig.cycle(rig.msg("mode_request", to="PAUSED"))
    assert paused["t"] == "ack" and rig.a.start_block is not None


def test_stop_rearm_and_the_session_end_stay_open():
    rig = _rig()
    rig.cycle({"t": "stop", "request_id": "s1", "session_epoch": EPOCH})
    assert rig.a.mode == "STOPPED"
    rig.stand()
    [rearm] = rig.cycle(rig.msg("rearm"))
    assert rearm["t"] == "ack" and rig.a.mode == "PAUSED" and rig.a.start_block is not None
    [end] = rig.cycle(rig.msg("session_end"))
    assert end["t"] == "ack" and rig.a.session_request == "end"


def test_a_scene_reset_is_taken_and_the_latch_stays_in_the_epoch():
    rig = _rig()
    [reset] = rig.cycle(rig.msg("sim_reset"))
    assert reset["t"] == "ack" and rig.a.session_request == "reset" and rig.a.start_block is not None
    [again] = rig.cycle(rig.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0))
    assert again["t"] == "reject" and again["reason"] in ("operator_placed_start", "session_ending")


def test_the_latch_survives_a_stop_a_new_lease_and_a_reconnection():
    rig = _rig()
    rig.cycle({"t": "stop", "request_id": "s1", "session_epoch": EPOCH})
    rig.a.disconnect("op", rig.t)
    rig.a.connect("op", rig.t)
    assert rig.a.start_block == {"reason": "operator_placed_start", "source": "operator_placed",
                                 "since_mono": pytest.approx(rig.a.start_block["since_mono"])}


def test_the_latch_is_set_once_and_never_cleared_by_the_arbiter():
    rig = _rig()
    first = dict(rig.a.start_block)
    rig.a.block_start("matcher", rig.t + 5.0)  # a second call keeps the first (nothing in the epoch lifts it)
    assert rig.a.start_block == first
    assert [e for e in rig.a.events if e.get("event") == "start_block"] == [
        {"t": first["since_mono"], "event": "start_block", "reason": "operator_placed_start",
         "source": "operator_placed"}]


def test_the_relocalization_answers_before_the_latch():
    rig = _rig()
    rig.a.reloc = object()
    [reply] = rig.cycle(rig.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0))
    assert reply["reason"] == "relocalizing"


def test_the_latch_answers_before_the_unconfirmed_position():
    rig = _rig()
    rig.a.pose = {"status": "unconfirmed", "drive_ready": True, "reason": None}
    [reply] = rig.cycle(rig.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0))
    assert reply["reason"] == "operator_placed_start"


def test_auto_is_never_entered_with_the_latch_even_past_the_refusals():
    rig = _rig(block=False)
    msg = rig.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0)
    with pytest.raises(RuntimeError, match="operator_placed"):
        # queued before the latch, accepted by a lifecycle that was not told: the AUTO entry itself refuses
        rig.cycle(msg, between=lambda: rig.a.block_start("operator_placed", rig.t))
    assert rig.a.mode != "AUTO"


def test_a_running_mission_is_not_the_latchs_concern():
    # the latch is set before the mission core opens (the runtime's start); an AUTO already on is out of its scope
    rig = Rig()
    auto(rig)
    assert rig.a.start_block is None


# --- the mission commands' own refusal (the second layer) --------------------------------------------------------------
def _apply(m, t, ctx, command):
    m.queue(command)
    [rec] = m.apply(t, ctx)
    return rec


def test_the_commands_refuse_a_submit_with_the_fixed_code():
    m = MissionCommands()
    rec = _apply(m, 0.0, Context(start_blocked=True), Submit("P", "D", None, request_id="a"))
    assert rec["accepted"] is False and rec["reason"] == "operator_placed_start" and m.status == "idle"
    rec = _apply(m, 0.1, Context(), Submit("P", "D", None, request_id="b"))
    assert rec["accepted"] and m.status == "accepted"


def test_the_commands_refuse_a_loaded_submit_too():
    m = MissionCommands()
    ctx = Context(start_blocked=True, submit_refusal="cargo_held", loaded_pallet="P")
    assert _apply(m, 0.0, ctx, Submit("P", "D", None, request_id="a"))["reason"] == "operator_placed_start"


def test_the_commands_refuse_a_resume():
    m = MissionCommands()
    m.status = "paused"
    rec = _apply(m, 0.0, Context(start_blocked=True), Resume(request_id="r"))
    assert rec["accepted"] is False and rec["reason"] == "operator_placed_start" and m.status == "paused"
