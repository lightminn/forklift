"""B-P2c: the arbiter while the start is unknown (docs/plans/2026-10-29-start-relocalization.md 1.2 step 4, 7; 1.3):
relocalize_pick / relocalize_retry leased and judged by the relocalization's own state, anything that could move the
truck or replace the session refused ``relocalizing``, STOP as always (and counted for the phase), session end."""

import pytest
from test_arbiter_modes import EPOCH, Rig

from forklift_core.localization import global_match as G
from forklift_core.localization import start_reloc as R
from forklift_core.safety.arbiter import LEASED, RELIABLE


def _awaiting():
    r = R.StartRelocalizer(R.RelocConfig(e_min=0.8, r_max=0.95, r_struct=0.8, attempts=1), lambda: 0.0)
    r.start()
    cands = (G.Candidate((3.0, 1.0, 0.5), 0.9, 0.97), G.Candidate((9.0, 4.0, 2.0), 0.88, 0.96))
    r.record(G.GlobalMatch("ok", cands), G.Decision(False, "competitor", 0.98))
    assert r.state == "awaiting_pick" and r.generation == 1
    return r


def _rig(reloc=None):
    rig = Rig()
    rig.a.reloc = reloc
    rig.stand()
    return rig


def test_the_new_commands_are_leased_and_reliable():
    assert {"relocalize_pick", "relocalize_retry"} <= set(LEASED) <= set(RELIABLE)


def test_a_pick_of_the_current_generation_is_the_estimate():
    reloc = _awaiting()
    rig = _rig(reloc)
    [reply] = rig.cycle(rig.msg("relocalize_pick", generation=1, candidate=1))
    assert reply["t"] == "ack" and reloc.state == "chosen" and reloc.estimate.source == "operator"
    assert reloc.estimate.base == pytest.approx(R.base_from_laser((9.0, 4.0, 2.0)))


@pytest.mark.parametrize("fields, reason", [
    ({"generation": 0, "candidate": 0}, "reloc_stale_pick"),
    ({"generation": 1, "candidate": 5}, "reloc_pick_refused"),
    ({"generation": True, "candidate": 0}, "invalid"),  # a bool is no generation (Codex B-P2c design P2)
    ({"generation": 1}, "invalid"),
    ({"candidate": 0}, "invalid"),
    ({"generation": 1.0, "candidate": 0}, "invalid"),
])
def test_a_wrong_pick_is_refused_and_changes_nothing(fields, reason):
    reloc = _awaiting()
    rig = _rig(reloc)
    [reply] = rig.cycle(rig.msg("relocalize_pick", **fields))
    assert reply["t"] == "reject" and reply["reason"] == reason and reloc.state == "awaiting_pick"


def test_a_pick_needs_the_lease():
    reloc = _awaiting()
    rig = _rig(reloc)
    msg = rig.msg("relocalize_pick", generation=1, candidate=0)
    msg["lease_id"] = "someone-else"
    [reply] = rig.cycle(msg)
    assert reply["t"] == "reject" and reply["reason"] in ("no_lease", "stale_lease") and reloc.state == "awaiting_pick"


def test_a_retry_moves_the_generation_on():
    reloc = _awaiting()
    rig = _rig(reloc)
    [reply] = rig.cycle(rig.msg("relocalize_retry", generation=1))
    assert reply["t"] == "ack" and reloc.state == "relocalizing" and reloc.generation == 2
    [again] = rig.cycle(rig.msg("relocalize_retry", generation=1))
    assert again["t"] == "reject" and again["reason"] == "reloc_not_awaiting"


def test_without_a_relocalization_the_commands_are_refused():
    rig = _rig(None)
    for msg in (rig.msg("relocalize_pick", generation=1, candidate=0), rig.msg("relocalize_retry", generation=1)):
        [reply] = rig.cycle(msg)
        assert reply["t"] == "reject" and reply["reason"] == "not_relocalizing"


@pytest.mark.parametrize("kind, fields", [
    ("mission_submit", {"pallet_id": "P", "destination_id": "D", "targets_revision": 0}),
    ("mode_request", {"to": "AUTO"}),
    ("mode_request", {"to": "TELEOP", "source": "direct"}),
    ("arm", {}),
    ("sim_reset", {}),  # Codex B-P2c design v3 P2: a reset while the start is unknown is refused
])
def test_what_could_move_the_truck_or_reset_the_scene_is_refused_while_relocalizing(kind, fields):
    rig = _rig(_awaiting())
    rig.a.sim_reset_allowed = True
    [reply] = rig.cycle(rig.msg(kind, **fields))
    assert reply is not None and reply["t"] == "reject" and reply["reason"] == "relocalizing"
    assert rig.a.mode == "PAUSED" and rig.m.status != "running" and rig.a.session_request is None


def test_a_session_end_is_taken_while_relocalizing():
    rig = _rig(_awaiting())
    [reply] = rig.cycle(rig.msg("session_end"))
    assert reply["t"] == "ack" and rig.a.session_request == "end"


def test_every_stop_is_counted_for_the_phase():
    rig = _rig(_awaiting())
    assert rig.a.stops == 0
    rig.cycle({"t": "stop", "request_id": "s1", "session_epoch": EPOCH})
    rig.cycle({"t": "stop", "request_id": "s2", "session_epoch": EPOCH})
    assert rig.a.mode == "STOPPED" and rig.a.stops == 2


def test_after_the_relocalization_the_mission_goes_as_before():
    rig = _rig(_awaiting())
    rig.a.reloc = None  # adopted: localized
    msg = rig.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0)
    rig.cycle(msg)
    assert rig.reply(msg["request_id"])["t"] == "ack" and rig.a.mode == "AUTO"


# --- F1: the operator's placed pose (docs/plans/2026-10-31-operator-placed-start.md 1.2) ---------------------------------
def test_the_placement_commands_are_leased_and_reliable():
    assert {"relocalize_place", "relocalize_place_confirm"} <= set(LEASED) <= set(RELIABLE)


def test_a_placement_and_its_confirmation_go_through_the_relocalizations_state():
    reloc = _awaiting()
    rig = _rig(reloc)
    [reply] = rig.cycle(rig.msg("relocalize_place", generation=1, pose=[5.0, 2.0, 0.3]))
    assert reply["t"] == "ack" and reloc.state == "placing" and reloc.placed == (5.0, 2.0, 0.3)
    reloc.placed_result(True, (5.1, 2.0, 0.31))
    [reply] = rig.cycle(rig.msg("relocalize_place_confirm", generation=1, preview_id=1))
    assert reply["t"] == "ack" and reloc.state == "chosen" and reloc.estimate.source == "operator_placed"


@pytest.mark.parametrize("kind, fields, reason", [
    ("relocalize_place", {"generation": 1, "pose": [5.0, 2.0]}, "invalid"),
    ("relocalize_place", {"generation": 1, "pose": [5.0, "2", 0.3]}, "invalid"),
    ("relocalize_place", {"generation": 1, "pose": [5.0, 2.0, True]}, "invalid"),
    ("relocalize_place", {"pose": [5.0, 2.0, 0.3]}, "invalid"),
    ("relocalize_place", {"generation": 0, "pose": [5.0, 2.0, 0.3]}, "reloc_stale_pick"),
    ("relocalize_place_confirm", {"generation": 1}, "invalid"),
    ("relocalize_place_confirm", {"generation": 1, "preview_id": True}, "invalid"),
    ("relocalize_place_confirm", {"generation": 1, "preview_id": 1}, "reloc_not_awaiting"),
])
def test_a_wrong_placement_or_confirmation_is_refused_and_changes_nothing(kind, fields, reason):
    reloc = _awaiting()
    rig = _rig(reloc)
    [reply] = rig.cycle(rig.msg(kind, **fields))
    assert reply["t"] == "reject" and reply["reason"] == reason and reloc.state == "awaiting_pick"
