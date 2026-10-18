"""Persistent console plan D14 (5th ed.): manual lift and the recovery-only teleop rules. The policy says, per
control cycle, what manual driving and which lift directions are allowed; inside a pocket only the recovery rows of
the table open anything (lower first, then a straight reverse along the pocket axis), and only after a live mission
ended recovery_required."""

import pytest

from forklift_core.safety.teleop_policy import teleop_policy

FREE = {"load": {"loaded": False}, "fork_in_pocket": {"value": False, "relation": "free"}, "near": None,
        "frame": {"hold": "none", "slam": True}, "mission": {"phase": None, "status": "idle"}}


def in_pocket(relation="constrained", **recovery):
    rec = {"stage": "in_pocket", "grounded": True, "evidence": True, "aligned": True, **recovery}
    return {**FREE, "load": {"loaded": True}, "fork_in_pocket": {"value": True, "relation": relation},
            "recovery_required": True, "recovery": rec, "frame": {"hold": "insert", "slam": True}}


def policy(safety, *, standing=True, lift_m=0.0, steering_rad=0.0):
    return teleop_policy(safety, feedback_ok=True, standing=standing, lift_m=lift_m, steering_rad=steering_rad)


def test_free_and_standing_lifts_both_ways_and_drives_normally():
    p = policy(FREE)
    assert p.drive_allowed and p.drive_mode == "normal" and p.lift_dirs == frozenset({-1, 1}) and p.lift_allowed
    moving = policy(FREE, standing=False)
    assert moving.drive_allowed and moving.lift_dirs == frozenset()


def test_carrying_drives_loaded_and_never_lifts():
    safety = {**FREE, "load": {"loaded": True}, "fork_in_pocket": {"value": True, "relation": "carrying"}}
    p = policy(safety, lift_m=0.2)
    assert p.drive_allowed and p.loaded and p.lift_dirs == frozenset()


def test_a_lifted_recovery_lowers_first_and_does_not_drive():
    p = policy(in_pocket(), lift_m=0.05)
    assert not p.drive_allowed and p.lift_dirs == frozenset({-1}) and p.teleop_allowed  # TELEOP may be entered
    assert p.reason == "recovery:lower_first"
    assert policy(in_pocket(), lift_m=0.05, standing=False).lift_dirs == frozenset()  # lowering stands


def test_a_lowered_recovery_reverses_straight_only():
    p = policy(in_pocket(), lift_m=0.005)
    assert p.drive_allowed and p.drive_mode == "reverse_straight" and p.lift_dirs == frozenset()


@pytest.mark.parametrize("missing, reason", [
    ({"grounded": False}, "recovery:ground_wait"), ({"evidence": False}, "recovery:evidence"),
    ({"aligned": False}, "recovery:misaligned"),
])
def test_a_lowered_recovery_without_its_conditions_opens_nothing(missing, reason):
    p = policy(in_pocket(**missing), lift_m=0.005)
    assert not p.drive_allowed and p.lift_dirs == frozenset() and p.reason == reason


def test_without_the_measurements_the_recovery_rows_stay_shut():
    assert not policy(in_pocket(), lift_m=None).teleop_allowed
    p = policy(in_pocket(), lift_m=0.005, steering_rad=None)
    assert not p.drive_allowed and p.reason == "recovery:no_steering"


def test_an_unknown_relation_only_lowers():
    assert policy(in_pocket("unknown"), lift_m=0.05).lift_dirs == frozenset({-1})
    p = policy(in_pocket("unknown"), lift_m=0.005)
    assert not p.drive_allowed and p.lift_dirs == frozenset()


def test_the_manual_clear_stage_reverses_straight_while_its_evidence_holds():
    safety = {**FREE, "recovery": {"stage": "manual_clear", "grounded": True, "evidence": True, "aligned": True}}
    p = policy(safety, lift_m=0.0)
    assert p.drive_allowed and p.drive_mode == "reverse_straight" and p.lift_dirs == frozenset()
    safety["recovery"]["evidence"] = False
    assert not policy(safety, lift_m=0.0).drive_allowed


def test_a_running_near_contract_refuses_but_an_ended_missions_hold_goes_to_the_recovery():
    """Codex D14 design round 2 P2-3: an insert that ended leaves frame.hold "insert" behind."""
    running = {**in_pocket(), "recovery_required": False, "recovery": None,
               "mission": {"phase": "insert", "status": "paused"}}
    assert not policy(running, lift_m=0.005).teleop_allowed  # refused as before (in the pocket: fork_in_pocket)
    assert policy({**FREE, "near": {"active": True}}).reason == "near_section"
    assert policy(in_pocket(), lift_m=0.005).drive_mode == "reverse_straight"


def test_a_pocket_without_a_recovery_is_refused_as_before():
    safety = {**in_pocket(), "recovery_required": False, "recovery": None, "mission": {"phase": None,
                                                                                         "status": "idle"}}
    p = policy(safety, lift_m=0.0)
    assert not p.teleop_allowed and p.reason == "fork_in_pocket"
