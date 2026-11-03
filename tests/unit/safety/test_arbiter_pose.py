"""The unconfirmed position in the arbiter and the teleop policy (docs/plans/2026-11-02-unconfirmed-runner.md 2.3):
while not confirmed nothing that moves the truck by the map opens -- missions, picks, recoveries, resets, AUTO --
and the teleop is the straight reverse only, no forks; the policy's own refusals come first; once ended, the
controls stay locked while STOP and the session end stay open (design 1.4)."""

import pytest
from test_arbiter_modes import SAFE, Rig, auto

from forklift_core.mission.commands import Context, MissionCommands, Resume, Submit
from forklift_core.safety.teleop_policy import teleop_policy

READY = {"status": "unconfirmed", "drive_ready": True, "reason": None}


def _rig(pose):
    rig = Rig()
    rig.a.pose = pose
    rig.stand()
    return rig


def test_without_the_check_the_policy_is_unchanged():
    base = teleop_policy(SAFE, feedback_ok=True, standing=True)
    assert teleop_policy(SAFE, feedback_ok=True, standing=True, pose=None) == base


def test_an_unconfirmed_pose_opens_only_the_straight_reverse():
    p = teleop_policy(SAFE, feedback_ok=True, standing=True, pose=READY)
    assert p.drive_allowed and not p.lift_allowed and p.lift_dirs == frozenset() and p.drive_mode == "reverse_straight"
    closed = {"status": "unconfirmed", "drive_ready": False, "reason": "pose_unconfirmed:stroke"}
    p = teleop_policy(SAFE, feedback_ok=True, standing=True, pose=closed)
    assert not p.teleop_allowed and p.reason == "pose_unconfirmed:stroke"
    for status, reason in (("checking", "pose_checking"), ("ended:pose_revoked:int", "pose_ended")):
        p = teleop_policy(SAFE, feedback_ok=True, standing=True, pose={"status": status, "drive_ready": False,
                                                                       "reason": reason})
        assert not p.teleop_allowed and p.reason == reason


def test_the_policys_own_refusals_come_first():
    p = teleop_policy(SAFE, feedback_ok=False, standing=True, pose=READY)
    assert not p.teleop_allowed and p.reason == "no_feedback:rear_axle_speed"
    in_pocket = {**SAFE, "fork_in_pocket": {"value": True}}
    assert teleop_policy(in_pocket, feedback_ok=True, standing=True, pose=READY).reason == "fork_in_pocket"


def test_a_confirmed_pose_is_the_policy_as_before():
    confirmed = {"status": "confirmed", "drive_ready": False, "reason": None}
    assert teleop_policy(SAFE, feedback_ok=True, standing=True, pose=confirmed) == teleop_policy(
        SAFE, feedback_ok=True, standing=True)


@pytest.mark.parametrize("kind, fields", [
    ("mission_submit", {"pallet_id": "P", "destination_id": "D", "targets_revision": 0}),
    ("mission_resume", {}),
    ("mode_request", {"to": "AUTO"}),
    ("recovery", {}),
    ("sim_reset", {}),
    ("pick_request", {}),
])
@pytest.mark.parametrize("status", ["unconfirmed", "checking"])
def test_what_moves_the_truck_by_the_map_is_refused(kind, fields, status):
    rig = _rig({"status": status, "drive_ready": status == "unconfirmed", "reason": None})
    rig.a.sim_reset_allowed = rig.a.next_pick_allowed = True
    [reply] = rig.cycle(rig.msg(kind, **fields))
    assert reply["t"] == "reject" and reply["reason"] == "pose_unconfirmed"
    assert rig.a.mode == "PAUSED" and rig.m.status != "running" and rig.a.session_request is None


def test_the_teleop_is_entered_at_a_ready_stroke_and_drives_straight_back():
    rig = _rig(READY)
    [reply] = rig.cycle(rig.msg("mode_request", to="TELEOP", source="direct"))
    assert reply["t"] == "ack" and rig.a.mode == "TELEOP"
    [arm] = rig.cycle(rig.msg("arm"))
    assert arm["t"] == "ack"
    assert rig.out.policy.drive_mode == "reverse_straight" and rig.out.policy.lift_dirs == frozenset()


def test_the_teleop_is_not_entered_while_checking():
    rig = _rig({"status": "checking", "drive_ready": False, "reason": "pose_checking"})
    [reply] = rig.cycle(rig.msg("mode_request", to="TELEOP", source="direct"))
    assert reply["t"] == "reject" and reply["reason"] == "state_forbids_teleop" and reply["detail"] == "pose_checking"


def test_the_end_stops_bumps_the_control_generation_and_locks_the_controls():
    rig = _rig(READY)
    gen = rig.a.control_gen
    rig.a.end_pose(rig.t, "pose_revoked:int")
    assert rig.a.mode == "STOPPED" and rig.a.control_gen > gen and rig.a.pose["status"] == "ended:pose_revoked:int"
    rig.stand()
    for kind, fields in (("rearm", {}), ("arm", {}), ("mode_request", {"to": "TELEOP", "source": "direct"}),
                         ("mode_request", {"to": "PAUSED"}), ("recovery", {})):
        [reply] = rig.cycle(rig.msg(kind, **fields))
        assert reply["t"] == "reject" and reply["reason"] == "pose_ended", kind
    [stop] = rig.cycle(rig.msg("stop"))
    assert stop is None or stop["t"] != "reject"
    [end] = rig.cycle(rig.msg("session_end"))
    assert end["t"] == "ack" and rig.a.session_request == "end"  # design 1.4: the session end stays open


def test_an_end_during_a_mission_stops_auto():
    rig = Rig()
    auto(rig)
    rig.a.pose = {"status": "confirmed", "drive_ready": False, "reason": None}
    rig.a.end_pose(rig.t, "pose_revoked:int")
    rig.cycle()
    assert rig.a.mode == "STOPPED" and rig.out.speed_mps == 0.0


def test_the_mission_lifecycle_refuses_a_submit_and_a_resume_while_the_pose_is_not_confirmed():
    # the lifecycle's own gate (Codex draft 1 P1-1: localization_ok is checked for a submit only); its own Context
    # field, so a run with the check off is unchanged
    m = MissionCommands()
    m.queue(Submit("P", "D", 0))
    assert m.apply(0.0, Context(pose_ok=False))[-1]["reason"] == "pose_unconfirmed" and m.status == "idle"
    m.queue(Submit("P", "D", 0))
    m.apply(0.0, Context())
    m.start()
    m.status = "paused"
    m.queue(Resume())
    assert m.apply(0.0, Context(pose_ok=False))[-1]["reason"] == "pose_unconfirmed" and m.status == "paused"


def test_the_end_is_a_latch_the_next_view_cannot_lift():
    # Codex piece 2a P3
    rig = _rig(READY)
    rig.a.end_pose(rig.t, "pose_revoked:int")
    rig.a.pose = {"status": "confirmed", "drive_ready": False, "reason": None}
    rig.stand()
    [reply] = rig.cycle(rig.msg("rearm"))
    assert reply["t"] == "reject" and reply["reason"] == "pose_ended"
    assert not teleop_policy(SAFE, feedback_ok=True, standing=True, pose=rig.a._pose_view()).teleop_allowed
