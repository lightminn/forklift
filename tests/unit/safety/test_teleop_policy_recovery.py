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


def test_a_manual_clear_whose_forks_turned_unknown_opens_nothing():
    """Codex D14 impl round 1 P1-2: a contact in the manual clear makes the relation unknown; no reverse then."""
    safety = {**FREE, "fork_in_pocket": {"value": True, "relation": "unknown"}, "load": {"loaded": True},
              "recovery": {"stage": "manual_clear", "grounded": True, "evidence": True, "aligned": True}}
    p = policy(safety, lift_m=0.0)
    assert not p.drive_allowed and p.reason == "recovery:unknown"


# --- main session 2026-10-19: entering a lower-only TELEOP without a confirmed stand ------------------------------
import itertools  # noqa: E402

from forklift_core.mission.commands import MissionCommands  # noqa: E402
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback  # noqa: E402

EPOCH, DT = 5, 1 / 120


class Shaky:
    """An arbiter fed a truck whose forks press a pallet: the speed jitters around zero (the stop detector never
    confirms), or rolls steadily."""

    def __init__(self, safety, lift_m=0.0436):
        n = itertools.count(1)
        self.a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(n)}", mission=MissionCommands())
        self.t, self.safety, self.lift_m, self.k = 0.0, safety, lift_m, 0
        self.a.connect("op", 0.0)
        self.lease = self.a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH},
                                   0.0)["lease_id"]
        self.n = itertools.count()

    def cycle(self, speed, *msgs):
        self.t += DT
        self.k += 1
        a = self.a
        tick = a.tick(self.t)
        a.handle("op", {"t": "echo", "session_epoch": EPOCH, "lease_id": self.lease,
                        "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}}, self.t)
        a.observe(self.t, Feedback(self.t, speed, lift_m=self.lift_m, steering_rad=0.0))
        out = [a.handle("op", m, self.t) for m in msgs]
        a.mission_context(self.t)
        self.out = a.step(self.t, self.safety, (0.0, 0.0))
        return out

    def msg(self, kind, **fields):
        return {"t": kind, "request_id": f"r{next(self.n)}", "session_epoch": EPOCH, "lease_id": self.lease,
                "control_gen": self.a.control_gen, **fields}


def jitter(k):
    # +-3 cm/s, flipping every half second (the forks rocking on the deck): the stop detector's 0.1 s mean stays
    # above 1.2 cm/s except at the flips, too briefly to confirm; the net travel over a second stays within 1.5 cm.
    return 0.03 if (k // 60) % 2 else -0.03


def test_a_lower_only_state_enters_teleop_without_a_confirmed_stand_and_lowers():
    r = Shaky(in_pocket())
    for _ in range(240):
        r.cycle(jitter(r.k))
    assert not r.a.stop_confirmed
    reply = r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]
    assert reply["t"] == "ack" and r.a.mode == "TELEOP"
    assert any(e.get("event") == "stand_unconfirmed" for e in r.a.events)
    r.cycle(jitter(r.k))
    assert r.out.policy.lift_dirs == frozenset({-1}) and not r.out.policy.drive_allowed


def test_after_lowering_the_drive_waits_for_a_confirmed_stand():
    r = Shaky(in_pocket())
    for _ in range(240):
        r.cycle(jitter(r.k))
    r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))
    r.lift_m = 0.005  # lowered: the reverse_straight row would open
    r.cycle(jitter(r.k))
    assert not r.out.policy.drive_allowed and r.out.policy.reason == "stand_unconfirmed"
    for _ in range(120):  # the support released, the shaking stops: the detector confirms
        r.cycle(0.0)
    assert r.a.stop_confirmed and r.out.policy.drive_allowed and r.out.policy.drive_mode == "reverse_straight"


def test_a_rolling_truck_does_not_enter_on_the_lower_only_exception():
    r = Shaky(in_pocket())
    for _ in range(240):
        r.cycle(0.03)  # 3 cm in a second: more than the 2 cm allowed
    reply = r.cycle(0.03, r.msg("mode_request", to="TELEOP"))[0]
    assert reply["t"] == "reject" and reply["reason"] == "not_stopped" and r.a.mode != "TELEOP"


def test_a_state_that_also_drives_still_needs_the_confirmed_stand():
    r = Shaky(FREE)
    for _ in range(240):
        r.cycle(jitter(r.k))
    reply = r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]
    assert reply["t"] == "reject" and reply["reason"] == "not_stopped"
