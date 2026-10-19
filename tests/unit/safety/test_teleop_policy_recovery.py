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

    def __init__(self, safety, lift_m=0.0436, odom=False):
        n = itertools.count(1)
        self.a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(n)}", mission=MissionCommands())
        self.t, self.safety, self.lift_m, self.k = 0.0, safety, lift_m, 0
        self.a.connect("op", 0.0)
        self.lease = self.a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH},
                                   0.0)["lease_id"]
        self.n = itertools.count()
        self.seq = 0
        self.odom, self.x = odom, 0.0  # odom: report the odometry position (Feedback.odom_xy), x along the pocket

    def cycle(self, speed, *msgs, teleop=None, dt=DT, jump=0.0):
        """One control cycle; ``teleop``: the fields of a teleop command sent this cycle (armed first)."""
        self.t += dt
        self.k += 1
        a = self.a
        tick = a.tick(self.t)
        a.handle("op", {"t": "echo", "session_epoch": EPOCH, "lease_id": self.lease,
                        "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}}, self.t)
        self.x += speed * dt + jump  # jump: travel the speeds do not show (made during a feedback gap)
        a.observe(self.t, Feedback(self.t, speed, lift_m=self.lift_m, steering_rad=0.0,
                                   odom_xy=(self.x, 0.0) if self.odom else None))
        out = [a.handle("op", m, self.t) for m in msgs]
        if teleop is not None and not a.armed and a.preempt_latch is None:
            assert a.handle("op", self.msg("arm"), self.t)["t"] == "ack"
        if teleop is not None and a.armed:
            self.seq += 1
            a.handle("op", {"t": "teleop", "session_epoch": EPOCH, "lease_id": self.lease,
                            "control_gen": a.control_gen, "arm_gen": a.arm_gen, "seq": self.seq,
                            "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}, "mode": "direct",
                            "deadman": True, "speed_ratio": 0.0, "curvature_ratio": 0.0, "lift_dir": 0, **teleop},
                     self.t)
        a.mission_context(self.t)
        drive = {"measured_mps": speed, "measured_kappa": 0.0, "steering_error_rad": 0.0,
                 "permit": lambda *k: (1.0, "ok")}
        self.out = a.step(self.t, self.safety, (0.0, 0.0), drive)
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


REVERSE = {"speed_ratio": -0.5}  # a straight reverse asked for (the reverse_straight row's direction)
LOWER = {"lift_dir": -1}


def shaken(safety=None, seconds=2.0):
    r = Shaky(safety or in_pocket())
    for _ in range(int(seconds * 120)):
        r.cycle(jitter(r.k))
    return r


def test_the_operator_lowers_after_the_unconfirmed_entry_and_no_wheel_speed_comes_out():
    """Codex review of 8f4a799 P3: the real outputs, not the policy alone."""
    r = shaken()
    assert r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]["t"] == "ack"
    start = r.lift_m
    for _ in range(60):
        r.cycle(jitter(r.k), teleop={**LOWER, **REVERSE})
        assert r.out.speed_mps == 0.0
    assert r.out.lift_target_m < start - 0.005  # the lowering went through


def test_after_the_lowering_the_reverse_still_gives_no_speed_until_a_confirmed_stand():
    r = shaken()
    r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))
    r.lift_m = 0.005
    for _ in range(120):
        r.cycle(jitter(r.k), teleop=REVERSE)
        assert r.out.speed_mps == 0.0 and r.out.policy.reason == "stand_unconfirmed"
    first = None
    for _ in range(60):  # the shaking stops: the detector confirms, and the reverse starts on that confirmed tick
        r.cycle(0.0, teleop=REVERSE)
        if first is None and r.out.speed_mps != 0.0:
            first = r.a.stop_confirmed  # this cycle's observe judged the stand before the step
    assert r.out.speed_mps < 0.0 and first


def test_a_stand_confirmed_while_still_lifted_does_not_open_a_later_reverse_in_the_shaking():
    """Codex review of 8f4a799: the drive opens only on a confirmed tick, not after any confirmed tick."""
    r = shaken()
    r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))
    for _ in range(60):
        r.cycle(0.0)  # a brief confirmed stand while the forks are still up (lower only)
    assert r.a.stop_confirmed
    for _ in range(120):
        r.cycle(jitter(r.k))  # the rocking again
    assert not r.a.stop_confirmed
    r.lift_m = 0.005
    for _ in range(30):
        r.cycle(jitter(r.k), teleop=REVERSE)
        assert r.out.speed_mps == 0.0


def test_the_stillness_is_judged_over_a_whole_second():
    """Codex review of 8f4a799 P2: 2.01 cm/s held is 2.01 cm in a second -- over the bound."""
    r = Shaky(in_pocket())
    for _ in range(600):
        r.cycle(0.0201)
    reply = r.cycle(0.0201, r.msg("mode_request", to="TELEOP"))[0]
    assert reply["t"] == "reject" and reply["reason"] == "not_stopped"


def test_less_than_a_second_of_history_is_not_still():
    r = Shaky(in_pocket())
    for _ in range(100):  # 0.83 s
        r.cycle(jitter(r.k))
    assert r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]["reason"] == "not_stopped"


def test_a_feedback_gap_inside_the_second_is_not_still():
    r = shaken()
    r.cycle(0.0, dt=0.15)  # fresh, but nothing measured in between for 150 ms (the age limit is 100 ms)
    assert r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]["reason"] == "not_stopped"


def test_a_still_lower_only_pause_is_not_a_stop_fault_and_can_still_be_entered():
    """Codex review of 8f4a799 P2: 10 s of rocking in PAUSED made a not_stopped fault (STOPPED), and STOPPED
    could not be left without a confirmed stand -- the lowering that would end the rocking was out of reach."""
    r = shaken(seconds=12.0)
    assert r.a.mode == "PAUSED"
    assert r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]["t"] == "ack"


def test_a_stopped_lower_only_state_can_be_rearmed_while_still():
    r = shaken()
    r.a.stop(r.t, "operator")
    assert r.a.mode == "STOPPED"
    assert r.cycle(jitter(r.k), r.msg("rearm"))[0]["t"] == "ack" and r.a.mode == "PAUSED"
    assert r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]["t"] == "ack"


def test_a_rolling_stopped_truck_is_not_rearmed():
    r = Shaky(in_pocket())
    for _ in range(240):
        r.cycle(0.03)
    r.a.stop(r.t, "operator")
    assert r.cycle(0.03, r.msg("rearm"))[0]["reason"] == "not_stopped"


def test_the_state_says_when_the_stand_is_waived():
    r = shaken()
    assert r.a.stand_waiver() == "lower_only"
    rolling = Shaky(in_pocket())
    for _ in range(240):
        rolling.cycle(0.03)
    assert rolling.a.stand_waiver() is None


def lowered_and_rocking(r):
    r.lift_m = 0.005
    for _ in range(120):
        r.cycle(jitter(r.k))
    assert not r.a.stop_confirmed


def test_a_stand_confirmed_while_the_pacing_blocks_does_not_open_a_later_reverse():
    """Codex re-review P2: the waiver ends at the first non-zero speed, not at a confirmed tick that gave none."""
    r = shaken()
    r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))
    r.lift_m = 0.005
    r.a.pacing_ok = False
    for _ in range(60):  # the reverse row open but the pacing blocks it: a confirmed stand, no speed
        r.cycle(0.0)
        assert r.out.speed_mps == 0.0
    assert r.a.stop_confirmed and r.out.policy.reason == "pacing"
    r.a.pacing_ok = True
    lowered_and_rocking(r)
    for _ in range(30):
        r.cycle(jitter(r.k), teleop=REVERSE)
        assert r.out.speed_mps == 0.0


def test_a_stand_confirmed_while_disarmed_does_not_open_a_later_reverse():
    r = shaken()
    r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))
    r.lift_m = 0.005
    for _ in range(60):
        r.cycle(0.0)  # in TELEOP, not armed: no command, a confirmed stand
    assert r.a.stop_confirmed and not r.a.armed
    lowered_and_rocking(r)
    for _ in range(30):
        r.cycle(jitter(r.k), teleop=REVERSE)
        assert r.out.speed_mps == 0.0


def test_an_expired_command_latch_is_released_by_the_waiver_and_the_lowering_goes_on():
    """Codex re-review P2: a preempt latch was released only by a confirmed stand -- the lowering stayed locked."""
    r = shaken()
    r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))
    for _ in range(10):
        r.cycle(jitter(r.k), teleop=LOWER)
    for _ in range(60):  # the operator lets go: the command expires, the latch is set
        r.cycle(jitter(r.k))
    before = r.out.lift_target_m
    for _ in range(60):
        r.cycle(jitter(r.k), teleop={**LOWER, **REVERSE})
        assert r.out.speed_mps == 0.0
    assert r.a.preempt_latch is None and r.out.lift_target_m < before - 0.002


def test_a_slow_creep_is_not_waived_for_ever():
    """Codex re-review P2: 1.9 cm/s held passes every one-second window but travels 1.14 m in a minute."""
    r = Shaky(in_pocket())
    for _ in range(int(12 * 120)):
        r.cycle(0.019)
    assert r.a.stand_waiver() is None and r.a.mode == "STOPPED"


def shaken_odom(seconds=2.0):
    r = Shaky(in_pocket(), odom=True)
    for _ in range(60):
        r.cycle(0.0)  # a confirmed stand: the creep's origin is the odometry position here
    for _ in range(int(seconds * 120)):
        r.cycle(jitter(r.k))
    assert r.a.stand_waiver() == "lower_only"
    return r


def test_the_travel_across_feedback_gaps_counts_toward_the_creep():
    """Main 2026-10-19: the odometry positions before and after a gap show the real travel; 3 cm twice ends it."""
    r = shaken_odom()
    for _ in range(2):
        r.cycle(jitter(r.k), dt=0.5, jump=0.03)
        for _ in range(150):
            r.cycle(jitter(r.k))
    assert r.a.stand_waiver() is None
    assert r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]["reason"] == "not_stopped"


def test_a_gap_with_no_net_travel_leaves_the_waiver_after_a_clean_second():
    """Codex round 3: the gap's span at the next sample's speed is not travel; the positions decide."""
    r = shaken_odom()
    for _ in range(4):
        r.cycle(0.03, dt=0.5, jump=-0.015)  # the speed says 1.5 cm, the positions say none
        assert r.a.stand_waiver() is None  # main: in a gap, rocking or creep cannot be told apart
        for _ in range(30):
            r.cycle(jitter(r.k))
    for _ in range(150):
        r.cycle(jitter(r.k))
    assert r.cycle(jitter(r.k), r.msg("mode_request", to="TELEOP"))[0]["t"] == "ack"


def test_without_odometry_positions_a_gap_ends_the_waiver_until_a_confirmed_stand():
    r = shaken()
    r.cycle(jitter(r.k), dt=0.5)
    for _ in range(240):
        r.cycle(jitter(r.k))
    assert r.a.stand_waiver() is None  # the travel in the gap is unknown
    for _ in range(60):
        r.cycle(0.0)
    for _ in range(240):
        r.cycle(jitter(r.k))
    assert r.a.stand_waiver() == "lower_only"


def test_a_latch_released_by_the_waiver_in_teleop_is_recorded_as_stand_unconfirmed():
    """Codex round 3 P3: the contract names the event for this case too."""
    r = Shaky(in_pocket())
    for _ in range(60):
        r.cycle(0.0)  # a confirmed stand: an ordinary entry
    assert r.cycle(0.0, r.msg("mode_request", to="TELEOP"))[0]["t"] == "ack"
    for _ in range(240):
        r.cycle(jitter(r.k), teleop=LOWER)
    for _ in range(60):  # the operator lets go in the rocking: expired, then released under the waiver
        r.cycle(jitter(r.k))
    events = [e for e in r.a.events if e.get("event") in ("latch_released", "stand_unconfirmed")]
    assert any(e.get("stand_waiver") for e in events)
    assert any(e.get("event") == "stand_unconfirmed" and e.get("reason") == "latch_lower_only" for e in events)
