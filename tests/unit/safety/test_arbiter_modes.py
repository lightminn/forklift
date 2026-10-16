"""M-b2: modes, stop kinds, the cycle order and the mission requests (plan B3, B5, B6, B9, B11)."""

import itertools

from forklift_core.mission.commands import Context, MissionCommands
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback

EPOCH = 7
DT = 1 / 120
SAFE = {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
        "frame": {"hold": "none", "slam": True}, "mission": {"phase": "observe"}}


class Rig:
    def __init__(self):
        counter = itertools.count(1)
        self.m = MissionCommands()
        self.a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(counter)}", mission=self.m)
        self.t = 0.0
        self.speed = 0.0
        self.safety = dict(SAFE)
        self.a.connect("op", 0.0)
        reply = self.a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH}, 0.0)
        self.lease = reply["lease_id"]
        self.n = itertools.count()
        self.replies = []
        self.records = []

    def msg(self, kind, **fields):
        return {"t": kind, "request_id": f"r{next(self.n)}", "session_epoch": EPOCH, "lease_id": self.lease,
                "control_gen": self.a.control_gen, **fields}

    def cycle(self, *msgs, feedback=True, between=None, after_tick=None):
        self.t += DT
        a, m = self.a, self.m
        a.tick(self.t)  # keep the lease alive through its holder's echo
        a.handle("op", {"t": "echo", "session_epoch": EPOCH, "lease_id": self.lease,
                        "echo": {"state_seq": a.ticks.state_seq, "token": a.ticks._sent[a.ticks.state_seq][0]}},
                 self.t)
        a.observe(self.t, Feedback(self.t, self.speed) if feedback else None)
        out = [a.handle("op", msg, self.t) for msg in msgs]
        if between is not None:
            between()
        stop_kind = a.mission_context(self.t)
        self.records = m.apply(self.t, Context(stop_kind=stop_kind))
        if m.status == "accepted":
            m.start()
        if after_tick is not None:
            after_tick()
        self.replies = a.resolve(self.t, self.records)
        self.out = a.step(self.t, self.safety, (0.3 if m.status == "running" else 0.0, 0.0))
        return out

    def stand(self, cycles=40):
        for _ in range(cycles):
            self.cycle()

    def reply(self, request_id):
        return next(r for r in self.replies if r["request_id"] == request_id)


def auto(r):
    r.stand()
    msg = r.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0)
    assert r.cycle(msg) == [None]  # answered this cycle, after the lifecycle's verdict
    assert r.reply(msg["request_id"])["t"] == "ack" and r.a.mode == "AUTO" and r.m.status == "running"
    return msg


def test_a_submit_from_a_confirmed_stop_starts_auto_and_the_mission():
    r = Rig()
    msg = auto(r)
    assert r.reply(msg["request_id"])["mission_id"] == "m1" and r.out.source == "auto" and r.out.speed_mps == 0.3


def test_a_submit_before_the_stop_is_confirmed_is_refused():
    r = Rig()
    r.cycle()
    (reply,) = r.cycle(r.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0))
    assert reply["reason"] == "not_stopped" and r.m.status == "idle"


def test_paused_stops_the_mission_whether_or_not_it_takes_a_pause():
    r = Rig()
    auto(r)
    gen = r.a.control_gen
    (reply,) = r.cycle(r.msg("mode_request", to="PAUSED"))
    assert reply["t"] == "ack" and r.a.mode == "PAUSED" and r.a.control_gen > gen
    assert r.m.status == "paused" and r.out.stop_kind == "controlled"


def test_a_submit_in_the_batch_after_a_pause_does_not_release_the_running_mission():
    """Codex M-b plan round 2 P1: none is given only when the lifecycle can take the submit."""
    r = Rig()
    auto(r)
    r.a.stop_confirmed = True  # standing already (the counterexample's condition)
    pause = r.msg("mode_request", to="PAUSED")
    r.a.observe = lambda *a, **k: None  # keep the confirmed stop for this cycle
    # made after the pause's boundary (its generation), sent in the same batch
    submit = r.msg("mission_submit", pallet_id="P2", destination_id="D", targets_revision=0)
    submit["control_gen"] += 1
    r.cycle(pause, submit)
    assert r.m.status == "paused" and r.a.mode == "PAUSED"
    assert r.reply(submit["request_id"])["reason"] == "mission_active"


def test_a_boundary_between_the_messages_and_the_mission_refuses_the_request():
    r = Rig()
    r.stand()
    submit = r.msg("mission_submit", pallet_id="P", destination_id="D", targets_revision=0)
    r.cycle(submit, between=lambda: r.a.stop(r.t, "stop:console"))
    assert r.reply(submit["request_id"])["reason"] == "stale_control" and r.m.status == "idle"
    assert r.a.mode == "STOPPED"


def test_a_resume_refused_by_the_ticks_own_checks_leaves_the_arbiter_paused():
    """Codex M-b plan round 2 P2: AUTO is kept by the mission's state after the tick."""
    r = Rig()
    auto(r)
    r.cycle(r.msg("mode_request", to="PAUSED"))
    r.stand()
    resume = r.msg("mission_resume", mission_id="m1")
    r.cycle(resume, after_tick=lambda: r.m.refuse_resume(r.t, "no_resume:near_tracking"))
    assert r.reply(resume["request_id"])["t"] == "ack" and r.m.status == "paused" and r.a.mode == "PAUSED"


def test_a_mission_ending_in_auto_returns_to_paused():
    r = Rig()
    auto(r)
    r.m.finish("done", r.t)
    r.cycle()
    assert r.a.mode == "PAUSED"


def test_teleop_holds_the_mission_and_losing_the_lease_stops_and_hands_it_back():
    r = Rig()
    auto(r)
    r.cycle(r.msg("mode_request", to="PAUSED"))
    r.stand()
    (reply,) = r.cycle(r.msg("mode_request", to="TELEOP"))
    assert reply["t"] == "ack" and r.a.mode == "TELEOP" and r.m.status == "held"
    assert r.out.stop_kind == "none" and r.a.mission_stop_kind() == "controlled"
    r.a._expire(r.t + 5.0)  # the holder went quiet
    assert r.a.mode == "PAUSED" and r.a.preempt_latch == "lease_expired" and r.m.status == "paused"
    assert r.a.mission_stop_kind() == "preempt"


def test_no_direct_auto_to_teleop_and_teleop_needs_a_confirmed_stop_and_the_state():
    r = Rig()
    auto(r)
    assert r.cycle(r.msg("mode_request", to="TELEOP"))[0]["reason"] == "invalid"
    r.cycle(r.msg("mode_request", to="PAUSED"))
    r.safety = dict(SAFE, fork_in_pocket={"value": True})
    r.stand()
    reply = r.cycle(r.msg("mode_request", to="TELEOP"))[0]
    assert (reply["reason"], reply["detail"]) == ("state_forbids_teleop", "fork_in_pocket")


def test_a_stop_that_does_not_happen_becomes_stopped():
    r = Rig()
    auto(r)
    r.speed = 0.3  # never stands
    r.cycle(r.msg("mode_request", to="PAUSED"))
    for _ in range(int(10.0 / DT) + 2):
        r.cycle()
    assert r.a.mode == "STOPPED" and r.a.events[-1]["reason"] == "not_stopped"


def test_rearm_only_from_a_confirmed_stop_and_without_feedback_never():
    r = Rig()
    r.cycle({"t": "stop", "request_id": "S1"})
    assert r.a.mode == "STOPPED"
    assert r.cycle(r.msg("rearm"))[0]["reason"] == "not_stopped"
    for _ in range(5):
        r.cycle(feedback=False)
    assert r.cycle(r.msg("rearm"), feedback=False)[0]["reason"] == "no_feedback:rear_axle_speed"
    r.stand()
    assert r.cycle(r.msg("rearm"))[0]["t"] == "ack" and r.a.mode == "PAUSED"


def test_losing_feedback_in_teleop_latches_a_preempt():
    r = Rig()
    r.stand()
    r.cycle(r.msg("mode_request", to="TELEOP"))
    assert r.a.mode == "TELEOP"
    r.cycle(feedback=False)
    assert r.a.preempt_latch == "no_feedback" and r.out.stop_kind == "preempt" and r.out.speed_mps == 0.0
    assert r.out.policy.reason == "no_feedback:rear_axle_speed"


def test_a_pause_is_refused_inside_the_near_contract():
    r = Rig()
    auto(r)
    r.safety = dict(SAFE, near={"active": True})
    r.cycle()
    assert r.cycle(r.msg("mode_request", to="PAUSED"))[0]["reason"] == "near_tracking_active" and r.a.mode == "AUTO"
