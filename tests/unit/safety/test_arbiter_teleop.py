"""M-b3: teleoperation commands -- order of checks, the absolute deadline, video freshness, arming (plan B10)."""

import itertools

import pytest

from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback

EPOCH = 3
SAFE = {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
        "frame": {"hold": "none", "slam": True}, "mission": {"phase": "observe"}}


def teleop_rig():
    counter = itertools.count(1)
    a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(counter)}")
    a.connect("op", 0.0)
    lease_id = a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH}, 0.0)["lease_id"]
    t = 0.0
    for _ in range(40):  # stand
        t += 1 / 120
        a.observe(t, Feedback(t, 0.0))
        a.step(t, SAFE, (0.0, 0.0))
    a.last_safety = SAFE
    base = {"session_epoch": EPOCH, "lease_id": lease_id}
    assert a.handle("op", {**base, "t": "mode_request", "request_id": "M", "control_gen": a.control_gen,
                           "to": "TELEOP"}, t)["t"] == "ack"
    arm = a.handle("op", {**base, "t": "arm", "request_id": "A", "control_gen": a.control_gen}, t)
    return a, base, t, arm["arm_gen"]


def command(a, base, arm_gen, tick, seq, **fields):
    return {**base, "t": "teleop", "control_gen": a.control_gen, "arm_gen": arm_gen, "seq": seq,
            "echo": {"state_seq": tick["state_seq"], "token": tick["token"]}, "mode": "direct", "deadman": True,
            "speed_ratio": 0.5, "curvature_ratio": 0.0, "lift_dir": 0, **fields}


def test_an_accepted_command_expires_at_the_tick_send_time_plus_the_age_limit_without_any_packet():
    a, base, t, arm_gen = teleop_rig()
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1), t + 0.29)
    assert a.teleop["deadline"] == pytest.approx(t + 0.30) and a.armed
    a.observe(t + 0.30, Feedback(t + 0.30, 0.2))
    assert a.armed  # at the deadline: still valid
    gen = a.control_gen
    a.observe(t + 0.3001, Feedback(t + 0.3001, 0.2))
    assert not a.armed and a.preempt_latch == "expired" and a.control_gen == gen + 1 and a.teleop is None


@pytest.mark.parametrize("change, reason", [
    ({"session_epoch": EPOCH + 1}, "stale_epoch"), ({"lease_id": "x"}, "stale_lease"),
    ({"control_gen": -1}, "stale_control"), ({"arm_gen": 0}, "stale_arm"), ({"seq": 0}, "stale_seq"),
    ({"echo": {"state_seq": 99, "token": "x"}}, "bad_echo")])
def test_each_check_drops_the_command_with_its_reason(change, reason):
    a, base, t, arm_gen = teleop_rig()
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1), t)
    assert a.handle("op", {**command(a, base, arm_gen, tick, 2), **change}, t) is None
    assert a.last_teleop_reject == reason


def test_a_command_already_past_its_deadline_is_dropped():
    a, base, t, arm_gen = teleop_rig()
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1), t + 0.31)
    assert a.last_teleop_reject == "expired" and a.teleop is None


def test_a_stale_video_frame_drops_and_disarms_and_the_deadline_is_the_earlier():
    a, base, t, arm_gen = teleop_rig()
    a.register_frame(5, t - 0.30)
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1, mode="video", frame_echo={"frame_id": 5}), t)
    assert a.teleop["deadline"] == pytest.approx(t - 0.30 + 0.5 - 0.15)  # the video bound comes first
    a.observe(t + 0.051, Feedback(t + 0.051, 0.0))  # standing: the latch is released at once
    assert not a.armed and [e["reason"] for e in a.events if e["event"] == "control_gen"][-1] == "stale_video"
    a2, base2, t2, arm2 = teleop_rig()
    a2.register_frame(5, t2 - 0.40)  # 0.40 + 0.15 > 0.5
    tick2 = a2.tick(t2)
    a2.handle("op", command(a2, base2, arm2, tick2, 1, mode="video", frame_echo={"frame_id": 5}), t2)
    assert a2.last_teleop_reject == "stale_video" and not a2.armed


def test_a_late_command_from_an_old_arm_generation_does_not_rearm():
    a, base, t, arm_gen = teleop_rig()
    a.handle("op", {**base, "t": "input_state", "state": "hidden"}, t)
    assert not a.armed
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1), t)
    assert a.last_teleop_reject == "not_armed" and a.teleop is None


def test_ratios_are_clamped_and_garbage_is_zero():
    a, base, t, arm_gen = teleop_rig()
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1, speed_ratio=3.0, curvature_ratio="x"), t)
    assert a.teleop["speed_ratio"] == 1.0 and a.teleop["curvature_ratio"] == 0.0


def test_the_arbiter_drives_teleop_through_the_chain_from_the_measured_steering():
    a, base, t, arm_gen = teleop_rig()
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1, curvature_ratio=1.0), t)
    asked = []

    def permit(curvatures, direction, check):
        asked.append(curvatures)
        return 1.0, "ok"

    drive = {"measured_mps": 0.0, "measured_kappa": 0.2, "steering_error_rad": 0.0, "permit": permit}
    a.observe(t + 0.01, Feedback(t + 0.01, 0.0))
    out = a.step(t + 0.01, SAFE, (0.0, 0.0), drive)
    assert out.source == "teleop" and out.speed_mps == pytest.approx(0.8 / 120)
    assert out.curvature_inv_m == pytest.approx(0.2 + 0.5 / 120) and asked[0][0] == 0.2
    assert out.permit == (1.0, "ok")


def test_a_stale_frame_at_receipt_is_a_preempt_and_a_boundary_and_a_future_frame_is_stale():
    """Codex M-b review round 2 P1 x2."""
    a, base, t, arm_gen = teleop_rig()
    a.register_frame(5, t)
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1, mode="video", frame_echo={"frame_id": 5}), t)
    assert a.teleop is not None
    a.register_frame(6, t - 0.4)
    gen = a.control_gen
    a.handle("op", command(a, base, arm_gen, tick, 2, mode="video", frame_echo={"frame_id": 6}), t)
    assert a.last_teleop_reject == "stale_video" and a.preempt_latch == "stale_video" and a.control_gen == gen + 1
    b, base_b, tb, arm_b = teleop_rig()
    b.register_frame(9, tb + 10.0)
    tick_b = b.tick(tb)
    b.handle("op", command(b, base_b, arm_b, tick_b, 1, mode="video", frame_echo={"frame_id": 9}), tb)
    assert b.last_teleop_reject == "stale_video" and not b.armed


def test_the_state_carries_the_applied_command_only_when_given_and_the_resume_steps_per_step():
    from forklift_core.safety.state import resume_steps, state_message

    a, base, t, arm_gen = teleop_rig()
    out = a.step(t, SAFE, (0.0, 0.0))
    assert state_message(a, out, SAFE, now_mono=t)["applied"] is None
    applied = {"speed_mps": 0.18, "curvature_inv_m": 0.19, "steering_rad": [0.1, 0.09]}
    assert state_message(a, out, SAFE, now_mono=t, applied=applied)["applied"]["speed_mps"] == 0.18
    assert resume_steps({"step": "path_rechecked", "reason": "replanned"}) == {
        "stop_confirmed": "ok", "state_verified": "ok", "frame_reexpressed": "ok", "path_rechecked": "pending"}
    refused = {"step": "state_verified", "reason": "no_resume:near_tracking", "refused": True}
    assert resume_steps(refused)["state_verified"] == "failed(no_resume:near_tracking)"
    # A STOP mid-resume (no refusal on the check) is pending; a later unrelated record does not hide a failure.
    assert resume_steps({"step": "path_rechecked", "reason": "replanned"})["path_rechecked"] == "pending"
    a.mission = __import__("forklift_core.mission.commands", fromlist=["MissionCommands"]).MissionCommands()
    a.mission.log.append({"command": "submit", "accepted": False, "reason": "mission_active"})
    paused = dict(SAFE, mission={"id": "m1", "status": "paused", "phase": "approach", "resume": refused})
    msg = state_message(a, out, paused, now_mono=t)["mission"]
    assert msg["reason"] == "no_resume:near_tracking" and msg["resume_steps"]["state_verified"].startswith("failed")
    held = dict(SAFE, mission={"id": "m1", "status": "held", "phase": "observe"})
    assert state_message(a, out, held, now_mono=t)["mission"]["reason"] == "teleop"


def test_step_rechecks_the_deadline_after_a_long_mission_tick():
    """Codex M-c plan round 1 P1: observe at 0.29 s after the tick, step at 0.49 s -- the command has expired."""
    a, base, t, arm_gen = teleop_rig()
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1), t)
    a.observe(t + 0.29, Feedback(t + 0.29, 0.2))
    assert a.armed
    out = a.step(t + 0.49, SAFE, (0.0, 0.0), {"measured_mps": 0.2, "measured_kappa": 0.0, "steering_error_rad": 0.0,
                                              "permit": lambda *k: (1.0, "ok")})
    assert out.speed_mps == 0.0 and out.stop_kind == "preempt" and not a.armed


def test_the_physics_step_gate_sees_an_expired_command_and_a_stop():
    a, base, t, arm_gen = teleop_rig()
    tick = a.tick(t)
    a.handle("op", command(a, base, arm_gen, tick, 1), t)
    assert a.gate(t + 0.30) is False
    assert a.gate(t + 0.31) is True and a.preempt_latch == "expired"
    b, base_b, tb, _ = teleop_rig()
    b.handle("op", {"t": "stop", "request_id": "S"}, tb)
    assert b.gate(tb) is True and b.mode == "STOPPED"


def test_a_late_disconnect_of_an_old_connection_leaves_the_new_one():
    """Codex M-c plan round 1 P1: A and B of one client; A's late close must not remove B (B's STOP is answered)."""
    import itertools

    counter = itertools.count(1)
    a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(counter)}")
    a.connect("op", 0.0, conn_id="A")
    a.connect("op", 0.1, conn_id="B")
    a.disconnect("op", 0.2, conn_id="A")
    assert a.handle("op", {"t": "stop", "request_id": "S"}, 0.3, conn_id="B")["t"] == "ack"
    lease = a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH}, 0.4, conn_id="A")
    assert lease is None  # the old connection's requests do not count
    a.disconnect("op", 0.5, conn_id="B")
    assert "op" not in a.clients


def test_without_real_time_there_is_no_teleop_drive():
    a, base, t, arm_gen = teleop_rig()
    a.pacing_ok = False
    out = a.step(t, SAFE, (0.0, 0.0))
    assert not out.policy.drive_allowed and out.policy.reason == "pacing"
