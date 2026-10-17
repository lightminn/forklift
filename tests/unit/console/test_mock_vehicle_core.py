import itertools

import pytest

from console.mock_vehicle.core import Config, MockVehicle


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock_only():
    return Clock()


@pytest.fixture
def env():
    clock = Clock()
    tokens = (f"tok{i}" for i in itertools.count())
    v = MockVehicle(clock, lambda: next(tokens), Config())
    return v, clock


def _rid():
    _rid.n += 1
    return f"r{_rid.n}"


_rid.n = 0


def lease(v, cid="a"):
    w = v.connect(cid)
    ack = v.handle(
        cid,
        {
            "t": "lease_request",
            "request_id": _rid(),
            "session_epoch": w["session_epoch"],
        },
    )
    assert ack["t"] == "ack", ack
    return w["session_epoch"], ack["lease_id"]


def req(v, cid, t, epoch, lid, **kw):
    return v.handle(
        cid,
        {
            "t": t,
            "request_id": kw.pop("request_id", _rid()),
            "session_epoch": epoch,
            "lease_id": lid,
            "control_gen": kw.pop("control_gen", v.control_gen),
            **kw,
        },
    )


def teleop(v, cid, epoch, lid, tick, **kw):
    msg = {
        "t": "teleop",
        "session_epoch": epoch,
        "lease_id": lid,
        "control_gen": v.control_gen,
        "arm_gen": v.arm_gen,
        "seq": kw.pop("seq"),
        "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
        "mode": kw.pop("mode", "direct"),
        "deadman": True,
        "speed_ratio": kw.pop("speed", 0.5),
        "curvature_ratio": 0.0,
        "lift_dir": 0,
        **kw,
    }
    return v.handle(cid, msg)


def into_teleop(v, cid, epoch, lid):
    assert req(v, cid, "mode_request", epoch, lid, to="TELEOP")["t"] == "ack"
    assert req(v, cid, "arm", epoch, lid)["t"] == "ack"


def test_second_client_is_observer_without_lease(env):
    v, _ = env
    lease(v, "a")
    w = v.connect("b")
    r = v.handle(
        "b",
        {"t": "lease_request", "request_id": "x", "session_epoch": w["session_epoch"]},
    )
    assert r["t"] == "reject" and r["reason"] == "busy"


def test_teleop_moves_then_stops_at_absolute_deadline(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    clock.t += 0.29  # echo arrives 290 ms after the tick was sent
    teleop(v, "a", epoch, lid, tick, seq=1)
    v.step(0.005)
    assert v.applied_speed_mps > 0.0
    clock.t += 0.02  # 310 ms after the tick: past the 300 ms deadline, no new packet
    v.step(0.005)
    assert (
        v.applied_speed_mps == 0.0 and v.stop_kind == "safety_preempt" and not v.armed
    )


def test_teleop_stops_when_commands_stop(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    for i in range(5):
        tick = v.make_tick()
        teleop(v, "a", epoch, lid, tick, seq=i + 1)
        clock.t += 0.05
        v.step(0.05)
    assert v.applied_speed_mps > 0.0
    for _ in range(8):
        clock.t += 0.05
        v.step(0.05)
    assert v.applied_speed_mps == 0.0 and not v.armed


def test_echo_older_than_deadline_on_arrival_is_rejected(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    clock.t += 0.31
    teleop(v, "a", epoch, lid, tick, seq=1)
    v.step(0.005)
    assert v.applied_speed_mps == 0.0 and v.last_teleop_reject == "expired"


def test_wrong_token_is_rejected(env):
    v, _ = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = dict(v.make_tick(), token="guess")
    teleop(v, "a", epoch, lid, tick, seq=1)
    assert v.last_teleop_reject == "bad_echo"


def test_video_deadline_is_earlier_of_two(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    v.record_frame(7)  # acquired at t=100.00
    clock.t += 0.40
    tick = v.make_tick()  # sent at 100.40; command deadline 100.70
    teleop(v, "a", epoch, lid, tick, seq=1, mode="video", frame_echo={"frame_id": 7})
    v.step(0.005)
    assert v.applied_speed_mps > 0.0
    clock.t += 0.06  # 100.46 > 100.00 + 0.50 - 0.05 = 100.45 (video deadline)
    v.step(0.005)
    assert v.applied_speed_mps == 0.0 and not v.armed


def test_stop_locks_until_rearm_and_paused_request_cannot_bypass(env):
    v, clock = env
    epoch, lid = lease(v)
    w = v.connect("obs")
    assert (
        v.handle(
            "obs",
            {
                "t": "stop",
                "request_id": "s1",
                "session_epoch": w["session_epoch"],
                "source": "console",
            },
        )["t"]
        == "ack"
    )
    assert v.mode == "STOPPED"
    r = req(v, "a", "mode_request", epoch, lid, to="PAUSED")
    assert r["t"] == "reject" and r["reason"] == "stopped_locked"
    assert (
        req(v, "a", "rearm", epoch, lid)["reason"] == "not_stopped"
    )  # stop not yet confirmed
    for _ in range(12):  # 0.6 s below the speed threshold
        clock.t += 0.05
        v.step(0.05)
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"
    assert v.mode == "PAUSED"


def test_late_resume_after_stop_is_rejected(env):
    v, clock = env
    epoch, lid = lease(v)
    assert (
        req(
            v,
            "a",
            "mission_submit",
            epoch,
            lid,
            pallet_id="P-01",
            destination_id="D-A",
            targets_revision=v.targets_revision,
        )["t"]
        == "ack"
    )
    mission_id = v.mission["mission_id"]
    assert req(v, "a", "mission_pause", epoch, lid, mission_id=mission_id)["t"] == "ack"
    stale_gen = v.control_gen  # valid until the STOP below
    w = v.connect("obs")
    v.handle(
        "obs",
        {
            "t": "stop",
            "request_id": "s2",
            "session_epoch": w["session_epoch"],
            "source": "console",
        },
    )
    for _ in range(12):
        clock.t += 0.05
        v.step(0.05)
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"
    late = req(
        v,
        "a",
        "mission_resume",
        epoch,
        lid,
        mission_id=mission_id,
        control_gen=stale_gen,
    )
    assert late["t"] == "reject" and late["reason"] == "stale_control"


def test_duplicate_request_returns_cached_result_without_rerun(env):
    v, _ = env
    epoch, lid = lease(v)
    r1 = req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        request_id="dup",
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    gen_after = v.control_gen
    r2 = req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        request_id="dup",
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
        control_gen=gen_after - 1,
    )
    assert r1 == r2 and v.missions_created == 1


def test_stale_targets_revision_is_rejected(env):
    v, _ = env
    epoch, lid = lease(v)
    r = req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision - 1,
    )
    assert r["t"] == "reject" and r["reason"] == "stale_targets"


def test_auto_continues_when_lease_expires(env):
    v, clock = env
    epoch, lid = lease(v)
    req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    gen = v.control_gen
    for _ in range(30):  # 1.5 s without echoes
        clock.t += 0.05
        v.step(0.05)
    assert v.mode == "AUTO" and v.lease is None and v.control_gen > gen


def test_closing_a_duplicate_connection_keeps_the_holders_lease(env):
    v, _ = env
    w = v.connect("a", conn_id="c1")
    lid = v.handle(
        "a",
        {"t": "lease_request", "request_id": "l1", "session_epoch": w["session_epoch"]},
        conn_id="c1",
    )["lease_id"]
    v.connect(
        "a", conn_id="c2"
    )  # second window with the copied client_id, observer only
    v.disconnect("a", conn_id="c2")
    assert v.lease is not None and v.lease["lease_id"] == lid


def test_late_close_of_an_old_connection_keeps_the_new_lease(env):
    v, clock = env
    w = v.connect("a", conn_id="old")
    v.handle(
        "a",
        {"t": "lease_request", "request_id": "l1", "session_epoch": w["session_epoch"]},
        conn_id="old",
    )
    clock.t += 1.5
    v.step(0.05)  # the old connection went silent: lease expired, reserved for "a"
    v.connect("a", conn_id="new")
    new = v.handle(
        "a",
        {"t": "lease_request", "request_id": "l2", "session_epoch": w["session_epoch"]},
        conn_id="new",
    )
    assert new["t"] == "ack"
    v.disconnect("a", conn_id="old")  # the old socket finally closes
    assert v.lease is not None and v.lease["lease_id"] == new["lease_id"]


def test_reconnect_within_grace_gets_new_lease_id(env):
    v, clock = env
    epoch, lid = lease(v)
    v.disconnect("a")
    clock.t += 2.0
    v.step(0.05)
    w2 = v.connect("b")
    r = v.handle(
        "b",
        {"t": "lease_request", "request_id": "q", "session_epoch": w2["session_epoch"]},
    )
    assert r["reason"] == "lease_reserved"
    w = v.connect("a")
    r = v.handle(
        "a",
        {"t": "lease_request", "request_id": "z", "session_epoch": w["session_epoch"]},
    )
    assert r["t"] == "ack" and r["lease_id"] != lid
    assert req(v, "a", "arm", epoch, lid)["reason"] == "stale_lease"


def test_teleop_refused_with_forks_in_pocket_but_withdraw_allowed(env):
    v, _ = env
    epoch, lid = lease(v)
    v.debug_set_phase("insert", forks_in_pocket=True)
    r = req(v, "a", "mode_request", epoch, lid, to="TELEOP")
    assert r["t"] == "reject" and r["reason"] == "state_forbids_teleop"
    assert req(v, "a", "recovery", epoch, lid, action="withdraw_straight")["t"] == "ack"


def test_teleop_allowed_while_carrying_in_transport_with_loaded_caps(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    into_teleop(v, "a", epoch, lid)
    for i in range(40):
        tick = v.make_tick()
        teleop(v, "a", epoch, lid, tick, seq=i + 1, speed=1.0)
        clock.t += 0.05
        v.step(0.05)
    assert 0.0 < v.applied_speed_mps <= 0.30 + 1e-9


def _stop_and_settle(v, clock, rid="s9"):
    w = v.connect("obs")
    v.handle(
        "obs",
        {
            "t": "stop",
            "request_id": rid,
            "session_epoch": w["session_epoch"],
            "source": "console",
        },
    )
    for _ in range(12):
        clock.t += 0.05
        v.step(0.05)
    return w


def test_stopped_lock_also_refuses_mission_and_recovery(env):
    v, clock = env
    epoch, lid = lease(v)
    _stop_and_settle(v, clock)
    r = req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    assert r["reason"] == "stopped_locked"
    assert (
        req(v, "a", "recovery", epoch, lid, action="withdraw_straight")["reason"]
        == "stopped_locked"
    )
    assert v.mode == "STOPPED"


def test_resent_stop_after_rearm_stops_again(env):
    # contract 1, rule 0 (agreed with main): STOP is never cached; a late duplicate stops again (safe side)
    v, clock = env
    epoch, lid = lease(v)
    w = _stop_and_settle(v, clock, rid="once")
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"
    gen = v.control_gen
    again = v.handle(
        "obs",
        {
            "t": "stop",
            "request_id": "once",
            "session_epoch": w["session_epoch"],
            "source": "console",
        },
    )
    assert again["t"] == "ack" and v.mode == "STOPPED" and v.control_gen == gen + 1


def test_full_result_cache_refuses_new_requests_but_not_stop(clock_only):
    v = MockVehicle(clock_only, lambda: "tok", Config(result_cache_max=2))
    w = v.connect("a")
    e = w["session_epoch"]
    assert (
        v.handle("a", {"t": "targets_request", "request_id": "q1", "session_epoch": e})[
            "t"
        ]
        == "ack"
    )
    assert (
        v.handle("a", {"t": "targets_request", "request_id": "q2", "session_epoch": e})[
            "t"
        ]
        == "ack"
    )
    r = v.handle("a", {"t": "lease_request", "request_id": "q3", "session_epoch": e})
    assert r["t"] == "reject" and r["reason"] == "busy"
    assert (
        v.handle("a", {"t": "targets_request", "request_id": "q1", "session_epoch": e})[
            "t"
        ]
        == "ack"
    )  # cached
    assert (
        v.handle(
            "a",
            {"t": "stop", "request_id": "s", "session_epoch": e, "source": "console"},
        )["t"]
        == "ack"
    )
    assert v.mode == "STOPPED"


def _idle(v, clock, epoch, lid, n):
    for _ in range(n):
        tick = v.make_tick()
        v.handle(
            "a",
            {
                "t": "echo",
                "session_epoch": epoch,
                "lease_id": lid,
                "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
            },
        )
        clock.t += 0.05
        v.step(0.05)


def _drive(v, clock, epoch, lid, n, speed=1.0, lift=0, start_seq=1):
    for i in range(n):
        tick = v.make_tick()
        teleop(v, "a", epoch, lid, tick, seq=start_seq + i, speed=speed, lift_dir=lift)
        clock.t += 0.05
        v.step(0.05)


def test_drive_output_is_cut_the_tick_the_phase_forbids_it(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 20)
    assert v.applied_speed_mps > 0.2  # moving at the loaded cap
    v.debug_set_phase(
        "lower", forks_in_pocket=True, carried="P-01"
    )  # state changes after entry
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=100, speed=1.0)
    v.step(0.005)
    assert v.make_state()["drive_allowed"] is False and v.applied_speed_mps == 0.0


def test_lift_and_drive_are_exclusive(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 10)  # moving
    lift0 = v.lift_m
    _drive(
        v, clock, epoch, lid, 3, speed=1.0, lift=1, start_seq=50
    )  # both requested while moving
    assert (
        v.applied_speed_mps == 0.0 and v.lift_m == lift0
    )  # drive cut, lift waits for stop
    _drive(
        v, clock, epoch, lid, 15, speed=0.0, lift=1, start_seq=60
    )  # stop confirmed, lifting
    assert v.lift_m > lift0
    _drive(
        v, clock, epoch, lid, 1, speed=1.0, lift=0, start_seq=80
    )  # first tick after release: lift moved last tick
    assert v.applied_speed_mps == 0.0
    _drive(
        v, clock, epoch, lid, 3, speed=1.0, lift=0, start_seq=81
    )  # then driving resumes normally
    assert v.applied_speed_mps > 0.0


def test_second_stop_while_stopped_invalidates_a_late_rearm(env):
    v, clock = env
    epoch, lid = lease(v)
    w = _stop_and_settle(v, clock, rid="s1")
    old_gen = v.control_gen  # a rearm is prepared with this generation
    v.handle(
        "obs",
        {
            "t": "stop",
            "request_id": "s2",
            "session_epoch": w["session_epoch"],
            "source": "console",
        },
    )
    assert v.mode == "STOPPED" and v.control_gen == old_gen + 1
    late = req(v, "a", "rearm", epoch, lid, control_gen=old_gen)
    assert late["t"] == "reject" and late["reason"] == "stale_control"


def test_second_pause_invalidates_a_late_resume(env):
    v, clock = env
    epoch, lid = lease(v)
    req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    mid = v.mission["mission_id"]
    assert req(v, "a", "mission_pause", epoch, lid, mission_id=mid)["t"] == "ack"
    old_gen = v.control_gen  # a resume is prepared with this generation
    assert (
        req(v, "a", "mission_pause", epoch, lid, mission_id=mid)["t"] == "ack"
    )  # new request id
    assert v.mode == "PAUSED" and v.control_gen == old_gen + 1
    late = req(
        v, "a", "mission_resume", epoch, lid, mission_id=mid, control_gen=old_gen
    )
    assert late["t"] == "reject" and late["reason"] == "stale_control"


def test_pause_decelerates_and_auto_drives(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 10, speed=1.0)
    moving, gen = v.applied_speed_mps, v.control_gen
    assert req(v, "a", "mode_request", epoch, lid, to="PAUSED")["t"] == "ack"
    assert (
        v.armed is False
        and v.control_gen == gen + 1
        and v.stop_kind == "controlled_decel"
    )
    v.step(0.005)
    assert 0.0 < v.applied_speed_mps < moving  # controlled deceleration, not a cut
    _idle(v, clock, epoch, lid, 40)  # echoes keep the lease while waiting
    assert (
        req(
            v,
            "a",
            "mission_submit",
            epoch,
            lid,
            pallet_id="P-01",
            destination_id="D-A",
            targets_revision=v.targets_revision,
        )["t"]
        == "ack"
    )
    assert v.mode == "AUTO" and v.stop_kind == "none"
    _idle(v, clock, epoch, lid, 60)  # observe phase 1.5 s, then approach
    assert v.applied_speed_mps > 0.0


def test_permission_direction_holds_until_stop_is_confirmed(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    v.applied_speed_mps = (
        0.019  # forward residue below the 0.02 threshold, stop not confirmed
    )
    v.stopped_confirmed = False
    v._last_nonzero_sign = 1
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=1, speed=-1.0)
    v.step(0.005)
    assert v.last_permission_direction == 1


def test_old_teleop_command_does_not_resume_after_new_lease_and_arm(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=1, speed=1.0)
    v.disconnect("a")  # lease lost; the accepted command is still before its deadline
    w = v.connect("a")
    ack = v.handle(
        "a",
        {"t": "lease_request", "request_id": "re", "session_epoch": w["session_epoch"]},
    )
    assert req(v, "a", "arm", epoch, ack["lease_id"])["t"] == "ack"
    v.applied_speed_mps = 0.0
    v.step(0.005)
    assert v.applied_speed_mps == 0.0


@pytest.mark.parametrize(
    "bad",
    [
        {"seq": "1"},
        {"speed_ratio": float("nan")},
        {"speed_ratio": 1.5},
        {"lift_dir": 2},
        {"mode": "auto"},
    ],
)
def test_malformed_teleop_is_dropped(env, bad):
    v, _ = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    tick = v.make_tick()
    msg = {
        "t": "teleop",
        "session_epoch": epoch,
        "lease_id": lid,
        "control_gen": v.control_gen,
        "arm_gen": v.arm_gen,
        "seq": 1,
        "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
        "mode": "direct",
        "deadman": True,
        "speed_ratio": 0.5,
        "curvature_ratio": 0.0,
        "lift_dir": 0,
    }
    msg.update(bad)
    assert v.handle("a", msg) is None
    assert v.last_teleop_reject == "invalid"


def test_reliable_request_without_request_id_is_invalid(env):
    v, _ = env
    w = v.connect("a")
    r = v.handle("a", {"t": "lease_request", "session_epoch": w["session_epoch"]})
    assert r["t"] == "reject" and r["reason"] == "invalid"


def test_instances_created_back_to_back_have_distinct_session_epochs():
    epochs = {MockVehicle(Clock(), lambda: "t").session_epoch for _ in range(200)}
    assert len(epochs) == 200


def test_leaving_teleop_disarms_and_reentry_needs_a_new_arm(env):
    v, _ = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    old_arm = v.arm_gen
    assert req(v, "a", "mode_request", epoch, lid, to="PAUSED")["t"] == "ack"
    assert req(v, "a", "mode_request", epoch, lid, to="TELEOP")["t"] == "ack"
    assert v.armed is False
    tick = v.make_tick()
    msg = {
        "t": "teleop",
        "session_epoch": epoch,
        "lease_id": lid,
        "control_gen": v.control_gen,
        "arm_gen": old_arm,
        "seq": 99,
        "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
        "mode": "direct",
        "deadman": True,
        "speed_ratio": 0.5,
        "curvature_ratio": 0.0,
        "lift_dir": 0,
    }
    v.handle("a", msg)
    assert v.last_teleop_reject == "not_armed"


def test_cancel_while_stopped_keeps_the_lock(env):
    v, clock = env
    epoch, lid = lease(v)
    req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    _stop_and_settle(v, clock)
    assert (
        req(v, "a", "mission_cancel", epoch, lid, mission_id=v.mission["mission_id"])[
            "t"
        ]
        == "ack"
    )
    assert v.mode == "STOPPED"
    assert (
        req(v, "a", "mode_request", epoch, lid, to="TELEOP")["reason"]
        == "stopped_locked"
    )


def test_lease_can_be_taken_while_stopped_so_someone_can_rearm(env):
    v, clock = env
    _stop_and_settle(v, clock)  # an observer stops before anyone holds the lease
    epoch, lid = lease(v, "a")
    assert req(v, "a", "rearm", epoch, lid)["t"] == "ack"


def test_slew_then_permission_cap_never_exceeds_cap(env):
    v, _ = env
    v.applied_speed_mps = 0.5  # debug_output(target, curvature, explicit cap, dt) bypasses the world check
    out = v.debug_output(
        target_mps=0.5, curvature_inv_m=0.0, permission_cap_mps=0.1, dt_s=1 / 120
    )
    assert out <= 0.1 + 1e-12


@pytest.mark.parametrize(
    "pallet_id,dest",
    [("P-01", "D-A"), ("P-01", "D-B"), ("P-02", "D-A"), ("P-02", "D-B")],
)
def test_mock_mission_completes_for_every_pallet_and_drop(pallet_id, dest):
    clock = Clock()
    tokens = (f"k{i}" for i in itertools.count())
    v = MockVehicle(clock, lambda: next(tokens))
    w = v.connect("a")
    e = w["session_epoch"]
    lid = v.handle("a", {"t": "lease_request", "request_id": "l", "session_epoch": e})[
        "lease_id"
    ]
    r = v.handle(
        "a",
        {
            "t": "mission_submit",
            "request_id": "m",
            "session_epoch": e,
            "lease_id": lid,
            "control_gen": v.control_gen,
            "pallet_id": pallet_id,
            "destination_id": dest,
            "targets_revision": v.targets_revision,
        },
    )
    assert r["t"] == "ack"
    for _ in range(120 * 300):
        tick = v.make_tick()
        v.handle(
            "a",
            {
                "t": "echo",
                "session_epoch": e,
                "lease_id": lid,
                "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
            },
        )
        clock.t += 1 / 120
        v.step(1 / 120)
        if v.mission["status"] == "done":
            break
    assert v.mission["status"] == "done", (v.mission["phase"], v.permit)


# --- Codex review of Tasks 1-2 (round 1) -------------------------------------------------


def test_repeated_teleop_mode_request_does_not_reopen_old_seqs(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    t1 = v.make_tick()
    teleop(v, "a", epoch, lid, t1, seq=1)
    teleop(v, "a", epoch, lid, v.make_tick(), seq=2, deadman=False)
    assert req(v, "a", "mode_request", epoch, lid, to="TELEOP")["t"] == "ack"
    teleop(v, "a", epoch, lid, t1, seq=1)  # replayed old command
    assert v.last_teleop_reject == "stale_seq"


def test_reversal_waits_for_stop_confirmation(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 10, speed=1.0)
    assert v.applied_speed_mps > 0.2
    seen = []
    for i in range(
        12
    ):  # reverse requested; stop is confirmed only 0.5 s after reaching 0
        tick = v.make_tick()
        teleop(v, "a", epoch, lid, tick, seq=100 + i, speed=-1.0)
        clock.t += 0.05
        v.step(0.05)
        seen.append((v.applied_speed_mps, v.stopped_confirmed))
    assert all(s >= 0.0 for s, confirmed in seen if not confirmed)
    _drive(v, clock, epoch, lid, 6, speed=-1.0, start_seq=200)
    assert v.applied_speed_mps < 0.0


def test_auto_playback_never_moves_against_the_applied_speed(env):
    v, clock = env
    epoch, lid = lease(v)
    req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    for _ in range(120 * 120):
        before = v.pose
        tick = v.make_tick()
        v.handle(
            "a",
            {
                "t": "echo",
                "session_epoch": epoch,
                "lease_id": lid,
                "echo": {"state_seq": tick["state_seq"], "token": tick["token"]},
            },
        )
        clock.t += 1 / 120
        v.step(1 / 120)
        import math as _m

        dx, dy = v.pose.x_m - before.x_m, v.pose.y_m - before.y_m
        along = dx * _m.cos(before.yaw_rad) + dy * _m.sin(before.yaw_rad)
        assert along * v.applied_speed_mps >= -1e-9, (
            v.mission["phase"],
            along,
            v.applied_speed_mps,
        )
        if v.mission["status"] == "done":
            break


def test_cancel_keeps_load_and_fork_state(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    v.lift_m = 0.12
    assert (
        req(v, "a", "mission_cancel", epoch, lid, mission_id=v.mission["mission_id"])[
            "t"
        ]
        == "ack"
    )
    s = v.make_state()
    assert s["load"]["state"] == "loaded" and s["limits"][
        "forward_mps"
    ] == pytest.approx(0.30)
    v2 = MockVehicle(Clock(), lambda: "t2")
    e2, l2 = lease(v2)
    v2.debug_set_phase("insert", forks_in_pocket=True)
    assert (
        req(v2, "a", "mission_cancel", e2, l2, mission_id=v2.mission["mission_id"])["t"]
        == "ack"
    )
    assert v2.forks_in_pocket is True
    assert (
        req(v2, "a", "mode_request", e2, l2, to="TELEOP")["reason"]
        == "state_forbids_teleop"
    )


def _pause_and_settle(v, clock, epoch, lid):
    assert (
        req(v, "a", "mission_pause", epoch, lid, mission_id=v.mission["mission_id"])[
            "t"
        ]
        == "ack"
    )
    _idle(v, clock, epoch, lid, 14)


def test_resume_with_forks_in_pocket_fails_state_verified(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("insert", forks_in_pocket=True)
    _pause_and_settle(v, clock, epoch, lid)
    r = req(v, "a", "mission_resume", epoch, lid, mission_id=v.mission["mission_id"])
    assert r["t"] == "reject"
    steps = {s["name"]: s for s in r["resume_steps"]}
    assert (
        steps["state_verified"]["result"] == "failed"
        and steps["state_verified"]["reason"] == "fork_in_pocket"
    )


def test_resume_refused_when_the_rechecked_path_is_blocked(env):
    v, clock = env
    epoch, lid = lease(v)
    req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    _idle(v, clock, epoch, lid, 50)  # into approach
    _pause_and_settle(v, clock, epoch, lid)
    p = v.pose
    import math as _m

    ahead = (p.x_m + _m.cos(p.yaw_rad) * 1.3, p.y_m + _m.sin(p.yaw_rad) * 1.3)
    v.world.boxes.append((ahead[0], ahead[1], 0.3, 0.3))
    r = req(v, "a", "mission_resume", epoch, lid, mission_id=v.mission["mission_id"])
    assert r["t"] == "reject"
    last = r["resume_steps"][-1]
    assert last["name"] == "path_rechecked" and last["result"] == "failed"
    assert last["reason"].startswith("path_blocked:")
    assert v.mode == "PAUSED"


def test_command_shows_the_request_even_when_drive_is_blocked(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    _idle(v, clock, epoch, lid, 12)
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=1, speed=1.0, lift_dir=1, curvature_ratio=0.5)
    v.step(0.005)
    s = v.make_state()
    assert s["command"]["speed_mps"] == pytest.approx(0.5)
    assert s["command"]["curvature_inv_m"] == pytest.approx(0.18)
    assert s["applied"]["speed_mps"] == 0.0


def test_targets_revision_moves_when_a_pallet_changes_state(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("insert", forks_in_pocket=True)
    rev = v.targets_revision
    v._next_phase()  # insert -> lift: the pallet becomes carried
    assert v.targets_revision > rev


def test_wrong_epoch_teleop_is_reported_as_stale_epoch(env):
    v, _ = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    teleop(v, "a", epoch + 1, lid, v.make_tick(), seq=1)
    assert v.last_teleop_reject == "stale_epoch"


# --- Codex review of Tasks 1-2 (round 2) -------------------------------------------------


def test_new_mission_refused_while_forks_in_pocket_or_loaded(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("insert", forks_in_pocket=True)
    req(v, "a", "mission_cancel", epoch, lid, mission_id=v.mission["mission_id"])
    r = req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-02",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    assert r["t"] == "reject" and r["reason"] == "fork_in_pocket"
    assert v.forks_in_pocket is True
    v.forks_in_pocket = False
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    req(v, "a", "mission_cancel", epoch, lid, mission_id=v.mission["mission_id"])
    r = req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-02",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    assert r["t"] == "reject" and r["reason"] == "load_present"


def test_resume_checks_the_whole_remaining_path(env):
    v, clock = env
    epoch, lid = lease(v)
    req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    _idle(v, clock, epoch, lid, 50)  # into approach
    _pause_and_settle(v, clock, epoch, lid)
    v._replan()
    leg = v.mission["legs"][v.mission["leg_index"]]
    mid = leg.points[len(leg.points) // 2]
    import math as _m

    ahead = (mid[0] + _m.cos(mid[2]) * 1.3, mid[1] + _m.sin(mid[2]) * 1.3)
    v.world.circles.append((ahead[0], ahead[1], 0.12))
    r = req(v, "a", "mission_resume", epoch, lid, mission_id=v.mission["mission_id"])
    assert r["t"] == "reject" and r["resume_steps"][-1]["reason"].startswith(
        "path_blocked:"
    )


def test_playback_moves_no_further_than_the_applied_speed_allows(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("insert", forks_in_pocket=True)
    v.applied_speed_mps = 0.08  # still creeping forward when the withdraw is requested
    v.stopped_confirmed = False
    assert req(v, "a", "recovery", epoch, lid, action="withdraw_straight")["t"] == "ack"
    import math as _m

    for _ in range(240):
        before = v.pose
        clock.t += 1 / 120
        v._still_since = v._still_since  # untouched; stop confirmation runs normally
        v.step(1 / 120)
        moved = _m.hypot(v.pose.x_m - before.x_m, v.pose.y_m - before.y_m)
        assert moved <= abs(v.applied_speed_mps) / 120 + 1e-6, (
            moved,
            v.applied_speed_mps,
        )
        if v.mission["status"] != "running":
            break


def test_carried_pallets_do_not_publish_a_moving_pose(env):
    v, clock = env
    epoch, lid = lease(v)
    v.debug_set_phase("transport", forks_in_pocket=False, carried="P-01")
    t = v.make_targets()
    carried = next(p for p in t["pallets"] if p["pallet_id"] == "P-01")
    assert carried["state"] == "carried" and carried["pose"] is None


# --- Codex review of Tasks 1-2 (round 3) -------------------------------------------------


def test_permission_follows_residual_motion_even_after_stop_confirmation(env):
    v, clock = env
    epoch, lid = lease(v)
    into_teleop(v, "a", epoch, lid)
    v.applied_speed_mps = 0.018  # below the stop threshold but still rolling forward
    v.stopped_confirmed = True
    tick = v.make_tick()
    teleop(v, "a", epoch, lid, tick, seq=1, speed=-1.0)
    v.step(1 / 120)
    assert v.applied_speed_mps >= 0.0
    if v.applied_speed_mps > 0.0:
        assert v.last_permission_direction == 1


def test_cancelled_job_pallet_is_an_obstacle_again_in_teleop(env):
    v, clock = env
    epoch, lid = lease(v)
    req(
        v,
        "a",
        "mission_submit",
        epoch,
        lid,
        pallet_id="P-01",
        destination_id="D-A",
        targets_revision=v.targets_revision,
    )
    p = next(q for q in v.world.pallets if q.pallet_id == "P-01")
    from console.mock_vehicle.world import CHASSIS, Pose

    # put the truck 0.15 m short of the pallet face, still in the approach phase
    v.mission["phase"] = "approach"
    v.pose = Pose(
        p.pose.x_m - p.depth_m / 2 - CHASSIS.nose_from_rear_m - 0.15, p.pose.y_m, 0.0
    )
    assert (
        req(v, "a", "mission_cancel", epoch, lid, mission_id=v.mission["mission_id"])[
            "t"
        ]
        == "ack"
    )
    into_teleop(v, "a", epoch, lid)
    _drive(v, clock, epoch, lid, 5, speed=1.0)
    assert v.applied_speed_mps == 0.0 and v.permit[1].startswith("occupied")
