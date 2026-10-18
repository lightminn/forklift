"""M-b5: the arbiter in a closed loop with a point-mass truck, a synthetic grid and a client (plan section 5)."""

import itertools
import math

import numpy as np
import pytest

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.mission.commands import Context, MissionCommands
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.planning.geometry import Footprint
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback
from forklift_core.safety.state import state_message
from forklift_core.safety.teleop_drive import arc_permit

EPOCH = 11
DT = 1 / 120
FOOT = Footprint(1.29, 0.17, 0.36)
BODY = Footprint(0.75, 0.17, 0.36)
STOP = StoppingModel(latency_s=0.15, decel_mps2=1.5, margin_m=0.05)
CONFIG = PermissionConfig(STOP, envelope_offset_m=0.01, evidence_max_age_s=0.5)
SAFE = {"load": {"loaded": False, "basis": "phase"}, "fork_in_pocket": {"value": False}, "near": None,
        "frame": {"hold": "none", "slam": True}, "mission": {"phase": "observe"}}
OBSTACLE_X = 4.0


class Loop:
    """The cycle of plan B3 with a truck whose speed follows the applied command with a 0.1 s lag."""

    def __init__(self):
        counter = itertools.count(1)
        self.m = MissionCommands()
        self.a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(counter)}", mission=self.m)
        self.t, self.x, self.v = 0.0, 0.0, 0.0
        state = np.full((240, 120), FREE, dtype=np.uint8)  # x -1..11, y -3..3
        i = int(math.floor((OBSTACLE_X + 1.0) / 0.05))
        state[i:i + 2, 50:70] = OCCUPIED
        self.snap = GridSnapshot(state, np.where(state == FREE, 0.0, np.nan), 0.0, (0.0, 0.0, 0.0), 0, 1,
                                 {"low": 0.0}, -1.0, -3.0, 0.05)
        self.perm = DrivePermission(CONFIG)
        self.a.connect("op", 0.0)
        self.lease = self.a.handle("op", {"t": "lease_request", "request_id": "L", "session_epoch": EPOCH},
                                   0.0)["lease_id"]
        self.n = itertools.count()
        self.last_tick = None
        self.client_alive = True
        self.seq = itertools.count(1)
        self.state = None

    def msg(self, kind, **fields):
        return {"t": kind, "request_id": f"r{next(self.n)}", "session_epoch": EPOCH, "lease_id": self.lease,
                "control_gen": self.a.control_gen, **fields}

    def permit(self, curvatures, direction, check):
        snap = self.snap
        snap.free_stamp[np.isfinite(snap.free_stamp)] = self.t  # fresh evidence every cycle
        return arc_permit(self.perm, snap, self.t, {"low": self.t}, pose=(self.x, 0.0, 0.0), curvatures=curvatures,
                          direction=direction, check_speed_mps=check, footprint=FOOT, own_footprint=BODY)

    def cycle(self, *msgs, teleop=None):
        a = self.a
        self.t += DT
        if round(self.t / DT) % 6 == 0:  # a tick every 50 ms
            self.last_tick = a.tick(self.t)
        a.observe(self.t, Feedback(self.t, self.v))
        replies = [a.handle("op", m, self.t) for m in msgs]
        if self.client_alive and self.last_tick is not None:
            echo = {"state_seq": self.last_tick["state_seq"], "token": self.last_tick["token"]}
            a.handle("op", {"t": "echo", "session_epoch": EPOCH, "lease_id": self.lease, "echo": echo}, self.t)
            if teleop is not None and a.mode == "TELEOP" and a.armed:
                a.handle("op", {"t": "teleop", "session_epoch": EPOCH, "lease_id": self.lease,
                                "control_gen": a.control_gen, "arm_gen": a.arm_gen, "seq": next(self.seq),
                                "echo": echo, "mode": "direct", "lift_dir": 0, **teleop}, self.t)
        stop_kind = a.mission_context(self.t)
        records = self.m.apply(self.t, Context(stop_kind=stop_kind))
        a.resolve(self.t, records)
        out = a.step(self.t, SAFE, (0.0, 0.0), {"measured_mps": self.v, "measured_kappa": 0.0,
                                                 "steering_error_rad": 0.0, "permit": self.permit})
        # The point-mass back end runs the arbiter's output unchanged: that is its applied command.
        applied = {"speed_mps": out.speed_mps, "curvature_inv_m": out.curvature_inv_m}
        self.state = state_message(a, out, SAFE, now_mono=self.t, rear_axle_speed_mps=self.v, applied=applied)
        self.v += (out.speed_mps - self.v) * DT / 0.1
        self.x += self.v * DT
        return out, replies

    def to_teleop_armed(self):
        for _ in range(40):
            self.cycle()
        assert self.cycle(self.msg("mode_request", to="TELEOP"))[1][0]["t"] == "ack"
        assert self.cycle(self.msg("arm"))[1][0]["t"] == "ack"


def test_full_forward_teleop_stops_short_of_an_obstacle():
    loop = Loop()
    loop.to_teleop_armed()
    peak = 0.0
    for _ in range(int(8 / DT)):  # stops about 6.5 s in, 0.054 m short of the cells
        loop.cycle(teleop={"deadman": True, "speed_ratio": 1.0, "curvature_ratio": 0.0})
        peak = max(peak, loop.v)
        assert loop.x + FOOT.front_m < OBSTACLE_X  # the forks never reach the obstacle's cells
    assert peak > 0.3 and abs(loop.v) < 0.01 and loop.state["permit"]["reason"] in ("occupied", "limit")
    assert loop.state["mode"] == "TELEOP" and loop.state["applied"]["speed_mps"] == pytest.approx(0.0, abs=1e-3)


def test_a_client_that_goes_silent_stops_the_truck_by_the_deadline_alone():
    loop = Loop()
    loop.to_teleop_armed()
    for _ in range(int(2 / DT)):
        loop.cycle(teleop={"deadman": True, "speed_ratio": 1.0, "curvature_ratio": 0.0})
    assert loop.v > 0.2
    loop.client_alive = False
    last_echoed = loop.a.ticks._sent[loop.last_tick["state_seq"]][1]  # the last tick the client answered
    first_zero = None
    for _ in range(int(0.40 / DT)):
        out, _ = loop.cycle()
        if first_zero is None and out.speed_mps == 0.0:
            first_zero = loop.t
    # The first cycle past the deadline (that tick's send time + 0.30 s) outputs zero -- no packet needed.
    assert first_zero is not None and first_zero <= last_echoed + 0.30 + DT + 1e-9
    assert not loop.a.armed


def test_stop_then_rearm_then_teleop_again():
    loop = Loop()
    loop.to_teleop_armed()
    for _ in range(int(1 / DT)):
        loop.cycle(teleop={"deadman": True, "speed_ratio": 1.0, "curvature_ratio": 0.0})
    out, _ = loop.cycle({"t": "stop", "request_id": "S"})
    assert out.mode == "STOPPED" and out.speed_mps == 0.0
    assert loop.cycle(loop.msg("rearm"))[1][0]["reason"] == "not_stopped"  # still rolling
    for _ in range(80):
        loop.cycle()
    assert loop.cycle(loop.msg("rearm"))[1][0]["t"] == "ack" and loop.a.mode == "PAUSED"
    for _ in range(40):
        loop.cycle()
    loop.to_teleop_armed()
    assert loop.a.mode == "TELEOP" and loop.a.armed


def test_the_state_says_whether_the_stop_is_confirmed_and_names_the_mission_targets():
    # the console enables rearm on stopped_confirmed and shows pallet -> destination (C3)
    loop = Loop()
    for _ in range(40):
        loop.cycle()
    assert loop.state["stopped_confirmed"] is True  # standing still, fresh feedback
    safety = dict(SAFE, mission={"id": "m1", "status": "running", "phase": "approach",
                                 "pallet_id": "P-1", "destination_id": "D-1"})
    out = loop.a.step(loop.t, safety, (0.0, 0.0), {"measured_mps": 0.0, "measured_kappa": 0.0,
                                                   "steering_error_rad": 0.0, "permit": loop.permit})
    st = state_message(loop.a, out, safety, now_mono=loop.t)
    assert st["mission"]["pallet_id"] == "P-1" and st["mission"]["destination_id"] == "D-1"


def test_a_loaded_state_names_the_pallet_on_the_forks():
    # the console draws the carried pallet from load.pallet_id (Codex C3 review P3)
    loop = Loop()
    safety = dict(SAFE, load={"loaded": True, "basis": "phase"},
                  mission={"id": "m1", "status": "running", "phase": "transport",
                           "pallet_id": "P-1", "destination_id": "D-1"})
    out = loop.a.step(loop.t, safety, (0.0, 0.0), {"measured_mps": 0.0, "measured_kappa": 0.0,
                                                   "steering_error_rad": 0.0, "permit": loop.permit})
    st = state_message(loop.a, out, safety, now_mono=loop.t)
    assert st["load"] == {"state": "loaded", "basis": "phase", "pallet_id": "P-1"}
    empty = state_message(loop.a, out, dict(safety, load={"loaded": False, "basis": "phase"}), now_mono=loop.t)
    assert empty["load"]["pallet_id"] is None


def test_measured_steering_and_lift_height_are_sent_when_given_and_null_otherwise():
    # contract 6: steering_rad and lift_m are what the vehicle measured; the live runner reads its joints (C3 b)
    loop = Loop()
    out = loop.a.step(loop.t, SAFE, (0.0, 0.0), {"measured_mps": 0.0, "measured_kappa": 0.0,
                                                 "steering_error_rad": 0.0, "permit": loop.permit})
    st = state_message(loop.a, out, SAFE, now_mono=loop.t)
    assert st["steering_rad"] is None and st["lift_m"] is None
    st = state_message(loop.a, out, SAFE, now_mono=loop.t, steering_rad=[0.1, 0.08], lift_m=0.05)
    assert st["steering_rad"] == [0.1, 0.08] and st["lift_m"] == 0.05


def test_an_ended_mission_shows_its_end_reason_not_an_earlier_refused_resume():
    """Codex persistent-console stage 1 P3: resume refused -> cancel -> confirmed standing showed the old refusal."""
    loop = Loop()
    loop.m.log.append({"t": 1.0, "command": "end", "accepted": True, "reason": "operator", "mission_id": "m1",
                       "status": "cancelled"})
    safety = dict(SAFE, mission={"id": "m1", "status": "cancelled", "phase": "observe",
                                 "resume": {"refused": True, "reason": "path_blocked"}})
    out = loop.a.step(loop.t, safety, (0.0, 0.0), {"measured_mps": 0.0, "measured_kappa": 0.0,
                                                   "steering_error_rad": 0.0, "permit": loop.permit})
    assert state_message(loop.a, out, safety, now_mono=loop.t)["mission"]["reason"] == "operator"
    running = dict(safety, mission=dict(safety["mission"], status="paused"))
    assert state_message(loop.a, out, running, now_mono=loop.t)["mission"]["reason"] == "path_blocked"


def test_a_live_load_names_its_pallet_after_the_mission_ended():
    """Plan D5a (Codex round 1 P3): the mission is cleared the tick after its end; the carried pallet stays drawn."""
    loop = Loop()
    safety = dict(SAFE, load={"loaded": True, "basis": "phase+cargo", "pallet_id": "P-2", "state": "confirmed"},
                  mission={"id": None, "status": "idle", "phase": None, "pallet_id": None, "destination_id": None})
    out = loop.a.step(loop.t, safety, (0.0, 0.0), {"measured_mps": 0.0, "measured_kappa": 0.0,
                                                   "steering_error_rad": 0.0, "permit": loop.permit})
    st = state_message(loop.a, out, safety, now_mono=loop.t)
    assert st["load"] == {"state": "loaded", "basis": "phase+cargo", "pallet_id": "P-2"}
