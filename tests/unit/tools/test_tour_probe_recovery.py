"""The console probe's tour scenario through a disarm (gyro-heading plan 4.5 F1, Codex F1-F3 review P2): against a
fake vehicle -- an arbiter that disarms on cue, takes ``arm`` with a new generation and refuses commands of an older
one -- the tour arms again, even when the ack is lost, and keeps watching the states while a replan plans."""

import asyncio
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402
import survey_tour  # noqa: E402  (the module tour_scenario imports, as the probe does)


class FakeVehicle:
    """A rear-axle point driven by the teleop it accepts, states at 20 Hz of wall time (factor 1)."""

    def __init__(self, disarm_at_s=(), lose_ack=False, shift_at_s=None):
        self.state, self.report, self.closed_code = None, {}, None
        self.lease_id, self.arm_gen, self.server_gen, self.epoch = None, 0, 0, 1
        self.armed, self.mode, self.latch = False, "PAUSED", False
        self.x = self.y = self.v = self.k = 0.0
        self.cmd = (0.0, 0.0)
        self.disarm_at, self.lose_ack, self.shift_at = list(disarm_at_s), lose_ack, shift_at_s
        self.refused = 0
        self.seq = 0
        self.t0 = None

    def standing(self, s):
        return abs(s.get("rear_axle_speed_mps") or 0.0) < 0.01

    async def open(self):
        self.t0 = time.monotonic()
        self._task = asyncio.create_task(self._run())

    async def _run(self):
        dt = 0.02
        while True:
            now = time.monotonic() - self.t0
            if self.disarm_at and now >= self.disarm_at[0] and self.armed:
                self.disarm_at.pop(0)
                self.armed, self.latch = False, True  # the arbiter's preempt: zero speed, the curvature held
            if self.shift_at is not None and now >= self.shift_at:
                self.shift_at, self.y = None, self.y + 0.7  # off the path: the tracker fails and replans
            target_v = self.cmd[0] * 0.5 if self.armed else 0.0
            if self.armed:
                self.k += max(-0.5 * dt, min(0.5 * dt, self.cmd[1] * 0.36 - self.k))
            self.v += max(-0.8 * dt, min(0.8 * dt, target_v - self.v))
            self.x += self.v * dt
            if self.latch and abs(self.v) < 0.01:
                self.latch = False
            self.seq += 1
            self.state = {"state_seq": self.seq, "mode": self.mode, "armed": self.armed, "arm_gen": self.server_gen,
                          "rear_axle_speed_mps": self.v, "stopped_confirmed": abs(self.v) < 0.01, "feedback_ok": True,
                          "stop_latch": {"kind": "preempt"} if self.latch else None,
                          "applied": {"speed_mps": self.v, "curvature_inv_m": self.k},
                          "steering_rad": [math.atan(self.k * 0.66)] * 2,
                          "pose": {"x_m": self.x, "y_m": self.y, "yaw_rad": 0.0, "correction_version": 1},
                          "pacing": {"target_factor": 1.0, "ok": True}, "limits": {}}
            await asyncio.sleep(dt)

    async def until(self, pred, timeout, hz=20, label=""):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state is not None and pred(self.state):
                return True
            await asyncio.sleep(1 / hz)
        return False

    async def request(self, kind, timeout=5.0, **fields):
        if kind == "lease_request":
            return {"t": "ack", "lease_id": "L"}
        if kind == "mode_request":
            self.mode = fields.get("to", self.mode)
            return {"t": "ack"}
        if kind == "arm":
            if self.latch or abs(self.v) >= 0.01:
                return {"t": "reject", "reason": "invalid"}
            self.server_gen += 1
            self.armed = True
            return None if self.lose_ack else {"t": "ack", "arm_gen": self.server_gen}
        return {"t": "ack"}

    async def request_until_standing(self, kind, **fields):
        return await self.request(kind, **fields)

    async def teleop(self, speed, curvature, deadman=True, **_):
        if self.armed and self.arm_gen == self.server_gen:
            self.cmd = (speed, curvature)
        else:
            self.refused += 1  # stale_arm / disarmed: nothing changes


def _tour(tmp_path, length_m=1.5):
    n = int(length_m / 0.05) + 1
    poses = np.column_stack((np.linspace(0, length_m, n), np.zeros(n), np.zeros(n)))
    path = tmp_path / "tour.npz"
    np.savez(path, poses=poses, directions=np.ones(n, dtype=np.int8), curvatures_inv_m=np.zeros(n),
             tour=np.asarray([[length_m, 0.0, 0.0]]), plan_input=np.asarray('{"stub": true}'))
    return path


def _args(tour):
    return SimpleNamespace(tour_path=tour, tour_cruise_mps=0.3, tour_stall_s=60.0, tour_pacing_wait_s=900.0,
                           end_session=False)


def _run(vehicle, tour, limit_s=40.0):
    async def go():
        await asyncio.wait_for(lcp.tour_scenario(vehicle, _args(tour)), limit_s)
        vehicle._task.cancel()
    asyncio.run(go())
    return vehicle.report["tour"]


def test_a_lost_arm_ack_is_recovered_through_the_states_generation(tmp_path):
    vehicle = FakeVehicle(disarm_at_s=(2.0,), lose_ack=True)
    tour = _run(vehicle, _tour(tmp_path))
    names = [e["event"] for e in tour["events"]]
    assert tour["end"] == "arrived", names
    assert "disarmed" in names and "rearm_sent" in names and "rearmed" in names
    assert vehicle.arm_gen == vehicle.server_gen == 2  # the probe took the state's generation, the ack being lost


def test_a_disarm_while_a_replan_plans_is_watched_and_armed_again(tmp_path, monkeypatch):
    calls = []

    def slow_replan(plan_input, left, start, driven=(), judged=None, radius_m=0.0, *, budget_s=60.0, hard_s=120.0,
                    log=None):
        calls.append(time.monotonic())
        time.sleep(3.0)  # long enough for the vehicle to disarm meanwhile
        x0, y0 = start[0], start[1]
        n = 31
        poses = np.column_stack((np.linspace(x0, x0 + 1.5, n), np.full(n, y0), np.zeros(n)))
        plan = SimpleNamespace(success=True, poses=poses, directions=np.ones(n, dtype=np.int8),
                               curvatures_inv_m=np.zeros(n), length_m=1.5, clearance_m=0.3, status="success")
        return left, plan, None, None

    monkeypatch.setattr(survey_tour, "replan_tour_bounded", slow_replan)
    vehicle = FakeVehicle(shift_at_s=2.0, disarm_at_s=(4.5,))
    tour = _run(vehicle, _tour(tmp_path, 3.0), limit_s=60.0)
    names = [e["event"] for e in tour["events"]]
    assert calls and "tracker_failed" in names and "replanned" in names, names
    disarmed = next(e for e in tour["events"] if e["event"] == "disarmed")
    rearmed = next(e for e in tour["events"] if e["event"] == "rearmed")
    replanned = next(e for e in tour["events"] if e["event"] == "replanned")
    assert disarmed["wall"] < replanned["wall"] and rearmed["wall"] < replanned["wall"]  # handled during the plan
    io = next(e for e in tour["events"] if e["event"] == "replan_io")
    assert len(io["path"]) == 31 and io["driven"] and len(io["driven"][0]) == 4 and io["elapsed_s"] >= 3.0
    assert io["error"] is None
    assert tour["end"] == "arrived", names


def test_a_replan_past_its_hard_limit_is_a_failed_replan():
    left, plan, coverage, error = survey_tour.replan_tour_bounded({"stub": True}, [], (0.0, 0.0, 0.0), hard_s=0.0)
    assert plan is None and coverage is None and error == "timeout"


def test_a_lost_ack_of_the_first_arm_still_drives(tmp_path):
    """Codex F1-F3 recheck P2: no later disarm opens a recovery; the probe sends the armed state's generation."""
    vehicle = FakeVehicle(lose_ack=True)
    tour = _run(vehicle, _tour(tmp_path))
    assert tour["end"] == "arrived" and vehicle.arm_gen == vehicle.server_gen == 1
