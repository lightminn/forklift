"""The probe's confirm scenario (docs/plans/2026-10-17-realtime-sim.md 5.17): STOPs in AUTO and in TELEOP with their send
times, teleop holds with their intervals, the command-expiry test, and the scenario's order -- against a fake probe."""

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakeProbe:
    """Requests are logged with their send time; the state moves as the vehicle would answer them."""

    def __init__(self, clock, *, stop_reply="ack", teleop_entry="ack", rearm="ack", moving=True):
        self.clock = clock
        self.state = {"mode": "AUTO", "rear_axle_speed_mps": 0.2 if moving else 0.0, "armed": True,
                      "mission": {"mission_id": "m1", "status": "running", "phase": "transport"}}
        self.requests, self.teleops, self.report = [], [], {}
        self.arm_gen = 0
        self._stop_reply, self._teleop_entry, self._rearm = stop_reply, teleop_entry, rearm

    def standing(self, s):
        return abs(s.get("rear_axle_speed_mps") or 0.0) < 0.01

    def event(self, name, **fields):
        self.report.setdefault("events", []).append({"event": name, **fields})

    async def request(self, kind, **fields):
        self.requests.append((kind, fields, self.clock.t))
        if kind == "stop":
            if self._stop_reply == "ack":
                self.state = {**self.state, "mode": "STOPPED", "rear_axle_speed_mps": 0.0, "armed": False}
            return {"t": self._stop_reply, "request_id": f"r{len(self.requests)}"} if self._stop_reply else None
        if kind == "rearm":
            if self._rearm == "ack":
                self.state = {**self.state, "mode": "PAUSED"}
            return {"t": self._rearm}
        if kind == "mode_request":
            to = fields.get("to")
            if to == "TELEOP" and self._teleop_entry != "ack":
                return {"t": "reject", "reason": "teleop_forbidden", "detail": self._teleop_entry}
            self.state = {**self.state, "mode": to}
            return {"t": "ack"}
        if kind == "arm":
            self.state = {**self.state, "armed": True}
            return {"t": "ack", "arm_gen": 3}
        return {"t": "ack"}

    async def request_until_standing(self, kind, tries=20, **fields):
        return await self.request(kind, **fields)

    async def teleop(self, speed, curvature, deadman=True, mode="direct", lift_dir=0):
        self.teleops.append((self.clock.t, speed, curvature, deadman, mode, lift_dir))
        moving = deadman and (speed != 0.0) and self.state.get("mode") == "TELEOP" and self.state.get("armed")
        self.state = {**self.state, "rear_axle_speed_mps": 0.15 if moving else 0.0}

    async def until(self, pred, timeout, hz=20, label=""):
        return bool(pred(self.state))

    async def sleep(self, seconds):
        self.clock.t = round(self.clock.t + seconds, 6)


def run(coro):
    return asyncio.run(coro)


def test_a_stop_records_its_send_time_and_request_and_waits_for_the_stand_and_rearm():
    clock = Clock()
    p = FakeProbe(clock)
    rec = run(lcp.confirm_stop(p, "A1", clock=clock))
    assert rec["label"] == "A1" and rec["sent_mono"] == 100.0 and rec["moving_at_send"] is True
    assert rec["stopped"] and rec["standing"] and rec["rearm"] == "ack"
    assert [k for k, _, _ in p.requests] == ["stop", "rearm"]
    assert p.requests[0][1] == {"source": "key"}


def test_a_stop_not_acknowledged_is_recorded_and_not_rearmed():
    clock = Clock()
    p = FakeProbe(clock, stop_reply=None)
    rec = run(lcp.confirm_stop(p, "A2", clock=clock))
    assert rec["reply"] is None and not rec["stopped"] and rec["rearm"] is None
    assert [k for k, _, _ in p.requests] == ["stop"]


def test_a_hold_sends_commands_and_records_its_interval_mode_and_ratios():
    clock = Clock()
    p = FakeProbe(clock)
    p.state = {**p.state, "mode": "TELEOP", "armed": True}
    rec = run(lcp.hold_teleop(p, "forward", 0.4, 0.3, 0.0, factor=0.4, clock=clock))  # 0.4 sim s = 1 wall s
    assert rec["label"] == "forward" and rec["mode"] == "direct" and rec["deadman"] is True
    assert rec["speed_ratio"] == 0.3 and rec["curvature_ratio"] == 0.0 and rec["lift_dir"] == 0
    assert rec["start_mono"] == 100.0 and rec["end_mono"] >= 101.0 - 1e-9
    held = [t for t in p.teleops if t[0] < rec["end_mono"]]
    assert len(held) >= 19 and all(t[1] == 0.3 and t[3] for t in held)
    assert all(t[1] == 0.0 for t in p.teleops if t[0] >= rec["end_mono"])  # then zero commands until it stands


def test_a_stop_inside_a_hold_is_sent_while_moving_and_the_hold_ends_there():
    clock = Clock()
    p = FakeProbe(clock)
    p.state = {**p.state, "mode": "TELEOP", "armed": True}
    rec = run(lcp.hold_teleop(p, "forward_stop", 3.0, 0.3, 0.0, stop_after_sim_s=1.5, factor=1.0, clock=clock))
    assert rec["stop"]["label"] == "T" and rec["stop"]["moving_at_send"] is True
    assert rec["stop"]["sent_mono"] == pytest.approx(101.5, abs=0.06)
    assert rec["end_mono"] == pytest.approx(rec["stop"]["sent_mono"], abs=1e-9)


def test_the_expiry_test_moves_first_then_stops_only_the_commands_for_its_window():
    clock = Clock()
    p = FakeProbe(clock)
    p.state = {**p.state, "mode": "TELEOP", "armed": True}
    rec = run(lcp.command_expiry(p, factor=0.4, clock=clock))
    assert rec["moving_before"] is True
    assert rec["silence_s"] == pytest.approx(0.5 / 0.4)
    assert rec["silence_end_mono"] - rec["silence_start_mono"] >= rec["silence_s"] - 1e-9
    sent = [t for t, *_ in p.teleops]
    assert not any(rec["silence_start_mono"] < t < rec["silence_end_mono"] for t in sent)  # no command in the window
    assert "armed_after" in rec and "last_teleop_reject_after" in rec
    assert sent[-1] < rec["silence_start_mono"] and sent[0] >= 100.0
    assert rec["silence_start_mono"] - 100.0 >= 1.0 / 0.4 - 1e-9  # moved for 1 sim s first


def test_teleop_entry_refused_for_pacing_is_counted():
    clock = Clock()
    p = FakeProbe(clock, teleop_entry="pacing")
    rec = run(lcp.confirm_enter_teleop(p, tries=3, clock=clock))
    assert rec["entered"] is False and rec["pacing_refusals"] == 3


def test_the_scenario_is_registered_and_its_order_is_the_registration():
    import inspect
    src = inspect.getsource(lcp.confirm_scenario)
    order = ["confirm_stop(p, \"A1\"", "hold_teleop(p, \"loaded_forward\"", "confirm_stop(p, \"A2\"",
             "hold_teleop(p, \"forward\"", "hold_teleop(p, \"reverse\"", "hold_teleop(p, \"left\"",
             "hold_teleop(p, \"right\"", "hold_teleop(p, \"lift_up\"", "hold_teleop(p, \"lift_down\"",
             "hold_teleop(p, \"video_forward\"", "hold_teleop(p, \"video_reverse\"", "command_expiry(p"]
    pos = [src.index(k) for k in order]
    assert pos == sorted(pos)
    assert "stop_after_sim_s=1.5" in src.split("hold_teleop(p, \"forward\"")[1].split(")")[0]
    assert src.count("factor=f") >= 9  # every hold in simulated seconds at the declared factor
    assert "\"confirm\"" in inspect.getsource(lcp.main)


def test_the_ratios_give_the_registered_speeds_and_curvature():
    lim = lcp.CONFIRM_LIMITS
    assert lcp.CONFIRM_FORWARD_RATIO * lim["direct_forward_mps"] == pytest.approx(0.15)
    assert lcp.CONFIRM_REVERSE_RATIO * lim["direct_reverse_mps"] == pytest.approx(0.15)
    assert lcp.CONFIRM_CURVATURE_RATIO * lim["curvature_inv_m"] == pytest.approx(0.18)
    assert lcp.CONFIRM_LOADED_RATIO * lim["loaded_mps"] == pytest.approx(0.15)


def test_the_probe_limits_are_the_vehicles():
    from forklift_core.safety.teleop_drive import TeleopLimits
    t = TeleopLimits()
    lim = lcp.CONFIRM_LIMITS
    assert (lim["direct_forward_mps"], lim["direct_reverse_mps"], lim["curvature_inv_m"], lim["loaded_mps"]) == (
        t.direct_forward_mps, t.direct_reverse_mps, t.curvature_inv_m, t.loaded_mps)


def test_a_hold_in_sim_seconds_runs_sim_over_factor_wall_seconds_and_records_its_stand():
    clock = Clock()
    p = FakeProbe(clock)
    p.state = {**p.state, "mode": "TELEOP", "armed": True}
    rec = run(lcp.hold_teleop(p, "reverse", 3.0, -0.6, 0.0, factor=0.3, clock=clock))
    assert rec["end_mono"] - rec["start_mono"] == pytest.approx(10.0, abs=0.06)
    assert rec["stand_mono"] >= rec["end_mono"] and rec["sim_seconds"] == 3.0 and rec["factor"] == 0.3


@pytest.mark.parametrize("factor,wait", [(1.0, 1.0), (0.4, 2.5), (0.3, 1.0 / 0.3)])
def test_a_stop_is_held_one_sim_second_before_the_rearm(monkeypatch, factor, wait):
    """R1 results (Codex tools review P3): the STOP's four-wheel stillness must be observable before the rearm."""
    clock = Clock()
    p = FakeProbe(clock)
    waits = []

    async def psleep(p_, seconds):
        waits.append(seconds)

    monkeypatch.setattr(lcp, "_psleep", psleep)
    run(lcp.confirm_stop(p, "A1", clock=clock, factor=factor))
    assert waits == [pytest.approx(wait)]


def test_every_confirm_stop_passes_the_factor():
    import inspect
    src = inspect.getsource(lcp.confirm_scenario) + inspect.getsource(lcp.hold_teleop)
    calls = [line for line in src.splitlines() if "confirm_stop(" in line]
    assert calls and all("factor=" in line for line in calls)
