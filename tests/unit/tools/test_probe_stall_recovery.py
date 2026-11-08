"""The probe's stall recovery (docs/plans/2026-11-08-live-riib-defaults-validation.md 3.2): a running mission whose truck
stands within 0.01 m for --stall-recover-s wall s is cancelled, backed off straight by hand once, and the cycle goes on;
a recovery that cannot be made ends the session."""

import asyncio
import inspect
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


SEQ = [0]


def state(*, x=1.0, v=0.0, status="running", mode="AUTO", mid="m1"):
    SEQ[0] += 1  # a new state from the vehicle carries a new sequence
    return {"state_seq": SEQ[0], "mission": {"mission_id": mid, "status": status}, "mode": mode,
            "rear_axle_speed_mps": v, "pose": {"frame": "map", "x_m": x, "y_m": 2.0, "yaw_rad": 0.0}}


def feed(watch, clock, until, step=0.5, **kw):
    fired = False
    while clock.t < until - 1e-9:
        clock.t = round(clock.t + step, 6)
        fired = watch(state(**kw))
    return fired


def test_a_standing_running_mission_is_a_stall_after_the_wall_seconds_only():
    clock = Clock()
    watch = lcp.StallWatch(60.0, "m1", clock=clock)
    assert not watch(state())  # the count starts
    assert not feed(watch, clock, 59.5, x=1.005)  # within the spot
    assert feed(watch, clock, 60.0, x=1.005)


@pytest.mark.parametrize("breaker", [dict(v=0.02), dict(x=1.02), dict(status="paused"), dict(mode="STOPPED"),
                                     dict(mid="m2")])
def test_a_move_a_pause_a_stop_or_another_mission_starts_the_count_again(breaker):
    clock = Clock()
    watch = lcp.StallWatch(60.0, "m1", clock=clock)
    watch(state())
    assert not feed(watch, clock, 50.0)
    clock.t = 50.5
    assert not watch(state(**breaker))
    assert not feed(watch, clock, 110.0)  # a new count from 51.0
    assert feed(watch, clock, 111.0)


def test_only_fresh_states_count_and_a_silence_starts_the_count_again():
    clock = Clock()
    watch = lcp.StallWatch(60.0, "m1", clock=clock)
    s = state()
    assert not watch(s)
    clock.t = 70.0
    assert not watch(s)  # the same state object again: no new evidence, and a silence over 1 s resets
    assert watch.since is None
    for k in range(1, 40):  # fresh states every 0.5 s, 19.5 s: not yet
        clock.t = 70.0 + 0.5 * k
        assert not watch(state())
    clock.t = 70.5 + 0.5 * 120
    assert not watch(state())  # 60 s since the last fresh one: stale, a new count
    for k in range(1, 121):
        clock.t = 130.5 + 0.5 * k
        fired = watch(state())
    assert fired  # 60 s of fresh standing states


def test_no_pose_or_speed_is_never_a_stall():
    clock = Clock()
    watch = lcp.StallWatch(1.0, "m1", clock=clock)
    for clock.t in (0.0, 0.5, 1.0, 1.5):
        s = state()
        s["pose"] = None
        assert not watch(s)
    for clock.t in (2.0, 2.5, 3.0, 3.5):
        s = state()
        s["rear_axle_speed_mps"] = None
        assert not watch(s)


class FakeProbe:
    def __init__(self, result, *, ends=True, load="empty", cancel="ack", paused="ack"):
        self.state = {**state(), "load": {"state": load}}
        self.requests = []
        self._result, self._ends, self._load, self._cancel, self._paused = result, ends, load, cancel, paused

    def standing(self, s):
        return abs(s.get("rear_axle_speed_mps") or 0.0) < 0.01

    async def request(self, kind, **fields):
        self.requests.append((kind, fields))
        if kind == "mission_cancel":
            if self._ends:
                self.state = {**state(status="cancelled", mode="PAUSED"), "load": {"state": self._load},
                              "last_result": {"seq": 5, "mission_id": "m1", **self._result}}
            return {"t": self._cancel}
        if kind == "mode_request" and fields.get("to") == "PAUSED":
            return {"t": self._paused}
        return {"t": "ack"}

    async def until(self, pred, timeout, hz=20, label=""):
        if getattr(self, "pending", None) is not None:  # the state that shows a reply arrives after it
            self.state, self.pending = self.pending, None
        return bool(pred(self.state))


def run(probe, monkeypatch, *, teleop=("ack",), moved=0.17, armed=True):
    async def enter(p):
        if teleop[0] == "ack":
            p.arm_gen = 7 if armed else 0
            p.pending = {**p.state, "mode": "TELEOP", "armed": armed, "arm_gen": 7 if armed else 0}  # not yet shown
        return {"t": teleop[0]} if teleop[0] else None

    async def back(p, low, high, max_s):
        p.requests.append(("back_off", {"low": low, "high": high}))
        return {"moved_m": moved, "target_m": 0.5 * (low + high), "within": low <= moved <= high}

    monkeypatch.setattr(lcp, "enter_teleop", enter)
    monkeypatch.setattr(lcp, "back_off_by", back)
    return asyncio.run(lcp.stall_recovery(probe, SimpleNamespace(stall_backoff_m=0.30), "m1", seen=4))


def test_the_recovery_cancels_waits_backs_off_once_by_hand_and_pauses(monkeypatch):
    probe = FakeProbe({"outcome": "cancelled"})
    out = run(probe, monkeypatch)
    assert out["outcome"] == "recovered" and out["ended"] and out["mission"]["mission_id"] == "m1"
    assert [k for k, _ in probe.requests] == ["mission_cancel", "back_off", "mode_request"]
    assert probe.requests[1][1] == {"low": 0.05, "high": 0.30} and probe.requests[2][1] == {"to": "PAUSED"}


@pytest.mark.parametrize("kw, outcome", [(dict(teleop=(None,)), "teleop_refused"),
                                         (dict(teleop=("reject",)), "teleop_refused"),
                                         (dict(armed=False), "teleop_refused"),
                                         (dict(moved=0.01), "reverse_out_of_interval"),
                                         (dict(moved=0.40), "reverse_out_of_interval")])
def test_a_refused_teleop_or_a_reverse_outside_its_interval_is_its_own_outcome(monkeypatch, kw, outcome):
    assert run(FakeProbe({"outcome": "cancelled"}), monkeypatch, **kw)["outcome"] == outcome


def test_the_armed_teleop_is_waited_for_in_the_state_after_the_replies(monkeypatch):
    # Codex probe review round 2 P2: the acks come first; the decision waits for the state that shows them
    probe = FakeProbe({"outcome": "cancelled"})
    out = run(probe, monkeypatch)
    assert out["armed"] is True and out["outcome"] == "recovered"


def test_a_refused_pause_is_not_a_recovery(monkeypatch):
    assert run(FakeProbe({"outcome": "cancelled"}, paused="reject"), monkeypatch)["outcome"] == "paused_refused"


def test_a_loaded_cancel_is_a_hold_and_nothing_moves(monkeypatch):
    probe = FakeProbe({"outcome": "cancelled"}, load="loaded")
    assert run(probe, monkeypatch)["outcome"] == "loaded_hold"
    assert [k for k, _ in probe.requests] == ["mission_cancel"]


@pytest.mark.parametrize("result, cancel, outcome", [({"outcome": "done"}, "ack", "completed_before_cancel"),
                                                     ({"outcome": "failed"}, "ack", "ended_otherwise"),
                                                     ({"outcome": "done"}, "reject", "completed_before_cancel"),
                                                     ({}, "reject", "cancel_refused")])
def test_a_mission_that_ended_by_itself_or_a_refused_cancel_moves_nothing(monkeypatch, result, cancel, outcome):
    probe = FakeProbe(result, cancel=cancel, ends=bool(result))
    assert run(probe, monkeypatch)["outcome"] == outcome
    assert [k for k, _ in probe.requests] == ["mission_cancel"]


def test_a_cancel_that_leaves_a_recovery_or_never_ends_stops_there(monkeypatch):
    probe = FakeProbe({"outcome": "recovery_required"})
    assert run(probe, monkeypatch)["outcome"] == "recovery_required"
    assert [k for k, _ in probe.requests] == ["mission_cancel"]
    probe = FakeProbe({}, ends=False)
    assert run(probe, monkeypatch)["outcome"] == "cancel_not_ended"


def test_the_cycle_watches_only_when_asked_and_ends_the_session_on_a_recovery_not_made():
    src = inspect.getsource(lcp.cycle_scenario)
    assert 'StallWatch(args.stall_recover_s, submit.get("mission_id"), clock=getattr(p, "clock", None))' in src
    assert 'entry["stall_recovery"] = await stall_recovery(p, args, submit.get("mission_id"), seen)' in src
    assert "not in STALL_GOES_ON" in src and 'p.event("stall_recovery_not_done"' in src
    args = lcp_args()
    assert args.stall_recover_s == 0.0 and args.stall_backoff_m == pytest.approx(0.30)


def lcp_args():
    import argparse

    captured = {}
    real = argparse.ArgumentParser.parse_args

    def grab(self, *a, **k):
        captured["args"] = real(self, ["--port-file", "x", "--report", "r.json", "--scenario", "cycle"])
        raise SystemExit(0)

    argparse.ArgumentParser.parse_args = grab
    try:
        with pytest.raises(SystemExit):
            lcp.main()
    finally:
        argparse.ArgumentParser.parse_args = real
    return captured["args"]


# -- the cycle itself, through state transitions (Codex probe review P2: not a source check alone)

class CycleVehicle:
    """A console-facing vehicle on a fake clock, stepped by the probe's waits (a new state every 0.1 s)."""

    def __init__(self, script):
        self.t, self.script = 0.0, script  # script(vehicle) -> None, sets the fields below as time goes on
        self.mode, self.status, self.mission_id, self.speed, self.x = "PAUSED", None, None, 0.0, 1.0
        self.load, self.result, self.seq, self.submits, self.requests = "empty", None, 0, 0, []

    def state(self):
        self.seq += 1
        mission = None if self.mission_id is None else {"mission_id": self.mission_id, "status": self.status}
        return {"state_seq": self.seq, "mode": self.mode, "rear_axle_speed_mps": self.speed, "mission": mission,
                "pose": {"frame": "map", "x_m": self.x, "y_m": 2.0, "yaw_rad": 0.0}, "load": {"state": self.load},
                "last_result": self.result, "armed": getattr(self, "armed", False),
                "arm_gen": 3 if getattr(self, "armed", False) else 0}

    def end(self, outcome):
        self.result = {"seq": (self.result or {}).get("seq", 0) + 1, "mission_id": self.mission_id, "outcome": outcome}
        self.status, self.mode = outcome, "PAUSED"


class CycleProbe:
    def __init__(self, vehicle):
        self.v = vehicle
        self.state = vehicle.state()
        self.report, self.events, self.targets = {}, [], {"targets_revision": 1}
        self.closed_code, self.lease_id, self.epoch, self.missions = None, None, 1, None

    def clock(self):
        return self.v.t

    def event(self, name, **kw):
        self.events.append((name, kw))

    def standing(self, s):
        return abs(s.get("rear_axle_speed_mps") or 0.0) < 0.01

    async def open(self):
        return None

    async def open_media(self):
        return None

    async def until(self, pred, timeout, hz=20, label=""):
        end = self.v.t + timeout
        while self.v.t <= end:
            self.v.script(self.v)
            self.state = self.v.state()
            if pred(self.state):
                return True
            self.v.t = round(self.v.t + 0.1, 6)
        return False

    async def request(self, kind, **fields):
        v = self.v
        v.requests.append((round(v.t, 1), kind, fields))
        if kind == "mission_submit":
            v.submits += 1
            v.mission_id, v.status, v.mode, v.started = f"m{v.submits}", "running", "AUTO", v.t
            return {"t": "ack", "mission_id": v.mission_id}
        if kind == "mission_cancel" and v.status in ("running", "paused"):
            v.end("cancelled")
        if kind == "stop":
            v.mode = "STOPPED"
        if kind == "rearm":
            v.mode = "PAUSED"
        if kind == "mission_resume":
            v.mode = "AUTO"
        self.state = v.state()
        return {"t": "ack", "lease_id": "L"} if kind == "lease_request" else {"t": "ack"}

    async def request_until_standing(self, kind, tries=20, **fields):
        return await self.request(kind, **fields)


def cycle_args(**over):
    args = lcp_args()
    for k, v in {"cycle": "P-1:D-1,P-2:D-2", "stall_recover_s": 60.0, "end_session": False,
                 "mission_timeout_s": 400.0, **over}.items():
        setattr(args, k, v)
    return args


def run_cycle(script, monkeypatch, **over):
    async def enter(p):
        p.v.mode, p.v.armed, p.arm_gen = "TELEOP", True, 3  # shown by the next state the probe waits for
        return {"t": "ack"}

    async def back(p, low, high, max_s):
        p.v.requests.append((round(p.v.t, 1), "back_off", {}))
        return {"moved_m": 0.17, "target_m": 0.175, "within": True}

    monkeypatch.setattr(lcp, "enter_teleop", enter)
    monkeypatch.setattr(lcp, "back_off_by", back)
    probe = CycleProbe(CycleVehicle(script))
    asyncio.run(lcp.cycle_scenario(probe, cycle_args(**over)))
    return probe


def test_a_stand_is_cancelled_backed_off_and_the_cycle_goes_on_to_the_next_mission(monkeypatch):
    def script(v):
        if v.mission_id == "m2" and v.status == "running" and v.t - v.started > 5.0:
            v.end("done")  # the second mission runs normally

    probe = run_cycle(script, monkeypatch)
    first, second = probe.report["missions_run"]
    assert first["stall_recovery"]["outcome"] == "recovered" and second["result"]["outcome"] == "done"
    kinds = [k for _, k, _ in probe.v.requests]
    assert kinds.index("mission_cancel") < kinds.index("back_off") < kinds.index("mission_submit", 2)


def test_a_stop_in_the_middle_of_a_stand_starts_the_count_again(monkeypatch):
    def script(v):
        if v.mission_id == "m1" and v.status == "running" and v.mode == "AUTO":
            if 40.0 <= v.t - v.started < 40.2 and not getattr(v, "stopped_once", False):
                v.stopped_once, v.mode = True, "STOPPED"  # a STOP 40 s into the stand; the probe rearms and resumes
            elif v.t - v.started > 95.0:
                v.end("done")  # 95 s in all, under 60 s on each side of the STOP

    probe = run_cycle(script, monkeypatch, cycle="P-1:D-1")
    (only,) = probe.report["missions_run"]
    assert "stall_recovery" not in only and only["result"]["outcome"] == "done"
    assert "rearm" in [k for _, k, _ in probe.v.requests]


def test_a_loaded_stand_is_a_hold_and_the_session_is_ended_without_a_next_mission(monkeypatch):
    def script(v):
        v.load = "loaded"  # the guard stood the truck in transport

    probe = run_cycle(script, monkeypatch)
    (only,) = probe.report["missions_run"]
    assert only["stall_recovery"]["outcome"] == "loaded_hold"
    kinds = [k for _, k, _ in probe.v.requests]
    assert "back_off" not in kinds and kinds.count("mission_submit") == 1
    assert kinds[-2:] == ["stop", "session_end"] and ("stall_recovery_not_done", {"outcome": "loaded_hold"}) in probe.events
