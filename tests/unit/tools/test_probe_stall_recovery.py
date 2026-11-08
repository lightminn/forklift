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


def state(*, x=1.0, v=0.0, status="running", mode="AUTO", mid="m1"):
    return {"mission": {"mission_id": mid, "status": status}, "mode": mode, "rear_axle_speed_mps": v,
            "pose": {"frame": "map", "x_m": x, "y_m": 2.0, "yaw_rad": 0.0}}


def test_a_standing_running_mission_is_a_stall_after_the_wall_seconds_only():
    clock = Clock()
    watch = lcp.StallWatch(60.0, "m1", clock=clock)
    assert not watch(state())  # the count starts
    clock.t = 59.0
    assert not watch(state(x=1.005))  # within the spot
    clock.t = 60.0
    assert watch(state(x=1.005))


@pytest.mark.parametrize("breaker", [dict(v=0.02), dict(x=1.02), dict(status="paused"), dict(mode="STOPPED"),
                                     dict(mid="m2")])
def test_a_move_a_pause_a_stop_or_another_mission_starts_the_count_again(breaker):
    clock = Clock()
    watch = lcp.StallWatch(60.0, "m1", clock=clock)
    watch(state())
    clock.t = 50.0
    assert not watch(state(**breaker))
    clock.t = 100.0
    assert not watch(state())  # a new count from here
    clock.t = 159.0
    assert not watch(state())
    clock.t = 160.0
    assert watch(state())


def test_no_pose_or_speed_is_never_a_stall():
    clock = Clock()
    watch = lcp.StallWatch(1.0, "m1", clock=clock)
    s = state()
    s["pose"] = None
    for clock.t in (0.0, 5.0):
        assert not watch(s)
    s = state()
    s["rear_axle_speed_mps"] = None
    for clock.t in (6.0, 12.0):
        assert not watch(s)


class FakeProbe:
    def __init__(self, result, *, ends=True):
        self.state = state()
        self.requests = []
        self._result, self._ends = result, ends

    async def request(self, kind, **fields):
        self.requests.append((kind, fields))
        if kind == "mission_cancel" and self._ends:
            self.state = {**state(status="cancelled"), "last_result": {"seq": 5, "mission_id": "m1", **self._result}}
        return {"t": "ack"}

    async def until(self, pred, timeout, hz=20, label=""):
        return bool(pred(self.state))


def run(probe, monkeypatch, *, teleop=("ack",), moved=0.17):
    async def enter(p):
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
                                         (dict(moved=0.01), "reverse_refused")])
def test_a_refused_teleop_or_reverse_is_its_own_outcome(monkeypatch, kw, outcome):
    assert run(FakeProbe({"outcome": "cancelled"}), monkeypatch, **kw)["outcome"] == outcome


def test_a_cancel_that_leaves_a_recovery_or_never_ends_stops_there(monkeypatch):
    probe = FakeProbe({"outcome": "recovery_required"})
    assert run(probe, monkeypatch)["outcome"] == "recovery_required"
    assert [k for k, _ in probe.requests] == ["mission_cancel"]
    probe = FakeProbe({}, ends=False)
    assert run(probe, monkeypatch)["outcome"] == "cancel_not_ended"


def test_the_cycle_watches_only_when_asked_and_ends_the_session_on_a_recovery_not_made():
    src = inspect.getsource(lcp.cycle_scenario)
    assert "StallWatch(args.stall_recover_s, submit.get(\"mission_id\")) if args.stall_recover_s > 0 else None" in src
    assert 'entry["stall_recovery"] = await stall_recovery(p, args, submit.get("mission_id"), seen)' in src
    assert 'not in ("recovered", "recovery_required")' in src and 'p.event("stall_recovery_not_done"' in src
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
