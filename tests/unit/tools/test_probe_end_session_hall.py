"""The marker and approach scenarios end the session through its own path under --end-session (realtime plan 5.17:
ws1 job 16866 TERMed the launcher after them and lost the end summary, the tick trace and the spool of all eight hall
sessions). The end comes after every step the analysis reads -- the frames, the freeze samples, the legs -- so the
measured windows are the same with or without it."""

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402

WRAPPER = ROOT / "docs" / "validation" / "2026-11-08-r1-confirmation" / "r1c_session.sh"


class FakeProbe:
    def __init__(self):
        self.closed_code = None
        self.log = []
        self.report = {}
        self.lease_id = None

    async def request(self, kind, **fields):
        self.log.append(("request", kind))
        return {"t": "ack", "lease_id": "L1"} if kind == "lease_request" else {"t": "ack"}

    async def request_until_standing(self, kind, tries=20, **fields):
        self.log.append(("request", kind))
        if kind == "session_end":
            self.closed_code = 1000  # the runner closes the console as the epoch ends
        return {"t": "ack"}


def steps_stub(name):
    async def steps(p, args):
        # what the analysis reads, recorded inside the scenario's own steps
        p.log.append(("step", name))
        p.report.setdefault("events", []).append({"event": "leg_done" if name == "approach" else "stale_video_disarmed"})
    return steps


@pytest.mark.parametrize("scenario,steps", [("marker", "_marker_steps"), ("approach", "_approach_steps")])
def test_end_session_closes_the_session_after_every_measured_step(monkeypatch, scenario, steps):
    monkeypatch.setattr(lcp, steps, steps_stub(scenario))
    p = FakeProbe()
    asyncio.run(getattr(lcp, f"{scenario}_scenario")(p, SimpleNamespace(end_session=True)))
    kinds = [k for _, k in p.log]
    assert kinds == [scenario, "lease_request", "session_end"]  # the steps first, the end last
    assert p.report["close_code"] == 1000 and p.report["session_end"] == {"t": "ack"}


@pytest.mark.parametrize("scenario,steps", [("marker", "_marker_steps"), ("approach", "_approach_steps")])
def test_without_end_session_the_scenario_leaves_the_session_open(monkeypatch, scenario, steps):
    monkeypatch.setattr(lcp, steps, steps_stub(scenario))
    p = FakeProbe()
    asyncio.run(getattr(lcp, f"{scenario}_scenario")(p, SimpleNamespace(end_session=False)))
    assert [k for _, k in p.log] == [scenario]
    assert "session_end" not in p.report


def test_an_already_closed_session_is_not_asked_again(monkeypatch):
    monkeypatch.setattr(lcp, "_marker_steps", steps_stub("marker"))
    p = FakeProbe()
    p.closed_code = 1011  # the runner ended on its own during the steps
    asyncio.run(lcp.marker_scenario(p, SimpleNamespace(end_session=True)))
    assert [k for _, k in p.log] == ["marker"]
    assert p.report["close_code"] == 1011


def test_the_r1c_wrapper_ends_every_set_through_the_probe():
    src = WRAPPER.read_text()
    assert "END=(--end-session)" in src
    assert "END=()" not in src  # no set resets it (16866's mark and wall did)
    assert '"${PROBE[@]}" "${END[@]}"' in src


class MarkerProbe:
    """The marker steps' freeze loop: stale_video refusals first while frames still flow (a stall at sim 25 s), then
    with the frames stopped at the freeze (last frame 29.95 s)."""

    def __init__(self):
        self.closed_code = None
        self.report, self.events = {}, []
        self.frames = [{"stamp_ns": int(20.0e9)}]
        self.state = {"armed": False, "last_teleop_reject": "stale_video", "rear_axle_speed_mps": 0.0}
        self.calls = 0
        self.lease_id = self.arm_gen = None

    async def open(self):
        pass

    async def open_media(self):
        pass

    def standing(self, s):
        return True

    async def until(self, pred, timeout, hz=20, label=""):
        return True

    async def request(self, kind, **fields):
        return {"t": "ack", "lease_id": "L1", "arm_gen": 1}

    async def teleop(self, speed, curvature, deadman=True, mode="direct", lift_dir=0):
        self.calls += 1
        self.frames.append({"stamp_ns": int((25.0 if self.calls < 4 else 29.95) * 1e9)})

    def event(self, name, **fields):
        self.events.append((name, fields))


def test_a_disarm_before_the_freeze_is_armed_again_and_one_in_it_ends_the_steps(tmp_path, monkeypatch):
    """R1 results: a session disarmed before the freeze proved nothing (now armed again), and the freeze's own disarm
    ends the steps whatever reject the state shows by then (r1c_18 waited 600 s for a stale_video sample)."""
    async def fast_sleep(_):
        return None
    monkeypatch.setattr(lcp.asyncio, "sleep", fast_sleep)
    p = MarkerProbe()
    arms = []
    real_request = p.request

    async def request(kind, **fields):
        arms.append(kind)
        return await real_request(kind, **fields)

    p.request = request
    p.state = {**p.state, "last_teleop_reject": "not_armed"}  # overwritten already
    args = SimpleNamespace(frames_dir=tmp_path / "f", frames=0, marker_s=60.0, freeze_from=30.0)
    asyncio.run(lcp._marker_steps(p, args))
    assert p.calls == 4  # three samples at 25 s (re-armed each time), the fourth (frames stopped at 29.95 s) ends it
    names = [e[0] for e in p.events]
    assert names == ["rearmed_before_freeze"] * 3 + ["disarmed_in_freeze"]
    assert arms.count("arm") == 1 + 3  # the first arm, then one per disarm before the freeze


def test_the_real_approach_steps_refused_teleop_still_end_the_session(monkeypatch):
    """Codex review P3: the approach body returns early when TELEOP is refused; the end still follows it."""
    class ApproachProbe(FakeProbe):
        async def open(self):
            pass

        async def open_media(self):
            pass

        def standing(self, s):
            return True

        async def until(self, pred, timeout, hz=20, label=""):
            return True

        async def request(self, kind, **fields):
            self.log.append(("request", kind))
            if kind == "mode_request":
                return {"t": "reject", "reason": "teleop_forbidden", "detail": "start_block"}
            return {"t": "ack", "lease_id": "L1"}

    p = ApproachProbe()
    asyncio.run(lcp.approach_scenario(p, SimpleNamespace(end_session=True)))
    kinds = [k for _, k in p.log]
    assert kinds[:2] == ["lease_request", "mode_request"] and "arm" not in kinds
    assert kinds[-2:] == ["lease_request", "session_end"]
    assert p.report["teleop_entry"]["t"] == "reject" and "approach" not in p.report
    assert p.report["close_code"] == 1000 and p.report["end_requested_mono"] > 0


def test_an_expiry_mid_leg_is_armed_again_and_is_not_a_hold(monkeypatch):
    """R1 results: every arm's wall leg ended on a command expiry, counted as a hold; the later legs never moved."""
    async def fast_sleep(_):
        return None
    monkeypatch.setattr(lcp.asyncio, "sleep", fast_sleep)

    class ApproachProbe(FakeProbe):
        def __init__(self):
            super().__init__()
            self.ticks = 0
            self.state = {"armed": True, "rear_axle_speed_mps": 0.2, "applied": {"speed_mps": 0.2},
                          "permit": {"limit_mps": 0.3, "reason": "ok"}}

        async def open(self):
            pass

        async def open_media(self):
            pass

        def standing(self, s):
            return True

        async def until(self, pred, timeout, hz=20, label=""):
            return True

        def event(self, name, **fields):
            self.log.append(("event", name))

        async def request(self, kind, **fields):
            self.log.append(("request", kind))
            if kind == "arm":
                self.state = {**self.state, "armed": True}
            return {"t": "ack", "lease_id": "L1", "arm_gen": 2}

        async def teleop(self, speed, curvature, deadman=True, mode="direct", lift_dir=0):
            self.ticks += 1
            if self.ticks == 3:  # an expiry: disarmed, the output zero, the truck stopping
                self.state = {**self.state, "armed": False, "applied": {"speed_mps": 0.0}, "rear_axle_speed_mps": 0.0,
                              "last_teleop_reject": "not_armed"}
            elif self.ticks > 3 and self.state["armed"]:
                self.state = {**self.state, "applied": {"speed_mps": 0.0}, "rear_axle_speed_mps": 0.0,
                              "permit": {"limit_mps": 0.3, "reason": "ok"}}

    p = ApproachProbe()
    args = SimpleNamespace(end_session=False, approach_curvature=-0.1, approach_s=0.5, held_s=0.0,
                           approach_weave_s=0.0, approach_weave_n=0)
    asyncio.run(lcp._approach_steps(p, args))
    assert ("event", "rearmed_in_leg") in p.log and p.log.count(("request", "arm")) >= 2
    first = p.report["approach"][0]
    assert first["held"] is False  # stopped with the permission's limit above zero: not the permission's hold
