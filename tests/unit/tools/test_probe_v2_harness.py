"""Validation v2 harness (docs/plans/2026-11-08-live-riib-defaults-v2.md H1, H2): the main scenario ends its session
when asked, and a request's reply event keeps when it was sent and when the reply came."""

import argparse
import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import live_console_probe as lcp  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def time(self):
        return self.t


class MainProbe:
    """Every wait reached at once, every request acknowledged; session_end closes the connection."""

    def __init__(self):
        self.state = {"mode": "AUTO", "armed": False, "last_result": {"seq": 1, "mission_id": "m1"}}
        self.targets = {"targets_revision": 1}
        self.closed_code = None
        self.report, self.epoch, self.lease_id, self.arm_gen = {}, 7, None, 0
        self.requests, self.labels = [], []
        self.ws = SimpleNamespace(close=self._close)

    async def _close(self):
        pass

    async def open(self):
        pass

    async def open_media(self):
        pass

    def standing(self, s=None):
        return True

    async def until(self, pred, timeout, label=""):
        self.labels.append(label)
        return True

    async def request(self, kind, timeout=5.0, **fields):
        self.requests.append(kind)
        if kind == "session_end":
            self.closed_code = 1000
        return {"t": "ack", "lease_id": "L1", "mission_id": "m1", "arm_gen": 1}

    async def _request(self, kind, timeout=5.0, **fields):
        return await self.request(kind, timeout, **fields)

    async def request_until_standing(self, kind, tries=20, **fields):
        return await self.request(kind, **fields)

    async def keep_alive(self, seconds):
        pass

    async def teleop(self, *a, **k):
        pass


def run_main(monkeypatch, end_session):
    clock = Clock()
    monkeypatch.setattr(lcp, "time", clock)

    async def fake_sleep(s):
        clock.t += s

    monkeypatch.setattr(lcp.asyncio, "sleep", fake_sleep)
    p = MainProbe()
    args = argparse.Namespace(expect_stop_in_capture=True, observe_timeout_s=10.0, auto_s=1.0,
                              mission_timeout_s=100.0, end_session=end_session)
    asyncio.run(lcp.main_scenario(p, args))
    return p


def test_main_ends_its_session_when_asked(monkeypatch):
    p = run_main(monkeypatch, True)
    assert p.requests[-1] == "session_end" and p.report["session_end"]["t"] == "ack"
    assert p.report["close_code"] == 1000


def test_main_leaves_the_session_open_without_the_option(monkeypatch):
    p = run_main(monkeypatch, False)
    assert "session_end" not in p.requests and p.report["close_code"] is None


def test_a_reply_event_keeps_its_send_and_reply_times(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(lcp, "time", clock)
    events = []

    async def send(raw):
        clock.t += 0.02  # the reply comes 20 ms after the send
        dummy.replies["r1"] = (clock.t, {"t": "ack"})

    async def fake_sleep(s):
        clock.t += s

    monkeypatch.setattr(lcp.asyncio, "sleep", fake_sleep)
    dummy = SimpleNamespace(n=0, epoch=7, request_token=None, lease_id="L", control_gen=1, replies={},
                            ws=SimpleNamespace(send=send),
                            event=lambda name, **f: events.append({"event": name, **f}))
    reply = asyncio.run(lcp.Probe._request(dummy, "stop"))
    assert reply == {"t": "ack"}
    (e,) = events
    assert e["sent_mono"] == 1000.0 and abs(e["reply_mono"] - 1000.02) < 1e-9 and abs(e["rtt_s"] - 0.02) < 1e-9


def test_main_with_a_mission_still_active_rearms_cancels_waits_for_its_end_and_then_ends_the_session(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(lcp, "time", clock)

    async def fake_sleep(s):
        clock.t += s

    monkeypatch.setattr(lcp.asyncio, "sleep", fake_sleep)

    class Active(MainProbe):
        """As the arbiter and the core: a cancel is refused while STOPPED (stopped_locked), an acked cancel passes
        through 'cancelling' before 'cancelled', and session_end is refused while the mission is not final."""

        async def request(self, kind, timeout=5.0, **fields):
            m = self.state.get("mission") or {}
            if kind == "mission_cancel" and self.state.get("mode") == "STOPPED":
                self.requests.append(kind)
                return {"t": "reject", "reason": "stopped_locked"}
            if kind == "session_end" and m.get("status") not in lcp.FINAL_MISSION:
                self.requests.append(kind)
                return {"t": "reject", "reason": "mission_active"}
            out = await super().request(kind, timeout, **fields)
            if kind == "rearm":
                self.state = dict(self.state, mode="PAUSED")
            if kind == "mission_cancel":
                self.state = dict(self.state, mission={"mission_id": "m1", "status": "cancelling"})
            return out

        async def until(self, pred, timeout, label=""):
            self.labels.append(label)
            if label == "mission_end_or_stop":  # the final resume refused: the mission left paused, the arbiter PAUSED
                self.state = dict(self.state, mode="PAUSED", mission={"mission_id": "m1", "status": "paused"})
                return True
            if label == "end_cancel_ended":  # the core finishes the cancel
                self.state = dict(self.state, mission={"mission_id": "m1", "status": "cancelled"})
            return pred(self.state) if label.startswith("end_") else True

        async def request_until_standing(self, kind, tries=20, **fields):
            return await self.request(kind, **fields)

    p = Active()
    args = argparse.Namespace(expect_stop_in_capture=True, observe_timeout_s=10.0, auto_s=1.0,
                              mission_timeout_s=100.0, end_session=True)
    p.state = dict(p.state, mode="STOPPED", mission={"mission_id": "m1", "status": "paused"})
    asyncio.run(lcp.end_active_mission(p, "m1"))
    assert p.requests == ["rearm", "mission_cancel"]
    assert p.labels[-1] == "end_cancel_ended" and (p.state["mission"] or {}).get("status") == "cancelled"
    assert p.report == {}  # end_active_mission returns its record; main keeps it
    p2 = Active()
    asyncio.run(lcp.main_scenario(p2, args))
    assert p2.report["end_active"]["cancel"]["t"] == "ack" and p2.report["end_active"]["cancel_ended"]
    assert p2.requests[-1] == "session_end" and p2.report["close_code"] == 1000
    assert p2.requests[-3:] == ["mission_cancel", "lease_request", "session_end"]


def test_a_cancel_that_becomes_a_paused_recovery_is_resumed_until_the_mission_is_final(monkeypatch):
    # Codex: forks in a pocket -- paused -> cancelling -> recovering -> paused (the PAUSED mode pauses the recovery);
    # nothing moves until it is resumed, as the cycle path's U30 handling does
    clock = Clock()
    monkeypatch.setattr(lcp, "time", clock)

    async def fake_sleep(s):
        clock.t += s

    monkeypatch.setattr(lcp.asyncio, "sleep", fake_sleep)

    class Recovering(MainProbe):
        async def request(self, kind, timeout=5.0, **fields):
            out = await super().request(kind, timeout, **fields)
            if kind == "mission_cancel":
                self.state = dict(self.state, mission={"mission_id": "m1", "status": "cancelling", "phase": "insert"})
            if kind == "mission_resume" and (self.state.get("mission") or {}).get("status") == "paused":
                self.state = dict(self.state, mission={"mission_id": "m1", "status": "recovering",
                                                       "phase": "recover_insert"})
            return out

        async def request_until_standing(self, kind, tries=20, **fields):
            return await self.request(kind, **fields)

        async def until(self, pred, timeout, label=""):
            self.labels.append(label)
            m = self.state.get("mission") or {}
            if label == "end_cancel_ended":
                if m.get("status") == "cancelling":  # the recovery stands paused in PAUSED
                    self.state = dict(self.state, mission={"mission_id": "m1", "status": "paused",
                                                           "phase": "recover_insert"})
                elif m.get("status") == "recovering":  # resumed: the recovery ends the cancel
                    self.state = dict(self.state, mission={"mission_id": "m1", "status": "cancelled"})
            return pred(self.state)

    p = Recovering()
    p.state = dict(p.state, mode="PAUSED", mission={"mission_id": "m1", "status": "paused", "phase": "insert"})
    out = asyncio.run(lcp.end_active_mission(p, "m1"))
    assert p.requests == ["mission_cancel", "mission_resume"]
    assert out["recovery_resume"]["t"] == "ack" and out["cancel_ended"] is True
