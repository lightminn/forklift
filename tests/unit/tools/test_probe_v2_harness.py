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


def test_main_with_a_mission_still_active_stops_cancels_and_then_ends_the_session(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(lcp, "time", clock)

    async def fake_sleep(s):
        clock.t += s

    monkeypatch.setattr(lcp.asyncio, "sleep", fake_sleep)

    class Active(MainProbe):
        async def request(self, kind, timeout=5.0, **fields):
            out = await super().request(kind, timeout, **fields)
            if kind == "stop":
                self.state = dict(self.state, mode="STOPPED")
            if kind == "mission_cancel":
                self.state = dict(self.state, mission={"mission_id": "m1", "status": "cancelled"})
            if kind == "session_end" and (self.state.get("mission") or {}).get("status") == "paused":
                self.closed_code = None
                return {"t": "reject", "reason": "mission_active"}
            return out

        async def until(self, pred, timeout, label=""):
            self.labels.append(label)
            if label == "mission_end_or_stop":  # the final resume left the mission paused (a repeated STOP)
                self.state = dict(self.state, mode="AUTO", mission={"mission_id": "m1", "status": "paused"})
            return pred(self.state) if label.startswith("end_") else True

    p = Active()
    args = argparse.Namespace(expect_stop_in_capture=True, observe_timeout_s=10.0, auto_s=1.0,
                              mission_timeout_s=100.0, end_session=True)
    asyncio.run(lcp.main_scenario(p, args))
    tail = p.requests[p.requests.index("stop", len(p.requests) - 5):]
    assert tail[:2] == ["stop", "mission_cancel"] and tail[-1] == "session_end"
    assert p.report["end_active"]["cancel_ended"] and p.report["close_code"] == 1000
