"""B-P2d (start relocalization plan 4.4): the probe's hold scenario -- a relocalized session that reached running stands
for the hold's simulated time (wall = sim / factor, plus a second) before the probe ends it; a session that closes
meanwhile is not ended by the probe and says so."""

import asyncio
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402


class StubProbe:
    def __init__(self, close_after_s=None):
        self.report, self.calls, self.closed_code, self.lease_id = {}, [], None, None
        self.state = {"pose": {"x_m": 1.0}}
        self._close_after = close_after_s
        self._opened = None

    async def open(self):
        self.calls.append(("open", time.monotonic()))
        self._opened = time.monotonic()

    async def until(self, pred, timeout, hz=20, label=""):
        self.calls.append(("until", label))
        assert pred(self.state)
        return self.state

    async def request(self, kind, timeout=5.0, **fields):
        self.calls.append((kind, time.monotonic()))
        return {"t": "ack", "lease_id": "L"} if kind == "lease_request" else {"t": "ack"}

    async def request_until_standing(self, kind, tries=20, **fields):
        self.calls.append((kind, time.monotonic()))
        self.closed_code = 1000
        return {"t": "ack"}

    def tick_close(self):
        if self._close_after is not None and time.monotonic() - self._opened > self._close_after:
            self.closed_code = 1006


def _run(p, hold_sim_s=0.3, factor=1.0):
    args = SimpleNamespace(hold_sim_s=hold_sim_s, live_factor=factor)
    orig = asyncio.sleep

    async def sleep(s):
        p.tick_close()
        await orig(min(s, 0.05))

    lcp.asyncio.sleep = sleep
    try:
        asyncio.run(lcp.hold_scenario(p, args))
    finally:
        lcp.asyncio.sleep = orig


def test_the_session_stands_its_hold_then_is_ended():
    p = StubProbe()
    t0 = time.monotonic()
    _run(p, hold_sim_s=0.3, factor=0.5)
    kinds = [c[0] for c in p.calls]
    assert kinds == ["open", "until", "lease_request", "session_end"]
    assert p.calls[1][1] == "localized_state"
    assert dict(p.calls)["lease_request"] - t0 >= 0.3 / 0.5 + 1.0 - 0.05  # sim 0.3 s at factor 0.5, plus a second
    hold = p.report["hold"]
    assert hold["hold_sim_s"] == 0.3 and hold["factor"] == 0.5 and hold["closed_during"] is None
    assert p.report["session_end"] == {"t": "ack"} and p.report["close_code"] == 1000


def test_a_session_that_closes_during_the_hold_is_not_ended_by_the_probe():
    p = StubProbe(close_after_s=0.2)
    _run(p, hold_sim_s=2.0, factor=1.0)
    assert [c[0] for c in p.calls] == ["open", "until"]
    assert p.report["hold"]["closed_during"] == 1006 and "session_end" not in p.report
