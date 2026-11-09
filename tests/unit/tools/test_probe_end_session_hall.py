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
