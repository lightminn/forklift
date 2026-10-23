"""The probe's queue scenario (CT-1 PR 4): the --cycle legs submitted by the console queue's port
(tools/mission_queue.py) only when it asks -- standing confirmed, 1 s after the last result -- and none after it
stops (a failure, a refusal, a STOP between missions, a feedback loss that lasts). A scripted vehicle on a fake
clock; the probe's own receive path is stood in for by feeding the queue each state, as Probe._read does."""

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402

TARGETS = {"targets_revision": 1, "pallets": [{"pallet_id": f"P-{k}", "state": "available"} for k in (1, 2, 3)],
           "destinations": [{"destination_id": f"D-{k}"} for k in (1, 2, 3)]}


class Clock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


class ScriptedVehicle:
    """PAUSED and standing; a submit acked (or refused) and run for ``run_s``, its result in a state, then standing
    again after ``settle_s``. ``events``: {time_s: state overrides} the vehicle shows at that time."""

    def __init__(self, clock, outcomes, *, refuse=(), run_s=3.0, settle_s=0.4, events=None, answer_after_s=0.0,
                 silent=()):
        self.clock, self.outcomes, self.refuse = clock, list(outcomes), set(refuse)
        self.run_s, self.settle_s, self.events = run_s, settle_s, dict(events or {})
        self.answer_after_s, self.silent = answer_after_s, silent  # (from, to) s: no states then
        self.client_id, self.queue_has_lease = "probe", False
        self.report, self.missions, self.closed_code, self.epoch, self.lease_id = {}, None, None, 1, None
        self.targets, self.queue, self.submitted, self.seq = TARGETS, None, [], 0
        self.mission = None  # (mission_id, pallet, destination, start time)
        self.last_result = None
        self.state = self._state()

    def _state(self):
        now = self.clock.t
        s = {"mode": "PAUSED", "stopped_confirmed": True, "feedback_ok": True, "rear_axle_speed_mps": 0.0,
             "mission": None, "last_result": self.last_result}
        if self.mission is not None:
            mid, pal, dst, t0 = self.mission
            if now - t0 < self.run_s:
                s.update(mode="AUTO", stopped_confirmed=False, rear_axle_speed_mps=0.3,
                         mission={"mission_id": mid, "status": "running", "pallet_id": pal, "destination_id": dst})
            else:
                self.seq += 1
                self.last_result = {"seq": self.seq, "mission_id": mid,
                                    "outcome": self.outcomes[int(mid[1:]) - 1], "reason": None}
                self.mission, self.ended = None, now
                s["last_result"] = self.last_result
                s["stopped_confirmed"] = False  # not yet confirmed standing at the result's tick
        elif getattr(self, "ended", None) is not None and now - self.ended < self.settle_s:
            s["stopped_confirmed"] = False
        for at, over in self.events.items():
            if at <= now - 1000.0 < at + 0.6:
                s.update(over)
        return s

    async def open(self):
        pass

    async def open_media(self):
        pass

    def event(self, name, **fields):
        self.report.setdefault("events", []).append({"event": name, **fields})

    def standing(self, s):
        return abs(s.get("rear_axle_speed_mps") or 0.0) < 0.01

    async def until(self, pred, timeout, hz=20, label=""):
        return bool(pred(self.state))

    async def request(self, kind, **kw):
        return {"t": "ack", "lease_id": "L"} if kind == "lease_request" else {"t": "ack"}

    async def request_until_standing(self, kind, **kw):
        return {"t": "ack"}

    async def request_resent(self, kind, request_id, **kw):
        t0 = self.clock.t
        while self.clock.t - t0 < self.answer_after_s:  # the answer comes late (the queue's timer runs meanwhile)
            await asyncio.sleep(0.1)
        return await self._request(kind, request_id=request_id, **kw)

    async def _request(self, kind, request_id=None, **kw):
        n = len(self.submitted) + 1
        self.submitted.append((kw["pallet_id"], kw["destination_id"], round(self.clock.t - 1000.0, 2)))
        if n in self.refuse:
            return {"t": "reject", "reason": "not_stopped"}
        self.mission = (f"m{n}", kw["pallet_id"], kw["destination_id"], self.clock.t)
        return {"t": "ack", "mission_id": f"m{n}"}

    async def echo(self):
        pass

    def tick_vehicle(self):
        self.state = self._state()
        if self.silent and self.silent[0] <= self.clock.t - 1000.0 < self.silent[1]:
            return  # no state arrives
        if self.queue is not None:  # as Probe._read feeds it: state, the lease it shows, its result
            had = self.queue_has_lease
            if self.state.get("lease") and self.state["lease"].get("holder") != self.client_id:
                self.queue_has_lease = False
            self.queue.on_state(self.state, self.clock.t * 1000.0)
            if had and not self.queue_has_lease:
                self.queue.on_lease_lost()
            if self.state.get("last_result"):
                self.queue.on_result(self.state["last_result"], self.clock.t * 1000.0)


def _run(vehicle, clock, cycle="P-1:D-1,P-2:D-2,P-3:D-3"):
    real_sleep, real_mono = asyncio.sleep, lcp.time.monotonic

    async def fake_sleep(s):
        steps = max(1, int(round(s / 0.1)))
        for _ in range(steps):  # states every 0.1 s, as the runner sends them
            clock.t += min(s, 0.1)
            vehicle.tick_vehicle()
        await real_sleep(0)

    lcp.asyncio.sleep, lcp.time.monotonic = fake_sleep, clock.monotonic
    try:
        args = SimpleNamespace(cycle=cycle, mission_timeout_s=60.0, end_session=False)
        asyncio.run(lcp.queue_scenario(vehicle, args))
    finally:
        lcp.asyncio.sleep, lcp.time.monotonic = real_sleep, real_mono
    return vehicle


def test_all_done_submits_each_only_after_standing_and_the_settle():
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "done", "done"]), clock)
    assert [s[:2] for s in v.submitted] == [("P-1", "D-1"), ("P-2", "D-2"), ("P-3", "D-3")]
    gaps = [b[2] - a[2] for a, b in zip(v.submitted, v.submitted[1:])]
    assert all(g >= 3.0 + 1.0 - 1e-6 for g in gaps)  # the run, then 1 s after the result (and standing confirmed)
    assert v.report["queue"]["stop_reason"] == "완료" and v.report["queue"]["counts"]["done"] == 3


def test_a_failure_stops_the_queue_with_nothing_more_submitted():
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "failed", "done"]), clock)
    assert len(v.submitted) == 2 and v.report["queue"]["stop_reason"] == "실패"
    assert v.report["queue"]["items_left"] == ["P-3:D-3"]


def test_a_refusal_stops_at_once():
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "done", "done"], refuse={2}), clock)
    assert len(v.submitted) == 2 and v.report["queue"]["stop_reason"] == "거부: not_stopped"


def test_a_teleop_between_missions_stops_before_the_next_submit():
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "done", "done"], events={3.3: {"mode": "TELEOP"}}), clock)
    assert len(v.submitted) == 1 and v.report["queue"]["stop_reason"] == "수동 조종"


def test_a_feedback_loss_that_lasts_stops_even_when_it_recovers():
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "done", "done"], events={1.0: {"feedback_ok": False}}), clock)
    # 0.6 s of false states from 1.0 s, then true: the loss lasted 0.5 s -- the mission out runs to its end
    assert len(v.submitted) == 1 and v.report["queue"]["stop_reason"] == "차량 피드백 없음"
    assert v.report["missions_run"][0]["result"]["outcome"] == "done"


def test_a_state_that_shows_another_holder_takes_the_lease_and_stops_the_queue():
    """Codex PR 4 re-review P2-1: protocol.js drops the lease on a state whose holder is not this client."""
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "done", "done"], events={3.3: {"lease": {"holder": "other"}}}), clock)
    assert len(v.submitted) == 1 and v.report["queue"]["stop_reason"] == "조종 권한 상실"


def test_the_queue_timer_runs_while_a_submit_waits_for_its_answer():
    """Codex PR 4 re-review P2-2: states stop for 3 s while the answer is out -- the queue stops for the silence
    and the item is left uncertain; the late ack is not taken for a mission of the queue."""
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "done", "done"], answer_after_s=3.0, silent=(0.5, 3.6)), clock)
    q = v.report["queue"]
    assert q["stop_reason"] == "상태 수신 끊김" and len(q["uncertain"]) == 1 and q["counts"]["done"] == 0


def test_an_answer_that_is_slow_keeps_the_queue_waiting_not_refused():
    """Codex PR 4 re-review P2-3: no made-up reject -- the states keep coming, the answer comes after 6 s."""
    clock = Clock()
    v = _run(ScriptedVehicle(clock, ["done", "done", "done"], answer_after_s=6.0), clock)
    assert v.report["queue"]["stop_reason"] == "완료" and v.report["queue"]["counts"]["done"] == 3
