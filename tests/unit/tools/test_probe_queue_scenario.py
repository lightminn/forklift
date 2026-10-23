"""The probe's queue scenario (CT-1 PR 4) through its real wiring: Probe._read, request_resent and queue_scenario
against a fake websocket vehicle on a fake clock. The --cycle legs are submitted by the console queue's port
(tools/mission_queue.py) only when it asks -- standing confirmed, 1 s after the last result -- and none after it
stops; the answers and the states reach the queue in the order they arrive (Codex PR 4 review rounds 1-3)."""

import asyncio
import json
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402

REAL_SLEEP = asyncio.sleep
TARGETS = {"t": "targets", "targets_revision": 1,
           "pallets": [{"pallet_id": f"P-{k}", "state": "available"} for k in (1, 2, 3)],
           "destinations": [{"destination_id": f"D-{k}"} for k in (1, 2, 3)]}


class Clock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def rel(self):
        return round(self.t - 1000.0, 3)


class FakeVehicle:
    """A console-facing vehicle: states every 0.1 s; a submit acked after ``ack_s`` (resends of the same request
    id are counted, not run twice); the mission runs ``run_s`` from the submit, then its result, then standing
    confirmed after ``settle_s``. ``events``: {time: overrides of the states from then on for 0.6 s} and
    ``silent``: (from, to) with no states."""

    def __init__(self, clock, outcomes, *, refuse=(), ack_s=0.02, run_s=3.0, settle_s=0.4, events=None, silent=None):
        self.clock, self.outcomes, self.refuse = clock, list(outcomes), set(refuse)
        self.ack_s, self.run_s, self.settle_s = ack_s, run_s, settle_s
        self.events, self.silent = dict(events or {}), silent
        self.outbox = [(0.0, TARGETS)]  # (time, message)
        self.next_state, self.seq = 0.0, 0
        self.mission = None  # (mission_id, pallet, destination, started)
        self.ended = None
        self.last_result = None
        self.submits = []  # (time, request_id, first?)
        self.seen = {}  # request id -> mission number

    def on_message(self, msg):
        now = self.clock.rel()
        t = msg.get("t")
        if t == "lease_request":
            self.outbox.append((now, {"t": "ack", "request_id": msg["request_id"], "lease_id": "L"}))
        elif t == "mission_submit":
            rid = msg["request_id"]
            first = rid not in self.seen
            self.submits.append((now, rid, first))
            if not first:
                return
            n = len(self.seen) + 1
            self.seen[rid] = n
            if n in self.refuse:
                self.outbox.append((now + self.ack_s, {"t": "reject", "request_id": rid, "reason": "not_stopped"}))
                return
            self.mission = (f"m{n}", msg["pallet_id"], msg["destination_id"], now)
            self.outbox.append((now + self.ack_s, {"t": "ack", "request_id": rid, "mission_id": f"m{n}"}))

    def _state(self, now):
        s = {"t": "state", "mode": "PAUSED", "stopped_confirmed": True, "feedback_ok": True,
             "rear_axle_speed_mps": 0.0, "mission": None, "last_result": self.last_result,
             "lease": {"holder": "probe"}}
        if self.mission is not None:
            mid, pal, dst, t0 = self.mission
            if now - t0 < self.run_s:
                s.update(mode="AUTO", stopped_confirmed=False, rear_axle_speed_mps=0.3,
                         mission={"mission_id": mid, "status": "running", "pallet_id": pal, "destination_id": dst})
            else:
                self.seq += 1
                self.last_result = {"seq": self.seq, "mission_id": mid, "outcome": self.outcomes[int(mid[1:]) - 1]}
                s["last_result"], s["stopped_confirmed"] = self.last_result, False
                self.mission, self.ended = None, now
        elif self.ended is not None and now - self.ended < self.settle_s:
            s["stopped_confirmed"] = False
        for at, over in self.events.items():
            if at <= now < at + 0.6:
                s.update(over)
        return s

    def due(self):
        now = self.clock.rel()
        if now >= self.next_state:
            self.next_state += 0.1
            if not (self.silent and self.silent[0] <= now < self.silent[1]):
                self.outbox.append((now, self._state(now)))
        self.outbox.sort(key=lambda m: m[0])
        if self.outbox and self.outbox[0][0] <= now:
            return self.outbox.pop(0)[1]
        return None


class FakeWS:
    close_code = None

    def __init__(self, vehicle):
        self.vehicle = vehicle

    async def send(self, raw):
        self.vehicle.on_message(json.loads(raw))

    def __aiter__(self):
        return self

    async def __anext__(self):
        while True:
            msg = self.vehicle.due()
            if msg is not None:
                return json.dumps(msg)
            await REAL_SLEEP(0)


class WiredProbe(lcp.Probe):
    def __init__(self, vehicle):
        super().__init__(0, {})
        self.vehicle = vehicle

    async def open(self):
        self.epoch, self.request_token = 1, None
        self.ws = FakeWS(self.vehicle)
        self._reader = asyncio.create_task(self._read(self.ws))

    async def open_media(self):
        pass

    async def echo(self):
        pass


def _run(vehicle, clock, cycle="P-1:D-1,P-2:D-2,P-3:D-3"):
    real_mono = lcp.time.monotonic

    async def fake_sleep(s):
        steps = max(1, int(round(s / 0.01)))
        for _ in range(steps):
            clock.t += min(s, 0.01)
            await REAL_SLEEP(0)

    lcp.asyncio.sleep, lcp.time.monotonic = fake_sleep, clock.monotonic
    probe = WiredProbe(vehicle)
    try:
        args = SimpleNamespace(cycle=cycle, mission_timeout_s=60.0, end_session=False)

        async def go():
            await lcp.queue_scenario(probe, args)
            probe._reader.cancel()

        asyncio.run(go())
    finally:
        lcp.asyncio.sleep, lcp.time.monotonic = REAL_SLEEP, real_mono
    return probe


def test_all_done_submits_each_only_after_standing_and_the_settle():
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"])
    p = _run(v, clock)
    firsts = [t for t, _, first in v.submits if first]
    assert len(firsts) == 3 and all(b - a >= 3.0 + 1.0 - 1e-6 for a, b in zip(firsts, firsts[1:]))
    assert p.report["queue"]["stop_reason"] == "완료" and p.report["queue"]["counts"]["done"] == 3


def test_a_failure_stops_the_queue_with_nothing_more_submitted():
    clock = Clock()
    v = FakeVehicle(clock, ["done", "failed", "done"])
    p = _run(v, clock)
    assert len(v.seen) == 2 and p.report["queue"]["stop_reason"] == "실패"
    assert p.report["queue"]["items_left"] == ["P-3:D-3"]


def test_a_refusal_stops_at_once():
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], refuse={2})
    p = _run(v, clock)
    assert len(v.seen) == 2 and p.report["queue"]["stop_reason"] == "거부: not_stopped"


def test_a_teleop_between_missions_stops_before_the_next_submit():
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], events={3.3: {"mode": "TELEOP"}})
    p = _run(v, clock)
    assert len(v.seen) == 1 and p.report["queue"]["stop_reason"] == "수동 조종"


def test_a_feedback_loss_that_lasts_stops_even_when_it_recovers_and_the_mission_out_ends():
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], events={1.0: {"feedback_ok": False}})
    p = _run(v, clock)
    assert len(v.seen) == 1 and p.report["queue"]["stop_reason"] == "차량 피드백 없음"
    assert p.report["missions_run"][0]["result"]["outcome"] == "done"


def test_a_state_that_shows_another_holder_takes_the_lease_and_stops_the_queue():
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], events={3.3: {"lease": {"holder": "other"}}})
    p = _run(v, clock)
    assert len(v.seen) == 1 and p.report["queue"]["stop_reason"] == "조종 권한 상실"


def test_a_slow_answer_is_resent_and_waited_for_not_refused():
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], ack_s=2.0)
    p = _run(v, clock)
    assert p.report["queue"]["stop_reason"] == "완료" and p.report["queue"]["counts"]["done"] == 3
    assert sum(1 for _, _, first in v.submits if not first) >= 3  # resent every 0.5 s while unanswered


def test_a_stop_while_the_answer_is_out_ends_the_resending_at_once():
    """Codex PR 4 review round 3 P2-1: the queue stops (and forgets the submit) -- no resend after it."""
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], ack_s=5.0, events={1.0: {"mode": "STOPPED"}})
    p = _run(v, clock)
    stop_at = next(ms for _, ms in p.queue.stops) / 1000.0 - 1000.0
    assert p.report["queue"]["stop_reason"] == "정지됨(STOP)" and len(p.report["queue"]["uncertain"]) == 1
    assert len(v.submits) >= 2 and v.submits[0][0] < stop_at  # out before the STOP, resent while unanswered
    assert all(t <= stop_at + 1e-6 for t, _, _ in v.submits)


def test_the_answer_reaches_the_queue_before_a_later_stop():
    """Codex PR 4 review round 3 P2-2: submit -> ack (+0.03 s) -> STOP (+0.08 s) -> done (+0.12 s): the ack is the
    queue's first, so the mission is its own and its result is counted, nothing uncertain."""
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], ack_s=0.03, run_s=0.12, events={})
    first_submit = {}

    def on_message(msg, orig=v.on_message):
        if msg.get("t") == "mission_submit" and not first_submit:
            first_submit["t"] = clock.rel()
            v.events[round(clock.rel() + 0.08, 3)] = {"mode": "STOPPED"}
            v.next_state = clock.rel() + 0.08  # the STOP's state comes at +0.08 s
        orig(msg)

    v.on_message = on_message
    p = _run(v, clock)
    q = p.report["queue"]
    assert q["stop_reason"] == "정지됨(STOP)" and q["uncertain"] == [] and q["counts"]["done"] == 1


def _gen_request(kind):
    """A vehicle whose generation moved before the probe saw a state with it; ``kind`` is checked against it."""
    clock = Clock()
    v = FakeVehicle(clock, ["done"])
    v.control_gen, v.asked = 1, []
    state_ = v._state

    def state_with_gen(now):
        s = state_(now)
        s["control_gen"] = v.control_gen
        return s

    v._state = state_with_gen
    real = v.on_message

    def on_message(msg):
        if msg.get("t") == kind:
            ok = msg.get("control_gen") == v.control_gen
            v.asked.append(msg.get("control_gen"))
            v.outbox.append((clock.rel(), {"t": "ack" if ok else "reject", "request_id": msg["request_id"],
                                           **({} if ok else {"reason": "stale_control"})}))
            return
        real(msg)

    v.on_message = on_message
    real_mono = lcp.time.monotonic

    async def fake_sleep(s):
        for _ in range(max(1, int(round(s / 0.01)))):
            clock.t += min(s, 0.01)
            await REAL_SLEEP(0)

    lcp.asyncio.sleep, lcp.time.monotonic = fake_sleep, clock.monotonic
    probe = WiredProbe(v)
    try:
        async def go():
            await probe.open()
            probe.control_gen = 0  # sent before the first state brings generation 1
            reply = await probe.request_until_standing(kind)
            probe._reader.cancel()
            return reply

        return asyncio.run(go()), v
    finally:
        lcp.asyncio.sleep, lcp.time.monotonic = REAL_SLEEP, real_mono


def test_a_session_end_refused_for_a_moved_generation_is_asked_again_with_the_states_generation():
    """CT-1 PR 4 q2: three session_ends refused stale_control, the runner's result.json never written; asked again
    with the generation the next state carries, it ends."""
    reply, v = _gen_request("session_end")
    assert v.asked[0] == 0 and reply["t"] == "ack" and v.asked[-1] == 1


def test_a_rearm_refused_for_a_moved_generation_is_returned_not_retried():
    """Codex PR 4 doc review P2: a rearm (or resume) made before a later STOP must not go through after it -- the
    stale_control goes back to the caller, asked once."""
    reply, v = _gen_request("rearm")
    assert v.asked == [0] and reply["t"] == "reject" and reply["reason"] == "stale_control"


def test_an_answer_at_a_resend_boundary_is_not_followed_by_another_send():
    """Codex PR 4 review round 4 P2: a refusal arriving as the 0.5 s resend comes due -- nothing is sent after it."""
    clock = Clock()
    v = FakeVehicle(clock, ["done", "done", "done"], refuse={1}, ack_s=0.5)
    real = v.on_message
    answered = {}

    def on_message(msg):
        real(msg)
        if msg.get("t") == "mission_submit" and "t" not in answered:
            answered["t"] = clock.rel() + v.ack_s

    v.on_message = on_message
    p = _run(v, clock)
    assert p.report["queue"]["stop_reason"] == "거부: not_stopped"
    assert all(t < answered["t"] + 1e-9 for t, _, _ in v.submits)
