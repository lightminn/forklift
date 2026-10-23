"""The console probe's cycle under the console queue's policy (CT-1 PR 4, docs/plans/2026-10-22-console-mission-queue.md
D4-D6): no next submit after a result other than done, a refusal, or a stop event while the mission ran; the queue's
watcher counts a feedback loss only once it lasts 0.5 s."""

import asyncio
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402


class ScriptedProbe:
    """Standing, a submit acked (or refused) and its result in the next state; an anomaly put in during a mission."""

    def __init__(self, outcomes, *, refuse=(), anomaly_in=()):
        self.outcomes, self.refuse, self.anomaly_in = list(outcomes), set(refuse), set(anomaly_in)
        self.report, self.missions, self.closed_code, self.epoch, self.lease_id = {}, None, None, 1, None
        self.targets = {"targets_revision": 1}
        self.state = {"mode": "PAUSED", "rear_axle_speed_mps": 0.0, "last_result": {"seq": 0}}
        self.anomalies, self.submitted = [], []

    async def open(self):
        pass

    async def open_media(self):
        pass

    def event(self, name, **fields):
        self.report.setdefault("events", []).append({"event": name, **fields})

    def standing(self, s):
        return True

    async def until(self, pred, timeout, hz=20, label=""):
        return bool(pred(self.state))

    async def request(self, kind, **kw):
        if kind == "lease_request":
            return {"t": "ack", "lease_id": "L"}
        if kind != "mission_submit":
            return {"t": "ack"}
        n = len(self.submitted) + 1
        self.submitted.append((kw["pallet_id"], kw["destination_id"]))
        if n in self.refuse:
            return {"t": "reject", "reason": "busy"}
        if n in self.anomaly_in:
            self.anomalies.append({"kind": "feedback_lost", "since_mono": 0.0, "mono": 0.5})
        self.state = {**self.state, "last_result": {"seq": n, "mission_id": f"m{n}", "outcome": self.outcomes[n - 1]}}
        return {"t": "ack", "mission_id": f"m{n}"}


def _args(**kw):
    base = dict(idle_s=0.0, pre_teleop_s=0.0, cycle="P-1:D-1,P-2:D-2,P-3:D-3", manual_recovery=False, stop_wait_at="",
                cancel_at="", mission_timeout_s=5.0, reconnect_at_end=False, end_session=False, stop_on_anomaly=True)
    base.update(kw)
    return SimpleNamespace(**base)


def _run(probe, **kw):
    real_sleep = asyncio.sleep
    lcp.asyncio.sleep = lambda s: real_sleep(0)  # the probe's 1 s settle waits, made instant
    try:
        asyncio.run(lcp.cycle_scenario(probe, _args(**kw)))
    finally:
        lcp.asyncio.sleep = real_sleep
    return probe


def test_all_done_runs_the_whole_queue():
    p = _run(ScriptedProbe(["done", "done", "done"]))
    assert len(p.submitted) == 3 and p.report["queue"]["stopped"] == {"why": "completed", "leg": 3}


def test_a_failed_result_stops_before_the_next_submit():
    p = _run(ScriptedProbe(["done", "failed", "done"]))
    assert p.submitted == [("P-1", "D-1"), ("P-2", "D-2")]
    assert p.report["queue"]["stopped"] == {"why": "outcome:failed", "leg": 1}


def test_a_refusal_stops_instead_of_skipping_to_the_next_leg():
    p = _run(ScriptedProbe(["done", "done", "done"], refuse={2}))
    assert len(p.submitted) == 2 and p.report["queue"]["stopped"]["why"] == "refused:busy"
    q = _run(ScriptedProbe(["done", "done", "done"], refuse={2}), stop_on_anomaly=False)  # the old cycle goes on
    assert len(q.submitted) == 3 and "queue" not in q.report


def test_a_stop_event_during_a_done_mission_stops_the_queue_after_it():
    p = _run(ScriptedProbe(["done", "done", "done"], anomaly_in={1}))
    assert len(p.submitted) == 1 and p.report["queue"]["stopped"] == {"why": "event:feedback_lost", "leg": 0}
    assert p.report["missions_run"][0]["anomalies"][0]["kind"] == "feedback_lost"


def test_the_watcher_counts_a_feedback_loss_only_after_half_a_second_and_once():
    probe = lcp.Probe(0, {})
    clock = {"t": 100.0}
    real = lcp.time.monotonic
    lcp.time.monotonic = lambda: clock["t"]
    try:
        for dt, ok in ((0.0, False), (0.3, False), (0.1, True), (0.1, False), (0.4, False), (0.2, False),
                       (0.3, False)):
            clock["t"] += dt
            probe._watch_anomalies({"feedback_ok": ok, "mode": "AUTO"})
        assert [a["kind"] for a in probe.anomalies] == ["feedback_lost"]  # 0.3 s then a recovery: none; 0.6 s: one
        clock["t"] += 0.1
        probe._watch_anomalies({"feedback_ok": True, "mode": "STOPPED"})
        clock["t"] += 0.1
        probe._watch_anomalies({"feedback_ok": True, "mode": "STOPPED"})  # the same STOP going on: one event
        assert [a["kind"] for a in probe.anomalies] == ["feedback_lost", "mode:STOPPED"]
    finally:
        lcp.time.monotonic = real
