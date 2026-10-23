"""tools/mission_queue.py against console/web/js/queue.js itself: the same events into both, the same state after
every one (run, stop reason, phase, current mission, the submit asked). Needs Node (skipped without it, as on ws1);
run it locally after touching either side."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import mission_queue as mq  # noqa: E402

NODE = shutil.which("node")
DRIVER = r"""
import { MissionQueue, newItem } from "%s";
const events = JSON.parse(process.argv[2]);
const q = new MissionQueue();
let n = 0; const newId = () => `id${++n}`;
let last = null; const out = [];
for (const e of events) {
  let submit = null;
  if (e.op === "add") q.add(newItem({ pallet_id: e.pallet_id, destination_id: e.destination_id }, 0, newId));
  else if (e.op === "resume") q.resume(e.now);
  else if (e.op === "welcome") q.onWelcome(e.epoch);
  else if (e.op === "state") { q.onState(e.state, e.now); if (e.state?.last_result) q.onResult(e.state.last_result, e.now); }
  else if (e.op === "result") q.onResult(e.result, e.now);
  else if (e.op === "answer") q.onAnswer(last, e.answer);
  else if (e.op === "tick") q.tick(e.now);
  else if (e.op === "next") { const r = q.next(e.ctx, e.now, newId); if (r) { submit = r.submit; last = { epoch: q.epoch, item_id: r.submit.item_id, request_id: r.submit.request_id }; } }
  else if (e.op === "disconnect") q.onDisconnect();
  else if (e.op === "lease_lost") q.onLeaseLost();
  out.push({ run: q.run, reason: q.stopReason, phase: q.phase, mission: q.current?.mission_id ?? null,
             items: q.items.length, uncertain: q.items.filter((x) => x.uncertain).length, submit });
}
console.log(JSON.stringify(out));
"""


def run_py(events):
    q = mq.MissionQueue()
    n = {"k": 0}

    def new_id():
        n["k"] += 1
        return f"id{n['k']}"

    last, out = None, []
    for e in events:
        submit = None
        op = e["op"]
        if op == "add":
            q.add(mq.new_item(e["pallet_id"], e["destination_id"], new_id()))
        elif op == "resume":
            q.resume(e["now"])
        elif op == "welcome":
            q.on_welcome(e["epoch"])
        elif op == "state":
            q.on_state(e["state"], e["now"])
            if (e["state"] or {}).get("last_result"):
                q.on_result(e["state"]["last_result"], e["now"])
        elif op == "result":
            q.on_result(e["result"], e["now"])
        elif op == "answer":
            q.on_answer(last, e["answer"])
        elif op == "tick":
            q.tick(e["now"])
        elif op == "next":
            ctx = dict(e["ctx"])
            ctx["has_lease"] = ctx.pop("hasLease")
            r = q.next(ctx, e["now"], new_id)
            if r:
                submit = r["submit"]
                last = {"epoch": q.epoch, "item_id": submit["item_id"], "request_id": submit["request_id"]}
        elif op == "disconnect":
            q.on_disconnect()
        elif op == "lease_lost":
            q.on_lease_lost()
        out.append({"run": q.run, "reason": q.stop_reason, "phase": q.phase,
                    "mission": (q.current or {}).get("mission_id"), "items": len(q.items),
                    "uncertain": sum(1 for x in q.items if x.get("uncertain")), "submit": submit})
    return out


def run_js(events, tmp_path):
    script = tmp_path / "drive.mjs"
    script.write_text(DRIVER % (ROOT / "console/web/js/queue.js").as_posix())
    done = subprocess.run([NODE, str(script), json.dumps(events)], capture_output=True, text=True, check=True)
    return json.loads(done.stdout)


TARGETS = {"targets_revision": 3, "pallets": [{"pallet_id": f"P-{k}", "state": "available"} for k in (1, 2, 3)],
           "destinations": [{"destination_id": f"D-{k}"} for k in (1, 2, 3)]}
CTX = {"connected": True, "welcomed": True, "hasLease": True, "targets": TARGETS}


def standing(now, **kw):
    s = {"mode": "PAUSED", "stopped_confirmed": True, "feedback_ok": True, "mission": None}
    s.update(kw)
    return {"op": "state", "now": now, "state": s}


def nxt(now, state):
    return {"op": "next", "now": now, "ctx": {**CTX, "state": state["state"]}}


def scenario_base(*tail):
    ev = [{"op": "welcome", "epoch": 7}]
    for k in (1, 2, 3):
        ev.append({"op": "add", "pallet_id": f"P-{k}", "destination_id": f"D-{k}"})
    s0 = standing(0)
    return ev + [s0, {"op": "resume", "now": 10}, nxt(20, s0)] + list(tail)


def running(now, mid, status="running"):
    return standing(now, mode="AUTO", stopped_confirmed=False,
                    mission={"mission_id": mid, "status": status, "pallet_id": "P-1", "destination_id": "D-1"})


CASES = {
    "done_then_next_after_settle": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m1"),
        standing(400, last_result={"mission_id": "m1", "outcome": "done"}),
        nxt(900, standing(900)), nxt(1500, standing(1500))),
    "failure_stops": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m1"),
        standing(400, last_result={"mission_id": "m1", "outcome": "failed", "reason": "x"}), nxt(2000, standing(2000))),
    "refusal_busy_uncertain": scenario_base({"op": "answer", "answer": {"t": "reject", "reason": "busy"}}),
    "refusal_not_stopped": scenario_base({"op": "answer", "answer": {"t": "reject", "reason": "not_stopped"}}),
    "feedback_loss_recovered_after_600ms": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m1"),
        {"op": "state", "now": 300, "state": {**running(300, "m1")["state"], "feedback_ok": False}},
        running(900, "m1")),
    "feedback_loss_short": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m1"),
        {"op": "state", "now": 300, "state": {**running(300, "m1")["state"], "feedback_ok": False}},
        running(600, "m1")),
    "silence": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m1"), {"op": "tick", "now": 2300}),
    "teleop_between_missions": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m1"),
        standing(400, last_result={"mission_id": "m1", "outcome": "done"}), standing(600, mode="TELEOP"),
        nxt(2000, standing(2000))),
    "paused_mission": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m1"), running(300, "m1", "paused")),
    "foreign_mission": scenario_base(
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(200, "m9")),
    "early_result_before_ack": scenario_base(
        {"op": "result", "now": 100, "result": {"mission_id": "m1", "outcome": "done"}},
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}),
    "new_session": scenario_base({"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}},
                                 {"op": "welcome", "epoch": 8}),
    "lease_lost": scenario_base({"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, {"op": "lease_lost"}),
}


@pytest.mark.skipif(NODE is None, reason="needs Node to run console/web/js/queue.js")
@pytest.mark.parametrize("name", sorted(CASES))
def test_the_port_decides_as_the_console_queue(name, tmp_path):
    events = CASES[name]
    assert run_py(events) == run_js(events, tmp_path)


def test_the_port_runs_on_its_own():
    out = run_py(CASES["done_then_next_after_settle"])
    assert out[6]["submit"]["pallet_id"] == "P-1"  # the first submit
    assert out[-2]["submit"] is None and out[-1]["submit"]["pallet_id"] == "P-2"  # only 1 s after the result
    assert run_py(CASES["refusal_not_stopped"])[-1]["reason"] == "거부: not_stopped"
    assert run_py(CASES["feedback_loss_recovered_after_600ms"])[-1]["reason"] == "차량 피드백 없음"
