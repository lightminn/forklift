"""tools/mission_queue.py's auto item against console/web/js/queue.js itself (CT-2 console plan C8): the same events
into both, the same state after every one (run, stop reason, phase, the pick asked or held, items with their uncertain
marks, the finished list, the submit or pick_request asked). Needs Node, and a mission_queue.py with the auto item
(it comes with the CT-2 vehicle side); skipped otherwise."""

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
pytestmark = [pytest.mark.skipif(NODE is None, reason="needs node"),
              pytest.mark.skipif(not hasattr(mq.MissionQueue, "on_pick"), reason="mission_queue.py has no auto item")]

DRIVER = r"""
import { MissionQueue, newItem } from "%s";
const events = JSON.parse(process.argv[2]);
const q = new MissionQueue();
let n = 0; const newId = () => `id${++n}`;
let submitKey = null, pickKey = null; const out = [];
for (const e of events) {
  let asked = null;
  if (e.op === "add_auto") q.add(newItem({ kind: "auto", label: "남은 팔레트 전부" }, 0, newId));
  else if (e.op === "add") q.add(newItem({ pallet_id: e.pallet_id, destination_id: e.destination_id }, 0, newId));
  else if (e.op === "resume") q.resume(e.now);
  else if (e.op === "welcome") q.onWelcome(e.epoch);
  else if (e.op === "state") { q.onState(e.state, e.now); if (e.state?.last_result) q.onResult(e.state.last_result, e.now); }
  else if (e.op === "result") q.onResult(e.result, e.now);
  else if (e.op === "answer") q.onAnswer(submitKey, e.answer);
  else if (e.op === "answer_pick") q.onAnswer(pickKey, e.answer, e.now);
  else if (e.op === "pick") q.onPick(pickKey, { ...e.msg, request_id: pickKey.request_id }, e.now, e.revision);
  else if (e.op === "tick") q.tick(e.now);
  else if (e.op === "stop") q.operatorAction("stop");
  else if (e.op === "disconnect") q.onDisconnect();
  else if (e.op === "next") {
    const r = q.next(e.ctx, e.now, newId);
    if (r?.submit) { asked = { submit: r.submit }; submitKey = { epoch: q.epoch, item_id: r.submit.item_id, request_id: r.submit.request_id }; }
    if (r?.pick_request) { asked = { pick_request: r.pick_request }; pickKey = { epoch: q.epoch, item_id: r.pick_request.item_id, request_id: r.pick_request.request_id }; }
  }
  out.push({ run: q.run, reason: q.stopReason, phase: q.phase,
             held: q.held ? [q.held.pallet_id, q.held.destination_id] : null, forget: q.drainForget(),
             items: q.items.map((x) => [x.item_id, x.kind, Boolean(x.uncertain), x.uncertain_pallet_id ?? null, x.moved ?? 0]),
             finished: q.finished.map((f) => [f.mission_id, f.outcome, f.pallet_id ?? null, f.destination_id ?? null]), asked });
}
console.log(JSON.stringify(out));
"""


def run_py(events):
    q = mq.MissionQueue()
    n = {"k": 0}

    def new_id():
        n["k"] += 1
        return f"id{n['k']}"

    submit_key = pick_key = None
    out = []
    for e in events:
        asked = None
        op = e["op"]
        if op == "add_auto":
            q.add(mq.new_auto_item(new_id()) if hasattr(mq, "new_auto_item")
                  else {"item_id": new_id(), "kind": "auto", "pallet_id": None, "destination_id": None,
                        "label": "남은 팔레트 전부", "added_unix": 0})
        elif op == "add":
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
            q.on_answer(submit_key, e["answer"])
        elif op == "answer_pick":
            q.on_answer(pick_key, e["answer"], *([e["now"]] if "now" in e else []))
        elif op == "pick":
            q.on_pick(pick_key, {**e["msg"], "request_id": pick_key["request_id"]}, e["now"], e["revision"])
        elif op == "tick":
            q.tick(e["now"])
        elif op == "stop":
            q.operator_action("stop") if hasattr(q, "operator_action") else q.stop("STOP")
        elif op == "disconnect":
            q.on_disconnect()
        elif op == "next":
            ctx = dict(e["ctx"])
            ctx["has_lease"] = ctx.pop("hasLease")
            ctx["next_pick"] = ctx.pop("nextPick")
            r = q.next(ctx, e["now"], new_id)
            if r and r.get("submit"):
                asked = {"submit": r["submit"]}
                submit_key = {"epoch": q.epoch, "item_id": r["submit"]["item_id"], "request_id": r["submit"]["request_id"]}
            if r and r.get("pick_request"):
                asked = {"pick_request": r["pick_request"]}
                pick_key = {"epoch": q.epoch, "item_id": r["pick_request"]["item_id"],
                            "request_id": r["pick_request"]["request_id"]}
        held = q.held
        out.append({"run": q.run, "reason": q.stop_reason, "phase": q.phase,
                    "held": [held["pallet_id"], held["destination_id"]] if held else None, "forget": q.drain_forget(),
                    "items": [[x["item_id"], x["kind"], bool(x.get("uncertain")), x.get("uncertain_pallet_id"),
                               x.get("moved", 0)] for x in q.items],
                    "finished": [[f["mission_id"], f["outcome"], f.get("pallet_id"), f.get("destination_id")]
                                 for f in q.finished], "asked": asked})
    return out


def run_js(events, tmp_path):
    script = tmp_path / "drive_auto.mjs"
    script.write_text(DRIVER % (ROOT / "console/web/js/queue.js").as_posix())
    done = subprocess.run([NODE, str(script), json.dumps(events)], capture_output=True, text=True, check=True)
    return json.loads(done.stdout)


TARGETS = {"targets_revision": 3, "pallets": [{"pallet_id": f"P-{k}", "state": "available"} for k in (1, 2)],
           "destinations": [{"destination_id": f"D-{k}"} for k in (1, 2)]}


def standing(now, **kw):
    s = {"mode": "PAUSED", "stopped_confirmed": True, "feedback_ok": True, "mission": None}
    s.update(kw)
    return {"op": "state", "now": now, "state": s}


def nxt(now, state, **ctx):
    return {"op": "next", "now": now, "ctx": {"connected": True, "welcomed": True, "hasLease": True,
                                              "targets": TARGETS, "nextPick": True, **ctx, "state": state["state"]}}


def ok_pick(now, p, d, rev=3):
    return {"op": "pick", "now": now, "revision": 3,
            "msg": {"t": "pick", "targets_revision": rev, "pick": {"pallet_id": p, "destination_id": d},
                    "reason": "ok", "plan": [[p, d]], "missions": 1}}


def reason_pick(now, reason):
    return {"op": "pick", "now": now, "revision": 3,
            "msg": {"t": "pick", "targets_revision": 3, "pick": None, "reason": reason, "plan": [], "missions": 0,
                    "blocked": {"P-1": ["pickup:P-2"]}}}


def running(now, mid, p="P-1", d="D-1"):
    return standing(now, mode="AUTO", stopped_confirmed=False,
                    mission={"mission_id": mid, "status": "running", "pallet_id": p, "destination_id": d})


def base(*tail):
    s0 = standing(0)
    return [{"op": "welcome", "epoch": 7}, {"op": "add_auto"}, s0, {"op": "resume", "now": 10}, nxt(20, s0), *tail]


ACK = {"op": "answer_pick", "answer": {"t": "ack"}}
CASES = {
    "two_picks_then_none_remaining": base(
        ACK, ok_pick(100, "P-1", "D-1"), nxt(150, standing(150)),
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(300, "m1"),
        standing(400, last_result={"mission_id": "m1", "outcome": "done"}), nxt(900, standing(900)),
        nxt(1500, standing(1500)), ACK, ok_pick(1600, "P-2", "D-2"), nxt(1650, standing(1650)),
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m2"}}, running(1800, "m2", "P-2", "D-2"),
        standing(1900, last_result={"mission_id": "m2", "outcome": "done"}), nxt(3000, standing(3000)), ACK,
        reason_pick(3100, "none_remaining")),
    # Codex CT-2 r2: none remaining before its ack waits for it; a refusal then stops before the fixed item behind
    "none_before_ack_then_refusal": [
        {"op": "welcome", "epoch": 7}, {"op": "add_auto"}, {"op": "add", "pallet_id": "P-2", "destination_id": "D-2"},
        standing(0), {"op": "resume", "now": 10}, nxt(20, standing(20)), reason_pick(100, "none_remaining"),
        nxt(200, standing(200)), {"op": "answer_pick", "now": 300, "answer": {"t": "reject", "reason": "busy"}},
        nxt(400, standing(400))],
    "none_before_ack_then_ack": [
        {"op": "welcome", "epoch": 7}, {"op": "add_auto"}, {"op": "add", "pallet_id": "P-2", "destination_id": "D-2"},
        standing(0), {"op": "resume", "now": 10}, nxt(20, standing(20)), reason_pick(100, "none_remaining"),
        {"op": "answer_pick", "now": 200, "answer": {"t": "ack"}}, nxt(300, standing(300))],
    "none_before_ack_deadline": base(reason_pick(100, "none_remaining"), standing(2000), standing(4000),
                                     {"op": "tick", "now": 5100}),
    "late_ack": base(ok_pick(100, "P-1", "D-1"), standing(2000), standing(4000), {"op": "tick", "now": 5020},
                     {"op": "answer_pick", "now": 5100, "answer": {"t": "ack"}}, nxt(5150, standing(5150))),
    "pick_before_ack_waits": base(ok_pick(100, "P-1", "D-1"), nxt(150, standing(150)), ACK, nxt(200, standing(200))),
    "refusal_after_pick": base(ok_pick(100, "P-1", "D-1"), {"op": "answer_pick", "answer": {"t": "reject", "reason": "busy"}},
                               nxt(200, standing(200))),
    "all_blocked": base(ACK, reason_pick(100, "all_blocked")),
    "error": base(ACK, reason_pick(100, "error:ValueError")),
    "deadline_by_tick": base(ACK, standing(2000), standing(4000), {"op": "tick", "now": 5100}),
    "late_pick": base(ACK, standing(2000), standing(4000), ok_pick(5100, "P-1", "D-1")),
    "unacked_held_deadline": base(ok_pick(100, "P-1", "D-1"), standing(2000), standing(4000), {"op": "tick", "now": 5100}),
    "revision_twice": base(ACK, ok_pick(100, "P-1", "D-1", rev=2), nxt(200, standing(200)), ACK,
                           ok_pick(300, "P-1", "D-1", rev=2)),
    "stop_while_held_unacked": base(ok_pick(100, "P-1", "D-1"), {"op": "stop"}),
    "disconnect_while_picking": base({"op": "disconnect"}),
    "busy_auto_submit_uncertain": base(ACK, ok_pick(100, "P-1", "D-1"), nxt(150, standing(150)),
                                       {"op": "answer", "answer": {"t": "reject", "reason": "busy"}}),
    "failed_auto_mission_stops": base(
        ACK, ok_pick(100, "P-1", "D-1"), nxt(150, standing(150)),
        {"op": "answer", "answer": {"t": "ack", "mission_id": "m1"}}, running(300, "m1"),
        standing(400, last_result={"mission_id": "m1", "outcome": "failed", "reason": "x"})),
    "no_feature": base(),
    "fixed_after_none_remaining": [
        {"op": "welcome", "epoch": 7}, {"op": "add_auto"}, {"op": "add", "pallet_id": "P-2", "destination_id": "D-2"},
        standing(0), {"op": "resume", "now": 10}, nxt(20, standing(20)), ACK, reason_pick(100, "none_remaining"),
        nxt(200, standing(200))],
}
CASES["no_feature"] = [{"op": "welcome", "epoch": 7}, {"op": "add_auto"}, standing(0), {"op": "resume", "now": 10},
                       nxt(20, standing(20), nextPick=False)]


@pytest.mark.parametrize("name", sorted(CASES))
def test_python_port_follows_the_console_queue_for_the_auto_item(name, tmp_path):
    js, py = run_js(CASES[name], tmp_path), run_py(CASES[name])
    for k, (a, b) in enumerate(zip(js, py)):
        assert a == b, f"{name} event {k} ({CASES[name][k]['op']}): js {a} != py {b}"
    assert len(js) == len(py)
