import { test } from "node:test";
import assert from "node:assert/strict";
import { MissionQueue, newItem, START_RETRY_STAND_MS } from "../js/queue.js";

// User decision 2026-10-25 (docs/plans/2026-10-22-console-mission-queue.md D5, the one exception): the session's
// first queue submit refused with not_stopped is resubmitted once, the same item, after the truck has stood for
// START_RETRY_STAND_MS in a row (PAUSED, stop confirmed, feedback valid); everything else stops as before.

const TARGETS = { targets_revision: 7, pallets: [{ pallet_id: "P-1", state: "available" }, { pallet_id: "P-4", state: "available" }],
                  destinations: [{ destination_id: "D-1" }] };
const READY = { mode: "PAUSED", stop_kind: "controlled", stopped_confirmed: true, feedback_ok: true, mission: null };
const ROLLING = { ...READY, stopped_confirmed: false };

let ids = 0;
const newId = () => `id-${++ids}`;
const item = (p = "P-1") => newItem({ kind: "fixed", pallet_id: p, destination_id: "D-1" }, 1000, newId);
const ctx = (state = READY) => ({ connected: true, welcomed: true, hasLease: true, state, targets: TARGETS });

function running(items = [item()], epoch = 3, t = 10_000) {
  const q = new MissionQueue();
  q.onWelcome(epoch);
  q.onState(READY, t);
  for (const it of items) q.add(it);
  q.resume(t);
  return q;
}

function submit(q, t, state = READY) {
  q.onState(state, t);
  const n = q.next(ctx(state), t, newId);
  assert.ok(n && n.submit, `expected a submit at ${t}`);
  return n.submit;
}

const key = (q, s) => ({ epoch: q.epoch, item_id: s.item_id, request_id: s.request_id });
const refuse = (q, s, reason = "not_stopped") => q.onAnswer(key(q, s), { t: "reject", reason });

// states every 100 ms from..to (inclusive), asking for a submit each time; the submits seen
function standFor(q, from, to, state = READY) {
  const out = [];
  for (let t = from; t <= to; t += 100) {
    q.onState(state, t);
    const n = q.next(ctx(state), t, newId);
    if (n?.submit) out.push({ t, submit: n.submit });
  }
  return out;
}

test("the first submit refused not_stopped waits for a 1 s stand and resubmits the same item once", () => {
  assert.equal(START_RETRY_STAND_MS, 1000);
  const q = running();
  const s1 = submit(q, 10_000);
  refuse(q, s1);
  assert.equal(q.run, "running");
  assert.equal(q.phase, "start_retry");
  assert.equal(q.current, null);
  // rolling: no stand yet; then standing from 10_500 -- the resubmit comes once 1 s of standing is seen
  assert.deepEqual(standFor(q, 10_100, 10_400, ROLLING), []);
  const got = standFor(q, 10_500, 12_000);
  assert.equal(got.length, 1);
  assert.equal(got[0].t, 11_500);
  assert.equal(got[0].submit.item_id, s1.item_id);
  assert.notEqual(got[0].submit.request_id, s1.request_id);
  assert.equal(q.phase, "submitting");
})

test("a break in the stand starts the second over", () => {
  const q = running();
  refuse(q, submit(q, 10_000));
  assert.deepEqual(standFor(q, 10_100, 10_900), []); // 0.8 s
  assert.deepEqual(standFor(q, 11_000, 11_000, ROLLING), []);
  const got = standFor(q, 11_100, 12_500);
  assert.equal(got[0].t, 12_100);
});

test("a second not_stopped stops the queue as before", () => {
  const q = running();
  refuse(q, submit(q, 10_000));
  const [{ submit: s2 }] = standFor(q, 10_100, 11_200);
  refuse(q, s2);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "거부: not_stopped");
});

test("only the session's first submit: a not_stopped later in the session stops", () => {
  const q = running([item(), item("P-4")]);
  const s1 = submit(q, 10_000);
  q.onAnswer(key(q, s1), { t: "ack", mission_id: "M-1" });
  q.onState({ ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: "M-1", status: "running" } }, 11_000);
  q.onResult({ mission_id: "M-1", outcome: "done" }, 12_000);
  q.onState(READY, 12_000);
  const s2 = submit(q, 13_100);
  refuse(q, s2);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "거부: not_stopped");
});

test("one resubmit per session: resumed after a stop, a not_stopped stops; a new session gets its own", () => {
  const q = running();
  refuse(q, submit(q, 10_000));
  q.operatorAction("stop");
  assert.equal(q.phase, "waiting");
  q.resume(11_000);
  refuse(q, submit(q, 11_000));
  assert.equal(q.stopReason, "거부: not_stopped");
  q.onWelcome(4);
  q.onState(READY, 20_000);
  q.resume(20_000);
  refuse(q, submit(q, 20_000));
  assert.equal(q.phase, "start_retry");
});

test("other refusals of the first submit stop as before", () => {
  for (const reason of ["already_there", "busy", "invalid"]) {
    const q = running();
    refuse(q, submit(q, 10_000), reason);
    assert.equal(q.run, "stopped", reason);
  }
});

test("while it waits, the usual anomalies stop it and nothing is resubmitted", () => {
  for (const [state, why] of [[{ ...READY, mode: "STOPPED" }, "정지됨(STOP)"], [{ ...READY, mode: "TELEOP" }, "수동 조종"],
                               [{ ...READY, stop_kind: "safety_preempt" }, "안전 선점 정지"]]) {
    const q = running();
    refuse(q, submit(q, 10_000));
    q.onState(state, 10_200);
    assert.equal(q.stopReason, why);
    assert.equal(q.phase, "waiting");
    assert.deepEqual(standFor(q, 10_300, 12_000), []);
  }
  const q = running();
  refuse(q, submit(q, 10_000));
  assert.deepEqual(standFor(q, 10_100, 10_700, { ...READY, feedback_ok: false }), []);
  assert.equal(q.stopReason, "차량 피드백 없음");
});

test("the item in front changed while it waited: no resubmit, the queue stops", () => {
  const q = running([item(), item("P-4")]);
  const s1 = submit(q, 10_000);
  refuse(q, s1);
  q.remove(s1.item_id);
  assert.deepEqual(standFor(q, 10_100, 12_000), []);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "시작 직후 재제출 취소: 맨 앞 항목이 바뀜");
});

test("a page reopened in the wait does not keep it", () => {
  const q = running();
  refuse(q, submit(q, 10_000));
  const r = MissionQueue.fromStorage(q.toStorage(), { epoch: 3, results: [] });
  assert.equal(r.phase, "waiting");
  assert.equal(r.run, "stopped");
});

test("once per session survives a reopen: the session's count and the spent resubmit are kept for that session only", () => {
  const q = running();
  refuse(q, submit(q, 10_000));
  q.operatorAction("stop"); // the resubmit spent, the queue stopped
  const same = MissionQueue.fromStorage(q.toStorage(), { epoch: 3, results: [] });
  same.onWelcome(3);
  same.onState(READY, 20_000);
  same.resume(20_000);
  refuse(same, submit(same, 20_000));
  assert.equal(same.stopReason, "거부: not_stopped", "the same session: no second resubmit after a reopen");
  const other = MissionQueue.fromStorage(q.toStorage(), { epoch: 4, results: [] });
  other.onWelcome(4);
  other.onState(READY, 20_000);
  other.resume(20_000);
  refuse(other, submit(other, 20_000));
  assert.equal(other.phase, "start_retry", "a new session has its own");
});

// ---- Codex round 1 ------------------------------------------------------------------------------------------------
test("an auto item's refused submit goes out again with the same pallet and destination, no new pick", () => {
  const q = running([newItem({ kind: "auto", label: "남은 팔레트 전부" }, 1000, newId)]);
  const c = { ...ctx(READY), nextPick: true };
  q.onState(READY, 10_000);
  const ask = q.next(c, 10_000, newId).pick_request;
  const k = { epoch: q.epoch, item_id: ask.item_id, request_id: ask.request_id };
  q.onAnswer(k, { t: "ack" }, 10_050);
  q.onPick(k, { t: "pick", request_id: ask.request_id, targets_revision: 7, reason: "ok",
                pick: { pallet_id: "P-1", destination_id: "D-1" }, plan: [["P-1", "D-1"]], missions: 1 }, 10_100, 7);
  q.onState(READY, 10_150);
  const s1 = q.next(c, 10_150, newId).submit;
  refuse(q, s1);
  assert.equal(q.phase, "start_retry");
  let got = null;
  for (let t = 10_200; t <= 11_500 && !got; t += 100) {
    q.onState(READY, t);
    got = q.next(c, t, newId);
  }
  assert.ok(got?.submit, `expected a submit, got ${JSON.stringify(got)}`);
  assert.deepEqual([got.submit.pallet_id, got.submit.destination_id], ["P-1", "D-1"]);
});

test("a mission of the same pallet and destination seen before the refusal: no resubmit, it stops", () => {
  const q = running();
  const s1 = submit(q, 10_000);
  q.onState({ ...READY, mode: "AUTO", stop_kind: "none",
              mission: { mission_id: "M-9", status: "running", pallet_id: "P-1", destination_id: "D-1" } }, 10_050);
  refuse(q, s1);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "거부: not_stopped");
});

test("a mission seen earlier in the session (a manual one): the queue's first submit is not 'just after the start'", () => {
  const q = running();
  q.onResult({ mission_id: "M-1", outcome: "done" }, 9_000);
  refuse(q, submit(q, 10_100));
  assert.equal(q.stopReason, "거부: not_stopped");
});

test("the front item removed or moved while it waits stops at once", () => {
  const q = running();
  const s1 = submit(q, 10_000);
  refuse(q, s1);
  q.remove(s1.item_id);
  assert.equal(q.run, "stopped");
  assert.equal(q.phase, "waiting");
  assert.equal(q.stopReason, "시작 직후 재제출 취소: 맨 앞 항목이 바뀜");
  const r = running([item(), item("P-4")]);
  const s2 = submit(r, 10_000);
  refuse(r, s2);
  r.move(s2.item_id, 1);
  assert.equal(r.stopReason, "시작 직후 재제출 취소: 맨 앞 항목이 바뀜");
});

test("a STOP and rearm between two states (control_gen moved) stops the wait", () => {
  const q = running();
  q.onState({ ...READY, control_gen: 1 }, 10_000);
  const n = q.next(ctx({ ...READY, control_gen: 1 }), 10_000, newId);
  refuse(q, n.submit);
  q.onState({ ...READY, control_gen: 3 }, 10_200);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "시작 직후 재제출 취소: 제어 세대가 바뀜");
});

// ---- Codex round 2 ------------------------------------------------------------------------------------------------
test("a mission's result arriving while it waits ends 'just after the start': no resubmit, it stops", () => {
  const q = running();
  refuse(q, submit(q, 10_000));
  q.onResult({ mission_id: "M-7", outcome: "done" }, 10_200);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "시작 직후 재제출 취소: 미션이 보임");
  assert.deepEqual(standFor(q, 10_300, 12_000), []);
});

test("seeing a mission is a change the storage keeps, a stopped queue too", () => {
  const q = new MissionQueue();
  q.onWelcome(3);
  const v0 = q.version;
  q.onResult({ mission_id: "M-1", outcome: "done" }, 1_000);
  assert.ok(q.version > v0);
  assert.equal(q.toStorage().mission_seen_this_epoch, true);
  const r = new MissionQueue();
  r.onWelcome(3);
  const v1 = r.version;
  r.onState({ ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: "M-2", status: "running" } }, 1_000);
  assert.ok(r.version > v1);
});

test("an older stored queue without the session's counts is restored as if the resubmit were spent", () => {
  const q = running();
  submit(q, 10_000);
  const old = q.toStorage();
  delete old.submits_this_epoch;
  delete old.start_retry_used;
  delete old.mission_seen_this_epoch;
  const r = MissionQueue.fromStorage(old, { epoch: 3, results: [] });
  r.onWelcome(3);
  r.resolveUncertain(r.items[0].item_id, "resubmit");
  r.onState(READY, 20_000);
  r.resume(20_000);
  refuse(r, submit(r, 20_000));
  assert.equal(r.stopReason, "거부: not_stopped");
});
