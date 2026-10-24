import { test } from "node:test";
import assert from "node:assert/strict";
import { MissionQueue, newItem } from "../js/queue.js";

// CT-2 console plan (docs/plans/2026-10-24-ct2-console-auto-item.md): the queue's auto item "all remaining pallets"
// asks the vehicle for the next pallet and destination (pick_request -> pick) and submits them as fixed ids.

const TARGETS = {
  targets_revision: 7,
  pallets: [{ pallet_id: "P-1", state: "available" }, { pallet_id: "P-2", state: "available" },
            { pallet_id: "P-3", state: "delivered" }],
  destinations: [{ destination_id: "D-1" }, { destination_id: "D-2" }],
};
const READY = { mode: "PAUSED", stop_kind: "controlled", stopped_confirmed: true, feedback_ok: true, mission: null };
const AUTO_ST = (id) => ({ ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: id, status: "running" } });

let ids = 0;
const newId = () => `id-${++ids}`;
const auto = () => newItem({ kind: "auto", label: "남은 팔레트 전부" }, 1000, newId);
const fixed = (p, d) => newItem({ kind: "fixed", pallet_id: p, destination_id: d }, 1000, newId);
const ctx = (over = {}) => ({ connected: true, welcomed: true, hasLease: true, state: READY, targets: TARGETS,
                              nextPick: true, ...over });

function running(items, t = 10_000) {
  const q = new MissionQueue();
  q.onWelcome(3);
  q.onState(READY, t);
  for (const it of items) q.add(it);
  assert.equal(q.resume(t), true);
  return q;
}
// states every second, as the vehicle sends them
function feed(q, from, to, st = READY) {
  for (let t = from; t <= to; t += 1000) q.onState(st, t);
}
// asks for a pick and (by default) has the vehicle ack it -- the contract's usual order
function ask(q, t, c = ctx(), { ack = true } = {}) {
  q.onState(c.state, t);
  const n = q.next(c, t, newId);
  assert.ok(n && n.pick_request, `expected a pick_request, got ${JSON.stringify(n)}`);
  const key = { epoch: q.epoch, item_id: n.pick_request.item_id, request_id: n.pick_request.request_id };
  if (ack) q.onAnswer(key, { t: "ack", request_id: key.request_id });
  return key;
}
const pick = (key, pallet, dest, over = {}) => ({ t: "pick", request_id: key.request_id, targets_revision: 7,
  pick: pallet ? { pallet_id: pallet, destination_id: dest } : null, reason: pallet ? "ok" : "none_remaining",
  plan: pallet ? [[pallet, dest]] : [], missions: pallet ? 1 : 0, ...over });

test("an auto item at the front asks the vehicle for a pick, not a submit", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  assert.equal(q.phase, "picking");
  assert.equal(q.current, null);
  assert.equal(q.next(ctx(), 10_100, newId), null, "one request at a time");
  assert.equal(key.epoch, 3);
});

test("a pick is held, and submitted only once the submit conditions hold again", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-2", "D-1"), 10_300, 7);
  assert.equal(q.phase, "picked");
  assert.equal(q.next(ctx({ state: { ...READY, stopped_confirmed: false } }), 10_400, newId), null, "waits, not stops");
  assert.equal(q.run, "running");
  q.onState(READY, 10_500);
  const n = q.next(ctx(), 10_500, newId);
  assert.deepEqual([n.submit.pallet_id, n.submit.destination_id, n.submit.targets_revision], ["P-2", "D-1", 7]);
  assert.equal(q.phase, "submitting");
  assert.deepEqual([q.current.pallet_id, q.current.destination_id], ["P-2", "D-1"]);
});

test("a pick for another targets revision asks once more, a second one stops", () => {
  const q = running([auto()]);
  const k1 = ask(q, 10_000);
  q.onPick(k1, pick(k1, "P-1", "D-1", { targets_revision: 6 }), 10_200, 7);
  assert.equal(q.phase, "waiting");
  const k2 = ask(q, 10_300);
  assert.notEqual(k2.request_id, k1.request_id);
  q.onPick(k2, pick(k2, "P-1", "D-1", { targets_revision: 6 }), 10_500, 7);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "대상 목록이 계속 바뀜");
});

test("the targets changing while a pick is held asks again (the same one re-ask)", () => {
  const q = running([auto()]);
  const k1 = ask(q, 10_000);
  q.onPick(k1, pick(k1, "P-1", "D-1"), 10_200, 7);
  const k2 = ask(q, 10_300, ctx({ targets: { ...TARGETS, targets_revision: 8 } }));
  q.onPick(k2, pick(k2, "P-1", "D-1", { targets_revision: 9 }), 10_500, 8);
  assert.equal(q.run, "stopped", "that was the one re-ask");
});

test("none remaining ends the auto item and goes on to the next item, or ends the queue", () => {
  const q = running([auto(), fixed("P-1", "D-2")]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, null, null), 10_200, 7);
  assert.equal(q.items.length, 1);
  assert.equal(q.items[0].kind, "fixed");
  assert.equal(q.run, "running");
  q.onState(READY, 10_300);
  assert.ok(q.next(ctx(), 10_300, newId).submit);
  const r = running([auto()]);
  const k = ask(r, 10_000);
  r.onPick(k, pick(k, null, null), 10_200, 7);
  assert.equal(r.run, "stopped");
  assert.equal(r.stopReason, "완료");
  assert.equal(r.items.length, 0);
});

test("no free destination, all blocked and a computation error stop with the auto item kept, not uncertain", () => {
  for (const reason of ["no_free_destination", "all_blocked", "error:ValueError", "something_new"]) {
    const q = running([auto()]);
    const key = ask(q, 10_000);
    q.onPick(key, { ...pick(key, null, null), reason, blocked: { "P-1": ["pickup:P-2"] } }, 10_200, 7);
    assert.equal(q.run, "stopped", reason);
    assert.equal(q.stopReason, `자동 선택 실패: ${reason}`);
    assert.equal(q.items.length, 1);
    assert.ok(!q.items[0].uncertain, reason);
    assert.deepEqual(q.lastPickBlocked, { "P-1": ["pickup:P-2"] });
  }
});

test("a refused pick_request stops, nothing uncertain", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000, ctx(), { ack: false });
  q.onAnswer(key, { t: "reject", request_id: key.request_id, reason: "mission_active" });
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "자동 선택 거부: mission_active");
  assert.ok(!q.items[0].uncertain);
  assert.equal(q.phase, "waiting");
});

test("no pick within the vehicle's deadline stops; a late pick is ignored", () => {
  const q = running([auto()]);
  q.pickDeadlineMs = 5000;
  const key = ask(q, 10_000);
  q.onAnswer(key, { t: "ack", request_id: key.request_id }); // acked, the pick does not come
  feed(q, 11_000, 14_000);
  assert.deepEqual(q.tick(14_900), {});
  assert.equal(q.run, "running");
  const out = q.tick(15_100);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "자동 선택 응답 없음");
  assert.equal(out.forget, key.request_id);
  q.onPick(key, pick(key, "P-1", "D-1"), 15_200, 7);
  assert.equal(q.phase, "waiting");
  assert.ok(!q.items[0].uncertain);
});

test("a stop while picking forgets the request and leaves nothing uncertain; its late pick is ignored", () => {
  for (const hit of [(q) => q.onDisconnect(), (q) => q.operatorAction("stop"), (q) => q.onLeaseLost(),
                     (q) => q.onState({ ...READY, mode: "STOPPED" }, 10_400)]) {
    const q = running([auto()]);
    const key = ask(q, 10_000);
    const out = hit(q);
    assert.equal(q.run, "stopped");
    assert.equal(q.phase, "waiting");
    assert.ok(!q.items[0].uncertain);
    if (out && "forget" in out) assert.equal(out.forget, key.request_id);
    q.onPick(key, pick(key, "P-1", "D-1"), 10_500, 7);
    assert.equal(q.phase, "waiting");
  }
});

test("a stop while a pick is held drops it", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-1", "D-1"), 10_200, 7);
  q.operatorAction("stop");
  assert.equal(q.phase, "waiting");
  assert.ok(!q.items[0].uncertain);
  q.resume(11_000);
  q.onState(READY, 11_100);
  assert.ok(q.next(ctx(), 11_100, newId).pick_request, "asks again after a resume");
});

test("a pick that comes before its ack is held, and submitted only once the ack has come (Codex CT-2 r1)", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000, ctx(), { ack: false });
  q.onPick(key, pick(key, "P-1", "D-1"), 10_100, 7);
  assert.equal(q.phase, "picked");
  q.onState(READY, 10_200);
  assert.equal(q.next(ctx(), 10_200, newId), null, "not before the ack");
  q.onAnswer(key, { t: "ack", request_id: key.request_id });
  q.onState(READY, 10_300);
  assert.ok(q.next(ctx(), 10_300, newId).submit);
});

test("a refusal after the pick stops the queue and nothing is submitted (Codex CT-2 r1)", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000, ctx(), { ack: false });
  q.onPick(key, pick(key, "P-1", "D-1"), 10_100, 7);
  q.onAnswer(key, { t: "reject", request_id: key.request_id, reason: "busy" });
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "자동 선택 거부: busy");
  q.onState(READY, 10_300);
  assert.equal(q.next(ctx(), 10_300, newId), null);
  assert.ok(!q.items[0].uncertain);
});

test("a held pick whose ack does not come within the deadline stops and the request is forgotten (Codex CT-2 r1)", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000, ctx(), { ack: false });
  q.onPick(key, pick(key, "P-1", "D-1"), 10_100, 7);
  feed(q, 11_000, 15_000);
  q.tick(15_100);
  assert.equal(q.stopReason, "자동 선택 응답 없음");
  assert.ok(q.drainForget().includes(key.request_id));
});

test("a stop while a pick is held before its ack, and a re-ask, forget the unacked request (Codex CT-2 r1)", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000, ctx(), { ack: false });
  q.onPick(key, pick(key, "P-1", "D-1"), 10_100, 7);
  q.operatorAction("stop");
  assert.deepEqual(q.drainForget(), [key.request_id]);
  const r = running([auto()]);
  const k1 = ask(r, 10_000, ctx(), { ack: false });
  r.onPick(k1, pick(k1, "P-1", "D-1", { targets_revision: 6 }), 10_100, 7);
  assert.deepEqual(r.drainForget(), [k1.request_id], "the re-ask lets the first request go");
});

test("a pick that arrives after the deadline is not taken (Codex CT-2 r1)", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  feed(q, 11_000, 15_000);
  q.onPick(key, pick(key, "P-1", "D-1"), 15_100, 7);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "자동 선택 응답 없음");
  q.onState(READY, 15_250);
  assert.equal(q.next(ctx(), 15_250, newId), null);
});

test("a busy refusal of an auto submit keeps the pallet and destination on the uncertain mark (Codex CT-2 r1)", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-2", "D-1"), 10_100, 7);
  q.onState(READY, 10_200);
  const s = q.next(ctx(), 10_200, newId).submit;
  q.onAnswer({ epoch: 3, item_id: s.item_id, request_id: s.request_id }, { t: "reject", reason: "busy" });
  assert.equal(q.items[0].uncertain, true);
  assert.deepEqual([q.items[0].uncertain_pallet_id, q.items[0].uncertain_destination_id], ["P-2", "D-1"]);
  const stored = JSON.parse(JSON.stringify(q.toStorage())).items[0];
  assert.deepEqual([stored.uncertain_pallet_id, stored.uncertain_destination_id], ["P-2", "D-1"]);
});

test("removing the auto item lets go of its last pick; a done counts on the item (Codex CT-2 r1, agy)", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-1", "D-1"), 10_100, 7);
  q.operatorAction("stop");
  q.remove(q.items[0].item_id);
  assert.equal(q.lastPick, null);
  q.add(auto());
  assert.equal(q.lastPick, null);
  const r = running([auto()]);
  const k = ask(r, 10_000);
  r.onPick(k, pick(k, "P-2", "D-1"), 10_100, 7);
  r.onState(READY, 10_200);
  const s = r.next(ctx(), 10_200, newId).submit;
  r.onAnswer({ epoch: 3, item_id: s.item_id, request_id: s.request_id }, { t: "ack", mission_id: "M-1" });
  feed(r, 11_000, 20_000, AUTO_ST("M-1"));
  r.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_000);
  assert.equal(r.items[0].moved, 1);
});

test("a done auto mission is recorded with its pallet and destination and the auto item stays for the next pick", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-2", "D-1"), 10_100, 7);
  q.onState(READY, 10_200);
  const s = q.next(ctx(), 10_200, newId).submit;
  q.onAnswer({ epoch: 3, item_id: s.item_id, request_id: s.request_id }, { t: "ack", mission_id: "M-1" });
  feed(q, 11_000, 20_000, AUTO_ST("M-1"));
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_000);
  assert.equal(q.items.length, 1);
  assert.equal(q.items[0].kind, "auto");
  assert.equal(q.run, "running");
  assert.deepEqual([q.finished[0].pallet_id, q.finished[0].destination_id, q.finished[0].outcome], ["P-2", "D-1", "done"]);
  assert.equal(q.counts.done, 1);
  q.onState(READY, 20_500);
  assert.equal(q.next(ctx(), 20_500, newId), null, "the settle time after a result");
  q.onState(READY, 21_100);
  assert.ok(q.next(ctx(), 21_100, newId).pick_request);
});

test("a failed auto mission stops the queue with the auto item at the front", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-2", "D-1"), 10_100, 7);
  q.onState(READY, 10_200);
  const s = q.next(ctx(), 10_200, newId).submit;
  q.onAnswer({ epoch: 3, item_id: s.item_id, request_id: s.request_id }, { t: "ack", mission_id: "M-1" });
  feed(q, 11_000, 20_000, AUTO_ST("M-1"));
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "failed", reason: "x" }, 20_000);
  assert.equal(q.run, "stopped");
  assert.equal(q.items[0].kind, "auto");
});

test("an auto submit cut off is uncertain on that submit; clearing keeps the auto item, drop removes it", () => {
  const make = () => {
    const q = running([auto()]);
    const key = ask(q, 10_000);
    q.onPick(key, pick(key, "P-2", "D-1"), 10_100, 7);
    q.onState(READY, 10_200);
    q.next(ctx(), 10_200, newId);
    q.onDisconnect();
    return q;
  };
  const q = make();
  assert.equal(q.items[0].uncertain, true);
  assert.deepEqual([q.items[0].uncertain_pallet_id, q.items[0].uncertain_destination_id], ["P-2", "D-1"]);
  assert.equal(q.resume(11_000), false);
  assert.equal(q.resolveUncertain(q.items[0].item_id, "clear"), true);
  assert.equal(q.items.length, 1);
  assert.ok(!q.items[0].uncertain);
  assert.equal(q.resume(11_000), true);
  const r = make();
  assert.equal(r.resolveUncertain(r.items[0].item_id, "drop"), true);
  assert.equal(r.items.length, 0);
});

test("an uncertain auto submit whose mission ends in the same session is recorded and cleared, the item kept", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-2", "D-1"), 10_100, 7);
  q.onState(READY, 10_200);
  const s = q.next(ctx(), 10_200, newId).submit;
  q.onAnswer({ epoch: 3, item_id: s.item_id, request_id: s.request_id }, { t: "ack", mission_id: "M-1" });
  const r = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  r.onWelcome(3);
  assert.equal(r.items[0].uncertain, true);
  r.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 40_000);
  assert.equal(r.items.length, 1);
  assert.ok(!r.items[0].uncertain);
  assert.equal(r.counts.done, 1);
  assert.equal(r.run, "stopped", "still waits for the operator");
});

test("a reopened page never resumes a pick: picking and picked are not stored", () => {
  const q = running([auto()]);
  const key = ask(q, 10_000);
  q.onPick(key, pick(key, "P-2", "D-1"), 10_100, 7);
  const r = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  assert.equal(r.phase, "waiting");
  assert.ok(!r.items[0].uncertain);
  assert.equal(JSON.stringify(q.toStorage()).includes("picked"), false);
});

test("a vehicle without the feature stops the auto item at once", () => {
  const q = running([auto()]);
  q.onState(READY, 10_000);
  assert.equal(q.next(ctx({ nextPick: false }), 10_000, newId), null);
  assert.equal(q.run, "stopped");
  assert.equal(q.stopReason, "차량이 자동 선택을 지원하지 않습니다");
});

test("removing the auto item while it waits for a pick forgets the request", () => {
  const q = running([auto(), fixed("P-1", "D-2")]);
  const key = ask(q, 10_000, ctx(), { ack: false }); // still resending: an acked request has nothing to forget
  assert.equal(q.remove(q.items[0].item_id), true);
  assert.deepEqual(q.drainForget(), [key.request_id]);
  assert.deepEqual(q.drainForget(), []);
  assert.equal(q.phase, "waiting");
});

test("the last pick shown is let go by a failed pick and by a new session", () => {
  const q = running([auto()]);
  const k1 = ask(q, 10_000);
  q.onPick(k1, pick(k1, "P-1", "D-1"), 10_100, 7);
  assert.equal(q.lastPick.pallet_id, "P-1");
  q.operatorAction("stop");
  q.resume(11_000);
  const k2 = ask(q, 11_100);
  q.onPick(k2, { ...pick(k2, null, null), reason: "all_blocked" }, 11_200, 7);
  assert.equal(q.lastPick, null);
  const r = running([auto()]);
  const k3 = ask(r, 10_000);
  r.onPick(k3, pick(k3, "P-1", "D-1"), 10_100, 7);
  r.onWelcome(4);
  assert.equal(r.lastPick, null);
  assert.equal(r.lastPickBlocked, null);
});

test("only one auto item; nothing moves in front of it while it picks", () => {
  const q = new MissionQueue();
  assert.equal(q.add(auto()), true);
  assert.equal(q.add(auto()), false);
  const r = running([auto(), fixed("P-1", "D-2")]);
  ask(r, 10_000);
  assert.equal(r.move(r.items[1].item_id, -1), false);
});
