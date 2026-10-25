import { test } from "node:test";
import assert from "node:assert/strict";
import { MissionQueue, newItem, presetStatus, resolveItem } from "../js/queue.js";

// ---- fixtures: a vehicle standing ready, a targets list, a clock ------------------------------------------------
const TARGETS = {
  targets_revision: 7,
  pallets: [
    { pallet_id: "P-1", state: "available" },
    { pallet_id: "P-2", state: "delivered" },
    { pallet_id: "P-3", state: "carried" },
  ],
  destinations: [{ destination_id: "D-0" }, { destination_id: "D-1" }],
};
const READY = { mode: "PAUSED", stop_kind: "controlled", stopped_confirmed: true, feedback_ok: true, mission: null };

let ids = 0;
const newId = () => `id-${++ids}`;

function item(pallet = "P-1", dest = "D-1", kind = "fixed") {
  return newItem({ kind, pallet_id: pallet, destination_id: dest, label: `${pallet}→${dest}` }, 1000, newId);
}

// A queue that has seen a welcome and a ready state, with items, resumed.
function running(items = [item()], { epoch = 3, t = 10_000 } = {}) {
  const q = new MissionQueue();
  q.onWelcome(epoch);
  q.onState(READY, t);
  for (const it of items) q.add(it);
  assert.equal(q.resume(t), true);
  return q;
}

const ctx = (over = {}) => ({ connected: true, welcomed: true, hasLease: true, state: READY, targets: TARGETS, ...over });

function submitOnce(q, t, c = ctx()) {
  q.onState(c.state, t);
  const n = q.next(c, t, newId);
  assert.ok(n && n.submit, "expected a submit");
  return n.submit;
}

// States keep coming while a mission runs (the vehicle sends them at 10-20 Hz): one per second here.
function feed(q, from, to, st = { ...READY, mode: "AUTO", stop_kind: "none" }) {
  for (let t = from; t <= to; t += 1000) q.onState(st, t);
}

function ack(q, s, missionId, epoch = 3) {
  return q.onAnswer({ epoch, item_id: s.item_id, request_id: s.request_id }, { t: "ack", mission_id: missionId });
}

// ---- tests ------------------------------------------------------------------------------------------------------
test("a new queue and a restored one are stopped and ask for nothing", () => {
  const q = new MissionQueue();
  q.onWelcome(3);
  q.onState(READY, 0);
  q.add(item());
  assert.equal(q.run, "stopped");
  assert.equal(q.next(ctx(), 0, newId), null);
  const r = MissionQueue.fromStorage(running().toStorage(), { epoch: 3, results: [] });
  assert.equal(r.run, "stopped");
  assert.match(r.stopReason, /다시 열림/);
  r.onState(READY, 0);
  assert.equal(r.next(ctx(), 0, newId), null);
});

test("resumed with every condition true it submits the front item with the targets revision", () => {
  const first = item("P-1", "D-1");
  const q = running([first, item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  assert.equal(s.item_id, first.item_id);
  assert.equal(s.pallet_id, "P-1");
  assert.equal(s.destination_id, "D-1");
  assert.equal(s.targets_revision, 7);
  assert.ok(s.request_id);
  assert.equal(q.phase, "submitting");
  assert.equal(q.next(ctx(), 10_100, newId), null, "no second submit while one is out");
});

test("an unmet condition waits without stopping", () => {
  const cases = [
    ctx({ connected: false }),
    ctx({ welcomed: false }),
    ctx({ hasLease: false }),
    ctx({ targets: null }),
    ctx({ state: { ...READY, mission: { mission_id: "M-9", status: "running" } } }),
    ctx({ state: { ...READY, mission: { mission_id: "M-9", status: "cancelling" } } }),
    ctx({ state: { ...READY, stopped_confirmed: false } }),
    ctx({ state: { ...READY, mode: "AUTO" } }),
    ctx({ state: { ...READY, mode: null } }),
  ];
  for (const c of cases) {
    const q = running();
    q.onState(c.state, 10_000);
    // a foreign active mission is a stop event (tested below); here only the gate matters
    if (q.run !== "running") continue;
    assert.equal(q.next(c, 10_000, newId), null, JSON.stringify(c.state));
    assert.equal(q.run, "running");
  }
});

test("an item that does not resolve stops the queue and stays at the front", () => {
  for (const it of [item("P-9", "D-1"), item("P-3", "D-1"), item("P-1", "D-9"), item("P-1", "D-1", "auto")]) {
    const q = running([it]);
    q.onState(READY, 10_000);
    assert.equal(q.next(ctx(), 10_000, newId), null);
    assert.equal(q.run, "stopped");
    assert.equal(q.items[0].item_id, it.item_id);
  }
});

test("resolveItem follows the wizard's pick rules, the load and the kind", () => {
  assert.deepEqual(resolveItem(item("P-1", "D-1"), TARGETS, READY), { ok: true, pallet_id: "P-1", destination_id: "D-1" });
  assert.equal(resolveItem(item("P-2", "D-0"), TARGETS, READY).ok, true, "delivered is pickable");
  assert.equal(resolveItem(item("P-3", "D-0"), TARGETS, READY).ok, false, "carried, not the load");
  const loaded = { ...READY, load: { state: "loaded", pallet_id: "P-3" } };
  assert.equal(resolveItem(item("P-3", "D-0"), TARGETS, loaded).ok, true, "the pallet on the forks");
  assert.equal(resolveItem(item("P-1", "D-1", "auto"), TARGETS, READY).reason, "unsupported_kind");
  assert.equal(resolveItem(item("P-1", "D-1"), null, READY).ok, false);
});

test("after a result it waits 1.0 s before the next submit", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  ack(q, s, "M-1");
  feed(q, 11_000, 20_000);
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_000);
  q.onState(READY, 20_500);
  assert.equal(q.next(ctx(), 20_500, newId), null);
  q.onState(READY, 21_000);
  assert.ok(q.next(ctx(), 21_000, newId)?.submit);
});

test("done moves on and the last done stops with 완료; failures and cancels stop", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  let s = submitOnce(q, 10_000);
  ack(q, s, "M-1");
  feed(q, 11_000, 20_000);
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_000);
  assert.equal(q.run, "running");
  assert.equal(q.items.length, 1);
  s = submitOnce(q, 21_000);
  ack(q, s, "M-2");
  feed(q, 22_000, 30_000);
  q.onResult({ seq: 2, mission_id: "M-2", outcome: "done" }, 30_000);
  assert.equal(q.run, "stopped");
  assert.match(q.stopReason, /완료/);
  assert.equal(q.counts.done, 2);

  for (const [outcome, word] of [["failed", /실패/], ["recovery_required", /실패/], ["cancelled", /취소/]]) {
    const r = running([item("P-1", "D-1"), item("P-2", "D-0")]);
    const s2 = submitOnce(r, 10_000);
    ack(r, s2, "M-5");
    r.onResult({ seq: 5, mission_id: "M-5", outcome, reason: "x" }, 20_000);
    assert.equal(r.run, "stopped", outcome);
    assert.match(r.stopReason, word);
    assert.equal(r.items.length, 1, "the next item waits for the operator");
    r.onState(READY, 22_000);
    assert.equal(r.next(ctx(), 22_000, newId), null);
  }
});

test("the same result seen twice (last_result and missions) counts once; other missions' results are ignored", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  ack(q, s, "M-1");
  q.onResult({ seq: 9, mission_id: "M-0", outcome: "failed" }, 15_000);
  assert.equal(q.run, "running", "a past mission's result");
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_000);
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_100);
  assert.equal(q.counts.done, 1);
  assert.equal(q.items.length, 1);
});

test("a result that arrives before the ack is applied when the ack comes", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 12_000);
  ack(q, s, "M-1");
  assert.equal(q.counts.done, 1);
  assert.equal(q.items.length, 1);
});

// (The session's first submit refused not_stopped is plan D5's one exception: queue_start_retry.test.mjs.)
test("every reject stops; busy and stale also mark the item uncertain; nothing is resubmitted", () => {
  for (const [reason, uncertain] of [["destination_occupied", false], ["stale_targets", false],
    ["no_feedback:rear_axle_speed", false], ["busy", true], ["stale", true]]) {
    const it = item();
    const q = running([it]);
    const s = submitOnce(q, 10_000);
    q.onAnswer({ epoch: 3, item_id: s.item_id, request_id: s.request_id }, { t: "reject", reason });
    assert.equal(q.run, "stopped", reason);
    assert.equal(q.items[0].item_id, it.item_id);
    assert.equal(Boolean(q.items[0].uncertain), uncertain, reason);
    q.onState(READY, 20_000);
    assert.equal(q.next(ctx(), 20_000, newId), null);
  }
});

test("operator actions stop at once, and a later done does not submit the next (cancel right after completion)", () => {
  for (const kind of ["cancel", "pause", "stop", "manual_submit"]) {
    const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
    const s = submitOnce(q, 10_000);
    ack(q, s, "M-1");
    q.operatorAction(kind);
    assert.equal(q.run, "stopped", kind);
    q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_000);
    assert.equal(q.counts.done, 1, "the result is still recorded");
    q.onState(READY, 22_000);
    assert.equal(q.next(ctx(), 22_000, newId), null, kind);
  }
});

test("STOPPED, TELEOP, a safety preempt and the queue mission paused or cancelling stop; normal PAUSED does not", () => {
  const stops = [
    { ...READY, mode: "STOPPED", stop_kind: "operator" },
    { ...READY, mode: "TELEOP", stop_kind: "none" },
    { ...READY, stop_kind: "preempt" },
    { ...READY, stop_kind: "safety_preempt" },
  ];
  for (const st of stops) {
    const q = running();
    q.onState(st, 10_000);
    assert.equal(q.run, "stopped", JSON.stringify(st));
  }
  for (const status of ["paused", "cancelling"]) {
    const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
    const s = submitOnce(q, 10_000);
    ack(q, s, "M-1");
    q.onState({ ...READY, mode: "AUTO", mission: { mission_id: "M-1", status } }, 11_000);
    assert.equal(q.run, "stopped", status);
  }
  for (const kind of ["controlled", "controlled_decel"]) {
    const q = running([item("P-1", "D-1"), item("P-2", "D-0"), item("P-1", "D-0")]);
    let t = 10_000;
    for (let i = 1; i <= 3; i += 1) {
      const s = submitOnce(q, t, ctx({ state: { ...READY, stop_kind: kind } }));
      ack(q, s, `M-${i}`);
      q.onState({ ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: `M-${i}`, status: "running" } }, t + 500);
      q.onResult({ seq: i, mission_id: `M-${i}`, outcome: "done" }, t + 1_000);
      q.onState({ ...READY, stop_kind: kind }, t + 1_500);
      t += 2_100;
    }
    assert.equal(q.counts.done, 3, kind);
    assert.match(q.stopReason, /완료/);
  }
});

test("a lost vehicle feedback between items stops, and its recovery does not submit", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  ack(q, s, "M-1");
  feed(q, 11_000, 20_000);
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 20_000);
  q.onState({ ...READY, feedback_ok: false, stopped_confirmed: false }, 20_200);
  assert.equal(q.next(ctx({ state: { ...READY, feedback_ok: false } }), 20_200, newId), null, "nothing goes out while it is lost");
  q.onState({ ...READY, feedback_ok: false, stopped_confirmed: false }, 20_700);
  assert.equal(q.run, "stopped");
  assert.match(q.stopReason, /피드백/);
  q.onState(READY, 21_500);
  assert.equal(q.next(ctx(), 21_500, newId), null);
});

test("one stale-feedback state while the queue's mission runs does not stop it (Isaac live run 2026-10-22)", () => {
  // the live runner sends a state every 0.1 s with a 0.1 s feedback age limit; a control tick longer than that
  // (rendering, planning) shows as one state with feedback_ok false during a normal AUTO run
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  ack(q, submitOnce(q, 10_000), "M-1");
  const auto = { ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: "M-1", status: "running" } };
  feed(q, 10_100, 15_000, auto);
  q.onState({ ...auto, feedback_ok: false }, 15_100);
  q.onState(auto, 15_200);
  q.onState({ ...auto, feedback_ok: false }, 15_500);
  q.onState(auto, 15_600);
  assert.equal(q.run, "running");
  q.onState({ ...auto, feedback_ok: false }, 16_000);
  q.onState({ ...auto, feedback_ok: false }, 16_300);
  assert.equal(q.run, "running", "0.3 s of loss");
  q.onState({ ...auto, feedback_ok: false }, 16_500);
  assert.equal(q.run, "stopped", "0.5 s of loss is a loss");
  assert.match(q.stopReason, /피드백/);
});

test("a loss that outlasts 0.5 s stops even when the next state already says recovered (Codex ops round 1)", () => {
  for (const viaTick of [true, false]) {
    const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
    ack(q, submitOnce(q, 10_000), "M-1");
    feed(q, 10_100, 11_000, { ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: "M-1", status: "running" } });
    q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 11_200);
    for (let t = 11_250; t <= 11_650; t += 100) q.onState({ ...READY, feedback_ok: false, stopped_confirmed: false }, t);
    assert.equal(q.run, "running", "0.4 s so far");
    if (viaTick) q.tick(11_800);
    q.onState({ ...READY, stopped_confirmed: false }, 12_000); // recovered, not yet standing
    q.onState(READY, 12_250);
    assert.equal(q.run, "stopped", viaTick ? "the timer" : "the recovered state");
    assert.equal(q.next(ctx(), 12_300, newId), null);
  }
});

test("a foreign active mission stops the queue; our own mission seen before the ack does not", () => {
  const q = running();
  q.onState({ ...READY, mode: "AUTO", mission: { mission_id: "M-7", status: "running", pallet_id: "P-2", destination_id: "D-0" } }, 10_000);
  assert.equal(q.run, "stopped");

  const r = running([item("P-1", "D-1")]);
  submitOnce(r, 10_000);
  r.onState({ ...READY, mode: "AUTO", mission: { mission_id: "M-8", status: "preparing", pallet_id: "P-1", destination_id: "D-1" } }, 10_200);
  assert.equal(r.run, "running", "the submit's own mission before its ack");
});

test("a stop while submitting returns the request to forget and leaves the item uncertain; its late ack is ignored", () => {
  for (const stop of [(q) => q.onDisconnect(), (q) => q.onLeaseLost(), (q) => q.stop("운용자"), (q) => q.operatorAction("stop")]) {
    const q = running();
    const s = submitOnce(q, 10_000);
    const out = stop(q);
    assert.equal(out.forget, s.request_id);
    assert.equal(q.run, "stopped");
    assert.equal(q.items[0].uncertain, true);
    ack(q, s, "M-1");
    assert.equal(q.phase, "waiting");
    assert.equal(q.current, null);
  }
});

test("disconnect, lease loss and a new epoch stop; a new epoch makes the in-mission item uncertain", () => {
  let q = running();
  q.onDisconnect();
  assert.match(q.stopReason, /연결/);
  q = running();
  q.onLeaseLost();
  assert.match(q.stopReason, /권한/);
  q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  ack(q, s, "M-1");
  q.onWelcome(4);
  assert.match(q.stopReason, /새 세션/);
  assert.equal(q.items[0].uncertain, true);
  assert.equal(q.current, null);
});

test("a reconnect in the same epoch stays stopped but records the mission's result", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  ack(q, s, "M-1");
  q.onDisconnect();
  q.onWelcome(3);
  q.onState(READY, 30_000);
  q.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 30_000);
  assert.equal(q.counts.done, 1);
  assert.equal(q.run, "stopped");
  q.onState(READY, 32_000);
  assert.equal(q.next(ctx(), 32_000, newId), null);
});

test("a state silence of over 2 s stops, by the timer or by the late state itself", () => {
  const q = running();
  q.onState(READY, 10_000);
  q.tick(11_990);
  assert.equal(q.run, "running");
  q.tick(12_100);
  assert.equal(q.run, "stopped");

  const r = running();
  r.onState(READY, 10_000);
  r.tick(11_990);
  r.onState(READY, 12_010); // arrives before the next timer check
  assert.equal(r.run, "stopped");
  assert.match(r.stopReason, /상태/);
});

test("an uncertain front item blocks resume until dropped or marked resubmit", () => {
  const q = running();
  submitOnce(q, 10_000);
  q.onDisconnect();
  assert.equal(q.resume(11_000), false);
  const id = q.items[0].item_id;
  assert.equal(q.resolveUncertain(id, "resubmit"), true);
  assert.equal(q.items[0].uncertain, false);
  assert.equal(q.resume(11_000), true);

  const r = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  submitOnce(r, 10_000);
  r.onDisconnect();
  r.resolveUncertain(r.items[0].item_id, "drop");
  assert.equal(r.items.length, 1);
  assert.equal(r.resume(11_000), true);
});

test("restore: stopped; submitting becomes uncertain; in-mission is settled from the results or uncertain", () => {
  let q = running();
  submitOnce(q, 10_000);
  let r = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  assert.equal(r.run, "stopped");
  assert.equal(r.items[0].uncertain, true);

  q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  ack(q, submitOnce(q, 10_000), "M-1");
  const saved = JSON.parse(JSON.stringify(q.toStorage()));
  r = MissionQueue.fromStorage(saved, { epoch: 3, results: [{ seq: 1, mission_id: "M-1", outcome: "done" }] });
  assert.equal(r.counts.done, 1);
  assert.equal(r.items.length, 1);
  assert.equal(r.items[0].uncertain, undefined);
  r = MissionQueue.fromStorage(saved, { epoch: 3, results: [] });
  assert.equal(r.items[0].uncertain, true, "the result is not in the kept list");
  r = MissionQueue.fromStorage(saved, { epoch: 4, results: [{ seq: 1, mission_id: "M-1", outcome: "done" }] });
  assert.equal(r.items[0].uncertain, true, "another session's results are not this one's");
  assert.equal(MissionQueue.fromStorage(null, { epoch: 3, results: [] }).items.length, 0);
  assert.equal(MissionQueue.fromStorage({ garbage: true }, { epoch: 3, results: [] }).items.length, 0);
});

test("editing: the item out or in a mission cannot be moved or removed; an uncertain one can be removed", () => {
  const a = item("P-1", "D-1");
  const b = item("P-2", "D-0");
  const c = item("P-1", "D-0");
  const q = running([a, b, c]);
  submitOnce(q, 10_000);
  assert.equal(q.remove(a.item_id), false);
  assert.equal(q.move(a.item_id, 1), false);
  assert.equal(q.move(c.item_id, -1), true);
  assert.deepEqual(q.items.map((x) => x.item_id), [a.item_id, c.item_id, b.item_id]);
  assert.equal(q.move(c.item_id, -1), false, "nothing moves in front of the item out");
  q.onDisconnect();
  assert.equal(q.remove(a.item_id), true);
});

test("presetStatus says why a saved transport cannot be queued now", () => {
  assert.deepEqual(presetStatus({ pallet_id: "P-1", destination_id: "D-1" }, TARGETS, READY), { usable: true, reason: null });
  assert.match(presetStatus({ pallet_id: "P-9", destination_id: "D-1" }, TARGETS, READY).reason, /팔레트/);
  assert.match(presetStatus({ pallet_id: "P-1", destination_id: "D-9" }, TARGETS, READY).reason, /하역/);
  assert.match(presetStatus({ pallet_id: "P-1", destination_id: "D-1" }, null, READY).reason, /대상 목록/);
});

test("an uncertain item that named its mission is settled when that mission's result comes in the same session", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  ack(q, submitOnce(q, 10_000), "M-1");
  const r = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  assert.equal(r.items[0].uncertain, true);
  r.onWelcome(3);
  r.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 40_000);
  assert.equal(r.items.length, 1, "settled and out");
  assert.equal(r.counts.done, 1);
  assert.equal(r.run, "stopped", "still waits for the operator");
  const other = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  other.onWelcome(4);
  other.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 40_000);
  assert.equal(other.items[0].uncertain, true, "another session's M-1 is not this one");
});

test("a matching mission seen before the ack must be the one the ack names (Codex round 4)", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const s = submitOnce(q, 10_000);
  // another console's submit for the same pallet and destination shows up first
  q.onState({ ...READY, mode: "AUTO", mission: { mission_id: "M-8", status: "preparing", pallet_id: "P-1", destination_id: "D-1" } }, 10_200);
  assert.equal(q.run, "running");
  ack(q, s, "M-9");
  assert.equal(q.run, "stopped");
  assert.match(q.stopReason, /대기열이 내지 않은 미션/);

  const r = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  const t = submitOnce(r, 10_000);
  r.onState({ ...READY, mode: "AUTO", mission: { mission_id: "M-8", status: "preparing", pallet_id: "P-1", destination_id: "D-1" } }, 10_200);
  ack(r, t, "M-8");
  assert.equal(r.run, "running", "the same mission: ours");
});

test("an uncertain item anywhere blocks resume and nothing moves across it (Codex round 4)", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  ack(q, submitOnce(q, 10_000), "M-1");
  const r = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  r.onWelcome(3);
  const [u, b] = r.items;
  assert.equal(u.uncertain, true);
  assert.equal(r.move(b.item_id, -1), false, "nothing moves in front of an uncertain item");
  assert.equal(r.move(u.item_id, 1), false, "an uncertain item does not move");
  assert.equal(r.resume(40_000), false);
});

test("a late result for an uncertain item leaves the submit in flight alone (Codex round 4)", () => {
  const q = running([item("P-1", "D-1"), item("P-2", "D-0")]);
  ack(q, submitOnce(q, 10_000), "M-1");
  const r = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  r.onWelcome(3);
  r.onState(READY, 40_000);
  const [u, b] = r.items;
  r.resolveUncertain(u.item_id, "drop");
  r.add({ ...u, item_id: "again", uncertain: false, maybe: undefined }); // A queued again, behind B
  assert.equal(r.resume(40_000), true);
  const s = submitOnce(r, 40_000);
  assert.equal(s.item_id, b.item_id);
  r.onResult({ seq: 1, mission_id: "M-1", outcome: "done" }, 40_100);
  assert.equal(r.current?.request_id, s.request_id, "the submit for B is still tracked");
  assert.equal(r.phase, "submitting");
});

test("after a reconnect only a state received since counts: an old one never lets a resume submit (Codex round 5)", () => {
  const q = new MissionQueue();
  q.onWelcome(3);
  q.onState(READY, 10_000);
  q.add(item());
  q.onDisconnect();
  q.onWelcome(3); // the same session again; the page still holds the 90 s old state
  assert.equal(q.resume(100_000), true);
  assert.equal(q.next(ctx(), 100_000, newId), null, "no state since the welcome");
  assert.deepEqual(q.tick(101_000), {});
  assert.equal(q.run, "running", "waits for the first state");
  q.tick(102_100);
  assert.equal(q.run, "stopped", "no first state within 2 s");
  assert.equal(q.stopReason, "상태 수신 끊김");
  assert.equal(q.resume(103_000), true);
  q.onState(READY, 103_500);
  assert.ok(q.next(ctx(), 103_500, newId), "a state received since: it may submit");
});

test("marking an uncertain item for resubmit forgets the mission it may have started", () => {
  const q = running([item("P-1", "D-1")]);
  ack(q, submitOnce(q, 10_000), "M-1");
  const r = MissionQueue.fromStorage(JSON.parse(JSON.stringify(q.toStorage())), { epoch: 3, results: [] });
  r.onWelcome(3);
  r.resolveUncertain(r.items[0].item_id, "resubmit");
  assert.equal(r.items[0].maybe, undefined);
});
