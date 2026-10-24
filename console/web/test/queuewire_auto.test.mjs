import { test } from "node:test";
import assert from "node:assert/strict";
import { createQueueWire } from "../js/queuewire.js";

// CT-2 console plan: the auto item through the wiring -- what goes out (pick_request, mission_submit) is counted.

const TARGETS = {
  targets_revision: 7,
  pallets: [{ pallet_id: "P-1", state: "available" }, { pallet_id: "P-2", state: "available" }],
  destinations: [{ destination_id: "D-1" }, { destination_id: "D-2" }],
};
const READY = { mode: "PAUSED", stop_kind: "controlled", stopped_confirmed: true, feedback_ok: true, mission: null };
const RUNNING = (id) => ({ ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: id, status: "running" } });
const CAPS = { control: true, mission: true, next_pick: true };

function fakeStore() {
  const data = new Map();
  return { data, getItem: (k) => (data.has(k) ? data.get(k) : null), setItem: (k, v) => data.set(k, v),
           removeItem: (k) => data.delete(k) };
}

function harness(caps = CAPS) {
  let t = 10_000, n = 0;
  const session = {
    sent: [], forgotten: [], answers: new Map(), state: null, targets: TARGETS, welcome: null, hasLease: true,
    request(type, fields, opts) {
      session.sent.push({ t: type, request_id: opts.requestId, ...fields });
      return new Promise((resolve) => session.answers.set(opts.requestId, resolve));
    },
    forget(id) { session.forgotten.push(id); session.answers.delete(id); },
  };
  const w = createQueueWire({ session, tabStore: fakeStore(), sharedStore: fakeStore(), clientId: "c1",
                              nowMs: () => t, nowUnix: () => 1000, newId: () => `id-${++n}` });
  const h = {
    w, session,
    welcome(epoch = 3) {
      session.welcome = { t: "welcome", vehicle_id: "V1", session_epoch: epoch, capabilities: caps };
      w.onOpen();
      w.onWelcome(session.welcome);
    },
    state(st) { session.state = st; w.onState(st); },
    wait(ms) { t += ms; },
    run(ms, st = session.state) { for (let k = 0; k < ms; k += 250) { t += 250; h.state(st); w.tick(); } },
    sent: (type) => session.sent.filter((m) => m.t === type),
    async answer(msg, extra) {
      session.answers.get(msg.request_id)?.({ request_id: msg.request_id, ...extra });
      await Promise.resolve();
      await Promise.resolve();
    },
    pick(req, pallet, dest, over = {}) {
      w.onPick({ t: "pick", request_id: req.request_id, targets_revision: 7,
                 pick: pallet ? { pallet_id: pallet, destination_id: dest } : null,
                 reason: pallet ? "ok" : "none_remaining", plan: pallet ? [[pallet, dest]] : [], ...over });
    },
    result(id, outcome = "done") { w.onResults([{ seq: 1, mission_id: id, outcome }]); },
  };
  return h;
}

async function startAuto(h) {
  h.welcome();
  h.state(READY);
  assert.equal(h.w.addAuto(), true);
  assert.equal(h.w.resume(), true);
}

test("all remaining pallets: two picks moved, then none remaining -- three asks and exactly two submits", async () => {
  const h = harness();
  await startAuto(h);
  for (const [i, [p, d]] of [["P-1", "D-1"], ["P-2", "D-2"]].entries()) {
    h.run(250, READY);
    const ask = h.sent("pick_request").at(-1);
    await h.answer(ask, { t: "ack" });
    h.pick(ask, p, d);
    h.run(250, READY);
    const s = h.sent("mission_submit").at(-1);
    assert.deepEqual([s.pallet_id, s.destination_id, s.targets_revision], [p, d, 7]);
    await h.answer(s, { t: "ack", mission_id: `M-${i + 1}` });
    h.run(1000, RUNNING(`M-${i + 1}`));
    h.result(`M-${i + 1}`);
    h.run(1250, READY); // the settle time
  }
  const last = h.sent("pick_request").at(-1);
  await h.answer(last, { t: "ack" });
  h.pick(last, null, null);
  h.run(3000, READY);
  assert.equal(h.sent("pick_request").length, 3);
  assert.equal(h.sent("mission_submit").length, 2);
  assert.equal(h.w.queue.run, "stopped");
  assert.equal(h.w.queue.stopReason, "완료");
  assert.deepEqual(h.w.queue.finished.map((f) => [f.pallet_id, f.destination_id]), [["P-2", "D-2"], ["P-1", "D-1"]]);
});

test("the vehicle's deadline comes from its welcome", async () => {
  const h = harness({ ...CAPS, next_pick_deadline_ms: 1500 });
  await startAuto(h);
  assert.equal(h.w.queue.pickDeadlineMs, 1500);
  h.run(250, READY);
  h.run(1500, READY);
  assert.equal(h.w.queue.run, "running", "1.5 s exactly is not over");
  h.run(250, READY);
  assert.equal(h.w.queue.stopReason, "자동 선택 응답 없음");
  assert.deepEqual(h.session.forgotten, [h.sent("pick_request")[0].request_id]);
  const d = harness(CAPS);
  await startAuto(d);
  assert.equal(d.w.queue.pickDeadlineMs, 5000, "5000 when the vehicle does not say");
});

test("a vehicle without next_pick: no auto item, and nothing goes out", async () => {
  const h = harness({ control: true, mission: true });
  h.welcome();
  h.state(READY);
  assert.equal(h.w.canAuto, false);
  assert.equal(h.w.addAuto(), false);
  assert.equal(h.sent("pick_request").length, 0);
});

test("removing the auto item while it waits for its pick forgets the request once", async () => {
  const h = harness();
  await startAuto(h);
  h.run(250, READY);
  const ask = h.sent("pick_request")[0];
  h.w.remove(h.w.queue.items[0].item_id);
  assert.deepEqual(h.session.forgotten, [ask.request_id]);
  h.pick(ask, "P-1", "D-1"); // late: ignored
  h.run(2000, READY);
  assert.equal(h.sent("mission_submit").length, 0);
});

test("a disconnect while picking forgets it and nothing is uncertain; a refusal stops with nothing sent after", async () => {
  const h = harness();
  await startAuto(h);
  h.run(250, READY);
  const ask = h.sent("pick_request")[0];
  h.w.onDisconnect();
  assert.deepEqual(h.session.forgotten, [ask.request_id]);
  assert.ok(!h.w.queue.items[0].uncertain);
  const r = harness();
  await startAuto(r);
  r.run(250, READY);
  await r.answer(r.sent("pick_request")[0], { t: "reject", reason: "unsupported" });
  r.run(3000, READY);
  assert.equal(r.w.queue.stopReason, "자동 선택 거부: unsupported");
  assert.equal(r.sent("pick_request").length, 1);
  assert.equal(r.sent("mission_submit").length, 0);
});

test("an ack past the deadline stops through the wiring; none remaining waits for its ack (Codex CT-2 r2)", async () => {
  const h = harness();
  await startAuto(h);
  h.run(250, READY);
  const ask = h.sent("pick_request").at(-1);
  h.pick(ask, "P-1", "D-1");
  h.run(5000, READY); // the tick at the boundary itself is still in time
  h.wait(100); // the ack then comes before the next tick
  await h.answer(ask, { t: "ack" });
  h.run(500, READY);
  assert.equal(h.w.queue.stopReason, "자동 선택 응답 없음");
  assert.equal(h.sent("mission_submit").length, 0);

  const g = harness();
  await startAuto(g);
  assert.equal(g.w.add({ pallet_id: "P-2", destination_id: "D-1" }), true);
  g.run(250, READY);
  const req = g.sent("pick_request").at(-1);
  g.pick(req, null, null);
  g.run(500, READY);
  await g.answer(req, { t: "reject", reason: "busy" });
  g.run(500, READY);
  assert.equal(g.w.queue.stopReason, "자동 선택 거부: busy");
  assert.equal(g.sent("mission_submit").length, 0, "the fixed item behind it is not submitted");
});
