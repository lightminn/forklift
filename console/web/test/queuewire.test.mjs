import { test } from "node:test";
import assert from "node:assert/strict";
import { createQueueWire, presetsKey, queueKey } from "../js/queuewire.js";

// ---- a fake session and fake storage: the tests count what goes out ---------------------------------------------
const TARGETS = {
  targets_revision: 7,
  pallets: [{ pallet_id: "P-1", state: "available" }, { pallet_id: "P-2", state: "available" },
            { pallet_id: "P-3", state: "available" }],
  destinations: [{ destination_id: "D-0" }, { destination_id: "D-1" }],
};
const READY = { mode: "PAUSED", stop_kind: "controlled", stopped_confirmed: true, feedback_ok: true, mission: null };
const RUNNING = (id) => ({ ...READY, mode: "AUTO", stop_kind: "none", mission: { mission_id: id, status: "running" } });

function fakeStore({ failSet = false } = {}) {
  const data = new Map();
  const log = [];
  return {
    data, log,
    getItem: (k) => (data.has(k) ? data.get(k) : null),
    setItem: (k, v) => { log.push(["set", k]); if (failSet) throw new Error("quota"); data.set(k, v); },
    removeItem: (k) => { log.push(["remove", k]); data.delete(k); },
  };
}

function fakeSession(order) {
  const s = {
    sent: [], forgotten: [], answers: new Map(),
    state: null, targets: TARGETS, welcome: null, hasLease: true,
    request(type, fields, opts) {
      order?.push("send");
      const msg = { t: type, request_id: opts.requestId, ...fields };
      s.sent.push(msg);
      return new Promise((resolve) => s.answers.set(opts.requestId, resolve));
    },
    forget(id) { s.forgotten.push(id); s.answers.delete(id); },
  };
  return s;
}

function harness({ tabStore = fakeStore(), sharedStore = fakeStore(), order } = {}) {
  let t = 10_000, n = 0;
  const session = fakeSession(order);
  const w = createQueueWire({ session, tabStore, sharedStore, clientId: "c1", nowMs: () => t, nowUnix: () => 1000,
                              newId: () => `id-${++n}` });
  const h = {
    w, session, tabStore, sharedStore,
    at: () => t,
    welcome(epoch = 3) {
      session.welcome = { t: "welcome", vehicle_id: "V1", session_epoch: epoch };
      w.onOpen();
      w.onWelcome(session.welcome);
    },
    state(st) { session.state = st; w.onState(st); },
    // time passes with a state every 250 ms and the 250 ms timer firing
    run(ms, st = session.state) {
      for (let k = 0; k < ms; k += 250) { t += 250; h.state(st); w.tick(); }
    },
    submits: () => session.sent.filter((m) => m.t === "mission_submit"),
    async answer(msg, extra) {
      session.answers.get(msg.request_id)?.({ request_id: msg.request_id, ...extra });
      await Promise.resolve();
      await Promise.resolve();
    },
    result(id, outcome = "done", reason = null) {
      w.onResults([{ seq: 1, mission_id: id, outcome, reason }]);
    },
  };
  return h;
}

async function ready(h, items = [["P-1", "D-0"]]) {
  h.welcome();
  h.state(READY);
  for (const [p, d] of items) h.w.add({ pallet_id: p, destination_id: d });
  assert.equal(h.w.resume(), true);
}

// one mission from submit to result; returns the submit
async function cycle(h, id, outcome = "done") {
  h.run(250, READY);
  const s = h.submits().at(-1);
  await h.answer(s, { t: "ack", mission_id: id });
  h.run(2000, RUNNING(id));
  h.result(id, outcome);
  h.run(250, READY);
  return s;
}

// ---- tests ------------------------------------------------------------------------------------------------------
test("three queued transports go out as exactly three submits, one after another", async () => {
  const h = harness();
  await ready(h, [["P-1", "D-0"], ["P-2", "D-1"], ["P-3", "D-0"]]);
  await cycle(h, "M-1");
  h.run(1000, READY); // the settle time after a result
  await cycle(h, "M-2");
  h.run(1000, READY);
  await cycle(h, "M-3");
  h.run(5000, READY);
  assert.deepEqual(h.submits().map((m) => [m.pallet_id, m.destination_id, m.targets_revision]),
                   [["P-1", "D-0", 7], ["P-2", "D-1", 7], ["P-3", "D-0", 7]]);
  assert.equal(h.w.queue.run, "stopped");
  assert.equal(h.w.queue.stopReason, "완료");
  assert.deepEqual(h.w.queue.counts, { done: 3, failed: 0, cancelled: 0 });
});

test("a failed mission stops the queue and nothing more is submitted", async () => {
  const h = harness();
  await ready(h, [["P-1", "D-0"], ["P-2", "D-1"]]);
  await cycle(h, "M-1", "failed");
  h.run(10_000, READY);
  assert.equal(h.submits().length, 1);
  assert.equal(h.w.queue.run, "stopped");
});

test("each anomaly while a mission runs: the mission is left alone and no next submit goes out", async () => {
  const anomalies = {
    disconnect: (h) => h.w.onDisconnect(),
    lease: (h) => { h.session.hasLease = false; h.state(RUNNING("M-1")); },
    cancel: (h) => h.w.operatorAction("cancel"),
    stop: (h) => h.w.operatorAction("stop"),
    preempt: (h) => h.state({ ...RUNNING("M-1"), stop_kind: "safety_preempt" }),
    feedback: (h) => h.state({ ...RUNNING("M-1"), feedback_ok: false }),
  };
  for (const [name, hit] of Object.entries(anomalies)) {
    const h = harness();
    await ready(h, [["P-1", "D-0"], ["P-2", "D-1"]]);
    h.run(250, READY);
    await h.answer(h.submits()[0], { t: "ack", mission_id: "M-1" });
    h.run(1000, RUNNING("M-1"));
    hit(h);
    assert.equal(h.w.queue.run, "stopped", name);
    // the vehicle comes back, finishes the mission and stands ready again
    if (name === "disconnect") h.welcome();
    h.session.hasLease = true;
    h.result("M-1", "done");
    h.run(10_000, READY);
    assert.equal(h.submits().length, 1, name);
    assert.equal(h.session.sent.filter((m) => m.t !== "mission_submit").length, 0, `${name}: no cancel or pause sent`);
  }
});

test("a disconnect while the submit is unanswered forgets it once and marks the item uncertain", async () => {
  const h = harness();
  await ready(h, [["P-1", "D-0"], ["P-2", "D-1"]]);
  h.run(250, READY);
  const s = h.submits()[0];
  h.w.onDisconnect();
  assert.deepEqual(h.session.forgotten, [s.request_id]);
  assert.equal(h.w.queue.items[0].uncertain, true);
  h.welcome();
  assert.equal(h.w.resume(), false, "an uncertain front item blocks resume");
  h.run(5000, READY);
  assert.equal(h.submits().length, 1);
  assert.deepEqual(h.session.forgotten, [s.request_id]);
});

test("a reopened connection submits nothing before its own welcome", async () => {
  const h = harness();
  h.welcome();
  h.state(READY);
  h.w.add({ pallet_id: "P-1", destination_id: "D-0" });
  h.w.onDisconnect();
  h.w.onOpen(); // the socket is open again; the session still holds the old welcome and state
  assert.equal(h.w.resume(), true);
  h.w.tick();
  assert.equal(h.submits().length, 0);
  h.welcome();
  h.run(250, READY);
  assert.equal(h.submits().length, 1);
});

test("a result that comes before the ack is applied once the ack names it", async () => {
  const h = harness();
  await ready(h, [["P-1", "D-0"], ["P-2", "D-1"]]);
  h.run(250, READY);
  h.result("M-1", "done");
  await h.answer(h.submits()[0], { t: "ack", mission_id: "M-1" });
  assert.equal(h.w.queue.items.length, 1);
  h.run(2000, READY);
  assert.equal(h.submits().length, 2);
  assert.equal(h.submits()[1].pallet_id, "P-2");
});

test("a reject stops the queue; busy leaves the item uncertain; nothing is resubmitted", async () => {
  for (const [reason, unsure] of [["destination_occupied", false], ["busy", true]]) {
    const h = harness();
    await ready(h, [["P-1", "D-0"]]);
    h.run(250, READY);
    await h.answer(h.submits()[0], { t: "reject", reason });
    h.run(5000, READY);
    assert.equal(h.submits().length, 1, reason);
    assert.equal(h.w.queue.run, "stopped", reason);
    assert.equal(Boolean(h.w.queue.items[0].uncertain), unsure, reason);
  }
});

test("the submit is stored before it is sent, under the tab's own key", async () => {
  const order = [];
  const tabStore = fakeStore();
  const realSet = tabStore.setItem;
  tabStore.setItem = (k, v) => { if (JSON.parse(v).phase === "submitting") order.push("save"); realSet(k, v); };
  const h = harness({ tabStore, order });
  await ready(h);
  h.run(250, READY);
  assert.deepEqual(order.slice(0, 2), ["save", "send"]);
  assert.equal(queueKey("V1", "c1"), "console.queue.V1.c1");
  const stored = JSON.parse(tabStore.data.get("console.queue.V1.c1"));
  assert.equal(stored.current.request_id, h.submits()[0].request_id);
});

test("a queue that cannot be stored says so and leaves no older copy behind", async () => {
  const tabStore = fakeStore({ failSet: true });
  const h = harness({ tabStore });
  await ready(h);
  assert.match(h.w.saveNote, /저장 안 됨/);
  assert.ok(tabStore.log.some(([op, k]) => op === "remove" && k === "console.queue.V1.c1"));
});

test("a reopened page restores the queue stopped; a submit in flight comes back uncertain", async () => {
  const tabStore = fakeStore();
  const a = harness({ tabStore });
  await ready(a, [["P-1", "D-0"], ["P-2", "D-1"]]);
  a.run(250, READY);
  const b = harness({ tabStore });
  b.welcome();
  b.state(READY);
  assert.equal(b.w.queue.run, "stopped");
  assert.equal(b.w.queue.stopReason, "콘솔 다시 열림");
  assert.equal(b.w.queue.items.length, 2);
  assert.equal(b.w.queue.items[0].uncertain, true);
  b.run(5000, READY);
  assert.equal(b.submits().length, 0);
});

test("a reopened page settles a mission that was running when the result arrives in the same session", async () => {
  const tabStore = fakeStore();
  const a = harness({ tabStore });
  await ready(a, [["P-1", "D-0"], ["P-2", "D-1"]]);
  a.run(250, READY);
  await a.answer(a.submits()[0], { t: "ack", mission_id: "M-1" });
  const b = harness({ tabStore });
  b.welcome();
  assert.equal(b.w.queue.items[0].uncertain, true);
  b.result("M-1", "done");
  assert.equal(b.w.queue.items.length, 1);
  assert.equal(b.w.queue.items[0].pallet_id, "P-2");
  assert.equal(b.w.queue.run, "stopped", "still waits for the operator");
  b.run(5000, READY);
  assert.equal(b.submits().length, 0);
});

test("the state's last_result finishes the mission like the results book does", async () => {
  const h = harness();
  await ready(h, [["P-1", "D-0"], ["P-2", "D-1"]]);
  h.run(250, READY);
  await h.answer(h.submits()[0], { t: "ack", mission_id: "M-1" });
  h.run(1000, RUNNING("M-1"));
  h.run(2000, { ...READY, last_result: { mission_id: "M-1", outcome: "done" } });
  assert.equal(h.submits().length, 2);
});

test("saved transports live in localStorage per vehicle and can be queued when usable", async () => {
  const sharedStore = fakeStore();
  const h = harness({ sharedStore });
  h.welcome();
  h.state(READY);
  const p = h.w.savePreset({ pallet_id: "P-1", destination_id: "D-1" });
  assert.equal(p.label, "P-1 → D-1");
  assert.equal(presetsKey("V1"), "console.presets.V1");
  assert.equal(JSON.parse(sharedStore.data.get("console.presets.V1")).length, 1);
  assert.equal(h.w.queuePreset(p.preset_id), true);
  assert.equal(h.w.queue.items[0].pallet_id, "P-1");
  const other = harness({ sharedStore });
  other.welcome();
  assert.equal(other.w.presets.length, 1, "another tab of the same vehicle sees it");
  const gone = h.w.savePreset({ pallet_id: "P-9", destination_id: "D-1" });
  assert.equal(h.w.queuePreset(gone.preset_id), false, "a pallet not in the list cannot be queued");
  h.w.removePreset(p.preset_id);
  assert.equal(JSON.parse(sharedStore.data.get("console.presets.V1")).length, 1);
});
