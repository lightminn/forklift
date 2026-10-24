import { test } from "node:test";
import assert from "node:assert/strict";
import { createQueueWire } from "../js/queuewire.js";

// Queue plan D5's one exception through the wiring: the session's first mission_submit refused not_stopped goes out
// once more -- the same pallet and destination, a new request id -- after a 1 s stand; a second refusal stops.

const TARGETS = { targets_revision: 7, pallets: [{ pallet_id: "P-1", state: "available" }], destinations: [{ destination_id: "D-1" }] };
const READY = { mode: "PAUSED", stop_kind: "controlled", stopped_confirmed: true, feedback_ok: true, mission: null };
const SETTLING = { ...READY, stopped_confirmed: false };

function fakeStore() {
  const data = new Map();
  return { getItem: (k) => (data.has(k) ? data.get(k) : null), setItem: (k, v) => data.set(k, v), removeItem: (k) => data.delete(k) };
}

function harness() {
  let t = 10_000, n = 0;
  const session = {
    sent: [], answers: new Map(), state: null, targets: TARGETS, welcome: null, hasLease: true,
    request(type, fields, opts) {
      session.sent.push({ t: type, request_id: opts.requestId, ...fields });
      return new Promise((resolve) => session.answers.set(opts.requestId, resolve));
    },
    forget(id) { session.answers.delete(id); },
  };
  const w = createQueueWire({ session, tabStore: fakeStore(), sharedStore: fakeStore(), clientId: "c1",
                              nowMs: () => t, nowUnix: () => 1000, newId: () => `id-${++n}` });
  const h = {
    w, session,
    start() {
      session.welcome = { t: "welcome", vehicle_id: "V1", session_epoch: 3, capabilities: { control: true, mission: true } };
      w.onOpen();
      w.onWelcome(session.welcome);
      h.state(READY);
      assert.equal(w.add({ pallet_id: "P-1", destination_id: "D-1" }), true);
      assert.equal(w.resume(), true);
    },
    state(st) { session.state = st; w.onState(st); },
    run(ms, st) { for (let k = 0; k < ms; k += 100) { t += 100; h.state(st); w.tick(); } },
    submits: () => session.sent.filter((m) => m.t === "mission_submit"),
    async refuse(msg, reason = "not_stopped") {
      session.answers.get(msg.request_id)?.({ t: "reject", request_id: msg.request_id, reason });
      await Promise.resolve();
      await Promise.resolve();
    },
  };
  return h;
}

test("the settling truck's refusal is followed by one resubmit after the stand, and a second refusal stops", async () => {
  const h = harness();
  h.start();
  h.run(100, READY);
  assert.equal(h.submits().length, 1);
  await h.refuse(h.submits()[0]);
  h.run(500, SETTLING);
  h.run(900, READY);
  assert.equal(h.submits().length, 1, "not before 1 s of standing");
  h.run(300, READY);
  assert.equal(h.submits().length, 2);
  const [a, b] = h.submits();
  assert.deepEqual([b.pallet_id, b.destination_id], [a.pallet_id, a.destination_id]);
  assert.notEqual(b.request_id, a.request_id);
  await h.refuse(b);
  h.run(3000, READY);
  assert.equal(h.submits().length, 2);
  assert.equal(h.w.queue.stopReason, "거부: not_stopped");
});
