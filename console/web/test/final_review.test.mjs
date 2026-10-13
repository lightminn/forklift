import { test } from "node:test";
import assert from "node:assert/strict";
import { createSession } from "../js/protocol.js";
import { chooseClientId, createTeleopGate } from "../js/gate.js";

function setup() {
  const sent = [];
  let t = 0, n = 0;
  const s = createSession({ clientId: "a", send: (m) => sent.push(m), now: () => t, newId: () => `id${++n}` });
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {} });
  s.onMessage({ t: "state", state_seq: 1, session_epoch: 3, control_gen: 7, arm_gen: 0, armed: false, mode: "AUTO" });
  return { s, sent, tick: (dt) => { t += dt; } };
}

test("STOP is resent with the same id until acknowledged", async () => {
  const { s, sent, tick } = setup();
  const p = s.stop("console");
  const id = sent.at(-1).request_id;
  tick(600); s.pump();
  assert.equal(sent.filter((m) => m.t === "stop" && m.request_id === id).length, 2);
  s.onMessage({ t: "ack", request_id: id });
  await p;
  tick(600); s.pump();
  assert.equal(sent.filter((m) => m.t === "stop" && m.request_id === id).length, 2);
});

test("a lost connection drops the old lease and the next welcome reclaims it", async () => {
  const { s, sent } = setup();
  const p = s.request("lease_request", {});
  s.onMessage({ t: "ack", request_id: sent.at(-1).request_id, lease_id: "L1" });
  await p;
  s.onDisconnect();
  assert.equal(s.hasLease, false);
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {} });
  assert.equal(sent.at(-1).t, "lease_request");
});

test("teleop needs this tab's lease, not just an armed vehicle", () => {
  const { s } = setup();
  s.onMessage({ t: "state", state_seq: 2, session_epoch: 3, control_gen: 7, arm_gen: 4, armed: true, mode: "TELEOP" });
  s.onMessage({ t: "tick", session_epoch: 3, state_seq: 9, token: "k9" });
  assert.equal(s.teleopCommand({ mode: "direct", deadman: true, speedRatio: 0.5, curvatureRatio: 0, liftDir: 0 }), null);
});

test("direct-view confirmation is cleared only by the explicit confirmation", () => {
  const g = createTeleopGate();
  g.setMode("video");
  g.onArmed(true);
  g.onArmed(false); // any disarm in video mode: the vehicle may have expired the video
  g.setMode("direct");
  assert.equal(g.canSend(), false);
  g.setMode("video");
  g.setMode("direct");
  assert.equal(g.canSend(), false);
  g.confirmDirect();
  assert.equal(g.canSend(), true);
});

test("a copied tab gets its own client id; a reload keeps it", () => {
  const ids = ["fresh-1", "fresh-2"];
  const first = chooseClientId({ stored: null, windowName: "", newId: () => ids.shift() });
  assert.equal(first.id, "fresh-1");
  const reload = chooseClientId({ stored: "fresh-1", windowName: first.windowName, newId: () => "never" });
  assert.equal(reload.id, "fresh-1");
  const copied = chooseClientId({ stored: "fresh-1", windowName: "", newId: () => ids.shift() });
  assert.equal(copied.id, "fresh-2");
});

test("any disarm in video mode requires the direct-view confirmation", () => {
  const g = createTeleopGate();
  g.setMode("video");
  g.onArmed(true);
  g.onArmed(false);
  g.setMode("direct");
  assert.equal(g.canSend(), false);
});

test("a reload that held the lease reclaims it on the first welcome", () => {
  const sent = [];
  let n = 0;
  const s = createSession({ clientId: "a", send: (m) => sent.push(m), now: () => 0, newId: () => `id${++n}` });
  s.expectReclaim();
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {} });
  assert.equal(sent.at(-1).t, "lease_request");
});

test("a lease request unanswered before a disconnect is replaced, not resent", async () => {
  const { s, sent } = setup();
  const p = s.request("lease_request", {});
  const oldId = sent.at(-1).request_id;
  s.onDisconnect();
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {} });
  const r = await p;
  assert.equal(r.t, "reject");
  const fresh = sent.filter((m) => m.t === "lease_request");
  assert.equal(fresh.length, 2);
  assert.notEqual(fresh.at(-1).request_id, oldId);
});

test("switching to direct before the disarm arrives still needs the confirmation", () => {
  const g = createTeleopGate();
  g.setMode("video");
  g.onArmed(true);
  g.setMode("direct"); // pressed before the disarm state arrived
  g.onArmed(false);
  assert.equal(g.canSend(), false);
});

test("requests never carry null generations", () => {
  const sent = [];
  let n = 0;
  const s = createSession({ clientId: "a", send: (m) => sent.push(m), now: () => 0, newId: () => `id${++n}` });
  s.expectReclaim();
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {} });
  const msg = sent.at(-1);
  assert.equal(msg.t, "lease_request");
  for (const k of ["lease_id", "control_gen"]) assert.ok(!(k in msg) || Number.isInteger(msg[k]) || typeof msg[k] === "string", k);
  assert.ok(!Object.values(msg).includes(null));
});
