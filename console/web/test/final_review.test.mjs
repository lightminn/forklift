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
  g.onArmed(true, 0.0);
  g.onArmed(false, 0.9); // disarmed while video was stale
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
