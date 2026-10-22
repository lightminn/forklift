import { test } from "node:test";
import assert from "node:assert/strict";
import { createSession } from "../js/protocol.js";

function setup() {
  const sent = [];
  let t = 0, n = 0;
  const s = createSession({ clientId: "a", send: (m) => sent.push(m), now: () => t, newId: () => `id${++n}` });
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {} });
  s.onMessage({ t: "state", state_seq: 1, session_epoch: 3, control_gen: 7, arm_gen: 0, armed: false, mode: "PAUSED" });
  return { s, sent, tick: (dt) => { t += dt; } };
}

test("requests carry current generations", async () => {
  const { s, sent } = setup();
  const p = s.request("lease_request", {});
  assert.equal(sent.at(-1).session_epoch, 3);
  s.onMessage({ t: "ack", request_id: sent.at(-1).request_id, lease_id: "L1" });
  await p;
  s.request("arm", {});
  assert.equal(sent.at(-1).lease_id, "L1");
  assert.equal(sent.at(-1).control_gen, 7);
});

test("stale reject is not auto-resent", async () => {
  const { s, sent } = setup();
  const p = s.request("mission_resume", { mission_id: "M-1" });
  const id = sent.at(-1).request_id;
  s.onMessage({ t: "reject", request_id: id, reason: "stale_control" });
  const r = await p;
  assert.equal(r.reason, "stale_control");
  const count = sent.filter((m) => m.request_id === id).length;
  assert.equal(count, 1);
});

test("unanswered request is resent with the same id", async () => {
  const { s, sent, tick } = setup();
  s.request("mission_pause", { mission_id: "M-1" });
  const id = sent.at(-1).request_id;
  tick(600);
  s.pump();
  assert.equal(sent.filter((m) => m.request_id === id).length, 2);
});

function withLease(s, sent) {
  s.request("lease_request", {});
  s.onMessage({ t: "ack", request_id: sent.at(-1).request_id, lease_id: "L1" });
}

test("teleop command needs arm and a received tick, echoes the latest token", () => {
  const { s, sent } = setup();
  withLease(s, sent);
  assert.equal(s.teleopCommand({ mode: "direct", deadman: true, speedRatio: 0.5, curvatureRatio: 0, liftDir: 0 }), null);
  s.onMessage({ t: "state", state_seq: 2, session_epoch: 3, control_gen: 7, arm_gen: 4, armed: true, mode: "TELEOP" });
  assert.equal(s.teleopCommand({ mode: "direct", deadman: true, speedRatio: 0.5, curvatureRatio: 0, liftDir: 0 }), null);
  s.onMessage({ t: "tick", session_epoch: 3, state_seq: 9, token: "k9" });
  const c = s.teleopCommand({ mode: "video", deadman: true, speedRatio: 0.5, curvatureRatio: 0, liftDir: 0, frameId: 12 });
  assert.deepEqual(c.echo, { state_seq: 9, token: "k9" });
  assert.equal(c.arm_gen, 4);
  assert.equal(c.seq, 1);
  assert.deepEqual(c.frame_echo, { frame_id: 12 });
});

test("seq restarts when arm_gen changes", () => {
  const { s, sent } = setup();
  withLease(s, sent);
  s.onMessage({ t: "state", state_seq: 2, session_epoch: 3, control_gen: 7, arm_gen: 4, armed: true, mode: "TELEOP" });
  s.onMessage({ t: "tick", session_epoch: 3, state_seq: 9, token: "k9" });
  const args = { mode: "direct", deadman: true, speedRatio: 0.1, curvatureRatio: 0, liftDir: 0 };
  s.teleopCommand(args); s.teleopCommand(args);
  s.onMessage({ t: "state", state_seq: 3, session_epoch: 3, control_gen: 8, arm_gen: 5, armed: true, mode: "TELEOP" });
  assert.equal(s.teleopCommand(args).seq, 1);
});

test("a new session_epoch drops lease and tick", () => {
  const { s } = setup();
  s.onMessage({ t: "tick", session_epoch: 3, state_seq: 9, token: "k9" });
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 4, capabilities: {} });
  assert.equal(s.hasLease, false);
  assert.equal(s.echoMessage(), null);
});

test("a state without a plan keeps the last plan; plan null clears it", () => {
  const { s } = setup();
  const plan = { revision: 1, poses: [[0, 0, 1], [1, 0, 1]] };
  s.onMessage({ t: "state", state_seq: 2, session_epoch: 3, control_gen: 7, arm_gen: 0, armed: false, mode: "AUTO", plan });
  s.onMessage({ t: "state", state_seq: 3, session_epoch: 3, control_gen: 7, arm_gen: 0, armed: false, mode: "AUTO" });
  assert.deepEqual(s.state.plan, plan);
  s.onMessage({ t: "state", state_seq: 4, session_epoch: 3, control_gen: 7, arm_gen: 0, armed: false, mode: "AUTO", plan: null });
  assert.equal(s.state.plan, null);
});

// persistent console plan D7 (stage 4c): the request ledger's token and seq
function tokened() {
  const sent = [];
  let t = 0, n = 0, reconnects = 0;
  const s = createSession({ clientId: "a", send: (m) => sent.push(m), now: () => t, newId: () => `id${++n}`,
    onStaleToken: () => { reconnects += 1; } });
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {}, request_token: 41 });
  return { s, sent, tick: (dt) => { t += dt; }, reconnects: () => reconnects };
}

test("reliable requests carry the connection's token and a growing seq; a resend is the same message", () => {
  const { s, sent, tick } = tokened();
  s.request("lease_request", {});
  s.request("rearm", {});
  assert.deepEqual(sent.map((m) => [m.request_token, m.request_seq]), [[41, 1], [41, 2]]);
  tick(600);
  s.pump();
  assert.deepEqual(sent.slice(2).map((m) => [m.request_token, m.request_seq]), [[41, 1], [41, 2]]);
  s.stop("key");
  assert.equal(sent.at(-1).request_token, undefined); // STOP runs without one
});

test("a new connection's token starts its own seq", () => {
  const { s, sent } = tokened();
  s.request("rearm", {});
  s.onDisconnect();
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {}, request_token: 42 });
  s.request("rearm", {});
  assert.deepEqual([sent.at(-1).request_token, sent.at(-1).request_seq], [42, 1]);
});

test("stale on an old token ends that request only; stale on the current one asks for a new connection", async () => {
  const { s, sent, reconnects } = tokened();
  const old = s.request("rearm", {});
  const oldId = sent.at(-1).request_id;
  s.onDisconnect();
  s.onMessage({ t: "welcome", api: "console/1", session_epoch: 3, capabilities: {}, request_token: 42 });
  s.onMessage({ t: "reject", request_id: oldId, reason: "stale" });
  assert.equal((await old).reason, "stale");
  assert.equal(reconnects(), 0);
  const cur = s.request("rearm", {});
  s.onMessage({ t: "reject", request_id: sent.at(-1).request_id, reason: "stale" });
  assert.equal((await cur).reason, "stale");
  assert.equal(reconnects(), 1);
});

test("without a token (the fake vehicle) requests are as before", () => {
  const { s, sent } = setup();
  s.request("rearm", {});
  assert.equal(sent.at(-1).request_token, undefined);
  assert.equal(sent.at(-1).request_seq, undefined);
});

test("a request can carry an id chosen by the caller (the mission queue saves it before sending)", () => {
  const { s, sent } = setup();
  s.request("mission_submit", { pallet_id: "P-1" }, { requestId: "q-1" });
  assert.equal(sent.at(-1).request_id, "q-1");
  assert.equal(sent.at(-1).pallet_id, "P-1");
});

test("a forgotten request is not resent and its late answer settles nothing", async () => {
  const { s, sent, tick } = setup();
  let settled = false;
  s.request("mission_submit", { pallet_id: "P-1" }, { requestId: "q-1" }).then(() => { settled = true; });
  const other = s.request("mission_pause", { mission_id: "M-1" });
  const otherId = sent.at(-1).request_id;
  s.forget("q-1");
  tick(600);
  s.pump();
  assert.equal(sent.filter((m) => m.request_id === "q-1").length, 1, "sent once, never again");
  assert.equal(sent.filter((m) => m.request_id === otherId).length, 2, "the other request is still resent");
  s.onMessage({ t: "ack", request_id: "q-1", mission_id: "M-7" });
  await Promise.resolve();
  assert.equal(settled, false);
  s.onMessage({ t: "ack", request_id: otherId });
  assert.equal((await other).t, "ack");
});
