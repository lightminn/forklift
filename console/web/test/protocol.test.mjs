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
