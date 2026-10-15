import { test } from "node:test";
import assert from "node:assert/strict";
import { blockReason, currentDestination, missionProgress, nextAllowed, subtaskOf, templatesFor } from "../js/wizard.js";

test("only templates the vehicle supports are offered", () => {
  assert.deepEqual(templatesFor({ mission: false }), []);
  assert.deepEqual(templatesFor(null), []);
  const t = templatesFor({ mission: true });
  assert.equal(t.length, 1);
  assert.equal(t[0].code, "PALLET_TRANSPORT");
  assert.equal(t[0].subtasks.length, 4);
});

test("template search matches the Korean name, the code and the subtasks", () => {
  const caps = { mission: true };
  assert.equal(templatesFor(caps, "운반").length, 1);
  assert.equal(templatesFor(caps, "pallet_tr").length, 1);
  assert.equal(templatesFor(caps, "하역").length, 1);
  assert.equal(templatesFor(caps, "충전").length, 0);
});

test("each wizard step needs its own input before moving on", () => {
  assert.equal(nextAllowed(1, { template: null }), false);
  assert.equal(nextAllowed(1, { template: "PALLET_TRANSPORT" }), true);
  assert.equal(nextAllowed(2, { template: "PALLET_TRANSPORT", pallet: null, destination: "D-A" }), false);
  assert.equal(nextAllowed(2, { template: "PALLET_TRANSPORT", pallet: "P-01", destination: "" }), false);
  assert.equal(nextAllowed(2, { template: "PALLET_TRANSPORT", pallet: "P-01", destination: "D-A" }), true);
});

test("the start is blocked with a reason the operator can act on", () => {
  const base = { hasLease: true, mode: "PAUSED", mission: null };
  assert.equal(blockReason(base), null);
  assert.match(blockReason({ ...base, hasLease: false }), /조종 권한/);
  assert.match(blockReason({ ...base, mode: "STOPPED" }), /재무장/);
  assert.match(blockReason({ ...base, mode: "TELEOP" }), /수동 조종/);
  assert.match(blockReason({ ...base, mission: { status: "held" } }), /진행 중/);
  assert.equal(blockReason({ ...base, mission: { status: "done" } }), null);
  assert.match(blockReason({ ...base, state: null }), /연결/);
});

test("phases map onto the four subtasks", () => {
  assert.equal(subtaskOf("observe"), 0);
  assert.equal(subtaskOf("approach"), 0);
  assert.equal(subtaskOf("insert"), 1);
  assert.equal(subtaskOf("extract"), 1);
  assert.equal(subtaskOf("transport"), 2);
  assert.equal(subtaskOf("withdraw"), 3);
  assert.equal(subtaskOf("return"), 4);
  assert.equal(subtaskOf("unknown"), -1);
});

test("the current destination follows the subtask", () => {
  const mi = { pallet_id: "P-01", destination_id: "D-B", status: "running" };
  assert.match(currentDestination({ ...mi, phase: "approach" }), /P-01/);
  assert.match(currentDestination({ ...mi, phase: "transport" }), /D-B/);
  assert.match(currentDestination({ ...mi, phase: "return" }), /복귀/);
  assert.equal(currentDestination({ ...mi, phase: "transport", status: "done" }), null);
  assert.equal(currentDestination(null), null);
});

test("progress counts finished phases and is full only when done", () => {
  assert.equal(missionProgress(null), 0);
  assert.equal(missionProgress({ phase: "observe", status: "running" }), 0);
  assert.ok(Math.abs(missionProgress({ phase: "transport", status: "running" }) - 5 / 9) < 1e-12);
  assert.equal(missionProgress({ phase: "return", status: "done" }), 1);
});

test("a connection that takes no tasks says so before anything else about the vehicle state", () => {
  const base = { hasLease: true, mode: "PAUSED", mission: null };
  assert.match(blockReason({ ...base, canMission: false }), /작업 지시를 받지 않습니다/);
  assert.match(blockReason({ ...base, hasLease: false, canMission: false }), /작업 지시를 받지 않습니다/);
  assert.equal(blockReason({ ...base, canMission: true }), null);
});
