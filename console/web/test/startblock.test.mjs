import { test } from "node:test";
import assert from "node:assert/strict";
import { startBand } from "../js/posecheck.js";
import { blockReason } from "../js/wizard.js";
import { resumeState } from "../js/profile.js";

// docs/plans/2026-11-08-f1-vehicle-block.md 2.4: an operator-placed start latched on the vehicle (state start_block)

const BLOCK = { reason: "operator_placed_start", source: "operator_placed", since_mono: 1.0 };

test("no field shows nothing", () => {
  assert.equal(startBand({}), null);
  assert.equal(startBand(null), null);
});

test("a placed start says the position is not confirmed and only manual driving goes", () => {
  const band = startBand({ start_block: BLOCK });
  assert.match(band.text, /운전자가 놓은 위치 · 확인되지 않음 · 지도 표시를 믿고 운전하지 마세요/);
  assert.match(band.text, /자동 운행과 미션은 받지 않습니다/);
  assert.equal(band.tone, "bad");
});

test("a start whose source is not known says so", () => {
  const band = startBand({ start_block: { ...BLOCK, source: null } });
  assert.match(band.text, /출발 위치의 출처를 확인할 수 없습니다/);
});

test("a new mission is not offered", () => {
  const why = blockReason({ state: { start_block: BLOCK, mode: "PAUSED", stopped_confirmed: true }, canMission: true,
                            hasLease: true, mode: "PAUSED", mission: null });
  assert.match(why, /자동 운행과 미션은 받지 않습니다/);
});

test("a paused mission is not resumed", () => {
  const r = resumeState({ state: { start_block: BLOCK, mode: "PAUSED", stopped_confirmed: true }, hasLease: true,
                          mission: { status: "paused", phase: "approach" } });
  assert.equal(r.enabled, false);
  assert.match(r.why, /자동 운행과 미션은 받지 않습니다/);
});
