import { test } from "node:test";
import assert from "node:assert/strict";
import { escapeRequest, escapeState } from "../js/profile.js";

// docs/design/2026-11-12-standing-direction-rule.md 4.1-4.3: the S6 bridge offers a wall escape; the console shows
// the button only while an offer stands, and the press approves exactly that offer

const OFFER = { offer_id: 41, direction: 1, blocked: -1, reason: "blocked_behind" };
const ST = { mode: "TELEOP", armed: true, arm_gen: 3, control_gen: 7, escape_offer: OFFER, escape: null };

test("no offer, no button (Isaac and the mock never send one)", () => {
  assert.equal(escapeState({ mode: "TELEOP", armed: true }, true).shown, false);
  assert.equal(escapeState({ ...ST, escape_offer: null }, true).shown, false);
  assert.equal(escapeState(null, true).shown, false);
});

test("an offer shows the button with the blocked side named, usable with the lease", () => {
  const s = escapeState(ST, true);
  assert.equal(s.shown, true);
  assert.equal(s.enabled, true);
  assert.match(s.label, /벽 앞 탈출/);
  assert.match(s.label, /앞으로/);
  assert.match(s.hint, /뒤/);
  assert.match(s.hint, /직접 보고/);
  const rev = escapeState({ ...ST, escape_offer: { ...OFFER, direction: -1, blocked: 1 } }, true);
  assert.match(rev.label, /뒤로/);
  assert.match(rev.hint, /앞/);
});

test("without the lease the button shows but cannot be pressed", () => {
  const s = escapeState(ST, false);
  assert.equal(s.shown, true);
  assert.equal(s.enabled, false);
});

test("an escape running shows its progress instead of the button", () => {
  const s = escapeState({ ...ST, escape_offer: null, escape: { offer_id: 41, direction: 1, travel_m: [0.05, 0.04],
    elapsed_s: 1.2, budget_m: 0.2, time_s: 4 } }, true);
  assert.equal(s.shown, false);
  assert.match(s.running, /탈출 중/);
  assert.match(s.running, /0\.05/);
});

test("the request approves exactly the offer shown, with the arm generation it was made under", () => {
  assert.deepEqual(escapeRequest(ST), { arm_gen: 3, offer_id: 41 });
  assert.equal(escapeRequest({ ...ST, escape_offer: null }), null);
  assert.equal(escapeRequest({ ...ST, arm_gen: undefined }), null);
});

test("the escape's refusals read in Korean", async () => {
  const { rejectText } = await import("../js/profile.js");
  for (const r of ["escape_no_offer", "escape_stale", "escape_active", "stale_arm", "not_teleop", "not_armed", "latched"]) {
    assert.notEqual(rejectText(r), r, r);
  }
});
