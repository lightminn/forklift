// Persistent console plan D13 (docs/plans/2026-10-18-persistent-live-console.md, 3rd edition): the results book,
// the reason explanations, the recovery guidance, the session buttons and their two-step confirm, the advice.
import { test } from "node:test";
import assert from "node:assert/strict";

import {
  REASONS, ResultsBook, ConfirmGate, adviceText, destinationLabel, reasonText, recoveryGuide, sessionButtons,
} from "../js/results.js";

test("a reason is explained by its longest known head, the code kept beside it", () => {
  const r = reasonText("approach:no_path");
  assert.equal(r.code, "approach:no_path");
  assert.ok(r.text && r.text !== r.code);
  const longer = reasonText("near_field_contract:x");
  const shorter = reasonText("near_field_x");
  assert.notEqual(longer.text, shorter.text); // near_field_contract beats near_field_
});

test("an unknown reason is shown as its code", () => {
  assert.deepEqual(reasonText("brand_new_reason"), { text: "brand_new_reason", code: "brand_new_reason" });
  assert.equal(reasonText(null).text, "");
});

test("a loaded submit's failures say what to do, its other stages fall to the loaded head", () => {
  // persistent console plan D5 6c item 3: no delivery straight left -> back off straight and submit again
  assert.match(reasonText("prepare:loaded:transport:no_final_straight").text, /곧게 더 물러난 뒤/);
  assert.match(reasonText("prepare:loaded:no_final_straight").text, /곧게 더 물러난 뒤/);
  assert.match(reasonText("prepare:loaded:start_wait:lift,leg").text, /60 초/);
  assert.equal(reasonText("prepare:loaded:return_home:no_path").text, REASONS["prepare:loaded:"]);
  assert.notEqual(REASONS["prepare:loaded:"], REASONS["prepare:"]);
});

test("a preparation whose candidates all failed is summed up by their reasons", () => {
  const r = reasonText("prepare:all_candidates_failed:invalid_start;invalid_start;no_path");
  assert.match(r.text, /출발 자세가 막힘 2/);
  assert.match(r.text, /경로 없음 1/);
  assert.match(r.text, /3 후보/);
});

test("the results book shows the vehicle's newest list, newest first, and never an older one", () => {
  const book = new ResultsBook();
  book.welcome(5);
  book.update({ missions_seq: 2, results: [{ seq: 1, mission_id: "a" }, { seq: 2, mission_id: "b" }] });
  assert.deepEqual(book.rows().map((r) => r.mission_id), ["b", "a"]);
  book.update({ missions_seq: 2, results: [{ seq: 1, mission_id: "a" }, { seq: 2, mission_id: "b", record_error: "x" }] });
  assert.equal(book.rows().length, 2);
  assert.equal(book.rows()[0].record_error, "x"); // a note on a kept result is shown
  book.update({ missions_seq: 4, older: 2, results: [{ seq: 3, mission_id: "c" }, { seq: 4, mission_id: "d" }] });
  assert.equal(book.older, 2);
  assert.deepEqual(book.rows().map((r) => r.seq), [4, 3]); // what the vehicle keeps, not every seq ever seen
  book.update({ missions_seq: 3, results: [{ seq: 3, mission_id: "c" }] }); // an older message arriving late
  assert.deepEqual(book.rows().map((r) => r.seq), [4, 3]);
});

test("a long session does not grow the page's list past what the vehicle keeps", () => {
  const book = new ResultsBook();
  book.welcome(1);
  for (let n = 1; n <= 1001; n++) {
    const results = [];
    for (let s = Math.max(1, n - 99); s <= n; s++) results.push({ seq: s, mission_id: `m${s}` });
    book.update({ missions_seq: n, older: Math.max(0, n - 100), results });
  }
  assert.equal(book.rows().length, 100);
  assert.equal(book.older, 901);
});

test("a new session epoch empties the book (its seq starts again at 1)", () => {
  const book = new ResultsBook();
  book.welcome(5);
  book.update({ missions_seq: 7, results: [{ seq: 7, mission_id: "old" }] });
  book.welcome(5); // the same session reconnecting: kept
  assert.equal(book.rows().length, 1);
  book.welcome(6);
  assert.equal(book.rows().length, 0);
  book.update({ missions_seq: 1, results: [{ seq: 1, mission_id: "new" }] });
  assert.deepEqual(book.rows().map((r) => r.mission_id), ["new"]);
});

test("the recovery guidance comes from the state alone, a failed mission included", () => {
  assert.equal(recoveryGuide({ recovery: null }), null);
  const lower = recoveryGuide({ recovery: { stage: "in_pocket", grounded: false, evidence: true, aligned: true },
                                lift_dirs: [-1], drive_mode: "normal", teleop_forbidden_reason: "recovery:lower_first",
                                stand_waiver: "lower_only", lift_m: 0.08, last_result: { outcome: "failed" } });
  assert.match(lower.next, /내리기/);
  assert.match(lower.notes.join(" "), /서 있음 확인 없이/);
  const steering = recoveryGuide({ recovery: { stage: "in_pocket", grounded: true, evidence: true, aligned: true },
                                   lift_dirs: [], drive_mode: "reverse_straight", teleop_forbidden_reason: null,
                                   permit: { limit_mps: 0, reason: "recovery:steering" } });
  assert.match(steering.next, /곧게 후진/);
  assert.match(steering.permit, /현재 구동 허가/);
  assert.match(steering.permit, /조향/);
});

function st(over = {}) {
  return { mode: "PAUSED", feedback_ok: true, stopped_confirmed: true, mission: null, session_epoch: 3,
           control_gen: 4, ...over };
}

test("the session buttons follow the arbiter's order: lease, feedback, stand, no mission", () => {
  const caps = { sim_reset: true };
  assert.match(sessionButtons(st(), { hasLease: false, caps }).end.why, /조종 권한/);
  assert.match(sessionButtons(st({ feedback_ok: false, stopped_confirmed: false }), { hasLease: true, caps }).end.why,
               /피드백/);
  assert.match(sessionButtons(st({ stopped_confirmed: false, stand_waiver: "lower_only" }),
                              { hasLease: true, caps }).end.why, /정지 확인/); // no waiver for the session
  assert.match(sessionButtons(st({ mission: { status: "running" } }), { hasLease: true, caps }).end.why, /미션/);
  // the arbiter's ACTIVE (commands.py) includes cancelling (Codex D13 impl round 2 P2)
  assert.match(sessionButtons(st({ mission: { status: "cancelling" } }), { hasLease: true, caps }).end.why, /미션/);
  const ok = sessionButtons(st(), { hasLease: true, caps });
  assert.equal(ok.end.enabled, true);
  assert.equal(ok.reset.shown, true);
  assert.equal(sessionButtons(st(), { hasLease: true, caps: {} }).reset.shown, false);
});

test("the two-step confirm takes a second click from 0.5 s to 5 s, bound to its epoch, lease and generation", () => {
  const gate = new ConfirmGate();
  const key = { kind: "sim_reset", epoch: 3, leaseId: "L1", controlGen: 4 };
  assert.equal(gate.click(key, 0), "armed");
  assert.equal(gate.click(key, 300), "too_soon"); // a double click does not send
  assert.equal(gate.click(key, 800), "send");
  assert.equal(gate.click(key, 900), "armed"); // sent: a new click starts over
  assert.equal(gate.click(key, 6500), "armed"); // expired, armed again
  assert.equal(gate.click({ ...key, controlGen: 5 }, 7200), "armed"); // a new generation: not the same confirm
  gate.click(key, 10000);
  gate.cancel();
  assert.equal(gate.click(key, 10800), "armed");
});

test("the advice reads as a guide from the pose at the failure, never as a permit", () => {
  assert.equal(adviceText(null), null);
  assert.match(adviceText({ backoff_m: 0.15, backoff_max_m: 0.6 }), /0\.15–0\.60 m/);
  assert.match(adviceText({ backoff_m: 0.15, backoff_max_m: 0.6 }), /실패 때 자세 기준/);
  assert.match(adviceText({ backoff_m: null, within_m: 1.0 }), /1\.0 m 안/);
  assert.match(adviceText({ backoff_m: null, blocked_at_m: 0.35 }), /0\.35 m/);
  assert.match(adviceText({ skipped: "deadline" }), /시간/);
});

test("the destinations get their names and an unknown one stays as it is", () => {
  assert.match(destinationLabel("D-0"), /원래 자리/);
  assert.match(destinationLabel("D-1"), /목적지/);
  assert.equal(destinationLabel("D-9"), "D-9");
});

test("the reasons table is plain text keyed by reason heads", () => {
  for (const [k, v] of Object.entries(REASONS)) {
    assert.equal(typeof k, "string");
    assert.equal(typeof v, "string");
    assert.ok(v.length > 0);
  }
});
