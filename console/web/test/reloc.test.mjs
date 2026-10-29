// B-P2c (docs/plans/2026-10-29-start-relocalization.md 1.3): the start relocalization on the console -- what the
// screen says, what the operator may do, the candidate a click lands on and the pick it sends.
import { test } from "node:test";
import assert from "node:assert/strict";
import { candidateAt, pickKey, relocActions, relocOf, relocSummary } from "../js/reloc.js";
import { contentBounds } from "../js/mapview.js";
import { ConfirmGate } from "../js/results.js";

const RELOC = {
  state: "awaiting_pick", generation: 2, attempts: 3, last_reason: "competitor", deadline_left_s: 0,
  operator_left_s: 271.4,
  candidates: [
    { index: 0, base: [3.0, 1.0, 0.5], score: 0.91, endpoint_fraction: 0.97 },
    { index: 1, base: [9.0, 4.0, 2.0], score: 0.88, endpoint_fraction: 0.96 },
  ],
};
const state = (reloc) => ({ pose: null, slam: { mode: "localization", map_id: "m".repeat(64), reloc } });

test("the relocalization is read from the state's slam field, absent when localized", () => {
  assert.deepEqual(relocOf(state(RELOC)), RELOC);
  assert.equal(relocOf({ slam: { mode: "localization" } }), null);
  assert.equal(relocOf(null), null);
});

test("each state says what is going on and the operator's time left", () => {
  assert.match(relocSummary(RELOC).text, /후보 2개/);
  assert.match(relocSummary(RELOC).text, /4:32/); // rounded up: 0:00 only when none is left
  assert.match(relocSummary({ ...RELOC, state: "relocalizing", attempts: 1, candidates: [] }).text, /찾는 중/);
  assert.match(relocSummary({ ...RELOC, state: "reloc_failed", candidates: [] }).text, /정할 수 없/);
  assert.match(relocSummary({ ...RELOC, state: "stopped", candidates: [] }).text, /정지/);
  assert.match(relocSummary({ ...RELOC, state: "localizing", candidates: [] }).text, /SLAM/);
  assert.match(relocSummary(RELOC).title, /competitor/);
});

test("a pick needs the lease and candidates to pick from; a retry the lease and a waiting state", () => {
  assert.deepEqual(relocActions(RELOC, true), { pick: true, retry: { enabled: true, why: "" } });
  const noLease = relocActions(RELOC, false);
  assert.equal(noLease.pick, false);
  assert.equal(noLease.retry.enabled, false);
  assert.match(noLease.retry.why, /조종 권한/);
  for (const s of ["relocalizing", "localizing"]) {
    const a = relocActions({ ...RELOC, state: s }, true);
    assert.equal(a.pick, false);
    assert.equal(a.retry.enabled, false);
  }
  for (const s of ["reloc_failed", "stopped"]) {
    const a = relocActions({ ...RELOC, state: s, candidates: [] }, true);
    assert.equal(a.pick, false);
    assert.equal(a.retry.enabled, true);
  }
});

test("a pick is bound to the generation it was shown in (the first click's)", () => {
  const key = pickKey(RELOC, 1, { kind: "x", epoch: 5, leaseId: "L", controlGen: 3 });
  assert.deepEqual(key, { kind: "relocalize_pick", epoch: 5, leaseId: "L", controlGen: 3, generation: 2, candidate: 1 });
  assert.notDeepEqual(pickKey({ ...RELOC, generation: 3 }, 1, key), key); // a new set: the second click arms again
});

test("the confirm sends only a second click on the same candidate of the same set", () => {
  const gate = new ConfirmGate(0, 5000);
  const base = { kind: "x", epoch: 5, leaseId: "L", controlGen: 3 };
  assert.equal(gate.click(pickKey(RELOC, 0, base), 0), "armed");
  assert.equal(gate.click(pickKey(RELOC, 1, base), 10), "armed"); // another candidate arms it instead
  assert.equal(gate.click(pickKey({ ...RELOC, generation: 3 }, 1, base), 20), "armed"); // a newer set: again
  assert.equal(gate.click(pickKey({ ...RELOC, generation: 3 }, 1, base), 30), "send");
});

test("a click lands on the candidate under it, at the map's scale", () => {
  const view = { fitted: true, scale: 50, cx: 3.0, cy: 1.0 };
  const W = 400;
  const H = 300;
  // the view's centre is the first candidate
  assert.equal(candidateAt(view, W, H, RELOC, W / 2, H / 2), 0);
  assert.equal(candidateAt(view, W, H, RELOC, W / 2 + 10, H / 2), 0); // 0.2 m off
  assert.equal(candidateAt(view, W, H, RELOC, W / 2 + 40, H / 2), null); // 0.8 m off
  assert.equal(candidateAt({ ...view, fitted: false }, W, H, RELOC, W / 2, H / 2), null);
  assert.equal(candidateAt(view, W, H, null, W / 2, H / 2), null);
});

test("the map fits the candidates while the truck's place is unknown", () => {
  const b = contentBounds(null, null, state(RELOC));
  assert.ok(b.x0 <= 3.0 - 1 && b.x0 + b.w >= 9.0 + 1 && b.y0 <= 0.0 && b.y0 + b.h >= 5.0);
});
