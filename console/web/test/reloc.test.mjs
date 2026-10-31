// B-P2c (docs/plans/2026-10-29-start-relocalization.md 1.3): the start relocalization on the console -- what the
// screen says, what the operator may do, the candidate a click lands on and the pick it sends.
import { test } from "node:test";
import assert from "node:assert/strict";
import { candidateAt, candidateButtons, pickKey, relocActions, relocOf, relocSummary } from "../js/reloc.js";
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

test("every candidate has its own button, also two at one place with other headings", () => {
  // Codex B-P2c impl review P2: the map's click reaches one of two candidates on the same spot only
  const twin = { ...RELOC, candidates: [
    { index: 0, base: [3.0, 2.0, 0.0], score: 0.9, endpoint_fraction: 0.97 },
    { index: 1, base: [3.0, 2.0, 3.14], score: 0.89, endpoint_fraction: 0.96 }] };
  const buttons = candidateButtons(twin, relocActions(twin, true));
  assert.deepEqual(buttons.map((b) => b.index), [0, 1]);
  assert.ok(buttons.every((b) => b.enabled));
  assert.match(buttons[1].label, /후보 2/);
  assert.match(buttons[1].label, /180°/); // the heading tells the twins apart
  assert.ok(candidateButtons(twin, relocActions(twin, false)).every((b) => !b.enabled));
  assert.deepEqual(candidateButtons({ ...twin, state: "relocalizing" }, relocActions(twin, true)), []);
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

// --- F1: the operator's placed pose (docs/plans/2026-10-31-operator-placed-start.md 1.2) --------------------------------
import { placeActions, placeFromDrag, placeKey, previewKey } from "../js/reloc.js";

const VIEW = { cx: 0, cy: 0, scale: 20, fitted: true }; // 20 px per metre, the map's centre at the screen's
const PLACING = { ...RELOC, places: 1, max_places: 3, placed: [5.0, 2.0, 0.3], preview: null, preview_id: null };

test("a press and a drag on the map place a pose: where pressed, heading along the drag", () => {
  const p = placeFromDrag(VIEW, 400, 300, { x: 200, y: 150 }, { x: 240, y: 110 });
  assert.deepEqual(p.slice(0, 2), [0, 0]);
  assert.ok(Math.abs(p[2] - Math.PI / 4) < 1e-9); // up and right on the screen is +x +y in the map
  assert.equal(placeFromDrag(VIEW, 400, 300, { x: 200, y: 150 }, { x: 205, y: 150 }), null); // no heading yet
});

test("placing needs the lease, a candidate stage or a preview, and placements left", () => {
  assert.equal(placeActions({ ...PLACING, state: "awaiting_pick" }, true).place.enabled, true);
  assert.equal(placeActions({ ...PLACING, state: "reloc_failed" }, true).place.enabled, true);
  assert.equal(placeActions({ ...PLACING, state: "place_preview", preview_id: 1 }, true).place.enabled, true);
  assert.equal(placeActions({ ...PLACING, state: "placing" }, true).place.enabled, false);
  assert.equal(placeActions({ ...PLACING, state: "relocalizing" }, true).place.enabled, false);
  const used = placeActions({ ...PLACING, state: "awaiting_pick", places: 3 }, true).place;
  assert.equal(used.enabled, false);
  assert.match(used.why, /세 번/);
  assert.match(placeActions({ ...PLACING, state: "awaiting_pick" }, false).place.why, /조종 권한/);
});

test("only a preview may be confirmed, with the lease", () => {
  assert.equal(placeActions({ ...PLACING, state: "place_preview", preview_id: 1 }, true).confirm, true);
  assert.equal(placeActions({ ...PLACING, state: "place_preview", preview_id: 1 }, false).confirm, false);
  assert.equal(placeActions({ ...PLACING, state: "awaiting_pick" }, true).confirm, false);
});

test("the two-step keys bind the pose or the preview to the set and the session they were made in", () => {
  const base = { epoch: 5, leaseId: "L", controlGen: 3 };
  const k = placeKey(PLACING, [5.0, 2.0, 0.3], base);
  assert.equal(k.kind, "relocalize_place");
  assert.notDeepEqual(k, placeKey(PLACING, [5.5, 2.0, 0.3], base)); // another pose arms again
  const pk = previewKey({ ...PLACING, state: "place_preview", preview_id: 2 }, base);
  assert.deepEqual([pk.kind, pk.generation, pk.preview_id], ["relocalize_place_confirm", 2, 2]);
});

test("the placement states and a refusal say what happened", () => {
  assert.match(relocSummary({ ...PLACING, state: "placing" }).text, /맞추는 중/);
  assert.match(relocSummary({ ...PLACING, state: "place_preview", preview_id: 1 }).text, /미리보기/);
  assert.match(relocSummary({ ...PLACING, state: "awaiting_pick", place_reason: "ambiguous" }).text, /스캔만으로/);
  assert.match(relocSummary({ ...PLACING, state: "awaiting_pick" }).text, /놓기 1\/3/);
});

test("a second click sends only the same placed pose or the same preview it armed", () => {
  const base = { epoch: 5, leaseId: "L", controlGen: 3 };
  const gate = new ConfirmGate();
  assert.equal(gate.click(placeKey(PLACING, [5.0, 2.0, 0.3], base), 0), "armed");
  assert.equal(gate.click(placeKey(PLACING, [5.5, 2.0, 0.3], base), 800), "armed"); // re-dragged: arms again
  assert.equal(gate.click(placeKey(PLACING, [5.5, 2.0, 0.3], base), 1600), "send");
  const prev = { ...PLACING, state: "place_preview" };
  assert.equal(gate.click(previewKey({ ...prev, preview_id: 1 }, base), 0), "armed");
  assert.equal(gate.click(previewKey({ ...prev, preview_id: 2 }, base), 800), "armed"); // a newer preview
});

test("a preview may also be dropped for a new global attempt, and says how to confirm it", () => {
  const prev = { ...PLACING, state: "place_preview", preview_id: 1 };
  assert.equal(relocActions(prev, true).retry.enabled, true);
  assert.equal(relocActions(prev, true).pick, false);
});

test("a preview with look-alike places asks the operator to check the bay (lead 2026-10-31)", () => {
  const base = { state: "place_preview", generation: 2, places: 1, max_places: 3, preview: [1, 2, 0], preview_id: 1 };
  assert.match(relocSummary({ ...base, preview_twins: 2 }).text, /같은 모양 칸 2곳 — 칸 번호를 확인하세요/);
  assert.doesNotMatch(relocSummary({ ...base, preview_twins: 0 }).text, /같은 모양/);
  assert.doesNotMatch(relocSummary({ ...base, preview_twins: null }).text, /같은 모양/);
  assert.doesNotMatch(relocSummary({ ...base, state: "awaiting_pick", preview_twins: 2 }).text, /같은 모양/);
});
