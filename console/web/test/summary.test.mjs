import { test } from "node:test";
import assert from "node:assert/strict";
import { summarize } from "../js/summary.js";
import { ResultsBook, reasonHead } from "../js/results.js";

let seq = 0;
function res(outcome, { reason = null, started_s = 10, ended_s = 30, ...rest } = {}) {
  seq += 1;
  return { seq, mission_id: `M-${seq}`, outcome, reason, started_s, ended_s, ended_unix: 1_800_000_000 + seq,
           pallet_id: "P-01", destination_id: "D-A", map_id: null, ...rest };
}
function book(results, { older = 0, missions_seq = null, record_errors_older = 0, epoch = 1 } = {}) {
  const b = new ResultsBook();
  b.welcome(epoch);
  const top = missions_seq ?? Math.max(0, ...results.map((r) => r.seq));
  b.update({ t: "missions", missions_seq: top, older, record_errors_older, results });
  return b;
}

test("counts each outcome and the done rate over the listed results", () => {
  const s = summarize(book([res("done"), res("done"), res("done"), res("failed", { reason: "x" }),
                            res("cancelled", { reason: "operator" }), res("recovery_required", { reason: "y" })]));
  assert.deepEqual(s.counts, { done: 3, failed: 1, cancelled: 1, recovery_required: 1, other: 0 });
  assert.equal(s.listed, 6);
  assert.deepEqual(s.doneRate, { rate: 0.5, of: 6 });
  const e = summarize(book([]));
  assert.equal(e.listed, 0);
  assert.equal(e.doneRate, null);
});

test("the done rate also without cancellations, with its denominators", () => {
  const s = summarize(book([res("done"), res("cancelled", { reason: "operator" })]));
  assert.deepEqual(s.doneRate, { rate: 0.5, of: 2 });
  assert.deepEqual(s.doneRateNoCancel, { rate: 1, of: 1 });
  assert.equal(summarize(book([res("cancelled"), res("cancelled")])).doneRateNoCancel, null);
});

test("groups reasons by their meaning, most first", () => {
  assert.equal(reasonHead("approach:no_path").head, reasonHead("approach:timeout").head);
  const s = summarize(book([res("failed", { reason: "localization_stale" }), res("failed", { reason: "localization_stale" }),
                            res("failed", { reason: "localization_stale" }),
                            res("cancelled", { reason: null }), res("done", { reason: "ignored" }),
                            res("failed", { reason: "approach:no_path" }), res("failed", { reason: "approach:timeout" })]));
  assert.equal(s.reasons[0].count, 3);
  assert.deepEqual(s.reasons[0].codes, [["localization_stale", 3]]);
  assert.ok(s.reasons.some((r) => r.text === "사유 없음" && r.count === 1));
  assert.ok(!s.reasons.some((r) => r.codes.some(([c]) => c === "ignored")), "a done result's reason is not counted");
  const approach = s.reasons.find((r) => r.codes.some(([c]) => c === "approach:no_path"));
  assert.equal(approach.count, 2, "one meaning, one line");
  assert.deepEqual(approach.codes.map(([c, n]) => [c, n]).sort(), [["approach:no_path", 1], ["approach:timeout", 1]]);
});

test("candidate failures group by their sorted status counts", () => {
  const a = "prepare:all_candidates_failed:invalid_start;timeout";
  const b = "prepare:all_candidates_failed:timeout;invalid_start";
  const c = "prepare:all_candidates_failed:invalid_start;invalid_start";
  const s = summarize(book([res("failed", { reason: a }), res("failed", { reason: b }), res("failed", { reason: c })]));
  const ab = s.reasons.find((r) => r.codes.some(([x]) => x === a));
  assert.equal(ab.count, 2);
  assert.ok(ab.codes.some(([x]) => x === b));
  assert.equal(s.reasons.length, 2);
});

test("per pallet done over total, with the kind when the targets know it", () => {
  const s = summarize(book([res("done", { pallet_id: "P-01" }), res("failed", { reason: "x", pallet_id: "P-01" }),
                            res("done", { pallet_id: "P-02" }), res("done", { pallet_id: null })]),
                      { pallets: [{ pallet_id: "P-01", kind: "epal6" }, { pallet_id: "P-02", kind: "t11_06" }] });
  const p1 = s.pallets.find((p) => p.pallet_id === "P-01");
  assert.deepEqual([p1.done, p1.total, p1.kind], [1, 2, "epal6"]);
  assert.equal(s.pallets.find((p) => p.pallet_id === "P-02").kind, "t11_06");
  assert.ok(s.pallets.some((p) => p.pallet_id === null && p.total === 1));
  assert.equal(summarize(book([res("done")])).pallets[0].kind, null, "no targets: no kind");
});

test("durations use only finite start and end, done and the rest", () => {
  const s = summarize(book([res("done", { started_s: 0, ended_s: 10 }), res("done", { started_s: 5, ended_s: 25 }),
                            res("done", { started_s: 0, ended_s: 40 }), res("failed", { reason: "x", started_s: null }),
                            res("failed", { reason: "x", started_s: 50, ended_s: 40 }),
                            res("cancelled", { reason: "operator", started_s: 0, ended_s: 7 })]));
  assert.deepEqual(s.durations.done, { n: 3, median: 20, max: 40 });
  assert.deepEqual(s.durations.notDone, { n: 1, median: 7, max: 7 });
  assert.equal(s.noDuration, 2);
  const even = summarize(book([res("done", { started_s: 0, ended_s: 10 }), res("done", { started_s: 0, ended_s: 30 })]));
  assert.equal(even.durations.done.median, 20);
  const none = summarize(book([res("done", { started_s: null })]));
  assert.equal(none.durations.done, null);
  assert.equal(none.noDuration, 1);
});

test("the coverage says what is left out", () => {
  const list = Array.from({ length: 100 }, () => res("done"));
  const s = summarize(book(list, { older: 30, missions_seq: 130, record_errors_older: 2, epoch: 7 }));
  assert.equal(s.total, 130);
  assert.equal(s.listed, 100);
  assert.equal(s.older, 30);
  assert.equal(s.recordErrorsOlder, 2);
  assert.equal(s.epoch, 7);
  const all = summarize(book([res("done"), res("failed", { reason: "x", record_error: "result.json: disk full" })], { missions_seq: 2 }));
  assert.equal(all.older, 0);
  assert.equal(all.total, all.listed);
  assert.equal(all.recordErrors, 1);
});

test("an unknown outcome is counted as other", () => {
  assert.equal(summarize(book([res("exploded")])).counts.other, 1);
});
