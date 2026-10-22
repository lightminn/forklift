// A summary of this session's results book (docs/plans/2026-10-22-console-results-summary.md). Pure module: it counts
// only what the console holds -- the vehicle's newest 100 results of the session (contract 5.2) -- and says what is
// left out; nothing is asked of the vehicle and nothing is kept across sessions.

import { reasonHead, reasonText } from "./results.js";

const OUTCOMES = ["done", "failed", "cancelled", "recovery_required"];
const ALL_FAILED = "prepare:all_candidates_failed:";

const rate = (n, of) => (of > 0 ? { rate: n / of, of } : null);

function median(sorted) {
  const m = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[m] : (sorted[m - 1] + sorted[m]) / 2;
}

function stats(values) {
  if (!values.length) return null;
  const s = [...values].sort((a, b) => a - b);
  return { n: s.length, median: median(s), max: s[s.length - 1] };
}

// One line per meaning (plan D2): the reason table's head, or for a failed first plan the candidates' statuses
// sorted, so the same failure in another candidate order is the same line.
function groupOf(code) {
  if (!code) return { key: "", text: "사유 없음" };
  if (code.startsWith(ALL_FAILED)) {
    const canonical = ALL_FAILED + code.slice(ALL_FAILED.length).split(";").filter(Boolean).sort().join(";");
    return { key: canonical, text: reasonText(canonical).text };
  }
  const { head, text } = reasonHead(code);
  return { key: head, text };
}

// book: results.js ResultsBook; targets: the targets list (for the pallets' kinds), may be null
export function summarize(book, targets = null) {
  const rows = book.rows();
  const counts = { done: 0, failed: 0, cancelled: 0, recovery_required: 0, other: 0 };
  const groups = new Map();
  const pallets = new Map();
  const done = [];
  const notDone = [];
  let noDuration = 0;
  let recordErrors = 0;
  const kinds = new Map((targets?.pallets ?? []).map((p) => [p.pallet_id, p.kind ?? null]));

  for (const r of rows) {
    const outcome = OUTCOMES.includes(r.outcome) ? r.outcome : "other";
    counts[outcome] += 1;
    if (r.record_error) recordErrors += 1;
    const pid = r.pallet_id ?? null;
    const p = pallets.get(pid) ?? { pallet_id: pid, kind: pid === null ? null : kinds.get(pid) ?? null, done: 0, total: 0 };
    p.total += 1;
    if (outcome === "done") p.done += 1;
    pallets.set(pid, p);
    if (outcome !== "done") {
      const g = groupOf(r.reason);
      const entry = groups.get(g.key) ?? { text: g.text, count: 0, latest: -Infinity, codes: new Map() };
      entry.count += 1;
      entry.latest = Math.max(entry.latest, r.seq ?? -Infinity);
      if (r.reason) entry.codes.set(r.reason, (entry.codes.get(r.reason) ?? 0) + 1);
      groups.set(g.key, entry);
    }
    const ok = Number.isFinite(r.started_s) && Number.isFinite(r.ended_s) && r.ended_s >= r.started_s;
    if (!ok) noDuration += 1;
    else (outcome === "done" ? done : notDone).push(r.ended_s - r.started_s);
  }

  const listed = rows.length;
  return {
    epoch: book.epoch,
    total: Math.max(book.seq ?? 0, listed),
    listed,
    older: book.older ?? 0,
    recordErrors,
    recordErrorsOlder: book.recordErrorsOlder ?? 0,
    counts,
    doneRate: rate(counts.done, listed),
    doneRateNoCancel: rate(counts.done, listed - counts.cancelled),
    reasons: [...groups.values()]
      .sort((a, b) => b.count - a.count || b.latest - a.latest)
      .map((g) => ({ text: g.text, count: g.count, codes: [...g.codes].sort((a, b) => b[1] - a[1]) })),
    pallets: [...pallets.values()].sort((a, b) => b.total - a.total || String(a.pallet_id).localeCompare(String(b.pallet_id))),
    durations: { done: stats(done), notDone: stats(notDone) },
    noDuration,
  };
}
