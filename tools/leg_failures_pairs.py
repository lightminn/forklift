"""The V4a seed-pair judgement of docs/plans/2026-10-25-leg-failures.md (B1/B2).

Each pair is one seed run on both arms: the session's re-approach completion rate (completed / attempted, a plan or
dry-run refusal counted as an attempt) on main and on the fixed arm, and the two arms' refusal counts. A pair with no
attempt on either side is not used; fewer than six usable pairs is undecidable. The one-sided signed-rank p (fixed
better) drops zero differences, gives the rest average ranks and enumerates every sign combination -- not scipy's
method="exact", which does not follow average ranks under ties (Codex plan round 4 P2). Adopted when p < 0.05, at most
two pairs are worse and the refusals do not increase.

    python tools/leg_failures_pairs.py pairs.json
pairs.json: [{"seed": 0, "main": [completed, attempted, refusals], "fix": [...]}, ...]
"""

from __future__ import annotations

import argparse
import itertools
import json
from fractions import Fraction

ALPHA = 0.05
MIN_PAIRS = 6
MAX_WORSE = 2


def _average_ranks(values):
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        for k in range(i, j + 1):
            ranks[order[k]] = (i + j) / 2 + 1
        i = j + 1
    return ranks


def signed_rank_p(differences) -> float:
    """P(W+ >= observed) under the null, over every sign combination of the average ranks (zeros dropped)."""
    # exact values (rates as Fractions): a float difference would split a tie (Codex plan round 5 P2)
    nonzero = [d for d in differences if d != 0]
    if not nonzero:
        return 1.0
    ranks = _average_ranks([abs(d) for d in nonzero])
    observed = sum(r for r, d in zip(ranks, nonzero) if d > 0)
    hits = 0
    for signs in itertools.product((0, 1), repeat=len(ranks)):
        if sum(r for r, s in zip(ranks, signs) if s) >= observed:
            hits += 1
    return hits / 2 ** len(ranks)


def judge(pairs) -> dict:
    """pairs: (main rate, fix rate, main refusals, fix refusals); a rate is None with no attempt on that side."""
    usable = [p for p in pairs if p[0] is not None and p[1] is not None]
    refusals = (sum(p[2] for p in pairs), sum(p[3] for p in pairs))
    out = {"usable_pairs": len(usable), "unused_pairs": len(pairs) - len(usable), "refusals_main": refusals[0],
           "refusals_fix": refusals[1]}
    if len(usable) < MIN_PAIRS:
        out.update(verdict="undecidable", p=None, worse_pairs=None)
        return out
    differences = [p[1] - p[0] for p in usable]
    out["p"] = signed_rank_p(differences)
    out["worse_pairs"] = sum(d < 0 for d in differences)
    ok = out["p"] < ALPHA and out["worse_pairs"] <= MAX_WORSE and refusals[1] <= refusals[0]
    out["verdict"] = "pass" if ok else "fail"
    return out


def pairs_from_rows(rows) -> list:
    """The judge's pairs from the counts: each rate an exact Fraction (None with no attempt)."""

    def rate(arm):
        completed, attempted, _ = arm
        return None if attempted == 0 else Fraction(int(completed), int(attempted))

    return [(rate(r["main"]), rate(r["fix"]), r["main"][2], r["fix"][2]) for r in rows]


def _json(path):
    with open(path) as handle:
        return json.load(handle)


def _session(run_dir):
    """A session's missions, every epoch, as (state, result) pairs -- either None when its file is missing -- with
    whether its records are inconsistent and the probe's acked count (None without the probe's report).

    The mission folders come first (a folder missing one file is still a mission, Codex V5 4th/6th review P2); each
    epoch whose run failed mid-mission (slam_link_failed and the like) adds the unfinished mission kept only in that
    run's result.json, with its own retries and reason, unless the record repeats that epoch's last mission folder's
    (Codex V4 hold review P2, V5 3rd review P2). Missions the probe acked beyond those were cut with no record at all
    (a killed run, Codex V5 2nd review P2): (None, None). More missions than acked submits is inconsistent (Codex V5
    3rd review P2: a duplicated done folder would hide a failure)."""
    import glob

    try:  # the probe's accepted submits: how many missions the session started (Codex V5 review P2-1)
        acked = sum(1 for m in _json(f"{run_dir}/probe.json").get("missions_run") or []
                    if (m.get("submit") or {}).get("t") == "ack")
    except (OSError, ValueError):
        acked = None
    out = []
    for run in sorted(glob.glob(f"{run_dir}/live/epoch_*/run")):
        last = None  # the epoch's last folder's state; "unknown" when its records are missing
        for md in sorted(glob.glob(f"{run}/missions/m*")):
            try:
                st = _json(f"{md}/records.json")["state"]
            except (OSError, ValueError, KeyError):
                st = None
            try:
                res = _json(f"{md}/result.json").get("result") or {}
            except (OSError, ValueError):
                res = None
            out.append((st, res))  # an empty folder is still a mission, both unknown (Codex V5 7th review P2)
            last = "unknown" if st is None else st
        try:
            top = _json(f"{run}/result.json")
        except (OSError, ValueError):
            continue
        retries = top.get("docking_retries") or []
        # after a folder with no records the run record cannot be told from that mission's: add nothing -- the
        # probe's count (or the scheduled count) fills a cut mission as unknown, read at both bounds
        if top.get("success") is False and last != "unknown" and (
                last is None or retries != (last.get("docking_retries") or [])):
            out.append(({"docking_retries": retries, "delivery_docking": top.get("delivery_docking") or {},
                         "forbidden_pocket_contacts": top.get("forbidden_pocket_contacts") or [],
                         "slam": top.get("slam") or {}},
                        {"outcome": "session_failed", "reason": top.get("failure_reason"), "ended_s": None}))
    inconsistent = acked is not None and len(out) > acked
    for _ in range(max(0, acked - len(out)) if acked is not None else 0):
        out.append((None, None))
    return out, inconsistent, acked


def _session_states(run_dir) -> list:
    """The V4 view: a missing file read as empty (an unknown mission adds no attempt and is not done)."""
    return [(st or {}, res if res is not None else {"outcome": "session_failed", "reason": "no_record",
                                                    "ended_s": None})
            for st, res in _session(run_dir)[0]]


def session_counts(run_dir) -> list:
    """[completed, attempted, refusals] over a session's counted D7 re-approaches (every epoch's missions). An attempt
    is a counted docking retry; it is completed when a docking round falls inside its window (up to the next counted
    retry or the mission's end -- the retry match is made only where the re-approach stands, as V2's extract); it is a
    refusal when its plan was refused (box, length, dry run) or not planned."""
    completed = attempted = refusals = 0
    for st, res in _session_states(run_dir):
        counted = [e for e in st.get("docking_retries") or [] if e.get("counted")]
        rounds = [float(r["time_s"]) for r in (st.get("delivery_docking") or {}).get("rounds") or []
                  if r.get("time_s") is not None]
        for k, ev in enumerate(counted):
            attempted += 1
            plan = ev.get("plan") or {}
            if plan.get("refused") or plan.get("length_m") is None:
                refusals += 1
                continue
            end = counted[k + 1]["time_s"] if k + 1 < len(counted) else res.get("ended_s")
            if any(t > ev["time_s"] + 1e-9 and (end is None or t <= float(end) + 1e-9) for t in rounds):
                completed += 1
    return [completed, attempted, refusals]


def session_missions(run_dir) -> dict:
    """A session's mission outcomes (re-approach hold plan V4a guard 2): missions run, done, ended on
    transport_recovery_after_docking, and the hold exits taken (uncounted docking retries ``h``); ``unknown``: the
    missions missing a file."""
    states, inconsistent, _ = _session(run_dir)
    out = {"missions": 0, "done": 0, "recovery_after_docking": 0, "hold_exits": 0, "session_failed": 0,
           "unknown": sum(st is None or res is None for st, res in states), "reasons": [],
           "inconsistent": inconsistent}
    for st, res in _session_states(run_dir):
        out["missions"] += 1
        out["session_failed"] += res.get("outcome") == "session_failed"
        out["done"] += res.get("outcome") == "done"
        out["recovery_after_docking"] += res.get("reason") == "transport_recovery_after_docking"
        out["hold_exits"] += sum(1 for e in st.get("docking_retries") or [] if e.get("trigger") == "h")
        if res.get("outcome") != "done":
            out["reasons"].append(res.get("reason"))
    return out


def _no_stall(st, res) -> bool:
    """Every counted re-approach arrived (a docking round in its window) and was planned."""
    counted = [e for e in st.get("docking_retries") or [] if e.get("counted")]
    rounds = [float(r["time_s"]) for r in (st.get("delivery_docking") or {}).get("rounds") or []
              if r.get("time_s") is not None]
    for k, ev in enumerate(counted):
        plan = ev.get("plan") or {}
        if plan.get("refused") or plan.get("length_m") is None:
            return False
        end = counted[k + 1]["time_s"] if k + 1 < len(counted) else (res or {}).get("ended_s")
        if not any(t > ev["time_s"] + 1e-9 and (end is None or t <= float(end) + 1e-9) for t in rounds):
            return False
    return True


def _mission_clean(st, res) -> bool:
    """Done, and with no re-approach stall."""
    return res.get("outcome") == "done" and _no_stall(st, res)


def session_score(run_dir, scheduled=3) -> int:
    """Re-approach hold plan 2a (V5) primary, known records only: of the session's scheduled missions, how many
    completed with no re-approach stall. A mission not reached, cut or failed scores 0, and so does one whose stall
    recovered. The V5 judgement uses session_v5, which bounds the unknown ones."""
    return min(int(scheduled), sum(_mission_clean(st, res) for st, res in _session(run_dir)[0]
                                   if st is not None and res is not None))


def _refused(e) -> bool:
    plan = e.get("plan") or {}
    return bool(e.get("counted")) and bool(plan.get("refused") or plan.get("length_m") is None)


def _mission_bound(st, res, best: bool) -> dict:
    """One mission's V5 quantities with a missing file filled at its bound (Codex V5 4th-6th review P2): ``best``
    (the off arm) takes an unknown outcome as done, an unknown record as clean, no D7 failure, no refusal and no
    contact; worst (the hold arm) takes an unknown outcome as not done, an unknown record as a stall, a D7 failure when
    not done, one refusal (a refused counted re-approach plan ends the mission: the require after the retry plan in
    core.py) and an unconfirmed contact."""
    done = res.get("outcome") == "done" if res is not None else best
    if st is None:
        return {"done": done, "clean": done and best, "d7_failed": not done and not best, "refusals": int(not best),
                "contacts": 0, "contact_unknown": not best}
    retries = st.get("docking_retries") or []
    return {"done": done, "clean": done and _no_stall(st, res),
            "d7_failed": not done and any(e.get("counted") or e.get("trigger") == "h" for e in retries),
            "refusals": sum(1 for e in retries if _refused(e)),
            # any forbidden contact counts, hold or not (stricter than the hold missions alone; no hold flag to miss)
            "contacts": int(bool(st.get("forbidden_pocket_contacts"))), "contact_unknown": False}


def session_v5(run_dir, scheduled=3) -> dict:
    """A session's V5 quantities at both bounds ("best" for off, "worst" for the hold arm): score (clean missions),
    done, d7_failed, refusals, contacts (any forbidden pocket contact), contact_unknown, and whether the session is
    inconsistent.
    Without the probe's report, the scheduled missions with no folder are unknown; with it, missions never acked were
    not reached (known, 0 on both)."""
    states, inconsistent, acked = _session(run_dir)
    if acked is None:
        states = states + [(None, None)] * max(0, int(scheduled) - len(states))
    inconsistent = inconsistent or len(states) > int(scheduled)
    out = {"inconsistent": inconsistent, "unknown": sum(st is None or res is None for st, res in states)}
    for name, best in (("best", True), ("worst", False)):
        ms = [_mission_bound(st, res, best) for st, res in states]
        out[name] = {"score": sum(int(m["clean"]) for m in ms), "done": sum(int(m["done"]) for m in ms),
                     **{k: sum(int(m[k]) for m in ms) for k in ("d7_failed", "refusals", "contacts",
                                                                "contact_unknown")}}
    return out


def sign_test_p(differences) -> float:
    """Exact one-sided sign test: P(at least the observed positives | the nonzero differences split as a fair coin)."""
    from math import comb

    pos = sum(1 for d in differences if d > 0)
    n = sum(1 for d in differences if d != 0)
    if n == 0:
        return 1.0
    return sum(comb(n, k) for k in range(pos, n + 1)) / 2 ** n


V5_FAILURE_MARGIN = 6


def v5_judge(rows, scheduled=3) -> dict:
    """Re-approach hold plan 2a (V5) judgement over collected rows, off at its best bound and the hold arm at its
    worst (session_v5): the exact sign test on the score differences (p < 0.05), the summed score not lower (Codex V5
    2nd review P2), failures (scheduled - done) on <= off + V5_FAILURE_MARGIN, missions not finished after a
    re-approach (d7_failed) not increased (3rd review), refusals not increased, and no forbidden contact on the hold
    arm with none unconfirmed (6th review P2). An inconsistent session stops it: "undecidable"."""
    off = [r["v5"]["main"]["best"] for r in rows]
    on = [r["v5"]["fix"]["worst"] for r in rows]
    bad = sorted({r.get("seed") for r in rows if r["v5"]["main"]["inconsistent"] or r["v5"]["fix"]["inconsistent"]})
    diffs = [b["score"] - a["score"] for a, b in zip(off, on)]

    def tot(arms, key):
        return sum(a[key] for a in arms)

    n = int(scheduled) * len(rows)
    out = {"pairs": len(rows), "wins": sum(d > 0 for d in diffs), "losses": sum(d < 0 for d in diffs),
           "p": sign_test_p(diffs), "score_main": tot(off, "score"), "score_fix": tot(on, "score"),
           "failures_main": n - tot(off, "done"), "failures_fix": n - tot(on, "done"),
           "d7_failed_main": tot(off, "d7_failed"), "d7_failed_fix": tot(on, "d7_failed"),
           "refusals_main": tot(off, "refusals"), "refusals_fix": tot(on, "refusals"),
           "contacts_fix": tot(on, "contacts"), "contact_unknown_fix": tot(on, "contact_unknown"),
           "unknown_main": sum(r["v5"]["main"]["unknown"] for r in rows),
           "unknown_fix": sum(r["v5"]["fix"]["unknown"] for r in rows), "inconsistent_seeds": bad}
    if bad:
        out["verdict"] = "undecidable"
        return out
    ok = (out["p"] < ALPHA and out["score_fix"] >= out["score_main"]
          and out["failures_fix"] <= out["failures_main"] + V5_FAILURE_MARGIN
          and out["d7_failed_fix"] <= out["d7_failed_main"] and out["refusals_fix"] <= out["refusals_main"]
          and out["contacts_fix"] == 0 and out["contact_unknown_fix"] == 0)
    out["verdict"] = "pass" if ok else "fail"
    return out


def collect(live_dir, prefix, scene, seeds, arms=("main", "fix")) -> list:
    """The pairs file's rows from the V4 run folders <live_dir>/<prefix>_<arm>_<scene>_s<seed>, the two arms read
    as "main" and "fix" (the baseline first), each with its session's mission outcomes."""
    base, fix = arms
    rows = []
    for s in seeds:
        b, f = f"{live_dir}/{prefix}_{base}_{scene}_s{s}", f"{live_dir}/{prefix}_{fix}_{scene}_s{s}"
        rows.append({"seed": s, "main": session_counts(b), "fix": session_counts(f),
                     "missions": {"main": session_missions(b), "fix": session_missions(f)},
                     "score": {"main": session_score(b), "fix": session_score(f)},
                     "v5": {"main": session_v5(b), "fix": session_v5(f)}})
    return rows


def mission_totals(rows) -> dict:
    """Each arm's mission outcomes summed over the seeds (re-approach hold plan V4a guard 2)."""
    out = {}
    for arm in ("main", "fix"):
        keys = ("missions", "done", "recovery_after_docking", "hold_exits", "session_failed")
        out[arm] = {k: sum(r["missions"][arm][k] for r in rows if "missions" in r) for k in keys}
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pairs", nargs="?", help="JSON list of {seed, main: [completed, attempted, refusals], fix}")
    parser.add_argument("--collect", nargs=3, metavar=("LIVE_DIR", "PREFIX", "SCENE"),
                        help="build the rows from the V4 run folders instead (seeds 0-9)")
    parser.add_argument("--out", help="write the collected rows here")
    parser.add_argument("--arms", nargs=2, default=("main", "fix"), metavar=("BASELINE", "FIX"),
                        help="the two arms' names in the run folders (default main fix)")
    parser.add_argument("--seeds", type=int, default=10, help="seeds 0..N-1 (default 10)")
    parser.add_argument("--first-seed", type=int, default=0, help="the first seed (V5: 10)")
    parser.add_argument("--v5", action="store_true",
                        help="the V5 judgement only -- the V4 signed-rank enumeration is 2^pairs (Codex V5 3rd review)")
    args = parser.parse_args()
    if args.collect:
        rows = collect(*args.collect, range(args.first_seed, args.first_seed + args.seeds), arms=tuple(args.arms))
        if args.out:
            with open(args.out, "w") as handle:
                json.dump(rows, handle, indent=1)
    else:
        with open(args.pairs) as handle:
            rows = json.load(handle)
    if args.v5:
        out = {"rows": rows, "v5": v5_judge(rows), "missions": mission_totals(rows)}
    else:
        out = {"rows": rows, "judgement": judge(pairs_from_rows(rows)), "missions": mission_totals(rows)}
    print(json.dumps(out, indent=2, default=float))


if __name__ == "__main__":
    main()
