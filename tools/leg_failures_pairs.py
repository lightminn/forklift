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


def _session_states(run_dir) -> list:
    """Each mission's (state, result) of a session, every epoch: the mission folders, then -- when the run failed
    mid-mission (slam_link_failed and the like) -- the unfinished mission kept only in the run's result.json, unless
    that record repeats the last mission folder's (Codex V4 hold review P2)."""
    import glob

    try:  # the probe's accepted submits: how many missions the session started (Codex V5 review P2-1)
        acked = sum(1 for m in _json(f"{run_dir}/probe.json").get("missions_run") or []
                    if (m.get("submit") or {}).get("t") == "ack")
    except (OSError, ValueError):
        acked = None
    out = []
    tail = None  # the last run's failure record, when the run failed
    for run in sorted(glob.glob(f"{run_dir}/live/epoch_*/run")):
        last = None
        for md in sorted(glob.glob(f"{run}/missions/m*")):
            try:
                st = _json(f"{md}/records.json")["state"]
                res = _json(f"{md}/result.json").get("result") or {}
            except (OSError, ValueError, KeyError):
                continue
            out.append((st, res))
            last = st
        try:
            top = _json(f"{run}/result.json")
        except (OSError, ValueError):
            continue
        if top.get("success") is False:
            retries = top.get("docking_retries") or []
            fresh = last is None or retries != (last.get("docking_retries") or [])
            tail = ({"docking_retries": retries if fresh else [],
                     "delivery_docking": (top.get("delivery_docking") or {}) if fresh else {}},
                    {"outcome": "session_failed", "reason": top.get("failure_reason"), "ended_s": None}, fresh)
    if tail is not None:
        st, res, fresh = tail
        # missions accepted but with no folder were cut; without the probe's count, a run record unlike the last
        # mission's stands for one (the earlier rule)
        missing = max(0, acked - len(out)) if acked is not None else int(fresh)
        for k in range(missing):
            out.append((st if k == 0 else {"docking_retries": []}, dict(res)))
    return out


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
    transport_recovery_after_docking, and the hold exits taken (uncounted docking retries ``h``)."""
    out = {"missions": 0, "done": 0, "recovery_after_docking": 0, "hold_exits": 0, "session_failed": 0}
    for st, res in _session_states(run_dir):
        out["missions"] += 1
        out["session_failed"] += res.get("outcome") == "session_failed"
        out["done"] += res.get("outcome") == "done"
        out["recovery_after_docking"] += res.get("reason") == "transport_recovery_after_docking"
        out["hold_exits"] += sum(1 for e in st.get("docking_retries") or [] if e.get("trigger") == "h")
    return out


def _mission_clean(st, res) -> bool:
    """Done, and every counted re-approach in it arrived (a docking round in its window) and was planned."""
    if res.get("outcome") != "done":
        return False
    counted = [e for e in st.get("docking_retries") or [] if e.get("counted")]
    rounds = [float(r["time_s"]) for r in (st.get("delivery_docking") or {}).get("rounds") or []
              if r.get("time_s") is not None]
    for k, ev in enumerate(counted):
        plan = ev.get("plan") or {}
        if plan.get("refused") or plan.get("length_m") is None:
            return False
        end = counted[k + 1]["time_s"] if k + 1 < len(counted) else res.get("ended_s")
        if not any(t > ev["time_s"] + 1e-9 and (end is None or t <= float(end) + 1e-9) for t in rounds):
            return False
    return True


def session_score(run_dir, scheduled=3) -> int:
    """Re-approach hold plan 2a (V5) primary: of the session's scheduled missions, how many completed with no
    re-approach stall. A mission not reached, cut or failed scores 0, and so does one whose stall recovered."""
    return min(int(scheduled), sum(_mission_clean(st, res) for st, res in _session_states(run_dir)))


def sign_test_p(differences) -> float:
    """Exact one-sided sign test: P(at least the observed positives | the nonzero differences split as a fair coin)."""
    from math import comb

    pos = sum(1 for d in differences if d > 0)
    n = sum(1 for d in differences if d != 0)
    if n == 0:
        return 1.0
    return sum(comb(n, k) for k in range(pos, n + 1)) / 2 ** n


def collect(live_dir, prefix, scene, seeds, arms=("main", "fix")) -> list:
    """The pairs file's rows from the V4 run folders <live_dir>/<prefix>_<arm>_<scene>_s<seed>, the two arms read
    as "main" and "fix" (the baseline first), each with its session's mission outcomes."""
    base, fix = arms
    rows = []
    for s in seeds:
        b, f = f"{live_dir}/{prefix}_{base}_{scene}_s{s}", f"{live_dir}/{prefix}_{fix}_{scene}_s{s}"
        rows.append({"seed": s, "main": session_counts(b), "fix": session_counts(f),
                     "missions": {"main": session_missions(b), "fix": session_missions(f)}})
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
    args = parser.parse_args()
    if args.collect:
        rows = collect(*args.collect, range(args.seeds), arms=tuple(args.arms))
        if args.out:
            with open(args.out, "w") as handle:
                json.dump(rows, handle, indent=1)
    else:
        with open(args.pairs) as handle:
            rows = json.load(handle)
    print(json.dumps({"rows": rows, "judgement": judge(pairs_from_rows(rows)), "missions": mission_totals(rows)},
                     indent=2, default=float))


if __name__ == "__main__":
    main()
