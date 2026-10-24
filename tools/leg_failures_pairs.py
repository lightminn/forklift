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
    nonzero = [float(d) for d in differences if d != 0]
    if not nonzero:
        return 1.0
    ranks = _average_ranks([abs(d) for d in nonzero])
    observed = sum(r for r, d in zip(ranks, nonzero) if d > 0)
    hits = 0
    for signs in itertools.product((0, 1), repeat=len(ranks)):
        if sum(r for r, s in zip(ranks, signs) if s) >= observed - 1e-9:
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pairs", help="JSON list of {seed, main: [completed, attempted, refusals], fix: [...]}")
    args = parser.parse_args()
    with open(args.pairs) as handle:
        rows = json.load(handle)

    def rate(arm):
        completed, attempted, _ = arm
        return None if attempted == 0 else completed / attempted

    pairs = [(rate(r["main"]), rate(r["fix"]), r["main"][2], r["fix"][2]) for r in rows]
    print(json.dumps(judge(pairs), indent=2))


if __name__ == "__main__":
    main()
