#!/usr/bin/env python3
"""The observation move's paired comparison (docs/plans/2026-11-16-reloc-observe-move.md 2): one batch with the move
on; each session's arm A is made from its own static stage.

- A session the static stage confirmed never moved: it is arm A's run as well, its ``reloc_judge`` outcome for both.
- A session the static stage did not confirm (``start_reloc.static_outcome`` recorded): arm A ended there
  (needs_operator / failed, no bridge started -- no unattended success); arm B is the session's own outcome.

Rescue b = A failed and B succeeded (``reloc_judge`` success: the matcher's confirmation, the success pose class and
the hold); worse c = A succeeded and B did not (0 by construction). The one-sided exact McNemar p = P(X >= b),
X ~ Bin(b + c, 1/2), only says the rescues are not chance; the size is b / n_static_failed (two-sided 95 %
Clopper-Pearson) and the success rate A -> B. Safety: B's wrong confirmations (``reloc_judge`` pose class wrong).

    reloc_move_compare.py --sessions DIR --starts val_s5.json --code-root SNAP --image IMG [--out compare.json]
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import reloc_judge as J  # noqa: E402


def binom_tail_ge(b: int, n: int) -> float:
    """P(X >= b) for X ~ Bin(n, 1/2)."""
    if n <= 0:
        return 1.0
    return sum(math.comb(n, k) for k in range(b, n + 1)) / 2 ** n


def _cdf(k: int, n: int, p: float) -> float:
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k + 1))


def cp_two_sided(k: int, n: int, alpha: float = 0.05) -> tuple | None:
    """The two-sided Clopper-Pearson interval of k in n (bisection on the binomial tails)."""
    if n <= 0:
        return None

    def solve(f) -> float:
        lo, hi = 0.0, 1.0
        for _ in range(80):
            mid = (lo + hi) / 2
            if f(mid):
                hi = mid
            else:
                lo = mid
        return (lo + hi) / 2

    lower = 0.0 if k == 0 else solve(lambda p: 1 - _cdf(k - 1, n, p) >= alpha / 2)
    upper = 1.0 if k == n else solve(lambda p: _cdf(k, n, p) <= alpha / 2)
    return (lower, upper)


def _move_times(rec: dict) -> dict | None:
    """The moved session's added time and each place's scan sets and deadline: the moves' own wall time plus the
    re-match attempts' (from the attempts recorded with a move phase)."""
    moves = rec.get("moves") or []
    if not moves:
        return None
    late = lambda a: a.get("reason") in ("deadline", "timeout") or a.get("status") == "timeout"  # noqa: E731
    places = {}
    for a in rec.get("attempts") or []:
        ph = a.get("phase")
        if ph in (None, "static"):
            continue
        p = places.setdefault(ph, {"scan_sets": 0, "deadline_hit": False, "wall_s": 0.0})
        p["scan_sets"] += 1
        p["deadline_hit"] = p["deadline_hit"] or late(a)
        p["wall_s"] += float(a.get("wall_s") or 0.0)
    added = sum(float(m.get("wall_s") or 0.0) for m in moves) + sum(p["wall_s"] for p in places.values())
    return {"added_s": round(added, 3), "places": places,
            "blocked": [m["dir"] for m in moves if m.get("blocked")],
            "stopped": any(m.get("stopped") for m in moves),
            "contact": any(m.get("move_contact") for m in moves)}


def pair(row: dict, rec: dict | None) -> dict:
    """One session's arms: (a_success, b_success, moved, b_wrong)."""
    b_ok = row.get("outcome") == "success"
    static = (rec or {}).get("static_outcome")
    moved = static is not None
    a_ok = False if moved else b_ok
    return {"id": row["id"], "a_success": a_ok, "b_success": b_ok, "moved": moved, "static_outcome": static,
            "b_outcome": row.get("outcome"), "b_wrong": row.get("pose_class") == "wrong",
            "times": _move_times(rec or {}) if moved else None}


def compare(rows: list, recs: dict) -> dict:
    valid = [r for r in rows if r.get("class") == "valid" and r.get("outcome") != "not_run"]
    # every valid session must show it ran with the move on (its record's "moves", empty when the static stage
    # confirmed): a missing record would silently move a session into arm A's success (Codex code review P2)
    # a session whose runner ended before the relocalization (no start_reloc at all: an Isaac start-up failure) has
    # no static stage -- both arms fail it alike, it is kept as such (not a pair that can differ) and listed
    no_reloc = [r["id"] for r in valid if recs.get(r["id"]) is None]
    missing = [r["id"] for r in valid if recs.get(r["id"]) is not None
               and (not isinstance(recs[r["id"]], dict) or "moves" not in recs[r["id"]])]
    if missing:
        raise ValueError(f"valid sessions without a move-on start_reloc record: {missing}")
    odd = [r["id"] for r in valid if r["id"] not in no_reloc
           and bool(recs[r["id"]]["moves"]) != (recs[r["id"]].get("static_outcome") is not None)]
    if odd:
        raise ValueError(f"sessions whose moves and static_outcome disagree: {odd}")
    pairs = [pair(r, recs.get(r["id"])) for r in valid]
    b = sum(1 for p in pairs if p["b_success"] and not p["a_success"])
    c = sum(1 for p in pairs if p["a_success"] and not p["b_success"])
    n_fail = sum(1 for p in pairs if p["moved"] or not p["a_success"])
    n_static_failed = sum(1 for p in pairs if p["moved"])
    added = [p["times"]["added_s"] for p in pairs if p["times"]]
    wrong = sum(1 for p in pairs if p["b_wrong"])
    both_blocked = sum(1 for p in pairs if p["times"] and len(p["times"]["blocked"]) == 2)
    n = len(pairs)
    return {
        "valid": n, "no_reloc": no_reloc, "static_failed": n_static_failed, "a_failed": n_fail,
        "a_success": sum(p["a_success"] for p in pairs), "b_success": sum(p["b_success"] for p in pairs),
        "rescued": b, "worse": c, "mcnemar_p_one_sided": binom_tail_ge(b, b + c),
        "rescue_rate": None if not n_static_failed else b / n_static_failed,
        "rescue_rate_cp95": cp_two_sided(b, n_static_failed),
        "b_wrong": wrong, "both_blocked": both_blocked,
        "added_s_median": statistics.median(added) if added else None, "added_s_max": max(added) if added else None,
        # the pre-registered rules (plan 2): no wrong confirmation, and 0.6 m only after b >= 3 with none wrong
        "rejected_for_safety": wrong > 0, "structural_defect": c > 0, "run_0_6": b >= 3 and wrong == 0,
        "pairs": pairs,
    }


def _records(root: Path, starts: list) -> dict:
    out = {}
    for s in starts:
        res = J._load(Path(root) / s["id"] / "epoch_1" / "run" / "result.json")
        out[s["id"]] = (res or {}).get("start_reloc")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sessions", type=Path, required=True)
    ap.add_argument("--starts", type=Path, required=True)
    ap.add_argument("--code-root", type=Path, required=True)
    ap.add_argument("--image", required=True)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    starts = json.loads(a.starts.read_text())["poses"]  # scan_capture_poses/1, as reloc_judge reads it
    fp = J.fingerprint(a.code_root, a.image)
    rows = J.judge_sessions(a.sessions, starts, fp, code_root=a.code_root)
    out = compare(rows, _records(a.sessions, starts))
    text = json.dumps(out, indent=1, sort_keys=True)
    if a.out is not None:
        a.out.write_text(text + "\n")
    summary = {k: v for k, v in out.items() if k != "pairs"}
    print(json.dumps(summary, indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
