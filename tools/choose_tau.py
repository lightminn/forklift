#!/usr/bin/env python3
"""Choose the learned proposer's identity threshold tau on dev (stage 3 plan, task 5; design §7.5, §3.4).

Reads the metrics.json of learned-fit runs (one per tau, ``learned.tau`` inside) and, per tau, the observed rates
of the gates the plan names: miss, negative_fp, extra, deferred_unambiguous (the console request rate) and
target_error, pooled over populations. The chosen tau is the lowest one whose rates are all within the design
bounds; when none is, the one with the least summed excess, with its shortfalls recorded. Low-score proposals are
never dropped to reach a bound -- they stay unconfirmed candidates (plan v3).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

GATES = ("miss", "negative_fp", "extra", "deferred_unambiguous", "target_error")


def rates(metrics: dict) -> dict:
    out = {}
    for gate in GATES:
        cells = [v for k, v in metrics["gates"].items() if k.rsplit(":", 1)[-1] == gate and v.get("n")]
        n, failures = sum(c["n"] for c in cells), sum(c["failures"] for c in cells)
        bound = cells[0]["bound"] if cells else None
        out[gate] = {"n": n, "failures": failures, "rate": failures / n if n else None, "bound": bound}
    return out


def choose(runs: list) -> dict:
    """runs: [(tau, metrics)] -> {"table": [...], "chosen": tau, "met": bool, "shortfalls": {...}}."""
    table = []
    for tau, metrics in sorted(runs, key=lambda r: r[0]):
        r = rates(metrics)
        excess = {g: v["rate"] - v["bound"] for g, v in r.items() if v["rate"] is not None and v["rate"] > v["bound"]}
        table.append({"tau": tau, "rates": r, "excess": excess, "total_excess": sum(excess.values())})
    meeting = [row for row in table if not row["excess"]]
    best = meeting[0] if meeting else min(table, key=lambda row: (row["total_excess"], row["tau"]))
    return {"table": table, "chosen": best["tau"], "met": bool(meeting), "shortfalls": best["excess"]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("runs", nargs="+", type=Path, help="learned-fit output directories (metrics.json inside)")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    runs = []
    for d in args.runs:
        m = json.loads((d / "metrics.json").read_text())
        runs.append((float(m["learned"]["tau"]), m))
    result = choose(runs)
    args.out.write_text(json.dumps(result, indent=1))
    print(json.dumps({"chosen": result["chosen"], "met": result["met"], "shortfalls": result["shortfalls"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
