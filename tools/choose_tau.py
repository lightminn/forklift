#!/usr/bin/env python3
"""Choose the learned proposer's identity threshold tau on dev (stage 3 plan, task 5; design §7.5, §3.4).

Reads the metrics.json of learned-fit runs (one per tau, ``learned.tau`` inside) and, per tau, the observed rates
of the gates the plan names: miss, negative_fp, extra, deferred_unambiguous (the console request rate) and
target_error. Every population's gate is held to its own bound (design §3.4 bounds are per shape; Codex review 1:
pooling let EPAL 6 miss 8 % pass beside two clean populations), and a population:gate cell named in any run
but without samples in this one (or a gate no population reports) is missing, never a pass. The chosen tau is
the lowest one with every cell within its bound and nothing missing; when none is, the one with the fewest
missing cells and then the least summed excess, with its shortfalls recorded. Low-score proposals are never dropped to reach a bound -- they stay unconfirmed candidates (plan v3).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

GATES = ("miss", "negative_fp", "extra", "deferred_unambiguous", "target_error")


def rates(metrics: dict) -> dict:
    """Pooled rates per gate (for the table only; the choice reads the cells)."""
    out = {}
    for gate in GATES:
        cells = [v for k, v in metrics["gates"].items() if k.rsplit(":", 1)[-1] == gate and v.get("n")]
        n, failures = sum(c["n"] for c in cells), sum(c["failures"] for c in cells)
        bound = cells[0]["bound"] if cells else None
        out[gate] = {"n": n, "failures": failures, "rate": failures / n if n else None, "bound": bound}
    return out


def cells(metrics: dict) -> dict:
    """{population:gate -> rate, bound} for the named gates with samples."""
    return {k: {"n": v["n"], "rate": v["failures"] / v["n"], "bound": v["bound"]}
            for k, v in metrics["gates"].items() if k.rsplit(":", 1)[-1] in GATES and v.get("n")}


def choose(runs: list) -> dict:
    """runs: [(tau, metrics)] -> {"table", "chosen", "met", "shortfalls", "missing"}."""
    # every population:gate cell named in any run is required in each (Codex re-review 1: a population's gate at
    # n = 0 must not be stood in for by another population's)
    required = sorted({k for _, m in runs for k in m["gates"] if k.rsplit(":", 1)[-1] in GATES})
    table = []
    for tau, metrics in sorted(runs, key=lambda r: r[0]):
        c = cells(metrics)
        excess = {k: v["rate"] - v["bound"] for k, v in c.items() if v["rate"] > v["bound"]}
        missing = ([k for k in required if k not in c]
                   + [g for g in GATES if not any(k.rsplit(":", 1)[-1] == g for k in required)])
        table.append({"tau": tau, "rates": rates(metrics), "cells": c, "excess": excess, "missing": missing,
                      "total_excess": sum(excess.values())})
    meeting = [row for row in table if not row["excess"] and not row["missing"]]
    best = meeting[0] if meeting else min(table, key=lambda row: (len(row["missing"]), row["total_excess"], row["tau"]))
    return {"table": table, "chosen": best["tau"], "met": bool(meeting), "shortfalls": best["excess"],
            "missing": best["missing"]}


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
    print(json.dumps({k: result[k] for k in ("chosen", "met", "shortfalls", "missing")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
