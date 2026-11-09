"""Real-time plan 5.27 1c-0: the shadow apply's array path (FORKLIFT_SHADOW_KERNEL) on recorded calls.

For every ``shadow_apply`` dump (FORKLIFT_PERF_DUMP/shadow_apply/*.pkl.gz) the call is replayed from its own
``before`` copy, with the switch off and on, in each kernel mode asked for:
- the self-reproduction: the switch-off replay equals the dump's ``after`` receiver and ``result``;
- the equality: the switch-on replay equals the switch-off one -- results, receivers and the memory's key list
  (order and element types; differences() does not look at key types, plan 5.27 Codex P2-2);
- each call's time off and on (off and on alternate per dump; one untimed replay per mode and switch first), whether
  the array path ran or the exact-float gate sent the call to the original, whether the memory moved and whether
  depth support came with it.

The expected saving re-weights the move calls to a given share (the sample's stratified collection holds more of
them than a run, plan 5.27) and scales the per-call saving onto the live denominator. Prediction only.

    PYTHONPATH=SNAP/src python g3_1c0_analyze.py DUMP_DIR [DUMP_DIR ...] --json out.json [--modes numba numpy]
"""

from __future__ import annotations

import argparse
import copy
import gzip
import json
import os
import pickle
import sys
import time
from pathlib import Path

if "forklift_core.perf" not in sys.modules:
    os.environ.pop("FORKLIFT_PERF_DUMP", None)  # replaying must not dump again (read when perf is imported)
import numpy as np  # noqa: E402

from forklift_core import kernels  # noqa: E402
from forklift_core.control import shadow_memory as sm  # noqa: E402
from forklift_core.perf import differences  # noqa: E402

APPLY = sm.ShadowMemory.apply
while hasattr(APPLY, "__wrapped__"):  # the dump and dual decorators off: the method itself
    APPLY = APPLY.__wrapped__
LIVE_MS_PER_SIM_S = 54.6  # line_m AUTO shadow.apply (docs/validation/2026-11-09-g2-measure-results.md 5)
RAN = {"arrays": 0, "move_s": 0.0}
_ARRAYS = sm.ShadowMemory._apply_arrays


def _counting(real):
    def inner(self, *a, **k):
        RAN["arrays"] += 1
        return real(self, *a, **k)
    return inner


def _timed(real):
    """The memory move's own time (Codex 1c-0 review P3: a move call's whole time is not the move's share)."""
    def inner(self, *a, **k):
        if RAN.get("in_move"):  # _move_arrays falling back to _move: counted once, by the outer call
            return real(self, *a, **k)
        RAN["in_move"] = True
        start = time.perf_counter()
        try:
            return real(self, *a, **k)
        finally:
            RAN["move_s"] += time.perf_counter() - start
            RAN["in_move"] = False
    return inner


def load(path: Path) -> dict:
    with gzip.open(path, "rb") as f:
        return pickle.load(f)


def replay(record, mode: str, on: bool):
    kernels.set_mode(mode)
    sm.SHADOW_KERNEL = on
    receiver, args, kwargs = copy.deepcopy(record["before"])
    ran, moved = RAN["arrays"], RAN["move_s"]
    start = time.perf_counter()
    out = APPLY(receiver, *args, **kwargs)
    return receiver, out, time.perf_counter() - start, RAN["arrays"] > ran, RAN["move_s"] - moved


def same(a, b) -> list:
    return (differences(a[1], b[1], limit=3) + differences(a[0], b[0], limit=3)
            + differences(list(a[0].memory), list(b[0].memory), limit=3))


def kind(record) -> dict:
    receiver, args, kwargs = record["before"]
    snap = args[0]
    moved = receiver.correction is not None and bool(receiver.memory) and \
        tuple(float(v) for v in snap.correction) != receiver.correction
    extra = kwargs.get("extra_support") or (args[3] if len(args) > 3 else None)
    return {"moved": bool(moved), "extra_support": bool(extra), "memory_cells": len(receiver.memory)}


def analyse(paths, modes) -> dict:
    """The replays below set the switch and the kernel mode; both, and the counting hook, are put back after."""
    saved = (sm.SHADOW_KERNEL, kernels.mode())
    cls = sm.ShadowMemory
    hooks = (cls._apply_arrays, cls._move, cls._move_arrays)  # what is there now, put back after
    cls._apply_arrays, cls._move, cls._move_arrays = _counting(hooks[0]), _timed(hooks[1]), _timed(hooks[2])
    try:
        return _analyse(paths, modes)
    finally:
        cls._apply_arrays, cls._move, cls._move_arrays = hooks
        sm.SHADOW_KERNEL = saved[0]
        kernels.set_mode(saved[1])


def _analyse(paths, modes) -> dict:
    if "numba" in modes:
        kernels.set_mode("numba")
        kernels.warm_up()
    if paths:
        first = load(paths[0])
        for mode in modes:
            replay(first, mode, False)
            replay(first, mode, True)
    rows = []
    for p in paths:
        rec = load(p)
        row = {"file": str(p), "call": rec["call"], **kind(rec)}
        for mode in modes:
            off = replay(rec, mode, False)
            on = replay(rec, mode, True)
            row[mode] = {"self_reproduced": same(off[:2], (rec["after"][0], rec["result"])) == [],
                         "equal": same(off[:2], on[:2]) == [], "off_s": off[2], "on_s": on[2], "arrays_ran": on[3],
                         "off_move_s": off[4], "on_move_s": on[4]}
        rows.append(row)
    summary = {"dumps": len(rows), "moved": sum(r["moved"] for r in rows),
               "extra_support": sum(r["extra_support"] for r in rows),
               "memory_cells": _q([r["memory_cells"] for r in rows])}
    for mode in modes:
        summary[mode] = {"self_reproduced": sum(r[mode]["self_reproduced"] for r in rows),
                         "equal": sum(r[mode]["equal"] for r in rows),
                         "arrays_ran": sum(r[mode]["arrays_ran"] for r in rows),
                         "off_ms": _q([1e3 * r[mode]["off_s"] for r in rows]),
                         "on_ms": _q([1e3 * r[mode]["on_s"] for r in rows]),
                         "off_ms_moved": _q([1e3 * r[mode]["off_s"] for r in rows if r["moved"]]),
                         "on_ms_moved": _q([1e3 * r[mode]["on_s"] for r in rows if r["moved"]]),
                         "off_ms_still": _q([1e3 * r[mode]["off_s"] for r in rows if not r["moved"]]),
                         "on_ms_still": _q([1e3 * r[mode]["on_s"] for r in rows if not r["moved"]]),
                         "off_ms_move_only": _q([1e3 * r[mode]["off_move_s"] for r in rows if r["moved"]]),
                         "on_ms_move_only": _q([1e3 * r[mode]["on_move_s"] for r in rows if r["moved"]])}
    return {"summary": summary, "rows": rows}


def expected(summary: dict, mode: str, move_share: float) -> dict:
    """Per-call saving with the move calls weighted to ``move_share``, scaled onto the live denominator."""
    s = summary[mode]

    def mean(key):
        return s[key]["mean"] if s[key] else float("nan")
    off = (1 - move_share) * mean("off_ms_still") + move_share * mean("off_ms_moved")
    on = (1 - move_share) * mean("on_ms_still") + move_share * mean("on_ms_moved")
    move_only = move_share * mean("off_ms_move_only")
    return {"move_share": move_share, "off_ms": off, "on_ms": on, "saving_share": 1 - on / off,
            "expected_ms_per_sim_s": -(1 - on / off) * LIVE_MS_PER_SIM_S,
            "off_move_share_of_apply": move_only / off}  # plan 5.27's 5 % rule reads the move's own time


def _q(v) -> dict:
    if not v:
        return {}
    a = np.asarray(v, dtype=float)
    return {"n": int(a.size), "mean": float(a.mean()), "p50": float(np.median(a)), "p90": float(np.percentile(a, 90)),
            "p99": float(np.percentile(a, 99)), "max": float(a.max())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", type=Path, nargs="+", help="FORKLIFT_PERF_DUMP folders (their shadow_apply/ is read)")
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--modes", nargs="+", default=["numba", "numpy"])
    ap.add_argument("--move-share", type=float, default=0.04, help="the run's share of calls that move the memory")
    ap.add_argument("--limit", type=int, default=0, help="0: every dump")
    ap.add_argument("--source-commit", default=None)
    a = ap.parse_args()
    missing = [str(d) for d in a.dirs if not any((d / "shadow_apply").glob("*.pkl.gz"))]
    if missing:
        print(f"no shadow_apply dumps in {missing}", file=sys.stderr)
        return 2
    paths = sorted(p for d in a.dirs for p in (d / "shadow_apply").glob("*.pkl.gz"))
    if a.limit:
        paths = paths[: a.limit]
    out = analyse(paths, a.modes)
    out["expected"] = {m: expected(out["summary"], m, a.move_share) for m in a.modes}
    import importlib.metadata as md

    def ver(name):
        try:
            return md.version(name)
        except md.PackageNotFoundError:
            return None
    out["conditions"] = {"source_commit": a.source_commit, "numpy": ver("numpy"), "numba": ver("numba"),
                         "modes": a.modes, "live_ms_per_sim_s": LIVE_MS_PER_SIM_S}
    a.json.write_text(json.dumps(out, indent=1, default=str))
    print(json.dumps({"summary": out["summary"], "expected": out["expected"]}, indent=1, default=str))
    s = out["summary"]
    return 0 if all(s[m]["self_reproduced"] == s[m]["equal"] == s["dumps"] for m in a.modes) else 1


if __name__ == "__main__":
    sys.exit(main())
