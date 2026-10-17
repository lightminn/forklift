"""Run dumped calls again and compare (real-time plan S0, docs/plans/2026-10-17-realtime-sim.md).

``forklift_core.perf.dumped`` writes, for chosen calls of a hot method, the receiver, arguments and keyword
arguments before and after the call (one object graph each) and the result. This tool calls the same method
again under each implementation named (``forklift_core.kernels``) and reports, per function and implementation:

- ``same``: the result and the receiver and arguments after the call are bit for bit the dumped ones (arrays
  by dtype, shape and bytes, so NaN payloads and -0.0 count; floats and NumPy scalars by type and bits; dicts in
  order; sets as sets);
- the first differences found, up to ``--show`` (display only: the judgement always compares everything);
- the wall time per call (median and p90), after one warm-up call per implementation, with the implementations'
  order rotated from one repeat to the next.

Two kinds of check:

- single calls (every dump): each starts from the dumped state before the call -- it does not test what an
  implementation carries from one call to the next;
- streams (``--streams``, dumps made with ``FORKLIFT_PERF_DUMP_STREAM``): each implementation starts from the
  first call's state and then carries its own receiver through the following calls, which must be consecutive.

    python tools/perf_bench.py DUMP_DIR [--impl reference numpy] [--repeat 3] [--streams] [--json OUT]
"""

from __future__ import annotations

import argparse
import copy
import dataclasses
import gzip
import json
import math
import os
import pickle
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sim/isaac"))

os.environ.pop("FORKLIFT_PERF_DUMP", None)  # a bench run must not dump again
os.environ.pop("FORKLIFT_KERNELS_DUAL", None)

# The bit-for-bit comparison, shared with the dual run.
from forklift_core.perf import differences  # noqa: E402

METHODS = {"check_arc": "check_arc", "check_path": "check_path", "grid_snapshot": "snapshot",
           "shadow_apply": "apply"}


def load(path: Path) -> dict:
    with gzip.open(path, "rb") as f:
        return pickle.load(f)


def call(dump: dict, state, impl: str):
    """One call under ``impl`` on (receiver, args, kwargs) ``state`` (changed in place); (seconds, result)."""
    from forklift_core import kernels

    kernels.set_mode(impl)
    receiver, args, kwargs = state
    method = getattr(type(receiver), METHODS[dump["name"]])
    method = getattr(method, "__wrapped__", method)
    start = time.perf_counter()
    result = method(receiver, *args, **kwargs)
    return time.perf_counter() - start, result


def start_state(dump: dict):
    """The state before the call, one graph (a pure call's state after it is the same)."""
    return copy.deepcopy(dump["before"] if dump["before"] is not None else dump["after"])


def judge(dump, result, state, show):
    diffs = differences(dump["result"], result, "result", limit=show)
    return differences(dump["after"], state, "after", diffs, limit=show)


def single(files, impls, repeat, show):
    entry = {impl: {"n": 0, "same": 0, "times": [], "diffs": []} for impl in impls}
    warmed = False
    for path in files:
        dump = load(path)
        if not warmed:
            for impl in impls:
                call(dump, start_state(dump), impl)
            warmed = True
        times = {impl: [] for impl in impls}
        for r in range(max(1, repeat)):
            order = impls[r % len(impls):] + impls[:r % len(impls)]
            for impl in order:
                state = start_state(dump)
                dt, result = call(dump, state, impl)
                times[impl].append(dt)
                if r == 0:
                    e = entry[impl]
                    diffs = judge(dump, result, state, show)
                    e["n"] += 1
                    e["same"] += not diffs
                    if diffs and len(e["diffs"]) < show:
                        e["diffs"].append({"dump": path.name, "first": diffs})
        for impl in impls:
            entry[impl]["times"].append(float(np.median(times[impl])))
    return entry


def _reachable(obj, seen=None) -> set:
    """ids of the mutable objects reachable from obj (containers, arrays, objects with attributes)."""
    seen = set() if seen is None else seen
    if isinstance(obj, (str, bytes, int, float, bool, type(None), np.generic)) or id(obj) in seen:
        return seen
    seen.add(id(obj))
    if isinstance(obj, dict):
        for k, v in obj.items():
            _reachable(k, seen), _reachable(v, seen)
    elif isinstance(obj, (list, tuple, set, frozenset, deque)):
        for v in obj:
            _reachable(v, seen)
    elif isinstance(obj, np.ndarray):
        if obj.base is not None:
            _reachable(obj.base, seen)
    elif dataclasses.is_dataclass(obj):
        for f in dataclasses.fields(obj):
            _reachable(getattr(obj, f.name), seen)
    elif hasattr(obj, "__dict__"):
        _reachable(vars(obj), seen)
    return seen


def shares_state(dump: dict) -> bool:
    """Whether a call's arguments reach an object its receiver also reaches (a stream cannot carry that
    relation from one call to the next: such a stream is reported, not compared)."""
    receiver, args, kwargs = dump["before"] if dump["before"] is not None else dump["after"]
    return bool(_reachable(receiver) & _reachable((args, kwargs)) - {id(args), id(kwargs)})


def streams(files, impls, show):
    """Each stream's calls in order, each implementation carrying its own receiver from the first call (the
    first call's whole state as one graph; later calls take their dumped arguments, so a stream whose
    arguments share objects with its receiver is not compared)."""
    groups: dict = {}
    for path in files:
        dump = load(path)
        if dump.get("stream") is None:
            continue
        pid, rid, seq = dump["stream"]
        groups.setdefault((pid, rid), []).append((seq, dump))
    entry = {impl: {"streams": 0, "calls": 0, "same": 0, "broken_at": [], "diffs": [], "shared": 0}
             for impl in impls}
    for key, calls in groups.items():
        calls.sort(key=lambda c: c[0])
        if [c[0] for c in calls] != list(range(len(calls))):
            raise SystemExit(f"stream {key} is not consecutive: {[c[0] for c in calls][:10]}")
        if any(shares_state(dump) for _, dump in calls):
            for impl in impls:
                entry[impl]["shared"] += 1
            continue
        for impl in impls:
            e = entry[impl]
            e["streams"] += 1
            receiver = None
            for seq, dump in calls:
                # The first call's state as one graph; then this call's arguments as dumped, with the receiver
                # this implementation carried.
                state = start_state(dump)
                if receiver is None:
                    receiver = state[0]
                state = (receiver, state[1], state[2])
                _, result = call(dump, state, impl)
                diffs = judge(dump, result, state, show)
                e["calls"] += 1
                e["same"] += not diffs
                if diffs:
                    e["broken_at"].append([list(key), seq])
                    if len(e["diffs"]) < show:
                        e["diffs"].append({"stream": list(key), "seq": seq, "first": diffs})
                    break
    return entry


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("dumps", type=Path, help="FORKLIFT_PERF_DUMP directory (one folder per function)")
    p.add_argument("--impl", nargs="+", default=["reference"])
    p.add_argument("--only", nargs="*", default=None, help="function names (default: every folder)")
    p.add_argument("--repeat", type=int, default=3)
    p.add_argument("--limit", type=int, default=0, help="at most this many single-call dumps per function (0: all)")
    p.add_argument("--streams", action="store_true", help="also check the stream dumps call after call")
    p.add_argument("--show", type=int, default=5)
    p.add_argument("--json", type=Path)
    args = p.parse_args()
    report = {}
    for folder in sorted(d for d in args.dumps.iterdir() if d.is_dir()):
        if args.only and folder.name not in args.only:
            continue
        files = sorted(folder.glob("*.pkl.gz"))
        chosen = files[:: max(1, math.ceil(len(files) / args.limit))] if args.limit else files
        entry = single(chosen, args.impl, args.repeat, args.show)
        for e in entry.values():
            t = np.asarray(e.pop("times")) * 1000
            e["ms_median"] = float(np.median(t)) if len(t) else None
            e["ms_p90"] = float(np.percentile(t, 90)) if len(t) else None
            e["ms_total"] = float(t.sum()) if len(t) else None
        report[folder.name] = {"single": entry}
        line = "  ".join(f"{impl}: {e['same']}/{e['n']} same, {e['ms_median']:.3f} ms med, {e['ms_p90']:.3f} p90"
                         for impl, e in entry.items() if e["n"])
        print(f"{folder.name:14s} {line}", flush=True)
        if args.streams:
            st = streams(files, args.impl, args.show)
            report[folder.name]["streams"] = st
            if any(e["streams"] or e["shared"] for e in st.values()):
                print(f"{'':14s} streams: " + "  ".join(
                    f"{impl}: {e['same']}/{e['calls']} calls same over {e['streams']} streams"
                    f" ({e['shared']} not compared: arguments share the receiver's objects)"
                    for impl, e in st.items()), flush=True)
    if args.json:
        args.json.write_text(json.dumps(report, indent=1, default=str))


if __name__ == "__main__":
    main()
