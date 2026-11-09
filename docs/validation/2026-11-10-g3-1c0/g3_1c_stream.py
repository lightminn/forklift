"""Real-time plan 5.27: the shadow-level event replay of 1c.

The shadow memory's state changes only inside ``ShadowMemory.apply`` and the call's arguments are all its inputs (the
depth support is evaluated just before the call), so every apply call of one receiver, in order, decides its state.
The recording is the dump module's stream mode (``FORKLIFT_PERF_DUMP_STREAM=shadow_apply=<large>``, every other
dump off) with its completeness marker (``stream_totals_<pid>.json``). For every receiver:

1. completeness and continuity: the records are seq 0, 1, 2, ... without a gap, as many as the marker's calls, the
   limit not reached; record n's receiver before the call equals record n-1's after it. Anything else: the stream is
   invalid and the replay stops there.
2. the reference first: from record 0's receiver, the switch off (the recording's kernel mode), each call carried on
   from the replay's own receiver; every result and receiver equals the recording.
3. the candidates: the same start, the switch on, in each kernel mode asked; every call equals step 2 (results,
   receivers and the memory's key list with its element types).

Files are read one at a time (a call's arrays are about 1.6 MB). Reported: per receiver the counts above, the
continuous replay's total and per-kind time per arm, and the coverage of the cases plan 5.27 lists.

    PYTHONPATH=SNAP/src python g3_1c_stream.py DUMP_DIR [DUMP_DIR ...] --json out.json [--modes numba numpy]
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
from collections import Counter, defaultdict
from pathlib import Path

os.environ.pop("FORKLIFT_PERF_DUMP", None)  # replaying must not dump again
import numpy as np  # noqa: E402

from forklift_core import kernels  # noqa: E402
from forklift_core.control import shadow_memory as sm  # noqa: E402
from forklift_core.control.drive_permission import RETAINED  # noqa: E402
from forklift_core.perception.obstacle_grid import FREE, UNKNOWN  # noqa: E402
from forklift_core.perf import differences  # noqa: E402

APPLY = sm.ShadowMemory.apply
while hasattr(APPLY, "__wrapped__"):
    APPLY = APPLY.__wrapped__
_ARRAYS = sm.ShadowMemory._apply_arrays
_MOVE_ARRAYS = sm.ShadowMemory._move_arrays
RAN = Counter()


def _count(name, real):
    def inner(self, *a, **k):
        RAN[name] += 1
        return real(self, *a, **k)
    return inner


def load(path: Path) -> dict:
    with gzip.open(path, "rb") as f:
        return pickle.load(f)


def same(a, b) -> list:
    return (differences(a[1], b[1], limit=3) + differences(a[0], b[0], limit=3)
            + differences(list(a[0].memory), list(b[0].memory), limit=3))


def call(receiver, record, mode: str, on: bool):
    kernels.set_mode(mode)
    sm.SHADOW_KERNEL = on
    _, args, kwargs = copy.deepcopy(record["before"])
    start = time.perf_counter()
    out = APPLY(receiver, *args, **kwargs)
    return out, time.perf_counter() - start


def kinds(record, previous) -> list[str]:
    """The cases plan 5.27 asks the replay to have met, for this call."""
    receiver, args, kwargs = record["before"]
    snap, pose, foot = args[0], args[1], args[2]
    extra = kwargs.get("extra_support") or (args[3] if len(args) > 3 else None)
    result = record["result"]
    out = []
    if not receiver.started:
        out.append("startup")
    elif receiver.correction is not None and receiver.memory and \
            tuple(float(v) for v in snap.correction) != receiver.correction:
        out.append("move")
    if extra:
        out.append("extra_support")
    fresh = snap.state == FREE
    if np.any(fresh & ((snap.free_stamp == 0.0) | np.isnan(snap.free_stamp))):
        out.append("zero_or_nan")
    out.append("retained" if (result.state == RETAINED).any() else "no_retained")
    if previous is not None:
        p_snap, p_foot = previous
        if (snap.origin_x_m, snap.origin_y_m) != (p_snap.origin_x_m, p_snap.origin_y_m):
            out.append("origin_shift")
        if foot != p_foot:
            out.append("footprint_change")
    after = record["after"][0]
    nx, ny = snap.state.shape
    oi, oj = int(round(snap.origin_x_m / snap.resolution_m)), int(round(snap.origin_y_m / snap.resolution_m))
    near_edge = [(a - oi, b - oj) for a, b in after.memory if 0 <= a - oi < nx and 0 <= b - oj < ny
                 and snap.state[a - oi, b - oj] == UNKNOWN and min(a - oi, b - oj, nx - 1 - (a - oi), ny - 1 - (b - oj)) < 3]
    if near_edge:
        out.append("candidate_near_edge")
    return out


class Stream:
    def __init__(self, key, modes):
        self.key, self.modes = key, modes
        self.next_seq = 0
        self.last_after = None
        self.previous = None
        self.ref = None
        self.cand = {}
        self.invalid = None
        self.mismatch = []
        self.cover = Counter()
        self.time = defaultdict(lambda: defaultdict(float))  # arm -> kind -> s
        self.calls = 0

    def feed(self, record) -> None:
        if self.invalid:
            return
        seq = record["stream"][2]
        if seq != self.next_seq:
            self.invalid = f"seq {seq} where {self.next_seq} was due"
            return
        if self.last_after is not None and differences(record["before"][0], self.last_after, limit=3):
            self.invalid = f"seq {seq}: the receiver before the call is not the previous call's after it"
            return
        if seq == 0:
            self.ref = copy.deepcopy(record["before"][0])
            self.cand = {m: copy.deepcopy(record["before"][0]) for m in self.modes}
        kind = kinds(record, self.previous)
        self.cover.update(kind)
        bucket = "move" if "move" in kind else "extra_support" if "extra_support" in kind else "still"
        out, dt = call(self.ref, record, self.modes[0], False)
        self.time["off"][bucket] += dt
        bad = same((self.ref, out), (record["after"][0], record["result"]))
        if bad:
            self.mismatch.append({"seq": seq, "arm": "reference", "diff": bad[:3]})
        for m in self.modes:
            arrays = RAN["arrays"]
            c_out, c_dt = call(self.cand[m], record, m, True)
            self.time[f"on_{m}"][bucket] += c_dt
            if RAN["arrays"] == arrays and self.ref.started and "startup" not in kind:
                self.cover[f"gate_original_{m}"] += 1
            bad = same((self.ref, out), (self.cand[m], c_out))
            if bad:
                self.mismatch.append({"seq": seq, "arm": m, "diff": bad[:3]})
        self.last_after = record["after"][0]
        self.previous = (record["before"][1][0], record["before"][1][2])
        self.next_seq += 1
        self.calls += 1


def replay(dirs, modes) -> dict:
    totals = {}
    for d in dirs:
        for f in sorted(Path(d).glob("stream_totals_*.json")):
            for row in json.loads(f.read_text()):
                if row["name"] == "shadow_apply":
                    totals[(row["pid"], row["receiver"])] = row
    files = sorted((p for d in dirs for p in (Path(d) / "shadow_apply").glob("*.pkl.gz")),
                   key=lambda p: (p.parent, int(p.stem.split("_")[0]), int(p.name.split("_")[1].split(".")[0])))
    streams: dict[tuple, Stream] = {}
    unstreamed = 0
    if "numba" in modes:
        kernels.set_mode("numba")
        kernels.warm_up()
    for p in files:
        record = load(p)
        if record.get("stream") is None:
            unstreamed += 1
            continue
        key = (record["stream"][0], record["stream"][1])
        streams.setdefault(key, Stream(key, modes)).feed(record)
    out = {"unstreamed_records": unstreamed, "receivers": []}
    for key, s in streams.items():
        t = totals.get(key)
        complete = None
        if s.invalid is None:
            if t is None:
                complete = "no completeness marker for this receiver"
            elif t["exhausted"] or t["records"] != t["calls"] or s.calls != t["calls"]:
                complete = f"{s.calls} replayed, marker: {t['records']} recorded of {t['calls']} calls" + \
                           (" (limit reached)" if t["exhausted"] else "")
        out["receivers"].append({
            "pid": key[0], "receiver": key[1], "replayed": s.calls, "marker": t,
            "invalid": s.invalid or complete, "mismatches": len(s.mismatch), "first_mismatches": s.mismatch[:5],
            "coverage": dict(s.cover),
            "time_s": {arm: {k: round(v, 6) for k, v in kinds_.items()} for arm, kinds_ in s.time.items()}})
    out["missing_receivers"] = [list(k) for k in totals if k not in streams]
    out["valid"] = bool(streams) and not out["missing_receivers"] and all(
        r["invalid"] is None and r["mismatches"] == 0 for r in out["receivers"])
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", type=Path, nargs="+", help="FORKLIFT_PERF_DUMP folders of stream recordings")
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--modes", nargs="+", default=["numba", "numpy"],
                    help="the first is the recording's kernel mode (the reference replay's)")
    ap.add_argument("--source-commit", default=None)
    a = ap.parse_args()
    saved = (sm.SHADOW_KERNEL, kernels.mode())
    sm.ShadowMemory._apply_arrays = _count("arrays", _ARRAYS)
    sm.ShadowMemory._move_arrays = _count("move_arrays", _MOVE_ARRAYS)
    try:
        out = replay(a.dirs, a.modes)
    finally:
        sm.ShadowMemory._apply_arrays, sm.ShadowMemory._move_arrays = _ARRAYS, _MOVE_ARRAYS
        sm.SHADOW_KERNEL = saved[0]
        kernels.set_mode(saved[1])
    import importlib.metadata as md

    def ver(name):
        try:
            return md.version(name)
        except md.PackageNotFoundError:
            return None
    out["conditions"] = {"source_commit": a.source_commit, "modes": a.modes, "numpy": ver("numpy"),
                         "numba": ver("numba"), "dirs": [str(d) for d in a.dirs]}
    a.json.write_text(json.dumps(out, indent=1, default=str))
    print(json.dumps({k: v for k, v in out.items() if k != "receivers"}, indent=1, default=str))
    for r in out["receivers"]:
        print(json.dumps({k: r[k] for k in ("pid", "replayed", "invalid", "mismatches", "coverage")}, default=str))
    return 0 if out["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
