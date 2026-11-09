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

Files are read one at a time (a call's arrays are about 1.6 MB). Every folder given must hold a marker and streamed
records, and none but streamed ones (a stream recording dumps nothing else). Reported: per receiver the counts above,
the continuous replay's total and per-kind time per arm, the memory move's own time per arm (``_move`` /
``_move_arrays``, so its share of apply can be judged, plan 5.27), and the coverage of the cases plan 5.27 lists, read
off the branches the reference replay actually took (the candidates handed to the RETAINED judgement, which of its
paths ran, its result).

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

if "forklift_core.perf" not in sys.modules:
    os.environ.pop("FORKLIFT_PERF_DUMP", None)  # replaying must not dump again (read when perf is imported)
import numpy as np  # noqa: E402

from forklift_core import kernels  # noqa: E402
from forklift_core.control import shadow_memory as sm  # noqa: E402
from forklift_core.control.drive_permission import RETAINED  # noqa: E402
from forklift_core.perf import differences  # noqa: E402

APPLY = sm.ShadowMemory.apply
while hasattr(APPLY, "__wrapped__"):
    APPLY = APPLY.__wrapped__
_ARRAYS = sm.ShadowMemory._apply_arrays
_MOVE_ARRAYS = sm.ShadowMemory._move_arrays
RAN = Counter()
SEEN: dict = {}  # what the reference replay's RETAINED judgement got and gave, and the move's time
HOOKS = (("ShadowMemory", "_apply_arrays"), ("ShadowMemory", "_move_arrays"), ("ShadowMemory", "_move"),
         ("ShadowMemory", "_retained_stamps"), ("module", "_retained_stamps_batched"))


def _count(name, real):
    def inner(self, *a, **k):
        RAN[name] += 1
        return real(self, *a, **k)
    return inner


def _timed_move(real):
    def inner(self, *a, **k):
        if SEEN.get("in_move"):  # _move_arrays falling back to _move: counted once, by the outer call
            return real(self, *a, **k)
        SEEN["in_move"] = True
        start = time.perf_counter()
        try:
            return real(self, *a, **k)
        finally:
            SEEN["move_s"] = SEEN.get("move_s", 0.0) + time.perf_counter() - start
            SEEN["in_move"] = False
    return inner


def _seen_retained(path, real):
    def inner(cand, res, fresh, stamps, whole, nx, ny):
        out = real(cand, res, fresh, stamps, whole, nx, ny)
        SEEN.update(path=path, cand=dict(cand), stamp=dict(out), res=res, shape=(nx, ny))
        return out
    return inner


def install() -> dict:
    """The counting, timing and inspecting hooks; returns what was there (the class's own entries, a staticmethod
    object included), for ``uninstall``."""
    cls = sm.ShadowMemory
    saved = {h: (cls.__dict__[h[1]] if h[0] == "ShadowMemory" else getattr(sm, h[1])) for h in HOOKS}
    cls._apply_arrays = _count("arrays", cls.__dict__["_apply_arrays"])
    cls._move_arrays = _timed_move(_count("move_arrays", cls.__dict__["_move_arrays"]))
    cls._move = _timed_move(cls.__dict__["_move"])
    cls._retained_stamps = staticmethod(_seen_retained("reference", cls.__dict__["_retained_stamps"].__func__))
    sm._retained_stamps_batched = _seen_retained("batched", saved[("module", "_retained_stamps_batched")])
    return saved


def uninstall(saved: dict) -> None:
    for (owner, name), value in saved.items():
        setattr(sm.ShadowMemory if owner == "ShadowMemory" else sm, name, value)


def branches() -> list[str]:
    """Coverage from the reference replay's RETAINED judgement (Codex 1c-0 review P3: the cases as taken, not
    guessed from the inputs): the reference path (a zero or NaN, depth support included), no candidates, an empty
    or all-inf result, and a candidate whose reach crosses the grid's edge."""
    out = []
    if "cand" not in SEEN:
        return out
    cand, stamp, res, (nx, ny) = SEEN["cand"], SEEN["stamp"], SEEN["res"], SEEN["shape"]
    if SEEN["path"] == "reference":
        out.append("zero_or_nan")
    if not cand:
        out.append("no_candidates")
    elif not stamp:
        out.append("empty_stamp")
    elif not any(np.isfinite(v) for v in stamp.values()):
        out.append("all_inf_stamp")
    for (a, b), r in cand.items():
        k = int(np.ceil(r / res)) + 1
        if a - k < 0 or b - k < 0 or a + k >= nx or b + k >= ny:
            out.append("candidate_reach_off_grid")
            break
    return out


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
    out.append("retained" if (result.state == RETAINED).any() else "no_retained")
    if previous is not None:
        p_snap, p_foot = previous
        if (snap.origin_x_m, snap.origin_y_m) != (p_snap.origin_x_m, p_snap.origin_y_m):
            out.append("origin_shift")
        if foot != p_foot:
            out.append("footprint_change")
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
        SEEN.clear()
        out, dt = call(self.ref, record, self.modes[0], False)
        self.cover.update(branches())
        self.time["off"][bucket] += dt
        self.time["off"]["move_only"] += SEEN.get("move_s", 0.0)
        bad = same((self.ref, out), (record["after"][0], record["result"]))
        if bad:
            self.mismatch.append({"seq": seq, "arm": "reference", "diff": bad[:3]})
        for m in self.modes:
            arrays = RAN["arrays"]
            SEEN.clear()
            c_out, c_dt = call(self.cand[m], record, m, True)
            self.time[f"on_{m}"][bucket] += c_dt
            self.time[f"on_{m}"]["move_only"] += SEEN.get("move_s", 0.0)
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
    out = {"unstreamed_records": unstreamed, "receivers": [], "folders": []}
    for d in dirs:  # every folder given counts (Codex 1c-0 review P2: a missing session must not pass unseen)
        markers = list(Path(d).glob("stream_totals_*.json"))
        records = any((Path(d) / "shadow_apply").glob("*.pkl.gz"))
        out["folders"].append({"dir": str(d), "markers": len(markers), "records": bool(records),
                               "problem": None if markers and records else
                               "no completeness marker" if records else "no shadow_apply records"})
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
    out["valid"] = bool(streams) and not out["missing_receivers"] and not unstreamed and all(
        f["problem"] is None for f in out["folders"]) and all(
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
    hooks = install()  # what is there now, not at import (Codex 1c-0 review P3)
    try:
        out = replay(a.dirs, a.modes)
    finally:
        uninstall(hooks)
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
