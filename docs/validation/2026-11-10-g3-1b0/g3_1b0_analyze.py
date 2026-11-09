"""Real-time plan 5.26 1b-0: what the grid_snapshot dumps say about _clear_group, replayed with the switch off.

For every dump (FORKLIFT_PERF_DUMP/grid_snapshot/*.pkl.gz) the call is replayed from its own ``before`` copy:
- the self-reproduction (plan 5.26 / Codex P2-4): the replayed snapshot's state and free_stamp equal the dumped
  result's, byte for byte -- only then is the dump used;
- per _clear_group call inside it: the raw cells, the bands that act (raw in the band and a usable radius), each acting
  band's radius and offset count, the grid shape, and its wall time per kernel mode (reference, numba; numba warmed up
  first and its first call left out);
- the dump's caller and mission phase (its stratum), so the sample's spread over callers and phases is written down.

Prediction only (single-call times on copies); the stage is judged in closed loop.

    PYTHONPATH=SNAP/src python g3_1b0_analyze.py DUMP_DIR [DUMP_DIR ...] --json out.json [--modes reference numba]
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
from collections import Counter
from pathlib import Path

os.environ.pop("FORKLIFT_PERF_DUMP", None)  # replaying must not dump again
import numpy as np  # noqa: E402

from forklift_core import kernels  # noqa: E402
from forklift_core.perception import obstacle_grid as og  # noqa: E402

_ORIGINAL = og.ObstacleGrid._clear_group


def raw_mask(grid, group) -> np.ndarray:
    """_clear_group's raw (the beams' cells), computed as it does."""
    cfg = grid.config
    raw = np.zeros(cfg.shape, dtype=bool)
    for scan, age, geo in group:
        rear_map, laser, ranges, cos_a, sin_a, usable, limit = geo
        finite = np.isfinite(ranges)
        clear_to = np.where(ranges == np.inf, cfg.max_clear_m, np.where(finite, np.minimum(ranges, cfg.max_mark_m), 0.0))
        clear_to = np.where(usable, np.minimum(np.minimum(clear_to, cfg.max_clear_m), limit), 0.0)
        bound = cfg.sensor_bound_at(np.where(finite, ranges, 0.0))
        clear_to = np.where(finite & (ranges <= cfg.max_mark_m), clear_to - bound - cfg.resolution_m, clear_to)
        raw |= grid._ray_mask(laser, cos_a, sin_a, np.maximum(clear_to, 0.0))
    return raw


def acting_bands(grid, group, raw) -> list[dict]:
    """The bands _clear_group erodes in: raw in the band and a radius within the cap (its own skip rules)."""
    cfg = grid.config
    nx, ny = cfg.shape
    res = cfg.resolution_m
    scan, age, geo = group[0]
    rear_map = geo[0]
    gx = cfg.x_min_m + (np.arange(nx) + 0.5) * res
    gy = cfg.y_min_m + (np.arange(ny) + 0.5) * res
    rho = np.hypot(gx[:, None] - rear_map[0], gy[None, :] - rear_map[1])
    out, lower = [], 0.0
    for edge in cfg.rho_bands_m:
        band = (rho >= lower) & (rho < edge)
        lower_used, lower = lower, edge
        cells = int((band & raw).sum())
        if not cells:
            continue
        r_band = grid.radius_m(max(age, cfg.free_max_age_s), edge, sensor=False)
        if r_band is None or r_band > cfg.free_r_cap_m:
            continue
        out.append({"lower_m": lower_used, "edge_m": edge, "raw_cells": cells, "radius_m": r_band,
                    "offsets": int(len(og._reach_offsets(max(r_band / res, 0.0))))})
    return out


def replay(record, mode: str, calls: list) -> tuple[object, float]:
    """The dumped call again from its ``before`` copy in kernel mode ``mode``; each _clear_group call's stats and time
    appended to ``calls``. Returns the snapshot and the whole call's wall time."""
    receiver, args, kwargs = copy.deepcopy(record["before"])

    def clear_group(self, group, state, free_stamp):
        raw = raw_mask(self, group)
        stats = {"raw_cells": int(raw.sum()), "shape": list(self.config.shape),
                 "bands": acting_bands(self, group, raw) if raw.any() else []}
        t0 = time.perf_counter()
        _ORIGINAL(self, group, state, free_stamp)
        stats["s"] = time.perf_counter() - t0
        calls.append(stats)

    kernels.set_mode(mode)
    og.ObstacleGrid._clear_group = clear_group
    try:
        t0 = time.perf_counter()
        snap = receiver.snapshot(*args, **kwargs)
        return snap, time.perf_counter() - t0
    finally:
        og.ObstacleGrid._clear_group = _ORIGINAL


def same_snapshot(a, b) -> bool:
    return (a.state.tobytes() == b.state.tobytes()
            and np.asarray(a.free_stamp).tobytes() == np.asarray(b.free_stamp).tobytes())


def load(path: Path) -> dict:
    with gzip.open(path, "rb") as f:
        return pickle.load(f)


def analyse(paths, modes) -> dict:
    if "numba" in modes:
        kernels.set_mode("numba")
        kernels.warm_up()
    rows, strata = [], Counter()
    for p in paths:
        rec = load(p)
        stratum = rec.get("stratum") or ()
        caller, phase = (stratum[1], stratum[2]) if len(stratum) > 2 else (None, None)
        strata[(caller, phase)] += 1
        row = {"file": str(p), "call": rec["call"], "caller": caller, "phase": phase}
        for mode in modes:
            calls: list = []
            snap, whole = replay(rec, mode, calls)
            row[mode] = {"self_reproduced": same_snapshot(snap, rec["result"]),
                         "snapshot_s_with_stats": whole,  # includes this tool's raw/band counting: not the call's cost
                         "clear_group_s": [c["s"] for c in calls]}
            if mode == modes[0]:
                row["clear_group_calls"] = [{k: v for k, v in c.items() if k != "s"} for c in calls]
        rows.append(row)
    used = [r for r in rows if all(r[m]["self_reproduced"] for m in modes)]
    calls = [c for r in used for c in r["clear_group_calls"]]
    summary = {
        "dumps": len(rows), "self_reproduced": len(used),
        "by_caller_phase": [[c, ph, n] for (c, ph), n in strata.most_common()],
        "clear_group_calls": len(calls),
        "raw_cells": _q([c["raw_cells"] for c in calls]),
        "acting_bands": _q([len(c["bands"]) for c in calls]),
        "offsets": dict(Counter(b["offsets"] for c in calls for b in c["bands"])),
        "shapes": [[list(k), n] for k, n in Counter(tuple(c["shape"]) for c in calls).most_common(5)],
    }
    for m in modes:  # every _clear_group call's own time (Codex 1b-0 review P2: no per-dump averaging)
        summary[f"{m}_clear_group_ms_per_call"] = _q([1e3 * v for r in used for v in r[m]["clear_group_s"]])
    return {"summary": summary, "rows": rows}


def _q(v) -> dict:
    if not v:
        return {}
    a = np.asarray(v, float)
    return {"n": int(a.size), "p50": float(np.median(a)), "p90": float(np.percentile(a, 90)), "max": float(a.max())}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", type=Path, nargs="+", help="FORKLIFT_PERF_DUMP folders (their grid_snapshot/ is read)")
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--modes", nargs="+", default=["reference", "numba"])
    ap.add_argument("--limit", type=int, default=0, help="0: every dump")
    a = ap.parse_args()
    missing = [str(d) for d in a.dirs if not any((d / "grid_snapshot").glob("*.pkl.gz"))]
    if missing:
        print(f"no grid_snapshot dumps in {missing}", file=sys.stderr)
        return 2
    paths = sorted(p for d in a.dirs for p in (d / "grid_snapshot").glob("*.pkl.gz"))
    if a.limit:
        paths = paths[: a.limit]
    out = analyse(paths, a.modes)
    a.json.write_text(json.dumps(out, indent=1, default=str))
    print(json.dumps(out["summary"], indent=1, default=str))
    return 0 if out["summary"]["self_reproduced"] == out["summary"]["dumps"] else 1


if __name__ == "__main__":
    sys.exit(main())
