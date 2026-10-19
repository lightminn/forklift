"""Real-time plan 5.12 (D12a): where a live run's wall time went, from its per-step trace (descriptive only).

For each run directory (holding ``run/tick_trace.npz`` and ``run/slam_records.json``), over the driving period of
plan 5.10 (from the first ``tracking`` SLAM record's scan time on):
  - the measured wall per sim second (from the paced starts, S[i+1] - S[i]) and the pacing model's Σmax(W_i, h),
    which leaves out the wake-ups' overshoot;
  - the pacer's spacing h, the work W_i = q[i+1] - S[i] and the overrun max(W_i - h, 0), per sim second, by the
    row's kind (render and scan, render, scan, plain; a render is a step whose world.step rendered) and by the
    physics tick's phase (tick % 12);
  - the parts of W_i, which add up to it: from the paced start to world.step (the pacer's own work and the stop
    gate), world.step, the rest of step_world (the scan's processing), after_step (the camera read), up to the
    next control tick, the control tick (its SLAM replies are part of it, shown apart), and on to the next wait;
    and the wait's overshoot S - (q + d), outside W_i;
  - render counts by who asked (live camera, video frame, the runner's render argument and, of those, a capture's
    or a pocket frame's step; ``unattributed``: rendered with no request bit -- the far recognition's frames);
  - the steps of the driving period whose world.step, or the step from its paced start to after_step, took
    longer than --stall-s (the last step included).

    python tools/tick_trace_report.py RUN_DIR [RUN_DIR ...] [--factor 1.0] [--stall-s 0.1] [--json out.json]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))
from tick_trace import COLUMNS, FLAGS, work  # noqa: E402

DT = 1 / 120
C = {name: i for i, name in enumerate(COLUMNS)}
BIT = {name: 1 << i for i, name in enumerate(FLAGS)}


def load(run: Path) -> np.ndarray:
    with np.load(run / "run" / "tick_trace.npz") as saved:
        if list(saved["columns"]) != list(COLUMNS) or list(saved["flags"]) != list(FLAGS):
            raise ValueError(f"{run}: trace columns differ from this tool's")
        return saved["data"]


def driving_start_s(run: Path) -> float:
    records = json.loads((run / "run" / "slam_records.json").read_text())
    stamps = [r["stamp_s"] for r in records if r.get("mode") == "tracking"]
    if not stamps:
        raise ValueError(f"{run}: no tracking SLAM record")
    return float(min(stamps))


ASKED = BIT["live_camera"] | BIT["video"] | BIT["render_arg"]


def kind(flags: np.ndarray) -> np.ndarray:
    """By what the step did: a render is a step whose world.step rendered (asked for or not)."""
    bits = np.nan_to_num(flags).astype(int)
    render = (bits & BIT["rendered"]) != 0
    scan = (bits & BIT["scan"]) != 0
    return np.where(render & scan, "render+scan", np.where(render, "render", np.where(scan, "scan", "plain")))


def report(run: Path, factor: float = 1.0, stall_s: float = 0.1) -> dict:
    data = load(run)
    h = DT / factor
    sim_s = data[:, C["tick"]] * DT
    driving = sim_s >= driving_start_s(run)
    w_all = work(data)
    keep = driving & np.isfinite(w_all)  # rows with a next row: W_i is known
    rows = data[keep]
    w = w_all[keep]
    sim_seconds = len(rows) * DT
    over = np.maximum(w - h, 0.0)
    kinds = kind(rows[:, C["flags"]])
    phase = (rows[:, C["tick"]].astype(int)) % 12
    # The next row's control tick belongs to this row's W (it runs before the next wait request).
    nxt = data[np.flatnonzero(keep) + 1]
    parts = {
        "gate": rows[:, C["world_start"]] - rows[:, C["S"]],
        "world": rows[:, C["world_end"]] - rows[:, C["world_start"]],
        "step_rest": rows[:, C["step_end"]] - rows[:, C["world_end"]],
        "after": rows[:, C["after_end"]] - rows[:, C["step_end"]],
        "between": nxt[:, C["ctrl_start"]] - rows[:, C["after_end"]],
        "control": nxt[:, C["ctrl_end"]] - nxt[:, C["ctrl_start"]],
        "to_wait": nxt[:, C["q"]] - nxt[:, C["ctrl_end"]],
    }
    apart = {"control_replies": np.nan_to_num(nxt[:, C["replies_s"]]),  # within "control"
             "overshoot": nxt[:, C["S"]] - (nxt[:, C["q"]] + nxt[:, C["d"]])}  # the next wake-up, outside W_i
    bits = np.nan_to_num(rows[:, C["flags"]]).astype(int)
    # Stalls over every driving row, the last one included (its W is unknown, its own marks are not).
    drows = data[driving]
    dbits = np.nan_to_num(drows[:, C["flags"]]).astype(int)
    dworld = drows[:, C["world_end"]] - drows[:, C["world_start"]]
    dstep = drows[:, C["after_end"]] - drows[:, C["S"]]
    dkinds = kind(drows[:, C["flags"]])
    stalls = [{"sim_s": float(drows[i, C["tick"]] * DT), "tick": int(drows[i, C["tick"]]),
               "world_s": float(dworld[i]), "step_s": float(dstep[i]), "kind": str(dkinds[i]),
               "phase": int(drows[i, C["tick"]]) % 12, "flags": [f for f in FLAGS if dbits[i] & BIT[f]],
               "S_mono": float(drows[i, C["S"]])}
              for i in np.flatnonzero((dworld > stall_s) | (dstep > stall_s))]
    starts = drows[:, C["S"]]
    return {
        "run": run.name,
        "rows": int(len(rows)),
        "sim_s": sim_seconds,
        "wall_per_sim_s": float((starts[-1] - starts[0]) / ((len(starts) - 1) * DT)) if len(starts) > 1 else None,
        "model_wall_per_sim_s": float(np.sum(np.maximum(w, h)) / sim_seconds) if sim_seconds else None,
        "overrun_per_sim_s": float(over.sum() / sim_seconds),
        "overrun_by_kind": {k: float(over[kinds == k].sum() / sim_seconds) for k in sorted(set(kinds))},
        "rows_by_kind": {k: int(np.sum(kinds == k)) for k in sorted(set(kinds))},
        "overrun_by_phase": [float(over[phase == k].sum() / sim_seconds) for k in range(12)],
        "mean_ms_by_kind": {k: {p: float(np.nanmean(v[kinds == k]) * 1000) for p, v in {**parts, **apart}.items()}
                            for k in sorted(set(kinds))},
        "renders": {**{f: int(np.sum((bits & BIT[f]) != 0))
                       for f in ("live_camera", "video", "render_arg", "capture", "pocket", "rendered")},
                    "unattributed": int(np.sum(((bits & BIT["rendered"]) != 0) & ((bits & ASKED) == 0)))},
        "stalls": stalls,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--factor", type=float, default=1.0)
    parser.add_argument("--stall-s", type=float, default=0.1)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    out = [report(run, args.factor, args.stall_s) for run in args.runs]
    for r in out:
        print(f"{r['run']:28s} rows {r['rows']:6d} wall/sim {r['wall_per_sim_s']:.4f} "
              f"(model {r['model_wall_per_sim_s']:.4f}) "
              f"overrun/sim-s {r['overrun_per_sim_s'] * 1000:6.1f} ms  by kind "
              + " ".join(f"{k} {v * 1000:.1f}" for k, v in r["overrun_by_kind"].items())
              + f"  renders {r['renders']}  stalls {len(r['stalls'])}")
    if args.json:
        args.json.write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
