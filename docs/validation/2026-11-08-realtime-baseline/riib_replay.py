"""Real-time plan 5.15 follow-up (lead's request, 2026-11-08): the console's riib batch traces (3 sessions a GPU, the
learned worker on, line_m / random_m, factor 0.3) replayed through the real Pacer at their declared factor with
catch-up 0 (the check against the measured ratio), 6 and 12, and the camera phase split -- an offline counterfactual
only (it assumes each step's work does not depend on when it starts, which G1 supported for the hall alone).

    python riib_replay.py SNAP TRACE [TRACE ...] [--factor 0.3]

Every recorded row with a finite W and a next start is replayed (the traces cover their first --max-sim-seconds
only). The split moves a live-camera render's own cost (world.step above the plain steps' median, and the camera
read) six ticks on, as pacer_replay.py does; far-recognition frames and captures stay where they were, so the split
here is indicative only (G1 validated it without far frames).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("snap", type=Path)
    ap.add_argument("traces", nargs="+", type=Path)
    ap.add_argument("--factor", type=float, default=0.3)
    a = ap.parse_args()
    sys.path.insert(0, str(a.snap.resolve() / "sim" / "isaac"))
    sys.path.insert(0, str(a.snap.resolve() / "src"))
    from tick_trace import COLUMNS, FLAGS, work
    from forklift_core.live.pacing import Pacer

    dt = 1 / 120
    C = {n: i for i, n in enumerate(COLUMNS)}
    BIT = {n: 1 << i for i, n in enumerate(FLAGS)}

    def replay(w, eps, tse, catchup):
        p = Pacer(a.factor, dt, catchup_steps=catchup)
        t, t_first, oks = 0.0, None, []
        for wi, ei, si in zip(w, eps, tse):
            sleep = p.wait_s(t)
            t += sleep + (ei if sleep > 0 else 0.0)
            p.started(t)
            t_first = t if t_first is None else t_first
            p.after_step(t + min(max(si, 0.0), wi))
            oks.append(p.ok)
            t += wi
        oks = np.array(oks)
        first = int(np.argmax(oks)) if oks.any() else None
        return len(w) * dt / (t - t_first), (None if first is None else float(oks[first:].mean()))

    for path in a.traces:
        with np.load(path) as z:
            data = z["data"]
        S, q, d = data[:, C["S"]], data[:, C["q"]], data[:, C["d"]]
        w = work(data)
        interval = np.r_[S[1:] - S[:-1], np.nan]
        eps = np.nan_to_num(S - (q + d))  # each start's own overshoot (Codex G3-0 review P2: was one step late)
        K = np.isfinite(w) & np.isfinite(interval) & np.isfinite(data[:, C["step_end"]])
        idx = np.flatnonzero(K)
        bits = np.nan_to_num(data[:, C["flags"]]).astype(int)
        live = (bits & BIT["live_camera"]) != 0
        rendered = (bits & BIT["rendered"]) != 0
        world = data[:, C["world_end"]] - data[:, C["world_start"]]
        after = data[:, C["after_end"]] - data[:, C["step_end"]]
        tse = data[:, C["step_end"]] - S
        measured = len(idx) * dt / np.sum(interval[idx])
        W, E, T = w[idx], eps[idx], tse[idx]
        line = [f"{path.parent.parent.parent.parent.name}: measured {measured:.4f}, mean W/h {np.mean(W) / (dt / a.factor):.3f}"]
        for k in (0, 6, 12):
            r, ok = replay(W, E, T, k)
            line.append(f"k{k} {r:.4f}/ok {ok if ok is None else round(ok, 3)}")
        plain_world = np.nanmedian(world[~rendered])
        own_r = np.where(live & rendered, np.maximum(np.nan_to_num(world) - plain_world, 0), 0.0)
        own_a = np.where(live & rendered, np.nan_to_num(after), 0.0)
        w2, t2 = w.copy(), tse.copy()
        w2 -= own_r + own_a
        t2 -= own_r
        w2[6:] += own_r[:-6] + own_a[:-6]
        t2[6:] += own_r[:-6]
        r, ok = replay(w2[idx], E, t2[idx], 6)
        line.append(f"split+k6 {r:.4f}/ok {ok if ok is None else round(ok, 3)}")
        print("  ".join(line))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
