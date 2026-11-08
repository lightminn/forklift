"""Real-time plan 5.15 G1: the tick trace's own cost per physics step (a CPU microbenchmark, run on ws1 alone).

    python trace_bench.py SNAP [--steps 120000] [--repeat 5]

Per step it makes the calls the live loop makes when FORKLIFT_LIVE_TICK_TRACE=1 (sim/isaac/run_transport.py and
live_runtime.py): the three control marks before the step, begin(), the set() of q, d, S, world_start, world_end,
step_end, after_end with a clock read for each, tick and live_step, and two flags every twelfth step (the
runner's call sites: 15 in run_transport.py and live_runtime.py). It prints the time per step
(median of the repeats) against the 0.05 ms per step bound of plan 5.15.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("snap", type=Path)
    ap.add_argument("--steps", type=int, default=120_000)
    ap.add_argument("--repeat", type=int, default=5)
    a = ap.parse_args()
    sys.path.insert(0, str(a.snap.resolve() / "sim" / "isaac"))
    from tick_trace import TickTrace

    clock = time.perf_counter
    per_step = []
    for _ in range(a.repeat):
        tr = TickTrace(a.steps + 16, clock)
        t0 = clock()
        for i in range(a.steps):
            tr.control("ctrl_start", tr.now())
            tr.control("replies_s", 0.0)
            tr.control("ctrl_end", tr.now())
            tr.begin()
            tr.set("tick", float(i))
            tr.set("live_step", float(i))
            for name in ("q", "d", "S", "world_start", "world_end", "step_end", "after_end"):
                tr.set(name, tr.now())
            if i % 12 == 0:
                tr.flag("scan")
                tr.flag("rendered")
        per_step.append((clock() - t0) / a.steps)
        tr.set("tick", 0.0)
    per_step.sort()
    med = per_step[len(per_step) // 2]
    print(f"trace cost per step: median {med * 1e3:.4f} ms over {a.repeat} x {a.steps} steps "
          f"(min {per_step[0] * 1e3:.4f}, max {per_step[-1] * 1e3:.4f}); bound 0.05 ms: "
          f"{'within' if med <= 5e-5 else 'over'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
