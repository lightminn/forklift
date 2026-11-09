"""Real-time plan 5.22 M7: the per-call cost of the measurement mode's wrappers on this node and this Python (run with
the forklift venv on ws1, beside the measurement run). A no-op method called directly, wrapped with the long-call
record off (the status-line timer only), and wrapped with it on below the threshold (stack, count, two clocks each),
by_parent too. Prints JSON: microseconds per call, median of repeats.

    PYTHONPATH=SNAP/src python g2m_bench.py [--n 200000] [--repeats 7]
"""

import argparse
import json
import statistics
import time

from forklift_core.perf import Timers


def per_call(fn, n):
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n * 1e6


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=200_000)
    ap.add_argument("--repeats", type=int, default=7)
    a = ap.parse_args()

    class Target:
        def plain(self):
            return None

        def timed(self):
            return None

        def timed_parent(self):
            return None

        def long_only(self):
            return None

    t = Timers()
    t.enabled = True
    t.wrap(Target, "timed", "x.timed")
    t.wrap(Target, "timed_parent", "x.timed_parent", by_parent=True)
    t.wrap(Target, "long_only", "x.long_only", long_only=True)
    obj = Target()
    out = {}
    for label, fn, on in (("direct", obj.plain, False), ("timer_off_record", obj.timed, False),
                          ("timer_on_record", obj.timed, True), ("timer_by_parent_on_record", obj.timed_parent, True),
                          ("long_only_on_record", obj.long_only, True)):
        t.long_s = 10.0 if on else None
        samples = [per_call(fn, a.n) for _ in range(a.repeats)]
        t.take()
        out[label] = statistics.median(samples)
    # a by_parent call inside an open region (the parent name is built, as in the run)
    t.long_s = 10.0
    with t.region("parent<caller|grand"):
        samples = [per_call(obj.timed_parent, a.n) for _ in range(a.repeats)]
    t.take()
    out["timer_by_parent_in_parent_on_record"] = statistics.median(samples)
    samples = [per_call(t.sched_mark, a.n) for _ in range(a.repeats)]
    out["sched_mark_us"] = statistics.median(samples)
    clock = time.perf_counter
    samples = [per_call(clock, a.n) for _ in range(a.repeats)]
    out["perf_counter_us"] = statistics.median(samples)
    out["added_on_vs_off_us"] = out["timer_on_record"] - out["timer_off_record"]
    out["long_only_added_us"] = out["long_only_on_record"] - out["direct"]
    # Plan 5.22 M7 (Codex round 2 P2-1): every entered region is charged the largest full cost (a timer that does not
    # exist when the measurement is off pays all of it): the by_parent call inside a parent, less the direct call.
    out["per_region_call_bound_us"] = max(out["timer_by_parent_in_parent_on_record"], out["timer_by_parent_on_record"],
                                          out["timer_on_record"], out["long_only_on_record"]) - out["direct"]
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
