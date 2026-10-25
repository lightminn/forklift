"""Slot time of ws1 bundle jobs (tools/isaac_bundle.py), for docs/plans/2026-10-26-ws1-bundle-node-queue.md.

  idle JOB:SLOTS:RESULTS.jsonl ...   held vs used slot-minutes of finished bundle jobs (start/end from sacct; a job
                                     that ran nothing may name a missing results file), and the tail: what the jobs
                                     held after the list's last start, counted from max(job start, last start), minus
                                     what they used there. The tail is the idle area up to the jobs' real ends -- an
                                     upper bound on what any scheme could fill, not what one would recover. A result's
                                     `end` is its top process's end, slightly before the slot's release.
  sim RESULTS.jsonl ...              the same runs' recorded durations replayed on 2 jobs x 4 slots and 4 jobs x 2
                                     slots sharing one list, in list order and longest first. A model: every job
                                     starts at 0, the next run goes to the slot that can start it first (max(slot free,
                                     its job's 1 min stagger)), no back-off, VRAM gate or Slurm queue; a job holds its
                                     slots until its last run ends.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time


def _rows(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [r for r in (json.loads(line) for line in f if line.strip()) if r.get("start") and r.get("end")]


def idle_table(jobs):
    """jobs: [(job, slots, start, end, rows)] -> ([per-job dict], totals dict); minutes."""
    starts = [r["start"] for *_, rows in jobs for r in rows]
    if not starts:
        raise ValueError("no run started in these jobs")
    last_start = max(starts)
    out, tot = [], {"held": 0.0, "used": 0.0, "tail": 0.0}
    for job, slots, s, e, rows in jobs:
        held = slots * (e - s) / 60
        used = sum(r["end"] - r["start"] for r in rows) / 60
        t0 = max(s, last_start)
        tail = (slots * max(0.0, e - t0) - sum(max(0.0, r["end"] - max(r["start"], t0)) for r in rows)) / 60
        out.append({"job": job, "wall": (e - s) / 60, "runs": len(rows), "held": held, "used": used, "tail": tail})
        tot["held"] += held
        tot["used"] += used
        tot["tail"] += tail
    return out, tot


def simulate(durations, jobs, slots, stagger=1.0):
    """(held slot-min, used, makespan) of `durations` (min) on jobs x slots sharing one list."""
    free = [[0.0] * slots for _ in range(jobs)]
    next_start = [0.0] * jobs
    end_of = [0.0] * jobs
    for d in durations:
        t, j, s = min((max(free[j][s], next_start[j]), j, s) for j in range(jobs) for s in range(slots))
        next_start[j] = t + stagger
        end_of[j] = max(end_of[j], t + d)
        free[j][s] = t + d
    return sum(slots * e for e in end_of), sum(durations), max(end_of)


def _sacct(job):
    out = subprocess.run(["sacct", "-X", "-n", "-P", "-j", job, "-o", "Start,End"], capture_output=True,
                         text=True, check=True).stdout.strip().splitlines()[0].split("|")
    return [time.mktime(time.strptime(x, "%Y-%m-%dT%H:%M:%S")) for x in out]


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) < 2 or argv[0] not in ("idle", "sim"):
        print(__doc__, file=sys.stderr)
        return 2
    if argv[0] == "idle":
        jobs = []
        for arg in argv[1:]:
            job, slots, path = arg.split(":", 2)
            jobs.append((job, int(slots), *_sacct(job), _rows(path)))
        per, tot = idle_table(jobs)
        for r in per:
            print(f"{r['job']}: wall {r['wall']:.1f} min, runs {r['runs']}, held {r['held']:.1f} slot-min, "
                  f"used {r['used']:.1f}, idle {r['held'] - r['used']:.1f}, tail {r['tail']:.1f}")
        print(f"total: held {tot['held']:.1f}, used {tot['used']:.1f} ({tot['used'] / tot['held']:.0%}), "
              f"idle {tot['held'] - tot['used']:.1f}, tail {tot['tail']:.1f} slot-min "
              f"({tot['tail'] / tot['held']:.1%})")
        return 0
    runs = sorted((r["start"], (r["end"] - r["start"]) / 60) for path in argv[1:] for r in _rows(path))
    order = [d for _, d in runs]
    for name, durs in (("list order", order), ("longest first", sorted(order, reverse=True))):
        for jobs, slots in ((2, 4), (4, 2)):
            held, used, span = simulate(durs, jobs, slots)
            print(f"{name:13s} {jobs} jobs x {slots}: held {held:6.1f} used {used:6.1f} idle {held - used:6.1f} "
                  f"({(held - used) / held:5.1%}) makespan {span:5.1f} min")
    return 0


if __name__ == "__main__":
    sys.exit(main())
