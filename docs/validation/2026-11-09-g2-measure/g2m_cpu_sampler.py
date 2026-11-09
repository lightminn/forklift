"""Real-time plan 5.22 correction (2026-11-09, the lead's rule for the set-F supplement, fixed before the run): every
10 s, the CPU each user's processes used on the node since the previous sample, from the process table only
(/proc/<pid>/stat's utime + stime and the /proc/<pid> owner -- what ps reads; nobody's files are opened). ps's %CPU is a
lifetime average and its cputimes has a 1 s step, so neither gives a 10 s rate; this differences the clock ticks.

A sample: {"wall", "mono", "dt", "cores": {user: cores over dt}, "other_cores": the sum over users other than root,
system users (uid < 1000), nobody and the job's own user, "new": processes first seen in this sample}. A process that
started and ended between two samples is not seen (an undercount noted with the rule).

    python g2m_cpu_sampler.py OUT.jsonl [--every 10]   # runs until killed (SIGTERM ends it cleanly)
"""

from __future__ import annotations

import json
import os
import pwd
import signal
import sys
import time

TCK = os.sysconf("SC_CLK_TCK")
NOBODY = 65534


def table() -> dict[int, tuple[int, int]]:
    """{pid: (uid, utime + stime ticks)} from the process table."""
    out = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            uid = os.stat(f"/proc/{d}").st_uid
            with open(f"/proc/{d}/stat", "rb") as f:
                raw = f.read()
        except OSError:
            continue  # ended while reading
        fields = raw[raw.rindex(b")") + 2:].split()  # after "pid (comm) "; comm may hold spaces
        out[int(d)] = (uid, int(fields[11]) + int(fields[12]))  # utime, stime: stat fields 14 and 15
    return out


def name(uid: int, cache: dict) -> str:
    if uid not in cache:
        try:
            cache[uid] = pwd.getpwuid(uid).pw_name
        except KeyError:
            cache[uid] = str(uid)
    return cache[uid]


def other_user(uid: int, own: int) -> bool:
    return uid >= 1000 and uid not in (own, NOBODY)


def sample(prev: dict, now: dict, dt: float, own: int, names: dict) -> dict:
    ticks: dict[int, int] = {}
    new = 0
    for pid, (uid, t) in now.items():
        before = prev.get(pid)
        if before is None or before[0] != uid:
            new += 1
            continue  # first seen: its earlier time is not this interval's
        ticks[uid] = ticks.get(uid, 0) + max(0, t - before[1])
    cores = {name(u, names): round(n / TCK / dt, 4) for u, n in sorted(ticks.items()) if n}
    other = sum(n for u, n in ticks.items() if other_user(u, own)) / TCK / dt
    return {"cores": cores, "other_cores": round(other, 4), "new": new}


def main() -> int:
    path = sys.argv[1]
    every = float(sys.argv[sys.argv.index("--every") + 1]) if "--every" in sys.argv else 10.0
    own = os.getuid()
    names: dict = {}
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    prev, t_prev = table(), time.monotonic()
    with open(path, "a", buffering=1) as out:
        out.write(json.dumps({"sampler": "g2m_cpu_sampler", "every_s": every, "own_uid": own, "tck": TCK}) + "\n")
        while not stop:
            time.sleep(every)
            now, t = table(), time.monotonic()
            rec = {"wall": time.time(), "mono": t, "dt": round(t - t_prev, 4)}
            rec.update(sample(prev, now, t - t_prev, own, names))
            out.write(json.dumps(rec) + "\n")
            prev, t_prev = now, t
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
