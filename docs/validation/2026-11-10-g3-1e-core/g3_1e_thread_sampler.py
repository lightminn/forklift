"""1e stage 1 (docs/plans/2026-11-10-g3-1e-process-worker.md 3, report only): every 10 s, each thread of this job's
processes -- the processes of this user in the sampler's own cgroup (the Slurm job's; the SLAM bridge containers are
outside it) -- with its command, CPU ticks (utime + stime), scheduler counters (run ns, run-queue delay ns) and the CPU
it last ran on, from /proc/<pid>/task/<tid>/{stat,schedstat}. Only the process table and this job's own threads are
read. A thread that started and ended between two samples is not seen (reported as such, its number unknown).

A sample: {"wall", "mono", "threads": {"<pid>/<tid>": [process comm, thread comm, ticks, run_ns, delay_ns, cpu,
affinity (the thread's Cpus_allowed_list)]}}.

    python g3_1e_thread_sampler.py OUT.jsonl [--every 10]   # until SIGTERM
"""

from __future__ import annotations

import json
import os
import signal
import sys
import time


def own_cgroup(pid="self") -> str | None:
    try:
        with open(f"/proc/{pid}/cgroup") as f:
            return f.read()
    except OSError:
        return None


def _stat(path: str):
    with open(path, "rb") as f:
        raw = f.read()
    comm = raw[raw.index(b"(") + 1:raw.rindex(b")")].decode(errors="replace")
    fields = raw[raw.rindex(b")") + 2:].split()
    return comm, int(fields[11]) + int(fields[12]), int(fields[36])  # utime + stime; processor (stat field 39)


def _schedstat(path: str):
    with open(path) as f:
        a, b = f.read().split()[:2]
    return int(a), int(b)


def _allowed(path: str) -> str | None:
    with open(path) as f:
        for line in f:
            if line.startswith("Cpus_allowed_list:"):
                return line.split(":", 1)[1].strip()
    return None


def sample(proc="/proc", uid=None, cgroup=None) -> dict:
    uid = os.getuid() if uid is None else uid
    cgroup = own_cgroup() if cgroup is None else cgroup
    out = {}
    for d in os.listdir(proc):
        if not d.isdigit():
            continue
        base = f"{proc}/{d}"
        try:
            if os.stat(base).st_uid != uid:
                continue
            with open(f"{base}/cgroup") as f:
                if f.read() != cgroup:
                    continue
            pcomm = _stat(f"{base}/stat")[0]
            tids = os.listdir(f"{base}/task")
        except OSError:
            continue  # ended while reading
        for t in tids:
            try:
                tcomm, ticks, cpu = _stat(f"{base}/task/{t}/stat")
                run_ns, delay_ns = _schedstat(f"{base}/task/{t}/schedstat")
                allowed = _allowed(f"{base}/task/{t}/status")
            except (OSError, ValueError, IndexError):
                continue
            out[f"{d}/{t}"] = [pcomm, tcomm, ticks, run_ns, delay_ns, cpu, allowed]
    return out


def main() -> int:
    out_path = sys.argv[1]
    every = float(sys.argv[sys.argv.index("--every") + 1]) if "--every" in sys.argv else 10.0
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    with open(out_path, "a") as f:
        f.write(json.dumps({"sampler": "g3_1e_thread_sampler", "every_s": every, "uid": os.getuid(),
                            "tck": os.sysconf("SC_CLK_TCK")}) + "\n")
        while not stop["now"]:
            f.write(json.dumps({"wall": time.time(), "mono": time.monotonic(), "threads": sample()}) + "\n")
            f.flush()
            end = time.monotonic() + every
            while not stop["now"] and time.monotonic() < end:
                time.sleep(0.2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
