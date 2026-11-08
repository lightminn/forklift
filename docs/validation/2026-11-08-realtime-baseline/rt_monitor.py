"""Real-time plan 5.13: CPU/GPU samples beside one live session (measurement only, nothing is changed).

    python rt_monitor.py OUT_DIR BRIDGE_CONTAINER_NAME

Writes, each line stamped with CLOCK_MONOTONIC (time.monotonic, the clock of the tick trace's perf_counter on Linux)
and the wall clock:
  OUT_DIR/gpu.csv     nvidia-smi --query-gpu every 200 ms: utilization (the device's own sampling window, about
                      1/6-1 s -- not an instantaneous load), memory, SM clock, temperature, power, active throttle
                      reasons
  OUT_DIR/host.jsonl  every 1 s: every process on the GPU -- compute and graphics, from `nvidia-smi -q -d PIDS` --
                      with whether it sits in this Slurm job's cgroup (None: could not tell), the query's own status
                      ("ok", or the error: a failed query is not an empty GPU), /proc/loadavg, the per-CPU /proc/stat
                      lines, and the bridge container's cgroup cpu.stat (usage, not CPU wait); every 10 s the Slurm
                      queue's running jobs
Ends on SIGTERM.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

out = Path(sys.argv[1])
bridge = sys.argv[2]
out.mkdir(parents=True, exist_ok=True)
job = os.environ.get("SLURM_JOB_ID", "")
running = True


def stop(*_):
    global running
    running = False


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)

gpu_file = open(out / "gpu.csv", "w")
gpu_file.write(f"# start mono {time.monotonic():.6f} wall {time.time():.6f}\n")
gpu_file.flush()
smi = subprocess.Popen(
    ["nvidia-smi", "--query-gpu=timestamp,uuid,utilization.gpu,memory.used,clocks.sm,temperature.gpu,power.draw,"
     "clocks_throttle_reasons.active", "--format=csv,noheader,nounits", "-lms", "200"],
    stdout=gpu_file, stderr=subprocess.DEVNULL)


def own(pid: str):
    try:
        return bool(job) and f"job_{job}" in Path(f"/proc/{pid}/cgroup").read_text()
    except OSError:
        return None


def gpu_processes():
    """(status, [[pid, type, name, own], ...]) for every process on the GPU, graphics ones included."""
    try:
        p = subprocess.run(["nvidia-smi", "-q", "-d", "PIDS"], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return repr(exc), None
    if p.returncode != 0:
        return f"rc {p.returncode}", None
    rows, cur = [], {}
    for line in p.stdout.splitlines():
        m = re.match(r"\s*(Process ID|Type|Name)\s*:\s*(.*)$", line)
        if not m:
            continue
        key, value = m.group(1), m.group(2).strip()
        if key == "Process ID":
            if cur:
                rows.append(cur)
            cur = {"pid": value}
        else:
            cur[key.lower()] = value
    if cur:
        rows.append(cur)
    return "ok", [[r.get("pid"), r.get("type"), r.get("name"), own(r.get("pid", ""))] for r in rows]


bridge_stat = None


def bridge_cpu():
    global bridge_stat
    if bridge_stat is None:
        try:
            cid = subprocess.run(["docker", "inspect", "-f", "{{.Id}}", bridge], capture_output=True, text=True,
                                 timeout=5).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            return None
        if not cid:
            return None
        for cand in (f"/sys/fs/cgroup/system.slice/docker-{cid}.scope/cpu.stat",
                     f"/sys/fs/cgroup/docker/{cid}/cpu.stat"):
            if Path(cand).exists():
                bridge_stat = Path(cand)
                break
        else:
            return None
    try:
        return dict(line.split() for line in bridge_stat.read_text().splitlines())
    except OSError:
        bridge_stat = None  # the container ended or was replaced: look it up again
        return None


def queue():
    try:
        return subprocess.run(["squeue", "-h", "-t", "R", "-o", "%i %u %j %b %C"], capture_output=True, text=True,
                              timeout=5).stdout.splitlines()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return repr(exc)


n = 0
with open(out / "host.jsonl", "w") as host:
    while running:
        t0 = time.monotonic()
        status, procs = gpu_processes()
        row = {"mono": t0, "wall": time.time(), "gpu_status": status, "gpu_procs": procs,
               "loadavg": Path("/proc/loadavg").read_text().split()[:3],
               "cpus": [line.split() for line in Path("/proc/stat").read_text().splitlines()
                        if line.startswith("cpu")],
               "bridge": bridge_cpu()}
        if n % 10 == 0:
            row["squeue"] = queue()
        n += 1
        host.write(json.dumps(row) + "\n")
        host.flush()
        time.sleep(max(0.0, 1.0 - (time.monotonic() - t0)))
smi.terminate()
try:
    smi.wait(timeout=5)
except subprocess.TimeoutExpired:
    smi.kill()
gpu_file.write(f"# end mono {time.monotonic():.6f} wall {time.time():.6f}\n")
gpu_file.close()
