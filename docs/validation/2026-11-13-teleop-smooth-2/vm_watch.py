#!/usr/bin/env python3
"""The VM rule's evidence (docs/plans/2026-11-13-teleop-smooth-2.md 2, teleop-smooth-3.md 2): ws1's VM
``ws1-win-gpu1`` takes GPU 1 (vfio-pci) and its qemu runs as libvirt-qemu, so neither the bundle's CPU sampler nor
sacct sees it, and its libvirt log is root-only. This samples, from outside ws1, whether a qemu-system process runs and
which GPUs nvidia-smi lists, and then reads each session window against the samples.

    python vm_watch.py sample WATCH.jsonl [--every 30]     # until stopped; one JSON line per sample, ssh failures too
    python vm_watch.py judge WATCH.jsonl NAME START END ... # START/END as sacct prints them (ws1 local time)

A window [START, END] is read over [START - 45 s, END + 45 s] (1.5 periods: a sample's own ssh jitter) and nothing
outside it. ``vm`` when a sample in that range saw qemu running or nvidia-smi listing fewer than two GPUs -- either
query alone counts, a failed other half does not erase it. ``clear`` when complete samples (both queries answered,
no VM) bracket the window inside the range -- one at or before START, one at or after END -- with no two consecutive
complete samples more than 60 s apart; this is "no VM seen at the registered sampling", not proof that none ran: a VM
up and down again within one gap would be missed (the 2026-11-13 11:30:12-11:32:38 round trip took about 146 s).
Otherwise ``unevaluated`` (a missing record is neither clear nor an overlap, so it earns no rerun). Stop the sampler
only after a complete sample past the last END + 45 s. Times are ws1's own clock, read in each sample.
"""

from __future__ import annotations

import datetime as dt
import json
import subprocess
import sys
import time

PROBE = ("echo T $(date +%s.%N); for p in $(pgrep -f '^/usr/bin/qemu-system'); do echo qemu $p $(ps -o lstart= -p $p); "
         "done; echo QEMU_DONE; g=$(nvidia-smi --query-gpu=index,pci.bus_id,persistence_mode --format=csv,noheader); "
         "echo GPU_EXIT $?; printf '%s\\n' \"$g\" | sed 's/^/gpu /'")


def parse(text: str) -> dict:
    """Each half on its own: qemu_ok once the process scan finished, gpu_ok when nvidia-smi exited 0."""
    lines = [x.strip() for x in text.splitlines() if x.strip()]
    t = next((float(x[2:]) for x in lines if x.startswith("T ")), None)
    gpu_exit = next((x.split()[1] for x in lines if x.startswith("GPU_EXIT ")), None)
    return {"t_ws1": t, "qemu_ok": "QEMU_DONE" in lines, "qemu": [x[5:] for x in lines if x.startswith("qemu ")],
            "gpu_ok": gpu_exit == "0",
            "gpus": [[c.strip() for c in x[4:].split(",")] for x in lines if x.startswith("gpu ") and x[4:].strip()]}


def sample(path: str, every: float) -> int:
    while True:
        t_local = time.time()
        try:
            r = subprocess.run(["ssh", "-o", "ClearAllForwardings=yes", "-o", "ConnectTimeout=15", "-o",
                                "BatchMode=yes", "ws1", PROBE], capture_output=True, text=True, timeout=45)
            row = {"t_local": t_local, "ssh_exit": r.returncode, **parse(r.stdout), "err": r.stderr[-200:]}
        except (subprocess.TimeoutExpired, ValueError) as exc:
            row = {"t_local": t_local, "t_ws1": None, "err": repr(exc)[-200:]}
        with open(path, "a") as f:
            f.write(json.dumps(row) + "\n")
        time.sleep(max(0.0, t_local + every - time.time()))


def positive(row: dict) -> bool:
    return bool(row.get("qemu_ok") and row["qemu"]) or bool(row.get("gpu_ok") and len(row["gpus"]) < 2)


def complete(row: dict) -> bool:
    return bool(row.get("qemu_ok") and row.get("gpu_ok"))


def judge(rows: list, start: float, end: float, every: float) -> str:
    lo, hi = start - 1.5 * every, end + 1.5 * every
    seen = sorted((r for r in rows if r.get("t_ws1") is not None and lo <= r["t_ws1"] <= hi),
                  key=lambda r: r["t_ws1"])
    if any(positive(r) for r in seen):
        return "vm"
    full = [r for r in seen if complete(r)]
    if not any(r["t_ws1"] <= start for r in full) or not any(r["t_ws1"] >= end for r in full):
        return "unevaluated"
    if any(b["t_ws1"] - a["t_ws1"] > 2 * every for a, b in zip(full, full[1:])):
        return "unevaluated"
    return "clear"


def ws1_time(text: str) -> float:
    # sacct prints ws1's local time without a zone; ws1 and this sampler's t_ws1 (epoch) meet through KST
    return dt.datetime.fromisoformat(text).replace(tzinfo=dt.timezone(dt.timedelta(hours=9))).timestamp()


def main(argv: list) -> int:
    every = 30.0
    if "--every" in argv:
        i = argv.index("--every")
        every = float(argv[i + 1])
        del argv[i:i + 2]
    if argv[1] == "sample":
        return sample(argv[2], every)
    if argv[1] == "judge":
        rows = [json.loads(x) for x in open(argv[2]) if x.strip()]
        rest = argv[3:]
        if len(rest) % 3:
            raise SystemExit("judge takes NAME START END triples")
        for i in range(0, len(rest), 3):
            name, s, e = rest[i:i + 3]
            print(name, judge(rows, ws1_time(s), ws1_time(e), every))
        return 0
    raise SystemExit(__doc__)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
