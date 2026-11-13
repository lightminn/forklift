#!/usr/bin/env python3
"""The VM rule's evidence (docs/plans/2026-11-13-teleop-smooth-2.md 2, teleop-smooth-3.md 2): ws1's VM
``ws1-win-gpu1`` takes GPU 1 (vfio-pci) and its qemu runs as libvirt-qemu, so neither the bundle's CPU sampler nor
sacct sees it, and its libvirt log is root-only. This samples, from outside ws1, whether a qemu-system process runs and
which GPUs nvidia-smi lists, and then reads each session window against the samples.

    python vm_watch.py sample WATCH.jsonl [--every 30]     # until stopped; one JSON line per sample, ssh failures too
    python vm_watch.py judge WATCH.jsonl NAME START END ... # START/END as sacct prints them (ws1 local time)

A window is ``vm`` when a good sample inside it (or within one period of either edge) shows qemu or fewer than two
GPUs; ``clear`` when good samples cover it -- one at or before START, one at or after END, no two consecutive more than
two periods apart -- and none shows the VM; otherwise ``unevaluated`` (a missing record is neither clear nor an
overlap, so it earns no rerun). Times are ws1's own clock, read in each sample.
"""

from __future__ import annotations

import datetime as dt
import json
import subprocess
import sys
import time

PROBE = ("date +%s.%N; for p in $(pgrep -f '^/usr/bin/qemu-system'); do echo qemu $p $(ps -o lstart= -p $p); done; "
         "nvidia-smi --query-gpu=index,pci.bus_id,persistence_mode --format=csv,noheader")


def parse(text: str) -> dict:
    lines = [x.strip() for x in text.strip().splitlines() if x.strip()]
    t = float(lines[0])
    qemu = [x[5:] for x in lines[1:] if x.startswith("qemu ")]
    gpus = [[c.strip() for c in x.split(",")] for x in lines[1:] if not x.startswith("qemu ")]
    return {"t_ws1": t, "qemu": qemu, "gpus": gpus}


def sample(path: str, every: float) -> int:
    while True:
        t_local = time.time()
        try:
            r = subprocess.run(["ssh", "-o", "ClearAllForwardings=yes", "-o", "ConnectTimeout=15", "-o",
                                "BatchMode=yes", "ws1", PROBE], capture_output=True, text=True, timeout=45)
            row = {"t_local": t_local, "ok": r.returncode == 0, **(parse(r.stdout) if r.returncode == 0 else
                                                                    {"err": r.stderr[-200:]})}
        except (subprocess.TimeoutExpired, ValueError, IndexError) as exc:
            row = {"t_local": t_local, "ok": False, "err": repr(exc)[-200:]}
        with open(path, "a") as f:
            f.write(json.dumps(row) + "\n")
        time.sleep(max(0.0, t_local + every - time.time()))


def vm_seen(row: dict) -> bool:
    return bool(row["qemu"]) or len(row["gpus"]) < 2


def judge(rows: list, start: float, end: float, every: float) -> str:
    good = sorted((r for r in rows if r.get("ok")), key=lambda r: r["t_ws1"])
    near = [r for r in good if start - every <= r["t_ws1"] <= end + every]
    if any(vm_seen(r) for r in near):
        return "vm"
    before = [r for r in good if r["t_ws1"] <= start]
    after = [r for r in good if r["t_ws1"] >= end]
    if not before or not after:
        return "unevaluated"
    span = [before[-1]] + [r for r in good if start < r["t_ws1"] < end] + [after[0]]
    if any(b["t_ws1"] - a["t_ws1"] > 2 * every for a, b in zip(span, span[1:])):
        return "unevaluated"
    if any(vm_seen(r) for r in span):
        return "vm"
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
