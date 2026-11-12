#!/usr/bin/env python3
"""The teleop smoothness confirmation's job lists (docs/plans/2026-11-12-teleop-smooth-confirmation.md 2): six
single-session live jobs, r1c_session.sh line_m far confirm, seed 0, INSTR off, the latency batch B's G3 settings, no
injected latency -- arm K09 (FACTOR 0.9, catch-up 6) and arm L04 (FACTOR 0.4, catch-up 0: the console launcher's
default), in the registered order K L L K K L.

    python ts_lists.py SNAP OUT     # writes OUT/<name>.jsonl, prints "<name> <arm>" per job in order
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
BASE = {"SET": "far", "SEED": "0", "INSTR": "off", "LAYER_BUDGET": "off", "FORKLIFT_GRID_CLEAR_KERNEL": "1",
        "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": "fast"}
ARMS = {"k09": {"ARM": "K", "FACTOR": "0.9"}, "l04": {"ARM": "L", "FACTOR": "0.4"}}
ORDER = ("k09", "l04", "l04", "k09", "k09", "l04")
SOCK_TAIL = "/live/epoch_999/sock/slam.sock"


def entry(snap: str, out: str, name: str, arm: str) -> dict:
    run = f"{out}/{name}/s"
    if len((run + SOCK_TAIL).encode()) > 107:
        raise SystemExit(f"{run}: the SLAM socket path would pass 107 bytes -- a shorter OUT")
    return {"name": name, "argv": ["bash", f"{snap}/{WRAPPER}", run], "cwd": snap,
            "env": {**BASE, **ARMS[arm], "TS_ARM": arm},
            "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
            "first_record": f"{run}/live/epoch_1/run/scenario.json", "scan": [f"{run}/live/epoch_1/run.log"]}


def jobs(snap: str, out: str) -> list[dict]:
    seen: dict = {}
    rows = []
    for n, arm in enumerate(ORDER, start=1):
        seen[arm] = seen.get(arm, 0) + 1
        rows.append(entry(snap, out, f"ts{n}_{arm}_r{seen[arm]}", arm))
    assert len(rows) == 6 and seen == {"k09": 3, "l04": 3}
    return rows


def main(argv: list) -> int:
    snap, out = argv[1], argv[2]
    Path(out).mkdir(parents=True, exist_ok=True)
    for j in jobs(snap, out):
        (Path(out) / f"{j['name']}.jsonl").write_text(json.dumps(j) + "\n", encoding="utf-8")
        print(j["name"], j["env"]["TS_ARM"])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
