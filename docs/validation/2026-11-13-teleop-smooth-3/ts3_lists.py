#!/usr/bin/env python3
"""The teleop smoothness confirmation 3 (K09)'s job lists (docs/plans/2026-11-13-teleop-smooth-3.md 1): three single-session
live jobs, r1c_session.sh line_m far confirm, seed 0, INSTR off, the latency batch B's G3 settings, no injected latency,
**LAYER_BUDGET unset** (the launcher's default budget file) and the main the snapshot is cut from (the exact-key caches
included). One arm, K09 (catch-up 6, FACTOR 0.9), three slots.

    python ts3_lists.py SNAP OUT            # writes OUT/<name>.jsonl, prints "<name> <arm>" per job in order
    python ts3_lists.py SNAP OUT NAME ...   # the one rerun of each named job (overlap rule): <name>_re
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
BASE = {"SET": "far", "SEED": "0", "INSTR": "off", "FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1",
        "LIDAR_CAST": "fast"}  # no LAYER_BUDGET: the launcher's default (budget on)
ARMS = {"k09": {"ARM": "K", "FACTOR": "0.9"}}
ORDER = ("k09", "k09", "k09")
SOCK_TAIL = "/live/epoch_999/sock/slam.sock"


def entry(snap: str, out: str, name: str, arm: str) -> dict:
    run = f"{out}/{name}/s"
    # checked with the slot's _re name from the first run on, so an OUT that fits the originals fits their reruns
    # (Codex registration review P3)
    probe = run if name.endswith("_re") else f"{out}/{name}_re/s"
    if len((probe + SOCK_TAIL).encode()) > 107:
        raise SystemExit(f"{probe}: the SLAM socket path would pass 107 bytes -- a shorter OUT")
    env = {**BASE, **ARMS[arm], "TS_ARM": arm}
    assert "LAYER_BUDGET" not in env
    return {"name": name, "argv": ["bash", f"{snap}/{WRAPPER}", run], "cwd": snap, "env": env,
            "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
            "first_record": f"{run}/live/epoch_1/run/scenario.json", "scan": [f"{run}/live/epoch_1/run.log"]}


def jobs(snap: str, out: str) -> list[dict]:
    seen: dict = {}
    rows = []
    for n, arm in enumerate(ORDER, start=1):
        seen[arm] = seen.get(arm, 0) + 1
        rows.append(entry(snap, out, f"u{n:02d}_{arm}_r{seen[arm]}", arm))
    assert len(rows) == 3 and seen == {"k09": 3}
    return rows


def reruns(snap: str, out: str, names: list) -> list[dict]:
    by_name = {j["name"]: j for j in jobs(snap, out)}
    if len(set(names)) != len(names):
        raise SystemExit(f"{names}: a name given twice (one rerun per slot)")
    rows = []
    for n in names:
        if n not in by_name:
            raise SystemExit(f"{n}: not a registered job (a rerun is of an original, once)")
        rows.append(entry(snap, out, f"{n}_re", by_name[n]["env"]["TS_ARM"]))
    return rows


def main(argv: list) -> int:
    snap, out = argv[1], argv[2]
    Path(out).mkdir(parents=True, exist_ok=True)
    for j in (reruns(snap, out, argv[3:]) if len(argv) > 3 else jobs(snap, out)):
        (Path(out) / f"{j['name']}.jsonl").write_text(json.dumps(j) + "\n", encoding="utf-8")
        print(j["name"], j["env"]["TS_ARM"])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
