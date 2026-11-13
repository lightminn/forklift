#!/usr/bin/env python3
"""PR #482's live Isaac smoke (map_start_pose as floats): one single-session
live jobs, r1c_session.sh line_m far confirm, seed 0, INSTR off, the latency batch B's G3 settings, no injected latency,
**LAYER_BUDGET unset** (the launcher's default budget file) and the main the snapshot is cut from (the exact-key caches
included). One session, the console default L04 (catch-up 0, FACTOR 0.4): the live bridge container localizes
on the registry's line_m bundle with its start pose, the path this PR changes.

    python smoke_lists.py SNAP OUT          # writes OUT/<name>.jsonl, prints "<name> <arm>" per job in order
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
BASE = {"SET": "far", "SEED": "0", "INSTR": "off", "FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1",
        "LIDAR_CAST": "fast"}  # no LAYER_BUDGET: the launcher's default (budget on)
ARMS = {"l04": {"ARM": "L", "FACTOR": "0.4"}}
ORDER = ("l04",)
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
        rows.append(entry(snap, out, f"smoke_{arm}", arm))
    assert len(rows) == 1 and seen == {"l04": 1}
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
