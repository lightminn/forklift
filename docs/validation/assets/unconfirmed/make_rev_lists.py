"""The Isaac lists of the monitor revision (docs/plans/2026-11-07-unconfirmed-monitor-revision.md 2 C, 2 D, 9.11), fixed
before any of them runs, from seed 4's starts outside the D list (dev_starts.jsonl). No seed 5 row is read.

- tail_starts.jsonl (9.11): 20 starts the start check confirmed with a clear rear, a mission after the strokes: the window
  limit's tail and the post-mission watch. Used for the W2 value alone.
- inject_starts.jsonl (2 C): 24 starts, 3 each of the eight wrong estimates of V1b. May overlap the tail list: they measure
  the start check, not the window.
- loop_starts.jsonl (2 D): the tail list's first 3 starts under each of 4 correction jumps, the check on.

    python docs/validation/assets/unconfirmed/make_rev_lists.py
"""

import json
import random
from pathlib import Path

HERE = Path(__file__).resolve().parent
SEED = 20261108
KINDS = ("full", "half")
WRONG = ("shift x+1", "shift x-1", "shift y+1", "shift y-1", "shift x+2", "shift y-2", "turn", "slide")
# x, y, yaw, sim_s: the transport is under way by 200 s in D's missions, and 125 s is usually inside a hold
JUMPS = {"step_0.35": "0.35,0,0,200", "step_6": "6,0,0,200", "yaw_0.12": "0,0,0.12,200", "hold_yaw_0.12": "0,0,0.12,125"}


def rows() -> list:
    used = {json.loads(line)["id"] for line in (HERE / "dev_starts.jsonl").read_text().splitlines()}
    all_ = [json.loads(line) for line in (HERE / "s4_starts_rows.jsonl").read_text().splitlines()]
    return sorted((r for r in all_ if r["id"] not in used), key=lambda r: r["id"])


def lists(pool) -> dict:
    rng = random.Random(SEED)
    clear = [r for r in pool if r["confirmed"] and r["rear_clear"]]
    tail = rng.sample(clear, 20)
    inject = rng.sample(pool, 24)
    return {
        "tail_starts.jsonl": [{"id": r["id"], "start_pose": r["truth"], "kind": KINDS[k % 2]} for k, r in enumerate(tail)],
        "inject_starts.jsonl": [{"id": r["id"], "start_pose": r["truth"], "inject": WRONG[k % 8], "kind": KINDS[k % 2]}
                                for k, r in enumerate(inject)],
        "loop_starts.jsonl": [{"id": r["id"], "start_pose": r["truth"], "jump": name, "correction_inject": value}
                              for name, value in JUMPS.items() for r in tail[:3]],
    }


if __name__ == "__main__":
    for name, out in lists(rows()).items():
        (HERE / name).write_text("".join(json.dumps(r) + "\n" for r in out))
