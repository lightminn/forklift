"""The D sessions' start list (docs/plans/2026-11-06-unconfirmed-validation.md 3.1), fixed before any D result: 30 of seed
4's unattended-confirmed starts drawn at a fixed seed, the pressing rule turned through full, half, chatter (10 each),
and within each rule every other start with the mission after the strokes (15 of 30).

    python docs/validation/assets/unconfirmed/make_dev_list.py > docs/validation/assets/unconfirmed/dev_starts.jsonl
"""

import json
import random
from pathlib import Path

HERE = Path(__file__).resolve().parent
SEED = 20261106
COUNT = 30
KINDS = ("full", "half", "chatter")


def rows() -> list:
    return [json.loads(line) for line in (HERE / "s4_starts_rows.jsonl").read_text().splitlines()]


def dev_list(starts) -> list:
    drawn = random.Random(SEED).sample(sorted(starts, key=lambda r: r["id"]), COUNT)
    return [{"id": r["id"], "start_pose": r["truth"], "kind": KINDS[k % 3], "mission": (k // 3) % 2 == (k % 3) % 2,
             "start_confirmed": r["confirmed"], "rear_clear": r["rear_clear"]} for k, r in enumerate(drawn)]


if __name__ == "__main__":
    for row in dev_list(rows()):
        print(json.dumps(row))
