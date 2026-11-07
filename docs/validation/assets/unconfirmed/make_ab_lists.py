"""The two small seed 4 lists of the lead's decision of 2026-11-07 (docs/plans/2026-11-07-unconfirmed-monitor-revision.md),
fixed before either runs. make_rev_lists.py and make_dev_list.py cannot make them.

- a_starts.jsonl: the F1 path's reach, the check on: the 11 recorded wrong confirmations (s4_wrong_rows.jsonl, 9 starts),
  once each in the file's order. click_kind is the record click, reverse_kind the pressing rule, in turn.
- b_starts.jsonl: the naturally unconfirmed starts 032, 052, 072 under full and half, the check on, a mission after it.

    python docs/validation/assets/unconfirmed/make_ab_lists.py
"""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
RULES = ("full", "half")
NATURAL = ("val3_032", "val3_052", "val3_072")


def _rows(name) -> list:
    return [json.loads(line) for line in (HERE / name).read_text().splitlines()]


def a_list() -> list:
    return [{"id": r["id"], "start_pose": r["truth"], "click_kind": r["kind"], "reverse_kind": RULES[k % 2]}
            for k, r in enumerate(_rows("s4_wrong_rows.jsonl"))]


def b_list() -> list:
    by_id = {r["id"]: r for r in _rows("s4_starts_rows.jsonl")}
    return [{"id": i, "start_pose": by_id[i]["truth"], "reverse_kind": k} for i in NATURAL for k in RULES]


if __name__ == "__main__":
    for name, rows in (("a_starts.jsonl", a_list()), ("b_starts.jsonl", b_list())):
        (HERE / name).write_text("".join(json.dumps(r) + "\n" for r in rows))
