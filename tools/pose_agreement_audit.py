"""The record audit of the SLAM agreement (the revision plan 9.16, lead's note of 2026-11-07): every re-match's agreement and
the true errors of its evidence g and of the applied pose, by start class, reverse distance and attempt. It prints no
value: the table goes to OUT, to be read once the lead has fixed the rule that uses it.

    python tools/pose_agreement_audit.py OUT.json FOLDER...

A FOLDER is a glob of sessions, each with epoch_1/run/result.json. A shadow session's generations are its raw records; an on
session's are its agreement events. ``wider`` says which pose stands farther from the truth in heading.
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys
from pathlib import Path

NATURAL_UNCONFIRMED = ("val3_016", "val3_074", "val3_032", "val3_052")  # the lead's list; 072 never reached the check


def _gap(a, b) -> tuple:
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(math.remainder(a[2] - b[2], math.tau))


def _start_id(session: str) -> str:
    for part in os.path.basename(session.rstrip("/")).split("_"):
        if part.isdigit() and len(part) == 3:
            return f"val3_{part}"
    return os.path.basename(session.rstrip("/"))


def start_class(record: dict, session: str) -> str:
    """confirmed / natural_unconfirmed / competitor: the start check's own result, the natural list named by id."""
    if _start_id(session) in NATURAL_UNCONFIRMED:
        return "natural_unconfirmed"
    start = record.get("start") or {}
    result = start.get("result") or {}
    raw = (record.get("raw") or {}).get("start") or {}
    if raw.get("confirmed") or result.get("confirmed"):
        return "confirmed"
    return "competitor"


def _sample_at(samples, t: float):
    rows = [x for x in samples if x.get("o") and x.get("truth") and x["t"] <= t]
    return rows[-1] if rows else None


def _generations(record: dict) -> list:
    """(t, g, agree_m, agree_rad, kind) of each generation with an agreement; g None when the record has none."""
    raw = (record.get("raw") or {}).get("rematches")
    if raw is not None:
        return [(g["t"], g.get("g"), g["agree_m"], g["agree_rad"], "raw") for g in raw if g.get("agree_m") is not None]
    return [(e["t"], e.get("evidence"), e["agree_m"], e["agree_rad"], e["kind"]) for e in record.get("events", [])
            if e.get("kind") in ("slam_agreement", "rematch_agreement")]


def session_rows(path: str) -> list:
    record = json.load(open(os.path.join(path, "epoch_1", "run", "result.json"))).get("pose_check") or {}
    samples = record.get("samples") or []
    first = next((x for x in samples if x.get("o")), None)
    cls = start_class(record, path)
    rows = []
    for attempt, (t, g, agree_m, agree_rad, kind) in enumerate(_generations(record), 1):
        at = _sample_at(samples, t)
        row = {"session": os.path.basename(path.rstrip("/")), "class": cls, "attempt": attempt, "t": t, "kind": kind,
               "agree_m": agree_m, "agree_rad": agree_rad, "reverse_m": None, "g_err": None, "c_err": None, "wider": None}
        if at is not None and first is not None:
            row["reverse_m"] = math.hypot(at["o"][0] - first["o"][0], at["o"][1] - first["o"][1])
            row["c_err"] = list(_gap(at["applied"], at["truth"]))
            if g is not None:
                row["g_err"] = list(_gap(g, at["truth"]))
                row["wider"] = "g" if row["g_err"][1] > row["c_err"][1] else "c"
        rows.append(row)
    return rows


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    out, folders = Path(args[0]), args[1:]
    rows = [r for f in folders for s in sorted(glob.glob(f)) if os.path.exists(os.path.join(s, "epoch_1", "run", "result.json"))
            for r in session_rows(s)]
    out.write_text(json.dumps({"rows": rows}, indent=0) + "\n")
    print(f"{len(rows)} generations written to {out}: no value printed")  # the rule comes before the reading
    return 0


if __name__ == "__main__":
    sys.exit(main())
