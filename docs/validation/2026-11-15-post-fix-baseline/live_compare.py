"""Post-fix Isaac baseline: the live continuous-transport arms beside the pre-fix status survey, mission by mission
(plan docs/plans/2026-11-15-post-fix-baseline.md 2.3, 3.1 and 4.1, column L). No verdict, no threshold.

Both inputs are ``missions.csv`` files as ``tools/ct_survey.py --csv`` writes them (one row per scheduled or started
mission). Only the columns the survey comparison may use (plan 4.2) are read: arm, scene, session, order, start,
outcome, reason, stage, class, delivery_error, started_s, ended_s, forbidden_contacts.

- ``--arm-map 'LD=D,LO=O6'`` names each new arm and the pre-fix arm it sits beside. Arms the map does not name (the
  survey's D' ``Dp`` and ``O0``, or anything else) are left out and listed in ``ignored_arms``.
- Mission counts, not ratios: scheduled (sessions x ``--scheduled``), listed rows, start confirmed, done, failed
  (start confirmed but not done), not submitted, and any other start state (``unknown``/``rejected``) counted apart.
- A numeric cell that is empty or not a finite number is left out of its statistic and counted in that statistic's
  ``missing`` -- never a default. Delivery error and time are read from done missions; time is the simulated
  ``ended_s - started_s``. Forbidden contacts are summed, their ``missing`` counted over start-confirmed missions.
- Pairs: the new and pre-fix sessions of one arm and scene share the seed in the session name (``..._s<n>``); the
  missions of the same order whose outcome differs are listed. Time and delivery error are not paired (plan 1: live
  deterministic sessions differ in time from the first mission).
- An arm, scene, seed and order seen twice in one file is a ValueError (the pairing would be a guess), as is a compared
  arm's row without an integer order or a session name ending in ``_s<seed>``.

  python live_compare.py --new NEW/missions.csv --prefix docs/validation/2026-11-06-ct-status-survey/missions.csv \\
      --arm-map 'LD=D,LO=O6' --out live_compare.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

import numpy as np

COLUMNS = ("arm", "scene", "session", "order", "start", "outcome", "reason", "stage", "class", "delivery_error",
           "started_s", "ended_s", "forbidden_contacts")
SCENES = ("line_m", "random_m")
SEED = re.compile(r"_s(\d+)$")
EMPTY = "(empty)"  # the key a blank class or stage is counted under


def parse_arm_map(text: str) -> dict:
    """``'LD=D,LO=O6'`` -> {"LD": "D", "LO": "O6"} (new name -> pre-fix name)."""
    out: dict = {}
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        new, sep, old = (x.strip() for x in part.partition("="))
        if not sep or not new or not old:
            raise ValueError(f"arm map entry {part!r} is not NEW=PREFIX")
        if new in out:
            raise ValueError(f"arm map names {new!r} twice")
        out[new] = old
    if not out:
        raise ValueError("empty arm map")
    return out


def _cell(v) -> str | None:
    v = (v or "").strip()
    return v or None


def _number(v) -> float | None:
    """A finite float, else None (empty or not a number: the caller counts it missing)."""
    if v is None:
        return None
    try:
        x = float(v)
    except ValueError:
        return None
    return x if math.isfinite(x) else None


def read_missions(path: Path) -> list:
    """The rows of one missions.csv, the used columns only, text cells stripped (empty -> None)."""
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        absent = [c for c in COLUMNS if c not in (reader.fieldnames or [])]
        if absent:
            raise ValueError(f"{path}: missing columns {absent}")
        rows = []
        for i, raw in enumerate(reader, start=2):
            row = {c: _cell(raw.get(c)) for c in COLUMNS}
            row["line"] = f"{path}:{i}"
            if row["arm"] is None:
                raise ValueError(f"{row['line']}: empty arm")
            rows.append(row)
    return rows


def keyed(row: dict) -> dict:
    """The row with an integer ``order`` and the ``seed`` its session name ends in; a ValueError when either cannot
    be read (only the arms being compared are keyed, so an ignored arm's rows are not held to this)."""
    for c in ("scene", "session", "order"):
        if row[c] is None:
            raise ValueError(f"{row['line']}: empty {c}")
    try:
        order = int(row["order"])
    except (TypeError, ValueError):
        raise ValueError(f"{row['line']}: order {row['order']!r} is not an integer") from None
    m = SEED.search(row["session"])
    if not m:
        raise ValueError(f"{row['line']}: session {row['session']!r} does not end in _s<seed>")
    return {**row, "order": order, "seed": int(m.group(1))}


def index(rows: list, label: str) -> dict:
    """{(arm, scene, seed): {order: row}} of keyed rows; a key seen twice is a ValueError."""
    out: dict = {}
    for r in rows:
        missions = out.setdefault((r["arm"], r["scene"], r["seed"]), {})
        if r["order"] in missions:
            raise ValueError(f"{label}: arm {r['arm']} scene {r['scene']} seed {r['seed']} order {r['order']} "
                             f"appears twice ({missions[r['order']]['session']}, {r['session']})")
        missions[r["order"]] = r
    return out


def _stats(values: list, missing: int) -> dict:
    if not values:
        return {"n": 0, "median": None, "p90": None, "max": None, "missing": missing}
    return {"n": len(values), "median": float(np.median(values)), "p90": float(np.percentile(values, 90)),
            "max": float(max(values)), "missing": missing}


def summarize(rows: list, scheduled: int = 3) -> dict:
    """Descriptive counts of one arm and scene's missions."""
    sessions: dict = {}
    for r in rows:
        sessions.setdefault(r["session"], []).append(r)
    confirmed = [r for r in rows if r["start"] == "confirmed"]
    done = [r for r in rows if r["outcome"] == "done"]
    failed = [r for r in confirmed if r["outcome"] != "done"]
    errors, err_missing = [], 0
    times, time_missing = [], 0
    for r in done:
        e = _number(r["delivery_error"])
        if e is None:
            err_missing += 1
        else:
            errors.append(e)
        t0, t1 = _number(r["started_s"]), _number(r["ended_s"])
        if t0 is None or t1 is None:
            time_missing += 1
        else:
            times.append(t1 - t0)
    contacts, contact_missing = 0, 0
    for r in rows:
        c = _number(r["forbidden_contacts"])
        if c is not None and c.is_integer():
            contacts += int(c)
        elif r["start"] == "confirmed":
            contact_missing += 1
    order = sorted(failed, key=lambda r: (r["session"], r["order"]))
    return {
        "sessions": len(sessions),
        "sessions_all_done": sum(1 for ms in sessions.values() if all(r["outcome"] == "done" for r in ms)),
        "missions": {
            "scheduled": len(sessions) * scheduled,
            "listed": len(rows),
            "confirmed": len(confirmed),
            "done": len(done),
            "failed": len(failed),
            "not_submitted": sum(1 for r in rows if r["start"] == "not_submitted"),
            "start_other": dict(Counter(r["start"] or EMPTY for r in rows
                                        if r["start"] not in ("confirmed", "not_submitted"))),
        },
        "failed_classes": dict(Counter(r["class"] or EMPTY for r in failed)),
        "failed_stages": dict(Counter(r["stage"] or EMPTY for r in failed)),
        "failed_missions": [{"session": r["session"], "order": r["order"], "outcome": r["outcome"],
                             "reason": r["reason"]} for r in order],
        "delivery_error_m": _stats(errors, err_missing),
        "mission_time_s": _stats(times, time_missing),
        "forbidden_contacts": {"total": contacts, "missing": contact_missing},
    }


def _side(r: dict | None) -> dict:
    if r is None:
        return {"session": None, "outcome": None, "reason": None, "listed": False}
    return {"session": r["session"], "outcome": r["outcome"], "reason": r["reason"], "listed": True}


def pair_differences(new_idx: dict, pre_idx: dict, new_arm: str, pre_arm: str, scene: str) -> dict:
    """Same seed, same order: the missions whose outcome differs (a mission listed on one side only counts as
    differing when the listed side has an outcome)."""
    new = {k[2]: v for k, v in new_idx.items() if k[0] == new_arm and k[1] == scene}
    pre = {k[2]: v for k, v in pre_idx.items() if k[0] == pre_arm and k[1] == scene}
    common = sorted(set(new) & set(pre))
    changed = []
    for seed in common:
        for order in sorted(set(new[seed]) | set(pre[seed])):
            a, b = new[seed].get(order), pre[seed].get(order)
            if (a or {}).get("outcome") != (b or {}).get("outcome"):
                changed.append({"seed": seed, "order": order, "new": _side(a), "prefix": _side(b)})
    return {"common_seeds": common, "only_new_seeds": sorted(set(new) - set(pre)),
            "only_prefix_seeds": sorted(set(pre) - set(new)), "changed": changed}


def compare(new_rows: list, pre_rows: list, arm_map: dict, scheduled: int = 3) -> dict:
    wanted_pre = set(arm_map.values())
    ignored = {"new": sorted({r["arm"] for r in new_rows} - set(arm_map)),
               "prefix": sorted({r["arm"] for r in pre_rows} - wanted_pre)}
    new_rows = [keyed(r) for r in new_rows if r["arm"] in arm_map]
    pre_rows = [keyed(r) for r in pre_rows if r["arm"] in wanted_pre]
    new_idx, pre_idx = index(new_rows, "new"), index(pre_rows, "prefix")
    arms: dict = {}
    for new_arm, pre_arm in arm_map.items():
        n = [r for r in new_rows if r["arm"] == new_arm]
        p = [r for r in pre_rows if r["arm"] == pre_arm]
        scenes = list(SCENES) + sorted({r["scene"] for r in n + p} - set(SCENES))
        arms[new_arm] = {"prefix_arm": pre_arm, "scenes": {
            s: {"new": summarize([r for r in n if r["scene"] == s], scheduled),
                "prefix": summarize([r for r in p if r["scene"] == s], scheduled),
                "pairs": pair_differences(new_idx, pre_idx, new_arm, pre_arm, s)} for s in scenes}}
    return {"arms": arms, "ignored_arms": ignored}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--new", type=Path, required=True, help="this baseline's missions.csv")
    ap.add_argument("--prefix", type=Path, required=True, help="the pre-fix survey's missions.csv")
    ap.add_argument("--arm-map", required=True, help="NEW=PREFIX[,NEW=PREFIX...], e.g. 'LD=D,LO=O6'")
    ap.add_argument("--scheduled", type=int, default=3, help="missions scheduled per session")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    arm_map = parse_arm_map(a.arm_map)
    out = {"schema": "post_fix_live_compare/1", "new": str(a.new), "prefix": str(a.prefix), "arm_map": arm_map,
           "scheduled_per_session": a.scheduled,
           **compare(read_missions(a.new), read_missions(a.prefix), arm_map, a.scheduled)}
    a.out.write_text(json.dumps(out, indent=1, sort_keys=True, allow_nan=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
