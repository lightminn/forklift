"""The record audit of the SLAM agreement (the revision plan 9.16, lead's note of 2026-11-07): every re-match's agreement and
the signed true errors (along, across, heading in the truth's frame) of its evidence g and of the applied pose, the agreement
passed or refused, by start class, stand, generation, attempt, scan and result times, the odometry's move between them and
the reverse distance. It prints no value: the table goes to OUT, to be read once the lead has fixed the rule that uses it.
A generation recorded since #351 carries its ``diag`` (per attempt the lattice pose, the pose returned, the ICP's own end,
``refined``, the ICP's reason and the endpoint fraction, laser poses; the conservative check's local result ``res_base``);
the row copies it. An older record has none and lists those fields under ``unrecorded``.

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


def _nearest(samples, t: float):
    rows = [x for x in samples if x.get("o") and x.get("truth")]
    return min(rows, key=lambda x: abs(x["t"] - t)) if rows else None


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _signed(pose, truth) -> list:
    """(along, across, heading) of pose - truth in the truth's frame: the sign kept."""
    dx, dy = pose[0] - truth[0], pose[1] - truth[1]
    c, s = math.cos(truth[2]), math.sin(truth[2])
    return [c * dx + s * dy, -s * dx + c * dy, math.remainder(pose[2] - truth[2], math.tau)]


def _agreements(record: dict) -> list:
    """Every agreement the check judged, passed or not: a shadow's raw generations, an on run's agreement events."""
    raw = (record.get("raw") or {}).get("rematches")
    if raw is not None:
        start = (record.get("raw") or {}).get("start") or {}
        rows = []
        if start.get("agree_m") is not None:  # the start check's own agreement, judged when it confirmed
            rows.append({"t": start["t"], "g": start.get("r"), "agree_m": start["agree_m"], "agree_rad": start["agree_rad"],
                         "ok": None, "kind": "raw_start", "scan_t": start.get("scan_t"), "o_scan": start.get("o_scan"),
                         "qualified": bool(start.get("confirmed")), "applied": None, "C": start.get("C")})
        rows += [{"t": g["t"], "g": g.get("g"), "agree_m": g.get("agree_m"), "agree_rad": g.get("agree_rad"), "ok": None,
                  "kind": "raw", "scan_t": g.get("scan_t"), "o_scan": g.get("o_scan"),
                  "qualified": bool(g.get("residual_confirmed") and g.get("applicable") and g.get("g") is not None
                                    and g.get("agree_m") is not None), "applied": None, "C": g.get("C")} for g in raw]
        return rows
    # an on run judges an agreement only past the conservative check and the discarding: every one qualifies. The start
    # check's agreement (the first with no generation at its time) is judged against the truth at the start's scan, as a
    # generation's is at its own (Codex plan review P2); a generation's scan comes from its record in session_rows
    start = record.get("start") or {}
    rows, start_used = [], False
    for e in record.get("events", []):
        if e.get("kind") not in ("slam_agreement", "rematch_agreement"):
            continue
        own_start = not start_used and _generation_at(record, e["t"]) is None and start.get("scan_t") is not None
        start_used = start_used or own_start
        rows.append({"t": e["t"], "g": e.get("evidence"), "agree_m": e["agree_m"], "agree_rad": e["agree_rad"],
                     "ok": e.get("ok"), "kind": e["kind"], "scan_t": start["scan_t"] if own_start else None,
                     "o_scan": start.get("o_scan") if own_start else None, "qualified": True,
                     "applied": e.get("applied"), "C": None})
    return rows


def _generation_at(record: dict, t: float):
    return next((r for r in record.get("rematches") or [] if abs(r["t"] - t) < 1e-6), None)


def session_rows(path: str) -> tuple:
    """(rows, counts): one row per agreement with its generation; the session's discards, competitor endings, deadlines
    and contacts counted apart."""
    with open(os.path.join(path, "epoch_1", "run", "result.json")) as fh:
        record = json.load(fh).get("pose_check") or {}
    samples = record.get("samples") or []
    first = next((x for x in samples if x.get("o")), None)
    cls = start_class(record, path)
    gens = record.get("rematches") or []
    stand_of = {}
    for r in gens:  # a generation's stand: the odometry it began at
        stand_of.setdefault(tuple(r.get("o_begin") or ()), len(stand_of) + 1)
    rows = []
    for k, ag in enumerate(_agreements(record), 1):
        gen = _generation_at(record, ag["t"])
        stamps = (gen or {}).get("scan_stamps") or []
        scan_t = ag["scan_t"] if ag["scan_t"] is not None else (stamps[-1][-1] if stamps and stamps[-1] else None)
        o_scan = ag["o_scan"] if ag["o_scan"] is not None else (gen or {}).get("o_scan")
        at, scanned = _nearest(samples, ag["t"]), _nearest(samples, scan_t) if scan_t is not None else None
        contact = any(c["t0"] <= ag["t"] <= c["t1"] for c in ((record.get("contacts") or {}).get("intervals")) or [])
        row = {"session": os.path.basename(path.rstrip("/")), "class": cls, "agreement": k, "kind": ag["kind"],
               "ok": ag["ok"], "qualified": ag["qualified"], "in_contact": contact, "t": ag["t"], "scan_t": scan_t, "stand": stand_of.get(tuple((gen or {}).get("o_begin") or ())),
               "generation": gens.index(gen) + 1 if gen in gens else None,
               "source": "rematch" if gen is not None or ag["kind"] == "raw" else "start",
               "attempts": [a.get("reason") for a in (gen or {}).get("attempts") or []] if isinstance((gen or {}).get("attempts"), list) else (gen or {}).get("attempts"),
               "outcome": (gen or {}).get("outcome"), "why": (gen or {}).get("why"),
               "agree_m": ag["agree_m"], "agree_rad": ag["agree_rad"], "delta_o": None, "reverse_m": None,
               "g_err": None, "c_err": None, "wider": None,
               # #351's diagnostics when the generation has them; an older record lists what it lacks
               "diag": (gen or {}).get("diag"),
               "unrecorded": [] if (gen or {}).get("diag") is not None else
               ["lattice_pose", "icp_pose", "refined", "icp_reason", "res_base"]}
        if at is not None and first is not None:
            row["reverse_m"] = math.hypot(at["o"][0] - first["o"][0], at["o"][1] - first["o"][1])
            if o_scan is not None:
                row["delta_o"] = list(_gap(at["o"], o_scan))
            truth = (scanned or at)["truth"]
            # the applied pose the agreement judged: the event's own (on), else the result's correction at the odometry
            # nearest its tick (shadow), never an earlier sample's
            applied = ag["applied"] or (_compose(ag["C"], at["o"]) if ag["C"] is not None else at["applied"])
            row["c_err"] = _signed(applied, at["truth"])
            if ag["g"] is not None:
                row["g_err"] = _signed(ag["g"], truth)
                row["wider"] = "g" if abs(row["g_err"][2]) > abs(row["c_err"][2]) else "c"
                row["g_outside_success"] = math.hypot(*row["g_err"][:2]) > 0.10 or abs(row["g_err"][2]) > 0.05
        rows.append(row)
    events = record.get("events", [])
    counts = {"session": os.path.basename(path.rstrip("/")), "class": cls, "generations": len(gens),
              "discarded": sum(e.get("kind") == "rematch_discarded" for e in events),
              "competitor": sum("competitor" in (r.get("why") or []) for r in gens),
              "deadline": sum(any(a.get("reason") == "deadline" for a in r.get("attempts") or [] if isinstance(a, dict))
                              for r in gens),
              "contacts": len(((record.get("contacts") or {}).get("intervals")) or [])}
    return rows, counts


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    out, folders = Path(args[0]), args[1:]
    rows, counts = [], []
    for f in folders:
        for s in sorted(glob.glob(f)):
            if os.path.exists(os.path.join(s, "epoch_1", "run", "result.json")):
                r, c = session_rows(s)
                rows += r
                counts.append(c)
    out.write_text(json.dumps({"rows": rows, "sessions": counts}, indent=0) + "\n")
    print(f"{len(rows)} generations written to {out}: no value printed")  # the rule comes before the reading
    return 0


if __name__ == "__main__":
    sys.exit(main())
