"""Continuous transport status survey (docs/plans/2026-11-06-ct-status-survey.md): live queue sessions -> one row per
scheduled mission with its start state, outcome, stage, failure class and the evidence paths, the stage exposure, the
queue's own stops, the loaded-transport tracking failures flagged apart, and a mission-level comparison of same-seed
pairs. A description, not a verdict: no thresholds.

    python tools/ct_survey.py OUT --json SURVEY [--csv MISSIONS] [--scheduled 3] [--pairs A_DIR:B_DIR ...]

OUT holds one folder per session attempt (``probe.json`` and ``live/epoch_*/run``); a folder's ``survey_arm.txt``
(one line: the arm name) groups it, else the folder name's first ``_`` field."""

from __future__ import annotations

import argparse
import csv
import glob
import json
import re
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mission_queue import UNSURE_REJECTS  # noqa: E402

# The mission phases in the order a mission runs them (forklift_core/mission/core.py transitions), with the lifecycle's
# "preparing" first: an accepted mission prepares before its first transition (observe) is added, and can end there
# (prepare:timeout, prepare:all_candidates_failed, prepare:reference:*). A mission that ended in one reached every
# earlier one unless the session's transitions say otherwise.
PHASE_ORDER = ("preparing", "observe", "approach", "insert", "lift", "extract", "transport", "lower", "withdraw", "return_home",
               "complete")

# Failure classes by reason string (plan section 3); the first match wins. New strings are reported as "other" with
# the string kept, and the results document adds a rule for them.
CLASSES = (
    ("tracking", re.compile(r"^Tracking (failed|timeout) in ")),
    ("slam_link", re.compile(r"^(slam_link_failed|slam_not_localized|slam_map_)")),
    ("perception", re.compile(r"^(observe:|near_|recognition|no pallet|bottom_deck)", re.I)),
    ("preparation", re.compile(r"^prepare:")),
    ("docking", re.compile(r"(transport_recovery_after_docking|Pallet missed|delivery|docking)", re.I)),
    ("planning", re.compile(r"(scene_replan|permit_stop|invalid_start|cusp|plan(ning)? (failed|refused)|no path)", re.I)),
    ("load_safety", re.compile(r"(tilt|slip|contact|tip)", re.I)),
)


def classify(reason: str | None) -> str:
    if not reason:
        return "unknown"
    for name, pattern in CLASSES:
        if pattern.search(reason):
            return name
    return "other"


def stage_of(reason: str | None, phase: str | None) -> str | None:
    """The stage a reason names ('Tracking failed in transport' -> transport; 'prepare:*' -> preparing), else the
    mission's last phase (None when neither says: unknown)."""
    if (reason or "").startswith("prepare:"):
        return "preparing"
    m = re.search(r" in ([a-z_]+)\s*$", reason or "")
    if m and m.group(1) in PHASE_ORDER:
        return m.group(1)
    return phase


def _json(path) -> dict | None:
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def start_states(probe: dict | None, scheduled: int) -> list[dict]:
    """The scheduled missions' start states from the probe's submissions, one per queue item (a resubmitted item keeps
    its last answer): confirmed (ack), unknown (no answer, or a busy/stale reject the queue holds unsure), rejected (a
    settled reject), not_submitted (the queue stopped before sending it)."""
    items: dict = {}
    order: list = []
    for entry in (probe or {}).get("missions_run") or []:
        key = entry.get("item_id") or f"{entry.get('pallet_id')}:{entry.get('destination_id')}:{len(order)}"
        if key not in items:
            order.append(key)
        submit = entry.get("submit") or {}
        if submit.get("t") == "ack":
            state = "confirmed"
        elif submit.get("t") == "reject" and submit.get("reason") not in UNSURE_REJECTS:
            state = "rejected"
        else:
            state = "unknown"
        items[key] = {"item": key, "pallet_id": entry.get("pallet_id"), "destination_id": entry.get("destination_id"),
                      "start": state, "mission_id": submit.get("mission_id"), "reject_reason": submit.get("reason"),
                      "probe_result": entry.get("result")}
    out = [items[k] for k in order]
    for _ in range(max(0, scheduled - len(out))):
        out.append({"item": None, "start": "not_submitted", "mission_id": None})
    return out


def epoch_runs(session: Path) -> list[dict]:
    """Each epoch's run: its mission folders (result, phase at the end, safety records) and its run record."""
    out = []
    for run in sorted(glob.glob(str(session / "live" / "epoch_*" / "run")), key=lambda p: int(re.search(r"epoch_(\d+)", p).group(1))):
        missions = {}
        for md in sorted(glob.glob(f"{run}/missions/*")):
            res = (_json(f"{md}/result.json") or {}).get("result")
            rec = _json(f"{md}/records.json")
            st = (rec or {}).get("state") or {}
            plans = ((rec or {}).get("obstacle") or {}).get("grid_plans") or []
            retries = st.get("docking_retries") or []
            missions[Path(md).name] = {"result": res, "records": rec is not None, "phase": st.get("phase"),
                                       "plans": len(plans), "plans_slam_used": sum(bool(g.get("slam_used")) for g in plans),
                                       # the static layer the plans used: on the survey map a localized run uses it
                                       # in both modes; off_deterministic only without a survey map (plan D4)
                                       "plans_static_off": sum((g.get("slam_map_note") or {}).get("slam_static_layer")
                                                               == "off_deterministic" for g in plans),
                                       "docking_retries": len(retries),
                                       "forbidden_contacts": len(st.get("forbidden_pocket_contacts") or []),
                                       "lift_tilt_peak": st.get("lift_tilt_peak"),
                                       "docking_rounds": len(((st.get("delivery_docking") or {}).get("rounds")) or []),
                                       "delivery_error": st.get("delivery_error_m"),
                                       "folder": md}
        manifest = _json(f"{run}/spool/manifest.json")
        spool = None if manifest is None else {
            "complete": manifest.get("complete"), "error_count": manifest.get("error_count"),
            "dropped": {k: v.get("dropped") for k, v in manifest.items() if isinstance(v, dict) and v.get("dropped")}}
        out.append({"run": run, "missions": missions, "result": _json(f"{run}/result.json"), "spool": spool})
    return out


def mission_phases(transitions: list, started: float | None, ended: float | None) -> list[str] | None:
    """The phases the session's transitions show inside a mission's interval, in order (None without the interval)."""
    if started is None or not transitions:
        return None
    seq = []
    for t in transitions:
        ts = t.get("time_s")
        if ts is None or ts < started - 1e-6 or (ended is not None and ts > ended + 1e-6):
            continue
        for ph in (t.get("from"), t.get("to")):
            if ph and (not seq or seq[-1] != ph):
                seq.append(ph)
    return seq or None


def substage(stage: str | None, m: dict) -> str | None:
    """Within the loaded legs, where the delivery docking stood: before its match, after a match, or in a D7
    re-approach (the same stage can need different fixes)."""
    if stage not in ("transport", "lower", "withdraw"):
        return None
    if m.get("docking_retries"):
        return "reapproach"
    return "after_delivery_match" if m.get("docking_rounds") else "before_delivery_match"


def exposure_from(phases: list[str] | None, end_phase: str | None, done: bool) -> tuple[list[str], str]:
    """The stages a mission entered and where that came from: the session's transitions (with preparing, which every
    started mission entered), else the phase order up to its end phase (all for a done mission)."""
    if phases:
        return [p for p in PHASE_ORDER if p in phases or p == "preparing"], "transitions"
    if done:
        return list(PHASE_ORDER), "order"
    if end_phase in PHASE_ORDER:
        return list(PHASE_ORDER[: PHASE_ORDER.index(end_phase) + 1]), "order"
    return [], "none"


def session_rows(session: Path, scheduled: int = 3) -> dict:
    """One session attempt: its missions (start state, outcome, stage, class, evidence), the queue's stop and the run
    records' session end."""
    probe = _json(session / "probe.json")
    starts = start_states(probe, scheduled)
    runs = epoch_runs(session)
    by_id: dict = {}
    for k, r in enumerate(runs):
        for mid, m in r["missions"].items():
            by_id[(k, mid)] = (r, m)
    rows = []
    used = set()
    for idx, s in enumerate(starts):
        row = {"session": session.name, "order": idx + 1, "start": s["start"], "pallet_id": s.get("pallet_id"),
               "destination_id": s.get("destination_id"), "mission_id": s.get("mission_id"),
               "reject_reason": s.get("reject_reason"), "outcome": None, "reason": None, "stage": None,
               "class": None, "exposure": [], "exposure_from": "none", "evidence": None, "epoch": None,
               "forbidden_contacts": None, "lift_tilt_peak": None, "docking_rounds": None, "delivery_error": None,
               "ended_s": None, "substage": None, "plans": None, "plans_slam_used": None, "plans_static_off": None}
        if s["start"] != "confirmed" and s["start"] != "unknown":
            rows.append(row)
            continue
        found = None
        for k, r in enumerate(runs):  # the mission's folder in the epoch that ran it (ids restart per epoch: newest first)
            if s.get("mission_id") and (k, s["mission_id"]) in by_id and (k, s["mission_id"]) not in used:
                found = (k, *by_id[(k, s["mission_id"])])
        if found is not None:
            k, r, m = found
            used.add((k, s["mission_id"]))
            res = m["result"] or {}
            transitions = (r["result"] or {}).get("transitions") or []
            phases = mission_phases(transitions, res.get("started_s"), res.get("ended_s"))
            done = res.get("outcome") == "done"
            row.update(outcome=res.get("outcome"), reason=res.get("reason"), epoch=k + 1, evidence=m["folder"],
                       forbidden_contacts=m["forbidden_contacts"], lift_tilt_peak=m["lift_tilt_peak"],
                       docking_rounds=m["docking_rounds"], delivery_error=m["delivery_error"], ended_s=res.get("ended_s"))
            row["stage"] = None if done else stage_of(res.get("reason"), m["phase"])
            row["substage"] = substage(row["stage"], m)
            row.update(plans=m["plans"], plans_slam_used=m["plans_slam_used"], plans_static_off=m["plans_static_off"])
            row["exposure"], row["exposure_from"] = exposure_from(phases, row["stage"] or m["phase"], done)
            if res.get("outcome") is None:
                row["outcome"] = "no_record"
        else:
            # no folder: the mission the session ended in, read from that epoch's run record -- only for the mission
            # running then (the last started one with no result of its own), never over a settled result
            last_run = runs[-1] if runs else None
            top = (last_run or {}).get("result") or {}
            later_started = any(t["start"] in ("confirmed", "unknown") for t in starts[idx + 1:])
            if s["start"] == "confirmed" and not later_started and top.get("success") is False:
                transitions = top.get("transitions") or []
                end_phase = transitions[-1].get("to") if transitions else None
                row.update(outcome="session_failed", reason=top.get("failure_reason"), epoch=len(runs),
                           evidence=last_run["run"] if last_run else None)
                row["stage"] = stage_of(top.get("failure_reason"), end_phase)
                row["exposure"], row["exposure_from"] = exposure_from(None, row["stage"], False)
            else:
                row["outcome"] = "no_record" if s["start"] == "confirmed" else "not_started_or_unknown"
        if row["outcome"] not in (None, "done"):
            row["class"] = classify(row["reason"])
        rows.append(row)
    queue = (probe or {}).get("queue") or {}
    started_ok = [r for r in rows if r["start"] == "confirmed"]
    blocked = bool(starts and any(r["start"] == "not_submitted" for r in rows)
                   and started_ok and all(r["outcome"] == "done" for r in started_ok))
    return {"session": session.name, "rows": rows, "probe_found": probe is not None,
            "queue": {"stop_reason": queue.get("stop_reason"), "stops": queue.get("stops"),
                      "items_left": queue.get("items_left"), "counts": queue.get("counts")},
            "queue_blocked": blocked,
            "spool": [r["spool"] for r in runs], "epochs": len(runs), "run_success": [((r["result"] or {}).get("success")) for r in runs],
            "run_failure": [((r["result"] or {}).get("failure_reason")) for r in runs]}


def compare_pair(a: dict, b: dict) -> dict:
    """Mission-level agreement of two same-seed sessions (not the D8 determinism verdict): per scheduled mission, the
    outcome, reason, stage, end time and docking rounds; a value missing on either side makes that mission
    undecidable, not equal."""
    keys = ("start", "outcome", "reason", "stage", "ended_s", "docking_rounds")
    out = []
    for ra, rb in zip(a["rows"], b["rows"]):
        missing = [k for k in keys if (ra.get(k) is None) != (rb.get(k) is None)]
        both_none = [k for k in ("outcome", "ended_s") if ra.get(k) is None and rb.get(k) is None]
        if missing or (both_none and ra.get("start") in ("confirmed", "unknown")):
            out.append({"order": ra["order"], "verdict": "undecidable", "missing": missing + both_none})
            continue
        diff = next((k for k in keys if ra.get(k) != rb.get(k)), None)
        out.append({"order": ra["order"], "verdict": "same" if diff is None else "different", "first_difference": diff,
                    "a": ra.get(diff) if diff else None, "b": rb.get(diff) if diff else None})
    return {"a": a["session"], "b": b["session"], "missions": out}


def arm_of(session: Path) -> str:
    f = session / "survey_arm.txt"
    if f.exists():
        return f.read_text().strip()
    return session.name.split("_")[0]


def summarize(sessions: list[dict], arms: dict) -> dict:
    """Per arm and scene: sessions completing every mission, start states, outcomes, classes, stage failures over stage
    exposure, queue stops, and the loaded-transport tracking failures."""
    out: dict = {}
    for s in sessions:
        key = f"{arms[s['session']]}|{s.get('scene') or '?'}"
        o = out.setdefault(key, {"sessions": 0, "all_done": 0, "start": Counter(), "outcome": Counter(),
                                 "class": Counter(), "stage_fail": Counter(), "stage_exposure": Counter(),
                                 "queue_blocked": 0, "queue_stop": Counter(), "transport_tracking": [],
                                 "forbidden_contacts": 0, "first_block": Counter(), "unexposed_after_block": 0})
        o["sessions"] += 1
        rows = s["rows"]
        o["all_done"] += all(r["outcome"] == "done" for r in rows)
        o["queue_blocked"] += s["queue_blocked"]
        if s["queue"].get("stop_reason"):
            o["queue_stop"][s["queue"]["stop_reason"]] += 1
        first_block = None
        for r in rows:
            o["start"][r["start"]] += 1
            if r["outcome"]:
                o["outcome"][r["outcome"]] += 1
            if r["class"]:
                o["class"][r["class"]] += 1
            for st in r["exposure"]:
                o["stage_exposure"][st] += 1
            if r["stage"] and r["outcome"] not in (None, "done"):
                o["stage_fail"][r["stage"]] += 1
            if r["class"] and first_block is None:
                first_block = r
            o["forbidden_contacts"] += int(r.get("forbidden_contacts") or 0)
            if r["reason"] and re.match(r"^Tracking (failed|timeout) in transport", r["reason"]):
                o["transport_tracking"].append({"session": s["session"], "order": r["order"], "reason": r["reason"],
                                                "evidence": r["evidence"]})
        if first_block is not None:
            o["first_block"][f"{first_block['class']}@{first_block['order']}"] += 1
            o["unexposed_after_block"] += sum(1 for r in rows if r["order"] > first_block["order"]
                                              and r["start"] == "not_submitted")
    for o in out.values():
        for k, v in list(o.items()):
            if isinstance(v, Counter):
                o[k] = dict(v)
    return out


def scene_of(session: Path) -> str | None:
    """The session scene from the run's arguments (scene file stem), else None."""
    for run in sorted(glob.glob(str(session / "live" / "epoch_*" / "run"))):
        meta = _json(f"{run}/scenario.json") or {}
        sf = (meta.get("arguments") or {}).get("scene_file") or meta.get("scene_file")
        if sf:
            return Path(sf).stem.replace("_session", "")
    m = re.search(r"(line_m|random_m)", session.name)
    return m.group(1) if m else None


def survey(out_dir: Path, scheduled: int = 3, pairs: list[tuple[str, str]] | None = None) -> dict:
    sessions, arms = [], {}
    for d in sorted(p for p in out_dir.iterdir() if p.is_dir() and ((p / "probe.json").exists() or (p / "live").exists())):
        s = session_rows(d, scheduled)
        s["scene"] = scene_of(d)
        arms[d.name] = arm_of(d)
        s["arm"] = arms[d.name]
        sessions.append(s)
    by_name = {s["session"]: s for s in sessions}
    pair_out = [compare_pair(by_name[a], by_name[b]) for a, b in (pairs or []) if a in by_name and b in by_name]
    return {"sessions": sessions, "summary": summarize(sessions, arms), "pairs": pair_out,
            "other_reasons": sorted({r["reason"] for s in sessions for r in s["rows"] if r["class"] == "other"})}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--csv", type=Path)
    ap.add_argument("--scheduled", type=int, default=3)
    ap.add_argument("--pairs", nargs="*", default=[], help="A:B session folder names compared mission by mission")
    args = ap.parse_args(argv)
    res = survey(args.out, args.scheduled, [tuple(p.split(":", 1)) for p in args.pairs])
    args.json.write_text(json.dumps(res, indent=1, default=str, allow_nan=False) + "\n")
    if args.csv:
        rows = [{"arm": s["arm"], "scene": s["scene"], **r, "exposure": "|".join(r["exposure"])}
                for s in res["sessions"] for r in s["rows"]]
        keys = list(rows[0]) if rows else []
        with args.csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    print(json.dumps(res["summary"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
