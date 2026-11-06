"""Continuous transport status survey (docs/plans/2026-11-06-ct-status-survey.md): live queue sessions -> one row per
scheduled mission with its start state, outcome, stage, failure class and the evidence paths, the stage exposure, the
queue's own stops, the loaded-transport tracking failures flagged apart, and a mission-level comparison of same-seed
pairs. A description, not a verdict: no thresholds.

    python tools/ct_survey.py OUT --json SURVEY [--csv MISSIONS] [--scheduled 3] [--pairs A_DIR:B_DIR ...]
        [--near-tally NEAR_JSON]

OUT holds one folder per session attempt (``probe.json`` and ``live/epoch_*/run``); a folder's ``survey_arm.txt``
(one line: the arm name) groups it, else the folder name's first ``_`` field."""

from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import itertools
import json
import math
import re
import statistics
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mission_queue import UNSURE_REJECTS  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.mission.failures import PREFIXES  # noqa: E402

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


def probe_entries(probe: dict | None) -> list:
    """The probe's submissions: the queue's, or the reverse scenario's one mission (the console's ③)."""
    runs = list((probe or {}).get("missions_run") or [])
    mission = ((probe or {}).get("reverse") or {}).get("mission")
    return runs + ([mission] if mission else [])


def start_states(probe: dict | None, scheduled: int) -> list[dict]:
    """The scheduled missions' start states from the probe's submissions, one per queue item (a resubmitted item keeps
    its last answer): confirmed (ack), unknown (no answer, or a busy/stale reject the queue holds unsure), rejected (a
    settled reject), not_submitted (the queue stopped before sending it). A mission folder found later can still
    confirm an unknown or unlisted one (session_rows)."""
    items: dict = {}
    order: list = []
    for entry in probe_entries(probe):
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


def _mission_key(name: str):
    m = re.search(r"(\d+)$", name)
    return (int(m.group(1)) if m else 0, name)


def epoch_runs(session: Path) -> list[dict]:
    """Each epoch's run: its mission folders in order (result, delivery error, phase at the end, docking state, safety
    records), its run record (transitions, the session's grid plans) and its spool manifest."""
    out = []
    for run in sorted(glob.glob(str(session / "live" / "epoch_*" / "run")),
                      key=lambda p: int(re.search(r"epoch_(\d+)", p).group(1))):
        missions = []
        for md in sorted(glob.glob(f"{run}/missions/*"), key=lambda p: _mission_key(Path(p).name)):
            rj = _json(f"{md}/result.json")
            rec = _json(f"{md}/records.json")
            st = (rec or {}).get("state") or {}
            dock = st.get("delivery_docking") or {}
            missions.append({"mission_id": Path(md).name, "result": (rj or {}).get("result"), "result_file": rj is not None,
                             "records": rec is not None, "phase": st.get("phase"),
                             "delivery_error": (rj or {}).get("delivery_error_m"),
                             "docking_status": dock.get("status") if rec is not None else None,
                             "docking_round_times": [float(r["time_s"]) for r in dock.get("rounds") or []
                                                     if r.get("time_s") is not None],
                             "docking_retries": [{"time_s": e.get("time_s"), "counted": e.get("counted"),
                                                  "refused": bool((e.get("plan") or {}).get("refused")
                                                                  or (e.get("plan") or {}).get("length_m") is None)}
                                                 for e in st.get("docking_retries") or []],
                             "forbidden_contacts": len(st.get("forbidden_pocket_contacts") or []) if rec is not None else None,
                             "lift_tilt_peak": st.get("lift_tilt_peak"), "folder": md,
                             "indicators": indicators(st) if rec is not None else None})
        top = _json(f"{run}/result.json")
        # the run record's results book (written when the session ends normally): a mission whose folder is missing
        # keeps its place and result; a gap in the mission numbers keeps its place as a mission with no record
        book = {e.get("mission_id"): e for e in (top or {}).get("mission_results") or [] if e.get("mission_id")}
        for m in missions:  # a folder without its result file: the book's settled result and times
            if m["result"] is None and m["mission_id"] in book:
                m["result"], m["result_from_book"] = book[m["mission_id"]], True
        have = {m["mission_id"] for m in missions}
        for mid, e in book.items():
            if mid not in have:
                missions.append({"mission_id": mid, "result": e, "result_file": False, "records": False, "phase": None,
                                 "delivery_error": None, "docking_status": None, "docking_round_times": [],
                                 "docking_retries": [], "forbidden_contacts": None, "lift_tilt_peak": None,
                                 "folder": None, "from_book": True})
        nums = sorted(_mission_key(m["mission_id"])[0] for m in missions if re.fullmatch(r"m\d+", m["mission_id"]))
        for n in range(1, (nums[-1] if nums else 0) + 1):
            if n not in nums:
                missions.append({"mission_id": f"m{n}", "result": None, "result_file": False, "records": False,
                                 "phase": None, "delivery_error": None, "docking_status": None, "docking_round_times": [],
                                 "docking_retries": [], "forbidden_contacts": None, "lift_tilt_peak": None,
                                 "folder": None, "gap": True})
        missions.sort(key=lambda m: _mission_key(m["mission_id"]))
        manifest = _json(f"{run}/spool/manifest.json")
        spool = None if manifest is None else {
            "complete": manifest.get("complete"), "error_count": manifest.get("error_count"),
            "dropped_items": {k: v["dropped_items"] for k, v in manifest.items()
                              if isinstance(v, dict) and v.get("dropped_items")},
            "unsettled_items": {k: v["unsettled_items"] for k, v in manifest.items()
                                if isinstance(v, dict) and v.get("unsettled_items")}}
        out.append({"run": run, "missions": missions, "result": top, "spool": spool,
                    "transitions": (top or {}).get("transitions") or [],
                    "grid_plans": (((top or {}).get("obstacle_layer") or {}).get("grid_plans")
                                   or ((top or {}).get("obstacle_layer") or {}).get("live_plans") or [])})
    return out


RECOVERY_LISTS = ("stall_replans", "cusp_stalls", "observe_replans", "scene_replans", "cusp_replans", "cusp_heading_holds")


def indicators(st: dict) -> dict:
    """What a mission needed on its way, finished or not (the near misses of a survey with few failures): the recovery
    lists' lengths and phases, the counted D7 re-approaches and their triggers, extra observation attempts, the
    delivery docking's status, and the precision a done mission reached."""
    out = {k: len(st.get(k) or []) for k in RECOVERY_LISTS}
    out["recovery_phases"] = sorted({e.get("phase") for k in RECOVERY_LISTS for e in st.get(k) or []
                                     if isinstance(e, dict) and e.get("phase")})
    retries = [e for e in st.get("docking_retries") or [] if e.get("counted")]
    out["d7_reapproaches"] = len(retries)
    out["d7_triggers"] = sorted({str(e.get("replaces") or e.get("trigger") or "").split(":", 1)[0] for e in retries})
    out["observation_attempts"] = len(st.get("observation_attempts") or [])
    out["docking_status"] = (st.get("delivery_docking") or {}).get("status")
    out["near_capture_status"] = (st.get("near_capture") or {}).get("status")
    ins = st.get("insertion_error") or {}
    out["insertion_error_m"] = ins.get("position_m")
    lat = st.get("lateral_clearance_min_m") or {}
    out["lateral_clearance_insert_m"] = min(((lat.get("insert") or {}).values()), default=None)
    out["return_home_error_m"] = (st.get("return_home_error") or {}).get("position_m")
    return out


def mission_phases(transitions: list, started: float | None, ended: float | None,
                   after: bool = False) -> list[str] | None:
    """The phases a mission entered: the transitions' targets inside its interval, in order (a transition's 'from' at
    the boundary is the previous mission's last phase -- the adoption writes previous -> observe -- and is not counted).
    None without the interval or with no transition inside it."""
    if started is None or not transitions:
        return None
    seq = []
    for t in transitions:
        ts = t.get("time_s")
        if ts is None or ts < started - 1e-6 or (ended is not None and ts > ended + 1e-6):
            continue
        if after and ts <= started + 1e-9:  # strictly after: the previous mission's complete shares its end time
            continue
        ph = t.get("to")
        if ph and (not seq or seq[-1] != ph):
            seq.append(ph)
    return seq or None


def substage(stage: str | None, m: dict, ended: float | None) -> str | None:
    """Within the loaded legs, where the delivery docking stood when the mission ended: before its first match, after a
    match, in a D7 re-approach (a counted retry with no docking round after it), or after a refused re-approach plan.
    The retries recorded before the end only (a retry is logged before its plan is judged, and stays after it ends)."""
    if stage not in ("transport", "lower", "withdraw"):
        return None
    end = float("inf") if ended is None else float(ended)
    retries = [e for e in m.get("docking_retries") or [] if e.get("counted") and e.get("time_s") is not None
               and e["time_s"] <= end + 1e-9]
    rounds = [t for t in m.get("docking_round_times") or [] if t <= end + 1e-9]
    if retries:
        last = retries[-1]
        if last["refused"]:
            return "reapproach_refused"
        if not any(t > last["time_s"] + 1e-9 for t in rounds):
            return "reapproach"
    return "after_delivery_match" if rounds else "before_delivery_match"


def exposure_from(phases: list[str] | None, end_phase: str | None, done: bool) -> tuple[list[str], str]:
    """The stages a mission entered and where that came from: the session's transitions (with preparing, which every
    started mission entered), else the phase order up to its end phase (all for a done mission)."""
    if phases is not None and (phases or end_phase == "preparing"):
        return [p for p in PHASE_ORDER if p in phases or p == "preparing"], "transitions"
    if done:
        return list(PHASE_ORDER), "order"
    if end_phase in PHASE_ORDER:
        return list(PHASE_ORDER[: PHASE_ORDER.index(end_phase) + 1]), "order"
    return [], "none"


def _blank(session: str, order: int, s: dict) -> dict:
    return {"session": session, "order": order, "start": s["start"], "start_evidence": "probe" if s.get("item") else None,
            "pallet_id": s.get("pallet_id"), "destination_id": s.get("destination_id"), "mission_id": s.get("mission_id"),
            "reject_reason": s.get("reject_reason"), "outcome": None, "reason": None, "stage": None, "substage": None,
            "class": None, "exposure": [], "exposure_from": "none", "evidence": None, "epoch": None,
            "forbidden_contacts": None, "lift_tilt_peak": None, "docking_rounds": None, "docking_status": None,
            "delivery_error": None, "started_s": None, "ended_s": None, "outcome_from": None,
            "plans": None, "plans_slam_used": None, "plans_static_off": None, "phase_history": None,
            "indicators": None}


def session_rows(session: Path, scheduled: int = 3) -> dict:
    """One session attempt: its missions (start state, outcome, stage, class, evidence), the queue's stop, the probe's
    own error and the run records' session end."""
    probe = _json(session / "probe.json")
    starts = start_states(probe, scheduled)
    runs = epoch_runs(session)
    folders = [(k, m) for k, r in enumerate(runs) for m in r["missions"]]  # chronological: epochs, then mission order
    used: set = set()

    def take(mid=None, pallet=None, dest=None):
        """The next unused folder: by mission id when acked, else the next one in order whose pallet and destination
        do not contradict the item."""
        for i, (k, m) in enumerate(folders):
            if i in used:
                continue
            res = m["result"] or {}
            if mid is not None:
                if m["mission_id"] == mid:
                    used.add(i)
                    return k, m
                continue
            if (pallet and res.get("pallet_id") and res["pallet_id"] != pallet) or \
                    (dest and res.get("destination_id") and res["destination_id"] != dest):
                return None
            used.add(i)
            return k, m
        return None

    rows = []
    for idx, s in enumerate(starts):
        row = _blank(session.name, idx + 1, s)
        found = None
        if s["start"] == "confirmed" and s.get("mission_id"):
            found = take(mid=s["mission_id"])
        elif s["start"] in ("unknown", "not_submitted"):
            found = take(pallet=s.get("pallet_id"), dest=s.get("destination_id"))  # a folder confirms it started
            if found is not None:
                m_ = found[1]
                row["start"] = "confirmed"
                row["start_evidence"] = ("mission_numbering" if m_.get("gap") else
                                         "results_book" if m_.get("from_book") else "mission_folder")
                row["mission_id"] = m_["mission_id"]
        rows.append((row, s, found))
    # folders no probe entry claimed (no probe, or more folders than items): appended as confirmed rows
    for i, (k, m) in enumerate(folders):
        if i not in used:
            row = _blank(session.name, len(rows) + 1, {"start": "confirmed", "mission_id": m["mission_id"]})
            row["start_evidence"] = ("mission_numbering" if m.get("gap") else
                                     "results_book" if m.get("from_book") else "mission_folder")
            rows.append((row, {}, (k, m)))
            used.add(i)
    if not probe_entries(probe):
        # no probe submissions: the rows are the epochs' missions in order, padded to the schedule
        rows = [x for x in rows if x[0]["start"] != "not_submitted"]
        for j, x in enumerate(rows):
            x[0]["order"] = j + 1
        for j in range(len(rows), scheduled):
            rows.append((_blank(session.name, j + 1, {"start": "not_submitted", "mission_id": None}), {}, None))
    last_started = max((j for j, (r, _, _) in enumerate(rows) if r["start"] in ("confirmed", "unknown")), default=None)
    for j, (row, s, found) in enumerate(rows):
        if row["start"] not in ("confirmed", "unknown"):
            continue
        probe_res = s.get("probe_result") or {}
        k, m = found if found is not None else (None, None)
        r = runs[k] if k is not None else (runs[-1] if runs else None)
        res = (m or {}).get("result") or {}
        if res.get("outcome"):
            row.update(outcome=res["outcome"], reason=res.get("reason"), outcome_from="mission_result")
        elif probe_res.get("outcome"):  # settled by the console's result even when the file is missing
            row.update(outcome=probe_res["outcome"], reason=probe_res.get("reason"), outcome_from="probe_result")
        elif j == last_started and r is not None and (r["result"] or {}).get("success") is False:
            # the mission running when the session ended: the run record's reason, only for that mission
            row.update(outcome="session_failed", reason=r["result"].get("failure_reason"), outcome_from="run_result")
        else:
            row.update(outcome="no_record", outcome_from=None)
        started, ended = res.get("started_s"), res.get("ended_s")
        row["pallet_id"] = row["pallet_id"] or res.get("pallet_id")
        row["destination_id"] = row["destination_id"] or res.get("destination_id")
        row.update(started_s=started, ended_s=ended, epoch=(k + 1) if k is not None else (len(runs) or None),
                   evidence=(m or {}).get("folder") or (r or {}).get("run"))
        if m is not None:
            row.update(forbidden_contacts=m["forbidden_contacts"], lift_tilt_peak=m["lift_tilt_peak"],
                       docking_rounds=len(m["docking_round_times"]) if m["records"] else None,
                       docking_status=m["docking_status"], delivery_error=m["delivery_error"],
                       indicators=m.get("indicators"))
        transitions = (r or {}).get("transitions") or []
        phases = mission_phases(transitions, started, ended)
        done = row["outcome"] == "done"
        end_phase = (m or {}).get("phase") or (phases[-1] if phases else None)
        if row["outcome_from"] == "run_result" and end_phase is None:
            # the mission the session ended in, with no interval of its own: only transitions after the previous
            # mission's end are its own; none means it never left the preparation (an accepted mission prepares
            # before its first transition); with no previous end known the stage stays unknown
            prev_end = 0.0 if j == 0 else rows[j - 1][0].get("ended_s")
            if prev_end is not None:
                phases = mission_phases(transitions, float(prev_end), None, after=True)
                if phases is None:
                    phases = []  # observed: no transition of its own, it never left the preparation
                end_phase = phases[-1] if phases else "preparing"
        if not done and row["outcome"] != "no_record":
            row["stage"] = stage_of(row["reason"], end_phase)
            row["substage"] = substage(row["stage"], m or {}, ended)
            row["class"] = classify(row["reason"])
        elif row["outcome"] == "no_record":
            row["class"] = "unknown"
        row["phase_history"] = phases
        row["exposure"], row["exposure_from"] = exposure_from(phases, row["stage"] or end_phase, done)
        if started is not None and r is not None and r["grid_plans"]:  # a live run may keep no plan list: not recorded
            plans = [g for g in r["grid_plans"] if g.get("stamp_s") is not None
                     and started - 1e-6 <= g["stamp_s"] <= (ended if ended is not None else float("inf")) + 1e-6]
            row.update(plans=len(plans), plans_slam_used=sum(bool(g.get("slam_used")) for g in plans),
                       plans_static_off=sum((g.get("slam_map_note") or {}).get("slam_static_layer") == "off_deterministic"
                                            for g in plans))
    rows = [r for r, _, _ in rows]
    queue = (probe or {}).get("queue") or {}
    started_ok = [r for r in rows if r["start"] == "confirmed"]
    # the queue's own block: every started mission done, and a later one not started (not submitted, a settled reject,
    # or an answer that never came)
    block_row = next((r for r in rows if r["start"] in ("not_submitted", "rejected", "unknown")
                      and r["outcome"] in (None, "no_record")), None)
    blocked = bool(block_row is not None and all(r["outcome"] == "done" for r in started_ok)
                   and probe is not None and not probe.get("error"))
    if blocked:
        block_row["class"] = "queue"
        block_row["reason"] = block_row.get("reject_reason") or queue.get("stop_reason")
    infra = None
    if probe is None and not runs:
        infra = "no_records"
    elif (probe or {}).get("error"):
        infra = f"probe_error: {probe['error']}"
    elif probe is None:
        infra = "no_probe_report"
    if infra is not None and rows and not any(r["class"] for r in rows):
        first_open = next((r for r in rows if r["outcome"] != "done"), None)
        if first_open is not None:
            first_open["class"], first_open["reason"] = "infra", infra
    return {"session": session.name, "rows": rows, "probe_found": probe is not None, "infra": infra,
            "queue": {"stop_reason": queue.get("stop_reason"), "stops": queue.get("stops"),
                      "items_left": queue.get("items_left"), "counts": queue.get("counts")},
            "queue_blocked": blocked, "spool": [r["spool"] for r in runs], "epochs": len(runs),
            "run_success": [((r["result"] or {}).get("success")) for r in runs],
            "run_failure": [((r["result"] or {}).get("failure_reason")) for r in runs]}


PAIR_KEYS = ("start", "outcome", "reason", "stage", "substage", "phase_history", "ended_s", "docking_rounds",
             "delivery_error")


def compare_pair(a: dict, b: dict) -> dict:
    """Mission-level agreement of two same-seed sessions (not the D8 determinism verdict): per scheduled mission, the
    start, outcome, reason, stage and substage, the phase history (in order, repeats kept), the end time, the docking
    rounds and the delivery error. A value required for a started or possibly started mission (outcome, end time,
    docking rounds; the delivery error of a done one) missing on either side makes that mission undecidable, never
    equal."""
    out = []
    for ra, rb in zip(a["rows"], b["rows"]):
        required = []
        if ra.get("start") in ("confirmed", "unknown") or rb.get("start") in ("confirmed", "unknown"):
            required = ["outcome", "ended_s", "docking_rounds"]
            if ra.get("outcome") == "done" or rb.get("outcome") == "done":
                required.append("delivery_error")
            # a mission past the preparation went through transitions: its history is evidence, not optional
            # (an observed preparation-only mission has [] , not None)
            if any(r.get("outcome") == "done" or r.get("stage") not in (None, "preparing") for r in (ra, rb)):
                required.append("phase_history")
        missing = [k for k in required if ra.get(k) is None or rb.get(k) is None]
        missing += [k for k in PAIR_KEYS if k not in required and (ra.get(k) is None) != (rb.get(k) is None)]
        if missing:
            out.append({"order": ra["order"], "verdict": "undecidable", "missing": missing})
            continue
        diff = next((k for k in PAIR_KEYS if ra.get(k) != rb.get(k)), None)
        out.append({"order": ra["order"], "verdict": "same" if diff is None else "different", "first_difference": diff,
                    "a": ra.get(diff) if diff else None, "b": rb.get(diff) if diff else None})
    return {"a": a["session"], "b": b["session"], "missions": out}


def arm_of(session: Path) -> str:
    f = session / "survey_arm.txt"
    if f.exists():
        return f.read_text().strip()
    return session.name.split(".attempt")[0].split("_")[0]


def summarize(sessions: list[dict], arms: dict) -> dict:
    """Per arm and scene: sessions completing every mission, start states, outcomes, classes, stage failures over stage
    exposure, queue stops, and the loaded-transport tracking failures."""
    out: dict = {}
    for s in sessions:
        key = f"{arms[s['session']]}|{s.get('scene') or '?'}"
        o = out.setdefault(key, {"sessions": 0, "all_done": 0, "start": Counter(), "outcome": Counter(),
                                 "class": Counter(), "stage_fail": Counter(), "stage_exposure": Counter(),
                                 "queue_blocked": 0, "queue_stop": Counter(), "transport_tracking": [],
                                 "forbidden_contacts": 0, "first_block": Counter(), "unexposed_after_block": 0,
                                 "infra": 0, "recoveries": Counter(), "recovery_phases": Counter(),
                                 "d7_triggers": Counter(), "docking_status": Counter(), "delivery_error_m": [],
                                 "missions_with_recovery": 0, "done_missions": 0, "observation_attempts": Counter()})
        o["sessions"] += 1
        rows = s["rows"]
        o["all_done"] += all(r["outcome"] == "done" for r in rows)
        o["queue_blocked"] += s["queue_blocked"]
        o["infra"] += s.get("infra") is not None
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
            ind = r.get("indicators") or {}
            if ind:
                hit = False
                for k in RECOVERY_LISTS + ("d7_reapproaches",):
                    if ind.get(k):
                        o["recoveries"][k] += 1
                        hit = True
                o["observation_attempts"][ind.get("observation_attempts")] += 1  # 2 is the normal flow
                o["missions_with_recovery"] += hit
                for ph in ind.get("recovery_phases") or []:
                    o["recovery_phases"][ph] += 1
                for t in ind.get("d7_triggers") or []:
                    o["d7_triggers"][t] += 1
                if ind.get("docking_status"):
                    o["docking_status"][ind["docking_status"]] += 1
            if r["outcome"] == "done":
                o["done_missions"] += 1
                if r.get("delivery_error") is not None:
                    o["delivery_error_m"].append(r["delivery_error"])
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
        de = sorted(o.pop("delivery_error_m"))
        o["delivery_error_m"] = ({"n": len(de), "median": float(statistics.median(de)), "max": de[-1]} if de else None)
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


def survey(out_dir: Path, scheduled: int = 3, pairs: list[tuple[str, str]] | None = None,
           runs_list: Path | None = None) -> dict:
    sessions, arms = [], {}
    names = set()
    if runs_list is not None:  # every planned attempt is listed, even one that left no folder at all
        names = {json.loads(line)["name"] for line in Path(runs_list).read_text().splitlines() if line.strip()}
    found = [p for p in out_dir.iterdir() if p.is_dir() and ((p / "probe.json").exists() or (p / "live").exists()
                                                             or p.name.split(".attempt")[0] in names)]
    for name in sorted(names - {p.name for p in found}):
        found.append(out_dir / name)  # missing: session_rows reports no_records
    for d in sorted(found, key=lambda p: p.name):
        s = session_rows(d, scheduled)
        s["scene"] = scene_of(d)
        arms[d.name] = arm_of(d)
        s["arm"] = arms[d.name]
        sessions.append(s)
    by_name = {s["session"]: s for s in sessions}
    pair_out = [compare_pair(by_name[a], by_name[b]) for a, b in (pairs or []) if a in by_name and b in by_name]
    return {"sessions": sessions, "summary": summarize(sessions, arms), "pairs": pair_out,
            "other_reasons": sorted({r["reason"] for s in sessions for r in s["rows"] if r["class"] == "other"})}


# --- near capture tally (docs/plans/2026-11-07-near-capture-frequency.md sections 3, 4; the console's ③ calls it too) ---

NEAR_END_PREFIX = "near_capture_"
# A run's own values, kept out of the condition; the scene's arguments are kept out of the operating configuration
RUN_ARGUMENTS = ("output", "seed", "slam_noise_seed", "live_port", "live_port_file", "slam_feedback", "slam_map_dir")
SCENE_ARGUMENTS = ("scene_file", "slam_map_bundle", "expected_map_id", "slam_choice")
# The review triggers of #338: priority values, not an accepted failure rate
TRIGGER_MIN_MISSIONS = 50
TRIGGER_RATE = 0.02
TRIGGER_SESSIONS = 2
MAX_ASSIGNED = 12  # unknown missions a trigger enumerates (three to four ways each); above it: undecidable
WILSON_Z = 1.959964
SHAPES = ("epal6", "t11")


def _digest(data) -> str:
    return hashlib.sha256(json.dumps(data, sort_keys=True, default=str).encode()).hexdigest()


def _scene_waypoints(arguments: dict) -> bool:
    """The recorded observation points are the scene's own: a session's first mission writes its pallet's, and each
    adoption overwrites them, so the record depends on how far the queue went. Others were set by hand."""
    points = arguments.get("observation_waypoints")
    scene = _json(arguments["scene_file"]) if points is not None and arguments.get("scene_file") else None
    return any(p.get("observation_waypoints") == points for p in [scene or {}, *(scene or {}).get("pallets", [])])


def config_hashes(arguments: dict | None) -> tuple[str | None, str | None]:
    """(condition_sha256, operating_config_sha256) of a run's recorded arguments; (None, None) without them."""
    if not isinstance(arguments, dict):
        return None, None
    drop = set(RUN_ARGUMENTS) | ({"observation_waypoints"} if _scene_waypoints(arguments) else set())
    kept = {k: v for k, v in arguments.items() if k not in drop}
    return _digest(kept), _digest({k: v for k, v in kept.items() if k not in SCENE_ARGUMENTS})


def _capture_time(attempt: dict) -> float | None:
    control = attempt.get("capture_control")
    t = control["time_s"] if control is not None else attempt.get("t_before_capture_s")
    return None if t is None else float(t)


def _run_samples(run: Path) -> list:
    out = []
    for name in sorted(glob.glob(str(run / "spool" / "state.samples" / "*.jsonl"))):
        with open(name) as handle:
            out += [json.loads(line) for line in handle if line.strip()]
    return out


def _sample_at(samples: list, t: float | None, pallet_id=None) -> dict | None:
    """The last sample with a pallet at or before ``t``, of that pallet when both name one."""
    best = None
    for s in samples if t is not None else []:
        if s.get("pallet_position_m") is None or float(s["time_s"]) > t:
            continue
        if pallet_id and s.get("pallet_id") and s["pallet_id"] != pallet_id:
            continue
        best = s
    return best


def entry_face(attempt: dict, samples: list, pallet_id=None) -> str:
    """The pallet's x face that looks at the truck when it captured; unknown without its sample or pose."""
    sample = _sample_at(samples, _capture_time(attempt), pallet_id)
    pose = attempt.get("accepted_pose")
    if sample is None or not pose or sample.get("pallet_yaw_rad") is None:
        return "unknown"
    px, py = sample["pallet_position_m"][:2]
    yaw = float(sample["pallet_yaw_rad"])
    toward = (float(pose[0][0]) - px) * math.cos(yaw) + (float(pose[0][1]) - py) * math.sin(yaw)
    return "+x" if toward >= 0 else "-x"


def _is_near(index: int, attempt: dict, state: dict) -> bool:
    """A near capture: its expected site, or an attempt after the far one (near_refit_compare.attempt_role)."""
    if attempt.get("expected") is not None:
        return True
    far = (state.get("near_capture") or {}).get("far_attempt")
    return isinstance(far, int) and index + 1 > far


def _reason_key(record: dict | None) -> str:
    record = record or {}
    return f"{record.get('status') or 'no_observation'}:{record.get('reason')}"


def _end_key(reason: str) -> str:
    """status:reason for a failed detection, else the reason's name (a refusal's numbers stay out of the key)."""
    head, _, rest = reason.partition(":")
    if head == "near_capture_failed":
        return ":".join(rest.removeprefix("perception_").split(":")[:2])
    return head


def cause_of(reason: str | None) -> str | None:
    """The longest known prefix (forklift_core.mission.failures), else the reason's first field."""
    if not reason:
        return None
    known = [p for p in PREFIXES if reason.startswith(p)]
    return max(known, key=len) if known else reason.split(":")[0][:60]


def _attempt_id(attempt: dict) -> tuple:
    return attempt.get("depth_sha256"), attempt.get("t_before_capture_s"), attempt.get("loop_step")


def _blank_near(row: dict) -> dict:
    return {"order": row["order"], "pallet_id": row["pallet_id"], "near": [], "state": None, "moved": False}


def _session_near(session: Path, rows: list) -> tuple[list, list, dict | None]:
    """Each started row's near attempts and mission state, the attempts nobody can place, and the run's arguments. A
    mission's records hold its own; the run's result keeps those of the mission the run ended in, whose records may
    never have been written. Those go to the last started row unless the samples name another pallet."""
    entries = {r["order"]: _blank_near(r) for r in rows}
    unplaced, arguments, seen = [], None, set()
    for run_dir in sorted(glob.glob(str(session / "live" / "epoch_*" / "run"))):
        run = Path(run_dir)
        top = _json(run / "result.json") or {}
        arguments = top.get("arguments") or arguments
        samples = _run_samples(run)
        for i, r in enumerate(rows):  # an unacknowledged mission's own transition: after the previous mission's end
            before = 0.0 if i == 0 else rows[i - 1].get("ended_s")
            if r["start"] == "unknown" and before is not None and any(
                    t.get("to") in PHASE_ORDER[1:-1] and float(t.get("time_s") or 0.0) > before
                    for t in top.get("transitions") or []):
                entries[r["order"]]["moved"] = True
        for r in rows:
            mid = r["mission_id"]
            records = _json(run / "missions" / mid / "records.json") if mid else None
            if records is None:
                continue
            state = records.get("state") or {}
            attempts = state.get("observation_attempts") or []
            seen |= {_attempt_id(a) for a in attempts}
            e = entries[r["order"]]
            e["state"], e["samples"] = state, samples
            e["near"] = [a for i, a in enumerate(attempts) if _is_near(i, a, state)]
        extra = [(i, a) for i, a in enumerate(top.get("observation_attempts") or []) if _attempt_id(a) not in seen]
        near = [a for i, a in extra if _is_near(i, a, top)]
        started = [r for r in rows if r["start"] in ("confirmed", "unknown")]
        if not extra or not started:
            unplaced += near
            continue
        last = next((r for r in reversed(started) if entries[r["order"]]["state"] is None), None)
        named = _sample_at(samples, _capture_time(near[0]) if near else None)
        if last is None or (named and named.get("pallet_id") and last["pallet_id"]
                            and named["pallet_id"] != last["pallet_id"]):
            unplaced += near
            continue
        e = entries[last["order"]]
        e["state"], e["samples"] = top, samples
        e["near"] = near
    return [entries[r["order"]] for r in rows], unplaced, arguments


def _near_mission(common: dict, row: dict, e: dict) -> dict:
    near, state = e["near"], e["state"]
    reason = row["reason"] or ""
    done = row["outcome"] == "done"
    # a run's end reason alone does not say an unacknowledged mission started: its own result or records do
    seen = (row["start"] == "confirmed" or row.get("outcome_from") in ("mission_result", "probe_result")
            or bool(near) or state is not None or e.get("moved", False))
    started = "confirmed" if seen else "unknown"
    missing = not seen or row["outcome"] in (None, "no_record") or (row["outcome"] == "session_failed" and not reason)
    ended = seen and not done and reason.startswith(NEAR_END_PREFIX)
    touched = bool(near) or ended or bool(state and (state.get("near_capture") or state.get("near_capture_history")))
    first, final, seconds, rescues = Counter(), Counter(), 0, 0
    verdicts = []
    for a in near:
        obs = a.get("pocket_observation") or {}
        waiver = a.get("lower_waiver") or {}
        head = waiver.get("first") or obs
        if head.get("status") != "valid":
            first[_reason_key(head)] += 1
        if obs.get("status") != "valid":
            final[_reason_key(obs)] += 1
        seconds += "second" in waiver
        rescues += bool(waiver.get("applied"))
        verdicts.append(obs.get("status") == "valid")
    again = any(not v and any(verdicts[i + 1:]) for i, v in enumerate(verdicts))
    return {**common, "start": started, "mission_seq": row["order"], "pallet_id": row["pallet_id"],
            "entry_face": entry_face(near[0], e.get("samples") or [], row["pallet_id"]) if near else "unknown",
            "reached": touched,
            "reached_unknown": (started == "confirmed" and not touched and state is None
                                and row["stage"] != "preparing"),  # a near leg's phase is observe too
            "result_missing": missing, "ended_near": ended, "ended_key": _end_key(reason) if ended else None,
            "ended_without_capture": ended and not near, "cause": None if done or missing else cause_of(reason),
            "class": None if done or missing else classify(reason), "capture_attempts": len(near),
            "first_invalid": first, "final_invalid": final, "waiver_second_calls": seconds, "waiver_rescues": rescues,
            "recovered_same": not ended and rescues > 0, "recovered_new": not ended and again}


CELL_KEYS = ("condition_sha256", "operating_config_sha256", "shape", "entry_face", "arm", "pose_check", "scene", "seed",
             "mission_seq", "reloc_inject")
SESSION_CELL_KEYS = tuple(k for k in CELL_KEYS if k not in ("mission_seq", "entry_face"))


def _tally_cell(key: tuple, members: list) -> dict:
    cell = {"schema": "near_capture_tally/1", **dict(zip(CELL_KEYS, key)),
            "run_config_sha256s": sorted({m["run_config_sha256"] for m in members if m["run_config_sha256"]})}
    real = [m for m in members if not m.get("unplaced")]
    ended = [m for m in real if m["ended_near"]]
    first, final = Counter(), Counter()
    for m in members:
        first.update(m["first_invalid"])
        final.update(m["final_invalid"])
    cell.update(missions_started=sum(m["start"] == "confirmed" for m in real),
                start_unknown=sum(m["start"] == "unknown" for m in real),
                missions_reached_near=sum(m["reached"] for m in real), missions_ended_near=len(ended),
                ended_without_capture=sum(m["ended_without_capture"] for m in real),
                ended_by_reason=dict(Counter(m["ended_key"] for m in ended)),
                ended_session_ids=sorted({m["session"] for m in ended}),
                capture_attempts=sum(m["capture_attempts"] for m in members),
                first_call_invalid_by_reason=dict(first), final_invalid_by_reason=dict(final),
                waiver_second_calls=sum(m["waiver_second_calls"] for m in members),
                waiver_rescues=sum(m["waiver_rescues"] for m in members),
                missions_recovered_same_capture=sum(m["recovered_same"] for m in real),
                missions_recovered_new_capture=sum(m["recovered_new"] for m in real),
                reached_unknown=sum(m["reached_unknown"] for m in real),
                result_missing=sum(m["result_missing"] for m in real))
    return cell


def wilson(k: int, n: int) -> list:
    """The two-sided 95 % Wilson interval (a reference: a session's missions are not independent)."""
    p, z2 = k / n, WILSON_Z * WILSON_Z
    d = 1 + z2 / n
    c, h = (p + z2 / (2 * n)) / d, WILSON_Z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / d
    return [max(0.0, c - h), min(1.0, c + h)]


def _bounds(low_n: int, low_d: int, high_n: int, high_d: int) -> dict | None:
    if low_d <= 0 or high_d <= 0:
        return None
    return {"low": low_n / low_d, "high": high_n / high_d}


def near_rates(*, k: int, N: int, S: int, R: int, U: int, m_R: int, m_U: int) -> dict:
    """The rates over N confirmed started, S start unknown, R reached, U reached unknown; m_R and m_U of them with no
    result (plan section 3): a range, since a result-missing mission may have ended at near capture."""
    near_end = _bounds(k, N + S, k + m_R + m_U + S, N + S)
    if near_end is not None and N:
        near_end["wilson_observed"] = wilson(k, N)
    return {"near_end": near_end, "conditional": _bounds(k, R + U + S, k + m_R + m_U + S, R + m_U + S)}


def review_triggers(missions: list) -> dict:
    """(a) and (b) of #338 over every assignment of the unknown missions: met, not_met, denominator_short (a: no
    assignment reaches the count) or undecidable (the assignments differ). A mission with no result ended at near
    capture, at the top other cause or well; one with an unknown start also may never have started."""
    unknown = [m for m in missions if m["result_missing"]]
    fixed = [m for m in missions if not m["result_missing"]]
    base = Counter(m["cause"] for m in fixed if m["cause"])
    near_cause = NEAR_END_PREFIX
    if len(unknown) > MAX_ASSIGNED:  # only what holds in every assignment
        k = sum(m["ended_near"] for m in fixed)
        sessions = len({m["session"] for m in fixed if m["ended_near"]}) >= TRIGGER_SESSIONS
        least = sum(m["start"] == "confirmed" for m in missions)
        top = max((n for c, n in base.items() if c != near_cause), default=0)
        a = ("denominator_short" if len(missions) < TRIGGER_MIN_MISSIONS else
             "met" if least >= TRIGGER_MIN_MISSIONS and k >= TRIGGER_RATE * len(missions) and sessions else "undecidable")
        return {"a": a, "b": "met" if sessions and k >= top + len(unknown) else "undecidable", "assigned": None}
    other_top = max((c for c in base if c != near_cause), key=lambda c: base[c], default="(unknown)")
    choices = [("near", "other", "fine") + (("absent",) if m["start"] == "unknown" else ()) for m in unknown]
    verdicts_a, verdicts_b = set(), set()
    for picks in itertools.product(*choices):
        started = len(fixed) + sum(p != "absent" for p in picks)
        sessions = {m["session"] for m in fixed if m["ended_near"]}
        sessions |= {m["session"] for m, p in zip(unknown, picks) if p == "near"}
        k = sum(m["ended_near"] for m in fixed) + picks.count("near")
        causes = Counter(base)
        causes[near_cause] = k
        causes[other_top] += picks.count("other")
        if started < TRIGGER_MIN_MISSIONS:
            verdicts_a.add("denominator_short")
        else:
            verdicts_a.add("met" if k >= TRIGGER_RATE * started and len(sessions) >= TRIGGER_SESSIONS else "not_met")
        verdicts_b.add("met" if len(sessions) >= TRIGGER_SESSIONS and k >= max(causes.values()) else "not_met")
    one = lambda found: found.pop() if len(found) == 1 else "undecidable"  # noqa: E731
    return {"a": one(verdicts_a), "b": one(verdicts_b), "assigned": len(unknown)}


def _population(key: tuple, missions: list, assumed: list) -> dict:
    real = [m for m in missions if not m.get("unplaced")]
    started = [m for m in real if m["start"] == "confirmed"]
    unknown = [m for m in real if m["start"] == "unknown"]
    near_end = [m for m in real if m["ended_near"]]
    reached = [m for m in real if m["reached"]]
    wide = [m for m in real if m["reached_unknown"]]
    rates = near_rates(k=len(near_end), N=len(started), S=len(unknown), R=len(reached), U=len(wide),
                       m_R=sum(m["result_missing"] and m["start"] == "confirmed" for m in reached),
                       m_U=sum(m["result_missing"] and m["start"] == "confirmed" for m in wide))
    sessions = {m["session"] for m in started}
    stopped = {m["session"] for m in near_end}
    start_open = {m["session"] for m in unknown} - sessions
    by_scene: dict = {}
    for m in real:
        s = by_scene.setdefault(m["scene"], {"started": 0, "ended_near": 0, "reached": 0})
        s["started"] += m["start"] == "confirmed"
        s["ended_near"] += m["ended_near"]
        s["reached"] += m["reached"]
    seq = Counter((m["mission_seq"], m["pallet_id"]) for m in reached)
    return {"schema": "near_capture_population/1", "operating_config_sha256": key[0], "shape": key[1], "arm": key[2],
            "pose_check": key[3], "reloc_inject": key[4], "config_assumed_sessions": sorted(assumed),
            "rates": {"N": len(started), "S": len(unknown), "k": len(near_end), "R": len(reached), "U": len(wide),
                      "result_missing": sum(m["result_missing"] and m["start"] == "confirmed" for m in real),
                      "ended_without_capture": sum(m["ended_without_capture"] for m in real), **rates},
            "sessions": {"stopped_near": len(stopped), "started": len(sessions), "start_unknown": len(start_open),
                         "wilson_observed": wilson(len(stopped), len(sessions)) if sessions else None},
            "ended_by_reason": dict(Counter(m["ended_key"] for m in near_end)),
            "causes": dict(Counter(m["cause"] for m in real if m["cause"])),
            "classes": dict(Counter(m["class"] for m in real if m["class"])),
            "by_scene": by_scene, "reached_by_order_pallet": {f"{o}:{p}": n for (o, p), n in sorted(seq.items(), key=str)},
            "triggers": review_triggers([m for m in real if m["start"] == "unknown" or m["start"] == "confirmed"])}


def _attempt_order(name: str) -> tuple:
    """(base, turn): a retry moves the earlier attempt to ``name.attemptN``, and the bare name is the last."""
    base, _, number = name.partition(".attempt")
    return base, int(number) if number.isdigit() else math.inf


def _took_submission(out_dir: Path, s: dict) -> bool:
    """An acknowledged or refused submission, or a mission's own records or transition (what makes it started)."""
    if any(r["start"] in ("confirmed", "rejected") or r.get("outcome_from") in ("mission_result", "probe_result")
           for r in s["rows"]):
        return True
    entries, unplaced, _ = _session_near(Path(out_dir) / s["session"], s["rows"])
    return bool(unplaced) or any(e["near"] or e["state"] is not None or e["moved"] for e in entries)


def valid_attempts(out_dir: Path, sessions: list) -> tuple[list, list]:
    """Each run's first attempt that took a submission, else its last one (start unknown, plan section 2), and the
    attempts left out. Evidence, not the folder's name, keeps an attempt."""
    runs: dict = {}
    for s in sessions:
        runs.setdefault(_attempt_order(s["session"])[0], []).append(s)
    kept, left = [], []
    for attempts in runs.values():
        attempts.sort(key=lambda s: _attempt_order(s["session"])[1])
        took = [s for s in attempts if _took_submission(out_dir, s)]
        chosen = took[0] if took else attempts[-1]
        kept.append(chosen)
        left += [s["session"] for s in attempts if s is not chosen]
    return kept, sorted(left)


def near_capture_tally(out_dir: Path, sessions: list) -> dict:
    """Plan section 3's cells, session cells and, per operating configuration, the three rates and the triggers.
    Attempts that took no submission and were retried are left out and named (valid_attempts). A session whose run
    left no arguments joins its arm's only configuration and is named; with several it stays apart."""
    chosen, excluded = valid_attempts(out_dir, sessions)
    missions, open_sessions = [], []
    for s in chosen:
        path = Path(out_dir) / s["session"]
        started = [r for r in s["rows"] if r["start"] in ("confirmed", "unknown")]
        entries, unplaced, arguments = _session_near(path, s["rows"])
        condition, operating = config_hashes(arguments)
        a = arguments or {}
        shape = str((a.get("pallet_prior_loaded") or {}).get("catalogue_version") or "unknown")
        common = {"session": s["session"], "arm": s.get("arm"), "scene": s.get("scene"), "seed": a.get("seed"),
                  "shape": shape, "pose_check": a.get("pose_check") or "off", "reloc_inject": a.get("reloc_inject") or "none",
                  "condition_sha256": condition, "operating_config_sha256": operating,
                  "run_config_sha256": _digest(arguments) if arguments else None}
        ms = [_near_mission(common, r, e) for r, e in zip(s["rows"], entries) if r in started]
        stop = next((m["mission_seq"] for m in ms if m["ended_near"]), None)
        session_row = {**common, "started": any(m["start"] == "confirmed" for m in ms), "start_open": bool(ms),
                       "stopped_near": stop is not None,
                       "unsubmitted_after_stop": 0 if stop is None else sum(
                           r["order"] > stop and r["start"] == "not_submitted" for r in s["rows"])}
        if unplaced:
            blank = {"order": "unknown", "pallet_id": None, "reason": None, "outcome": "done", "start": "confirmed", "stage": None}
            extra = _near_mission(common, blank, {"near": unplaced, "state": None, "samples": []})
            ms.append({**extra, "unplaced": True, "reached": False, "ended_near": False})
        open_sessions.append((session_row, ms))
    known = {(r["arm"], r["reloc_inject"]): set() for r, _ in open_sessions}
    for r, _ in open_sessions:
        if r["operating_config_sha256"]:
            known[(r["arm"], r["reloc_inject"])].add((r["operating_config_sha256"], r["shape"], r["pose_check"]))
    assumed: dict = {}
    for r, ms in open_sessions:
        fits = known[(r["arm"], r["reloc_inject"])]
        if r["operating_config_sha256"] is None and len(fits) == 1:
            op, shape, pose = next(iter(fits))
            for x in [r, *ms]:
                x.update(operating_config_sha256=op, shape=shape, pose_check=pose)
            assumed.setdefault((op, shape, r["arm"], pose, r["reloc_inject"]), []).append(r["session"])
        missions += ms
    grouped: dict = {}
    for m in missions:
        grouped.setdefault(tuple(str(m[k]) for k in CELL_KEYS), []).append(m)
    cells = [_tally_cell(tuple(members[0][k] for k in CELL_KEYS), members) for _, members in sorted(grouped.items())]
    session_groups: dict = {}
    for r, _ in open_sessions:
        session_groups.setdefault(tuple(str(r[k]) for k in SESSION_CELL_KEYS), []).append(r)
    session_cells = [{"schema": "near_capture_session_tally/1", **{k: rs[0][k] for k in SESSION_CELL_KEYS},
                      "started_session_ids": sorted(r["session"] for r in rs if r["started"]),
                      "start_unknown_session_ids": sorted(r["session"] for r in rs if r["start_open"] and not r["started"]),
                      "stopped_near_session_ids": sorted(r["session"] for r in rs if r["stopped_near"]),
                      "unsubmitted_after_stop": sum(r["unsubmitted_after_stop"] for r in rs)}
                     for _, rs in sorted(session_groups.items())]
    pools: dict = {}
    for r, ms in open_sessions:
        key = (r["operating_config_sha256"], r["shape"], r["arm"], r["pose_check"], r["reloc_inject"])
        pools.setdefault(key, []).extend(ms)
    populations = [_population(k, v, assumed.get(k, [])) for k, v in sorted(pools.items(), key=str)]
    seen_shapes = {p["shape"].split("_")[0] for p in populations if p["rates"]["N"]}
    return {"cells": cells, "session_cells": session_cells, "populations": populations, "excluded_attempts": excluded,
            "unverified_shapes": [s for s in SHAPES if s not in seen_shapes]}

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--csv", type=Path)
    ap.add_argument("--scheduled", type=int, default=3)
    ap.add_argument("--pairs", nargs="*", default=[], help="A:B session folder names compared mission by mission")
    ap.add_argument("--runs", type=Path, help="the bundle's run list: attempts that left no folder are reported")
    ap.add_argument("--near-tally", type=Path, help="write the near capture tally (near_capture_tally/1) here")
    args = ap.parse_args(argv)
    res = survey(args.out, args.scheduled, [tuple(p.split(":", 1)) for p in args.pairs], args.runs)
    args.json.write_text(json.dumps(res, indent=1, default=str, allow_nan=False) + "\n")
    if args.csv:
        rows = [{"arm": s["arm"], "scene": s["scene"], **r, "exposure": "|".join(r["exposure"])}
                for s in res["sessions"] for r in s["rows"]]
        keys = list(rows[0]) if rows else []
        with args.csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    if args.near_tally:
        args.near_tally.write_text(json.dumps(near_capture_tally(args.out, res["sessions"]), indent=1, default=str,
                                              allow_nan=False) + "\n")
    print(json.dumps(res["summary"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
