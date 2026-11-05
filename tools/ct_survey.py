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
    settled reject), not_submitted (the queue stopped before sending it). A mission folder found later can still
    confirm an unknown or unlisted one (session_rows)."""
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
    if not (probe or {}).get("missions_run"):
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
                                 "missions_with_recovery": 0, "done_missions": 0})
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
                if ind.get("observation_attempts", 0) > 1:
                    o["recoveries"]["observation_retry"] += 1
                    hit = True
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
        o["delivery_error_m"] = ({"n": len(de), "median": de[len(de) // 2], "max": de[-1]} if de else None)
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--csv", type=Path)
    ap.add_argument("--scheduled", type=int, default=3)
    ap.add_argument("--pairs", nargs="*", default=[], help="A:B session folder names compared mission by mission")
    ap.add_argument("--runs", type=Path, help="the bundle's run list: attempts that left no folder are reported")
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
    print(json.dumps(res["summary"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
