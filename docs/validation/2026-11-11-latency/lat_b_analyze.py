"""The latency test B's judgement (docs/plans/2026-11-11-latency-tolerance.md 4.1, 4.2, 4.6, 4.7; fixed before the
results).

    python lat_b_analyze.py OUT_ROOT --snap SNAP [--json OUT.json]

Fifteen live sessions (lat_lists.b_jobs): line_m far K f 1.0 confirm, the planning answers held d ms after arrival.
Per session:
- environment (4.7 B): the CPU sampler's overlap or a missing sampler record, the bundle's attempt (one, not retried),
  a trace of running, the registered conditions (the 1e (c) session's); the GPU monitor's findings and other jobs are
  notes (B is not a timing measurement). A probe's non-zero exit is the mission's, not the environment's.
- the arm: the runner's --latency-plan-ms and the injection record's delay equal the list's d; the launcher's
  RETURN_START / NEAR_END_STAND and the map choice the same in every session.
- safety (4.1, read whenever the session ran): forbidden pocket contacts (the session's and each mission's records),
  unpermitted entries, scene_damaged, and the low objects (tools/low_clearance_report.py on the session's spooled
  samples and control rows: a physical overlap or a physical stop-arc violation). A confirmed violation stands whatever
  else could not be read.
- the mission m1 by its result (not the session's success), the stop after a failure (4.2: the truth speed under
  0.05 m/s, no AUTO or teleop mode after the mission's end, the control rows unbroken, for 0.5 s up to the last row),
  the exposure (>= 3 answers of this mission delivered, prepare and dock among them; d 0 from the mission's own plans).
"B passes" when the 12 judged sessions (d 0/25/50/100, the effective run of each) are valid, safe, done and exposed.
Every safety event (any session, d 500 and first runs included) is a P1 defect; a d 500 failure is the lowest. A rerun
is allowed only for an environment-invalid first run without a mission failure; the first run stays on record.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
JUDGED = (0, 25, 50, 100)
STAND_S, STILL_MPS, CONTROL_GAP_S = 0.5, 0.05, 3 / 120 + 1e-6
NOTE_PREFIXES = ("no monitor record", "monitor query not ok", "a GPU process not known", "another job on the node")
DRIVING_MODES = ("AUTO", "TELEOP")


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


REGISTERED_LAUNCHER = {"RETURN_START": "boxed", "NEAR_END_STAND": "on", "map": True}  # plan 3.2


def split_invalid(invalid: list, ran: bool = True) -> dict:
    """A probe's exit is the mission's only for a session that ran; one that never started is the environment's."""
    env, notes, mission = [], [], []
    for x in invalid:
        if x.startswith(NOTE_PREFIXES):
            notes.append(x)
        elif x.startswith("probe exit") and ran:
            mission.append(x)
        else:
            env.append(x)
    return {"environment": env, "notes": notes, "mission": mission}


def exposure_of(session_result: dict, mission_state: dict, delay_ms: int) -> dict:
    """4.5 B: this mission's answers delivered -- for d > 0 the injection's delivered records joined to the mission's
    plans by job id (an answer not of this mission is not counted); for d 0 the mission's own taken plans."""
    plans = (mission_state or {}).get("async_plans") or []
    if delay_ms:
        recs = ((session_result or {}).get("latency_injection") or {}).get("plan", {}).get("records") or []
        site_of = {p.get("job"): p.get("site") for p in plans}
        mine = [r for r in recs if r.get("delivered") is not None and r.get("job") in site_of]
        sites = [site_of[r["job"]] for r in mine]
        held = len([r for r in recs if r.get("delivered") is None and r.get("job") in site_of])
        injected = sorted(round(float(r["injected_s"]), 4) for r in mine)
    else:
        mine = [p for p in plans if p.get("finished_s") is not None and not p.get("dropped")]
        sites, held, injected = [p.get("site") for p in mine], 0, []
    return {"delivered": len(sites), "held_dropped_or_cut": held, "sites": sorted(set(sites)), "injected_s": injected,
            "ok": len(sites) >= 3 and {"prepare", "dock"} <= set(sites)}


def stop_after(control_rows: list, ended_s: float | None, modes: list) -> str:
    """4.2 B. control_rows: [t, rear x, y, yaw, truth x, y, yaw]; modes: [(t, mode)] from the live log."""
    if ended_s is None:
        return "observed_end"
    rows = [r for r in control_rows if r[0] >= ended_s]
    if len(rows) < 2:
        return "observed_end"
    events = sorted((t, m) for t, m in modes if t is not None)

    def mode_at(t):
        last = None
        for et, m in events:
            if et <= t:
                last = m
            else:
                break
        return last  # None: unknown, never confirmable

    still_since = None
    for a, b in zip(rows, rows[1:]):
        dt = b[0] - a[0]
        mode, mode_b = mode_at(a[0]), mode_at(b[0])  # both ends of the pair (the last row's mode too)
        if dt > CONTROL_GAP_S or mode is None or mode in DRIVING_MODES or mode_b is None or mode_b in DRIVING_MODES:
            still_since = None  # a broken control record, an unknown or a driving mode restarts the confirmation
            continue
        v = ((b[4] - a[4]) ** 2 + (b[5] - a[5]) ** 2) ** 0.5 / dt if dt > 0 else 0.0
        still_since = (a[0] if still_since is None else still_since) if v < STILL_MPS else None
    return "safe_failure" if still_since is not None and rows[-1][0] - still_since >= STAND_S - 1e-9 else "observed_end"


def safety_of(session: dict, missions: dict, low: dict | None) -> dict:
    """4.1, confirmed counts and completeness apart. missions: {id: records state or None (missing)}."""
    contacts = len(session.get("forbidden_pocket_contacts") or [])
    missing = [m for m, st in missions.items() if st is None]
    contacts += sum(len((st or {}).get("forbidden_pocket_contacts") or []) for st in missions.values())
    ol = session.get("obstacle_layer") or {}
    unp, unp_dir = ol.get("unpermitted_entries"), ol.get("unpermitted_dir_entries")
    low_read = (low is not None and "error" not in low and bool(low.get("rows"))
                and "no samples" not in (low.get("unevaluated") or []))
    overlaps, arcs = [], []
    if low_read:
        overlaps = [k for k, r in low["rows"].items() if k.endswith("physical") and r.get("min_m") is not None
                    and r["min_m"] <= 0.0]
        arcs = list(low.get("physical_arc_violations") or [])
    damaged = 1 if session.get("scene_damaged") else 0
    confirmed = contacts + (unp or 0) + (unp_dir or 0) + len(overlaps) + len(arcs) + damaged
    unreadable = bool(missing) or unp is None or unp_dir is None or not low_read
    return {"forbidden_contacts": contacts, "missions_without_records": missing, "unpermitted_entries": unp,
            "unpermitted_dir_entries": unp_dir, "physical_overlaps": overlaps, "physical_arcs": len(arcs),
            "scene_damaged": session.get("scene_damaged"), "low_clearance_read": low_read, "unreadable": unreadable,
            "violations": confirmed if confirmed else (None if unreadable else 0)}


def low_report(rd: Path, session: dict, low_mod) -> dict | None:
    """The low-object report on a live session: its spooled samples and control rows laid out as a batch run's."""
    import numpy as np
    import yaml

    from forklift_core.live.spool import load_list

    tmp = Path(tempfile.mkdtemp(prefix="latb_low_"))
    try:
        res = dict(session)
        res["samples"] = load_list(rd / "spool", "state.samples")
        (tmp / "result.json").write_text(json.dumps(res, default=str))
        ctl = np.asarray(load_list(rd / "spool", "slam.control"), dtype=float).reshape(-1, 7)
        np.save(tmp / "slam_control.npy", ctl)
        layer = (session.get("arguments") or {}).get("obstacle_layer")
        p = Path(layer) if Path(layer).is_absolute() else ROOT / layer
        perm = (yaml.safe_load(p.read_text()) or {})["permission"]
        stopping = low_mod.StoppingModel(float(perm["latency_s"]), float(perm["decel_mps2"]), float(perm["margin_m"]))
        return low_mod.run_report(str(tmp), stopping, float(perm.get("envelope_m", 0.01)))
    except Exception as exc:  # noqa: BLE001 -- unreadable, never silently safe
        return {"error": repr(exc)[:300]}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def judge(rows: dict) -> dict:
    """4.6 over the effective runs; safety from every row."""
    dangerous = sorted(n for n, r in rows.items() if (r.get("safety") or {}).get("violations"))
    eff = {n: r for n, r in rows.items() if r.get("effective")}
    judged = {n: r for n, r in eff.items() if r["delay_ms"] in JUDGED}
    problems = {}
    for n, r in sorted(judged.items()):
        why = []
        if r["kinds"]["environment"]:
            why.append("invalid: " + "; ".join(r["kinds"]["environment"]))
        if (r.get("safety") or {}).get("violations") is None:
            why.append("safety unreadable")
        if r.get("arm_problems"):
            why.append("arm: " + "; ".join(r["arm_problems"]))
        if not r.get("mission_done"):
            why.append(f"mission not done ({r.get('stop')})")
        elif not (r.get("exposure") or {}).get("ok"):
            why.append("underexposed")
        if why:
            problems[n] = why
    judged_danger = [n for n in dangerous if n in judged]
    counts = {d: sum(1 for r in judged.values() if r["delay_ms"] == d) for d in JUDGED}
    complete = all(c == 3 for c in counts.values())
    clean = complete and not problems and not judged_danger
    failures = sorted(n for n, r in rows.items() if r.get("ran") and r.get("mission_done") is False)  # first runs too
    defects = ([{"run": n, "priority": 1, "class": "dangerous", "delay_ms": rows[n]["delay_ms"]} for n in dangerous]
               + [{"run": n, "priority": 2 if rows[n].get("stop") == "observed_end" else 3,
                   "class": rows[n].get("stop"), "delay_ms": rows[n]["delay_ms"]}
                  for n in failures if rows[n]["delay_ms"] in JUDGED]
               + [{"run": n, "priority": 4, "class": rows[n].get("stop"), "delay_ms": 500}
                  for n in failures if rows[n]["delay_ms"] not in JUDGED])
    launcher = {json.dumps(r.get("launcher"), sort_keys=True) for r in rows.values() if r.get("ran")}
    return {"decision": "B passes" if clean and len(launcher) <= 1 else (
        "to the lead" if not complete or len(launcher) > 1 else "B does not pass"),
            "problems": problems, "judged_per_delay": counts, "launcher_values_agree": len(launcher) <= 1,
            "defects": sorted(defects, key=lambda d: (d["priority"], d["run"]))}


def report_of(session: dict, mres: dict, mrec: dict) -> dict:
    plans = mrec.get("async_plans") or []
    by_site: dict = {}
    for p in plans:
        b = by_site.setdefault(p.get("site"), {"asked": 0, "dropped": {}, "wait_s": 0.0})
        b["asked"] += 1
        if p.get("dropped"):
            b["dropped"][p["dropped"]] = b["dropped"].get(p["dropped"], 0) + 1
        b["wait_s"] = round(b["wait_s"] + (p.get("finished_s") or 0) - (p.get("requested_s") or 0), 3)
    r = mres.get("result") or {}
    return {"plan_sites": by_site, "async_plan_drops": mrec.get("async_plan_drops"),
            "mission_sim_s": (r["ended_s"] - r["started_s"]) if r.get("ended_s") is not None and r.get("started_s")
            is not None else None, "delivery_error_m": mres.get("delivery_error_m"),
            "auto_factor": (session.get("live") or {}).get("pace") or session.get("pace")}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out_root", type=Path)
    ap.add_argument("--snap", type=Path, required=True)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    lists = _load("lat_lists", HERE / "lat_lists.py")
    c = _load("g3_1e_c_analyze", HERE.parent / "2026-11-11-g3-1e-c" / "g3_1e_c_analyze.py")
    low_mod = _load("low_clearance_report", ROOT / "tools/low_clearance_report.py")
    e0, g2m = c.e0, c.g2m
    registered = [j["name"] for j in lists.b_jobs(str(a.snap), str(a.out_root))]
    if [n for n in registered if (a.out_root / f"{n}.jsonl").exists()] != registered:
        print(json.dumps({"list_not_registered": True}))
        return 2

    def read(name: str) -> dict:
        e = json.loads((a.out_root / f"{name}.jsonl").read_text().splitlines()[0])
        d = int(e["env"]["LAT_DELAY_MS"])
        run = Path(e["argv"][-1])
        ps_f = a.out_root / f"{name}_ps.jsonl"
        ps = g2m.read_ps(ps_f.read_text()) if ps_f.exists() else None
        res_f = a.out_root / name / "bundle" / "results.jsonl"
        recs = [json.loads(x) for x in res_f.read_text().splitlines() if x.strip()] if res_f.exists() else []
        recs = [r for r in recs if r.get("name") == name]
        s = e0.OffSession(run, e, ps=ps)
        invalid = list(s.invalid) + [p for p in (c.attempt_problem(recs),
                                                 None if ps is not None else "no CPU sampler record") if p]
        rd = run / "live" / "epoch_1" / "run"
        ran = (run / "live" / "epoch_1" / "run.log").exists() or (rd / "result.json").exists()
        if not ran:
            invalid.append("session not started (no trace of running)")
        launcher = {**c.launcher_values(run), "map": (_json(rd / "result.json") or {}).get("arguments", {}).get(
            "slam_map_bundle") is not None}
        session = _json(rd / "result.json") or {}
        mres = _json(rd / "missions" / "m1" / "result.json") or {}
        mrec_all = _json(rd / "missions" / "m1" / "records.json")
        mrec = (mrec_all or {}).get("state") or {}
        outcome = (mres.get("result") or {}).get("outcome")
        arm = []
        got_arg = (session.get("arguments") or {}).get("latency_plan_ms")
        got_inj = ((session.get("latency_injection") or {}).get("plan") or {}).get("delay_ms")
        if ran and session:
            if float(got_arg or 0.0) != float(d):
                arm.append(f"runner --latency-plan-ms {got_arg}, registered {d}")
            if (d and got_inj != float(d)) or (not d and got_inj is not None):
                arm.append(f"injection record delay {got_inj}, registered {d}")
        if ran and session:
            arm += [f"{k}={launcher.get(k)}, registered {v}" for k, v in REGISTERED_LAUNCHER.items()
                    if launcher.get(k) != v]
        row = {"name": name, "delay_ms": d, "ran": ran, "kinds": split_invalid(invalid, ran), "arm_problems": arm,
               "launcher": launcher,
               "mission_done": (outcome == "done") if ran else None, "mission_outcome": outcome,
               "mission_reason": (mres.get("result") or {}).get("reason"),
               "exposure": exposure_of(session, mrec, d) if outcome == "done" else None,
               "evidence_stale_ticks": e0.stale_ticks(session), **report_of(session, mres, mrec)}
        if ran:
            missions = {"m1": mrec if mrec_all is not None else None}
            row["safety"] = safety_of(session, missions, low_report(rd, session, low_mod) if session else None)
        if ran and outcome != "done":
            ctl, modes = [], []
            try:
                import numpy as np

                from forklift_core.live.spool import load_list

                ctl = np.asarray(load_list(rd / "spool", "slam.control"), dtype=float).reshape(-1, 7).tolist()
            except Exception:  # noqa: BLE001 -- no control record: the stop is not confirmed
                pass
            for line in (rd / "live_log.jsonl").read_text().splitlines() if (rd / "live_log.jsonl").exists() else []:
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue
                if ev.get("event") == "mode":
                    modes.append((ev.get("tick_t"), ev.get("to")))
            row["stop"] = stop_after(ctl, (mres.get("result") or {}).get("ended_s"), modes)
        return row

    rows: dict = {}
    not_allowed = []
    for n in registered:
        rows[n] = read(n)
        rr = f"lat_r_{n}"
        rows[n]["effective"] = not (a.out_root / f"{rr}.jsonl").exists()
        if not rows[n]["effective"]:
            first = rows[n]
            if not first["kinds"]["environment"] or first.get("mission_done") is False or first["kinds"]["mission"]:
                not_allowed.append(n)  # a rerun replaces only an environment-invalid first run without a failure
                first["effective"] = True
            rows[rr] = {**read(rr), "rerun_of": n, "effective": n not in not_allowed}
    out = {"sessions": rows, "result": judge(rows), "rerun_not_allowed": not_allowed,
           "rerun_needed": sorted(n for n, r in rows.items() if r["kinds"]["environment"] and r.get("effective")
                                  and not r.get("rerun_of") and not (r["ran"] and r.get("mission_done") is False)),
           "to_lead": sorted(n for n, r in rows.items() if r["kinds"]["environment"] and r.get("rerun_of"))}
    if a.json is not None:
        a.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({k: out[k] for k in ("result", "rerun_not_allowed", "rerun_needed", "to_lead")}, indent=1,
                     default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
