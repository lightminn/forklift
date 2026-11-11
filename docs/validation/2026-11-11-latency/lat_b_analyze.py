"""The latency test B's judgement (docs/plans/2026-11-11-latency-tolerance.md 4.6, 4.7; fixed before the results).

    python lat_b_analyze.py OUT_ROOT --snap SNAP [--json OUT.json]

Fifteen live sessions (lat_lists.b_jobs): line_m far K f 1.0 confirm, the planning answers held d ms after arrival.
Per session: environment validity (the CPU sampler's overlap, its record, one bundle attempt, a trace of running --
the 1e (c) rules, read by its analyzer), safety (forbidden pocket contacts, unpermitted entries, scene_damaged; read
whenever the session ran), the expected mission (m1) done by the mission results, not the session's success, the
stop-vs-end split of a failure, the exposure (>= 3 delayed answers, prepare and dock among them, for a session that
finished its mission). "B passes" when the 12 judged sessions (d 0/25/50/100) are all safe, done and exposed. d 500 and
the per-arm waits, drops and pacing are reported only. A rerun (lat_r_<name>) is a row of its own; nothing it shows
erases its first run's safety or mission reading.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
JUDGED = (0, 25, 50, 100)
STAND_S, STILL_MPS = 0.5, 0.05


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


def exposure_of(session_result: dict, mission_state: dict) -> dict:
    """4.5 B: the delayed answers delivered (arrival + d), joined to the mission's async plan sites by job id."""
    recs = ((session_result or {}).get("latency_injection") or {}).get("plan", {}).get("records") or []
    sites = {p.get("job"): p.get("site") for p in (mission_state or {}).get("async_plans") or []}
    delivered = [r for r in recs if r.get("delivered") is not None]
    got = sorted({sites.get(r["job"]) for r in delivered} - {None})
    return {"delivered": len(delivered), "held_dropped_or_cut": len(recs) - len(delivered), "sites": got,
            "injected_s": sorted(round(float(r["injected_s"]), 4) for r in delivered),
            "ok": len(delivered) >= 3 and {"prepare", "dock"} <= set(got)}


def stop_after(control_rows: list, ended_s: float | None) -> str | None:
    """4.2 for a failed mission in a live session: safe_failure when, after the mission's end, the truth speed stays
    under 0.05 m/s for 0.5 s or more up to the session's last control row; observed_end otherwise. control_rows:
    [t, rear x, y, yaw, truth x, y, yaw] (slam.control)."""
    if ended_s is None:
        return "observed_end"
    rows = [r for r in control_rows if r[0] >= ended_s]
    if len(rows) < 2:
        return "observed_end"
    still_since = None
    for a, b in zip(rows, rows[1:]):
        dt = b[0] - a[0]
        v = ((b[4] - a[4]) ** 2 + (b[5] - a[5]) ** 2) ** 0.5 / dt if dt > 0 else 0.0
        if v < STILL_MPS:
            still_since = a[0] if still_since is None else still_since
        else:
            still_since = None
    return "safe_failure" if still_since is not None and rows[-1][0] - still_since >= STAND_S - 1e-9 else "observed_end"


def judge(rows: dict) -> dict:
    """4.6: B passes when every judged session (d 0/25/50/100, the effective run of each) is valid, safe, done and
    exposed. Safety events from any session (first runs, reruns, invalid ones, d 500) are listed first."""
    dangerous = sorted(n for n, r in rows.items() if (r.get("safety") or {}).get("violations"))
    eff = {n: r for n, r in rows.items() if r.get("effective")}
    judged = {n: r for n, r in eff.items() if r["delay_ms"] in JUDGED}
    problems = {}
    for n, r in sorted(judged.items()):
        why = []
        if r["invalid"]:
            why.append("invalid: " + "; ".join(r["invalid"]))
        if (r.get("safety") or {}).get("violations") is None:
            why.append("safety unreadable")
        if not r.get("mission_done"):
            why.append(f"mission not done ({r.get('stop')})")
        elif not (r.get("exposure") or {}).get("ok"):
            why.append("underexposed")
        if why:
            problems[n] = why
    counts = {d: sum(1 for r in judged.values() if r["delay_ms"] == d) for d in JUDGED}
    complete = all(c == 3 for c in counts.values())
    ok = complete and not dangerous and not problems
    return {"decision": "B passes" if ok else ("to the lead" if not complete else "B does not pass"),
            "dangerous": dangerous, "problems": problems, "judged_per_delay": counts,
            "defects": dangerous + sorted(n for n, w in problems.items() if not any(x.startswith("invalid") for x in w))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out_root", type=Path)
    ap.add_argument("--snap", type=Path, required=True)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    lists = _load("lat_lists", HERE / "lat_lists.py")
    c = _load("g3_1e_c_analyze", HERE.parent / "2026-11-11-g3-1e-c" / "g3_1e_c_analyze.py")
    e0, g2m = c.e0, c.g2m
    registered = [j["name"] for j in lists.b_jobs(str(a.snap), str(a.out_root))]
    names = [n for n in registered if (a.out_root / f"{n}.jsonl").exists()]
    if names != registered:
        print(json.dumps({"list_not_registered": sorted(set(registered) - set(names))}))
        return 2
    rows: dict = {}

    def read(name: str) -> dict:
        e = json.loads((a.out_root / f"{name}.jsonl").read_text().splitlines()[0])
        jobdir = a.out_root / name
        run = Path(e["argv"][-1])
        ps_f = a.out_root / f"{name}_ps.jsonl"
        ps = g2m.read_ps(ps_f.read_text()) if ps_f.exists() else None
        res_f = jobdir / "bundle" / "results.jsonl"
        recs = [json.loads(x) for x in res_f.read_text().splitlines() if x.strip()] if res_f.exists() else []
        recs = [r for r in recs if r.get("name") == name]
        s = e0.OffSession(run, e, ps=ps)
        s.invalid.extend(p for p in (c.attempt_problem(recs), None if ps is not None else "no CPU sampler record") if p)
        ran = (run / "live" / "epoch_1" / "run.log").exists() or (run / "live" / "epoch_1" / "run" / "result.json").exists()
        rd = run / "live" / "epoch_1" / "run"
        session = _json(rd / "result.json") or {}
        mres = _json(rd / "missions" / "m1" / "result.json") or {}
        mrec = (_json(rd / "missions" / "m1" / "records.json") or {}).get("state") or {}
        outcome = (mres.get("result") or {}).get("outcome")
        safety = c.safety_of(s)
        if safety is not None:
            damaged = session.get("scene_damaged")
            safety = {**safety, "scene_damaged": damaged}
            parts = [safety.get("forbidden_contacts"), safety.get("unpermitted_entries"),
                     safety.get("unpermitted_dir_entries")]
            confirmed = sum(x for x in parts if x) + (1 if damaged else 0)
            safety["violations"] = confirmed if confirmed else (None if any(x is None for x in parts) else 0)
        row = {"name": name, "delay_ms": int(e["env"]["LAT_DELAY_MS"]), "invalid": list(s.invalid), "ran": ran,
               "safety": safety if ran else None, "mission_done": outcome == "done", "mission_outcome": outcome,
               "mission_reason": (mres.get("result") or {}).get("reason"),
               "exposure": exposure_of(session, mrec) if outcome == "done" else None,
               # report only
               "async_plan_drops": mrec.get("async_plan_drops"), "plan_waits_s": [
                   round((p.get("finished_s") or 0) - (p.get("requested_s") or 0), 3)
                   for p in mrec.get("async_plans") or []],
               "delivery_error_m": mres.get("delivery_error_m"), "evidence_stale_ticks": e0.stale_ticks(session)}
        if outcome != "done" and ran:
            ctl = []
            try:
                import numpy as np

                from forklift_core.live.spool import load_list

                ctl = np.asarray(load_list(rd / "spool", "slam.control"), dtype=float).reshape(-1, 7).tolist()
            except Exception:  # noqa: BLE001 -- no control record: the stop is not confirmed
                pass
            row["stop"] = stop_after(ctl, (mres.get("result") or {}).get("ended_s"))
        if not ran:
            row["invalid"].append("session not started (no trace of running)")
        return row

    for n in registered:
        rows[n] = read(n)
        rr = f"lat_r_{n}"
        rows[n]["effective"] = not (a.out_root / f"{rr}.jsonl").exists()
        if not rows[n]["effective"]:
            rows[rr] = {**read(rr), "rerun_of": n, "effective": True}
    # 4.7: an environment-invalid session is run once more, unless it ran and failed its mission (not erased by a rerun)
    rerun = sorted(n for n, r in rows.items() if r["invalid"] and r.get("effective") and not r.get("rerun_of")
                   and not (r["ran"] and not r["mission_done"]))
    out = {"sessions": rows, "result": judge(rows), "rerun_needed": rerun,
           "to_lead": sorted(n for n, r in rows.items() if r["invalid"] and r.get("rerun_of"))}
    if a.json is not None:
        a.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps(out["result"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
