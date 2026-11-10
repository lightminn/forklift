"""Real-time plan 5.23 G2-1a: the Isaac check's measures and its pass line (5.18 G2-1), fixed before the results.

    PYTHONPATH=SNAP/src python g2_1a_analyze.py OUT_DIR --list OUT_DIR.jsonl [--ps PS.jsonl] [--json out.json]

Sessions are g2_1a_lists.py's (hall A6P and line_m far K f 0.4, sync and async alternating, pairs (01,02) (05,06)
(09,10) hall and (03,04) (07,08) (11,12) line_m). Validity is g2m_analyze.py's session reading (live log, tick trace,
probe exit, the monitor's GPU processes; with --ps the other-user CPU rule of the 5.22 F supplement) with this run's
registered conditions; a session whose arguments do not carry its arm's --live-capture-dump mode is invalid.

Per session, over the trace from the first control tick on (5.23 names no AUTO restriction; the captures fall in the
missions):
- capture ticks: a capture renders over consecutive control ticks, every one of them flagging its step ``capture``
  (run_transport.py, D9c); once its frame is taken, the next control tick runs capture_and_detect, where save_capture
  writes (sync) or hands over (async) the files. So the saved capture's tick is the control tick of the row after a
  run of capture-flagged rows: row (last of the run) + 1, its ctrl_end - ctrl_start and its start gap (Codex review
  P1: the retry rows of a run save nothing). The records cannot tell every run's end apart: a STOP in a capture tick
  aborts it and a pause abandons it (no save), while a capture that saved and then failed its detection posts PAUSED
  in that same tick (Codex rounds 2-3). So each session keeps two populations: all runs' following ticks (A) and those
  of the runs whose status lines (the flagged rows and the tick after) are all AUTO with no stop (B; the others are
  counted as suspect). Line 3 must hold on both. The control tick of the run's last row is reported beside it as a
  check that the save sits in the tick after; the async dumper's written entries are reported beside the run counts.
  A capture tick whose control time was not recorded (a tick that failed may skip its end mark) is counted as
  missing and leaves that session's pair undecided for line 3.
- start gaps over 0.20 s (from the first control tick): each gap's parts as g2m_analyze.py's M1 (world, step_rest,
  after, control_next); a gap is control-made when control_next is strictly its largest measured part and at least
  half the gap (so a gap mostly outside the measured parts is not control-made; a part not measured counts as none).
  Counted for 0.20 < gap <= 0.25 (near) and gap > 0.25 (over), control-made and from any source.
- the capture files in the run folder (perception_capture_<n>_rgb.png, _depth_m.npy, _depth_vis.png): the attempt
  numbers restart per mission and the files carry no mission id, so a later mission overwrites the earlier ones and
  the folder holds each number's last capture; a number is taken before the capture renders, so an abandoned attempt
  leaves a gap (Codex round 3). Every number present must have rgb and depth, the depth picture too when the depth has
  a finite value, and every file must open (PIL verify, numpy load). With async -- the arm under test -- every entry
  the dumper wrote (``done``) must have its files: there the expected set is known exactly. The sync arm is the
  unchanged path (files written on the control thread before the tick goes on); only its numbers present are known.
- result.json ``capture_dump`` (async): dropped_full, errors, unsettled; the live log's end summary dropped lines.
- the missions' outcomes (g2m_analyze.py), the forbidden pocket contacts -- each finished mission's own record
  (missions/<id>/records.json, where the mission's state goes at its end) plus result.json's for a mission still on; a
  record without the key makes the count unknown (Codex round 2) -- and the whole-trace world.step counts over 0.25 / 0.5 s (M8; the lead's 2026-11-10 decision: the hall sessions are
  where the old hall long steps are looked for again).

Pass (5.23 / 5.18 G2-1), on the valid sessions, all of:
1. no control-made start gap over 0.25 s in any async session;
2. per scene (hall, line_m), control-made near gaps (0.20-0.25 s) as a share of the near gaps lower with async than
   with sync, each arm pooled over its three sessions, and their count not higher -- when neither arm of a scene has a
   control-made near gap the scene holds (nothing to lower; a registration of this analyzer, 5.23 names no zero case);
3. the captures' control time lower with async in every one of the six pairs (Delta = the sync session's median over
   its capture ticks - the async one's, every Delta > 0), and the median of the six Deltas at least 0.03 s (both
   scenes together; a pair need not drop 0.03 s itself) -- on population A and on population B alike;
4. missions and safety no worse: in every pair the async missions all done when the sync ones are, an unknown sync
   outcome leaving the pair undecided (reported, the line not passed), and the async forbidden pocket contacts no
   more than the sync ones;
5. no capture file missing or unreadable, no dropped_full, error or unsettled capture, no lost live-log line;
6. per scene, start gaps over 0.25 s from any source no more with async than with sync, summed over its sessions.
A pair with an invalid session is left out of 3-4 and reported; every line names what it counted.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
_spec = importlib.util.spec_from_file_location("g2m_analyze", HERE.parents[1] / "2026-11-09-g2-measure" / "g2m_analyze.py")
g2m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g2m)
C, BIT = g2m.C, g2m.BIT
NEAR_S, EDGE_S = 0.20, 0.25
PAIRS = {"H": [(1, 2), (5, 6), (9, 10)], "F": [(3, 4), (7, 8), (11, 12)]}
MEDIAN_DROP_S = 0.03


def arm_of(name: str) -> tuple[int, str, str]:
    m = re.fullmatch(r"g2a_(\d+)_([HF])_(sync|async)", name)
    if not m:
        raise ValueError(f"not a g2_1a_lists.py name: {name}")
    return int(m.group(1)), m.group(2), m.group(3)


class Session(g2m.Session):
    def _check_conditions(self):
        c = self.cond
        _, scene, mode = arm_of(self.name)
        for k in c:
            if k.startswith(g2m.FORBIDDEN):
                self.invalid.append(f"{k} set (dumps and the dual run stay out of the check)")
        for k in g2m.INSTR_ON:  # INSTR off in every session: no profiler on either arm
            if c.get(k) is not None:
                self.invalid.append(f"{k} set in an INSTR off session")
        if scene == "H":
            want = {"FACTOR": "1.0", "CATCHUP": "6", "PHASE": "6", "LEVEL": "off"}
        else:
            want = {"SET": "far", "ARM": "K", "FACTOR": "0.4", "FORKLIFT_LIVE_CATCHUP_STEPS": "6"}
        for k, v in want.items():
            if c.get(k) != v:
                self.invalid.append(f"{k}={c.get(k)}, registered {v}")
        extra = c.get("EXTRA_ARGS", "").replace("\\ ", " ")  # r1c_session.sh writes it with printf %q
        if f"--live-capture-dump {mode}" not in extra:
            self.invalid.append(f"EXTRA_ARGS does not carry --live-capture-dump {mode}: {c.get('EXTRA_ARGS')!r}")


def capture_ticks(D: np.ndarray, cmd: list | None = None) -> dict:
    bits = np.nan_to_num(D[:, C["flags"]]).astype(int)
    flagged = (bits & BIT["capture"]) != 0
    last = flagged & ~np.r_[flagged[1:], False]  # a run's last flagged row (the next row is not flagged)
    rows = np.flatnonzero(last)
    rows = rows[rows + 1 < len(D)]
    by_step = {}
    for r in cmd or []:
        if r.get("step") is not None:
            by_step.setdefault(int(r["step"]), []).append(r)
    all_rows = rows.copy()
    kept, cut = [], 0
    for row in rows:
        first = row
        while first > 0 and flagged[first - 1]:
            first -= 1
        steps = [D[k, C["live_step"]] for k in range(first, row + 2)]
        lines = [x for st in steps if np.isfinite(st) for x in by_step.get(int(st), [])]
        if any(x.get("mode") != "AUTO" or (x.get("stop_kind") or "none") != "none" for x in lines):
            cut += 1
            continue
        kept.append(row)
    rows = np.array(kept, dtype=int)
    ctrl_all = (D[:, C["ctrl_end"]] - D[:, C["ctrl_start"]])[all_rows + 1] if len(all_rows) else np.zeros(0)
    ctrl = D[:, C["ctrl_end"]] - D[:, C["ctrl_start"]]
    nxt = ctrl[rows + 1]
    same = ctrl[rows]
    gap = D[rows + 1, C["S"]] - D[rows, C["S"]]

    def q(v):
        missing = int((~np.isfinite(v)).sum())  # a tick that failed may leave its end unrecorded (Codex round 4)
        v = v[np.isfinite(v)]
        out = {"n": int(v.size), "missing": missing}
        if v.size:
            out.update(median=float(np.median(v)), p99=float(np.percentile(v, 99)), max=float(v.max()))
        return out
    return {"runs": int(len(all_rows)), "runs_suspect": cut, "flagged_rows": int(flagged.sum()),
            "rows": all_rows.tolist(), "rows_clean": rows.tolist(),
            "control_next_s": q(ctrl_all), "control_next_clean_s": q(nxt),
            "control_last_flagged_row_s": q(same), "gap_clean_s": q(gap)}


def gap_counts(D: np.ndarray, first: int) -> dict:
    S = D[:, C["S"]]
    out = {"near": 0, "near_control": 0, "over": 0, "over_control": 0}
    for i in range(first, len(D) - 1):
        gap = S[i + 1] - S[i]
        if not np.isfinite(gap) or gap <= NEAR_S:
            continue
        parts = {"world": D[i, C["world_end"]] - D[i, C["world_start"]],
                 "step_rest": D[i, C["step_end"]] - D[i, C["world_end"]],
                 "after": D[i, C["after_end"]] - D[i, C["step_end"]],
                 "control_next": D[i + 1, C["ctrl_end"]] - D[i + 1, C["ctrl_start"]]}
        parts = {k: (float(v) if np.isfinite(v) else 0.0) for k, v in parts.items()}
        others = max(v for k, v in parts.items() if k != "control_next")
        control = parts["control_next"] > others and parts["control_next"] >= 0.5 * gap
        key = "near" if gap <= EDGE_S else "over"
        out[key] += 1
        out[key + "_control"] += int(control)
    return out


def capture_files(run: Path, entries: list | None) -> dict:
    from PIL import Image

    by = defaultdict(dict)
    for f in run.glob("perception_capture_*"):
        m = re.fullmatch(r"perception_capture_(\d+)_(rgb\.png|depth_m\.npy|depth_vis\.png)", f.name)
        if m:
            by[int(m.group(1))][m.group(2)] = f
    missing, unreadable = [], []
    for n, files in sorted(by.items()):
        for need in ("rgb.png", "depth_m.npy"):
            if need not in files:
                missing.append(f"{n}:{need}")
        depth = None
        for kind, f in files.items():
            try:
                if kind.endswith(".png"):
                    with Image.open(f) as im:
                        im.verify()
                else:
                    depth = np.load(f, allow_pickle=False)
            except Exception as exc:  # noqa: BLE001 -- recorded, judged below
                unreadable.append(f"{f.name}: {type(exc).__name__}")
        if depth is not None and np.isfinite(depth).any() and "depth_vis.png" not in files:
            missing.append(f"{n}:depth_vis.png")
    for e in entries or []:
        if e.get("done") and not {"rgb.png", "depth_m.npy"} <= set(by.get(int(e["attempt"]), {})):
            missing.append(f"{e['attempt']}: written by the dumper but its files are not there")
    return {"attempts": sorted(by), "missing": missing, "unreadable": unreadable}


def mission_ids(out: Path) -> list[str] | None:
    """The missions the probe reports (the confirm scenario's one, the cycle scenario's missions_run); None when the
    probe report is missing or a reported result has no mission id."""
    f = out / "probe.json"
    if not f.exists():
        return None
    j = json.loads(f.read_text())
    results = [(j.get("confirm") or {}).get("mission_result")] + [m.get("result") for m in j.get("missions_run") or []]
    results = [r for r in results if r is not None]
    if any(not r.get("mission_id") for r in results):
        return None  # a reported mission without its id: its record cannot be found (Codex round 4)
    return [r["mission_id"] for r in results]


def forbidden_contacts(out: Path, run: Path, result: dict | None) -> int | None:
    """Forbidden pocket contacts of the session: every mission the probe reports must have its record
    (missions/<id>/records.json, where the mission's state goes at its end) with a list there, and result.json a list
    for a mission still on; anything missing or null makes the count unknown (Codex review rounds 2-3)."""
    ids = mission_ids(out)
    if ids is None or not result or not isinstance(result.get("forbidden_pocket_contacts"), list):
        return None
    total = len(result["forbidden_pocket_contacts"])
    for mid in ids:
        f = run / "missions" / mid / "records.json"
        if not f.exists():
            return None
        state = (json.loads(f.read_text()).get("state")) or {}
        if not isinstance(state.get("forbidden_pocket_contacts"), list):
            return None
        total += len(state["forbidden_pocket_contacts"])
    return total


def analyse_session(s: Session) -> dict:
    n, scene, mode = arm_of(s.name)
    row = {"name": s.name, "n": n, "scene": scene, "mode": mode, "invalid": s.invalid, "ps": s.ps,
           "other_jobs": s.other_jobs, "mission_outcomes": s.mission_outcomes, "mission_done": s.mission_done}
    if s.trace is None or s.invalid:
        return row
    D = s.trace
    dump = (s.result or {}).get("capture_dump")
    end = next((r["log_end"] for r in s.log if isinstance(r.get("log_end"), dict)), {})
    cap = capture_ticks(D, s.cmd)
    row.update({"capture": cap, "gaps": gap_counts(D, s.first_ctrl_row()),
                "files": capture_files(s.run, (dump or {}).get("entries") if mode == "async" else None),
                "capture_dump": None if dump is None else {k: dump.get(k) for k in ("mode", "closed", "dropped_full",
                                                                                   "errors", "unsettled")},
                "log_dropped_lines": end.get("dropped_lines"), "M8": s.m8_report_only(),
                "forbidden_pocket_contacts": forbidden_contacts(s.out, s.run, s.result)})
    if mode == "async" and dump is not None:
        written = sum(1 for e in dump.get("entries") or [] if e.get("mode") in ("async", "dropped_full"))
        row["capture"]["entries_written"] = written  # reported beside the run counts
    return row


def verdict(rows: list[dict]) -> dict:
    ok = {r["n"]: r for r in rows if not r["invalid"] and "gaps" in r}
    by_mode = {m: [r for r in ok.values() if r["mode"] == m] for m in ("sync", "async")}
    lines = {}
    over_ctrl = sum(r["gaps"]["over_control"] for r in by_mode["async"])
    lines["1_no_control_over_0_25_async"] = {"count": over_ctrl, "pass": over_ctrl == 0}
    per_scene = {}
    for scene in PAIRS:
        near = {m: (sum(r["gaps"]["near_control"] for r in v if r["scene"] == scene),
                    sum(r["gaps"]["near"] for r in v if r["scene"] == scene)) for m, v in by_mode.items()}
        share = {m: (c / t if t else 0.0) for m, (c, t) in near.items()}
        neither = near["sync"][0] == 0 and near["async"][0] == 0
        per_scene[scene] = {"control_near_by_arm": near, "share": share,
                            "pass": neither or (share["async"] < share["sync"] and near["async"][0] <= near["sync"][0])}
    lines["2_near_control_share_lower"] = {"scenes": per_scene, "pass": all(v["pass"] for v in per_scene.values())}
    drops, missions, left_out = [], [], []
    for scene, pairs in PAIRS.items():
        for a, b in pairs:
            ra, rb = ok.get(a), ok.get(b)
            if ra is None or rb is None:
                left_out.append([a, b])
                continue
            sync, asy = (ra, rb) if ra["mode"] == "sync" else (rb, ra)
            contacts = (sync.get("forbidden_pocket_contacts"), asy.get("forbidden_pocket_contacts"))
            missions.append({"pair": [a, b], "sync_done": sync["mission_done"], "async_done": asy["mission_done"],
                             "sync_outcomes": sync["mission_outcomes"], "async_outcomes": asy["mission_outcomes"],
                             "forbidden_pocket_contacts": contacts,
                             "pass": sync["mission_done"] is not None and None not in contacts
                             and not (sync["mission_done"] and not asy["mission_done"])
                             and contacts[1] <= contacts[0]})
            if any(r["capture"][k].get("missing") for r in (sync, asy) for k in ("control_next_s",
                                                                              "control_next_clean_s")):
                left_out.append([a, b])  # a capture tick without its control time: line 3 undecided for the pair
                continue
            for pop, key in (("A", "control_next_s"), ("B", "control_next_clean_s")):
                ms, ma = sync["capture"][key].get("median"), asy["capture"][key].get("median")
                drops.append({"pair": [a, b], "scene": scene, "population": pop, "sync_median_s": ms,
                              "async_median_s": ma, "drop_s": None if ms is None or ma is None else ms - ma})
    pops = {}
    for pop in ("A", "B"):
        d = [x["drop_s"] for x in drops if x["population"] == pop]
        pops[pop] = {"median_drop_s": float(np.median(d)) if d and None not in d else None,
                     "pass": not left_out and len(d) == 6 and None not in d and all(x > 0 for x in d)
                     and float(np.median(d)) >= MEDIAN_DROP_S}
    lines["3_capture_control_lower"] = {"pairs": drops, "left_out": left_out, "populations": pops,
                                        "pass": pops["A"]["pass"] and pops["B"]["pass"]}
    lines["4_missions_no_worse"] = {"pairs": missions, "pass": bool(missions) and all(m["pass"] for m in missions)}
    bad = []
    for r in ok.values():
        f = r["files"]
        if f["missing"] or f["unreadable"]:
            bad.append(f"{r['name']}: files {f['missing'] + f['unreadable']}")
        cd = r["capture_dump"]
        if r["mode"] == "async":
            if cd is None or cd.get("mode") != "async":
                bad.append(f"{r['name']}: no async capture_dump record")
            elif cd.get("dropped_full") or cd.get("errors") or cd.get("unsettled"):
                bad.append(f"{r['name']}: capture_dump {cd}")
        if r["log_dropped_lines"]:
            bad.append(f"{r['name']}: live log dropped {r['log_dropped_lines']}")
    lines["5_no_loss"] = {"problems": bad, "pass": not bad}
    over = {scene: {m: sum(r["gaps"]["over"] for r in v if r["scene"] == scene) for m, v in by_mode.items()}
            for scene in PAIRS}
    lines["6_any_source_over_0_25_not_more"] = {"scenes": over,
                                                "pass": all(o["async"] <= o["sync"] for o in over.values())}
    return {"valid_sessions": sorted(ok), "invalid_sessions": sorted(r["name"] for r in rows if r["invalid"]),
            "lines": lines, "pass": all(v["pass"] for v in lines.values()) and not left_out}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--ps", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    ps = g2m.read_ps(a.ps.read_text()) if a.ps else None
    entries = [json.loads(x) for x in a.list.read_text().splitlines() if x.strip()]
    rows = [analyse_session(Session(a.out / e["name"], e, ps=ps)) for e in entries]
    out = {"sessions": rows, "verdict": verdict(rows),
           "hall_long_world_steps": [{"name": r["name"], **(r.get("M8") or {})} for r in rows if r["scene"] == "H"]}
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "invalid", "mission_done")}, default=str))
    print(json.dumps(out["verdict"], indent=1, default=str))
    print(json.dumps(out["hall_long_world_steps"], default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
