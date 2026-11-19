#!/usr/bin/env python3
"""The bottom-deck structure stage-1 Isaac check (docs/plans/2026-11-18-bottom-deck-structure-default.md 3-4): one row
per run of the bds1 list and the plan's verdicts for S-, S+ and S+g.

Per run (its <run>/run/result.json and riib_defaults.txt): the launcher's applied BOTTOM_DECK and scene file, the
mission result, every phase change's time, and the bottom-deck block -- faces and their verdicts, the record's
verdict, the refusal (reason, stage, detail), the re-observation outcome and the insert permission.

  S-   each run: the far observation adopted (the side stage began: a target recorded), a face judged closed_face
       (unobserved), the refusal from it, and no transition to insert. A run refused before the side stage is
       reported apart (not an S- sample).
  S+   per scene: all three structure runs reach both face verdicts and are permitted; at least two complete. Cycle
       differences only on seeds where both arms completed; each arm's stopping stage otherwise.
  S+g  all three: two distinct faces observed, record verdict absent, insert permitted (no no_bottom_deck); at least
       two complete.

  python summarize.py BATCH_DIR [--json OUT]
"""

import argparse
import json
import re
import sys
from pathlib import Path


def phase_times(result: dict) -> dict:
    """The first time each phase was entered."""
    out = {}
    for tr in result.get("transitions") or []:
        out.setdefault(tr.get("to"), tr.get("time_s"))
    return out


def run_row(run_dir: Path) -> dict:
    row = {"name": run_dir.name}
    defaults = run_dir / "riib_defaults.txt"
    if defaults.exists():
        text = defaults.read_text()
        m = re.search(r"BOTTOM_DECK=(\S+) BOTTOM_DECK_FROM=\S+ BOTTOM_DECK_REASON=(\S+)", text)
        row["bottom_deck"], row["bottom_deck_reason"] = (m.group(1), m.group(2)) if m else (None, None)
        m = re.search(r"SCENE_FILE=(\S+)", text)
        row["scene_file"] = Path(m.group(1)).name if m else None
    code = run_dir / "run/exit_code"  # the runner's own {"code", "reason"} (a failure ends without mission_result)
    if code.exists():
        recorded = json.loads(code.read_text())
        row["exit_code"], row["exit_reason"] = recorded.get("code"), recorded.get("reason")
    path = run_dir / "run/result.json"
    if not path.exists():
        row["status"] = "no_result"
        return row
    result = json.loads(path.read_text())
    times = phase_times(result)
    row.update(status="result", success=bool(result.get("success")), mission_result=result.get("mission_result"),
               phases=times, inserted="insert" in times, completed="complete" in times)
    deck = result.get("bottom_deck")
    if deck is not None:
        record = deck.get("record") or {}
        faces = deck.get("faces") or []
        attempts = deck.get("attempts") or []
        row["deck"] = {
            "status": deck.get("status"), "target": deck.get("target") is not None,
            "faces": [(f.get("face"), f.get("verdict"), f.get("reason")) for f in faces],
            "record_verdict": record.get("verdict"),
            "refused": deck.get("refused"), "permitted": deck.get("permitted"),
            "reachable": deck.get("reachable"),
            "reobservation": (deck.get("reobservation") or {}).get("refusal", "none")
            if deck.get("reobservation") is not None else None,
            "first_leg_s": min((a.get("time_s") for a in attempts if a.get("time_s") is not None), default=None),
            "reobserved_s": (deck.get("reobservation") or {}).get("time_s"),
        }
    return row


def arm_of(name: str) -> tuple:
    m = re.match(r"(sm|sp|sg)_(.+?)(?:_(on|off))?_s(\d+)$", name)
    return (m.group(1), m.group(2), m.group(3) or "on", int(m.group(4))) if m else (None, None, None, None)


def judge(rows: list) -> dict:
    out = {}
    sm = [r for r in rows if arm_of(r["name"])[0] == "sm"]
    sm_cases = []
    for r in sm:
        d = r.get("deck") or {}
        began = bool(d.get("target"))
        closed = any(v == "unobserved" and (reason or "").startswith("closed") for _, v, reason in d.get("faces", []))
        refused = bool(d.get("refused"))
        ok = began and closed and refused and not r.get("inserted")
        sm_cases.append({"name": r["name"], "side_stage": began, "closed_face": closed, "refused": refused,
                         "inserted": r.get("inserted"), "sample": began, "pass": ok if began else None,
                         "refusal": d.get("refused"), "mission_result": r.get("mission_result"),
                         "exit_reason": r.get("exit_reason")})
    samples = [c for c in sm_cases if c["sample"]]
    out["S-"] = {"runs": sm_cases, "samples": len(samples),
                 "pass": len(samples) >= 3 and all(c["pass"] for c in samples)}
    out["S+"] = {}
    for scene in sorted({arm_of(r["name"])[1] for r in rows if arm_of(r["name"])[0] == "sp"}):
        on = {arm_of(r["name"])[3]: r for r in rows if arm_of(r["name"])[:3] == ("sp", scene, "on")}
        off = {arm_of(r["name"])[3]: r for r in rows if arm_of(r["name"])[:3] == ("sp", scene, "off")}
        permitted = [s for s, r in on.items() if (r.get("deck") or {}).get("permitted")
                     and len((r.get("deck") or {}).get("faces", [])) == 2]
        completed = [s for s, r in on.items() if r.get("completed")]
        pairs = []
        for seed in sorted(set(on) & set(off)):
            a, b = on[seed], off[seed]
            row = {"seed": seed, "on_completed": a.get("completed"), "off_completed": b.get("completed"),
                   "on_stop": None if a.get("completed") else a.get("mission_result"),
                   "off_stop": None if b.get("completed") else b.get("mission_result")}
            for key in ("approach", "insert", "complete"):
                ta, tb = (a.get("phases") or {}).get(key), (b.get("phases") or {}).get(key)
                row[f"{key}_on_s"], row[f"{key}_off_s"] = ta, tb
                row[f"{key}_delta_s"] = None if ta is None or tb is None else ta - tb
            pairs.append(row)
        out["S+"][scene] = {"permitted_both_faces": len(permitted), "completed_on": len(completed),
                            "completed_off": sum(1 for r in off.values() if r.get("completed")),
                            "completed_pairs": sum(1 for p in pairs if p["on_completed"] and p["off_completed"]),
                            "pairs": pairs, "pass": len(permitted) == len(on) == 3 and len(completed) >= 2}
    sg = [r for r in rows if arm_of(r["name"])[0] == "sg"]
    sg_cases = []
    for r in sg:
        d = r.get("deck") or {}
        faces = d.get("faces", [])
        distinct = len({f for f, v, _ in faces if v in ("present", "absent")}) == 2
        sg_cases.append({"name": r["name"], "faces": faces, "record_verdict": d.get("record_verdict"),
                         "permitted": bool(d.get("permitted")), "completed": r.get("completed"),
                         "absent_branch": distinct and d.get("record_verdict") == "absent" and bool(d.get("permitted")),
                         "mission_result": r.get("mission_result")})
    out["S+g"] = {"runs": sg_cases, "pass": len(sg_cases) == 3 and all(c["absent_branch"] for c in sg_cases)
                  and sum(1 for c in sg_cases if c["completed"]) >= 2}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("batch", type=Path)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args(argv)
    names = [json.loads(line)["name"] for line in (a.batch / "RUNS.jsonl").read_text().splitlines() if line.strip()]
    rows = [run_row(a.batch / n) for n in names]
    verdict = judge(rows)
    report = {"rows": rows, "verdict": verdict}
    text = json.dumps(report, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    print(json.dumps(verdict, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
