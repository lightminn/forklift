"""The branches a set of format-2 boundary records took, against the D2 required list (M-a1 replay plan R5/R6).

Each tick of a record lists the branches the core took in it (the record lists of its dictionaries that grew,
tagged with the new item's identifying fields). This collects them across runs (time stamps dropped), and checks
each D2 item -- a name and the branch-id patterns that show it was taken -- against them. An item no run shows
stays "not covered"; the report names the runs that showed each covered one.

    python tools/branch_coverage.py RUN_DIR [RUN_DIR ...] [--json OUT]
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import sys
from pathlib import Path

# D2 required branches (docs/plans/2026-10-15-m-a-mission-core.md, D2 "분기 통과 목록") as branch-id patterns.
D2_ITEMS = {
    "observation candidate switch / re-observation": [r"^state\.observation_candidates", r"^state\.observe_replans",
                                                      r"^state\.transitions\[from=approach,to=observe"],
    "near correction accepted": [r"^evidence:correction=accepted"],
    "near correction refused": [r"^evidence:correction=refused"],
    "observation lost": [r"^near\.events\[event=stop,reason=lost", r"^near\.events\[event=drop_latch"],
    "near capture redraw / pickup retry": [r"^state\.near_capture_history", r"event=retry_start"],
    "realignment": [r"^evidence:realign"],
    "D7 reapproach": [r"^state\.docking_retries", r"release_docking_retry"],
    "cusp replan / heading hold": [r"^state\.cusp_(heading_holds|replans|stalls)"],
    "stall replan": [r"^state\.stall_replans", r"release_before_stall_replan"],
    "obstacle stop": [r"^obstacle\.backoff_records", r"^obstacle\.unpermitted"],
    "obstacle backoff": [r"^obstacle\.backoff_records", r"release_after_backoff"],
    "obstacle escape": [r"^evidence:escape", r"^obstacle\.replans\[.*status=escape"],
    "replan failure": [r"^obstacle\.replans\[.*status=(?!success|escape)", r"^exit:obstacle_replan_failed"],
    "certified withdrawal": [r"^evidence:withdraw=certified"],
    "docking": [r"^obstacle\.plans\[kind=docking", r"^state\.transitions\[from=approach,to=insert"],
    "return home": [r"^state\.transitions\[from=settle,to=return_home"],
    # The core's own refusals (MissionFailed reasons); an evaluation invariant (an "Actual ..." truth check) or any
    # other exception is listed as an exit, not counted here (Codex P2).
    "mission refusal": [r"^exit:(obstacle_|near_field_|Mission planning failed|withdraw_not_certified|docking_"
                        r"|pickup_|observation_|mission_|observe_|transport_|return_)"],
}


def _evidence(run_dir: Path) -> list:
    """Branches the run's own records state as values (the tick probe sees only list growth): the near corrections'
    redraw outcome, a realignment record, an obstacle escape, the withdrawal certificate (Codex P2: from these
    decision fields, never from a key name alone)."""
    out = []
    near_file = Path(run_dir) / "near_tracking.json"
    if near_file.exists():
        near = json.loads(near_file.read_text())
        for c in near.get("corrections") or []:
            accepted = (c or {}).get("accepted")  # True accepted, False refused, None still pending (not counted)
            if accepted is True:
                out.append("evidence:correction=accepted")
            elif accepted is False:
                out.append("evidence:correction=refused")
        if near.get("realign"):
            out.append("evidence:realign")
    result_file = Path(run_dir) / "result.json"
    if result_file.exists():
        result = json.loads(result_file.read_text())
        if (result.get("near_tracking") or {}).get("realign"):
            out.append("evidence:realign")
        for rec in ((result.get("obstacle_layer") or {}).get("replans") or []):  # an escape installed (Codex P2)
            if isinstance(rec, dict) and rec.get("status") == "escape":
                out.append("evidence:escape")
        for rec in ((result.get("obstacle_layer") or {}).get("backoff_records") or []):
            if isinstance(rec, dict) and rec.get("escape") and (rec["escape"] if not isinstance(rec["escape"], dict)
                                                                 else rec["escape"].get("chosen") is not None):
                out.append("evidence:escape")
        cert = result.get("withdraw_certificate")
        if isinstance(cert, dict):
            out.append("evidence:withdraw=" + ("certified" if cert.get("certified") else "not_certified"))
    return out


def run_branches(run_dir: Path) -> collections.Counter:
    counts: collections.Counter = collections.Counter()
    boundary = Path(run_dir) / "boundary"
    for path in sorted(boundary.glob("boundary_*.jsonl")):
        with path.open() as fh:
            for line in fh:
                if not line.strip():
                    continue
                tick = json.loads(line)
                for branch in tick.get("branches", []):
                    counts[re.sub(r",?time_s=[^,\]]*", "", branch)] += 1
    counts.update(_evidence(run_dir))
    meta = json.loads((boundary / "boundary_meta.json").read_text())
    ending = meta.get("ending") or {}
    if ending.get("kind") == "exception":  # a mission refusal ends the run with its reason
        counts[f"exit:{ending.get('message', '')[:80]}"] += 1
    return counts


def coverage(run_dirs: list[Path]) -> dict:
    per_run = {str(r): run_branches(r) for r in run_dirs}
    union: collections.Counter = collections.Counter()
    for c in per_run.values():
        union.update(c)
    items = {}
    for item, patterns in D2_ITEMS.items():
        shown = {run: sorted(b for b in c if any(re.search(p, b) for p in patterns)) for run, c in per_run.items()}
        shown = {run: b for run, b in shown.items() if b}
        items[item] = {"covered": bool(shown), "runs": {run: b[:6] for run, b in shown.items()}}
    return {"items": items, "not_covered": [k for k, v in items.items() if not v["covered"]],
            "branches": dict(sorted(union.items()))}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    out = coverage(args.runs)
    text = json.dumps(out, indent=1)
    if args.json:
        args.json.write_text(text + "\n")
    for item, v in out["items"].items():
        print(("covered    " if v["covered"] else "NOT COVERED"), item, "<-", ", ".join(Path(r).parts[-3] if len(Path(r).parts) > 2 else r for r in v["runs"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
