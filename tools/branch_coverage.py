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
    "observation candidate switch / re-observation": [r"^state\.observation_attempts", r"candidate", r"reobserv"],
    "near correction accepted": [r"^near\.events\[event=correction_trigger", r"^near\.corrections"],
    "near correction refused": [r"refus", r"reject"],
    "observation lost": [r"lost", r"drop"],
    "re-capture / realignment": [r"realign", r"recapture", r"near_capture"],
    "D7 reapproach": [r"reapproach", r"docking_retries", r"release_docking_retry"],
    "cusp replan / heading hold": [r"cusp"],
    "stall replan": [r"stall"],
    "obstacle stop": [r"obstacle\.(events|stops)", r"event=stop", r"unpermitted"],
    "obstacle backoff": [r"backoff"],
    "obstacle escape": [r"escape"],
    "replan failure": [r"status=(?!success)", r"replan.*fail"],
    "certified withdrawal": [r"to=withdraw", r"withdraw"],
    "docking": [r"plans\[kind=docking", r"to=insert"],
    "return home": [r"to=return_home", r"live_plans\[phase=return_home"],
    "mission refusal": [r"^exit:"],
}


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
