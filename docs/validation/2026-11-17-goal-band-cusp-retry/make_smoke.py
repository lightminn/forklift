"""Isaac smoke of the riib launcher's GOAL_CREEP_BAND=on default (#442 rule): the launcher as an operator runs it --
non-deterministic, nothing in EXTRA_ARGS, so riib_defaults.txt must read GOAL_CREEP_BAND=on ..._FROM=launcher.

  python make_smoke.py SNAP_NAME OUT_DIR
random_1 s24 (the seed the band was found on) and one run each of line_2 and aisle_rack (scenes the pair never ran),
to see completion and that the band latches nowhere a truck was not standing (goal_creep_stalls band entries).
Read per run: riib_defaults.txt's GOAL_CREEP_BAND line, success, failure_reason, and every goal_creep_stalls entry with
band true (its time, remaining, commanded and odometry); a run with no band entry and a normal arrival is the
"not taken" case the smoke must contain, and any band entry is checked against the truck standing (odometry ~0).
"""
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("pfb_runs", HERE.parent / "2026-11-15-post-fix-baseline" / "make_runs.py")
pfb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pfb)
RUNS = [("random_1", 24), ("line_2", 0), ("aisle_rack", 1)]


def rows(snap: str) -> list:
    root = f"{pfb.P}/test_runs/gbc/{snap}/smoke"
    return [pfb.riib(snap, root, f"gbcSmoke_{sc}_s{s}", sc, s, {"EXTRA_ARGS": ""}) for sc, s in RUNS]


def main(snap: str, out_dir: str) -> None:
    r = rows(snap)
    path = Path(out_dir) / f"RUNS_gbc_smoke_{snap}.jsonl"
    path.write_text("".join(json.dumps(x) + "\n" for x in r))
    print(path, len(r))


if __name__ == "__main__":
    main(*sys.argv[1:3])
