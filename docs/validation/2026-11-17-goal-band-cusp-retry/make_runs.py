"""Run list of the goal band / cusp retry pairs (docs/plans/2026-11-17-goal-band-and-cusp-retry.md, verification):
the post-fix baseline's riib rows (its make_runs.riib), random_1, one snapshot, the arms kept apart.

  python make_runs.py SNAP_NAME OUT_DIR
deterministic: off s24·s11·s49 (both options off), A on s24 (--goal-creep-band on), B on s11·s49 (--cusp-replan-retry
on); replay records (non-deterministic, not for the verdict): A on s24 and B on s11 with --record-boundary format 2.
"""
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("pfb_runs", HERE.parent / "2026-11-15-post-fix-baseline" / "make_runs.py")
pfb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pfb)
P = pfb.P
A_ON, B_ON = "--goal-creep-band on", "--cusp-replan-retry on"
# the riib launcher turns A on by default since the 2026-11-17 decision: every arm names its own value, so a later
# snapshot builds the same arms (the A arm's --goal-creep-band on in EXTRA_ARGS wins over this; Codex review P2)
A_OFF = {"GOAL_CREEP_BAND": "off"}
REC = "--record-boundary --record-boundary-format 2"


def rows(snap: str) -> list:
    root = f"{P}/test_runs/gbc/{snap}"
    out = [pfb.riib(snap, f"{root}/off", f"gbcOff_random_1_s{s}", "random_1", s, A_OFF) for s in (24, 11, 49)]
    out.append(pfb.riib(snap, f"{root}/A", "gbcA_random_1_s24", "random_1", 24,
                        {**A_OFF, "EXTRA_ARGS": f"--deterministic {A_ON}"}))
    out += [pfb.riib(snap, f"{root}/B", f"gbcB_random_1_s{s}", "random_1", s, {**A_OFF, "EXTRA_ARGS": f"--deterministic {B_ON}"})
            for s in (11, 49)]
    out.append(pfb.riib(snap, f"{root}/rec", "gbcArec_random_1_s24", "random_1", 24, {**A_OFF, "EXTRA_ARGS": f"{A_ON} {REC}"}))
    out.append(pfb.riib(snap, f"{root}/rec", "gbcBrec_random_1_s11", "random_1", 11, {**A_OFF, "EXTRA_ARGS": f"{B_ON} {REC}"}))
    return out


def main(snap: str, out_dir: str) -> None:
    r = rows(snap)
    assert len({x["name"] for x in r}) == len(r) == 8
    path = Path(out_dir) / f"RUNS_gbc_{snap}.jsonl"
    path.write_text("".join(json.dumps(x) + "\n" for x in r))
    print(path, len(r))


if __name__ == "__main__":
    main(*sys.argv[1:3])
