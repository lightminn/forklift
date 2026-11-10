"""Real-time plan 5.23 G2-1a: the Isaac check's bundle list -- 12 single sessions (--slots 1), the capture's diagnostic
files written sync or async, alternating:

- hall A6P (rt_session.sh: f 1.0, catch-up 6, phase 6, tick trace) sync and async 3 times each, the option passed as
  MORE="--live-capture-dump <mode>";
- line_m far K f 0.4 (r1c_session.sh, the confirm scenario) sync and async 3 times each, as CAPTURE_DUMP=<mode>.

The order alternates the arms and flips which comes first in the second round, so neither arm always runs first:
round 1 H sync, H async, F sync, F async; round 2 H async, H sync, F async, F sync; round 3 as round 1. Every session
gets the mode explicitly (sync too), INSTR off, seed 0. Names g2a_<nn>_<H|F>_<mode>.

    python g2_1a_lists.py SNAP_DIR OUT_DIR > OUT_DIR.jsonl
"""

import json
import sys

HALL = ("docs/validation/2026-11-08-realtime-baseline/rt_session.sh",
        {"LEVEL": "off", "FACTOR": "1.0", "CATCHUP": "6", "PHASE": "6", "TRACE": "1", "SEED": "0", "INSTR": "off"})
LINE_M = ("docs/validation/2026-11-08-r1-confirmation/r1c_session.sh",
          {"SET": "far", "ARM": "K", "FACTOR": "0.4", "SEED": "0", "INSTR": "off"})
ROUNDS = ((("H", "sync"), ("H", "async"), ("F", "sync"), ("F", "async")),
          (("H", "async"), ("H", "sync"), ("F", "async"), ("F", "sync")),
          (("H", "sync"), ("H", "async"), ("F", "sync"), ("F", "async")))


def entries(snap: str, out_root: str) -> list[dict]:
    out = []
    n = 0
    for rnd in ROUNDS:
        for scene, mode in rnd:
            n += 1
            name = f"g2a_{n:02d}_{scene}_{mode}"
            wrapper, env = HALL if scene == "H" else LINE_M
            env = dict(env)
            if scene == "H":
                env["MORE"] = f"--live-capture-dump {mode}"
            else:
                env["CAPTURE_DUMP"] = mode
            run = f"{out_root}/{name}"
            out.append({"name": name, "argv": ["bash", f"{snap}/{wrapper}", run], "env": env, "cwd": snap,
                        "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
                        "first_record": f"{run}/live/epoch_1/run/scenario.json",
                        "scan": [f"{run}/live/epoch_1/run.log"]})
    return out


def main() -> int:
    snap, out_root = sys.argv[1], sys.argv[2].rstrip("/")
    for e in entries(snap, out_root):
        print(json.dumps(e))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
