"""Real-time plan 5.24: the G5-0 arms' bundle list in the registered order (one job, --slots 1): a set-F warm-up, then
two blocks of the five arms (seed 5240): P1 P2 P0 O W | P2 P0 W O P1. Every session is line_m far, K, f 0.4, seed 0,
r1c_session.sh's confirm scenario, INSTR off.

    python g5_0_lists.py SNAP_DIR OUT_DIR > OUT_DIR.jsonl
"""

import json
import sys

ORDER = ["warm", "P1", "P2", "P0", "O", "W", "P2", "P0", "W", "O", "P1"]
ARM_ENV = {"warm": {}, "P0": {}, "P1": {"PREP_RENDERS": "1"}, "P2": {"PREP_RENDERS": "2"}, "W": {"PREP_WAIT_S": "3.5"},
           "O": {"PROPOSALS": "oracle"}}


def main() -> int:
    snap, out_root = sys.argv[1], sys.argv[2].rstrip("/")
    for n, arm in enumerate(ORDER):
        name = f"g50_{n:02d}_{arm}"
        out = f"{out_root}/{name}"
        env = {"SET": "far", "ARM": "K", "FACTOR": "0.4", "SEED": "0", "INSTR": "off", "LIFECYCLE": "on", **ARM_ENV[arm]}
        print(json.dumps({"name": name, "argv": ["bash", f"{snap}/docs/validation/2026-11-08-r1-confirmation/r1c_session.sh", out],
                          "env": env, "cwd": snap, "retry_move": [out],
                          "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl",
                          "first_record": f"{out}/live/epoch_1/run/scenario.json",
                          "scan": [f"{out}/live/epoch_1/run.log"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
