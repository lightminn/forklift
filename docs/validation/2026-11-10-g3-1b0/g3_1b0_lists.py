"""Real-time plan 5.26 1b-0: the dump run's bundle list -- one line_m far session (r1c_session.sh, K, f 0.4, the
confirm scenario) and one hall session (rt_session.sh, A6P: f 1.0, catch-up 6, phase 6), both with the grid switch off
(the original _clear_group) and FORKLIFT_PERF_DUMP into the session's own folder. Not a measurement run: the dumps'
copies slow the loop; nothing here is timed.

Sampling (written down, plan 5.26 / Codex P2-4): grid_snapshot every 5th call and shadow_apply every 10th (the later 1c
replay uses them), every other dumped function every 50th; the first 5 calls of each stratum (FORKLIFT_PERF_DUMP_STRATA_K
default). One job, --slots 1.

    python g3_1b0_lists.py SNAP_DIR OUT_DIR > OUT_DIR.jsonl
"""

import json
import sys

EVERY = "grid_snapshot=5,shadow_apply=10"


def main() -> int:
    snap, out_root = sys.argv[1], sys.argv[2].rstrip("/")
    for name, wrapper, env in (
            ("g3b0_F", "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh",
             {"SET": "far", "ARM": "K", "FACTOR": "0.4", "SEED": "0", "INSTR": "off"}),
            ("g3b0_H", "docs/validation/2026-11-08-realtime-baseline/rt_session.sh",
             {"LEVEL": "off", "FACTOR": "1.0", "CATCHUP": "6", "PHASE": "6", "TRACE": "1", "SEED": "0", "INSTR": "off"})):
        out = f"{out_root}/{name}"
        env = {**env, "FORKLIFT_PERF_DUMP": f"{out}/perf_dump", "FORKLIFT_PERF_DUMP_EVERY": EVERY}
        print(json.dumps({"name": name, "argv": ["bash", f"{snap}/{wrapper}", out], "env": env, "cwd": snap,
                          "retry_move": [out], "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl",
                          "first_record": f"{out}/live/epoch_1/run/scenario.json",
                          "scan": [f"{out}/live/epoch_1/run.log"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
