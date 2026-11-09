"""Real-time plan 5.27 1c-0: the event recording's bundle list -- the sessions of the 1b-0 dump run (line_m far: the
r1c_session.sh confirm scenario, K, f 0.4; hall: rt_session.sh A6P, f 1.0, catch-up 6, phase 6), each recording every
ShadowMemory.apply call of its receivers in order (FORKLIFT_PERF_DUMP_STREAM=shadow_apply=1000000) and nothing else
(FORKLIFT_PERF_DUMP_EVERY=0, FORKLIFT_PERF_DUMP_STRATA_K=0), into OUT_DIR_dumps/<name> beside the session's output (the
bundle's preflight refuses an environment naming a retry_move output). The runner writes the completeness marker
(stream_totals_<pid>.json) before app.close(). Both switches (grid, shadow) are left at their default, off. Not a
measurement run: the copies before every dumped call slow the loop; nothing here is timed.

    python g3_1c0_lists.py SNAP_DIR OUT_DIR > OUT_DIR.jsonl
"""

import json
import sys

SESSIONS = (
    ("g3c0_F", "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh",
     {"SET": "far", "ARM": "K", "FACTOR": "0.4", "SEED": "0", "INSTR": "off"}),
    ("g3c0_H", "docs/validation/2026-11-08-realtime-baseline/rt_session.sh",
     {"LEVEL": "off", "FACTOR": "1.0", "CATCHUP": "6", "PHASE": "6", "TRACE": "1", "SEED": "0", "INSTR": "off"}),
)


def main() -> int:
    snap, out_root = sys.argv[1], sys.argv[2].rstrip("/")
    for name, wrapper, env in SESSIONS:
        out = f"{out_root}/{name}"
        env = {**env, "FORKLIFT_PERF_DUMP": f"{out_root}_dumps/{name}", "FORKLIFT_PERF_DUMP_EVERY": "0",
               "FORKLIFT_PERF_DUMP_STRATA_K": "0", "FORKLIFT_PERF_DUMP_STREAM": "shadow_apply=1000000",
               "FORKLIFT_GRID_CLEAR_KERNEL": "0", "FORKLIFT_SHADOW_KERNEL": "0"}  # pinned, not inherited
        print(json.dumps({"name": name, "argv": ["bash", f"{snap}/{wrapper}", out], "env": env, "cwd": snap,
                          "retry_move": [out], "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl",
                          "first_record": f"{out}/live/epoch_1/run/scenario.json",
                          "scan": [f"{out}/live/epoch_1/run.log"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
