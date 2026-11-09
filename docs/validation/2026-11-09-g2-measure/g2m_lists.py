"""Real-time plan 5.22: the G2-0 / G3-1a / G5-0 measurement run's bundle list in the registered order (one single job,
--slots 1). Set H: the factory hall with G1's A6P profile (rt_session.sh, f 1.0, catch-up 6, phase 6); set F: line_m
far with the K profile at 0.4 (r1c_session.sh, the confirm scenario). Each set: INSTR on off on off on; first a set-H
warm-up session with INSTR off (G1's warm run: the node's first session pays the shader and extension caches), not judged.

    python g2m_lists.py SNAP_DIR OUT_DIR > OUT_DIR.jsonl
"""

import json
import sys

ORDER = ([("H", "warm")] + [("H", i) for i in ("on", "off", "on", "off", "on")]
         + [("F", i) for i in ("on", "off", "on", "off", "on")])  # warm: G1's first-session warm-up, not judged


def main() -> int:
    snap, out_root = sys.argv[1], sys.argv[2].rstrip("/")
    for n, (set_, instr) in enumerate(ORDER):
        name = f"g2m_{n:02d}_{set_}_{instr}"
        out = f"{out_root}/{name}"
        if set_ == "H":
            wrapper = f"{snap}/docs/validation/2026-11-08-realtime-baseline/rt_session.sh"
            env = {"LEVEL": "off", "FACTOR": "1.0", "CATCHUP": "6", "PHASE": "6", "TRACE": "1", "SEED": "0",
                   "INSTR": "off" if instr == "warm" else instr}
        else:
            wrapper = f"{snap}/docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
            env = {"SET": "far", "ARM": "K", "FACTOR": "0.4", "SEED": "0", "INSTR": instr}
        print(json.dumps({"name": name, "argv": ["bash", wrapper, out], "env": env, "cwd": snap,
                          "retry_move": [out], "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl",
                          "first_record": f"{out}/live/epoch_1/run/scenario.json",
                          "scan": [f"{out}/live/epoch_1/run.log"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
