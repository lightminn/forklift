"""Real-time plan 5.28 1e-0: the prototype measurement's list -- two single sessions, line_m far K f 1.0
(r1c_session.sh, the confirm scenario), 1b, 1c and 1d on (FORKLIFT_GRID_CLEAR_KERNEL=1, FORKLIFT_SHADOW_KERNEL=1,
LIDAR_CAST=fast: the operating default after #405), the layer worker delay then thread, k = 3 (the plan's proposal;
the slack to the control start of the scan tick + 1..4 is recorded either way), INSTR off, seed 0, no dumps.
Names g3e0_<nn>_<delay|thread>.

The re-measurement after the 1e-0 fix (docs/validation/2026-11-10-g3-1e0-run.md) adds ``--control``: a third session,
g3e0_03_off, the same switches without a worker (no LAYER_WORKER: r1c_session adds no --layer-worker), report only.

    python g3_1e0_lists.py SNAP_DIR OUT_DIR [--control] > OUT_DIR.jsonl
"""

import json
import sys

WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
BASE = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "SEED": "0", "INSTR": "off", "FORKLIFT_GRID_CLEAR_KERNEL": "1",
        "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": "fast", "LAYER_WORKER_K": "3"}
ORDER = ("delay", "thread")


def entries(snap: str, out_root: str, control: bool = False) -> list[dict]:
    out = []
    for n, mode in enumerate(ORDER + (("off",) if control else ()), start=1):
        name = f"g3e0_{n:02d}_{mode}"
        run = f"{out_root}/{name}"
        env = {**BASE, "ARM_NAME": mode} if mode == "off" else {**BASE, "LAYER_WORKER": mode, "ARM_NAME": mode}
        out.append({"name": name, "argv": ["bash", f"{snap}/{WRAPPER}", run],
                    "env": env, "cwd": snap, "retry_move": [run],
                    "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
                    "first_record": f"{run}/live/epoch_1/run/scenario.json",
                    "scan": [f"{run}/live/epoch_1/run.log"]})
    return out


def main() -> int:
    snap, out_root = sys.argv[1], sys.argv[2].rstrip("/")
    for e in entries(snap, out_root, control="--control" in sys.argv[3:]):
        print(json.dumps(e))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
