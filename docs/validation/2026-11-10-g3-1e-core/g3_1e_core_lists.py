"""1e stage 1 (docs/plans/2026-11-10-g3-1e-process-worker.md 3): the measurement's list -- D T T D, single sessions,
line_m far K f 1.0 (r1c_session.sh, the confirm scenario), 1b, 1c and 1d on, INSTR off, seed 0. D is ``delay``; T is
``thread`` pinned to the session's second core (LAYER_WORKER_CPUS=auto). Pairs: (01 D, 02 T) and (04 D, 03 T).
``--k 4`` writes the supplementary pair's list (the same four at k = 4, names g3ec4_*).

    python g3_1e_core_lists.py SNAP_DIR OUT_DIR [--k 4] > OUT_DIR.jsonl
    sbatch --cpus-per-task=6 ... g3_1e_core_job.sh SNAP PS.jsonl THREADS.jsonl OUT_DIR.jsonl OUT_DIR/bundle live
"""

import json
import sys

WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
BASE = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "SEED": "0", "INSTR": "off", "FORKLIFT_GRID_CLEAR_KERNEL": "1",
        "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": "fast"}
ORDER = (("D", 1), ("T", 1), ("T", 2), ("D", 2))


def entries(snap: str, out_root: str, k: int = 3) -> list[dict]:
    if k not in (3, 4):
        raise ValueError("k is 3 (the run) or 4 (the supplementary pair)")
    tag = "g3ec" if k == 3 else "g3ec4"
    out = []
    for n, (arm, pair) in enumerate(ORDER, start=1):
        name = f"{tag}_{n:02d}_{arm}"
        run = f"{out_root}/{name}"
        env = {**BASE, "LAYER_WORKER": "delay" if arm == "D" else "thread", "LAYER_WORKER_K": str(k),
               "ARM_NAME": arm, "PAIR": str(pair)}
        if arm == "T":
            env["LAYER_WORKER_CPUS"] = "auto"
        out.append({"name": name, "argv": ["bash", f"{snap}/{WRAPPER}", run], "env": env, "cwd": snap,
                    "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
                    "first_record": f"{run}/live/epoch_1/run/scenario.json",
                    "scan": [f"{run}/live/epoch_1/run.log"]})
    return out


def main() -> int:
    snap, out_root = sys.argv[1], sys.argv[2].rstrip("/")
    k = int(sys.argv[sys.argv.index("--k") + 1]) if "--k" in sys.argv else 3
    for e in entries(snap, out_root, k):
        print(json.dumps(e))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
