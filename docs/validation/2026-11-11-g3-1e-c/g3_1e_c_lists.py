"""1e (c) core-only direct comparison (docs/plans/2026-11-11-g3-1e-core-only.md): four single-session jobs in the
order A B B A on one snapshot. A: one physical core for the session (job CPU 4: the bundle's --bridge-cpus auto sets one
core apart for the SLAM bridge); B: two (job CPU 6). Both arms 1e off (no LAYER_WORKER), line_m far K f 1.0 confirm
(r1c_session.sh), seed 0, 1b 1c 1d on (explicit), INSTR off; RETURN_START / NEAR_END_STAND are the launcher's defaults
in both arms (not set here; the analyzer checks the recorded values agree). Pairs: (A 1, B 1) and (B 2, A 2).

    python g3_1e_c_lists.py SNAP_DIR OUT_ROOT     # writes OUT_ROOT/<name>.jsonl, prints "name arm pair cpus list"
"""

import json
import sys
from pathlib import Path

WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
BASE = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "SEED": "0", "INSTR": "off", "FORKLIFT_GRID_CLEAR_KERNEL": "1",
        "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": "fast"}
ORDER = (("A", 1, 4), ("B", 1, 6), ("B", 2, 6), ("A", 2, 4))


def jobs(snap: str, out_root: str) -> list[dict]:
    out = []
    for n, (arm, pair, cpus) in enumerate(ORDER, start=1):
        name = f"g3cc_{n}_{arm}"
        run = f"{out_root}/{name}/{name}"
        entry = {"name": name, "argv": ["bash", f"{snap}/{WRAPPER}", run],
                 "env": {**BASE, "ARM_NAME": arm, "PAIR": str(pair), "CORES": "1" if arm == "A" else "2"},
                 "cwd": snap, "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
                 "first_record": f"{run}/live/epoch_1/run/scenario.json", "scan": [f"{run}/live/epoch_1/run.log"]}
        out.append({"name": name, "arm": arm, "pair": pair, "cpus": cpus, "out": f"{out_root}/{name}",
                    "entries": [entry]})
    return out


def main_args(argv: list) -> tuple[str, str]:
    if len(argv) != 2:
        raise SystemExit("usage: g3_1e_c_lists.py SNAP_DIR OUT_ROOT")
    return argv[0], argv[1].rstrip("/")


def main() -> int:
    snap, out_root = main_args(sys.argv[1:])
    Path(out_root).mkdir(parents=True, exist_ok=True)
    for j in jobs(snap, out_root):
        f = Path(out_root) / f"{j['name']}.jsonl"
        f.write_text(json.dumps(j["entries"][0]) + "\n")
        print(j["name"], j["arm"], j["pair"], j["cpus"], f)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
