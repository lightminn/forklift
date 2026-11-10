"""Real-time plan 5.27 G3-1c stage check (5.20 pairing; plan 5.26 amendment for the order and the reading): line_m far K
f 1.0 alone (r1c_session.sh, the confirm scenario), the shadow switch FORKLIFT_SHADOW_KERNEL off and on, 3 pairs in the
same seeded balanced order as the 1b check (g3_1b_loop_lists.order, seed 527). Both arms carry the same grid switch:
FORKLIFT_GRID_CLEAR_KERNEL=1 when 1b passed its stage check, 0 when it did not (5.27: the 1b state at that time). INSTR
off, seed 0, no dumps. Names g3cl_<g1|g0>_<nn>_<off|on>.

    python g3_1c_loop_lists.py SNAP_DIR OUT_DIR --grid {0,1} [--pairs 3] [--seed 527] [--first 1] > OUT_DIR.jsonl
"""

import importlib.util
import json
import sys
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "g3_1b_loop_lists", Path(__file__).resolve().parents[1] / "2026-11-10-g3-1b-loop" / "g3_1b_loop_lists.py")
_b = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_b)
WRAPPER = _b.WRAPPER
BASE = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "SEED": "0", "INSTR": "off"}


def entries(snap: str, out_root: str, grid: str, pairs: int = 3, seed: int = 527, first: int = 1) -> list[dict]:
    if grid not in ("0", "1"):
        raise ValueError("--grid is 0 or 1 (the 1b stage check's verdict)")
    out = []
    for n, (p, arm) in enumerate(_b.order(pairs, seed), start=1):
        if p < first:
            continue
        name = f"g3cl_g{grid}_{n:02d}_{arm}"
        env = {**BASE, "FORKLIFT_SHADOW_KERNEL": "1" if arm == "on" else "0", "FORKLIFT_GRID_CLEAR_KERNEL": grid,
               "PAIR": str(p)}
        run = f"{out_root}/{name}"
        out.append({"name": name, "argv": ["bash", f"{snap}/{WRAPPER}", run], "env": env, "cwd": snap,
                    "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
                    "first_record": f"{run}/live/epoch_1/run/scenario.json",
                    "scan": [f"{run}/live/epoch_1/run.log"]})
    return out


def main() -> int:
    a = sys.argv[1:]
    snap, out_root = a[0], a[1].rstrip("/")
    grid = a[a.index("--grid") + 1]
    opt = {k: int(a[a.index(k) + 1]) for k in ("--pairs", "--seed", "--first") if k in a}
    for e in entries(snap, out_root, grid, opt.get("--pairs", 3), opt.get("--seed", 527), opt.get("--first", 1)):
        print(json.dumps(e))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
