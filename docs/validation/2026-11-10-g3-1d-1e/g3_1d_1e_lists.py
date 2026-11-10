"""Real-time plan 5.26: the G3-1d stage check's list and the 1e judgement run's two lists, line_m far K f 1.0 alone
(r1c_session.sh, the confirm scenario), INSTR off, seed 0, no dumps, pairs in the seeded balanced order of the 1b check
(g3_1b_loop_lists.order).

- 1d: the LiDAR casting --lidar-cast loop and fast (LIDAR_CAST), both arms on 1b and 1c (FORKLIFT_GRID_CLEAR_KERNEL=1,
  FORKLIFT_SHADOW_KERNEL=1, both passed their stage checks), 3 pairs, seed 529. Names g3dl_<nn>_<loop|fast>.
- 1e: the baseline (all three off: grid 0, shadow 0, loop) and the candidate (grid 1, shadow 1, and the LiDAR casting
  fast when 1d passed its stage check, loop when it did not), 3 pairs, seed 530 (the 5.26 rule's extension: --pairs 6
  --first 4, keeping the first three). Names g3el_<f|l>_<nn>_<base|cand>.

    python g3_1d_1e_lists.py 1d SNAP_DIR OUT_DIR [--pairs 3] [--first 1] > OUT_DIR.jsonl
    python g3_1d_1e_lists.py 1e SNAP_DIR OUT_DIR --lidar {fast,loop} [--pairs 3] [--first 1] > OUT_DIR.jsonl
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
SEED = {"1d": 529, "1e": 530}


def arm_env(kind: str, arm: str, lidar: str | None = None) -> dict:
    if kind == "1d":
        return {"FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": arm}
    if arm == "base":
        return {"FORKLIFT_GRID_CLEAR_KERNEL": "0", "FORKLIFT_SHADOW_KERNEL": "0", "LIDAR_CAST": "loop"}
    return {"FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": lidar}


def entries(kind: str, snap: str, out_root: str, lidar: str | None = None, pairs: int = 3, first: int = 1) -> list:
    if kind not in SEED:
        raise ValueError("kind is 1d or 1e")
    if kind == "1e" and lidar not in ("fast", "loop"):
        raise ValueError("1e needs --lidar fast (1d passed) or loop (it did not)")
    names = ("loop", "fast") if kind == "1d" else ("base", "cand")
    out = []
    for n, (p, a) in enumerate(_b.order(pairs, SEED[kind]), start=1):
        if p < first:
            continue
        arm = names[0] if a == "off" else names[1]
        name = f"g3dl_{n:02d}_{arm}" if kind == "1d" else f"g3el_{lidar[0]}_{n:02d}_{arm}"
        env = {**BASE, **arm_env(kind, arm, lidar), "ARM_NAME": arm, "PAIR": str(p)}
        run = f"{out_root}/{name}"
        out.append({"name": name, "argv": ["bash", f"{snap}/{WRAPPER}", run], "env": env, "cwd": snap,
                    "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
                    "first_record": f"{run}/live/epoch_1/run/scenario.json",
                    "scan": [f"{run}/live/epoch_1/run.log"]})
    return out


def main() -> int:
    a = sys.argv[1:]
    kind, snap, out_root = a[0], a[1], a[2].rstrip("/")
    lidar = a[a.index("--lidar") + 1] if "--lidar" in a else None
    opt = {k: int(a[a.index(k) + 1]) for k in ("--pairs", "--first") if k in a}
    for e in entries(kind, snap, out_root, lidar, opt.get("--pairs", 3), opt.get("--first", 1)):
        print(json.dumps(e))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
