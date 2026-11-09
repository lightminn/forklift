"""Real-time plan 5.26 G3-1b stage check (5.20 pairing): line_m far K f 1.0 alone (r1c_session.sh, the confirm scenario),
the grid switch FORKLIFT_GRID_CLEAR_KERNEL off and on, 3 pairs, the order inside each pair random with a fixed seed
and balanced (see ``order``).
Every session names both switches explicitly (FORKLIFT_SHADOW_KERNEL off: 1c is not in this check), INSTR off, seed 0,
no dumps. Names g3bl_<nn>_<off|on>.

    python g3_1b_loop_lists.py SNAP_DIR OUT_DIR [--pairs 3] [--seed 526] > OUT_DIR.jsonl
    (--pairs 6 --first 4 lists the three more pairs 5.20 asks for when the pairs disagree: g3bl_07 ...)
"""

import json
import random
import sys

WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
BASE = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "SEED": "0", "INSTR": "off"}


def order(pairs: int, seed: int) -> list[tuple[int, str]]:
    """Each pair's first arm, random but balanced (decided before any result): the pairs are taken in blocks of two
    holding one off-first and one on-first pair, the seed shuffling each block (and an odd last pair's arm), so the
    arm never coincides with the order for all pairs -- an unconstrained draw put on first in all three pairs."""
    rng = random.Random(seed)
    firsts = []
    while len(firsts) < pairs:
        block = ["off", "on"]
        rng.shuffle(block)
        firsts += block
    out = []
    for p, f in enumerate(firsts[:pairs], start=1):
        out += [(p, f), (p, "on" if f == "off" else "off")]
    return out


def entries(snap: str, out_root: str, pairs: int = 3, seed: int = 526, first: int = 1) -> list[dict]:
    out = []
    for n, (p, arm) in enumerate(order(pairs, seed), start=1):
        if p < first:
            continue
        name = f"g3bl_{n:02d}_{arm}"
        env = {**BASE, "FORKLIFT_GRID_CLEAR_KERNEL": "1" if arm == "on" else "0", "FORKLIFT_SHADOW_KERNEL": "0",
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
    opt = {k: int(a[a.index(k) + 1]) for k in ("--pairs", "--seed", "--first") if k in a}
    for e in entries(snap, out_root, opt.get("--pairs", 3), opt.get("--seed", 526), opt.get("--first", 1)):
        print(json.dumps(e))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
