"""The lead's second B (revision plan 9.16, decided 2026-11-07), fixed before it runs. Two parts, reported apart:

- 032 under full and half, RELOC_PROBE reverse (the repeat of b_starts.jsonl's 032);
- seed 4's operator starts, RELOC_PROBE reverse_place: the B-P1b seed 4 starts (b-p1b/val3_rows.jsonl, valid) whose
  scan left the matcher unconfirmed with a pickable candidate (endpoint fraction >= E_MIN 0.80, refinement moved less
  than 0.3 m and 0.1 rad: start_reloc's pool rule), i.e. the starts the live console would show as awaiting_pick on
  that scan set (a proxy: the live session takes up to three scan sets); kept when the straight corridor behind the
  footprint is clear of occupied survey cells for 3 m (tools/unconfirmed_cost.py rear_clear, rear_m 3.0, the survey
  grid: obstacles only, the pallet zones not in it); shuffled with seed 20261108, the first six kept, the pressing rule
  full and half in turn. The click is the click model's (CLICK_SEED 0, CLICK_INDEX the row's index in this part),
  never the truth's.

    python docs/validation/assets/unconfirmed/make_b2_list.py --grid <bp1b_survey/hall_s4_G0/bundle>/grid.npz \
        --rows docs/validation/assets/b-p1b/val3_rows.jsonl > docs/validation/assets/unconfirmed/b2_starts.jsonl

  (map_id 9a5cefd5...; on this grid the pool is 13, 5 clear 3 m behind, so 5 are kept, not 6)
"""

import argparse
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

import unconfirmed_cost as U  # noqa: E402

from forklift_core.localization.slam_map import Grid  # noqa: E402

SEED, KEEP, REAR_M, E_MIN, SEP_M, SEP_RAD = 20261108, 6, 3.0, 0.80, 0.3, 0.1
RULES = ("full", "half")


def pickable(c) -> bool:
    lp, p = c["lattice_pose"], c["pose"]
    d = math.atan2(math.sin(p[2] - lp[2]), math.cos(p[2] - lp[2]))
    return c["endpoint_fraction"] >= E_MIN and math.hypot(p[0] - lp[0], p[1] - lp[1]) < SEP_M and abs(d) < SEP_RAD


def operator_starts(rows, grid) -> tuple:
    pool = sorted((r for r in rows if r["valid"] and r["decision"] != "confirmed"
                   and any(pickable(c) for c in r.get("candidates") or ())), key=lambda r: r["id"])
    order = list(pool)
    random.Random(SEED).shuffle(order)
    seen = [(r["id"], r["decision"], U.rear_clear(grid, r["truth_base"], REAR_M)) for r in order]
    kept = [r for r, (_, _, clear) in zip(order, seen) if clear][:KEEP]
    return kept, seen


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", type=Path, required=True)
    ap.add_argument("--rows", type=Path, required=True)
    ap.add_argument("--natural", type=Path, default=Path(__file__).resolve().parent / "s4_starts_rows.jsonl")
    a = ap.parse_args()
    grid = Grid.from_npz(a.grid)
    rows = [json.loads(line) for line in a.rows.read_text().splitlines() if line.strip()]
    natural = {r["id"]: r for r in map(json.loads, a.natural.read_text().splitlines())}
    out = [{"id": "val3_032", "part": "032", "start_pose": natural["val3_032"]["truth"], "reverse_kind": k,
            "probe": "reverse"} for k in RULES]
    kept, seen = operator_starts(rows, grid)
    out += [{"id": r["id"], "part": "operator", "start_pose": r["truth_base"], "reverse_kind": RULES[k % 2],
             "probe": "reverse_place", "click_seed": 0, "click_index": k, "b_p1b_decision": r["decision"]}
            for k, r in enumerate(kept)]
    sys.stdout.write("".join(json.dumps(r) + "\n" for r in out))
    sys.stderr.write(json.dumps({"pool": len(seen), "clear": sum(c for *_, c in seen), "kept": len(kept),
                                 "order": seen}) + "\n")


if __name__ == "__main__":
    main()
