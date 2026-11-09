"""Real-time plan 5.17: the R1 confirmation run's bundle list in the registered order (one single job, --slots 1).

    python r1c_lists.py SNAP_DIR R1C_DIR > R1C_DIR.jsonl
    python r1c_lists.py SNAP_DIR R1C_DIR --sets mark,wall > R1C_DIR.jsonl   # only those sets, numbers kept

(plan 5.25: the mark and wall sets again, after ws1's GPU persistence mode was turned on -- r1c2 ran them under the
driver stalls of 2026-11-09, docs/validation/2026-11-09-g2-measure-results.md section 4)

S-far 0.4 (L K, K L, L K), S-off 0.4 (L KP, KP L), S-far 0.3 (L K, K L, L K), S-mark (L 0.4, K 0.4, KP 0.4, L 0.3,
K 0.3), S-wall (L, K, KP at 0.4). Each entry runs r1c_session.sh with SET, ARM and FACTOR in its env.
"""

import json
import sys

ORDER = ([("far", a, "0.4") for a in ("L", "K", "K", "L", "L", "K")]
         + [("off", a, "0.4") for a in ("L", "KP", "KP", "L")]
         + [("far", a, "0.3") for a in ("L", "K", "K", "L", "L", "K")]
         + [("mark", "L", "0.4"), ("mark", "K", "0.4"), ("mark", "KP", "0.4"), ("mark", "L", "0.3"), ("mark", "K", "0.3")]
         + [("wall", a, "0.4") for a in ("L", "K", "KP")])


def main() -> int:
    snap, r1c = sys.argv[1], sys.argv[2].rstrip("/")
    sets = set(sys.argv[sys.argv.index("--sets") + 1].split(",")) if "--sets" in sys.argv else None
    wrapper = f"{snap}/docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
    for n, (set_, arm, factor) in enumerate(ORDER, 1):
        if sets is not None and set_ not in sets:
            continue
        name = f"r1c_{n:02d}_{set_}_{arm}_{factor.replace('.', '')}"
        out = f"{r1c}/{name}"
        print(json.dumps({"name": name, "argv": ["bash", wrapper, out],
                          "env": {"SET": set_, "ARM": arm, "FACTOR": factor, "SEED": "0"}, "cwd": snap,
                          "retry_move": [out], "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl",
                          "first_record": f"{out}/live/epoch_1/run/scenario.json",
                          "scan": [f"{out}/live/epoch_1/run.log"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
