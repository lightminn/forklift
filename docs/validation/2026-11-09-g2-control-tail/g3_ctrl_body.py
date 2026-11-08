"""G3 counterfactual split (Codex 5.18/5.19 review P2-4): pacer_replay's ctrl0.5 halves every control tick, the long
tail included, so its ok cannot be read as "the control mean alone". This replays phase 6 + scan 0.5 with the control
tick halved only where it is <= 50 ms (tail kept), next to the all-ticks case and the tail-flattened-only case.

usage: g3_ctrl_body.py SNAP ANALYSIS_JSON TRACE_ROOT RUN [RUN ...]   (same arguments as pacer_replay.py)
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "2026-11-08-realtime-baseline"))
import pacer_replay as pr  # noqa: E402  (reads SNAP from argv[1])
import json  # noqa: E402

TAIL = 0.05


def body_ctrl(r, w, factor):
    c = np.nan_to_num(r["parts"]["control"])
    return w - np.where(c <= TAIL, (1 - factor) * c, 0.0)


def main() -> int:
    analysis = {x["name"]: x for x in json.loads(Path(sys.argv[2]).read_text())}
    root = Path(sys.argv[3])
    for name in sys.argv[4:]:
        r = pr.load(root, analysis[name])
        c = np.nan_to_num(r["parts"]["control"])
        print(f"{name}: control ticks > {TAIL*1e3:.0f} ms {int((c > TAIL).sum())}, their share of control time "
              f"{c[c > TAIL].sum() / c.sum():.3f}")
        cases = []
        w, tse = pr.transformed(r, split=True)
        cases.append(("split", w, tse))
        w, tse = pr.transformed(r, split=True, scan=0.5)
        cases.append(("split+scan0.5", w, tse))
        cases.append(("split+scan0.5+ctrl_body0.5 (tail kept)", body_ctrl(r, w, 0.5), tse))
        cases.append(("split+scan0.5+tail_flat(<=50ms)", w - np.where(c > TAIL, c - TAIL, 0.0), tse))
        w2, tse2 = pr.transformed(r, split=True, scan=0.5, ctrl=0.5)
        cases.append(("split+scan0.5+ctrl0.5 (all ticks)", w2, tse2))
        for label, w, tse in cases:
            ratio, ok = pr.replay(w, r["eps"], 6, tse, with_ok=True)
            print(f"   {label:42s} ratio {ratio:.3f} ok {ok if ok is None else round(ok, 3)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
