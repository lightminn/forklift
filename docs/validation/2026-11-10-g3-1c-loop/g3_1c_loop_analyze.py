"""Real-time plan 5.27 G3-1c stage check, fixed before the results: g3_1b_loop_analyze.py's reading (validity, AUTO
scan-step share, AUTO rate, missions and safety, the pass / extend to 6 / fail lines, the list check) with the arm on
FORKLIFT_SHADOW_KERNEL and the grid switch fixed across all sessions at the value the list names (the 1b verdict; plan
5.26 amendment: 1c is read the same way as 1b). The per-pair threshold is half of 1c's registered prediction, about
-30 ms/sim s from the ws1 event replay (docs/validation/2026-11-10-g3-1c0.md): 0.015 s/sim s.

    PYTHONPATH=SNAP/src python g3_1c_loop_analyze.py OUT_DIR --list OUT_DIR.jsonl [--ps PS.jsonl] [--json out.json]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "g3_1b_loop_analyze", Path(__file__).resolve().parents[1] / "2026-11-10-g3-1b-loop" / "g3_1b_loop_analyze.py")
base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(base)
base.HALF_PREDICTION = 0.015  # s per sim s: half of 1c's about -30 ms/sim s (verdict() reads the module's value)
HALF_PREDICTION = base.HALF_PREDICTION
g2m = base.g2m


class Session(base.Session):
    def __init__(self, out: Path, entry: dict, grid: str, ps=None):
        self.grid = grid
        self.arm = "on" if entry["env"].get("FORKLIFT_SHADOW_KERNEL") == "1" else "off"
        self.pair = int(entry["env"]["PAIR"])
        g2m.Session.__init__(self, out, entry, ps=ps)

    def _check_conditions(self):
        c = self.cond
        for k in c:
            if k.startswith(g2m.FORBIDDEN):
                self.invalid.append(f"{k} set (dumps and the dual run stay out of the check)")
        want = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "FORKLIFT_LIVE_CATCHUP_STEPS": "6", "SEED": "0",
                "FORKLIFT_SHADOW_KERNEL": "1" if self.arm == "on" else "0", "FORKLIFT_GRID_CLEAR_KERNEL": self.grid}
        for k, v in want.items():
            if c.get(k) != v:
                self.invalid.append(f"{k}={c.get(k)}, registered {v}")
        for k in g2m.INSTR_ON:
            if c.get(k) is not None:
                self.invalid.append(f"{k} set in an INSTR off session")


def list_problems(entries: list[dict]) -> list[str]:
    """Pairs 1..3 or 1..6, each one shadow-off and one shadow-on session, one grid value across the list."""
    bad = [e.get("name") for e in entries if e["env"].get("FORKLIFT_SHADOW_KERNEL") not in ("0", "1")]
    if bad:
        return [f"shadow switch not 0 or 1 in {bad}"]
    out = base.list_problems([{"env": {"PAIR": e["env"]["PAIR"],
                                       "FORKLIFT_GRID_CLEAR_KERNEL": e["env"].get("FORKLIFT_SHADOW_KERNEL")}}
                              for e in entries])
    grids = {e["env"].get("FORKLIFT_GRID_CLEAR_KERNEL") for e in entries}
    if len(grids) != 1 or not grids <= {"0", "1"}:
        out.append(f"grid switch values {sorted(map(str, grids))}, registered one of 0 or 1 for every session")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--ps", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    ps = g2m.read_ps(a.ps.read_text()) if a.ps else None
    entries = [json.loads(x) for x in a.list.read_text().splitlines() if x.strip()]
    problems = list_problems(entries)
    if problems:
        print(json.dumps({"list_not_registered": problems}))
        return 2
    grid = entries[0]["env"]["FORKLIFT_GRID_CLEAR_KERNEL"]
    rows = [base.analyse_session(Session(a.out / e["name"], e, grid, ps=ps)) for e in entries]
    out = {"grid": grid, "sessions": rows, "verdict": base.verdict(rows, len({e["env"]["PAIR"] for e in entries}))}
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "invalid", "mission_done", "scan_step_rest_s_per_sim_s")},
                         default=str))
    print(json.dumps(out["verdict"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
