#!/usr/bin/env python3
"""F2 plan 4.4: the October plant gates recomputed on the 2x2's M2 runs of one arm -- V2 (tools/plant_gates.v16_probe:
every +-0.36 arc's 0.1 s-block window, body curvature against the steering angles' curvature and the per-sample
left/right check) at 120 and 240 Hz, and V4 = whether every arc that has a window at both dt keeps its V2 verdict.

  PYTHONPATH=<snapshot>/src python tools/f2_v4_report.py --run120 RUN_DIR --run240 RUN_DIR --json OUT.json

"v4_v2_preserved" is true or false only when at least one arc has a window at both dt; otherwise it is null (not a
report: the 2x2 tool then selects no candidate, plan 4.4 with amendment 1).
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def compare(v120: dict, v240: dict) -> dict:
    a = {(e["label"], e["kappa_cmd"]): e for e in v120["v2"]}
    b = {(e["label"], e["kappa_cmd"]): e for e in v240["v2"]}
    rows = []
    for key in sorted(set(a) & set(b)):
        x, y = a[key], b[key]
        both = x.get("window") is not None and y.get("window") is not None
        row = {"label": key[0], "kappa_cmd": key[1], "window_120": x.get("window") is not None,
               "window_240": y.get("window") is not None, "pass_120": x.get("pass"), "pass_240": y.get("pass")}
        if both:
            row["same_verdict"] = x["pass"] == y["pass"]
            for k in ("kappa_true", "kappa_angles"):
                if x.get(k) is not None and y.get(k) is not None:
                    row[f"{k}_diff"] = y[k] - x[k]
        rows.append(row)
    comparable = [r for r in rows if "same_verdict" in r]
    return {"arcs": rows, "comparable": len(comparable),
            "v4_v2_preserved": (all(r["same_verdict"] for r in comparable) if comparable else None)}


def main() -> None:
    import plant_gates

    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--run120", type=Path, required=True)
    p.add_argument("--run240", type=Path, required=True)
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    import f2_2x2

    v120, v240 = plant_gates.v16_probe(args.run120), plant_gates.v16_probe(args.run240)
    # the two inputs' identity, for the 2x2 tool to bind the report to the candidate's own first M2 runs
    s120, s240 = f2_2x2.read_steer(args.run120), f2_2x2.read_steer(args.run240)
    out = {"run120": str(args.run120), "run240": str(args.run240), "dt_120": v120["dt_s"], "dt_240": v240["dt_s"],
           "digest_120": s120["digest"], "digest_240": s240["digest"], "arm_120": s120["arm"], "arm_240": s240["arm"],
           **compare(v120, v240), "v2_120": v120["v2"], "v2_240": v240["v2"]}
    if abs(1 / v120["dt_s"] - 120) > 1e-6 or abs(1 / v240["dt_s"] - 240) > 1e-6:
        out["v4_v2_preserved"] = None
        out["problem"] = "the runs are not 120 and 240 Hz"
    args.json.write_text(json.dumps(out, indent=1, default=float) + "\n")
    print(json.dumps({k: out[k] for k in ("comparable", "v4_v2_preserved")}))


if __name__ == "__main__":
    main()
