#!/usr/bin/env python3
"""F1 v2 stage 0 (docs/plans/2026-11-04-f1-v2-registration.md section 2): does scan_docking.dock() on a recorded
delivery match's own inputs give back the recorded round-1 result, bit for bit? If not, a probe that calls dock() cannot
be taken to reproduce the operating measurement.

  python tools/f1v2_dock_repro.py --pair MATCH.npz RESULT.json [--pair ...] --json OUT.json

MATCH.npz is a run's delivery_match_1.npz (reference, live, rear_from_laser, estimate, seed); RESULT.json holds the run's
result.json, or a dict with its delivery_docking.rounds and arguments.docking_acceptance.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from forklift_core.localization.scan_docking import dock


def recorded_round1(result: dict) -> dict:
    rounds = result.get("rounds") or (result.get("delivery_docking") or {}).get("rounds") or []
    if not rounds:
        raise ValueError("no recorded delivery_docking round")
    return rounds[0]


def acceptance_of(result: dict) -> str:
    acc = result.get("docking_acceptance") or (result.get("arguments") or {}).get("docking_acceptance")
    return acc or "information"


def compare(match_path: Path, result_path: Path) -> dict:
    z = np.load(match_path)
    result = json.loads(result_path.read_text())
    rec = recorded_round1(result)
    acceptance = acceptance_of(result)
    kw = {} if acceptance == "information" else {"acceptance": acceptance}
    got = dock(z["reference"], z["live"], rear_from_laser=tuple(float(v) for v in z["rear_from_laser"]),
               estimate_rear=tuple(float(v) for v in z["estimate"]), delivery_rear=tuple(float(v) for v in z["seed"]),
               **kw)
    starts_got = [[bool(a), str(r), [float(v) for v in q]] for a, r, q in got.starts]
    starts_rec = [[bool(s[0]), str(s[1]), [float(v) for v in s[2]]] for s in rec.get("starts") or []]
    details_got = [{k: (float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else v)
                    for k, v in d.items()} for d in got.start_details]
    details_rec = rec.get("start_details") or []
    keys = ("inlier_fraction", "median_residual_m", "min_information", "iterations")
    detail_equal = len(details_got) == len(details_rec) and all(
        dg.get(k) == dr.get(k) for dg, dr in zip(details_got, details_rec) for k in keys)
    return {"match": str(match_path), "acceptance": acceptance,
            "accepted": {"got": bool(got.accepted), "recorded": bool(rec.get("accepted"))},
            "reason": {"got": got.reason, "recorded": rec.get("reason")},
            "starts_equal": starts_got == starts_rec, "details_equal": detail_equal,
            "min_information": {"got": min(d["min_information"] for d in details_got),
                                "recorded": min(d["min_information"] for d in details_rec)},
            "bit_identical": (starts_got == starts_rec and detail_equal and bool(got.accepted) == bool(rec.get("accepted"))
                              and got.reason == rec.get("reason"))}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--pair", nargs=2, action="append", required=True, metavar=("MATCH_NPZ", "RESULT_JSON"))
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    rows = [compare(Path(m), Path(r)) for m, r in args.pair]
    out = {"all_bit_identical": all(r["bit_identical"] for r in rows), "rows": rows}
    args.json.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({"all_bit_identical": out["all_bit_identical"],
                      "rows": [(Path(r["match"]).name, r["bit_identical"], r["min_information"]) for r in rows]}))


if __name__ == "__main__":
    main()
