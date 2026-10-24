"""Curvature response plan arm B (docs/plans/2026-10-24-curvature-response.md): every recorded D7 docking retry's
re-approach planned again at the planner curvature limits given, from the recorded start to the recorded docking
line start, in the mission's own recorded planning scene (state.mission_scenario: its props and bounds) -- without the
live obstacle grid, which the records do not keep (an approximation, so a plan at the run's own limit that does not
reproduce the recorded length is marked unreproduced). Reports per limit the outcome (success, no_path, leaves_box,
too_long, dry_run refusal), the length and the gear changes.

    python tools/d7_reapproach_kappa.py RUN_DIR... --kappa 0.29 0.27 --out RESULT.json
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p_ in (ROOT / "src", ROOT / "tools", ROOT / "sim/isaac", ROOT):
    if str(p_) not in sys.path:
        sys.path.insert(0, str(p_))

from bay_planning_sweep import mission_geometry  # noqa: E402
from riibotics_scenario_paths import PALLETS  # noqa: E402
from scene_file import scenario_from_record  # noqa: E402
from session_plan_check import RESERVE_M, STRAIGHTS, URDF  # noqa: E402

from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning import Pose2D  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    D7_PLANNER_OPTIONS,
    SearchBudget,
    make_transport_planner_config,
    plan_docking_reapproach,
)


def gear_changes(path) -> int:
    d = np.asarray(path.directions)
    return int(np.sum(d[1:] != d[:-1])) if len(d) > 1 else 0


def run(runs, kappas, pallet_kind="epal6") -> dict:
    pallet = load_pallet_geometry(PALLETS[pallet_kind])
    geometry = mission_geometry(URDF, pallet, RESERVE_M, **STRAIGHTS)
    rows = []
    for run_dir in runs:
        name = run_dir.rstrip("/").split("/")[-4]
        for md in sorted(glob.glob(f"{run_dir}/missions/m*")):
            try:
                st = json.load(open(f"{md}/records.json"))["state"]
            except Exception:
                continue
            scene = st.get("mission_scenario")
            for k, retry in enumerate(st.get("docking_retries") or []):
                plan = retry.get("plan") or {}
                if not plan or retry.get("line_start") is None or retry.get("start") is None:
                    continue
                row = {"run": name, "mission": os.path.basename(md), "retry": k, "trigger": retry.get("trigger"),
                       "recorded": {"status": plan.get("status"), "refused": plan.get("refused"),
                                    "length_m": plan.get("length_m"), "gear_changes": plan.get("gear_changes")},
                       "by_kappa": {}}
                if scene is None:
                    row["why"] = "no mission_scenario"
                    rows.append(row)
                    continue
                scenario = scenario_from_record(scene)
                for kappa in kappas:
                    config = make_transport_planner_config(curvature_limit_inv_m=kappa, clearance_m=0.10,
                                                           max_expansions=30000, **D7_PLANNER_OPTIONS)
                    path, record = plan_docking_reapproach(
                        scenario, Pose2D(*retry["start"]), Pose2D(*retry["line_start"]), config, geometry=geometry,
                        keep_m=geometry.delivery_straight_m, deadline=SearchBudget(20.0))
                    row["by_kappa"][str(kappa)] = {
                        "status": record.get("status"), "refused": record.get("refused"),
                        "length_m": None if path is None else float(path.length_m),
                        "gear_changes": None if path is None else gear_changes(path)}
                # the self-check: at the run's own limit (the first one given) the recorded plan comes back
                own = row["by_kappa"][str(kappas[0])]
                rec_len = plan.get("length_m")
                row["reproduced"] = (own["length_m"] is not None and rec_len is not None
                                     and abs(own["length_m"] - float(rec_len)) < 1e-6) or (
                    own["length_m"] is None and rec_len is None)
                rows.append(row)
    summary = {"retries": len(rows), "reproduced": sum(1 for r in rows if r.get("reproduced"))}
    for kappa in kappas:
        sel = [r["by_kappa"].get(str(kappa)) for r in rows if r.get("reproduced") and str(kappa) in r["by_kappa"]]
        summary[str(kappa)] = {"plans": len(sel), "planned": sum(1 for s in sel if s["length_m"] is not None),
                               "refused": {why: sum(1 for s in sel if s["length_m"] is None and (s["refused"] or s["status"]) == why)
                                           for why in sorted({str(s["refused"] or s["status"]) for s in sel if s["length_m"] is None})},
                               "gear_changes_total": sum(s["gear_changes"] or 0 for s in sel if s["length_m"] is not None)}
    return {"kappas": kappas, "summary": summary, "rows": rows}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--kappa", type=float, nargs="+", default=[0.29, 0.27])
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    result = run(args.runs, args.kappa)
    args.out.write_text(json.dumps(result, indent=1, default=float))
    print(json.dumps(result["summary"], indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
