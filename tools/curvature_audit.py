"""Curvature response plan arm A (docs/plans/2026-10-24-curvature-response.md): the recorded pre-install dry runs
re-judged with a curvature response, per call site and with that site's own acceptance rule, paired with how the
installed leg actually ended.

Sites covered from the records:
  near   the near capture's default straight (state.near_capture.dry_run; the approach path; start = the dry
         run's first pose; acceptance: arrived, <= 6 mm, <= 15 mrad -- and the neighbour sweep, which needs the
         neighbours' outlines the record does not keep: unverified when there were any)
  d7     the D7 docking retry's re-approach (state.docking_retries[].plan.dry_run; the installed path from
         paths_replaced / paths; start = the retry's start; docking_stop_config = the transport tracker's config at
         3 cm / 0.05 rad; acceptance: arrived, <= 3 cm, <= 0.05 rad)
Each call is first re-run at r = 1 and must reproduce the recorded result (status, errors within 1e-6) -- otherwise
its inputs were not recovered and it counts as unverified (U). Recorded refusals whose path was not kept are U too.
The cusp stall judgement and the near straight candidates (pallet_mission dry_runs) are not recoverable from the
live records (the cells' FREE times, the candidates' paths) and are listed as not covered.

    python tools/curvature_audit.py RUN_DIR... --forward 0.88 --reverse 1.0 --out RESULT.json
RUN_DIR is a run's .../live/epoch_1/run (its result.json for the tracker configs, its missions/m*/records.json)."""

from __future__ import annotations

import argparse
import dataclasses
import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from forklift_core.control import TrackerConfig  # noqa: E402
from forklift_core.control.rollout import CurvatureResponse, bicycle_rollout  # noqa: E402

FIELDS = {f.name for f in dataclasses.fields(TrackerConfig)}


def tracker_config(record: dict) -> TrackerConfig:
    record = record.get("config", record)
    return TrackerConfig(**{k: v for k, v in record.items() if k in FIELDS})


def path_arrays(record: dict):
    return (np.asarray(record["poses"], float), np.asarray(record["directions"], np.int8),
            np.asarray(record["curvatures_inv_m"], float))


def same(roll, recorded: dict) -> bool:
    if roll.status != recorded.get("status"):
        return False
    for key in ("position_error_m", "yaw_error_rad"):
        a, b = getattr(roll, key), recorded.get(key)
        if b is None or not (math.isfinite(a) and math.isfinite(b)) or abs(a - b) > 1e-6:
            if not (b is not None and not math.isfinite(a) and not math.isfinite(b)):
                return False
    return True


def near_ok(roll) -> bool:
    return roll.status == "arrived" and roll.position_error_m <= 0.006 and abs(roll.yaw_error_rad) <= 0.015


def d7_ok(roll) -> bool:
    return roll.status == "arrived" and roll.position_error_m <= 0.03 and abs(roll.yaw_error_rad) <= 0.05


def mission_outcome(run: str, md: str) -> str:
    try:
        res = json.load(open(f"{md}/result.json")).get("result") or {}
        return str(res.get("outcome"))
    except Exception:
        return "unknown"


def audit(runs, response: CurvatureResponse) -> dict:
    rows = []
    for run in runs:
        name = run.rstrip("/").split("/")[-4] if run.rstrip("/").endswith("epoch_1/run") else os.path.basename(run)
        try:
            configs = json.load(open(f"{run}/result.json")).get("tracker_configs") or {}
        except Exception:
            configs = {}
        for md in sorted(glob.glob(f"{run}/missions/m*")):
            try:
                st = json.load(open(f"{md}/records.json"))["state"]
            except Exception:
                continue
            outcome = mission_outcome(run, md)
            base = {"run": name, "mission": os.path.basename(md), "outcome": outcome}
            # near capture default straight
            nc = st.get("near_capture") or {}
            dry = nc.get("dry_run")
            if dry is not None:
                row = {**base, "site": "near", "recorded_ok": None, "verified": False, "why": None}
                approach = (st.get("paths") or {}).get("approach")
                traj = dry.get("trajectory") or []
                if "approach" not in configs:
                    row["why"] = "no tracker config (no run-level result.json)"
                elif not traj or not approach or "poses" not in approach:
                    row["why"] = "no start or path"
                else:
                    cfg, start, arrays = tracker_config(configs["approach"]), np.asarray(traj[0], float), path_arrays(approach)
                    ideal = bicycle_rollout(*arrays, cfg, start)
                    row["recorded_ok"] = (dry.get("status") == "arrived" and dry.get("position_error_m", 1) <= 0.006
                                          and abs(dry.get("yaw_error_rad", 1)) <= 0.015)
                    if not same(ideal, dry):
                        row["why"] = "r = 1 does not reproduce the record"
                    else:
                        new = bicycle_rollout(*arrays, cfg, start, curvature_response=response)
                        neighbours = (nc.get("neighbour_sweep") or {}).get("neighbours", 0)
                        row.update(verified=True, new_ok=near_ok(new), new_status=new.status,
                                   new_position_error_m=new.position_error_m, new_yaw_error_rad=new.yaw_error_rad,
                                   sweep_unverified=bool(neighbours and near_ok(new)))
                rows.append(row)
            # D7 docking retries
            retries = [r for r in st.get("docking_retries") or [] if (r.get("plan") or {}).get("dry_run")]
            # every transport path the mission held (each replaced one, then the last): a re-approach is found by its
            # planned length (a later delivery straight or re-approach replaces it, so its place in the list varies)
            held = [x.get("path") for x in st.get("paths_replaced") or [] if x.get("leg") == "transport"]
            held.append((st.get("paths") or {}).get("transport"))
            held = [h for h in held if h and "poses" in h]
            for k, r in enumerate(retries):
                row = {**base, "site": "d7", "retry": k, "trigger": r.get("trigger"), "recorded_ok": None,
                       "verified": False, "why": None, "later_retry": k + 1 < len(retries)}
                dry_d = r["plan"]["dry_run"]
                row["recorded_ok"] = (dry_d.get("status") == "arrived" and dry_d.get("position_error_m", 1) <= 0.03
                                      and abs(dry_d.get("yaw_error_rad", 1)) <= 0.05)
                length = (r.get("plan") or {}).get("length_m")
                matches = [h for h in held if length is not None and abs(float(h.get("length_m", -1)) - float(length)) < 1e-6]
                path = matches[0] if len(matches) == 1 else None
                if "transport" not in configs:
                    row["why"] = "no tracker config"
                elif not path or "poses" not in path or not row["recorded_ok"]:
                    row["why"] = ("path not kept (or not unique by length)" if row["recorded_ok"]
                                  else "refused: path not kept")
                else:
                    cfg = dataclasses.replace(tracker_config(configs["transport"]), position_tolerance_m=0.03,
                                              yaw_tolerance_rad=0.05)
                    arrays, start = path_arrays(path), np.asarray(r["start"], float)
                    ideal = bicycle_rollout(*arrays, cfg, start)
                    if not same(ideal, dry_d):
                        row["why"] = "r = 1 does not reproduce the record"
                    else:
                        new = bicycle_rollout(*arrays, cfg, start, curvature_response=response)
                        row.update(verified=True, new_ok=d7_ok(new), new_status=new.status,
                                   new_position_error_m=new.position_error_m, new_yaw_error_rad=new.yaw_error_rad)
                rows.append(row)
    summary = {}
    for site in ("near", "d7"):
        sel = [r for r in rows if r["site"] == site]
        ver = [r for r in sel if r["verified"]]
        good_later = [r for r in ver if r["recorded_ok"] and r["outcome"] == "done" and not r.get("later_retry")]
        bad_later = [r for r in ver if r["recorded_ok"] and (r["outcome"] != "done" or r.get("later_retry"))]
        summary[site] = {
            "calls": len(sel), "verified": len(ver), "unverified": len(sel) - len(ver),
            "unverified_why": sorted({str(r["why"]) for r in sel if not r["verified"]}),
            "accept_to_refuse": sum(1 for r in ver if r["recorded_ok"] and not r["new_ok"]),
            "refuse_to_accept": sum(1 for r in ver if not r["recorded_ok"] and r["new_ok"]),
            "later_good_refused_now": sum(1 for r in good_later if not r["new_ok"]),
            "later_good": len(good_later),
            "later_bad_refused_now": sum(1 for r in bad_later if not r["new_ok"]),
            "later_bad": len(bad_later),
            "sweep_unverified": sum(1 for r in ver if r.get("sweep_unverified")),
        }
    return {"response": dataclasses.asdict(response), "summary": summary, "rows": rows,
            "not_covered": ["cusp stall judgement (cells' FREE times not recorded)",
                            "pallet_mission near straight candidates (their paths not recorded)",
                            "near capture redraw (none in these records) "]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--forward", type=float, default=0.88)
    ap.add_argument("--reverse", type=float, default=1.0)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    result = audit(args.runs, CurvatureResponse(args.forward, args.reverse))
    args.out.write_text(json.dumps(result, indent=1, default=float))
    print(json.dumps(result["summary"], indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
