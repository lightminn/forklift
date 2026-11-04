"""near_end_stand vs the delivery round-1 SLAM estimate error, from every riib/simphys record on ws1 (read only).
Per run: batch, arm, scene, seed, near_end_stand setting, its records (observe_end_stands kind near: outcome), the
observe phase's replans by trigger, the delivery round-1 estimate-vs-truth (world x, y), the docking reason, the end.
JSON lines on stdout."""
import glob
import json
import os
import sys

ROOTS = ["/home/projects/forklift/test_runs/riibotics", "/home/projects/forklift/test_runs/simphys"]
n = 0
for root in ROOTS:
    for res in glob.glob(os.path.join(root, "**", "run", "result.json"), recursive=True):
        try:
            d = json.load(open(res))
        except (OSError, ValueError):
            continue
        a = d.get("arguments") or {}
        dd = d.get("delivery_docking") or {}
        r1 = [r for r in dd.get("rounds") or [] if r.get("round") == 1]
        stands = d.get("observe_end_stands") or []
        replans = d.get("observe_replans") or []
        run_dir = os.path.dirname(res)
        rel = run_dir.split("test_runs/")[1]
        row = {
            "run": rel, "batch": rel.split("/")[1], "name": rel.split("/")[-2],
            "scene": os.path.basename(a.get("scene_file") or "").rsplit(".", 1)[0] or a.get("layout"),
            "seed": a.get("seed"), "near_end_stand": a.get("near_end_stand"),
            "near_tracking": a.get("near_tracking"), "encoder_model": a.get("encoder_model"),
            "urdf": (d.get("forklift_urdf_sha256") or "")[:8],
            "pallet_kg": (d.get("pallet_mass") or {}).get("kg") if isinstance(d.get("pallet_mass"), dict) else None,
            "stands": [{"kind": s.get("kind"), "outcome": s.get("outcome"), "time_s": s.get("time_s")} for s in stands],
            "replan_triggers": [r.get("trigger") for r in replans],
            "r1": [{"est_vs_truth": r.get("estimate_vs_truth_at_match"), "reason": r.get("reason"),
                    "info": min((s.get("min_information") for s in r.get("start_details") or []
                                 if s.get("min_information") is not None), default=None),
                    "time_s": r.get("time_s"), "retry": r.get("retry_match")} for r in r1],
            "docking_reason": dd.get("reason"), "failure_reason": d.get("failure_reason"), "success": d.get("success"),
            "attempt_folder": ".attempt" in run_dir,
        }
        print(json.dumps(row))
        n += 1
print(json.dumps({"rows": n}), file=sys.stderr)
