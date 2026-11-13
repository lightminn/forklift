"""S3 criterion 4 (docs/plans/2026-11-13-slam-online-bounded.md 4): each map measurement run's /map scored against
the truth obstacle cells at the registered checkpoints -- every unit start (the last grid written before the unit's
first scan), right after C's first deletion (the first grid written after the keyframe whose trace shows a deleted
node), and the end (the last grid). Uses tools/slam_map_quality.py of the snapshot.
    python s3_map_quality.py SNAPSHOT TRUTH_RUN... -- RUN_DIR...   (prints JSON)"""
import json
import sys
from pathlib import Path

snap = Path(sys.argv[1])
sep = sys.argv.index("--")
truth_runs = [Path(p) for p in sys.argv[2:sep]]
runs = [Path(p) for p in sys.argv[sep + 1:]]
sys.path.insert(0, str(snap / "tools"))
sys.path.insert(0, str(snap / "src"))
import numpy as np  # noqa: E402
import slam_map_quality as q  # noqa: E402

truth = q._truth_from_runs(truth_runs, 0.05, 3)


def score(bridge: Path, rows: list, idx: int) -> dict:
    meta = next(r for r in rows if r.get("index") == idx)
    with np.load(bridge / "maps" / f"map_{idx:06d}.npz") as z:
        g = q.Grid(np.asarray(z["data"]), tuple(meta["origin"]), float(meta["resolution_m"]))
    return {"map_index": idx, **q.quality(q.grid_cells(g), truth, resolution_m=0.05)}


out = {"truth_cells": len(truth), "runs": {}}
for run in runs:
    bridge = run / "bridge"
    rows = [json.loads(x) for x in (bridge / "maps.jsonl").read_text().splitlines() if x.strip()]
    drive = [json.loads(x) for x in (run / "drive.jsonl").read_text().splitlines() if x.strip()]
    status = json.loads((run / "status.json").read_text())
    firsts = {}
    for r in drive:
        firsts.setdefault(int(r.get("unit", 0)), int(r["scan_id"]))
    res = {"status": status, "units": [], "first_deletion": None, "end": None}
    for unit in sorted(firsts):
        idx = q.map_index_before(rows, firsts[unit])
        if idx is not None:
            res["units"].append({"unit": unit, **score(bridge, rows, idx)})
    trace = bridge / "slam_trace.jsonl"
    if trace.is_file():
        first = None
        for line in trace.read_text().splitlines():
            rec = json.loads(line)
            if rec.get("kind") == "scan" and int(rec.get("deleted_nodes", 0) or 0) > 0:
                first = rec
                break
        if first is not None:
            stamp = float(first["stamp_s"])
            sid = min((int(r["scan_id"]) for r in drive if abs(float(r["stamp_s"]) - stamp) < 1e-6), default=None)
            after = [r["index"] for r in rows if "index" in r and int(r.get("after_scan_id", -1)) >= (sid or 0)]
            if sid is not None and after:
                res["first_deletion"] = {"scan_id": sid, "stamp_s": stamp, **score(bridge, rows, min(after))}
            else:
                res["first_deletion"] = {"scan_id": sid, "stamp_s": stamp, "map": None}
    last = max(r["index"] for r in rows if "index" in r)
    res["end"] = score(bridge, rows, last)
    out["runs"][run.name] = res
print(json.dumps(out, indent=1))
