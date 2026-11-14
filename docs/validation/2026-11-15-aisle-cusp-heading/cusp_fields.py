"""④ fields (plan 2026-11-15-aisle-cusp-heading 5): the K1 re-check aisle_rack runs' result keys and their
transport-window slam_control rows, keyed by run name (as tdep_fields.json.gz). Rows from 1 s before the first
transport tracker's installation (the first transport legs start in extract) to 1 s after transport ends, rounded
to 1e-6. A K1 run whose window rows equal its B0 pair's keeps a sha256 and the pair's name instead of the rows."""
import gzip, hashlib, json, sys
from pathlib import Path
import numpy as np
KEYS = ("transitions", "leg_switches", "tracker_versions", "cusp_replans", "cusp_stalls", "scene_replans", "paths",
        "paths_replaced", "tracking_failure", "success", "failure_reason", "d7_docking", "docking_retries")
root, out = Path(sys.argv[1]), Path(sys.argv[2])
fields, sha = {}, {}
for d in sorted(root.glob("k1r_*_aisle_rack_s*")):  # B0 before K1 (sorted)
    p = d / "run" / "result.json"
    if not p.exists():
        continue
    r = json.loads(p.read_text())
    tr = r["transitions"]
    t0 = next(t["time_s"] for t in tr if t["to"] == "transport")
    t1 = next((t["time_s"] for t in tr if t["from"] == "transport"), float("inf"))
    inst = [v["installed_s"] for v in r.get("tracker_versions") or [] if v.get("phase") == "transport"]
    lo = min([t0] + inst) - 1.0
    res = {k: r[k] for k in KEYS if k in r}
    res["samples"] = [s for s in r.get("samples") or [] if lo <= float(s["time_s"]) <= t1 + 1.0]
    res["transport_window_s"] = [t0, None if t1 == float("inf") else t1]
    sc = np.load(d / "run" / "slam_control.npy")
    rows = np.round(sc[(sc[:, 0] >= lo) & (sc[:, 0] <= t1 + 1.0)], 6)
    h = hashlib.sha256(np.ascontiguousarray(rows).tobytes()).hexdigest()
    entry = {"result": res, "slam_control_sha256": h}
    pair = d.name.replace("_K1_", "_B0_")
    if pair != d.name and sha.get(pair) == h:
        entry["slam_control_rows_same_as"] = pair
    else:
        entry["slam_control_rows"] = rows.tolist()
    sha[d.name] = h
    fields[d.name] = entry
with gzip.open(out, "wt") as f:
    json.dump(fields, f, sort_keys=True, separators=(",", ":"))
print(len(fields), out, {k: ("same_as" if "slam_control_rows_same_as" in v else len(v["slam_control_rows"])) for k, v in fields.items()})
