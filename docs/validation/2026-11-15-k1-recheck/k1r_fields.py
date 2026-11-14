"""The K1 re-check's raw fields: the keys k1_raw_fields.json.gz (#498) kept, from each run's result.json."""
import gzip, json, sys
from pathlib import Path
KEYS = ("deterministic", "failure_reason", "forbidden_pocket_contacts", "near_capture", "near_loose_fallbacks",
        "near_loose_refusals", "near_loose_watch", "observe_end_stands", "observe_loose_arrivals", "observe_replans",
        "phase", "simulated_time_s", "success", "tracking_failure", "transitions")
root, out = Path(sys.argv[1]), Path(sys.argv[2])
fields = {}
for d in sorted(root.glob("k1r_*")):
    p = d / "run" / "result.json"
    if not (d.is_dir() and p.exists()):
        continue
    r = json.loads(p.read_text())
    fields[d.name.replace("k1r_", "k1c_", 1)] = {k: r[k] for k in KEYS if k in r}
with gzip.open(out, "wt") as f:
    json.dump(fields, f, sort_keys=True)
print(len(fields), out)
