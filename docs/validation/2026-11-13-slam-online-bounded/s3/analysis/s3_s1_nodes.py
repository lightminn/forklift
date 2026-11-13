"""S1 criterion 9: permanent nodes per unit from the S1 map measurement run's trace (s1_permanent_nodes after each
keyframe), the unit of each keyframe from drive.jsonl by stamp; growth in units 8-14 against those made in unit 1."""
import json, sys
from pathlib import Path
run = Path(sys.argv[1])
drive = [json.loads(x) for x in (run / "drive.jsonl").read_text().splitlines() if x.strip()]
unit_of = {round(float(r["stamp_s"]), 6): int(r.get("unit", 0)) for r in drive}
last = {}
first = {}
promoted = deleted = 0
temp = 0
for line in (run / "bridge" / "slam_trace.jsonl").read_text().splitlines():
    rec = json.loads(line)
    if rec.get("kind") != "scan" or "s1_permanent_nodes" not in rec:
        continue
    u = unit_of.get(round(float(rec["stamp_s"]), 6))
    if u is None:
        continue
    first.setdefault(u, rec["s1_permanent_nodes"])
    last[u] = rec["s1_permanent_nodes"]
    promoted, deleted = rec.get("s1_promoted_total", promoted), rec.get("s1_deleted_total", deleted)
units = sorted(last)
end_of = {u: last[u] for u in units}
made = {u: end_of[u] - (end_of[u - 1] if u - 1 in end_of else 0) for u in units}
grow_8_14 = end_of[max(units)] - end_of.get(7, 0)
print(json.dumps({"permanent_at_unit_end": end_of, "made_per_unit": made, "unit1_made": made.get(1),
                  "growth_units_8_14": grow_8_14, "ratio": grow_8_14 / made[1] if made.get(1) else None,
                  "promoted_total": promoted, "deleted_total": deleted}, indent=1))
