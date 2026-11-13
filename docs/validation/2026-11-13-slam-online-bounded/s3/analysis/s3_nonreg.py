"""S3 criterion 3 non-regression against P0 (same method as the S2 doc): same (unit, run, src) pairs, the control's
observed span only; candidate median - control median, max - max, heading max difference, excess growth."""
import json, math, statistics, sys
from pathlib import Path
sys.path.insert(0, sys.argv[1] + "/tools")
import slam_cost_summary as cs
base = Path(sys.argv[2])
def rows(p): return [json.loads(x) for x in open(p) if x.strip()]
def yawerr(r):
    m, o, t = r["map_from_odom"], r["odom_base"], r["truth_base"]
    y = o[2] + m[2] - t[2]; return abs(math.degrees(math.atan2(math.sin(y), math.cos(y))))
def nonreg(cand, ctl):
    ref = {(int(r.get("unit", 0)), int(r.get("run", 0)), int(r["src"])): r for r in ctl}
    pc, pb, yc, yb = [], [], [], []
    for r in cand:
        k = (int(r.get("unit", 0)), int(r.get("run", 0)), int(r["src"]))
        if k in ref:
            pc.append(cs._pos_err(r)); pb.append(cs._pos_err(ref[k])); yc.append(yawerr(r)); yb.append(yawerr(ref[k]))
    ex = cs.excess_over_baseline(cand, ctl)
    return {"pairs": len(pc), "ctl_end_sim": round(max(float(r["stamp_s"]) for r in ctl), 1),
            "cand_end_sim": round(max(float(r["stamp_s"]) for r in cand), 1),
            "median_cm": round(100 * (statistics.median(pc) - statistics.median(pb)), 2),
            "max_cm": round(100 * (max(pc) - max(pb)), 2), "yaw_max_deg": round(max(yc) - max(yb), 2),
            "excess_pairs": ex["matched"],
            "excess_growth_cm": None if ex["excess_growth_m"] is None else round(100 * ex["excess_growth_m"], 2)}
out = {}
for c in ["c600_1", "c600_2", "c1200_1", "c1200_2", "cwm_1", "cwm_2", "s1_1", "s1_2"]:
    for p in ["p0_1", "p0_2"]:
        out[f"{c}_vs_{p}"] = nonreg(rows(base / c / "drive.jsonl"), rows(base / p / "drive.jsonl"))
print(json.dumps(out, indent=1))
