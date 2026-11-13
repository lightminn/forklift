"""Per-scan reply comparison of the two repeats of each arm on their common (unit, run, src) keys, and the sim 0-3,600 s
maximum keyframe time of every judged run."""
import json, math, sys
from pathlib import Path
base = Path(sys.argv[1])
def rows(a): return [json.loads(x) for x in (base / a / "drive.jsonl").read_text().splitlines() if x.strip()]
out = {"first_hour_max_s": {}, "pairs": {}}
R = {a: rows(a) for a in ["p0_1", "p0_2", "cwm_1", "cwm_2", "c600_1", "c600_2", "c1200_1", "c1200_2", "s1_1", "s1_2"]}
for a, r in R.items():
    f = [float(x["slam_wall_s"]) for x in r if float(x["stamp_s"]) <= 3600]
    out["first_hour_max_s"][a] = {"max": max(f), "end_sim": max(float(x["stamp_s"]) for x in r),
                                  "covers_3600": max(float(x["stamp_s"]) for x in r) >= 3600}
for arm in ["p0", "cwm", "c600", "c1200", "s1"]:
    a, b = R[arm + "_1"], R[arm + "_2"]
    kb = {(int(x.get("unit", 0)), int(x.get("run", 0)), int(x["src"])): x for x in b}
    n = mism = 0; dp = dy = 0.0
    for x in a:
        k = (int(x.get("unit", 0)), int(x.get("run", 0)), int(x["src"]))
        if k not in kb: continue
        y = kb[k]; n += 1
        mx, my = x["map_from_odom"], y["map_from_odom"]
        p = math.hypot(mx[0] - my[0], mx[1] - my[1]); d = mx[2] - my[2]; w = abs(math.atan2(math.sin(d), math.cos(d)))
        dp, dy = max(dp, p), max(dy, w); mism += (p > 1e-6 or w > 1e-6)
    out["pairs"][arm] = {"common": n, "max_pos_m": dp, "max_yaw_rad": dy, "over_1e-6": mism}
print(json.dumps(out, indent=1))
