#!/usr/bin/env python3
"""Per-session left-wheel steering deficit (|command| - |actual|, rad) in forward travel with the tracker's curvature
saturated (|curvature_inv_m| >= 0.355), split loaded (lift_m > 0.1) / unloaded. usage: rv3_steer_session.py SESSIONS_DIR OUT.json"""
import glob, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nf_transport_cmp import read

def main(root, outp):
    res = {}
    for run in sorted(glob.glob(f"{root}/*/live/epoch_1/run")):
        name = run[len(root) + 1:].split("/")[0]
        g = {"loaded": [0, 0.0], "unloaded": [0, 0.0]}
        for x in read(run, "state.samples"):
            if abs(x["curvature_inv_m"]) < 0.355 or x["signed_speed_mps"] <= 0.05:
                continue
            k = "loaded" if x["lift_m"] > 0.1 else "unloaded"
            g[k][0] += 1
            g[k][1] += abs(x["steering_command_rad"][0]) - abs(x["steering_actual_rad"][0])
        res[name] = {k: {"n": n, "left_deficit": round(s / n, 4) if n else None} for k, (n, s) in g.items()}
    summary = {k: {"sessions_n_ge_50": len(v), "min": min(v), "max": max(v)} for k in ("loaded", "unloaded")
               for v in [[r[k]["left_deficit"] for r in res.values() if r[k]["n"] >= 50]]}
    json.dump({"sessions": res, "summary": summary}, open(outp, "w"), indent=1)
    print(json.dumps(summary))

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
