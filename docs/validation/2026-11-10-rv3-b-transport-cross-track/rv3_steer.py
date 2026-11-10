#!/usr/bin/env python3
"""Steering command vs actual per wheel along the P-5 transport (state.samples), Ackermann curvature each wheel implies.
index 0 = left, 1 = right (the failure record's left-turn command has [0] larger). usage: rv3_steer.py SESSIONS_DIR OUT.json"""
import glob, json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nf_transport_cmp import read
L, T = 0.66, 0.53

def kappa(delta, side):
    t = math.tan(delta)
    return t / (L + T / 2 * t) if side == 0 else t / (L - T / 2 * t)

def measure(run, mission):
    result = json.load(open(f"{run}/missions/{mission}/result.json"))["result"]
    top = json.load(open(f"{run}/result.json"))
    t_in = next(t["time_s"] for t in top["transitions"]
                if t["to"] == "transport" and result["started_s"] <= t["time_s"] <= result["ended_s"])
    later = [t["time_s"] for t in top["transitions"] if t["time_s"] > t_in + 1e-6 and t["to"] != "transport"]
    t_out = min(later + [result["ended_s"]])
    s = [x for x in read(run, "state.samples") if t_in <= float(x["time_s"]) <= t_out and x["signed_speed_mps"] > 0.05]
    out = {}
    for name, sign in (("right_turn", -1), ("left_turn", 1)):
        sat = [x for x in s if x["curvature_inv_m"] * sign >= 0.355]
        if not sat:
            continue
        inner = 1 if sign < 0 else 0
        outer = 1 - inner
        cmd_in = [abs(x["steering_command_rad"][inner]) for x in sat]
        act_in = [abs(x["steering_actual_rad"][inner]) for x in sat]
        cmd_out = [abs(x["steering_command_rad"][outer]) for x in sat]
        act_out = [abs(x["steering_actual_rad"][outer]) for x in sat]
        k_in = [abs(kappa(x["steering_actual_rad"][inner], inner)) for x in sat]
        k_out = [abs(kappa(x["steering_actual_rad"][outer], outer)) for x in sat]
        m = lambda v: round(sum(v) / len(v), 4)
        out[name] = {"samples": len(sat), "inner_cmd_mean": m(cmd_in), "inner_act_mean": m(act_in), "inner_act_max": round(max(act_in), 4),
                     "outer_cmd_mean": m(cmd_out), "outer_act_mean": m(act_out), "kappa_inner_act": m(k_in), "kappa_outer_act": m(k_out)}
    return out

def main(root, outp):
    table = {}
    for run in sorted(glob.glob(f"{root}/*/live/epoch_1/run")):
        name = run[len(root) + 1:].split("/")[0]
        for mf in sorted(glob.glob(f"{run}/missions/m*/result.json")):
            mission = mf.split("/")[-2]
            st = json.load(open(os.path.dirname(mf) + "/records.json"))["state"]
            r = json.load(open(mf))["result"]
            if (r.get("pallet_id") or st.get("pallet_id")) != "P-5":
                continue
            table[f"{name}/{mission}"] = measure(run, mission)
    json.dump(table, open(outp, "w"), indent=1)
    for k, v in table.items():
        print(k, json.dumps(v))

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
