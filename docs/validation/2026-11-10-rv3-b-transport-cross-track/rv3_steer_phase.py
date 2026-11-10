#!/usr/bin/env python3
"""Left/right steering deficit (command - actual, rad) when the tracker's curvature command is saturated, by phase and
direction of travel, pooled over the given sessions' whole runs. usage: rv3_steer_phase.py OUT.json RUN..."""
import json, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nf_transport_cmp import read
from collections import defaultdict

def main(outp, runs):
    acc = defaultdict(lambda: [0, 0.0, 0.0])
    for run in runs:
        for x in read(run, "state.samples"):
            if abs(x["curvature_inv_m"]) < 0.355 or abs(x["signed_speed_mps"]) < 0.05:
                continue
            turn = "left" if x["curvature_inv_m"] > 0 else "right"
            gear = "fwd" if x["signed_speed_mps"] > 0 else "rev"
            key = f'{x["phase"]}|{gear}|{turn}|lift>{0.1 if x["lift_m"] > 0.1 else 0}'
            a = acc[key]
            a[0] += 1
            a[1] += abs(x["steering_command_rad"][0]) - abs(x["steering_actual_rad"][0])
            a[2] += abs(x["steering_command_rad"][1]) - abs(x["steering_actual_rad"][1])
    table = {k: {"n": n, "left_deficit": round(l / n, 4), "right_deficit": round(r / n, 4)} for k, (n, l, r) in sorted(acc.items())}
    json.dump(table, open(outp, "w"), indent=1)
    for k, v in table.items():
        print(k, v)

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
