"""Round 1 actual stop of every V3 run: the true rear axle relative to the reference rear pose in its
delivery_match_1.npz (dx < 0: short of the reference), one JSON line per run (first_stops.jsonl; "stop": null when the
run has no round-1 match file).
    python first_stops.py BATCH_DIR > first_stops.jsonl"""
import glob, json, math, os, sys
import numpy as np
O = sys.argv[1]
def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2]); return (a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1], a[2] + b[2])
def invert(a):
    c, s = math.cos(a[2]), math.sin(a[2]); return (-c*a[0] - s*a[1], s*a[0] - c*a[1], -a[2])
for d in sorted(glob.glob(f"{O}/ddc_*/run")):
    name = d.split("/")[-2]
    f = os.path.join(d, "delivery_match_1.npz")
    if not os.path.exists(f):
        print(json.dumps({"name": name, "stop": None})); continue
    z = np.load(f)
    t, ref = tuple(map(float, z["truth_rear"])), tuple(map(float, z["reference_rear"]))
    rel = compose(invert(ref), t)
    print(json.dumps({"name": name, "dx": rel[0], "dy": rel[1], "dyaw": math.atan2(math.sin(rel[2]), math.cos(rel[2]))}))
