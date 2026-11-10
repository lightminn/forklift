#!/usr/bin/env python3
"""The tracker's cross-track at the failing tick, reconstructed: the control estimate's distance to the recorded
segment (segment_index -> +1), and the truth's. usage: rv3_fail_point.py RUN MISSION"""
import bisect, json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nf_transport_cmp import read

def seg_dist(p, a, b):
    ax, ay = a[0], a[1]; bx, by = b[0], b[1]
    vx, vy = bx - ax, by - ay
    u = max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / (vx * vx + vy * vy)))
    return math.hypot(p[0] - ax - u * vx, p[1] - ay - u * vy)

run, mission = sys.argv[1], sys.argv[2]
tf = json.load(open(f"{run}/missions/{mission}/records.json"))["state"]["tracking_failure"]
poses = tf["path"]["poses"]
i = tf["tracking"]["segment_index"]
rows = sorted(read(run, "slam.control"), key=lambda r: r[0])
times = [r[0] for r in rows]
k = bisect.bisect_right(times, tf["time_s"]) - 1
r = rows[k]
out = {"t_row": round(r[0], 3), "t_fail": round(tf["time_s"], 3), "segment": i,
       "estimate_to_segment_m": round(seg_dist(r[1:3], poses[i], poses[i + 1]), 4),
       "truth_to_segment_m": round(seg_dist(r[4:6], poses[i], poses[i + 1]), 4),
       "estimate_minus_truth_m": round(math.hypot(r[1] - r[4], r[2] - r[5]), 4),
       "rear_pose_record": [round(v, 3) for v in tf["rear_pose"]], "truth": [round(v, 3) for v in r[4:7]],
       "estimate": [round(v, 3) for v in r[1:4]], "path_start": [round(v, 3) for v in poses[0]],
       "path_cusps": tf["cusp_indices"], "cusp_poses": [[round(v, 3) for v in poses[c]] for c in tf["cusp_indices"]],
       "flip_region": [round(v, 3) for v in poses[119]], "final": [round(v, 3) for v in poses[-1]]}
print(json.dumps(out, indent=1))
