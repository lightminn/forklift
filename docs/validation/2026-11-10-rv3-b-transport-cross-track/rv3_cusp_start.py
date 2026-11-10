#!/usr/bin/env python3
"""Where the forward leg's start error comes from: at the reverse leg's end (the runner's leg switch at the first cusp) and at forward start,
truth and control-estimate offsets from the plan, the SLAM estimate-truth gap over the transport, and commands.
usage: rv3_cusp_start.py SESSIONS_DIR OUT.json"""
import bisect, glob, json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nf_transport_cmp import read, wrap, run_lengths, legs_of

def offsets(poses, i, x, y, yaw):
    px, py, pyaw = poses[i]
    c, s = math.cos(pyaw), math.sin(pyaw)
    dx, dy = x - px, y - py
    return {"along_m": round(c * dx + s * dy, 4), "lateral_m": round(-s * dx + c * dy, 4), "yaw_rad": round(wrap(yaw - pyaw), 4)}

def nearest(poses, lo, hi, x, y):
    return min(range(lo, hi + 1), key=lambda i: math.hypot(poses[i][0] - x, poses[i][1] - y))

def measure(run, mission):
    state = json.load(open(f"{run}/missions/{mission}/records.json"))["state"]
    result = json.load(open(f"{run}/missions/{mission}/result.json"))["result"]
    top = json.load(open(f"{run}/result.json"))
    replaced = [e["path"] for e in state.get("paths_replaced") or [] if e.get("leg") == "transport"]
    path = replaced[0] if replaced else state["paths"]["transport"]
    poses = path["poses"]
    cusps, directions = legs_of(path)
    t_in = next(t["time_s"] for t in top["transitions"]
                if t["to"] == "transport" and result["started_s"] <= t["time_s"] <= result["ended_s"])
    later = [t["time_s"] for t in top["transitions"] if t["time_s"] > t_in + 1e-6 and t["to"] != "transport"]
    t_out = min(later + [result["ended_s"]])
    rows = sorted(read(run, "slam.control"), key=lambda r: r[0])
    times = [r[0] for r in rows]
    w = rows[bisect.bisect_left(times, t_in): bisect.bisect_right(times, t_out)]
    samples = [s for s in read(run, "state.samples") if t_in <= float(s["time_s"]) <= t_out]
    gap = [math.hypot(r[1] - r[4], r[2] - r[5]) for r in w]
    ygap = [abs(wrap(r[3] - r[6])) for r in w]
    out = {"t_in": round(t_in, 2), "t_out": round(t_out, 2), "cusps": cusps, "start_pose_vs_truth": offsets(poses, 0, *w[0][4:7]),
           "start_pose_vs_estimate": offsets(poses, 0, *w[0][1:4]),
           "slam_gap_rmse_m": round(math.sqrt(sum(g * g for g in gap) / len(gap)), 4), "slam_gap_max_m": round(max(gap), 4),
           "slam_yaw_gap_max_rad": round(max(ygap), 4)}
    # the first reverse leg ends at cusps[0]: the moment the truck stops reversing (the first forward run start)
    runs = run_lengths(samples, lambda s: s["signed_speed_mps"] > 0.05)
    t_fwd = max(runs, key=lambda x: x[1] - x[0])[0] if runs else None
    # the reverse leg's end: the runner's own leg switch at the first cusp (result.json leg_switches, leg 0 of this
    # transport); the last reversing sample comes about a second earlier, before the creep and brake to the cusp
    switch = next((e for e in top.get("leg_switches") or [] if e.get("phase") == "transport" and e.get("leg") == 0
                   and t_in <= e["time_s"] <= t_out), None)
    rev = run_lengths(samples, lambda s: s["signed_speed_mps"] < -0.05)
    t_rev_end = switch["time_s"] if switch else max((b for a, b in rev if t_fwd is None or b <= t_fwd), default=None)
    out["reverse_end_source"] = "leg_switch" if switch else "last_reversing_sample"
    edges = [0, *cusps, len(poses) - 1]
    forward = max(zip(edges, edges[1:]), key=lambda e: e[1] - e[0])[0]
    for label, t in (("reverse_end", t_rev_end), ("forward_start", t_fwd)):
        if t is None:
            continue
        r = rows[min(bisect.bisect_left(times, t), len(rows) - 1)]
        out[label] = {"t": round(t, 2), "truth_vs_cusp": offsets(poses, forward, *r[4:7]),
                      "estimate_vs_cusp": offsets(poses, forward, *r[1:4]),
                      "estimate_minus_truth": {"dx": round(r[1] - r[4], 4), "dy": round(r[2] - r[5], 4), "dyaw": round(wrap(r[3] - r[6]), 4)}}
    # the reverse leg's commands: saturation and the truth's distance to the reverse leg at its end
    seg = [s for s in samples if t_rev_end is not None and float(s["time_s"]) <= t_rev_end]
    out["reverse_saturated_s"] = round(sum(b - a for a, b in run_lengths(seg, lambda s: abs(s["curvature_inv_m"]) >= 0.355)), 2)
    out["reverse_cross_max_m"] = round(max((min(math.hypot(poses[i][0] - r[4], poses[i][1] - r[5]) for i in range(0, forward + 1))
                                           for r in w if t_rev_end is not None and r[0] <= t_rev_end), default=0.0), 4)
    out["transport_path_start"] = [round(v, 3) for v in poses[0]]
    out["cusp_pose"] = [round(v, 3) for v in poses[forward]]
    out["failure"] = (state.get("tracking_failure") or {})
    return out

def main(root, outp):
    table = {}
    for run in sorted(glob.glob(f"{root}/*/live/epoch_1/run")):
        name = run[len(root) + 1:].split("/")[0]
        for m in sorted(glob.glob(f"{run}/missions/m*/result.json")):
            mission = m.split("/")[-2]
            st = json.load(open(os.path.dirname(m) + "/records.json"))["state"]
            r = json.load(open(m))["result"]
            if (r.get("pallet_id") or st.get("pallet_id")) != "P-5":
                continue
            table[f"{name}/{mission}"] = measure(run, mission)
    json.dump(table, open(outp, "w"), indent=1)
    for k, v in table.items():
        v2 = dict(v); v2.pop("failure", None)
        print(k, json.dumps(v2)[:900])

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
