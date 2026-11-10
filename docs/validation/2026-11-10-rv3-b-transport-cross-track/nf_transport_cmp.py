#!/usr/bin/env python3
"""The ten sessions' P-5 -> D-2 transport on the same measures (docs/validation/2026-11-07-near-capture-frequency.md).

Records only: state.paths.transport, slam.control (t, estimate x y yaw, truth x y yaw), state.samples. The first forward
leg's planned S: the arc before and after its first curvature sign change. usage: nf_transport_cmp.py BATCH OUT.json"""
import bisect
import glob
import json
import math
import os
import sys

SATURATED = 0.355  # the tracker's limit is 0.36 /m
SEEDS = range(100, 110)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read(run, name):
    rows = []
    for path in sorted(glob.glob(f"{run}/spool/{name}/*.jsonl")):
        with open(path) as handle:
            rows += [json.loads(line) for line in handle if line.strip()]
    return rows


def legs_of(path):
    directions = path["directions"]
    cusps = [i - 1 for i in range(1, len(directions)) if directions[i] * directions[i - 1] < 0]  # directions[i] arrives at i
    return cusps, directions


def run_lengths(rows, test):
    """(start, end) of each run of samples for which test holds."""
    out, start = [], None
    for r in rows:
        if test(r):
            start = float(r["time_s"]) if start is None else start
            end = float(r["time_s"])
        elif start is not None:
            out.append((start, end))
            start = None
    if start is not None:
        out.append((start, end))
    return out


def measure(run, mission="m2"):
    state = json.load(open(f"{run}/missions/{mission}/records.json"))["state"]
    result = json.load(open(f"{run}/missions/{mission}/result.json"))["result"]
    top = json.load(open(f"{run}/result.json"))
    # the delivery's straight replaces the transport path at the docking: the plan a finished mission drove is the first
    # one it replaced
    replaced = [e["path"] for e in state.get("paths_replaced") or [] if e.get("leg") == "transport"]
    path = replaced[0] if replaced else state["paths"]["transport"]
    poses, curvature = path["poses"], path["curvatures_inv_m"]
    cusps, directions = legs_of(path)
    t_in = next(t["time_s"] for t in top["transitions"]
                if t["to"] == "transport" and result["started_s"] <= t["time_s"] <= result["ended_s"])
    later = [t["time_s"] for t in top["transitions"] if t["time_s"] > t_in + 1e-6 and t["to"] != "transport"]
    t_out = min(later + [result["ended_s"]])
    rows = sorted(read(run, "slam.control"), key=lambda r: r[0])
    times = [r[0] for r in rows]
    samples = [s for s in read(run, "state.samples") if t_in <= float(s["time_s"]) <= t_out]
    window = rows[bisect.bisect_left(times, t_in): bisect.bisect_right(times, t_out)]
    # the S: the plan's longest leg between cusps, and the truck's longest forward run
    edges = [0, *cusps, len(poses) - 1]
    forward, end_leg = max(zip(edges, edges[1:]), key=lambda e: e[1] - e[0])
    flip = next((i for i in range(forward + 3, end_leg)  # past the cusp, where the plan's curvature sign changes too
                 if curvature[i] * curvature[i - 1] < 0 or (curvature[i] and not curvature[i - 1] and i > forward + 5
                                                            and any(c * curvature[i] < 0 for c in curvature[forward:i]))), None)
    # truth against the path along the first forward leg, from the first forward motion, the index only moving on
    runs = run_lengths(samples, lambda s: s["signed_speed_mps"] > 0.05)
    t_forward = max(runs, key=lambda x: x[1] - x[0])[0] if runs else t_in
    track, j = [], forward
    for r in window:
        if float(r[0]) < t_forward:
            continue
        best, at = math.inf, j
        for i in range(max(forward, j - 3), min(end_leg, j + 40) + 1):
            d = math.hypot(poses[i][0] - r[4], poses[i][1] - r[5])
            if d < best:
                best, at = d, i
        j = at
        track.append((r[0], at, best, wrap(r[6] - poses[at][2])))
    # the leg ends where the truck reaches the next cusp or the path is replaced: past that the distance means nothing
    t_replaced = next((e["time_s"] for e in state.get("paths_replaced") or [] if e.get("leg") == "transport"), math.inf)
    t_end = min(t_replaced, next((t[0] for t in track if t[1] >= end_leg - 2), math.inf))
    leg = [t for t in track if t[0] <= t_end]
    early = [t for t in leg if t[0] <= t_forward + 3.0]
    t_flip = next((t[0] for t in leg if flip is not None and t[1] >= flip), None)
    first = [s for s in samples if t_forward <= float(s["time_s"]) <= min(t_flip or t_out, t_end)]
    second = [s for s in samples if t_flip is not None and t_flip < float(s["time_s"]) <= t_end]

    def arc(part, sign):
        """Longest saturated run of the planned sign: its length and the truth's curvature along it."""
        runs = run_lengths(part, lambda s: s["curvature_inv_m"] * sign >= SATURATED)
        if not runs:
            return {"saturated_s": 0.0, "longest_s": 0.0, "truth_curvature": None}
        a, b = max(runs, key=lambda x: x[1] - x[0])
        w = rows[bisect.bisect_left(times, a): bisect.bisect_right(times, b)]
        s = sum(math.dist(p[4:6], q[4:6]) for p, q in zip(w, w[1:]))
        yaw = sum(wrap(q[6] - p[6]) for p, q in zip(w, w[1:]))
        return {"saturated_s": round(sum(y - x for x, y in runs), 2), "longest_s": round(b - a, 2),
                "truth_curvature": round(yaw / s, 4) if s > 0.2 else None}

    sign = 1.0 if flip is None or curvature[flip - 2] >= 0 else -1.0
    # from the sign change on only: a recovery before it is not one after it
    around = [t for t in leg if t_flip is not None and t_flip <= t[0] <= t_flip + 4.0]
    peak = max(around, key=lambda t: abs(t[3])) if around else None
    after = [t for t in leg if peak is not None and t[0] >= peak[0]]
    back = next((t[0] for t in after if abs(t[3]) <= 0.05), None)

    def truth_speed(a, b):
        """(mean, lowest over 0.2 s) of the truth's rear axle from a to b."""
        w = rows[bisect.bisect_left(times, a): bisect.bisect_right(times, b)]
        hops = [(math.dist(p[4:6], q[4:6]), q[0] - p[0]) for p, q in zip(w[::24], w[24::24])]
        hops = [d / dt for d, dt in hops if dt > 0]
        return (sum(hops) / len(hops), min(hops)) if hops else (None, None)

    speed_second = truth_speed(t_flip, t_end)[0] if t_flip is not None and t_end > t_flip else None
    speed_flip = truth_speed(t_flip - 3.0, t_flip + 3.0)[1] if t_flip is not None else None
    load = None
    if samples:
        s = samples[len(samples) // 4]
        r = rows[min(bisect.bisect_left(times, float(s["time_s"])), len(rows) - 1)]
        dx, dy = s["pallet_position_m"][0] - r[4], s["pallet_position_m"][1] - r[5]
        c, n = math.cos(r[6]), math.sin(r[6])
        load = {"along_m": round(c * dx + n * dy, 4), "lateral_m": round(-n * dx + c * dy, 4),
                "yaw_rad": round(wrap(s["pallet_yaw_rad"] - r[6]), 4), "lift_m": round(s["lift_m"], 4),
                "pallet_tilt_rad": round(s["pallet_tilt_rad"], 4)}
    return {"outcome": result["outcome"], "reason": (result.get("reason") or "")[:40], "path_m": round(path["length_m"], 2),
            "cusps": cusps, "flip": flip, "planned": round(max(abs(c) for c in curvature), 3),
            "first_arc": arc(first, sign), "second_arc": arc(second, -sign),
            "flip_yaw_peak_rad": round(peak[3], 3) if peak else None,
            "back_within_0_05_s": None if peak is None or back is None else round(back - peak[0], 2),
            "cross_first_m": round(max((t[2] for t in leg if t_flip is None or t[0] <= t_flip), default=0.0), 3),
            "cross_second_m": round(max((t[2] for t in leg if t_flip is not None and t[0] > t_flip), default=0.0), 3),
            "cross_at_failure_progress_m": None if flip is None else round(max(
                (t[2] for t in leg if t_flip is not None and t[0] > t_flip and t[1] <= flip + 42), default=0.0), 3),
            "index_reached": leg[-1][1] if leg else None, "end_leg": end_leg,
            "start_yaw_peak_rad": round(max((t[3] for t in early), key=abs, default=0.0), 3),
            "start_cross_m": round(max((t[2] for t in early), default=0.0), 3),
            "speed_at_flip_min": None if speed_flip is None else round(speed_flip, 3),
            "speed_second_mean": None if speed_second is None else round(speed_second, 3), "load": load,
            "stall_replans": len(state.get("stall_replans") or []), "cusp_stalls": len(state.get("cusp_stalls") or []),
            "cusp_replans": len(state.get("cusp_replans") or [])}


def main(batch, out):
    table = {}
    for seed in SEEDS:
        run = f"{batch}/O0_random_m_s{seed}/live/epoch_1/run"
        try:
            table[seed] = measure(run)
        except (OSError, KeyError, StopIteration, ValueError) as exc:
            table[seed] = {"error": repr(exc)[:200]}
    with open(out, "w") as handle:
        json.dump(table, handle, indent=1)
    for seed, r in table.items():
        print(seed, json.dumps(r)[:640])


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
