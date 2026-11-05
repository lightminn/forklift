"""line_2 delivery docking, round by round, from every riib/simphys record on ws1 (read only).

Per run: identity (URDF, pallet mass, scene file, batch), each docking round's recorded verdict and information, the
run's end. Per round 1 (and 2) match file: the laser's truth pose against the reference pose, the live beam count, and
the information recomputed with the snapshot's match_scans from the recorded seed -- with its weak direction and how
the weak direction's information splits across the reference's faces.
Output: JSON lines on stdout.
"""
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, sys.argv[1])  # a snapshot's src
from forklift_core.localization import scan_match as sm  # noqa: E402

ROOTS = ["/home/projects/forklift/test_runs/riibotics", "/home/projects/forklift/test_runs/simphys"]


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def invert(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def wrap(t):
    return math.atan2(math.sin(t), math.cos(t))


def decompose(ref, live, pose):
    """Information matrix at pose over inliers, its eigen split, and the weak direction's share by face groups."""
    moved = sm.transform_points(live, pose)
    normals = sm._normals(ref)
    j, d = sm._nearest(ref, moved)
    inl = d < 0.05
    n = normals[j[inl]]
    lever = moved[inl] - np.asarray(pose[:2])
    jac = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -lever[:, 1] + n[:, 1] * lever[:, 0]))
    w, v = np.linalg.eigh(jac.T @ jac)
    weak = v[:, 0]
    contrib = (jac @ weak) ** 2  # per inlier share of the weak-direction information
    pts = moved[inl]
    # faces: reference normal direction class (|n_y| > 0.3 = faces with a lateral normal, the rest along the wall)
    lateral = np.abs(n[:, 1]) > 0.3
    top = np.argsort(contrib)[::-1][: max(1, len(contrib) // 10)]
    return {
        "eig": [float(x) for x in w], "weak_vec": [float(x) for x in weak], "inliers": int(inl.sum()),
        "lateral_normal_inliers": int(lateral.sum()),
        "weak_share_lateral": float(contrib[lateral].sum() / max(contrib.sum(), 1e-12)),
        "weak_share_top10pct": float(contrib[top].sum() / max(contrib.sum(), 1e-12)),
        "weak_top_points": [[round(float(p[0]), 2), round(float(p[1]), 2)] for p in pts[top][:12]],
    }


def match_file(path):
    z = np.load(path, allow_pickle=True)
    ref, live = z["reference"], z["live"]
    out = {"live_beams": int(len(live)), "reference_beams": int(len(ref))}
    x = tuple(float(t) for t in z["rear_from_laser"])
    truth, reference_rear = tuple(float(t) for t in z["truth_rear"]), tuple(float(t) for t in z["reference_rear"])
    est = tuple(float(t) for t in z["estimate"])
    # the truth pose of the truck relative to the reference pose, in the reference rear frame
    rel = compose(invert(reference_rear), truth)
    out["truth_rel_reference"] = [rel[0], rel[1], wrap(rel[2])]
    out["estimate_minus_truth"] = [est[0] - truth[0], est[1] - truth[1], wrap(est[2] - truth[2])]
    laser_truth = compose(truth, x)
    out["laser_truth_world"] = list(laser_truth)
    # the match from the recorded seed with the snapshot's code (the first start)
    planned = compose(invert(reference_rear), est)  # scan_docking.dock: R0 = delivery_rear^-1 o estimate_rear
    q0 = compose(compose(invert(x), planned), x)
    res = sm.match_scans(ref, live, initial=q0)
    out["recomputed"] = {"accepted": bool(res.accepted), "reason": res.reason, "min_information": res.min_information,
                         "inlier_fraction": res.inlier_fraction}
    # at the truth pose: the information the scan carries when the match is right
    q_truth = compose(compose(invert(x), rel), x)
    out["at_truth"] = decompose(ref, live, q_truth)
    # the recomputed match against the truth (laser frame): what a rejected match would have given
    out["match_minus_truth_laser"] = [res.pose[0] - q_truth[0], res.pose[1] - q_truth[1], wrap(res.pose[2] - q_truth[2])]
    out["at_match"] = decompose(ref, live, res.pose)
    return out


def scene_of(args):
    f = args.get("scene_file") or ""
    return os.path.basename(f).rsplit(".", 1)[0] if f else (args.get("layout") or "")


rows = 0
for root in ROOTS:
    for res_path in glob.glob(os.path.join(root, "**", "run", "result.json"), recursive=True):
        try:
            d = json.load(open(res_path))
        except (OSError, ValueError):
            continue
        args = d.get("arguments") or {}
        scene = scene_of(args)
        if "line_2" not in scene:
            continue
        dd = d.get("delivery_docking") or {}
        rounds = dd.get("rounds") or []
        run_dir = os.path.dirname(res_path)
        row = {
            "path": run_dir, "scene": scene, "seed": args.get("seed"),
            "urdf": (d.get("forklift_urdf_sha256") or "")[:8],
            "pallet_kg": (d.get("pallet_mass") or {}).get("kg") if isinstance(d.get("pallet_mass"), dict) else None,
            "docking_acceptance": args.get("docking_acceptance"),
            "failure_reason": d.get("failure_reason"), "success": d.get("success"),
            "phase": d.get("phase"), "docking_status": dd.get("status"), "docking_reason": dd.get("reason"),
            "docking_accepted": dd.get("accepted"), "attempt_folder": ".attempt" in run_dir,
            "rounds": [],
        }
        for r in rounds:
            infos = [s.get("min_information") for s in (r.get("start_details") or []) if s.get("min_information") is not None]
            row["rounds"].append({
                "round": r.get("round"), "keep_m": r.get("keep_m"), "accepted": r.get("accepted"), "reason": r.get("reason"),
                "info_min": min(infos) if infos else None, "info_max": max(infos) if infos else None,
                "estimate_vs_truth_at_match": r.get("estimate_vs_truth_at_match"), "retry_match": r.get("retry_match"),
                "goal_error_vs_truth": r.get("goal_error_vs_truth"), "time_s": r.get("time_s"),
            })
        for n in (1, 2):
            mp = os.path.join(run_dir, f"delivery_match_{n}.npz")
            if os.path.exists(mp):
                try:
                    row[f"match_{n}"] = match_file(mp)
                except Exception as exc:  # noqa: BLE001 -- recorded, the scan goes on
                    row[f"match_{n}"] = {"error": repr(exc)}
        print(json.dumps(row, default=float))
        rows += 1
print(json.dumps({"rows": rows}), file=sys.stderr)
