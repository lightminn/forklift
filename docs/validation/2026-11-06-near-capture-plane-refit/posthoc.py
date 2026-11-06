"""POST HOC, NOT JUDGED (the registered verdict stands). Why the arms failed: which point moved, whether the frame rule
is symmetric, and a null: A0 with only its RANSAC seed changed, which a seed-perturbing arm cannot be expected to beat."""
import dataclasses, json, sys
from collections import Counter
from pathlib import Path
import numpy as np

S = Path(sys.argv[1]); sys.path.insert(0, str(S / "src")); sys.path.insert(0, str(S))
from tools import near_refit_compare as C

R = Path("/home/projects/forklift/test_runs/near_refit")
T = Path("/home/projects/forklift/test_runs")
m = json.loads((R / "manifest.frozen.json").read_text())
replay = {f["id"]: f["arms"] for f in json.loads((R / "replay_a.json").read_text())["frames"]}
near = [e for e in m["entries"] if e["path"]["kind"] == "near_expected"]
out = {}


def frame_stats(pairs):
    """(entry, A, B): how often B is > 5 mm worse than A at a point, how often better, by point class."""
    worse, better, cls, peak, big = 0, 0, Counter(), [], 0
    for entry, a, b in pairs:
        if a["status"] != "valid" or b["status"] != "valid":
            continue
        e0, e1 = C.pose_errors(a, entry["truth"]), C.pose_errors(b, entry["truth"])
        d = {p: e1[p] - e0[p] for p in C.POINTS}
        w = max(d, key=d.get)
        if d[w] > C.POINT_WORSENING_M:
            worse += 1
            cls["wall" if w.startswith("wall") else "along" if w.startswith("along") else "centre"] += 1
            peak.append(max(e1[p] for p in C.POINTS))
        if -min(d.values()) > C.POINT_WORSENING_M:
            better += 1
        big += max(e1[p] for p in C.POINTS) > 0.015
    return {"worse_gt5mm": worse, "better_gt5mm": better, "worse_class": dict(cls),
            "arm_max_error_on_worse_max": max(peak) if peak else None, "frames_over_15mm": big}


for arm in C.CANDIDATES:
    rows = [(e, replay[e["id"]]["A0"], replay[e["id"]][arm]) for e in near]
    s = frame_stats(rows)
    s["flips"] = [(e["id"], b["status"], b["reason"]) for e, a, b in rows if a["status"] == "valid" and b["status"] != "valid"]
    new = []
    for e, a, b in rows:
        if a["status"] != "valid" and b["status"] == "valid":
            err = C.pose_errors(b, e["truth"])
            new.append((e["id"], C.quality_pass(err), round(err["yaw"], 4), round(max(err[p] for p in C.LATERAL), 4),
                        round(max(err[p] for p in C.ALONG), 4), a["reason"]))
    s["new_valid"] = new
    y = [(C.pose_errors(b, e["truth"])["yaw"], C.pose_errors(a, e["truth"])["yaw"]) for e, a, b in rows
         if a["status"] == "valid" and b["status"] == "valid"]
    s["yaw_gt10mrad"] = {"arm": sum(v[0] > 0.01 for v in y), "a0": sum(v[1] > 0.01 for v in y)}
    out[arm] = s

null = {}
cache = {}
for seed in (20260914, 20260915):
    rows = []
    for e in near:
        scene, prior, params = C.load_frame(T, e, cache)
        d, sec = C.detect_path(scene, prior, dataclasses.replace(params, seed=seed), e["path"], "A0")
        rows.append((e, replay[e["id"]]["A0"], C.detection_record(d, sec)))
    s = frame_stats(rows)
    s["valid"] = sum(b["status"] == "valid" for *_, b in rows)
    s["flips"] = sum(a["status"] == "valid" and b["status"] != "valid" for _, a, b in rows)
    null[seed] = s
out["null_A0_other_seed"] = null

e3 = json.loads((R / "e3.json").read_text())
fp = Counter()
for r in e3["rows"]:
    if r["valid"]:
        fp[(r["arm"], r["level"], r["prior"], r["kind"], r["lower"])] += 1
out["negatives_valid"] = {str(k): v for k, v in sorted(fp.items())}
e4 = json.loads((R / "e4.json").read_text())
out["toy"] = [{k: r.get(k) for k in ("capture", "prior", "arm", "valid", "frames", "centre_std_m")} for r in e4["rows"] if r["kind"] != "wall"]
out["toy_wall_valid"] = [r for r in e4["rows"] if r["kind"] == "wall" and r["valid"]]
seeds = json.loads((R / "seeds.json").read_text())
out["seed_frames"] = {k: {a: v[a]["failures"] for a in C.ARMS} for k, v in seeds["frames"].items()}
out["seed_e1"] = {a: seeds["e1"][a]["failures"] for a in C.ARMS}
(R / "posthoc.json").write_text(json.dumps(out, indent=1, default=str))
print(json.dumps({k: v for k, v in out.items() if k != "toy"}, indent=0, default=str)[:7000])
