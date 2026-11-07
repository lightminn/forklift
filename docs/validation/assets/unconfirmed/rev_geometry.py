import glob, json, math, os, sys
sys.path[:0] = [sys.argv[1] + "/src", sys.argv[1] + "/tools"]
import forklift_core.planning.pallet_mission as pm
import forklift_core.planning.hybrid_astar as ha
s = json.load(open(sys.argv[2] + "/rev/scenario.json"))
R = ha.Rectangle if hasattr(ha, "Rectangle") else pm.Rectangle
F = ha.Footprint if hasattr(ha, "Footprint") else pm.Footprint
B = ha.Bounds if hasattr(ha, "Bounds") else pm.Bounds
rects = [R(**{k: p["rectangle"][k] for k in ("x_m", "y_m", "length_m", "width_m", "yaw_rad")}) for p in s["props"]]
bounds = B(**s["bounds"])
hull = F(1.29, 0.17, 0.36)
ck = ha.make_checker(rects, hull, bounds, None)
ck_b = ha.make_checker([], hull, bounds, None)
def rear(b):
    return (b[0] - 0.34 * math.cos(b[2]), b[1] - 0.34 * math.sin(b[2]), b[2])
def check(p):
    return {"props0": ck.free(p, 0.0), "props.1": ck.free(p, 0.1), "bounds0": ck_b.free(p, 0.0), "bounds.1": ck_b.free(p, 0.1)}
A = sys.argv[1] + "/docs/validation/assets/unconfirmed/"
tail = {json.loads(l)["id"]: json.loads(l) for f in (A + "tail_starts.jsonl", sys.argv[2] + "/topup_starts.jsonl") for l in open(f)}
dev = {json.loads(l)["id"]: json.loads(l) for l in open(A + "dev_starts.jsonl")}
res = {}
for f in glob.glob(sys.argv[2] + "/rev/t_*"):
    i = os.path.basename(f)[2:]
    p = json.load(open(f + "/probe.json")); r = json.load(open(f + "/epoch_1/run/result.json"))
    res["t_" + i] = (tail[i]["kind"], tail[i]["start_pose"], bool(r.get("scene_damaged")), (p["reverse"].get("mission") or {}).get("result"), p["reverse"].get("blocked"))
for i, v in dev.items():
    r = json.load(open(f"{sys.argv[2]}/d1/{i}/epoch_1/run/result.json")); p = json.load(open(f"{sys.argv[2]}/d1/{i}/probe.json"))
    sd = r.get("scene_damaged")
    res["d_" + i] = (v["kind"], v["start_pose"], bool(sd and sd.get("t", 9) < 1), (p["reverse"].get("mission") or {}).get("result"), p["reverse"].get("blocked"))
agree = {}
for k, (kind, pose, dmg, mres, blocked) in sorted(res.items()):
    c = check(rear(pose))
    pred = not (c["props0"] and c["bounds0"])
    agree[(pred, dmg)] = agree.get((pred, dmg), 0) + 1
    if pred != dmg:
        print("MISMATCH", k, kind, [round(v, 2) for v in pose], c, dmg)
print("start check predicted_damage vs damaged:", agree)
print("-- invalid_start")
adv = json.load(open(sys.argv[2] + "/rev/advice.json"))
for k, v in adv.items():
    kind, pose, dmg, mres, blocked = res[k]
    fr = tuple(v["advice"]["from_rear"])
    st = rear(pose)
    print(k, kind, "blocked", blocked, "moved", round(math.hypot(fr[0]-st[0], fr[1]-st[1]), 2), "from", [round(a, 2) for a in fr], "at_stop", check(fr), "at_start", check(st), "blocked_at", v["advice"]["blocked_at_m"])
