"""The D batch's table for docs/validation/2026-11-06-unconfirmed-d-batch.md: Delta_int against the distance since its
reference, the truth error beside it, the sessions with a scene contact left out. Run in the batch's folder with
tools/pose_check_dev.py on the path."""
import json, glob, os, sys, math
sys.path.insert(0, ".")
import pose_check_dev as D
recs, exc = [], []
for s in sorted(glob.glob("s/*")):
    n = os.path.basename(s); r = json.load(open(s + "/epoch_1/run/result.json")); pc = r["pose_check"]
    iv = [i for i in pc["contacts"]["intervals"] if "/World/Pallet/" not in i["other"] and "SM_FloorDecal" not in i["other"]]
    (exc if iv else recs).append((n, pc))
print("left out, scene contact:", [(n, [i["other"].split("/")[-1][:20] for i in p["contacts"]["intervals"]][:2]) for n, p in exc])
print("sessions", len(recs))
rows = []
for n, pc in recs:
    rows += [dict(r, n=n) for r in D.delta_rows(pc)]
def q(v, p):
    v = sorted(v); return v[min(len(v) - 1, int(p * len(v)))] if v else 0
print("bin sess n | delta_m p50 p95 max | delta_rad p95 max | truth_m p95 max | truth_rad max")
for a, b in D.DISTANCE_BINS:
    w = [r for r in rows if a <= r["since_m"] < b]
    if w:
        print(a, b, len({r["n"] for r in w}), len(w), "|", *[round(q([r["delta_m"] for r in w], p), 3) for p in (.5, .95, 1)], "|", *[round(q([r["delta_rad"] for r in w], p), 3) for p in (.95, 1)], "|", *[round(q([r["truth_m"] for r in w], p), 3) for p in (.95, 1)], "|", round(max(r["truth_rad"] for r in w), 3))
ratio = sorted(r["delta_m"] / r["since_m"] for r in rows if r["since_m"] >= 5)
print("delta_m per metre of path, since >= 5 m: p50 %.4f p95 %.4f max %.4f" % (q(ratio, .5), q(ratio, .95), ratio[-1]))
ratio = sorted(r["delta_rad"] / r["since_m"] for r in rows if r["since_m"] >= 5)
print("delta_rad per metre: p50 %.5f p95 %.5f max %.5f" % (q(ratio, .5), q(ratio, .95), ratio[-1]))
gated = [r for r in rows if r["truth_m"] > 0.30 or r["truth_rad"] > 0.10]
print("truth wrong samples", len(gated), "imprecise", sum(1 for r in rows if r["truth_m"] > 0.10 or r["truth_rad"] > 0.05))
v = D.dev_values(recs, min_samples=20); print({k: v[k] for k in ("odom", "e_still", "e_slam", "flags")}); print(v["basis"]["e_still"], v["basis"]["e_slam"], v["basis"]["odom"]["max_e_p"], v["basis"]["odom"]["max_e_theta"], v["basis"]["odom"]["windows_in_contact"])
st = sorted(((s["max_rad"], s["max_m"], s["mission"], s["near"], round(s["t0"], 1), round(s["t1"] - s["t0"], 1), n) for n, p in recs for s in p["raw"]["stands"]), reverse=True)[:6]
print("still top", [(round(a, 4), round(b, 3), c, d, e, f, g) for a, b, c, d, e, f, g in st])
print("still over 0.03 rad", sum(1 for n, p in recs for s in p["raw"]["stands"] if s["max_rad"] > 0.03), "of", sum(len(p["raw"]["stands"]) for n, p in recs))
nomiss = sorted((round(s["max_rad"], 4), round(s["max_m"], 3)) for n, p in recs for s in p["raw"]["stands"] if not s["mission"])[-3:]
print("still top outside missions", nomiss)
caps = sum(len(p["stand_sets"]) for n, p in recs); print("stand_sets", caps, "skipped", sum(p["stand_sets_skipped"] for n, p in recs), "sets", sum(len(c["sets"]) for n, p in recs for c in p["stand_sets"]))
t = [g for n, p in recs for g in p["raw"]["rematches"]]; print("gens", len(t), "with g", sum(1 for g in t if g.get("g")), "residual ok", sum(1 for g in t if g.get("residual_confirmed")), "truth classes", {c: sum(1 for g in t if (g.get("truth") or {}).get("class") == c) for c in ("success", "imprecise", "wrong", "unknown")})
