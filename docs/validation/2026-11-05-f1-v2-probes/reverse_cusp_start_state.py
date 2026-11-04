import json, numpy as np, collections
for arm in ("O", "F1"):
    A = json.load(open(f"c_{arm}_a/run/f1v2_cusp_probe.json")); R = json.load(open(f"c_{arm}_r/run/f1v2_cusp_probe.json"))
    a = {t["id"]: t for t in A["trials"]}; r = {t["id"]: t for t in R["trials"]}
    ta = np.load(f"c_{arm}_a/run/f1v2_cusp_ticks.npz"); tr = np.load(f"c_{arm}_r/run/f1v2_cusp_ticks.npz"); cols = list(ta["columns"])
    d = sorted(((abs(a[i]["e_stop_rad"] - r[i]["e_stop_rad"]), i) for i in a), reverse=True)
    print(arm, "n>0.0005", sum(x > 0.0005 for x, _ in d), "by combo",
          collections.Counter("/".join(i.split("/")[-2:]) for x, i in d if x > 0.0005))
    print("  median diff", np.median([x for x, _ in d]))
    for x, i in d[:4]:
        A0 = ta["t%04d" % a[i]["index"]]; R0 = tr["t%04d" % r[i]["index"]]
        s = [cols.index("steer_l"), cols.index("steer_r")]
        print("  ", i[-44:], round(x, 5), "start off mm", round(a[i]["start_offset_m"] * 1000, 3),
              round(r[i]["start_offset_m"] * 1000, 3), "settle", a[i]["settle"]["steps"], r[i]["settle"]["steps"],
              "steer0", np.round(A0[0, s], 5), np.round(R0[0, s], 5), "ticks", a[i]["ticks"], r[i]["ticks"],
              "e", round(a[i]["e_stop_rad"], 4), round(r[i]["e_stop_rad"], 4))
