import json, sys
from pathlib import Path
IDENT = 0.50
rows = []; stages = {}; false_ref = correct_ref = undet = 0; reached = {}
for root in sys.argv[1:]:
    for res_p in sorted(Path(root).glob("*/live/epoch_*/run/result.json")):
        res = json.loads(res_p.read_text()); a = res["arguments"]
        name = res_p.parents[3].name
        mr = res.get("mission_results") or []
        ends = [(m.get("mission_id") or m.get("id"), m.get("status") or m.get("outcome"), m.get("reason") or m.get("failure_reason")) for m in mr] if isinstance(mr, list) else mr
        print(f"## {name} margin={a.get('adoption_zone_margin_m')} bundle={'Y' if a.get('slam_map_bundle') else 'N'} enc={a.get('encoder_model')} recog={a.get('recognition')}")
        print("   ends:", json.dumps(ends)[:600])
        for rec in sorted((res_p.parent / "missions").glob("*/records.json")):
            st = json.loads(rec.read_text())["state"]
            checks = (st.get("adoption_zone") or {}).get("checks") or []
            for c in checks:
                t = c.get("truth")
                if t is None:
                    undet += 1; print("   UNDET", rec.parent.name, c["stage"]); continue
                err = max(abs(c["site"][0]-t[0]), abs(c["site"][1]-t[1]))
                stg = c["stage"]; s = stages.setdefault(stg, [0, 0.0, -9.0]); s[0] += 1; s[1] = max(s[1], err); s[2] = max(s[2], c["edge_m"])
                reached.setdefault(stg, set()).add((name, rec.parent.name))
                if not c["inside"]:
                    if err <= IDENT: false_ref += 1; tag = "FALSE_REFUSAL"
                    else: correct_ref += 1; tag = "refused_other"
                    print(f"   {tag} {rec.parent.name} {stg} err={err:.3f} edge={c['edge_m']:.3f} site={c['site']} truth={t}")
                elif err > IDENT:
                    print(f"   ACCEPTED_FAR {rec.parent.name} {stg} err={err:.3f} edge={c['edge_m']:.3f}")
print("stages n/max_err/max_edge:", {k: (v[0], round(v[1], 3), round(v[2], 3)) for k, v in stages.items()})
print("missions reaching:", {k: len(v) for k, v in reached.items()})
print("false_refusal", false_ref, "refused_other", correct_ref, "undetermined", undet)
