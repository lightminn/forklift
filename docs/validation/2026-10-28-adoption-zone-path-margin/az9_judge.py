"""§8b judge (docs/plans/2026-10-28-adoption-zone-check.md): run list -> epoch -> mission_results -> records.

    python3 az9_judge.py RUNS.jsonl

Every epoch directory must hold a result.json, every records.json must belong to a reported mission, and every
mission a session reports must have a records.json with adoption checks; a missing record, a mission with no
check, or a check without truth is undetermined. A check whose axis error to the mission pallet's truth is <= 0.50 m
is about the instructed pallet (only these count toward stage reach); above that it must lie within 0.50 m of another
object's position (another pallet's start truth or a destination centre) to count as another object, else it is
undetermined. A refusal of an instructed-pallet check is a false refusal.
"""
import json
import sys
from pathlib import Path

IDENT = 0.50


def axis_err(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


stages = {}
reached = {}
false_ref = other_ref = other_acc = 0
undet = []
for line in Path(sys.argv[1]).read_text().splitlines():
    entry = json.loads(line)
    name = entry["name"]
    run_dir = Path(entry["argv"][-1])
    scene = json.loads(Path(entry["env"]["SCENE_FILE"]).read_text())
    objects = {p["pallet_id"]: (p["truth"]["x_m"], p["truth"]["y_m"]) for p in scene["pallets"]}
    objects.update({d["destination_id"]: (d["x_m"], d["y_m"]) for d in scene["destinations"]})
    log = (run_dir / "live.log").read_text(errors="replace").splitlines()
    slam = sorted({x.strip() for x in log if x.startswith("slam: ")})
    epochs = sorted(p for p in run_dir.glob("live/epoch_*") if p.is_dir())
    if not epochs:
        undet.append((name, "-", "no epoch"))
        print(f"## {name}: NO EPOCH")
        continue
    results = []
    for ep in epochs:
        if (ep / "run" / "result.json").exists():
            results.append(ep / "run" / "result.json")
        else:
            undet.append((name, ep.name, "no result.json"))
            print(f"## {name} {ep.name}: NO RESULT")
    for res_p in results:
        res = json.loads(res_p.read_text())
        a = res["arguments"]
        epoch = res_p.parents[1].name
        mr = res.get("mission_results") or []
        print(f"## {name} {epoch} margin={a.get('adoption_zone_margin_m')} bundle={a.get('slam_map_bundle')} "
              f"enc={a.get('encoder_model')} recog={a.get('recognition')}")
        print(f"   slam: {slam}")
        print("   ends:", [(m["mission_id"], m["pallet_id"], m["destination_id"], m["outcome"], m["reason"]) for m in mr])
        if not mr:
            undet.append((name, epoch, "no mission_results"))
        reported = {m["mission_id"] for m in mr}
        for extra in sorted(p.parent.name for p in (res_p.parent / "missions").glob("*/records.json")):
            if extra not in reported:
                undet.append((name, epoch, extra, "records.json without a mission_results entry"))
        for m in mr:
            key = (name, epoch, m["mission_id"])
            rec = res_p.parent / "missions" / m["mission_id"] / "records.json"
            if not rec.exists():
                undet.append(key + ("no records.json",))
                continue
            checks = ((json.loads(rec.read_text())["state"].get("adoption_zone")) or {}).get("checks") or []
            if not checks:
                undet.append(key + ("no checks",))
                continue
            for c in checks:
                t = c.get("truth")
                stg = c["stage"]
                if t is None:
                    undet.append(key + (f"{stg}: no truth",))
                    continue
                err = axis_err(c["site"], t)
                if err <= IDENT:
                    s = stages.setdefault(stg, [0, 0.0, -9.0])
                    s[0] += 1
                    s[1] = max(s[1], err)
                    s[2] = max(s[2], c["edge_m"])
                    reached.setdefault(stg, set()).add(key)
                    if not c["inside"]:
                        false_ref += 1
                        print(f"   FALSE_REFUSAL {m['mission_id']} {stg} err={err:.3f} edge={c['edge_m']:.4f}")
                    continue
                near = [k for k, xy in objects.items() if k != m["pallet_id"] and axis_err(c["site"], xy) <= IDENT]
                if not near:
                    undet.append(key + (f"{stg}: err {err:.3f} matches no other object",))
                    continue
                if c["inside"]:
                    other_acc += 1
                else:
                    other_ref += 1
                print(f"   OTHER_OBJECT {m['mission_id']} {stg} err={err:.3f} near={near} inside={c['inside']}")
print("instructed-pallet checks per stage n / max axis err / max edge_m:",
      {k: (v[0], round(v[1], 4), round(v[2], 4)) for k, v in stages.items()})
print("missions reaching (instructed pallet):", {k: len(v) for k, v in reached.items()})
print("false_refusal", false_ref, "other_object_refused", other_ref, "other_object_accepted", other_acc)
print("undetermined", len(undet), undet)
