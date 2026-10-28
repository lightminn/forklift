"""F1 Isaac check judge (docs/plans/2026-10-28-far-target-binding.md section 4).

    python3 f1_judge.py feasible/RUNS.jsonl blocked/RUNS.jsonl

Walks run list -> epoch -> mission_results -> records. Per mission: outcome, the zone sources its adoption
checks used, its far_expected records (candidate count, inside / outside, and which scene object each candidate
centre lies within 0.50 m of, in world coordinates via the attempt's arrived_pose), and its adoption checks
judged by the 0.50 m identity rule (instructed pallet / other object / undetermined).
"""
import json
import math
import sys
from pathlib import Path

IDENT = 0.50


def axis_err(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def to_world(pose, xy):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * xy[0] - s * xy[1], pose[1] + s * xy[0] + c * xy[1])


undet = []
for runs in sys.argv[1:]:
    for line in Path(runs).read_text().splitlines():
        entry = json.loads(line)
        name = entry["name"]
        run_dir = Path(entry["argv"][-1])
        scene = json.loads(Path(entry["env"]["SCENE_FILE"]).read_text())
        start = {p["pallet_id"]: (p["truth"]["x_m"], p["truth"]["y_m"]) for p in scene["pallets"]}
        dests = {d["destination_id"]: (d["x_m"], d["y_m"]) for d in scene["destinations"]}
        log = (run_dir / "live.log").read_text(errors="replace").splitlines()
        slam = sorted({x.strip() for x in log if x.startswith("slam: ")})
        epochs = sorted(p for p in run_dir.glob("live/epoch_*") if p.is_dir())
        if not epochs:
            undet.append((name, "no epoch"))
            print(f"## {name}: NO EPOCH")
            continue
        for ep in epochs:
            res_p = ep / "run" / "result.json"
            if not res_p.exists():
                undet.append((name, ep.name, "no result.json"))
                print(f"## {name} {ep.name}: NO RESULT")
                continue
            res = json.loads(res_p.read_text())
            a = res["arguments"]
            mr = res.get("mission_results") or []
            print(f"## {name} {ep.name} recog={a.get('recognition')} fce={a.get('far_capture_expected')} "
                  f"azc={a.get('adoption_zone_check')} m={a.get('adoption_zone_margin_m')} {slam}")
            # where each pallet sits when a mission starts: the start truth, moved to the destination when delivered
            where = dict(start)
            for m in mr:
                key = (name, ep.name, m["mission_id"])
                rec = res_p.parent / "missions" / m["mission_id"] / "records.json"
                print(f"   {m['mission_id']} {m['pallet_id']}->{m['destination_id']} {m['outcome']} {m['reason']}")
                objects = {k: v for k, v in where.items()}
                if not rec.exists():
                    undet.append(key + ("no records.json",))
                    continue
                st = json.loads(rec.read_text())["state"]
                checks = (st.get("adoption_zone") or {}).get("checks") or []
                sources = sorted({c.get("zone_source") for c in checks})
                for att in st.get("observation_attempts") or []:
                    fe = att.get("far_expected")
                    if not fe:
                        continue
                    pose = att.get("arrived_pose")
                    obs = att.get("pocket_observation") or {}
                    desc = []
                    for c in fe.get("candidates") or []:
                        w = to_world(pose, c["centre_m"]) if pose else None
                        near = [k for k, xy in objects.items() if w and axis_err(w, xy) <= IDENT]
                        desc.append(f"inside={c['inside']} near={near} w=({w[0]:.2f},{w[1]:.2f})" if w else
                                    f"inside={c['inside']} (no pose)")
                    print(f"      far_expected[{att.get('candidate_index')}] src={fe.get('zone_source')} "
                          f"obs={obs.get('status')}:{obs.get('reason')} cands={desc}")
                for c in checks:
                    t = c.get("truth")
                    if t is None:
                        undet.append(key + (f"{c['stage']}: no truth",))
                        continue
                    err = axis_err(c["site"], t)
                    if err <= IDENT:
                        tag = "OK" if c["inside"] else "FALSE_REFUSAL"
                    else:
                        near = [k for k, xy in objects.items() if k != m["pallet_id"] and axis_err(c["site"], xy) <= IDENT]
                        tag = f"OTHER{near}" if near else "UNDETERMINED"
                        if not near:
                            undet.append(key + (f"{c['stage']}: err {err:.3f} matches nothing",))
                    print(f"      check {c['stage']} src={c.get('zone_source')} err={err:.3f} "
                          f"edge={c['edge_m']:.4f} inside={c['inside']} {tag}")
                if not checks:
                    print(f"      no adoption checks (sources {sources})")
                if m["outcome"] == "done":
                    where[m["pallet_id"]] = dests[m["destination_id"]]
print("undetermined", len(undet), undet)
