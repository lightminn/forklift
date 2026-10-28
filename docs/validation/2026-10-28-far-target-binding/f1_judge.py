"""F1 Isaac check judge (docs/plans/2026-10-28-far-target-binding.md section 4).

    python3 f1_judge.py feasible/RUNS.jsonl blocked/RUNS.jsonl

Walks run list -> epoch -> mission_results -> records. Per mission: outcome, its far_expected records and its
adoption checks, both judged by the 0.50 m identity rule (instructed pallet / other object / undetermined).
A far candidate's centre_m is the estimated pallet centre in base_link at the capture; it goes to world through
the capture's control pose (capture_control.control_rear moved forward by |rear_axle_offset_m| to base_link),
the frame the adoption check uses. An outside candidate of an attempt that ended no_expected_candidate is an F1
refusal: false when it lies within 0.50 m (axis) of the instructed pallet's truth (the mission's adoption-check
truth, else where it was put down), correct when it lies within 0.50 m of another pallet or destination,
otherwise undetermined. The F1 tally is printed apart from the adoption-check tally.
"""
import json
import math
import sys
from pathlib import Path

IDENT = 0.50


def axis_err(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def base_pose(rear, offset):
    """base_link in world from the capture's control rear-axle pose: the rear axle is offset behind base_link."""
    return (rear[0] + offset * math.cos(rear[2]), rear[1] + offset * math.sin(rear[2]), rear[2])


def to_world(pose, xy):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * xy[0] - s * xy[1], pose[1] + s * xy[0] + c * xy[1])


undet = []
f1_false, f1_undet = [], []
f1_correct = 0
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
                # the instructed pallet's truth: the adoption checks' (exact), else where it was put down
                target = checks[0]["truth"] if checks and checks[0].get("truth") is not None else where[m["pallet_id"]]
                offset = abs(float(a["rear_axle_offset_m"]))
                for att in st.get("observation_attempts") or []:
                    fe = att.get("far_expected")
                    if not fe:
                        continue
                    rear = (att.get("capture_control") or {}).get("control_rear")
                    pose = None if rear is None else base_pose(rear, offset)
                    obs = att.get("pocket_observation") or {}
                    refused = obs.get("status") == "no_pallet" and obs.get("reason") == "no_expected_candidate"
                    desc = []
                    for c in fe.get("candidates") or []:
                        if pose is None:
                            if refused and not c["inside"]:
                                f1_undet.append(key + (att.get("candidate_index"), "no capture_control"))
                            desc.append(f"inside={c['inside']} (no pose)")
                            continue
                        w = to_world(pose, c["centre_m"])
                        err = axis_err(w, target)
                        near = [k for k, xy in objects.items() if k != m["pallet_id"] and axis_err(w, xy) <= IDENT]
                        tag = ""
                        if refused and not c["inside"]:
                            if err <= IDENT:
                                tag = " F1_FALSE_REFUSAL"
                                f1_false.append(key + (att.get("candidate_index"), round(err, 4)))
                            elif near:
                                tag = " F1_CORRECT_REFUSAL"
                                f1_correct += 1
                            else:
                                tag = " F1_UNDETERMINED"
                                f1_undet.append(key + (att.get("candidate_index"), round(err, 4)))
                        desc.append(f"inside={c['inside']} err={err:.3f} near={near} w=({w[0]:.2f},{w[1]:.2f}){tag}")
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
print("adoption checks: undetermined", len(undet), undet)
print("F1 refusals (no_expected_candidate, outside candidates): false", len(f1_false), f1_false,
      "correct", f1_correct, "undetermined", len(f1_undet), f1_undet)
