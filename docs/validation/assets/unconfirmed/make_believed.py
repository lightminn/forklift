"""Inputs of tools/unconfirmed_cost.py run (plan 1.3): believed starts per world and F1's wrong confirmations.
    python make_believed.py SNAP OUT"""
import json, math, sys
from pathlib import Path
SNAP, OUT = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(SNAP / "src"))
from forklift_core.localization.start_reloc import base_from_laser
P = Path("/home/projects/forklift/test_runs/live")
OUT.mkdir(parents=True, exist_ok=True)

def write(name, rows):
    (OUT / name).write_text("".join(json.dumps(r) + "\n" for r in rows))
    print(name, len(rows))

# seed 4: B-P1b's validation rows confirmed by the matcher (first scan set), b = the top candidate's base pose, the
# generation's pool = its kept candidates (laser poses there, base poses here)
s4 = []
for line in open(SNAP / "docs/validation/assets/b-p1b/val3_rows.jsonl"):
    r = json.loads(line)
    if r["valid"] and r["decision"] == "confirmed":
        s4.append({"id": r["id"], "base": list(base_from_laser(r["candidates"][0]["pose"])),
                   "pool": [list(base_from_laser(c["pose"])) for c in r["candidates"]]})
write("s4_starts.jsonl", s4)
# seed 5: B-P2d's sessions started on the matcher's confirmation, b = the session's estimate, the generation's pool =
# the confirming attempt's kept candidates (base poses)
j = json.load(open(SNAP / "docs/validation/assets/b-p2d/judgement.json"))
s5 = []
for r in j["rows"]:
    if r["class"] == "valid" and r["outcome"] == "success":
        sr = json.load(open(P / "bp2d_val/s" / r["id"] / "epoch_1/run/result.json"))["start_reloc"]
        att = [a for a in sr["attempts"] if a["reason"] == "confirmed"][-1]
        s5.append({"id": r["id"], "base": sr["estimate"], "pool": [c["base"] for c in att["candidates"]]})
write("s5_starts.jsonl", s5)

def wrong(stress):
    out = []
    for line in open(stress):
        r = json.loads(line)
        if r.get("reason") != "confirmed" or r.get("base") is None or (r.get("amb_ratio") or 0.0) >= 0.98:
            continue
        t, b = r["truth_base"], r["base"]
        d = math.hypot(b[0] - t[0], b[1] - t[1]); a = abs(math.atan2(math.sin(b[2] - t[2]), math.cos(b[2] - t[2])))
        if d > 0.30 or a > 0.10:
            out.append({"id": r["id"], "base": b, "kind": r["kind"]})
    return out
write("s4_wrong.jsonl", wrong(P / "f1_dev/stress_s4.jsonl"))
write("s5_wrong.jsonl", wrong(P / "f1_s5/stress_s5.jsonl"))
