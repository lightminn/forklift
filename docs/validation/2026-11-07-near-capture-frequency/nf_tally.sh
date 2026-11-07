#!/bin/bash
# The batch's tally with the tool as merged (#342, 3154eb63); nothing in the batch's folder but tally/ is written.
set -eu
B=/home/projects/forklift/test_runs/near_freq/nf_46d8929d
T=$B/tally
PY=/home/projects/forklift/.venv/bin/python
touch "$T/src/forklift_core/__init__.py" "$T/src/forklift_core/mission/__init__.py"
cd "$T"
$PY tools/ct_survey.py "$B" --json "$T/survey.json" --near-tally "$T/near.json" > /dev/null
$PY - <<EOF
import collections, json
d = json.load(open("$T/near.json"))
print("cells", len(d["cells"]), "session cells", len(d["session_cells"]), "unverified", d["unverified_shapes"], "excluded", d["excluded_attempts"])
for p in d["populations"]:
    print(p["arm"], p["shape"], p["pose_check"], p["reloc_inject"], p["operating_config_sha256"], p["config_assumed_sessions"])
    print(" rates", p["rates"])
    print(" sessions", p["sessions"])
    print(" ended", p["ended_by_reason"], "causes", p["causes"], "classes", p["classes"])
    print(" by_scene", p["by_scene"])
    print(" order", p["reached_by_order_pallet"])
    print(" triggers", p["triggers"])
print(collections.Counter(c["entry_face"] for c in d["cells"]))
print("conditions", collections.Counter((c["condition_sha256"] or "none")[:10] for c in d["cells"]))
print("capture", sum(c["capture_attempts"] for c in d["cells"]), "second", sum(c["waiver_second_calls"] for c in d["cells"]), "rescue", sum(c["waiver_rescues"] for c in d["cells"]))
first, final = collections.Counter(), collections.Counter()
for c in d["cells"]:
    first.update(c["first_call_invalid_by_reason"]); final.update(c["final_invalid_by_reason"])
print("first invalid", dict(first)); print("final invalid", dict(final))
print("recovered same", sum(c["missions_recovered_same_capture"] for c in d["cells"]), "new", sum(c["missions_recovered_new_capture"] for c in d["cells"]))
s = json.load(open("$T/survey.json"))
print("sessions", len(s["sessions"]), "other reasons", s["other_reasons"])
print("infra", [x["session"] for x in s["sessions"] if x.get("infra")])
for x in s["sessions"]:
    bad = [(r["order"], r["start"], r["outcome"], (r["reason"] or "")[:90], r["pallet_id"]) for r in x["rows"] if r["outcome"] != "done"]
    if bad:
        print(x["session"], x["queue"]["stop_reason"], bad)
print("all done", sum(all(r["outcome"] == "done" for r in x["rows"]) for x in s["sessions"]))
EOF
