#!/usr/bin/env python3
"""rv3's P-5 -> D-2 transports on #346's measures (nf_transport_cmp.measure, unchanged).
usage: rv3_transport_cmp.py SESSIONS_DIR OUT.json"""
import glob, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nf_transport_cmp import measure

def main(root, out):
    table = {}
    for run in sorted(glob.glob(f"{root}/*/live/epoch_1/run")):
        name = run[len(root) + 1:].split("/")[0]
        for m in sorted(glob.glob(f"{run}/missions/m*/result.json")):
            mission = m.split("/")[-2]
            st = json.load(open(os.path.dirname(m) + "/records.json"))["state"]
            r = json.load(open(m))["result"]
            if (r.get("pallet_id") or st.get("pallet_id")) != "P-5":
                continue
            try:
                table[f"{name}/{mission}"] = measure(run, mission)
            except (OSError, KeyError, StopIteration, ValueError) as exc:
                table[f"{name}/{mission}"] = {"error": repr(exc)[:200]}
    json.dump(table, open(out, "w"), indent=1)
    for k, r in table.items():
        print(k, json.dumps(r)[:700])

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
