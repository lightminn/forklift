"""Live (Isaac) records: does slam_toolbox's TF message filter drop scans there too, and does a drop coincide with a
failed bridge reply / slam_link_failed? Scans every bridge folder holding a slam_toolbox.log under the roots given,
except the offline replay folders (slam_online, slam_cost_*). Prints one JSON line per bridge folder with a drop or a
failed reply, then a summary.
    python3 tf_drop_live.py ROOT...
"""
import json
import os
import re
import sys

DROP = re.compile(r"Message Filter dropping message: frame '([^']+)' at time ([0-9.]+) for reason '([^']+)'")
SKIP = ("slam_online", "slam_cost_", "/s3", "/s2_")
total = with_drop = with_fail = both = 0
mode_count = {}
for root in sys.argv[1:]:
    for d, _, files in os.walk(root):
        if "slam_toolbox.log" not in files or any(s in d for s in SKIP):
            continue
        total += 1
        log = open(os.path.join(d, "slam_toolbox.log"), errors="replace").read()
        drops = [(m.group(1), float(m.group(2)), m.group(3)) for m in DROP.finditer(log)]
        mode = "localization" if "localization_slam_toolbox_node" in log else (
            "sync" if "sync_slam_toolbox_node" in log else "other")
        mode_count[mode] = mode_count.get(mode, 0) + 1
        failed = []
        rec = os.path.join(d, "records.jsonl")
        if os.path.isfile(rec):
            for line in open(rec, errors="replace"):
                if '"failed"' in line:
                    try:
                        r = json.loads(line)
                    except ValueError:
                        continue
                    if r.get("status") == "failed":
                        failed.append({"scan_id": r.get("scan_id"), "stamp_s": r.get("stamp_s"),
                                       "wall_s": r.get("wall_s")})
        link_failed = 0
        epoch = os.path.dirname(d)
        for f in os.listdir(epoch):
            p = os.path.join(epoch, f)
            if os.path.isfile(p) and (f.endswith(".log") or f.endswith(".jsonl")) and os.path.getsize(p) < 200_000_000:
                try:
                    link_failed += open(p, errors="replace").read().count("slam_link_failed")
                except OSError:
                    pass
        with_drop += bool(drops)
        with_fail += bool(failed)
        both += bool(drops) and bool(failed)
        if drops or failed or link_failed:
            print(json.dumps({"dir": d, "mode": mode, "drops": drops[:5], "n_drops": len(drops),
                              "failed_replies": failed[:5], "n_failed": len(failed),
                              "slam_link_failed_mentions": link_failed}))
print(json.dumps({"summary": {"bridge_dirs": total, "modes": mode_count, "with_drop": with_drop,
                              "with_failed_reply": with_fail, "drop_and_failed": both}}))
