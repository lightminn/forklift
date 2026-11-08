import json, sys, collections
from pathlib import Path
for run in sys.argv[1:]:
    run = Path(run)
    mx = collections.defaultdict(float); n = collections.Counter(); tot = collections.defaultdict(float)
    for line in open(run / "live_log.jsonl"):
        r = json.loads(line)
        t = r.get("timers")
        if isinstance(t, dict):
            for name, v in t.items():
                if v[2] is not None:
                    mx[name] = max(mx[name], v[2]); n[name] += v[1]; tot[name] += v[0]
    print("==", run.parts[-4])
    for name in sorted(mx, key=lambda k: -mx[k])[:28]:
        if mx[name] > 0.03:
            print(f"   {name:48s} max {mx[name]*1000:7.1f} ms  calls {n[name]:7d}  per call {1000*tot[name]/max(n[name],1):7.2f} ms")
