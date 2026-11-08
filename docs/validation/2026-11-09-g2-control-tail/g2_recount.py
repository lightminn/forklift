import json, sys
from pathlib import Path
for run in sys.argv[1:]:
    run = Path(run); n50 = n100 = 0; sim0 = sim1 = None
    for line in open(run / "live_log.jsonl"):
        r = json.loads(line)
        t = r.get("timers")
        if isinstance(t, dict):
            m = max((v[2] or 0) for k, v in t.items() if k.startswith("core.") and k not in ("core.job_hold", "core.live_leg", "core.release_gate"))
            # job_hold, live_leg and release_gate wrap the others: count the innermost core.* call only
            n50 += m > 0.05; n100 += m > 0.1
    print(run.parts[-4], "windows with a core call > 50 ms:", n50, "> 100 ms:", n100)
