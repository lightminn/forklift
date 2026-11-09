"""Whole-session world.step lengths (tick trace) of the R1 mark and wall sessions, for the 이상 5 scope."""
import json, sys
import numpy as np
from pathlib import Path
sys.path.insert(0, sys.argv[1] + "/sim/isaac")
from tick_trace import COLUMNS
C = {n: i for i, n in enumerate(COLUMNS)}
for root in sys.argv[2:]:
    for s in sorted(p.name for p in Path(root).glob("r1c_*")):
        d = np.load(Path(root) / s / "live/epoch_1/run/tick_trace.npz")["data"]
        w = d[:, C["world_end"]] - d[:, C["world_start"]]
        print(json.dumps({"root": root.split("/")[-1], "session": s, "steps": int(len(w)), "max_s": round(float(w.max()), 3),
                          "over_0_25": int((w > 0.25).sum()), "over_0_5": int((w > 0.5).sum())}))
