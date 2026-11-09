"""Real-time plan 5.25 wall reading: per wall session, the probe's legs (moved, held, armed, the permission's limit and
reason at the leg's end) and the world.step lengths from the tick trace -- over the whole session and up to the first
leg's end, the latter taken as the first leg's wall duration from the trace's first paced start (an approximation: the
leg starts a little after the trace does, so the window is slightly long). The whole-session figures carry the reading.

    VENV_PY wall_read.py SNAP_DIR R1C_DIR
"""
import json, sys
import numpy as np
from pathlib import Path
sys.path.insert(0, sys.argv[1] + "/sim/isaac")
from tick_trace import COLUMNS
C = {n: i for i, n in enumerate(COLUMNS)}
root = Path(sys.argv[2])
for s in ("r1c_22_wall_L_04", "r1c_23_wall_K_04", "r1c_24_wall_KP_04"):
    p = json.loads((root / s / "probe.json").read_text())
    d = np.load(root / s / "live/epoch_1/run/tick_trace.npz")["data"]
    w = d[:, C["world_end"]] - d[:, C["world_start"]]
    S = d[:, C["S"]] - d[0, C["S"]]
    legs = []
    for a in p["approach"]:
        legs.append({k: a.get(k) for k in ("leg", "wall_s", "moved", "held", "armed")} |
                    {"permit": a.get("permit"), "stop_kind": (a.get("applied") or {}).get("stop_kind")})
    first = legs[0]["wall_s"] if legs else None
    seg = w[S <= first] if first else w
    print(json.dumps({"session": s, "legs": legs, "world_step_max_s_to_first_leg_end": round(float(seg.max()), 3),
                      "world_steps_over_0_25_to_first_leg_end": int((seg > 0.25).sum()),
                      "world_step_max_s_session": round(float(w.max()), 3),
                      "world_steps_over_0_25_session": int((w > 0.25).sum()), "steps": int(len(w))}))
