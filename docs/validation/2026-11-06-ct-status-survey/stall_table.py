"""(가)1 stall events (exploratory, records only): for each return_home stall replan, the last slam.control row strictly
before the event (before its release/replan rewrites the row) gives the estimate and the truth; both are expressed
against the return path's final pose (the home goal) in the goal frame (along, lateral, yaw), with est - truth, the
follower's recorded errors and the mission's final truth home error."""
import glob, json, math, sys
import numpy as np

out = sys.argv[1]
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def in_goal(p, g):
    c, s = math.cos(g[2]), math.sin(g[2]); dx, dy = p[0] - g[0], p[1] - g[1]
    return c * dx + s * dy, -s * dx + c * dy, wrap(p[2] - g[2])
print("session mission t | follower pos yaw | est-goal along lat yaw | truth-goal along lat yaw | est-truth yaw | final truth home pos yaw")
n = 0
for sess in sorted(glob.glob(f"{out}/*_s[0-9]")):
    run = f"{sess}/live/epoch_1/run"
    mds = sorted(glob.glob(f"{run}/missions/m*"))
    events = []
    for md in mds:
        st = json.load(open(f"{md}/records.json"))["state"]
        ev = [e for e in st.get("stall_replans") or [] if e.get("phase") == "return_home"]
        if ev:
            goal = ((st.get("paths") or {}).get("return_home") or {}).get("poses", [None])[-1]
            events.append((md.split("/")[-1], ev, goal, st.get("return_home_error") or {}))
    if not events:
        continue
    ctrl = []
    for f in sorted(glob.glob(f"{run}/spool/slam.control/*.jsonl")):
        ctrl += [json.loads(l) for l in open(f)]
    C = np.array(ctrl, float)
    for mid, ev, goal, rhe in events:
        for e in ev:
            t = float(e["time_s"]); k = int(np.searchsorted(C[:, 0], t, side="left")) - 1
            row = C[k]; est, tru = row[1:4], row[4:7]
            a1 = in_goal(est, goal); a2 = in_goal(tru, goal)
            n += 1
            print(f"{sess.split('/')[-1]} {mid} {t:.1f} | {e.get('position_error_m'):.4f} {e.get('yaw_error_rad'):+.4f} | "
                  f"{a1[0]:+.4f} {a1[1]:+.4f} {a1[2]:+.4f} | {a2[0]:+.4f} {a2[1]:+.4f} {a2[2]:+.4f} | {wrap(est[2]-tru[2]):+.4f} | "
                  f"{rhe.get('position_m'):.4f} {rhe.get('yaw_rad'):+.4f} | row_dt {t-row[0]:.3f}")
print("events", n)
