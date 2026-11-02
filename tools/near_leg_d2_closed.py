"""Near-leg D2' part 3: the closed-loop rollout of the failing reverse leg from the recorded release -- the tracker on
the whole recorded path (its own leg switch past the cusp), the runner's command chain (ackermann_command, creep,
slew_cap_stop with accel 0.8 and the stopping model's 1.5 m/s^2, steering command rate 1.0 rad/s) and the part-2
plant (wheel steering lag 0.17 s, curvature from the wheel angles x 0.94 forward / 1.12 reverse, speed x 0.94).
The tracker reads (A) the truth or (B) the truth plus the recorded estimate error (control - truth, by time).
    python d2_closed.py RUN_DIR SRC"""
import json, math, sys
from dataclasses import replace
import numpy as np
sys.path.insert(0, sys.argv[2])
from forklift_core.control.path_tracking import RearAxlePathTracker, TrackerConfig, AckermannGeometry, ackermann_command
from forklift_core.control.speed_chain import creep, slew_cap_stop

R = sys.argv[1]
s = json.load(open(R + "/result.json")); tf = s["tracking_failure"]; t0 = tf["phase_started_s"]
P = np.array(tf["path"]["poses"]); Dn = np.array(tf["path"]["directions"]); K = np.array(tf["path"]["curvatures_inv_m"])
cfg = TrackerConfig(**tf["config"])
geom = AckermannGeometry(**s["arguments"]["drive_geometry"])
C = np.load(R + "/slam_control.npy"); Z = np.load(R + "/slam_log.npz")
st = [c for c in s["cusp_stalls"] if c.get("disposition") == "released" and c["time_s"] >= t0][-1]
trel = st["time_s"]
TAU, L, GF, GR, SG, DT = 0.17, 0.6645, 0.94, 1.12, 0.94, 1 / 120
ts = np.array([x["time_s"] for x in s["samples"]]); scmd = np.array([x["steering_command_rad"] for x in s["samples"]])


def at(tt):
    return int(np.argmin(np.abs(C[:, 0] - tt)))


def run(feed):
    i = at(trel)
    truth = C[i, 4:7].copy()
    tr = RearAxlePathTracker(P, Dn, K, replace(cfg, cusp_heading_pending_rad=None)); tr.skip_legs(1)
    j = int(np.argmin(np.abs(Z["joint_stamps_s"] - trel)))
    act = Z["steering_rad"][j].astype(float).copy()
    scommand = np.array([np.interp(trel, ts, scmd[:, k]) for k in (0, 1)])
    v = 0.0; prev = 0.0; t = trel; status = "timeout"; a = math.exp(-DT / TAU)
    for _ in range(int(40 / DT)):
        est = truth.copy()
        if feed:
            k = at(t)
            est += C[k, 1:4] - C[k, 4:7]
        cmd = tr.update(est, v, DT)
        if cmd.status in ("arrived", "failed"):
            status = cmd.status + ("" if cmd.failure is None else ":" + cmd.failure)
            break
        drv = ackermann_command(cmd.speed_mps, cmd.curvature_inv_m, geom)
        target = np.asarray(drv.steering_rad)
        ws = creep(cmd.speed_mps, float(np.max(np.abs(target - act))))
        ws = slew_cap_stop(prev, ws, decel_mps2=1.5, accel_mps2=0.8, dt=DT)
        prev = ws
        drv2 = ackermann_command(ws, cmd.curvature_inv_m, geom)
        scommand += np.clip(target - scommand, -1.0 * DT, 1.0 * DT)
        act = a * act + (1 - a) * scommand
        v = drv2.speed_mps * SG
        kap = (math.tan(act[0]) + math.tan(act[1])) / (2 * L) * (GF if v >= 0 else GR)
        ch = v * kap * DT; h = truth[2] + ch / 2
        truth[0] += v * DT * math.cos(h); truth[1] += v * DT * math.sin(h); truth[2] += ch
        t += DT
        if abs(v) < 1e-4 and cmd.speed_mps == 0.0 and tr.remaining_to_goal_m() <= 1e-6:
            status = "end_stand"
            break
    g = P[-1]
    def err(p):
        return (round(1000 * math.hypot(p[0] - g[0], p[1] - g[1]), 1), round(math.atan2(math.sin(p[2] - g[2]), math.cos(p[2] - g[2])), 3))
    return status, "est", err(est), "truth", err(truth), round(t - trel, 1)


real_ctl = (round(1000 * tf["tracking"]["position_error_m"], 1), round(-tf["tracking"]["yaw_error_rad"], 3))
k = at(tf["time_s"]); g = P[-1]; tp = C[k, 4:7]
real_truth = (round(1000 * math.hypot(tp[0] - g[0], tp[1] - g[1]), 1), round(math.atan2(math.sin(tp[2] - g[2]), math.cos(tp[2] - g[2])), 3))
print("real end: control (pos mm, pose-goal yaw)", real_ctl, "truth", real_truth)
print("(A) truth fed          ", run(False))
print("(B) recorded est error ", run(True))
