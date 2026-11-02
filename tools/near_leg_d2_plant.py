"""Near-leg plan D2' part 2 (docs/validation/2026-11-02-near-leg-d2-plant.md): the plant from the applied commands.
(a) steering: first-order lag (+ dead time) from the applied wheel steering command to the joint angle, fitted on the
identification runs, scored on the validation runs. (b) motion: the applied speed through a first-order lag, the
curvature from the two simulated wheel angles, integrated from the truth rear pose over 3 s windows; scored at the
window end against the truth.
    python d2_plant.py RUNS_DIR OUT.json [--round2]   (RUNS_DIR holds st_*/run/{result.json,slam_log.npz})
--round2 (registered after round 1 missed (b)): the realised curvature over the wheel-angle curvature by gear (g_f,
g_r) and a speed gain s, the curvature-response plan's prior (forward ~0.90, reverse ~1.05); tau_v = 0. Picked on
the identification windows' heading p95 (the heading change is linear in them per window), then the validation
windows are integrated in full."""
import glob, json, math, os, sys
import numpy as np
from scipy.signal import lfilter

EXCLUDED = {"st_off_random_1_s2", "st_on_random_1_s2", "st_on_random_1_s5", "st_on_random_1_s9"}  # diagnostic only
IDENT = {f"st_{a}_{s}" for a in ("on", "off") for s in
         ("random_1_s0", "random_1_s1", "random_1_s3", "aisle_rack_s0", "aisle_rack_s1", "line_2_s0", "line_2_s1",
          "trailer_s0")}
OFFSET = 0.34  # rear axle behind the base origin (the runs' --rear-axle-offset-m)
TAU_GRID = [round(0.05 + 0.01 * i, 2) for i in range(26)]
DEAD_GRID = [0.0, 0.025, 0.05]
TAUV_GRID = [0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5]


def load(d):
    s = json.load(open(f"{d}/run/result.json"))
    z = np.load(f"{d}/run/slam_log.npz")
    S = s["samples"]
    ts = np.array([x["time_s"] for x in S])
    return dict(
        ts=ts, cmd=np.array([x["steering_command_rad"] for x in S]), v=np.array([x["speed_command_mps"] for x in S]),
        k=np.array([x["curvature_inv_m"] for x in S]), tj=z["joint_stamps_s"], act=z["steering_rad"],
        pose=z["base_pose_world"])


def lag(u, tj, y0, tau):
    if tau <= 0:
        return u.copy()
    dt = float(np.median(np.diff(tj)))
    a = math.exp(-dt / tau)
    y, _ = lfilter([1 - a], [1, -a], u, zi=[a * y0])
    return y


def steer_sim(r, k, tau, dead):
    u = np.interp(r["tj"] - dead, r["ts"], r["cmd"][:, k])
    return lag(u, r["tj"], r["act"][0, k], tau)


def yaw_of(q):  # base_pose_world: x y z qw qx qy qz (checked against the rear yaw below)
    w, x, y, z = q[3], q[4], q[5], q[6]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    runs = {os.path.basename(d): d for d in sorted(glob.glob(sys.argv[1] + "/st_*")) if os.path.isdir(d + "/run")}
    data = {n: load(d) for n, d in runs.items() if n not in EXCLUDED}
    ident = [n for n in data if n in IDENT]
    valid = [n for n in data if n not in IDENT]
    out = {"ident": ident, "valid": valid, "excluded": sorted(EXCLUDED)}
    # (a) steering
    best = None
    for tau in TAU_GRID:
        for dead in DEAD_GRID:
            e = np.concatenate([steer_sim(data[n], k, tau, dead) - data[n]["act"][:, k] for n in ident for k in (0, 1)])
            rmse = float(np.sqrt(np.mean(e ** 2)))
            if best is None or rmse < best[0]:
                best = (rmse, tau, dead)
    _, tau, dead = best
    out["steer"] = {"tau_s": tau, "dead_s": dead, "ident_rmse": best[0]}
    per = {}
    allv = []
    for n in valid:
        e = np.concatenate([steer_sim(data[n], k, tau, dead) - data[n]["act"][:, k] for k in (0, 1)])
        allv.append(e)
        per[n] = {"rmse": float(np.sqrt(np.mean(e ** 2))), "p99": float(np.percentile(np.abs(e), 99))}
    allv = np.concatenate(allv)
    out["steer"].update(valid_rmse=float(np.sqrt(np.mean(allv ** 2))),
                        runs_p99_ok=sum(v["p99"] <= 0.06 for v in per.values()), runs=len(per), per_run=per)
    # (b) motion: the curvature from the wheel angles -- k = (tan dL + tan dR) / (2 L), L fitted on the identification
    # runs' applied (command curvature, command angles); the speed through a lag tau_v
    num = np.concatenate([(np.tan(data[n]["cmd"][:, 0]) + np.tan(data[n]["cmd"][:, 1])) / 2 for n in ident])
    kk = np.concatenate([data[n]["k"] for n in ident])
    m = np.abs(kk) > 0.05
    L = float(np.median(num[m] / kk[m]))
    out["motion"] = {"wheelbase_fit_m": L}

    def windows(r, tau_v):
        tj, pose = r["tj"], r["pose"]
        dL, dR = steer_sim(r, 0, tau, dead), steer_sim(r, 1, tau, dead)
        kap = (np.tan(dL) + np.tan(dR)) / (2 * L)
        vcmd = np.interp(tj, r["ts"], r["v"])
        dt = float(np.median(np.diff(tj)))
        res = []
        t0 = tj[0]
        while t0 + 3.0 <= tj[-1]:
            i0 = int(np.searchsorted(tj, t0)); i1 = int(np.searchsorted(tj, t0 + 3.0))
            if np.max(np.abs(vcmd[i0:i1])) > 0.05:
                yaw = yaw_of(pose[i0]); x = pose[i0, 0] - OFFSET * math.cos(yaw); y = pose[i0, 1] - OFFSET * math.sin(yaw)
                # the truth speed at the start, from the truth rear positions one tick apart
                yb = yaw_of(pose[i0 - 1]) if i0 > 0 else yaw
                xb = pose[i0 - 1, 0] - OFFSET * math.cos(yb) if i0 > 0 else x
                ybb = pose[i0 - 1, 1] - OFFSET * math.sin(yb) if i0 > 0 else y
                v = ((x - xb) * math.cos(yaw) + (y - ybb) * math.sin(yaw)) / dt
                a = math.exp(-dt / tau_v) if tau_v > 0 else 0.0
                for i in range(i0, i1):
                    v = a * v + (1 - a) * vcmd[i]
                    ch = v * kap[i] * dt
                    h = yaw + ch / 2
                    x += v * dt * math.cos(h); y += v * dt * math.sin(h); yaw += ch
                ty = yaw_of(pose[i1]); tx = pose[i1, 0] - OFFSET * math.cos(ty); tyy = pose[i1, 1] - OFFSET * math.sin(ty)
                he = math.atan2(math.sin(yaw - ty), math.cos(yaw - ty))
                lat = -math.sin(ty) * (x - tx) + math.cos(ty) * (y - tyy)
                res.append((abs(he), abs(lat)))
            t0 += 1.0
        return res

    if "--round2" in sys.argv:
        G = np.round(np.arange(0.80, 1.1501, 0.01), 2)
        SG = np.round(np.arange(0.90, 1.1001, 0.02), 2)

        def parts(r):
            tj, pose = r["tj"], r["pose"]
            dL, dR = steer_sim(r, 0, tau, dead), steer_sim(r, 1, tau, dead)
            kap = (np.tan(dL) + np.tan(dR)) / (2 * L)
            vcmd = np.interp(tj, r["ts"], r["v"])
            dt = float(np.median(np.diff(tj)))
            rows = []
            t0 = tj[0]
            while t0 + 3.0 <= tj[-1]:
                i0 = int(np.searchsorted(tj, t0)); i1 = int(np.searchsorted(tj, t0 + 3.0))
                if np.max(np.abs(vcmd[i0:i1])) > 0.05:
                    w = vcmd[i0:i1] * kap[i0:i1] * dt
                    af, ar = float(w[vcmd[i0:i1] >= 0].sum()), float(w[vcmd[i0:i1] < 0].sum())
                    dy = math.atan2(math.sin(yaw_of(pose[i1]) - yaw_of(pose[i0])), math.cos(yaw_of(pose[i1]) - yaw_of(pose[i0])))
                    rows.append((af, ar, dy))
                t0 += 1.0
            return rows

        P_ = np.array([x for n in ident for x in parts(data[n])])
        gf, gr, sg = np.meshgrid(G, G, SG, indexing="ij")
        pred = sg[..., None] * (gf[..., None] * P_[:, 0] + gr[..., None] * P_[:, 1])
        err = np.abs(np.arctan2(np.sin(pred - P_[:, 2]), np.cos(pred - P_[:, 2])))
        p95 = np.percentile(err, 95, axis=-1)
        i, j, k_ = np.unravel_index(int(np.argmin(p95)), p95.shape)
        g_f, g_r, s_g = float(G[i]), float(G[j]), float(SG[k_])

        def windows2(r):
            tj, pose = r["tj"], r["pose"]
            dL, dR = steer_sim(r, 0, tau, dead), steer_sim(r, 1, tau, dead)
            kap = (np.tan(dL) + np.tan(dR)) / (2 * L)
            vraw = np.interp(tj, r["ts"], r["v"])
            vcmd = vraw * s_g
            dt = float(np.median(np.diff(tj)))
            res = []
            t0 = tj[0]
            while t0 + 3.0 <= tj[-1]:
                i0 = int(np.searchsorted(tj, t0)); i1 = int(np.searchsorted(tj, t0 + 3.0))
                # the registered window selection: the applied speed before the gain (Codex D2' review)
                if np.max(np.abs(vraw[i0:i1])) > 0.05:
                    yaw = yaw_of(pose[i0]); x = pose[i0, 0] - OFFSET * math.cos(yaw); y = pose[i0, 1] - OFFSET * math.sin(yaw)
                    for i in range(i0, i1):
                        v = vcmd[i]
                        ch = v * kap[i] * (g_f if v >= 0 else g_r) * dt
                        h = yaw + ch / 2
                        x += v * dt * math.cos(h); y += v * dt * math.sin(h); yaw += ch
                    ty = yaw_of(pose[i1]); tx = pose[i1, 0] - OFFSET * math.cos(ty); tyy = pose[i1, 1] - OFFSET * math.sin(ty)
                    he = math.atan2(math.sin(yaw - ty), math.cos(yaw - ty))
                    lat = -math.sin(ty) * (x - tx) + math.cos(ty) * (y - tyy)
                    res.append((abs(he), abs(lat)))
                t0 += 1.0
            return res

        wi = [x for n in ident for x in windows2(data[n])]
        wv = [x for n in valid for x in windows2(data[n])]
        out["round2"] = {"g_forward": g_f, "g_reverse": g_r, "speed_gain": s_g,
                         "ident_heading_p95": float(np.percentile([h for h, _ in wi], 95)),
                         "ident_lateral_p95_m": float(np.percentile([l for _, l in wi], 95)), "windows": len(wv),
                         "valid_heading_p95": float(np.percentile([h for h, _ in wv], 95)),
                         "valid_lateral_p95_m": float(np.percentile([l for _, l in wv], 95))}
    bestv = None
    for tv in TAUV_GRID:
        w = [x for n in ident for x in windows(data[n], tv)]
        sc = float(np.percentile([h for h, _ in w], 95))
        if bestv is None or sc < bestv[0]:
            bestv = (sc, tv)
    tau_v = bestv[1]
    wv = [x for n in valid for x in windows(data[n], tau_v)]
    out["motion"].update(tau_v_s=tau_v, ident_heading_p95=bestv[0], windows=len(wv),
                         valid_heading_p95=float(np.percentile([h for h, _ in wv], 95)),
                         valid_lateral_p95_m=float(np.percentile([l for _, l in wv], 95)))
    json.dump(out, open(sys.argv[2], "w"), indent=1)
    print(json.dumps({k: (v if k != "steer" else {kk: vv for kk, vv in v.items() if kk != "per_run"}) for k, v in out.items()
                      if k not in ("ident", "valid")}, indent=1))


if __name__ == "__main__":
    main()
