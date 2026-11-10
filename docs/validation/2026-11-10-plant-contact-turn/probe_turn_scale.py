"""Turning scale of the simulated truck per plant arm, from existing steering-probe records only (lead assignment
2026-11-10, step 1 after #410: the plant's contact and friction, load separated on a matched path).

A steering probe run (``--steering-probe``, steering_probe.npz: one row per physics step with the segment, the
commanded speed and curvature, the steering joints' positions q_FL / q_FR, the rear wheels' joint angles p_RL / p_RR and
the rear axle's pose rear_x / rear_y / rear_yaw) drives fixed segments of constant speed and curvature. For every
driving segment with a nonzero curvature, over its second half (settled):
- the truth's heading change, and the model's -- the Ackermann curvature of the steering joints' positions (the
  odometry's own formula, IncrementalWheelOdometry: wheelbase 0.66 m, track 0.53 m), mid-interval, times the rear
  axle's signed forward travel along its heading;
- scale = truth / model (1: the truck turns as the kinematics says; below 1: it turns less);
- each front wheel's steering shortfall e = (command - joint position) x sign(command), the command being the
  rate-limited one of the same row (cmd_FL / cmd_FR; positive: steered less than commanded), as
  tools/steer_asym_judge.py reads it;
- each wheel's floor normal force, -Jz / dt of the tyre contact impulses (tyre_contacts), mean over the window.
Reported per arm: the forward segments' mean and range of the scale, the reverse segments' mean, and the forward
segments' mean e_L, e_R and normal forces.

    python probe_turn_scale.py RUN_DIR... [--json out.json]   (RUN_DIR holds steering_probe.npz; its name is the arm)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

WB, TR = 0.66, 0.53


def curvature(left, right):
    tl, tr = np.tan(left), np.tan(right)
    return (tl / (WB + tl * TR / 2) + tr / (WB - tr * TR / 2)) / 2


def arm(run: Path) -> dict:
    z = np.load(run / "steering_probe.npz", allow_pickle=True)
    X = z["record"]
    C = {str(c): i for i, c in enumerate(z["columns"])}

    def col(name):
        return X[:, C[name]]

    seg = col("segment").astype(int)
    yaw = np.unwrap(col("rear_yaw"))
    x, y = col("rear_x"), col("rear_y")
    k_q = curvature(col("q_FL"), col("q_FR"))
    dt = float(np.median(np.diff(col("t"))))
    normal = -z["tyre_contacts"][:, :, 2] / dt  # FL, FR, RL, RR
    rows = []
    for s in np.unique(seg):
        m = np.nonzero(seg == s)[0]
        v, kc = float(col("speed_cmd")[m[0]]), float(col("kappa_cmd")[m[0]])
        if v == 0 or kc == 0 or len(m) < 120:
            continue
        w = m[len(m) // 2:]
        fwd = np.diff(x[w]) * np.cos(yaw[w][:-1]) + np.diff(y[w]) * np.sin(yaw[w][:-1])
        model = float(np.sum((k_q[w][:-1] + k_q[w][1:]) / 2 * fwd))
        truth = float(np.sum(np.diff(yaw[w])))
        e = {side: float(np.mean((col(f"cmd_{side}")[w] - col(f"q_{side}")[w]) * np.sign(col(f"cmd_{side}")[w])))
             for side in ("FL", "FR")}
        n = normal[w].mean(axis=0)
        rows.append({"segment": int(s), "speed_mps": v, "kappa": kc, "truth_dyaw_rad": truth, "model_dyaw_rad": model,
                     "travel_m": float(np.sum(fwd)), "scale": truth / model if abs(model) > 1e-6 else None,
                     "e_L_rad": e["FL"], "e_R_rad": e["FR"],
                     "normal_N": {k: float(v_) for k, v_ in zip(("FL", "FR", "RL", "RR"), n)}})
    fwd_s = [r["scale"] for r in rows if r["speed_mps"] > 0 and r["scale"] is not None]
    rev_s = [r["scale"] for r in rows if r["speed_mps"] < 0 and r["scale"] is not None]
    fw = [r for r in rows if r["speed_mps"] > 0]
    return {"segments": rows, "forward_mean": float(np.mean(fwd_s)), "forward_min": float(np.min(fwd_s)),
            "forward_max": float(np.max(fwd_s)), "forward_n": len(fwd_s), "reverse_mean": float(np.mean(rev_s)),
            "forward_e_L_rad": float(np.mean([r["e_L_rad"] for r in fw])),
            "forward_e_R_rad": float(np.mean([r["e_R_rad"] for r in fw])),
            "forward_normal_N": {k: float(np.mean([r["normal_N"][k] for r in fw])) for k in ("FL", "FR", "RL", "RR")}}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    out = {r.name: arm(r) for r in a.runs}
    if a.json:
        a.json.write_text(json.dumps(out, indent=1))
    for name, r in out.items():
        nn = r["forward_normal_N"]
        print(f"{name:16s} forward {r['forward_mean']:.4f} [{r['forward_min']:.3f}, {r['forward_max']:.3f}] "
              f"n={r['forward_n']}  reverse {r['reverse_mean']:.4f}  e_L {1e3 * r['forward_e_L_rad']:5.1f} "
              f"e_R {1e3 * r['forward_e_R_rad']:5.1f} mrad  N FL/FR/RL/RR {nn['FL']:.0f}/{nn['FR']:.0f}/{nn['RL']:.0f}/{nn['RR']:.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
