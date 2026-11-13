"""The left steering shortfall table of docs/validation/2026-11-14-left-steer-shortfall.md 2.

    python left_steer_table.py FIELDS.json.gz --out OUT.json

FIELDS is docs/validation/assets/transport-departure/tdep_fields.json.gz: per run (key "<root>/<run>") the transport
samples (result.samples: time_s, signed_speed_mps, curvature_inv_m = the tracker's commanded curvature,
steering_command_rad / steering_actual_rad = [left, right] joint targets and joint angles) and slam_control rows
[t, est x, y, yaw, true x, y, yaw] at those sample times.

A pair is two consecutive transport samples a, b whose slam_control rows exist (|t_row - t| <= 1e-6). It is kept when
a's commanded curvature is saturated (|kappa_cmd| >= 0.355), a's speed > 0.05 m/s, the true rear axle moved >= 0.02 m
from a to b, and the observed curvature (true yaw change / true distance, a to b) has kappa_cmd's sign. Per pair:
- observed curvature kappa_obs;
- each front wheel's shortfall e = sign(kappa_cmd) * (command - actual) at a (positive: turned less);
- the curvature each wheel's actual angle implies by Ackermann geometry (wheelbase 0.66, track 0.53 m):
  inner wheel tan(d) / (L + w/2 tan|d|), outer wheel tan(d) / (L - w/2 tan|d|), signs as d's.
Reported by turn direction and by scene. Reads only.
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import re

import numpy as np

SAT, MOVING, MIN_STEP, L, W = 0.355, 0.05, 0.02, 0.66, 0.53
SCENE = re.compile(r"_(random_\d+|line_\d+|aisle_rack|aisle|trailer)_s\d+$")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wheel_kappa(d: float, inner: bool) -> float:
    t = math.tan(d)
    return t / (L + W / 2 * abs(t)) if inner else t / (L - W / 2 * abs(t))


def pairs(fields: dict):
    for key, v in sorted(fields.items()):
        rows = np.asarray(v["slam_control_rows"], dtype=float).reshape(-1, 7)
        if not len(rows):
            continue
        scene = (SCENE.search(key) or [None, "?"])[1]
        s = [x for x in v["result"]["samples"] if x.get("phase") == "transport"]
        for a, b in zip(s, s[1:]):
            kc, sp = float(a.get("curvature_inv_m") or 0.0), float(a.get("signed_speed_mps") or 0.0)
            if abs(kc) < SAT or sp <= MOVING:
                continue
            ra = rows[np.abs(rows[:, 0] - a["time_s"]) <= 1e-6]
            rb = rows[np.abs(rows[:, 0] - b["time_s"]) <= 1e-6]
            if not len(ra) or not len(rb):
                continue
            ra, rb = ra[0], rb[0]
            step = math.hypot(rb[4] - ra[4], rb[5] - ra[5])
            if step < MIN_STEP:
                continue
            k_obs = wrap(rb[6] - ra[6]) / step
            sg = 1 if kc > 0 else -1
            if k_obs * sg <= 0:
                continue
            c, x = a["steering_command_rad"], a["steering_actual_rad"]
            left_inner = sg > 0
            yield {"run": key, "scene": scene, "sign": sg, "k_cmd": kc, "k_obs": k_obs,
                   "e_L": sg * (c[0] - x[0]), "e_R": sg * (c[1] - x[1]),
                   "k_left": wheel_kappa(x[0], left_inner), "k_right": wheel_kappa(x[1], not left_inner),
                   "cmd": [c[0], c[1]]}


def summary(rows: list[dict]) -> dict:
    a = lambda k: np.asarray([r[k] for r in rows], dtype=float)  # noqa: E731
    q = lambda v: [round(float(x), 4) for x in np.percentile(v, [10, 50, 90])]  # noqa: E731
    return {"n": len(rows), "runs": len({r["run"] for r in rows}),
            "k_obs_p10_p50_p90": q(np.abs(a("k_obs"))), "obs_over_cmd_p10_p50_p90": q(a("k_obs") / a("k_cmd")),
            "e_L_mrad_p50": round(float(np.median(a("e_L"))) * 1000, 1),
            "e_R_mrad_p50": round(float(np.median(a("e_R"))) * 1000, 1),
            "k_left_wheel_p50": round(float(np.median(np.abs(a("k_left")))), 4),
            "k_right_wheel_p50": round(float(np.median(np.abs(a("k_right")))), 4),
            "cmd_left_right_p50": [round(float(np.median([r["cmd"][i] for r in rows])), 4) for i in (0, 1)]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("fields")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    with gzip.open(a.fields, "rt") as f:
        rows = list(pairs(json.load(f)))
    out = {"by_turn": {}, "by_turn_scene": {}}
    for sg, name in ((1, "left_turn"), (-1, "right_turn")):
        sel = [r for r in rows if r["sign"] == sg]
        out["by_turn"][name] = summary(sel)
        for scene in sorted({r["scene"] for r in sel}):
            out["by_turn_scene"][f"{name}/{scene}"] = summary([r for r in sel if r["scene"] == scene])
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out["by_turn"], indent=1))
    print({k: v["n"] for k, v in out["by_turn_scene"].items()})


if __name__ == "__main__":
    main()
