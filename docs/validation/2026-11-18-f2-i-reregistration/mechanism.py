"""F2/I re-registration, the primary judgement's mechanism metric (docs/plans/2026-11-18-f2-i-reregistration.md 6.1,
fixed before the runs).

    python mechanism.py --root DIR --out OUT.json [--prefix f2i]

DIR holds <prefix>_<arm>_<scene>_s<seed>/run/{result.json, slam_control.npy}; arm in O, F2, I. slam_control columns:
time, estimated rear x, y, yaw, true rear x, y, yaw (as docs/validation/2026-11-14-transport-departure reads them).

Per run, over every transport-phase sample in time order (none filtered beforehand), sample i weighs the time to the
next transport sample and is excluded (reason counted) when there is no next sample, a path install or a leg switch
falls between the two, either time lacks a slam_control row within 1e-6 s, or the true rear moved less than 0.02 m.
Moving (|speed| > 0.05 m/s) and saturated (|commanded curvature| >= 0.355 /m) samples go to one of eight strata
(forward/reverse x left/right by the commanded sign x slow/fast at 0.4 m/s); the achieved curvature is the true yaw change over the true
displacement (signed by the travel direction) and must share the command's sign. Per stratum: ratio = time-weighted
mean achieved/commanded, mabs = time-weighted mean |command - actual| steering angle per wheel; valid when the
agreeing time is >= 0.5 s and >= half the stratum's saturated time. A pair's metric is the equal-weight mean over the
strata valid in both arms; a stratum valid in O that the candidate saturates (>= 0.5 s) but no longer holds is
'lost' (by moving saturated exposure, measurable or not) and that pair
counts as no improvement, also when it has no common stratum left. Reads only.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np

SAT = 0.355
MOVING = 0.05
MIN_STEP_M = 0.02
PAIR_TOL_S = 1e-6
MIN_AGREE_S = 0.5
AGREE_SHARE = 0.5
MIN_PAIRS = 6
RATIO_FLOOR = 0.95
MAJORITY = 0.75
SLOW_FAST_MPS = 0.4
STRATA = tuple(f"{d}_{t}_{v}" for d in ("fwd", "rev") for t in ("left", "right") for v in ("slow", "fast"))
NAME = re.compile(r"^(?P<prefix>[a-z0-9]+)_(?P<arm>O|F2|I)_(?P<scene>.+)_s(?P<seed>\d+)$")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def boundaries(r: dict) -> list[float]:
    """Times a transport path was installed or the tracker switched legs."""
    t = [float(x["time_s"]) for x in r.get("transitions") or [] if x.get("to") == "transport"]
    t += [float(c["time_s"]) for c in r.get("cusp_replans") or []]
    t += [float(c["time_s"]) for c in r.get("stall_replans") or []]
    t += [float(c["time_s"]) for c in r.get("paths_replaced") or [] if c.get("leg") == "transport"]
    t += [float(c["time_s"]) for c in r.get("scene_replans") or []
          if c.get("phase") == "transport" and c.get("outcome") == "installed"]
    ob = r.get("obstacle_layer") or {}
    for key in ("replans", "mission_replans"):
        t += [float(c["time_s"]) for c in ob.get(key) or []
              if isinstance(c, dict) and c.get("phase") == "transport" and "time_s" in c]
    t += [float(s["time_s"]) for s in r.get("leg_switches") or [] if s.get("phase") == "transport"]
    return sorted(t)


def row_exact(sc: np.ndarray, t: float):
    k = int(np.searchsorted(sc[:, 0], t - PAIR_TOL_S, side="left"))
    if k < len(sc) and abs(sc[k, 0] - t) <= PAIR_TOL_S:
        return sc[k]
    return None


def strata(r: dict, sc: np.ndarray) -> dict:
    samples = sorted((s for s in r.get("samples") or [] if s.get("phase") == "transport"),
                     key=lambda s: float(s["time_s"]))
    bounds = np.asarray(boundaries(r), float)
    acc = {k: {"exposure_s": 0.0, "saturated_s": 0.0, "agree_s": 0.0, "ratio": [], "mabs": []} for k in STRATA}
    excluded = {"last_sample": 0.0, "path_install": 0.0, "unpaired_row": 0.0, "short_step": 0.0}
    for i, s in enumerate(samples):
        t = float(s["time_s"])
        if i + 1 == len(samples):
            excluded["last_sample"] += 0.0
            continue
        t_next = float(samples[i + 1]["time_s"])
        w = max(0.0, t_next - t)
        if bounds.size and np.any((bounds > t + 1e-9) & (bounds <= t_next + 1e-9)):
            excluded["path_install"] += w
            continue
        a, b = row_exact(sc, t), row_exact(sc, t_next)
        if a is None or b is None:
            excluded["unpaired_row"] += w
            continue
        speed = float(s.get("signed_speed_mps", 0.0))
        if abs(speed) <= MOVING:
            continue
        kc = float(s["curvature_inv_m"])
        if abs(kc) < SAT:
            continue
        key = (("fwd" if speed > 0 else "rev") + ("_left" if kc > 0 else "_right")
               + ("_slow" if abs(speed) < SLOW_FAST_MPS else "_fast"))
        st = acc[key]
        # exposure: moving and saturated, whether or not the curvature is measurable (the lost-stratum test reads it)
        st["exposure_s"] += w
        step = math.hypot(b[4] - a[4], b[5] - a[5])
        if step < MIN_STEP_M:
            excluded["short_step"] += w
            continue
        st["saturated_s"] += w
        kach = wrap(b[6] - a[6]) / step * (1 if speed > 0 else -1)
        if np.sign(kach) != np.sign(kc):
            continue
        st["agree_s"] += w
        st["ratio"].append((w, kach / kc))
        cmd, act = s.get("steering_command_rad"), s.get("steering_actual_rad")
        if cmd and act:
            st["mabs"].append((w, [abs(float(c) - float(x)) for c, x in zip(cmd, act)]))
    out = {"excluded_s": {k: round(v, 4) for k, v in excluded.items()}}
    for k, st in acc.items():
        tw = sum(w for w, _ in st["ratio"])
        mw = sum(w for w, _ in st["mabs"])
        valid = st["agree_s"] >= MIN_AGREE_S - 1e-9 and st["agree_s"] >= AGREE_SHARE * st["saturated_s"] - 1e-9
        out[k] = {"exposure_s": round(st["exposure_s"], 4), "saturated_s": round(st["saturated_s"], 4), "agree_s": round(st["agree_s"], 4),
                  "valid": bool(valid and tw > 0 and mw > 0),
                  "ratio": None if tw <= 0 else sum(w * v for w, v in st["ratio"]) / tw,
                  "mabs": None if mw <= 0 else [sum(w * v[j] for w, v in st["mabs"]) / mw for j in range(2)]}
    return out


def pair_metric(o: dict, x: dict) -> dict:
    common = [k for k in STRATA if o[k]["valid"] and x[k]["valid"]]
    # a stratum valid in O that the candidate saturates but no longer holds (Codex plan review): counted against it
    lost = [k for k in STRATA if o[k]["valid"] and not x[k]["valid"] and x[k]["exposure_s"] >= MIN_AGREE_S - 1e-9]
    if not common:
        return {"common": [], "lost": lost}

    def mean(d, f):
        return float(np.mean([f(d[k]) for k in common]))

    return {"common": common, "lost": lost,
            "R_o": mean(o, lambda s: s["ratio"]), "R_x": mean(x, lambda s: s["ratio"]),
            "A_o": mean(o, lambda s: abs(s["mabs"][0] - s["mabs"][1])),
            "A_x": mean(x, lambda s: abs(s["mabs"][0] - s["mabs"][1]))}


def judge(pairs: list[dict]) -> dict:
    # a pair counts when it has a common stratum or lost one (Codex plan review: a pair that lost every common
    # stratum stays in the denominator as no improvement)
    ok = [p for p in pairs if "R_o" in p or p.get("lost")]
    n = len(ok)
    if n < MIN_PAIRS:
        return {"pairs": n, "mechanism": "undecided", "asymmetry": "undecided"}
    up = sum("R_o" in p and p["R_x"] > p["R_o"] and not p.get("lost") for p in ok) / n
    have_r = [p["R_x"] for p in ok if "R_o" in p]
    med = float(np.median(have_r)) if have_r else float("nan")
    down = sum("R_o" in p and p["A_x"] < p["A_o"] and not p.get("lost") for p in ok) / n
    return {"pairs": n, "median_R_x": med, "share_R_up": up, "share_A_down": down,
            "mechanism": "fixes" if med >= RATIO_FLOOR and up >= MAJORITY else "does_not_fix",
            "asymmetry": "reduces" if down >= MAJORITY else "does_not_reduce"}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--prefix", default="f2i")
    a = ap.parse_args()
    runs: dict[tuple[str, int], dict[str, dict]] = {}
    problems = {}
    for d in sorted(a.root.iterdir()):
        m = NAME.match(d.name)
        if not m or m["prefix"] != a.prefix:
            continue
        res, slam = d / "run/result.json", d / "run/slam_control.npy"
        if not (res.exists() and slam.exists()):
            problems[d.name] = "missing result.json or slam_control.npy"
            continue
        runs.setdefault((m["scene"], int(m["seed"])), {})[m["arm"]] = strata(json.loads(res.read_text()),
                                                                              np.load(slam))
    out = {"runs": {f"{s}_s{k}": v for (s, k), v in sorted(runs.items())}, "problems": problems, "verdict": {}}
    for arm in ("F2", "I"):
        pairs = []
        for (scene, seed), arms in sorted(runs.items()):
            if "O" in arms and arm in arms:
                pairs.append({"case": f"{scene}_s{seed}", **pair_metric(arms["O"], arms[arm])})
        out["verdict"][arm] = {**judge(pairs), "per_pair": pairs}
    a.out.write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "per_pair"} for k, v in out["verdict"].items()},
                     indent=1))


if __name__ == "__main__":
    main()
