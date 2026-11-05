"""The F2 2x2 results document's reported (not judged) numbers and the run identity:

- runs: per run folder its job, runner exit, the settings file's hash (taken when this script runs, not bound to the
  run), the run's own recorded settings (settings_synthetic) and probe summary (settle / load / contact failures), script / forklift URDF / scene layout hashes, a hash of
  the generated scene USD hashes, physics dt and the four tyres' read-back material;
- settings_diff: per run, the keys of its recorded settings that differ from the same measure / dt / repeat of another
  arm (O+F2 vs O, F1+F2 vs F1, F1 vs O);
- M1: per-wheel loads of the forward run and the largest forward-reversed difference;
- M3: valid trials of every run (120 Hz a/b/r, 240 Hz a/b), the unsettled trials by state and which settle quantity
  was still above its tolerance in the last window, and the order-sensitive count;
- M4: per run the points within the planar tolerance, the points whose every re-settle converged, both (= pose_ok),
  the shifts used, the final planar residual (Euclidean x-y, as the matching uses, and |yaw|; numpy median), the
  unmatched labels and the order-sensitive count. The matching history is the residual to the target after the
  previous drift was pre-compensated, not the drift of each re-settle; the drift itself is not recorded.

  PYTHONPATH=<judge snapshot>/src python report_numbers.py MANIFEST_X2.json --tools <judge snapshot> [--sacct FILE]
"""

import argparse
import collections
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

# sim/isaac/f1v2_probes.py at 9743d640 (the probe snapshot): SETTLE_TOL_F2 and SETTLE_EXTRA_TOL
SETTLE_TOL = {"base_z_m": 5e-5, "pitch_rad": 2e-6, "roll_rad": 2e-6, "lift_m": 5e-5, "steer_rad": 5e-5,
              "pallet_rel_m": 5e-5, "pallet_rel_yaw_rad": 5e-5}


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_identity(run: Path, sacct: dict) -> dict:
    res = json.loads((run / "result.json").read_text())
    log = (run.parent / "probe.log").read_text()
    job = re.search(r"job (\d+)", log).group(1)
    settings = re.search(r"settings (\S+)", log).group(1)
    exit_ = re.search(r"runner exit (\d+)", log)
    tyres = res.get("plant_tyre_contact") or []
    return {"job": job, "slurm_state": sacct.get(job), "runner_exit": None if exit_ is None else int(exit_.group(1)),
            "settings": settings, "settings_sha256": _sha(Path(settings)) if Path(settings).exists() else None,
            "script_sha256": res.get("script_sha256"), "forklift_urdf_sha256": res.get("forklift_urdf_sha256"),
            "scene_usd_digest": hashlib.sha256(json.dumps(res.get("scene_sha256"), sort_keys=True).encode()).hexdigest(),
            "scene_layout_sha256": (res.get("scene_layout") or {}).get("sha256"),
            "base_scene": (res.get("arguments") or {}).get("base_scene"),
            "physics_dt_s": (res.get("physics_scene") or {}).get("dt_s"), "python": res.get("python"),
            "settings_synthetic": res.get("settings_synthetic"), "probe_summary": res.get("f1v2_probe"),
            "tyres": sorted({(t.get("compliant_stiffness"), t.get("compliant_damping"), t.get("static_friction"),
                              t.get("dynamic_friction")) for t in tyres}, key=str), "n_tyres": len(tyres)}


def _last_settle(t: dict) -> dict:
    s = t["settle"]
    return s["lift"] if "lift" in s else s["seat"]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("manifest", type=Path)
    p.add_argument("--tools", type=Path, required=True)
    p.add_argument("--sacct", type=Path, help="sacct -X -n -P -o JobID,State output")
    args = p.parse_args()
    sys.path.insert(0, str(args.tools / "tools"))
    sys.path.insert(0, str(args.tools / "sim" / "isaac"))
    import f2_2x2 as T

    sacct = {}
    if args.sacct:
        for line in args.sacct.read_text().split():
            job, state = line.split("|")[:2]
            sacct[job] = state
    man = json.loads(args.manifest.read_text())
    load = lambda q: json.loads(Path(q).read_text())  # noqa: E731
    runs = set()
    for m in ("static", "cusp", "delivery"):
        for dt in man[m].values():
            for v in dt.values():
                for q in (v.values() if isinstance(v, dict) else [v]):
                    runs.add(Path(q).parent)
    for dt in man["steer"].values():
        runs.update(Path(q) for q in dt.values())
    out = {"runs": {str(r.parent.name): run_identity(r, sacct) for r in sorted(runs)}, "m1": {}, "m3": {}, "m4": {}}
    diff = {}
    for name, ident in out["runs"].items():
        for arm, ref in (("OpF2", "O"), ("F1pF2", "F1"), ("F1", "O")):
            if f"_{arm}_" in name:
                other = out["runs"].get(name.replace(f"_{arm}_", f"_{ref}_"))
                a, b = ident["settings_synthetic"] or {}, (other or {}).get("settings_synthetic") or {}
                diff[f"{name} vs {ref}"] = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
    out["settings_diff"] = diff
    for arm in T.ARMS:
        st = man["static"]["120"]
        f, r = T.m1_values(load(st[arm])), T.m1_values(load(st[arm + "#r"]))
        out["m1"][arm] = {"loads_n": f, "reverse_max_abs_n": max(abs(f[s] - r[s]) for s in f)}
        valid = {}
        for dt, keys in (("120", "abr"), ("240", "ab")):
            for k in keys:
                x = load(man["cusp"][dt][arm][k])
                valid[f"{dt}/{k}"] = [sum(T._trial_valid(t) for t in x["trials"]), len(x["trials"])]
        cu = man["cusp"]["120"][arm]
        a, rr = load(cu["a"]), load(cu["r"])
        bad = [t for t in a["trials"] if not t["settle"]["converged"]]
        above = collections.Counter(k for t in bad for k, v in _last_settle(t)["last_window"].items()
                                    if k in SETTLE_TOL and v >= SETTLE_TOL[k])
        rolls = [_last_settle(t)["last_window"]["roll_rad"] for t in bad]
        out["m3"][arm] = {"valid": valid, "unsettled": len(bad),
                          "unsettled_by_state": dict(collections.Counter("/".join(t["id"].split("/")[-2:]) for t in bad)),
                          "unsettled_above_tolerance": dict(above),
                          "unsettled_last_roll_rad": [min(rolls), float(np.median(rolls)), max(rolls)] if rolls else None,
                          "order_sensitive": len(T.cusp_order_sensitive(a, rr))}
        de = man["delivery"]["120"][arm]
        m4 = {}
        for k in "abr":
            d = load(de[k])
            pts = d["points"]
            xy = [math.hypot(q["planar"]["residual"][0], q["planar"]["residual"][1]) for q in pts]
            yaw = [abs(q["planar"]["residual"][2]) for q in pts]
            m4[k] = {"points": len(pts), "within_tolerance": sum(q["planar"]["within_tolerance"] for q in pts),
                     "all_settles": sum(all(q["planar"]["settles"]) for q in pts),
                     "pose_ok": sum(q["pose_ok"] for q in pts),
                     "shifts": dict(collections.Counter(q["planar"]["shifts"] for q in pts)),
                     "residual_xy_m": [float(np.median(xy)), max(xy)], "residual_yaw_rad": [float(np.median(yaw)), max(yaw)],
                     "unmatched": [q["label"] for q in pts if not q["pose_ok"]][:22]}
        m4["order_sensitive"] = len(T.delivery_order_sensitive(load(de["a"]), load(de["r"])))
        out["m4"][arm] = m4
    print(json.dumps(out, indent=1, default=float))


if __name__ == "__main__":
    main()
