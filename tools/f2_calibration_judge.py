#!/usr/bin/env python3
"""F2 stage C (docs/plans/2026-11-05-f2-tyre-2x2.md section 3): the material value whose apparent per-wheel stiffness
against the rigid arm of the same mass, dt, wheel and start pose is closest to 400 kN/m, fixed before any result.

  python tools/f2_calibration_judge.py MANIFEST.json --json OUT.json

MANIFEST: {"rigid": {"O": [STATIC.json, STATIC.json], "F1": [...]},           # two repeats each, 120 Hz
           "grid": {"100000": {"O": STATIC.json, "F1": STATIC.json}, ...},   # k_mat in N/m -> the F2 runs
           "dt240": {"rigid": {"O": STATIC.json, "F1": ...}, "chosen": {"O": ..., "F1": ...}}}   # optional, reported
The STATIC files are run/f1v2_static_probe.json of --f1v2-probe static runs.
"""

import argparse
import json
import math
from pathlib import Path

TARGET = 400_000.0  # N/m per wheel (the rubber representative, a Hertz tangent stiffness)
BAND = (280_000.0, 520_000.0)  # +-30 %, an engineering band, not a measurement accuracy
MAX_WIDTH_RATIO = 2.0  # the stiffness interval's upper / lower
R_M, W_M = 0.125, 0.08  # the tyre collider (cylinder radius and length)
DAMPING_PER_STIFFNESS_S = 310.0 / 400_000.0  # c_mat = k_mat x this (plan 3)
WHEELS = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
LOADS = ("empty", "loaded")
MASSES = ("O", "F1")
GRID = (100_000.0, 120_000.0, 140_000.0, 160_000.0, 180_000.0)  # plan 3, fixed before the results
ENGINE_KG = {"O": 26.0, "F1": 24.0002}  # the arms' engine masses (F1 v2 records)
TYRE_FRICTION = (0.8, 0.7)  # the operating settings' tyre material (isaac_transport_measured.yaml); F2 keeps it (#304)


def support_height(alpha: float) -> float:
    """The vertical support height of a cylinder tyre whose axle is alpha from the horizontal."""
    return R_M * math.cos(alpha) + (W_M / 2) * abs(math.sin(alpha))


def wheel_stiffness(rigid: dict, f2: dict) -> dict:
    """One wheel: compression = (z_rigid - z_F2) + (h(alpha_F2) - h(alpha_rigid)); the force is the F2 run's floor push;
    u = the rigid contact's penetration bound + both windows' height ranges."""
    n = f2["fz"]["mean"]
    comp = (rigid["z"]["mean"] - f2["z"]["mean"]) + (support_height(f2["tilt"]["mean"])
                                                     - support_height(rigid["tilt"]["mean"]))
    sep = rigid["min_sep"]["min"]
    if sep is None or not math.isfinite(sep):
        return {"force_n": n, "compression_m": comp, "u_m": None, "valid": False, "k": None, "k_lo": None,
                "k_hi": None, "why": "no rigid contact separation"}
    u = abs(min(sep, 0.0)) + (rigid["z"]["max"] - rigid["z"]["min"]) + (f2["z"]["max"] - f2["z"]["min"])
    out = {"force_n": n, "compression_m": comp, "u_m": u}
    if n is None or n <= 0 or comp <= 0:
        out.update(valid=False, k=None, k_lo=None, k_hi=None)
        return out
    k = n / comp
    k_lo, k_hi = n / (comp + u), n / max(comp - u, 1e-6)
    ok = (BAND[0] <= k <= BAND[1] and k_hi >= BAND[0] and k_lo <= BAND[1] and k_hi / k_lo <= MAX_WIDTH_RATIO)
    out.update(valid=True, k=k, k_lo=k_lo, k_hi=k_hi, in_band=ok)
    return out


def grid_point(rigid: dict, f2: dict) -> dict:
    """Every mass x load x wheel of one k_mat: rigid = {mass: static record}, f2 = {mass: static record}."""
    rows = {}
    for m in MASSES:
        for load in LOADS:
            for w in WHEELS:
                rows[f"{m}/{load}/{w}"] = wheel_stiffness(rigid[m]["loads"][load]["wheels"][w],
                                                          f2[m]["loads"][load]["wheels"][w])
    ks = [r["k"] for r in rows.values() if r["valid"]]
    passes = len(ks) == len(rows) and all(r.get("in_band") for r in rows.values())
    spread = max(abs(math.log(k / TARGET)) for k in ks) if ks else None
    return {"rows": rows, "passes": passes, "max_abs_log_ratio": spread}


def repeat_spread(a: dict, b: dict) -> dict:
    """3 x the rigid repeats' largest difference in wheel height and force (plan 3)."""
    dz, df = 0.0, 0.0
    for load in LOADS:
        for w in WHEELS:
            ra, rb = a["loads"][load]["wheels"][w], b["loads"][load]["wheels"][w]
            dz = max(dz, abs(ra["z"]["mean"] - rb["z"]["mean"]))
            df = max(df, abs(ra["fz"]["mean"] - rb["fz"]["mean"]))
    return {"three_delta_z_m": 3 * dz, "three_delta_force_n": 3 * df}


def _pose_ok(pose) -> bool:
    return isinstance(pose, (list, tuple)) and len(pose) == 3 and all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in pose)


def record_problems(rec: dict, mass: str, hz: float, k_mat, start, combine=None) -> list:
    """Why a static record is not the registered condition: dt, the arm's engine mass, the start pose, the order, and
    the tyre material read back (rigid: no compliance; F2: k_mat and c_mat on all four tyres)."""
    out = []
    if rec.get("physics_hz") != hz:
        out.append(f"physics_hz {rec.get('physics_hz')}")
    if not math.isclose(rec.get("robot_mass_kg", -1), ENGINE_KG[mass], abs_tol=1e-3):
        out.append(f"engine mass {rec.get('robot_mass_kg')}")
    if rec.get("order", "forward") != "forward":
        out.append(f"order {rec.get('order')}")
    pose = rec.get("start_base")
    if not _pose_ok(pose) or not _pose_ok(start) or any(abs(a - b) > 1e-6 for a, b in zip(pose, start)):
        out.append(f"start pose {pose} (reference {start})")
    tyres = rec.get("tyre_readback") or []
    if len(tyres) != 4:
        out.append(f"{len(tyres)} tyres read back")
    for t in tyres:
        modes = (t.get("friction_combine_mode"), t.get("damping_combine_mode"))
        if combine is not None and modes != combine:
            out.append(f"{t.get('collider')}: combine modes {modes} (reference {combine})")
    for t in tyres:
        fr = (t.get("static_friction"), t.get("dynamic_friction"))  # read back as float32
        if None in fr or not all(math.isclose(a, b, rel_tol=1e-6) for a, b in zip(fr, TYRE_FRICTION)):
            out.append(f"{t.get('collider')}: friction {t.get('static_friction')}/{t.get('dynamic_friction')}")
        k, c = t.get("compliant_stiffness"), t.get("compliant_damping")
        if k_mat is None:
            if k is not None:
                out.append(f"rigid arm with compliance on {t.get('collider')}")
        elif k is None or not math.isclose(k, k_mat, rel_tol=1e-6) or c is None \
                or not math.isclose(c, k_mat * DAMPING_PER_STIFFNESS_S, rel_tol=1e-6):
            out.append(f"{t.get('collider')}: k {k} c {c}, expected {k_mat}")
    return out


def judge(manifest: dict, load=lambda p: json.loads(Path(p).read_text())) -> dict:
    rigid_runs = {m: [load(p) for p in manifest["rigid"][m]] for m in MASSES}
    unusable = []
    ref = rigid_runs["O"][0] if rigid_runs["O"] else {}
    start = ref.get("start_base")
    ref_tyres = ref.get("tyre_readback") or [{}]
    combine = (ref_tyres[0].get("friction_combine_mode"), ref_tyres[0].get("damping_combine_mode"))
    if sorted(float(k) for k in manifest["grid"]) != list(GRID):
        unusable.append(f"grid {sorted(manifest['grid'])} is not the registered {GRID}")
    for m in MASSES:
        for i, run in enumerate(rigid_runs[m]):
            unusable += [f"rigid {m} #{i + 1}: {w}" for w in record_problems(run, m, 120.0, None, start, combine)]
    for m in MASSES:
        if len(rigid_runs[m]) != 2:
            unusable.append(f"rigid {m}: two repeats needed")
        for run in rigid_runs[m]:
            for ld in LOADS:
                rec = run["loads"][ld]
                if not rec["start"]["converged"] or rec["foreign_contacts"] or abs(rec["wheel_load_ratio"] - 1) > 0.01:
                    unusable.append(f"rigid {m} {ld}: settle {rec['start']['converged']}, foreign "
                                    f"{rec['foreign_contacts']}, load ratio {rec['wheel_load_ratio']:.4f}")
    rigid = {m: rigid_runs[m][0] for m in MASSES}
    grid = {}
    for k_mat, runs in sorted(manifest["grid"].items(), key=lambda kv: float(kv[0])):
        f2 = {m: load(runs[m]) for m in MASSES}
        for m in MASSES:
            unusable += [f"grid {k_mat} {m}: {w}"
                         for w in record_problems(f2[m], m, 120.0, float(k_mat), start, combine)]
        point = grid_point(rigid, f2)
        bad = [f"{m} {ld}" for m in MASSES for ld in LOADS
               if not f2[m]["loads"][ld]["start"]["converged"] or f2[m]["loads"][ld]["foreign_contacts"]
               or abs(f2[m]["loads"][ld]["wheel_load_ratio"] - 1) > 0.01]
        point["run_problems"] = bad
        point["passes"] = point["passes"] and not bad
        point["c_mat"] = float(k_mat) * DAMPING_PER_STIFFNESS_S
        grid[k_mat] = point
    passing = [(p["max_abs_log_ratio"], float(k)) for k, p in grid.items() if p["passes"]]
    chosen = None if unusable or not passing else min(passing)[1]
    out = {"target_n_per_m": TARGET, "band": BAND, "unusable": unusable,
           "repeats": {m: repeat_spread(*rigid_runs[m]) for m in MASSES if len(rigid_runs[m]) == 2},
           "grid": grid, "chosen_k_mat": chosen,
           "chosen_c_mat": None if chosen is None else chosen * DAMPING_PER_STIFFNESS_S,
           "verdict": "chosen" if chosen is not None else "none (F2 registration stops, plan 3)"}
    if "dt240" in manifest and chosen is not None:
        d = manifest["dt240"]
        r240, c240 = {m: load(d["rigid"][m]) for m in MASSES}, {m: load(d["chosen"][m]) for m in MASSES}
        problems = [f"{m}: {w}" for m in MASSES for w in record_problems(r240[m], m, 240.0, None, start, combine)
                    + record_problems(c240[m], m, 240.0, chosen, start, combine)]
        out["dt240_reported"] = {"problems": problems, **grid_point(r240, c240)}
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("manifest", type=Path)
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    out = judge(json.loads(args.manifest.read_text()))
    args.json.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: out[k] for k in ("verdict", "chosen_k_mat", "chosen_c_mat", "unusable")}, indent=1))


if __name__ == "__main__":
    main()
