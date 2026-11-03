#!/usr/bin/env python3
"""The steering left-asymmetry probe batch read by the rules fixed before the results (docs/plans/
2026-11-03-steering-left-asymmetry.md section 4.3; windows, thresholds and validity in 4.1-4.2, A8 in 5.1).

Reads BATCH/runs/<arm>/{result.json, steering_probe.npz, steering_probe_arms.npz} for the arms of make_runs.py and
writes the verdicts (each one of the named outcomes, or 'undecided' with why) and the numbers behind them.

  python tools/steer_asym_judge.py BATCH --json OUT.json

Conventions (plan 4.1): epsilon = cmd - q (the rate-limited command of the same row, physical sign) feeds the PD forms;
the shortfall e = epsilon * sign(cmd) is what the rules read (positive = turned less than commanded); sizes and ratios
use |e|. A window is a segment's last half (at most 1 s), valid when both commands are constant over it and it has
>= 30 rows; only nonzero-curvature segments are read. Floor forces are -impulse / dt (tyre_floor_forces' sign).
Thresholds: T_e = 3 delta + 1 mrad, T_N = 3 delta + 2 N, T_tau = 3 delta + 0.1 N m, T_w = 3 delta + 0.01 rad/s, delta
the A0 repeat difference of that quantity in that segment (a development rule, not a statistical bound). A decision whose
margin to its boundary is below the quantity's threshold is 'indistinguishable' (undecided, never a negative).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

G = 9.81
MIN_ROWS = 30
REGISTERED_KW = {"A8r": 400e3, "A8s": 240e3, "A8h": 800e3, "A8r240": 400e3}  # plan 5.1, per wheel, N/m
H3_TARGET, H3_TOL = 0.0102, 0.005  # plan 4.3 H3: the records' same-cell difference (4.93 - 10 kg)


class Run:
    def __init__(self, folder: Path):
        self.name = folder.name
        self.result = json.loads((folder / "result.json").read_text())
        probe = np.load(folder / "steering_probe.npz", allow_pickle=False)
        self.x = probe["record"]
        self.c = {str(k): i for i, k in enumerate(probe["columns"])}
        self.tyre = probe["tyre_contacts"]  # (T, 4, fields): Jx Jy Jz ...
        arms = np.load(folder / "steering_probe_arms.npz", allow_pickle=False)
        self.arms = {k: arms[k] for k in arms.files}
        sp = self.result["steering_probe"]
        self.dt = float(sp["dt_s"])
        self.segments = sp["segments"]
        self.arm_record = sp.get("arms") or {}

    def col(self, name):
        return self.x[:, self.c[name]]

    def window(self, index: int):
        """Row indices of segment ``index``'s window, or None when the window is not valid."""
        rows = np.flatnonzero(self.col("segment") == index)
        if not len(rows):
            return None
        seconds = len(rows) * self.dt
        n = int(round(min(1.0, seconds / 2) / self.dt))
        w = rows[-n:]
        cmd_l, cmd_r = self.col("cmd_FL")[w], self.col("cmd_FR")[w]
        if len(w) < MIN_ROWS or np.ptp(cmd_l) > 1e-12 or np.ptp(cmd_r) > 1e-12:
            return None
        return w

    def seg_values(self, index: int) -> dict | None:
        w = self.window(index)
        if w is None:
            return None
        out = {}
        for side, s in (("L", "FL"), ("R", "FR")):
            cmd, q = self.col(f"cmd_{s}")[w], self.col(f"q_{s}")[w]
            sign = np.sign(cmd)
            out[f"e_{side}"] = float(np.mean((cmd - q) * sign))
        out["e_LR"] = out["e_L"] - out["e_R"]
        forces = -self.tyre[w, :, 2] / self.dt
        for i, wheel in enumerate(("FL", "FR", "RL", "RR")):
            out[f"N_{wheel}"] = float(np.nanmean(forces[:, i]))
        # PD forms (left steering joint, physical sign): reported rate and the position difference
        k = self.arms["gains"][w, 0]
        d = self.arms["damping"][w, 0]
        cap = self.arms["max_effort"][w, 0]
        eps = self.col("cmd_FL")[w] - self.col("q_FL")[w]
        qd_rep = self.col("qd_FL")[w]
        q_all = self.col("q_FL")
        qd_diff = (q_all[w] - q_all[w - 1]) / self.dt
        spring = k * eps
        tau_rep = spring - d * qd_rep
        out["spring_over_cap_frac"] = float(np.mean(np.abs(spring) > cap))
        out["cancel_frac"] = float(np.mean((np.abs(spring) > cap) & (np.abs(tau_rep) <= cap)))
        out["tau_diff_over_cap_frac"] = float(np.mean(np.abs(spring - d * qd_diff) >= cap))
        out["qd_gap_median"] = float(np.median(np.abs(qd_rep - qd_diff)))
        out["abs_tau_rep_mean"] = float(np.mean(np.abs(tau_rep)))
        return out

    def nonzero(self, labels=None):
        return [s["segment"] for s in self.segments
                if abs(float(s["kappa_cmd"])) > 0 and (labels is None or s["label"] in labels)]

    def first_still_window(self):
        t = self.col("t")
        return np.flatnonzero((t >= 0.5) & (t <= 1.0))


def threshold(kind: str, delta: float) -> float:
    return 3 * delta + {"e": 1e-3, "N": 2.0, "tau": 0.1, "w": 0.01}[kind]


def compare(x: float, boundary: float, t: float) -> str:
    """'below' / 'above' the boundary by more than t, else 'indistinguishable'."""
    if x < boundary - t:
        return "below"
    if x > boundary + t:
        return "above"
    return "indistinguishable"


def majority(results: list[str], yes: str = "yes") -> str:
    n = len(results)
    if n == 0:
        return "undecided (no valid segment)"
    if sum(r == yes for r in results) * 2 > n:
        return "yes"
    if sum(r == "no" for r in results) * 2 >= n:
        return "no"
    return "undecided (indistinguishable or invalid segments)"


def judge(batch: Path) -> dict:
    runs = {}
    for folder in sorted((batch / "runs").iterdir()):
        try:
            runs[folder.name] = Run(folder)
        except (FileNotFoundError, KeyError) as exc:
            runs[folder.name] = exc
    bad = {k: repr(v) for k, v in runs.items() if isinstance(v, Exception)}
    runs = {k: v for k, v in runs.items() if not isinstance(v, Exception)}
    out = {"missing_or_broken": bad, "verdicts": {}, "numbers": {}}
    a0s = [runs[k] for k in ("A0s1", "A0s2") if k in runs]
    if len(a0s) < 2:
        out["verdicts"]["all"] = "undecided (A0 standard needs both repeats)"
        return out
    segs = a0s[0].nonzero()
    base, delta = {}, {}
    for i in segs:
        v1, v2 = a0s[0].seg_values(i), a0s[1].seg_values(i)
        if v1 is None or v2 is None:
            continue
        base[i] = {k: (v1[k] + v2[k]) / 2 for k in v1}
        delta[i] = {k: abs(v1[k] - v2[k]) for k in v1}
    out["numbers"]["A0"] = {"segments": base, "repeat_delta": delta,
                            "bit_identical": bool(np.array_equal(a0s[0].x, a0s[1].x))}
    labels = {s["segment"]: s["label"] for s in a0s[0].segments}
    still = [i for i in base if labels[i] == "still"]
    driving = [i for i in base if labels[i] != "still"]

    def te(i, key="e_L"):
        return threshold("e", delta[i][key])

    def arm_values(name):
        if name not in runs:
            return None
        return {i: runs[name].seg_values(i) for i in base}

    def ratio_test(name, factor, keys=("e_L",), segments=None, relation="below"):
        """Per segment: |x(arm)| against factor x |x(A0)| (|A0| >= 2 mrad), 'yes' when on the relation's side."""
        vals = arm_values(name)
        if vals is None:
            return None
        res = []
        for i in (segments or list(base)):
            for key in keys:
                ref = abs(base[i][key])
                if ref < 2e-3 or vals[i] is None:
                    continue
                c = compare(abs(vals[i][key]), factor * ref, te(i, key))
                res.append("yes" if c == relation else ("no" if c != "indistinguishable" else "indistinguishable"))
        return res

    v = out["verdicts"]
    # ---- H4
    h4 = {}
    r = ratio_test("A1", 0.5)
    h4["limit_sensitive"] = majority(r) if r is not None else "undecided (A1 missing)"
    cancel = []
    for i in base:
        if abs(base[i]["e_L"]) < 2e-3:
            continue
        frac_ok = min(a.seg_values(i)["cancel_frac"] for a in a0s) >= 0.9
        gap = base[i]["qd_gap_median"]
        g = compare(gap, 0.05, threshold("w", delta[i]["qd_gap_median"]))
        cancel.append("yes" if frac_ok and g == "above" else ("no" if (not frac_ok or g == "below") else "indistinguishable"))
    r2b = ratio_test("A2b", 0.5)
    h4["reported_cancel_and_damping_sensitive"] = (
        "yes" if majority(cancel) == "yes" and r2b is not None and majority(r2b) == "yes"
        else f"no or undecided (cancel {majority(cancel)}, A2b {majority(r2b) if r2b is not None else 'missing'})")
    vals = arm_values("A2a")
    if vals is not None:
        res = []
        for i in base:
            ref = abs(base[i]["e_L"])
            if ref < 2e-3 or vals[i] is None:
                continue
            lo, hi = compare(abs(vals[i]["e_L"]), 5 * ref, te(i)), compare(abs(vals[i]["e_L"]), 20 * ref, te(i))
            res.append("yes" if lo == "above" and hi == "below" else
                       ("indistinguishable" if "indistinguishable" in (lo, hi) else "no"))
        h4["stiffness_dominated"] = majority(res)
    sens = []
    for arm in ("A2c", "A2d"):
        down, up = ratio_test(arm, 0.5), ratio_test(arm, 2.0, relation="above")
        if down is not None:
            sens.append(f"{arm}: halves {majority(down)}, doubles {majority(up)}")
    h4["dt_iteration"] = sens
    v["H4"] = h4
    # ---- H1
    h1 = {}
    hits, n = 0, 0
    for i in base:
        b = base[i]
        dn = compare(b["N_FL"] - b["N_FR"], 0.0, threshold("N", max(delta[i]["N_FL"], delta[i]["N_FR"])))
        de = compare(b["e_L"] - b["e_R"], 0.0, te(i, "e_LR"))
        n += 1
        hits += dn == "above" and de == "above"
    h1["i_diagonal_and_left_error"] = {"segments": n, "both": hits, "verdict": "yes" if n and hits >= 0.8 * n else "no"}
    if "A3b" in runs and "A3a" in runs:
        a3b, a3a = runs["A3b"], runs["A3a"]
        contacts = float(np.nansum(np.abs(a3b.tyre[:, :, 2])))
        res = []
        for i in still:
            vb = a3b.seg_values(i)
            if vb is None:
                continue
            ok = all(abs(vb[k]) + 3 * delta[i][k] <= 2e-3 for k in ("e_L", "e_R"))
            fail = any(abs(vb[k]) - 3 * delta[i][k] > 2e-3 for k in ("e_L", "e_R"))
            va = a3a.seg_values(i)
            remains = compare(va["e_LR"], 0.5 * base[i]["e_LR"], te(i, "e_LR")) if va else "indistinguishable"
            res.append("yes" if ok and remains == "above" else ("no" if fail or remains == "below" else "indistinguishable"))
        weight = sum(runs["A3a"].result["plant_backend"]["masses_kg"]) * G
        w = runs["A3a"].first_still_window()
        floor = float(np.nanmean(-runs["A3a"].tyre[w, :, 2].sum(axis=1) / runs["A3a"].dt))
        h1["ii_lifted_no_error_touching_remains"] = {
            "verdict": majority(res), "A3b_floor_impulse_sum": contacts,
            "A3a_floor_N": floor, "A3a_fixed_reaction_N": weight - floor,
            "A3a_within_preload_limit": abs(weight - floor) <= 0.2 * weight,
            "A3b_valid": contacts == 0.0}
    r6 = ratio_test("A6", 0.25, segments=still)
    if r6 is not None:
        tyre_after = (runs["A6"].result.get("steering_probe_arms_stage") or {}).get("tyre_contact_after") or []
        front = [t for t in tyre_after if "front" in t["collider"]]
        valid = bool(front) and all(t["static_friction"] == 0 and t["dynamic_friction"] == 0
                                    and t["friction_combine_mode"] == "min" for t in front)
        full = runs["A6"].arms["full"]
        rows = np.concatenate([runs["A6"].window(i) for i in still if runs["A6"].window(i) is not None])
        tangent = float(np.nanmax(full[rows, :2, 1] / np.maximum(np.abs(full[rows, :2, 0]), 1e-12)))
        h1["iii_front_frictionless"] = {"verdict": majority(r6) if valid and tangent <= 0.01 else
                                        "invalid (readback or tangential impulse)",
                                        "readback_valid": valid, "max_tangent_over_normal": tangent}
    h1["iv_tyre_compliant"] = a8_verdict(runs, base, delta, a0s)
    if "A7R" in runs:
        res = []
        for i in still:
            vr = runs["A7R"].seg_values(i)
            if vr is None:
                continue
            moved = vr["N_FR"] > vr["N_FL"] + threshold("N", 0) and abs(vr["e_R"]) > abs(vr["e_L"]) + threshold("e", 0)
            res.append(moved)
        h1["A7R_error_follows_load"] = {"segments": len(res), "moved": sum(res)}
    v["H1"] = h1
    # ---- H5 (plan 4.3: conclusions limited to the tested interventions)
    h5 = {}
    for arm in ("A9a", "A9b", "A10"):
        vals = arm_values(arm)
        if vals is None:
            continue
        diag, left, t_n, t_e = [], [], [], []
        for i in base:
            if vals[i] is None:
                continue
            b = vals[i]
            diag.append((b["N_FL"] + b["N_RR"]) - (b["N_FR"] + b["N_RL"]))
            left.append(b["e_L"] - b["e_R"])
            t_n.append(threshold("N", max(delta[i][k] for k in ("N_FL", "N_FR", "N_RL", "N_RR"))))
            t_e.append(te(i, "e_LR"))
        if not diag:
            h5[arm] = {"verdict": "undecided (no valid segment)"}
            continue
        d, e = float(np.mean(diag)), float(np.mean(left))
        dc, ec = compare(d, 0.0, max(t_n)), compare(e, 0.0, max(t_e))
        h5[arm] = {"diagonal_FLRR_minus_FRRL_N_mean": d, "eL_minus_eR_mean": e, "diagonal": dc, "left_error": ec}
    yaw = [h5.get(a) for a in ("A9a", "A9b")]
    if all(y and y.get("diagonal") == "above" and y.get("left_error") == "above" for y in yaw):
        h5["yaw"] = "insensitive to the tested spawn yaws"
    elif all(y for y in yaw):
        h5["yaw"] = "undecided"
    a10 = h5.get("A10")
    if a10 and "diagonal" in a10:
        if a10["diagonal"] == "below" and a10["left_error"] == "below":
            h5["tree_order"] = "follows the name (tree) order"
        elif a10["diagonal"] == "above" and a10["left_error"] == "above":
            h5["tree_order"] = "insensitive to the tested order change"
        else:
            h5["tree_order"] = "undecided"
    v["H5"] = h5
    # ---- H6
    if "A11" in runs:
        vals = arm_values("A11")
        segs6 = [i for i in driving if vals[i] is not None]
        if segs6:
            a, b = float(np.mean([abs(vals[i]["e_LR"]) for i in segs6])), float(np.mean([abs(base[i]["e_LR"]) for i in segs6]))
            c6 = compare(a, 0.5 * b, max(te(i, "e_LR") for i in segs6))
            v["H6"] = {"A11_mean_abs_eLR": a, "A0_mean_abs_eLR": b,
                       "contribution": {"below": "yes (front spin drive constraint contributes)", "above": "no"}.get(
                           c6, "undecided (indistinguishable)")}
        else:
            v["H6"] = {"contribution": "undecided (no valid driving segment)"}
    # ---- H2
    if "A5" in runs:
        vals = arm_values("A5")
        v["H2"] = {i: {"front_axle_change_N": vals[i]["N_FL"] + vals[i]["N_FR"] - base[i]["N_FL"] - base[i]["N_FR"],
                       "e_L_change_mrad": 1e3 * (vals[i]["e_L"] - base[i]["e_L"])}
                   for i in still if vals[i] is not None}
    # ---- H3
    if "A4a" in runs and "A4b" in runs:
        primary = [s["segment"] for s in a0s[0].segments
                   if s["label"] == "fwd" and abs(s["kappa_cmd"] + 0.36) < 1e-9]
        h3 = {"primary_segment": primary}
        lifts = {a: (runs[a].arm_record.get("lift") or {}) for a in ("A4a", "A4b")}
        h3["lift"] = lifts
        lift_ok = {}
        for a in ("A4a", "A4b"):
            rows = np.concatenate([w for w in (runs[a].window(j) for j in base) if w is not None])
            lift_ok[a] = bool(np.all(np.abs(runs[a].arms["lift_q"][rows] - 0.178) <= 0.003))
        h3["lift_within_0.178_pm_0.003"] = lift_ok
        for i in primary:
            va, vb = runs["A4a"].seg_values(i), runs["A4b"].seg_values(i)
            if not all(lift_ok.values()):
                h3["verdict"] = "invalid (lift outside 0.178 +- 0.003 m in a read window)"
                continue
            if va is None or vb is None:
                h3["verdict"] = "undecided (window not valid)"
                continue
            d = va["e_L"] - vb["e_L"]
            t = te(i)
            if abs(d) <= t:
                verdict = "undecided (indistinguishable)"
            elif abs(d - H3_TARGET) + t <= H3_TOL:
                verdict = "reproduced by the pallet mass in this approximation"
            elif d > 0:
                verdict = "direction only"
            else:
                verdict = "not reproduced in this approximation"
            h3.update(delta_e_L=d, threshold=t, verdict=verdict)
        v["H3"] = h3
    return out


def a8_verdict(runs, base, delta, a0s) -> dict:
    """Plan 4.2 A8 validity (per wheel effective stiffness within 30 %) and H1 (iv) (A8r halves the mean |e_L - e_R|,
    with the same verdict for A8r240 against A2c)."""
    out = {}
    a0w = a0s[0].first_still_window()
    z0 = np.mean([a.arms["wheel_z"][a0w] for a in a0s], axis=(0, 1))
    dz = np.abs(a0s[0].arms["wheel_z"][a0w].mean(axis=0) - a0s[1].arms["wheel_z"][a0w].mean(axis=0))
    for arm, kw in REGISTERED_KW.items():
        if arm not in runs:
            continue
        r = runs[arm]
        w = r.first_still_window()
        n = -r.tyre[w, :, 2].mean(axis=0) / r.dt
        sag = z0 - r.arms["wheel_z"][w].mean(axis=0)
        k_eff = n / np.where(np.abs(sag) > 1e-9, sag, np.nan)
        margin = 0.3 * kw
        valid = bool(np.all(np.abs(k_eff - kw) <= margin)) and bool(np.all(3 * dz < 0.3 * np.abs(sag)))
        out[arm] = {"k_eff_per_wheel": k_eff.tolist(), "sag_m": sag.tolist(), "normal_N": n.tolist(), "valid": valid,
                    "contacts_mean": np.nanmean(r.arms["full"][w, :, 3], axis=0).tolist()}

    def mean_abs(name, ref):
        segs = [i for i in base if runs[name].seg_values(i) is not None and ref.seg_values(i) is not None] \
            if name in runs else []
        return (float(np.mean([abs(runs[name].seg_values(i)["e_LR"]) for i in segs])) if segs else None,
                float(np.mean([abs(ref.seg_values(i)["e_LR"]) for i in segs])) if segs else None)

    a8r, a0 = mean_abs("A8r", a0s[0]) if "A8r" in runs else (None, None)
    out["A8r_mean_abs_eLR"], out["A0_mean_abs_eLR"] = a8r, a0
    if "A8r240" in runs and "A2c" in runs:
        x240, ref240 = mean_abs("A8r240", runs["A2c"])
        out["A8r240_mean_abs_eLR"], out["A2c_mean_abs_eLR"] = x240, ref240
        same = (a8r is not None and x240 is not None
                and ((a8r <= 0.5 * a0) == (x240 <= 0.5 * ref240)))
    else:
        same = False
    if not out.get("A8r", {}).get("valid"):
        out["verdict"] = "undecided (A8r invalid by the per-wheel stiffness check)"
    elif not same:
        out["verdict"] = "undecided (dt-dependent: 120 and 240 Hz disagree, or a 240 Hz arm missing)"
    else:
        out["verdict"] = "yes" if a8r <= 0.5 * a0 else "no"
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("batch", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    out = judge(args.batch)
    text = json.dumps(out, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    if args.json:
        args.json.write_text(text + "\n")
    print(json.dumps(out["verdicts"], indent=1, default=str))


if __name__ == "__main__":
    main()
