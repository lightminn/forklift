#!/usr/bin/env python3
"""The steering left-asymmetry probe batch read by the rules fixed before the results (docs/plans/
2026-11-03-steering-left-asymmetry.md section 4.3; windows, thresholds and validity in 4.1-4.2, A8 in 5.1, the H5
aggregation as section 4.3 states it).

Reads BATCH/runs/<arm>/{result.json, steering_probe.npz, steering_probe_arms.npz} for the arms of make_runs.py and
writes each hypothesis's outcome under its registered name, or 'undecided (<why>)', with the numbers behind it.

  python tools/steer_asym_judge.py BATCH --json OUT.json

Conventions (plan 4.1): epsilon = cmd - q (the rate-limited command of the same row, physical sign) feeds the PD forms;
the shortfall e = epsilon * sign(cmd) is what the rules read (positive = turned less than commanded); sizes and ratios
use |e|; a ratio rule is not applied (undecided) where the reference |e| is below 2 mrad. A window is a segment's last
half (at most 1 s), valid when both commands are constant over it and it has >= 30 rows; only nonzero-curvature
segments are read. Floor forces are -impulse / dt (tyre_floor_forces' sign). Thresholds per quantity: T_e = 3 delta +
1 mrad, T_N = 3 delta + 2 N, T_tau = 3 delta + 0.1 N m, T_w = 3 delta + 0.01 rad/s, delta the A0 repeat difference of
that same quantity in that segment (a development rule, not a statistical bound). A decision whose margin to its
boundary is smaller than the threshold is 'indistinguishable', which is undecided, never a negative. An absolute
bound B is met when |x| + 3 delta <= B, not met when |x| - 3 delta > B, else indistinguishable.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

G = 9.81
MIN_ROWS = 30
REF_MIN = 2e-3  # rad: a ratio rule needs a reference shortfall at least this large (plan 4.1)
REGISTERED_KW = {"A8r": 400e3, "A8s": 240e3, "A8h": 800e3, "A8r240": 400e3}  # plan 5.1, per wheel, N/m
H3_TARGET, H3_TOL = 0.0102, 0.005  # plan 4.3 H3: the records' same-cell difference (4.93 - 10 kg)
LIFT_M, LIFT_TOL = 0.178, 0.003
OP_STEER_K, OP_STEER_D, OP_STEER_CAP = 57295.78, 5729.578, 20.0  # the operating drive, engine units (rad)
WHEELS = ("FL", "FR", "RL", "RR")


def finite_or_none(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): finite_or_none(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_or_none(v) for v in value]
    if isinstance(value, np.generic):
        return finite_or_none(value.item())
    if isinstance(value, np.ndarray):
        return finite_or_none(value.tolist())
    return value


def threshold(kind: str, delta: float) -> float:
    return 3 * delta + {"e": 1e-3, "N": 2.0, "tau": 0.1, "w": 0.01}[kind]


def side(x: float, boundary: float, t: float) -> str:
    """'below' / 'above' the boundary when the margin is at least t, else 'indistinguishable' (plan 4.1)."""
    if boundary - x >= t:
        return "below"
    if x - boundary >= t:
        return "above"
    return "indistinguishable"


def tri(conditions: list[str]) -> str:
    """All of several per-condition results: 'no' if any is no, 'yes' if all are yes, else indistinguishable."""
    if "no" in conditions:
        return "no"
    if all(c == "yes" for c in conditions):
        return "yes"
    return "indistinguishable"


def majority(results: list[str]) -> str:
    """Plan 4.3 'in a majority of valid segments': yes when more than half are yes, no when at least half are no
    (so yes is out of reach), else undecided. Every valid segment counts, an indistinguishable one included."""
    results = [r for r in results if r != "invalid"]
    n = len(results)
    if n == 0:
        return "undecided (no valid segment)"
    if sum(r == "yes" for r in results) * 2 > n:
        return "yes"
    if sum(r == "no" for r in results) * 2 >= n:
        return "no"
    return "undecided (indistinguishable segments)"


def every(results: list[str]) -> str:
    """Plan 4.3 conditions without a majority clause hold in every segment read: no if any no, else yes or undecided."""
    results = [r for r in results if r != "invalid"]
    if not results:
        return "undecided (no valid segment)"
    if "no" in results:
        return "no"
    if all(r == "yes" for r in results):
        return "yes"
    return "undecided (indistinguishable segments)"


class Run:
    def __init__(self, folder: Path):
        self.name = folder.name
        self.result = json.loads((folder / "result.json").read_text())
        probe = np.load(folder / "steering_probe.npz", allow_pickle=False)
        self.x = probe["record"]
        self.c = {str(k): i for i, k in enumerate(probe["columns"])}
        self.tyre = probe["tyre_contacts"]
        arms = np.load(folder / "steering_probe_arms.npz", allow_pickle=False)
        self.arms = {k: arms[k] for k in arms.files}
        sp = self.result["steering_probe"]
        self.dt = float(sp["dt_s"])
        self.segments = sp["segments"]
        self.arm_record = sp.get("arms") or {}
        self._cache = {}

    def col(self, name):
        return self.x[:, self.c[name]]

    def window(self, index: int):
        rows = np.flatnonzero(self.col("segment") == index)
        if not len(rows):
            return None
        n = int(round(min(1.0, len(rows) * self.dt / 2) / self.dt))
        w = rows[-n:]
        if len(w) < MIN_ROWS or np.ptp(self.col("cmd_FL")[w]) > 1e-12 or np.ptp(self.col("cmd_FR")[w]) > 1e-12:
            return None
        # a clean window only: no robot body touching the scene (a tyre the floor or the shim excepted)
        if "env_contacts" not in self.arms or not np.all(self.arms["env_contacts"][w] == 0):
            return None  # NaN (the report failed) is not clean either
        return w

    def first_rows(self):
        """The first standstill rows, 0.5-1.0 s into the probe (plan 4.2 A8: the sag check)."""
        t = self.col("t")
        return np.flatnonzero((t >= 0.5) & (t <= 1.0))

    def values(self, index: int) -> dict | None:
        if index in self._cache:
            return self._cache[index]
        w = self.window(index)
        out = None
        if w is not None:
            out = {}
            for s_, s in (("L", "FL"), ("R", "FR")):
                cmd, q = self.col(f"cmd_{s}")[w], self.col(f"q_{s}")[w]
                out[f"e_{s_}"] = float(np.mean((cmd - q) * np.sign(cmd)))
            out["e_LR"] = out["e_L"] - out["e_R"]
            out["abs_LR"] = abs(out["e_L"]) - abs(out["e_R"])
            forces = -self.tyre[w, :, 2] / self.dt
            for i, wh in enumerate(WHEELS):
                out[f"N_{wh}"] = float(np.mean(forces[:, i]))
            out["dN_front"] = out["N_FL"] - out["N_FR"]
            out["diag"] = out["N_FL"] + out["N_RR"] - out["N_FR"] - out["N_RL"]
            out["N_front"] = out["N_FL"] + out["N_FR"]
            k, d, cap = (self.arms[n][w, 0] for n in ("gains", "damping", "max_effort"))
            eps = self.col("cmd_FL")[w] - self.col("q_FL")[w]
            qd_rep = self.col("qd_FL")[w]
            q = self.col("q_FL")
            qd_diff = (q[w] - q[w - 1]) / self.dt
            spring = k * eps
            out["_rows"] = {"spring": spring, "tau_rep": spring - d * qd_rep, "tau_diff": spring - d * qd_diff,
                            "cap": cap, "qd_gap": np.abs(qd_rep - qd_diff)}
            out["abs_tau_rep_mean"] = float(np.mean(np.abs(out["_rows"]["tau_rep"])))
            out["tau_diff_exceed_frac"] = float(np.mean(np.abs(out["_rows"]["tau_diff"]) >= cap))
            out["contacts_mean"] = np.nanmean(self.tyre[w, :, 6], axis=0).tolist()
        self._cache[index] = out
        return out


def scalar(v: dict) -> dict:
    return {k: x for k, x in v.items() if not k.startswith("_")}


class Batch:
    def __init__(self, batch: Path):
        self.runs, self.broken = {}, {}
        for folder in sorted(p for p in (batch / "runs").iterdir() if p.is_dir()):  # not the stall-stack files
            try:
                self.runs[folder.name] = Run(folder)
            except Exception as exc:  # noqa: BLE001 -- a run that did not finish is reported, not fatal
                self.broken[folder.name] = repr(exc)
        self.a0 = [self.runs[k] for k in ("A0s1", "A0s2") if k in self.runs]
        self.base, self.delta = {}, {}
        if len(self.a0) == 2:
            for i in self.nonzero(self.a0[0]):
                v1, v2 = self.a0[0].values(i), self.a0[1].values(i)
                if v1 is None or v2 is None:
                    continue
                keys = [k for k in v1 if not k.startswith("_") and isinstance(v1[k], float)]
                self.base[i] = {k: (v1[k] + v2[k]) / 2 for k in keys}
                self.delta[i] = {k: abs(v1[k] - v2[k]) for k in keys}
        self.labels = {s["segment"]: s for s in self.a0[0].segments} if self.a0 else {}

    @staticmethod
    def nonzero(run):
        return [s["segment"] for s in run.segments if abs(float(s["kappa_cmd"])) > 0]

    def segs(self, kind=None):
        out = []
        for i in self.base:
            label = self.labels[i]["label"]
            if kind == "still" and label != "still":
                continue
            if kind == "driving" and label == "still":
                continue
            out.append(i)
        return out

    def t(self, kind, i, key):
        return threshold(kind, self.delta[i][key])

    def ratio(self, arm, i, factor, key="e_L", relation="below") -> str:
        """|key(arm)| against factor x |key(A0)| in segment i: yes / no / indistinguishable (an invalid arm window or
        a reference under 2 mrad is indistinguishable -- undecided)."""
        v = self.runs[arm].values(i)
        if v is None:
            return "invalid"  # the arm's window is not valid: out of the denominator (plan 4.1)
        ref = abs(self.base[i][key])
        if ref < REF_MIN:
            return "indistinguishable"  # stays in the denominator, undecided
        s = side(abs(v[key]), factor * ref, self.t("e", i, key))
        if s == "indistinguishable":
            return s
        return "yes" if s == relation else "no"


# ---- arm validity (plan 4.2, the table's validity column and the readbacks of 4.1)

def dof(run: Run, name: str) -> int:
    return run.result["plant_backend"]["dof_names"].index(name)


def drive_value(run: Run, field: str, name: str) -> float:
    return float(run.result["plant_backend"][field][dof(run, name)])


def validity(b: Batch, arm: str) -> tuple[bool, list[str]]:
    try:
        return _validity(b, arm)
    except (KeyError, ValueError, TypeError, IndexError) as exc:  # a readback missing: that arm only is invalid
        return False, [f"readback missing: {exc!r}"]


def _validity(b: Batch, arm: str) -> tuple[bool, list[str]]:
    run = b.runs[arm]
    why = []
    steers = ("left_steer", "right_steer")
    if not run.result.get("success"):
        why.append("run did not succeed")
    if arm == "A1":
        if not all(abs(drive_value(run, "max_efforts", s) - 200.0) < 1e-3 for s in steers):
            why.append("steer max effort readback is not 200")
    if arm == "A2a":
        if not all(abs(drive_value(run, "drive_stiffness", s) - 0.1 * OP_STEER_K) < 1.0 for s in steers):
            why.append("steer stiffness readback is not x0.1")
    if arm == "A2b":
        if not all(abs(drive_value(run, "drive_damping", s) - 0.1 * OP_STEER_D) < 0.1 for s in steers):
            why.append("steer damping readback is not x0.1")
    if arm in ("A2c", "A8r240"):
        if abs(float(run.result["physics_scene"]["dt_s"]) - 1 / 240) > 1e-9:
            why.append("physics dt is not 1/240")
    if arm == "A2d":
        it = run.result["plant"]["articulation_solver_iterations"]
        if not any(tuple(v) == (64, 16) if isinstance(v, (list, tuple)) else False for v in
                   (it.values() if isinstance(it, dict) else [it])):
            why.append(f"solver iterations readback {it} is not 64/16")
    if arm == "A5":
        pb = run.result["plant_backend"]
        tips = [m for n, m in zip(pb["body_names"], pb["masses_kg"]) if "tip" in n]
        if not tips or not all(abs(m - 1e-4) < 1e-6 for m in tips):
            why.append(f"fork tip masses {tips} are not 1e-4 kg")
    if arm.startswith("A8"):
        compliant = run.result["plant"].get("compliant_colliders") or []
        if len(compliant) != 4 or not all("_tire_collision" in c for c in compliant):
            why.append(f"compliant colliders {compliant} are not exactly the four tyres")
    if arm == "A6":
        after = (run.result.get("steering_probe_arms_stage") or {}).get("tyre_contact_after") or []
        front = [t for t in after if "front_" in t["collider"]]
        if len(front) != 2 or not all(t["static_friction"] == 0 and t["dynamic_friction"] == 0
                                      and t["friction_combine_mode"] == "min" for t in front):
            why.append("front tyre readback is not mu 0 / min on both")
        rows = [w for w in (run.window(i) for i in b.segs("still")) if w is not None]
        full = run.arms["full"]
        if run.arm_record.get("anchor_index_problem") is not None or not rows:
            why.append("tangential impulse not verifiable: friction anchors not reported (or no clean window)")
        else:
            r_ = np.concatenate(rows)
            front = full[r_, :2, :]
            if not np.all(np.isfinite(front)) or np.any(front[:, :, 7] > 0):
                why.append("contact report missing or anchors missing in a read window")
            elif np.any(front[:, :, 1] > 0.01 * np.abs(front[:, :, 0])):
                why.append("front tangential impulse above 1 % of the normal")
    if arm == "A10":
        place = run.arm_record.get("assignment") or {}
        if place.get("steer") != ["right_steer", "left_steer"]:
            why.append(f"assignment {place.get('steer')} does not show the swapped model")
        else:
            names = run.result["plant_backend"]["dof_names"]
            if not names.index("left_steer") < names.index("right_steer"):  # left_steer is the physical right here
                why.append("engine DOF order does not put the physical right steer (named left_steer) before the left")
    if arm == "A11":
        free = run.arm_record.get("front_spin_free") or {}
        if not free or any(abs(v) > 1e-9 for k in ("stiffness", "damping", "max_effort") for v in free.get(k, [1])):
            why.append("front spin drives not freed in the readback")
    if arm in ("A4a", "A4b"):
        lift = run.arm_record.get("lift") or {}
        settled = lift.get("settled_q_m")
        if settled is None or abs(settled - LIFT_M) > LIFT_TOL:
            why.append(f"settled lift {settled} not within {LIFT_M} +- {LIFT_TOL}")
        q = run.arms["lift_q"]
        if not len(q) or np.any(np.abs(q - LIFT_M) > LIFT_TOL) or np.any(q < 0.005):
            why.append("lift left 0.178 +- 0.003 m (or touched the lower stop) during the segments")
    return not why, why


# ---- hypotheses

def h4(b: Batch) -> dict:
    out = {}
    if "A1" in b.runs:
        ok, why = validity(b, "A1")
        res = [b.ratio("A1", i, 0.5) for i in b.base]
        out["limit_sensitive"] = majority(res) if ok else f"undecided (A1 invalid: {why})"
        out["tau_diff_exceed_frac"] = {i: b.base[i]["tau_diff_exceed_frac"] for i in b.base}
    cancel = {}
    for i in b.base:
        if abs(b.base[i]["e_L"]) < REF_MIN:
            cancel[i] = "indistinguishable"
            continue
        fracs, gaps = [], []
        for a in b.a0:
            r = a.values(i)["_rows"]
            t_tau = b.t("tau", i, "abs_tau_rep_mean")
            over = np.abs(r["spring"]) - r["cap"]
            under = r["cap"] - np.abs(r["tau_rep"])
            sat = (over >= t_tau) & (under >= t_tau)
            unsure = ~sat & ((np.abs(over) < t_tau) | (np.abs(under) < t_tau))
            fracs.append((float(np.mean(sat)), float(np.mean(sat | unsure))))
            gaps.append(float(np.median(r["qd_gap"][sat])) if sat.any() else math.nan)
        if all(f >= 0.9 for f, _ in fracs) and all(math.isfinite(g) for g in gaps):
            # the velocity gap's threshold from the two repeats' medians (plan 4.1 T_w)
            gap = side(float(np.mean(gaps)), 0.05, threshold("w", abs(gaps[0] - gaps[1])))
            cancel[i] = {"above": "yes", "below": "no"}.get(gap, "indistinguishable")
        elif any(m < 0.9 for _, m in fracs):
            cancel[i] = "no"
        else:
            cancel[i] = "indistinguishable"
    out["cancel_segments"] = cancel
    holding = [i for i, c in cancel.items() if c == "yes"]
    if majority(list(cancel.values())) == "yes" and "A2b" in b.runs:
        ok, why = validity(b, "A2b")
        res = [b.ratio("A2b", i, 0.5) for i in holding]
        out["reported_cancel_and_damping_sensitive"] = majority(res) if ok else f"undecided (A2b invalid: {why})"
    else:
        out["reported_cancel_and_damping_sensitive"] = f"no or undecided (cancel {majority(list(cancel.values()))})"
    if "A2a" in b.runs:
        ok, why = validity(b, "A2a")
        res = []
        for i in b.base:
            lo, hi = b.ratio("A2a", i, 5, relation="above"), b.ratio("A2a", i, 20, relation="below")
            res.append("invalid" if "invalid" in (lo, hi) else tri([lo, hi]))
        out["stiffness_dominated"] = majority(res) if ok else f"undecided (A2a invalid: {why})"
    for arm in ("A2c", "A2d"):
        if arm not in b.runs:
            continue
        ok, why = validity(b, arm)
        res = []
        for i in b.base:
            down, up = b.ratio(arm, i, 0.5), b.ratio(arm, i, 2.0, relation="above")
            res.append("invalid" if "invalid" in (down, up) else "yes" if "yes" in (down, up) else
                       ("no" if down == "no" and up == "no" else "indistinguishable"))
        out[f"dt_iteration_{arm}"] = majority(res) if ok else f"undecided ({arm} invalid: {why})"
    named = [k for k in ("limit_sensitive", "reported_cancel_and_damping_sensitive", "stiffness_dominated")
             if out.get(k) == "yes"]
    out["outcome"] = named or ["undecided"]
    out["solver_qualifier"] = (out.get("reported_cancel_and_damping_sensitive") == "yes"
                               or any(out.get(f"dt_iteration_{a}") == "yes" for a in ("A2c", "A2d")))
    return out


def h1(b: Batch, solver_qualifier: bool) -> dict:
    out = {}
    res = []
    for i in b.base:
        dn = side(b.base[i]["dN_front"], 0.0, b.t("N", i, "dN_front"))
        de = side(b.base[i]["e_LR"], 0.0, b.t("e", i, "e_LR"))
        res.append("yes" if dn == de == "above" else ("no" if "below" in (dn, de) or
                                                         (dn != "indistinguishable" and de != "indistinguishable") else
                                                         "indistinguishable"))
    n, yes, maybe = len(res), res.count("yes"), res.count("indistinguishable")
    out["i"] = {"segments": n, "yes": yes, "indistinguishable": maybe,
                "verdict": "yes" if n and yes >= 0.8 * n else ("no" if yes + maybe < 0.8 * n else "undecided")}
    if "A3a" in b.runs and "A3b" in b.runs:
        a3a, a3b = b.runs["A3a"], b.runs["A3b"]
        cont = a3b.tyre[:, :, 6]
        a3b_valid = bool(np.all(np.isfinite(cont))) and float(np.max(cont)) == 0.0
        try:
            weight = sum(a3a.result["plant_backend"]["masses_kg"]) * G
        except (KeyError, TypeError):
            weight = math.nan  # A3a's mass readback missing: its preload limit cannot hold
        per_seg, preload_ok = [], True
        for i in b.segs("still"):
            vb, va = a3b.values(i), a3a.values(i)
            if vb is None or va is None:
                per_seg.append("invalid")  # out of the set read
                continue
            met = [("yes" if abs(vb[k]) + 3 * b.delta[i][k] <= 2e-3 else
                    "no" if abs(vb[k]) - 3 * b.delta[i][k] > 2e-3 else "indistinguishable") for k in ("e_L", "e_R")]
            ref = b.base[i]["e_LR"]
            remains = side(va["e_LR"], 0.5 * ref, b.t("e", i, "e_LR")) if abs(ref) >= REF_MIN else "indistinguishable"
            per_seg.append(tri(met + [{"above": "yes", "below": "no"}.get(remains, "indistinguishable")]))
            floor = sum(va[f"N_{w}"] for w in WHEELS)
            wheel_ok = all(abs(va[f"N_{w}"] - b.base[i][f"N_{w}"]) <= 0.2 * abs(b.base[i][f"N_{w}"]) for w in WHEELS)
            preload_ok &= bool(wheel_ok and abs(weight - floor) <= 0.2 * weight)
        verdict = every(per_seg) if a3b_valid else "undecided (A3b invalid: floor contact or no contact record)"
        out["ii"] = {"verdict": verdict, "A3b_valid": a3b_valid, "A3a_within_load_and_preload_limits": preload_ok,
                     "scope": "as measured" if preload_ok else "limited to A3a's fixed and preloaded condition"}
    if "A6" in b.runs:
        ok, why = validity(b, "A6")
        res = [b.ratio("A6", i, 0.25) for i in b.segs("still")]
        out["iii"] = {"verdict": every(res) if ok else f"undecided (A6 validity: {why})", "segments": res}
    out["iv"] = a8(b)
    if "A7R" in b.runs:
        res = []
        for i in b.segs("still"):
            v = b.runs["A7R"].values(i)
            if v is None:
                res.append("invalid")
                continue
            rows = b.runs["A7R"].window(i)
            full = b.runs["A7R"].arms["full"]
            shim = np.abs(full[rows, :, 5]).sum(axis=0)
            edge = full[rows, :, 8].sum() if full.shape[2] > 8 else math.nan
            if not (shim[1] > 0 and shim[0] == shim[2] == shim[3] == 0) or not edge == 0:
                res.append("invalid")  # the shim is not under the front right alone in this window: not read
                continue
            load = side(v["N_FR"] - v["N_FL"], 0.0, b.t("N", i, "dN_front"))
            err = side(-v["abs_LR"], 0.0, b.t("e", i, "abs_LR"))
            res.append("yes" if load == err == "above" else ("no" if "below" in (load, err) else "indistinguishable"))
        out["A7R"] = {"error_follows_load": every(res), "segments": res,
                      "shim_check": "per window: shim impulse on the front right tyre only and no side (edge) contact"}
    if "A7L" in b.runs:
        out["A7L_e_L_change_mrad"] = {i: 1e3 * (b.runs["A7L"].values(i)["e_L"] - b.base[i]["e_L"])
                                      for i in b.segs("still") if b.runs["A7L"].values(i) is not None}
    items = [out.get(k, {}).get("verdict") for k in ("i", "ii", "iii", "iv")]
    out["summary"] = ("H1 supported" if all(x == "yes" for x in items) else
                      f"items met: {[k for k, x in zip(('i', 'ii', 'iii', 'iv'), items) if x == 'yes']}")
    if solver_qualifier:
        out["summary"] += " (on top of a solver interaction)"
    return out


def a8(b: Batch) -> dict:
    out = {}
    a0w = b.a0[0].first_rows()
    z = [a.arms["wheel_z"][a0w].mean(axis=0) for a in b.a0]
    n0 = [(-a.tyre[a0w, :, 2] / a.dt).mean(axis=0) for a in b.a0]
    z0, dz, dn = (z[0] + z[1]) / 2, np.abs(z[0] - z[1]), np.abs(n0[0] - n0[1])
    for arm, kw in REGISTERED_KW.items():
        if arm not in b.runs:
            continue
        r = b.runs[arm]
        w = r.first_rows()
        n = (-r.tyre[w, :, 2] / r.dt).mean(axis=0)
        sag = z0 - r.arms["wheel_z"][w].mean(axis=0)
        with np.errstate(divide="ignore", invalid="ignore"):
            k_eff = n / sag
            # the bounds with A0's repeat differences put into the quotient itself (3 delta of load and height)
            k_lo = (n - 3 * dn) / (sag + 3 * dz)
            k_hi = np.where(sag - 3 * dz > 0, (n + 3 * dn) / (sag - 3 * dz), np.inf)
        ok_r, why = validity(b, arm)
        inside = (k_lo >= 0.7 * kw) & (k_hi <= 1.3 * kw)
        outside = (k_hi < 0.7 * kw) | (k_lo > 1.3 * kw)
        stiff = "valid" if bool(np.all(inside)) else ("invalid" if bool(np.any(outside)) else "indistinguishable")
        u = np.stack([k_lo, k_hi], axis=1)
        out[arm] = {"k_eff_per_wheel": k_eff, "k_bounds": u, "sag_m": sag, "normal_N": n, "stiffness": stiff,
                    "readback_ok": ok_r, "readback_why": why}
    rows = {}
    for arm, ref_name in (("A8r", None), ("A8s", None), ("A8h", None), ("A8o", None), ("A8r240", "A2c")):
        if arm not in b.runs or (ref_name and ref_name not in b.runs):
            continue
        segs = [i for i in b.base if b.runs[arm].values(i) is not None
                and (ref_name is None or b.runs[ref_name].values(i) is not None)]
        if not segs:
            continue
        x = float(np.mean([abs(b.runs[arm].values(i)["e_LR"]) for i in segs]))
        ref = float(np.mean([abs(b.base[i]["e_LR"]) if ref_name is None else abs(b.runs[ref_name].values(i)["e_LR"])
                             for i in segs]))
        t = max(b.t("e", i, "e_LR") for i in segs)
        rows[arm] = {"mean_abs_eLR": x, "reference": ref_name or "A0 (mean of the repeats)", "reference_mean": ref,
                     "half": ("indistinguishable" if ref < REF_MIN else
                              {"below": "yes", "above": "no"}.get(side(x, 0.5 * ref, t), "indistinguishable"))}
    out["metric"] = rows
    r120, r240 = rows.get("A8r", {}).get("half"), rows.get("A8r240", {}).get("half")
    valid = (all(out.get(a, {}).get("stiffness") == "valid" and out[a]["readback_ok"] for a in ("A8r", "A8r240"))
             and "A2c" in b.runs and validity(b, "A2c")[0])
    if not valid:
        out["verdict"] = "undecided (A8r, A8r240 or A2c invalid: per-wheel stiffness or readback)"
    elif r120 in (None, "indistinguishable") or r240 in (None, "indistinguishable"):
        out["verdict"] = "undecided (indistinguishable or missing)"
    elif r120 != r240:
        out["verdict"] = "undecided (dt-dependent: 120 and 240 Hz disagree)"
    else:
        out["verdict"] = r120
    return out


def pattern(v: dict, i: int, b: Batch) -> tuple[str, str]:
    """(larger diagonal, larger error side) of one segment: FLRR / FRRL and L / R by magnitude, or '?' within the
    thresholds."""
    d = side(v["diag"], 0.0, b.t("N", i, "diag"))
    e = side(v["abs_LR"], 0.0, b.t("e", i, "abs_LR"))
    return ({"above": "FLRR", "below": "FRRL"}.get(d, "?"), {"above": "L", "below": "R"}.get(e, "?"))


def h5(b: Batch) -> dict:
    """Plan 4.3 H5, aggregated per segment against A0: an arm 'keeps' A0's pattern when in every segment where A0's
    (diagonal, error side) is decided the arm's is the same, 'flips' it when the arm's is the mirror in every such
    segment; anything else is undecided."""
    out = {}
    ref = {i: pattern(b.base[i], i, b) for i in b.base}
    out["A0_pattern"] = ref
    mirror = {"FLRR": "FRRL", "FRRL": "FLRR", "L": "R", "R": "L"}
    for arm in ("A9a", "A9b", "A10"):
        if arm not in b.runs:
            continue
        ok, why = validity(b, arm)
        keep, flip, n = 0, 0, 0
        missing = 0
        for i, (d0, e0) in ref.items():
            if "?" in (d0, e0):
                continue
            v = b.runs[arm].values(i)
            n += 1
            if v is None:
                missing += 1  # a decided A0 segment the arm cannot show: not 'every segment'
                continue
            d, e = pattern(v, i, b)
            keep += (d, e) == (d0, e0)
            flip += (d, e) == (mirror[d0], mirror[e0])
        out[arm] = {"segments": n, "keep": keep, "flip": flip, "missing": missing, "valid": ok, "why": why,
                    "result": ("undecided (invalid)" if not ok else "undecided (missing windows)" if missing else
                               "keeps" if n and keep == n else "flips" if n and flip == n else "undecided")}
    if all(a in out for a in ("A9a", "A9b")):
        out["yaw"] = ("insensitive to the tested spawn yaws" if out["A9a"]["result"] == out["A9b"]["result"] == "keeps"
                      else "undecided")
    if "A10" in out:
        out["tree_order"] = {"flips": "follows the name (tree) order", "keeps": "insensitive to the tested order change"
                             }.get(out["A10"]["result"], "undecided")
    return out


def h6(b: Batch) -> dict:
    if "A11" not in b.runs:
        return {}
    ok, why = validity(b, "A11")
    segs = [i for i in b.segs("driving") if b.runs["A11"].values(i) is not None]
    if not segs:
        return {"contribution": "undecided (no valid driving segment)"}
    x = float(np.mean([abs(b.runs["A11"].values(i)["e_LR"]) for i in segs]))
    ref = float(np.mean([abs(b.base[i]["e_LR"]) for i in segs]))
    t = max(b.t("e", i, "e_LR") for i in segs)
    res = "indistinguishable" if ref < REF_MIN else side(x, 0.5 * ref, t)
    return {"A11_mean_abs_eLR": x, "A0_mean_abs_eLR": ref, "valid": ok, "why": why,
            "contribution": ("undecided (A11 invalid)" if not ok else
                             {"below": "yes (front spin drive constraint contributes)", "above": "no"}.get(
                                 res, "undecided (indistinguishable)"))}


def h2(b: Batch) -> dict:
    if "A5" not in b.runs:
        return {}
    ok, why = validity(b, "A5")
    rows = {}
    for i in b.base:
        v = b.runs["A5"].values(i)
        if v is None:
            continue
        rows[i] = {"label": b.labels[i]["label"], "front_axle_change_N": v["N_front"] - b.base[i]["N_front"],
                   "T_N": b.t("N", i, "N_front"), "e_L_change_mrad": 1e3 * (v["e_L"] - b.base[i]["e_L"]),
                   "T_e_mrad": 1e3 * b.t("e", i, "e_L")}
    return {"valid": ok, "why": why, "segments": rows}


def h3(b: Batch) -> dict:
    if not all(a in b.runs for a in ("A4a", "A4b")):
        return {}
    out = {}
    oks = {a: validity(b, a) for a in ("A4a", "A4b")}
    out["validity"] = oks
    primary = [s["segment"] for s in b.a0[0].segments if s["label"] == "fwd" and abs(s["kappa_cmd"] + 0.36) < 1e-9]
    out["primary_segment"] = primary
    diag = {}
    for s in b.a0[0].segments:
        if s["kappa_cmd"] >= 0 or s["label"] == "still":
            continue
        va, vb = b.runs["A4a"].values(s["segment"]), b.runs["A4b"].values(s["segment"])
        if va is not None and vb is not None:
            diag[s["segment"]] = {"label": s["label"], "kappa": s["kappa_cmd"], "delta_e_L": va["e_L"] - vb["e_L"]}
    out["right_turn_diagnostics"] = diag
    if not all(ok for ok, _ in oks.values()):
        out["verdict"] = "undecided (A4 invalid)"
        return out
    for i in primary:
        va, vb = b.runs["A4a"].values(i), b.runs["A4b"].values(i)
        if va is None or vb is None or i not in b.delta:
            out["verdict"] = "undecided (window not valid in an arm or in A0)"
            continue
        d, t = va["e_L"] - vb["e_L"], b.t("e", i, "e_L")
        inside = side(abs(d - H3_TARGET), H3_TOL, t)
        zero = side(d, 0.0, t)
        if inside == "below":
            verdict = "reproduced by the pallet mass in this approximation"
        elif inside == "indistinguishable":
            verdict = "undecided (indistinguishable from the tolerance edge)"
        elif zero == "above":
            verdict = "direction only"
        elif zero == "below":
            verdict = "not reproduced in this approximation"
        else:
            verdict = "undecided (indistinguishable from zero)"
        out.update(delta_e_L=d, T_e=t, verdict=verdict)
    return out


def judge(batch: Path) -> dict:
    b = Batch(batch)
    out = {"broken_runs": b.broken, "verdicts": {}, "numbers": {}}
    if len(b.a0) < 2:
        out["verdicts"]["all"] = "undecided (A0 standard needs both repeats)"
        return out
    long = [b.runs[k] for k in ("A0l1", "A0l2") if k in b.runs]
    out["numbers"]["determinism"] = {
        "standard_bit_identical": bool(np.array_equal(b.a0[0].x, b.a0[1].x)),
        "long_bit_identical": bool(np.array_equal(long[0].x, long[1].x)) if len(long) == 2 else None}
    out["numbers"]["A0"] = {i: {"label": b.labels[i]["label"], "speed": b.labels[i]["speed_mps"],
                                "kappa": b.labels[i]["kappa_cmd"], **b.base[i], "repeat_delta": b.delta[i]}
                            for i in b.base}
    out["numbers"]["moment_balance"] = (
        "not evaluated: friction anchors not reported in this build (anchor_index_problem in the A0 record), so "
        "contact moments are not used (plan 4.1) -- H1 reads loads only"
        if b.a0[0].arm_record.get("anchor_index_problem") is not None else "anchors reported: evaluate before use")
    out["numbers"]["validity"] = {a: validity(b, a) for a in b.runs if a not in ("A0s1", "A0s2", "A0l1", "A0l2")}
    v = out["verdicts"]
    v["H4"] = h4(b)
    v["H1"] = h1(b, v["H4"]["solver_qualifier"])
    v["H5"] = h5(b)
    v["H6"] = h6(b)
    v["H2"] = h2(b)
    v["H3"] = h3(b)
    return finite_or_none(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("batch", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    out = judge(args.batch)
    text = json.dumps(out, indent=1, allow_nan=False)
    if args.json:
        args.json.write_text(text + "\n")
    print(json.dumps(out["verdicts"], indent=1, allow_nan=False))


if __name__ == "__main__":
    main()
