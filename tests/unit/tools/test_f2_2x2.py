"""F2 2x2 tool (plan 4.3, 4.4, 6, amendment 1): effects and interaction per state, validity (determinism, physical
validity, population, the required reversed runs), the dt condition on three contrasts, and the candidate rule with
its preconditions and escalations."""

import copy
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1v2_probe_judge as PJ  # noqa: E402
import f2_2x2 as J  # noqa: E402

ARMS = J.ARMS
REGISTRY = {"versions": [{"id": "v", "sha256": "h" * 64, "cusp_legs": 9}], "tracker_configs": {}}
ST = {"O": (110, 30, 30, 80), "F1": (95, 30, 30, 80), "O+F2": (70, 70, 55, 55), "F1+F2": (60, 60, 55, 55)}
LR = {"O": 20.0, "F1": 18.0, "O+F2": 4.0, "F1+F2": 3.5}


def _static(fz, order="forward"):
    w = {wh: {"fz": {"mean": f, "min": f, "max": f}} for wh, f in zip(J.WHEELS, fz)}
    rec = {"wheels": w, "start": {"converged": True}, "foreign_contacts": 0, "wheel_load_ratio": 1.0}
    return {"loads": {ld: copy.deepcopy(rec) for ld in J.LOADS}, "order": order}


def _steer(lr_mrad, digest="d"):
    vals, i = {}, 0
    for label in J.M2_LABELS:
        for k in (0.29, -0.29, 0.36, -0.36):
            vals[f"{label}/{k:+.2f}/{i}"] = lr_mrad * 1e-3
            i += 1
    return {"values": vals, "digest": digest}


def _trial(tid, e=0.003, outcome="release"):
    return {"id": tid, "e_stop_rad": e, "outcome": outcome, "path_sha256": PJ.expected_trials(REGISTRY)[tid] or "s",
            "events": {"first_at_endpoint": {"tick": 1}, "first_exceed_after_endpoint": None},
            "settle": {"converged": True}, "wheel_load_ok": True, "foreign_contacts": 0}


def _cusp(dt="120", **kw):
    return {"trials": [_trial(i, **kw) for i in J.expected_cusp(REGISTRY, dt)]}


def _point(lab, accepted=True, ge=(0.0, 0.0, 0.0), info=55.0):
    ev = {"accepted": accepted, "min_information": info, "goal_error_vs_truth": list(ge)}
    return {"label": lab, "pose_ok": True, "settle": {"converged": True},
            "evals": {f"{c}/{e}": dict(ev) for c in J.ALL_CASTS for e in J.ESTIMATES}}


def _labels():
    import f1v2_probe_core as core
    return [p["label"] for p in core.f2_grid((0.0, 0.0, 0.0), 0.4, (0.0, 0.0), 0.0)]


def _delivery(**kw):
    return {"points": [_point(lab, **kw) for lab in _labels()]}


def _manifest(files, static=None, lr=None, cusp=None, delivery=None, v4=True):
    static, lr = static or ST, lr or LR
    man = {"static": {}, "steer": {}, "cusp": {"120": {}, "240": {}}, "delivery": {"120": {}},
           "calibration": "calib", "registry": "reg", "plant_gates": {"F2": "v4", "F1+F2": "v4"}}
    files["calib"], files["reg"], files["v4"] = {"chosen_k_mat": 140000.0}, REGISTRY, {"v4_v2_preserved": v4}
    for dt in ("120", "240"):
        man["static"][dt], man["steer"][dt] = {}, {}
        for a in ARMS:
            for rep in ("", "#b") + (("#r",) if dt == "120" else ()):
                files[f"st{dt}{a}{rep}"] = _static(static[a], "reverse" if rep == "#r" else "forward")
                man["static"][dt][f"{a}{rep}"] = f"st{dt}{a}{rep}"
            for rep in ("", "#b"):
                files[f"sr{dt}{a}{rep}"] = _steer(lr[a], digest=f"{dt}{a}")
                man["steer"][dt][f"{a}{rep}"] = f"sr{dt}{a}{rep}"
    for a in ARMS:
        c = (cusp or {}).get(a) or _cusp()
        d = (delivery or {}).get(a) or _delivery()
        for k in ("a", "b", "r"):
            files[f"cu{a}{k}"], files[f"de{a}{k}"] = copy.deepcopy(c), copy.deepcopy(d)
        for k in ("a", "b"):
            files[f"c240{a}{k}"] = _cusp("240")
        man["cusp"]["120"][a] = {k: f"cu{a}{k}" for k in ("a", "b", "r")}
        man["cusp"]["240"][a] = {k: f"c240{a}{k}" for k in ("a", "b")}
        man["delivery"]["120"][a] = {k: f"de{a}{k}" for k in ("a", "b", "r")}
    return man


def _run(man, files):
    return J.run(man, load=files.__getitem__, steer_load=files.__getitem__)


def test_effects_and_interaction():
    v = {"O": {"s": 10.0}, "F1": {"s": 8.0}, "O+F2": {"s": 3.0}, "F1+F2": {"s": 2.0}}
    e = J.effects(v)
    assert e["F1"]["s"] == -2 and e["F2"]["s"] == -7 and e["F1|F2"]["s"] == -1 and e["I"]["s"] == pytest.approx(1.0)


def test_default_candidate_dt_and_per_state_output():
    files = {}
    out = _run(_manifest(files), files)
    assert out["candidate"]["candidate"] == "F1+F2" and not out["candidate"]["preconditions"]
    assert out["dt_condition"]["F2|O"]["M1"]["verdict"] == "pass"
    s = out["effects"]["M2"]["120"]["F2"]
    assert s["median_abs"] == pytest.approx(0.016) and len(s["largest"]) == J.TOP
    assert "still/+0.29/0" in out["effects_per_state"]["M2"]["120"]["F2"]
    assert "M3 e_stop (signed)" in out["effects"] and "M3 |e_stop|" in out["effects"]
    # loads that do not change leave M1 with too few distinguishable states: dt undecided, no candidate (plan 4.4)
    files = {}
    flat = {a: (74.0, 74.0, 53.0, 53.0) for a in ARMS}
    out = _run(_manifest(files, static=flat), files)
    assert out["dt_condition"]["F2|O"]["M1"]["verdict"] == "undecided" and out["candidate"]["candidate"] is None


def test_dt_dependent_blocks_the_candidate():
    files = {}
    man = _manifest(files)
    for rep in ("", "#b"):
        files[f"sr240O+F2{rep}"] = _steer(19.9, digest="x")
        files[f"sr240F1+F2{rep}"] = _steer(19.9, digest="y")
    out = _run(man, files)
    assert out["dt_condition"]["F2|O"]["M2"]["verdict"] in ("dt-dependent", "undecided")
    assert out["candidate"]["candidate"] is None


def test_f1_worse_under_f2_picks_f2_alone_and_escalations():
    files = {}
    out = _run(_manifest(files, cusp={"F1+F2": _cusp(e=0.012), "O+F2": _cusp(e=0.003)}), files)
    assert out["candidate"]["f1_under_f2"]["M3"]["worse"] == len(J.expected_cusp(REGISTRY, "120"))
    assert out["candidate"]["candidate"] == "F2"
    files = {}
    dlv = {"O+F2": _delivery(accepted=False), "F1+F2": _delivery(accepted=False)}
    out = _run(_manifest(files, delivery=dlv), files)
    assert out["candidate"]["escalate_self_degradation"] and "to the lead" in out["candidate"]["verdict"]
    files = {}
    out = _run(_manifest(files, v4=False), files)
    assert out["candidate"]["escalate_v4"] and "V4" in out["candidate"]["verdict"]
    files = {}
    man = _manifest(files)
    del man["plant_gates"]
    assert _run(man, files)["candidate"]["candidate"] is None


def test_m4_mixed_axes_count_both_ways():
    x = {"points": [_point("p", ge=(0.002, 0.0, 0.0))]}
    y = {"points": [_point("p", ge=(0.0, 0.002, 0.0))]}
    c = J.m4_counts(x, y)
    assert c["worse"] == c["better"] == len(J.JUDGED_CASTS) * 2
    assert J.m4_counts(y, x)["worse"] == c["worse"]


def test_population_validity_and_reversed_runs():
    files = {}
    man = _manifest(files)
    for k in ("a", "b", "r"):
        files[f"cuO{k}"]["trials"] = files[f"cuO{k}"]["trials"][:-1]  # one registered trial missing
    out = _run(man, files)
    assert out["candidate"]["candidate"] is None and out["validity"]["M3"]["120"]["O"]["population"]
    files = {}
    man = _manifest(files)
    del man["cusp"]["120"]["F1"]["r"]  # a required reversed run not made
    assert _run(man, files)["candidate"]["candidate"] is None
    files = {}
    man = _manifest(files)
    del man["static"]["120"]["O+F2#r"]
    assert _run(man, files)["candidate"]["candidate"] is None
    files = {}
    man = _manifest(files)
    files["st120F1+F2#r"] = _static((60.5, 60, 55, 55), "reverse")  # order-sensitive beyond 0.2 N
    assert _run(man, files)["validity"]["M1"]["120"]["F1+F2"]["order_sensitive"] is True
    files = {}
    man = _manifest(files)
    bad = _static(ST["O"])
    bad["loads"]["loaded"].update(foreign_contacts=100, wheel_load_ratio=0.5)
    files["st120O"] = files["st120O#b"] = bad
    assert _run(man, files)["candidate"]["candidate"] is None
    files = {}
    man = _manifest(files)
    files["sr120O#b"] = _steer(LR["O"], digest="other")  # not deterministic
    assert _run(man, files)["candidate"]["candidate"] is None
    files = {}
    man = _manifest(files)
    s = _steer(LR["F1"], digest="120F1")
    s["values"][next(iter(s["values"]))] = None  # a window not clean
    files["sr120F1"] = files["sr120F1#b"] = s
    assert _run(man, files)["candidate"]["candidate"] is None
    files = {}
    man = _manifest(files)
    for k in ("a", "b", "r"):
        files[f"deO{k}"]["points"] = files[f"deO{k}"]["points"][:-1]
    assert _run(man, files)["candidate"]["candidate"] is None


def test_240_subset_population():
    want = J.expected_cusp(REGISTRY, "240")
    legs = {i.rsplit("/", 2)[0] for i in want if i.startswith("v/")}
    assert legs == {"v/leg0", "v/leg8"} and sum(i.startswith("synthetic") for i in want) == 4 * 4
