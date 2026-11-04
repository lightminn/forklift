"""F2 2x2 tool (plan 4.3, 4.4, 6): effects and interaction, validity, the dt condition on three contrasts, and the
candidate rule with its preconditions and the F2 self-degradation escalation."""

import copy
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f2_2x2 as J  # noqa: E402

ARMS = J.ARMS


def _static(fz):
    w = {wh: {"fz": {"mean": f, "min": f, "max": f}} for wh, f in zip(J.WHEELS, fz)}
    return {"loads": {ld: {"wheels": w} for ld in J.LOADS}}


def _steer(lr_mrad):
    """20 judged segments whose |e_L - e_R| is lr_mrad (FL short by that, FR exact)."""
    segs, i = [], 0
    for label in ("still", "fwd", "rev", "fwd_fast", "rev_fast"):
        for k in (0.29, -0.29, 0.36, -0.36):
            cmd = 0.2 * (1 if k > 0 else -1)
            segs.append({"segment": i, "label": label, "kappa_cmd": k, "target_FL": cmd, "q_FL": cmd - (lr_mrad * 1e-3)
                         * (1 if cmd > 0 else -1), "target_FR": cmd, "q_FR": cmd})
            i += 1
    segs.append({"segment": i, "label": "stop", "kappa_cmd": 0.0, "target_FL": 0, "q_FL": 0, "target_FR": 0, "q_FR": 0})
    return {"steering_probe": {"segments": segs}}


def _trial(i, e=0.003, outcome="release"):
    return {"id": f"t{i}", "e_stop_rad": e, "outcome": outcome,
            "events": {"first_at_endpoint": {"tick": 1}, "first_exceed_after_endpoint": None},
            "settle": {"converged": True}, "wheel_load_ok": True, "foreign_contacts": 0}


def _cusp(n=10, **kw):
    return {"trials": [_trial(i, **kw) for i in range(n)]}


def _point(lab, accepted=True, ge=0.0, info=55.0):
    ev = {"accepted": accepted, "min_information": info, "goal_error_vs_truth": [ge, 0.0, 0.0]}
    return {"label": lab, "pose_ok": True, "settle": {"converged": True},
            "evals": {f"{c}/e{k}": dict(ev) for c in J.JUDGED_CASTS + ("c", "d", "e") for k in (0, 1)}}


def _delivery(n=22, **kw):
    return {"points": [_point(f"p{i}", **kw) for i in range(n)]}


def _manifest(files, static=None, steer=None, cusp=None, delivery=None):
    static = static or {a: (74.0, 74.0, 53.0, 53.0) for a in ARMS}
    steer = steer or {"O": 20.0, "F1": 18.0, "O+F2": 4.0, "F1+F2": 3.5}
    man = {"static": {}, "steer": {}, "cusp": {"120": {}}, "delivery": {"120": {}}, "calibration": "calib"}
    files["calib"] = {"chosen_k_mat": 140000.0}
    for dt, scale in (("120", 1.0), ("240", 1.0)):
        man["static"][dt], man["steer"][dt] = {}, {}
        for a in ARMS:
            for rep in ("", "#b"):
                files[f"st{dt}{a}{rep}"] = _static(static[a])
                files[f"sr{dt}{a}{rep}"] = _steer(steer[a] * scale)
                man["static"][dt][f"{a}{rep}"] = f"st{dt}{a}{rep}"
                man["steer"][dt][f"{a}{rep}"] = f"sr{dt}{a}{rep}"
    for a in ARMS:
        c = (cusp or {}).get(a) or _cusp()
        d = (delivery or {}).get(a) or _delivery()
        for k in ("a", "b", "r"):
            files[f"cu{a}{k}"], files[f"de{a}{k}"] = copy.deepcopy(c), copy.deepcopy(d)
        man["cusp"]["120"][a] = {k: f"cu{a}{k}" for k in ("a", "b", "r")}
        man["delivery"]["120"][a] = {k: f"de{a}{k}" for k in ("a", "b", "r")}
    return man


def test_effects_and_interaction():
    v = {"O": {"s": 10.0}, "F1": {"s": 8.0}, "O+F2": {"s": 3.0}, "F1+F2": {"s": 2.0}}
    e = J.effects(v)
    assert e["F1"]["s"] == -2 and e["F2"]["s"] == -7 and e["F1|F2"]["s"] == -1 and e["I"]["s"] == pytest.approx(1.0)


def test_default_candidate_and_dt_pass():
    files = {}
    out = J.run(_manifest(files), load=files.__getitem__)
    # loads that do not change leave M1 with too few distinguishable states: dt undecided, so no candidate (plan 4.4)
    assert out["dt_condition"]["F2|O"]["M2"]["verdict"] == "pass"
    assert out["dt_condition"]["F2|O"]["M1"]["verdict"] == "undecided" and out["candidate"]["candidate"] is None
    files = {}
    st = {"O": (110, 30, 30, 80), "F1": (95, 30, 30, 80), "O+F2": (70, 70, 55, 55), "F1+F2": (60, 60, 55, 55)}
    out = J.run(_manifest(files, static=st), load=files.__getitem__)
    assert out["dt_condition"]["F2|O"]["M1"]["verdict"] == "pass"
    assert out["candidate"]["candidate"] == "F1+F2" and not out["candidate"]["preconditions"]
    assert out["effects"]["M2"]["120"]["F2"]["median"] == pytest.approx(0.016)


def test_dt_dependent_blocks_the_candidate():
    files = {}
    st = {"O": (110, 30, 30, 80), "F1": (95, 30, 30, 80), "O+F2": (70, 70, 55, 55), "F1+F2": (60, 60, 55, 55)}
    man = _manifest(files, static=st)
    # at 240 Hz the F2 effect on |eL - eR| vanishes: same sign possible but the size does not hold
    for rep in ("", "#b"):
        files[f"sr240O+F2{rep}"] = J_steer = _steer(19.9)
        files[f"sr240F1+F2{rep}"] = J_steer
    out = J.run(man, load=files.__getitem__)
    assert out["dt_condition"]["F2|O"]["M2"]["verdict"] in ("dt-dependent", "undecided")
    assert out["candidate"]["candidate"] is None


def test_f1_worse_under_f2_picks_f2_alone_and_self_degradation():
    files = {}
    cusp = {"F1+F2": _cusp(e=0.012), "O+F2": _cusp(e=0.003)}
    st = {"O": (110, 30, 30, 80), "F1": (95, 30, 30, 80), "O+F2": (70, 70, 55, 55), "F1+F2": (60, 60, 55, 55)}
    out = J.run(_manifest(files, cusp=cusp, static=st), load=files.__getitem__)
    assert out["candidate"]["f1_under_f2"]["M3"]["worse"] == 10 and out["candidate"]["candidate"] == "F2"
    # F2 itself worsens the delivery: escalated
    files = {}
    dlv = {"O+F2": _delivery(accepted=False), "F1+F2": _delivery(accepted=False)}
    out = J.run(_manifest(files, delivery=dlv, static=st), load=files.__getitem__)
    assert out["candidate"]["escalate_self_degradation"] and "to the lead" in out["candidate"]["verdict"]


def test_preconditions():
    st = {"O": (110, 30, 30, 80), "F1": (95, 30, 30, 80), "O+F2": (70, 70, 55, 55), "F1+F2": (60, 60, 55, 55)}
    files = {}
    man = _manifest(files, static=st)
    files["cuO+F2r"]["trials"][0]["e_stop_rad"] += 0.001  # order-sensitive
    out = J.run(man, load=files.__getitem__)
    assert out["candidate"]["candidate"] is None and any("M3" in p for p in out["candidate"]["preconditions"])
    files = {}
    man = _manifest(files, static=st)
    files["calib"] = {"chosen_k_mat": None}
    assert J.run(man, load=files.__getitem__)["candidate"]["candidate"] is None
    files = {}
    man = _manifest(files, static=st)
    for k in ("a", "b", "r"):
        for p in files[f"deF1+F2{k}"]["points"][:5]:
            p["pose_ok"] = False
    out = J.run(man, load=files.__getitem__)
    assert any("M4 valid 17" in p for p in out["candidate"]["preconditions"])
    files = {}
    man = _manifest(files, static=st)
    files["st120O#b"] = _static((111, 30, 30, 80))  # not deterministic
    assert J.run(man, load=files.__getitem__)["candidate"]["candidate"] is None
