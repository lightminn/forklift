"""F1 v2 probe judge (plan sections 2-3, amendments 1-2): every precondition makes the verdict unjudgeable, every margin
and crossing makes it different, and each pair over a margin is listed."""

import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1v2_probe_judge as J  # noqa: E402

LABELS = list(J.REPRESENTATIVE) + ["r1_d1.40_y-9.000_w0.00"]


def _eval(info=52.0, accepted=True, beams=540, ge=(0.0, 0.0, 0.0)):
    return {"accepted": accepted, "min_information": info, "live_beams": beams, "goal_error_vs_truth": list(ge),
            "starts": [{"information": info, "inliers": 400}, {"information": info + 3, "inliers": 410}]}


def _point(label, info=52.0):
    evals = {f"{c}/{e}": _eval(info) for c in J.JUDGED_CASTS + J.PATH_CASTS for e in J.ESTIMATES}
    return {"label": label, "d_m": 1.5, "target_laser": [0, 0, 0], "pose_ok": True,
            "settle": {"converged": True}, "evals": evals,
            "state": {"lift_m": 0.178, "base": [0, 0, 1e-6], "q": [1, 0, 0, 0], "pitch_rad": 0.0}}


def _run(info=52.0, stops=()):
    pts = [_point(lab, info) for lab in LABELS] + [_point(s, info) for s in stops]
    return {"points": pts}


STOPS = [{"label": "v1_O_s0", "arm": "O", "recorded": {"lift_m": 0.178, "base_z_m": 1e-6, "base_tilt_rad": 0.0}},
         {"label": "v1_F1_s0", "arm": "F1", "recorded": {"lift_m": 0.178, "base_z_m": 1e-6, "base_tilt_rad": 0.0}}]
STOP_LABELS = [s["label"] for s in STOPS]


def _delivery(o=None, f1=None, stage0=True, o_b=None, rev=None):
    o = o or _run(stops=STOP_LABELS)
    f1 = f1 or _run(stops=STOP_LABELS)
    return J.delivery({"all_bit_identical": stage0}, STOPS, [o, o_b or copy.deepcopy(o)], rev or copy.deepcopy(o),
                      [f1, copy.deepcopy(f1)], copy.deepcopy(f1))


def test_delivery_same_when_identical():
    out = _delivery()
    assert out["verdict"] == "same" and out["over_count"] == 0 and out["grid_points"] == len(LABELS)
    assert out["pairs"] == len(LABELS) * len(J.JUDGED_CASTS) * len(J.ESTIMATES)


def test_delivery_delta_I_and_crossing_and_correction_error():
    f1 = _run(stops=STOP_LABELS)
    p = next(p for p in f1["points"] if p["label"] == LABELS[-1])
    p["evals"]["b3/e1"] = _eval(52.6, beams=538)
    p["evals"]["a/e0"] = _eval(49.0, accepted=False)
    p["evals"]["b0/e0"] = _eval(52.0, ge=(0.0, 0.0011, 0.0))
    out = _delivery(f1=f1)
    assert out["verdict"] == "different" and out["over_count"] == 3
    why = {(r["cast"], r["estimate"]): r["why"] for r in out["over"]}
    assert why[("b3", "e1")] == ["delta_I"] and "accept_crossing_O_only" in why[("a", "e0")]
    assert why[("b0", "e0")] == ["correction_error"]
    r = next(r for r in out["over"] if r["cast"] == "b3")
    assert r["beams_diff"] == -2 and abs(r["delta_I"] - 0.6) < 1e-9 and r["inliers_diff"] == 0
    # (c)/(d) are reported, never judged
    f1b = _run(stops=STOP_LABELS)
    next(p for p in f1b["points"] if p["label"] == LABELS[0])["evals"]["c/e0"] = _eval(60.0)
    assert _delivery(f1=f1b)["verdict"] == "same"


def test_delivery_preconditions_make_it_unjudgeable():
    assert _delivery(stage0=False)["verdict"] == "unjudgeable"
    o = _run(stops=STOP_LABELS)
    o_b = copy.deepcopy(o)
    o_b["points"][0]["evals"]["a/e0"]["min_information"] += 1e-12
    assert _delivery(o=o, o_b=o_b)["verdict"] == "unjudgeable"
    rev = copy.deepcopy(o)
    next(p for p in rev["points"] if p["label"] == J.REPRESENTATIVE[2])["state"]["lift_m"] += 1e-9
    out = _delivery(o=o, rev=rev)
    assert out["verdict"] == "unjudgeable" and out["determinism"]["reverse_O"]["differ"] == [J.REPRESENTATIVE[2]]
    # a reversed run differing away from the five representative points is not a failure
    rev2 = copy.deepcopy(o)
    next(p for p in rev2["points"] if p["label"] == LABELS[-1])["state"]["lift_m"] += 1e-9
    assert _delivery(o=o, rev=rev2)["verdict"] == "same"
    # the operating comparison: the O stop's lift 1.1 mm off
    o3 = _run(stops=STOP_LABELS)
    next(p for p in o3["points"] if p["label"] == "v1_O_s0")["state"]["lift_m"] += 0.0011
    assert _delivery(o=o3)["verdict"] == "unjudgeable"
    # a pose out of tolerance anywhere on the grid
    o4 = _run(stops=STOP_LABELS)
    o4["points"][1]["pose_ok"] = False
    assert _delivery(o=o4)["verdict"] == "unjudgeable"


def test_delivery_settle_rules():
    f1 = _run(stops=STOP_LABELS)
    f1["points"][0]["settle"]["converged"] = False
    out = _delivery(f1=f1)
    assert out["verdict"] == "different" and out["settled_one_arm_only"] == [f1["points"][0]["label"]]
    o = _run(stops=STOP_LABELS)
    o["points"][0]["settle"]["converged"] = False
    assert _delivery(o=o, f1=f1)["verdict"] == "unjudgeable"


def _trial(i, e=0.02, outcome="release", reached=True, exceed=False, load="loaded", config="transport"):
    ev = {"first_at_endpoint": {"yaw_error_rad": e} if reached else None,
          "first_exceed_after_endpoint": {"tick": 5} if exceed else None}
    return {"id": f"v/leg{i}/{config}/{load}", "version": "v", "leg": i, "config": config, "load": load,
            "e_stop_rad": e if outcome != "timeout" else None, "stop": None if outcome == "timeout" else {"t_s": 3.0},
            "outcome": outcome, "events": ev, "trajectory_sha256": f"h{i}{e}", "ticks": 400,
            "wheel_load_ok": True, "settle": {"converged": True}, "foreign_contacts": 0, "start_offset_m": 0.001}


def _cusp_run(trials):
    return {"trials": trials}


def _cusp(o_trials, f1_trials, o_off=None, f1_off=None):
    o, f1 = _cusp_run(o_trials), _cusp_run(f1_trials)
    return J.cusp([o, copy.deepcopy(o)], o_off or copy.deepcopy(o), [f1, copy.deepcopy(f1)], f1_off or copy.deepcopy(f1))


def test_cusp_same_and_margins():
    base = [_trial(i) for i in range(4)]
    assert _cusp(base, copy.deepcopy(base))["verdict"] == "same"
    f1 = copy.deepcopy(base)
    f1[0] = _trial(0, e=0.0251)  # |e| difference 0.0051 > 0.005
    f1[1] = _trial(1, e=-0.024)  # 0.004: inside (absolute values compared)
    f1[2] = _trial(2, outcome="failed_endpoint_heading")
    f1[3] = _trial(3, exceed=True)
    out = _cusp(base, f1)
    assert out["verdict"] == "different" and out["over_count"] == 3
    why = {r["leg"]: r["why"] for r in out["over"]}
    assert why[0] == ["delta_e_stop"] and why[2] == ["outcome"] and why[3] == ["exceed_crossing_F1_only"]
    assert out["by_combination"]["transport/loaded"]["F1"]["exceed_rate"] == 0.25


def test_cusp_one_arm_and_both_arm_rules():
    base = [_trial(i) for i in range(3)]
    f1 = copy.deepcopy(base)
    f1[0] = _trial(0, reached=False)
    f1[1] = _trial(1, outcome="timeout")
    out = _cusp(base, f1)
    assert out["verdict"] == "different"
    assert "endpoint_reached_one_arm_only" in out["over"][0]["why"] and "stop_one_arm_only" in out["over"][1]["why"]
    both = copy.deepcopy(base)
    both[2] = _trial(2, outcome="timeout")
    out = _cusp(both, copy.deepcopy(both))
    assert out["verdict"] == "unjudgeable"
    c = copy.deepcopy(base)
    c[0]["foreign_contacts"] = 3
    assert "foreign_contact_one_arm_only" in _cusp(base, c)["over"][0]["why"]
    assert _cusp(c, copy.deepcopy(c))["verdict"] == "unjudgeable"


def test_cusp_preconditions():
    base = [_trial(i) for i in range(3)]
    off = _cusp_run(copy.deepcopy(base))
    off["trials"][1]["trajectory_sha256"] = "x"
    assert _cusp(base, copy.deepcopy(base), o_off=off)["verdict"] == "unjudgeable"
    bad = copy.deepcopy(base)
    bad[0]["wheel_load_ok"] = False
    assert _cusp(base, bad)["verdict"] == "unjudgeable"
    o, f1 = _cusp_run(base), _cusp_run(copy.deepcopy(base))
    o_b = copy.deepcopy(o)
    o_b["trials"][0]["ticks"] += 1
    assert J.cusp([o, o_b], copy.deepcopy(o), [f1, copy.deepcopy(f1)], copy.deepcopy(f1))["verdict"] == "unjudgeable"
