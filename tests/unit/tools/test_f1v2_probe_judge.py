"""F1 v2 probe judge (plan sections 2-3, amendments 1-2): every precondition makes the verdict unjudgeable, every margin
and crossing makes it different, and each pair over a margin is listed."""

import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1v2_probe_judge as J  # noqa: E402

LABELS = list(J.GRID_LABELS)
assert len(LABELS) == 132 and set(J.REPRESENTATIVE) <= set(LABELS)


def _eval(info=52.0, accepted=True, beams=540, ge=(0.0, 0.0, 0.0)):
    return {"accepted": accepted, "min_information": info, "live_beams": beams, "goal_error_vs_truth": list(ge),
            "starts": [{"information": info, "inliers": 400}, {"information": info + 3, "inliers": 410}]}


def _point(label, info=52.0):
    evals = {f"{c}/{e}": _eval(info) for c in J.JUDGED_CASTS + J.PATH_CASTS for e in J.ESTIMATES}
    return {"label": label, "d_m": 1.5, "target_laser": [0, 0, 0], "laser": [0.0, 0.0, 0.0], "pose_ok": True,
            "laser_origin_world": [0.0, 0.0, 0.2], "d_origin_z_m": 0.2,
            "settle": {"converged": True}, "evals": evals,
            "state": {"lift_m": 0.178, "base": [0, 0, 1e-6], "q": [1, 0, 0, 0], "pitch_rad": 0.0}}


def _run(info=52.0, stops=(), d_reference="own"):
    pts = [_point(lab, info) for lab in LABELS] + [_point(s, info) for s in stops]
    return {"points": pts, "order": "forward", "d_reference": d_reference}


STOPS = [{"label": lab, "arm": lab.split("_")[1], "recorded": {"lift_m": 0.178, "base_z_m": 1e-6,
                                                               "base_tilt_rad": 0.0}} for lab in J.V1_STOP_LABELS]
STOP_LABELS = [s["label"] for s in STOPS]
STAGE0 = {"all_bit_identical": True,
          "rows": [{"match": f"/x/{n}/run/delivery_match_1.npz", "bit_identical": True} for n in J.STAGE0_RUNS]}


def _rev(run):
    r = copy.deepcopy(run)
    r["order"] = "reverse"
    return r


def _delivery(o=None, f1=None, stage0=None, o_b=None, rev=None):
    o = o or _run(stops=STOP_LABELS)
    f1 = f1 or _run(stops=STOP_LABELS, d_reference="O")
    return J.delivery(stage0 or STAGE0, STOPS, [o, o_b or copy.deepcopy(o)], rev or _rev(o),
                      [f1, copy.deepcopy(f1)], _rev(f1))


def test_delivery_same_when_identical():
    out = _delivery()
    assert out["verdict"] == "same" and out["over_count"] == 0 and out["grid_points"] == len(LABELS)
    assert out["pairs"] == len(LABELS) * len(J.JUDGED_CASTS) * len(J.ESTIMATES)


def test_delivery_delta_I_and_crossing_and_correction_error():
    f1 = _run(stops=STOP_LABELS, d_reference="O")
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
    f1b = _run(stops=STOP_LABELS, d_reference="O")
    next(p for p in f1b["points"] if p["label"] == LABELS[0])["evals"]["c/e0"] = _eval(60.0)
    out = _delivery(f1=f1b)
    assert out["verdict"] == "same"
    row = next(r for r in out["per_point_e0"] if r["label"] == LABELS[0])
    assert row["dI_c"] == 8.0 and row["dI_a"] == 0.0 and row["a_minus_c_F1"] == -8.0


def test_delivery_preconditions_make_it_unjudgeable():
    assert _delivery(stage0={**STAGE0, "all_bit_identical": False})["verdict"] == "unjudgeable"
    assert _delivery(stage0={**STAGE0, "rows": STAGE0["rows"][:5]})["verdict"] == "unjudgeable"
    o = _run(stops=STOP_LABELS)
    o_b = copy.deepcopy(o)
    o_b["points"][0]["evals"]["a/e0"]["min_information"] += 1e-12
    assert _delivery(o=o, o_b=o_b)["verdict"] == "unjudgeable"
    # the reversed run: within amendment 2's agreement it passes (not bitwise, reported); beyond it, unjudgeable
    rev = _rev(o)
    next(p for p in rev["points"] if p["label"] == J.REPRESENTATIVE[2])["state"]["lift_m"] += 1e-9
    out = _delivery(o=o, rev=rev)
    assert out["verdict"] == "same"
    assert out["determinism"]["reverse_O"]["points"][J.REPRESENTATIVE[2]]["bitwise"] is False
    rev["points"][[p["label"] for p in rev["points"]].index(J.REPRESENTATIVE[2])]["evals"]["b2/e1"]["min_information"] += 0.06
    out = _delivery(o=o, rev=rev)
    assert out["verdict"] == "unjudgeable" and out["determinism"]["reverse_O"]["differ"] == [J.REPRESENTATIVE[2]]
    # a reversed run differing away from the five representative points is not a failure
    rev2 = _rev(o)
    other = next(lab for lab in LABELS if lab not in J.REPRESENTATIVE)
    next(p for p in rev2["points"] if p["label"] == other)["state"]["lift_m"] += 0.01
    assert _delivery(o=o, rev=rev2)["verdict"] == "same"
    # the population: a grid point missing in both arms, a wrong (d) reference
    o5, f5 = _run(stops=STOP_LABELS), _run(stops=STOP_LABELS, d_reference="O")
    for run in (o5, f5):
        run["points"] = [p for p in run["points"] if p["label"] != LABELS[7]]
    assert J.delivery(STAGE0, STOPS, [o5, copy.deepcopy(o5)], _rev(o5), [f5, copy.deepcopy(f5)],
                      _rev(f5))["verdict"] == "unjudgeable"
    assert _delivery(f1=_run(stops=STOP_LABELS))["verdict"] == "unjudgeable"  # F1 cast (d) at its own height
    # the operating comparison: the O stop's lift 1.1 mm off
    o3 = _run(stops=STOP_LABELS)
    next(p for p in o3["points"] if p["label"] == "v1_O_line_2_s0")["state"]["lift_m"] += 0.0011
    assert _delivery(o=o3)["verdict"] == "unjudgeable"
    # a pose out of tolerance anywhere on the grid
    o4 = _run(stops=STOP_LABELS)
    o4["points"][1]["pose_ok"] = False
    assert _delivery(o=o4)["verdict"] == "unjudgeable"


def test_delivery_settle_rules():
    f1 = _run(stops=STOP_LABELS, d_reference="O")
    f1["points"][0]["settle"]["converged"] = False
    out = _delivery(f1=f1)
    assert out["verdict"] == "different" and out["settled_one_arm_only"] == [f1["points"][0]["label"]]
    o = _run(stops=STOP_LABELS)
    o["points"][0]["settle"]["converged"] = False
    assert _delivery(o=o, f1=f1)["verdict"] == "unjudgeable"


REGISTRY = {"versions": [{"id": "v", "sha256": "h" * 64, "cusp_legs": 2}], "tracker_configs": {"transport": {}}}
EXPECTED = J.expected_trials(REGISTRY)


def _trial(tid, e=0.02, outcome="release", reached=True, exceed=False):
    vid, leg, config, load = tid.rsplit("/", 3)
    ev = {"first_at_endpoint": {"yaw_error_rad": e} if reached else None,
          "first_exceed_after_endpoint": {"tick": 5} if exceed else None}
    return {"id": tid, "version": vid, "leg": int(leg[3:]), "config": config, "load": load,
            "path_sha256": EXPECTED[tid] or "s" * 64,
            "e_stop_rad": e if outcome != "timeout" else None, "stop": None if outcome == "timeout" else {"t_s": 3.0},
            "outcome": outcome, "events": ev, "trajectory_sha256": f"h{tid}{e}", "ticks": 400,
            "wheel_load_ok": True, "settle": {"converged": True}, "foreign_contacts": 0, "start_offset_m": 0.001}


IDS = sorted(EXPECTED)


def _base():
    return {i: _trial(i) for i in IDS}


def _cusp_run(trials, record=True):
    return {"trials": list(trials.values()), "record": record, "registry_sha256": "r",
            "tracker_configs": REGISTRY["tracker_configs"], "dt_s": 1 / 120, "decel_mps2": 1.5, "accel_mps2": 0.8,
            "lift_command_m": 0.2, "pallet_mass_kg": 4.93, "scene_start_rear": [0, 0, 0]}


def _crev(trials):
    r = _cusp_run(copy.deepcopy(trials))
    r["order"] = "reverse"
    return r


def _cusp(o_trials, f1_trials, o_off=None, f1_off=None, registry=REGISTRY, o_rev=None, f1_rev=None):
    o, f1 = _cusp_run(o_trials), _cusp_run(f1_trials)
    return J.cusp([o, copy.deepcopy(o)], o_off or _cusp_run(o_trials, record=False), [f1, copy.deepcopy(f1)],
                  f1_off or _cusp_run(f1_trials, record=False), registry=registry, registry_sha256="r",
                  o_reverse=o_rev or _crev(o_trials), f1_reverse=f1_rev or _crev(f1_trials))


def test_cusp_population_is_the_registry():
    assert len(IDS) == (2 + 4) * 2 * 2
    assert _cusp(_base(), _base())["verdict"] == "same"
    short = _base()
    short.pop(IDS[0])
    assert _cusp(short, copy.deepcopy(short))["verdict"] == "unjudgeable"  # missing in both arms
    wrong = _base()
    i = next(i for i in IDS if i.startswith("v/"))
    wrong[i]["path_sha256"] = "x" * 64
    assert _cusp(_base(), wrong)["verdict"] == "unjudgeable"
    syn = _base()
    for i in IDS:
        if i.startswith("synthetic_k0.20_left"):
            syn[i]["path_sha256"] = "y" * 64
    assert _cusp(_base(), syn)["verdict"] == "unjudgeable"  # the synthetic path is not shared
    assert J.cusp([_cusp_run(_base())] * 2, _cusp_run(_base(), record=False), [_cusp_run(_base())] * 2,
                  _cusp_run(_base(), record=False), o_reverse=_crev(_base()),
                  f1_reverse=_crev(_base()))["verdict"] == "unjudgeable"  # no registry


def test_cusp_same_and_margins():
    base = _base()
    f1 = copy.deepcopy(base)
    a, b, c, d = IDS[:4]
    f1[a] = _trial(a, e=0.0251)  # |e| difference 0.0051 > 0.005
    f1[b] = _trial(b, e=-0.024)  # 0.004: inside (absolute values compared)
    f1[c] = _trial(c, outcome="failed_endpoint_heading")
    f1[d] = _trial(d, exceed=True)
    out = _cusp(base, f1)
    assert out["verdict"] == "different" and out["over_count"] == 3
    why = {r["id"]: r["why"] for r in out["over"]}
    assert why[a] == ["delta_e_stop"] and why[c] == ["outcome"] and why[d] == ["exceed_crossing_F1_only"]


def test_cusp_one_arm_and_both_arm_rules():
    base = _base()
    a, b, c = IDS[:3]
    f1 = copy.deepcopy(base)
    f1[a] = _trial(a, reached=False)
    f1[b] = _trial(b, outcome="timeout")
    out = _cusp(base, f1)
    assert out["verdict"] == "different"
    why = {r["id"]: r["why"] for r in out["over"]}
    assert "endpoint_reached_one_arm_only" in why[a] and "stop_one_arm_only" in why[b]
    both = copy.deepcopy(base)
    both[c] = _trial(c, outcome="timeout")
    assert _cusp(both, copy.deepcopy(both))["verdict"] == "unjudgeable"
    cc = copy.deepcopy(base)
    cc[a]["foreign_contacts"] = 3
    assert "foreign_contact_one_arm_only" in {r["id"]: r["why"] for r in _cusp(base, cc)["over"]}[a]
    assert _cusp(cc, copy.deepcopy(cc))["verdict"] == "unjudgeable"


def test_cusp_preconditions():
    base = _base()
    off = _cusp_run(copy.deepcopy(base), record=False)
    off["trials"][1]["trajectory_sha256"] = "x"
    assert _cusp(base, copy.deepcopy(base), o_off=off)["verdict"] == "unjudgeable"
    assert _cusp(base, copy.deepcopy(base), o_off=_cusp_run(base, record=True))["verdict"] == "unjudgeable"
    bad = copy.deepcopy(base)
    bad[IDS[0]]["wheel_load_ok"] = False
    assert _cusp(base, bad)["verdict"] == "unjudgeable"
    o, f1 = _cusp_run(base), _cusp_run(copy.deepcopy(base))
    o_b = copy.deepcopy(o)
    o_b["trials"][0]["ticks"] += 1
    assert J.cusp([o, o_b], _cusp_run(base, record=False), [f1, copy.deepcopy(f1)], _cusp_run(base, record=False),
                  registry=REGISTRY, registry_sha256="r", o_reverse=_crev(base),
                  f1_reverse=_crev(base))["verdict"] == "unjudgeable"


def test_delivery_reversed_pairs_keep_the_verdict():
    # forward dI 0.49 (inside); reversed each arm moves 0.04 (inside the reversed-run agreement) but dI 0.57: the
    # verdict flips at a representative point -> unjudgeable
    lab = J.REPRESENTATIVE[0]
    o, f1 = _run(stops=STOP_LABELS), _run(stops=STOP_LABELS, d_reference="O")
    next(p for p in f1["points"] if p["label"] == lab)["evals"]["a/e0"]["min_information"] = 52.49
    o_rev, f1_rev = _rev(o), _rev(f1)
    next(p for p in o_rev["points"] if p["label"] == lab)["evals"]["a/e0"]["min_information"] = 51.96
    next(p for p in f1_rev["points"] if p["label"] == lab)["evals"]["a/e0"]["min_information"] = 52.53
    out = J.delivery(STAGE0, STOPS, [o, copy.deepcopy(o)], o_rev, [f1, copy.deepcopy(f1)], f1_rev)
    assert out["verdict"] == "unjudgeable" and out["determinism"]["reverse_pair_verdicts_differ"][0]["key"] == "a/e0"


def test_delivery_d_height_bound_to_the_O_run():
    f1 = _run(stops=STOP_LABELS, d_reference="O")
    f1["points"][3]["d_origin_z_m"] = 0.25  # not the O run's laser origin height
    assert _delivery(f1=f1)["verdict"] == "unjudgeable"
    o = _run(stops=STOP_LABELS)
    o["points"][3]["d_origin_z_m"] = 0.25  # the O run's (d) is its (a)
    assert _delivery(o=o)["verdict"] == "unjudgeable"


def test_cusp_reversed_run_tenth_of_the_margin():
    base = _base()
    a = IDS[0]
    rev = copy.deepcopy(base)
    rev[a] = _trial(a, e=0.0204)  # 0.0004 from the forward 0.02: inside, not bitwise
    out = _cusp(base, copy.deepcopy(base), o_rev=_crev(rev))
    assert out["verdict"] == "same" and out["determinism"]["reverse_O"]["max_abs_e_stop_diff"] > 0.0003
    rev[a] = _trial(a, e=0.0206)  # 0.0006: beyond a tenth of the margin
    assert _cusp(base, copy.deepcopy(base), o_rev=_crev(rev))["verdict"] == "unjudgeable"
    # each arm moves 0.0004 the other way: inside per arm, but the pair's |e| difference crosses 0.005
    f1 = copy.deepcopy(base)
    f1[a] = _trial(a, e=0.0247)
    o_rev, f1_rev = copy.deepcopy(base), copy.deepcopy(f1)
    o_rev[a] = _trial(a, e=0.0196)
    f1_rev[a] = _trial(a, e=0.0251)
    out = _cusp(base, f1, o_rev=_crev(o_rev), f1_rev=_crev(f1_rev))
    assert out["verdict"] == "unjudgeable" and out["determinism"]["reverse_pair_verdicts_differ"] == [a]
    missing = _crev(base)
    missing["trials"] = missing["trials"][1:]
    assert _cusp(base, copy.deepcopy(base), o_rev=missing)["verdict"] == "unjudgeable"


def test_cusp_reversed_run_validity():
    base = _base()
    a = IDS[0]
    for field, value in (("foreign_contacts", 1), ("wheel_load_ok", False)):
        rev = copy.deepcopy(base)
        rev[a][field] = value
        assert _cusp(base, copy.deepcopy(base), o_rev=_crev(rev))["verdict"] == "unjudgeable", field
    o_rev, f1_rev = copy.deepcopy(base), copy.deepcopy(base)
    o_rev[a]["settle"] = {"converged": False}
    f1_rev[a]["settle"] = {"converged": False}
    assert _cusp(base, copy.deepcopy(base), o_rev=_crev(o_rev), f1_rev=_crev(f1_rev))["verdict"] == "unjudgeable"


def test_stage0_matches_relative_and_absolute_paths():
    rel = {"all_bit_identical": True,
           "rows": [{"match": f"{n}/run/delivery_match_1.npz", "bit_identical": True} for n in J.STAGE0_RUNS]}
    assert _delivery(stage0=rel)["verdict"] == "same"
    assert _delivery(stage0=STAGE0)["verdict"] == "same"
