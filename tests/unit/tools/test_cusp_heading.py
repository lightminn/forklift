"""Baseline priority 4 aisle_rack cusp heading judgements
(docs/validation/2026-11-15-aisle-cusp-heading/cusp_heading.py) on synthetic records (plan 2026-11-15 5)."""

import copy
import gzip
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "cusp_heading", ROOT / "docs/validation/2026-11-15-aisle-cusp-heading/cusp_heading.py")
ch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ch)
td = ch.td

DT = 0.1


def zigzag(legs, step=0.1, y=0.0):
    """Back and forth along x (yaw 0): `legs` segment counts, a cusp at every turn."""
    poses, dirs, x, d = [[0.0, y, 0.0]], [1], 0.0, 1
    for n in legs:
        for _ in range(n):
            x = round(x + d * step, 10)
            poses.append([x, y, 0.0])
            dirs.append(d)
        d = -d
    return {"poses": poses, "directions": dirs, "curvatures_inv_m": [0.0] * len(poses)}


def cusps_of(path, keep=None):
    P = td.Path2(path)
    cs = [{"index": int(i), "pose": [float(v) for v in P.poses[i]]} for i in P.cusps]
    return cs if keep is None else cs[:keep]


class Rec:
    """A synthetic run: result.json fields and slam_control rows, built leg by leg."""

    def __init__(self, t0=10.0):
        self.t = t0
        self.rows, self.samples = [], []
        self.r = {"transitions": [{"from": "observe", "to": "approach", "time_s": 2.0},
                                  {"from": "approach", "to": "extract", "time_s": 5.0},
                                  {"from": "extract", "to": "transport", "time_s": t0}],
                  "leg_switches": [], "tracker_versions": [], "cusp_replans": [], "paths": {}, "paths_replaced": [],
                  "cusp_stalls": [], "samples": self.samples, "success": True}
        self.pose = None

    def add(self, x, y, yaw_est, yaw_true, seg, speed, true_dy=0.0):
        self.rows.append([self.t, x, y, yaw_est, x, y + true_dy, yaw_true])
        self.samples.append({"time_s": self.t, "phase": "transport", "signed_speed_mps": speed,
                             "speed_command_mps": speed, "curvature_inv_m": 0.0,
                             "steering_command_rad": [0.01, 0.01], "steering_actual_rad": [0.0, 0.0],
                             "tracking": {"segment_index": seg, "yaw_error_rad": 0.0}})
        self.pose = [x, y, yaw_est]

    def start(self, path):
        p = path["poses"][0]
        self.add(p[0], p[1], 0.0, 0.0, 0, 0.0)

    def drive(self, path, leg, yaw_E, yaw_T, hold=3, hold_yaw_E=None):
        """Moving rows one pose a tick to the leg end (yaw errors yaw_E, yaw_T), then `hold` standing rows."""
        P = td.Path2(path)
        lo, hi = P.leg_range(leg)
        end_yaw = float(P.poses[hi][2])
        for i in range(lo + 1, hi + 1):
            self.t = round(self.t + DT, 9)
            self.add(float(P.poses[i][0]), float(P.poses[i][1]), end_yaw - yaw_E, end_yaw - yaw_T, i - 1,
                     float(P.directions[i]))
        he = yaw_E if hold_yaw_E is None else hold_yaw_E
        for _ in range(hold):
            self.t = round(self.t + DT, 9)
            self.add(float(P.poses[hi][0]), float(P.poses[hi][1]), end_yaw - he, end_yaw - yaw_T, hi - 1, 0.0)
        return P, hi, he

    def serial(self, serial, path, installed, keep=None, phase="transport"):
        self.r["tracker_versions"].append({"serial": serial, "phase": phase, "installed_s": installed,
                                           "replaced_s": None, "cusps": cusps_of(path, keep)})

    def replaced(self, serial, t):
        next(v for v in self.r["tracker_versions"] if v["serial"] == serial)["replaced_s"] = t

    def switch(self, serial, path, leg, yaw_e, via="arrival", phase="transport", t=None):
        P = td.Path2(path)
        end = P.ends[leg]
        self.r["leg_switches"].append({"via": via, "leg": leg, "endpoint_index": int(end),
                                       "endpoint_pose": [float(v) for v in P.poses[end]], "pose": list(self.pose),
                                       "position_error_m": 0.0, "yaw_error_rad": yaw_e, "serial": serial,
                                       "phase": phase, "time_s": self.t if t is None else t, "stamped": "tick"})

    def cusp_replan(self, old, new, yaw_e, status="success"):
        self.r["cusp_replans"].append({"time_s": self.t, "loop_step": 1, "rear_pose": list(self.pose),
                                       "position_error_m": 0.0, "yaw_error_rad": yaw_e, "status": status,
                                       "planning_wall_s": 1.0, "search_attempts": [], "path": new, "replaced_path": old})

    def end_transport(self, dt=1.0):
        self.t = round(self.t + dt, 9)
        self.r["transitions"].append({"from": "transport", "to": "lower", "time_s": self.t})
        self.r["transitions"].append({"from": "lower", "to": "return_home", "time_s": self.t + 5})

    def run(self, name="k1r_B0_aisle_rack_s0"):
        return ch.Run("root", name, copy.deepcopy(self.r), np.asarray(self.rows, dtype=float))


A = zigzag([10, 10, 10])  # legs of 1 m: cusps 10, 20, end 30
B = zigzag([5, 5, 5])  # legs of 0.5 m (Short): cusps 5, 10, end 15
C = zigzag([5, 5], y=0.0)
C["poses"] = [[p[0], 0.0, 0.0] for p in C["poses"]]
C["curvatures_inv_m"] = [0.0] * len(C["poses"])
C["curvatures_inv_m"][-1] = 1e-3  # a different digest from B's front


def base(rec=None, fail_yaw=(0.07, 0.07), outcome="recover", mismatch=False):
    """Mission path A (serial 5): leg 0 passes, leg 1 fails (cusp replan to B). B is installed as serial 6 and cut at
    once by arm_docking (serial 7 keeps B's first cusp only)."""
    rec = rec or Rec()
    rec.serial(5, A, 10.0)
    rec.start(A)
    if mismatch:
        rec.drive(A, 0, 0.07, 0.07, hold_yaw_E=0.01)
    else:
        rec.drive(A, 0, 0.01, 0.01)
    rec.switch(5, A, 0, 0.01)
    rec.drive(A, 1, fail_yaw[0], fail_yaw[1])
    rec.cusp_replan(A, B, fail_yaw[0], status="success" if outcome != "replan_failed" else "no_path")
    t_c1 = rec.t
    rec.replaced(5, t_c1)
    if outcome == "replan_failed":
        rec.end_transport()
        return rec, t_c1
    rec.serial(6, B, t_c1)
    rec.replaced(6, t_c1)
    rec.serial(7, B, t_c1, keep=1)
    if outcome == "cut":
        rec.t = round(rec.t + 0.5, 9)
        rec.add(*rec.pose[:2], rec.pose[2], 0.0, 0, 0.0)
        rec.r["paths_replaced"].append({"leg": "transport", "time_s": rec.t, "by": "scene_replan", "path": C})
        rec.replaced(7, rec.t)
        rec.serial(8, C, rec.t)
        rec.drive(C, 0, 0.0, 0.0)
        rec.switch(8, C, 0, 0.0)
        rec.end_transport()
        return rec, t_c1
    if outcome == "refail":
        rec.drive(B, 0, -0.06, -0.06)
        rec.cusp_replan(B, C, -0.06)
        rec.replaced(7, rec.t)
        rec.serial(8, C, rec.t)
        rec.drive(C, 0, 0.0, 0.0)
        rec.switch(8, C, 0, 0.0)
        rec.end_transport()
        return rec, t_c1
    rec.drive(B, 0, 0.02, 0.02)
    rec.switch(7, B, 0, 0.02)
    rec.drive(B, 1, 0.0, 0.0)
    rec.switch(7, B, 1, 0.0)  # serial 7's last leg end: not one of its cusps (excluded)
    rec.r["paths_replaced"].append({"leg": "transport", "time_s": rec.t + 0.5, "by": "delivery_straight", "path": B})
    rec.end_transport()
    return rec, t_c1


def analyse(*runs, period="postfix", tdep=None):
    return ch.analyse(list(runs), period, tdep)


def kinds(out):
    return [(j["kind"], j.get("leg"), j.get("path_digest")) for j in out["judgements"]]


# ---- passes, the last leg, exclusions ------------------------------------------------------------------------------


def test_a_pass_is_an_arrival_at_a_serial_cusp_and_the_last_leg_is_left_out():
    rec, _ = base()
    out = analyse(rec.run())
    passes = [j for j in out["judgements"] if j["kind"] == "pass"]
    assert [(j["serial"], j["leg"], j["end_index"]) for j in passes] == [(5, 0, 10), (7, 0, 5)]
    assert all(j["is_cusp"] and not j["release"] for j in passes)
    assert passes[0]["path_digest"] == td.Path2(A).digest() and passes[0]["source"] == "mission"
    assert out["counts"]["excluded"]["leg_switch"]["endpoint_index not a cusp of the serial"] == 1


def test_a_release_counts_as_a_pass_with_its_mark():
    rec, _ = base()
    rec.r["leg_switches"][0]["via"] = "release"
    out = analyse(rec.run())
    j = out["judgements"][0]
    assert j["kind"] == "pass" and j["release"] and out["counts"]["passes_release"] == 1


def test_records_outside_transport_are_left_out_with_their_reasons():
    rec, _ = base()
    rec.switch(1, A, 0, 0.0, phase="observe", t=3.0)
    rec.switch(9, A, 0, 0.0, phase="return_home", t=rec.t + 10)
    rec.switch(7, B, 0, 0.0, via="skip")
    rec.r["leg_switches"][-1]["time_s"] = 11.0
    rec.r["leg_switches"].append(dict(rec.r["leg_switches"][0], time_s=rec.t + 2, phase="transport"))  # after transport
    rec.r["cusp_replans"].append(dict(rec.r["cusp_replans"][0], time_s=rec.t + 3))
    out = analyse(rec.run())
    ex = out["counts"]["excluded"]
    assert ex["leg_switch"] == {"endpoint_index not a cusp of the serial": 1, "phase:observe": 1,
                                "phase:return_home": 1, "via:skip": 1, "outside transport window": 1}
    assert ex["cusp_replan"] == {"outside transport window": 1}
    assert out["counts"]["passes"] == 2 and out["counts"]["failures_replan"] == 1


# ---- failures: each record reconstructed on its own ---------------------------------------------------------------


def test_the_first_and_second_failures_are_each_reconstructed_on_their_active_path():
    rec, t_c1 = base(outcome="refail")
    out = analyse(rec.run())
    f1, f2 = [j for j in out["judgements"] if j["kind"] == "fail_replan"]
    assert (f1["path_digest"], f1["new_path_digest"], f1["leg"], f1["serial"]) == (
        td.Path2(A).digest(), td.Path2(B).digest(), 1, 5)
    assert (f2["path_digest"], f2["new_path_digest"], f2["leg"], f2["serial"]) == (
        td.Path2(B).digest(), td.Path2(C).digest(), 0, 7)
    assert f1["source"] == "mission" and f2["source"] == "cusp_replan" and f2["replan_ordinal"] == 1
    assert f2["leg_start_s"] == pytest.approx(t_c1) and f2["replan_leg"] and not f1["replan_leg"]
    assert f1["leg_length_m"] == 1.0 and f2["leg_length_m"] == 0.5 and f2["Short"]
    assert f1["classification"] == "tracking_unsaturated" and f2["classification"] == "plan_short_leg"
    assert f1["serial_path_consistent"] and f2["serial_path_consistent"]


def test_the_failure_judged_end_must_be_a_serial_cusp():
    rec, _ = base()
    rec.r["tracker_versions"][0]["cusps"] = rec.r["tracker_versions"][0]["cusps"][:1]  # A's cusp 20 not in serial 5
    out = analyse(rec.run())
    f = next(j for j in out["judgements"] if j["kind"] == "fail_replan")
    assert not f["is_cusp"] and f["classification"] == "unresolved"


# ---- the reconstructed judgement tick ------------------------------------------------------------------------------


def test_the_tick_is_the_first_row_within_three_centimetres():
    rec, _ = base(fail_yaw=(0.07, 0.07))
    out = analyse(rec.run())
    f = next(j for j in out["judgements"] if j["kind"] == "fail_replan")
    t_end = rec.r["cusp_replans"][0]["time_s"] - 3 * DT  # the row reaching the end, before the 3 standing rows
    assert f["tick"]["found"] and f["tick"]["t"] == pytest.approx(t_end, abs=1e-6)
    assert f["tick"]["e_E"] == pytest.approx(0.07) and f["recorded"]["e_E"] == pytest.approx(0.07)
    assert f["tick"]["pre_entry"]["t"] == pytest.approx(t_end - DT, abs=1e-6)


def test_a_pass_whose_entry_tick_is_beyond_tau_is_marked_a_reconstruction_mismatch():
    rec, _ = base(mismatch=True)
    out = analyse(rec.run())
    p = out["judgements"][0]
    assert p["kind"] == "pass" and p["recorded"]["e_E"] == pytest.approx(0.01)
    assert p["tick"]["e_E"] == pytest.approx(0.07) and p["reconstruction_mismatch"]
    assert out["counts"]["reconstruction"]["pass_mismatch"] == 1
    assert out["counts"]["reconstruction"]["pass_mismatch_rate"] == pytest.approx(0.5)


def test_tick_not_found_and_overshoot():
    P = td.Path2(A)
    rows = np.array([[0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0], [0.1, 0.9, 0.0, 0.0, 0.9, 0.0, 0.0],
                     [0.2, 1.05, 0.0, 0.01, 1.05, 0.0, 0.0], [0.3, 1.01, 0.0, 0.06, 1.01, 0.0, 0.0]])
    t = ch.judgement_tick(rows, P, 10, 0.0, 0.3)
    assert t["found"] and t["t"] == pytest.approx(0.3) and t["e_E"] == pytest.approx(-0.06)
    assert t["overshoot_possible"]  # 0.05 m past the end along its heading before entering 0.03 m
    t2 = ch.judgement_tick(rows[:3], P, 10, 0.0, 0.2)
    assert not t2["found"] and t2["e_E"] is None and t2["overshoot_possible"]
    t3 = ch.judgement_tick(rows[:2], P, 10, 0.0, 0.1)
    assert not t3["found"] and not t3["overshoot_possible"]


def test_a_failure_without_a_tick_is_unresolved():
    rec, _ = base()
    # move the failing leg's rows 5 cm off the line: no row within 0.03 m of A's cusp 20
    t_c1 = rec.r["cusp_replans"][0]["time_s"]
    t_s1 = rec.r["leg_switches"][0]["time_s"]
    for row in rec.rows:
        if t_s1 + 1e-9 < row[0] <= t_c1 + 1e-9:
            row[2] += 0.05
    out = analyse(rec.run())
    f = next(j for j in out["judgements"] if j["kind"] == "fail_replan")
    assert not f["tick"]["found"] and f["classification"] == "unresolved"
    assert "e_T unavailable" in f["classification_why"]
    assert out["counts"]["reconstruction"]["tick_not_found"]["fail"] == 1


# ---- the leg right after a path replacement ------------------------------------------------------------------------


def test_the_first_leg_of_a_replan_path_starts_at_its_install_and_is_marked():
    rec, t_c1 = base()
    out = analyse(rec.run())
    p = [j for j in out["judgements"] if j["kind"] == "pass"][1]
    assert p["serial"] == 7 and p["leg"] == 0 and p["replan_leg"] and p["first_leg_after_install"] == "cusp_replan"
    assert p["leg_start_s"] == pytest.approx(t_c1) and p["source"] == "cusp_replan" and p["replan_ordinal"] == 1
    run = next(r for r in out["runs"] if r["name"] == "k1r_B0_aisle_rack_s0")
    s7 = next(s for s in run["serials"] if s["serial"] == 7)
    assert s7["truncation_of"] == 6 and s7["path_digest"] == td.Path2(B).digest()


def test_serial_path_matching():
    cands = {td.Path2(p).digest(): {"path": td.Path2(p), "origins": []} for p in (A, B)}
    assert ch.match_serial(cusps_of(B, keep=1), cands)[0] == td.Path2(B).digest()  # an arm_docking cut: the front
    assert ch.match_serial([{"index": 5, "pose": [9.0, 9.0, 0.0]}], cands)[0] is None
    B2 = copy.deepcopy(B)
    B2["curvatures_inv_m"][2] = 0.1  # same cusp poses, another path before the cusp
    cands[td.Path2(B2).digest()] = {"path": td.Path2(B2), "origins": []}
    d, why = ch.match_serial(cusps_of(B, keep=1), cands)
    assert d is None and why.startswith("ambiguous")
    B3 = copy.deepcopy(B)
    B3["curvatures_inv_m"][14] = 0.1  # differs only after the serial's last cusp: one path for the cusp legs
    cands.pop(td.Path2(B2).digest())
    cands[td.Path2(B3).digest()] = {"path": td.Path2(B3), "origins": []}
    assert ch.match_serial(cusps_of(B, keep=1), cands)[0] is not None


def test_an_unmatched_serial_leaves_its_passes_unresolved():
    rec, _ = base()
    rec.r["tracker_versions"][0]["cusps"][0]["pose"][0] += 0.01
    rec.r["leg_switches"][0]["endpoint_pose"][0] += 0.01
    out = analyse(rec.run())
    p = out["judgements"][0]
    assert p["kind"] == "pass" and p["path_digest"] is None and "no recorded path" in p["unresolved_why"]


# ---- outcomes after a failure --------------------------------------------------------------------------------------


@pytest.mark.parametrize("variant, expected", [
    ("recover", "recovered"), ("refail", "refailed"), ("replan_failed", "replan_failed"), ("cut", "cut:scene_replan")])
def test_the_outcome_after_a_failure(variant, expected):
    rec, _ = base(outcome=variant)
    out = analyse(rec.run())
    f = next(j for j in out["judgements"] if j["kind"] == "fail_replan")
    assert f["outcome"] == expected


def test_a_replan_path_reaching_its_cut_end_counts_as_an_arrival():
    rec = Rec()
    rec.serial(5, A, 10.0)
    rec.start(A)
    rec.drive(A, 0, 0.07, 0.07)
    rec.cusp_replan(A, zigzag([5]), 0.07)
    rec.replaced(5, rec.t)
    rec.serial(6, zigzag([5]), rec.t)
    rec.drive(zigzag([5]), 0, 0.0, 0.0)
    rec.r["paths_replaced"].append({"leg": "transport", "time_s": rec.t, "by": "delivery_straight", "path": zigzag([5])})
    rec.end_transport()
    f = analyse(rec.run())["judgements"][0]
    assert f["outcome"] == "recovered" and "delivery_straight" in f["outcome_next"]


def test_a_final_failure_is_judged_on_the_failure_path():
    rec, _ = base(outcome="replan_failed")
    rec.r["transitions"] = rec.r["transitions"][:3]
    tf_t = rec.r["cusp_replans"][0]["time_s"]
    rec.r["cusp_replans"] = []
    rec.r["tracking_failure"] = {"time_s": tf_t, "path": A, "rear_pose": list(rec.pose), "phase": "transport",
                                 "tracking": {"failure": "endpoint_heading", "at_cusp": True, "yaw_error_rad": 0.07,
                                              "position_error_m": 0.0}}
    out = analyse(rec.run())
    f = next(j for j in out["judgements"] if j["kind"] == "fail_final")
    assert f["leg"] == 1 and f["outcome"] == "final_failure" and f["classification"] == "tracking_unsaturated"


def test_a_pass_of_the_same_leg_before_a_failure_is_an_anomaly():
    rec, _ = base()
    sw = dict(rec.r["leg_switches"][0])
    sw.update(leg=1, endpoint_index=20, endpoint_pose=[0.0, 0.0, 0.0], time_s=rec.r["cusp_replans"][0]["time_s"] - 0.05)
    rec.r["leg_switches"].insert(1, sw)
    out = analyse(rec.run())
    f = next(j for j in out["judgements"] if j["kind"] == "fail_replan")
    assert f["anomaly_pass_same_leg"] and out["counts"]["anomalies_pass_same_leg"] == 1


# ---- the plan 3 table ----------------------------------------------------------------------------------------------


def _j(**kw):
    j = {"is_cusp": True, "path_digest": "x", "leg_start_s": 1.0, "installs_inside_window": [], "Short": False,
         "sat_end": {"S": False, "P": None, "M": None}}
    j.update(kw)
    return j


@pytest.mark.parametrize("j, e_E, e_T, label", [
    (_j(is_cusp=False), 0.07, 0.07, "unresolved"),
    (_j(path_digest=None), 0.07, 0.07, "unresolved"),
    (_j(installs_inside_window=["scene_replan"]), 0.07, 0.07, "unresolved"),
    (_j(), 0.07, None, "unresolved"),
    (_j(), 0.05, 0.07, "unresolved"),
    (_j(), 0.07, 0.03, "localisation_decision"),
    (_j(sat_end={"S": True, "P": None, "M": True}), 0.07, 0.07, "unresolved_saturated_no_curvature"),
    (_j(sat_end={"S": True, "P": True, "M": True}), 0.07, 0.07, "mixed_plan_margin_and_plant"),
    (_j(sat_end={"S": True, "P": True, "M": True}, Short=True), 0.07, 0.07, "mixed_plan_margin_and_plant"),
    (_j(Short=True), 0.07, 0.07, "plan_short_leg"),
    (_j(), -0.07, -0.07, "tracking_unsaturated"),
    (_j(sat_end={"S": None}), 0.07, 0.07, "unresolved"),
])
def test_the_table_first_matching_row(j, e_E, e_T, label):
    assert ch.classify(j, e_E, e_T)[0] == label


def test_the_classification_uses_the_tick_values_and_keeps_the_recorded_ones():
    rec, _ = base(fail_yaw=(0.07, 0.03))
    out = analyse(rec.run())
    f = next(j for j in out["judgements"] if j["kind"] == "fail_replan")
    assert f["D"] and f["classification"] == "localisation_decision"
    # a stop that settles to |e_T| > tau: the recorded values say "not D" while the tick says D
    for row in rec.rows:
        if row[0] > f["tick"]["t"] + 1e-6 and row[0] <= rec.r["cusp_replans"][0]["time_s"] + 1e-9:
            row[6] = -0.06
    f = next(j for j in analyse(rec.run())["judgements"] if j["kind"] == "fail_replan")
    assert f["classification"] == "localisation_decision" and f["classification_recorded"] == "tracking_unsaturated"


# ---- identical arms ------------------------------------------------------------------------------------------------


def test_identical_arms_are_one_case():
    rec, _ = base()
    a, b = rec.run("k1r_B0_aisle_rack_s0"), rec.run("k1r_K1_aisle_rack_s0")
    b.result["cusp_replans"][0]["planning_wall_s"] = 9.0  # wall clock only
    out = analyse(a, b)
    assert out["n_cases"] == 1 and out["cases"][0]["runs"] == ["k1r_B0_aisle_rack_s0", "k1r_K1_aisle_rack_s0"]
    assert out["cases"][0]["ignored_fields_differ"] and out["counts"]["judgements"] == 3
    assert all(j["case_runs"] == out["cases"][0]["runs"] for j in out["judgements"])
    b.result["samples"][3]["signed_speed_mps"] = 0.9
    out = analyse(a, b)
    assert out["n_cases"] == 2 and out["counts"]["judgements"] == 6


def test_different_slam_rows_split_the_arms():
    rec, _ = base()
    a, b = rec.run("k1r_B0_aisle_rack_s0"), rec.run("k1r_K1_aisle_rack_s0")
    b.sc = b.sc.copy()
    b.sc[5, 6] += 1e-4
    assert analyse(a, b)["n_cases"] == 2


# ---- before the fix ------------------------------------------------------------------------------------------------


def test_prefix_period_without_tracker_versions(tmp_path):
    rec, _ = base()
    rec.r.pop("tracker_versions")
    run = rec.run("lbg_B0_aisle_rack_s1")
    first = rec.r["cusp_replans"][0]
    tdep = tmp_path / "tdep.json"
    tdep.write_text(json.dumps({"cases": [{"kind": "endpoint_heading", "runs": ["lbg_B0_aisle_rack_s1"],
                                           "primary": "tracking_unsaturated", "leg": 1, "leg_length_m": 1.0,
                                           "e_E": 0.07, "e_T": 0.07, "Short": False,
                                           "path_sha": td.Path2(A).digest(),
                                           "sat_end": {"share": 0.0, "P": None, "M": None}}]}))
    out = analyse(run, period="prefix", tdep=tdep)
    js = out["judgements"]
    # without tracker_versions the arm_docking cut is invisible: B's cusp 10 (serial 7's last end) reads as a cusp
    assert [(j["kind"], j.get("end_index")) for j in js] == [("pass", 10), ("fail_replan", 20), ("pass", 5), ("pass", 10)]
    assert all(j["serial_link"].startswith("unresolved") for j in js)
    assert js[0]["path_match"] == "endpoint pose" and js[2]["path_digest"] == td.Path2(B).digest()
    assert js[1]["outcome"] is None and js[1]["outcome_time_order"] == "recovered"
    assert js[1]["recorded"]["e_E"] == pytest.approx(first["yaw_error_rad"])
    chk = out["tdep_check"][0]
    assert chk["agrees_with_p3_recorded"] and chk["values_agree_with_p3"] and not chk["tick_changes_classification"]
    assert "aggregate" not in out


# ---- inputs: a raw folder and a fields file give the same output ---------------------------------------------------


def test_raw_folder_and_fields_file_agree(tmp_path):
    rec, _ = base()
    raw = tmp_path / "k1r"
    for name in ("k1r_B0_aisle_rack_s0", "k1r_K1_aisle_rack_s0", "k1r_B0_other_s0"):
        d = raw / name / "run"
        d.mkdir(parents=True)
        (d / "result.json").write_text(json.dumps(rec.r))
        np.save(d / "slam_control.npy", np.asarray(rec.rows, dtype=float))
    sc = np.round(np.asarray(rec.rows, dtype=float), 6)
    fields = {"k1r_B0_aisle_rack_s0": {"result": rec.r, "slam_control_rows": sc.tolist(),
                                       "slam_control_sha256": ch.rows_sha(sc)},
              "k1r_K1_aisle_rack_s0": {"result": rec.r, "slam_control_rows_same_as": "k1r_B0_aisle_rack_s0",
                                       "slam_control_sha256": ch.rows_sha(sc)}}
    fz = tmp_path / "f.json.gz"
    with gzip.open(fz, "wt") as f:
        json.dump(fields, f)
    o1, o2 = tmp_path / "o1.json", tmp_path / "o2.json"
    ch.main(["--raw-root", str(raw), "--out", str(o1)])
    ch.main(["--fields", str(fz), "--out", str(o2)])
    a, b = json.loads(o1.read_text()), json.loads(o2.read_text())
    assert a["n_runs"] == 2 and a["n_cases"] == 1  # the name filter drops the non-aisle_rack run
    assert a["judgements"] == b["judgements"] and a["counts"] == b["counts"] and a["aggregate"] == b["aggregate"]
    assert all(r["slam_control_sha_ok"] for r in b["runs"])


def test_the_aggregate_ranges_and_plan4_counts():
    rec, _ = base(outcome="refail")
    out = analyse(rec.run())
    ag = out["aggregate"]
    rl = ag["ranges_pass_vs_fail"]["leg_length_m"]
    assert rl["fail"]["min"] == 0.5 and rl["fail"]["max"] == 1.0 and not rl["separated"]
    p4 = ag["plan4"]
    assert p4["e_E_star_in_band"] == {"n": 2, "in_band": 2, "share": 1.0}
    assert p4["non_short_failures"]["n"] == 1 and p4["non_short_failures"]["of"] == 2
    corner = ag["corners"][0]
    assert corner["fail"] >= 1 and corner["runs"] == ["k1r_B0_aisle_rack_s0"]


# ---- rules the plan did not fix (Codex tool review 1) --------------------------------------------------------------


def _final_failure(phase="transport", after_end=False):
    rec, _ = base(outcome="replan_failed")
    rec.r["transitions"] = rec.r["transitions"][:3]
    tf_t = rec.r["cusp_replans"][0]["time_s"]
    rec.r["cusp_replans"] = []
    if after_end:
        rec.r["transitions"].append({"from": "transport", "to": "lower", "time_s": tf_t - 0.5})
    tf = {"time_s": tf_t, "path": A, "rear_pose": list(rec.pose),
          "tracking": {"failure": "endpoint_heading", "at_cusp": True, "yaw_error_rad": 0.07, "position_error_m": 0.0}}
    if phase is not None:
        tf["phase"] = phase
    rec.r["tracking_failure"] = tf
    return analyse(rec.run())


def test_a_final_failure_counts_inside_the_transport_window_only():
    assert _final_failure()["counts"]["failures_final"] == 1
    assert _final_failure(phase=None)["counts"]["failures_final"] == 1  # no phase: judged by the window alone
    out = _final_failure(phase="return_home")
    assert out["counts"]["failures_final"] == 0 and out["counts"]["excluded"]["tracking_failure"] == {"phase return_home": 1}
    out = _final_failure(after_end=True)
    assert out["counts"]["failures_final"] == 0
    assert out["counts"]["excluded"]["tracking_failure"] == {"outside transport window": 1}


def test_an_object_id_differing_between_identical_arms_is_ignored_and_listed():
    rec, _ = base()
    rec.r["scene_replans"] = [{"time_s": 11.0, "phase": "transport", "tracker_serial": 1}]
    a, b = rec.run("k1r_B0_aisle_rack_s0"), rec.run("k1r_K1_aisle_rack_s0")
    b.result["scene_replans"][0]["tracker_serial"] = 2
    out = analyse(a, b)
    assert out["n_cases"] == 1 and out["cases"][0]["ignored_fields_differ"]
    b.result["scene_replans"][0]["time_s"] = 11.5  # a trajectory value: the arms split
    assert analyse(a, b)["n_cases"] == 2


def test_a_serial_installed_at_the_transport_start_is_the_mission_path():
    rec, _ = base()
    assert analyse(rec.run())["judgements"][0]["source"] == "mission"  # serial 5 installed at the transition
    rec.r["tracker_versions"][0]["installed_s"] = 10.0 + 2 * ch.MISSION_SLACK_S
    assert analyse(rec.run())["judgements"][0]["source"] == "other"


def test_an_unknown_s_end_leaves_the_failure_unresolved():
    j = {"is_cusp": True, "path_digest": "d", "leg_start_s": 1.0, "installs_inside_window": [], "Short": True,
         "sat_end": {"S": None}}
    assert ch.classify(j, 0.07, 0.07) == ("unresolved", ["S_end unknown"])
    j["sat_end"] = {"S": False}
    assert ch.classify(j, 0.07, 0.07)[0] == "plan_short_leg"


def test_corner_grouping_is_order_free_and_bounded():
    def j(run, t, pose, kind="pass"):
        return {"run": run, "time_s": t, "end_pose": pose, "kind": kind}
    js = [j("r1", 1.0, [0.0, 0.0, 0.0]), j("r2", 2.0, [0.05, 0.05, 0.05], "fail_replan"), j("r1", 3.0, [0.5, 0.0, 0.0])]
    a, b = ch.corners(js), ch.corners(list(reversed(js)))
    assert a == b  # the input order does not change the groups
    assert [(g["pass"], g["fail"]) for g in a] == [(1, 1), (1, 0)]
    far = ch.corners([j("r1", 1.0, [0.0, 0.0, 0.0]), j("r1", 2.0, [0.0, 0.0, ch.CORNER_YAW_RAD + 0.01])])
    assert len(far) == 2  # outside the yaw bound: another corner
