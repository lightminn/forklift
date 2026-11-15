"""m_entry.analyze, M3 values on synthetic trials (docs/design/2026-11-15-m-analyze-entry.md 3.7, 3.8 and 6, PR 3a).

The merge's judging point and moving points, the moving-point IMU heading (A) and the q-step delay. The bias-slope
check over a block of trials (3.3 7) is PR 3b: until then every moving-point IMU heading carries
``bias_slope_unconfirmed``, and the tests here pin that it is the only reason left when everything else passes."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from forklift_core.plant import m_entry as me
from forklift_core.plant import m_synth as ms
from forklift_core.plant.real_vehicle import load_real_vehicle

REPO = Path(__file__).resolve().parents[3]
CFG = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
UP = (0.0, -1.0, 0.0)
SIGMA = {"pose_xy_m": 5e-4, "pose_yaw_rad": 3e-4, "source": "synthetic"}
SHORT = 3.0
L09 = "m3_merge_o20_l09_left"
L21 = "m3_merge_o20_l21_right"


def profile(name):
    return yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / f"{name}.yaml").read_text(encoding="utf-8"))


def make(name, **kw):
    kw.setdefault("still_s", SHORT)
    out = ms.synth_trial(profile(name), CFG, **kw)
    out["truth"][0]["sigma"] = dict(SIGMA)
    return out


def run(out, name, *, truth=True):
    return me.analyze(out["record"], out["imu"], out["truth"] if truth else None, None, profile(name), CFG, UP,
                      allow_synthetic=True, no_truth=not truth)


def trial1(res):
    (t,) = res["trials"]
    assert t["status"] == "valid", t
    return t


# --- merge: the judging point (3.7) --------------------------------------------------------------------------------

@pytest.mark.parametrize("name", [L09, "m3_merge_o40_l09_right", "m3_merge_o05_l21_left"])
def test_m3_merge_a_substitute_judges_the_cont_end_stop(name):
    yaw_per_m, scale = 0.004, 1.01
    t = trial1(run(make(name, true_yaw_per_m=yaw_per_m, true_radius_scale=scale, seed=3), name))
    v = t["values"]
    assert t["truth_rule"] == dict(method="synthetic", rule="A_substitute_cont_end_stop",
                                   substitute_pending_lead=True)
    s_true = t["distance"]["s_true_m"]
    seg = {s["label"]: s for s in profile(name)["segments"]}
    planned = seg["merge"]["distance_m"] + seg["cont"]["distance_m"]
    assert s_true == pytest.approx(planned / scale, abs=0.01)
    e = v["end"]
    # the odometry never sees the true extra yaw: e_theta = odo - truth = -yaw_per_m x s (to the stop)
    assert e["e_theta_rad"]["judgeable"] and e["e_theta_rad"]["value"] == pytest.approx(-yaw_per_m * s_true, abs=2e-4)
    assert e["e_y_m"]["judgeable"] and e["e_x_m"]["report_only"]
    assert e["role"] == "judging_point"
    assert t["truth_budget"]["ok"]
    assert "run_042" not in v and "merge_end" not in v  # A has no moving-point truth


def test_m3_merge_lateral_error_sign_is_odo_minus_truth():
    # an extra true left yaw puts the truck left of where the odometry thinks it is: e_y < 0
    t = trial1(run(make(L09, true_yaw_per_m=0.01, seed=4), L09))
    e = t["values"]["end"]
    assert e["e_y_m"]["value"] < -1e-3
    assert e["e_x_m"]["value"] == pytest.approx(0.0, abs=2e-3)


def test_m3_merge_truth_over_budget_and_no_truth():
    out = make(L09, seed=5)
    out["truth"][0]["sigma"] = {"pose_xy_m": 5e-3, "pose_yaw_rad": 3e-3, "source": "synthetic"}
    e = trial1(run(out, L09))["values"]["end"]
    assert not e["e_theta_rad"]["judgeable"] and e["e_theta_rad"]["why"] == ["truth_over_budget"]
    t = trial1(run(make(L09, seed=5), L09, truth=False))
    assert t["values"]["end"]["e_theta_rad"]["why"] == ["no_truth"] and t["truth_rule"] is None


def test_m3_merge_b_reads_run_042_report_only_until_defined():
    t = trial1(run(make(L09, truth_method="B_camera", true_yaw_per_m=0.004, seed=6), L09))
    v = t["values"]
    assert t["truth_rule"] == dict(method="B_camera", rule="B_run_042_moving_point", substitute_pending_lead=False)
    r = v["run_042"]
    assert r["role"] == "judging_point"
    assert not r["e_theta_rad"]["judgeable"] and r["e_theta_rad"]["why"] == ["run_042_definition_pending"]
    assert r["e_theta_rad"]["value"] is not None and r["odo_dt_ns"] <= me.MAX_GAP_S * 1e9
    # merge 0.9 m + 0.42 m with the extra yaw only: e_theta about -yaw_per_m x 1.32 at the moving point
    assert r["e_theta_rad"]["value"] == pytest.approx(-0.004 * 1.32, abs=3e-4)
    for pid in ("merge_end", "end"):
        assert v[pid]["role"] == "report" and v[pid]["e_theta_rad"]["report_only"]
    assert v["merge_end"]["e_theta_rad"]["value"] == pytest.approx(-0.004 * 0.9, abs=3e-4)


def test_m3_merge_moving_point_outside_the_states_is_an_interp_gap():
    out = make(L09, truth_method="B_camera", seed=7)
    pt = out["truth"][1]["points"]["run_042"]
    last = max(r["read_ns"] for r in out["record"] if r.get("t") == "state")
    pt["host_ns"] = last + 10**9
    r = trial1(run(out, L09))["values"]["run_042"]
    assert r["e_theta_rad"]["value"] is None and "odo_interp_gap" in r["e_theta_rad"]["why"]


# --- merge: the moving-point IMU heading (A; 3.7, registration 6.3) ------------------------------------------------

def test_m3_k_limit_formula_reproduces_the_registered_values():
    assert me.k_limit(9.125) == pytest.approx(me.REGISTERED["k_limit_merge_end_l09"], abs=0.01)
    assert me.k_limit(13.325) == pytest.approx(me.REGISTERED["k_limit_run_042_l09"], abs=0.01)
    assert me.k_limit(21.125) < 1.0  # the 2.1 m merge end: not judgeable at the desk N (1.23 mrad all in)


def test_m3_imu_heading_at_moving_points_only_waits_on_the_bias_slope_check():
    out = make(L09, still_s=None, arw=0.7 * ms.ARW_DESK, seed=8)
    t = trial1(run(out, L09))
    for pid in ("merge_end", "run_042"):
        h = t["values"]["imu_heading"][pid]
        assert h["report_only"] and h["why"] == ["bias_slope_unconfirmed"], (pid, h)
        assert h["k_limit"] == pytest.approx(me.k_limit(h["drive_s"]))
        assert h["n_body_over_desk"]["before"] < h["k_limit"] and h["n_body_over_desk"]["after"] < h["k_limit"]
        assert h["sigma_lower"]["body"]["lower_bound"] <= me.REGISTERED["budget_dyaw_rad"]
        # odometry minus IMU heading at the point: no true error, so within the noise
        assert h["value"] == pytest.approx(0.0, abs=4 * h["sigma_lower"]["body"]["lower_bound"] + 2e-4)
    assert t["values"]["imu_heading"]["run_042"]["drive_s"] == pytest.approx(13.325, abs=0.15)


def test_m3_imu_heading_over_the_k_limit_on_the_long_merge():
    t = trial1(run(make(L21, still_s=None, seed=9), L21))
    h = t["values"]["imu_heading"]["merge_end"]
    assert h["k_limit"] < 1.0 and "imu_noise_over_limit" in h["why"]


def test_m3_imu_heading_without_bias_windows():
    h = trial1(run(make(L09, seed=10), L09))["values"]["imu_heading"]["merge_end"]
    assert h["value"] is None and "no_bias_window" in h["why"]


def test_m3_imu_heading_with_net_rotation_needs_the_gyro_scale():
    t = trial1(run(make(L09, still_s=None, arw=0.7 * ms.ARW_DESK, true_yaw_per_m=0.06, seed=11), L09))
    assert "no_gyro_scale" in t["values"]["imu_heading"]["merge_end"]["why"]


# --- q step (3.8) --------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["m3_qstep_left", "m3_qstep_right"])
@pytest.mark.parametrize("delay", [0.0, 0.083])
def test_m3_qstep_delay(name, delay):
    # the measured q slews at 0.5 rad/s (0.26 s for 0.1312 rad): its own t63 is the share the difference cancels
    t = trial1(run(make(name, delay_s=delay, q_slew_rad_s=0.5, seed=12), name, truth=False))
    d = t["values"]["encoder_delay_s"]
    assert d["judgeable"] and d["value"] == pytest.approx(delay, abs=0.005)
    assert d["y"]["delay_s"] - d["q"]["delay_s"] == pytest.approx(d["value"])
    assert d["q"]["delay_s"] > 0.1
    assert d["post_window_ns"][0] > d["q"]["onset_ns"]
    assert t["values"]["direction"] == (1 if name.endswith("left") else -1)


def test_m3_imu_delay_carries_the_clock_offset():
    name = "m3_qstep_left"
    t = trial1(run(make(name, delay_s=0.083, mcu_imu_offset_s=0.015, seed=13), name, truth=False))
    imu = t["values"]["imu_delay_s"]
    assert imu["report_only"] and imu["unknown_clock_offset"] is True
    assert imu["value"] == pytest.approx(0.083 + 0.015, abs=0.006)
    assert t["values"]["encoder_delay_s"]["value"] == pytest.approx(0.083, abs=0.005)


def test_m3_qstep_delay_void_keeps_the_trial():
    name = "m3_qstep_right"
    t = trial1(run(make(name, delay_s=1.6, seed=14), name, truth=False))
    d = t["values"]["encoder_delay_s"]
    assert d["value"] is None and not d["judgeable"] and len(d["why"]) == 1 and d["why"][0] != "ok"
    assert t["status"] == "valid"


def test_m3_qstep_needs_no_truth():
    name = "m3_qstep_left"
    t = trial1(run(make(name, seed=15), name))
    assert t["values"]["encoder_delay_s"]["judgeable"]
    assert math.isfinite(t["values"]["encoder_delay_s"]["value"])
