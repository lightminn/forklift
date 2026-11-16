"""m_entry.analyze, M2 values on synthetic trials (docs/design/2026-11-15-m-analyze-entry.md 3.6 and 6, PR 2)."""

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


@pytest.mark.parametrize("name,q", [("m2_q008_left_fwd", 0.08), ("m2_q008_left_rev", 0.08),
                                    ("m2_q008_right_fwd", -0.08), ("m2_q008_right_rev", -0.08)])
def test_m2_gains_fwd_rev(name, q):
    gain, off = 0.9, 0.005
    t = trial1(run(make(name, gain=gain, q_offset=off, seed=2), name))
    v = t["values"]
    assert v["gain_measured_q"]["judgeable"] and v["gain_measured_q"]["value"] == pytest.approx(gain, abs=2e-3)
    k_meas = CFG.steering.curvature(q - off)
    k_cmd = CFG.steering.curvature(q)
    assert v["gain_commanded_q"]["value"] == pytest.approx(gain * k_meas / k_cmd, abs=2e-3)
    assert v["kappa_true_inv_m"]["value"] == pytest.approx(gain * k_meas, abs=3e-4)
    assert v["direction"] == (1 if name.endswith("fwd") else -1)
    assert t["distance"]["source"] == "truth"
    assert t["distance"]["s_true_m"] == pytest.approx(1.5, abs=0.02)
    # the report values
    assert v["drive_minus_meas_q_median_rad"]["value"] == pytest.approx(off, abs=1e-6)
    assert v["meas_q_spread_rad"]["value"] == pytest.approx(0.0, abs=1e-6)
    assert v["presteer_approach"]["value"]["sign"] == (1 if q > 0 else -1)
    assert v["presteer_approach"]["value"]["from_zero"] is True
    assert "under_reached" not in v  # not an at_max profile
    assert t["imu"]["report_only"] is True


def test_m2_gain_one_default():
    name = "m2_q008_left_fwd"
    t = trial1(run(make(name), name))
    assert t["values"]["gain_measured_q"]["value"] == pytest.approx(1.0, abs=2e-3)
    assert t["values"]["gain_commanded_q"]["value"] == pytest.approx(1.0, abs=2e-3)


def test_m2_under_reached_arc_only():
    name = "m2_max_left_fwd"
    ok = trial1(run(make(name), name))["values"]["under_reached"]
    assert ok["judgeable"] and ok["value"] is False and ok["none_fraction"] == 0.0
    # q stops 0.03 short in the arc only: the presteer (and the runner's own check) still passed
    t = trial1(run(make(name, q_short_drive=0.03), name))
    ur = t["values"]["under_reached"]
    assert ur["judgeable"] and ur["value"] is True and ur["n_rows"] > 0
    # the gain is still given (an under_reached trial is not dropped, registration 4.5)
    assert t["values"]["gain_measured_q"]["judgeable"]


def test_under_reached_helper_none_fraction():
    k = 0.36
    q = CFG.steering.angle(k)
    rows = [{"host_ns": 10, "req_kappa": k, "meas_q": None}, {"host_ns": 20, "req_kappa": k, "meas_q": q - 0.03},
            {"host_ns": 30, "req_kappa": k, "meas_q": q - 0.03}, {"host_ns": 40, "req_kappa": k, "meas_q": None},
            {"host_ns": 5, "req_kappa": k, "meas_q": q},  # before the movement: ignored
            {"host_ns": 50, "req_kappa": 0.0, "meas_q": 0.0}]  # no curvature request: ignored
    r = me.under_reached(rows, CFG, 10)
    assert r["value"] is True and r["none_fraction"] == pytest.approx(0.5) and r["n_rows"] == 4
    rows[1]["meas_q"] = q - 0.01
    assert me.under_reached(rows, CFG, 10)["value"] is False
    gone = me.under_reached([{**x, "meas_q": None} for x in rows], CFG, 10)
    assert gone["judgeable"] is False and gone["why"] == ["meas_q_missing"] and gone["none_fraction"] == 1.0


def test_arc_length_sinc_guard():
    assert me.arc_length(1.0, 0.0) == 1.0
    assert me.arc_length(1.0, 1e-5) == pytest.approx(1.0, abs=1e-10)
    th = 0.5
    assert me.arc_length(2 * math.sin(th / 2), th) == pytest.approx(th, abs=1e-12)  # radius 1
    assert me.arc_length(1.0, -0.3) == me.arc_length(1.0, 0.3)


def test_m2_small_rotation():
    name = "m2_q003_left_fwd"
    t = trial1(run(make(name, gain=0.0), name))
    for key in ("gain_measured_q", "gain_commanded_q"):
        assert t["values"][key]["judgeable"] is False and "rotation_too_small" in t["values"][key]["why"]
    assert t["values"]["meas_q_spread_rad"]["judgeable"]


def test_m2_no_truth():
    name = "m2_q008_left_fwd"
    t = trial1(run(make(name, gain=0.9), name, truth=False))
    assert t["distance"]["source"] == "encoder" and "truth_budget" not in t
    for key in ("gain_measured_q", "gain_commanded_q"):
        assert t["values"][key] == dict(value=None, judgeable=False, why=["no_truth"])
    assert t["values"]["kappa_true_inv_m"]["why"] == ["no_truth"]
    assert t["values"]["drive_minus_meas_q_median_rad"]["judgeable"]  # the record alone gives it


def test_m2_truth_over_budget():
    name = "m2_q008_left_fwd"
    out = make(name)
    out["truth"][0]["sigma"] = {"pose_xy_m": 5e-3, "pose_yaw_rad": 3e-3, "source": "synthetic"}
    t = trial1(run(out, name))
    assert t["truth_budget"]["ok"] is False
    assert t["values"]["gain_measured_q"]["why"] == ["truth_over_budget"]


def test_m2_reports_the_imu_curvature_never_judgeable_without_the_gyro_scale():
    """Codex PR 2 review P2 (registration 4.3): IMU kappa is a report value needing the G3 scale."""
    import yaml as _yaml

    prof = _yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / "m2_q008_left_fwd.yaml").read_text())
    out = ms.synth_trial(prof, CFG, still_s=60.0, seed=1)
    out["truth"][0]["sigma"] = {"pose_yaw_rad": 1e-4, "pose_xy_m": 2e-4, "source": "test"}
    res = me.analyze(out["record"], out["imu"], out["truth"], None, prof, CFG, (0.0, -1.0, 0.0),
                     allow_synthetic=True)
    tr = res["trials"][0]
    k = tr["values"]["imu_kappa_inv_m"]
    assert k["judgeable"] is False and k["why"][0] == "no_gyro_scale" and k["report_only"] is True
    assert k["value"] is not None  # 60 s stills: the bias windows exist, so the IMU heading does too
    if True:  # the IMU heading over the arc, over the true arc length
        assert k["distance_source"] == "truth"
        assert abs(k["value"] - tr["values"]["kappa_true_inv_m"]["value"]) < 0.05 * abs(tr["values"]["kappa_true_inv_m"]["value"])

