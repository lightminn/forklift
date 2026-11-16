"""The rehearsal truth stand-in (rehearsal_truth/1, heading only) read by the G8 entry under --allow-synthetic only
(entry design 2; rehearsal 2 R2-2)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from forklift_core.plant import m_analysis as ma
from forklift_core.plant import m_entry as me
from forklift_core.plant import m_synth as ms
from forklift_core.plant.real_vehicle import load_real_vehicle

REPO = Path(__file__).resolve().parents[3]
CFG = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
M1 = yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / "m1_fwd_v010.yaml").read_text(encoding="utf-8"))


def rehearsal_truth(out):
    """What tools/rehearsal_imu_truth.py writes: per trial the heading change between its ends."""
    pts = out["truth"][1]["points"]
    d = ma.relative_pose((pts["start"]["x_m"], pts["start"]["y_m"], pts["start"]["yaw_rad"]),
                         (pts["end"]["x_m"], pts["end"]["y_m"], pts["end"]["yaw_rad"]))["dyaw"]
    return [{"t": "header", "schema": "rehearsal_truth/1", "synthetic": True, "source_record": "x"},
            {"t": "truth", "trial": out["truth"][1]["trial"], "dyaw_rad": d, "method": "synthetic"}]


def test_rehearsal_truth_needs_allow_synthetic():
    out = ms.synth_trial(M1, CFG, still_s=3.0, true_yaw_per_m=0.01)
    with pytest.raises(me.Refused, match="rehearsal"):
        me.analyze(out["record"], out["imu"], rehearsal_truth(out), None, M1, CFG, (0, -1, 0), allow_synthetic=False)


def test_rehearsal_truth_gives_the_m1_heading_drift_on_encoder_distance():
    yaw = 0.01
    out = ms.synth_trial(M1, CFG, still_s=3.0, true_yaw_per_m=yaw)
    res = me.analyze(out["record"], out["imu"], rehearsal_truth(out), None, M1, CFG, (0, -1, 0), allow_synthetic=True)
    (t,) = res["trials"]
    v = t["values"]
    assert t["status"] == "valid" and res["inputs"]["truth"]["schema"] == "rehearsal_truth/1"
    assert v["d_rad_per_m"]["value"] == pytest.approx(-yaw, rel=0.05)
    assert v["d_rad_per_m"]["distance_source"] == "encoder"
    assert v["end_lateral_m"]["why"] == ["heading_only_truth"] and v["radius_scale"]["why"] == ["heading_only_truth"]
