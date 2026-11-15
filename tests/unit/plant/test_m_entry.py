"""m_entry.analyze on synthetic trials (docs/design/2026-11-15-m-analyze-entry.md 6, PR 1)."""

from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
import yaml

from forklift_core.plant import m_entry as me
from forklift_core.plant import m_synth as ms
from forklift_core.plant.real_vehicle import load_real_vehicle

REPO = Path(__file__).resolve().parents[3]
CFG = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
M1 = yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / "m1_fwd_v010.yaml").read_text(encoding="utf-8"))
SIGMA = {"pose_xy_m": 5e-4, "pose_yaw_rad": 3e-4, "source": "synthetic"}  # inside the registered budget at 3 m
UP = (0.0, -1.0, 0.0)
SHORT = 3.0


def make(profile=M1, **kw):
    kw.setdefault("still_s", SHORT)
    out = ms.synth_trial(profile, CFG, **kw)
    out["truth"][0]["sigma"] = dict(SIGMA)
    return out


def run(out, *, profile=M1, truth=True, **kw):
    kw.setdefault("allow_synthetic", True)
    return me.analyze(out["record"], out["imu"], out["truth"] if truth else None, None, profile, CFG, UP,
                      no_truth=not truth, **kw)


def why_of(fn):
    with pytest.raises(me.Refused) as e:
        fn()
    return e.value.why


def with_field(out, fl):
    return lambda: me.analyze(out["record"], out["imu"], out["truth"], fl, M1, CFG, UP, allow_synthetic=True)


@pytest.fixture(scope="module")
def drift():
    return make(true_yaw_per_m=0.003, true_radius_scale=1.02, seed=3)


def test_registered_constants_exposed():
    assert me.REGISTERED["revision"] == "초안 개정 2"
    assert (me.REGISTERED["quiet_max_s"], me.REGISTERED["quiet_min_s"]) == (60.0, 50.0)
    assert me.REGISTERED["quiet_gyro_tol_rad_s"] == 0.025 and me.REGISTERED["desk_n_rad_sqrt_s"] == 2.054e-4
    assert me.REGISTERED["budget_dyaw_rad"] == 1e-3 and me.REGISTERED["budget_lat_m"] == 1.7e-3
    assert me.IMU_GAP_NS == 10_000_000 and me.MAX_GAP_S == 0.05


def test_m1_recovers_drift_and_radius(drift):
    res = run(drift)
    assert list(res)[:2] == ["synthetic_inputs", "not_for_judgement"]
    assert res["schema"] == "m_analyze/1" and res["measurement"] == "m1"
    assert res["registration"]["constants"]["revision"] == "초안 개정 2"
    t = res["trials"][0]
    assert t["status"] == "valid" and res["counts"] == dict(trials=1, valid=1, void={})
    d = t["values"]["d_rad_per_m"]
    assert d["judgeable"] and d["value"] == pytest.approx(-0.003, abs=1e-4)
    # the encoders over-read by 1.02, so true / encoder path is 1 / 1.02 (the entry's definition)
    assert t["distance"]["radius_scale"] == pytest.approx(1 / 1.02, rel=2e-3)
    assert t["distance"]["s_enc_path_m"] >= t["distance"]["disp_enc_m"] == pytest.approx(3.006, rel=1e-3)
    lat = t["values"]["end_lateral_m"]
    assert lat["report_only"] and lat["judgeable"]
    assert t["imu"]["why"] == ["no_bias_window"]  # 3 s holds: no 50 s window, the trial stays valid


def test_truth_over_budget_not_judgeable():
    out = ms.synth_trial(M1, CFG, still_s=SHORT)  # default synthetic sigma: 2 mm lateral over 3 m
    v = run(out)["trials"][0]["values"]["d_rad_per_m"]
    assert not v["judgeable"] and v["why"] == ["truth_over_budget"]


def test_registered_windows_give_imu_heading():
    out = make(M1, still_s=None, true_yaw_per_m=0.01, bias=(0.0, 3e-4, 0.0), seed=1)
    t = run(out)["trials"][0]
    imu = t["imu"]
    assert imu["why"] == [] and imu["report_only"]
    tol = 4 * imu["sigma_lower"]["body"]["lower_bound"] + 1e-3
    assert imu["dyaw_rad"] == pytest.approx(t["distance"]["dyaw_true_rad"], abs=tol)
    assert imu["n_body"]["before"] == pytest.approx(ms.ARW_DESK, rel=0.3)
    assert abs(imu["bias_change_yaw_rad_s"]) < me.REGISTERED["bias_jump_rad_s"]


@pytest.mark.parametrize("void, why", [("deadman", "void:deadman"), ("ENCODER_JUMP_L", "fault:ENCODER_JUMP_L"),
                                       ("boot_change", "mcu_boot_changed"), ("encoder_gap", "odometry:gap")])
def test_void_trials_are_void_with_their_reason(void, why):
    res = run(make(void=void))
    t = res["trials"][0]
    assert t["status"] == "void" and t["why"] == why and "values" not in t
    key = why if why.startswith("void:") else "void:" + why
    assert res["counts"]["void"] == {key: 1} and res["counts"]["valid"] == 0


def test_synthetic_refused_without_allow_and_marked_with():
    out = make()
    assert "synthetic" in why_of(lambda: run(out, allow_synthetic=False))
    assert run(out)["synthetic_inputs"] is True


def test_synthetic_truth_alone_refused():
    out = make(synthetic=False)
    # Codex D4 impl review P1: a truth whose method is synthetic but whose header is unmarked is refused
    assert "method synthetic" in why_of(lambda: run(out, allow_synthetic=False))
    out["truth"][0]["method"] = "A_floor_marks"  # an unmarked real-looking day
    assert "synthetic_inputs" not in run(out, allow_synthetic=False)
    out["truth"][0]["method"] = "synthetic"
    out["truth"][0]["synthetic"] = True
    assert why_of(lambda: run(out, allow_synthetic=False)).startswith("truth")
    assert run(out)["not_for_judgement"] is True


def test_identity_refusals():
    out = make()
    bad = copy.deepcopy(out)
    bad["imu"][0]["host_boot_id"] = "other"
    assert "host_boot_id" in why_of(lambda: run(bad))
    bad = copy.deepcopy(out)
    bad["truth"][0]["record"]["start_mono_ns"] += 1
    assert "start_mono_ns" in why_of(lambda: run(bad))
    other = dict(M1, name="m1_fwd_v005")
    assert "profile" in why_of(lambda: run(out, profile=other))
    bad = copy.deepcopy(out)
    bad["record"].pop()
    assert "footer" in why_of(lambda: run(bad))
    bad = copy.deepcopy(out)
    bad["record"][-1]["dropped"] = 1
    assert "footer" in why_of(lambda: run(bad))
    bad = copy.deepcopy(out)
    bad["truth"].pop(0)
    assert "header" in why_of(lambda: run(bad))
    assert "m1_" in why_of(lambda: run(out, profile=dict(M1, name="g3_arc_left")))


def test_truth_rows_rules():
    out = make()
    dup = copy.deepcopy(out)
    dup["truth"].append(copy.deepcopy(dup["truth"][1]))
    assert "two rows" in why_of(lambda: run(dup))
    extra = copy.deepcopy(out)
    extra["truth"].append(dict(extra["truth"][1], trial=9))
    assert "trial 9" in why_of(lambda: run(extra))
    moving = copy.deepcopy(out)
    moving["truth"][1]["points"]["run_042"] = dict(moving["truth"][1]["points"]["end"], kind="moving_point")
    assert "needs host_ns and cov" in why_of(lambda: run(moving))
    unknown = copy.deepcopy(out)
    unknown["truth"][1]["points"]["zzz"] = dict(unknown["truth"][1]["points"]["end"])
    assert "zzz" in why_of(lambda: run(unknown))


def test_field_void_rows_and_log_identity():
    out = make()
    voided = copy.deepcopy(out)
    voided["truth"].append(dict(t="void", trial=1, reason="bumped", closed_before_results=True))
    assert run(voided)["trials"][0]["why"] == "field:bumped"
    voided["truth"][-1]["closed_before_results"] = False
    assert "closed_before_results" in why_of(lambda: run(voided))
    rec_id = dict(host_boot_id=out["record"][0]["host_boot_id"], start_mono_ns=ms.T0_NS)
    head = dict(t="header", schema="field_log/1", synthetic=True)
    assert "identity" in why_of(with_field(out, [dict(head, record=dict(rec_id, host_boot_id="other"))]))

    def recheck(x, end):
        return dict(t="bias_recheck", block="a", exceeded_host_ns=x, recheck_start_ns=6, recheck_end_ns=end,
                    slope_rad_s2=0.0, passed=True)

    assert "time order" in why_of(with_field(out, [dict(head, record=rec_id), recheck(5, 7), recheck(4, 7)]))
    start = next(r["host_ns"] for r in out["record"] if r.get("t") == "profile" and r.get("state") == "running")
    assert "next trial" in why_of(with_field(out, [dict(head, record=rec_id), recheck(start - 10, start + 5)]))


def test_no_truth_and_truth_missing():
    out = make()
    t = run(out, truth=False)["trials"][0]
    assert t["status"] == "valid" and t["distance"]["source"] == "encoder"
    assert all(not v["judgeable"] and v["why"] == ["no_truth"] for v in t["values"].values())
    gone = copy.deepcopy(out)
    gone["truth"].pop(1)
    t = run(gone)["trials"][0]
    assert t["status"] == "void" and t["why"] == "truth_missing"
    assert "required" in why_of(lambda: me.analyze(out["record"], out["imu"], None, None, M1, CFG, UP,
                                                   allow_synthetic=True))
    both = lambda: me.analyze(out["record"], out["imu"], out["truth"], None, M1, CFG, UP,  # noqa: E731
                              allow_synthetic=True, no_truth=True)
    assert "not both" in why_of(both)


def test_m3_merge_is_routed_to_its_values():
    # M3's values: tests/unit/plant/test_m_entry_m3.py (PR 3a)
    path = REPO / "config" / "profiles" / "m1_m3" / "m3_merge_o05_l09_left.yaml"
    prof = yaml.safe_load(path.read_text(encoding="utf-8"))
    res = run(make(prof), profile=prof)
    assert res["measurement"] == "m3_merge"
    v = res["trials"][0]["values"]
    assert v["end"]["role"] == "judging_point" and "imu_heading" in v


def test_drift_sign_is_odo_minus_truth():
    d = run(make(true_yaw_per_m=0.01))["trials"][0]["values"]["d_rad_per_m"]["value"]
    assert math.copysign(1, d) == -1


def test_a_record_without_a_started_profile_is_refused():
    out = make()
    out["record"] = [r for r in out["record"] if r.get("t") != "profile_start"]
    assert "profile_start" in why_of(lambda: run(out))


def test_an_extrapolated_mcu_time_voids_the_trial(monkeypatch):
    out = make()
    real = me._state_times

    def extra(rows):
        srows, times, fi, fits, ex = real(rows)
        return srows, times, fi, fits, [True] * len(ex)  # as if every STATE lay outside the fit's device range

    monkeypatch.setattr(me, "_state_times", extra)
    res = run(out)
    assert all(tr["status"] == "void" and "mcu_clock_extrapolated" in tr["why"] for tr in res["trials"])


def test_an_imu_that_misses_the_bias_search_reports_imu_coverage_not_no_window():
    """Codex D4 impl review P2: coverage is the whole bias-window search the record allows, not the trial only."""
    out = make()
    first_move = min(r["host_ns"] for r in out["record"] if r.get("t") == "profile" and r.get("drive_mps"))
    out["imu"] = [r for r in out["imu"] if r.get("t") != "imu" or r["host_ns"] > first_move - 1_000_000_000]
    res = run(out)
    assert res["trials"][0]["status"] == "valid" and "imu_coverage" in res["trials"][0]["imu"]["why"]

