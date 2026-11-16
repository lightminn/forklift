"""Registration revision 4 (11.2-11.4): the tape truth (`truth/1` method `tape`, the left rear axle end's distance
along and offset from the start line in cm) and the cm/degree values with their L/H bands. Budgets, k limits and the
bias slope never enter these values."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from forklift_core.plant import m_entry as me
from forklift_core.plant import m_rev4 as r4
from forklift_core.plant import m_synth as ms
from forklift_core.plant.real_vehicle import load_real_vehicle

REPO = Path(__file__).resolve().parents[3]
CFG = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
UP = (0.0, -1.0, 0.0)


def profile(name):
    return yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / f"{name}.yaml").read_text(encoding="utf-8"))


def tape_rows(truth_rows, *, ds_cm=0.0, dy_cm=0.0, skip=()):
    """A tape file made from the synthetic truth: the left rear axle end seen from its start position and heading."""
    h = truth_rows[0]
    out = [{"t": "header", "schema": "truth/1", "method": "tape", "synthetic": True, "record": dict(h["record"]),
            "body_point": "left_rear_axle_end", "frame": "start_tape_line", "sigma": {"pos_m": 0.02}}]
    w2 = CFG.track_m / 2
    for r in truth_rows[1:]:
        if r.get("t") != "truth" or r["trial"] in skip:
            continue
        p0, p1 = r["points"]["start"], r["points"]["end"]
        c, s = math.cos(p0["yaw_rad"]), math.sin(p0["yaw_rad"])
        ex, ey = p1["x_m"] - p0["x_m"], p1["y_m"] - p0["y_m"]
        dx, dy, dth = c * ex + s * ey, -s * ex + c * ey, p1["yaw_rad"] - p0["yaw_rad"]
        out.append({"t": "tape", "trial": r["trial"], "s_cm": 100 * (dx - math.sin(dth) * w2) + ds_cm,
                    "y_cm": 100 * (dy + (math.cos(dth) - 1) * w2) + dy_cm})
    return out


def run(name, n=1, **tape):
    out = ms.synth_block([{"profile": profile(name)} for _ in range(n)], CFG, seed=11)
    truth = out["truths"][-1]
    rows = tape_rows(truth, **tape) if tape.get("skip") != "all" else None
    return me.analyze(out["records"][-1], out["imu"], rows, None, profile(name), CFG, UP, allow_synthetic=True,
                      no_truth=rows is None)


def rev4(res):
    (t,) = res["trials"]
    assert t["status"] == "valid", t
    return t["rev4"]


def test_m1_tape_values_follow_their_definitions():
    res = run("m1_fwd_v010")
    v = rev4(res)
    assert res["profile_name"] == "m1_fwd_v010" and res["verdict"]["revision"] == r4.REVISION
    assert set(res["verdict"]["gates_not_applied"]) >= {"truth_budget", "k_limit", "bias_slope"}
    t = res["trials"][0]
    s_enc = t["distance"]["s_enc_path_m"]
    de = v["distance_error"]
    assert de["value"] == pytest.approx((s_enc - de["s_tape_m"]) / de["s_tape_m"])
    # the synthetic truth and odometry agree to well under a centimetre: the left-point geometry and signs are right
    assert abs(v["end_lateral_cm"]["value"]) < 1.0 and v["end_lateral_cm"]["band"] == "good"
    hd = v["heading_drift_deg"]
    assert hd["value"] == pytest.approx(hd["odo_dyaw_deg"] - math.degrees(t["imu"]["dyaw_rad"]))
    assert hd["band"] in ("good", "watch", "action")


def test_tape_offsets_move_the_values_and_the_bands():
    base = rev4(run("m1_fwd_v010"))
    off = rev4(run("m1_fwd_v010", dy_cm=7.0, ds_cm=-12.0))
    assert off["end_lateral_cm"]["value"] == pytest.approx(base["end_lateral_cm"]["value"] - 7.0)
    assert off["end_lateral_cm"]["band"] == "watch"  # 5 <= |v| < 10 cm
    # s_tape 12 cm short of ~3 m: about +4 %, between L 3 % and H 6 %
    assert off["distance_error"]["band"] == "watch" and off["distance_error"]["value"] > 0.03
    far = rev4(run("m1_fwd_v010", dy_cm=-11.0))
    assert far["end_lateral_cm"]["band"] == "action"


def test_without_tape_the_heading_value_still_comes():
    v = rev4(run("m1_fwd_v010", skip="all"))
    assert v["distance_error"]["value"] is None and v["distance_error"]["why"] == ["no_tape"]
    assert v["end_lateral_cm"]["value"] is None and v["heading_drift_deg"]["value"] is not None


def test_a_trial_without_a_tape_row_is_not_void():
    out = ms.synth_block([{"profile": profile("m1_fwd_v010")}], CFG, seed=12)
    rows = tape_rows(out["truths"][-1], skip=(1,))
    res = me.analyze(out["records"][-1], out["imu"], rows, None, profile("m1_fwd_v010"), CFG, UP,
                     allow_synthetic=True)
    t = res["trials"][0]
    assert t["status"] == "valid" and t["rev4"]["end_lateral_cm"]["why"] == ["tape_missing"]


def test_m3_merge_gets_lateral_and_heading_difference():
    v = rev4(run("m3_merge_o20_l09_left"))
    assert set(v) >= {"end_lateral_cm", "heading_diff_deg"}
    assert abs(v["end_lateral_cm"]["value"]) < 1.0


def test_m2_gain_is_reported_without_a_band():
    res = run("m2_q008_left_fwd", skip="all")
    t = res["trials"][0]
    g = t["rev4"]["gain"]
    kappa = t["values"]["kappa_measured_q_inv_m"]["value"]
    assert g["value"] == pytest.approx(abs(t["imu"]["dyaw_rad"]) / t["distance"]["s_enc_path_m"] / abs(kappa))
    assert g["report_only"] is True and "band" not in g


@pytest.mark.parametrize("bad, why", [
    ({"sigma": {}}, "sigma"),
    ({"body_point": "rear_axle_center"}, "body_point"),
    ({"frame": "floor"}, "frame"),
])
def test_tape_header_is_checked(bad, why):
    out = ms.synth_block([{"profile": profile("m1_fwd_v010")}], CFG, seed=13)
    rows = tape_rows(out["truths"][-1])
    rows[0] = {**rows[0], **bad}
    with pytest.raises(me.Refused, match=why):
        me.analyze(out["records"][-1], out["imu"], rows, None, profile("m1_fwd_v010"), CFG, UP, allow_synthetic=True)


def test_tape_rows_are_checked():
    out = ms.synth_block([{"profile": profile("m1_fwd_v010")}], CFG, seed=14)
    rows = tape_rows(out["truths"][-1])
    for bad in ({"trial": 9}, {"s_cm": float("nan")}, {"s_cm": -3.0}):
        broken = rows[:1] + [{**rows[1], **bad}]
        with pytest.raises(me.Refused, match="tape"):
            me.analyze(out["records"][-1], out["imu"], broken, None, profile("m1_fwd_v010"), CFG, UP,
                       allow_synthetic=True)
    with pytest.raises(me.Refused, match="two rows"):
        me.analyze(out["records"][-1], out["imu"], rows + [rows[1]], None, profile("m1_fwd_v010"), CFG, UP,
                   allow_synthetic=True)


def _res(name, trials):
    return {"schema": "m_analyze/1", "profile_name": name, "measurement": name.split("_")[0] if not
            name.startswith("m3_merge") else "m3_merge", "trials": trials}


def _t(**rev):
    return {"status": "valid", "rev4": {k: {"value": v} for k, v in rev.items()}}


def test_summary_medians_bands_and_m2_ratio():
    results = [
        _res("m1_fwd_v010", [_t(distance_error=0.01, end_lateral_cm=2.0, heading_drift_deg=0.2),
                             _t(distance_error=0.02, end_lateral_cm=-12.0, heading_drift_deg=0.4)]),
        _res("m1_fwd_v010", [_t(distance_error=0.015, end_lateral_cm=3.0, heading_drift_deg=0.3)]),
        _res("m2_q008_left_fwd", [_t(gain=1.10)]),
        _res("m2_q008_right_fwd", [_t(gain=0.95)]),
        _res("m2_max_left_fwd", [_t(gain=1.0)]),
        _res("m2_max_right_fwd", [{"status": "void", "why": "x"}]),
        _res("m3_merge_o20_l09_left", [_t(end_lateral_cm=1.0, heading_diff_deg=-1.5)]),
    ]
    s = r4.summary(results)
    m1 = s["m1"]
    assert m1["end_lateral_cm"]["median"] == pytest.approx(2.0) and m1["end_lateral_cm"]["band"] == "good"
    assert m1["end_lateral_cm"]["over_h"] == [-12.0]  # one trial over H is named for a look (11.3)
    assert m1["distance_error"]["n"] == 3
    q = s["m2"]["lr_gain_ratio"]["m2_q008_fwd"]
    assert q["value"] == pytest.approx(1.10 / 0.95) and q["band"] == "watch"  # |r - 1| = 15.8 %
    assert s["m2"]["lr_gain_ratio"]["m2_max_fwd"]["value"] is None  # the right arc has no valid trial
    assert s["m3_merge"]["heading_diff_deg"]["band"] == "watch"
    assert s["voids"] == 1


def test_summary_cli_reads_m_analyze_lines(tmp_path, capsys):
    import importlib.util
    import json

    spec = importlib.util.spec_from_file_location("m_summary", REPO / "tools" / "m_summary.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    res = run("m1_fwd_v010")
    a = tmp_path / "a.json"
    a.write_text(json.dumps(res, default=str) + "\n")
    assert mod.main([str(a)]) == 0
    s = json.loads(capsys.readouterr().out)
    assert s["result"] == "ok" and s["m1"]["end_lateral_cm"]["n"] == 1
    bad = tmp_path / "b.json"
    bad.write_text(json.dumps({"result": "refused", "why": "x"}) + "\n")
    assert mod.main([str(a), str(bad)]) == 2


def test_a_tape_void_row_voids_the_trial():
    out = ms.synth_block([{"profile": profile("m1_fwd_v010")}], CFG, seed=15)
    rows = tape_rows(out["truths"][-1]) + [{"t": "void", "trial": 1, "reason": "tape_bumped",
                                             "closed_before_results": True}]
    res = me.analyze(out["records"][-1], out["imu"], rows, None, profile("m1_fwd_v010"), CFG, UP,
                     allow_synthetic=True)
    t = res["trials"][0]
    assert t["status"] == "void" and t["why"] == "field:tape_bumped" and "rev4" not in t
    rows[-1] = {k: v for k, v in rows[-1].items() if k != "closed_before_results"}
    with pytest.raises(me.Refused, match="closed_before_results"):
        me.analyze(out["records"][-1], out["imu"], rows, None, profile("m1_fwd_v010"), CFG, UP,
                   allow_synthetic=True)


def test_gyro_window_must_hold_every_wheel_movement():
    w = {"before": [100, 200], "after": [900, 1000]}
    assert r4.gyro_covers(w, 210, 880)
    assert not r4.gyro_covers(w, 190, 880)  # the before window runs into the motion
    assert not r4.gyro_covers(w, 210, 950)  # the motion goes on into the after window
    assert not r4.gyro_covers(None, 210, 880)
    v = r4.trial_values("m1", rel_odo={"dx_fwd": 3.0, "dy_lat": 0.0, "dyaw": 0.0}, s_enc_m=3.0, gyro_dyaw_rad=None,
                        gyro_why="gyro_window_not_covering_motion", tape=None, tape_why="no_tape", half_track_m=0.1)
    assert v["heading_drift_deg"]["why"] == ["gyro_window_not_covering_motion"]


def test_the_lever_term_for_a_turn():
    # rear axle centre turns 90 deg left on the spot: the left axle end swings back by w/2 and right by w/2
    s, y = r4.left_point({"dx_fwd": 0.0, "dy_lat": 0.0, "dyaw": math.pi / 2}, 0.1)
    assert s == pytest.approx(-0.1) and y == pytest.approx(-0.1)
