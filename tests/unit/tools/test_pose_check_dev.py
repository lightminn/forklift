"""The unconfirmed position check's development values from the shadow D sessions (tools/pose_check_dev.py,
docs/plans/2026-11-06-unconfirmed-validation.md 3.2): the odometry bound a + b d over every sample from the start and
from each stroke's start, E_STILL from every stand, E_SLAM from the agreements the operating path would check, each
1.5 x the observed largest, with the samples, the distance bins and the inputs' hashes."""

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import pose_check_dev as D  # noqa: E402


def test_the_bound_is_the_least_at_3_m_over_every_sample_and_then_the_smaller_constant():
    pts = [(0.0, 0.004), (1.0, 0.010), (3.0, 0.020)]  # (d, e): the bound must hold 1.5 e
    a, b = D.fit_bound(pts)
    assert (a, b) == pytest.approx((0.0075, 0.0075))
    assert all(a + b * d >= 1.5 * e - 1e-12 for d, e in pts)
    assert D.fit_bound([(0.5, 0.01)]) == pytest.approx((0.015, 0.0))  # one sample: the constant (smaller a + 3b tie)
    assert D.fit_bound([]) == (0.0, 0.0)


def _session(drift=0.01, yaw=0.3, strokes=(1.0, 1.0), stands=((0.004, 0.001),), start_agree=(0.03, 0.01),
             rematch_agree=((0.05, 0.02, "confirm"),), shadow=True):
    """A reverse along the heading: the odometry moves (1 + drift) of the truth's distance; strokes of those lengths."""
    samples, events, t, s_total = [], [], 0.0, 0.0
    c, s = math.cos(yaw), math.sin(yaw)

    def at(dist, scale):
        return [-(dist * scale) * c, -(dist * scale) * s, yaw]

    samples.append({"t": t, "o": at(0, 1), "truth": at(0, 1)})
    for k, length in enumerate(strokes):
        t += 1.0
        samples.append({"t": t - 0.05, "o": at(s_total, 1 + drift), "truth": at(s_total, 1)})  # stood before the press
        events.append({"t": t, "kind": "stroke_open"})
        for step in range(1, 11):
            t += 0.1
            samples.append({"t": t, "o": at(s_total + length * step / 10, 1 + drift),
                            "truth": at(s_total + length * step / 10, 1)})
        s_total += length
        t += 0.5
        events.append({"t": t, "kind": "stroke_done"})
    raw = {"start": {"r": [0, 0, yaw], "confirmed": True, "agree_m": start_agree[0], "agree_rad": start_agree[1],
                     "truth": {"class": "success"}},
           "rematches": [{"g": [0, 0, 0], "outcome": out, "residual_confirmed": out == "confirm", "applicable": True,
                          "agree_m": m, "agree_rad": r, "truth": {"class": "success"}}
                         for m, r, out in rematch_agree],
           "stands": [{"max_m": m, "max_rad": r, "t0": 0.0, "t1": 0.0} for m, r in stands]}
    rec = {"mode": "shadow" if shadow else "on", "start": {"scan_t": 0.0}, "samples": samples, "raw": raw,
           "meter_events": events, "events": []}
    return rec


def test_the_odometry_errors_run_from_the_start_and_from_each_strokes_start():
    rec = _session(drift=0.01, strokes=(1.0, 1.0))
    from_start, from_strokes = D.odometry_errors(rec)
    d, e_p, e_th = from_start[-1]
    assert d == pytest.approx(2.02) and e_p == pytest.approx(0.02) and e_th == pytest.approx(0.0)
    assert len(from_strokes) == 20 and max(e for _, e, _ in from_strokes) == pytest.approx(0.01)


def test_the_development_values_and_their_basis(tmp_path):
    recs = [("s1", _session()), ("s2", _session(stands=((0.01, 0.004),), start_agree=(0.06, 0.02),
                                                 rematch_agree=((0.08, 0.03, "confirm"), (2.0, 1.0, "twin_preferred"))))]
    out = D.dev_values(recs, min_samples=2)
    assert out["e_still"] == pytest.approx([0.015, 0.006])
    assert out["e_slam"] == pytest.approx([0.12, 0.045])  # the re-match's 0.08 m, 0.03 rad: the larger of the two kinds
    assert out["basis"]["e_slam"]["start"]["n"] == 2 and out["basis"]["e_slam"]["rematch"]["n"] == 2  # not the twin
    assert out["basis"]["e_slam"]["rematch"]["all_g_n"] == 3
    a, b, c, e = out["odom"]
    # the samples reach 2.02 m, their error in proportion: least at 3.0 m is the constant 1.5 x 0.02 (the rule as fixed)
    assert (a, b) == pytest.approx((1.5 * 0.02, 0.0))
    assert (c, e) == pytest.approx((0.0, 0.0), abs=1e-12)
    assert out["basis"]["odom"]["bins"]["2.00"] == 2 and out["flags"] == []
    assert json.loads(json.dumps(out))


def test_too_few_samples_or_a_still_bound_over_the_budget_is_flagged_not_cut():
    out = D.dev_values([("s1", _session(stands=((0.2, 0.001),)))], min_samples=20)
    assert out["e_still"][0] == pytest.approx(0.3)  # 1.5 x 0.2: not cut to 0.20 (plan 3.2: reported to the lead)
    assert "e_still_over_budget" in out["flags"] and "start_samples_under_20" in out["flags"]
    assert "rematch_samples_under_20" in out["flags"]


def test_evidence_outside_the_success_tolerance_is_kept_and_counted():
    rec = _session()
    rec["raw"]["rematches"][0]["truth"] = {"class": "imprecise"}
    out = D.dev_values([("s1", rec)], min_samples=1)
    assert out["basis"]["e_slam"]["rematch"]["n"] == 1 and out["basis"]["e_slam"]["outside_success"] == 1


def test_the_cli_reads_each_sessions_record_and_hashes_it(tmp_path):
    for name in ("a", "b"):
        run = tmp_path / name / "epoch_1" / "run"
        run.mkdir(parents=True)
        (run / "result.json").write_text(json.dumps({"pose_check": _session()}))
    out = tmp_path / "dev.json"
    assert D.main([str(tmp_path / "a"), str(tmp_path / "b"), "--out", str(out), "--min-samples", "1"]) == 0
    data = json.loads(out.read_text())
    assert [i["session"] for i in data["inputs"]] == ["a", "b"] and len(data["inputs"][0]["sha256"]) == 64
    assert set(data) >= {"odom", "e_still", "e_slam", "basis", "flags", "inputs"}


def test_the_samples_eligibility_is_the_conservative_check_and_the_operating_paths_application():
    # Codex 2d P2-3, P2-4: never the provisional outcome; a result the operating path discards is out
    rec = _session(rematch_agree=((0.05, 0.02, "confirm"), (0.09, 0.03, "confirm"), (0.2, 0.05, "none")))
    rec["raw"]["rematches"][0]["outcome"] = "out_of_set"  # the provisional outcome says nothing: still a sample
    rec["raw"]["rematches"][1]["applicable"] = False  # moved after its scans: discarded when on
    out = D.dev_values([("s", rec)], min_samples=1)
    r = out["basis"]["e_slam"]["rematch"]
    assert r["n"] == 1 and r["max"] == pytest.approx([0.05, 0.02]) and r["all_g_n"] == 3 and r["not_applicable"] == 1


def test_the_strokes_cut_short_are_counted_beside_the_bins():
    # Codex 2d round 3: many cuts mean fewer long samples and more stops and starts: said with the distance bins
    rec = _session()
    rec["meter_events"] += [{"t": 1.0, "kind": "stroke_close", "s_m": 0.01, "released": True},
                            {"t": 2.0, "kind": "stroke_close", "s_m": 0.93, "released": False}]
    odom = D.dev_values([("s", rec)], min_samples=1)["basis"]["odom"]
    assert odom["strokes"] == {"closed": 2, "under_0_05_m": 1, "under_0_5_m": 1}


def test_wrong_evidence_among_the_samples_is_flagged():
    rec = _session()
    rec["raw"]["rematches"][0]["truth"] = {"class": "wrong"}
    assert "wrong_evidence_in_samples" in D.dev_values([("s", rec)], min_samples=1)["flags"]


def test_an_origin_is_the_sample_at_the_boundary_and_a_window_without_one_is_counted():
    # Codex 2d P2-5: never a sample after the boundary
    rec = _session(drift=0.0)
    rec["samples"] = rec["samples"][1:]  # nothing at or before the start's scan
    from_start, _ = D.odometry_errors(rec)
    assert from_start == []
    assert D.dev_values([("s", rec)], min_samples=1)["basis"]["odom"]["windows_without_origin"] >= 1


# --- the design revision's evidence (plan 6 step 3): Delta_int beside the truth, the standing drift ---
def _driven(steps=200, step_m=0.1, delta_per_m=0.02, err=0.03):
    """A straight drive with the correction moving delta_per_m per metre and a steady truth error."""
    samples = []
    for k in range(steps):
        d = k * step_m
        samples.append({"t": 0.1 * k, "o": [d, 0.0, 0.0], "truth": [d, err, 0.0], "applied": [d, 0.0, 0.0],
                        "delta": [delta_per_m * d, 0.1 * delta_per_m * d], "ref": "int", "open": True})
    return {"samples": samples, "events": [{"t": 0.0, "kind": "slam_agreement", "ok": True}], "raw": {"stands": [],
                                                                                                    "rematches": []}}


def test_the_delta_is_kept_against_the_distance_since_its_reference_with_the_truth_error_beside_it():
    rows = D.delta_rows(_driven())
    assert rows[-1]["since_m"] == pytest.approx(19.9) and rows[-1]["delta_m"] == pytest.approx(0.398)
    assert rows[-1]["truth_m"] == pytest.approx(0.03)
    out = D.design_evidence([("a", _driven())])
    bin_ = out["delta_by_distance"]["10-20"]
    assert bin_["sessions"] == 1 and bin_["delta_m_max"] == pytest.approx(0.398) and bin_["truth_m_max"] == pytest.approx(0.03)
    [first] = out["crossings"]
    assert first["session"] == "a" and first["since_m"] == pytest.approx(10.0, abs=0.11)
    assert first["truth_m"] == pytest.approx(0.03) and first["gate"] == "m"


def test_a_reference_renewed_starts_the_distance_again():
    rec = _driven()
    rec["events"].append({"t": 10.0, "kind": "rematch_agreement", "ok": True})
    rows = D.delta_rows(rec)
    assert rows[-1]["since_m"] == pytest.approx(9.9)


def test_a_samples_delta_from_the_start_evidence_is_not_the_monitors():
    rec = _driven()
    for s in rec["samples"]:
        s["ref"] = "start"
    assert D.delta_rows(rec) == []


def _stand(t0, seconds, rate):
    return [{"t": t0 + 0.1 * k, "o": [1.0, 2.0, rate * 0.1 * k], "truth": [1.0, 2.0, 0.0], "applied": [1.0, 2.0, 0.0]}
            for k in range(int(seconds * 10))]


def test_the_standing_drift_is_the_odometrys_heading_rate_with_the_truth_still():
    rec = {"samples": _stand(0.0, 5.0, 0.002) + _stand(10.0, 1.0, 0.5) + _stand(20.0, 5.0, -0.0004),
           "raw": {"stands": [{"t0": 0.0, "t1": 4.9}, {"t0": 10.0, "t1": 10.9}, {"t0": 20.0, "t1": 24.9}],
                   "rematches": [{"g": [0, 0, 0], "moved": [0.0001, 0.003]}, {"g": [0, 0, 0], "moved": [0.0001, 0.0002]},
                                 {"g": None}]}}
    out = D.design_evidence([("a", rec)])["standing_drift"]
    assert out["stands"] == 2 and out["rate_max"] == pytest.approx(0.002)  # the 1 s stand is too short to read
    assert out["over_still_in_1_5_s"] == 1 and out["generations"] == 2 and out["moved_over_still"] == 1


def test_a_window_in_contact_is_left_out_of_the_odometry_bound_and_counted():
    # D session val3_016: the forks on a scene pallet from the start, the wheels slipping -- not a straight reverse's
    # odometry. The rule fixed after three sessions and before the rest were read.
    rec = _session(drift=0.01)
    clean = D.dev_values([("s", rec)], min_samples=1)["basis"]["odom"]
    rec["contacts"] = {"intervals": [{"t0": 0.0, "t1": 2.5, "body": "b", "other": "o"}]}  # the first stroke's time
    odom = D.dev_values([("s", rec)], min_samples=1)["basis"]["odom"]
    assert odom["windows_in_contact"] == 2  # from the start and the first stroke; the second stroke is kept
    assert 0 < odom["samples"] < clean["samples"] and odom["windows_without_origin"] == 0


def test_a_generation_scanned_in_contact_stays_out_of_e_slam_and_a_floor_decal_is_no_contact():
    # val3_016: the agreement is the slip's, 0.34 m, the g itself right; val3_042: a floor decal's collider
    rec = _session(rematch_agree=((0.05, 0.02, "confirm"), (0.34, 0.01, "confirm")))
    rec["raw"]["rematches"][1]["scan_t"] = 3.0
    rec["contacts"] = {"intervals": [{"t0": 2.9, "t1": 3.1, "body": "b", "other": "/World/Box"},
                                     {"t0": 0.0, "t1": 9.0, "body": "b", "other": "/World/x/SM_FloorDecal_RecRed/m"}]}
    r = D.dev_values([("s", rec)], min_samples=1)["basis"]["e_slam"]["rematch"]
    assert r["n"] == 1 and r["max"] == pytest.approx([0.05, 0.02]) and r["in_contact"] == 1


def test_the_values_without_the_contact_rule_can_be_had(tmp_path):
    rec = _session(rematch_agree=((0.05, 0.02, "confirm"), (0.34, 0.01, "confirm")))
    rec["raw"]["rematches"][1]["scan_t"] = 3.0
    rec["contacts"] = {"intervals": [{"t0": 2.9, "t1": 3.1, "body": "b", "other": "/World/Box"}]}
    run = tmp_path / "a" / "epoch_1" / "run"
    run.mkdir(parents=True)
    (run / "result.json").write_text(json.dumps({"pose_check": rec}))
    out = tmp_path / "o.json"
    assert D.main([str(tmp_path / "a"), "--out", str(out), "--min-samples", "1", "--ignore-contacts"]) == 0
    assert json.loads(out.read_text())["basis"]["e_slam"]["rematch"]["max"][0] == pytest.approx(0.34)
