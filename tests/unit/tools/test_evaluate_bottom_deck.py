# tests/unit/tools/test_evaluate_bottom_deck.py
"""Two-face policy evaluation of the bottom-deck evidence (plan 2026-10-19 task 2)."""

import json
import math
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.perception import bottom_deck_evidence as B

from tools import bottom_deck_scenes as S
from tools import evaluate_bottom_deck as E


def test_bounds_match_the_plan():
    assert E.upper_bound(0, 299) == pytest.approx(1 - 0.05 ** (1 / 299), rel=1e-6)
    assert E.upper_bound(0, 298) > 0.01 > E.upper_bound(0, 299)
    assert E.lower_bound(278, 299) >= 0.90 > E.lower_bound(277, 299)


def test_policy_needs_both_faces():
    f = lambda *vs: [{"verdict": v} for v in vs]
    assert E.policy(f("present", "present")) == "present"
    assert E.policy(f("present", "unobserved")) == "unobserved"
    assert E.policy(f("unobserved", "absent")) == "absent"
    assert E.policy(f("present", "absent")) == "absent"


def test_draws_are_deterministic_and_bands_differ():
    a = E.scene_draws("epal6:x", "grounded+ledge2", 5, "dev")
    assert a == E.scene_draws("epal6:x", "grounded+ledge2", 5, "dev")
    assert a != E.scene_draws("epal6:x", "grounded+ledge2", 5, "eval")
    assert len(a["ledges"]) == 2 and len({x["face"] for x in a["ledges"]}) == 1
    assert len(E.scene_draws("epal6:x", "grounded+ledge1", 5, "dev")["ledges"]) == 1
    assert len(E.scene_draws("epal6:x", "grounded+ledge4", 5, "dev")["ledges"]) == 4
    assert E.scene_draws("epal6:x", "grounded", 5, "dev")["ledges"] == []


@pytest.mark.parametrize("group", ["epal6:x", "t11_06:y"])
def test_both_faces_are_the_two_opposite_model_faces(group):
    """Codex v7 P2: T11 y at +-55 deg saw the same +y face twice."""
    draws = E.scene_draws(group, "pallet", 0, "dev")
    seen = []
    for face in E.FACES:
        scene, target, _ = S.render_side(E.face_case(group, "pallet", draws, face, False, "rig"))
        seen.append(B.evaluate_side(scene, target).face)
    assert sorted(seen) == [-1, 1]


def test_one_scene_runs_under_every_configuration(tmp_path):
    rows = E.run_job(("epal6:x", "pallet", 0, "dev"))
    assert [(r["smooth"], r["K"]) for r in rows] == list(E.CONFIGS)
    assert all(r["opposite_faces"] for r in rows)
    assert all(r["final"] in ("present", "absent", "unobserved") for r in rows)
    path = tmp_path / "rows.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    out = E.summarise(rows)
    assert set(out) == {f"dev epal6:x smooth={s} K={k}" for s, k in E.CONFIGS}
    assert not any(res["pass"] for res in out.values())         # one scene is not a complete evaluation set


def test_damaged_face_cuts_only_gap_zero_of_its_outer_board():
    g = S.geometry("epal6")
    from tools import measure_pocket_evidence as mpe
    boxes = S._damaged(mpe.structure(g, "pallet"), g, 1)
    assert len(boxes) == len(mpe.structure(g, "pallet")) + 1
    gap_lo = -g.overall_depth_m / 2 + g.block_depth_m
    gap_hi = gap_lo + (g.overall_depth_m - 3 * g.block_depth_m) / 2
    for b in boxes:
        if b.centre_m[1] > 0 and abs(b.centre_m[2] - g.deck_bottom_m / 2) < 1e-9:
            lo, hi = b.centre_m[0] - b.size_m[0] / 2, b.centre_m[0] + b.size_m[0] / 2
            assert hi <= gap_lo + 1e-9 or lo >= gap_hi - 1e-9


def test_four_gap_band_scenes_are_set_apart():
    draws = E.scene_draws("epal6:x", "grounded+ledge4", 0, "dev")
    inside = [{**x, "h": 0.022 + d} for x, d in zip(draws["ledges"], (-0.009, 0.0, 0.005, 0.011))]
    assert E.ledge4_inside_band("epal6:x", "grounded+ledge4", {**draws, "ledges": inside})
    one_out = [{**x, "h": h} for x, h in zip(draws["ledges"], (0.020, 0.025, 0.012, 0.030))]
    assert not E.ledge4_inside_band("epal6:x", "grounded+ledge4", {**draws, "ledges": one_out})
    assert not E.ledge4_inside_band("epal6:x", "grounded+ledge2", {**draws, "ledges": inside})
    t11 = [{**x, "h": 0.015 + d} for x, d in zip(draws["ledges"], (-0.009, 0.0, 0.005, 0.011))]
    assert E.ledge4_inside_band("t11_06:x", "grounded+ledge4", {**draws, "ledges": t11})


@pytest.mark.parametrize("band", ["dev", "eval"])
def test_counted_negatives_stay_at_n(band):
    """Set-apart scenes must not shrink the bound's denominator below 299."""
    jobs = E.job_list(["epal6:x"], 299, 0, band)
    neg = [j for j in jobs if j[1] in E.NEG_KINDS]
    counted = [j for j in neg if not E.ledge4_inside_band(j[0], j[1], E.scene_draws(*j, band))]
    assert len(counted) == 299
    assert len([j for j in jobs if j[1] in E.REAL_KINDS]) == 299
    assert E.upper_bound(0, len(counted)) <= 0.01


def test_face_record_carries_the_actual_stand():
    draws = E.scene_draws("epal6:x", "pallet", 3, "dev")
    rec = E.judge_face("epal6:x", "pallet", draws, 1, False, "rig")
    assert abs(rec["theta_e_deg"] - 35.0) < 10.0 and 1.2 < rec["range_m"] < 1.5
    assert rec["arrived"] == B.arrival_ok(*S.render_side(E.face_case("epal6:x", "pallet", draws, 1, False,
                                                                        "rig"))[:2], face=1, axis="x")


def test_dimension_tolerance_builds_the_article_off_nominal():
    case = E.face_case("epal6:x", "pallet", E.scene_draws("epal6:x", "pallet", 0, "dev"), 1, False, "rig",
                       (("deck_bottom_m", 0.002),))
    _, target, out = S.render_side(case)
    nominal = S.render_side(replace(case, dims_delta_m=()))
    assert target.spec == nominal[1].spec                       # the judge keeps the nominal spec
    assert not np.array_equal(out.scene.depth_m, nominal[2].scene.depth_m, equal_nan=True)
    with pytest.raises(SystemExit):
        E.main(["run", "--band", "eval", "--n", "1", "--records", "0", "--out", "/dev/null",
                "--dims", "deck_bottom_m=0.002"])
    with pytest.raises(SystemExit):
        E._dims("block_widths_m=0.002")


def _planned_rows(band="dev"):
    """Synthetic rows for the full default plan (299 / 32): negatives unobserved, real present, occluder and stacked
    unobserved."""
    rows = []
    for _, k, i in E.job_list(["epal6:x"], 299, 32, band):
        final = "unobserved" if k in E.NEG_KINDS + ("pallet+occluder", "pallet+stacked") else "present"
        rows.append({"band": band, "group": "epal6:x", "kind": k, "index": i, "smooth": "off", "K": "rig",
                     "variant": "", "faces": [], "final": final, "opposite_faces": True, "plan_n": [299, 32],
                     "outside_assumption": E.ledge4_inside_band("epal6:x", k, E.scene_draws("epal6:x", k, i, band))})
    return rows


KEY = "dev epal6:x smooth=off K=rig"


def test_summary_refuses_duplicates_and_keeps_bands_apart():
    """Codex task-2 P2: the same 150 scenes read twice gave 0/300 and a pass."""
    rows = _planned_rows()
    assert E.summarise(rows)[KEY]["pass"]
    with pytest.raises(ValueError):
        E.summarise(rows + rows[:1])
    both = E.summarise(rows[:150] + [{**r, "band": "eval"} for r in rows[:150]])
    assert {k.split()[0] for k in both} == {"dev", "eval"}
    assert not any(res["pass"] for res in both.values())


def test_pass_needs_every_planned_scene_and_opposite_faces():
    """Codex task-2 P2: missing occluder / stacked records passed; then one lost occluder row (the failing one) did."""
    rows = _planned_rows()
    assert not E.summarise([r for r in rows if r["kind"] not in ("pallet+occluder", "pallet+stacked")])[KEY]["pass"]
    lost = [r for r in rows if not (r["kind"] == "pallet+occluder" and r["index"] == 0)]
    out = E.summarise(lost)[KEY]
    assert not out["complete"] and not out["pass"]
    assert not E.summarise(rows + [{**rows[0], "index": 10_000}])[KEY]["complete"]
    flipped = [{**rows[0], "opposite_faces": False}] + rows[1:]
    assert not E.summarise(flipped)[KEY]["pass"]


def test_stacked_scene_still_shows_opposite_faces():
    """The physical face comes from the true pose, not from the evidence (stacked returns before the face)."""
    rows = E.run_job(("epal6:x", "pallet+stacked", 0, "dev"))
    assert all(r["opposite_faces"] for r in rows)
    assert all(f["seen_face"] == 0 for r in rows for f in r["faces"])


def test_arrival_error_turns_the_camera_mount_with_the_base():
    """Codex task-2 P2: dx -0.098 m with a 5 deg heading puts the camera 1.4514 m from the centre."""
    case = S.SideCase("epal6", "pallet", stand_dx_m=-0.098, heading_err_deg=5.0)
    (cx, cy), _ = S.estimate_pose(case)
    assert math.hypot(cx - S.CAMERA.xyz_m[0], cy) == pytest.approx(1.4513585, abs=1e-6)
    scene, target, _ = S.render_side(case)
    assert not B.arrival_ok(scene, target, face=1, axis="x")


def test_occluder_stays_in_the_other_face_render():
    """Codex task-2 P3: one physical board, in both renders of the scene."""
    draws = E.scene_draws("epal6:x", "pallet+occluder", 0, "dev")
    other = -draws["occluder_face"]
    case = E.face_case("epal6:x", "pallet+occluder", draws, other, False, "rig")
    assert case.occluder and case.occluder_face == draws["occluder_face"]
    _, _, with_board = S.render_side(case)
    _, _, without = S.render_side(replace(case, occluder=False))
    assert not np.array_equal(with_board.scene.depth_m, without.scene.depth_m, equal_nan=True)


def test_dev_records_are_named_and_refused_on_eval():
    assert E.variant_name((), {"noise": "off"}) == "noise=off"
    draws = E.scene_draws("t11_06:y", "pallet", 0, "dev")
    case = E.face_case("t11_06:y", "pallet", draws, 1, False, "rig", (), {"normal_err_m": 0.06, "noise": "off"})
    assert abs(case.lateral_err_m) == 0.06 and case.noise_seed is None
    for flag in (["--noise-off"], ["--normal-err-m", "0.06"]):
        with pytest.raises(SystemExit):
            E.main(["run", "--band", "eval", "--n", "1", "--records", "0", "--out", "/dev/null", *flag])


def test_stand_record_is_the_true_one():
    """Codex task-2 P3: the candidate error must not leak into the reached-stand record."""
    draws = {**E.scene_draws("t11_06:y", "pallet", 0, "dev"), "along": 0.1, "lateral": 0.03, "yaw": 5.0}
    draws["face1"] = {**draws["face1"], "dx": 0.0, "dy": 0.0, "heading": 0.0}
    rec = E.judge_face("t11_06:y", "pallet", draws, 1, False, "rig")
    # the stand is planned from the estimate: the arrival check sees the nominal stand, the truth is off by the error
    assert rec["range_est_m"] == pytest.approx(1.35, abs=1e-9)
    assert rec["theta_e_est_deg"] == pytest.approx(0.0, abs=1e-6)
    assert rec["range_m"] == pytest.approx(1.3836, abs=1e-3) and rec["theta_e_deg"] > 0.5   # along and yaw partly cancel


def test_failed_arrival_is_captured_again_from_a_fresh_stand():
    draws = E.scene_draws("epal6:x", "pallet", 0, "dev")
    bad = {**draws, "face1": {**draws["face1"], "dx": -0.1, "dy": 0.0, "heading": 5.0}}
    rec = E.judge_face("epal6:x", "pallet", bad, 1, False, "rig")
    assert 2 <= rec["attempts"] <= E.CAPTURES
    assert E.retry_stand(bad, 1, 1) == E.retry_stand(bad, 1, 1) != E.retry_stand(bad, 1, 2)
    assert E.retry_stand(bad, 1, 1)["tilt"] == bad["face1"]["tilt"]       # the calibration error stays


@pytest.mark.parametrize("group,index,face", [("epal6:x", 13, 1), ("epal6:x", 47, -1), ("t11_06:x", 13, 1)])
def test_coplanar_neighbour_joined_to_the_end_row(group, index, face):
    """Task 2 dev 2: the neighbour's first row ran into the target's end row (the 50 mm between them hidden)."""
    draws = E.scene_draws(group, "pallet+nb", index, "dev")
    assert E.judge_face(group, "pallet+nb", draws, face, False, "rig")["verdict"] == "present"


def test_empty_plans_summarise_without_passing():
    """Codex task-2 P3: records 0 raised KeyError, n 0 compared None."""
    no_records = [r for r in _planned_rows() if r["kind"] not in E.RECORD_KINDS]
    no_records = [{**r, "plan_n": [299, 0]} for r in no_records]
    assert not E.summarise(no_records)[KEY]["pass"]
    only_records = [{**r, "plan_n": [0, 32]} for r in _planned_rows() if r["kind"] in E.RECORD_KINDS]
    assert not E.summarise(only_records)[KEY]["pass"]


def test_capture_stops_at_the_first_decided_verdict():
    """The retake is for unobserved frames only: an absent face is not looked at again until it reads present."""
    for kind, index in (("grounded", 0), ("pallet+occluder", 0)):
        draws = E.scene_draws("epal6:x", kind, index, "dev")
        for face in E.FACES:
            rec = E.judge_face("epal6:x", kind, draws, face, False, "rig")
            if rec["verdict"] != "unobserved":
                again = E._judge_once("epal6:x", kind, {**draws, f"face{face}": E.retry_stand(draws, face,
                                                                                             rec["attempts"] - 1)},
                                      face, False, "rig")
                assert again["verdict"] == rec["verdict"]
            else:
                assert rec["attempts"] == E.CAPTURES


def test_board_in_front_of_a_real_face_is_never_absent():
    """Task 2 dev 5: head-on, the refit locked onto an occluder 80 mm in front (inside the 50 mm window after the
    candidate error) and read it as a closed face -> absent on a real pallet (closed faces are unobserved now)."""
    draws = E.scene_draws("t11_06:y", "pallet+occluder", 2, "dev")
    for attempt in range(E.CAPTURES):
        rec = E._judge_once("t11_06:y", "pallet+occluder", {**draws, "face1": E.retry_stand(draws, 1, attempt)}, 1,
                            True, "d435i")
        assert rec["verdict"] != "absent"


@pytest.mark.parametrize("group", ["epal6:x", "t11_06:x", "t11_06:y"])
def test_shelf_reads_a_closed_face(group):
    draws = E.scene_draws(group, "shelf", 8, "dev")
    recs = [E._judge_once(group, "shelf", draws, f, False, "rig") for f in E.FACES]
    assert {(r["verdict"], r["evidence_reason"]) for r in recs} == {("unobserved", "closed_face")}


# -- the v2 error model (plan task 5: Isaac side stands put the target 4-9 cm off along the face normal) -----------

def test_the_v1_bands_draw_exactly_as_before_and_v2_bands_are_their_own():
    assert E.BANDS["dev"] == 300_000 and E.BANDS["eval"] == 700_000  # the opened eval stays what it was
    assert E.MODEL["dev"] == E.MODEL["eval"] == "v1" and E.MODEL["dev2"] == E.MODEL["eval2"] == "v2"
    assert len({E.BANDS[b] for b in E.BANDS}) == len(E.BANDS)
    v1 = E.scene_draws("epal6:x", "pallet", 3, "dev")
    assert abs(v1["lateral"]) <= 0.03 and v1["model"] == "v1"
    lat = [E.scene_draws("epal6:x", "pallet", i, "dev2")["lateral"] for i in range(200)]
    assert max(map(abs, lat)) <= 0.10 and max(map(abs, lat)) > 0.08  # the normal error reaches the Isaac range


def test_v2_adds_a_per_capture_pose_error_and_v1_does_not():
    d1 = E.scene_draws("t11_06:y", "pallet", 5, "dev")
    assert "loc" not in E.retry_stand(d1, 1, 0) and "loc" not in E.retry_stand(d1, 1, 2)
    d2 = E.scene_draws("t11_06:y", "pallet", 5, "dev2")
    a, b = E.retry_stand(d2, 1, 0)["loc"], E.retry_stand(d2, 1, 1)["loc"]
    assert a != b and all(abs(v) <= 0.02 for v in (*a[:2], *b[:2])) and abs(a[2]) <= 1.0
    case = E.face_case("t11_06:y", "pallet", {**d2, "face1": E.retry_stand(d2, 1, 0)}, 1, False, "rig")
    assert case.lateral_err_m == pytest.approx(d2["lateral"] + a[1])
    assert case.along_err_m == pytest.approx(d2["along"] + a[0]) and case.yaw_err_deg == pytest.approx(d2["yaw"] + a[2])


def test_rows_name_their_model_seed_base_and_judge():
    rows = E.run_job(("epal6:x", "pallet", 0, "dev2"))
    assert {(r["model"], r["seed_base"]) for r in rows} == {("v2", 400_000)}
    assert len({r["judge_sha256"] for r in rows}) == 1 and len(rows[0]["judge_sha256"]) == 64


def test_dev_records_are_refused_on_both_eval_bands():
    for band in ("eval", "eval2"):
        with pytest.raises(SystemExit):
            E.main(["run", "--band", band, "--n", "1", "--records", "0", "--out", "/dev/null", "--noise-off"])


def test_a_band_mixing_judges_models_or_seed_bases_is_refused():
    # Codex anchor round 1 P2 2: two judge hashes, or v1 / 300000 labels in dev2, passed the summary
    rows = [{**r, "band": "dev2", "model": "v2", "seed_base": 400_000, "judge_sha256": "a" * 64,
             "tools_sha256": "b" * 64} for r in _planned_rows("dev2")]
    assert E.summarise(rows)["dev2 epal6:x smooth=off K=rig"]["complete"]
    for field, value, msg in (("judge_sha256", "c" * 64, "judge_sha256"), ("tools_sha256", "d" * 64, "tools_sha256"),
                              ("model", "v1", "model"), ("seed_base", 300_000, "seed_base")):
        bad = [dict(r) for r in rows]
        bad[0][field] = value
        with pytest.raises(ValueError, match=msg):
            E.summarise(bad)


def test_a_v2_band_always_needs_its_provenance():
    # Codex anchor round 2 P2: dropping "model" from every dev2 row skipped the check
    rows = [{**r, "band": "dev2", "model": "v2", "seed_base": 400_000, "judge_sha256": "a" * 64,
             "tools_sha256": "b" * 64} for r in _planned_rows("dev2")]
    for drop in (("model",), ("model", "seed_base", "judge_sha256", "tools_sha256")):
        bad = [{k: v for k, v in r.items() if k not in drop} for r in rows]
        with pytest.raises(ValueError):
            E.summarise(bad)
    E.summarise(_planned_rows("dev"))  # v1 rows from before the fields still summarise


def test_v1_rows_keeping_any_provenance_field_are_checked():
    # Codex anchor round 3 P2 2: v1 rows with mixed judges passed once "model" alone was dropped
    rows = [{**r, "judge_sha256": "a" * 64, "tools_sha256": "b" * 64, "seed_base": 300_000} for r in _planned_rows("dev")]
    rows[0]["judge_sha256"] = "c" * 64
    with pytest.raises(ValueError):
        E.summarise(rows)
