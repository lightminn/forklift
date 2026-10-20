"""The bottom-deck recompute (plan 2026-10-19 task 4): a run's side frames judged again match its record."""

import json
from dataclasses import asdict

import numpy as np

from forklift_core.perception import bottom_deck_evidence as E
from tools import bottom_deck_scenes as S
from tools import recompute_bottom_deck as R


MAP_TARGET = (3.0, 0.0, 0.0)  # the judged target in the map (centre, model yaw)


def _save(run, number, case):
    """A side frame as MissionCore.bottom_deck_capture saves it."""
    scene, target, _ = S.render_side(case)
    ev = E.evaluate_side(scene, target, face=case.face)
    k = scene.intrinsics
    name = f"bottom_deck/side_{number:02d}.npz"
    (run / "bottom_deck").mkdir(parents=True, exist_ok=True)
    np.savez_compressed(run / name, depth_m=np.asarray(scene.depth_m, float),
                        intrinsics=np.asarray([k.width, k.height, k.fx, k.fy, k.cx, k.cy], float),
                        intrinsics_frame=k.frame_id,
                        mount_translation=np.asarray(scene.base_from_optical.translation_m, float),
                        mount_rotation=np.asarray(scene.base_from_optical.rotation, float),
                        target_base=np.asarray([*target.centre_xy, target.model_yaw_rad, target.support_z_m]),
                        target_map=np.asarray(MAP_TARGET),
                        face=case.face, axis="x", shape=case.shape, params=json.dumps(asdict(E.BottomDeckParams())),
                        verdict=ev.verdict, reason=ev.reason, stamp_ns=int(scene.stamp_ns))
    return {"face": case.face, "verdict": ev.verdict, "reason": ev.reason, "npz": name}


def _run(tmp_path, kind):
    attempts = [_save(tmp_path, 1, S.SideCase("epal6", kind, face=1)),
                _save(tmp_path, 2, S.SideCase("epal6", kind, face=-1))]
    faces = [{"face": a["face"], "verdict": a["verdict"], "reason": a["reason"], "captures": 1} for a in attempts]
    verdict = R.policy(faces)
    from forklift_core.perception.bottom_deck_permit import params_digest

    record = {"verdict": verdict, "faces": faces, "shape": "epal6", "axis": "x", "centre_xy": list(MAP_TARGET[:2]),
              "model_yaw_rad": MAP_TARGET[2],
              "params_sha256": params_digest(E.BottomDeckParams())}
    (tmp_path / "result.json").write_text(json.dumps({"bottom_deck": {
        "attempts": attempts, "faces": faces, "record": record}}))
    return verdict


def test_a_pallet_run_and_a_grounded_run_recompute_exactly(tmp_path):
    for kind, want in (("pallet", "present"), ("grounded", "absent")):
        run = tmp_path / kind
        run.mkdir()
        assert _run(run, kind) == want
        out = R.recompute(run)
        assert out["match"] and out["policy"] == want and R.main([str(run)]) == 0


def test_a_record_that_differs_from_its_frames_is_reported(tmp_path):
    _run(tmp_path, "pallet")
    rec = json.loads((tmp_path / "result.json").read_text())
    rec["bottom_deck"]["attempts"][1]["verdict"] = "absent"
    (tmp_path / "result.json").write_text(json.dumps(rec))
    out = R.recompute(tmp_path)
    assert not out["match"] and not out["captures"][1]["match"] and R.main([str(tmp_path)]) == 1


def test_a_present_record_over_grounded_frames_is_caught_even_when_its_faces_agree_with_it(tmp_path):
    # Codex implementation round 1 P2: the faces and the record rewritten together must not pass
    _run(tmp_path, "grounded")
    rec = json.loads((tmp_path / "result.json").read_text())
    for f in rec["bottom_deck"]["faces"]:
        f["verdict"] = "present"
    rec["bottom_deck"]["record"]["verdict"] = "present"
    rec["bottom_deck"]["record"]["faces"] = rec["bottom_deck"]["faces"]
    (tmp_path / "result.json").write_text(json.dumps(rec))
    out = R.recompute(tmp_path)
    assert not out["match"] and out["policy"] == "absent"


def test_a_record_with_other_parameters_or_shape_is_caught(tmp_path):
    _run(tmp_path, "pallet")
    rec = json.loads((tmp_path / "result.json").read_text())
    rec["bottom_deck"]["record"]["params_sha256"] = "0" * 64
    rec["bottom_deck"]["record"]["shape"] = "t11_06"
    (tmp_path / "result.json").write_text(json.dumps(rec))
    out = R.recompute(tmp_path)
    assert not out["match"] and any("parameters" in p for p in out["problems"]) and any(
        "shape/axis" in p for p in out["problems"])


def test_a_record_bound_to_another_centre_is_caught(tmp_path):
    # Codex implementation round 2 P2: the insert gate reads the record's centre, so it must be the judged one
    _run(tmp_path, "pallet")
    rec = json.loads((tmp_path / "result.json").read_text())
    rec["bottom_deck"]["record"]["centre_xy"] = [3.3, 0.0]
    (tmp_path / "result.json").write_text(json.dumps(rec))
    out = R.recompute(tmp_path)
    assert not out["match"] and any("judged target" in p for p in out["problems"])


def test_a_release_after_a_capture_carries_the_judged_target_to_the_record(tmp_path):
    _run(tmp_path, "pallet")
    rec = json.loads((tmp_path / "result.json").read_text())
    rec["bottom_deck"]["releases"] = [{"frame_change": [0.2, 0.0, 0.0], "after_attempts": 2}]
    (tmp_path / "result.json").write_text(json.dumps(rec))
    assert not R.recompute(tmp_path)["match"]  # the record still holds the pre-release centre
    rec["bottom_deck"]["record"]["centre_xy"] = [3.2, 0.0]
    (tmp_path / "result.json").write_text(json.dumps(rec))
    assert R.recompute(tmp_path)["match"]


def test_a_decision_without_a_saved_frame_is_not_evidence(tmp_path):
    (tmp_path / "result.json").write_text(json.dumps({"bottom_deck": {
        "attempts": [{"face": 1, "verdict": "present", "reason": "capture_failed:not_still"},
                     {"face": -1, "verdict": "present", "reason": "capture_failed:not_still"}],
        "faces": [{"face": 1, "verdict": "present"}, {"face": -1, "verdict": "present"}],
        "record": {"verdict": "present"}}}))
    out = R.recompute(tmp_path)
    assert not out["match"] and out["policy"] == "present" and len(out["problems"]) >= 2
