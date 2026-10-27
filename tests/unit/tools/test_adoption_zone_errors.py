import hashlib
import json
import math

import pytest

from tools import adoption_zone_errors as E


def obs(x_front, lateral=0.0, yaw=0.0, status="valid"):
    return {"status": status, "insertion_yaw_rad": yaw,
            "left": {"center_m": [x_front, lateral + 0.18, 0.06]}, "right": {"center_m": [x_front, lateral - 0.18, 0.06]}}


def attempt(rear, observation):
    return {"capture_control": {"control_rear": list(rear)}, "pocket_observation": observation}


def state(attempts, *, estimate=None, second=None, near_attempt=None, handoff=None):
    near = {"status": "done", "far_attempt": 1}
    if near_attempt is not None:
        near["near_attempt"] = near_attempt
    if second is not None:
        near["second"] = second
    s = {"observation_attempts": attempts, "near_capture": near, "perception": {}}
    if estimate is not None:
        s["perception"]["perception_pickup_estimate_m"] = {"x_m": estimate[0], "y_m": estimate[1], "yaw_rad": 0.0}
    if handoff is not None:
        s["handoff"] = {"pallet_position_m": list(handoff)}
    return s


CONST = {"initial_pallet_m": [3.0, 0.0, 0.0], "geometry": {"pallet_depth_m": 0.6}, "rear_axle_offset_m": 0.34}


def record(attempts, **kw):
    return {**CONST, **state(attempts, **kw)}


def test_the_site_is_the_front_midpoint_half_a_depth_in_placed_with_the_capture_base():
    site = E.site_of(attempt((0.0, 0.0, 0.0), obs(2.36)), offset_m=0.34, depth_m=0.6)
    assert site == pytest.approx((3.0, 0.0))
    turned = E.site_of(attempt((1.0, 1.0, math.pi / 2), obs(2.36)), offset_m=0.34, depth_m=0.6)
    assert turned == pytest.approx((1.0, 4.0))


def test_errors_are_per_stage_and_per_axis_and_the_record_is_verified_against_its_estimate():
    far = attempt((0.0, 0.0, 0.0), obs(2.40, lateral=0.05))     # site (3.04, 0.05)
    near = attempt((1.0, 0.0, 0.0), obs(1.38, lateral=-0.02))   # site (3.02, -0.02)
    rows, problem = E.record_errors(record([far, near], estimate=(3.02, -0.02), near_attempt=2), CONST)
    assert problem is None
    assert [(s, round(dx, 6), round(dy, 6)) for s, dx, dy in rows] == [("far", 0.04, 0.05), ("near", 0.02, -0.02)]


def test_the_handoff_truth_is_the_missions_own_pallet():
    # a live mission's pallet is not the session's first (initial_pallet_m); handoff holds the truth of its own
    s = state([attempt((0.0, 0.0, 0.0), obs(2.36))], estimate=(3.0, 0.0), handoff=(3.0, 0.25, 0.1))
    rows, problem = E.record_errors(s, CONST)
    assert problem is None and rows == [("far", pytest.approx(0.0), pytest.approx(-0.25))]


def test_a_record_whose_estimate_no_attempt_reproduces_is_undetermined():
    rows, problem = E.record_errors(record([attempt((0.0, 0.0, 0.0), obs(2.36))], estimate=(3.5, 0.0)), CONST)
    assert rows == [] and "reproduce" in problem


def test_a_valid_attempt_without_its_capture_pose_is_undetermined_not_skipped():
    no_pose = {"pocket_observation": obs(1.36)}
    rows, problem = E.record_errors(record([attempt((0.0, 0.0, 0.0), obs(2.36)), no_pose], estimate=(3.0, 0.0)),
                                    CONST)
    assert rows == [] and "capture_control" in problem


def test_an_adopted_second_capture_must_reproduce_its_target():
    first = attempt((0.0, 0.0, 0.0), obs(2.36))
    second = attempt((1.0, 0.0, 0.0), obs(1.36, lateral=0.03))  # site (3.0, 0.03)
    ok = {"capture_attempt": 2, "status": "adopted", "target": [3.0, 0.03, 0.0]}
    rows, problem = E.record_errors(record([first, second], estimate=(3.0, 0.0), second=ok), CONST)
    assert problem is None and [s for s, _, _ in rows] == ["far", "second_near"]
    bad = {**ok, "target": [3.0, 0.2, 0.0]}
    assert "second" in E.record_errors(record([first, second], estimate=(3.0, 0.0), second=bad), CONST)[1]


def test_invalid_observations_are_not_estimates():
    rows, _ = E.record_errors(record([attempt((0.0, 0.0, 0.0), obs(2.36)),
                                      attempt((0.5, 0.0, 0.0), obs(1.0, status="invalid"))], estimate=(3.0, 0.0)),
                              CONST)
    assert [s for s, _, _ in rows] == ["far"]


def test_the_summary_takes_the_largest_per_axis_error_and_rounds_the_margin_up():
    rows = [("far", 0.04, -0.21), ("near", 0.18, 0.02), ("near", -0.03, 0.01)]
    s = E.summarize(rows)
    assert s["stages"]["far"]["max_abs_m"] == pytest.approx(0.21)
    assert s["stages"]["near"]["n"] == 2 and s["stages"]["near"]["max_abs_m"] == pytest.approx(0.18)
    assert s["overall_max_abs_m"] == pytest.approx(0.21) and s["margin_m"] == pytest.approx(0.25)
    assert E.summarize([("far", 0.2, 0.0)])["margin_m"] == pytest.approx(0.20)  # exactly on a step stays


def write_run(root, name, result, missions=()):
    run = root / name / "run"
    run.mkdir(parents=True)
    (run / "result.json").write_text(json.dumps(result))
    for i, m in enumerate(missions, start=1):
        (run / "missions" / f"m{i}").mkdir(parents=True)
        (run / "missions" / f"m{i}" / "records.json").write_text(json.dumps({"state": m}))


def test_cli_reads_mission_records_with_the_runs_constants_and_writes_a_manifest(tmp_path):
    good = record([attempt((0.0, 0.0, 0.0), obs(2.36))], estimate=(3.0, 0.0))
    bad = record([attempt((0.0, 0.0, 0.0), obs(2.36))], estimate=(9.0, 0.0))
    write_run(tmp_path, "a", good)
    write_run(tmp_path, "b", bad)
    m2 = state([attempt((0.0, 0.0, 0.0), obs(2.36))], estimate=(3.0, 0.0), handoff=(3.0, 0.1, 0.0))
    # a live run: its result.json repeats the last mission's state; only the mission records count
    write_run(tmp_path, "live", {**CONST, **m2, "session_switches": []}, missions=[m2])
    out, manifest = tmp_path / "e.json", tmp_path / "manifest.json"
    assert E.main([str(tmp_path), "--out", str(out), "--manifest-out", str(manifest)]) == 0
    s = json.loads(out.read_text())
    assert s["records"] == 3 and s["undetermined"] == 1 and s["stages"]["far"]["n"] == 2
    assert s["stages"]["far"]["max_abs_m"] == pytest.approx(0.1)  # the live mission's own truth (handoff)
    listed = json.loads(manifest.read_text())
    assert len(listed) == 3 and all(len(e["sha256"]) == 64 for e in listed)
    # the same sample again from the manifest; a changed file is refused
    assert E.main(["--manifest-in", str(manifest), "--out", str(out)]) == 0
    (tmp_path / "a" / "run" / "result.json").write_text(json.dumps(good) + " ")
    assert E.main(["--manifest-in", str(manifest), "--out", str(out)]) == 2


def test_a_live_runs_last_mission_without_its_own_records_is_read_from_the_result(tmp_path):
    # the session ended in its last mission before that mission's records were written: result.json holds it
    m1 = state([attempt((0.0, 0.0, 0.0), obs(2.36))], estimate=(3.0, 0.0), handoff=(3.0, 0.0, 0.0))
    last = state([attempt((0.0, 0.0, 0.0), obs(2.36, lateral=0.1))], estimate=(3.0, 0.1), handoff=(3.0, -0.2, 0.0))
    write_run(tmp_path, "live", {**CONST, **last, "session_switches": []}, missions=[m1])
    out = tmp_path / "e.json"
    assert E.main([str(tmp_path), "--out", str(out)]) == 0
    s = json.loads(out.read_text())
    assert s["records"] == 2 and s["stages"]["far"]["max_abs_m"] == pytest.approx(0.3)


def test_a_manifest_hash_is_the_files_sha256(tmp_path):
    p = tmp_path / "x.json"
    p.write_text("{}")
    assert E.sha256(p) == hashlib.sha256(b"{}").hexdigest()
