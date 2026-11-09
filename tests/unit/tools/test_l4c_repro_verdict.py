"""The cancel-then-other-target repro batch's verdict (docs/plans/2026-11-09-l4c-repro-cancel-other-target.md 2)."""

import itertools
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import l4c_repro_verdict as R  # noqa: E402


@pytest.mark.parametrize("c,b_min", [(0, 5), (1, 7), (2, 9), (3, 10)])
def test_the_registered_mcnemar_thresholds(c, b_min):
    assert R.mcnemar_p(b_min, c) <= 0.05 < R.mcnemar_p(b_min - 1, c)


def test_the_conclusion_rows():
    assert R.conclusion(5, 0, 0, 0, True) == "h1_excess"
    assert R.conclusion(5, 0, 2, 0, True) == "both"
    assert R.conclusion(1, 1, 0, 2, True) == "h0_same_type"
    assert R.conclusion(0, 1, 1, 1, True) == "undecided_or_other"  # one F_p and one F_c are not a reproduction
    assert R.conclusion(0, 0, 0, 0, False) == "not_reproduced"


@pytest.mark.parametrize("seed", range(6))
def test_the_compressed_enumeration_matches_the_brute_force(seed):
    import random

    rng = random.Random(seed)
    for planned in (1, 2, 3):
        for run in range(planned + 1):
            pick = R.OUTCOMES + (None, ("c", "pc"), ("none", "p"))
            pairs = [tuple(rng.choice(pick) for _ in range(2)) for _ in range(run)]
            assert R.rows_possible(pairs, planned) == R.rows_brute(pairs, planned)


def test_the_early_stop_settles_only_when_no_assignment_changes_the_row():
    five = [("p", "none")] * 5
    v = R.batch_verdict(five, 6)
    assert not v["settled"]  # the sixth pair could make c = 1, and then b = 5 is not enough
    assert "h1_excess" in v["rows_possible"]
    all_none = [("none", "none")] * 24
    assert R.batch_verdict(all_none, 24) == {**R.batch_verdict(all_none, 24), "settled": True,
                                             "conclusion": "not_reproduced"}


def _scan(t, blocked, phase="transport"):
    return {"stamp_s": t, "blocked": blocked, "phase": phase}


def test_f_p_on_the_anchor_shape_and_its_edges():
    track = [(130.0, 0.0, -8.4, 0.0), (236.6, 1.95, -8.7, -0.44)]  # 1.47 m from D-2 at the end
    run = [_scan(138.2 + 0.1 * k, "occupied") for k in range(985)]  # to 236.6
    m = {"ended_s": 236.70}
    assert R.f_p(m, run, track) == "yes"
    released = run + [_scan(236.65, None)]
    assert R.f_p(m, released, track) == "no"  # the last scan before the end is not occupied
    short = [_scan(232.0 + 0.1 * k, "occupied") for k in range(46)]
    assert R.f_p(m, short, track) == "no"  # under 5 s
    stopped = [s for s in run if s["stamp_s"] < 236.0]
    assert R.f_p(m, stopped, track) == "undecided"  # the scans stopped 0.7 s before the end
    far = [(130.0, 0.0, -8.4, 0.0), (236.6, -1.0, -8.0, 0.0)]
    assert R.f_p(m, run, far) == "no"  # outside the entry
    gap = run[:400] + run[402:]
    assert R.occupied_run_at_end(gap, 236.7)["length_s"] < 60  # a 0.3 s gap breaks the run
    unknown = run[:500] + [_scan(188.25, "unknown")] + run[501:]
    assert R.occupied_run_at_end(unknown, 236.7)["start_s"] > 188.2  # unknown breaks it


def test_f_c_needs_the_cusp_refusal_at_the_entry():
    rec = {"scene_replans": [{"trigger": "cusp_pose", "outcome": "refused", "mission_end": True,
                              "permit_waits": [{"why": "blocked:occupied", "since_s": 162.717, "until_s": 165.717}],
                              "candidate": {"why": "blocked:occupied"}, "truth_rear": [1.677, -8.660, -0.236]}]}
    assert R.f_c(rec) == "yes"
    late = {"scene_replans": [{**rec["scene_replans"][0], "candidate": {"why": "blocked:unknown"},
                               "permit_waits": [{"why": "blocked:occupied", "since_s": 162.7, "until_s": 163.0},
                                                {"why": "blocked:unknown", "since_s": 163.0, "until_s": 166.0}]}]}
    assert R.f_c(late) == "no"  # the final refusal was for an unknown cell (Codex P2-1)
    quick = {"scene_replans": [{**rec["scene_replans"][0],
                                "permit_waits": [{"why": "blocked:occupied", "since_s": 162.7, "until_s": 163.2}]}]}
    assert R.f_c(quick) == "no"  # refused before its 3 s re-check
    assert R.f_c({}) == "no"  # an absent key: no replan
    far = {"scene_replans": [{**rec["scene_replans"][0], "truth_rear": [-1.0, -2.0, 0.0]}]}
    assert R.f_c(far) == "no"
    other = {"scene_replans": [{**rec["scene_replans"][0], "trigger": "permit_stop"}]}
    assert R.f_c(other) == "no"


def test_the_feasibility_check():
    ok_h1 = {"valid": {"stop": True}, "p5": [{"outcome": "done", "reached_transport": True}]}
    bad_h1 = {"valid": {"stop": False}, "p5": [{"outcome": "done", "reached_transport": True}]}
    h0 = {"valid": {}, "p5": [{"outcome": "failed", "reached_transport": True}]}  # failed after transport: reached
    assert R.feasibility([(ok_h1, h0)] * 4 + [(bad_h1, h0)] * 2)["ok"]
    assert not R.feasibility([(ok_h1, h0)] * 3 + [(bad_h1, h0)] * 3)["ok"]
    none = {"valid": {"stop": True}, "p5": []}
    assert not R.feasibility([(ok_h1, none)] * 6)["ok"]  # 6 of 12 reached transport


def test_a_stopped_observation_is_undecided_even_for_a_short_run():
    m = {"ended_s": 10.0}
    scans = [_scan(0.5 + 0.1 * k, "occupied") for k in range(6)]  # occupied 0.5-1.0, then nothing to 10.0
    assert R.f_p(m, scans, [(0.0, 3.4, -8.4, 0.0)]) == "undecided"
    assert R.f_p(m, [], [(0.0, 3.4, -8.4, 0.0)]) == "undecided"  # no transport scan at all


def test_partly_known_sessions_keep_their_possibilities():
    # Codex P2-3: F_c yes with F_p undecided is {c, pc}, which can change the row when H0 already has an F_p seed
    pairs = [("none", "p"), ("none", ("c", "pc"))] + [("none", "none")] * 22
    rows = R.rows_possible(pairs, 24)
    assert rows == {"h0_same_type", "undecided_or_other"}
    assert not R.batch_verdict(pairs, 24)["settled"]


def test_the_plan_shape_and_the_attempt_choice():
    def pair(k, h1="ok", h0="ok"):
        return {"seed": k, "H1": [{"name": f"h1_s{k}", "base_failure": h1 != "ok"}],
                "H0": [{"name": f"h0_s{k}", "base_failure": h0 != "ok"}]}
    good = {"planned_pairs": 24, "pairs": [pair(k) for k in range(2, 8)]}
    assert R.validate_plan(good) == []
    assert "seeds" in R.validate_plan({**good, "pairs": [pair(3), pair(2)]})
    dup = {**good, "pairs": [pair(2), {**pair(3), "H0": [{"name": "h0_s2", "base_failure": False}]}]}
    assert "duplicate_session" in R.validate_plan(dup)
    assert "planned_pairs" in R.validate_plan({**good, "planned_pairs": 16})
    assert R.chosen_attempt([{"name": "a", "base_failure": True}, {"name": "b", "base_failure": False}]) == "b"
    assert R.chosen_attempt([{"name": "a", "base_failure": True}, {"name": "b", "base_failure": True}]) is None


def test_a_session_must_match_its_pair_and_arm():
    known = "-0.9340,0.9106,0.1706"
    assert R.args_mismatch({"SEED": "4", "SCENARIO": "cycle"}, 4, "H1") == []
    assert R.args_mismatch({"SEED": "4", "SCENARIO": "queue", "START_POSE_KNOWN": known}, 4, "H0") == []
    assert R.args_mismatch({"SEED": "5", "SCENARIO": "cycle"}, 4, "H1") == ["args:seed"]
    assert R.args_mismatch({"SEED": "4", "SCENARIO": "queue"}, 4, "H0") == ["args:arm"]  # no start pose
    # correction 1.5: the relocalizing H0 of chunk 1 is not this arm, nor is a known start at another pose
    assert R.args_mismatch({"SEED": "4", "SCENARIO": "queue", "START_POSE": known}, 4, "H0") == ["args:arm"]
    assert R.args_mismatch({"SEED": "4", "SCENARIO": "queue", "START_POSE_KNOWN": known, "START_POSE": known}, 4,
                           "H0") == ["args:arm"]
    assert R.args_mismatch({"SEED": "4", "SCENARIO": "queue", "START_POSE_KNOWN": "-0.9,0.9,0.17"}, 4,
                           "H0") == ["args:arm"]
    assert R.args_mismatch({"SEED": "4", "SCENARIO": "cycle", "START_POSE_KNOWN": known}, 4, "H1") == ["args:arm"]


def test_h0_is_valid_on_the_known_start_at_the_target_pose():
    rear = (-1.2691, 0.8528, 0.1706)
    known = {"base": [-0.934, 0.9106, 0.1706], "rear": [-1.2690, 0.8530, 0.1706]}
    track = [(0.0, *rear), (50.0, *rear)]
    p5 = [{"started_s": 10.0}]
    v = R.h0_valid({"result": {"start_known": known}}, track, p5)
    assert v["ok"] and v["known_start"] and v["pose_ok"]
    assert v["start_error"][0] == pytest.approx(math.hypot(0.0001, 0.0002), abs=1e-9)
    assert not R.h0_valid({"result": {}}, track, p5)["ok"]  # no known start record
    reloc = {"start_known": known, "start_reloc": {"status": "chosen"}}
    assert not R.h0_valid({"result": reloc}, track, p5)["ok"]  # a relocalization is not this arm
    other = {"start_known": {**known, "base": [-0.9, 0.9106, 0.1706]}}
    assert not R.h0_valid({"result": other}, track, p5)["ok"]  # another placed pose
    moved = [(0.0, *rear), (50.0, rear[0] + 0.03, rear[1], rear[2])]
    v = R.h0_valid({"result": {"start_known": known}}, moved, [{"started_s": 50.0}])
    assert v["known_start"] and v["pose_ok"] is False and not v["ok"]  # the settle missed the target by 3 cm
    # Codex fix A review P3: an incomplete or broken record is not the known start (undecided), never an error
    for bad in ({**known, "base": [-0.934]}, {**known, "rear": [-1.2691]}, {**known, "rear": None},
                {**known, "base": [-0.934, 0.9106, float("nan")]}, {**known, "rear": ["x", 0.0, 0.0]}):
        v = R.h0_valid({"result": {"start_known": bad}}, track, p5)
        assert not v["ok"] and not v["known_start"] and v["start_error"] is None


def test_a_mission_without_its_result_or_folder_is_missing(tmp_path):
    import json

    run = tmp_path / "run"
    for mid, res in (("m1", {"pallet_id": "P-5", "destination_id": "D-2", "outcome": "done"}), ("m2", None)):
        (run / "missions" / mid).mkdir(parents=True)
        (run / "missions" / mid / "records.json").write_text("{}")
        if res is not None:
            (run / "missions" / mid / "result.json").write_text(json.dumps({"result": res}))
    sess = {"run": run, "rows": [{"mission_id": "m1"}, {"mission_id": "m2"}, {"mission_id": "m3"}]}
    p5, missing = R._missions(sess)
    assert [m["mission_id"] for m in p5] == ["m1"]
    assert missing == ["result:m2", "mission_folder:m3"]


def test_f_c_counts_the_whole_candidate_wait_within_its_stretch():
    rec = {"scene_replans": [{"trigger": "cusp_pose", "outcome": "refused", "mission_end": True,
                              "candidate": {"why": "blocked:occupied"}, "truth_rear": [1.677, -8.660, -0.236],
                              "permit_waits": [
                                  {"why": "blocked:unknown", "since_s": 162.7, "until_s": 163.7, "stretch": 1},
                                  {"why": "blocked:occupied", "since_s": 163.7, "until_s": 165.7, "stretch": 1}]}]}
    assert R.f_c(rec) == "yes"  # unknown 1 s then occupied 2 s, refused for occupied (Codex P2-1)
    other = {"scene_replans": [{**rec["scene_replans"][0], "permit_waits": [
        {"why": "blocked:occupied", "since_s": 150.0, "until_s": 152.0, "stretch": 0},
        {"why": "blocked:occupied", "since_s": 164.7, "until_s": 165.7, "stretch": 1}]}]}
    assert R.f_c(other) == "no"  # an earlier stretch does not count toward this candidate's wait
