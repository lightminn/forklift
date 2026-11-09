"""The cancel-then-other-target repro batch's verdict (docs/plans/2026-11-09-l4c-repro-cancel-other-target.md 2)."""

import itertools
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
            pairs = [tuple(rng.choice(R.OUTCOMES + (None,)) for _ in range(2)) for _ in range(run)]
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
                              "permit_waits": [{"why": "blocked:occupied", "since_s": 162.7, "until_s": 165.7}],
                              "truth_rear": [1.677, -8.660, -0.236]}]}
    assert R.f_c(rec) == "yes"
    assert R.f_c({}) == "no"  # an absent key: no replan
    far = {"scene_replans": [{**rec["scene_replans"][0], "truth_rear": [-1.0, -2.0, 0.0]}]}
    assert R.f_c(far) == "no"
    other = {"scene_replans": [{**rec["scene_replans"][0], "trigger": "permit_stop"}]}
    assert R.f_c(other) == "no"


def test_the_feasibility_check():
    ok_h1 = {"valid": {"stop": True}, "p5": [{"outcome": "done"}]}
    bad_h1 = {"valid": {"stop": False}, "p5": [{"outcome": "done"}]}
    h0 = {"valid": {}, "p5": [{"outcome": "done"}]}
    assert R.feasibility([(ok_h1, h0)] * 4 + [(bad_h1, h0)] * 2)["ok"]
    assert not R.feasibility([(ok_h1, h0)] * 3 + [(bad_h1, h0)] * 3)["ok"]
    none = {"valid": {"stop": True}, "p5": []}
    assert not R.feasibility([(ok_h1, none)] * 6)["ok"]  # 6 of 12 reached transport
