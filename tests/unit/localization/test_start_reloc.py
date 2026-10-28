"""Start relocalization (docs/plans/2026-10-29-start-relocalization.md 1.1-1.3): the three poses' frames, the
attempt / rescan / operator-pick state machine under one session deadline, candidate generations, and the epoch
files that hand the estimate to the supervisor."""

import json
import math

import pytest

from forklift_core.localization import global_match as G
from forklift_core.localization import start_reloc as R


def _cand(laser, frac=0.97, score=0.9, lattice=None):
    return G.Candidate(tuple(laser), score, frac, True, tuple(lattice or laser), score)


def _match(*cands, status="ok"):
    return G.GlobalMatch(status, tuple(cands))


class Clock:
    def __init__(self):
        self.t = 50.0

    def __call__(self):
        return self.t


def _reloc(clock=None, **kw):
    cfg = R.RelocConfig(e_min=0.8, r_max=0.95, r_struct=0.8, **kw)
    r = R.StartRelocalizer(cfg, clock or Clock())
    r.start()
    return r


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -math.pi / 2])
def test_the_three_frames_round_trip_at_four_headings(yaw):
    base = (2.0, -1.0, yaw)
    laser = R.laser_from_base(base)
    assert laser[0] == pytest.approx(2.0 - 0.12 * math.cos(yaw), abs=1e-12)
    assert laser[1] == pytest.approx(-1.0 - 0.12 * math.sin(yaw), abs=1e-12)
    assert R.base_from_laser(laser) == pytest.approx(base, abs=1e-12)
    rear = R.rear_from_base(base)
    assert rear[:2] == pytest.approx((2.0 - 0.34 * math.cos(yaw), -1.0 - 0.34 * math.sin(yaw)), abs=1e-12)


def test_a_confirmed_first_attempt_chooses_the_base_pose_of_the_top():
    r = _reloc()
    top = _cand((3.0, 1.0, 0.5))
    assert r.record(_match(top), G.Decision(True, "confirmed", 0.0)) == "chosen"
    assert r.state == "chosen" and r.estimate.source == "matcher" and r.estimate.attempts == 1
    assert r.estimate.base == pytest.approx(R.base_from_laser(top.pose))


def test_unconfirmed_attempts_rescan_up_to_the_limit_then_ask_the_operator():
    r = _reloc()
    a, b = _cand((3.0, 1.0, 0.5), score=0.9), _cand((9.0, 4.0, 2.0), score=0.88)
    refused = G.Decision(False, "competitor", 0.98)
    assert r.record(_match(a, b), refused) == "rescan"
    assert r.record(_match(a, b), refused) == "rescan"
    assert r.record(_match(a, b), refused) == "awaiting_pick"
    assert r.attempts == 3 and r.generation == 1 and [c.pose for c in r.shown] == [a.pose, b.pose]


def test_only_pickable_candidates_are_shown_and_none_is_a_failure():
    r = _reloc(attempts=1)
    low = _cand((3.0, 1.0, 0.5), frac=0.5)  # under E_MIN
    moved = _cand((6.0, 1.0, 0.5), lattice=(6.5, 1.0, 0.5))  # refinement walked 0.5 m
    good = _cand((9.0, 4.0, 2.0))
    assert r.record(_match(low, moved, good), G.Decision(False, "low_fit", 0.9)) == "awaiting_pick"
    assert [c.pose for c in r.shown] == [good.pose]
    r2 = _reloc(attempts=1)
    assert r2.record(_match(low, moved), G.Decision(False, "low_fit", 0.9)) == "reloc_failed"
    assert r2.state == "reloc_failed" and r2.shown == ()


def test_at_most_four_candidates_are_shown():
    r = _reloc(attempts=1)
    cands = [_cand((float(i), 0.0, 0.0), score=0.9 - 0.01 * i) for i in range(6)]
    r.record(_match(*cands), G.Decision(False, "periodic", 0.99))
    assert len(r.shown) == 4


def test_a_result_after_the_session_deadline_is_not_confirmed_and_keeps_earlier_candidates():
    clock = Clock()
    r = _reloc(clock)
    a = _cand((3.0, 1.0, 0.5))
    assert r.record(_match(a), G.Decision(False, "competitor", 0.97)) == "rescan"
    clock.t += 31.0  # the next attempt ran past the 30 s session deadline
    assert r.record(_match(status="timeout"), G.Decision(False, "timeout")) == "awaiting_pick"
    assert [c.pose for c in r.shown] == [a.pose]
    r2 = _reloc(clock2 := Clock())
    clock2.t += 31.0
    assert r2.record(_match(a), G.Decision(True, "confirmed", 0.0)) == "awaiting_pick"  # late: never chosen
    assert r2.estimate is None


def test_the_deadline_is_one_for_all_attempts_from_the_first_scan():
    clock = Clock()
    r = _reloc(clock)
    assert r.deadline_at == pytest.approx(clock.t + 30.0)
    first = r.deadline_at
    r.record(_match(_cand((3.0, 1.0, 0.5))), G.Decision(False, "competitor", 0.97))
    assert r.deadline_at == first  # a rescan does not move it


def test_a_pick_needs_the_current_generation_and_a_shown_index():
    r = _reloc(attempts=1)
    a, b = _cand((3.0, 1.0, 0.5)), _cand((9.0, 4.0, 2.0))
    r.record(_match(a, b), G.Decision(False, "competitor", 0.99))
    assert r.pick(r.generation + 1, 0) == (False, "reloc_stale_pick")
    assert r.pick(r.generation, 5) == (False, "reloc_pick_refused")
    assert r.state == "awaiting_pick"
    assert r.pick(r.generation, 1) == (True, "")
    assert r.state == "chosen" and r.estimate.source == "operator"
    assert r.estimate.base == pytest.approx(R.base_from_laser(b.pose))
    assert r.pick(r.estimate.generation, 1) == (True, "duplicate")  # the same request again
    assert r.pick(r.estimate.generation, 0) == (False, "reloc_not_awaiting")


def test_a_retry_restarts_with_a_new_generation_and_a_new_deadline():
    clock = Clock()
    r = _reloc(clock, attempts=1)
    r.record(_match(_cand((3.0, 1.0, 0.5))), G.Decision(False, "competitor", 0.99))
    gen = r.generation
    assert r.retry(gen - 1) == (False, "reloc_stale_pick")
    assert r.retry(gen) == (True, "")
    assert r.state == "relocalizing" and r.attempts == 0 and r.generation == gen + 1 and r.shown == ()
    assert r.pick(gen, 0) == (False, "reloc_not_awaiting")  # the old candidates are gone
    clock.t += 100.0
    r.start()
    assert r.deadline_at == pytest.approx(clock.t + 30.0)


def test_cancel_ends_the_relocalization_without_an_estimate():
    r = _reloc()
    r.cancel()
    assert r.state == "cancelled" and r.estimate is None
    assert r.record(_match(_cand((3.0, 1.0, 0.5))), G.Decision(True, "confirmed", 0.0)) == "cancelled"


def test_the_epoch_files_are_written_atomically_and_cleared(tmp_path):
    est = R.Estimate((1.0, 2.0, 0.3), "matcher", 1, 2)
    R.write_reloc_pose(tmp_path, est, {"attempts": [{"reason": "competitor"}]})
    data = R.read_reloc_pose(tmp_path)
    assert data["base"] == [1.0, 2.0, 0.3] and data["source"] == "matcher" and data["generation"] == 1
    assert data["attempts"] == 2 and data["record"]["attempts"][0]["reason"] == "competitor"
    assert sorted(p.name for p in tmp_path.iterdir()) == [R.RELOC_POSE]  # no temporary file left
    assert R.read_bridge_go(tmp_path) is None
    R.write_bridge_go(tmp_path, {"map_start_pose": [1.0, 2.0, 0.3]})
    assert R.read_bridge_go(tmp_path)["map_start_pose"] == [1.0, 2.0, 0.3]
    R.clear_epoch(tmp_path)
    assert R.read_reloc_pose(tmp_path) is None and R.read_bridge_go(tmp_path) is None


def test_a_half_written_file_reads_as_absent(tmp_path):
    (tmp_path / R.RELOC_POSE).write_text('{"base": [1.0')
    assert R.read_reloc_pose(tmp_path) is None
    json.loads("{}")
