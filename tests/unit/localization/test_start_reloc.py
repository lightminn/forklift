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


# --- Codex B-P2a review ----------------------------------------------------------------------------------------------


def test_a_result_before_start_is_a_programming_error_and_start_is_idempotent():
    clock = Clock()
    r = R.StartRelocalizer(R.RelocConfig(e_min=0.8, r_max=0.95, r_struct=0.8), clock)
    with pytest.raises(RuntimeError, match="start"):
        r.record(_match(_cand((3.0, 1.0, 0.5))), G.Decision(True, "confirmed", 0.0))
    r.start()
    first = r.deadline_at
    clock.t += 5.0
    r.start()  # a second call before any result does not extend the deadline
    assert r.deadline_at == first


def test_a_choice_bumps_the_generation_so_old_picks_are_stale():
    r = _reloc(attempts=1)
    a, b = _cand((3.0, 1.0, 0.5)), _cand((9.0, 4.0, 2.0))
    r.record(_match(a, b), G.Decision(False, "competitor", 0.99))
    gen = r.generation
    assert r.pick(gen, 0) == (True, "")
    assert r.estimate.generation == gen and r.generation == gen + 1
    assert r.pick(gen, 0) == (True, "duplicate")


def test_pickable_candidates_survive_across_attempts_whatever_the_raw_top():
    # Codex: attempt 1 only a moved 0.99 top; attempt 2 a moved 0.98 top and a good 0.97 -- the good one must stay
    r = _reloc(attempts=2)
    moved1 = G.Candidate((1.0, 1.0, 0.0), 0.99, 0.97, True, (1.5, 1.0, 0.0), 0.99)
    moved2 = G.Candidate((4.0, 1.0, 0.0), 0.98, 0.97, True, (4.5, 1.0, 0.0), 0.98)
    good = _cand((9.0, 4.0, 2.0), score=0.97)
    refused = G.Decision(False, "refine_moved", 0.0)
    assert r.record(_match(moved1), refused) == "rescan"
    assert r.record(_match(moved2, good), refused) == "awaiting_pick"
    assert [c.pose for c in r.shown] == [good.pose]


def test_the_same_place_from_two_attempts_is_shown_once_with_its_better_score():
    r = _reloc(attempts=2)
    a1, a2 = _cand((3.0, 1.0, 0.5), score=0.90), _cand((3.05, 1.0, 0.52), score=0.93)
    b = _cand((9.0, 4.0, 2.0), score=0.91)
    refused = G.Decision(False, "competitor", 0.99)
    r.record(_match(a1, b), refused)
    r.record(_match(a2), refused)
    assert [c.pose for c in r.shown] == [a2.pose, b.pose]


def test_the_shown_candidates_are_suppressed_over_the_whole_pool():
    # Codex B-P2a re-review: a better x=0.25 between pooled x=0 and x=0.5 must suppress both, freeing a slot for x=4
    r = _reloc(attempts=2)
    first = [_cand((x, 0.0, 0.0), score=s) for x, s in ((0.0, 0.90), (0.5, 0.89), (10.0, 0.88), (2.0, 0.87),
                                                       (4.0, 0.86))]
    refused = G.Decision(False, "competitor", 0.99)
    r.record(_match(*first), refused)
    r.record(_match(_cand((0.25, 0.0, 0.0), score=0.95)), refused)
    assert [c.pose[0] for c in r.shown] == [0.25, 10.0, 2.0, 4.0]


def test_a_stop_interrupts_an_attempt_and_leaves_a_retry():
    # B-P2c (Codex design P1): a STOP does not end the session; the attempt's result is dropped, a retry may follow
    r = _reloc()
    assert r.interrupt() is True and r.state == "stopped" and r.generation == 1 and r.shown == ()
    late = _cand((3.0, 1.0, 0.5))
    assert r.record(_match(late), G.Decision(True, "confirmed", 0.0)) == "stopped"  # the interrupted result goes
    assert r.estimate is None
    assert r.retry(0) == (False, "reloc_stale_pick") and r.retry(1) == (True, "")
    assert r.state == "relocalizing" and r.generation == 2 and r.deadline_at is None


def test_a_stop_while_awaiting_a_pick_clears_the_candidates():
    r = _reloc(attempts=1)
    r.record(_match(_cand((3.0, 1.0, 0.5)), _cand((9.0, 4.0, 2.0))), G.Decision(False, "competitor", 0.98))
    assert r.state == "awaiting_pick" and r.interrupt() is True
    assert r.state == "stopped" and r.shown == () and r.pick(1, 0) == (False, "reloc_not_awaiting")


def test_a_stop_after_the_choice_or_the_end_changes_nothing():
    r = _reloc()
    r.record(_match(_cand((3.0, 1.0, 0.5))), G.Decision(True, "confirmed", 0.0))
    assert r.interrupt() is False and r.state == "chosen" and r.estimate is not None  # the estimate stands
    r2 = _reloc()
    r2.cancel()
    assert r2.interrupt() is False and r2.state == "cancelled"


@pytest.mark.parametrize("generation, index", [(True, 0), (1, True), (1.0, 0), (1, 0.0), ("1", 0), (1, -1), (1, 2)])
def test_a_pick_takes_plain_integers_in_range_only(generation, index):
    r = _reloc(attempts=1)
    r.record(_match(_cand((3.0, 1.0, 0.5)), _cand((9.0, 4.0, 2.0))), G.Decision(False, "competitor", 0.98))
    ok, why = r.pick(generation, index)
    assert not ok and why in ("reloc_pick_refused", "reloc_stale_pick") and r.state == "awaiting_pick"


def test_no_pick_or_retry_after_the_operators_time():
    # Codex B-P2c impl review P2: the operator's deadline holds in the handler, not only in the phase's loop
    clock = Clock()
    r = _reloc(clock, attempts=1)
    r.record(_match(_cand((3.0, 1.0, 0.5)), _cand((9.0, 4.0, 2.0))), G.Decision(False, "competitor", 0.98))
    r.expires_at = clock.t + 10.0
    clock.t += 11.0
    assert r.pick(1, 0) == (False, "reloc_operator_timeout") and r.state == "awaiting_pick"
    assert r.retry(1) == (False, "reloc_operator_timeout")


def test_the_matcher_does_not_confirm_after_the_operators_time():
    clock = Clock()
    r = _reloc(clock)
    r.expires_at = clock.t + 10.0
    clock.t += 11.0  # inside the 30 s session deadline, past the operator's whole time
    assert r.record(_match(_cand((3.0, 1.0, 0.5))), G.Decision(True, "confirmed", 0.0)) != "chosen"
    assert r.estimate is None


def test_a_retry_takes_a_plain_integer_generation():
    r = _reloc(attempts=1)
    r.record(_match(_cand((3.0, 1.0, 0.5))), G.Decision(False, "low_fit", 0.9))
    assert r.retry(True) == (False, "reloc_stale_pick") and r.state != "relocalizing"


# --- F1: the operator's placed pose (docs/plans/2026-10-31-operator-placed-start.md 1.2) ---------------------------------
def _asking(**kw):
    r = _reloc(**kw)
    a, b = _cand((3.0, 1.0, 0.5), score=0.9), _cand((9.0, 4.0, 2.0), score=0.88)
    for _ in range(3):
        r.record(_match(a, b), G.Decision(False, "competitor", 0.98))
    assert r.state == "awaiting_pick"
    return r


PLACED = (5.0, 2.0, 0.3)


def test_a_placement_previews_and_the_preview_is_confirmed_as_the_operators_placed_estimate():
    r = _asking()
    gen = r.generation
    assert r.place(gen, PLACED) == (True, "") and r.state == "placing" and r.places == 1
    assert r.placed_result(True, (5.1, 2.0, 0.31)) == "place_preview"
    assert r.preview_id == 1 and r.preview == (5.1, 2.0, 0.31)
    assert r.confirm_place(gen, 1) == (True, "") and r.state == "chosen"
    assert r.estimate.source == "operator_placed" and r.estimate.base == (5.1, 2.0, 0.31)
    assert r.confirm_place(gen, 1) == (True, "duplicate")  # the same request twice: once


def test_a_refused_placement_returns_to_the_candidate_stage_and_its_picks_stay_valid():
    r = _asking()
    gen = r.generation
    r.place(gen, PLACED)
    assert r.placed_result(False, None, "ambiguous") == "awaiting_pick" and r.place_reason == "ambiguous"
    assert r.pick(gen, 0) == (True, "")


def test_a_new_placement_from_the_preview_invalidates_it_and_a_refusal_never_returns_to_it():
    r = _asking()
    gen = r.generation
    r.place(gen, PLACED)
    r.placed_result(True, (5.1, 2.0, 0.31))
    assert r.place(gen, (5.5, 2.0, 0.3)) == (True, "") and r.preview is None
    assert r.confirm_place(gen, 1) == (False, "reloc_not_awaiting")  # placing: no preview to confirm
    assert r.placed_result(False, None, "edge") == "awaiting_pick"  # Codex F1 plan review P3: not the old preview
    assert r.confirm_place(gen, 1) == (False, "reloc_not_awaiting")


def test_the_placement_cap_blocks_new_placements_but_not_the_last_preview():
    r = _asking()
    gen = r.generation
    for _ in range(2):
        r.place(gen, PLACED)
        r.placed_result(False, None, "low_fit")
    assert r.place(gen, PLACED) == (True, "") and r.places == 3
    r.placed_result(True, (5.0, 2.0, 0.3))
    assert r.place(gen, PLACED) == (False, "reloc_place_limit")
    assert r.state == "place_preview" and r.confirm_place(gen, 3) == (True, "")


def test_a_global_retry_drops_the_preview_and_keeps_the_count():
    r = _asking()
    gen = r.generation
    r.place(gen, PLACED)
    r.placed_result(True, (5.1, 2.0, 0.31))
    assert r.retry(gen) == (True, "") and r.state == "relocalizing" and r.preview is None and r.places == 1


def test_a_stop_while_placing_or_previewing_drops_it_and_a_late_result_is_ignored():
    r = _asking()
    gen = r.generation
    r.place(gen, PLACED)
    assert r.interrupt() and r.state == "stopped"
    assert r.placed_result(True, (5.1, 2.0, 0.31)) == "stopped" and r.preview is None
    r2 = _asking()
    r2.place(r2.generation, PLACED)
    r2.placed_result(True, (5.1, 2.0, 0.31))
    assert r2.interrupt() and r2.state == "stopped" and r2.preview is None


@pytest.mark.parametrize("pose", [(1.0, 2.0), (1.0, float("nan"), 0.0), ("1", 2.0, 0.0), (1.0, 2.0, True)])
def test_a_placed_pose_is_three_finite_numbers(pose):
    r = _asking()
    assert r.place(r.generation, pose) == (False, "reloc_place_invalid") and r.state == "awaiting_pick"


def test_placements_need_the_current_generation_the_candidate_stage_and_the_operators_time():
    clock = Clock()
    r = _asking(clock=clock)
    assert r.place(r.generation - 1, PLACED) == (False, "reloc_stale_pick")
    r.expires_at = clock.t - 1
    assert r.place(r.generation, PLACED) == (False, "reloc_operator_timeout")
    r2 = _reloc()
    assert r2.place(r2.generation, PLACED) == (False, "reloc_not_awaiting")  # still relocalizing


def test_a_placement_outside_the_map_is_refused_without_using_a_placement():
    # Codex F1 implementation review P2 6: the map's extent, set by the phase from the survey grid
    r = _asking()
    r.place_bounds = (0.0, 0.0, 6.0, 4.0)
    assert r.place(r.generation, (-0.1, 2.0, 1.57)) == (False, "reloc_place_invalid") and r.places == 0
    assert r.place(r.generation, (0.6, 2.0, 1.57)) == (True, "")
