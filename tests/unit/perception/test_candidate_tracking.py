# tests/unit/perception/test_candidate_tracking.py
"""Far-range candidate tracking and fusion (stage 2B-3 plan sections 1-5)."""
import math
import random
from pathlib import Path
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception.candidate_tracking import (
    BoundRow,
    CandidateTracker,
    Frame,
    TrackBounds,
)
from forklift_core.perception.pallet_model import ObservedPalletShape, face_layout
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from tools import recognition_scenes as rs

S = 1_000_000_000  # ns per second


def _row(lo, hi, *, lat=0.05, along=0.05, yaw=0.0, z_s=0.02, h=0.02, s=0.02, sigma=0.01):
    return BoundRow(lo_m=lo, hi_m=hi, lateral_m=lat, along_m=along, centre_lateral_m=lat, centre_along_m=along,
                    centre_height_m=h, yaw_rad=yaw, support_m=z_s, layout_m=s, sigma_m=sigma)


def _bounds(*, rows=None, window_s=1.2, e=0.0, psi=0.0, rear_axle=(0.0, 0.0), suppress=False, layout=False):
    # suppress=False keeps the 2B-3c..3e tests on the duplicate-track machinery; the default in
    # TrackBounds (2B-3f) suppresses new tracks from suspected-duplicate readings. layout=False likewise
    # keeps in-place layout splits visible (the 2B-3h default hides them)
    rows = rows or (_row(1.0, 2.5), _row(2.5, 4.0))
    return TrackBounds(rows={"known": rows, "parametric": rows}, ages_s=(0.1, 0.5, 1.0, 10.0),
                       e_m=(e, e, e, e), psi_rad=(psi, psi, psi, psi), rear_axle_xy_m=rear_axle,
                       window_s=window_s, max_observations=10, expiry_s=0.5, suppress_duplicates=suppress,
                       hide_layout_splits=layout)


def _odom(x=0.0, y=0.0, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return RigidTransform("base_link", "odom", ((c, -s, 0), (s, c, 0), (0, 0, 1)), (x, y, 0.0))


def _candidate(mid=(3.0, 0.0, 0.06), yaw=0.0, *, spacing=0.4, hypothesis="epal6:x", stamp_ns=0, cid="c0",
               face_offset=0.0, support_z=0.0, **kw):
    """A counted candidate in base_link whose pocket pair has midpoint `mid` and insertion yaw `yaw`."""
    spec = rs.known_spec("epal6") if not hypothesis.startswith("parametric") else None
    shape = (ObservedPalletShape(spec, face_layout(spec, "x"), (("face", "known_shape"), ("depth", "known_shape")))
             if spec is not None else _parametric_shape())
    n = np.array([-math.sin(yaw), math.cos(yaw)])
    a = np.array([math.cos(yaw), math.sin(yaw)])
    m = np.asarray(mid, float)
    left = (*(m[:2] + spacing / 2 * n), m[2])
    right = (*(m[:2] - spacing / 2 * n), m[2])
    face = (*(m[:2] - face_offset * n), support_z)
    pocket = PocketEstimate(left, 0.2275, 0.078, "known_shape", "known_shape")
    base = dict(candidate_id=cid, proposal_id=cid, proposal_source="oracle", stamp_ns=stamp_ns, hypothesis=hypothesis,
                axis="x", alternatives=(), insertion_yaw_rad=yaw, face_centre_m=face,
                pallet_centre_m=(*(np.asarray(face[:2]) + 0.3 * a), support_z), left=pocket,
                right=replace(pocket, centre_m=right), support_z_m=support_z, support_state="visible", on_floor=True,
                range_m=float(m[0]), score=0.9, geometry="fitted", identity="confirmed", reasons=(), pickable=None,
                fit_reasons=(), shape=shape, pending=("depth", "lane_bounds", "top_layout"),
                alternative_centres_m=(), observed_centre_xy=tuple(m[:2]), observed_extent_m=0.4)
    return PocketCandidate(**{**base, **kw})


def _parametric_shape():
    spec = rs.known_spec("epal6")  # any face layout: a parametric reading carries no known shape
    return ObservedPalletShape(None, face_layout(spec, "x"), (("face", "observed"), ("depth", "unobserved")))


def _frame(t_s, candidates, *, odom=None, epoch=0, clock="ros_sim"):
    stamp = int(round(t_s * S))
    return Frame(stamp_ns=stamp, clock_domain=clock, odom_epoch=epoch, odom_from_base=odom or _odom(),
                 candidates=tuple(replace(c, stamp_ns=stamp) for c in candidates))


# Association --------------------------------------------------------------------------------

def test_consecutive_observations_of_one_pallet_join_one_track():
    tr = CandidateTracker(_bounds())
    for k in range(3):  # the robot drives 0.12 m per frame towards a pallet fixed at odom x = 3.5
        x = 0.12 * k
        tr.update(_frame(0.2 * k, [_candidate((3.5 - x, 0.0, 0.06))], odom=_odom(x)))
    out = tr.tracks(int(0.4 * S), _odom(0.24))
    assert len(out) == 1 and out[0].observations == 3


def test_two_pallets_one_behind_the_other_keep_their_own_tracks():
    tr = CandidateTracker(_bounds())
    for k in range(3):
        x = 0.12 * k
        tr.update(_frame(0.2 * k, [_candidate((3.0 - x, 0.0, 0.06), cid="a"),
                                   _candidate((3.9 - x, 0.0, 0.06), cid="b")], odom=_odom(x)))
    out = sorted(tr.tracks(int(0.4 * S), _odom(0.24)), key=lambda t: t.candidate.left.centre_m[0])
    assert [t.observations for t in out] == [3, 3]
    assert out[0].candidate.left.centre_m[0] == pytest.approx(2.76)
    assert out[1].candidate.left.centre_m[0] == pytest.approx(3.66)


def test_two_pallets_side_by_side_keep_their_own_tracks():
    tr = CandidateTracker(_bounds())
    for k in range(3):
        tr.update(_frame(0.2 * k, [_candidate((3.0, 0.6, 0.06), cid="a"), _candidate((3.0, -0.6, 0.06), cid="b")]))
    assert sorted(t.observations for t in tr.tracks(int(0.4 * S), _odom())) == [3, 3]


def test_the_other_face_of_a_square_pallet_starts_a_new_track():
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, yaw=0.5),)))
    tr.update(_frame(0.0, [_candidate(hypothesis="t11_06:x")]))
    report = tr.update(_frame(0.2, [_candidate(yaw=math.pi / 2, hypothesis="t11_06:y")]))
    assert report.new == 1
    assert len(tr.tracks(int(0.2 * S), _odom())) == 2


def test_a_switch_between_known_shape_and_parametric_reading_keeps_the_track():
    # 2B-3b user decision 2: the same pocket pair read as a known shape or as parametric is one track
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    report = tr.update(_frame(0.2, [_candidate(hypothesis="parametric:x")]))
    assert report.new == 0 and report.accepted == {"c0": 0}


def test_another_pocket_pair_on_the_same_face_starts_a_new_track():
    # wide position bounds, tight layout bound: the pair midpoint moved 0.15 m along the face
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.5, s=0.02),)))
    tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x", face_offset=0.0)]))
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.15, 0.06), hypothesis="parametric:x", face_offset=0.15)]))
    assert report.new == 1


def test_a_unique_maximum_matching_updates_both_tracks():
    # gate b = 0.05 + 0.05; tracks at y 0 and 0.12, observations at 0.08 and 0.20 (plan v1 greedy counterexample)
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="a"), _candidate((3.0, 0.12, 0.06), cid="b")]))
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.08, 0.06), cid="c"), _candidate((3.0, 0.20, 0.06), cid="d")]))
    assert report.held == 0 and report.new == 0 and len(report.accepted) == 2
    t0, t1 = report.accepted["c"], report.accepted["d"]
    assert t0 != t1
    out = {t.track_id: t for t in tr.tracks(int(0.2 * S), _odom())}
    assert out[t0].observations == 2 and out[t1].observations == 2


def test_two_equally_possible_matchings_hold_the_tracks_without_refreshing_them():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="a"), _candidate((3.0, 0.06, 0.06), cid="b")]))
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.02, 0.06), cid="c"), _candidate((3.0, 0.04, 0.06), cid="d")]))
    assert report.held == 2 and report.new == 0 and not report.accepted
    assert all(t.observations == 1 for t in tr.tracks(int(0.2 * S), _odom()))
    assert tr.tracks(int(0.51 * S), _odom()) == ()  # held tracks are not refreshed: they expire


def test_assignment_does_not_depend_on_candidate_order_or_ids():
    def run(order, prefix):
        tr = CandidateTracker(_bounds())
        cands = [_candidate((3.0, y, 0.06), cid=f"{prefix}{i}") for i, y in enumerate((0.0, 0.12, 0.7))]
        tr.update(_frame(0.0, [cands[i] for i in order]))
        nxt = [_candidate((3.0, y, 0.06), cid=f"{prefix}n{i}") for i, y in enumerate((0.08, 0.20, 0.72))]
        tr.update(_frame(0.2, [nxt[i] for i in reversed(order)]))
        return sorted((round(t.candidate.left.centre_m[1], 6), t.observations) for t in tr.tracks(int(0.2 * S), _odom()))
    orders = [list(p) for p in ((0, 1, 2), (2, 0, 1), (1, 2, 0))]
    results = {tuple(run(o, p)) for o, p in zip(orders, ("x", "q", "z"))}
    assert len(results) == 1


def test_a_track_kept_past_one_second_is_deleted_and_the_pallet_gets_a_new_track():
    # 2B-3b: a track is stale (not output, still revivable) up to keep_s = 1.0 s, then deleted for good
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    report = tr.update(_frame(1.1, [_candidate()]))
    assert report.new == 1 and report.revived == 0
    (out,) = tr.tracks(int(1.1 * S), _odom())
    assert out.track_id == 1


def test_uncounted_candidates_and_distances_outside_the_table_are_not_tracked():
    tr = CandidateTracker(_bounds())
    report = tr.update(_frame(0.0, [_candidate(geometry="ambiguous", reasons=("lateral_unconstrained",)),
                                    _candidate((5.0, 0.0, 0.06), cid="far")]))
    assert report.untracked == 2 and tr.tracks(0, _odom()) == ()


def test_odometry_error_and_its_lever_widen_the_gate():
    # a 0.12 m odometry slip fails a 0.1 m gate without odometry error and passes with e = 0.08 m
    for e, joined in ((0.0, False), (0.08, True)):
        tr = CandidateTracker(_bounds(e=e))
        tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06))]))
        report = tr.update(_frame(0.2, [_candidate((3.0, 0.15, 0.06))]))
        assert (report.new == 0) is joined
    tr = CandidateTracker(_bounds(psi=0.02, rear_axle=(-0.5, 0.0)))  # lever 3.5 m x 0.02 rad = 0.07 m
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06))]))
    assert tr.update(_frame(0.2, [_candidate((3.0, 0.15, 0.06))])).new == 0



def test_a_different_support_height_starts_a_new_track():
    tr = CandidateTracker(_bounds())  # support bound 0.02 + 0.02
    tr.update(_frame(0.0, [_candidate(support_z=0.0)]))
    assert tr.update(_frame(0.2, [_candidate(support_z=0.05)])).new == 1


def test_the_yaw_bound_mixes_the_along_tolerance_into_the_lateral_gate():
    # t_lat 0.1, t_along 1.0, mixing 0.2: a 0.2 m lateral step passes only through 1.0 x 0.2
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.05, along=0.5, yaw=0.1),)))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06))]))
    assert tr.update(_frame(0.2, [_candidate((3.0, 0.2, 0.06))])).new == 0

# Time contract --------------------------------------------------------------------------------

def test_a_track_expires_half_a_second_after_its_last_accepted_observation_even_without_candidates():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    tr.update(_frame(0.3, []))
    assert len(tr.tracks(int(0.5 * S), _odom())) == 1
    assert tr.tracks(int(0.5 * S) + 1, _odom()) == ()


def test_repeated_and_out_of_order_frames_are_ignored_and_never_refresh_a_track():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    tr.update(_frame(1.0, []))
    report = tr.update(_frame(0.4, [_candidate()]))  # arrives late
    assert report.ignored == "not_newer"
    assert tr.update(_frame(1.0, [_candidate()])).ignored == "not_newer"
    assert tr.tracks(int(1.0 * S), _odom()) == ()


def test_a_new_odometry_epoch_or_clock_domain_clears_every_track():
    for kw in ({"epoch": 1}, {"clock": "steady"}):
        tr = CandidateTracker(_bounds())
        tr.update(_frame(0.0, [_candidate()]))
        report = tr.update(_frame(0.2, [], **kw))
        assert report.cleared and tr.tracks(int(0.2 * S), _odom()) == ()


def test_a_query_before_the_last_frame_and_a_candidate_with_another_stamp_are_refused():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(1.0, [_candidate()]))
    with pytest.raises(ValueError, match="before"):
        tr.tracks(int(0.9 * S), _odom())
    bad = Frame(stamp_ns=2 * S, clock_domain="ros_sim", odom_epoch=0, odom_from_base=_odom(),
                candidates=(_candidate(stamp_ns=3 * S),))
    with pytest.raises(ValueError, match="stamp"):
        tr.update(bad)


def test_the_output_is_moved_into_the_base_frame_of_the_query_time():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06))]))
    out = tr.tracks(int(0.2 * S), _odom(0.12, 0.0, 0.0))[0]
    assert out.candidate.left.centre_m[0] == pytest.approx(2.88)
    assert out.age_s == pytest.approx(0.2)


# Fusion ----------------------------------------------------------------------------------------

def _yaw_track(yaws_and_ranges, *, window_s=1.2, rows=None):
    tr = CandidateTracker(_bounds(window_s=window_s, rows=rows))
    for k, (yaw, rng) in enumerate(yaws_and_ranges):
        x = 3.5 - rng
        tr.update(_frame(0.2 * k, [_candidate((rng, 0.0, 0.06), yaw)], odom=_odom(x)))
    t = 0.2 * (len(yaws_and_ranges) - 1)
    return tr.tracks(int(t * S), _odom(3.5 - yaws_and_ranges[-1][1]))[0]


def test_fusion_weights_the_nearer_observation_more():
    rows = (_row(1.0, 2.5, yaw=0.3, sigma=0.01), _row(2.5, 4.0, yaw=0.3, sigma=0.04))
    out = _yaw_track([(0.2, 3.0), (0.0, 2.0)], rows=rows)
    # weights 1/0.04^2 and 1/0.01^2: (0.2 * 625 + 0 * 10000) / 10625
    assert out.candidate.insertion_yaw_rad == pytest.approx(0.2 * 625 / 10625)
    assert out.observations == 2


def test_a_zero_window_outputs_the_latest_observation():
    out = _yaw_track([(0.2, 3.0), (0.05, 2.9)], window_s=0.0, rows=(_row(1.0, 4.0, yaw=0.3),))
    assert out.candidate.insertion_yaw_rad == pytest.approx(0.05) and out.observations == 1


def test_older_observations_weigh_less_when_odometry_error_grows():
    rows = (_row(1.0, 4.0, yaw=0.3, sigma=0.01),)
    tr = CandidateTracker(TrackBounds(rows={"known": rows, "parametric": rows}, ages_s=(0.1, 0.3, 10.0),
                                      e_m=(0.0, 0.0, 0.0), psi_rad=(0.0, 0.01, 0.01), rear_axle_xy_m=(0.0, 0.0),
                                      window_s=1.2, max_observations=10, expiry_s=0.5))
    tr.update(_frame(0.0, [_candidate(yaw=0.2)]))
    tr.update(_frame(0.2, [_candidate(yaw=0.0)]))
    out = tr.tracks(int(0.2 * S), _odom())[0]
    # the 0.2 s old observation gets o = L psi = 3.0 x 0.01: weight 1/(0.01^2 + 0.03^2)
    w_old, w_new = 1 / (0.01**2 + 0.03**2), 1 / 0.01**2
    assert out.candidate.insertion_yaw_rad == pytest.approx(0.2 * w_old / (w_old + w_new))


def test_yaw_is_averaged_unwrapped_around_the_latest_observation():
    out = _yaw_track([(math.pi - 0.01, 3.0), (-math.pi + 0.01, 3.0)], rows=(_row(1.0, 4.0, yaw=0.3),))
    assert abs(abs(out.candidate.insertion_yaw_rad) - math.pi) < 1e-9


def test_the_budget_follows_the_triangle_inequality_with_odometry_and_rotation_terms():
    rows = (_row(1.0, 4.0, lat=0.03, along=0.04, yaw=0.05, h=0.01, s=0.005, sigma=0.01),)
    tr = CandidateTracker(TrackBounds(rows={"known": rows, "parametric": rows}, ages_s=(0.1, 0.3, 10.0),
                                      e_m=(0.0, 0.02, 0.02), psi_rad=(0.0, 0.01, 0.01), rear_axle_xy_m=(0.0, 0.0),
                                      window_s=1.2, max_observations=10, expiry_s=0.5))
    tr.update(_frame(0.0, [_candidate()]))
    tr.update(_frame(0.2, [_candidate()]))
    out = tr.tracks(int(0.2 * S), _odom())[0]
    lever = 3.0
    o_old = 0.02 + lever * 0.01
    w_old, w_new = 1 / (0.01**2 + o_old**2), 1 / 0.01**2
    w = w_old + w_new
    b_lat_c = (w_old * (0.03 + o_old) + w_new * 0.03) / w
    b_along_c = (w_old * (0.04 + o_old) + w_new * 0.04) / w
    b_yaw = (w_old * (0.05 + 0.01) + w_new * 0.05) / w
    half = 0.2
    assert out.budget["lateral_m"] == pytest.approx(b_lat_c + 0.005 + half * (1 - math.cos(b_yaw)))
    assert out.budget["along_m"] == pytest.approx(b_along_c + half * math.sin(b_yaw))
    assert out.budget["yaw_rad"] == pytest.approx(b_yaw)
    assert out.budget["height_m"] == pytest.approx(0.01)


def test_a_conflicting_observation_resets_the_history():
    tr = CandidateTracker(_bounds())
    for k, y in enumerate((0.0, 0.09, 0.18)):  # each step passes the 0.1 m gate against the previous one
        report = tr.update(_frame(0.2 * k, [_candidate((3.0, y, 0.06))]))
        assert report.new == (1 if k == 0 else 0)
    out = tr.tracks(int(0.4 * S), _odom())[0]
    assert out.observations == 1 and out.resets == 1
    assert out.candidate.left.centre_m[1] == pytest.approx(0.18 + 0.2)


def test_the_fused_pose_moves_every_field_by_one_rigid_transform_and_keeps_the_spacing():
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, yaw=0.3, h=0.05),)))
    tr.update(_frame(0.0, [_candidate((3.0, 0.02, 0.07), 0.04, spacing=0.40, support_z=0.01)]))
    a_l = np.array([math.cos(-0.04), math.sin(-0.04)])
    seg = tuple((*(np.array([3.0, -0.02]) + d * a_l), 0.0) for d in (0.2, 0.6))  # along the latest reading's axis
    tr.update(_frame(0.2, [_candidate((3.0, -0.02, 0.05), -0.04, spacing=0.44, support_z=-0.01,
                                      alternative_centres_m=((3.4, 0.3, 0.0),), centre_segment_m=seg)]))
    c = tr.tracks(int(0.2 * S), _odom())[0].candidate
    left, right = np.asarray(c.left.centre_m), np.asarray(c.right.centre_m)
    assert np.linalg.norm(left[:2] - right[:2]) == pytest.approx(0.44)
    mid = (left + right) / 2
    assert mid[1] == pytest.approx(0.0, abs=1e-9) and mid[2] == pytest.approx(0.06)
    assert c.insertion_yaw_rad == pytest.approx(0.0, abs=1e-9)
    assert c.support_z_m == pytest.approx(0.0) and c.face_centre_m[2] == pytest.approx(0.0)
    # the latest reading's layout relative to the pair is kept: face centre sits on the pair midpoint
    assert np.allclose(c.face_centre_m[:2], mid[:2])
    a = np.array([1.0, 0.0])
    seg = np.asarray(c.centre_segment_m)
    assert np.allclose(seg[:, 1], 0.0, atol=1e-9) and np.allclose(seg[1, :2] - seg[0, :2], 0.4 * a)
    assert c.pending == ("depth", "lane_bounds", "top_layout") and c.pickable is None


def test_the_tracker_never_reads_proposal_ids():
    rng = random.Random(3)
    tr = CandidateTracker(_bounds())
    for k in range(4):
        cid = f"p{rng.randrange(10**6)}"
        tr.update(_frame(0.2 * k, [_candidate(cid=cid, proposal_id=cid)]))
    assert [t.observations for t in tr.tracks(int(0.6 * S), _odom())] == [4]


# Task 1 review (Codex) ---------------------------------------------------------------------------

def test_a_query_makes_a_track_stale_not_deleted_and_a_newer_frame_revives_it():
    # 2B-3b replaces "an expiry seen by a query is permanent": only keep_s deletes
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    assert tr.tracks(int(0.6 * S), _odom()) == ()
    report = tr.update(_frame(0.4, [_candidate()]))  # newer than the last processed frame, delivered after the query
    assert report.new == 0 and report.accepted == {"c0": 0}  # active by its own acquisition time
    (out,) = tr.tracks(int(0.6 * S), _odom())
    assert out.track_id == 0
    assert tr.tracks(int(1.5 * S), _odom()) == ()
    assert tr.update(_frame(1.45, [_candidate()])).new == 1  # deleted at the 1.5 s query: never revived


def test_the_layout_gate_compares_each_pockets_face_coordinate():
    rows = (_row(1.0, 4.0, lat=0.5, s=0.02),)
    # spacing 400 -> 438 mm with the pair moved 35 mm along the face: the left pocket moves 54 mm (> 40)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x", spacing=0.400, face_offset=0.0)]))
    moved = _candidate((3.0, 0.035, 0.06), hypothesis="parametric:x", spacing=0.438, face_offset=0.035)
    assert tr.update(_frame(0.2, [moved])).new == 1
    # spacing 400 -> 460 mm about the same midpoint: each pocket moves 30 mm (< 40), same pair
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x", spacing=0.400)]))
    assert tr.update(_frame(0.2, [_candidate(hypothesis="parametric:x", spacing=0.460)])).new == 0


def test_the_conflict_check_mixes_the_axes_like_the_gate():
    # midpoint bounds 32 / 54 mm, pocket bounds 35 / 90 mm, yaw 0.1: every observation is inside its budget
    row = BoundRow(lo_m=1.0, hi_m=4.0, lateral_m=0.035, along_m=0.09, centre_lateral_m=0.032, centre_along_m=0.054,
                   centre_height_m=0.02, yaw_rad=0.1, support_m=0.02, layout_m=0.02, sigma_m=0.01)
    tr = CandidateTracker(_bounds(rows=(row,)))
    tr.update(_frame(0.0, [_candidate((2.95, 0.03, 0.06), 0.0)]))
    report = tr.update(_frame(0.2, [_candidate((3.05, -0.03, 0.06), 0.16)]))
    assert report.new == 0
    (out,) = tr.tracks(int(0.2 * S), _odom())
    assert out.resets == 0 and out.observations == 2


@pytest.mark.parametrize("field, value", [("e_m", (math.nan,) * 4), ("psi_rad", (0, 0, math.inf, math.inf)),
                                          ("rear_axle_xy_m", (math.nan, 0.0)), ("window_s", math.nan),
                                          ("expiry_s", math.inf)])
def test_non_finite_bounds_are_refused(field, value):
    kw = dict(rows={"known": (_row(1.0, 4.0),), "parametric": (_row(1.0, 4.0),)}, ages_s=(0.1, 0.5, 1.0, 10.0),
              e_m=(0.0,) * 4, psi_rad=(0.0,) * 4, rear_axle_xy_m=(0.0, 0.0), window_s=1.2, max_observations=10,
              expiry_s=0.5)
    kw[field] = value
    with pytest.raises(ValueError):
        TrackBounds(**kw)


def test_the_stored_history_is_bounded_by_the_window_and_count():
    tr = CandidateTracker(_bounds(window_s=0.6))
    for k in range(200):
        tr.update(_frame(0.2 * k, [_candidate()]))
    (track,) = tr._tracks
    assert len(track.history) <= 4


def test_a_tilted_odometry_pose_is_refused():
    c, s = math.cos(0.001), math.sin(0.001)
    pitched = RigidTransform("base_link", "odom", ((c, 0, s), (0, 1, 0), (-s, 0, c)), (0.0, 0.0, 0.0))
    tr = CandidateTracker(_bounds())
    with pytest.raises(ValueError, match="planar"):
        tr.update(_frame(0.0, [_candidate()], odom=pitched))


def test_a_family_without_calibrated_rows_goes_untracked():
    rows = {"known": (_row(1.0, 4.0),), "parametric": ()}
    tr = CandidateTracker(TrackBounds(rows=rows, ages_s=(0.1, 10.0), e_m=(0.0, 0.0), psi_rad=(0.0, 0.0),
                                      rear_axle_xy_m=(0.0, 0.0), window_s=1.2))
    report = tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x"), _candidate((3.0, 0.8, 0.06), cid="k")]))
    assert report.untracked == 1 and report.new == 1


def test_a_weight_model_sets_the_weights_of_both_the_pose_and_the_budget():
    # weights from the assumed odometry noise (random walk), the budget's odometry terms stay the worst-case bound
    rows = (_row(1.0, 4.0, lat=0.024, along=0.024, yaw=0.3, sigma=0.008),)
    bounds = TrackBounds(rows={"known": rows, "parametric": rows}, ages_s=(0.1, 0.3, 10.0), e_m=(0.0, 0.0315, 0.0315),
                         psi_rad=(0.0, 0.0219, 0.0219), rear_axle_xy_m=(-1.0, 0.0), window_s=1.2,
                         weight_model={"position_m_per_sqrt_s": 0.0016, "yaw_rad_per_sqrt_s": 0.0003})
    tr = CandidateTracker(bounds)
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), 0.1)]))
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), 0.0)]))
    out = tr.tracks(int(0.2 * S), _odom())[0]
    lever = 4.0
    o_w = 0.0016 * math.sqrt(0.2) + lever * 0.0003 * math.sqrt(0.2)
    w_old, w_new = 1 / (0.008**2 + o_w**2), 1 / 0.008**2
    assert out.candidate.insertion_yaw_rad == pytest.approx(0.1 * w_old / (w_old + w_new))
    o_b = 0.0315 + lever * 0.0219
    assert out.budget["centre_lateral_m"] == pytest.approx((w_old * (0.024 + o_b) + w_new * 0.024) / (w_old + w_new))


# 2B-3b: stale tracks, revival, family compatibility, the layout anchor ----------------------------

def test_a_track_revives_with_its_id_after_a_gap_and_is_hidden_while_stale():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    tr.update(_frame(0.2, [_candidate()]))
    assert tr.tracks(int(0.8 * S), _odom()) == ()
    report = tr.update(_frame(0.8, [_candidate()]))
    assert report.revived == 1 and report.new == 0
    (out,) = tr.tracks(int(0.8 * S), _odom())
    assert out.track_id == 0 and out.revivals == 1


@pytest.mark.parametrize("gap_ns, active, revivable", [(0, True, True), (1, False, True),
                                                       (int(0.5 * S), False, True), (int(0.5 * S) + 1, False, False)])
def test_the_active_and_keep_boundaries_are_inclusive(gap_ns, active, revivable):
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    now = int(0.5 * S) + gap_ns
    assert bool(tr.tracks(now, _odom())) is active
    stamp = now / S
    report = tr.update(_frame(stamp, [_candidate()]))
    assert (report.revived == 1) is (revivable and not active) and (report.new == 1) is (not revivable)


def test_an_active_track_is_matched_before_a_stale_one():
    tr = CandidateTracker(_bounds())  # gate 0.1 m
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="a"), _candidate((3.0, 0.12, 0.06), cid="b")]))
    for t in (0.2, 0.4):
        tr.update(_frame(t, [_candidate((3.0, 0.0, 0.06), cid="a")]))
    report = tr.update(_frame(0.6, [_candidate((3.0, 0.05, 0.06), cid="c")]))  # inside both gates
    assert report.held == 0 and report.accepted == {"c": 0} and report.revived == 0


def test_a_track_started_by_a_parametric_reading_remembers_its_first_known_shape_even_after_a_reset():
    tr = CandidateTracker(_bounds())
    for k, (y, hyp) in enumerate(((0.0, "parametric:x"), (0.0, "epal6:x"), (0.09, "parametric:x"),
                                  (0.18, "parametric:x"))):  # the last one resets the history (conflict)
        report = tr.update(_frame(0.2 * k, [_candidate((3.0, y, 0.06), hypothesis=hyp)]))
        assert report.new == (1 if k == 0 else 0)
    (out,) = tr.tracks(int(0.6 * S), _odom())
    assert out.resets == 1
    assert tr.update(_frame(0.8, [_candidate((3.0, 0.18, 0.06), hypothesis="epal6:x")])).new == 0
    assert tr.update(_frame(1.0, [_candidate((3.0, 0.18, 0.06), hypothesis="t11_06:x")])).new == 1


def test_a_known_parametric_known_chain_across_shapes_starts_a_new_track():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate(hypothesis="epal6:x")]))
    assert tr.update(_frame(0.2, [_candidate(hypothesis="parametric:x")])).new == 0
    assert tr.update(_frame(0.4, [_candidate(hypothesis="t11_06:x")])).new == 1


def _pair(left_y, right_y=-0.18, **kw):
    mid = (left_y + right_y) / 2
    return _candidate((3.0, mid, 0.06), hypothesis="parametric:x", spacing=left_y - right_y, face_offset=mid, **kw)


def test_the_layout_anchor_stops_a_pocket_creeping_to_another_opening():
    # each 45 mm step passes against the last observation (0.03 + 0.03) but not against the anchor
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.5, s=0.03),)))
    assert tr.update(_frame(0.0, [_pair(0.180)])).new == 1
    assert tr.update(_frame(0.2, [_pair(0.135)])).new == 0
    assert tr.update(_frame(0.4, [_pair(0.090)])).new == 1


def test_known_limits_a_neighbour_or_another_object_inside_the_gate_joins_the_track():
    # the tracker cannot tell an object that sits inside the gate from the target; the evaluation counts it
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06))]))
    assert tr.update(_frame(0.8, [_candidate((3.7, 0.0, 0.06))])).new == 1      # outside: a new track
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06))]))
    assert tr.update(_frame(0.8, [_candidate((3.08, 0.0, 0.06))])).revived == 1  # inside: joins (limit)
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x")]))
    other = _candidate((3.0, 0.0, 0.06), hypothesis="parametric:x")             # same layout and support
    assert tr.update(_frame(0.2, [other])).new == 0                             # joins (limit)


def test_the_layout_intersection_stops_a_creep_through_shrinking_bounds():
    # Codex: 0 (b_s 32.5 mm) -> 44 (15.3) -> 60 (1.0): each step fits its predecessor, the last misses the first
    rows = (_row(1.0, 2.0, lat=0.5, s=0.001), _row(2.0, 3.0, lat=0.5, s=0.0153), _row(3.0, 4.0, lat=0.5, s=0.0325))
    tr = CandidateTracker(_bounds(rows=rows))
    for k, (rng, left) in enumerate(((3.5, 0.180), (2.5, 0.224), (1.5, 0.240))):
        odom = _odom(3.5 - rng)
        mid = (left - 0.18) / 2
        c = _candidate((rng, mid, 0.06), hypothesis="parametric:x", spacing=left + 0.18, face_offset=mid)
        report = tr.update(_frame(0.2 * k, [c], odom=odom))
        assert report.new == (1 if k in (0, 2) else 0), k


def test_an_observation_held_by_active_tracks_is_not_given_to_a_stale_track():
    # box gates of 0.2 m: S, A1, A2 are pairwise more than 0.2 m apart, all within 0.19 m of O
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.1, along=0.1),)))
    tr.update(_frame(0.0, [_candidate((3.19, 0.0, 0.06), cid="s")]))
    report = tr.update(_frame(0.6, [_candidate((2.81, 0.19, 0.06), cid="a1"), _candidate((2.81, -0.19, 0.06), cid="a2")]))
    assert report.new == 2  # S is stale and out of reach
    report = tr.update(_frame(0.8, [_candidate((3.0, 0.0, 0.06), cid="o")]))
    assert report.accepted == {} and report.revived == 0 and report.new == 0 and report.held == 2


# 2B-3c: the selected track first against its suspected duplicates (user decision B) -------------------

def _with_duplicate():
    """Selected track 0 (EPAL 6 reading) and track 1, born at the same place from a T11 reading that the
    selected track's family test refused -- a suspected duplicate of 0."""
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="a")]))
    tr.select(0)
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", cid="b")]))
    assert report.new == 1
    assert tr._tracks[1].duplicate_of == 0
    return tr


def test_the_selected_track_takes_an_observation_that_also_fits_its_duplicate():
    tr = _with_duplicate()
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="c")]))
    assert report.accepted == {"c": 0} and report.held == 0  # (2B-3e: the duplicate is then absorbed)
    tr = _with_duplicate()
    tr.select(None)
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="d")]))
    assert report.accepted == {} and report.held == 2  # without a selection: the 2B-3b hold


def test_a_stale_selected_track_is_revived_before_its_active_duplicate():
    tr = _with_duplicate()
    for t in (0.4, 0.6):
        tr.update(_frame(t, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", cid="b")]))  # only 1 fits
    report = tr.update(_frame(0.8, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="c")]))
    assert report.accepted == {"c": 0} and report.revived == 1


def test_a_real_neighbour_keeps_its_observation_from_a_stale_selected_track():
    # Codex: a neighbour tracked from the start is not a duplicate -- the 2B-3b active-first rule holds
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="a"), _candidate((3.0, 0.15, 0.06), cid="b")]))
    tr.select(0)
    assert tr._tracks[1].duplicate_of is None
    for t in (0.2, 0.4):
        tr.update(_frame(t, [_candidate((3.0, 0.15, 0.06), cid="b")]))
    report = tr.update(_frame(0.6, [_candidate((3.0, 0.07, 0.06), cid="c")]))  # fits stale 0 and active 1
    assert report.accepted == {"c": 1} and report.revived == 0


def test_two_observations_in_the_selected_gate_turn_the_priority_off_without_blocking_them():
    # agy: no black hole -- the 2B-3b rules apply; tracks 0 (y 0) and 1 (y 0.12), observations at 0.04 and -0.04
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="a"), _candidate((3.0, 0.12, 0.06), cid="b")]))
    tr.select(0)
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.04, 0.06), cid="c"), _candidate((3.0, -0.04, 0.06), cid="d")]))
    assert report.accepted == {"d": 0, "c": 1} and report.held == 0


@pytest.mark.parametrize("duplicate", [True, False])
def test_s_o1_a_o1_a_o2_gives_o1_to_the_selected_and_o2_to_the_other(duplicate):
    # Codex graph S-O1, A-O1, A-O2 (gate 0.1 m). Duplicate: A born at 0.05 from a T11 reading (inside S's
    # pose gate, refused by its family test). Not: A born at 0.12 (outside S's pose gate).
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s")]))
    tr.select(0)
    a_y, a_hyp = (0.05, "t11_06:x") if duplicate else (0.12, "epal6:x")
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s"),
                           _candidate((3.0, a_y, 0.06), hypothesis=a_hyp, cid="a")]))
    assert [x.duplicate_of for x in tr._tracks] == [None, 0 if duplicate else None]
    o2 = a_y + 0.09
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.06, 0.06), hypothesis="parametric:x", cid="o1"),
                                    _candidate((3.0, o2, 0.06), hypothesis="parametric:x", cid="o2")]))
    assert report.accepted == {"o1": 0, "o2": 1}


def test_a_neighbour_is_judged_against_the_selected_track_before_it_moves():
    # Codex: S moves to 0.09 in the same frame A (0.18) is born; A was outside S's pose gate before the update
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.09, 0.06), cid="s"), _candidate((3.0, 0.18, 0.06), cid="a")]))
    assert [x.duplicate_of for x in tr._tracks] == [None, None]
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.135, 0.06), cid="c")]))
    assert report.accepted == {} and report.held == 2  # the 2B-3b hold, not a priority grab


def test_s_o1_o2_a_o1_o3_holds_without_a_unique_matching():
    # S at 0 fits O1 (0.07) and O2 (-0.07); A at 0.14 fits O1 and O3 (0.21): three maximum matchings
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.14, 0.06), cid="a")]))
    tr.select(0)
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.07, 0.06), cid="o1"), _candidate((3.0, -0.07, 0.06), cid="o2"),
                                    _candidate((3.0, 0.21, 0.06), cid="o3")]))
    assert report.accepted == {} and report.held == 2 and report.new == 0


def test_the_selection_ends_with_the_track_and_never_comes_back_by_itself():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    tr.select(0)
    tr.tracks(int(1.1 * S), _odom())  # deleted by a query
    assert tr.selected is None
    tr.update(_frame(1.2, [_candidate()]))
    assert tr.selected is None
    tr.select(1)
    tr.update(_frame(1.4, [], epoch=1))  # a new odometry epoch clears every track and the selection
    assert tr.selected is None and tr.tracks(int(1.4 * S), _odom()) == ()
    with pytest.raises(ValueError):
        tr.select(5)
    tr.select(None)


def test_the_priority_does_not_depend_on_candidate_order():
    def run(order):
        tr = _with_duplicate()
        cands = [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="c"),
                 _candidate((3.0, 0.6, 0.06), hypothesis="parametric:x", cid="e")]
        return tr.update(_frame(0.4, [cands[i] for i in order])).accepted
    assert run((0, 1)) == run((1, 0))


def test_known_residual_a_wild_first_reading_then_accurate_ones_are_refused_by_the_intersection():
    # the plan's stated residual: a 183 mm-like first layout error excludes the truth from the intersection
    rows = (_row(1.0, 2.0, lat=0.5, s=0.02), _row(2.0, 4.0, lat=0.5, s=0.06))
    tr = CandidateTracker(_bounds(rows=rows))
    wild = _candidate((3.0, 0.15, 0.06), hypothesis="parametric:x", face_offset=0.15)   # offsets +0.35 / -0.05
    tr.update(_frame(0.0, [wild], odom=_odom(0.5)))
    tr.select(0)
    good = _candidate((1.5, 0.0, 0.06), hypothesis="parametric:x", face_offset=0.0)      # offsets +0.2 / -0.2
    assert tr.update(_frame(0.2, [good], odom=_odom(2.0))).new == 1


def test_a_track_born_outside_the_selected_pose_gate_is_not_a_duplicate():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="a")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), cid="a"), _candidate((3.0, 0.15, 0.06), cid="b")]))
    assert [t.duplicate_of for t in tr._tracks] == [None, None]


# 2B-3d: near misses on yaw or support (active selected track) are suspected duplicates too -----------

def _born(t_born=0.2, rows=None, **kw):
    """Selected track 0 at y 0 / yaw 0 / support 0 (t 0), then one reading at t_born that starts track 1."""
    rows = rows or (_row(1.0, 4.0, yaw=0.05, z_s=0.005),)  # yaw gate 0.1 rad, support gate 0.01 m
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s")]))
    tr.select(0)
    report = tr.update(_frame(t_born, [_candidate(**{"mid": (3.0, 0.0, 0.06), **kw}, cid="b")]))
    assert report.new == 1
    return tr


@pytest.mark.parametrize("kw, duplicate", [
    (dict(yaw=0.12), True),                                   # yaw 1.2 x
    (dict(support_z=0.015), True),                            # support 1.5 x, 15 mm
    (dict(yaw=0.12, support_z=0.015), True),                  # both near
    (dict(yaw=0.15), True),                                   # yaw exactly 1.5 x
    (dict(support_z=0.025), True),                            # support exactly 2.5 x
    (dict(yaw=0.16), False),                                  # yaw 1.6 x
    (dict(support_z=0.026), False),                           # support 2.6 x
    (dict(mid=(3.0, 0.15, 0.06), yaw=0.12), False),           # lateral miss: a neighbour
    (dict(mid=(3.15, 0.0, 0.06), yaw=0.12), False),           # along miss: a neighbour
    (dict(yaw=0.12, hypothesis="t11_06:x"), False),           # the reading splits too: not a near miss
])
def test_a_near_miss_on_yaw_or_support_marks_a_suspected_duplicate(kw, duplicate):
    tr = _born(**kw)
    assert tr._tracks[1].duplicate_of == (0 if duplicate else None)


def test_the_absolute_caps_bound_a_near_miss():
    # yaw gate 0.6 rad (34 deg): 41 deg is within 1.5 x (52 deg) but above the 40 deg cap (2B-3g)
    tr = _born(rows=(_row(1.0, 4.0, yaw=0.3, along=0.06, z_s=0.005),), yaw=math.radians(41))
    assert tr._tracks[1].duplicate_of is None
    # support gate 24 mm: 50 mm is within 2.5 x but not under 45 mm
    tr = _born(rows=(_row(1.0, 4.0, yaw=0.05, z_s=0.012),), support_z=0.050)
    assert tr._tracks[1].duplicate_of is None


def test_the_selected_track_wins_over_a_yaw_near_miss_duplicate():
    tr = _born(yaw=0.12)
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), 0.06, hypothesis="parametric:x", cid="c")]))
    assert report.accepted == {"c": 0}


def test_a_stale_selected_track_wins_over_its_support_near_miss_duplicate():
    tr = _born(support_z=0.015)
    for t in (0.4, 0.6):
        tr.update(_frame(t, [_candidate((3.0, 0.0, 0.06), support_z=0.015, cid="b")]))  # only 1 fits
    report = tr.update(_frame(0.8, [_candidate((3.0, 0.0, 0.06), support_z=0.008, cid="c")]))
    assert report.accepted == {"c": 0} and report.revived == 1


def test_a_neighbour_born_while_the_selected_track_is_stale_is_not_a_duplicate():
    # Codex: born at 0.6 s (selected stale, its gate widened) on a support near miss -- a real neighbour
    tr = _born(t_born=0.6, mid=(3.0, 0.05, 0.06), support_z=0.015)
    assert tr._tracks[1].duplicate_of is None
    report = tr.update(_frame(0.8, [_candidate((3.0, 0.05, 0.06), support_z=0.008, cid="c")]))
    assert report.accepted == {"c": 1}


def test_a_front_back_neighbour_born_while_the_selected_track_is_active_is_not_a_duplicate():
    # Codex, with the calibrated table: A at x 4.24 (range 3.49), B first seen 0.4 s later at x 4.71 with
    # support 20 mm (1.34 x) -- inside the odometry-widened along gate (501 mm) but 470 mm away
    bounds = TrackBounds.load(Path(__file__).resolve().parents[3] / "config/recognition_track_bounds.yaml")
    tr = CandidateTracker(bounds)
    tr.update(_frame(0.0, [_candidate((4.24, 0.0, 0.06), range_m=3.49, cid="a")]))
    tr.select(0)
    tr.update(_frame(0.4, [_candidate((4.71, 0.0, 0.06), range_m=3.96, support_z=0.020, cid="b")]))
    assert [t.duplicate_of for t in tr._tracks] == [None, None]
    report = tr.update(_frame(0.6, [_candidate((4.71, 0.0, 0.06), range_m=3.96, support_z=0.010, cid="c")]))
    assert report.accepted == {"c": 1} and report.revived == 0


def test_the_absolute_caps_are_exact_at_40_degrees_and_45_mm():
    tr = _born(rows=(_row(1.0, 4.0, yaw=0.3, along=0.06, z_s=0.005),), yaw=math.radians(40))
    assert tr._tracks[1].duplicate_of == 0
    tr = _born(rows=(_row(1.0, 4.0, yaw=0.05, z_s=0.012),), support_z=0.045)
    assert tr._tracks[1].duplicate_of is None


def test_a_near_miss_needs_the_pockets_within_the_position_cap():
    # wide position gate (lat 0.5): a 0.25 m lateral step with a yaw near miss is a neighbour, 0.15 m is not
    rows = (_row(1.0, 4.0, lat=0.5, yaw=0.05, z_s=0.005),)
    assert _born(rows=rows, mid=(3.0, 0.25, 0.06), yaw=0.12)._tracks[1].duplicate_of is None
    assert _born(rows=rows, mid=(3.0, 0.15, 0.06), yaw=0.12)._tracks[1].duplicate_of == 0


def test_the_position_cap_includes_exactly_0_2_m():
    rows = (_row(1.0, 4.0, lat=0.5, yaw=0.05, z_s=0.005),)
    # a support near miss so the pockets move exactly 0.2 m (a yaw miss would also rotate them)
    assert _born(rows=rows, mid=(3.0, 0.2, 0.06), support_z=0.015)._tracks[1].duplicate_of == 0


# 2B-3e: the selected track absorbs its suspected duplicates ---------------------------------------------

def test_the_selected_track_absorbs_a_near_miss_duplicate_that_fits_the_observation_it_takes():
    tr = _born(support_z=0.015)  # duplicate 1 born on a support near miss (same reading as the selected track)
    assert tr._tracks[1].duplicate_kind == "near_miss"
    history, layout = list(tr._tracks[0].history), list(tr._tracks[0].layout)
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", support_z=0.008, cid="c")]))
    assert report.accepted == {"c": 0} and report.absorbed == 1
    assert [(t.id, t.shadow) for t in tr._tracks] == [(0, False), (1, True)]  # kept as a shadow
    assert [t.track_id for t in tr.tracks(int(0.4 * S), _odom())] == [0]   # hidden from the output
    assert tr._tracks[0].history == history + [tr._tracks[0].last]  # its own readings only, nothing merged
    assert tr._tracks[0].layout == [(max(lo, x - b), min(hi, x + b)) for (lo, hi), x, b in
                                    zip(layout, tr._tracks[0].last.offsets, [tr._tracks[0].last.row.layout_m] * 2)]
    with pytest.raises(ValueError):
        tr.select(1)


def test_a_duplicate_that_does_not_fit_the_taken_observation_stays():
    tr = _with_duplicate()  # duplicate 1 reads t11_06:x; an epal6:x reading fits only the selected track
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="c")]))
    assert report.accepted == {"c": 0} and report.absorbed == 0 and [t.id for t in tr._tracks] == [0, 1]


def test_a_real_neighbour_is_never_absorbed():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.12, 0.06), cid="a")]))
    tr.select(0)
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.06, 0.06), cid="o1"), _candidate((3.0, 0.18, 0.06), cid="o2")]))
    assert report.accepted == {"o1": 0, "o2": 1} and report.absorbed == 0 and len(tr._tracks) == 2


def test_nothing_is_absorbed_without_a_selection_or_when_the_selected_track_takes_nothing():
    tr = _with_duplicate()
    tr.select(None)
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="c")]))
    assert report.absorbed == 0 and len(tr._tracks) == 2
    tr = _with_duplicate()
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", cid="c")]))  # only 1 fits
    assert report.accepted == {"c": 1} and report.absorbed == 0 and len(tr._tracks) == 2


def test_a_duplicate_that_took_another_reading_this_frame_is_not_absorbed():
    # S-O1, D-O1, D-O2: the duplicate D took O2, so it tracks something and stays
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s"),
                           _candidate((3.0, 0.05, 0.06), hypothesis="t11_06:x", cid="d")]))
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.06, 0.06), hypothesis="parametric:x", cid="o1"),
                                    _candidate((3.0, 0.14, 0.06), hypothesis="parametric:x", cid="o2")]))
    assert report.accepted == {"o1": 0, "o2": 1} and report.absorbed == 0


def test_a_stale_real_neighbour_that_fits_the_taken_reading_is_not_absorbed():
    # S (selected) and A born together at t 0 (A is not a duplicate); A goes stale; S takes a reading A fits too
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.15, 0.06), cid="a")]))
    tr.select(0)
    for t in (0.2, 0.4, 0.6):
        tr.update(_frame(t, [_candidate((3.0, 0.0, 0.06), cid="s")]))
    report = tr.update(_frame(0.8, [_candidate((3.0, 0.07, 0.06), cid="o")]))
    assert report.accepted == {"o": 0} and report.absorbed == 0 and [t.id for t in tr._tracks] == [0, 1]


def test_a_reading_split_duplicate_is_kept_so_a_later_reading_of_that_kind_stays_ambiguous():
    # Codex: S (EPAL 6) selected, neighbour E (T11) ahead; S read once as T11 starts duplicate D; a parametric
    # reading fits S and D; later S is read as T11 again -- with D gone, E would take it (identity error)
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, along=0.3),)))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s"),
                           _candidate((3.65, 0.0, 0.06), hypothesis="t11_06:x", cid="e")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", cid="d")]))
    assert tr._tracks[2].duplicate_of == 0
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="o")]))
    assert report.accepted == {"o": 0} and report.absorbed == 0  # a reading split: kept
    report = tr.update(_frame(0.6, [_candidate((3.1, 0.0, 0.06), hypothesis="t11_06:x", cid="t")]))
    assert report.accepted.get("t") != 1  # never handed to the neighbour E


def test_a_held_near_miss_duplicate_is_not_absorbed():
    # S-O, D-O/P, E-P: S takes O by priority (its only rival is D); D and E are held over P -- D stays
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005),)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.18, 0.06), support_z=0.015, cid="e")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.18, 0.06), support_z=0.015, cid="e"),
                           _candidate((3.0, 0.06, 0.06), support_z=0.015, cid="d")]))
    assert [t.duplicate_of for t in tr._tracks] == [None, None, 0]
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.03, 0.06), support_z=0.008, cid="o"),
                                    _candidate((3.0, 0.12, 0.06), support_z=0.015, cid="p")]))
    assert report.accepted == {"o": 0} and report.held == 2 and report.absorbed == 0 and len(tr._tracks) == 3


def test_a_near_miss_duplicate_that_later_learned_another_reading_becomes_a_shadow():
    # Codex: D is born on a support near miss (parametric), then takes a T11 reading -- its reading now
    # differs from the selected EPAL 6 track, so absorbing it would lose that reading
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005),)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", support_z=0.015, cid="d")]))
    assert tr._tracks[1].duplicate_kind == "near_miss"
    tr.update(_frame(0.3, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", support_z=0.015, cid="d2")]))
    assert tr._tracks[1].known == "t11_06:x"
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", support_z=0.008, cid="c")]))
    assert report.accepted == {"c": 0} and report.absorbed == 1 and tr._tracks[1].shadow
    # the shadow keeps its T11 hypothesis: it takes the T11 reading S refuses, hidden, with no new track
    report = tr.update(_frame(0.5, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", support_z=0.015, cid="t")]))
    assert report.accepted == {"t": 1} and report.new == 0
    assert [x.track_id for x in tr.tracks(int(0.5 * S), _odom())] == [0]




def test_a_shadow_keeps_the_pose_hypothesis_of_an_absorbed_duplicate():
    # Codex (accepted residual, plan 2B-3e): S, D, E share one reading; D (support 15 mm) is absorbed;
    # a later target reading misses S on support but would have fitted D -- E (support 15 mm) now takes it
    # where 2B-3d gave it to D (still active at 0.6 s here; held between D and E once D was stale)
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005, along=0.15),)  # along gate 0.3 m
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.4, 0.0, 0.06), support_z=0.015, cid="e")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), support_z=0.015, cid="d")]))
    assert tr._tracks[2].duplicate_kind == "near_miss"
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), support_z=0.008, cid="c")]))
    assert report.accepted == {"c": 0} and report.absorbed == 1
    report = tr.update(_frame(0.6, [_candidate((3.15, 0.0, 0.06), support_z=0.024, cid="t")]))
    assert report.accepted.get("t") != 1 and report.new == 0  # the shadow keeps it from E (was E before shadows)




def test_a_shadow_keeps_the_ambiguity_after_the_selected_track_decides_a_hypothesis():
    # Codex frames: S undecided (parametric) and T11 neighbour E; D born undecided on a support near miss;
    # D must survive the shared reading, so after S learns EPAL 6 a T11 target reading is held between D and E
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005, along=0.15),)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="s"),
                           _candidate((3.4, 0.0, 0.06), hypothesis="t11_06:x", support_z=0.012, cid="e")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", support_z=0.012, cid="d")]))
    assert tr._tracks[2].duplicate_kind == "near_miss"
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", support_z=0.006, cid="c")]))
    assert report.accepted == {"c": 0} and report.absorbed == 1
    tr.update(_frame(0.5, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", support_z=0.006, cid="c2")]))
    assert tr._tracks[0].known == "epal6:x"
    report = tr.update(_frame(0.6, [_candidate((3.15, 0.0, 0.06), hypothesis="t11_06:x", support_z=0.012, cid="t")]))
    assert report.accepted.get("t") != 1 and report.new == 0  # the shadow keeps it from E


def test_a_shadow_cannot_be_selected_and_expires_with_keep_s():
    tr = _born(support_z=0.015)
    tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", support_z=0.008, cid="c")]))
    assert tr._tracks[1].shadow
    with pytest.raises(ValueError):
        tr.select(1)
    assert 1 not in tr.stale_ids(int(1.0 * S))  # a shadow is never listed as revivable
    tr.update(_frame(1.3, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="c2")]))  # 1.1 s after D's reading
    assert [t.id for t in tr._tracks] == [0]


def test_the_shadow_veto_applies_before_the_matching_decides_uniqueness():
    # Codex: shadow D-O1, A-O1/O2, B-O2. Vetoing A-O1 after matching left B with O2; without A-O1 the
    # reading O2 is ambiguous between A and B and must be held
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005),)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.12, 0.06), support_z=0.02, cid="a"),
                           _candidate((3.0, 0.24, 0.06), support_z=0.02, cid="b")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.12, 0.06), support_z=0.02, cid="a"),
                           _candidate((3.0, 0.24, 0.06), support_z=0.02, cid="b"),
                           _candidate((3.0, 0.0, 0.06), support_z=0.015, cid="d")]))
    assert tr._tracks[3].duplicate_kind == "near_miss"
    report = tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), support_z=0.008, cid="s"),
                                    _candidate((3.0, 0.12, 0.06), support_z=0.02, cid="a"),
                                    _candidate((3.0, 0.24, 0.06), support_z=0.02, cid="b")]))
    assert report.absorbed == 1 and tr._tracks[3].shadow
    report = tr.update(_frame(0.6, [_candidate((3.0, 0.06, 0.06), support_z=0.02, cid="o1"),
                                    _candidate((3.0, 0.18, 0.06), support_z=0.02, cid="o2")]))
    assert "o2" not in report.accepted and report.new == 0


def _shadowed():
    """Selected S (y 0), real neighbour E (y 0.18, born with S, so not a duplicate), and S's shadow D
    (a support near miss absorbed at 0.4 s)."""
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005),)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.18, 0.06), cid="e")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), support_z=0.015, cid="d")]))
    tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), support_z=0.008, cid="s")]))
    assert tr._tracks[2].shadow and tr._tracks[1].duplicate_of is None
    return tr


def test_a_shadow_never_hands_a_real_neighbours_reading_to_a_stale_selected_track():
    # Codex: E active, S stale, the shadow fits too -- E keeps its reading (2B-3b active-first)
    tr = _shadowed()
    for t in (0.6, 0.8):
        tr.update(_frame(t, [_candidate((3.0, 0.12, 0.06), cid="e")]))
    report = tr.update(_frame(1.0, [_candidate((3.0, 0.06, 0.06), support_z=0.008, cid="o")]))  # fits E, stale S, shadow
    assert report.accepted == {"o": 1} and report.revived == 0


def test_a_shadow_keeps_the_hold_between_an_active_selected_track_and_a_real_neighbour():
    tr = _shadowed()
    tr.update(_frame(0.5, [_candidate((3.0, 0.12, 0.06), cid="e")]))
    report = tr.update(_frame(0.6, [_candidate((3.0, 0.05, 0.06), support_z=0.008, cid="o")]))
    assert report.accepted == {} and report.held >= 2  # S and E both fit: the 2B-3b hold stays


def test_switching_the_selection_keeps_the_old_targets_shadow_in_the_competition():
    # Codex: after absorbing D into S, selecting the neighbour E must not exempt E from S's shadow
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005, along=0.15),)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.4, 0.0, 0.06), support_z=0.015, cid="e")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), support_z=0.015, cid="d")]))
    tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), support_z=0.008, cid="c")]))
    assert tr._tracks[2].shadow
    tr.select(1)
    report = tr.update(_frame(0.6, [_candidate((3.15, 0.0, 0.06), support_z=0.024, cid="t")]))
    assert report.accepted.get("t") != 1  # the target's reading that fits S's shadow is not handed to E


def test_the_owner_exemption_needs_a_single_reading(monkeypatch):
    # Codex graph: S -> {O1, O2}, neighbour E -> {O2, O3}, S's shadow D -> {O1, O2, O3}. Exempting D from
    # every reading S fits settles S<-O1, E<-O2; with D kept the readings stay ambiguous and are held
    from forklift_core.perception import candidate_tracking as CT
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.6, 0.06), cid="e"),
                           _candidate((3.0, 1.2, 0.06), cid="d")]))
    tr.select(0)
    tr._tracks[2].shadow, tr._tracks[2].duplicate_of = True, 0
    edges = {(0, "o1"), (0, "o2"), (1, "o2"), (1, "o3"), (2, "o1"), (2, "o2"), (2, "o3")}
    monkeypatch.setattr(CT, "_gate", lambda track, o, bounds: (track.id, o.candidate.candidate_id) in edges)
    report = tr.update(_frame(0.2, [_candidate((3.0, y, 0.06), cid=c) for y, c in ((0.0, "o1"), (0.6, "o2"), (1.2, "o3"))]))
    assert report.accepted == {} and report.new == 0


def test_a_stale_owner_gives_its_shadow_no_exemption(monkeypatch):
    # Codex graph: stale S -> O1, active neighbour E -> {O1, O2}, S's shadow D -> {O1, O2}. Exempting D-O1
    # settles E<-O1, D<-O2 in the active stage; without the exemption both readings stay ambiguous
    from forklift_core.perception import candidate_tracking as CT
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.0, 0.6, 0.06), cid="e"),
                           _candidate((3.0, 1.2, 0.06), cid="d")]))
    tr.select(0)
    for t in (0.2, 0.4, 0.6):
        tr.update(_frame(t, [_candidate((3.0, 0.6, 0.06), cid="e"), _candidate((3.0, 1.2, 0.06), cid="d")]))
    tr._tracks[2].shadow, tr._tracks[2].duplicate_of = True, 0
    edges = {(0, "o1"), (1, "o1"), (1, "o2"), (2, "o1"), (2, "o2")}
    monkeypatch.setattr(CT, "_gate", lambda track, o, bounds: (track.id, o.candidate.candidate_id) in edges)
    report = tr.update(_frame(0.8, [_candidate((3.0, y, 0.06), cid=c) for y, c in ((0.0, "o1"), (0.6, "o2"))]))
    assert "o1" not in report.accepted and report.new == 0


def test_a_shadow_keeps_tracking_its_readings_so_it_stays_in_the_active_competition():
    # Codex: a reading only the shadow fits refreshes it (as the duplicate would have in 2B-3d); the next
    # reading that fits it and the neighbour E is then held instead of going to E
    rows = (_row(1.0, 4.0, yaw=0.05, z_s=0.005, along=0.15),)
    tr = CandidateTracker(_bounds(rows=rows))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.4, 0.0, 0.06), support_z=0.015, cid="e")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), support_z=0.015, cid="d")]))
    tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), support_z=0.008, cid="c")]))
    assert tr._tracks[2].shadow
    tr.update(_frame(0.5, [_candidate((3.4, 0.0, 0.06), support_z=0.015, cid="e")]))
    report = tr.update(_frame(0.6, [_candidate((3.0, 0.0, 0.06), support_z=0.024, cid="t1")]))
    assert report.accepted == {"t1": 2}  # the shadow takes it -- hidden, but its age is refreshed
    report = tr.update(_frame(0.8, [_candidate((3.15, 0.0, 0.06), support_z=0.024, cid="t2")]))
    assert report.accepted.get("t2") != 1  # never handed to the neighbour E
    assert [t.track_id for t in tr.tracks(int(0.8 * S), _odom())] != [] and 2 not in [t.track_id for t in tr.tracks(int(0.8 * S), _odom())]


def test_a_shadow_keeps_its_edges_when_a_real_neighbour_also_competes(monkeypatch):
    # Codex graph: active S -> O1, real A -> O1, S's shadow D -> {O1, O2}, real B -> {O2, O3}: with A competing
    # there is no priority, and cutting D-O1 settled D<-O2, B<-O3; with D's edges kept everything is held
    from forklift_core.perception import candidate_tracking as CT
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate((3.0, y, 0.06), cid=c) for y, c in ((0.0, "s"), (0.6, "a"), (1.2, "d"), (1.8, "b"))]))
    tr.select(0)
    tr._tracks[2].shadow, tr._tracks[2].duplicate_of = True, 0
    edges = {(0, "o1"), (1, "o1"), (2, "o1"), (2, "o2"), (3, "o2"), (3, "o3")}
    monkeypatch.setattr(CT, "_gate", lambda track, o, bounds: (track.id, o.candidate.candidate_id) in edges)
    report = tr.update(_frame(0.2, [_candidate((3.0, y, 0.06), cid=c) for y, c in ((0.0, "o1"), (0.6, "o2"), (1.2, "o3"))]))
    assert report.accepted == {} and report.new == 0


def test_a_shadow_reappears_as_an_ordinary_track_once_its_owner_is_deleted():
    # Codex: the target keeps being read only by the shadow; when the owner S is deleted the shadow must be
    # visible and selectable again (not chosen automatically)
    tr = _born(support_z=0.015)
    tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", support_z=0.008, cid="c")]))
    assert tr._tracks[1].shadow
    for k in range(3, 12):  # readings only the shadow fits, every 0.2 s
        tr.update(_frame(0.2 * k, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", support_z=0.024, cid=f"t{k}")]))
    now = int(2.2 * S)
    assert all(t.id != 0 for t in tr._tracks) and tr.selected is None
    assert [t.track_id for t in tr.tracks(now, _odom())] == [1]
    tr.select(1)



# 2B-3f: suspected-duplicate readings near an active selected track are born as shadows -------------------

def _suppressing(rows=None, **kw):
    tr = CandidateTracker(_bounds(rows=rows or (_row(1.0, 4.0, yaw=0.05, z_s=0.005),), suppress=True, **kw))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s")]))
    tr.select(0)
    return tr


@pytest.mark.parametrize("kw", [dict(hypothesis="t11_06:x"),                     # reading split
                                dict(hypothesis="epal6:x", support_z=0.015)])    # support near miss
def test_a_suspected_duplicate_reading_is_born_as_a_hidden_shadow(kw):
    tr = _suppressing()
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), cid="d", **kw)]))
    assert report.new == 1 and report.suppressed == 1 and report.accepted == {"d": 1}
    assert tr._tracks[1].shadow and [t.track_id for t in tr.tracks(int(0.2 * S), _odom())] == [0]


def test_without_a_selection_the_same_reading_is_an_ordinary_track():
    tr = _suppressing()
    tr.select(None)
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", cid="d")]))
    assert report.suppressed == 0 and not tr._tracks[1].shadow


def test_a_real_neighbour_is_an_ordinary_track():
    tr = _suppressing()
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.3, 0.06), hypothesis="t11_06:x", cid="n")]))
    assert report.suppressed == 0 and not tr._tracks[1].shadow


def test_a_different_reading_near_a_stale_selected_track_is_an_ordinary_track():
    # the hidden birth needs an active selected track and pockets within 0.2 m (Codex: no such limit let a
    # real neighbour stay hidden for as long as the selected track lived)
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0),), e=0.08, suppress=True))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s")]))
    tr.select(0)
    report = tr.update(_frame(0.8, [_candidate((3.0, 0.15, 0.06), hypothesis="t11_06:x", cid="n")]))
    assert report.suppressed == 0 and not tr._tracks[1].shadow


def test_a_persistent_neighbour_with_another_reading_stays_visible_and_selectable():
    # Codex: a T11 neighbour read every frame 0.42 m from an active EPAL 6 selected track
    tr = _suppressing(rows=(_row(1.0, 4.0, lat=0.15, along=0.3),))
    for k in range(1, 6):
        tr.update(_frame(0.2 * k, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s"),
                                   _candidate((3.42, 0.0, 0.06), hypothesis="t11_06:x", cid="n")]))
    assert [t.track_id for t in tr.tracks(int(1.0 * S), _odom())] == [0, 1]
    tr.select(1)


def _hidden_case(kind):
    """Codex's 2B-3e counterexamples with suppression on: S selected, neighbour E ahead, the target read once
    as a suspected duplicate (born hidden), then read again in a way that fits E and the hidden track only."""
    if kind == "reading":
        tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, along=0.3),), suppress=True))
        tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s"),
                               _candidate((3.65, 0.0, 0.06), hypothesis="t11_06:x", cid="e")]))
        tr.select(0)
        tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="t11_06:x", cid="d")]))
        tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="p")]))
        target = _candidate((3.1, 0.0, 0.06), hypothesis="t11_06:x", cid="t")
    else:
        tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, yaw=0.05, z_s=0.005, along=0.15),), suppress=True))
        tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), cid="s"), _candidate((3.4, 0.0, 0.06), support_z=0.015, cid="e")]))
        tr.select(0)
        tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), support_z=0.015, cid="d")]))
        tr.update(_frame(0.4, [_candidate((3.0, 0.0, 0.06), support_z=0.008, cid="p")]))
        target = _candidate((3.15, 0.0, 0.06), support_z=0.024, cid="t")
    assert tr._tracks[2].shadow
    return tr, target


@pytest.mark.parametrize("kind", ["reading", "near_miss"])
@pytest.mark.parametrize("switch", [False, True])
def test_with_hidden_births_the_target_reading_is_never_handed_to_the_neighbour(kind, switch):
    from forklift_core.perception import candidate_tracking as CT
    tr, target = _hidden_case(kind)
    if switch:
        tr.select(1)  # switching to the neighbour keeps the target's shadow competing
    o = CT._Observation.make(replace(target, stamp_ns=int(0.6 * S)), _frame(0.6, [target]), tr.bounds)
    assert CT._gate(tr._tracks[1], o, tr.bounds) and CT._gate(tr._tracks[2], o, tr.bounds)  # fits E and the shadow
    report = tr.update(_frame(0.6, [target]))
    assert report.accepted.get("t") != 1


@pytest.mark.parametrize("towards", [True, False])
def test_the_hidden_birth_is_judged_against_the_selected_track_before_it_moves(towards):
    # Codex: S at y 0 is read at y +/-0.15 in the same frame as another reading N; N's distance to S before
    # matching decides (0.30 m -> visible, 0.15 m -> hidden), not after S moved
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.15),), suppress=True))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", cid="s")]))
    tr.select(0)
    s_y, n_y = (0.15, 0.30) if towards else (-0.15, 0.15)
    report = tr.update(_frame(0.2, [_candidate((3.0, s_y, 0.06), hypothesis="epal6:x", cid="s"),
                                    _candidate((3.0, n_y, 0.06), hypothesis="t11_06:x", cid="n")]))
    assert report.accepted["s"] == 0
    assert tr._tracks[1].shadow is (not towards)


# 2B-3g: an axis reading switch in place, the wider yaw cap, family-crossing support ---------------------

def test_a_t11_axis_switch_on_the_same_face_keeps_the_track():
    # the same face read under the x and the y hypothesis (only the bottom-board evidence differs)
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate(hypothesis="t11_06:x")]))
    report = tr.update(_frame(0.2, [_candidate(hypothesis="t11_06:y")]))
    assert report.new == 0 and report.accepted == {"c0": 0}


def test_a_stale_track_revives_on_an_axis_switch_in_place():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate(hypothesis="t11_06:x")]))
    report = tr.update(_frame(0.8, [_candidate(hypothesis="t11_06:y")]))
    assert report.new == 0 and report.revived == 1


@pytest.mark.parametrize("dx, joins", [(0.2, True), (0.21, False)])
def test_an_axis_switch_joins_only_within_the_pocket_cap(dx, joins):
    # wide position gate (0.5 m): the pose fits either way, the 0.2 m pocket cap decides
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.5, along=0.5),)))
    tr.update(_frame(0.0, [_candidate(hypothesis="t11_06:x")]))
    report = tr.update(_frame(0.2, [_candidate((3.0 + dx, 0.0, 0.06), hypothesis="t11_06:y")]))
    assert (report.new == 0) is joins


def test_a_same_axis_reading_far_away_is_still_left_to_the_pose_gate():
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.5, along=0.5),)))
    tr.update(_frame(0.0, [_candidate(hypothesis="t11_06:x")]))
    report = tr.update(_frame(0.2, [_candidate((3.3, 0.0, 0.06), hypothesis="t11_06:x")]))
    assert report.new == 0


def test_another_known_shape_still_starts_a_new_track():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate(hypothesis="epal6:x")]))
    assert tr.update(_frame(0.2, [_candidate(hypothesis="t11_06:x")])).new == 1


@pytest.mark.parametrize("hypothesis, revived", [("t11_06:y", 0), ("t11_06:x", 1)])
def test_a_stale_selected_track_does_not_revive_on_another_pallets_other_axis_reading(hypothesis, revived):
    # Codex, with the calibrated table: T11 A and B 710 mm apart front to back; A read as x at base 4.23 m
    # and selected; 0.8 s later (robot 0.48 m on) only B is seen, at base 4.26 m (odom 4.74 m) -- 510 mm
    # from A's last reading but inside the stale pose gate (586 mm). Read on another axis it is refused by
    # the pocket cap; read on the same axis the pose gate alone decides (unchanged, its known limit)
    bounds = TrackBounds.load(Path(__file__).resolve().parents[3] / "config/recognition_track_bounds.yaml")
    tr = CandidateTracker(bounds)
    tr.update(_frame(0.0, [_candidate((4.23, 0.0, 0.06), hypothesis="t11_06:x", range_m=3.48, cid="a")]))
    tr.select(0)
    report = tr.update(_frame(0.8, [_candidate((4.26, 0.0, 0.06), hypothesis=hypothesis, range_m=3.51, cid="b")],
                              odom=_odom(0.48)))
    assert report.revived == revived and report.new == 1 - revived


@pytest.mark.parametrize("hypothesis, support, duplicate", [
    ("parametric:x", 0.044, True),   # known -> parametric, 44 mm (6.3 x the 7 mm gate): only the 45 mm cap
    ("parametric:x", 0.046, False),  # over the cap
    ("epal6:x", 0.044, False),       # same family: the 2.5 x ratio still applies
])
def test_a_family_crossing_support_near_miss_uses_only_the_absolute_cap(hypothesis, support, duplicate):
    tr = _born(rows=(_row(1.0, 4.0, yaw=0.05, z_s=0.0035),), hypothesis=hypothesis, support_z=support)
    assert tr._tracks[1].duplicate_of == (0 if duplicate else None)


def test_a_family_crossing_stacked_layer_misread_by_44_mm_is_hidden():
    # Codex P3, a synthetic residual: a lower layer read as known with support 46 mm, an upper layer read
    # parametric at 90 mm (44 mm apart, gate 7 mm) right above it is hidden. Pinned as the current behaviour
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, yaw=0.05, z_s=0.0035),), suppress=True))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", support_z=0.046, cid="s")]))
    tr.select(0)
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.0, 0.06), hypothesis="epal6:x", support_z=0.046, cid="s"),
                                    _candidate((3.0, 0.0, 0.15), hypothesis="parametric:x", support_z=0.090,
                                               cid="u")]))
    assert report.suppressed == 1
    assert len(tr.tracks(int(0.2 * S), _odom())) == 1


@pytest.mark.parametrize("first, second", [("t11_06:x", "t11_06:y"), ("t11_06:y", "t11_06:x")])
def test_a_switch_back_to_the_first_axis_is_still_an_axis_switch(first, second):
    # Codex: A read first, then in place on the other axis, then B 510 mm away 0.8 s later on the first
    # axis again -- comparing with the first hypothesis alone let B revive the selected track
    bounds = TrackBounds.load(Path(__file__).resolve().parents[3] / "config/recognition_track_bounds.yaml")
    tr = CandidateTracker(bounds)
    tr.update(_frame(0.0, [_candidate((3.30, 0.0, 0.06), hypothesis=first, range_m=3.30, cid="a")]))
    tr.select(0)
    assert tr.update(_frame(0.4, [_candidate((3.30, 0.0, 0.06), hypothesis=second, range_m=3.30,
                                             cid="a")])).accepted == {"a": 0}
    report = tr.update(_frame(1.2, [_candidate((3.81, 0.0, 0.06), hypothesis=first, range_m=3.81, cid="b")]))
    assert report.revived == 0 and report.new == 1


def test_without_an_axis_switch_the_same_reading_far_away_revives_on_the_pose_gate():
    # the control for the test above: the same 510 mm / 0.8 s step on one axis passes the stale pose gate
    bounds = TrackBounds.load(Path(__file__).resolve().parents[3] / "config/recognition_track_bounds.yaml")
    tr = CandidateTracker(bounds)
    for t in (0.0, 0.4):
        tr.update(_frame(t, [_candidate((3.30, 0.0, 0.06), hypothesis="t11_06:x", range_m=3.30, cid="a")]))
        tr.select(0)
    report = tr.update(_frame(1.2, [_candidate((3.81, 0.0, 0.06), hypothesis="t11_06:x", range_m=3.81, cid="b")]))
    assert report.revived == 1


def test_an_axis_switch_through_a_parametric_reading_keeps_the_pocket_cap():
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.5, along=0.5),)))
    tr.update(_frame(0.0, [_candidate(hypothesis="t11_06:x")]))
    assert tr.update(_frame(0.2, [_candidate(hypothesis="parametric:x")])).new == 0
    assert tr.update(_frame(0.4, [_candidate((3.3, 0.0, 0.06), hypothesis="t11_06:y")])).new == 1


def test_the_axis_of_the_last_known_reading_outlives_the_fusion_window():
    # W = 0 keeps only the latest reading in the history: the switch is still judged against y
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.5, along=0.5),), window_s=0.0))
    tr.update(_frame(0.0, [_candidate(hypothesis="t11_06:x")]))
    tr.update(_frame(0.2, [_candidate(hypothesis="t11_06:y")]))
    tr.update(_frame(0.4, [_candidate(hypothesis="parametric:x")]))
    assert tr.update(_frame(0.6, [_candidate((3.3, 0.0, 0.06), hypothesis="t11_06:x")])).new == 1



# 2B-3h: an in-place layout split of any active visible track is born as its hidden duplicate ----------

WIDE = (_row(1.0, 4.0, lat=0.5, along=0.5),)  # wide position gate, layout bound 0.02 per reading


def _split(dy=0.15, t=0.2, **kw):
    """Track 0 at y 0 (t 0, nothing selected), then a reading whose pockets moved dy sideways with the face
    centre left in place, so each pocket's face coordinate moves dy: a layout split."""
    tr = CandidateTracker(_bounds(rows=WIDE, layout=True, **kw))
    tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x", cid="a")]))
    report = tr.update(_frame(t, [_candidate((3.0, dy, 0.06), hypothesis="parametric:x", face_offset=dy, cid="b")]))
    return tr, report


def test_an_in_place_layout_split_is_born_as_the_tracks_hidden_duplicate():
    tr, report = _split()
    assert report.new == 1 and report.suppressed == 1
    assert (tr._tracks[1].duplicate_of, tr._tracks[1].duplicate_kind, tr._tracks[1].shadow) == (0, "layout", True)
    assert [t.track_id for t in tr.tracks(int(0.2 * S), _odom())] == [0]


def test_a_layout_split_beyond_the_pocket_cap_stays_visible():
    tr, report = _split(dy=0.21)
    assert report.suppressed == 0 and len(tr.tracks(int(0.2 * S), _odom())) == 2


def test_a_layout_split_of_a_stale_track_stays_visible():
    tr, report = _split(t=0.6)
    assert report.suppressed == 0 and [t.track_id for t in tr.tracks(int(0.6 * S), _odom())] == [1]


def test_a_layout_split_of_two_tracks_at_once_stays_visible():
    tr = CandidateTracker(_bounds(rows=WIDE, layout=True))
    tr.update(_frame(0.0, [_candidate((3.0, 0.0, 0.06), hypothesis="parametric:x", cid="a"),
                           _candidate((3.0, 0.1, 0.06), hypothesis="parametric:x", face_offset=0.1, cid="b")]))
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.05, 0.06), hypothesis="parametric:x", face_offset=0.2,
                                               cid="c")]))
    assert report.new == 1 and report.suppressed == 0


def test_another_known_shape_in_place_is_not_a_layout_split():
    tr = CandidateTracker(_bounds(rows=WIDE, layout=True))
    tr.update(_frame(0.0, [_candidate(hypothesis="epal6:x", cid="a")]))
    report = tr.update(_frame(0.2, [_candidate(hypothesis="t11_06:x", cid="b")]))
    assert report.new == 1 and report.suppressed == 0


def test_without_hiding_a_layout_split_stays_visible():
    tr = CandidateTracker(_bounds(rows=WIDE, layout=False))
    tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x", cid="a")]))
    report = tr.update(_frame(0.2, [_candidate((3.0, 0.15, 0.06), hypothesis="parametric:x", face_offset=0.15,
                                               cid="b")]))
    assert report.suppressed == 0 and len(tr.tracks(int(0.2 * S), _odom())) == 2


def test_the_selected_tracks_own_split_keeps_its_reading_kind():
    tr = CandidateTracker(_bounds(rows=WIDE, layout=True, suppress=True))
    tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x", cid="a")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate((3.0, 0.15, 0.06), hypothesis="parametric:x", face_offset=0.15, cid="b")]))
    assert (tr._tracks[1].duplicate_of, tr._tracks[1].duplicate_kind) == (0, "reading")


def test_a_layout_shadow_tracks_on_and_shows_once_its_owner_is_deleted():
    tr, _ = _split()
    for t in (0.4, 0.6, 0.8, 1.0, 1.2):  # only the split reading is seen: the owner (last 0.0 s) is deleted
        report = tr.update(_frame(t, [_candidate((3.0, 0.15, 0.06), hypothesis="parametric:x", face_offset=0.15,
                                                 cid="b")]))
        assert report.accepted == {"b": 1}
    assert [t.track_id for t in tr.tracks(int(1.2 * S), _odom())] == [1]


def test_a_layout_shadow_stays_hidden_while_its_owner_is_observed():
    # the stated limit (Codex P2): release is the owner's deletion only, no time bound
    tr, _ = _split()
    for k in range(2, 11):
        tr.update(_frame(0.2 * k, [_candidate(hypothesis="parametric:x", cid="a"),
                                   _candidate((3.0, 0.15, 0.06), hypothesis="parametric:x", face_offset=0.15,
                                              cid="b")]))
    assert [t.track_id for t in tr.tracks(int(2.0 * S), _odom())] == [0]


def test_a_layout_shadow_gives_its_owner_no_priority_once_the_owner_is_selected():
    # Codex, with the calibrated table: A (spacing 0.40) then B (0.56, layout 80 mm > 64 mm) in place at
    # 1.75 m, nothing selected -> B is A's layout shadow; A is then selected and a reading with spacing
    # 0.48 fits both gates. 2B-3g holds it; an owner priority would hand it to A
    bounds = TrackBounds.load(Path(__file__).resolve().parents[3] / "config/recognition_track_bounds.yaml")
    tr = CandidateTracker(bounds)
    tr.update(_frame(0.0, [_candidate((1.75, 0.0, 0.06), hypothesis="parametric:x", spacing=0.40, cid="a")]))
    tr.update(_frame(0.2, [_candidate((1.75, 0.0, 0.06), hypothesis="parametric:x", spacing=0.56,
                                      cid="b")]))
    assert tr._tracks[1].duplicate_kind == "layout"
    tr.select(0)
    report = tr.update(_frame(0.4, [_candidate((1.75, 0.0, 0.06), hypothesis="parametric:x", spacing=0.48,
                                               range_m=1.0, cid="c")]))
    assert report.accepted == {} and report.held >= 1


def test_a_layout_shadow_of_an_absorbed_duplicate_shows_when_its_direct_owner_goes():
    # S selected; A a visible yaw near miss of S (suppression off); B an in-place layout split of A (its yaw
    # misses S); S takes a reading A also fits, so A is absorbed (a shadow of S). When A is deleted B shows,
    # though S lives: the owner is the direct one
    tr = CandidateTracker(_bounds(rows=(_row(1.0, 4.0, lat=0.5, along=0.5, yaw=0.05),), layout=True))
    tr.update(_frame(0.0, [_candidate(hypothesis="parametric:x", cid="s")]))
    tr.select(0)
    tr.update(_frame(0.2, [_candidate(yaw=0.12, hypothesis="parametric:x", cid="a")]))
    assert tr._tracks[1].duplicate_kind == "near_miss" and not tr._tracks[1].shadow
    tr.update(_frame(0.4, [_candidate((3.0, 0.15, 0.06), 0.12, hypothesis="parametric:x", face_offset=0.15,
                                      cid="b")]))
    assert (tr._tracks[2].duplicate_of, tr._tracks[2].duplicate_kind) == (1, "layout")
    tr.update(_frame(0.6, [_candidate(yaw=0.06, hypothesis="parametric:x", cid="s")]))
    assert tr._tracks[1].shadow  # absorbed
    for t in (0.8, 1.0, 1.2, 1.4):
        tr.update(_frame(t, [_candidate(hypothesis="parametric:x", cid="s"),
                             _candidate((3.0, 0.15, 0.06), 0.12, hypothesis="parametric:x", face_offset=0.15,
                                        cid="b")]))
    assert {t.track_id for t in tr.tracks(int(1.4 * S), _odom())} == {0, 2}


def _record(frames, target=(0, 0, (1, 0))):
    """A minimal sequence record for the evaluator: static robot, truth face of the target pair."""
    face = {"left_m": (1.75, 0.2, 0.06), "right_m": (1.75, -0.2, 0.06), "insertion_yaw_rad": 0.0}
    return {"scene_id": "synthetic", "case": "S7", "shape_key": "nonstandard", "target": target,
            "frames": [{"t_s": t, "pose": (0.0, 0.0, 0.0), "candidates": cands, "labels": labels,
                        "faces": {0: {0: face}}} for t, cands, labels in frames]}


def test_the_evaluator_selects_a_shadow_that_shows_without_a_new_reading():
    # Codex: A (another pair) at 0.0 s only; the target B is A's layout shadow from 0.2 s; at 1.1 s nothing is
    # read, A passes keep_s and is deleted, B shows -- the selection proxy has to pick it there
    from tools import evaluate_track_sequences as E
    bounds = TrackBounds.load(Path(__file__).resolve().parents[3] / "config/recognition_track_bounds.yaml")
    a = _candidate((1.75, 0.0, 0.06), hypothesis="parametric:x", spacing=0.40, cid="a")
    b = _candidate((1.75, 0.0, 0.06), hypothesis="parametric:x", spacing=0.56, cid="b")
    frames = [(0.0, [a], {"a": (0, 0, (2, 1))})]
    frames += [(t, [b], {"b": (0, 0, (1, 0))}) for t in (0.2, 0.4, 0.6, 0.8)]
    frames += [(1.1, [], {})]
    res = E.run(_record(frames), bounds, "V0", {}, seed=0)
    assert not res["rows"][1]["initialised"]  # B is hidden while A lives
    assert res["rows"][-1]["initialised"] and res["rows"][-1]["retained"]
