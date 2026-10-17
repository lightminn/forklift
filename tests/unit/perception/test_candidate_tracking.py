# tests/unit/perception/test_candidate_tracking.py
"""Far-range candidate tracking and fusion (stage 2B-3 plan sections 1-5)."""
import math
import random
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


def _bounds(*, rows=None, window_s=1.2, e=0.0, psi=0.0, rear_axle=(0.0, 0.0)):
    rows = rows or (_row(1.0, 2.5), _row(2.5, 4.0))
    return TrackBounds(rows={"known": rows, "parametric": rows}, ages_s=(0.1, 0.5, 1.0, 10.0),
                       e_m=(e, e, e, e), psi_rad=(psi, psi, psi, psi), rear_axle_xy_m=rear_axle,
                       window_s=window_s, max_observations=10, expiry_s=0.5)


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


def test_a_switch_between_known_shape_and_parametric_reading_starts_a_new_track():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    report = tr.update(_frame(0.2, [_candidate(hypothesis="parametric:x")]))
    assert report.new == 1


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


def test_an_expired_track_takes_no_observation_and_the_pallet_gets_a_new_track():
    tr = CandidateTracker(_bounds())
    tr.update(_frame(0.0, [_candidate()]))
    report = tr.update(_frame(0.6, [_candidate()]))
    assert report.new == 1 and len(tr.tracks(int(0.6 * S), _odom())) == 1


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
