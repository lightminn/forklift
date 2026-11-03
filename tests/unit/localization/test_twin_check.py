"""The start check of the unconfirmed position plan (docs/plans/2026-11-02-unconfirmed-position.md 1.2): a believed
start b, its hypothesis set (the matcher's pool, the last global search's pre-suppression leaves, synthetic bay shifts
and turnarounds) and the residual ambiguity, judged conservatively -- every lattice leaf in each hypothesis's region,
unknown map cells counted as hits, a region dropped only when no pose in it is possible."""

import math

import numpy as np
import pytest
import test_global_match as TG

from forklift_core.localization import global_match as G
from forklift_core.localization import start_reloc as R
from forklift_core.localization import twin_check as T
from forklift_core.localization.slam_map import Grid
from forklift_core.localization.slam_pose import compose, invert

CFG = G.MatchConfig(yaw_step_rad=math.radians(2.0))


def _points_at(grid, base, **kw):
    return TG._points(grid, R.laser_from_base(base), **kw)


SHORT = {"range_max": 2.0}  # a short scan: the hall's end walls out of reach, so neighbouring bays look the same


def _bays(width_bays=2, unknown_from_x=None):
    """A hall of identical bays 6 m apart along x (a shelf block in each), optionally unknown beyond ``unknown_from_x``
    (survey value -1) -- the same picture from either bay unless the unknown part hides the difference."""
    boxes = []
    for k in range(width_bays):
        x = 6.0 * k
        boxes += [(x + 1.0, 2.8, x + 2.0, 3.6), (x + 3.5, 0.5, x + 4.5, 1.2)]
    g = TG._grid(6.0 * width_bays, 4.0, boxes)
    if unknown_from_x is not None:
        data = g.data.copy()
        data[:, int(unknown_from_x / TG.RES):] = -1
        g = Grid(data, g.origin, g.resolution_m)
    return g


def test_the_synthetic_set_is_the_bay_shifts_and_the_turnarounds():
    b = (1.0, 2.0, 0.3)
    syn = T.synthetic_twins(b, bay_m=6.0, multiples=(1, 2, 3))
    shifts = {(round(p[0], 6), round(p[1], 6)) for p, _ in syn}
    assert (7.0, 2.0) in shifts and (-17.0, 2.0) in shifts and (1.0, 20.0) in shifts and (1.0, 2.0) in shifts
    assert len(syn) == 25  # 12 shifts, each turned around too, and b turned around
    flips = [p for p, src in syn if src.startswith("turn")]
    assert all(abs(T.wrap(p[2] - (0.3 + math.pi))) < 1e-9 for p in flips) and len(flips) == 13


def test_hypotheses_drop_only_exact_repeats_so_the_union_of_regions_is_kept():
    # Codex piece 1 re-review: two centres 0.08 m apart do not search the same +-0.30 m region, so a pose within the
    # success tolerance of b or of another hypothesis is still its own region; only exact repeats go
    b = (1.0, 2.0, 0.0)
    hyps = T.hypotheses(b, pool=[(1.0, 2.0, 0.0), (1.05, 2.0, 0.0), (1.2, 2.0, 0.0)], leaves=[(1.2, 2.0, 0.0),
                        (1.2, 2.0, 0.01)], synthetic=[])
    poses = [p for p, _ in hyps]
    assert (1.0, 2.0, 0.0) not in poses  # b itself
    assert poses == [(1.05, 2.0, 0.0), (1.2, 2.0, 0.0), (1.2, 2.0, 0.01)]


def test_an_asymmetric_room_has_no_residual_ambiguity():
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    res = T.residual_ambiguity(grid, _points_at(grid, truth), truth, T.synthetic_twins(truth), CFG, e_min=0.8)
    assert res.confirmed and not res.competitors, res
    assert res.impossible  # the bay shifts fall outside the 6 m x 4 m room: impossible, recorded, not competing


def test_identical_bays_leave_the_twin_as_a_competitor():
    grid = _bays(3)
    truth = (8.5, 2.0, 0.0)  # the middle bay
    res = T.residual_ambiguity(grid, _points_at(grid, truth, **SHORT), truth, T.synthetic_twins(truth), CFG,
                               e_min=0.8)
    assert not res.confirmed and "competitor" in res.reasons
    assert any(abs(p[0] - 2.5) < 0.31 or abs(p[0] - 14.5) < 0.31 for p, _, _ in res.competitors)


def test_a_twin_hidden_by_unknown_cells_still_competes():
    # the second bay is unknown on the survey map: a plain matcher would see only the first bay; the conservative check
    # counts unknown endpoints as hits, so the hidden twin's region still scores as high as the start (Codex draft 5)
    grid = _bays(2, unknown_from_x=6.0)  # the second bay unknown: no free cell there for a plain matcher
    truth = (2.5, 2.0, 0.0)
    pts = _points_at(_bays(2), truth, **SHORT)
    res = T.residual_ambiguity(grid, pts, truth, T.synthetic_twins(truth), CFG, e_min=0.8)
    assert not res.confirmed and "competitor" in res.reasons
    assert any(abs(p[0] - 8.5) < 0.31 for p, _, _ in res.competitors)  # the hidden twin itself competes


def test_a_believed_start_that_does_not_fit_is_unconfirmed():
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    wrong = (3.4, 2.2, 0.6)  # a metre off: the local re-match around it does not fit
    res = T.residual_ambiguity(grid, _points_at(grid, truth), wrong, [], CFG, e_min=0.8)
    assert not res.confirmed and res.reasons and res.reasons[0].startswith("believed:")


def test_a_search_that_cannot_finish_leaves_the_start_unconfirmed():
    grid = _bays(3)
    truth = (8.5, 2.0, 0.0)
    res = T.residual_ambiguity(grid, _points_at(grid, truth, **SHORT), truth, T.synthetic_twins(truth),
                               G.MatchConfig(yaw_step_rad=math.radians(2.0), max_work=1), e_min=0.8)
    assert not res.confirmed and res.reasons


# --- the re-match after a straight reverse (plan 1.5) ---
def test_hypotheses_move_by_the_odometry_step_in_their_own_frame():
    o0, ot = (10.0, 5.0, 0.0), (9.0, 5.0, 0.0)  # one metre straight back
    assert T.moved((2.0, 2.0, 0.0), o0, ot) == (1.0, 2.0, 0.0)
    x, y, yaw = T.moved((2.0, 2.0, math.pi), o0, ot)  # a turned-around twin moves the other way on the map
    assert abs(x - 3.0) < 1e-9 and abs(y - 2.0) < 1e-9


def _rematch(grid, truth_now, b, hyps, decision, g, **kw):
    o0 = (0.0, 0.0, 0.0)
    ot = kw.pop("ot", (-0.6, 0.0, 0.0))
    return T.rematch_decision(grid, _points_at(grid, truth_now, **kw.pop("scan", {})), b, hyps, o0, ot,
                              decision=decision, g=g, cfg=CFG, e_min=0.8, **kw)


def test_a_rematch_agreeing_with_the_moved_start_confirms_where_nothing_competes():
    grid = TG._grid(*TG.ROOM)
    b = (3.0, 1.6, 0.0)
    now = (2.4, 1.6, 0.0)  # backed up 0.6 m
    out = _rematch(grid, now, b, [], "confirmed", now)
    assert out.outcome == "confirm" and out.residual.confirmed


def test_a_rematch_on_a_moved_twin_prefers_the_twin_and_one_elsewhere_ends_the_session():
    grid = TG._grid(*TG.ROOM)
    b = (3.0, 1.6, 0.0)
    twin = (3.0, 2.6, 0.0)
    out = _rematch(grid, (2.4, 2.6, 0.0), b, [(twin, "pool")], "confirmed", (2.4, 2.6, 0.0))
    assert out.outcome == "twin_preferred" and out.matched == "pool"
    out = _rematch(grid, (4.4, 1.0, 0.0), b, [(twin, "pool")], "confirmed", (4.4, 1.0, 0.0))
    assert out.outcome == "out_of_set"


def test_a_rematch_that_did_not_confirm_or_is_still_ambiguous_stays_unconfirmed():
    grid = TG._grid(*TG.ROOM)
    b = (3.0, 1.6, 0.0)
    for decision in ("competitor", "periodic", "no_candidate", "timeout", "uninformative"):
        assert _rematch(grid, (2.4, 1.6, 0.0), b, [], decision, None).outcome == "unconfirmed"
    bays = _bays(3)
    b2 = (9.1, 2.0, 0.0)
    out = _rematch(bays, (8.5, 2.0, 0.0), b2, T.synthetic_twins(b2), "confirmed", (8.5, 2.0, 0.0), scan=SHORT)
    assert out.outcome == "unconfirmed" and not out.residual.confirmed


def test_the_rematch_waits_for_d_min_and_the_agreement_allows_for_both_estimates_and_the_odometry():
    grid = TG._grid(*TG.ROOM)
    b = (3.0, 1.6, 0.0)
    assert _rematch(grid, (2.8, 1.6, 0.0), b, [], "confirmed", (2.8, 1.6, 0.0), ot=(-0.2, 0.0, 0.0)).outcome == "too_short"
    # g 0.18 m from the moved start: inside 0.20 m + 0.05 d (d = 0.6) with a zero odometry bound
    out = _rematch(grid, (2.4, 1.6, 0.0), b, [], "confirmed", (2.4, 1.78, 0.0))
    assert out.agree_m is not None and out.agree_m < 0.23 and out.outcome in ("confirm", "unconfirmed")
    assert out.outcome != "out_of_set"


# --- Codex piece 1 review ---
def test_region_leaves_are_every_lattice_leaf_above_the_absolute_threshold():
    # P1: pruning relative to the search's own best (keep_ratio) lost an in-region competitor when the best stood
    # outside the region; the threshold is absolute (R_RESID * s(b)), checked here against an exhaustive search
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    pts = G._thin(_points_at(grid, truth), grid.resolution_m)
    cons = T.ConservativeIndex(grid, CFG)
    region = (2.6, 1.7, 0.6)
    thr = 0.3
    bases, scores = T._region_leaves(grid, pts, CFG, cons, region, G._Budget(CFG), thr)
    s = G._Search(grid, pts, CFG, cons, G._Budget(CFG))
    rows, cols = np.nonzero(s.cand)
    want = set()
    for h in range(s.n_yaw):
        sc = s.scores(0, np.full(len(rows), h), rows, cols)
        for k in np.flatnonzero(sc >= thr):
            base = T.L._bases(np.array([s.pose(h, rows[k], cols[k])]))[0]
            if math.hypot(base[0] - region[0], base[1] - region[1]) <= T.REGION_M and \
                    abs(T.wrap(base[2] - region[2])) <= T.REGION_RAD + 1e-9:
                want.add((round(base[0], 6), round(base[1], 6), round(base[2], 6)))
    got = {(round(b[0], 6), round(b[1], 6), round(b[2], 6)) for b in bases}
    assert got == want and len(got) > 0
    assert all(sc >= thr for sc in scores)


def test_a_failed_hypothesis_search_leaves_the_start_unconfirmed():
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    res = T.residual_ambiguity(grid, _points_at(grid, truth), truth, [], CFG, e_min=0.8, hyp_status="overflow")
    assert not res.confirmed and "hypotheses:overflow" in res.reasons
    wrong = (3.4, 2.2, 0.6)  # the believed start fails too: both reasons are kept (Codex re-review P3)
    res = T.residual_ambiguity(grid, _points_at(grid, truth), wrong, [], CFG, e_min=0.8, hyp_status="overflow")
    assert res.reasons[0] == "hypotheses:overflow" and res.reasons[1].startswith("believed:")


def test_the_rematch_keeps_the_moved_starts_own_region():
    # P2: the moved start b' is a hypothesis at the new place too (g agrees with it only within E_AGREE), even when g
    # and b' are within each other's success tolerance (Codex re-review: 0.08 m apart merged it away)
    grid = TG._grid(*TG.ROOM)
    b = (3.0, 1.6, 0.0)
    for g in ((2.4, 1.78, 0.0), (2.48, 1.6, 0.0)):
        out = _rematch(grid, (2.4, 1.6, 0.0), b, [], "confirmed", g)
        assert out.residual is not None
        assert any(src == "moved-start" and T._same(p, (2.4, 1.6, 0.0)) for p, src in out.residual.searched)


def test_the_residual_searches_the_believed_start_as_well_as_its_refined_pose():
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    b = (2.47, 1.6, 0.6)  # refined towards the truth: two different regions
    res = T.residual_ambiguity(grid, _points_at(grid, truth), b, [], CFG, e_min=0.8)
    assert res.searched[0][1] == "believed" and (b, "believed-start") in res.searched


# --- piece 2 (docs/plans/2026-11-02-unconfirmed-runner.md 2.1) ---
from forklift_core.localization import pose_check as PC  # noqa: E402
from forklift_core.safety.pallet_permit import Zone  # noqa: E402

BOUNDS0 = PC.Bounds(odom=(0.0, 0.0, 0.0, 0.0), e_still=(0.03, 0.01), e_slam=(0.05, 0.025))


def test_last_leaves_honours_its_deadline():
    # Codex draft 1 P2-7: the config's deadline_s alone does not stop the search; the clock and deadline do
    grid = TG._grid(*TG.ROOM)
    pts = _points_at(grid, (2.4, 1.6, 0.6))
    status, leaves = T.last_leaves(grid, pts, CFG, G.MapIndex(grid, CFG), 0.98)
    assert status == "ok" and leaves
    ticks = iter(range(10**6))
    status, leaves = T.last_leaves(grid, pts, CFG, G.MapIndex(grid, CFG), 0.98, clock=lambda: next(ticks),
                                   deadline=3)
    assert status == "timeout" and leaves == []


def test_the_start_check_takes_the_generations_leaves_and_the_live_ones():
    grid = _bays(3)
    truth = (8.5, 2.0, 0.0)
    live = _points_at(grid, truth, **SHORT)
    gen = _points_at(grid, (2.5, 2.0, 0.0), **SHORT)  # another place: its leaves are other poses
    out = T.start_check(grid, live, truth, pool=[], gen_points=gen, cfg=CFG, e_min=0.8)
    assert not out.residual.confirmed
    index = G.MapIndex(grid, CFG)
    for pts in (live, gen):  # every leaf of both searches is a hypothesis (or b itself); exact repeats are one
        _, leaves = T.last_leaves(grid, pts, CFG, index, 0.98)
        assert leaves and all(T._same(p, truth) or any(T._same(p, q) for q, _ in out.hyps) for p in leaves)
    carried = T.start_check(grid, live, truth, pool=[(14.5, 2.0, 0.0)], gen_points=gen, cfg=CFG, e_min=0.8,
                            carry=lambda p: (p[0] + 0.2, p[1], p[2]))
    assert any(abs(p[0] - 14.7) < 1e-9 and src == "pool" for p, src in carried.hyps)  # moved once to O_scan's time


def test_a_failed_generation_search_leaves_the_start_unconfirmed():
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    pts = _points_at(grid, truth)
    out = T.start_check(grid, pts, truth, pool=[], gen_points=pts, cfg=G.MatchConfig(yaw_step_rad=math.radians(2.0),
                        max_work=1), e_min=0.8)
    assert not out.residual.confirmed


def test_odometry_zones_are_c_h_inverse_z_and_do_not_move_with_the_odometry():
    z = Zone("p", 5.0, 0.0, 1.0, 0.8, 0.0)
    o_s = (10.0, 3.0, math.pi / 2)
    (zone, rho0), = T.odom_zones([z], [(2.0, 0.0, 0.0)], o_s)
    # the pallet 3 m ahead of the hypothesis, so 3 m ahead of the truck at o_s (facing +y in odom)
    assert (zone.x_m, zone.y_m) == pytest.approx((10.0, 6.0)) and zone.yaw_rad == pytest.approx(math.pi / 2)
    assert rho0 == pytest.approx(math.hypot(3.5, 0.4))
    (turned, _), = T.odom_zones([z], [(2.0, 0.0, math.pi)], o_s)  # a turned-around hypothesis: behind the truck
    assert (turned.x_m, turned.y_m) == pytest.approx((10.0, 0.0))
    # the map pose c = C o O_t: in map coordinates the zone is C o Z, the same as c o h_t^-1 o z with h_t carried
    C = (1.0, -2.0, 0.3)
    o_t = compose(o_s, (-0.7, 0.0, 0.0))
    h_t = T.moved((2.0, 0.0, 0.0), o_s, o_t)
    via_map = compose(compose(compose(C, o_t), invert(h_t)), (5.0, 0.0, 0.0))
    via_odom = compose(C, (zone.x_m, zone.y_m, zone.yaw_rad))
    assert via_map == pytest.approx(via_odom)


def test_the_growth_is_the_designs():
    z = Zone("p", 0.0, 0.0, 1.0, 1.0, 0.0)
    (grown,) = T.grown_zones([(z, 1.0 - 2.5)], 2.5, BOUNDS0)  # rho bound rho0 + d+ = 1.0
    g = (grown.length_m - 1.0) / 2
    assert g == pytest.approx(0.649854, abs=1e-6)  # design 1.4's 0.65 m, Codex's 0.649854
    b = PC.Bounds(odom=(0.01, 0.02, 0.001, 0.002), e_still=(0.03, 0.01), e_slam=(0.05, 0.025))
    (g2,) = T.grown_zones([(z, 1.0)], 0.5, b)
    want = 0.30 + 0.02 + 2 * 0.5 * math.sin(0.05) + 2 * 1.5 * math.sin((0.10 + 0.002) / 2)
    assert (g2.length_m - 1.0) / 2 == pytest.approx(want) and (g2.width_m - 1.0) / 2 == pytest.approx(want)


def test_a_start_check_without_the_generations_scans_is_unconfirmed():
    # Codex piece 2a P2: a missing generation set is no evidence
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    out = T.start_check(grid, _points_at(grid, truth), truth, pool=[], gen_points=None, cfg=CFG, e_min=0.8)
    assert not out.residual.confirmed and out.statuses["generation"] == "missing"


def test_the_start_check_drops_the_hypotheses_it_proved_impossible():
    grid = TG._grid(*TG.ROOM)
    truth = (2.4, 1.6, 0.6)
    pts = _points_at(grid, truth)
    out = T.start_check(grid, pts, truth, pool=[], gen_points=pts, cfg=CFG, e_min=0.8)
    assert out.residual.impossible
    gone = {tuple(p) for p, _ in out.residual.impossible}
    assert out.hyps and not any(tuple(p) in gone for p, _ in out.hyps)
