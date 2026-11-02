"""The start check of the unconfirmed position plan (docs/plans/2026-11-02-unconfirmed-position.md 1.2): a believed
start b, its hypothesis set (the matcher's pool, the last global search's pre-suppression leaves, synthetic bay shifts
and turnarounds) and the residual ambiguity, judged conservatively -- every lattice leaf in each hypothesis's region,
unknown map cells counted as hits, a region dropped only when no pose in it is possible."""

import math

import test_global_match as TG

from forklift_core.localization import global_match as G
from forklift_core.localization import start_reloc as R
from forklift_core.localization import twin_check as T
from forklift_core.localization.slam_map import Grid

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


def test_hypotheses_merge_only_within_the_success_tolerance():
    b = (1.0, 2.0, 0.0)
    hyps = T.hypotheses(b, pool=[(1.05, 2.0, 0.0), (1.2, 2.0, 0.0)], leaves=[(1.2, 2.0, 0.01)], synthetic=[])
    poses = [p for p, _ in hyps]
    assert (1.05, 2.0, 0.0) not in poses  # within b's success tolerance: b itself
    assert len([p for p in poses if abs(p[0] - 1.2) < 0.02]) == 1  # the two 1.2 m poses are one hypothesis
    assert len(hyps) == 1


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
