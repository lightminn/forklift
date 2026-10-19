# tests/unit/perception/test_bottom_deck_evidence.py
"""Bottom-deck side evidence (plan 2026-10-19-bottom-deck-side-evidence.md, task 1)."""

import math
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.perception import bottom_deck_evidence as B
from tools import bottom_deck_scenes as S
from tools import recognition_scenes as rs


def judge(case, **kw):
    scene, target, _ = S.render_side(case)
    return B.evaluate_side(scene, target, **kw)


# -- clean scenes -----------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("shape", ["epal6", "t11_06"])
@pytest.mark.parametrize("theta", [25.0, 35.0, 45.0])
def test_clean_real_present_grounded_absent(shape, theta):
    for rng_m in (1.1, 1.35):
        real = judge(S.SideCase(shape, "pallet", theta_e_deg=theta, range_m=rng_m))
        fake = judge(S.SideCase(shape, "grounded", theta_e_deg=theta, range_m=rng_m))
        assert real.verdict == "present", (rng_m, real)
        assert fake.verdict == "absent", (rng_m, fake)
        assert len(real.gaps) == 2 and all(gp.state == "board" for gp in real.gaps)


def test_t11_y_head_on_face_is_judged():
    """T11 y: the evidence face is the entry face itself, seen head-on (theta_e = 0)."""
    assert judge(S.SideCase("t11_06", "pallet", theta_e_deg=0.0)).verdict == "present"
    assert judge(S.SideCase("t11_06", "grounded", theta_e_deg=0.0)).verdict == "absent"


def test_x_hypothesis_seen_from_the_entry_face_is_unobserved():
    """Camera on the entry face's normal: theta_e = 90 deg on the side face."""
    ev = judge(S.SideCase("epal6", "pallet", theta_e_deg=89.0))
    assert ev.verdict == "unobserved"


def test_closed_face_is_not_evidence_of_absence():
    """Task 2 dev 5 / Codex round 4: a shelf's closed face and a board standing in front of a real face look alike."""
    ev = judge(S.SideCase("epal6", "shelf", theta_e_deg=40.0))
    assert ev.verdict == "unobserved" and ev.reason == "closed_face"
    ev = judge(S.SideCase("epal6", "shelf", theta_e_deg=60.0))       # entry-face view 30 deg
    assert ev.verdict == "unobserved" and ev.theta_face_deg > 50.0
    for offset in (0.014, 0.08):                                    # 4 mm and 70 mm clear of the face
        ev = judge(S.SideCase("t11_06", "pallet", theta_e_deg=0.0, occluder=True, occluder_offset_m=offset))
        assert ev.verdict != "absent", (offset, ev)


def test_slab_reads_present():
    """A continuous slab under the openings is a band at the board height: accepted (plan section 5 limit)."""
    assert judge(S.SideCase("epal6", "slab")).verdict == "present"


def test_entry_view_50_deg_passes_arrival_and_reads_present():
    """Codex v7.1: theta_e (centre based) 40 deg while theta_face is near 50 deg."""
    for shape in ("epal6", "t11_06"):
        case = S.SideCase(shape, "pallet", theta_e_deg=40.0)
        scene, target, _ = S.render_side(case)
        ev = B.evaluate_side(scene, target)
        assert ev.verdict == "present"
        theta = B.stand_angle_deg(np.asarray(scene.base_from_optical.translation_m)[:2], target, face=1)
        assert theta == pytest.approx(40.0, abs=0.5)
        assert B.arrival_ok(scene, target, face=1, axis="x")
        assert ev.theta_face_deg > 45.0


# -- which face ---------------------------------------------------------------------------------------------------------

def test_face_reported_and_wrong_face_refused():
    case = S.SideCase("epal6", "pallet", face=-1)
    scene, target, _ = S.render_side(case)
    ev = B.evaluate_side(scene, target)
    assert ev.face == -1 and ev.verdict == "present"
    assert B.evaluate_side(scene, target, face=1).reason == "wrong_face"


def test_gaps_follow_the_spec():
    """A spec with the rows moved puts the expected gaps elsewhere: the real pallet no longer matches."""
    case = S.SideCase("epal6", "pallet")
    scene, target, _ = S.render_side(case)
    spec = target.spec
    moved = replace(spec, row_centres_x_m=(spec.row_centres_x_m[0], spec.row_centres_x_m[1] + 0.06,
                                           spec.row_centres_x_m[2]))
    ev = B.evaluate_side(scene, replace(target, spec=moved))
    assert ev.verdict != "present"


# -- pose and extrinsic errors ------------------------------------------------------------------------------------------

@pytest.mark.parametrize("err", [(0.1, 0.0, 0.0), (-0.1, 0.03, 5.0), (0.06, -0.03, -5.0)])
def test_candidate_error_keeps_the_verdict(err):
    a, l, y = err
    kw = dict(along_err_m=a, lateral_err_m=l, yaw_err_deg=y)
    assert judge(S.SideCase("epal6", "pallet", **kw)).verdict == "present"
    assert judge(S.SideCase("epal6", "grounded", **kw)).verdict == "absent"


@pytest.mark.parametrize("tilt,height,lift", [(1.0, 0.01, 0.0), (-1.0, -0.01, 0.0), (0.0, 0.0, -0.009)])
def test_grounded_never_present_under_extrinsic_and_support_error(tilt, height, lift):
    case = S.SideCase("epal6", "grounded", tilt_err_deg=tilt, height_err_m=height, lift_m=max(lift, 0.0),
                      support_z_m=lift)
    assert judge(case).verdict != "present"


def test_raised_support_disagrees_with_the_marker():
    """The structure stands 8 mm above the floor in front: the floor plane and the pallet's own marker disagree."""
    ev = judge(S.SideCase("epal6", "pallet", lift_m=0.008))
    assert ev.verdict == "unobserved" and ev.reason == "floor_marker_disagree"


# -- clutter ------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("sign", [1, -1])
def test_coplanar_neighbour(sign):
    assert judge(S.SideCase("epal6", "grounded", neighbour=sign)).verdict != "present"
    assert judge(S.SideCase("epal6", "pallet", neighbour=sign)).verdict != "absent"


@pytest.mark.parametrize("plate", [S.Plate(0.02, 0.8, 0.15, 0.3), S.Plate(0.01, 0.8, 0.15, 0.1),
                                   S.Plate(0.02, 1.0, 0.42, 0.03)])
def test_low_plate_in_front(plate):
    assert judge(S.SideCase("epal6", "grounded", plate=plate)).verdict != "present"
    assert judge(S.SideCase("epal6", "pallet", plate=plate)).verdict != "absent"


def test_ledge_in_one_gap_is_not_a_board():
    """Codex v6 P1: one faked gap must not carry the other."""
    ev = judge(S.SideCase("epal6", "grounded", theta_e_deg=35.0, range_m=1.1, ledges=(S.Ledge(0, 0.022, 0.003),)))
    assert ev.verdict != "present"
    ev = judge(S.SideCase("epal6", "grounded", ledges=(S.Ledge(0, 0.010, 0.003), S.Ledge(1, 0.022, 0.003))))
    assert ev.verdict != "present"


@pytest.mark.parametrize("h", [0.004, 0.035])
def test_ledges_outside_the_band_are_not_boards(h):
    """Both gaps faked, but at a height outside the bottom-board band (-9 / +11 mm)."""
    ledges = (S.Ledge(0, h, 0.005), S.Ledge(1, h, 0.005))
    assert judge(S.SideCase("epal6", "grounded", ledges=ledges)).verdict != "present"


def test_ledges_in_front_of_a_real_board_do_not_make_it_absent():
    ledges = (S.Ledge(0, 0.016, 0.01), S.Ledge(1, 0.016, 0.01))
    assert judge(S.SideCase("epal6", "pallet", ledges=ledges)).verdict != "absent"


def test_occluder_is_unobserved():
    assert judge(S.SideCase("epal6", "pallet", occluder=True)).verdict == "unobserved"
    assert judge(S.SideCase("epal6", "grounded", occluder=True)).verdict != "present"


def test_stacked_target_is_not_judged():
    ev = judge(S.SideCase("epal6", "pallet", stacked=True, support_z_m=0.144))
    assert ev.verdict == "unobserved" and ev.reason == "not_on_floor"


@pytest.mark.parametrize("shape", ["epal6", "t11_06"])
@pytest.mark.parametrize("support", [math.nan, math.inf])
def test_unknown_support_is_not_on_the_floor(shape, support):
    """Codex task-1 P2: abs(NaN) > limit is False -- a stacked grounded read the pallet below as its own deck."""
    ev = judge(S.SideCase(shape, "grounded", stacked=True, support_z_m=support))
    assert ev.verdict == "unobserved" and ev.reason == "not_on_floor"


# -- noise ------------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("smooth", [False, True])
def test_team_noise(smooth):
    for seed in range(2):
        real = judge(S.SideCase("epal6", "pallet", noise_seed=2000 + seed, smooth=smooth))
        fake = judge(S.SideCase("epal6", "grounded", noise_seed=2000 + seed, smooth=smooth))
        assert real.verdict == "present", real
        assert fake.verdict != "present", fake


def test_median_matches_scipy_median_filter():
    """Codex task-1 P2: the spikes smoothed with scipy's median_filter (invalid pixels as 0, border mode reflect)."""
    ndimage = pytest.importorskip("scipy.ndimage")
    rng = np.random.default_rng(3)
    d = rng.uniform(0.5, 2.0, (37, 41))
    d[rng.random(d.shape) < 0.1] = np.nan
    d[:3, :] = np.nan                                      # invalid rows at the border
    want = ndimage.median_filter(np.where(np.isfinite(d), d, 0.0), size=5)
    got = S._median5(d)
    finite = np.isfinite(d)
    assert np.array_equal(np.isfinite(got), finite)
    assert np.array_equal(got[finite], want[finite])


def test_deterministic():
    case = S.SideCase("t11_06", "pallet", noise_seed=2001)
    a, b = judge(case), judge(case)
    assert a == b


# -- floor plane --------------------------------------------------------------------------------------------------------

def test_final_floor_plane_slope_is_checked():
    """Codex v6 P2: a 1.7 deg plane whose RANSAC samples fit under 1.5 deg must still be refused."""
    rng = np.random.default_rng(0)
    s = rng.uniform(-0.5, 0.5, 2000)
    o = rng.uniform(0.03, 0.45, 2000)
    z = math.tan(math.radians(1.7)) * s + rng.normal(0.0, 0.0015, 2000)
    fit = B.dominant_floor(s, o, z, B.BottomDeckParams())
    assert fit is None or fit.tilted
    z_ok = math.tan(math.radians(0.5)) * s + rng.normal(0.0, 0.0015, 2000)
    ok = B.dominant_floor(s, o, z_ok, B.BottomDeckParams())
    assert ok is not None and not ok.tilted and not ok.ambiguous


def test_floor_with_a_second_surface_is_ambiguous():
    rng = np.random.default_rng(1)
    s, o = rng.uniform(-0.5, 0.5, 1000), rng.uniform(0.03, 0.45, 1000)
    z = np.where(o > 0.3, 0.02, 0.0) + rng.normal(0.0, 0.001, 1000)
    fit = B.dominant_floor(s, o, z, B.BottomDeckParams())
    assert fit is not None and fit.ambiguous


def test_four_rows_and_uneven_gaps_from_the_spec():
    """Codex task-1 P3: the row / gap matching follows the spec in the positive direction too."""
    base = rs.known_spec("epal6")
    spec = replace(base, overall_depth_m=0.9, row_depth_m=0.1, row_centres_x_m=(-0.4, -0.12, 0.13, 0.4))
    assert B.nominal_gaps(spec) == pytest.approx((0.18, 0.15, 0.17))
    real = judge(S.SideCase("epal6", "pallet", spec=spec))
    fake = judge(S.SideCase("epal6", "grounded", spec=spec))
    assert real.verdict == "present" and len(real.gaps) == 3, real
    assert fake.verdict == "absent", fake


def test_result_carries_the_refitted_face():
    ev = judge(S.SideCase("epal6", "pallet"))
    px, py, nx, ny = ev.plane
    assert math.hypot(nx, ny) == pytest.approx(1.0)


def test_params_are_from_the_spec_not_constants():
    spec = rs.known_spec("t11_06")
    gaps = B.nominal_gaps(spec)
    assert len(gaps) == 2 and gaps[0] == pytest.approx(0.21, abs=1e-6)


@pytest.mark.parametrize("structure,want", [("pallet", "present"), ("grounded", "absent")])
def test_low_t11_band_survives_camera_tilt_and_height_error(structure, want):
    """Task 2 dev: +0.9 deg tilt with -7 mm height slid T11's 27 mm block band onto the bottom board (absolute z)."""
    case = S.SideCase("t11_06", structure, face=-1, tilt_err_deg=0.91, height_err_m=-0.0074,
                      stand_dy_m=0.045, heading_err_deg=1.7, along_err_m=0.043, yaw_err_deg=-4.5)
    assert judge(case).verdict == want


def test_unknown_bins_do_not_split_a_row():
    assert B._row_runs(list("OOSS????SSSSOOO"), 0.5) == [["S", 2, 12]]
    assert B._row_runs(list("OOSSOSSSOO"), 0.5) == [["S", 5, 8]]          # an open bin splits; 2 bins too short
    assert B._row_runs(list("O??S????OO"), 0.5) == []                     # mostly unknown: not a row
