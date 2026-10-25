"""The carried load's slip watch (docs/plans/2026-10-25-slip-check-relative.md): the forks in the pockets, the outline's
motion from the insert-to-lift instant and the absolute along check, with the old absolute lateral kept as a record."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SW = _load("slip_watch", "sim/isaac/slip_watch.py")
IG = _load("insertion_geometry", "sim/isaac/insertion_geometry.py")
GEOMETRY = IG.InsertionGeometry.from_urdfs(ROOT / "sim/models/dls08_measured/forklift.urdf",
                                           ROOT / "sim/models/epal6_pallet/pallet.urdf")
ALONG = 0.93      # inserted_offset - |rear_axle_offset| for the tests (the runner passes its own)
LIFT, PALLET_Z = 0.165, 0.1175  # lifted: the pallet origin rides 0.1175 m up (the runs' pz 0.118)


def _q(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def gaps_at(rel, lift=LIFT, pz=PALLET_Z):
    """The real pocket side gaps for a pallet at rel = (along, lateral, yaw) in the truck's base frame (base at the
    origin, heading 0)."""
    x, y, a = rel
    return GEOMETRY.pocket_side_gaps((0.0, 0.0, 0.0), _q(0.0), lift, (x, y, pz), _q(a))


def watch():
    return SW.SlipWatch(inserted_along_m=ALONG, outline_half_m=(0.3, 0.4))


def carried(w, rel, *, summary=None, phase="transport", t=1.0, gaps=None, base=(0.0, 0.0, 0.0)):
    """One watched tick with the pallet at rel in the base frame of a truck at ``base`` (world x, y, yaw)."""
    bx, by, byaw = base
    x, y, a = rel
    c, s = math.cos(byaw), math.sin(byaw)
    pallet = (bx + c * x - s * y, by + s * x + c * y, math.atan2(math.sin(byaw + a), math.cos(byaw + a)))
    summary = {} if summary is None else summary
    return w.check(summary, t=t, phase=phase, base=base, pallet=pallet,
                   gaps=gaps_at(rel) if gaps is None else gaps), summary


def lifted(w, seat, base=(0.0, 0.0, 0.0)):
    """The insert-to-lift instant with the pallet at ``seat`` (base frame)."""
    bx, by, byaw = base
    x, y, a = seat
    c, s = math.cos(byaw), math.sin(byaw)
    pallet = (bx + c * x - s * y, by + s * x + c * y, math.atan2(math.sin(byaw + a), math.cos(byaw + a)))
    w.end_tick("insert", base, pallet, t=0.0)
    w.end_tick("lift", base, pallet, t=0.0)


# The three recorded "slips" (joint-angle promotion diagnosis 2.3): the seat at lift, never moved.
RECORDED = [
    pytest.param((0.9286, -0.0476, -0.0157), id="newtq_random_m_s3_P-5"),
    pytest.param((0.929, -0.0509, -0.0163), id="reptq_random_m_s1_r2_P-5"),
    pytest.param((0.9467, 0.0585, 0.0179), id="newrp_line_m_s1_P-2"),
]


@pytest.mark.parametrize("seat", RECORDED)
def test_a_recorded_seat_that_never_moved_passes_and_the_old_check_is_kept_as_failing(seat):
    w = watch()
    lifted(w, seat)
    fail, summary = carried(w, seat, phase="extract")
    assert fail is None
    assert summary["old_lateral_max_m"] == pytest.approx(abs(seat[1]), abs=1e-9)
    assert summary["old_would_fail_s"] == 1.0
    assert summary["outline_motion_max_m"] == pytest.approx(0.0, abs=1e-9)
    assert summary["baseline"]["seat"] == pytest.approx(list(seat), abs=1e-9)
    assert min(summary["pocket_gap_min_m"].values()) > SW.POCKET_MARGIN_M


def test_a_seat_offset_and_a_sideways_slide_into_a_block_fails_on_the_pocket_gap():
    """Codex slip plan review round 1: 35 mm seated, then 39 mm and -0.049 rad more -- within the motion bound,
    about 4 mm into a block."""
    w = watch()
    seat = (ALONG, 0.035, 0.0)
    lifted(w, seat)
    fail, summary = carried(w, (ALONG, 0.035 + 0.039, -0.049))
    assert fail == "pocket_clearance"
    assert summary["failure"]["reason"] == "pocket_clearance"


def test_the_same_slide_without_the_turn_stays_in_the_pockets_and_within_the_motion_bound():
    """The boundary twin: 35 + 39 mm with no yaw keeps about 12 mm of pocket and moves the outline 39 mm."""
    w = watch()
    lifted(w, (ALONG, 0.035, 0.0))
    fail, summary = carried(w, (ALONG, 0.074, 0.0))
    assert fail is None
    assert summary["outline_motion_max_m"] == pytest.approx(0.039, abs=1e-9)


def test_a_blade_outside_the_pallet_fails_even_when_seated_there():
    """Codex slip plan review round 2: a seat with one blade beyond the pallet's side -- no motion at all."""
    w = watch()
    seat = (0.92, 0.40, 0.0)
    lifted(w, seat)
    fail, _ = carried(w, seat)
    assert fail == "pocket_clearance"


@pytest.mark.parametrize("angle, expect", [(0.080, None), (0.081, "outline_motion")])
def test_a_turn_about_the_pallet_origin_fails_on_the_outline_motion(angle, expect):
    """The corners lie 0.5 m from the origin: 0.08 rad moves them 39.99 mm, 0.081 rad 40.5 mm. The old lateral
    (the origin's) does not move at all."""
    w = watch()
    lifted(w, (ALONG, 0.0, 0.0))
    fail, summary = carried(w, (ALONG, 0.0, angle), gaps={"left": (0.05, 0.05), "right": (0.05, 0.05)})
    assert fail == expect
    assert summary["old_would_fail_s"] is None


@pytest.mark.parametrize("slide, expect", [(0.039, None), (0.041, "outline_motion")])
def test_a_slide_along_the_forks_fails_on_the_outline_motion_before_the_absolute_along(slide, expect):
    w = watch()
    lifted(w, (ALONG, 0.0, 0.0))
    fail, _ = carried(w, (ALONG - slide, 0.0, 0.0))
    assert fail == expect


def test_the_absolute_along_check_stands():
    """Without a baseline (a load the watch never saw lifted) the along check still applies."""
    w = watch()
    fail, summary = carried(w, (ALONG - 0.081, 0.0, 0.0), gaps={"left": (0.05, 0.05), "right": (0.05, 0.05)})
    assert fail == "along" and summary["baseline"] is None


def test_a_slide_during_the_lift_is_caught_in_the_lift():
    w = watch()
    lifted(w, (ALONG, 0.0, 0.0))
    fail, _ = carried(w, (ALONG, 0.05, 0.0), phase="lift")
    assert fail == "outline_motion"


@pytest.mark.parametrize("gaps", [{"left": (None, 0.05), "right": (0.05, 0.05)},
                                  {"left": (0.05, 0.05), "right": (0.05, 0.0019)},
                                  {"left": (0.05, -0.001), "right": (0.05, 0.05)}])
def test_a_missing_wall_or_a_gap_at_the_margin_fails(gaps):
    w = watch()
    fail, _ = carried(w, (ALONG, 0.0, 0.0), gaps=gaps)
    assert fail == "pocket_clearance"


def test_a_gap_just_above_the_margin_passes():
    w = watch()
    fail, _ = carried(w, (ALONG, 0.0, 0.0), gaps={"left": (0.0021, 0.05), "right": (0.05, 0.0021)})
    assert fail is None


def test_the_yaw_difference_is_taken_across_plus_minus_pi():
    """A truck heading near pi, the pallet riding 0.002 rad to its left: the world yaws straddle +-pi (the pallet's
    reads near -pi while the truck's reads near +pi), and the relative pose must still be the seat's."""
    w = watch()
    base = (2.0, 1.0, math.pi - 0.001)
    lifted(w, (ALONG, 0.0, 0.002), base=base)
    fail, summary = carried(w, (ALONG, 0.0, 0.002), base=base,
                            gaps={"left": (0.05, 0.05), "right": (0.05, 0.05)})
    assert fail is None and summary["outline_motion_max_m"] < 1e-6
    assert summary["baseline"]["seat"][2] == pytest.approx(0.002, abs=1e-9)  # recorded wrapped, not -2 pi + 0.002


def test_the_baseline_follows_the_load_through_a_loaded_resubmit_and_resets_at_the_next_insert():
    w = watch()
    seat = (ALONG, 0.01, 0.0)
    lifted(w, seat)
    w.end_tick("transport", (0.0, 0.0, 0.0), (ALONG, 0.01, 0.0), t=1.0)
    w.end_tick("idle", (0.0, 0.0, 0.0), (ALONG, 0.01, 0.0), t=2.0)       # the mission ended carrying
    w.end_tick("transport", (0.0, 0.0, 0.0), (ALONG, 0.01, 0.0), t=3.0)  # a loaded submit starts in transport
    assert w.baseline is not None and w.baseline["seat"] == pytest.approx(list(seat))
    fail, _ = carried(w, (ALONG, 0.06, 0.0))
    assert fail == "outline_motion"
    w.end_tick("approach", (0.0, 0.0, 0.0), (5.0, 0.0, 0.0), t=4.0)
    w.end_tick("insert", (0.0, 0.0, 0.0), (5.0, 0.0, 0.0), t=5.0)
    assert w.baseline is None


def test_the_baseline_is_the_instant_of_the_insert_to_lift_transition_only():
    """A lift entered from anywhere but insert (a resumed lift) takes no new baseline."""
    w = watch()
    w.end_tick("extract", (0.0, 0.0, 0.0), (ALONG, 0.0, 0.0), t=0.0)
    w.end_tick("lift", (0.0, 0.0, 0.0), (ALONG, 0.0, 0.0), t=1.0)
    assert w.baseline is None


def test_the_summary_records_the_failing_tick_before_returning():
    w = watch()
    lifted(w, (ALONG, 0.0, 0.0))
    summary = {}
    carried(w, (ALONG, 0.0, 0.0), summary=summary, t=1.0, gaps={"left": (0.05, 0.05), "right": (0.05, 0.05)})
    fail, _ = carried(w, (ALONG, 0.05, 0.0), summary=summary, t=2.0, gaps={"left": (0.05, 0.05), "right": (0.05, 0.05)})
    assert fail == "outline_motion"
    f = summary["failure"]
    assert f["t"] == 2.0 and f["rel"] == pytest.approx([ALONG, 0.05, 0.0]) and f["outline_motion_m"] == pytest.approx(0.05)
    assert summary["checks"] == 2


def test_the_old_check_is_recorded_only_where_it_applied():
    """Codex slip impl round 1 P2-4: the earlier check never ran in the lift -- a 50 mm seat there is no old failure;
    in extract (and idle carrying) it is."""
    w = watch()
    seat = (ALONG, 0.05, 0.0)
    lifted(w, seat)
    summary = {}
    carried(w, seat, summary=summary, phase="lift", t=1.0, gaps={"left": (0.05, 0.05), "right": (0.05, 0.05)})
    assert summary["old_would_fail_s"] is None and summary["old_lateral_max_m"] == 0.0
    carried(w, seat, summary=summary, phase="idle", t=2.0, gaps={"left": (0.05, 0.05), "right": (0.05, 0.05)})
    assert summary["old_would_fail_s"] == 2.0
    assert summary["old_lateral_max_m"] == pytest.approx(0.05)


def test_each_blades_smallest_pocket_gap_keeps_its_time():
    """Codex slip impl round 1 P2-5 (plan: the smallest gap per blade and when)."""
    w = watch()
    summary = {}
    carried(w, (ALONG, 0.0, 0.0), summary=summary, t=1.0, gaps={"left": (0.03, 0.05), "right": (0.05, 0.04)})
    carried(w, (ALONG, 0.0, 0.0), summary=summary, t=2.0, gaps={"left": (0.05, 0.02), "right": (0.06, 0.05)})
    assert summary["pocket_gap_min_m"] == {"left": 0.02, "right": 0.04}
    assert summary["pocket_gap_min_t"] == {"left": 2.0, "right": 1.0}


def test_a_tick_with_no_joints_read_yet_leaves_the_pockets_unchecked_and_counted():
    """The runner's first tick has no steering read before it (the lift unknown): the pockets are not judged then."""
    w = watch()
    summary = {}
    fail = w.check(summary, t=0.0, phase="transport", base=(0.0, 0.0, 0.0), pallet=(ALONG, 0.0, 0.0), gaps=None)
    assert fail is None and summary["pocket_unchecked"] == 1


def test_the_pockets_are_judged_over_the_lift_the_step_can_reach():
    """Codex slip impl round 2 P2-1: the lift is the previous tick's read, so the gaps are taken at both ends of
    what one physics step can move it and each blade keeps the worse -- a wall missing at either end fails.
    The carried seat keeps walls over [0.087, 0.1895] m, 16 mm beyond the 8.3 mm step either way."""
    step = 1.0 / 120  # the URDF's fork_lift velocity limit (1.0 m/s) over one physics step
    def judged(lift, pz):
        return SW.merge_gaps(*(GEOMETRY.pocket_side_gaps((0.0, 0.0, 0.0), _q(0.0), lift + d, (0.93, 0.0, pz), _q(0.0))
                               for d in (-step, step)))
    w = watch()
    assert w.check({}, t=0.0, phase="transport", base=(0.0, 0.0, 0.0), pallet=(ALONG, 0.0, 0.0),
                   gaps=judged(LIFT, PALLET_Z)) is None
    # Codex's case: read at 0.1645 with the true lift 0.165 above the blocks of a pallet 25 mm low
    assert w.check({}, t=0.0, phase="transport", base=(0.0, 0.0, 0.0), pallet=(ALONG, 0.0, 0.0),
                   gaps=judged(0.1645, 0.09225)) == "pocket_clearance"


def test_merging_keeps_each_sides_smaller_gap_and_a_missing_wall():
    a = {"left": (0.03, 0.05), "right": (None, 0.04)}
    b = {"left": (0.04, 0.02), "right": (0.06, 0.05)}
    assert SW.merge_gaps(a, b) == {"left": (0.03, 0.02), "right": (None, 0.04)}
