"""Low-clearance guard (docs/plans/2026-10-25-low-clearance-guard.md): props below the LiDAR plane are watched along
the stop arc of the estimated pose, each check against the planned clearance the same distance ahead on the leg; the
speed is capped so the truck stops before the first violation, contact always being one; the arc ends at the leg end
plus the tracker's end allowance; interval midpoints keep the sweep between samples."""

import math

import numpy as np
import pytest

from forklift_core.control import low_clearance as lc
from forklift_core.control.drive_permission import StoppingModel
from forklift_core.planning import Bounds, Footprint, Rectangle
from forklift_core.planning.pallet_mission import AssetSpec, PalletSite, PlacedProp, TransportScenario
from forklift_core.planning.geometry import Pose2D

STOP = StoppingModel(0.15, 1.5, 0.05)
CFG = lc.LowClearanceConfig()
LOADED = Footprint(1.56, 0.17, 0.4)


def prop(x, y, length, width, *, height=0.144, uri="riibotics:block", yaw=0.0):
    return PlacedProp(AssetSpec(uri, length, width, height), Rectangle(x, y, length, width, yaw))


def scenario(*props):
    return TransportScenario(0, Pose2D(0.0, 0.0, 0.0), PalletSite(5.0, 0.0, 0.0), PalletSite(-5.0, 0.0, 0.0),
                             tuple(props), Bounds(-20.0, 20.0, -20.0, 20.0))


def straight(x0, x1, y=0.0, n=101):
    xs = np.linspace(x0, x1, n)
    return np.column_stack((xs, np.full(n, y), np.zeros(n)))


def cap(pose, leg, *, progress, remaining, kappa=0.0, direction=1, speed=0.3, end_allowance=0.008, shape=LOADED):
    return lc.guard_cap(pose, kappa, direction, speed, shape, leg, progress_m=progress, leg_remaining_m=remaining,
                        end_allowance_m=end_allowance, stopping=STOP, config=CFG)


# ---- D1: which props ----

def test_only_props_below_the_lidar_plane_and_of_the_stage_are_watched():
    sc = scenario(prop(1, 1, 0.2, 0.2), prop(2, 2, 0.2, 0.2, height=1.5, uri="riibotics:rack_upright"),
                  prop(3, 3, 0.2, 0.2, uri="session:pallet@search"), prop(4, 4, 0.2, 0.2, uri="session:pallet@slot"))
    search = {(r.x_m, r.y_m) for r in lc.low_props(sc, "search")}
    slot = {(r.x_m, r.y_m) for r in lc.low_props(sc, "slot")}
    assert search == {(1, 1), (3, 3)} and slot == {(1, 1), (4, 4)}


# ---- D2: distance and contact ----

def test_the_clearance_is_exact_and_zero_on_contact():
    block = Rectangle(2.0, 1.0, 1.0, 1.0, 0.0)  # its lower edge at y = 0.5
    assert lc.shape_clearance((0.0, 0.0, 0.0), LOADED, [block]) == pytest.approx(0.1)  # outline top at y = 0.4
    assert lc.shape_clearance((0.0, 0.1, 0.0), LOADED, [block]) == 0.0
    assert lc.shape_clearance((0.0, 0.0, 0.0), LOADED, []) == math.inf


# ---- D3: the stop-arc rule ----

def side_block(gap):
    """A low block beside the loaded outline's left side, ``gap`` from it while the truck runs along y = 0."""
    return Rectangle(1.0, 0.4 + gap + 0.5, 3.0, 1.0, 0.0)


def leg_along(path, *props):
    return lc.LegClearance(path, LOADED, search_props=list(props), slot_props=list(props))


def test_a_drift_toward_a_block_is_capped_and_a_drift_away_is_not():
    block = side_block(0.08)
    leg = leg_along(straight(-2.0, 4.0), block)
    toward = cap((0.0, 0.0, 0.0), leg, progress=2.0, remaining=4.0, kappa=0.36)
    away = cap((0.0, 0.0, 0.0), leg, progress=2.0, remaining=4.0, kappa=-0.36)
    on_path = cap((0.0, 0.0, 0.0), leg, progress=2.0, remaining=4.0)
    assert math.isinf(away.allowed_mps) and math.isinf(on_path.allowed_mps)
    # turning at 0.36 1/m the left front corner reaches the block well inside the stop arc
    assert toward.allowed_mps < 0.3 and toward.s_star_m > 0.0


def test_already_inside_the_margin_stops():
    block = side_block(0.144)
    leg = leg_along(straight(-2.0, 4.0), block)
    inside = cap((0.0, 0.144 - 0.03, 0.0), leg, progress=2.0, remaining=4.0)  # 3 cm from the block
    assert inside.allowed_mps == 0.0 and inside.s_star_m == 0.0


def test_the_cap_stops_short_of_the_first_violation():
    block = side_block(0.08)
    leg = leg_along(straight(-2.0, 4.0), block)
    out = cap((0.0, 0.0, 0.0), leg, progress=2.0, remaining=4.0, kappa=0.36)
    assert out.allowed_mps == pytest.approx(STOP.speed_for(out.s_star_m))
    if out.allowed_mps > 0.0:
        assert STOP.distance_m(out.allowed_mps) <= out.s_star_m + 1e-9
    else:
        assert out.s_star_m <= STOP.margin_m  # nearer than the stopping margin: stand
    # farther from the violation, the cap rises with the distance left to it
    far = cap((0.0, -0.06, 0.0), leg, progress=2.0, remaining=4.0, kappa=0.36)
    assert far.s_star_m > out.s_star_m and far.allowed_mps > out.allowed_mps
    assert STOP.distance_m(far.allowed_mps) <= far.s_star_m + 1e-9


def test_a_tight_plan_followed_exactly_is_never_capped_even_just_before_its_end():
    # Codex counterexample: a reverse straight that ends 0.04 m from a block behind it
    block = Rectangle(-1.0 - 0.17 - 0.04 - 0.5, 0.0, 1.0, 1.0, 0.0)  # its front face 0.04 m behind the goal's rear
    path = straight(0.0, -1.0)
    leg = leg_along(path, block)
    for done in (0.5, 0.9, 0.96, 0.984):  # 16 mm before the goal included
        pose = (-done, 0.0, 0.0)
        out = cap(pose, leg, progress=done, remaining=1.0 - done, direction=-1, speed=0.024)
        assert math.isinf(out.allowed_mps), done


def test_a_tight_plan_is_capped_once_followed_closer_than_the_deviation_allows():
    block = Rectangle(-1.0 - 0.17 - 0.04 - 0.5, 0.0, 1.0, 1.0, 0.0)
    leg = leg_along(straight(0.0, -1.0), block)
    # 3 cm deeper than planned at the goal: 0.01 left against the plan's 0.04 - 0.02
    out = cap((-1.03, 0.0, 0.0), leg, progress=0.99, remaining=0.01, direction=-1, speed=0.05)
    assert out.allowed_mps == 0.0


def test_contact_between_two_samples_is_caught():
    # After the Codex counterexample (loaded shape, curvature 0.36, an 8 mm square prop: samples that all pass with
    # the outline meeting the prop between them), at the guard's own 0.01 m spacing
    fp = LOADED
    kappa = 0.36
    pose = (0.0, 0.0, 0.0)
    from forklift_core.control.drive_permission import arc_poses

    poses, s = arc_poses(pose, kappa, 1, 0.01, 0.005)
    mid = poses[1]  # s = 0.005: halfway between the guard's 0.01 m samples
    # the left front corner at the midpoint pose; a tiny prop just touching it there
    cx = mid[0] + fp.front_m * math.cos(mid[2]) - fp.half_width_m * math.sin(mid[2])
    cy = mid[1] + fp.front_m * math.sin(mid[2]) + fp.half_width_m * math.cos(mid[2])
    tiny = Rectangle(cx + 0.003, cy + 0.003, 0.008, 0.008, 0.0)
    assert lc.shape_clearance(tuple(mid), fp, [tiny]) == 0.0
    leg = lc.LegClearance(straight(-1.0, 3.0, n=201), fp, search_props=[tiny], slot_props=[tiny])
    out = cap(pose, leg, progress=1.0, remaining=3.0, kappa=kappa, speed=0.02)
    assert out.allowed_mps < 0.02


def test_a_tight_plan_is_not_capped_before_its_end_with_the_extract_allowance():
    # Codex round 3: the extract tracker accepts a 0.03 m overrun; 16 mm before the goal of a straight ending 0.04 m
    # from a block, followed exactly at the end creep
    block = Rectangle(-1.0 - 0.17 - 0.04 - 0.5, 0.0, 1.0, 1.0, 0.0)
    leg = leg_along(straight(0.0, -1.0), block)
    out = cap((-0.984, 0.0, 0.0), leg, progress=0.984, remaining=0.016, direction=-1, speed=0.024, end_allowance=0.03)
    assert math.isinf(out.allowed_mps)


def test_past_the_leg_end_the_arc_keeps_the_physical_braking_distance():
    # Codex round 3: unloaded, reversing at 0.6 m/s 0.01 m before the end, a prop 0.11 m behind: the end allowance
    # (0.03) must not cut away the 0.21 m the truck needs to stop
    unloaded = Footprint(1.29, 0.17, 0.36)
    prop_ = Rectangle(-0.17 - 0.11 - 0.5, 0.0, 1.0, 1.0, 0.0)
    leg = lc.LegClearance(straight(1.0, 0.0), unloaded, search_props=[prop_], slot_props=[prop_])
    out = cap((0.0, 0.0, 0.0), leg, progress=0.99, remaining=0.01, direction=-1, speed=0.6, end_allowance=0.03,
              shape=unloaded)
    assert out.allowed_mps < 0.6 and out.arc_m >= 0.6 * 0.15 + 0.6 ** 2 / 3.0


def test_the_arc_ends_at_the_leg_end_plus_its_allowance():
    # a block just past the goal along the heading: beyond the leg end + allowance it is not looked at
    block = Rectangle(1.56 + 0.05 + 0.5, 0.0, 1.0, 1.0, 0.0)  # 5 cm ahead of the front at the goal x = 0
    leg = leg_along(straight(-1.0, 0.0), block)
    near_end = cap((-0.01, 0.0, 0.0), leg, progress=0.99, remaining=0.01, speed=0.05)
    assert math.isinf(near_end.allowed_mps)


def test_the_slot_props_count_only_on_the_slot_part_of_the_leg():
    # the search-stage estimate of a neighbour sits on the slot straight; only the slot copy counts there
    estimate = side_block(-0.05)  # overlapping the outline: a stale estimate
    slot_copy = side_block(0.2)
    leg = lc.LegClearance(straight(-2.0, 2.0), LOADED, search_props=[estimate], slot_props=[slot_copy], slot_from_m=1.0)
    assert math.isinf(cap((0.0, 0.0, 0.0), leg, progress=2.0, remaining=2.0).allowed_mps)
    assert cap((-1.5, 0.0, 0.0), leg, progress=0.5, remaining=3.5, speed=0.05).allowed_mps == 0.0


def test_the_planned_clearance_is_read_by_progress_along_the_leg():
    block = side_block(0.144)
    leg = leg_along(straight(-2.0, 4.0), block)
    assert leg.at(2.0) == pytest.approx(0.144, abs=1e-6)
    assert leg.at(-1.0) == leg.at(0.0) and leg.at(99.0) == leg.at(leg.length_m)
