"""M-b4: the teleoperated drive chain and the arc-bundle permission (plan B8); synthetic grids only."""

import math

import numpy as np
import pytest

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.planning.geometry import Footprint
from forklift_core.safety.teleop_drive import TeleopDrive, TeleopLimits, arc_permit, bundle  # noqa: F401

FOOT = Footprint(1.29, 0.17, 0.36)
BODY = Footprint(0.75, 0.17, 0.36)
STOP = StoppingModel(latency_s=0.15, decel_mps2=1.5, margin_m=0.05)
CONFIG = PermissionConfig(STOP, envelope_offset_m=0.01, evidence_max_age_s=0.5)
DT = 1 / 120


def snapshot():
    state = np.full((200, 120), FREE, dtype=np.uint8)  # x -1..9, y -3..3
    return GridSnapshot(state, np.where(state == FREE, 0.0, np.nan), 0.0, (0.0, 0.0, 0.0), 0, 1, {"low": 0.0},
                        -1.0, -3.0, 0.05)


def occupy(snap, x, y):
    i, j = int(math.floor((x + 1.0) / 0.05)), int(math.floor((y + 3.0) / 0.05))
    snap.state[i, j] = OCCUPIED
    snap.free_stamp[i, j] = np.nan


def permit_on(snap, t=0.1, pose=(0.0, 0.0, 0.0)):
    perm = DrivePermission(CONFIG)
    asked = []

    def permit(curvatures, direction, check):
        asked.append((list(curvatures), direction, check))
        return arc_permit(perm, snap, t, {"low": t}, pose=pose, curvatures=curvatures, direction=direction,
                          check_speed_mps=check, footprint=FOOT, own_footprint=BODY)
    return permit, asked


def cmd(speed=1.0, curvature=0.0, deadman=True, mode="direct"):
    return {"speed_ratio": speed, "curvature_ratio": curvature, "deadman": deadman, "mode": mode, "lift_dir": 0}


def drive(**k):
    return TeleopDrive(TeleopLimits(**k), STOP.decel_mps2, DT)


def run(d, teleop, permit, *, last=0.0, measured=0.0, kappa=0.0, armed=True, allowed=True, preempt=False, err=0.0):
    return d.step(teleop, armed=armed, drive_allowed=allowed, loaded=False, preempt=preempt, last_applied_mps=last,
                  measured_mps=measured, measured_kappa=kappa, steering_error_rad=err, permit=permit)


def test_the_bundle_is_the_grid_covering_both_curvatures_and_the_margin():
    """Teleop bundle margin plan D1: the 0.05 grid, outer points rounded outward, the margin on both sides."""
    assert bundle(-0.1, 0.12) == pytest.approx([-0.1, -0.05, 0.0, 0.05, 0.1, 0.15])
    assert bundle(0.12, -0.1) == bundle(-0.1, 0.12)  # either order
    assert bundle(0.2, 0.2) == [0.2]  # a grid point alone: one arc, not the same arc twice
    assert bundle(0.21, 0.21) == pytest.approx([0.2, 0.25])  # off the grid: bracketed
    assert bundle(0.0, 0.0, 0.04) == pytest.approx([-0.05, 0.0, 0.05])
    assert bundle(-0.092, -0.108, 0.04) == pytest.approx([-0.15, -0.1, -0.05])
    for low, high, margin in ((-0.37, 0.02, 0.04), (0.149999, 0.15, 0.0), (-0.0001, 0.0001, 0.03)):
        ks = bundle(low, high, margin)
        assert ks[0] <= low - margin + 1e-12 and ks[-1] >= high + margin - 1e-12
        assert max(np.diff(ks), default=0.0) <= 0.05 + 1e-12
        assert all(abs(k / 0.05 - round(k / 0.05)) < 1e-9 for k in ks)  # exactly on the grid


def test_the_limits_refuse_a_negative_margin():
    with pytest.raises(ValueError):
        TeleopLimits(curvature_margin_inv_m=-0.01)


def test_the_drive_asks_the_margined_bundle():
    """Plan D1: the chain asks the grid bundle of measured to output curvature grown by the limits' margin."""
    asked = []

    def permit(curvatures, direction, check):
        asked.append(list(curvatures))
        return 1.0, "ok"

    d = drive()
    d.kappa = -0.1
    run(d, cmd(curvature=-0.1 / 0.36), permit, kappa=-0.092)
    assert asked[-1] == pytest.approx([-0.15, -0.1, -0.05])
    assert asked[-1] == pytest.approx(bundle(-0.092, -0.1, 0.04))


def test_a_free_corridor_lets_the_slew_pull_away_and_the_arc_never_reaches_its_end():
    permit, asked = permit_on(snapshot())
    speed, _, (limit, why) = run(drive(), cmd(), permit)
    # Free: the limit is the stop arc's at the check speed (0.5 m/s asked: 0.540), not a goal's creep.
    assert speed == pytest.approx(0.8 * DT) and why == "ok" and limit == pytest.approx(STOP.speed_for(STOP.distance_m(0.5) + 0.02))
    assert asked[0][1:] == (1, pytest.approx(0.5))  # check speed covers this cycle's request


def test_an_obstacle_ahead_cuts_the_slewed_speed_as_a_step():
    snap = snapshot()
    occupy(snap, 1.5, 0.0)  # 0.21 m beyond the forks' tips (1.29 m from the rear axle)
    permit, _ = permit_on(snap)
    speed, _, (limit, why) = run(drive(), cmd(), permit, last=0.5, measured=0.5)
    assert why == "occupied" and speed == pytest.approx(limit) and speed < 0.5 - 1.5 * DT


def test_an_obstacle_on_the_turning_side_is_seen_while_the_steering_is_still_straight():
    """The bundle: steering measured straight, the operator turns left -- the turned arc is checked now."""
    snap = snapshot()
    occupy(snap, 1.45, 0.45)  # left of the straight swept outline (half width 0.36 m), in the turn's
    permit, asked = permit_on(snap)
    d = drive(curvature_rate_inv_m_s=60.0)  # the output reaches the target this cycle
    _, kappa, (limit_turn, why_turn) = run(d, cmd(speed=0.0, curvature=1.0), permit, last=0.4, measured=0.4)
    assert kappa == pytest.approx(0.36) and len(asked[-1][0]) >= 8
    permit2, _ = permit_on(snap)
    _, _, (limit_straight, why_straight) = run(drive(), cmd(speed=0.0), permit2, last=0.4, measured=0.4)
    assert (why_turn, why_straight) == ("occupied", "ok") and limit_turn < limit_straight


def test_the_curvature_is_rate_limited():
    permit, _ = permit_on(snapshot())
    d = drive()
    _, kappa, _ = run(d, cmd(curvature=1.0), permit)
    assert kappa == pytest.approx(0.5 * DT)


def test_releasing_the_dead_man_switch_brakes_on_the_slew_and_a_forbidden_drive_steps_to_zero():
    permit, _ = permit_on(snapshot())
    speed, _, _ = run(drive(), cmd(deadman=False), permit, last=0.3, measured=0.3)
    assert speed == pytest.approx(0.3 - 1.5 * DT)
    speed, _, (limit, why) = run(drive(), cmd(), permit, last=0.3, measured=0.3, allowed=False)
    assert speed == 0.0 and (limit, why) == (0.0, "drive_forbidden")


def test_the_check_covers_the_last_applied_command_while_braking():
    """Codex M-b plan round 2 P1: last 0.5, measured 0.4, request 0 checks at 0.5."""
    permit, asked = permit_on(snapshot())
    run(drive(), cmd(deadman=False), permit, last=0.5, measured=0.4)
    assert asked[-1][2] == pytest.approx(0.5)


def test_the_measured_direction_is_checked_while_a_reverse_is_asked_for():
    permit, asked = permit_on(snapshot())
    speed, _, _ = run(drive(), cmd(speed=-1.0), permit, last=0.2, measured=0.2)
    assert asked[-1][1] == 1 and speed == pytest.approx(0.2 - 1.5 * DT)  # through zero first


def test_limits_scale_the_ratios_and_direct_cannot_exceed_video():
    d = drive(direct_forward_mps=0.3, direct_reverse_mps=0.2)
    assert d.target(cmd(speed=1.0), loaded=False)[0] == 0.3
    assert d.target(cmd(speed=-1.0, mode="video"), loaded=False)[0] == -0.25
    assert d.target(cmd(speed=1.0, mode="video"), loaded=True)[0] == 0.30
    with pytest.raises(ValueError):
        TeleopLimits(direct_forward_mps=0.6)


def test_a_reversing_output_is_checked_backwards_while_forward_is_asked_for():
    """Codex M-b review round 2 P1: standing, last applied -0.25, forward asked -- the slew still outputs reverse;
    an obstacle behind must cut it."""
    snap = snapshot()
    occupy(snap, -0.25, 0.0)  # just behind the rear (0.17 m)
    permit, asked = permit_on(snap, pose=(0.0, 0.0, 0.0))
    speed, _, (limit, why) = run(drive(), cmd(speed=1.0), permit, last=-0.25, measured=0.0)
    assert {a[1] for a in asked} == {1, -1} and why == "occupied" and speed == pytest.approx(-limit)
    assert abs(speed) < 0.25 - 1.5 * DT


def test_the_bundle_permit_matches_the_full_check_at_install_and_follows_the_distance_driven():
    from forklift_core.safety.teleop_drive import BundlePermit

    snap = snapshot()
    occupy(snap, 1.42, 0.0)  # close enough that the path ahead, not the stop arc, sets the limit
    full, _ = permit_on(snap)
    bp = BundlePermit(CONFIG)
    kw = dict(pose=(0.0, 0.0, 0.0), curvatures=bundle(0.0, 0.0), direction=1, check_speed_mps=0.3, footprint=FOOT,
              own_footprint=BODY)
    first = bp.limit(snap, 1, 0.1, {"low": 0.1}, **kw)
    assert first == pytest.approx(full(bundle(0.0, 0.0), 1, 0.3))
    bp.advance(0.05)
    later = bp.limit(snap, 1, 0.1, {"low": 0.1}, **kw)  # same pose given, but 0.05 m driven
    assert later[0] < first[0]  # the path left shrank
    other = bp.limit(snap, 2, 0.1, {"low": 0.1}, **kw)  # a new grid: installed again
    assert other == pytest.approx(first)


def test_the_bundle_permit_is_cheap_between_refreshes():
    import time

    from forklift_core.safety.teleop_drive import BundlePermit

    snap = snapshot()
    bp = BundlePermit(CONFIG)
    kw = dict(pose=(0.0, 0.0, 0.0), curvatures=bundle(0.0, 0.36), direction=1, check_speed_mps=0.3,
              footprint=FOOT, own_footprint=BODY)
    bp.limit(snap, 1, 0.1, {"low": 0.1}, **kw)
    start = time.perf_counter()
    for _ in range(10):
        bp.advance(0.001)
        bp.limit(snap, 1, 0.1, {"low": 0.1}, **kw)
    assert (time.perf_counter() - start) / 10 < 0.02



def test_the_ends_are_asked_at_the_current_curvature_not_the_cached_one():
    """Codex M-c round 10 P1: +0.024 and -0.024 round to one key; the stop arc must use the current one."""
    from forklift_core.safety.teleop_drive import BundlePermit

    snap = snapshot()
    occupy(snap, 1.45, 0.45)
    bp = BundlePermit(CONFIG)
    kw = dict(pose=(0.0, 0.0, 0.0), direction=1, check_speed_mps=0.4, footprint=FOOT, own_footprint=BODY)
    bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=bundle(-0.024, -0.024), **kw)
    cached = bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=bundle(0.024, 0.024), **kw)
    perm = DrivePermission(CONFIG)
    direct = arc_permit(perm, snap, 0.1, {"low": 0.1}, curvatures=bundle(0.024, 0.024), **kw)
    assert cached[0] <= direct[0] + 1e-9


def test_stale_evidence_under_a_middle_arc_stops():
    """Codex M-c round 10 P1: the ends' evidence is fresh, a middle arc's is not."""
    from forklift_core.safety.teleop_drive import BundlePermit

    snap = snapshot()
    bp = BundlePermit(CONFIG)
    kw = dict(pose=(0.0, 0.0, 0.0), direction=1, check_speed_mps=0.3, footprint=FOOT, own_footprint=BODY)
    curv = bundle(0.0, 0.36)
    bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=curv, **kw)
    _, perm = bp._cache[1]["perms"][len(curv) // 2]
    perm.path_check.oldest_free_s = -1.0  # only this middle arc relies on old FREE evidence
    assert bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=curv, **kw) == (0.0, "evidence_stale")


def test_a_reversal_keeps_a_cache_per_direction():
    """Codex M-c round 10 P2: alternating directions must not reinstall every cycle."""
    from forklift_core.safety.teleop_drive import BundlePermit

    snap = snapshot()
    bp = BundlePermit(CONFIG)
    kw = dict(pose=(0.0, 0.0, 0.0), curvatures=bundle(0.0, 0.0), check_speed_mps=0.3, footprint=FOOT,
              own_footprint=BODY)
    for d in (1, -1):
        bp.limit(snap, 1, 0.1, {"low": 0.1}, direction=d, **kw)
    installed = {d: id(bp._cache[d]["perms"][0][1]) for d in (1, -1)}
    for d in (1, -1, 1, -1):
        bp.limit(snap, 1, 0.1, {"low": 0.1}, direction=d, **kw)
    assert {d: id(bp._cache[d]["perms"][0][1]) for d in (1, -1)} == installed



def test_steering_away_from_a_single_arc_installs_the_output_curvature():
    """Codex M-c round 14 P1: [0] installed, then [0, -0.024] (the same rounded ends) must check -0.024."""
    from forklift_core.safety.teleop_drive import BundlePermit

    snap = snapshot()
    occupy(snap, 0.8, -0.65)
    bp = BundlePermit(CONFIG)
    kw = dict(pose=(0.0, 0.034, 0.0), direction=1, check_speed_mps=0.5, footprint=FOOT, own_footprint=BODY)
    bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=bundle(0.0, 0.0), **kw)
    cached = bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=bundle(0.0, -0.024), **kw)
    assert [k for k, _ in bp._cache[1]["perms"]] == pytest.approx(bundle(0.0, -0.024))
    direct = arc_permit(DrivePermission(CONFIG), snap, 0.1, {"low": 0.1}, curvatures=bundle(0.0, -0.024), **kw)
    assert cached == pytest.approx(direct)


def test_an_end_drifting_inside_its_grid_cell_keeps_the_arcs_a_full_check_would_install():
    """Plan D1 (M-c evidence 1.5: the cache once allowed 0.361 m/s where the full check gave 0.341): the old ends
    rounded to one key while the installed arcs were the old exact curvatures; on the grid both are the same set."""
    from forklift_core.safety.teleop_drive import BundlePermit

    snap = snapshot()
    occupy(snap, 1.6, -0.25)
    bp = BundlePermit(CONFIG)
    kw = dict(pose=(0.0, 0.0, 0.0), direction=1, check_speed_mps=0.4, footprint=FOOT, own_footprint=BODY)
    first = bundle(-0.092, -0.108, 0.04)
    bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=first, **kw)
    installed = [id(p) for _, p in bp._cache[1]["perms"]]
    later = bundle(-0.097, -0.104, 0.04)  # drifted, same grid cells
    assert later == first
    cached = bp.limit(snap, 1, 0.1, {"low": 0.1}, curvatures=later, **kw)
    assert [id(p) for _, p in bp._cache[1]["perms"]] == installed  # no reinstall
    direct = arc_permit(DrivePermission(CONFIG), snap, 0.1, {"low": 0.1}, curvatures=later, **kw)
    assert cached == pytest.approx(direct)


def test_the_short_walk_gives_the_limit_the_full_lookahead_gives_up_to_the_teleop_caps():
    """Plan D6: an arc walks the refresh distance and the stop from 1 m/s; beyond that speed_for() is above every
    teleop cap, so the limit an occupied cell gives the chain is the 3 m lookahead's (the evidence age differs:
    test_only_the_walked_evidence_must_be_fresh)."""
    from forklift_core.safety.teleop_drive import WALK_SPEED_MPS, walk_m

    assert walk_m(CONFIG, 0.5) == pytest.approx(0.25 + STOP.distance_m(WALK_SPEED_MPS) + CONFIG.step_m)
    assert STOP.speed_for(walk_m(CONFIG, 0.5) - 0.25) > max(TeleopLimits().video_forward_mps, 0.6)
    for x_obstacle in (1.7, 1.9, 2.5, 4.0):  # the front face at 1.29 m: 0.41, 0.61, 1.21 and 2.71 m of travel
        snap = snapshot()
        occupy(snap, x_obstacle, 0.0)
        kw = dict(pose=(0.0, 0.0, 0.0), curvatures=bundle(0.0, 0.0, 0.04), direction=1, check_speed_mps=0.5,
                  footprint=FOOT, own_footprint=BODY)
        short = arc_permit(DrivePermission(CONFIG), snap, 0.1, {"low": 0.1}, **kw)
        full = DrivePermission(CONFIG)
        best = math.inf
        for k in kw["curvatures"]:
            from forklift_core.control.drive_permission import arc_poses

            path, _ = arc_poses((0.0, 0.0, 0.0), k, 1, 3.6, 0.02)
            full.update(snap, path, FOOT, BODY, current_pose=(0.0, 0.0, 0.0), direction=1)
            best = min(best, full.allowed_speed(0.1, {"low": 0.1}, current_pose=(0.0, 0.0, 0.0), curvature_inv_m=k,
                                                direction=1, footprint=FOOT, own_footprint=BODY,
                                                speed_cap_mps=0.5)[0])
        assert min(short[0], 0.6) == pytest.approx(min(best, 0.6))


def test_only_the_walked_evidence_must_be_fresh():
    """Plan D6 (Codex review r2 P2): the short walk asks fresh evidence only where a stop can reach -- an old FREE
    cell 2.8 m out no longer zeroes teleop (the 3 m lookahead's did), one inside the walk still does."""
    kw = dict(pose=(0.0, 0.0, 0.0), curvatures=bundle(0.0, 0.0, 0.04), direction=1, check_speed_mps=0.5,
              footprint=FOOT, own_footprint=BODY)
    for x, expected in ((2.8 + 1.29, "ok"), (0.5 + 1.29, "evidence_stale")):
        snap = snapshot()
        i, j = int(math.floor((x + 1.0) / 0.05)), int(math.floor((0.0 + 3.0) / 0.05))
        snap.free_stamp[i, j] = -10.0  # seen free long ago
        assert arc_permit(DrivePermission(CONFIG), snap, 0.1, {"low": 0.1}, **kw)[1] == expected


def test_a_forbidden_drive_does_not_ask_the_arcs():
    """The cap is 0 anyway; the arcs are the costly part of a cycle (console session profile, 2026-10-17)."""
    asked = []

    def permit(curvatures, direction, check):
        asked.append(curvatures)
        return 1.0, "ok"

    speed, _, (limit, why) = run(drive(), cmd(speed=1.0), permit, last=0.2, measured=0.2, allowed=False)
    assert asked == [] and (limit, why) == (0.0, "drive_forbidden") and speed < 0.2
