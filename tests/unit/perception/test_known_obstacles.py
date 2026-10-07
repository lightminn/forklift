"""Known pallets in the drive permission (plan v10 D5): synthetic grids only.

The single 1.05 m LiDAR never sees an EPAL 6 pallet, so the pickup zone, the
recognised pallet and the delivered pallet reach the permission as known
rectangles whose radius grows with the age of the last relative fix. Codex's
v10 counterexamples are kept: a delivered pallet inside a 0.26 m stopping
sweep with no-hit scans must stop the truck, and a withdrawal with the pallet
on the forks must still start once its corridor is certified.
"""

import math

import numpy as np
import pytest

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.perception.known_obstacles import (
    KnownRect,
    apply_known,
    corridor_mask,
    fork_pocket_gaps,
    known_radius,
    withdraw_certified,
)
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
from forklift_core.planning.geometry import Footprint

TABLE = AgeErrorTable((0.1, 0.5, 1.0, 3.0, 10.0), (0.024, 0.059, 0.092, 0.140, 0.314), (0.014, 0.030, 0.040, 0.077, 0.153))
FOOT = Footprint(1.29, 0.17, 0.36)
BODY = Footprint(0.75, 0.17, 0.36)


def snapshot(fill=FREE, stamp=0.0, correction=(0.0, 0.0, 0.0), version=0):
    state = np.full((200, 120), fill, dtype=np.uint8)  # x -1..9, y -3..3
    free_stamp = np.where(state == FREE, stamp, np.nan)
    return GridSnapshot(state, free_stamp, stamp, correction, version, 1, {"high": stamp}, -1.0, -3.0, 0.05)


def cell(x, y):
    return int(math.floor((x + 1.0) / 0.05)), int(math.floor((y + 3.0) / 0.05))


def test_the_radius_grows_with_age_by_the_d2_formula():
    r, state = known_radius(0.5, rho_m=1.5, r_fix_m=0.01, table=TABLE)
    assert r == pytest.approx(0.01 + 0.059 + 2 * 1.5 * math.sin(0.030 / 2))
    assert state == OCCUPIED
    r_old, state_old = known_radius(3.0, rho_m=1.5, r_fix_m=0.01, table=TABLE)
    assert r_old > r and state_old == UNKNOWN  # past the 0.20 m free-evidence cap


def test_past_the_table_an_empirical_bound_never_shrinks_the_radius():
    at_end, _ = known_radius(10.0, rho_m=1.5, r_fix_m=0.01, table=TABLE)
    r, state = known_radius(30.0, rho_m=1.5, r_fix_m=0.01, table=TABLE, empirical_m=0.05, empirical_max_age_s=60.0)
    assert r == pytest.approx(at_end) and state == UNKNOWN
    r_none, state_none = known_radius(30.0, rho_m=1.5, r_fix_m=0.01, table=TABLE)
    assert r_none == pytest.approx(at_end) and state_none == UNKNOWN
    r_out, _ = known_radius(90.0, rho_m=1.5, r_fix_m=0.01, table=TABLE, empirical_m=0.9, empirical_max_age_s=60.0)
    assert r_out == pytest.approx(at_end)  # outside its validity the sample does not apply


def test_a_known_pallet_marks_its_grown_rectangle_and_never_downgrades():
    snap = snapshot()
    snap.state[cell(3.0, 0.0)] = OCCUPIED
    rect = KnownRect((3.0, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.01, rho_m=1.5, frame_correction=(0, 0, 0))
    out = apply_known(snap, [rect], now_s=0.1, table=TABLE)
    assert out.state[cell(3.0, 0.0)] == OCCUPIED
    assert out.state[cell(3.0 + 0.3 + 0.03, 0.0)] == OCCUPIED  # inside the grown edge
    assert out.state[cell(3.0 + 0.3 + 0.12, 0.0)] == FREE
    old = apply_known(snap, [rect], now_s=5.0, table=TABLE)
    assert old.state[cell(3.0 + 0.3 + 0.12, 0.0)] == UNKNOWN  # aged past the cap: unknown, grown wider
    assert old.state[cell(3.0, 0.0)] == OCCUPIED  # an occupied cell stays occupied
    assert np.isnan(old.free_stamp[cell(3.0 + 0.3 + 0.12, 0.0)])


def test_the_pickup_zone_is_unknown_not_free():
    snap = snapshot()
    zone = KnownRect((4.0, 0.0, 1.0, 1.2, 0.0), fix_stamp_s=0.0, r_fix_m=0.0, rho_m=0.0,
                     frame_correction=(0, 0, 0), kind="zone")
    out = apply_known(snap, [zone], now_s=100.0)
    assert out.state[cell(4.0, 0.0)] == UNKNOWN


def test_a_new_correction_carries_the_rectangle_like_the_grid():
    # Stored in odometry under correction (0.5, 0, 0); a later correction (0.2, 0, 0) moves it 0.3 m back.
    rect = KnownRect((3.0, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.0, rho_m=0.0,
                     frame_correction=(0.5, 0.0, 0.0))
    out = apply_known(snapshot(correction=(0.2, 0.0, 0.0), version=1), [rect], now_s=0.0, table=TABLE)
    assert out.state[cell(2.71, 0.0)] == OCCUPIED and out.state[cell(2.95, 0.0)] == OCCUPIED
    assert out.state[cell(3.25, 0.0)] == FREE


def test_codex_counterexample_a_delivered_pallet_in_the_stopping_sweep_stops_the_truck():
    # No-hit scans keep everything FREE; the 0.144 m pallet sits 0.2 m ahead of the forks.
    stop = StoppingModel(latency_s=0.15, decel_mps2=1.5, margin_m=0.05)
    perm = DrivePermission(PermissionConfig(stop, envelope_offset_m=0.01, evidence_max_age_s=0.2))
    path = np.column_stack((np.linspace(0, 6, 121), np.zeros(121), np.zeros(121)))
    pallet = KnownRect((1.29 + 0.2 + 0.3, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.01, rho_m=1.8,
                       frame_correction=(0, 0, 0))
    plain = snapshot()
    perm.update(plain, path, FOOT, BODY, current_pose=(0, 0, 0))
    free_speed, _ = perm.allowed_speed(0.05, {"high": 0.05}, current_pose=(0, 0, 0), curvature_inv_m=0.0,
                                       direction=1, footprint=FOOT, own_footprint=BODY, speed_cap_mps=0.6)
    assert free_speed >= 0.6
    perm.update(apply_known(plain, [pallet], now_s=0.05, table=TABLE), path, FOOT, BODY, current_pose=(0, 0, 0))
    speed, why = perm.allowed_speed(0.05, {"high": 0.05}, current_pose=(0, 0, 0), curvature_inv_m=0.0,
                                    direction=1, footprint=FOOT, own_footprint=BODY, speed_cap_mps=0.6)
    radius, _ = known_radius(0.05, rho_m=1.8, r_fix_m=0.01, table=TABLE)
    assert speed < free_speed and why in ("occupied", "unknown")
    assert stop.distance_m(speed) <= 0.2 - radius + 1e-9  # it stops short of the grown pallet


def test_codex_counterexample_a_certified_corridor_lets_the_withdrawal_start():
    # After a drop the forks are in the pockets; the pallet's grown rectangle covers the blades and
    # the stopping envelope beside them (Codex v10 4th-5th rounds). Inside the checker's own
    # corridor the known pallet is not applied; outside it, it is.
    stop = StoppingModel(latency_s=0.15, decel_mps2=1.5, margin_m=0.05)
    cfg = PermissionConfig(stop, envelope_offset_m=0.01, evidence_max_age_s=0.2, step_m=0.005, envelope_ramp_m=0.0)
    perm = DrivePermission(cfg)
    # dls08_measured in the rear-axle frame: carriage face 0.944 m, blades 0.944-1.29 m at y +-0.145 (0.055 wide).
    body = Footprint(0.944, 0.17, 0.36)
    blade = Footprint(0.173, 0.173, 0.0275)
    parts = [(body, 0.0, 0.0), (blade, 0.145, 1.117), (blade, -0.145, 1.117)]
    back = np.column_stack((np.linspace(0, -0.6, 121), np.zeros(121), np.zeros(121)))
    pallet = KnownRect((1.29 - 0.30 + 0.30, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.01, rho_m=1.6,
                       frame_correction=(0, 0, 0))
    snap = snapshot()
    blocked = apply_known(snap, [pallet], now_s=0.05, table=TABLE)
    perm.update(blocked, back, parts, parts, current_pose=(0, 0, 0), direction=-1)
    stopped, _ = perm.allowed_speed(0.05, {"high": 0.05}, current_pose=(0, 0, 0), curvature_inv_m=0.0,
                                    direction=-1, footprint=parts, own_footprint=parts, speed_cap_mps=0.12)
    assert stopped == 0.0
    corridor = corridor_mask(snap, back, parts, cfg, direction=-1, until_m=0.30 + 0.10 + 0.08, tracking_m=0.006)
    certified = apply_known(snap, [pallet], now_s=0.05, exclude=corridor, table=TABLE)
    perm.update(certified, back, parts, parts, current_pose=(0, 0, 0), direction=-1)
    speed, _ = perm.allowed_speed(0.05, {"high": 0.05}, current_pose=(0, 0, 0), curvature_inv_m=0.0,
                                  direction=-1, footprint=parts, own_footprint=parts, speed_cap_mps=0.12)
    assert speed >= 0.12
    assert certified.state[cell(1.29 - 0.30 + 0.30, 0.0)] == OCCUPIED  # the pallet between the blades stays


def test_the_withdraw_certificate_uses_the_smallest_of_four_gaps():
    gaps = (0.045, 0.1275, 0.045, 0.1275)
    assert withdraw_certified(gaps, b_w=0.010, e_w=0.010, side_m=0.0185)
    assert not withdraw_certified(gaps, b_w=0.010, e_w=0.050, side_m=0.0185)  # Codex 6th: 0.0785 > 0.045
    assert not withdraw_certified(gaps, b_w=0.010, e_w=None, side_m=0.0185)  # unmeasured drift: refused
    assert not withdraw_certified((0.045, 0.1275, 0.025, 0.1475), b_w=0.010, e_w=0.010, side_m=0.0185)


def test_fork_pocket_gaps_on_the_measured_blades_and_epal6_pockets():
    geom = dict(blade_centre_m=0.145, blade_half_width_m=0.0275, blade_length_m=0.30,
                pocket_inner_m=0.0725, pocket_outer_m=0.300)
    centred = fork_pocket_gaps(0.0, 0.0, **geom)
    assert centred == pytest.approx((0.045, 0.1275, 0.045, 0.1275))
    shifted = fork_pocket_gaps(0.02, 0.0, **geom)  # the pallet 2 cm to the left
    assert shifted == pytest.approx((0.025, 0.1475, 0.065, 0.1075))
    turned = fork_pocket_gaps(0.0, 0.01, **geom)
    assert min(turned) == pytest.approx(0.045 - 0.30 * math.sin(0.01))


def test_rho_follows_the_truck_when_not_fixed():
    rect = KnownRect((3.0, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.0, rho_m=None, frame_correction=(0, 0, 0))
    near = apply_known(snapshot(), [rect], now_s=0.5, table=TABLE, current_pose=(2.0, 0.0, 0.0))
    far = apply_known(snapshot(), [rect], now_s=0.5, table=TABLE, current_pose=(-0.9, 0.0, 0.0))
    assert np.count_nonzero(far.state == OCCUPIED) + np.count_nonzero(far.state == UNKNOWN) >= np.count_nonzero(near.state != FREE)
    with pytest.raises(ValueError):
        apply_known(snapshot(), [rect], now_s=0.5, table=TABLE)


def test_codex_stage1_unknown_overrides_retained_band_cells():
    from forklift_core.control.drive_permission import RETAINED

    snap = snapshot()
    snap.state[cell(4.0, 0.0)] = RETAINED
    zone = KnownRect((4.0, 0.0, 1.0, 1.2, 0.0), fix_stamp_s=0.0, r_fix_m=0.0, rho_m=0.0,
                     frame_correction=(0, 0, 0), kind="zone")
    out = apply_known(snap, [zone], now_s=0.0)
    assert out.state[cell(4.0, 0.0)] == UNKNOWN


def test_codex_stage1_the_radius_covers_the_snapshot_reuse_horizon():
    # Pallet centre 1.816 m ahead: between ages 0.05 and 0.15 s the radius grows past a cell.
    rect = KnownRect((1.816, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.01, rho_m=None, frame_correction=(0, 0, 0))
    now_only = apply_known(snapshot(), [rect], now_s=0.05, table=TABLE, current_pose=(0, 0, 0))
    ahead = apply_known(snapshot(), [rect], now_s=0.05, table=TABLE, current_pose=(0, 0, 0), horizon_s=0.2)
    later = apply_known(snapshot(), [rect], now_s=0.25, table=TABLE, current_pose=(0, 0, 0))
    assert np.array_equal(ahead.state != FREE, later.state != FREE)
    assert np.count_nonzero(ahead.state != FREE) >= np.count_nonzero(now_only.state != FREE)
