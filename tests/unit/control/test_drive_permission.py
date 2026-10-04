"""Drive permission from the obstacle grid (priority-5 plan D4); synthetic grids only."""

import math

import numpy as np
import pytest

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, GridSnapshot
from forklift_core.planning.geometry import Footprint

FOOT = Footprint(1.29, 0.17, 0.36)
BODY = Footprint(0.75, 0.17, 0.36)
STOP = StoppingModel(latency_s=0.3, decel_mps2=1.0, margin_m=0.05)
CONFIG = PermissionConfig(STOP, envelope_offset_m=0.05, evidence_max_age_s=0.2)


def snapshot(fill=FREE, stamp=0.0, newest=0.0):
    state = np.full((200, 120), fill, dtype=np.uint8)  # x -1..9, y -3..3
    return GridSnapshot(state, stamp, (0.0, 0.0, 0.0), 0, 1, {"low": newest}, -1.0, -3.0, 0.05)


def set_cell(snap, x, y, value):
    snap.state[int(math.floor((x + 1.0) / 0.05)), int(math.floor((y + 3.0) / 0.05))] = value


STRAIGHT = np.column_stack((np.linspace(0, 6, 121), np.zeros(121), np.zeros(121)))


def test_the_stopping_model_inverts():
    for v in (0.0, 0.1, 0.3, 0.6):
        assert math.isclose(STOP.speed_for(STOP.distance_m(v)), v, abs_tol=1e-12)
    assert STOP.speed_for(0.01) == 0.0


def test_a_free_corridor_is_verified_to_the_lookahead():
    perm = DrivePermission(CONFIG)
    snap = snapshot()
    e = perm.evaluate(snap, STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert e.blocked is None and math.isclose(e.verified_m, CONFIG.lookahead_m)
    speed, _ = perm.allowed_speed(0.05, {"low": 0.0})
    assert math.isclose(speed, STOP.speed_for(3.0))


def test_the_truck_slows_and_stops_before_an_obstacle():
    perm = DrivePermission(CONFIG)
    snap = snapshot()
    set_cell(snap, 3.0, 0.0, OCCUPIED)
    e = perm.evaluate(snap, STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert e.blocked == "occupied"
    # The inflated front (1.29 + 0.05) reaches the cell's face at x = 3.0: about 1.65 m.
    assert 1.55 <= e.verified_m <= 1.7
    speeds = []
    for k in range(40):
        perm.advance(0.05)
        speeds.append(perm.allowed_speed(0.05, {"low": 0.0})[0])
    assert speeds == sorted(speeds, reverse=True) and speeds[-1] == 0.0
    # Where speed first hits zero, the stopping distance from the previous step still fit.
    first_zero = speeds.index(0.0)
    assert (first_zero + 1) * 0.05 <= e.verified_m


def test_unknown_ahead_blocks_and_own_cells_do_not():
    perm = DrivePermission(CONFIG)
    snap = snapshot()
    # Under the truck nothing is seen; that alone must not stop it.
    for x in np.arange(-0.15, 0.75, 0.05):
        for y in np.arange(-0.35, 0.36, 0.05):
            set_cell(snap, x, y, UNKNOWN)
    assert perm.evaluate(snap, STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0)).blocked is None
    set_cell(snap, 2.5, 0.3, UNKNOWN)
    e = perm.evaluate(snap, STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert e.blocked == "unknown" and e.verified_m < 1.3


def test_stale_evidence_and_silent_sensors_stop_the_truck():
    perm = DrivePermission(CONFIG)
    perm.evaluate(snapshot(newest=0.0), STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert perm.allowed_speed(0.19, {"low": 0.19})[0] > 0
    assert perm.allowed_speed(0.21, {"low": 0.21}) == (0.0, "evidence_stale")
    perm.evaluate(snapshot(stamp=1.0, newest=1.0), STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert perm.allowed_speed(1.05, {"low": 1.0, "rear": 0.7}) == (0.0, "sensor_silent:rear")
    with pytest.raises(ValueError):
        perm.evaluate(snapshot(), np.zeros((0, 3)), FOOT, BODY, current_pose=(0, 0, 0))
