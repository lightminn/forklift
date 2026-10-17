"""Shadow-band memory (priority-5 plan D4 delta, 2026-10-05); synthetic grids only.

The L3b v17 stop is the motivating case: one cell straddling the carriage
face stays UNKNOWN because every beam reaching its outside part ends on the
face. The tests keep the delta's limits: memory only in the band, only with
its own observation time, only while the surrounding cells are freshly FREE,
never across OCCUPIED, and expiring with the odometry table.
"""

import math

import numpy as np

from forklift_core.control.drive_permission import (
    RETAINED,
    DrivePermission,
    PermissionConfig,
    StoppingModel,
)
from forklift_core.control.shadow_memory import ShadowMemory
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
from forklift_core.planning.geometry import Footprint

BODY = Footprint(0.884, 0.17, 0.36)
FOOT = Footprint(1.29, 0.17, 0.36)
ERROR = AgeErrorTable((0.1, 1.0, 10.0), (0.005, 0.01, 0.02), (0.001, 0.002, 0.004))
POSE = (0.0, 0.0, 0.0)
FACE_CELL = (0.89, 0.22)  # straddles the face at 0.884 (cell 0.85..0.90)


def snapshot(stamp, fill=FREE, correction=(0.0, 0.0, 0.0)):
    state = np.full((200, 120), fill, dtype=np.uint8)  # x -1..9, y -3..3
    free_stamp = np.where(state == FREE, stamp, np.nan)
    return GridSnapshot(state, free_stamp, stamp, correction, 0, 1, {"low": stamp}, -1.0, -3.0, 0.05)


def cell(x, y):
    return int(math.floor((x + 1.0) / 0.05)), int(math.floor((y + 3.0) / 0.05))


def put(snap, x, y, value):
    i, j = cell(x, y)
    snap.state[i, j] = value
    snap.free_stamp[i, j] = snap.stamp_s if value == FREE else np.nan


def blind_inside(snap):
    """Cells wholly inside the body are never FREE (the grid withholds them)."""
    for x in np.arange(-0.15, 0.85, 0.05):
        for y in np.arange(-0.325, 0.33, 0.05):
            put(snap, x + 0.01, y + 0.01, UNKNOWN)
    return snap


def test_startup_premise_lets_the_face_cell_pass():
    mem = ShadowMemory(0.05, ERROR)
    snap = blind_inside(snapshot(0.0))
    put(snap, *FACE_CELL, UNKNOWN)
    out = mem.apply(snap, POSE, BODY)
    assert out.state[cell(*FACE_CELL)] == RETAINED
    assert mem.stats["startup_cells"] > 0


def test_a_cell_seen_free_is_kept_once_it_falls_into_the_shadow():
    mem = ShadowMemory(0.05, ERROR)
    first = blind_inside(snapshot(0.0))
    mem.apply(first, POSE, BODY)
    later = blind_inside(snapshot(0.5))
    put(later, *FACE_CELL, UNKNOWN)
    out = mem.apply(later, POSE, BODY)
    assert out.state[cell(*FACE_CELL)] == RETAINED


def test_occupied_is_never_retained_and_is_forgotten():
    mem = ShadowMemory(0.05, ERROR)
    mem.apply(blind_inside(snapshot(0.0)), POSE, BODY)
    hit = blind_inside(snapshot(0.1))
    put(hit, *FACE_CELL, OCCUPIED)
    assert mem.apply(hit, POSE, BODY).state[cell(*FACE_CELL)] == OCCUPIED
    again = blind_inside(snapshot(0.2))
    put(again, *FACE_CELL, UNKNOWN)
    assert mem.apply(again, POSE, BODY).state[cell(*FACE_CELL)] == UNKNOWN


def test_outside_the_band_nothing_is_remembered():
    mem = ShadowMemory(0.05, ERROR)
    mem.apply(blind_inside(snapshot(0.0)), POSE, BODY)
    later = blind_inside(snapshot(0.5))
    put(later, 1.10, 0.22, UNKNOWN)  # 0.2 m ahead of the face
    assert mem.apply(later, POSE, BODY).state[cell(1.10, 0.22)] == UNKNOWN


def test_an_unobserved_neighbour_outside_the_band_withdraws_support():
    mem = ShadowMemory(0.05, ERROR)
    mem.apply(blind_inside(snapshot(0.0)), POSE, BODY)
    later = blind_inside(snapshot(0.5))
    put(later, *FACE_CELL, UNKNOWN)
    put(later, 0.94, 0.22, UNKNOWN)  # the next cell out is in the band too
    put(later, 0.99, 0.22, UNKNOWN)  # and the one beyond is not: no fresh boundary
    out = mem.apply(later, POSE, BODY)
    assert out.state[cell(*FACE_CELL)] == UNKNOWN
    assert out.state[cell(0.94, 0.22)] == UNKNOWN


def test_memory_expires_past_the_error_table():
    mem = ShadowMemory(0.05, ERROR)
    mem.apply(blind_inside(snapshot(0.0)), POSE, BODY)
    late = blind_inside(snapshot(10.5))
    put(late, *FACE_CELL, UNKNOWN)
    assert mem.apply(late, POSE, BODY).state[cell(*FACE_CELL)] == UNKNOWN


def test_observation_time_is_not_renewed_by_the_memory():
    mem = ShadowMemory(0.05, ERROR)
    mem.apply(blind_inside(snapshot(0.0)), POSE, BODY)
    for t in (2.0, 5.0, 9.0):
        snap = blind_inside(snapshot(t))
        put(snap, *FACE_CELL, UNKNOWN)
        assert mem.apply(snap, POSE, BODY).state[cell(*FACE_CELL)] == RETAINED
    snap = blind_inside(snapshot(10.5))
    put(snap, *FACE_CELL, UNKNOWN)
    assert mem.apply(snap, POSE, BODY).state[cell(*FACE_CELL)] == UNKNOWN


def test_a_correction_change_moves_the_memory_conservatively():
    mem = ShadowMemory(0.05, ERROR)
    mem.apply(blind_inside(snapshot(0.0)), POSE, BODY)
    # A whole-cell shift keeps every remembered cell; the truck pose moves with it.
    moved = blind_inside(snapshot(0.5, correction=(0.05, 0.0, 0.0)))
    for x in np.arange(-0.15, 0.85, 0.05):
        for y in np.arange(-0.325, 0.33, 0.05):
            put(moved, x + 0.01, y + 0.01, FREE)
            put(moved, x + 0.06, y + 0.01, UNKNOWN)
    put(moved, FACE_CELL[0] + 0.05, FACE_CELL[1], UNKNOWN)
    out = mem.apply(moved, (0.05, 0.0, 0.0), BODY)
    assert mem.stats["moves"] == 1
    assert out.state[cell(FACE_CELL[0] + 0.05, FACE_CELL[1])] == RETAINED


def permission(band):
    return DrivePermission(PermissionConfig(StoppingModel(0.15, 1.5, 0.05), envelope_offset_m=0.01,
                                            evidence_max_age_s=0.2, envelope_ramp_m=0.0, shadow_band_m=band))


PATH = np.column_stack((np.linspace(0, 3, 61), np.zeros(61), np.zeros(61)))


def retained_snapshot():
    mem = ShadowMemory(0.05, ERROR)
    mem.apply(blind_inside(snapshot(0.0)), POSE, BODY)
    snap = blind_inside(snapshot(0.5))
    put(snap, *FACE_CELL, UNKNOWN)
    return snap, mem.apply(snap, POSE, BODY)


def test_the_permission_passes_retained_and_ages_it_with_its_support():
    raw, kept = retained_snapshot()
    perm = permission(0.05)
    strict = perm.update(raw, PATH, FOOT, BODY, current_pose=POSE, direction=1)
    assert strict.blocked == "unknown" and strict.verified_m == 0.0
    # The RETAINED stamp is its support's FREE time (0.5), not the memory's (0.0).
    assert kept.free_stamp[cell(*FACE_CELL)] == 0.5
    check = perm.update(kept, PATH, FOOT, BODY, current_pose=POSE, direction=1)
    assert check.blocked is None and check.verified_m > 2.5
    speed, why = perm.allowed_speed(0.55, {"low": 0.5}, current_pose=POSE, curvature_inv_m=0.0, direction=1,
                                    footprint=FOOT, own_footprint=BODY, speed_cap_mps=0.6)
    assert why == "ok" and speed > 0.5
    assert perm.allowed_speed(0.75, {"low": 0.74}, current_pose=POSE, curvature_inv_m=0.0, direction=1,
                              footprint=FOOT, own_footprint=BODY, speed_cap_mps=0.6) == (0.0, "evidence_stale")


def test_retained_is_refused_without_the_band_or_once_the_truck_has_moved_off_it():
    _, kept = retained_snapshot()
    assert permission(0.0).update(kept, PATH, FOOT, BODY, current_pose=POSE, direction=1).blocked == "unknown"
    # Backed off 0.12 m: the cell is now beyond the band in front of the face
    # and has to be freshly FREE again.
    back = (-0.12, 0.0, 0.0)
    perm = permission(0.05)
    path = np.column_stack((np.linspace(-0.12, 3, 61), np.zeros(61), np.zeros(61)))
    assert perm.update(kept, path, FOOT, BODY, current_pose=back, direction=1).blocked == "unknown"


def test_a_startup_cell_without_fresh_support_nearby_is_not_retained():
    mem = ShadowMemory(0.05, ERROR)
    snap = blind_inside(snapshot(0.0, fill=UNKNOWN))
    out = mem.apply(snap, POSE, BODY)
    assert not (out.state == RETAINED).any()


def test_a_cell_the_truck_backs_off_from_keeps_the_covered_evidence():
    mem = ShadowMemory(0.05, ERROR)
    # Long after start-up the truck has driven 0.10 m further: the cell now
    # straddling the face was wholly under the carriage a snapshot ago.
    forward = (0.10, 0.0, 0.0)
    mem.started = True
    first = blind_inside(snapshot(20.0))
    for x in np.arange(-0.05, 0.95, 0.05):
        for y in np.arange(-0.325, 0.33, 0.05):
            put(first, x + 0.01, y + 0.01, UNKNOWN)
    mem.apply(first, forward, BODY)
    back = blind_inside(snapshot(20.1))
    put(back, *FACE_CELL, UNKNOWN)
    out = mem.apply(back, POSE, BODY)
    assert out.state[cell(*FACE_CELL)] == RETAINED


def _reference_stamps(cand, res, fresh, stamps, whole, nx, ny):
    """The original repeated-scan rule, kept as the reference for the array version."""
    cand = dict(cand)
    changed = True
    while changed and cand:
        changed = False
        for (a, b), r in list(cand.items()):
            if ShadowMemory._support(a, b, r, res, fresh, whole, cand, nx, ny) is None:
                del cand[(a, b)]
                changed = True
    stamp = {}
    for (a, b), r in cand.items():
        fresh_n, _ = ShadowMemory._support(a, b, r, res, fresh, whole, cand, nx, ny)
        stamp[(a, b)] = min((float(stamps[p, q]) for p, q in fresh_n), default=np.inf)
    changed = True
    while changed:
        changed = False
        for (a, b), r in cand.items():
            _, ret_n = ShadowMemory._support(a, b, r, res, fresh, whole, cand, nx, ny)
            low = min([stamp[(a, b)]] + [stamp[n] for n in ret_n])
            if low < stamp[(a, b)]:
                stamp[(a, b)] = low
                changed = True
    return stamp


def test_the_array_support_matches_the_repeated_scan_on_random_grids():
    rng = np.random.default_rng(7)
    for _ in range(300):
        nx = ny = 14
        fresh = rng.random((nx, ny)) < rng.uniform(0.3, 0.9)
        stamps = rng.uniform(0.0, 1.0, (nx, ny))
        whole = {(int(a), int(b)) for a, b in rng.integers(0, nx, (rng.integers(0, 30), 2))}
        cand = {}
        for a, b in rng.integers(0, nx, (rng.integers(1, 25), 2)):
            if (int(a), int(b)) not in whole and not fresh[a, b]:
                cand[(int(a), int(b))] = float(rng.uniform(0.02, 0.2))
        got = ShadowMemory._retained_stamps(cand, 0.05, fresh, stamps, whole, nx, ny)
        assert got == _reference_stamps(cand, 0.05, fresh, stamps, whole, nx, ny)


def test_a_dropped_candidate_inside_the_outline_still_supports():
    # Codex checkpoint 10 P1 counterexample.
    fresh = np.ones((3, 3), dtype=bool)
    fresh[0, 1] = fresh[1, 1] = False
    stamps = np.ones((3, 3))
    whole = {(0, 1)}
    cand = {(1, 1): 0.025, (0, 1): 0.025}
    got = ShadowMemory._retained_stamps(cand, 0.05, fresh, stamps, whole, 3, 3)
    assert got == _reference_stamps(cand, 0.05, fresh, stamps, whole, 3, 3) == {(1, 1): 1.0}


def test_the_reach_edge_is_judged_like_the_repeated_scan():
    # Codex checkpoint 10 P2: np.hypot and math.hypot differ in the last bit.
    import math as m

    res = 0.005
    fresh = np.ones((65, 65), dtype=bool)
    fresh[32, 32] = fresh[50, 60] = False
    stamps = np.ones((65, 65))
    cand = {(32, 32): res * m.hypot(17, 27)}
    got = ShadowMemory._retained_stamps(cand, res, fresh, stamps, set(), 65, 65)
    assert got == _reference_stamps(cand, res, fresh, stamps, set(), 65, 65)


def test_random_grids_with_candidates_inside_the_outline_match_too():
    rng = np.random.default_rng(11)
    for _ in range(300):
        nx = ny = 12
        fresh = rng.random((nx, ny)) < rng.uniform(0.3, 0.9)
        stamps = rng.uniform(0.0, 1.0, (nx, ny))
        whole = {(int(a), int(b)) for a, b in rng.integers(0, nx, (rng.integers(0, 30), 2))}
        cand = {(int(a), int(b)): float(rng.uniform(0.02, 0.2))
                for a, b in rng.integers(0, nx, (rng.integers(1, 25), 2)) if not fresh[a, b]}
        got = ShadowMemory._retained_stamps(cand, 0.05, fresh, stamps, whole, nx, ny)
        assert got == _reference_stamps(cand, 0.05, fresh, stamps, whole, nx, ny)
