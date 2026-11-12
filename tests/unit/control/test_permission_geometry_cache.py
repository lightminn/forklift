"""Teleop pacing (docs/design/2026-11-12-teleop-pacing.md, lead decision 2026-11-12): the numba walk's own-outline
lookup and shadow band are cached on an exact key -- the grid's frame (origin, resolution, shape), the own outline's
pose and parts, the band width, every float by its bits. These tests show the cache recomputes on every change of the
key, gives the uncached arrays, never lets a band built on a grid with RETAINED cells stand in for one without, leaves
its arrays as the walk found them, and that a sequence of calls with the cache on and off gives the same outputs and
the same permission state (start-band anchor, release statistics, path check, last stop arc)."""

import copy
import importlib.util
import math

import numpy as np
import pytest

from forklift_core import kernels
from forklift_core.control import drive_permission as dp
from forklift_core.control.drive_permission import (
    RETAINED,
    DrivePermission,
    PermissionConfig,
    StoppingModel,
)
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, GridSnapshot
from forklift_core.perf import differences
from forklift_core.planning.geometry import Footprint

pytestmark = pytest.mark.skipif(importlib.util.find_spec("numba") is None, reason="the cache serves the numba walk")

HULL = Footprint(1.29, 0.17, 0.36)
BODY = Footprint(0.944, 0.17, 0.36)
PARTS = ((BODY, 0.0, 0.0), (Footprint(0.21, 0.21, 0.05), 0.2, 1.08), (Footprint(0.21, 0.21, 0.05), -0.2, 1.08))


def config(**kw):
    base = dict(stopping=StoppingModel(0.15, 1.5, 0.05), envelope_offset_m=0.01, evidence_max_age_s=0.5,
                envelope_ramp_m=0.2, shadow_band_m=0.05, start_band_m=0.08)
    base.update(kw)
    return PermissionConfig(**base)


def grid(rng, *, retained=0.02, occupied=0.004, nx=160, ny=160, origin=(-4.0, -4.0), res=0.05, stamp_s=0.3,
         version=0, clear_m=0.0, wall_x_m=None):
    """Random cells; within clear_m of (0, 0) all FREE (RETAINED kept), an OCCUPIED wall across x = wall_x_m."""
    u = rng.random((nx, ny))
    state = np.full((nx, ny), UNKNOWN, dtype=np.uint8)
    state[u < 0.93] = FREE
    state[u < occupied] = OCCUPIED
    cx = origin[0] + (np.arange(nx) + 0.5) * res
    cy = origin[1] + (np.arange(ny) + 0.5) * res
    near = np.hypot(cx[:, None], cy[None, :]) < clear_m
    state[near] = FREE
    if wall_x_m is not None:
        state[(np.abs(cx[:, None] - wall_x_m) < 0.05) & (np.abs(cy[None, :]) < 0.8)] = OCCUPIED
    state[u > 1 - retained] = RETAINED
    stamp = np.where((state == FREE) | (state == RETAINED), rng.uniform(0.1, 0.3, (nx, ny)), np.nan)
    return GridSnapshot(state, stamp, stamp_s, (0.0, 0.0, 0.0), version, 1, {"high": stamp_s}, origin[0], origin[1],
                        res)


@pytest.fixture(autouse=True)
def numba_mode():
    previous = kernels.set_mode("numba")
    kernels.warm_up()
    dp.clear_geometry_cache()
    yield
    dp.clear_geometry_cache()
    kernels.set_mode(previous)


@pytest.fixture
def counted(monkeypatch):
    """How often the own lookup and the band's cells were really computed."""
    from forklift_core.control import permission_numba as nb

    n = {"own": 0, "band": 0}
    real_own, real_cells = nb.own_lookup, dp.shape_cells

    def own(*a):
        n["own"] += 1
        return real_own(*a)

    def cells(*a, **k):
        n["band"] += 1
        return real_cells(*a, **k)

    monkeypatch.setattr(nb, "own_lookup", own)
    monkeypatch.setattr(dp, "shape_cells", cells)
    return n


def same(a, b) -> bool:
    if isinstance(a, np.ndarray):
        return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()
    if isinstance(a, tuple):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return type(a) is type(b) and a == b


def uncached(fn):
    on = dp.GEOMETRY_CACHE
    dp.GEOMETRY_CACHE = False
    try:
        return fn()
    finally:
        dp.GEOMETRY_CACHE = on


def frame(snap, *, origin=None, res=None, shape=None):
    nx, ny = shape or snap.state.shape
    state = np.full((nx, ny), FREE, dtype=np.uint8)
    state[: min(nx, snap.state.shape[0]), : min(ny, snap.state.shape[1])] = snap.state[:nx, :ny]
    ox, oy = origin or (snap.origin_x_m, snap.origin_y_m)
    return GridSnapshot(state, np.full((nx, ny), 0.2), snap.stamp_s, snap.correction, snap.correction_version, 1,
                        {}, ox, oy, res or snap.resolution_m)


def test_the_own_lookup_recomputes_on_every_change_of_its_key(counted):
    rng = np.random.default_rng(1)
    snap = grid(rng)
    pose = (0.31, -0.12, 0.4)
    variants = [
        (snap, pose, HULL),
        (frame(snap, origin=(-4.0 + 1e-12, -4.0)), pose, HULL),
        (frame(snap, origin=(-4.0, -3.95)), pose, HULL),
        (frame(snap, res=0.05 + 1e-12), pose, HULL),
        (frame(snap, shape=(150, 160)), pose, HULL),
        (snap, (np.nextafter(0.31, 1.0), -0.12, 0.4), HULL),
        (snap, (0.31, -0.12 + 0.05, 0.4), HULL),
        (snap, (0.31, -0.12, 0.4 + 1e-12), HULL),
        (snap, (0.31, -0.12, 0.0), HULL),
        (snap, (0.31, -0.12, -0.0), HULL),  # a signed zero is another key (sin(-0.0) is -0.0)
        (snap, pose, Footprint(1.29 + 1e-9, 0.17, 0.36)),
        (snap, pose, BODY),
        (snap, pose, PARTS),
        (snap, pose, ((BODY, 0.0, 0.0), (Footprint(0.21, 0.21, 0.05), 0.2 + 1e-9, 1.08),
                      (Footprint(0.21, 0.21, 0.05), -0.2, 1.08))),
    ]
    for k, (s, p, fp) in enumerate(variants, start=1):
        got = dp.own_lookup_cached(s, p, fp)
        assert counted["own"] == k, f"variant {k - 1} hit an entry of another key"
        assert same(got, uncached(lambda: dp.own_lookup_cached(s, p, fp)))
        counted["own"] = k  # the uncached call above counted too
        again = dp.own_lookup_cached(s, tuple(p), fp)
        assert counted["own"] == k and again is got  # the same key: no work, the same arrays
    # the same values in another container (a numpy pose, a list of parts) are the same key
    dp.own_lookup_cached(snap, np.array(pose), HULL)
    dp.own_lookup_cached(snap, pose, list(PARTS))
    assert counted["own"] == len(variants)


def test_the_band_recomputes_on_every_change_and_reads_the_grid_each_call(counted):
    rng = np.random.default_rng(2)
    snap = grid(rng)
    clear = grid(rng, retained=0.0)  # the same frame without RETAINED cells
    pose = (0.2, 0.1, -0.3)
    first = dp.shadow_band(snap, pose, HULL, 0.05)
    assert first.any() and counted["band"] == 1
    assert dp.shadow_band(snap, pose, HULL, 0.05) is first and counted["band"] == 1
    # the same frame, outline and width but no RETAINED cell: no band, whatever the cache holds
    none = dp.shadow_band(clear, pose, HULL, 0.05)
    assert not none.any() and none.shape == clear.state.shape
    assert same(none, uncached(lambda: dp.shadow_band(clear, pose, HULL, 0.05)))
    # a RETAINED cell appears in a later snapshot of the same frame: the band comes back
    later = copy.deepcopy(clear)
    later.state[3, 3] = RETAINED
    assert same(dp.shadow_band(later, pose, HULL, 0.05), first)
    n = counted["band"]
    for args in ((snap, pose, HULL, 0.06), (snap, (0.2, 0.1, -0.3 + 1e-12), HULL, 0.05),
                 (snap, pose, PARTS, 0.05), (frame(snap, origin=(-3.9, -4.0)), pose, HULL, 0.05),
                 (frame(snap, res=0.04), pose, HULL, 0.05), (frame(snap, shape=(160, 150)), pose, HULL, 0.05)):
        got = dp.shadow_band(*args)
        n += 1
        assert counted["band"] == n
        assert same(got, uncached(lambda: dp.shadow_band(*args)))
        n = counted["band"]
    # off: no pose, no outline or no width gives no band and computes nothing
    for args in ((snap, None, HULL, 0.05), (snap, pose, None, 0.05), (snap, pose, HULL, 0.0)):
        assert not dp.shadow_band(*args).any()
    assert counted["band"] == n


def sequence(seed: int) -> tuple[list, list]:
    """A teleop-like run: two permissions (the core's watch and the teleop permit) on the same snapshots, the truck
    standing, then creeping, a snapshot every few calls (RETAINED cells come and go, the grid's origin rolls, a
    correction raises the version), stop arcs both ways and path checks. Returns (outputs, states)."""
    rng = np.random.default_rng(seed)
    cfg = config()
    watch, permit = DrivePermission(cfg), DrivePermission(cfg)
    outs, states = [], []
    pose = np.array([0.0, 0.0, float(rng.uniform(-0.3, 0.3))])
    origin, version, snap = (-4.0, -4.0), 0, None
    for k in range(48):
        if k % 4 == 0:
            if k % 16 == 12:
                origin = (origin[0] + 0.05, origin[1])  # the rolling grid moved by a cell
            if k % 20 == 15:
                version += 1
            snap = grid(rng, retained=[0.0, 0.02][(k // 4) % 2], origin=origin, stamp_s=0.3 + 0.1 * k,
                        version=version, clear_m=2.5, wall_x_m=[None, 1.5, 1.32][(k // 4) % 3])
            snap.free_stamp[:] = np.where(np.isfinite(snap.free_stamp), 0.3 + 0.1 * k - 0.05, np.nan)
            path = np.array([pose + (d * math.cos(pose[2]), d * math.sin(pose[2]), 0.0) for d in (0, 0.5, 1.0, 1.5)])
            watch.update(snap, path, HULL, BODY, current_pose=tuple(pose))
        standing = k < 12 or k % 9 == 0
        for perm in (watch, permit):
            perm.note_standing(standing, tuple(pose), version)
        now = 0.3 + 0.1 * (k - k % 4) + 0.01
        shape = [HULL, PARTS][k % 2]
        out = [watch.allowed_speed(now, {"lidar": now}, current_pose=tuple(pose), curvature_inv_m=0.0,
                                   direction=1, footprint=HULL, own_footprint=BODY, speed_cap_mps=0.13)]
        for curv in (-0.5, 0.0, 0.5):
            out.append(permit.check_arc(snap, tuple(pose), curv, 1 if k % 3 else -1, shape, BODY, 0.13))
        out.append(permit.check_path(snap, path, HULL, BODY, current_pose=tuple(pose), direction=1))
        outs.append(out)
        states.append([copy.deepcopy({key: getattr(p, key, None) for key in ("release_anchor", "release_stats",
                                                                              "path_check", "last_estop",
                                                                              "_driven_since_m", "_standing")})
                       for p in (watch, permit)])
        if not standing:
            pose = pose + (0.004 * math.cos(pose[2]), 0.004 * math.sin(pose[2]), 0.002)
            watch.advance(0.004)
    return outs, states


@pytest.mark.parametrize("seed", range(6))
def test_a_sequence_gives_the_same_outputs_and_state_with_and_without_the_cache(seed):
    on = sequence(seed)
    dp.clear_geometry_cache()
    off = uncached(lambda: sequence(seed))
    assert differences(on[0], off[0], limit=5) == []
    assert differences(on[1], off[1], limit=5) == []
    assert any(o[0][0] > 0 for o in on[0]) and any(o[0][0] == 0 for o in on[0])  # both outcomes were met


def test_the_walk_leaves_the_cached_arrays_as_it_found_them():
    sequence(3)
    entries = dp.geometry_cache_entries()
    assert entries
    assert {key[0] for key, _, _ in entries} >= {"own", "band"}
    for key, value, compute in entries:
        assert same(value, compute()), key


def test_the_cache_stays_bounded():
    rng = np.random.default_rng(5)
    snap = grid(rng)
    for k in range(5 * dp.GEOMETRY_CACHE_MAX):
        dp.own_lookup_cached(snap, (0.001 * k, 0.0, 0.0), HULL)
        dp.shadow_band(snap, (0.001 * k, 0.0, 0.0), HULL, 0.05)
    assert len(dp.geometry_cache_entries()) <= dp.GEOMETRY_CACHE_MAX
