"""Real-time plan S1 (docs/plans/2026-10-17-realtime-sim.md, D2, V1): the vectorised permission and shadow-memory
kernels give the reference's results bit for bit on random grids -- shapes, directions, curvatures, the grid's
edge, cell-boundary poses, RETAINED and UNKNOWN cells, unknown-passing walks, extra margins -- and the shadow
memory carries the same state from call to call."""

import copy
import importlib.util
import math

import numpy as np
import pytest

from forklift_core import kernels
from forklift_core.control.drive_permission import (
    RETAINED,
    DrivePermission,
    PermissionConfig,
    StoppingModel,
    arc_poses,
)
from forklift_core.control.shadow_memory import ShadowMemory, whole_cells
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
from forklift_core.planning.geometry import Footprint
from forklift_core.perf import differences

HULL = Footprint(1.29, 0.17, 0.36)
LOADED = Footprint(1.56, 0.17, 0.40)
BODY = Footprint(0.944, 0.17, 0.36)
PARTS = [(BODY, 0.0, 0.0), (Footprint(0.21, 0.21, 0.05), 0.2, 1.08), (Footprint(0.21, 0.21, 0.05), -0.2, 1.08)]
ERROR = AgeErrorTable((0.1, 1.0, 10.0), (0.005, 0.01, 0.02), (0.001, 0.002, 0.004))


def config(**kw):
    base = dict(stopping=StoppingModel(0.15, 1.5, 0.05), envelope_offset_m=0.01, evidence_max_age_s=0.2,
                envelope_ramp_m=0.0, shadow_band_m=0.05)
    base.update(kw)
    return PermissionConfig(**base)


def grid(rng, *, free_share=0.9, occupied_share=0.01, retained_share=0.02, nx=160, ny=160, origin=(-4.0, -4.0)):
    u = rng.random((nx, ny))
    state = np.full((nx, ny), UNKNOWN, dtype=np.uint8)
    state[u < free_share] = FREE
    state[u < occupied_share] = OCCUPIED
    state[(u > 1 - retained_share)] = RETAINED
    stamp = np.where((state == FREE) | (state == RETAINED), rng.uniform(0.0, 0.2, (nx, ny)), np.nan)
    return GridSnapshot(state, stamp, 0.2, (0.0, 0.0, 0.0), 0, 1, {"high": 0.2}, origin[0], origin[1], 0.05)


MODES = ("reference", "numpy") + (("numba",) if importlib.util.find_spec("numba") is not None else ())


def both(fn):
    """fn() under the reference, then under every other implementation that can run here."""
    out = []
    for mode in MODES:
        previous = kernels.set_mode(mode)
        try:
            out.append(fn())
        finally:
            kernels.set_mode(previous)
    return out


def boundary_pose(rng):
    """A pose on a cell boundary, one ulp either side or exactly on it."""
    x = rng.integers(-20, 20) * 0.05 - 4.0
    x = [np.nextafter(x, -np.inf), x, np.nextafter(x, np.inf)][rng.integers(3)]
    y = rng.integers(-20, 20) * 0.05 - 4.0
    return (float(x), float(y), float(rng.choice([0.0, math.pi / 2, -math.pi / 4, rng.uniform(-math.pi, math.pi)])))


@pytest.mark.parametrize("seed", range(40))
def test_stop_arcs_and_paths_match_the_reference(seed):
    rng = np.random.default_rng(seed)
    snap = grid(rng, free_share=rng.choice([0.6, 0.95, 1.0]), occupied_share=rng.choice([0.0, 0.002, 0.02]))
    cfg = config(envelope_ramp_m=float(rng.choice([0.0, 0.2])), step_m=float(rng.choice([0.02, 0.005])))
    pose = boundary_pose(rng) if seed % 3 == 0 else (float(rng.uniform(-1, 1)), float(rng.uniform(-1, 1)),
                                                     float(rng.uniform(-math.pi, math.pi)))
    shape = [HULL, LOADED, PARTS][seed % 3]
    own = BODY if shape is HULL else shape
    direction = int(rng.choice([1, -1]))
    kappa = float(rng.choice([0.0, 0.05, -0.39, rng.uniform(-0.4, 0.4)]))
    cap = float(rng.uniform(0.0, 1.2))

    def arc():
        return DrivePermission(cfg).check_arc(snap, pose, kappa, direction, shape, own, cap)

    a, *rest = both(arc)
    for b in rest:
        assert differences(a, b, limit=5) == []
    path, _ = arc_poses(pose, kappa, direction, float(rng.uniform(0.3, 5.0)), 0.05)
    kw = dict(current_pose=pose, direction=direction, unknown_passes=bool(seed % 4 == 1),
              extra_margin_m=float(rng.choice([0.0, 0.03])), extra_ramp_m=rng.choice([None, 0.5]),
              lookahead_m=rng.choice([None, 1.0, 4.0]))

    def walk():
        return DrivePermission(cfg).check_path(snap, path, shape, own, **kw)

    a, *rest = both(walk)
    for b in rest:
        assert differences(a, b, limit=5) == []


def test_a_walk_off_the_grid_matches_the_reference():
    rng = np.random.default_rng(7)
    snap = grid(rng, free_share=1.0, occupied_share=0.0, retained_share=0.0, nx=60, ny=60, origin=(-1.5, -1.5))
    for direction in (1, -1):
        path, _ = arc_poses((0.0, 0.0, 0.3), 0.1, direction, 4.0, 0.05)
        for unknown_passes in (False, True):
            def walk():
                return DrivePermission(config()).check_path(snap, path, PARTS, PARTS, current_pose=(0.0, 0.0, 0.3),
                                                            direction=direction, unknown_passes=unknown_passes)

            a, *rest = both(walk)
            for b in rest:
                assert differences(a, b, limit=5) == []
            assert a.blocked == ("edge" if not unknown_passes else a.blocked)


@pytest.mark.parametrize("seed", range(20))
def test_whole_cells_match_the_reference_in_order(seed):
    rng = np.random.default_rng(100 + seed)
    snap = grid(rng)
    pose = boundary_pose(rng)
    a, *rest = both(lambda: whole_cells(snap, pose, [HULL, BODY][seed % 2]))
    for b in rest:
        assert a == b and list(a) == list(b)  # the same set, iterating alike


@pytest.mark.parametrize("seed", range(12))
def test_the_shadow_memory_carries_the_same_state_call_after_call(seed):
    rng = np.random.default_rng(200 + seed)
    memories = {"reference": ShadowMemory(0.05, ERROR), "numpy": ShadowMemory(0.05, ERROR)}
    pose = np.array([0.0, 0.0, float(rng.uniform(-0.3, 0.3))])
    correction = (0.0, 0.0, 0.0)
    for k in range(25):
        snap = grid(rng, free_share=float(rng.uniform(0.5, 0.95)), occupied_share=0.005, retained_share=0.0)
        snap = GridSnapshot(snap.state, snap.free_stamp, 0.1 * k, correction, k, 1, {"high": 0.1 * k},
                            snap.origin_x_m, snap.origin_y_m, 0.05)
        snap.free_stamp[:] = np.where(snap.state == FREE, 0.1 * k - rng.uniform(0, 0.15, snap.state.shape), np.nan)
        extra = {(int(rng.integers(70, 90)), int(rng.integers(70, 90))): 0.1 * k - 0.05} if k % 5 == 2 else None
        outs = {}
        for mode, mem in memories.items():
            previous = kernels.set_mode(mode)
            try:
                outs[mode] = mem.apply(copy.deepcopy(snap), tuple(pose), [BODY, HULL][seed % 2], extra_support=extra)
            finally:
                kernels.set_mode(previous)
        assert differences(outs["reference"], outs["numpy"], limit=5) == []
        assert differences(memories["reference"], memories["numpy"], limit=5) == []
        pose += (0.02 * math.cos(pose[2]), 0.02 * math.sin(pose[2]), 0.0)
        if k % 7 == 6:
            correction = (float(rng.uniform(-0.03, 0.03)), float(rng.uniform(-0.03, 0.03)),
                          float(rng.uniform(-0.01, 0.01)))


def test_signed_zero_stamps_keep_the_references_choice():
    """Codex real-time round 3 P1: with +0.0 and -0.0 among the stamps a minimum may pick either; the kernels
    hand those calls to the reference, so the bits stay the reference's."""
    from forklift_core.control.shadow_memory import _retained_stamps_batched

    fresh = np.ones((3, 3), dtype=bool)
    fresh[1, 1] = False
    stamps = np.array([[0.0, 1.0, -0.0], [1.0, np.nan, 1.0], [1.0, 1.0, 1.0]])
    cand = {(1, 1): 0.025}
    reference = ShadowMemory._retained_stamps(cand, 0.05, fresh, stamps, set(), 3, 3)
    assert np.signbit(reference[(1, 1)]) != np.signbit(_retained_stamps_batched(cand, 0.05, fresh, stamps, set(),
                                                                                 3, 3)[(1, 1)])  # the hazard
    rng = np.random.default_rng(3)
    snap = grid(rng, free_share=1.0, occupied_share=0.0, retained_share=0.0)
    snap.free_stamp[snap.state == FREE] = rng.choice([0.0, -0.0, 0.1], int((snap.state == FREE).sum()))
    mem_out = []
    for mode in MODES:
        previous = kernels.set_mode(mode)
        try:
            mem = ShadowMemory(0.15, ERROR)
            mem_out.append(mem.apply(copy.deepcopy(snap), (0.0, 0.0, 0.0), Footprint(0.94, 0.17, 0.36)))
        finally:
            kernels.set_mode(previous)
    for b in mem_out[1:]:
        assert differences(mem_out[0], b, limit=5) == []

    def arc():
        return DrivePermission(config()).check_arc(snap, (0.0, 0.0, 0.0), 0.1, 1, HULL, BODY, 0.6)

    a, *rest = both(arc)
    for b in rest:
        assert differences(a, b, limit=5) == []
