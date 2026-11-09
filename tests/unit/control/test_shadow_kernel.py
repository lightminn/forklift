"""Real-time plan 5.27 G3-1c (docs/plans/2026-10-17-realtime-sim.md): the shadow apply on arrays
(FORKLIFT_SHADOW_KERNEL=1) gives the original's results bit for bit -- the snapshot, and the memory with its key
order, key and value types and float bits, call after call -- in both fast kernel modes, through start-up, memory
moves, grid-origin shifts, depth support, signed zeros and non-float times (those take the original path)."""

import copy
import dataclasses
import importlib.util
import math
import sys

import numpy as np
import pytest

from forklift_core import kernels
from forklift_core.control import shadow_memory as sm
from forklift_core.control.drive_permission import footprint_cells
from forklift_core.control.shadow_memory import ShadowMemory, band_cells, whole_cells
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
from forklift_core.perf import differences
from forklift_core.planning.geometry import Footprint

HULL = Footprint(1.29, 0.17, 0.36)
BODY = Footprint(0.944, 0.17, 0.36)
LOADED = Footprint(1.56, 0.17, 0.40)
ERROR = AgeErrorTable((0.1, 0.2, 0.5, 1.0, 10.0), (0.005, 0.008, 0.01, 0.012, 0.02),
                      (0.001, 0.0015, 0.002, 0.003, 0.004))
FAST = ("numpy",) + (("numba",) if importlib.util.find_spec("numba") is not None else ())
RES = 0.05


def snapshot(rng, stamp, correction, origin, *, free_share=0.85, n=160):
    u = rng.random((n, n))
    state = np.full((n, n), UNKNOWN, dtype=np.uint8)
    state[u < free_share] = FREE
    state[u < 0.005] = OCCUPIED
    free_stamp = np.where(state == FREE, stamp - rng.uniform(0.0, 0.15, (n, n)), np.nan)
    return GridSnapshot(state, free_stamp, stamp, correction, 0, 1, {"high": stamp}, origin[0], origin[1], RES)


def same(a, b):
    """Results and receivers alike, the memory's key list (order and element types) included -- differences() does
    not look at a dict key's type (plan 5.27, Codex P2-2)."""
    return (differences(a[1], b[1], limit=5) + differences(a[0], b[0], limit=5)
            + differences(list(a[0].memory), list(b[0].memory), limit=5))


def call(mem, mode, on, snap, pose, foot, extra, monkeypatch):
    monkeypatch.setattr(sm, "SHADOW_KERNEL", on)
    previous = kernels.set_mode(mode)
    try:
        return mem.apply(copy.deepcopy(snap), pose, foot, extra_support=copy.deepcopy(extra))
    finally:
        kernels.set_mode(previous)


def drive(seed, mode, monkeypatch, steps=30, stamp_type=float):
    """One random session replayed by the original (switch off) and the array path (switch on), compared at every
    call; returns how often the array path and its memory move ran."""
    rng = np.random.default_rng(seed)
    off, on = ShadowMemory(0.15, ERROR), ShadowMemory(0.15, ERROR)
    used = {"apply": 0, "move": 0}
    for name, key in (("_apply_arrays", "apply"), ("_move_arrays", "move")):
        real = getattr(ShadowMemory, name)

        def counted(self, *a, _real=real, _key=key, **k):
            used[_key] += 1
            return _real(self, *a, **k)
        monkeypatch.setattr(ShadowMemory, name, counted)
    x, y, yaw = 0.0, 0.0, float(rng.uniform(-0.4, 0.4))
    correction = (0.0, 0.0, 0.0)
    origin = (-4.0, -4.0)
    for k in range(steps):
        snap = snapshot(rng, stamp_type(0.1 * (k + 1)), correction, origin,
                        free_share=float(rng.uniform(0.55, 0.95)))
        foot = (BODY, HULL, LOADED)[(k // 9 + seed) % 3]
        extra = None
        if k % 6 == 3:
            i, j = int(rng.integers(70, 100)), int(rng.integers(70, 100))
            extra = {(i, j): 0.1 * k - 0.07, (i + 1, j): 0.1 * k + 0.01, (-1, j): 0.0}
        pose = (x, y, yaw)
        a = (off, call(off, mode, False, snap, pose, foot, extra, monkeypatch))
        b = (on, call(on, mode, True, snap, pose, foot, extra, monkeypatch))
        assert same(a, b) == [], (seed, mode, k)
        x += 0.03 * math.cos(yaw)
        y += 0.03 * math.sin(yaw)
        yaw += float(rng.uniform(-0.05, 0.05))
        if k % 5 == 4:  # a SLAM correction change: the memory moves (not by whole cells)
            correction = (float(rng.uniform(-0.04, 0.04)), float(rng.uniform(-0.04, 0.04)),
                          float(rng.uniform(-0.02, 0.02)))
        if k % 8 == 7:  # the local window follows the truck: the origin shifts by whole cells
            origin = (origin[0] + RES * int(rng.integers(-6, 7)), origin[1] + RES * int(rng.integers(-6, 7)))
    return used


@pytest.mark.parametrize("mode", FAST)
@pytest.mark.parametrize("seed", range(10))
def test_the_array_path_carries_the_same_state_call_after_call(seed, mode, monkeypatch):
    used = drive(seed, mode, monkeypatch)
    assert used["apply"] == 29  # every call after start-up
    if mode == "numba":
        assert used["move"] >= 4


@pytest.mark.parametrize("stamp_type", [int, np.float64, np.float32])
def test_non_float_times_take_the_original_path_alike(stamp_type, monkeypatch):
    """A time that is not a built-in float changes NumPy's promotion (Codex plan 5.27 round 2: a float32 age crossed
    a table row); such calls run the original, with its value types."""
    used = drive(3, FAST[-1], monkeypatch, steps=12, stamp_type=stamp_type)
    assert used["apply"] == 0


def test_the_band_set_iterates_like_the_set_comprehension_also_after_removing_the_whole_cells():
    rng = np.random.default_rng(7)
    snap = snapshot(rng, 1.0, (0.0, 0.0, 0.0), (-4.0, -4.0), n=120)
    for _ in range(200):
        pose = (float(rng.uniform(-4.3, 2.3)), float(rng.uniform(-4.3, 2.3)), float(rng.uniform(-math.pi, math.pi)))
        foot = (BODY, HULL, LOADED)[int(rng.integers(3))]
        cells, _ = footprint_cells(snap, pose, foot, 0.15)
        old = band_cells(snap, pose, foot, 0.15)
        new = sm._cell_set(cells, *snap.state.shape)
        assert list(old) == list(new)
        whole = whole_cells(snap, pose, foot)
        old -= whole
        new -= whole
        assert list(old) == list(new)


@pytest.mark.parametrize("mode", FAST)
def test_memory_cells_off_the_grid_are_not_read_through_negative_indices(mode, monkeypatch):
    rng = np.random.default_rng(11)
    off, on = ShadowMemory(0.15, ERROR), ShadowMemory(0.15, ERROR)
    snap = snapshot(rng, 0.1, (0.0, 0.0, 0.0), (-4.0, -4.0))
    for mem in (off, on):
        call(mem, mode, False, snap, (0.0, 0.0, 0.0), BODY, None, monkeypatch)
        for key in ((-200, -200), (-81, 5), (5, -81), (500, 500)):  # cells the shifted window puts at negative indices
            mem.memory[key] = (0.05, "free", 1.0)
    shifted = snapshot(rng, 0.2, (0.0, 0.0, 0.0), (-3.0, -3.5))
    a = (off, call(off, mode, False, shifted, (0.01, 0.0, 0.0), BODY, None, monkeypatch))
    b = (on, call(on, mode, True, shifted, (0.01, 0.0, 0.0), BODY, None, monkeypatch))
    assert same(a, b) == []


@pytest.mark.parametrize("mode", FAST)
def test_equal_stamps_and_signed_zeros_match(mode, monkeypatch):
    """Many fresh neighbours with one time (the minimum's choice), and +0.0/-0.0 among them (the original path)."""
    for fill in ([0.125], [0.0, -0.0, 0.125]):
        rng = np.random.default_rng(5)
        off, on = ShadowMemory(0.15, ERROR), ShadowMemory(0.15, ERROR)
        for k in range(6):
            snap = snapshot(rng, 0.1 * (k + 1), (0.0, 0.0, 0.0), (-4.0, -4.0), free_share=0.7)
            free = snap.state == FREE
            snap.free_stamp[free] = rng.choice(fill, int(free.sum()))
            a = (off, call(off, mode, False, snap, (0.0, 0.01 * k, 0.0), BODY, None, monkeypatch))
            b = (on, call(on, mode, True, snap, (0.0, 0.01 * k, 0.0), BODY, None, monkeypatch))
            assert same(a, b) == []


@pytest.mark.parametrize("mode", FAST)
def test_a_zero_time_in_the_memory_moves_by_the_original(mode, monkeypatch):
    rng = np.random.default_rng(9)
    off, on = ShadowMemory(0.15, ERROR), ShadowMemory(0.15, ERROR)
    snap = snapshot(rng, 0.0, (0.0, 0.0, 0.0), (-4.0, -4.0))  # start-up at time 0.0: zero times in the memory
    for mem in (off, on):
        call(mem, mode, False, snap, (0.0, 0.0, 0.0), BODY, None, monkeypatch)
    moved = snapshot(rng, 0.1, (0.012, -0.007, 0.004), (-4.0, -4.0))
    a = (off, call(off, mode, False, moved, (0.0, 0.0, 0.0), BODY, None, monkeypatch))
    b = (on, call(on, mode, True, moved, (0.0, 0.0, 0.0), BODY, None, monkeypatch))
    assert same(a, b) == [] and on.stats["moves"] == 1


def test_without_numba_the_numpy_path_never_imports_the_kernels(monkeypatch):
    monkeypatch.setitem(sys.modules, "forklift_core.control.shadow_numba", None)  # an import would fail
    used = drive(4, "numpy", monkeypatch, steps=12)
    assert used["apply"] == 11 and used["move"] == 0


@pytest.mark.skipif(importlib.util.find_spec("numba") is None, reason="numba not installed")
def test_warm_up_compiles_the_signatures_a_session_uses(monkeypatch):
    """Plan D3 / 5.27 (Codex P2-3): after kernels.warm_up() the first RETAINED call and the first move compile
    nothing new."""
    from forklift_core.control import shadow_numba

    previous = kernels.set_mode("numba")
    try:
        kernels.warm_up()
        before = (len(shadow_numba.retained.signatures), len(shadow_numba.move_values.signatures))
    finally:
        kernels.set_mode(previous)
    used = drive(1, "numba", monkeypatch, steps=12)
    assert used["move"] >= 1
    assert (len(shadow_numba.retained.signatures), len(shadow_numba.move_values.signatures)) == before


@pytest.mark.parametrize("mode", FAST)
@pytest.mark.parametrize("dtype", [np.int64, np.longdouble])
def test_a_free_time_is_stored_as_the_original_float(mode, dtype, monkeypatch):
    """Codex 1c-0 review P2: the original stores float(free_stamp[a, b]); an integer or long-double stamp array
    must not leave its own scalar type in the memory."""
    rng = np.random.default_rng(13)
    off, on = ShadowMemory(0.15, ERROR), ShadowMemory(0.15, ERROR)
    used = {"apply": 0}
    real = ShadowMemory._apply_arrays

    def counted(self, *a, **k):
        used["apply"] += 1
        return real(self, *a, **k)
    monkeypatch.setattr(ShadowMemory, "_apply_arrays", counted)
    for k in range(4):
        snap = snapshot(rng, float(k + 1), (0.0, 0.0, 0.0), (-4.0, -4.0))
        snap = dataclasses.replace(snap, free_stamp=np.where(snap.state == FREE, k + 1, 0).astype(dtype))
        a = (off, call(off, mode, False, snap, (0.0, 0.01 * k, 0.0), BODY, None, monkeypatch))
        b = (on, call(on, mode, True, snap, (0.0, 0.01 * k, 0.0), BODY, None, monkeypatch))
        assert same(a, b) == []
    assert all(type(v[0]) is float for v in on.memory.values())
    assert used == {"apply": 0}  # a stamp array that is not float64 keeps the original path


@pytest.mark.parametrize("mode", FAST)
def test_a_non_finite_radius_behaves_like_the_original(mode, monkeypatch):
    """Codex 1c-0 review P3: with a NaN among the candidates' radii the batched code's max() decides; the array path
    hands such a call to it rather than to the kernel."""
    rng = np.random.default_rng(17)
    off, on = ShadowMemory(0.15, ERROR), ShadowMemory(0.15, ERROR)
    snap = snapshot(rng, 0.1, (0.0, 0.0, 0.0), (-4.0, -4.0))
    for mem in (off, on):
        call(mem, mode, False, snap, (0.0, 0.0, 0.0), BODY, None, monkeypatch)
        keys = list(mem.memory)
        mem.memory[keys[len(keys) // 2]] = (0.05, "free", float("nan"))
    later = snapshot(rng, 0.2, (0.0, 0.0, 0.0), (-4.0, -4.0), free_share=0.6)
    results = []
    for mem, on_ in ((off, False), (on, True)):
        try:
            results.append((mem, call(mem, mode, on_, later, (0.0, 0.0, 0.0), BODY, None, monkeypatch)))
        except ValueError as e:
            results.append(("raised", str(e)))
    if results[0][0] == "raised":
        assert results[1] == results[0]
    else:
        assert same(results[0], results[1]) == []
