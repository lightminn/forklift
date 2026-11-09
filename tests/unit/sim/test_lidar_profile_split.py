"""Real-time plan 5.20 G3-1a: the LiDAR's sampled time split casts every ray through the same steps as the plain
loop -- same distances, hits and own flags -- and books its parts into the live timers; off it never runs."""

import sys
import types
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))


class FakeQuery:
    def __init__(self):
        self.calls = []

    def raycast_closest(self, start, direction, range_m, both_sides):
        self.calls.append((tuple(direction), range_m, both_sides))
        i = len(self.calls) - 1
        if i % 3 == 2:
            return {"hit": False}
        body = "/World/Forklift/body" if i % 3 == 1 else "/World/Shelf"
        return {"hit": True, "distance": 0.5 + i, "rigidBody": body, "collision": "/c"}


@pytest.fixture
def fake_isaac(monkeypatch):
    carb = types.ModuleType("carb")
    carb.Float3 = lambda *v: tuple(v)
    omni = types.ModuleType("omni")
    physx = types.ModuleType("omni.physx")
    queries = []

    def iface():
        q = FakeQuery()
        queries.append(q)
        return q

    physx.get_physx_scene_query_interface = iface
    omni.physx = physx
    monkeypatch.setitem(sys.modules, "carb", carb)
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.physx", physx)
    return queries


def test_the_profiled_scan_equals_the_plain_loop_and_books_its_parts(fake_isaac, monkeypatch):
    import planar_lidar as pl
    from forklift_core.perf import TIMERS

    dirs = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    monkeypatch.setattr(pl, "_PROFILE_EVERY", 0)
    plain = pl.cast_scan_flags(np.zeros(3), dirs, 12.0)
    monkeypatch.setattr(pl, "_PROFILE_EVERY", 1)
    monkeypatch.setattr(TIMERS, "enabled", True)
    TIMERS.take()
    profiled = pl.cast_scan_flags(np.zeros(3), dirs, 12.0)
    for a, b in zip(plain, profiled):
        np.testing.assert_array_equal(a, b)
    assert fake_isaac[0].calls == fake_isaac[1].calls  # the same queries, in the same order
    booked = TIMERS.take()
    assert {"lidar.split.scans", "lidar.split.convert", "lidar.split.query", "lidar.split.hit_and_own"} <= set(booked)
    assert booked["lidar.split.rays"][0] == len(dirs)  # the ray count, in the seconds field


def test_only_every_nth_scan_is_profiled_and_only_with_the_live_timers(monkeypatch):
    import planar_lidar as pl
    from forklift_core.perf import TIMERS

    monkeypatch.setattr(pl, "_PROFILE_EVERY", 3)
    monkeypatch.setattr(pl, "_profile_count", [0])
    monkeypatch.setattr(TIMERS, "enabled", False)
    assert not any(pl._profile_due() for _ in range(6))  # a batch run: nothing would book it
    monkeypatch.setattr(TIMERS, "enabled", True)
    assert [pl._profile_due() for _ in range(6)] == [False, False, True, False, False, True]
    monkeypatch.setattr(pl, "_PROFILE_EVERY", 0)
    assert not any(pl._profile_due() for _ in range(5))
