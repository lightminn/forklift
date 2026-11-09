"""Real-time plan G3-1d candidate: cast_scan_flags_fast casts the same rays in the same order as cast_scan_flags and
returns the same distances, hits and own flags -- whatever the hit bodies, the own prefixes (the carried pallet's
come and go) and a cache shared across scans."""

import sys
import types
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

BODIES = [("/World/Shelf", "/World/Shelf/c"), ("/World/Forklift/body", "/c"), ("/World/P_1/deck", "/World/P_1/c"),
          ("/World/Env", "/World/Forklift/fork_l/col"), (None, None)]


class FakeQuery:
    def __init__(self):
        self.calls = []

    def raycast_closest(self, start, direction, range_m, both_sides):
        self.calls.append((tuple(start), tuple(direction), range_m, both_sides))
        i = len(self.calls) - 1
        rb, col = BODIES[i % len(BODIES)]
        if rb is None:
            return {"hit": False}
        return {"hit": True, "distance": 0.25 + 0.5 * i, "rigidBody": rb, "collision": col}


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


def test_fast_equals_the_plain_cast_for_any_prefixes_and_a_shared_cache(fake_isaac, monkeypatch):
    import planar_lidar as pl

    monkeypatch.setattr(pl, "_PROFILE_EVERY", 0)
    rng = np.random.default_rng(0)
    dirs = rng.normal(size=(37, 3))
    origin = np.array([1.0, -2.0, 0.3])
    cache = {}
    for prefixes in [(pl.SELF_PREFIX,), (pl.SELF_PREFIX, "/World/P_1"), (pl.SELF_PREFIX,)]:
        plain = pl.cast_scan_flags(origin, dirs, 12.0, own_prefixes=prefixes)
        fast = pl.cast_scan_flags_fast(origin, dirs, 12.0, own_prefixes=prefixes, own_cache=cache)
        for a, b in zip(plain, fast):
            np.testing.assert_array_equal(a, b)
        assert fake_isaac[-2].calls == fake_isaac[-1].calls  # the same queries, in the same order
    # the carried pallet's prefix changed the answer for its bodies and the cache kept the two apart
    assert len({k[0] for k in cache}) == 2


def test_fast_takes_a_given_query(fake_isaac):
    import planar_lidar as pl

    q = FakeQuery()
    pl.cast_scan_flags_fast(np.zeros(3), np.eye(3), 5.0, query=q)
    assert len(q.calls) == 3 and fake_isaac == []


def test_cast_modes_fast_and_oracle_return_the_loops_result_and_count(fake_isaac, monkeypatch):
    import planar_lidar as pl

    monkeypatch.setattr(pl, "_PROFILE_EVERY", 0)
    monkeypatch.setattr(pl, "_OWN_CACHE", {})
    monkeypatch.setattr(pl, "ORACLE", {"scans": 0, "mismatched_scans": 0, "mismatched_rays": 0, "own_true_rays": 0,
                                       "prefix_sets": [], "first_mismatch": None})
    dirs = np.random.default_rng(1).normal(size=(25, 3))
    monkeypatch.setattr(pl, "CAST_MODE", "loop")
    plain = pl.cast_scan_flags(np.zeros(3), dirs, 9.0, own_prefixes=(pl.SELF_PREFIX, "/World/P_1"))
    for mode in ("fast", "oracle"):
        monkeypatch.setattr(pl, "CAST_MODE", mode)
        got = pl.cast_scan_flags(np.zeros(3), dirs, 9.0, own_prefixes=(pl.SELF_PREFIX, "/World/P_1"))
        for a, b in zip(plain, got):
            np.testing.assert_array_equal(a, b)
    rec = pl.oracle_record()
    assert (rec["scans"], rec["mismatched_scans"], rec["mode"]) == (1, 0, "oracle")
    assert rec["own_true_rays"] == int(plain[2].sum()) > 0  # own hits were in the compared scans
    assert rec["prefix_sets"] == [[pl.SELF_PREFIX, "/World/P_1"]] and rec["own_cache_entries"] > 0
    assert pl.CAST_MODE == "oracle"  # the oracle's reference cast leaves the mode as it was


def test_the_oracle_counts_a_mismatch(fake_isaac, monkeypatch):
    import planar_lidar as pl

    monkeypatch.setattr(pl, "_PROFILE_EVERY", 0)
    monkeypatch.setattr(pl, "ORACLE", {"scans": 0, "mismatched_scans": 0, "mismatched_rays": 0, "own_true_rays": 0,
                                       "prefix_sets": [], "first_mismatch": None})
    monkeypatch.setattr(pl, "CAST_MODE", "oracle")
    real_fast = pl.cast_scan_flags_fast

    def wrong(*a, **k):
        d, h, o = real_fast(*a, **k)
        d = d.copy()
        d[0] = 99.0
        return d, h, o
    monkeypatch.setattr(pl, "cast_scan_flags_fast", wrong)
    pl.cast_scan_flags(np.zeros(3), np.eye(3), 5.0)
    rec = pl.oracle_record()
    assert rec["mismatched_scans"] == 1 and rec["first_mismatch"]["ray"] == 0
