"""G2 rows and the G7 link (docs/design/2026-11-14-imu-record-and-mount.md 3-4): the 32-bit device clock unwrapped,
the 2026-10-23 static capture's arrays turned into record rows, one stream's arrays, and its clock fit across a wrap."""

from __future__ import annotations

import numpy as np
import pytest

from forklift_core.sensors import imu_record as ir

WRAP = 1 << 32


def test_unwrap_adds_a_turn_per_wrap_and_leaves_a_small_backstep():
    assert ir.unwrap_u32_us([5, 10, 20]) == [5, 10, 20]
    one = ir.unwrap_u32_us([WRAP - 10, WRAP - 5, 3, 8])
    assert one == [WRAP - 10, WRAP - 5, WRAP + 3, WRAP + 8]
    q = WRAP >> 2  # quarter-turn steps: each unambiguous on its own
    two = ir.unwrap_u32_us([WRAP - 1, 0, q, 2 * q, 3 * q, WRAP - 1, 2])
    assert two == [WRAP - 1, WRAP, WRAP + q, WRAP + 2 * q, WRAP + 3 * q, 2 * WRAP - 1, 2 * WRAP + 2]
    # a few microseconds back is jitter, not a wrap: kept for clean_points to drop as device_back
    assert ir.unwrap_u32_us([100, 90, 110]) == [100, 90, 110]
    assert ir.unwrap_u32_us([]) == []


def test_a_late_sample_from_before_the_wrap_does_not_count_a_second_turn():
    assert ir.unwrap_u32_us([WRAP - 2, 1, WRAP - 1, 2]) == [WRAP - 2, WRAP + 1, WRAP - 1, WRAP + 2]


def test_a_restart_or_a_long_gap_splits_the_record():
    host = [0, 2_500_000, 5_000_000, 7_500_000, 10_007_500_000, 10_010_000_000]
    dev = [1000, 3500, 6000, 8500, 2000, 4500]  # the device restarted: its clock went back to near zero
    assert ir.continuous_segments(dev, host) == [(0, 4), (4, 6)]
    # a gap the device and the host agree on is not a break
    assert ir.continuous_segments([0, 2500, 9_002_500], [0, 2_500_000, 9_002_500_000]) == [(0, 3)]
    assert ir.continuous_segments([], []) == []


def test_a_restart_with_a_tiny_host_gap_gives_two_fits():
    """Codex re-review: raw 397500 -> 0 µs after a wrap with the host 2.5 ms on is a restart, not one fit."""
    rows, _ = synthetic_rows(seconds=20.0, dev0=WRAP - 6_000_000)
    gy = [r for r in rows if r.get("s") == "g"]
    k = 4000  # the wrap is at sample 2400; restart 1600 samples later, at raw 4_000_000 µs
    base = gy[k]["dev_us"]
    for r in gy[k:]:
        r["dev_us"] = (r["dev_us"] - base) % WRAP  # the device clock starts again from 0
    fits = ir.imu_clock_fits(rows, stream="g")
    assert len(fits) == 2 and all(f.valid for f in fits)
    assert all(abs(f.drift_ppm - 25.0) < 2.0 for f in fits)


def chunk(n, *, dev0, rate_hz, wall0=1.7e9, ts0=1.0e6, xyz=(0.0, -9.8, 0.1)):
    dev = (dev0 + np.round(np.arange(n) * 1e6 / rate_hz)).astype(np.int64) % WRAP
    t = np.arange(n) / rate_hz
    arr = np.empty((n, 6))
    arr[:, 0] = ts0 + t * 1e3
    arr[:, 1] = dev
    arr[:, 2] = wall0 + t + 0.002
    arr[:, 3:] = xyz
    return arr


def test_the_static_capture_becomes_rows_with_its_host_clock_marked_wall():
    g = chunk(4, dev0=1000, rate_hz=400, xyz=(0.01, 0.02, 0.03))
    a = chunk(2, dev0=1000, rate_hz=200)
    header, rows = ir.rows_from_static_chunk(g, a, source="d435i_imu_static_20261023/run1/chunk_0000.npz")
    assert header["schema"] == ir.SCHEMA and header["host_clock"] == "wall"
    assert header["source"].endswith("chunk_0000.npz")
    gy = [r for r in rows if r["s"] == "g"]
    ac = [r for r in rows if r["s"] == "a"]
    assert len(gy) == 4 and len(ac) == 2
    r0 = gy[0]
    assert r0["t"] == "imu" and r0["dev_us"] == 1000 and isinstance(r0["dev_us"], int)
    assert r0["host_ns"] == int(round((1.7e9 + 0.002) * 1e9)) and isinstance(r0["host_ns"], int)
    assert r0["ts_ms"] == pytest.approx(1.0e6) and r0["dom"] is None
    assert (r0["x"], r0["y"], r0["z"]) == pytest.approx((0.01, 0.02, 0.03))
    assert [r["host_ns"] for r in rows] == sorted(r["host_ns"] for r in rows)


def test_the_wall_clock_capture_is_refused_by_the_clock_fit_unless_asked():
    g = chunk(3000, dev0=WRAP - 2_000_000, rate_hz=400)
    header, rows = ir.rows_from_static_chunk(g, np.zeros((0, 6)), source="x")
    with pytest.raises(ValueError, match="monotonic"):
        ir.imu_clock_fits([header, *rows])
    fits = ir.imu_clock_fits([header, *rows], allow_wall=True)
    assert len(fits) == 1 and fits[0].valid and fits[0].n == 3000


def test_a_bad_array_shape_is_refused():
    with pytest.raises(ValueError, match="columns"):
        ir.rows_from_static_chunk(np.zeros((3, 5)), np.zeros((0, 6)), source="x")


def synthetic_rows(*, seconds=12.0, rate_hz=400, dev0=WRAP - 3_000_000, drift_ppm=25.0, seed=0, boot="b1"):
    rng = np.random.default_rng(seed)
    n = int(seconds * rate_hz)
    dev_true = dev0 + np.round(np.arange(n) * 1e6 / rate_hz).astype(np.int64)
    delay = 300_000 + rng.exponential(200_000, n).astype(np.int64)  # ns, never below 0.3 ms
    host = (5_000_000_000 + (dev_true - dev0) * 1000 * (1 + drift_ppm * 1e-6)).astype(np.int64) + delay
    rows = [{"t": "header", "schema": ir.SCHEMA, "host_clock": "monotonic", "host_boot_id": boot}]
    for d, h in zip(dev_true % WRAP, host, strict=True):
        rows.append({"t": "imu", "s": "g", "dev_us": int(d), "ts_ms": 0.0, "dom": "hardware", "host_ns": int(h),
                     "x": 0.0, "y": 0.0, "z": 0.0})
        rows.append({"t": "imu", "s": "a", "dev_us": int(d), "ts_ms": 0.0, "dom": "hardware", "host_ns": int(h),
                     "x": 0.0, "y": -9.8, "z": 0.0})
    return rows, dev_true


def test_imu_stream_unwraps_one_stream_and_names_the_host_boot():
    rows, dev_true = synthetic_rows(seconds=2.0)
    st = ir.imu_stream(rows, stream="a")
    assert st.host_boot == "b1" and st.host_clock == "monotonic" and st.domains == {"hardware"}
    assert list(st.dev_us) == list(dev_true)  # crossed the wrap without a jump
    assert st.xyz.shape == (len(dev_true), 3) and st.xyz[0, 1] == pytest.approx(-9.8)
    assert len(st.host_ns) == len(st.dev_us) and st.missing_dev == 0
    with pytest.raises(ValueError, match="stream"):
        ir.imu_stream(rows, stream="q")


def test_samples_without_a_device_time_are_left_out_and_counted():
    rows, dev_true = synthetic_rows(seconds=1.0)
    gy = [r for r in rows if r.get("s") == "g"]
    gy[3]["dev_us"] = None
    gy[5]["dev_us"] = WRAP + 7  # outside 32 bits: not a device time
    st = ir.imu_stream(rows, stream="g")
    assert st.missing_dev == 2 and len(st.dev_us) == len(dev_true) - 2


def test_the_clock_fit_crosses_the_wrap_and_finds_the_drift():
    rows, _ = synthetic_rows()
    fits = ir.imu_clock_fits(rows, stream="g")
    assert len(fits) == 1
    fit = fits[0]
    assert fit.valid and fit.host_boot == "b1"
    assert fit.drift_ppm == pytest.approx(25.0, abs=1.0)
    assert fit.dropped == {"device_back": 0, "host_back": 0, "device_repeat": 0}
    assert fit.delay_ns["p50"] < 1_000_000


def test_two_host_boots_in_one_file_are_refused():
    rows, _ = synthetic_rows(seconds=6.0)
    rows.insert(len(rows) // 2, {"t": "header", "schema": ir.SCHEMA, "host_clock": "monotonic", "host_boot_id": "b2"})
    with pytest.raises(ValueError, match="host boot"):
        ir.imu_stream(rows, stream="g")
