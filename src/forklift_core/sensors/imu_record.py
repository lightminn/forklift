"""D435i IMU record rows and their device clock (docs/design/2026-11-14-imu-record-and-mount.md 3-4, G2 and G7).

A record (``tools/imu_record.py``) is JSON lines: one header (``schema: imu_record/1``, ``host_boot_id``), ``imu`` rows
``{"t": "imu", "s": "g"|"a", "dev_us", "ts_ms", "dom", "host_ns", "x", "y", "z"}``, ``temp`` rows and a footer.
``dev_us`` is the frame's hardware timestamp metadata, a 32-bit µs counter that wraps every 4,295 s; ``host_ns`` is the
host monotonic time at the callback. ``imu_clock_fits`` maps one stream's unwrapped device time to the host clock with
``clock_map.fit_clock`` per continuous segment -- the same fit as the MCU's (G7), with the same limits.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from forklift_core.plant.clock_map import ClockFit, fit_clock

SCHEMA = "imu_record/1"
WRAP_US = 1 << 32
STREAMS = ("g", "a")
MAX_MISMATCH_S = 0.5  # device and host intervals differing more than this break continuity


def unwrap_u32_us(values) -> list[int]:
    """Unwrap a 32-bit µs counter: each value becomes the ``v + k * 2**32`` nearest to the previous unwrapped value.
    A late sample from before a wrap therefore lands before it (no second turn), and a small step back stays a step
    back (``clean_points`` drops it as ``device_back``). A device restart looks like a wrap -- ``continuous_segments``
    splits on the host clock."""
    out: list[int] = []
    prev = None
    for v in values:
        v = int(v)
        if prev is not None:
            k = round((prev - v) / WRAP_US)
            v += k * WRAP_US
        out.append(v)
        prev = v
    return out


def continuous_segments(dev_us, host_ns, *, max_mismatch_s: float = MAX_MISMATCH_S) -> list[tuple[int, int]]:
    """``[start, end)`` index ranges over which the unwrapped device clock and the host clock advance together: a
    step whose device and host intervals differ by more than ``max_mismatch_s`` (a restart, a gap longer than the
    unwrap can resolve) starts a new range."""
    n = len(dev_us)
    if n == 0:
        return []
    cuts = [0]
    for i in range(1, n):
        dd = (int(dev_us[i]) - int(dev_us[i - 1])) * 1e3
        dh = int(host_ns[i]) - int(host_ns[i - 1])
        if abs(dh - dd) > max_mismatch_s * 1e9:
            cuts.append(i)
    cuts.append(n)
    return [(a, b) for a, b in zip(cuts[:-1], cuts[1:], strict=True)]


def rows_from_static_chunk(gyro, accel, *, source: str) -> tuple[dict, list[dict]]:
    """The 2026-10-23 desk capture (``chunk_*.npz``: rows of ``get_timestamp`` ms, ``frame_timestamp`` µs,
    ``time.time()`` s, x, y, z) as record rows. Its host time is the wall clock, not the monotonic one, and its time
    domain was not recorded -- the header says so. Rows are in host-time order."""
    rows: list[dict] = []
    for stream, arr in (("g", gyro), ("a", accel)):
        arr = np.asarray(arr, float)
        if arr.ndim != 2 or arr.shape[1] != 6:
            raise ValueError(f"{stream}: expected N x 6 columns (ts_ms, dev_us, wall_s, x, y, z), got {arr.shape}")
        for ts_ms, dev_us, wall_s, x, y, z in arr:
            rows.append({"t": "imu", "s": stream, "dev_us": int(dev_us), "ts_ms": float(ts_ms), "dom": None,
                         "host_ns": int(round(wall_s * 1e9)), "x": float(x), "y": float(y), "z": float(z)})
    rows.sort(key=lambda r: r["host_ns"])
    header = {"t": "header", "schema": SCHEMA, "source": source, "host_clock": "wall", "host_boot_id": None,
              "note": "converted from the 2026-10-23 desk capture: host time is time.time(), time domain unrecorded"}
    return header, rows


@dataclass(frozen=True)
class ImuStream:
    dev_us: np.ndarray  # unwrapped device µs of the samples that have one
    host_ns: np.ndarray
    xyz: np.ndarray  # N x 3
    host_boot: str | None
    host_clock: str | None  # "monotonic" for this recorder; "wall" for the converted 2026-10-23 capture
    domains: set
    missing_dev: int  # samples with no (or an out-of-range) device time, left out


def imu_stream(rows, *, stream: str = "g") -> ImuStream:
    """One stream of a record, in record order. A file whose headers name two host boots (or two host clocks) is
    refused: its host times are not one clock."""
    if stream not in STREAMS:
        raise ValueError(f"stream must be one of {STREAMS}, got {stream!r}")
    boots: set = set()
    clocks: set = set()
    dev, host, xyz = [], [], []
    domains: set = set()
    missing = 0
    for r in rows:
        t = r.get("t")
        if t == "header":
            boots.add(r.get("host_boot_id"))
            clocks.add(r.get("host_clock"))
        elif t == "imu" and r.get("s") == stream:
            d = r.get("dev_us")
            if d is None or not (0 <= int(d) < WRAP_US):
                missing += 1
                continue
            domains.add(r.get("dom"))
            dev.append(d)
            host.append(r["host_ns"])
            xyz.append((r["x"], r["y"], r["z"]))
    if len(boots) > 1:
        raise ValueError(f"more than one host boot in one record: {sorted(map(str, boots))}")
    if len(clocks) > 1:
        raise ValueError(f"more than one host clock in one record: {sorted(map(str, clocks))}")
    return ImuStream(np.asarray(unwrap_u32_us(dev), dtype=np.int64), np.asarray(host, dtype=np.int64),
                     np.asarray(xyz, float).reshape(-1, 3), next(iter(boots), None), next(iter(clocks), None),
                     domains, missing)


def imu_clock_fits(rows, *, stream: str = "g", allow_wall: bool = False) -> list[ClockFit]:
    """One G7 fit per continuous segment of a stream. A record whose host clock is not monotonic is refused;
    ``allow_wall`` is for the format regression on the 2026-10-23 capture only -- its drift is against the wall clock
    and is no evidence of monotonic alignment."""
    st = imu_stream(rows, stream=stream)
    if st.host_clock != "monotonic" and not allow_wall:
        raise ValueError(f"host clock is {st.host_clock!r}, not monotonic: not a clock to align to")
    return [fit_clock(st.dev_us[a:b].tolist(), st.host_ns[a:b].tolist(), host_boot=st.host_boot)
            for a, b in continuous_segments(st.dev_us, st.host_ns)]
