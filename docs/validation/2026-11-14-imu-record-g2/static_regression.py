"""Format regression of the G2/G7 path on the 2026-10-23 desk capture (design 2026-11-14-imu-record-and-mount.md 4).

    python static_regression.py <run1 folder with chunk_*.npz> <out.json>

Every chunk goes through ``rows_from_static_chunk``; each stream through ``imu_stream`` and
``imu_clock_fits(allow_wall=True)``. The host clock of that capture is ``time.time()``: the drift and residuals below
are against the wall clock -- a check of the columns, order, unwrap and sample spacing, not of monotonic alignment.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from forklift_core.sensors.imu_record import (
    WRAP_US,
    continuous_segments,
    imu_clock_fits,
    imu_stream,
    rows_from_static_chunk,
)


def main(run: Path, out: Path) -> None:
    chunks = sorted(run.glob("chunk_*.npz"))
    rows: list[dict] = []
    header = None
    for c in chunks:
        z = np.load(c)
        h, r = rows_from_static_chunk(z["gyro"], z["accel"], source=f"{run.name}/{c.name}")
        header = header or {**h, "source": f"{run.name}/chunk_*.npz ({len(chunks)} chunks)"}
        rows.extend(r)
    rows.insert(0, header)
    result = {"chunks": len(chunks), "rows": len(rows) - 1, "streams": {}}
    for s in ("g", "a"):
        st = imu_stream(rows, stream=s)
        raw = [r["dev_us"] for r in rows if r.get("t") == "imu" and r["s"] == s]
        steps = np.diff(st.dev_us)
        segs = continuous_segments(st.dev_us, st.host_ns)
        fits = imu_clock_fits(rows, stream=s, allow_wall=True)
        result["streams"][s] = {
            "n": int(len(st.dev_us)), "missing_dev": st.missing_dev,
            "wraps": int((st.dev_us[-1] - raw[-1]) // WRAP_US) if raw else 0,
            "span_s": float((st.dev_us[-1] - st.dev_us[0]) / 1e6),
            "rate_hz": float((len(st.dev_us) - 1) / ((st.dev_us[-1] - st.dev_us[0]) / 1e6)),
            "step_us": {"median": float(np.median(steps)), "min": int(steps.min()), "max": int(steps.max()),
                        "back": int((steps < 0).sum()), "zero": int((steps == 0).sum())},
            "segments": [list(x) for x in segs],
            "fits": [f.summary() for f in fits],
        }
    out.write_text(json.dumps(result, indent=1, default=str))
    print(json.dumps({s: {k: v for k, v in d.items() if k != "fits"} | {"drift_ppm": [f["drift_ppm"] for f in d["fits"]]}
                      for s, d in result["streams"].items()}, indent=1, default=str))


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
