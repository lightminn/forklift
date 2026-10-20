"""Replay a run's recorded SLAM scans into a fresh mapping bridge, lockstep (plans 2026-10-20 L2 r6 diagnosis,
2026-10-21 gyro-heading V-G0).

Every scan the runner sent, in order: the recorded clean ranges plus the shared-scan noise drawn as the runner draws it
(obstacle layer rng ``default_rng([seed, 7])``, one truncated N(0, 0.02) draw per beam clipped to +-0.06, added to the
finite beams and clipped to the range limits) -- a redraw, not the live stream; ``--ranges-bin`` (tools range dump of a
saved graph) puts given ranges on the scans at its stamps instead (the live graph's own keyframes reproduce the live
graph). The odometry is the record's ``odom_base`` (spool ``slam.records``), or ``--odom-file`` (tools/
gyro_heading_offline.py), checked against the record: same run, spool manifest hash, scan ids and stamps. The bridge's
own KeyframeGate picks keyframes, as live. Prints ``sent N scans`` -- N is checked against the record by the summary.

    python tools/slam_remap_drive.py RUN_DIR SOCKET OUT_JSONL [--noise-seed S] [--odom-file F] [--ranges-bin B]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.live.spool import load_list, load_slam_log  # noqa: E402
from forklift_core.localization import slam_link  # noqa: E402

BEAMS, RMIN, RMAX = 1600, 0.2, 12.0
ANG0, DANG = -np.pi, 2 * np.pi / 1600
STD, CUT = 0.02, 0.06


def read_ranges_bin(path: Path) -> dict:
    """{stamp rounded to 1 us: ranges} from a range dump: per scan double stamp, odom x, y, yaw, int32 n, n doubles."""
    data, i, out = path.read_bytes(), 0, {}
    while i < len(data):
        head = np.frombuffer(data, np.float64, 4, i)
        i += 32
        n = int(np.frombuffer(data, np.int32, 1, i)[0])
        i += 4
        out[round(float(head[0]), 6)] = np.frombuffer(data, np.float64, n, i).copy()
        i += 8 * n
    return out


def load_odom_file(path: Path, run: Path, records: list) -> np.ndarray:
    z = np.load(path)
    if str(z["run"]) != str(run.resolve()):
        raise SystemExit(f"odometry file is for {z['run']}, not {run.resolve()}")
    if str(z["spool_manifest_sha256"]) != hashlib.sha256((run / "spool" / "manifest.json").read_bytes()).hexdigest():
        raise SystemExit("odometry file's spool manifest hash differs from the run's")
    if not (np.array_equal(z["scan_id"], [r["scan_id"] for r in records])
            and np.array_equal(z["stamp_s"], [r["stamp_s"] for r in records])):
        raise SystemExit("odometry file's scan ids or stamps differ from the record")
    return z["odom_base"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("socket")
    ap.add_argument("out", type=Path)
    ap.add_argument("--noise-seed", type=int, default=0)
    ap.add_argument("--odom-file", type=Path, default=None)
    ap.add_argument("--ranges-bin", type=Path, default=None)
    a = ap.parse_args(argv)
    records = load_list(a.run / "spool", "slam.records")
    log = load_slam_log(a.run)
    stamps = np.asarray(log["scan_stamps_s"], float)
    ranges = log["scan_ranges_m"]
    if len(np.asarray(log.get("silenced_scan_stamps_s", []))):
        raise SystemExit("silenced scans: the noise stream would need them skipped")
    if len(records) != len(stamps) or [r["scan_id"] for r in records] != list(range(len(records))):
        raise SystemExit("spool records are not scans 0..n-1 of the slam log")
    if not np.allclose([r["stamp_s"] for r in records], stamps, atol=1e-6):
        raise SystemExit("spool stamps differ from the slam log")
    odom = load_odom_file(a.odom_file, a.run, records) if a.odom_file else None
    given = read_ranges_bin(a.ranges_bin) if a.ranges_bin else {}
    rng = np.random.default_rng([a.noise_seed, 7])
    for _ in range(600):
        if Path(a.socket).exists():
            break
        time.sleep(1)
    client = slam_link.SlamLinkClient(a.socket, timeout_s=120.0)
    started, sent = time.monotonic(), 0
    with open(a.out, "w") as out:
        for r, stamp, clean in zip(records, stamps, ranges):
            scan_ranges = np.asarray(clean, float).copy()
            noise = rng.normal(0, STD, len(scan_ranges)).clip(-CUT, CUT)
            finite = np.isfinite(scan_ranges)
            scan_ranges[finite] = np.clip(scan_ranges[finite] + noise[finite], RMIN, RMAX)
            if given:
                scan_ranges = given.pop(round(float(stamp), 6), scan_ranges)
            pose = tuple(odom[r["scan_id"]]) if odom is not None else tuple(r["odom_base"])
            reply = client.exchange(slam_link.Scan(r["scan_id"], float(stamp), pose, scan_ranges.astype(np.float32),
                                                   ANG0, DANG, RMIN, RMAX))
            sent += 1
            if reply.status == "processed":
                out.write(json.dumps({"scan_id": r["scan_id"], "stamp_s": float(stamp), "slam_wall_s": reply.slam_wall_s,
                                      "map_from_odom": list(reply.map_from_odom)}) + "\n")
    client.close()
    if given:
        raise SystemExit(f"{len(given)} given scans were not sent")
    print(f"sent {sent} scans in {time.monotonic() - started:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
