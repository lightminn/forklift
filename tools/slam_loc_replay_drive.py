"""Replay a live localization session's keyframes into a fresh localization bridge, lockstep (SLAM localization plan,
the V5 diagnosis: docs/plans/2026-10-20-slam-localization-session.md).

The scans the live bridge sent to slam_toolbox (its ``records.jsonl``: every scan whose ``sent`` is not ``local`` -- the
seven start-up scans and the keyframes), in order, up to ``--until-s``; the bridge runs with ``keyframe_gate every``, so
which scans slam_toolbox matches does not change with the odometry an arm gives it. Ranges: the record's clean ranges
plus the shared-scan noise drawn as the runner draws it for every scan, sent or not (``default_rng([seed, 7])``, one
truncated N(0, 0.02) draw per beam clipped to +-0.06, added to the finite beams and clipped to the range limits) -- a
redraw, the same for every arm, not the live stream. Odometry: ``recorded`` (the record's ``odom_base``) or ``truth``
(its ``truth_base``). Each reply goes to OUT_JSONL with the odometry that arm sent.

    python tools/slam_loc_replay_drive.py RUN_DIR BRIDGE_RECORDS SOCKET OUT_JSONL --noise-seed S --odom {recorded,truth}
        [--until-s T]
"""

from __future__ import annotations

import argparse
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


def sent_ids(bridge_records: Path) -> list[int]:
    """The scan ids the live bridge sent to slam_toolbox, in order (start-up scans and keyframes)."""
    rows = [json.loads(line) for line in bridge_records.read_text().splitlines() if line]
    ids = [r["scan_id"] for r in rows]
    if ids != list(range(len(ids))):
        raise SystemExit("the bridge records are not scans 0..n-1")
    return [r["scan_id"] for r in rows if r["sent"] != "local"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("bridge_records", type=Path)
    ap.add_argument("socket")
    ap.add_argument("out", type=Path)
    ap.add_argument("--noise-seed", type=int, required=True)
    ap.add_argument("--odom", choices=("recorded", "truth"), required=True)
    ap.add_argument("--until-s", type=float, default=float("inf"))
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
    send = set(sent_ids(a.bridge_records))
    key = "odom_base" if a.odom == "recorded" else "truth_base"
    print("config " + json.dumps({"run": str(a.run.resolve()), "noise_seed": a.noise_seed, "odom": a.odom,
                                  "until_s": a.until_s, "keyframes": len(send)}), flush=True)
    rng = np.random.default_rng([a.noise_seed, 7])
    for _ in range(600):
        if Path(a.socket).exists():
            break
        time.sleep(1)
    client = slam_link.SlamLinkClient(a.socket, timeout_s=120.0)
    started, sent = time.monotonic(), 0
    with open(a.out, "w") as out:
        for r, stamp, clean in zip(records, stamps, ranges):
            if stamp > a.until_s:
                break
            scan_ranges = np.asarray(clean, float).copy()
            noise = rng.normal(0, STD, len(scan_ranges)).clip(-CUT, CUT)  # drawn for every scan, as live
            if r["scan_id"] not in send:
                continue
            finite = np.isfinite(scan_ranges)
            scan_ranges[finite] = np.clip(scan_ranges[finite] + noise[finite], RMIN, RMAX)
            pose = tuple(float(v) for v in r[key])
            reply = client.exchange(slam_link.Scan(r["scan_id"], float(stamp), pose, scan_ranges.astype(np.float32),
                                                   ANG0, DANG, RMIN, RMAX))
            sent += 1
            out.write(json.dumps({"scan_id": r["scan_id"], "stamp_s": float(stamp), "status": reply.status,
                                  "reason": reply.reason, "slam_wall_s": reply.slam_wall_s,
                                  "map_from_odom": list(reply.map_from_odom), "odom": list(pose),
                                  "truth_base": list(r["truth_base"])}) + "\n")
            if reply.status == "failed":
                print(f"failed at scan {r['scan_id']}: {reply.reason}", flush=True)
                break
    client.close()
    print(f"sent {sent} scans in {time.monotonic() - started:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
