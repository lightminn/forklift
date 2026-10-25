"""Replay a run's recorded SLAM scans into a fresh mapping bridge, lockstep (plans 2026-10-20 L2 r6 diagnosis,
2026-10-21 gyro-heading V-G0).

Every scan the runner sent, in order: the recorded clean ranges plus the shared-scan noise drawn as the runner draws it
(obstacle layer rng ``default_rng([seed, 7])``, one truncated N(0, 0.02) draw per beam clipped to +-0.06, added to the
finite beams and clipped to the range limits) -- a redraw, not the live stream; ``--ranges-bin`` (tools range dump of a
saved graph) puts given ranges on the scans at its stamps instead (the live graph's own keyframes reproduce the live
graph). The odometry is the record's ``odom_base`` (spool ``slam.records``), or ``--odom-file`` (tools/
gyro_heading_offline.py), checked against the record: same run, a hash over the record's arrays, scan ids and stamps. The bridge's
own KeyframeGate picks keyframes, as live. Prints ``config {...}`` (what this replay used) and ``sent N scans`` -- N is checked against the record by the summary.

slam_link_failed plan E0 (docs/plans/2026-10-25-slam-link-failed.md 3): ``--allow-unanswered-tail N`` replays a failed
run whose last scans were sent without a record; ``--pace-live`` sends each scan no earlier than its live wall offset,
the record's own send time (``sent_mono``, kept for every scan of the asynchronous link; the synchronous start-up
scans without one are filled by stamp and counted), so slam_toolbox's wall-clock map regeneration overlaps keyframes as
live (lockstep otherwise); ``--splice-to-sim-s T``
repeats the record from scan 0 to the last scan back at the start pose until sim T, stamps continued, odometry
continued by the record's own increments and the truth repeated. Each processed reply's line carries its unit, the
odometry sent and the truth for the accuracy summary.

    python tools/slam_remap_drive.py RUN_DIR SOCKET OUT_JSONL [--noise-seed S] [--odom-file F] [--ranges-bin B]
        [--allow-unanswered-tail N] [--pace-live] [--splice-to-sim-s T]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
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


SPLICE_TOL_M, SPLICE_TOL_RAD = 0.10, 0.05  # PR #62 S1's seam limits


def answered_length(records: int, scans: int, *, allow_tail: int) -> int:
    """How many scans to replay: the recorded ones, when the slam log holds at most ``allow_tail`` more (a failed run's
    last scans were sent without a record)."""
    if not 0 <= scans - records <= allow_tail:
        raise SystemExit(f"slam log holds {scans} scans, the record {records} (unanswered tail allowed: {allow_tail})")
    return records


def pace_offsets(records: list) -> tuple:
    """Each scan's live wall offset from scan 0 and how many were estimated: the record's ``sent_mono`` where kept,
    otherwise interpolated by stamp between the timed scans and held flat beyond them."""
    stamps = np.array([float(r["stamp_s"]) for r in records])
    timed = [i for i, r in enumerate(records) if r.get("sent_mono") is not None]
    if len(timed) < 2:
        raise SystemExit("pacing needs at least two scans with a recorded send time")
    mono = np.maximum.accumulate(np.array([float(records[i]["sent_mono"]) for i in timed]))
    wall = np.interp(stamps, stamps[timed], mono)
    wall[timed] = mono
    wall = np.maximum.accumulate(wall)
    return wall - wall[0], len(records) - len(timed)


def relative(a, b) -> tuple:
    """b expressed in a's frame (planar poses)."""
    ax, ay, ayaw = (float(v) for v in a)
    bx, by, byaw = (float(v) for v in b)
    c, s = math.cos(ayaw), math.sin(ayaw)
    dx, dy = bx - ax, by - ay
    return (c * dx + s * dy, -s * dx + c * dy, byaw - ayaw)


def compose(a, b) -> tuple:
    ax, ay, ayaw = (float(v) for v in a)
    bx, by, byaw = (float(v) for v in b)
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw)


def splice_end(truth: list) -> int:
    """The last scan in the record's second half whose true pose is back at scan 0's (within PR #62 S1's seam limits)."""
    x0, y0, yaw0 = (float(v) for v in truth[0])
    for k in range(len(truth) - 1, len(truth) // 2 - 1, -1):
        x, y, yaw = (float(v) for v in truth[k])
        if (math.hypot(x - x0, y - y0) <= SPLICE_TOL_M
                and abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))) <= SPLICE_TOL_RAD):
            return k
    raise SystemExit("no scan in the record's second half is back at the start pose; cannot splice")


@dataclass(frozen=True)
class Planned:
    src: int  # the record's scan this one repeats
    unit: int
    stamp_s: float
    odom: tuple
    truth: tuple


def splice_plan(stamps, odom, truth, *, end, until_sim_s) -> list:
    """The scans to send: the record once (``end`` None), or units of scans 0..end-1 repeated until sim
    ``until_sim_s`` -- unit j's stamps shifted by j x (stamp[end] - stamp[0]), its odometry the previous scan's composed
    with the record's increment (across a seam, the increment into scan ``end``), its truth the record's."""
    if end is None:
        return [Planned(i, 0, float(stamps[i]), tuple(float(v) for v in odom[i]), tuple(truth[i]))
                for i in range(len(stamps))]
    period = float(stamps[end]) - float(stamps[0])
    out, unit, pose = [], 0, tuple(float(v) for v in odom[0])
    while True:
        for i in range(end):
            if unit or i:
                prev = i - 1 if i else end - 1
                nxt = i if i else end
                pose = compose(pose, relative(odom[prev], odom[nxt]))
            stamp = float(stamps[i]) + unit * period
            if stamp > until_sim_s:
                return out
            out.append(Planned(i, unit, stamp, pose, tuple(truth[i])))
        unit += 1


def load_odom_file(path: Path, run: Path, records: list, digest: str) -> np.ndarray:
    """The file's odometry, refused unless it was built from this record (tools/gyro_heading_offline.record_digest)."""
    z = np.load(path)
    if str(z["run"]) != str(run.resolve()):
        raise SystemExit(f"odometry file is for {z['run']}, not {run.resolve()}")
    if str(z["record_sha256"]) != digest:
        raise SystemExit("odometry file's record hash differs from the run's")
    if not (np.array_equal(z["scan_id"], [r["scan_id"] for r in records])
            and np.array_equal(z["stamp_s"], [r["stamp_s"] for r in records])):
        raise SystemExit("odometry file's scan ids or stamps differ from the record")
    return z["odom_base"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("socket")
    ap.add_argument("out", type=Path)
    ap.add_argument("--noise-seed", type=int, required=True)
    ap.add_argument("--odom-file", type=Path, default=None)
    ap.add_argument("--ranges-bin", type=Path, default=None)
    ap.add_argument("--allow-unanswered-tail", type=int, default=0)
    ap.add_argument("--pace-live", action="store_true", help="send at the record's own send times")
    ap.add_argument("--splice-to-sim-s", type=float, default=None)
    a = ap.parse_args(argv)
    records = load_list(a.run / "spool", "slam.records")
    log = load_slam_log(a.run)
    stamps = np.asarray(log["scan_stamps_s"], float)
    ranges = log["scan_ranges_m"]
    if len(np.asarray(log.get("silenced_scan_stamps_s", []))):
        raise SystemExit("silenced scans: the noise stream would need them skipped")
    n = answered_length(len(records), len(stamps), allow_tail=a.allow_unanswered_tail)
    stamps, ranges = stamps[:n], ranges[:n]
    if [r["scan_id"] for r in records] != list(range(len(records))):
        raise SystemExit("spool records are not scans 0..n-1 of the slam log")
    if not np.allclose([r["stamp_s"] for r in records], stamps, atol=1e-6):
        raise SystemExit("spool stamps differ from the slam log")
    odom = None
    if a.odom_file:
        from gyro_heading_offline import record_digest, run_geometry

        odom = load_odom_file(a.odom_file, a.run, records, record_digest(log, records, run_geometry(a.run)))
    if a.splice_to_sim_s is not None and (a.ranges_bin or a.odom_file):
        raise SystemExit("--splice-to-sim-s takes the record's own odometry and redrawn noise only")
    src_odom = odom if odom is not None else [r["odom_base"] for r in records]
    truth = [tuple(r["truth_base"]) for r in records]
    end = splice_end(truth) if a.splice_to_sim_s is not None else None
    plan = splice_plan(stamps, src_odom, truth, end=end, until_sim_s=a.splice_to_sim_s)
    pace, estimated = None, None
    if a.pace_live:
        pace, estimated = pace_offsets(records[:n])
        unit_wall = float(pace[end]) if end is not None else 0.0
    print("config " + json.dumps({"run": str(a.run.resolve()), "noise_seed": a.noise_seed,
                                  "odom_file": str(a.odom_file) if a.odom_file else None,
                                  "ranges_bin": str(a.ranges_bin) if a.ranges_bin else None,
                                  "unanswered_tail": len(log["scan_stamps_s"]) - n,
                                  "pace_live": a.pace_live, "pace_estimated": estimated,
                                  "splice_end": end, "splice_to_sim_s": a.splice_to_sim_s, "planned": len(plan)}),
          flush=True)
    given = read_ranges_bin(a.ranges_bin) if a.ranges_bin else {}
    rng = np.random.default_rng([a.noise_seed, 7])
    for _ in range(600):
        if Path(a.socket).exists():
            break
        time.sleep(1)
    client = slam_link.SlamLinkClient(a.socket, timeout_s=120.0)
    started, sent = time.monotonic(), 0
    with open(a.out, "w") as out:
        for k, item in enumerate(plan):
            stamp, clean = item.stamp_s, ranges[item.src]
            if pace is not None:
                due = started + float(pace[item.src]) + item.unit * unit_wall
                wait = due - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
            scan_ranges = np.asarray(clean, float).copy()
            noise = rng.normal(0, STD, len(scan_ranges)).clip(-CUT, CUT)
            finite = np.isfinite(scan_ranges)
            scan_ranges[finite] = np.clip(scan_ranges[finite] + noise[finite], RMIN, RMAX)
            if given:
                scan_ranges = given.pop(round(float(stamp), 6), scan_ranges)
            reply = client.exchange(slam_link.Scan(k, float(stamp), item.odom, scan_ranges.astype(np.float32),
                                                   ANG0, DANG, RMIN, RMAX))
            sent += 1
            if reply.status == "processed":
                out.write(json.dumps({"scan_id": k, "src": item.src, "unit": item.unit, "stamp_s": float(stamp),
                                      "slam_wall_s": reply.slam_wall_s, "map_from_odom": list(reply.map_from_odom),
                                      "odom_base": list(item.odom), "truth_base": list(item.truth)}) + "\n")
    client.close()
    if given:
        raise SystemExit(f"{len(given)} given scans were not sent")
    print(f"sent {sent} scans in {time.monotonic() - started:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
