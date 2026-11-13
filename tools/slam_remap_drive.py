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
the record's own send time (``sent_mono``, kept for every scan of the asynchronous link; scans without one are
filled by stamp and counted, the synchronous start-up before the first timed scan is sent without waiting and the
pacing clock starts at that scan), so slam_toolbox's wall-clock map regeneration overlaps keyframes as live
(lockstep otherwise); ``--wall-budget-s B`` refuses a paced replay planned longer than B; ``--splice-to-sim-s T``
repeats the record from scan 0 to the last scan back at the start pose until sim T, stamps continued, odometry
continued by the record's own increments and the truth repeated; ``--splice-runs RUN ...`` takes the units in turn
from the main run and these other runs of the same scene (slam-bounded-graph plan 4.1). Each processed reply's line
carries its unit, source run and source scan, the odometry sent and the truth for the accuracy summary, and the
monotonic clock when it was sent and received (one host: comparable across the two phases of an H replay).

Online-SLAM plan (docs/plans/2026-11-13-slam-online-bounded.md): ``--end-unit 1`` sends only the spliced stream's
unit 0 and ``--start-unit 1`` the rest, the skipped units' noise drawn and dropped, so H's two phases (unit 0 mapped,
then localized on that map; deploy/slurm/slam_h_switch.sbatch) send exactly the scans one replay would;
``--session-bundle DIR --session-map-id ID`` refuses a localization map that is not this run's unit 0;
``--pace-sim F`` queues each scan at its stamp over F through the live asynchronous link (the D arms' fixed feed)
and records when each was queued.

    python tools/slam_remap_drive.py RUN_DIR SOCKET OUT_JSONL [--noise-seed S] [--odom-file F] [--ranges-bin B]
        [--allow-unanswered-tail N] [--pace-live] [--wall-budget-s B] [--splice-to-sim-s T [--splice-runs RUN ...]
        [--start-unit N] [--end-unit N] [--session-bundle DIR --session-map-id ID] [--pace-sim F]]
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
    """(offsets, estimated, first_timed): each scan's live wall offset from the first scan with a recorded send time
    (``sent_mono``), scans without one interpolated by stamp between timed scans and those before the first timed
    scan at 0 (sent without waiting: the synchronous start-up); refuses a non-finite or decreasing send time."""
    stamps = np.array([float(r["stamp_s"]) for r in records])
    timed = [i for i, r in enumerate(records) if r.get("sent_mono") is not None]
    if len(timed) < 2:
        raise SystemExit("pacing needs at least two scans with a recorded send time")
    mono = np.array([float(records[i]["sent_mono"]) for i in timed])
    if not np.all(np.isfinite(mono)) or np.any(np.diff(mono) < 0):
        raise SystemExit("recorded send times are not finite and non-decreasing")
    wall = np.interp(stamps, stamps[timed], mono)
    wall[timed] = mono
    return np.maximum(wall - mono[0], 0.0), len(records) - len(timed), timed[0]


def planned_wall_s(plan: list, pace, unit_wall: float) -> float:
    """The paced replay's wall length from its anchor: the latest due time of any planned scan."""
    return max(float(pace[p.src]) + p.unit * unit_wall for p in plan)


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
    run: int = 0  # which source run (splice_plan_multi)


@dataclass(frozen=True)
class Source:
    """One run's scans for a splice: its stamps, odometry, truth, ranges and the splice end (scan back at the start)."""
    name: str
    stamps: list
    odom: list
    truth: list
    ranges: object
    end: int


def splice_plan(stamps, odom, truth, *, end, until_sim_s) -> list:
    """The scans to send: the record once (``end`` None), or units of scans 0..end-1 repeated until sim
    ``until_sim_s`` -- unit j's stamps shifted by j x (stamp[end] - stamp[0]), its odometry the previous scan's composed
    with the record's increment (across a seam, the increment into scan ``end``), its truth the record's."""
    if end is None:
        return [Planned(i, 0, float(stamps[i]), tuple(float(v) for v in odom[i]), tuple(truth[i]))
                for i in range(len(stamps))]
    return splice_plan_multi([Source("", stamps, odom, truth, None, end)], until_sim_s=until_sim_s)


def first_unit_plan(stamps, odom, truth, *, end) -> list:
    """The record's scans 0..end-1 (one unit, up to its return scan): the run a replay map bundle is built from."""
    return splice_plan(stamps, odom, truth, end=end, until_sim_s=float(stamps[end - 1]))


def splice_plan_multi(sources: list, *, until_sim_s) -> list:
    """Units taken in turn from ``sources`` (runs of one scene, each from its scan 0 to before its return scan
    ``end``) until sim ``until_sim_s`` (slam-bounded-graph plan 4.1): each unit starts where the previous one's period
    ended, its odometry continues the previous scan's by the unit's own increments (across a seam, the previous
    run's step into its return scan), its truth and scan ids are its own run's."""
    out, unit, prev = [], 0, None
    pose = tuple(float(v) for v in sources[0].odom[0])
    t = float(sources[0].stamps[0])
    while True:
        k = unit % len(sources)
        src = sources[k]
        base = float(src.stamps[0])
        for i in range(src.end):
            if i:
                pose = compose(pose, relative(src.odom[i - 1], src.odom[i]))
            elif prev is not None:
                pose = compose(pose, relative(prev.odom[prev.end - 1], prev.odom[prev.end]))
            stamp = t + float(src.stamps[i]) - base
            if stamp > until_sim_s:
                return out
            out.append(Planned(i, unit, stamp, pose, tuple(src.truth[i]), run=k))
        t += float(src.stamps[src.end]) - base
        prev, unit = src, unit + 1


def load_source(run: Path, *, allow_tail: int) -> tuple:
    """(records, slam log, Source with end None) of one run's answered scans, checked against each other."""
    records = load_list(run / "spool", "slam.records")
    log = load_slam_log(run)
    stamps = np.asarray(log["scan_stamps_s"], float)
    if len(np.asarray(log.get("silenced_scan_stamps_s", []))):
        raise SystemExit(f"{run}: silenced scans: the noise stream would need them skipped")
    n = answered_length(len(records), len(stamps), allow_tail=allow_tail)
    if [r["scan_id"] for r in records] != list(range(len(records))):
        raise SystemExit(f"{run}: spool records are not scans 0..n-1 of the slam log")
    if not np.allclose([r["stamp_s"] for r in records], stamps[:n], atol=1e-6):
        raise SystemExit(f"{run}: spool stamps differ from the slam log")
    src = Source(str(run.resolve()), stamps[:n], [r["odom_base"] for r in records],
                 [tuple(r["truth_base"]) for r in records], log["scan_ranges_m"][:n], None)
    return records, log, src


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


def noisy_stream(plan: list, sources: list, *, seed: int, start_unit: int = 0, end_unit: int | None = None,
                 given: dict | None = None):
    """(k, item, ranges) of every planned scan from unit ``start_unit`` on: the record's clean ranges plus the shared
    noise drawn in plan order -- the draws of the skipped units are taken and dropped, so a replay that starts at a
    later unit (H's second phase, online-SLAM plan 1-H) sends the same scans as the whole stream. ``end_unit``
    stops before that unit (H's first phase: the session's own unit 0, mapped)."""
    rng = np.random.default_rng([seed, 7])
    for k, item in enumerate(plan):
        if end_unit is not None and item.unit >= end_unit:
            return
        clean = sources[item.run].ranges[item.src]
        noise = rng.normal(0, STD, len(clean)).clip(-CUT, CUT)
        if item.unit < start_unit:
            continue
        scan_ranges = np.asarray(clean, float).copy()
        finite = np.isfinite(scan_ranges)
        scan_ranges[finite] = np.clip(scan_ranges[finite] + noise[finite], RMIN, RMAX)
        if given:
            scan_ranges = given.pop(round(float(item.stamp_s), 6), scan_ranges)
        yield k, item, scan_ranges


def sim_due_offsets(plan: list, *, factor: float, start_unit: int = 0, end_unit: int | None = None) -> list:
    """Each sent scan's wall offset from the first one sent: its stamp's distance from the first sent stamp over the
    factor (online-SLAM plan 4: the D arms are fed at a fixed rate, factor 1.0, not as fast as replies come back)."""
    if not factor > 0:
        raise SystemExit("--pace-sim needs a factor above 0")
    sent = [p.stamp_s for p in plan if p.unit >= start_unit and (end_unit is None or p.unit < end_unit)]
    return [(t - sent[0]) / factor for t in sent] if sent else []


def feed_sim(client, stream, due: list, write, *, clock=time.monotonic, sleep=time.sleep) -> dict:
    """The D arms' fixed feed (online-SLAM plan 4): each scan queued at its due wall offset whether or not the bridge
    has answered the last, through the live ``AsyncSlamLink`` (one scan in flight, the rest queued, in order) with
    a deadline and a queue too generous to end a replay -- a slow reply is recorded, the summary judges it.
    ``write(k, item, reply, queued, sent, received)`` gets every processed reply; ``queued`` is when the scan was
    due and queued (the latency a live runner would see runs from there), ``sent`` when its last byte left."""
    link = slam_link.AsyncSlamLink(client, deadline_s=600.0, queue_max=1_000_000, clock=clock)
    anchor, sent, other = clock(), 0, 0

    def take():
        nonlocal other
        for reply, meta, sent_mono, recv_mono in link.poll():
            if reply.status == "processed":
                write(meta["k"], meta["item"], reply, meta["queued"], sent_mono, recv_mono)
            else:
                other += 1

    for (k, item, scan_ranges), d in zip(stream, due, strict=True):
        while clock() < anchor + d:
            take()
            sleep(min(0.005, max(0.0, anchor + d - clock())))
        link.enqueue(slam_link.Scan(k, float(item.stamp_s), item.odom, np.asarray(scan_ranges, np.float32), ANG0,
                                    DANG, RMIN, RMAX), {"k": k, "item": item, "queued": clock()})
        sent += 1
        take()
    while link.depth():
        take()
        sleep(0.005)
    return {"sent": sent, "not_processed": other, "max_depth": link.max_depth,
            "long_poll_gaps": len(link.long_poll_gaps)}


def check_session_bundle(manifest: dict, *, run: Path, map_id: str) -> None:
    """H's map (online-SLAM plan 1-H, lead 2026-11-13): a replay bundle built from this run's unit 0 alone, with the
    map id the localization is told -- never a registry bundle or another run's map."""
    src = manifest.get("source") or {}
    problems = []
    if manifest.get("replay") is not True:
        problems.append("not a replay bundle")
    if Path(str(src.get("run", ""))).resolve() != Path(run).resolve():
        problems.append(f"built from {src.get('run')!r}, not this run {str(Path(run).resolve())!r}")
    if src.get("units") != [0]:
        problems.append(f"built from units {src.get('units')!r}, not [0]")
    if manifest.get("map_id") != map_id:
        problems.append(f"map id {manifest.get('map_id')!r}, not {map_id!r}")
    if problems:
        raise SystemExit("session bundle refused: " + "; ".join(problems))


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
    ap.add_argument("--wall-budget-s", type=float, default=None, help="refuse a paced replay planned longer than this")
    ap.add_argument("--splice-to-sim-s", type=float, default=None)
    ap.add_argument("--first-unit-only", action="store_true",
                    help="send scan 0 up to the run's return scan only (a map for candidate A)")
    ap.add_argument("--splice-runs", type=Path, nargs="+", default=[],
                    help="other runs of the same scene whose units alternate with this one's (needs --splice-to-sim-s)")
    ap.add_argument("--start-unit", type=int, default=0,
                    help="send the spliced units from this one on, the earlier ones' noise drawn and dropped (H's "
                         "second phase)")
    ap.add_argument("--end-unit", type=int, default=None,
                    help="stop before this spliced unit (H's first phase: --end-unit 1 maps the session's unit 0)")
    ap.add_argument("--session-bundle", type=Path, default=None,
                    help="H: the localization's bundle folder, refused unless built from this run's unit 0")
    ap.add_argument("--session-map-id", default=None, help="H: the map id the localization was told")
    ap.add_argument("--pace-sim", type=float, default=None,
                    help="send each scan at its stamp over this factor (queued, not waiting on replies; the D arms)")
    a = ap.parse_args(argv)
    if (a.start_unit or a.end_unit is not None) and a.splice_to_sim_s is None:
        raise SystemExit("--start-unit and --end-unit need --splice-to-sim-s")
    if a.end_unit is not None and a.end_unit <= a.start_unit:
        raise SystemExit("--end-unit must be past --start-unit")
    if (a.session_bundle is None) != (a.session_map_id is None):
        raise SystemExit("--session-bundle and --session-map-id go together")
    if a.session_bundle is not None:
        check_session_bundle(json.loads((a.session_bundle / "manifest.json").read_text()), run=a.run,
                             map_id=a.session_map_id)
    if a.pace_sim is not None and (a.pace_live or a.first_unit_only):
        raise SystemExit("--pace-sim is its own pacing (not with --pace-live or --first-unit-only)")
    if a.splice_runs and (a.splice_to_sim_s is None or a.pace_live):
        raise SystemExit("--splice-runs needs --splice-to-sim-s and is not paced")
    records, log, main_src = load_source(a.run, allow_tail=a.allow_unanswered_tail)
    stamps, ranges, n = main_src.stamps, main_src.ranges, len(records)
    odom = None
    if a.odom_file:
        from gyro_heading_offline import record_digest, run_geometry

        odom = load_odom_file(a.odom_file, a.run, records, record_digest(log, records, run_geometry(a.run)))
    if a.splice_to_sim_s is not None and (a.ranges_bin or a.odom_file):
        raise SystemExit("--splice-to-sim-s takes the record's own odometry and redrawn noise only")
    src_odom = odom if odom is not None else main_src.odom
    truth = main_src.truth
    if a.first_unit_only and (a.splice_to_sim_s is not None or a.pace_live):
        raise SystemExit("--first-unit-only is not spliced or paced")
    end = splice_end(truth) if a.splice_to_sim_s is not None or a.first_unit_only else None
    sources = [Source(main_src.name, stamps, src_odom, truth, ranges, end)]
    for other in a.splice_runs:
        _, _, o = load_source(other, allow_tail=a.allow_unanswered_tail)
        sources.append(Source(o.name, o.stamps, o.odom, o.truth, o.ranges, splice_end(o.truth)))
    if a.first_unit_only:
        plan = first_unit_plan(stamps, src_odom, truth, end=end)
    elif len(sources) > 1:
        plan = splice_plan_multi(sources, until_sim_s=a.splice_to_sim_s)
    else:
        plan = splice_plan(stamps, src_odom, truth, end=end, until_sim_s=a.splice_to_sim_s)
    pace, estimated, first_timed, planned_wall = None, None, None, None
    if a.pace_live:
        pace, estimated, first_timed = pace_offsets(records[:n])
        unit_wall = float(pace[end]) if end is not None else 0.0
        planned_wall = planned_wall_s(plan, pace, unit_wall)
        if a.wall_budget_s is not None and planned_wall > a.wall_budget_s:
            raise SystemExit(f"paced replay needs {planned_wall:.0f} s of wall time, the budget is {a.wall_budget_s:.0f} s")
    print("config " + json.dumps({"run": str(a.run.resolve()), "noise_seed": a.noise_seed,
                                  "odom_file": str(a.odom_file) if a.odom_file else None,
                                  "ranges_bin": str(a.ranges_bin) if a.ranges_bin else None,
                                  "unanswered_tail": len(log["scan_stamps_s"]) - n,
                                  "pace_live": a.pace_live, "pace_estimated": estimated,
                                  "pace_first_timed": first_timed, "planned_wall_s": planned_wall,
                                  "splice_end": end, "splice_to_sim_s": a.splice_to_sim_s, "planned": len(plan),
                                  "start_unit": a.start_unit, "end_unit": a.end_unit, "pace_sim": a.pace_sim,
                                  "session_bundle": str(a.session_bundle) if a.session_bundle else None,
                                  "sources": [{"run": x.name, "splice_end": x.end} for x in sources]}),
          flush=True)
    given = read_ranges_bin(a.ranges_bin) if a.ranges_bin else {}
    for _ in range(600):
        if Path(a.socket).exists():
            break
        time.sleep(1)
    client = slam_link.SlamLinkClient(a.socket, timeout_s=120.0)
    started, sent = time.monotonic(), 0
    stream = noisy_stream(plan, sources, seed=a.noise_seed, start_unit=a.start_unit, end_unit=a.end_unit,
                          given=given)

    def line(k, item, reply, sent_mono, recv_mono, queued_mono=None):
        return json.dumps({"scan_id": k, "src": item.src, "run": item.run, "unit": item.unit,
                           "stamp_s": float(item.stamp_s), "slam_wall_s": reply.slam_wall_s,
                           "map_from_odom": list(reply.map_from_odom), "odom_base": list(item.odom),
                           "truth_base": list(item.truth), "queued_mono": queued_mono, "sent_mono": sent_mono,
                           "recv_mono": recv_mono}) + "\n"

    with open(a.out, "w") as out:
        if a.pace_sim is not None:
            stats = feed_sim(client, stream, sim_due_offsets(plan, factor=a.pace_sim, start_unit=a.start_unit,
                                                             end_unit=a.end_unit),
                             lambda k, item, reply, q, sm, rm: out.write(line(k, item, reply, sm, rm, q)))
            sent += stats["sent"]
            print("pace_sim " + json.dumps(stats), flush=True)
        else:
            anchor = None
            for k, item, scan_ranges in stream:
                stamp = item.stamp_s
                if pace is not None and (anchor is not None or (item.unit == 0 and item.src >= first_timed)):
                    if anchor is None:
                        anchor = time.monotonic() - float(pace[item.src])
                    wait = anchor + float(pace[item.src]) + item.unit * unit_wall - time.monotonic()
                    if wait > 0:
                        time.sleep(wait)
                sent_mono = time.monotonic()
                reply = client.exchange(slam_link.Scan(k, float(stamp), item.odom, scan_ranges.astype(np.float32),
                                                       ANG0, DANG, RMIN, RMAX))
                sent += 1
                if reply.status == "processed":
                    out.write(line(k, item, reply, sent_mono, time.monotonic()))
    client.close()
    if given:
        raise SystemExit(f"{len(given)} given scans were not sent")
    print(f"sent {sent} scans in {time.monotonic() - started:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
