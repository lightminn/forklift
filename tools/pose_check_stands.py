"""The two monitor candidates of the design revision, read from the shadow D records (plan
docs/plans/2026-11-06-unconfirmed-validation.md section 6 step 4; development data only).

- **monitors** (the revision plan's 1-3): the cumulative Delta_int, its proportional form and the sliding window, each
  fitted on the correct sessions (1.5 x the observed largest) and judged on them, on the positive sessions and on the
  offline injection: a fixed SE(2) correction added to the recorded stream, the true error computed from the result.
  It runs on the recorded trajectory, so it says nothing of what a real jump would do to the plan and the scans.
- **stands**: the frozen relocalizer and the conservative check on the mission stands' scan sets (plan 1.5, a snapshot
  approximation: sets in a row, compared with the last correction of the stand), outcome by outcome -- renew (it would
  have set C_int again), revoke (confirmed but the correction disagrees past E_SLAM), no_evidence (the matcher or the
  check gave nothing) or not_estimable (no scan set, no sample after them, or a stand past the recording's limit).

    python tools/pose_check_stands.py monitors SESSION... --out OUT
    python tools/pose_check_stands.py stands SESSION... --grid GRID.npz --e-slam M,RAD --out OUT
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "sim/isaac")]

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402

WINDOWS_M = (2.0, 5.0, 10.0, 20.0)
REAR_OFFSET_M = 0.34  # the measured chassis's: main checks it against each record's arguments
FACTOR = 1.5  # the development values' margin
RAMP_M = 20.0  # a slow injection's path
WRONG = (0.30, 0.10)  # B-P2 4.0's wrong boundary
ONSET_SEED, ONSET_DRAWS = 20261107, 20


def _gap(a, b, rear: float = REAR_OFFSET_M) -> tuple:
    ra, rb = R.rear_from_base(a, rear), R.rear_from_base(b, rear)
    return (max(math.hypot(a[0] - b[0], a[1] - b[1]), math.hypot(ra[0] - rb[0], ra[1] - rb[1])),
            abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))))


def _classify(pose, truth) -> str:
    m, rad = _gap(tuple(pose), tuple(truth), 0.0)
    return "success" if m <= 0.10 and rad <= 0.05 else ("wrong" if m > 0.30 or rad > 0.10 else "imprecise")


def rematch_stands(rec, scans, ctx, *, b, o0, e_slam) -> list:
    """One row a kept mission stand. ``scans``: the session's pose_check_stand_scans.npz; ``b`` at ``o0``: the estimate."""
    import pose_runtime as PR

    from forklift_core.localization import twin_check as T

    rows = []
    o_s = tuple(rec["start"]["o_scan"])
    hyps = rec.get("hyps") or []
    samples = rec.get("samples") or []
    for i, cap in enumerate(rec.get("stand_sets") or []):
        row = {"t": cap["t0"], "delta": cap.get("delta"), "sets": len(cap["sets"]), "sets_used": 0,
               "outcome": "not_estimable"}
        rows.append(row)
        rc = ctx.reloc_cfg
        relocalizer = R.StartRelocalizer(rc, time.monotonic)
        relocalizer.start()
        for k, st in enumerate(cap["sets"]):
            row["sets_used"] = k + 1
            row["outcome"] = "no_evidence"
            ranges = scans[f"s{i}_{k}"]
            sets = [((0.0, 0.0, 0.0), r, float(scans["angles"][0]), float(scans["angles"][1])) for r in ranges]
            rmin, rmax, mrange = (float(v) for v in scans["ranges_m"])
            points = G.stack_scans(sets, range_min_m=rmin, range_max_m=rmax, match_range_m=mrange)
            out = PR.run_job("relocalize", {"points": points, "deadline_at": relocalizer.deadline_at}, ctx)
            state = relocalizer.record(out["match"], out["decision"])
            if state == "rescan":
                continue
            if state != "chosen":
                break
            g = tuple(relocalizer.estimate.base)
            o_scan = tuple(st["o"])
            # the stand's last correction, the evidence carried to that odometry (plan 1.5; Codex revision review P2-3)
            end = cap["t1"] if cap.get("t1") is not None else math.inf
            after = [s for s in samples if st["stamps"][-1] <= s["t"] <= end and s.get("o") is not None]
            if not after:
                row["outcome"] = "not_estimable"
                break
            last = after[-1]
            res = PR.run_job("rematch", {"phase": "confirmed", "points": points, "b": T.moved(tuple(b), tuple(o0), o_s),
                                         "hyps": hyps, "o0": o_s, "ot": o_scan, "decision": "confirmed", "g": g,
                                         "deadline_at": relocalizer.deadline_at}, ctx)  # the generation's own
            row["agree_m"], row["agree_rad"] = _gap(tuple(last["applied"]),
                                                    compose(g, compose(invert(o_scan), tuple(last["o"]))))
            row["truth_class"] = _classify(g, last["truth"])
            if res["outcome"] == "confirm":
                ok = row["agree_m"] <= e_slam[0] and row["agree_rad"] <= e_slam[1]
                row["outcome"] = "renew" if ok else "revoke"
            break
    return rows


def summary(by_session, skipped: int = 0) -> dict:
    """``skipped``: stands with no record to read (past the recording's limit, or no scan file): not_estimable."""
    counts: dict = {}
    for rows in by_session.values():
        for r in rows:
            counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1
    if skipped:
        counts["not_estimable"] = counts.get("not_estimable", 0) + skipped
    return {"sessions": len(by_session), "stands": sum(len(r) for r in by_session.values()) + skipped, "outcomes": counts}


# --- the monitors on a recorded stream, and the offline injection -----------------------------------------------------
@dataclass(frozen=True)
class Monitor:
    kind: str  # M0 (cumulative) | P (cumulative, proportional) | W (sliding window of path)
    limit: tuple = (math.inf, math.inf)
    per_m: tuple = (0.0, 0.0)  # P: the allowance per metre of path since the reference
    window_m: float = 0.0


@dataclass(frozen=True)
class Injection:
    kind: str  # step_xy | step_yaw | ramp_xy | ramp_yaw
    size: float  # m, rad, or per metre for a ramp
    direction: tuple = (0.0, 0.0)  # map axis of the _xy kinds; the sign of a _yaw kind is the size's


class Series:
    """A session's stream from its first reference: arrays of the odometry, the applied pose, the truth, the path, the
    reference C_int in force and the path since it."""

    def __init__(self, t, o, applied, truth, ref, since):
        self.t, self.o, self.applied, self.truth, self.ref, self.since = t, o, applied, truth, ref, since
        d = np.hypot(np.diff(o[:, 0], prepend=o[0, 0]), np.diff(o[:, 1], prepend=o[0, 1]))
        self.path = np.cumsum(d)
        self.C = _compose(applied, _invert(o))


def _compose(a, b):
    c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
    return np.column_stack((a[:, 0] + c * b[:, 0] - s * b[:, 1], a[:, 1] + s * b[:, 0] + c * b[:, 1], a[:, 2] + b[:, 2]))


def _invert(a):
    c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
    return np.column_stack((-c * a[:, 0] - s * a[:, 1], s * a[:, 0] - c * a[:, 1], -a[:, 2]))


def _gaps(a, b) -> tuple:
    """(metres, radians) between arrays of base poses, the metres the larger of the base and the rear axle."""
    da = np.arctan2(np.sin(a[:, 2] - b[:, 2]), np.cos(a[:, 2] - b[:, 2]))
    base = np.hypot(a[:, 0] - b[:, 0], a[:, 1] - b[:, 1])
    rx = a[:, 0] - b[:, 0] - REAR_OFFSET_M * (np.cos(a[:, 2]) - np.cos(b[:, 2]))
    ry = a[:, 1] - b[:, 1] - REAR_OFFSET_M * (np.sin(a[:, 2]) - np.sin(b[:, 2]))
    return np.maximum(base, np.hypot(rx, ry)), np.abs(da)


def series(samples, renewals) -> Series:
    """``renewals``: (t, C_int) in time order, each set from that time on. Samples before the first are not the
    monitor's."""
    rows = [s for s in samples if s.get("o") is not None and s.get("applied") is not None and s.get("truth") is not None
            and s["t"] >= renewals[0][0] - 1e-9]
    times = [t for t, _ in renewals]
    ref, since = [], []
    arr_t = np.array([s["t"] for s in rows])
    idx = np.searchsorted(times, arr_t, side="right") - 1
    ref = np.array([renewals[k][1] for k in idx], float)
    o = np.array([s["o"] for s in rows], float)
    step = np.hypot(np.diff(o[:, 0], prepend=o[0, 0]), np.diff(o[:, 1], prepend=o[0, 1]))
    total = 0.0
    since = np.zeros(len(rows))
    for i in range(len(rows)):
        total = 0.0 if i and idx[i] != idx[i - 1] else total + step[i]
        since[i] = total
    return Series(arr_t, o, np.array([s["applied"] for s in rows], float), np.array([s["truth"] for s in rows], float),
                  ref, since)


def _pairs(ser, mon) -> tuple:
    """(the applied pose now, the pose compared with it, the index whose correction made that pose; None: the fixed
    reference, which an injected error does not reach)."""
    if mon.kind == "W":
        j = np.searchsorted(ser.path, ser.path - mon.window_m, side="left")
        return ser.applied, _compose(ser.C[j], ser.o), j
    return ser.applied, _compose(ser.ref, ser.o), None


def _alarm(ser, mon, m, rad):
    lim_m, lim_rad = mon.limit
    if mon.kind == "P":
        lim_m, lim_rad = lim_m + mon.per_m[0] * ser.since, lim_rad + mon.per_m[1] * ser.since
    return (m > lim_m) | (rad > lim_rad)


def first_alarm(ser, mon):
    now, other, _ = _pairs(ser, mon)
    hits = np.flatnonzero(_alarm(ser, mon, *_gaps(now, other)))
    return int(hits[0]) if len(hits) else None


def _shift(ser, onset, case) -> tuple:
    """Per sample the injected correction: translation (dx, dy), rotation th about the truck's applied pose at onset."""
    n = len(ser.t)
    grown = np.clip(ser.path - ser.path[onset], 0.0, RAMP_M)
    on = (np.arange(n) >= onset).astype(float)
    amount = on * (grown * case.size if case.kind.startswith("ramp") else case.size)
    if case.kind.endswith("yaw"):
        return np.zeros(n), np.zeros(n), amount
    return amount * case.direction[0], amount * case.direction[1], np.zeros(n)


def _apply(pose, dx, dy, th, pivot):
    c, s = np.cos(th), np.sin(th)
    rx, ry = pose[:, 0] - pivot[0], pose[:, 1] - pivot[1]
    return np.column_stack((pivot[0] + c * rx - s * ry + dx, pivot[1] + s * rx + c * ry + dy, pose[:, 2] + th))


def injected_applied(ser, onset, case):
    dx, dy, th = _shift(ser, onset, case)
    return _apply(ser.applied, dx, dy, th, ser.applied[onset][:2])


def run_case(ser, onset, case, mon) -> dict:
    """One injected case: when the monitor alarms from the onset on, and how far and long the true error was past the
    wrong boundary before that. ``wrong_reached`` False: the error never reached it (a miss is no failure then)."""
    dx, dy, th = _shift(ser, onset, case)
    pivot = ser.applied[onset][:2]
    now, other, j = _pairs(ser, mon)
    now2 = _apply(now, dx, dy, th, pivot)
    other2 = other if j is None else _apply(other, dx[j], dy[j], th[j], pivot)
    alarm = _alarm(ser, mon, *_gaps(now2, other2))
    hits = np.flatnonzero(alarm[onset:])
    err_m, err_rad = np.hypot(*(now2[:, :2] - ser.truth[:, :2]).T), np.abs(
        np.arctan2(np.sin(now2[:, 2] - ser.truth[:, 2]), np.cos(now2[:, 2] - ser.truth[:, 2])))
    wrong = ((err_m > WRONG[0]) | (err_rad > WRONG[1]))[onset:]
    end = onset + int(hits[0]) if len(hits) else len(ser.t) - 1
    span = np.arange(onset, end)
    exposed = wrong[: end - onset]
    dp, dt = np.diff(ser.path, append=ser.path[-1])[span], np.diff(ser.t, append=ser.t[-1])[span]
    return {"detected": bool(len(hits)), "wrong_reached": bool(wrong.any()),
            "delay_m": float(ser.path[end] - ser.path[onset]) if len(hits) else None,
            "delay_s": float(ser.t[end] - ser.t[onset]) if len(hits) else None,
            "exposure_m": float(dp[exposed].sum()), "exposure_s": float(dt[exposed].sum())}


def fit_limit(streams, mon) -> tuple:
    """(metres, radians): the correct sessions' largest values x 1.5."""
    m_max, rad_max = 0.0, 0.0
    for ser in streams.values():
        now, other, _ = _pairs(ser, mon)
        m, rad = _gaps(now, other)
        m_max, rad_max = max(m_max, float(m.max())), max(rad_max, float(rad.max()))
    return (FACTOR * m_max, FACTOR * rad_max)


def fit_per_m(streams, floor_m: float = 5.0) -> tuple:
    """P's allowance per metre: the correct sessions' largest excess over the gate per metre of path, x 1.5."""
    m_rate, rad_rate = 0.0, 0.0
    base = Monitor("M0")
    for ser in streams.values():
        now, other, _ = _pairs(ser, base)
        m, rad = _gaps(now, other)
        far = ser.since >= floor_m
        if far.any():
            m_rate = max(m_rate, float((m[far] / ser.since[far]).max()))
            rad_rate = max(rad_rate, float((rad[far] / ser.since[far]).max()))
    return (FACTOR * m_rate, FACTOR * rad_rate)


def leave_one_out(streams, mon) -> dict:
    """Session: whether the monitor fitted on the others alarms on it. Fitted with the same rule as the candidate."""
    out = {}
    for name, ser in streams.items():
        rest = {k: v for k, v in streams.items() if k != name}
        if mon.kind == "P":
            fitted = Monitor("P", limit=(0.20, 0.05), per_m=fit_per_m(rest))
        elif mon.kind == "W":
            fitted = Monitor("W", window_m=mon.window_m, limit=fit_limit(rest, mon))
        else:
            fitted = mon
        out[name] = first_alarm(ser, fitted) is not None
    return out


def kinds() -> list:
    """The injection design (the revision plan 2 B), every kind and sign."""
    axes = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
    out = [Injection("step_xy", s, a) for s in (0.35, 0.5, 1.0, 3.0, 6.0, 12.0) for a in axes]
    out += [Injection("step_yaw", sign * s) for s in (0.12, 0.30, math.pi) for sign in (1.0, -1.0)]
    out += [Injection("ramp_xy", s, a) for s in (0.02, 0.05, 0.10, 0.20) for a in axes]
    out += [Injection("ramp_yaw", sign * s) for s in (0.005, 0.01) for sign in (1.0, -1.0)]
    return out


def onsets(ser, name: str) -> list:
    """(label, index): ONSET_DRAWS uniform on the path, and the boundaries: right after the reference, the middle of the
    longest stand, the sharpest turn."""
    rng = random.Random(f"{ONSET_SEED}:{name}")
    n = len(ser.t)
    out = [("draw", int(np.searchsorted(ser.path, rng.uniform(0.0, float(ser.path[-1]))))) for _ in range(ONSET_DRAWS)]
    out.append(("first", min(10, n - 1)))
    moving = np.diff(ser.path, append=ser.path[-1]) > 1e-4
    runs, start = [], None
    for i in range(n):
        if not moving[i] and start is None:
            start = i
        if (moving[i] or i == n - 1) and start is not None:
            runs.append((i - start, start))
            start = None
    if runs:
        length, first = max(runs)
        out.append(("stand", first + length // 2))
    rate = np.abs(np.diff(ser.applied[:, 2], append=ser.applied[-1, 2]))
    out.append(("turn", int(np.argmax(rate))))
    return [(label, min(i, n - 2)) for label, i in out]


def _tally(rows) -> dict:
    wrong = [r for r in rows if r["wrong_reached"]]
    hit = [r for r in wrong if r["detected"]]

    def med(values):
        v = sorted(values)
        return v[len(v) // 2] if v else None

    return {"cases": len(rows), "wrong_reached": len(wrong), "detected": len(hit),
            "rate": len(hit) / len(wrong) if wrong else None,
            "delay_m_p50": med([r["delay_m"] for r in hit]), "delay_m_max": max((r["delay_m"] for r in hit), default=None),
            "delay_s_p50": med([r["delay_s"] for r in hit]),
            "exposure_m_p50": med([r["exposure_m"] for r in wrong]),
            "exposure_m_max": max((r["exposure_m"] for r in wrong), default=None),
            "exposure_s_max": max((r["exposure_s"] for r in wrong), default=None),
            "miss_exposure_m_max": max((r["exposure_m"] for r in wrong if not r["detected"]), default=None)}


def renewals_of(rec) -> list:
    """(t, C_int) of each agreement that held: the evidence's own correction at its scan."""
    scans = {r["t"]: r.get("o_scan") for r in rec.get("rematches") or []}
    out = []
    for e in rec.get("events") or []:
        if e["kind"] in ("slam_agreement", "rematch_agreement") and e.get("ok"):
            o_scan = scans.get(e["t"]) or (rec.get("start") or {}).get("o_scan")
            out.append((e["t"], compose(tuple(e["evidence"]), invert(tuple(o_scan)))))
    return out


def evaluate(correct, missions, positives) -> dict:
    """``correct``: name -> Series of every correct session; ``missions``: those that drove one; ``positives``: name ->
    (Series or None, the true-error onset's time) -- None: never confirmed, no monitor runs."""
    candidates = [Monitor("M0", limit=(0.20, 0.05)),
                  Monitor("P", limit=(0.20, 0.05), per_m=fit_per_m(correct))]
    candidates += [Monitor("W", window_m=w, limit=fit_limit(correct, Monitor("W", window_m=w))) for w in WINDOWS_M]
    cases = kinds()
    out = {"kinds": len(cases), "onsets": ONSET_DRAWS + 3, "sessions": len(missions), "candidates": {}}
    for mon in candidates:
        label = mon.kind if mon.kind != "W" else f"W{mon.window_m:g}"
        false = [k for k, ser in correct.items() if first_alarm(ser, mon) is not None]
        loo = leave_one_out(correct, mon)
        by: dict = {}
        for name in missions:
            ser = correct[name]
            for tag, onset in onsets(ser, name):
                for case in cases:
                    key = f"{case.kind}:{abs(case.size):g}"
                    row = run_case(ser, onset, case, mon)
                    by.setdefault(key, []).append(row)
                    by.setdefault(f"{key}@{tag}" if tag != "draw" else f"{key}@draw", []).append(row)
        a = {}
        for name, (ser, t0) in positives.items():
            if ser is None:
                a[name] = {"observed": False}
                continue
            at = first_alarm(ser, mon)
            on = int(np.searchsorted(ser.t, t0))
            a[name] = {"observed": True, "alarm_t": None if at is None else float(ser.t[at]),
                       "delay_s": None if at is None else float(ser.t[at] - t0),
                       "delay_m": None if at is None else float(ser.path[at] - ser.path[min(on, len(ser.t) - 1)])}
        out["candidates"][label] = {
            "limit": list(mon.limit), "per_m": list(mon.per_m), "window_m": mon.window_m,
            "false_revocations": false, "false_in_missions": [k for k in false if k in missions],
            "loo_false": sorted(k for k, v in loo.items() if v), "loo_false_in_missions":
            sorted(k for k, v in loo.items() if v and k in missions), "positive_a": a,
            "by_kind": {k: _tally(v) for k, v in sorted(by.items())}}
    return out


def _record(folder):
    return json.loads((folder / "epoch_1" / "run" / "result.json").read_text())


def _monitors_main(args) -> dict:
    correct, missions, positives = {}, [], {}
    for folder in args.sessions:
        result = _record(folder)
        rec = result["pose_check"]
        rear = abs(float(result["arguments"]["rear_axle_offset_m"]))
        if not math.isclose(rear, REAR_OFFSET_M, abs_tol=1e-6):
            raise SystemExit(f"{folder}: rear offset {rear}, this tool's {REAR_OFFSET_M}")
        samples = [s for s in rec["samples"] if s.get("o") is not None]
        err = [(s["t"], math.hypot(s["applied"][0] - s["truth"][0], s["applied"][1] - s["truth"][1]),
                abs(math.atan2(math.sin(s["applied"][2] - s["truth"][2]), math.cos(s["applied"][2] - s["truth"][2]))))
               for s in samples]
        wrong = next((t for t, m, rad in err if m > WRONG[0] or rad > WRONG[1]), None)
        ren = renewals_of(rec)
        ser = series(samples, ren) if ren else None
        if wrong is not None:  # a positive by the truth, not by the contact rule
            positives[folder.name] = (ser, wrong)
        elif ser is not None:
            correct[folder.name] = ser
            probe = json.loads((folder / "probe.json").read_text())
            if ((probe.get("reverse", {}).get("mission") or {}).get("result") or {}).get("outcome") == "done":
                missions.append(folder.name)
    return evaluate(correct, missions, positives)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("mode", choices=("monitors", "stands"))
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--grid", type=Path)
    ap.add_argument("--e-slam", default="0.109,0.0137")
    args = ap.parse_args(argv)
    if args.mode == "monitors":
        out = _monitors_main(args)
    else:
        import pose_runtime as PR
        import reloc_phase

        from forklift_core.localization.slam_map import Grid

        cfg, reloc_cfg, _ = reloc_phase.frozen_settings(ROOT)
        ctx = PR.JobContext(Grid.from_npz(args.grid), cfg, reloc_cfg, PR.SHADOW_BOUNDS)
        e_slam = tuple(float(v) for v in args.e_slam.split(","))
        by, skipped = {}, 0
        for folder in args.sessions:
            result = _record(folder)
            rec = result["pose_check"]
            skipped += int(rec.get("stand_sets_skipped") or 0)
            npz = folder / "epoch_1" / "run" / "pose_check_stand_scans.npz"
            if rec.get("stand_sets") and not npz.exists():
                skipped += len(rec["stand_sets"])
            elif rec.get("stand_sets"):
                est = tuple(result["start_reloc"]["estimate"])
                by[folder.name] = rematch_stands(rec, np.load(npz), ctx, b=est, o0=est, e_slam=e_slam)
                print(folder.name, summary({folder.name: by[folder.name]})["outcomes"], flush=True)
        out = {"summary": summary(by, skipped), "e_slam": e_slam, "rows": by}
    args.out.write_text(json.dumps(out, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
