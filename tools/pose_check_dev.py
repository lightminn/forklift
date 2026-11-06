"""The unconfirmed position check's development values from the shadow D sessions (docs/plans/2026-11-06-unconfirmed-
validation.md 3.2), read from each session's ``epoch_1/run/result.json`` (``pose_check``, the shadow's record).

- **delta_p(d), delta_theta(d)**: the odometry's relative motion against the truth's (base link) from the start check's
  scan and from each stroke's start, at every 10 Hz sample while the strokes ran, d the odometry's net displacement.
  The bound a + b d (a, b >= 0) is the one that holds 1.5 x every sample's error with the least value at
  d = D_TOTAL (3.0 m), then the smaller a. It is empirical: the distance bins say how many sessions reached each 0.25 m.
- **E_STILL**: every stand's largest move of the applied pose by the correction (base or rear axle), 1.5 x the largest
  -- flagged, never cut, over 0.20 m / 0.05 rad.
- **E_SLAM**: the raw agreements of evidence that passed the conservative check: a confirmed start's local r, and a
  generation's g with ``residual_confirmed`` that the operating path would have applied (stood since its scan, the same
  epoch) -- never the provisional outcome, whose meaning changes with the phase and delta. Start and re-match apart,
  1.5 x the larger's largest per component. Evidence outside the success tolerance stays in and is counted (design
  1.5's premise checked); wrong evidence is flagged: a twin's g would give metres.
- The sample check: at least --min-samples (20) of each kind, else flagged (plan 1.1: 10 more D sessions, at most 60).

Writes the values with their basis and the inputs' sha256; --pose-check-bounds reads the same file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

D_TOTAL = 3.0
BIN_M = 0.25
FACTOR = 1.5
E_STILL_MAX = (0.20, 0.05)  # design 3: the integrity budget E_STILL stays within
ORIGIN_MAX_S = 0.25  # the 10 Hz samples' gap, with room: an older one is no origin
INT_GATE = (0.20, 0.05)  # design 1.5: the monitor's gate, as written today
STILL = (0.002, 0.002)  # pose_check.STILL_M, STILL_RAD: a re-match result is discarded past these
DISTANCE_BINS = ((0, 1), (1, 2), (2, 5), (5, 10), (10, 20), (20, 50), (50, 10000))
STAND_SKIP_S = 0.5  # a stand's start left out (the truck still settling)
STAND_MIN_S = 2.0  # the shortest stand a heading rate is read over


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _invert(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def fit_bound(points) -> tuple:
    """(a, b) >= 0 with a + b d >= FACTOR e at every (d, e), least at d = D_TOTAL, then the smaller a: the line along
    an upper hull edge, the constant, or the line through the origin -- whichever holds and is least."""
    pts = [(float(d), FACTOR * float(e)) for d, e in points]
    if not pts:
        return (0.0, 0.0)
    cands = [(max(y for _, y in pts), 0.0)]
    if any(d > 0 for d, _ in pts) and all(y <= 0 or d > 0 for d, y in pts):
        cands.append((0.0, max((y / d for d, y in pts if d > 0), default=0.0)))
    hull = []  # upper hull, by d
    for p in sorted(set(pts)):
        while len(hull) >= 2 and ((hull[-1][0] - hull[-2][0]) * (p[1] - hull[-2][1])
                                  - (hull[-1][1] - hull[-2][1]) * (p[0] - hull[-2][0])) >= 0:
            hull.pop()
        hull.append(p)
    for (d0, y0), (d1, y1) in zip(hull, hull[1:]):
        if d1 > d0:
            b = (y1 - y0) / (d1 - d0)
            cands.append((y0 - b * d0, b))
    ok = [(a, b) for a, b in cands if a >= -1e-12 and b >= -1e-12
          and all(a + b * d >= y - 1e-12 for d, y in pts)]
    a, b = min(ok, key=lambda ab: (round(ab[0] + D_TOTAL * ab[1], 12), ab[0]))
    return (max(a, 0.0), max(b, 0.0))


def _origin(samples, t0):
    """The last sample at the boundary (the truck stands there), never one after it (Codex 2d P2-5)."""
    before = [s for s in samples if s["t"] <= t0 + 1e-9 and s.get("o") is not None]
    return before[-1] if before and t0 - before[-1]["t"] <= ORIGIN_MAX_S else None


def _errors(samples, t0, t1, missing=None) -> list:
    o0 = _origin(samples, t0)
    if o0 is None:
        if missing is not None:
            missing.append(t0)
        return []
    out = []
    inv_o, inv_t = _invert(o0["o"]), _invert(o0["truth"])
    for s in samples:
        if s is o0 or not (t0 < s["t"] <= t1 + 1e-9) or s.get("o") is None:
            continue
        ro, rt = _compose(inv_o, s["o"]), _compose(inv_t, s["truth"])
        d = math.hypot(s["o"][0] - o0["o"][0], s["o"][1] - o0["o"][1])
        out.append((d, math.hypot(ro[0] - rt[0], ro[1] - rt[1]), abs(_wrap(ro[2] - rt[2]))))
    return out


def _contacts(rec) -> list:
    """(t0, t1) of the recorded contacts, a floor decal's left out (records made before pose_contacts knew them)."""
    return [(i["t0"], i["t1"]) for i in (rec.get("contacts") or {}).get("intervals") or []
            if "SM_FloorDecal" not in i.get("other", "")]


def odometry_errors(rec, missing=None, touched=None) -> tuple:
    """((d, e_p, e_theta) from the start check's scan to the last stroke's end, the same from each stroke's open to
    its end) -- the meter's strokes in the shadow, the check's own when on. ``missing``: the windows without an origin.
    A window overlapping a recorded contact is left out and put in ``touched``: wheels slipping against something are
    not a straight reverse's odometry (D session val3_016: the forks on a scene pallet from the start)."""
    contacts = _contacts(rec)

    def clear(t0, t1) -> bool:
        if any(a <= t1 and b >= t0 for a, b in contacts):
            if touched is not None:
                touched.append(t0)
            return False
        return True

    events = rec.get("meter_events") or rec.get("events") or []
    samples = rec.get("samples") or []
    opens = [e["t"] for e in events if e["kind"] == "stroke_open"]
    dones = [e["t"] for e in events if e["kind"] == "stroke_done"]
    start_t = (rec.get("start") or {}).get("scan_t")
    if not dones or start_t is None:
        return [], []
    from_start = _errors(samples, start_t, dones[-1], missing) if clear(start_t, dones[-1]) else []
    from_strokes = []
    for t0 in opens:
        t1 = next((t for t in dones if t > t0), None)
        if t1 is not None and clear(t0, t1):
            from_strokes += _errors(samples, t0, t1, missing)
    return from_start, from_strokes


def _agreements(rec):
    raw = rec.get("raw") or {}
    start = raw.get("start") or {}
    starts = [start] if start.get("confirmed") and start.get("agree_m") is not None else []
    every = [g for g in raw.get("rematches") or [] if g.get("g") is not None and g.get("agree_m") is not None]
    passed = [g for g in every if g.get("residual_confirmed")]
    contacts = _contacts(rec)
    # scanned in contact: the agreement is the wheels' slip (val3_016, 0.34 m with g 1 cm from the truth)
    free = [g for g in passed if g.get("scan_t") is None or not any(a <= g["scan_t"] <= b for a, b in contacts)]
    return starts, [g for g in free if g.get("applicable")], every, len(free), len(passed) - len(free)


def dev_values(records, *, min_samples: int = 20) -> dict:
    """``records``: (name, the session's pose_check record) of every D session."""
    errs, bins = [], {f"{k * BIN_M:.2f}": 0 for k in range(int(D_TOTAL / BIN_M) + 1)}
    stands, starts, gens, all_g, passed = [], [], [], 0, 0
    missing: list = []
    touched: list = []
    in_contact = 0
    closes: list = []
    for _, rec in records:
        a_, b_ = odometry_errors(rec, missing, touched)
        errs += a_ + b_
        closes += [e.get("s_m", 0.0) for e in rec.get("meter_events") or [] if e["kind"] == "stroke_close"]
        reach = max((d for d, _, _ in a_), default=0.0)
        for key in bins:
            if reach >= float(key) - 1e-9:
                bins[key] += 1
        stands += (rec.get("raw") or {}).get("stands") or []
        s_, g_, every, n_passed, n_contact = _agreements(rec)
        in_contact += n_contact
        starts += s_
        gens += g_
        all_g += len(every)
        passed += n_passed
    in_reach = [e for e in errs if e[0] <= D_TOTAL + 1e-9]
    a_p, b_p = fit_bound([(d, e) for d, e, _ in in_reach])
    a_t, b_t = fit_bound([(d, e) for d, _, e in in_reach])
    plain = (max((s["max_m"] for s in stands), default=0.0), max((s["max_rad"] for s in stands), default=0.0))
    # the reference taken again at a hold's release (the revision plan 9.4); records before it hold the plain value
    still = (max((s.get("rebased_m", s["max_m"]) for s in stands), default=0.0),
             max((s.get("rebased_rad", s["max_rad"]) for s in stands), default=0.0))
    e_still = [FACTOR * still[0], FACTOR * still[1]]

    def largest(rows):
        return (max((r["agree_m"] for r in rows), default=0.0), max((r["agree_rad"] for r in rows), default=0.0))

    s_max, g_max = largest(starts), largest(gens)
    e_slam = [FACTOR * max(s_max[0], g_max[0]), FACTOR * max(s_max[1], g_max[1])]
    classes = [((r.get("truth") or {}).get("class") or "unknown") for r in starts + gens]
    flags = []
    if e_still[0] > E_STILL_MAX[0] or e_still[1] > E_STILL_MAX[1]:
        flags.append("e_still_over_budget")
    if "wrong" in classes:
        flags.append("wrong_evidence_in_samples")
    if missing:
        flags.append("windows_without_origin")
    if len(starts) < min_samples:
        flags.append(f"start_samples_under_{min_samples}")
    if len(gens) < min_samples:
        flags.append(f"rematch_samples_under_{min_samples}")
    return {
        "odom": [a_p, b_p, a_t, b_t], "e_still": e_still, "e_slam": e_slam, "flags": flags,
        "basis": {
            "factor": FACTOR, "sessions": len(records),
            "odom": {"samples": len(in_reach), "beyond_d_total": len(errs) - len(in_reach), "bins": bins,
                     "windows_without_origin": len(missing), "windows_in_contact": len(touched),
                     "strokes": {"closed": len(closes), "under_0_05_m": sum(s < 0.05 for s in closes),
                                 "under_0_5_m": sum(s < 0.5 for s in closes)},
                     "max_e_p": max((e for _, e, _ in in_reach), default=0.0),
                     "max_e_theta": max((e for _, _, e in in_reach), default=0.0)},
            "e_still": {"stands": len(stands), "max": list(still), "plain_max": list(plain),
                        "plain_x_factor": [FACTOR * plain[0], FACTOR * plain[1]]},
            "e_slam": {"start": {"n": len(starts), "max": list(s_max)},
                       "rematch": {"n": len(gens), "max": list(g_max), "all_g_n": all_g,
                                   "not_applicable": passed - len(gens), "in_contact": in_contact},
                       "outside_success": sum(c != "success" for c in classes),
                       "unknown": sum(c == "unknown" for c in classes)}}}


def delta_rows(rec) -> list:
    """Each sample with the monitor's Delta_int: the odometry path since the reference C_int was last set (an agreement
    that held) and the applied pose's error against the truth, which Delta_int is not."""
    renewals = sorted(e["t"] for e in rec.get("events") or []
                      if e["kind"] in ("slam_agreement", "rematch_agreement") and e.get("ok"))
    rows, since, last, next_ = [], 0.0, None, 0
    for s in rec.get("samples") or []:
        o = s.get("o")
        if o is None:
            continue
        if last is not None:
            since += math.hypot(o[0] - last[0], o[1] - last[1])
        last = o
        while next_ < len(renewals) and renewals[next_] <= s["t"]:
            since, next_ = 0.0, next_ + 1
        if s.get("ref") != "int" or s.get("delta") is None:
            continue
        rows.append({"t": s["t"], "since_m": since, "delta_m": s["delta"][0], "delta_rad": s["delta"][1],
                     "truth_m": math.hypot(s["applied"][0] - s["truth"][0], s["applied"][1] - s["truth"][1]),
                     "truth_rad": abs(_wrap(s["applied"][2] - s["truth"][2]))})
    return rows


def _standing_rates(rec) -> list:
    """The odometry's heading rate over each stand long enough to read (rad/s of simulated time)."""
    out = []
    samples = rec.get("samples") or []
    for st in (rec.get("raw") or {}).get("stands") or []:
        w = [s for s in samples if st["t0"] + STAND_SKIP_S <= s["t"] <= st["t1"] and s.get("o") is not None]
        if len(w) < 2 or w[-1]["t"] - w[0]["t"] < STAND_MIN_S:
            continue
        out.append(_wrap(w[-1]["o"][2] - w[0]["o"][2]) / (w[-1]["t"] - w[0]["t"]))
    return out


def design_evidence(records) -> dict:
    """What the design revision reads (plan 6 step 3; shadow sessions, the provisional verdicts not read): Delta_int
    against the distance since its reference with the truth error beside it, where each session first crossed the
    monitor's gate and how wrong it was then, and the standing heading drift with what it does to a re-match."""
    bins = {f"{a}-{b}": {"sessions": 0, "samples": 0, "delta_m_max": 0.0, "delta_rad_max": 0.0, "truth_m_max": 0.0,
                         "truth_rad_max": 0.0} for a, b in DISTANCE_BINS}
    crossings, rates, generations, moved_over = [], [], 0, 0
    for name, rec in records:
        rows, seen = delta_rows(rec), set()
        for r in rows:
            for a, b in DISTANCE_BINS:
                if a <= r["since_m"] < b:
                    row = bins[f"{a}-{b}"]
                    if (a, b) not in seen:
                        seen.add((a, b))
                        row["sessions"] += 1
                    row["samples"] += 1
                    row["delta_m_max"] = max(row["delta_m_max"], r["delta_m"])
                    row["delta_rad_max"] = max(row["delta_rad_max"], r["delta_rad"])
                    row["truth_m_max"] = max(row["truth_m_max"], r["truth_m"])
                    row["truth_rad_max"] = max(row["truth_rad_max"], r["truth_rad"])
        first = next((r for r in rows if r["delta_m"] > INT_GATE[0] or r["delta_rad"] > INT_GATE[1]), None)
        if first is not None:
            crossings.append({"session": name, **first, "gate": "m" if first["delta_m"] > INT_GATE[0] else "rad"})
        rates += _standing_rates(rec)
        for g in (rec.get("raw") or {}).get("rematches") or []:
            if g.get("g") is not None and g.get("moved") is not None:
                generations += 1
                moved_over += g["moved"][0] > STILL[0] or g["moved"][1] > STILL[1]
    mags = sorted(abs(r) for r in rates)
    return {"delta_by_distance": bins, "sessions_crossing": len(crossings), "sessions": len(records),
            "crossings": crossings,
            "standing_drift": {"stands": len(mags), "rate_p50": mags[len(mags) // 2] if mags else 0.0,
                               "rate_max": mags[-1] if mags else 0.0,
                               "over_still_in_1_5_s": sum(r * 1.5 > STILL[1] for r in mags),
                               "generations": generations, "moved_over_still": int(moved_over)}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sessions", nargs="+", type=Path, help="the D sessions' folders (each with epoch_1/run/result.json)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--min-samples", type=int, default=20)
    ap.add_argument("--ignore-contacts", action="store_true",
                    help="the values without the post-hoc contact rule, for the record beside the rule's")
    args = ap.parse_args(argv)
    records, inputs = [], []
    for folder in args.sessions:
        path = folder / "epoch_1" / "run" / "result.json"
        data = path.read_bytes()
        rec = json.loads(data).get("pose_check")
        if rec is None or rec.get("mode") != "shadow":
            print(f"{folder}: no shadow record", file=sys.stderr)
            return 2
        if args.ignore_contacts:
            rec = {**rec, "contacts": None}
        records.append((folder.name, rec))
        inputs.append({"session": folder.name, "sha256": hashlib.sha256(data).hexdigest()})
    out = dev_values(records, min_samples=args.min_samples)
    out["inputs"] = inputs
    out["design"] = design_evidence(records)
    args.out.write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps({k: out[k] for k in ("odom", "e_still", "e_slam", "flags")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
