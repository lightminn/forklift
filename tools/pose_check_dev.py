"""The unconfirmed position check's development values from the shadow D sessions (docs/plans/2026-11-06-unconfirmed-
validation.md 3.2), read from each session's ``epoch_1/run/result.json`` (``pose_check``, the shadow's record).

- **delta_p(d), delta_theta(d)**: the odometry's relative motion against the truth's (base link) from the start check's
  scan and from each stroke's start, at every 10 Hz sample while the strokes ran, d the odometry's net displacement.
  The bound a + b d (a, b >= 0) is the one that holds 1.5 x every sample's error with the least value at
  d = D_TOTAL (3.0 m), then the smaller a. It is empirical: the distance bins say how many sessions reached each 0.25 m.
- **E_STILL**: every stand's largest move of the applied pose by the correction (base or rear axle), 1.5 x the largest
  -- flagged, never cut, over 0.20 m / 0.05 rad.
- **E_SLAM**: the raw agreements the operating path would check -- a confirmed start's local r, a generation whose
  conservative outcome confirmed its g -- start and re-match apart, 1.5 x the larger's largest per component. Evidence
  outside the success tolerance stays in and is counted (design 1.5's premise checked).
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
    before = [s for s in samples if s["t"] <= t0 + 1e-9 and s.get("o") is not None]
    if before:
        return before[-1]
    after = [s for s in samples if s.get("o") is not None]
    return after[0] if after else None


def _errors(samples, t0, t1) -> list:
    o0 = _origin(samples, t0)
    if o0 is None:
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


def odometry_errors(rec) -> tuple:
    """((d, e_p, e_theta) from the start check's scan to the last stroke's end, the same from each stroke's open to
    its end) -- the meter's strokes in the shadow, the check's own when on."""
    events = rec.get("meter_events") or rec.get("events") or []
    samples = rec.get("samples") or []
    opens = [e["t"] for e in events if e["kind"] == "stroke_open"]
    dones = [e["t"] for e in events if e["kind"] == "stroke_done"]
    start_t = (rec.get("start") or {}).get("scan_t")
    if not dones or start_t is None:
        return [], []
    from_start = _errors(samples, start_t, dones[-1])
    from_strokes = []
    for t0 in opens:
        t1 = next((t for t in dones if t > t0), None)
        if t1 is not None:
            from_strokes += _errors(samples, t0, t1)
    return from_start, from_strokes


def _agreements(rec):
    raw = rec.get("raw") or {}
    start = raw.get("start") or {}
    starts = [start] if start.get("confirmed") and start.get("agree_m") is not None else []
    gens = [g for g in raw.get("rematches") or [] if g.get("g") is not None and g.get("agree_m") is not None]
    return starts, [g for g in gens if g.get("outcome") == "confirm"], gens


def dev_values(records, *, min_samples: int = 20) -> dict:
    """``records``: (name, the session's pose_check record) of every D session."""
    errs, bins = [], {f"{k * BIN_M:.2f}": 0 for k in range(int(D_TOTAL / BIN_M) + 1)}
    stands, starts, gens, all_g = [], [], [], 0
    for _, rec in records:
        a_, b_ = odometry_errors(rec)
        errs += a_ + b_
        reach = max((d for d, _, _ in a_), default=0.0)
        for key in bins:
            if reach >= float(key) - 1e-9:
                bins[key] += 1
        stands += (rec.get("raw") or {}).get("stands") or []
        s_, g_, every = _agreements(rec)
        starts += s_
        gens += g_
        all_g += len(every)
    in_reach = [e for e in errs if e[0] <= D_TOTAL + 1e-9]
    a_p, b_p = fit_bound([(d, e) for d, e, _ in in_reach])
    a_t, b_t = fit_bound([(d, e) for d, _, e in in_reach])
    still = (max((s["max_m"] for s in stands), default=0.0), max((s["max_rad"] for s in stands), default=0.0))
    e_still = [FACTOR * still[0], FACTOR * still[1]]

    def largest(rows):
        return (max((r["agree_m"] for r in rows), default=0.0), max((r["agree_rad"] for r in rows), default=0.0))

    s_max, g_max = largest(starts), largest(gens)
    e_slam = [FACTOR * max(s_max[0], g_max[0]), FACTOR * max(s_max[1], g_max[1])]
    classes = [((r.get("truth") or {}).get("class") or "unknown") for r in starts + gens]
    flags = []
    if e_still[0] > E_STILL_MAX[0] or e_still[1] > E_STILL_MAX[1]:
        flags.append("e_still_over_budget")
    if len(starts) < min_samples:
        flags.append(f"start_samples_under_{min_samples}")
    if len(gens) < min_samples:
        flags.append(f"rematch_samples_under_{min_samples}")
    return {
        "odom": [a_p, b_p, a_t, b_t], "e_still": e_still, "e_slam": e_slam, "flags": flags,
        "basis": {
            "factor": FACTOR, "sessions": len(records),
            "odom": {"samples": len(in_reach), "beyond_d_total": len(errs) - len(in_reach), "bins": bins,
                     "max_e_p": max((e for _, e, _ in in_reach), default=0.0),
                     "max_e_theta": max((e for _, _, e in in_reach), default=0.0)},
            "e_still": {"stands": len(stands), "max": list(still)},
            "e_slam": {"start": {"n": len(starts), "max": list(s_max)},
                       "rematch": {"n": len(gens), "max": list(g_max), "all_g_n": all_g},
                       "outside_success": sum(c != "success" for c in classes),
                       "unknown": sum(c == "unknown" for c in classes)}}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sessions", nargs="+", type=Path, help="the D sessions' folders (each with epoch_1/run/result.json)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--min-samples", type=int, default=20)
    args = ap.parse_args(argv)
    records, inputs = [], []
    for folder in args.sessions:
        path = folder / "epoch_1" / "run" / "result.json"
        data = path.read_bytes()
        rec = json.loads(data).get("pose_check")
        if rec is None or rec.get("mode") != "shadow":
            print(f"{folder}: no shadow record", file=sys.stderr)
            return 2
        records.append((folder.name, rec))
        inputs.append({"session": folder.name, "sha256": hashlib.sha256(data).hexdigest()})
    out = dev_values(records, min_samples=args.min_samples)
    out["inputs"] = inputs
    args.out.write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps({k: out[k] for k in ("odom", "e_still", "e_slam", "flags")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
