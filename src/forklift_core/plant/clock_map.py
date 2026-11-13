"""Device clock to host monotonic clock (docs/design/2026-11-14-teleop-record-state-markers-clock.md 5, G7).

Model ``host = a + b * device + d`` with ``d >= 0`` (a transport and processing delay is never negative). ``a`` and
``b`` are the line under every point with the smallest sum of residuals -- the lower convex hull's edge that is highest
at the mean device time. ``to_host_ns`` is therefore the receipt time estimate at the low edge of the delay; the event
itself is earlier by an unknown minimum delay, and a delay that trends over the record is absorbed into the slope
(drift). The residuals above the envelope are the observed delay variation, not an alignment error; the halves'
figures show how far the fit can be trusted: ``drift_split_ppm`` (the halves disagree), ``drift_vs_halves_ppm`` (the
whole disagrees with both halves -- a delay step taken as drift) and ``split_end_ns``. A linear delay trend gives the
same wrong slope to the whole and both halves, so all three stay near zero: small figures never guarantee the
alignment. Twenty points and five seconds
only make a fit computable.

Device times are µs (MCU ``mcu_time_us``; an IMU's device clock converted to µs), host times ns.
"""

from __future__ import annotations

from dataclasses import dataclass, field

MIN_POINTS = 20
MIN_SPAN_S = 5.0
# the halves' figures need each half to hold this much; otherwise they are None with ``split_reason``
MIN_HALF_POINTS = 10
MIN_HALF_SPAN_S = 2.5


def clean_points(device_us, host_ns) -> tuple[list[tuple[int, int]], dict]:
    """Points kept in order and the drops by kind: a device time not above the last kept one (``device_repeat`` when
    equal -- the earliest host time stays, ``device_back`` when lower), a host time below the last kept one
    (``host_back``)."""
    kept: list[tuple[int, int]] = []
    dropped = {"device_back": 0, "host_back": 0, "device_repeat": 0}
    for d, h in zip(device_us, host_ns):
        d, h = int(d), int(h)
        if kept:
            ld, lh = kept[-1]
            if d < ld:
                dropped["device_back"] += 1
                continue
            if d == ld:
                dropped["device_repeat"] += 1
                if h < lh:
                    kept[-1] = (d, h)
                continue
            if h < lh:
                dropped["host_back"] += 1
                continue
        kept.append((d, h))
    return kept, dropped


def _lower_hull(points: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Andrew's monotone chain, lower part; points sorted by device time with distinct device times. Collinear middle
    points are dropped, so collinear data gives one edge (a deterministic envelope)."""
    hull: list[tuple[int, int]] = []
    for p in points:
        while len(hull) >= 2:
            (x1, y1), (x2, y2) = hull[-2], hull[-1]
            if (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1) <= 0:
                hull.pop()
            else:
                break
        hull.append(p)
    return hull


def _envelope(points: list[tuple[int, int]]) -> tuple[float, float]:
    """(offset_ns, slope ns/µs) of the lower hull edge highest at the mean device time (the minimum residual sum)."""
    hull = _lower_hull(points)
    if len(hull) < 2:
        raise ValueError("an envelope needs two distinct device times")
    xbar = sum(p[0] for p in points) / len(points)
    best = None
    for (x1, y1), (x2, y2) in zip(hull, hull[1:]):
        b = (y2 - y1) / (x2 - x1)
        a = y1 - b * x1
        v = a + b * xbar
        if best is None or v > best[0]:  # strictly higher: the first (left) edge wins a tie
            best = (v, a, b)
    return best[1], best[2]


def _drift_ppm(slope: float) -> float:
    return (slope / 1000.0 - 1.0) * 1e6


def _quantile(sorted_vals: list[float], q: float) -> float:
    if not sorted_vals:
        return float("nan")
    i = min(len(sorted_vals) - 1, max(0, int(round(q * (len(sorted_vals) - 1)))))
    return sorted_vals[i]


@dataclass(frozen=True)
class ClockFit:
    valid: bool
    reason: str | None
    n: int
    span_s: float
    offset_ns: float | None
    slope: float | None  # host ns per device µs (1000 at no drift)
    drift_ppm: float | None
    drift_split_ppm: float | None  # second half's drift minus the first half's
    drift_vs_halves_ppm: float | None  # the larger |whole drift - a half's drift|: a delay step taken as drift
    split_end_ns: float | None  # the whole fit at the last device time minus the second half's fit there
    split_reason: str | None = None  # why the halves' figures are None (a valid fit may still have them None)
    residuals_ns: tuple = ()
    delay_ns: dict = field(default_factory=dict)  # residuals above the envelope: p50, p95, max
    dropped: dict = field(default_factory=dict)
    device_range_us: tuple | None = None
    host_range_ns: tuple | None = None
    host_boot: str | None = None
    boot_id: int | None = None

    def to_host_ns(self, device_us: int) -> tuple[float, bool]:
        """(host ns at the delay envelope, extrapolated) -- outside the fitted device range is flagged, never
        refused. An invalid fit raises."""
        if not self.valid:
            raise ValueError(f"invalid clock fit: {self.reason}")
        lo, hi = self.device_range_us
        return self.offset_ns + self.slope * device_us, not (lo <= device_us <= hi)

    def summary(self) -> dict:
        return {k: getattr(self, k) for k in ("valid", "reason", "n", "span_s", "offset_ns", "slope", "drift_ppm",
                                              "drift_split_ppm", "drift_vs_halves_ppm", "split_end_ns", "split_reason", "delay_ns", "dropped",
                                              "device_range_us", "host_range_ns", "host_boot", "boot_id")}


def fit_clock(device_us, host_ns, *, host_boot: str | None = None, boot_id: int | None = None) -> ClockFit:
    points, dropped = clean_points(device_us, host_ns)
    n = len(points)
    span_s = (points[-1][0] - points[0][0]) / 1e6 if n >= 2 else 0.0
    base = dict(n=n, span_s=span_s, dropped=dropped, host_boot=host_boot, boot_id=boot_id,
                device_range_us=(points[0][0], points[-1][0]) if points else None,
                host_range_ns=(points[0][1], points[-1][1]) if points else None)
    none = dict(offset_ns=None, slope=None, drift_ppm=None, drift_split_ppm=None, drift_vs_halves_ppm=None,
                split_end_ns=None)
    if n < MIN_POINTS:
        return ClockFit(False, f"{n} points after clean-up (< {MIN_POINTS})", **none, **base)
    if span_s < MIN_SPAN_S:
        return ClockFit(False, f"span {span_s:.3f} s after clean-up (< {MIN_SPAN_S} s)", **none, **base)
    a, b = _envelope(points)
    res = [h - (a + b * d) for d, h in points]
    srt = sorted(res)
    half = n // 2
    first, second = points[:half], points[half:]
    split = dict(drift_split_ppm=None, drift_vs_halves_ppm=None, split_end_ns=None, split_reason=None)
    short = [name for name, part in (("first", first), ("second", second))
             if len(part) < MIN_HALF_POINTS or (part[-1][0] - part[0][0]) / 1e6 < MIN_HALF_SPAN_S]
    if short:
        split["split_reason"] = (f"{' and '.join(short)} half under {MIN_HALF_POINTS} points or "
                                 f"{MIN_HALF_SPAN_S} s")
    else:
        a1, b1 = _envelope(first)
        a2, b2 = _envelope(second)
        x_end = points[-1][0]
        split.update(drift_split_ppm=_drift_ppm(b2) - _drift_ppm(b1),
                     drift_vs_halves_ppm=max(abs(_drift_ppm(b) - _drift_ppm(b1)), abs(_drift_ppm(b) - _drift_ppm(b2))),
                     split_end_ns=(a + b * x_end) - (a2 + b2 * x_end))
    return ClockFit(True, None, offset_ns=a, slope=b, drift_ppm=_drift_ppm(b), residuals_ns=tuple(res),
                    delay_ns={"p50": _quantile(srt, 0.5), "p95": _quantile(srt, 0.95), "max": srt[-1]}, **split,
                    **base)


def clock_fits_from_rows(rows, *, time_key: str = "read_ns") -> list[ClockFit]:
    """One fit per (host boot, MCU boot) run of state rows, in record order. A header row names the host boot
    (``host_boot_id``); a state row's host time is ``time_key`` (the teleop record's read time) or, without it,
    ``host_ns`` (a bringup record). Other rows are ignored."""
    groups: list[tuple[tuple, list, list]] = []
    host_boot = None
    for r in rows:
        t = r.get("t")
        if t == "header":
            host_boot = r.get("host_boot_id")
            continue
        if t != "state":
            continue
        key = (host_boot, r.get("boot_id"))
        h = r.get(time_key, r.get("host_ns"))
        if h is None:
            continue
        if not groups or groups[-1][0] != key:
            groups.append((key, [], []))
        groups[-1][1].append(r["mcu_time_us"])
        groups[-1][2].append(h)
    return [fit_clock(d, h, host_boot=k[0], boot_id=k[1]) for k, d, h in groups]
