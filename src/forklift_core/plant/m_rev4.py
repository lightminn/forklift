"""M1-M3 registration revision 4 (docs/plans/2026-11-15-m1-m3-registration.md 11): the truck's own sensors and a tape
measure, values in cm and degrees, L/H bands.

Per trial (``trial_values``): from the odometry between the start and end stops, the gyro's heading change over the
same stops and, for M1 and the M3 merge, the tape row of that trial. Across the day (``summary``): medians per value,
the trials over H named, and the M2 left/right gain ratio per q step. No truth budget, k limit, bias slope or sigma
lower bound enters these values (11.1); the older values in the same output stay as information."""

from __future__ import annotations

import math
import statistics

REVISION = "개정 4 (11.3)"
TAPE_SIGMA_M = 0.02  # 11.2: the tape line's own uncertainty, about 2 cm
GATES_NOT_APPLIED = ("truth_budget", "k_limit", "bias_slope", "sigma_lower")

# 11.3: (L, H) -- below L good, at or over H needs a measure, between them watch. On |value|.
BANDS = {
    ("m1", "distance_error"): (0.03, 0.06),
    ("m1", "end_lateral_cm"): (5.0, 10.0),
    ("m1", "heading_drift_deg"): (1.0, 2.0),
    ("m2", "lr_gain_ratio"): (0.10, 0.20),  # on |r - 1|
    ("m3_merge", "end_lateral_cm"): (5.0, 10.0),
    ("m3_merge", "heading_diff_deg"): (1.0, 2.0),
}


def band(kind: str, name: str, value) -> str | None:
    if value is None:
        return None
    lo, hi = BANDS[(kind, name)]
    a = abs(value)
    return "good" if a < lo else "action" if a >= hi else "watch"


def _v(kind, name, value, why=None, **extra) -> dict:
    out = dict(value=value, band=band(kind, name, value), **extra)
    if value is None:
        out["why"] = [why]
    return out


def gyro_covers(windows: dict | None, move_ns: int, last_move_ns: int) -> bool:
    """The gyro heading runs from the end of the standing window before to the start of the one after; it is the
    same stop-to-stop change as the odometry only if that stretch holds every wheel movement of the trial."""
    if not windows or "before" not in windows or "after" not in windows:
        return False
    return windows["before"][1] <= move_ns and windows["after"][0] >= last_move_ns


def left_point(rel: dict, half_track_m: float) -> tuple[float, float]:
    """Where the left rear axle end went, seen from its own start position and the start heading (the tape line,
    11.2): with c the rear axle centre and p = (0, w/2) the lever to the left axle end in the body frame, the end point
    seen from the start point in the start frame is R(-th0)(c1 - c0) + (R(dth) - I) p -- the rear axle centre's
    relative pose plus the lever's rotation. The tape line through the left front and rear axle ends is taken
    parallel to the body axis; its own direction error is the tape method's uncertainty (about 2 cm at 3 m)."""
    dth = rel["dyaw"]
    return (rel["dx_fwd"] - math.sin(dth) * half_track_m, rel["dy_lat"] + (math.cos(dth) - 1.0) * half_track_m)


def trial_values(kind: str, *, rel_odo: dict, s_enc_m: float, gyro_dyaw_rad, tape: dict | None, tape_why: str,
                 half_track_m: float, kappa_q=None, gyro_why: str = "no_gyro") -> dict:
    """The revision-4 values of one valid trial. ``tape`` is ``{"s_cm", "y_cm"}`` or None (``tape_why`` says why)."""
    if kind == "m2":
        ok = gyro_dyaw_rad is not None and kappa_q not in (None, 0) and s_enc_m > 0
        g = abs(gyro_dyaw_rad) / s_enc_m / abs(kappa_q) if ok else None
        out = dict(value=g, report_only=True)
        if g is None:
            out["why"] = ["no_gyro_or_kappa"]
        return dict(gain=out)
    if kind not in ("m1", "m3_merge"):
        return {}
    s_pt, y_pt = left_point(rel_odo, half_track_m)
    heading_name = "heading_drift_deg" if kind == "m1" else "heading_diff_deg"
    dh = None if gyro_dyaw_rad is None else math.degrees(rel_odo["dyaw"] - gyro_dyaw_rad)
    out = {heading_name: _v(kind, heading_name, dh, gyro_why, odo_dyaw_deg=math.degrees(rel_odo["dyaw"]),
                            gyro_dyaw_deg=None if gyro_dyaw_rad is None else math.degrees(gyro_dyaw_rad))}
    if tape is None:
        out["end_lateral_cm"] = _v(kind, "end_lateral_cm", None, tape_why, y_odo_cm=100 * y_pt)
        if kind == "m1":
            out["distance_error"] = _v(kind, "distance_error", None, tape_why, s_enc_m=s_enc_m)
        return out
    s_tape, y_tape = tape["s_cm"] / 100.0, tape["y_cm"] / 100.0
    out["end_lateral_cm"] = _v(kind, "end_lateral_cm", 100 * (y_pt - y_tape), y_odo_cm=100 * y_pt,
                               y_tape_cm=tape["y_cm"], s_odo_cm=100 * s_pt)
    if kind == "m1":
        out["distance_error"] = _v(kind, "distance_error", (s_enc_m - s_tape) / s_tape, s_enc_m=s_enc_m,
                                   s_tape_m=s_tape)
    return out


def _side_key(name: str) -> tuple[str, str] | None:
    parts = name.split("_")
    for side in ("left", "right"):
        if side in parts:
            return "_".join(p for p in parts if p != side), side
    return None


def summary(results: list[dict]) -> dict:
    """The day's 11.3 table from several ``m_analyze/1`` outputs: per value the median over the valid trials, its
    band and the trial values at or over H; the M2 left/right gain ratio of the medians per q step."""
    vals: dict = {}
    gains: dict = {}
    voids = 0
    for res in results:
        kind = res.get("measurement")
        name = res.get("profile_name") or ""
        for t in res.get("trials", []):
            if t.get("status") != "valid":
                voids += 1
                continue
            for k, v in (t.get("rev4") or {}).items():
                if v.get("value") is None:
                    continue
                if kind == "m2" and k == "gain":
                    key = _side_key(name)
                    if key:
                        gains.setdefault(key[0], {}).setdefault(key[1], []).append(v["value"])
                elif (kind, k) in BANDS:
                    vals.setdefault(kind, {}).setdefault(k, []).append(v["value"])
    out: dict = {"schema": "m_summary/1", "revision": REVISION, "gates_not_applied": list(GATES_NOT_APPLIED),
                 "voids": voids}
    for kind, d in vals.items():
        out[kind] = {}
        for k, xs in d.items():
            med = statistics.median(xs)
            hi = BANDS[(kind, k)][1]
            out[kind][k] = dict(median=med, n=len(xs), band=band(kind, k, med),
                                over_h=[x for x in xs if abs(x) >= hi])
    ratios = {}
    for step, sides in sorted(gains.items()):
        lft, rgt = sides.get("left"), sides.get("right")
        r = statistics.median(lft) / statistics.median(rgt) if lft and rgt else None
        ratios[step] = dict(value=r, band=band("m2", "lr_gain_ratio", None if r is None else r - 1.0),
                            n_left=len(lft or ()), n_right=len(rgt or ()))
    for res in results:  # a step whose arc had no valid trial still shows, as no value
        key = _side_key(res.get("profile_name") or "") if res.get("measurement") == "m2" else None
        if key and key[0] not in ratios:
            ratios[key[0]] = dict(value=None, band=None, n_left=0, n_right=0)
    if ratios:
        out.setdefault("m2", {})["lr_gain_ratio"] = ratios
    return out
