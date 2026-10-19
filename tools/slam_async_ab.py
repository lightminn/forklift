"""Real-time plan 5.10 (docs/plans/2026-10-17-realtime-sim.md): judge the SLAM async on/off comparison.

Usage: python tools/slam_async_ab.py MANIFEST.json [--json OUT]   # exit 0 always; the decision is in the output

The manifest lists every run in the order it was run: ``{"blocks": [0, 0, 1, 1, 0, 0], "runs": [{"tag", "arm":
"on"|"off", "set": "decision"|"general", "block": int|null, "dir": RUN_DIR, "rerun_of": tag|null}]}``. ``blocks`` is
the pre-drawn direction of each decision block (0: on off off on, 1: off on on off) and must match the arms. RUN_DIR
holds what ``live.sbatch`` wrote: ``run/result.json``, ``run/slam_records.json``, ``run/live_log.jsonl`` (any may be
missing for a failed run). Reruns are made only for the plan's external causes; the manifest lists the run that
counts, with ``rerun_of`` naming the slot's earlier attempt.

Per run (plan 5.10 "지표"): E (scan-time estimate error, warm-up excluded), S (unpermitted, forbidden contacts), P
(insertion: lateral clearance min, mean depth past centre, |true yaw|), M (outcome), T (driving-period gaps over
0.25 s and the AUTO pacing ratio, from the first command line after the first ``tracking`` scan's stamp), and the
descriptive L (async lag effect per scan) and J (correction jumps). Every rule is an exact randomisation test over
the 64 block-flip assignments under the constant-shift null (plan 5.10 "통계"); missing measures stop the automatic
decision (R0); the first matching row of the decision table is the outcome.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

GAP_S = 0.25
ALPHA = 0.05
BLOCK_ORDER = {0: ("on", "off", "off", "on"), 1: ("off", "on", "on", "off")}
# A run that ended for one of these never reached its mission's end: no T, no E (plan 5.10 "T", R0).
SESSION_FAILURES = ("slam_link_failed", "slam_prewarm_timeout")
MARGINS = {"e_p50_mm": 10.0, "e_p99_mm": 15.0, "lat_min_mm": -10.0, "beyond_mm": 10.0, "yaw_abs_rad": 0.005,
           "ratio_harm": -0.01}


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _load(path: Path):
    try:
        if path.suffix == ".jsonl":
            with path.open() as f:
                return [json.loads(line) for line in f]
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def session_failed(result) -> bool:
    if result is None:
        return True
    reason = str(result.get("failure_reason") or "")
    return reason.startswith(SESSION_FAILURES)


def estimate_error(records) -> dict | None:
    """E and L: the correction the scan was processed with, composed with its odometry, against the truth."""
    if not records:
        return None
    e, lag, jumps, previous = [], [], [], None
    for r in records:
        if r.get("mode") == "warming_up":
            continue
        if r.get("link") == "async":
            used, after = r.get("scan_correction"), r.get("applied_after")
        else:
            used = after = r.get("applied_map_from_odom")
        if used is None:
            continue
        truth = r["truth_base"]

        def err(c):
            p = _compose(c, r["odom_base"])
            return math.hypot(p[0] - truth[0], p[1] - truth[1]) * 1000.0

        e.append(err(used))
        if r.get("link") == "async" and after is not None:
            lag.append((err(used) - err(after), r.get("mode"), r.get("phase")))
        applied = after if after is not None else used
        if previous is not None:
            a, b = _compose(previous, r["odom_base"]), _compose(applied, r["odom_base"])
            at = r.get("applied_tick_s") if r.get("link") == "async" else None  # when control took it
            jumps.append((math.hypot(a[0] - b[0], a[1] - b[1]) * 1000.0, float(at if at is not None else r["stamp_s"])))
        previous = applied
    if not e:
        return None
    e = np.asarray(e)
    worse = [x for x in lag if x[0] > 20.0]
    return {
        "n": int(e.size), "p50_mm": float(np.percentile(e, 50)), "p99_mm": float(np.percentile(e, 99)),
        "max_mm": float(e.max()),
        "lag_over_20mm": len(worse), "lag_max_mm": max((x[0] for x in lag), default=None),
        "lag_over_20mm_modes": sorted({str(x[1]) for x in worse}),
        "lag_over_20mm_phases": sorted({str(x[2]) for x in worse}),
        "jumps_mm": jumps,
    }


def timing(records, log) -> dict | None:
    """T: driving-period gaps over 0.25 s (all causes) and the AUTO pacing ratio."""
    if not records or not log:
        return None
    t0 = next((float(r["stamp_s"]) for r in records if r.get("mode") == "tracking"), None)
    if t0 is None:
        return None
    lines = [d for d in log if "tick_t" in d and "mono" in d and not d.get("status") and not d.get("request_id")
             and "mode" in d]
    drive = [d for d in lines if float(d["tick_t"]) > t0]  # after the first tracking scan's stamp (plan 5.10)
    if len(drive) < 2:
        return None
    gaps = [b["mono"] - a["mono"] for a, b in zip(drive, drive[1:]) if b["mono"] - a["mono"] > GAP_S]
    sim = wall = 0.0
    for a, b in zip(drive, drive[1:]):
        if a.get("mode") == "AUTO" and b.get("mode") == "AUTO":
            sim += float(b["tick_t"]) - float(a["tick_t"])
            wall += float(b["mono"]) - float(a["mono"])
    first = next((d for d in lines), None)
    before = [d for d in lines if float(d["tick_t"]) <= t0] + drive[:1]
    warm_gaps = [b["mono"] - a["mono"] for a, b in zip(before, before[1:]) if b["mono"] - a["mono"] > GAP_S]
    return {
        "warmup_gaps": len(warm_gaps),
        "gaps": len(gaps), "gap_s": [round(g, 3) for g in gaps], "ratio": sim / wall if wall > 0 else None,
        "drive_wall_s": float(drive[-1]["mono"] - drive[0]["mono"]),
        "warmup_wall_s": float(drive[0]["mono"] - first["mono"]) if first else None,
        "curvature": [(float(d["tick_t"]), float(d.get("curvature_inv_m") or 0.0)) for d in drive],
    }


def insertion(result) -> dict | None:
    """P: physical insertion measures; None unless all three are there."""
    try:  # both sides of each, finite, or no P (Codex 5.10 script review P2)
        lat = result["lateral_clearance_min_m"]["insert"]
        sides = [float(lat["left"]), float(lat["right"])]
        if not all(math.isfinite(v) for v in sides):  # each side, before the min hides one (Codex round 10)
            return None
        lat_min = min(sides)
        measured = result["insertion_measured"]
        beyond = [float(measured["left"]["beyond_centre_m"]), float(measured["right"]["beyond_centre_m"])]
        yaw = float(result["insertion_truth_yaw_rad"])
    except (KeyError, TypeError, ValueError, AttributeError):
        return None
    if not all(math.isfinite(v) for v in [lat_min, yaw, *beyond]):
        return None
    try:
        tracking = 1000.0 * float((result.get("insertion_error") or {}).get("position_m"))
    except (TypeError, ValueError):
        tracking = None  # descriptive only
    return {"lat_min_mm": lat_min * 1000.0, "beyond_mm": float(np.mean(beyond)) * 1000.0,
            "yaw_abs_rad": abs(yaw), "tracking_error_mm": tracking}


def measure(run: dict) -> dict:
    d = Path(run["dir"]) / "run"
    result, records, log = _load(d / "result.json"), _load(d / "slam_records.json"), _load(d / "live_log.jsonl")
    out = {k: run.get(k) for k in ("tag", "arm", "set", "block", "seed", "rerun_of")}
    out["result"] = result is not None
    out["success"] = bool(result and result.get("success"))
    out["failure_reason"] = result.get("failure_reason") if result else "no_result"
    out["slam_async_record"] = bool(result and result.get("slam_async") is not None)
    # The arm comes from the schedule; the records must agree both ways (Codex 5.10 script review P2).
    any_async = any(r.get("link") == "async" for r in records or [])
    warmed = any(r.get("mode") == "tracking" for r in records or [])
    if run["arm"] != "on" and (out["slam_async_record"] or any_async):
        raise ValueError(f"{run['tag']}: an off run carries async SLAM records")
    if run["arm"] == "on" and warmed and not out["slam_async_record"]:
        raise ValueError(f"{run['tag']}: an on run past its warm-up has no slam_async")

    def count(container, key):  # a missing or null safety field is unknown, never 0
        value = (container or {}).get(key) if isinstance(container, dict) else None
        return len(value) if isinstance(value, list) else None

    out["unpermitted"] = count((result or {}).get("obstacle_layer"), "unpermitted")
    out["forbidden"] = count(result, "forbidden_pocket_contacts")
    ended = not session_failed(result)
    out["E"] = estimate_error(records) if ended else None
    out["T"] = timing(records, log) if ended else None
    if out["T"] is not None:  # the async arm warms up before its first command line (descriptive)
        out["T"]["prewarm_wall_s"] = ((result or {}).get("slam_async") or {}).get("prewarm_wall_s")
    out["P"] = insertion(result) if result else None
    out["J"] = jumps(out["E"], out["T"])
    return out


def jumps(e, t) -> dict | None:
    """J (descriptive): correction jumps over 20 mm and the largest curvature-command change within 0.5 s after."""
    if not e or not t:
        return None
    big = [(mm, at) for mm, at in e["jumps_mm"] if mm > 20.0]
    curv = t["curvature"]
    after = []
    for _mm, at in big:
        start = max([i for i, (tt, _k) in enumerate(curv) if tt < at] or [0])  # the command just before it
        window = [k for i, (tt, k) in enumerate(curv) if i >= start and tt <= at + 0.5]
        after.append(max((abs(b - a) for a, b in zip(window, window[1:])), default=0.0))
    every = [abs(b[1] - a[1]) for a, b in zip(curv, curv[1:])]
    return {"jumps_over_20mm": len(big), "jump_max_mm": max((mm for mm, _ in e["jumps_mm"]), default=None),
            "curvature_step_after_max": max(after, default=None),
            "curvature_step_p99": float(np.percentile(every, 99)) if every else None}


# --- the randomisation tests -------------------------------------------------------------------------------------
def assignments(blocks: list[int]):
    """Every block-flip assignment: for each, the arm of each decision run in run order."""
    for flips in itertools.product((0, 1), repeat=len(blocks)):
        arms = []
        for b, f in zip(blocks, flips):
            arms.extend(BLOCK_ORDER[b ^ f])
        yield arms


def stat(values, arms, kind):
    on = np.array([v for v, a in zip(values, arms) if a == "on"], dtype=float)
    off = np.array([v for v, a in zip(values, arms) if a == "off"], dtype=float)
    f = np.median if kind == "median" else np.mean
    return float(f(on) - f(off))


def shown(values, arms, blocks, c, side, kind):
    """p for 'diff > c' (side '>') or 'diff < c' (side '<') under the sharp null on = off + c."""
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError("a non-finite value reached a test")  # R0 keeps them out (Codex 5.10 script review P2)
    adjusted = [v - c if a == "on" else v for v, a in zip(values, arms)]
    observed = stat(adjusted, arms, kind)
    stats = [stat(adjusted, other, kind) for other in assignments(blocks)]
    eps = 1e-12
    if side == ">":
        p = sum(s >= observed - eps for s in stats) / len(stats)
    else:
        p = sum(s <= observed + eps for s in stats) / len(stats)
    return {"c": c, "side": side, "observed": stat(values, arms, kind), "p": p, "shown": p <= ALPHA}


def iqr_ratio(values, arms) -> float | None:
    """The on arm's interquartile range over the off arm's (descriptive; None when the off range is 0)."""
    def iqr(xs):
        q1, q3 = np.percentile(xs, [25, 75])
        return float(q3 - q1)
    a = iqr([v for v, x in zip(values, arms) if x == "on"])
    b = iqr([v for v, x in zip(values, arms) if x == "off"])
    return a / b if b > 0 else None


PLAN_BLOCKS = [0, 0, 1, 1, 0, 0]  # plan 5.10: numpy.random.default_rng(20261019).integers(0, 2, 6)
GENERAL = sorted([(1, "on"), (1, "off"), (2, "on"), (2, "off")])


def _finite(*xs) -> bool:
    try:
        return all(x is not None and math.isfinite(float(x)) for x in xs)
    except (TypeError, ValueError):
        return False


def judge(manifest: dict, runs: list[dict]) -> dict:
    """The plan's set of runs only (Codex 5.10 script review P2): six drawn blocks, 24 decision runs in block order,
    the four generalisation runs, every tag once."""
    blocks = list(manifest["blocks"])
    if blocks != PLAN_BLOCKS:
        raise ValueError(f"blocks {blocks} are not the plan's drawn {PLAN_BLOCKS}")
    tags = [r["tag"] for r in runs]
    if len(set(tags)) != len(tags):
        raise ValueError("a tag appears twice")
    decision = [r for r in runs if r["set"] == "decision"]
    general = [r for r in runs if r["set"] == "general"]
    if len(decision) != 24 or [r["block"] for r in decision] != [i // 4 for i in range(24)]:
        raise ValueError("the decision set is not 24 runs in block order")
    if any(r.get("seed") != 0 for r in decision):
        raise ValueError("a decision run is not seed 0")
    if sorted((r.get("seed"), r["arm"]) for r in general) != GENERAL:
        raise ValueError("the generalisation set is not seeds 1 and 2 once per arm")
    arms = [r["arm"] for r in decision]
    expected = [a for b in blocks for a in BLOCK_ORDER[b]]
    if arms != expected:
        raise ValueError(f"decision runs' arms {arms} do not follow the drawn blocks {expected}")
    on_all = [r for r in runs if r["arm"] == "on"]
    off_all = [r for r in runs if r["arm"] == "off"]

    def breach(r):
        return r["unpermitted"] is None or r["forbidden"] is None or r["unpermitted"] > 0 or r["forbidden"] > 0

    out = {"rules": {}, "tests": {}}
    r1 = not any(breach(r) for r in on_all)
    off_safe = not any(breach(r) for r in off_all)
    on_ok = sum(r["success"] for r in decision if r["arm"] == "on")
    off_ok = sum(r["success"] for r in decision if r["arm"] == "off")
    r2 = on_ok >= off_ok - 1
    r0t = all(r["T"] is not None and _finite(r["T"]["ratio"], r["T"]["gaps"]) for r in decision)
    r0ep = all(r["E"] is not None and r["P"] is not None and _finite(
        r["E"]["p50_mm"], r["E"]["p99_mm"], r["P"]["lat_min_mm"], r["P"]["beyond_mm"], r["P"]["yaw_abs_rad"])
        for r in decision)
    out["rules"].update(R1=r1, off_safe=off_safe, R2=r2, R0_T=r0t, R0_EP=r0ep,
                        successes={"on": on_ok, "off": off_ok})

    def test(name, values, c, side, kind):
        res = shown(values, arms, blocks, c, side, kind)
        res["iqr_ratio"] = iqr_ratio(values, arms)  # reported, not a gate (plan 5.10 "통계", 2026-10-19 change)
        out["tests"][name] = res
        return res["shown"]

    r5 = None
    if r0t:
        gaps = [r["T"]["gaps"] for r in decision]
        ratio = [r["T"]["ratio"] for r in decision]
        gap_gain = test("gaps<0", gaps, 0.0, "<", "mean") and sum(gaps) > 0 and stat(gaps, arms, "mean") < 0
        ratio_gain = test("ratio>0", ratio, 0.0, ">", "mean")
        ratio_harm_ok = test("ratio>-0.01", ratio, MARGINS["ratio_harm"], ">", "mean")
        n_on = sum(g for g, a in zip(gaps, arms) if a == "on")
        n_off = sum(g for g, a in zip(gaps, arms) if a == "off")
        max_on = max(g for g, a in zip(gaps, arms) if a == "on")
        max_off = max(g for g, a in zip(gaps, arms) if a == "off")
        gaps_harm_ok = n_on <= n_off + 3 and max_on <= max(max_off, 2)
        gain = gap_gain or ratio_gain
        harm_ok = ratio_harm_ok and gaps_harm_ok
        r5 = "no_gain" if not gain else ("gain" if harm_ok else "gain_over_limit")
        out["rules"].update(R5=r5, gap_gain=gap_gain, ratio_gain=ratio_gain, ratio_harm_ok=ratio_harm_ok,
                            gaps_harm_ok=gaps_harm_ok, gap_totals={"on": n_on, "off": n_off},
                            gap_max={"on": max_on, "off": max_off})
    r34 = None
    if r0ep:
        e50 = [r["E"]["p50_mm"] for r in decision]
        e99 = [r["E"]["p99_mm"] for r in decision]
        lat = [r["P"]["lat_min_mm"] for r in decision]
        beyond = [r["P"]["beyond_mm"] for r in decision]
        yaw = [r["P"]["yaw_abs_rad"] for r in decision]
        # Every test first, then the conjunction: the user gets every number (Codex 5.10 script review P3).
        t3 = [test("e_p50<10", e50, MARGINS["e_p50_mm"], "<", "median"),
              test("e_p99<15", e99, MARGINS["e_p99_mm"], "<", "median")]
        t4 = [test("lat_min>-10", lat, MARGINS["lat_min_mm"], ">", "median"),
              test("beyond>-10", beyond, -MARGINS["beyond_mm"], ">", "median"),
              test("beyond<10", beyond, MARGINS["beyond_mm"], "<", "median"),
              test("yaw<0.005", yaw, MARGINS["yaw_abs_rad"], "<", "median")]
        r3, r4 = all(t3), all(t4)
        r34 = r3 and r4
        out["rules"].update(R3=r3, R4=r4)

    # The decision table, first matching row (plan 5.10 "결론").
    if not r1 and not off_safe:
        row, outcome = 1, "both_unacceptable:user"
    elif not r1:
        row, outcome = 2, "off" if off_safe else "user:off_arm_unsafe"
    elif not r2:
        row, outcome = 3, "off" if off_safe else "user:off_arm_unsafe"
    elif not r0t:
        row, outcome = 4, "undecidable:user"
    elif r5 == "gain_over_limit":
        row, outcome = 5, "trade_off:user"
    elif r5 == "no_gain":
        row, outcome = 6, "off" if off_safe else "user:off_arm_unsafe"
    elif not r0ep:
        row, outcome = 7, "undecidable:user"
    elif not r34:
        row, outcome = 8, "trade_off:user"
    else:
        row, outcome = 9, "keep_on"
    out.update(row=row, outcome=outcome)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("manifest")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    manifest = json.loads(Path(a.manifest).read_text())
    runs = [measure(r) for r in manifest["runs"]]
    verdict = judge(manifest, runs)
    for r in runs:
        e, t, p = r["E"] or {}, r["T"] or {}, r["P"] or {}
        print(f"{r['tag']:<14} {r['arm']:<3} {r['set']:<8} ok={r['success']!s:<5} {r['failure_reason']!s:<24} "
              f"E50={e.get('p50_mm', float('nan')):6.1f} E99={e.get('p99_mm', float('nan')):6.1f} "
              f"gaps={t.get('gaps')} ratio={t.get('ratio')} lat={p.get('lat_min_mm')} "
              f"unpermitted={r['unpermitted']} forbidden={r['forbidden']}")
    for name, res in verdict["tests"].items():
        print(f"test {name:<12} observed {res['observed']:+.4f} p {res['p']:.4f} shown {res['shown']} "
              f"iqr on/off {res['iqr_ratio']}")
    print("rules", json.dumps(verdict["rules"]))
    print(f"DECISION row {verdict['row']}: {verdict['outcome']}")
    if a.json:
        slim = [{k: v for k, v in r.items() if k not in ("E", "T")} | {
            "E": {k: v for k, v in (r["E"] or {}).items() if k != "jumps_mm"} if r["E"] else None,
            "T": {k: v for k, v in (r["T"] or {}).items() if k != "curvature"} if r["T"] else None} for r in runs]
        Path(a.json).write_text(json.dumps({"runs": slim, "verdict": verdict}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
