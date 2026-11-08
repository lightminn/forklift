"""Real-time plan 5.15 G1 closed-loop diagnostic: its registered measures M1-M9 (descriptive; no V3/V4 verdict).

    PYTHONPATH=SNAP/src python g1_analyze.py SNAP RTB_DIR [--json out.json]

Reads RTB_DIR/g1/<run>/ (rt_session.sh outputs; the arm is the run name's last field: A0, A6, A6P; "warm" and
"trace0" are the warm-up and the trace-off run) and the bundle's results. Reuses rtb_analyze.py (driving period,
the interval-set ratio, W's parts, renders, stalls, the monitor) and pacer_replay.py (the replay through the real
Pacer).

  M1  ratio over the driving period (rtb_analyze), and over the whole session from the first control tick to the
      last status line (status lines' monotonic clock and tick_t)
  M2  pacing ok from the status lines (one per control tick; read_live_log verified, step continuity checked):
      wall from the live `start` event (after the SLAM warm-up; the runner's own start is not on this clock) to the
      first ok; the share of AUTO ticks
      with ok after the first ok (undecided without one); the ok-down transitions after it and, for each, the
      largest part of the longest step interval in the second before it
  M3  start intervals over 0.25 s in the driving period: interval = W + the wait + eps, the source the largest of
      the parts of W (gate, world, step_rest, after, between, control, to_wait), the wait, eps and unclassified
  M4  W of steps started ahead of their spacing (S_i - S_{i-1} < h) against the others, per step kind, beside the
      same split in a fixed-W replay of the run's own W sequence through the Pacer with its catch-up (the share the
      selection alone gives)
  M5  driving steps where the live camera and the scan coincide, where a capture render and the scan coincide,
      renders only, renders by source (far and pocket expected 0)
  M6  SLAM reply application delay applied_tick_s - stamp_s over processed records (p50, p90, max)
  M7  the end event's camera_latency_s / camera_pixel_latency_s (window_median, window_p99, max) and
      camera_pixel_unknown
  M8  the mission result and the hall failure mechanism figures (as rtb/tools/mech.py)
  M9  per block, the block's A0 replayed (catch-up 6; split + catch-up 6) against that block's A6 / A6P measured:
      ratio over the whole driving period and over the common sim window of the two runs, and the ok share
  trace-off: the status-line ratio with the driving period trimmed 2 s at each end, the same formula for the A0
      runs beside it
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SNAP = Path(sys.argv[1]).resolve()
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import rtb_analyze as ra  # noqa: E402  (reads SNAP from argv[1] too)
import pacer_replay as pr  # noqa: E402
from forklift_core.live.pacing import Pacer  # noqa: E402
from forklift_core.live.spool import iter_chunks, load_list, read_live_log  # noqa: E402

DT = 1 / 120
C, BIT = ra.C, ra.BIT
PART_NAMES = ("gate", "world", "step_rest", "after", "between", "control", "to_wait")


def status_lines(run: Path):
    rows = read_live_log(run)
    st = [r for r in rows if "pacing_ok" in r and r.get("tick_t") is not None and r.get("mono") is not None]
    steps = [r.get("step") for r in st if r.get("step") is not None]
    gaps = int(np.sum(np.diff(steps) != 1)) if len(steps) > 1 else None
    start = next((r["mono"] for r in rows if r.get("event") == "start"), None)
    end = next((r for r in rows if r.get("event") == "end"), {})
    return st, start, end, gaps


def status_ratio(st, t0=None, t1=None, trim=0.0):
    pts = [(r["tick_t"], r["mono"]) for r in st
           if (t0 is None or r["tick_t"] >= t0 + trim) and (t1 is None or r["tick_t"] <= t1 - trim)]
    if len(pts) < 2:
        return None
    (a, ma), (b, mb) = pts[0], pts[-1]
    return (b - a) / (mb - ma)


def m2(st, start_mono):
    oks = [bool(r["pacing_ok"]) for r in st]
    if not any(oks):
        return {"first_ok": None, "ok_share_auto": None}
    i0 = oks.index(True)
    auto = [r for r in st[i0:] if r.get("mode") == "AUTO"]
    downs = [st[i]["mono"] for i in range(i0 + 1, len(st)) if oks[i - 1] and not oks[i]]
    out = {"first_ok_from_live_start_s": st[i0]["mono"] - start_mono if start_mono else None,
           "ok_share_auto": float(np.mean([bool(r["pacing_ok"]) for r in auto])) if auto else None,
           "auto_ticks_after_first_ok": len(auto), "ok_down": len(downs), "_downs": downs}
    return out


def parts_of(data):
    S = data[:, C["S"]]
    nx = data[np.r_[np.arange(1, len(data)), len(data) - 1]]
    return {
        "gate": data[:, C["world_start"]] - S,
        "world": data[:, C["world_end"]] - data[:, C["world_start"]],
        "step_rest": data[:, C["step_end"]] - data[:, C["world_end"]],
        "after": data[:, C["after_end"]] - data[:, C["step_end"]],
        "between": nx[:, C["ctrl_start"]] - data[:, C["after_end"]],
        "control": nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]],
        "to_wait": nx[:, C["q"]] - nx[:, C["ctrl_end"]],
    }


def analyse(out: Path, arm: str, results: dict) -> dict:
    r = ra.analyse_run(out)
    r["arm"] = arm
    run = out / "live" / "epoch_1" / "run"
    b = results.get(out.name) or {}
    r["bundle"] = {k: b.get(k) for k in ("status", "attempt", "rc", "cpu_s", "wall_s", "vram_max_mib", "start")}
    try:
        st, start_mono, end, step_gaps = status_lines(run)
    except ValueError as exc:
        r["why"].append(f"live log: {exc}")
        st, start_mono, end, step_gaps = [], None, {}, None
    r["status_step_gaps"] = step_gaps
    if st:
        first_ctrl = st[0]
        r["M1_whole_ratio"] = status_ratio(st)
        r["M1_whole_from_tick_t"] = first_ctrl["tick_t"]
        r["M2"] = m2(st, start_mono)
        if r.get("driving_sim_s"):
            t0, t1 = r["driving_sim_s"]
            r["status_ratio_trim2"] = status_ratio(st, t0, t1, trim=2.0)
    r["M7"] = {k: ({kk: (end.get(k) or {}).get(kk) for kk in ("window_median", "window_p99", "max", "n")}
                   if isinstance(end.get(k), dict) else end.get(k))
               for k in ("camera_latency_s", "camera_pixel_latency_s", "camera_pixel_unknown")}
    res = run / "missions" / "m1" / "result.json"
    if res.exists():
        r["M8_result"] = json.loads(res.read_text())["result"]
    if "_arr" not in r:
        return r
    w, interval, eps, wait, parts, replies, h, data = r["_arr"]
    K = r["_K"]
    S = data[:, C["S"]]
    sim = r["_sim"]
    # M3
    gaps = []
    resid = w - np.nansum(np.vstack([parts[p] for p in PART_NAMES]), axis=0)
    for i in np.flatnonzero(K & (interval > 0.25)):
        cand = {p: float(parts[p][i]) for p in PART_NAMES}
        cand.update(wait=float(wait[i]), eps=float(eps[i]), unclassified=float(resid[i]))
        src = max(cand, key=lambda k: np.nan_to_num(cand[k], nan=-1))
        gaps.append({"sim_s": float(sim[i]), "interval_s": float(interval[i]), "source": src,
                     "parts_s": {k: round(v, 4) for k, v in cand.items()}})
    r["M3_gaps"] = gaps
    # M2 ok-down sources: the longest interval in the second before each transition
    if r.get("M2", {}).get("_downs"):
        srcs = []
        for m in r["M2"]["_downs"]:
            idx = np.flatnonzero((S >= m - 1.0) & (S <= m) & np.isfinite(interval))
            if not len(idx):
                continue
            i = idx[np.argmax(interval[idx])]
            cand = {p: float(np.nan_to_num(parts[p][i])) for p in PART_NAMES}
            srcs.append(max(cand, key=cand.get))
        r["M2"]["ok_down_sources"] = {s: srcs.count(s) for s in sorted(set(srcs))}
        del r["M2"]["_downs"]
    # M4
    kinds = ra.kind_of(data[:, C["flags"]])
    early = np.r_[False, (S[1:] - S[:-1]) < h - 1e-9]
    r["M4_measured"] = split_means(w, kinds, early, K)
    catchup = 6 if arm in ("A6", "A6P") else 0
    if catchup:
        idx = np.flatnonzero(K)
        starts = replay_starts(w[idx], np.nan_to_num(eps[idx]), catchup)
        early_rep = np.zeros(len(w), bool)
        early_rep[idx[1:]] = np.diff(starts) < h - 1e-9
        r["M4_fixed_w_replay"] = split_means(w, kinds, early_rep, K)
    # M5
    D = r["_D"]
    bits = np.nan_to_num(data[:, C["flags"]]).astype(int)
    has = lambda f: (bits & BIT[f]) != 0  # noqa: E731
    r["M5"] = {"live_camera_and_scan": int(np.sum(D & has("live_camera") & has("scan"))),
               "capture_and_scan": int(np.sum(D & has("capture") & has("scan") & has("rendered"))),
               "render_only": int(np.sum(D & has("rendered") & ~has("scan"))),
               "rendered_and_scan": int(np.sum(D & has("rendered") & has("scan"))),
               "far": int(np.sum(D & has("far"))), "pocket": int(np.sum(D & has("pocket")))}
    # M6
    try:
        recs = load_list(run / "spool", "slam.records")
        d = np.array([x["applied_tick_s"] - x["stamp_s"] for x in recs
                      if x.get("status") == "processed" and x.get("applied_tick_s") is not None])
        r["M6"] = {"n": int(len(d)), "p50": float(np.percentile(d, 50)), "p90": float(np.percentile(d, 90)),
                   "max": float(d.max())} if len(d) else {"n": 0}
    except ValueError as exc:
        r["M6"] = {"error": str(exc)}
    # M8 mechanism
    ctl = []
    for ch in iter_chunks(run / "spool", "slam.control"):
        ctl.extend(ch)
    if ctl:
        c = np.array(ctl, float).reshape(-1, 7)
        e = np.hypot(c[:, 1] - c[:, 4], c[:, 2] - c[:, 5])
        dd = np.diff(e)
        j = int(np.argmin(dd))
        mid = (c[:, 0] > 80) & (c[:, 0] < min(220, c[-1, 0] - 5))
        r["M8_mech"] = {"err_p50_80_220": float(np.median(e[mid])) if mid.any() else None,
                        "largest_drop_m": float(dd[j]), "largest_drop_t": float(c[j, 0]), "end_err": float(e[-1])}
    return r


def split_means(w, kinds, early, K):
    out = {}
    for kd in sorted(set(kinds[K])):
        m = K & (kinds == kd)
        a, b = m & early, m & ~early
        out[kd] = {"n_early": int(a.sum()), "n_other": int(b.sum()),
                   "w_early_ms": float(np.mean(w[a]) * 1000) if a.any() else None,
                   "w_other_ms": float(np.mean(w[b]) * 1000) if b.any() else None}
    return out


def replay_starts(w, eps, catchup):
    p = Pacer(1.0, DT, catchup_steps=catchup)
    t, starts = 0.0, []
    for wi, ei in zip(w, eps):
        sleep = p.wait_s(t)
        t += sleep + (ei if sleep > 0 else 0.0)
        p.started(t)
        starts.append(t)
        t += wi
        p.after_step(t)
    return np.array(starts)


def window_ratio(r, lo, hi):
    """Measured ratio over the run's driving steps whose sim time lies in [lo, hi) (sim seconds since its t0)."""
    w, interval, *_ = r["_arr"]
    t0 = r["driving_sim_s"][0]
    m = r["_K"] & (r["_sim"] >= t0 + lo) & (r["_sim"] < t0 + hi)
    return float(m.sum() * DT / np.sum(interval[m])) if m.any() else None


def m9(runs, trace_root: Path, analysis_by_name: dict):
    out = []
    blocks = {}
    for r in runs:
        tag = r["name"].split("_")[2]
        if tag.startswith("b"):
            blocks.setdefault(tag, {})[r["arm"]] = r
    for tag, arms in sorted(blocks.items()):
        a0 = arms.get("A0")
        if a0 is None or "_arr" not in a0:
            continue
        rep = pr.load(trace_root, {"name": a0["name"], "driving_sim_s": a0["driving_sim_s"]})
        for target, kw in (("A6", {}), ("A6P", {"split": True})):
            t = arms.get(target)
            if t is None or "_arr" not in t:
                continue
            w, tse = pr.transformed(rep, **kw)
            ratio, ok = pr.replay(w, rep["eps"], 6, tse, with_ok=True)
            span = min(a0["driving_sim_s"][1] - a0["driving_sim_s"][0],
                       t["driving_sim_s"][1] - t["driving_sim_s"][0])
            n = int(span / DT)
            ratio_c, ok_c = pr.replay(w[:n], rep["eps"][:n], 6, tse[:n], with_ok=True)
            out.append({"block": tag, "target": target, "predicted_ratio": ratio, "predicted_ok": ok,
                        "measured_ratio": t["ratio"], "common_span_sim_s": span,
                        "predicted_ratio_common": ratio_c, "predicted_ok_common": ok_c,
                        "measured_ratio_common": window_ratio(t, 0, span),
                        "measured_ok_auto": (t.get("M2") or {}).get("ok_share_auto")})
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("snap")
    ap.add_argument("rtb", type=Path)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    g1 = a.rtb / "g1"
    results = {}
    for line in (g1 / "bundle" / "results.jsonl").read_text().splitlines():
        x = json.loads(line)
        results[x["name"]] = x
    runs = []
    for out in sorted(p for p in g1.iterdir() if p.is_dir() and p.name != "bundle"):
        runs.append(analyse(out, out.name.rsplit("_", 1)[-1], results))
    m9_rows = m9(runs, a.rtb, {})
    for r in runs:
        for k in [k for k in r if k.startswith("_")]:
            del r[k]
        m2_ = r.get("M2") or {}
        print(f"{r['name']:22s} {r['arm']:4s} valid {r['valid']!s:5s} ratio {r.get('ratio', float('nan')):.4f} "
              f"whole {r.get('M1_whole_ratio') or float('nan'):.4f} ok {m2_.get('ok_share_auto')} "
              f"gaps>0.25 {len(r.get('M3_gaps', []))} M5 {r.get('M5')} why {r['why']}")
    for row in m9_rows:
        print("M9", {k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()})
    if a.json:
        a.json.write_text(json.dumps({"runs": runs, "m9": m9_rows}, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
